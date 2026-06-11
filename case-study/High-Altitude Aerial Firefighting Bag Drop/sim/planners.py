"""
planners.py
Twelve drop planners for the aerial firefighting bag drop case study.

A "planner" decides the release position correction (x_off, y_off) [m]
given a scenario's expected wind conditions.  The simulation runner then
evaluates each planner by running N_mc independent trajectory simulations
with random wind perturbations and computing accuracy metrics.

Planner interface:
  name()  -> str
  reset() -> None
  plan(scenario: dict, plant_params: dict) -> (x_off, y_off) [m]
    x_off : downrange correction (positive = release further along flight path)
    y_off : lateral correction  (positive = release to the right)
    e.g., crosswind wy > 0 pushes bag right -> y_off < 0 to compensate

ctrl_toolbox is imported with a graceful fallback.  Planners that require
it (GPSurrogate, BayesOpt) fall back to WindOffsetDrop when unavailable.
"""

import sys
import os
import math
import random

# Locate ctrl_toolbox: sim/ -> StudyDir/ -> case-study/ -> project root
_THIS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_THIS)))
sys.path.insert(0, os.path.join(_ROOT, 'build', 'bindings'))
if sys.platform == 'win32' and hasattr(os, 'add_dll_directory'):
    for _p in [r'C:\msys64\mingw64\bin']:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

try:
    import ctrl_toolbox as ctrl
    import numpy as np
    CTRL_AVAILABLE = True
except ImportError:
    CTRL_AVAILABLE = False

from bag_drop_plant import BagDropPlant


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get_wind(scenario: dict):
    """Return (wx_mean, wy_mean, sigma_w) from scenario dict."""
    return (float(scenario.get('wx_mean', 0.0)),
            float(scenario.get('wy_mean', 0.0)),
            float(scenario.get('wind_sigma', 0.5)))


def _fall_time_and_range(h_drop, V_aircraft, plant_params):
    """Run zero-wind trajectory to get reference t_fall and x_nom."""
    plant = BagDropPlant(plant_params)
    x_n, _, t_fall, _ = plant.nominal_impact(h_drop, V_aircraft)
    return t_fall, x_n


def _drift_sensitivity(h_drop, V_aircraft, plant_params):
    """
    Compute actual drift per unit wind speed [m / (m/s)] by running two
    reference trajectories (unit headwind, unit crosswind).
    Returns (dx_per_wx, dy_per_wy).
    Forward speed couples into drag, so drift != t_fall * wind.
    """
    x0, y0, _, _ = _run_with_wind(h_drop, V_aircraft, (0, 0, 0), plant_params)
    x1, _,  _, _ = _run_with_wind(h_drop, V_aircraft, (1, 0, 0), plant_params)
    _,  y1, _, _ = _run_with_wind(h_drop, V_aircraft, (0, 1, 0), plant_params)
    return (x1 - x0), y1   # dx per wx [m/(m/s)], dy per wy [m/(m/s)]


def _run_with_wind(h_drop, V_aircraft, wind, plant_params,
                   x_off=0.0, y_off=0.0):
    """Convenience wrapper: simulate one trajectory."""
    plant = BagDropPlant(plant_params)
    return plant.simulate(h_drop, V_aircraft, wind,
                          x0=x_off, y0=y_off)


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------
class DropPlanner:
    def name(self):   return "DropPlanner"
    def reset(self):  pass
    def plan(self, scenario, plant_params):
        return (0.0, 0.0)


# ---------------------------------------------------------------------------
# 1. NominalDrop  -- no correction at all, open-loop baseline
# ---------------------------------------------------------------------------
class NominalDrop(DropPlanner):
    def name(self):  return "NominalDrop"

    def plan(self, scenario, plant_params):
        return (0.0, 0.0)


# ---------------------------------------------------------------------------
# 2. WindOffsetDrop  -- analytical wind drift compensation
#    Estimates fall time from zero-wind trajectory, then precomputes drift.
# ---------------------------------------------------------------------------
class WindOffsetDrop(DropPlanner):
    def name(self):  return "WindOffset"

    def plan(self, scenario, plant_params):
        h = float(scenario.get('h_drop', plant_params['h_drop_nom']))
        Va = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
        wx, wy, _ = _get_wind(scenario)
        # Simulate actual drift with mean wind (accounts for drag coupling)
        x_nom, _, _, _ = _run_with_wind(h, Va, (0, 0, 0), plant_params)
        xi, yi, _, _   = _run_with_wind(h, Va, (wx, wy, 0), plant_params)
        return (x_nom - xi, -yi)


# ---------------------------------------------------------------------------
# 3. IterativeRefinementDrop  -- ILC-style cross-run refinement
#    Starts with WindOffset; stores actual drift from previous run per scenario
#    and refines the correction incrementally.
# ---------------------------------------------------------------------------
class IterativeRefinementDrop(DropPlanner):
    def __init__(self, plant_params):
        self._p       = plant_params
        self._history = {}   # scenario_id -> (x_off, y_off, n_updates)

    def name(self): return "IterativeRefine"

    def reset(self):
        self._history = {}

    def plan(self, scenario, plant_params):
        sid = scenario.get('id', 'unknown')
        if sid in self._history:
            x_off, y_off, _ = self._history[sid]
            return (x_off, y_off)
        # First call: use WindOffset as seed
        return WindOffsetDrop().plan(scenario, plant_params)

    def update(self, scenario, plant_params, x_off_used, y_off_used,
               x_impact_mean, y_impact_mean, x_nom):
        """Called by simulation runner after each run to refine estimate."""
        sid = scenario.get('id', 'unknown')
        # Residual error: how far did the centroid land from target (x_nom, 0)?
        ex = x_impact_mean - x_nom
        ey = y_impact_mean - 0.0
        lr = 0.6   # learning rate
        if sid in self._history:
            xo, yo, n = self._history[sid]
            self._history[sid] = (xo - lr * ex, yo - lr * ey, n + 1)
        else:
            self._history[sid] = (x_off_used - lr * ex,
                                  y_off_used - lr * ey, 1)


# ---------------------------------------------------------------------------
# 4. MCPredictor  -- internal Monte Carlo bias estimator
#    Runs N_internal trajectories with mean wind; uses centroid to correct.
# ---------------------------------------------------------------------------
class MCPredictor(DropPlanner):
    N_INTERNAL = 12

    def name(self): return "MCPredictor"

    def plan(self, scenario, plant_params):
        h = float(scenario.get('h_drop', plant_params['h_drop_nom']))
        Va = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
        wx, wy, sigma = _get_wind(scenario)
        x_nom, _, t_n, _ = _run_with_wind(h, Va, (0, 0, 0), plant_params)
        rng = random.Random(0)
        xs, ys = [], []
        for _ in range(self.N_INTERNAL):
            wx_s = wx + sigma * rng.gauss(0, 1)
            wy_s = wy + sigma * rng.gauss(0, 1)
            xi, yi, _, _ = _run_with_wind(h, Va, (wx_s, wy_s, 0), plant_params)
            xs.append(xi); ys.append(yi)
        x_cen = sum(xs) / len(xs)
        y_cen = sum(ys) / len(ys)
        return (x_nom - x_cen, -y_cen)


# ---------------------------------------------------------------------------
# 5. GPSurrogateDrop  -- Gaussian Process regression over wind space
#    Trains a GP on a 3x3 grid of (wx, wy) -> drift.
#    Requires ctrl_toolbox; falls back to WindOffsetDrop.
# ---------------------------------------------------------------------------
class GPSurrogateDrop(DropPlanner):
    def name(self): return "GPSurrogate"

    def plan(self, scenario, plant_params):
        if not CTRL_AVAILABLE:
            return WindOffsetDrop().plan(scenario, plant_params)
        h = float(scenario.get('h_drop', plant_params['h_drop_nom']))
        Va = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
        wx_q, wy_q, _ = _get_wind(scenario)
        x_nom, _, _, _ = _run_with_wind(h, Va, (0, 0, 0), plant_params)

        # Build 3x3 training grid
        wx_range = max(abs(wx_q) + 3.0, 6.0)
        wy_range = max(abs(wy_q) + 3.0, 6.0)
        wx_pts = [-wx_range, 0.0, wx_range]
        wy_pts = [-wy_range, 0.0, wy_range]

        X_pts, dx_pts, dy_pts = [], [], []
        for wx_i in wx_pts:
            for wy_i in wy_pts:
                xi, yi, _, _ = _run_with_wind(h, Va, (wx_i, wy_i, 0), plant_params)
                X_pts.append([wx_i, wy_i])
                dx_pts.append(xi - x_nom)
                dy_pts.append(yi)

        try:
            gp_p = ctrl.GPParams()
            gp_p.length_scale = 4.0
            gp_p.signal_var   = 100.0
            gp_p.noise_var    = 0.1
            gp_p.n_max        = 30

            gp_x = ctrl.GaussianProcess(2, gp_p)
            gp_y = ctrl.GaussianProcess(2, gp_p)

            for i, (wxi, wyi) in enumerate(X_pts):
                gp_x.add_point(np.array([wxi, wyi]), float(dx_pts[i]))
                gp_y.add_point(np.array([wxi, wyi]), float(dy_pts[i]))

            gp_x.fit()
            gp_y.fit()

            q = np.array([wx_q, wy_q])
            pred_x = gp_x.predict(q)
            pred_y = gp_y.predict(q)
            return (-float(pred_x.mean), -float(pred_y.mean))
        except Exception:
            return WindOffsetDrop().plan(scenario, plant_params)


# ---------------------------------------------------------------------------
# 6. BayesOptDrop  -- BayesianOptimizer to find optimal (x_off, y_off)
#    Objective: median radial error from N=5 internal MC trajectories.
#    Requires ctrl_toolbox; falls back to WindOffsetDrop.
# ---------------------------------------------------------------------------
class BayesOptDrop(DropPlanner):
    N_EVAL_MC = 6

    def name(self): return "BayesOptDrop"

    def plan(self, scenario, plant_params):
        if not CTRL_AVAILABLE:
            return WindOffsetDrop().plan(scenario, plant_params)
        h = float(scenario.get('h_drop', plant_params['h_drop_nom']))
        Va = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
        wx_m, wy_m, sigma = _get_wind(scenario)
        x_nom, _, t_f, _ = _run_with_wind(h, Va, (0, 0, 0), plant_params)

        # Search range: up to 2x expected drift in each axis
        half_x = max(abs(wx_m) * t_f * 2.0, 5.0)
        half_y = max(abs(wy_m) * t_f * 2.0, 5.0)

        rng_mc = random.Random(42)

        def cost_fn(params):
            xo, yo = float(params[0]), float(params[1])
            radii = []
            for _ in range(self.N_EVAL_MC):
                wx_s = wx_m + sigma * rng_mc.gauss(0, 1)
                wy_s = wy_m + sigma * rng_mc.gauss(0, 1)
                xi, yi, _, _ = _run_with_wind(h, Va, (wx_s, wy_s, 0),
                                              plant_params, x_off=xo, y_off=yo)
                radii.append(math.sqrt((xi - x_nom) ** 2 + yi * yi))
            radii.sort()
            return radii[len(radii) // 2]   # median = CEP estimate

        try:
            bp = ctrl.BayesOptParams()
            bp.n           = 2
            bp.n_init      = 4
            bp.maxIter     = 10
            bp.n_acq_restarts = 20
            bp.seed        = 7
            bp.lower       = np.array([-half_x, -half_y])
            bp.upper       = np.array([ half_x,  half_y])
            bp.acq         = ctrl.BayesAcquisition.UCB

            # Initial guess: WindOffset estimate
            wx0, wy0 = WindOffsetDrop().plan(scenario, plant_params)
            x0 = np.array([wx0, wy0])

            bo = ctrl.BayesianOptimizer(bp)
            result = bo.tune(cost_fn, x0)
            return (float(result.params[0]), float(result.params[1]))
        except Exception:
            return WindOffsetDrop().plan(scenario, plant_params)


# ---------------------------------------------------------------------------
# 7. SIRParticleFilterDrop  -- Sequential Importance Resampling
#    Models wind state [wx, wy] with N=60 particles.
#    "Observation" = noisy wind measurement from on-board anemometer.
#    Corrects using posterior mean wind estimate.
# ---------------------------------------------------------------------------
class SIRParticleFilterDrop(DropPlanner):
    N_PARTICLES = 60

    def name(self): return "SIRParticleFilter"

    def plan(self, scenario, plant_params):
        wx_m, wy_m, sigma = _get_wind(scenario)
        h  = float(scenario.get('h_drop', plant_params['h_drop_nom']))
        Va = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
        # Sensitivity: actual drift per unit wind (accounts for drag coupling)
        dx_per_wx, dy_per_wy = _drift_sensitivity(h, Va, plant_params)

        rng = random.Random(13)
        sigma_prior  = sigma + 1.0        # prior wider than true sigma
        sigma_obs    = 1.0                # anemometer noise std [m/s]

        # Prior particles
        particles = [(wx_m + sigma_prior * rng.gauss(0, 1),
                      wy_m + sigma_prior * rng.gauss(0, 1))
                     for _ in range(self.N_PARTICLES)]

        # Noisy wind measurement (simulated sensor reading)
        meas_x = wx_m + sigma_obs * rng.gauss(0, 1)
        meas_y = wy_m + sigma_obs * rng.gauss(0, 1)

        # Likelihood: Gaussian observation model
        def log_likelihood(wx_p, wy_p):
            ex = (meas_x - wx_p) / sigma_obs
            ey = (meas_y - wy_p) / sigma_obs
            return -0.5 * (ex * ex + ey * ey)

        log_w = [log_likelihood(p[0], p[1]) for p in particles]
        max_lw = max(log_w)
        w = [math.exp(lw - max_lw) for lw in log_w]
        w_sum = sum(w)
        w = [wi / w_sum for wi in w]

        wx_est = sum(w[i] * particles[i][0] for i in range(self.N_PARTICLES))
        wy_est = sum(w[i] * particles[i][1] for i in range(self.N_PARTICLES))

        return (-wx_est * dx_per_wx, -wy_est * dy_per_wy)


# ---------------------------------------------------------------------------
# 8. KalmanWindDrop  -- Linear Kalman filter for wind state [wx, wy]
#    Single update step: prior = (wx_mean, sigma) -> posterior after one
#    noisy measurement (anemometer).
# ---------------------------------------------------------------------------
class KalmanWindDrop(DropPlanner):
    def name(self): return "KalmanWind"

    def plan(self, scenario, plant_params):
        wx_m, wy_m, sigma = _get_wind(scenario)
        h  = float(scenario.get('h_drop', plant_params['h_drop_nom']))
        Va = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
        # Sensitivity: actual drift per unit wind
        dx_per_wx, dy_per_wy = _drift_sensitivity(h, Va, plant_params)

        sigma_meas = 1.0   # anemometer std [m/s]
        rng = random.Random(17)

        meas_x = wx_m + sigma_meas * rng.gauss(0, 1)
        meas_y = wy_m + sigma_meas * rng.gauss(0, 1)

        P_prior = sigma * sigma
        R = sigma_meas * sigma_meas
        K = P_prior / (P_prior + R)

        wx_post = wx_m + K * (meas_x - wx_m)
        wy_post = wy_m + K * (meas_y - wy_m)

        return (-wx_post * dx_per_wx, -wy_post * dy_per_wy)


# ---------------------------------------------------------------------------
# 9. RobustMinMaxDrop  -- Worst-case wind compensation
#    Plans for the worst case within the uncertainty ellipse:
#    offset = -(mean_wind + margin * sigma) * t_fall.
#    The margin ensures coverage even under adverse gusts.
# ---------------------------------------------------------------------------
class RobustMinMaxDrop(DropPlanner):
    MARGIN = 1.0   # sigma multiplier for safety

    def name(self): return "RobustMinMax"

    def plan(self, scenario, plant_params):
        wx_m, wy_m, sigma = _get_wind(scenario)
        h  = float(scenario.get('h_drop', plant_params['h_drop_nom']))
        Va = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
        dx_per_wx, dy_per_wy = _drift_sensitivity(h, Va, plant_params)

        # Worst-case wind in each axis (mean + margin * sigma)
        wx_wc = wx_m + self.MARGIN * sigma * (1.0 if wx_m >= 0.0 else -1.0)
        wy_wc = wy_m + self.MARGIN * sigma * (1.0 if wy_m >= 0.0 else -1.0)
        return (-wx_wc * dx_per_wx, -wy_wc * dy_per_wy)


# ---------------------------------------------------------------------------
# 10. AdaptiveRLSDrop  -- Recursive Least Squares wind-drift model
#     Fits: [x_drift, y_drift] = [[a, 0], [0, b]] * [wx, wy]
#     Initialized with synthetic data; updates across calls using RLS.
# ---------------------------------------------------------------------------
class AdaptiveRLSDrop(DropPlanner):
    def __init__(self, plant_params):
        self._p = plant_params
        # RLS state: slope estimates for each axis
        self._ax = 0.0   # x_drift per unit wx
        self._ay = 0.0   # y_drift per unit wy
        self._Px = 1.0   # RLS covariance (scalar)
        self._Py = 1.0
        self._lam = 0.95  # forgetting factor
        self._initialized = False

    def name(self): return "AdaptiveRLS"

    def reset(self):
        self._ax = 0.0; self._ay = 0.0
        self._Px = 1.0; self._Py = 1.0
        self._initialized = False

    def _init_from_physics(self, h, Va, plant_params):
        """Bootstrap RLS from simulated drift sensitivity."""
        dx_per_wx, dy_per_wy = _drift_sensitivity(h, Va, plant_params)
        self._ax = dx_per_wx
        self._ay = dy_per_wy
        self._initialized = True

    def plan(self, scenario, plant_params):
        h  = float(scenario.get('h_drop', plant_params['h_drop_nom']))
        Va = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
        wx_m, wy_m, _ = _get_wind(scenario)
        if not self._initialized:
            self._init_from_physics(h, Va, plant_params)
        return (-wx_m * self._ax, -wy_m * self._ay)

    def update(self, wx_actual, wy_actual, x_drift_actual, y_drift_actual):
        """RLS update after observing actual wind and resulting drift."""
        if abs(wx_actual) > 1e-3:
            phi_x = wx_actual
            err_x = x_drift_actual - self._ax * phi_x
            K_x = self._Px * phi_x / (self._lam + self._Px * phi_x * phi_x)
            self._ax += K_x * err_x
            self._Px = (self._Px - K_x * phi_x * self._Px) / self._lam
        if abs(wy_actual) > 1e-3:
            phi_y = wy_actual
            err_y = y_drift_actual - self._ay * phi_y
            K_y = self._Py * phi_y / (self._lam + self._Py * phi_y * phi_y)
            self._ay += K_y * err_y
            self._Py = (self._Py - K_y * phi_y * self._Py) / self._lam


# ---------------------------------------------------------------------------
# 11. EnsembleConsensusDrop  -- Weighted average of three strategies
#     WindOffset (w=0.4) + MCPredictor (w=0.4) + RobustMinMax (w=0.2)
# ---------------------------------------------------------------------------
class EnsembleConsensusDrop(DropPlanner):
    def __init__(self, plant_params):
        self._wind   = WindOffsetDrop()
        self._mc     = MCPredictor()
        self._robust = RobustMinMaxDrop()

    def name(self): return "EnsembleConsensus"

    def plan(self, scenario, plant_params):
        wx0, wy0 = self._wind.plan(scenario, plant_params)
        xm,  ym  = self._mc.plan(scenario, plant_params)
        xr,  yr  = self._robust.plan(scenario, plant_params)
        return (0.4 * wx0 + 0.4 * xm + 0.2 * xr,
                0.4 * wy0 + 0.4 * ym + 0.2 * yr)


# ---------------------------------------------------------------------------
# 12. ProfileAdaptiveDrop  -- Altitude-dependent correction model
#     Higher drop altitudes have longer fall times and greater wind sensitivity.
#     Uses a physics-based scaling: t_fall ~ sqrt(h) * K_drag.
# ---------------------------------------------------------------------------
class ProfileAdaptiveDrop(DropPlanner):
    def name(self): return "ProfileAdaptive"

    def plan(self, scenario, plant_params):
        h  = float(scenario.get('h_drop', plant_params['h_drop_nom']))
        Va = float(scenario.get('V_aircraft', plant_params['V_aircraft']))
        wx_m, wy_m, sigma = _get_wind(scenario)
        dx_per_wx, dy_per_wy = _drift_sensitivity(h, Va, plant_params)

        # Altitude-dependent safety margin: higher -> longer exposure to turbulence
        h_ref = float(plant_params.get('h_drop_nom', 90.0))
        K_alt = math.sqrt(h / h_ref)

        wx_eff = wx_m + 0.3 * sigma * K_alt * (1.0 if wx_m >= 0.0 else -1.0)
        wy_eff = wy_m + 0.3 * sigma * K_alt * (1.0 if wy_m >= 0.0 else -1.0)
        return (-wx_eff * dx_per_wx, -wy_eff * dy_per_wy)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def make_planners(plant_params: dict):
    """Return list of all 12 planners."""
    return [
        NominalDrop(),
        WindOffsetDrop(),
        IterativeRefinementDrop(plant_params),
        MCPredictor(),
        GPSurrogateDrop(),
        BayesOptDrop(),
        SIRParticleFilterDrop(),
        KalmanWindDrop(),
        RobustMinMaxDrop(),
        AdaptiveRLSDrop(plant_params),
        EnsembleConsensusDrop(plant_params),
        ProfileAdaptiveDrop(),
    ]
