"""
controllers.py
Seventeen controllers for the drill string velocity tracking case study.

All controllers share the interface:
  name()  -> str
  reset() -> None
  compute(omega_ref, omega_b, phi, t) -> float  (returns omega_t command [rad/s])

ctrl_toolbox is imported with a graceful fallback; if unavailable, only
OpenLoopCtrl is functional and the remaining controllers are stubs.
"""

import sys
import os
import math
import numpy as np

# Locate ctrl_toolbox binding from sim/ -> StudyName/ -> case-study/ -> root
_THIS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_THIS)))
sys.path.insert(0, _ROOT)
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    CTRL_AVAILABLE = True
except ImportError:
    CTRL_AVAILABLE = False


def build_linear_model(p: dict, omega_nom: float = 10.0):
    """ZOH-discretize the linearized drill string SS model at omega_nom."""
    if not CTRL_AVAILABLE:
        return None
    try:
        import scipy.signal as sig
    except ImportError:
        return None

    J_b = p['J_b']; k_t = p['k_t']; c_t = p['c_t']; Ts = p['Ts']
    WOB = p['WOB']; R_b = p['R_b']
    mu_s = p['mu_s']; mu_k = p['mu_k']
    eps_v = p['eps_v']; eps_tanh = p['eps_tanh']

    T0    = WOB * R_b
    exp_  = math.exp(-omega_nom / eps_v)
    strib = mu_k + (mu_s - mu_k) * exp_
    d_s   = -(mu_s - mu_k) / eps_v * exp_               # d(strib)/d(omega) at omega_nom
    th    = math.tanh(omega_nom / eps_tanh)
    dth   = 1.0 / (eps_tanh * math.cosh(omega_nom / eps_tanh) ** 2)
    dT    = T0 * (d_s * th + strib * dth)                # dT_bit/domega at omega_nom

    A_c = np.array([[0.0, -1.0],
                    [k_t / J_b, -(c_t + dT) / J_b]])
    B_c = np.array([[1.0], [c_t / J_b]])
    C   = np.array([[0.0, 1.0]])
    D   = np.array([[0.0]])

    Ad, Bd, Cd, Dd, _ = sig.cont2discrete((A_c, B_c, C, D), Ts, method='zoh')
    return ctrl.StateSpace(Ad, Bd, Cd, Dd, Ts)


# ---------------------------------------------------------------------------
# Base
# ---------------------------------------------------------------------------
class DrillController:
    def name(self):    return "BaseController"
    def reset(self):   pass
    def compute(self, omega_ref, omega_b, phi, t): return omega_ref


# ---------------------------------------------------------------------------
# 1. OpenLoop
# ---------------------------------------------------------------------------
class OpenLoopCtrl(DrillController):
    def name(self):    return "OpenLoop"
    def compute(self, omega_ref, omega_b, phi, t):
        return omega_ref


# ---------------------------------------------------------------------------
# 2. PID
# ---------------------------------------------------------------------------
class PIDCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        pp = ctrl.PIDParams()
        pp.Kp = 2.0; pp.Ki = 0.6; pp.Kd = 0.1
        pp.uMin = -30.0; pp.uMax = 30.0; pp.Kb = 0.5
        self._pid = ctrl.DiscretePID(pp, Ts)

    def name(self): return "PID"

    def reset(self):
        self._pid.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        return omega_ref + self._pid.compute(omega_ref - omega_b)


# ---------------------------------------------------------------------------
# 3. ADRC
# ---------------------------------------------------------------------------
class ADRCCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        ap = ctrl.ADRCParams()
        # omega_o * Ts = 3.0 * 0.1 = 0.3 < 0.5 (constraint satisfied)
        ap.omega_c = 1.0; ap.omega_o = 3.0; ap.b0 = 0.5
        ap.uMin = -30.0; ap.uMax = 30.0
        self._adrc = ctrl.DiscreteADRC(ap, Ts)

    def name(self): return "ADRC"

    def reset(self):
        self._adrc.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        return self._adrc.compute(omega_ref - omega_b)


# ---------------------------------------------------------------------------
# 4. SMC  (compute convention: compute(y - ref))
# ---------------------------------------------------------------------------
class SMCCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        sp = ctrl.SMCParams()
        sp.c_e = 1.0; sp.c_de = 0.2; sp.K = 5.0; sp.phi = 0.5
        sp.uMin = -30.0; sp.uMax = 30.0
        self._smc = ctrl.DiscreteSMC(sp, Ts)

    def name(self): return "SMC"

    def reset(self):
        self._smc.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        # SMC convention: compute(y - ref); output is a correction
        return omega_ref + self._smc.compute(omega_b - omega_ref)


# ---------------------------------------------------------------------------
# 5. LQR (full-state, reference compensation)
# ---------------------------------------------------------------------------
class LQRCtrl(DrillController):
    def __init__(self, p: dict, ss_model):
        self._p   = p
        lqr_p     = ctrl.LQRParams()
        lqr_p.Q   = np.diag([1.0, 50.0])
        lqr_p.R   = np.array([[0.1]])
        self._lqr = ctrl.DiscreteLQR(ss_model, lqr_p)

    def name(self): return "LQR"

    def _phi_ss(self, omega_ref: float) -> float:
        if abs(omega_ref) < 1e-6:
            return 0.0
        p = self._p
        T0    = p['WOB'] * p['R_b']
        strib = p['mu_k'] + (p['mu_s'] - p['mu_k']) * math.exp(-abs(omega_ref) / p['eps_v'])
        Tb    = T0 * strib * math.tanh(omega_ref / p['eps_tanh'])
        return Tb / p['k_t']

    def reset(self): pass

    def compute(self, omega_ref, omega_b, phi, t):
        x     = np.array([phi, omega_b])
        x_ref = np.array([self._phi_ss(omega_ref), omega_ref])
        # u = u_ss + LQR_correction = omega_ref + (-K*(x-x_ref))
        u_delta = float(self._lqr.compute(x, x_ref)[0])
        return omega_ref + u_delta


# ---------------------------------------------------------------------------
# 6. MPC (linearized model, output reference)
# ---------------------------------------------------------------------------
class MPCCtrl(DrillController):
    def __init__(self, p: dict, ss_model):
        mp            = ctrl.MPCParams()
        mp.Np         = 20; mp.Nc = 8
        mp.rho_y      = 50.0; mp.rho_u = 0.01
        mp.uMin       = -30.0; mp.uMax = 30.0
        mp.duMin      = -5.0;  mp.duMax = 5.0
        mp.qp_max_iter = 500
        self._mpc = ctrl.DiscreteMPC(ss_model, mp)

    def name(self): return "MPC"

    def reset(self):
        self._mpc.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        x     = np.array([phi, omega_b])
        r_ref = np.array([omega_ref])
        self._mpc.set_state(x)
        return float(self._mpc.compute_ref(x, r_ref)[0])


# ---------------------------------------------------------------------------
# 7. MRAC  (set_reference then compute(y_plant) - NOT compute(error))
# ---------------------------------------------------------------------------
class MRACCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        mp = ctrl.MRACParams()
        mp.a_m = 0.90; mp.b_m = 0.10
        mp.gamma_r = 5.0; mp.gamma_y = 2.0
        mp.sigma = 0.01; mp.theta_max = 50.0
        self._mrac = ctrl.MRACController(mp, Ts)

    def name(self): return "MRAC"

    def reset(self):
        self._mrac.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        self._mrac.set_reference(omega_ref)
        return self._mrac.compute(omega_b)


# ---------------------------------------------------------------------------
# 8. GainScheduled (2 PIDs blended by |omega_ref|)
# ---------------------------------------------------------------------------
class GainScheduledCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        # Low-speed PID: higher gain for stick-slip suppression
        pp_lo = ctrl.PIDParams()
        pp_lo.Kp = 3.5; pp_lo.Ki = 1.0; pp_lo.Kd = 0.2
        pp_lo.uMin = -30.0; pp_lo.uMax = 30.0; pp_lo.Kb = 0.5
        pid_lo = ctrl.DiscretePID(pp_lo, Ts)
        # High-speed PID: lighter touch
        pp_hi = ctrl.PIDParams()
        pp_hi.Kp = 1.5; pp_hi.Ki = 0.3; pp_hi.Kd = 0.05
        pp_hi.uMin = -30.0; pp_hi.uMax = 30.0; pp_hi.Kb = 0.5
        pid_hi = ctrl.DiscretePID(pp_hi, Ts)

        self._gs = ctrl.GainScheduledController(Ts)
        self._gs.add_schedule_point(0.0,  pid_lo)
        self._gs.add_schedule_point(10.0, pid_hi)

    def name(self): return "GainScheduled"

    def reset(self):
        self._gs.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        self._gs.set_scheduling_param(abs(omega_ref))
        return omega_ref + self._gs.compute(omega_ref - omega_b)


# ---------------------------------------------------------------------------
# 9. L1Adaptive  (set_reference then compute(y_plant) - same as MRAC)
# ---------------------------------------------------------------------------
class L1AdaptiveCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        lp = ctrl.L1AdaptiveParams()
        lp.a_m = 0.90; lp.b_m = 0.10; lp.k_g = 1.0
        lp.Gamma = 50.0; lp.omega_c = 3.0; lp.sigma_max = 50.0
        lp.uMin = -30.0; lp.uMax = 30.0
        self._l1 = ctrl.L1AdaptiveController(lp, Ts)

    def name(self): return "L1Adaptive"

    def reset(self):
        self._l1.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        self._l1.set_reference(omega_ref)
        return self._l1.compute(omega_b)


# ---------------------------------------------------------------------------
# 10. NeuralPID (online backprop adapts Kp/Ki/Kd each step)
# ---------------------------------------------------------------------------
class NeuralPIDCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        np_p = ctrl.NeuralPIDParams()
        np_p.n_hidden = 8; np_p.lr = 0.001; np_p.Ts = Ts
        np_p.plant_gain = 0.5; np_p.max_weight_norm = 10.0
        np_p.uMin = -30.0; np_p.uMax = 30.0
        np_p.Kp0 = 2.0; np_p.Ki0 = 0.4; np_p.Kd0 = 0.1
        self._npid = ctrl.NeuralPID(np_p)

    def name(self): return "NeuralPID"

    def reset(self):
        self._npid.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        return omega_ref + self._npid.compute(omega_ref - omega_b)


# ---------------------------------------------------------------------------
# 11. ILC (Iterative Learning Control - two-phase within one run)
#
# Phase 1 (k < N_TRIAL): pure PID feedback; record errors for trial 0.
# At k = N_TRIAL: update_feedforward() computes trial-1 feedforward.
# Phase 2 (k >= N_TRIAL): PID + ILC feedforward from learned trial.
# ---------------------------------------------------------------------------
class ILCCtrl(DrillController):
    N_TRIAL = 300  # 30 s at Ts=0.1

    def __init__(self, p: dict):
        Ts = p['Ts']
        pp = ctrl.PIDParams()
        pp.Kp = 2.0; pp.Ki = 0.6; pp.Kd = 0.1
        pp.uMin = -30.0; pp.uMax = 30.0; pp.Kb = 0.5
        self._pid = ctrl.DiscretePID(pp, Ts)
        ip = ctrl.ILCParams()
        ip.N = self.N_TRIAL; ip.Ts = Ts
        ip.mode = ctrl.ILCMode.PType
        ip.Lp = 0.6; ip.Q_filter = 0.95
        ip.uMin = -15.0; ip.uMax = 15.0
        self._ilc = ctrl.ILC(ip)
        self._k = 0
        self._phase2 = False

    def name(self): return "ILC"

    def reset(self):
        self._pid.reset()
        self._ilc.reset()
        self._k = 0
        self._phase2 = False

    def compute(self, omega_ref, omega_b, phi, t):
        e = omega_ref - omega_b
        u_pid = self._pid.compute(e)
        if not self._phase2:
            if self._k < self.N_TRIAL:
                self._ilc.record_error(self._k, e)
            self._k += 1
            if self._k == self.N_TRIAL:
                self._ilc.update_feedforward()
                self._phase2 = True
                self._k = 0
            return omega_ref + u_pid
        else:
            k2 = min(self._k, self.N_TRIAL - 1)
            u_ff = self._ilc.feedforward(k2)
            self._k += 1
            return omega_ref + u_pid + u_ff


# ---------------------------------------------------------------------------
# 12. DynaCtrl (Dyna MBRL: PID + SINDy error-dynamics model)
#
# DynaController wraps an IController, accumulates (error, u) transitions,
# and fits a SINDy model after n_collect steps. During simulation it behaves
# identically to the inner PID; the model is available for offline rollouts.
# ---------------------------------------------------------------------------
class DynaCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        pp = ctrl.PIDParams()
        pp.Kp = 2.0; pp.Ki = 0.6; pp.Kd = 0.1
        pp.uMin = -30.0; pp.uMax = 30.0; pp.Kb = 0.5
        pid = ctrl.DiscretePID(pp, Ts)
        dp = ctrl.DynaParams()
        dp.Ts = Ts; dp.n_collect = 80; dp.n_refit_every = 50
        self._dyna = ctrl.DynaController(dp, pid)

    def name(self): return "DynaCtrl"

    def reset(self):
        self._dyna.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        return omega_ref + self._dyna.compute(omega_ref - omega_b)


# ---------------------------------------------------------------------------
# 13. CEMCtrl (Cross-Entropy Method MPC on linearized drill string model)
#
# Uses the ZOH-linearized state function as the CEM rollout model.
# Tracks omega_b via the C = [0,1] output. "Bad fit" for stick-slip
# scenarios where the nonlinear friction makes the linear model inaccurate.
# ---------------------------------------------------------------------------
class CEMCtrl(DrillController):
    def __init__(self, p: dict, ss_model):
        Ts = p['Ts']
        A = ss_model.A; B = ss_model.B

        def _f(x, u):
            return A @ x + B @ u

        C = ss_model.C  # shape (1, 2) - outputs omega_b

        cp = ctrl.CEMParams()
        cp.Np = 10; cp.N_samples = 50; cp.n_iter = 3
        cp.elite_frac = 0.1
        cp.Q = 100.0; cp.R = 0.01
        cp.uMin = -25.0; cp.uMax = 25.0
        cp.sigma_init = 2.0; cp.seed = 7
        self._cem = ctrl.CEMController(cp, _f, C, Ts)

    def name(self): return "CEMCtrl"

    def reset(self):
        self._cem.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        x = np.array([phi, omega_b])
        r = np.array([omega_ref])
        self._cem.set_state(x)
        self._cem.set_reference(r)
        return float(self._cem.compute_ref(x, r)[0])


# ---------------------------------------------------------------------------
# 14. ScenarioMPCCtrl (Scenario MPC - stochastic robustness on linear model)
#
# Averages tracking cost over N_samples noise scenarios. Sigma_w is
# intentionally large to show effect of process-noise averaging.
# ---------------------------------------------------------------------------
class ScenarioMPCCtrl(DrillController):
    def __init__(self, p: dict, ss_model):
        Ts = p['Ts']
        sp = ctrl.ScenarioMPCParams()
        sp.Np = 15; sp.Nu = 5; sp.Ts = Ts
        sp.Q = np.array([[50.0]])
        sp.R = np.array([[0.01]])
        sp.Sigma_w = np.eye(2) * 0.5
        sp.N_samples = 20; sp.seed = 13
        sp.uMin = np.array([-25.0]); sp.uMax = np.array([25.0])
        sp.qp_max_iter = 300; sp.qp_tol = 1e-4
        self._smpc = ctrl.ScenarioMPC(ss_model, sp)

    def name(self): return "ScenarioMPC"

    def reset(self):
        self._smpc.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        x = np.array([phi, omega_b])
        self._smpc.set_state(x)
        self._smpc.set_reference(np.array([omega_ref]))
        return float(self._smpc.compute_control()[0])


# ---------------------------------------------------------------------------
# 15. KoopmanMPCCtrl (EDMD + DiscreteMPC)
#
# Warmup: first 80 steps collect plant snapshots while using PID fallback.
# After warmup: fit_projected() yields a linear model; build DiscreteMPC on it.
# The projected C is replaced to track omega_b only.
# ---------------------------------------------------------------------------
class KoopmanMPCCtrl(DrillController):
    _WARMUP = 80

    def __init__(self, p: dict):
        self._p = p
        self._Ts = p['Ts']
        self._reset_internals()

    def _make_pid(self):
        pp = ctrl.PIDParams()
        pp.Kp = 2.0; pp.Ki = 0.6; pp.Kd = 0.1
        pp.uMin = -30.0; pp.uMax = 30.0; pp.Kb = 0.5
        return ctrl.DiscretePID(pp, self._Ts)

    def _make_edmd(self):
        kp = ctrl.KoopmanEDMDParams()
        kp.n_state = 2; kp.n_input = 1
        kp.dict = ctrl.KoopmanDict.PolyDeg1; kp.tikhonov = 1e-6
        return ctrl.KoopmanEDMD(kp)

    def _reset_internals(self):
        self._pid   = self._make_pid()
        self._edmd  = self._make_edmd()
        self._mpc   = None
        self._step  = 0
        self._x_prev = None
        self._u_prev = 0.0
        self._fitted = False

    def name(self): return "KoopmanMPC"

    def reset(self):
        self._reset_internals()

    def compute(self, omega_ref, omega_b, phi, t):
        x_now = np.array([phi, omega_b])
        e = omega_ref - omega_b

        # Accumulate snapshots from previous step
        if self._step > 0 and not self._fitted and self._x_prev is not None:
            self._edmd.add_snapshot(
                self._x_prev, np.array([self._u_prev]), x_now)

            if self._edmd.snapshot_count() >= self._WARMUP:
                try:
                    ss_proj = self._edmd.fit_projected()
                    # Replace C to track omega_b (state index 1) and fix Ts
                    C_new = np.array([[0.0, 1.0]])
                    D_new = np.array([[0.0]])
                    ss_fixed = ctrl.StateSpace(
                        ss_proj.A, ss_proj.B, C_new, D_new, self._Ts)
                    mp = ctrl.MPCParams()
                    mp.Np = 15; mp.Nc = 5
                    mp.rho_y = 50.0; mp.rho_u = 0.01
                    mp.uMin = -25.0; mp.uMax = 25.0
                    mp.duMin = -5.0; mp.duMax = 5.0
                    mp.qp_max_iter = 300
                    self._mpc = ctrl.DiscreteMPC(ss_fixed, mp)
                    self._fitted = True
                except Exception:
                    pass  # fit failed; keep using PID

        # Compute control
        if self._fitted and self._mpc is not None:
            try:
                self._mpc.set_state(x_now)
                u = float(self._mpc.compute_ref(x_now, np.array([omega_ref]))[0])
            except Exception:
                u = omega_ref + self._pid.compute(e)
        else:
            u = omega_ref + self._pid.compute(e)

        self._x_prev = x_now.copy()
        self._u_prev = u
        self._step  += 1
        return u


# ---------------------------------------------------------------------------
# 16. ESNCtrl (Echo State Network predictor + model-inversion correction)
#
# Pre-trained on synthetic drill-string dynamics in __init__.
# At each step: predict omega_b_next using (u_prev, omega_b) as input;
# add a correction proportional to predicted tracking error.
# ---------------------------------------------------------------------------
class ESNCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        ep = ctrl.ESNParams()
        ep.n_res = 50; ep.n_in = 2; ep.n_out = 1
        ep.spectral_radius = 0.9; ep.sparsity = 0.85
        ep.alpha = 0.3; ep.ridge = 1e-4; ep.washout = 20; ep.seed = 42
        self._esn = ctrl.EchoStateNetwork(ep)

        # Pre-train on simplified drill string: omega_b_next approx = a*omega_b + b*omega_t
        c_t = p['c_t']; J_b = p['J_b']
        a = 1.0 - (c_t / J_b) * Ts   # approx = 0.973
        b = (c_t / J_b) * Ts          # approx = 0.027
        rng = np.random.default_rng(42)
        ob = 0.0
        for _ in range(300):
            ot = float(rng.uniform(-20.0, 20.0))
            ob_next = a * ob + b * ot
            self._esn.step_reservoir(np.array([ot, ob]))
            self._esn.add_training_target(np.array([ob_next]))
            ob = ob_next
        self._esn.fit_readout()
        self._esn.reset()
        self._u_prev = 0.0

    def name(self): return "ESNCtrl"

    def reset(self):
        self._esn.reset()
        self._u_prev = 0.0

    def compute(self, omega_ref, omega_b, phi, t):
        # Predict omega_b_next using previous control and current state
        pred_next = float(self._esn.predict(
            np.array([self._u_prev, omega_b]))[0])
        e = omega_ref - omega_b
        pred_err = omega_ref - pred_next
        # Proportional tracking + prediction-based feedforward correction
        u = omega_ref + 2.0 * e + 0.5 * pred_err
        u = float(np.clip(u, -25.0, 25.0))
        self._u_prev = u
        return u


# ---------------------------------------------------------------------------
# 17. CBFCtrl (Control Barrier Function safety filter wrapping PID)
#
# Nominal: PID correction. CBF constrains the correction with barrier
# h(omega_b) = omega_max - omega_b >= 0 (upper speed limit).
# Note: CBF constrains the correction, not the full omega_t; the safety
# guarantee is approximate. Intentional "bad fit" for comparison analysis.
# ---------------------------------------------------------------------------
class CBFCtrl(DrillController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        omega_max = p['omega_max']

        pp = ctrl.PIDParams()
        pp.Kp = 2.0; pp.Ki = 0.6; pp.Kd = 0.1
        pp.uMin = -omega_max; pp.uMax = omega_max; pp.Kb = 0.5
        pid = ctrl.DiscretePID(pp, Ts)

        cbf_p = ctrl.CBFParams()
        cbf_p.alpha = 2.0
        cbf_p.uMin = -omega_max; cbf_p.uMax = omega_max

        self._cbf = ctrl.CBFSafetyFilter(
            pid,
            lambda x: omega_max - x,   # h(omega_b) = omega_max - omega_b
            lambda x: -1.0,             # dh/dx
            lambda x: 0.0,              # f0: no drift term
            lambda x: 1.0,              # g: unit gain on correction
            cbf_p,
            Ts
        )
        self._omega_max = omega_max

    def name(self): return "CBFSafety"

    def reset(self):
        self._cbf.reset()

    def compute(self, omega_ref, omega_b, phi, t):
        self._cbf.set_state(omega_b)
        correction = self._cbf.compute(omega_ref - omega_b)
        return omega_ref + correction


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def make_controllers(plant_params: dict):
    """Return a list of all 17 DrillController instances."""
    controllers = [OpenLoopCtrl()]
    if not CTRL_AVAILABLE:
        print("WARNING: ctrl_toolbox not available - only OpenLoop will run.")
        return controllers

    ss = build_linear_model(plant_params, omega_nom=10.0)
    controllers += [
        PIDCtrl(plant_params),
        ADRCCtrl(plant_params),
        SMCCtrl(plant_params),
        LQRCtrl(plant_params, ss),
        MPCCtrl(plant_params, ss),
        MRACCtrl(plant_params),
        GainScheduledCtrl(plant_params),
        L1AdaptiveCtrl(plant_params),
        NeuralPIDCtrl(plant_params),
        ILCCtrl(plant_params),
        DynaCtrl(plant_params),
        CEMCtrl(plant_params, ss),
        ScenarioMPCCtrl(plant_params, ss),
        KoopmanMPCCtrl(plant_params),
        ESNCtrl(plant_params),
        CBFCtrl(plant_params),
    ]
    return controllers
