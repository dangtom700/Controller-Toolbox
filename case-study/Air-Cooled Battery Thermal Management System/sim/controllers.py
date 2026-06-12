"""
controllers.py
Twelve thermal management controllers for the Air-Cooled BTMS case study.

Controller interface:
  name()  -> str
  reset() -> None
  compute(DeltaT, x_norm_hot, soc, t) -> str   ('J', 'U', or 'L')

Design convention:
  - SelfAdaptive implements the paper's position-based switching rule directly.
  - ctrl_toolbox controllers operate on error e = DeltaT_ref - DeltaT and output
    a continuous threshold adjustment u in [-0.5, 0.5] K.  The flow pattern is then
    selected by the same position rule with an adjusted switching threshold:
        DeltaT_thresh = max(0.1, DeltaT_lim + u)
  - When DeltaT < DeltaT_activate (0.3 K hysteresis), no switch is forced.

ADRC constraint: omega_o * Ts = 0.3 * 1.0 = 0.30 < 0.5 (satisfied).

ctrl_toolbox is imported with a graceful fallback; controllers that require it
fall back to SelfAdaptive when the binding is unavailable.
"""

import sys
import os
import math

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


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------
DT_LIM      = 1.0   # K  target maximum temperature difference (paper's DeltaT_lim)
DT_REF      = 0.5   # K  internal control target for error-based controllers
DT_ACTIVATE = 0.3   # K  hysteresis: below this, keep current pattern

PATTERN_ENCODING = {'J': 1.0, 'L': 0.0, 'U': -1.0}


def _position_rule(x_norm: float) -> str:
    """Paper's Eq. 2.2: map hotspot position to flow pattern."""
    if x_norm <= 0.25:
        return 'U'
    elif x_norm <= 0.75:
        return 'L'
    else:
        return 'J'


def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------
class BTMSController:
    def name(self):   return "Base"
    def reset(self):  pass
    def compute(self, DeltaT: float, x_norm_hot: float,
                soc: float, t: float) -> str:
        return 'J'


# ---------------------------------------------------------------------------
# 1. OpenLoop -- fixed J-type, no switching (baseline)
# ---------------------------------------------------------------------------
class OpenLoopCtrl(BTMSController):
    def name(self): return "OpenLoop"

    def compute(self, DeltaT, x_norm_hot, soc, t):
        return 'J'


# ---------------------------------------------------------------------------
# 2. SelfAdaptive -- paper's strategy (Section 2.2)
#    Modified criterion: DeltaT > DeltaT_lim*(1-delta) - eps
# ---------------------------------------------------------------------------
class SelfAdaptiveCtrl(BTMSController):
    DELTA   = 0.07    # model correction factor (7%, Eq. 17)
    EPS     = 0.01    # transition smoothing [K]

    def __init__(self):
        self._prev = 'J'

    def name(self): return "SelfAdaptive"

    def reset(self):
        self._prev = 'J'

    def compute(self, DeltaT, x_norm_hot, soc, t):
        threshold = DT_LIM * (1.0 - self.DELTA) - self.EPS   # 0.92 K
        if DeltaT <= threshold:
            return self._prev
        mode = _position_rule(x_norm_hot)
        self._prev = mode
        return mode


# ---------------------------------------------------------------------------
# Helper: ctrl_toolbox wrapper that maps u -> flow pattern via position rule
# The ctrl_toolbox controller produces u = threshold_adjustment in [-0.5, 0.5].
# Actual switching threshold = DT_LIM + u (clamped to [0.1, 1.5]).
# Only switches when DeltaT > adjusted threshold.
# ---------------------------------------------------------------------------
class _CtrlToolboxBase(BTMSController):
    def __init__(self):
        self._prev = 'J'

    def _ctrl_output(self, e: float) -> float:
        """Override: return threshold adjustment u from ctrl_toolbox controller."""
        return 0.0

    def compute(self, DeltaT, x_norm_hot, soc, t):
        e = DT_REF - DeltaT   # positive when DeltaT is below target (less urgent)
        u = _clamp(self._ctrl_output(e), -0.5, 0.5)
        # Adjusted threshold: high u (DeltaT below target) -> raise threshold (less switching)
        # Low u (DeltaT above target) -> lower threshold (more aggressive switching)
        threshold = _clamp(DT_LIM + u, 0.1, 1.5)
        if DeltaT < DT_ACTIVATE:
            return self._prev
        if DeltaT <= threshold:
            return self._prev
        mode = _position_rule(x_norm_hot)
        self._prev = mode
        return mode


# ---------------------------------------------------------------------------
# 3. PID
# ---------------------------------------------------------------------------
class PIDCtrl(_CtrlToolboxBase):
    def __init__(self, p: dict):
        super().__init__()
        Ts   = float(p.get('Ts', 1.0))
        pp   = ctrl.PIDParams()
        pp.Kp = 0.4;  pp.Ki = 0.02;  pp.Kd = 0.05
        pp.uMin = -0.5;  pp.uMax = 0.5;  pp.Kb = 0.3
        self._pid = ctrl.DiscretePID(pp, Ts)

    def name(self): return "PID"

    def reset(self):
        super().reset()
        self._pid.reset()

    def _ctrl_output(self, e):
        return float(self._pid.compute(e))


# ---------------------------------------------------------------------------
# 4. ADRC  -- omega_o=0.3, Ts=1.0 -> omega_o*Ts=0.30 < 0.5 (check)
# ---------------------------------------------------------------------------
class ADRCCtrl(_CtrlToolboxBase):
    def __init__(self, p: dict):
        super().__init__()
        Ts  = float(p.get('Ts', 1.0))
        # First-order BTMS DeltaT model: tauapprox =120s, Kapprox =0.5 -> b0=K*(1-exp(-Ts/tau))=0.5*(1-exp(-1/120))
        b0  = 0.5 * (1.0 - math.exp(-Ts / 120.0))
        ap  = ctrl.ADRCParams()
        ap.omega_c = 0.1;  ap.omega_o = 0.3;  ap.b0 = b0
        ap.uMin = -0.5;  ap.uMax = 0.5
        self._adrc = ctrl.DiscreteADRC(ap, Ts)

    def name(self): return "ADRC"

    def reset(self):
        super().reset()
        self._adrc.reset()

    def _ctrl_output(self, e):
        return float(self._adrc.compute(e))


# ---------------------------------------------------------------------------
# 5. SMC  (compute(y - ref) convention: DiscreteSMC takes y - ref = -e)
# ---------------------------------------------------------------------------
class SMCCtrl(_CtrlToolboxBase):
    def __init__(self, p: dict):
        super().__init__()
        Ts = float(p.get('Ts', 1.0))
        sp = ctrl.SMCParams()
        sp.c_e = 1.0;  sp.c_de = 8.0;  sp.K = 0.3;  sp.phi = 0.05
        sp.uMin = -0.5;  sp.uMax = 0.5
        self._smc = ctrl.DiscreteSMC(sp, Ts)

    def name(self): return "SMC"

    def reset(self):
        super().reset()
        self._smc.reset()

    def _ctrl_output(self, e):
        # DiscreteSMC convention: compute(y - ref) = compute(-e)
        return float(self._smc.compute(-e))


# ---------------------------------------------------------------------------
# 6. MPC -- explicit 1-step lookahead over J/U/L (native Python)
#    Evaluates predicted DeltaT one step ahead for each pattern and picks
#    the one that minimizes predicted DeltaT.  No ctrl_toolbox required.
# ---------------------------------------------------------------------------
class MPCCtrl(BTMSController):
    def __init__(self, p: dict):
        self._Ts     = float(p.get('Ts', 1.0))
        self._prev   = 'J'
        self._plant_ref = None   # set by simulation_runner each step via _set_plant

    def name(self): return "MPC"

    def reset(self):
        self._prev = 'J'

    def set_plant_ref(self, plant):
        """Simulation runner injects current plant state for prediction."""
        self._plant_ref = plant

    def compute(self, DeltaT, x_norm_hot, soc, t):
        if DeltaT < DT_ACTIVATE:
            return self._prev

        if self._plant_ref is None:
            # No plant reference: fall back to SelfAdaptive logic
            threshold = DT_LIM * 0.93 - 0.01
            if DeltaT <= threshold:
                return self._prev
            mode = _position_rule(x_norm_hot)
            self._prev = mode
            return mode

        # Shallow copy plant state for each prediction
        import copy
        best_mode = self._prev
        best_dt   = 1e9
        for candidate in ('J', 'U', 'L'):
            p_copy = copy.deepcopy(self._plant_ref)
            p_copy.set_pattern(candidate)
            # Use last known phi_b (approximate)
            # We'll estimate phi from current DeltaT growth rate (simplified)
            phi_est = self._plant_ref._phi_cache if hasattr(self._plant_ref, '_phi_cache') else 6e4
            p_copy.step(phi_est, self._Ts)
            predicted_dt = p_copy.delta_T()
            if predicted_dt < best_dt:
                best_dt   = predicted_dt
                best_mode = candidate

        self._prev = best_mode
        return best_mode


# ---------------------------------------------------------------------------
# 7. LQR -- 1-state discrete LQR on DeltaT model
#    State: x = DeltaT,  Control: u = threshold adjustment
# ---------------------------------------------------------------------------
class LQRCtrl(_CtrlToolboxBase):
    def __init__(self, p: dict, ss_model):
        super().__init__()
        self._ss = ss_model
        if ss_model is None:
            return
        lp    = ctrl.LQRParams()
        # Q: penalize DeltaT deviation; R: penalize threshold changes
        lp.Q  = np.array([[2.0]])
        lp.R  = np.array([[1.0]])
        self._lqr = ctrl.DiscreteLQR(ss_model, lp)

    def name(self): return "LQR"

    def reset(self):
        super().reset()

    def _ctrl_output(self, e):
        if self._ss is None:
            return 0.0
        x     = np.array([DT_REF - e])    # x = DeltaT
        x_ref = np.array([DT_REF])        # target DeltaT
        u_raw = self._lqr.compute(x, x_ref)
        return _clamp(float(u_raw[0]), -0.5, 0.5)


# ---------------------------------------------------------------------------
# 8. MRAC (set_reference then compute(y) convention)
# ---------------------------------------------------------------------------
class MRACCtrl(_CtrlToolboxBase):
    def __init__(self, p: dict):
        super().__init__()
        Ts  = float(p.get('Ts', 1.0))
        # Reference model: slow pole matching thermal dynamics tau~120s
        a_m = math.exp(-Ts / 120.0)
        b_m = 1.0 - a_m
        mp  = ctrl.MRACParams()
        mp.a_m = a_m;  mp.b_m = b_m
        # Positive-gain plant: higher threshold reduces switching activity -> lower DeltaT signal
        mp.gamma_r = 0.01;  mp.gamma_y = 0.005
        mp.sigma   = 0.01;  mp.theta_max = 0.5
        self._mrac  = ctrl.MRACController(mp, Ts)
        self._DT_ref = DT_REF

    def name(self): return "MRAC"

    def reset(self):
        super().reset()
        self._mrac.reset()

    def _ctrl_output(self, e):
        DeltaT = DT_REF - e
        self._mrac.set_reference(self._DT_ref)
        u_raw = self._mrac.compute(DeltaT)
        return _clamp(float(u_raw) - self._DT_ref, -0.5, 0.5)


# ---------------------------------------------------------------------------
# 9. L1Adaptive
# ---------------------------------------------------------------------------
class L1AdaptiveCtrl(_CtrlToolboxBase):
    def __init__(self, p: dict):
        super().__init__()
        Ts  = float(p.get('Ts', 1.0))
        a_m = math.exp(-Ts / 120.0)
        b_m = 1.0 - a_m
        lp  = ctrl.L1AdaptiveParams()
        lp.a_m = a_m;  lp.b_m = b_m;  lp.k_g = 1.0
        lp.Gamma     = 0.05;  lp.omega_c = 0.08
        lp.sigma_max = 0.5
        lp.uMin = DT_REF - 0.5;  lp.uMax = DT_REF + 0.5
        self._l1    = ctrl.L1AdaptiveController(lp, Ts)
        self._DT_ref = DT_REF

    def name(self): return "L1Adaptive"

    def reset(self):
        super().reset()
        self._l1.reset()

    def _ctrl_output(self, e):
        DeltaT = DT_REF - e
        self._l1.set_reference(self._DT_ref)
        u_raw = self._l1.compute(DeltaT)
        return _clamp(float(u_raw) - self._DT_ref, -0.5, 0.5)


# ---------------------------------------------------------------------------
# 10. NeuralPID
# ---------------------------------------------------------------------------
class NeuralPIDCtrl(_CtrlToolboxBase):
    def __init__(self, p: dict):
        super().__init__()
        Ts   = float(p.get('Ts', 1.0))
        # Plant gain: first-order step response scale for DeltaT dynamics
        # K*(1-exp(-Ts/tau)) = 0.5*(1-exp(-1/120)) ~ 0.00415
        plant_gain = 0.5 * (1.0 - math.exp(-Ts / 120.0))
        np_p = ctrl.NeuralPIDParams()
        np_p.n_hidden = 8;  np_p.lr = 5e-4;  np_p.Ts = Ts
        np_p.plant_gain      = plant_gain
        np_p.max_weight_norm = 5.0
        np_p.uMin = -0.5;  np_p.uMax = 0.5
        np_p.Kp0  = 0.4;   np_p.Ki0  = 0.02;  np_p.Kd0 = 0.05
        self._npid = ctrl.NeuralPID(np_p)

    def name(self): return "NeuralPID"

    def reset(self):
        super().reset()
        self._npid.reset()

    def _ctrl_output(self, e):
        return float(self._npid.compute(e))


# ---------------------------------------------------------------------------
# 11. GainScheduled -- two PIDs blended on SOC (high SOC = gentle, low = aggressive)
# ---------------------------------------------------------------------------
class GainScheduledCtrl(_CtrlToolboxBase):
    def __init__(self, p: dict):
        super().__init__()
        Ts = float(p.get('Ts', 1.0))

        # High SOC: gentle PID (battery still cool, less urgency)
        pp_hi = ctrl.PIDParams()
        pp_hi.Kp = 0.2;  pp_hi.Ki = 0.01;  pp_hi.Kd = 0.02
        pp_hi.uMin = -0.5;  pp_hi.uMax = 0.5;  pp_hi.Kb = 0.3
        pid_hi = ctrl.DiscretePID(pp_hi, Ts)

        # Low SOC: aggressive PID (batteries hotter, more urgency)
        pp_lo = ctrl.PIDParams()
        pp_lo.Kp = 0.6;  pp_lo.Ki = 0.04;  pp_lo.Kd = 0.08
        pp_lo.uMin = -0.5;  pp_lo.uMax = 0.5;  pp_lo.Kb = 0.3
        pid_lo = ctrl.DiscretePID(pp_lo, Ts)

        self._gs  = ctrl.GainScheduledController(Ts)
        self._gs.add_schedule_point(0.0, pid_lo)   # SOC=0 -> aggressive
        self._gs.add_schedule_point(1.0, pid_hi)   # SOC=1 -> gentle
        self._soc = 1.0

    def name(self): return "GainScheduled"

    def reset(self):
        super().reset()
        self._gs.reset()
        self._soc = 1.0

    def compute(self, DeltaT, x_norm_hot, soc, t):
        self._soc = soc
        return super().compute(DeltaT, x_norm_hot, soc, t)

    def _ctrl_output(self, e):
        self._gs.set_scheduling_param(self._soc)
        return _clamp(float(self._gs.compute(e)), -0.5, 0.5)


# ---------------------------------------------------------------------------
# 12. ILC (Iterative Learning Control)
#    Learns the optimal switching sequence trial-to-trial.
#    Phase 1 (k < N_TRIAL): uses SelfAdaptive; records DeltaT error.
#    Phase 2: PID + learned feedforward threshold correction.
# ---------------------------------------------------------------------------
class ILCCtrl(_CtrlToolboxBase):
    N_TRIAL = 720   # one full 5C discharge at Ts=1s

    def __init__(self, p: dict):
        super().__init__()
        Ts = float(p.get('Ts', 1.0))
        # Inner PID
        pp = ctrl.PIDParams()
        pp.Kp = 0.4;  pp.Ki = 0.02;  pp.Kd = 0.05
        pp.uMin = -0.5;  pp.uMax = 0.5;  pp.Kb = 0.3
        self._pid = ctrl.DiscretePID(pp, Ts)
        # ILC
        ip = ctrl.ILCParams()
        ip.N          = self.N_TRIAL;  ip.Ts = Ts
        ip.mode       = ctrl.ILCMode.PType
        ip.Lp         = 0.5;  ip.Q_filter = 0.95
        ip.uMin       = -0.5;  ip.uMax = 0.5
        self._ilc     = ctrl.ILC(ip)
        self._k       = 0
        self._phase2  = False
        self._sa      = SelfAdaptiveCtrl()

    def name(self): return "ILC"

    def reset(self):
        super().reset()
        self._pid.reset()
        self._ilc.reset()
        self._sa.reset()
        self._k      = 0
        self._phase2 = False

    def _ctrl_output(self, e):
        u_pid = float(self._pid.compute(e))
        if not self._phase2:
            if self._k < self.N_TRIAL:
                self._ilc.record_error(self._k, e)
            self._k += 1
            if self._k == self.N_TRIAL:
                self._ilc.update_feedforward()
                self._phase2 = True
                self._k = 0
            return u_pid
        else:
            k2   = min(self._k, self.N_TRIAL - 1)
            u_ff = float(self._ilc.feedforward(k2))
            self._k += 1
            return _clamp(u_pid + u_ff, -0.5, 0.5)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def _build_ss_model(p: dict):
    """Build 1-state ZOH discrete state-space for DeltaT dynamics."""
    if not CTRL_AVAILABLE:
        return None
    try:
        Ts  = float(p.get('Ts', 1.0))
        tau = 120.0   # thermal time constant [s]
        K   = 0.5     # steady-state gain [K / (K threshold shift)]
        a   = math.exp(-Ts / tau)
        b   = K * (1.0 - a)
        Ad  = np.array([[a]])
        Bd  = np.array([[b]])
        Cd  = np.array([[1.0]])
        Dd  = np.array([[0.0]])
        return ctrl.StateSpace(Ad, Bd, Cd, Dd, Ts)
    except Exception:
        return None


def make_controllers(plant_params: dict) -> list:
    """Return list of all 12 BTMSController instances."""
    controllers = [
        OpenLoopCtrl(),
        SelfAdaptiveCtrl(),
    ]

    if not CTRL_AVAILABLE:
        print("WARNING: ctrl_toolbox not available -- only OpenLoop + SelfAdaptive will run.")
        return controllers

    ss = _build_ss_model(plant_params)
    controllers += [
        PIDCtrl(plant_params),
        ADRCCtrl(plant_params),
        SMCCtrl(plant_params),
        MPCCtrl(plant_params),
        LQRCtrl(plant_params, ss),
        MRACCtrl(plant_params),
        L1AdaptiveCtrl(plant_params),
        NeuralPIDCtrl(plant_params),
        GainScheduledCtrl(plant_params),
        ILCCtrl(plant_params),
    ]
    return controllers
