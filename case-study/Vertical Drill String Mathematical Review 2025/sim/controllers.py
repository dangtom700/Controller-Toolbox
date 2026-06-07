"""
controllers.py
Ten controllers for the drill string velocity tracking case study.

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
sys.path.insert(0, os.path.join(_ROOT, 'build', 'bindings'))
if sys.platform == 'win32' and hasattr(os, 'add_dll_directory'):
    for _p in [r'C:\msys64\mingw64\bin']:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

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
# Factory
# ---------------------------------------------------------------------------
def make_controllers(plant_params: dict):
    """Return a list of all 10 DrillController instances."""
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
    ]
    return controllers
