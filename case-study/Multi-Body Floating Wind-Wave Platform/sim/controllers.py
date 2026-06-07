"""
controllers.py
Eight controllers for the FOWT+WEC power-extraction case study.

All controllers share the interface:
  name()  -> str
  reset() -> None
  compute(x_ref, plant_state, t) -> float  (returns F_PTO command [N])

plant_state = (z, zdot, x_rel, xrel_dot)
x_ref       = target WEC arm displacement [m]  (sinusoidal reference for
              power-extraction tracking; passive/reactive controllers ignore it)

ctrl_toolbox is imported with graceful fallback.
"""

import sys
import os
import math
import numpy as np

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


def build_linear_model(p: dict):
    """Linearized 4-state FOWT+WEC SS model discretized at Ts."""
    if not CTRL_AVAILABLE:
        return None
    try:
        import scipy.signal as sig
    except ImportError:
        return None

    Ts = p['Ts']
    M_f = p['M_f']; B_f = p['B_f']; K_f = p['K_f']
    M_w = p['M_w']; B_w = p['B_w']; K_w = p['K_w']

    A_c = np.array([
        [0,       1,         0,         0],
        [-K_f/M_f, -B_f/M_f,  0,         0],
        [0,       0,         0,         1],
        [0,       0,        -K_w/M_w,  -B_w/M_w],
    ])
    # Input: F_PTO enters only WEC equation with sign -1/M_w
    B_c = np.array([[0], [0], [0], [-1.0/M_w]])
    # Output: x_rel (state index 2)
    C   = np.array([[0, 0, 1, 0]])
    D   = np.array([[0.0]])

    Ad, Bd, Cd, Dd, _ = sig.cont2discrete((A_c, B_c, C, D), Ts, method='zoh')
    return ctrl.StateSpace(Ad, Bd, Cd, Dd, Ts)


# ---------------------------------------------------------------------------
# Base
# ---------------------------------------------------------------------------
class WaveController:
    def name(self):    return "BaseController"
    def reset(self):   pass
    def compute(self, x_ref, plant_state, t): return 0.0


# ---------------------------------------------------------------------------
# 1. Passive (constant PTO damping, no reference)
# ---------------------------------------------------------------------------
class PassiveCtrl(WaveController):
    def __init__(self, p: dict):
        # PTO damping coefficient [N*s/m]
        K_w = p['K_w']; M_w = p['M_w']
        # Optimal resistive: B_opt = sqrt(K_w * M_w) = radiation resistance
        self._B_pto = math.sqrt(K_w * M_w)

    def name(self): return "Passive"

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, xrel_dot = plant_state
        # Damping only: F_PTO = B * velocity
        return self._B_pto * xrel_dot


# ---------------------------------------------------------------------------
# 2. Reactive (spring + damping - complex-conjugate approximation)
# ---------------------------------------------------------------------------
class ReactiveCtrl(WaveController):
    def __init__(self, p: dict):
        K_w = p['K_w']; M_w = p['M_w']; B_w = p['B_w']
        # Reactive: cancel internal stiffness + add optimal damping
        self._K_r   = -K_w       # cancel plant stiffness
        self._B_pto = math.sqrt(K_w * M_w)

    def name(self): return "Reactive"

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, xrel_dot = plant_state
        return self._B_pto * xrel_dot + self._K_r * x_rel


# ---------------------------------------------------------------------------
# 3. PID (track sinusoidal x_rel reference)
# ---------------------------------------------------------------------------
class PIDCtrl(WaveController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        pp = ctrl.PIDParams()
        pp.Kp = 5.0e4; pp.Ki = 1.0e3; pp.Kd = 2.0e3
        pp.uMin = -p['F_max']; pp.uMax = p['F_max']; pp.Kb = 0.5
        self._pid = ctrl.DiscretePID(pp, Ts)

    def name(self): return "PID"

    def reset(self):
        self._pid.reset()

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, _ = plant_state
        return self._pid.compute(x_ref - x_rel)


# ---------------------------------------------------------------------------
# 4. ADRC (treat x_rel as plant output, F_PTO as input)
# ---------------------------------------------------------------------------
class ADRCCtrl(WaveController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        ap = ctrl.ADRCParams()
        # omega_o * Ts must be < 0.5; with Ts=0.5: omega_o < 1.0
        ap.omega_c = 0.3; ap.omega_o = 0.8; ap.b0 = 1.0 / p['M_w']
        ap.uMin = -p['F_max']; ap.uMax = p['F_max']
        self._adrc = ctrl.DiscreteADRC(ap, Ts)

    def name(self): return "ADRC"

    def reset(self):
        self._adrc.reset()

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, _ = plant_state
        return self._adrc.compute(x_ref - x_rel)


# ---------------------------------------------------------------------------
# 5. SMC (sliding-mode tracking of x_rel)
# ---------------------------------------------------------------------------
class SMCCtrl(WaveController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        sp = ctrl.SMCParams()
        sp.c_e = 0.5; sp.c_de = 0.1; sp.K = 1.0e4; sp.phi = 0.1
        sp.uMin = -p['F_max']; sp.uMax = p['F_max']
        self._smc = ctrl.DiscreteSMC(sp, Ts)

    def name(self): return "SMC"

    def reset(self):
        self._smc.reset()

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, _ = plant_state
        # SMC convention: compute(y - ref)
        return self._smc.compute(x_rel - x_ref)


# ---------------------------------------------------------------------------
# 6. LQR (full-state, reference on x_rel only)
# ---------------------------------------------------------------------------
class LQRCtrl(WaveController):
    def __init__(self, p: dict, ss_model):
        lqr_p   = ctrl.LQRParams()
        # Penalize x_rel deviation and xrel_dot; light weight on FOWT states
        lqr_p.Q = np.diag([1.0e-2, 1.0e-4, 1.0e4, 1.0e2])
        lqr_p.R = np.array([[1.0e-10]])
        self._lqr = ctrl.DiscreteLQR(ss_model, lqr_p)

    def name(self): return "LQR"

    def compute(self, x_ref, plant_state, t):
        z, zd, xr, xrd = plant_state
        x     = np.array([z, zd, xr, xrd])
        x_ref_v = np.array([0.0, 0.0, x_ref, 0.0])  # desired: FOWT at 0, WEC at x_ref
        return float(self._lqr.compute(x, x_ref_v)[0])


# ---------------------------------------------------------------------------
# 7. MPC (linear MPC tracking x_rel reference)
# ---------------------------------------------------------------------------
class MPCCtrl(WaveController):
    def __init__(self, p: dict, ss_model):
        mp            = ctrl.MPCParams()
        mp.Np         = 10; mp.Nc = 4
        mp.rho_y      = 1.0e4; mp.rho_u = 1.0e-8
        mp.uMin       = -p['F_max']; mp.uMax = p['F_max']
        mp.duMin      = -1.0e5;      mp.duMax = 1.0e5
        mp.qp_max_iter = 300
        self._mpc = ctrl.DiscreteMPC(ss_model, mp)

    def name(self): return "MPC"

    def reset(self):
        self._mpc.reset()

    def compute(self, x_ref, plant_state, t):
        z, zd, xr, xrd = plant_state
        x     = np.array([z, zd, xr, xrd])
        r_ref = np.array([x_ref])
        self._mpc.set_state(x)
        return float(self._mpc.compute_ref(x, r_ref)[0])


# ---------------------------------------------------------------------------
# 8. MRAC (adaptive tracking of x_rel)
# ---------------------------------------------------------------------------
class MRACCtrl(WaveController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        mp = ctrl.MRACParams()
        mp.a_m = 0.85; mp.b_m = 0.15
        mp.gamma_r = 2.0; mp.gamma_y = 1.0
        mp.sigma = 0.01; mp.theta_max = 200.0
        self._mrac = ctrl.MRACController(mp, Ts)

    def name(self): return "MRAC"

    def reset(self):
        self._mrac.reset()

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, _ = plant_state
        self._mrac.set_reference(x_ref)
        return self._mrac.compute(x_rel)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def make_controllers(plant_params: dict):
    """Return a list of all 8 WaveController instances."""
    ctrl_list = [PassiveCtrl(plant_params), ReactiveCtrl(plant_params)]
    if not CTRL_AVAILABLE:
        print("WARNING: ctrl_toolbox not available - only Passive/Reactive will run.")
        return ctrl_list

    ss = build_linear_model(plant_params)
    ctrl_list += [
        PIDCtrl(plant_params),
        ADRCCtrl(plant_params),
        SMCCtrl(plant_params),
        LQRCtrl(plant_params, ss),
        MPCCtrl(plant_params, ss),
        MRACCtrl(plant_params),
    ]
    return ctrl_list
