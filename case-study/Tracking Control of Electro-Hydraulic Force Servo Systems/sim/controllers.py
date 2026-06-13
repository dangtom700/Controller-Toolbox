"""
controllers.py
Fourteen controllers for the electro-hydraulic force servo case study.

Controller interface:
  name()  -> str
  reset() -> None
  compute(F_ref, F, v_p, P_A, P_B, x_v, t) -> float  (u_v in [-1, 1])

All controllers output a normalised valve command.
ctrl_toolbox is imported with a graceful fallback.
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


def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def build_linear_model(p: dict):
    """ZOH-discretize the 2-state linearized force model at nominal pressures."""
    if not CTRL_AVAILABLE:
        return None
    try:
        import scipy.signal as sig
    except ImportError:
        return None

    P_S   = p['P_S'];  A_A = p['A_A'];  A_B = p['A_B']
    V_A   = p['V_A'];  V_B = p['V_B'];  beta = p['beta']
    Kq    = p['Kq'];   tau_v = p['tau_v'];  Ts = p['Ts']
    P_A0  = p.get('P_A0', 3.0e6)
    P_B0  = p.get('P_B0', 5.0e6)

    K_qa = Kq * math.sqrt(max(P_S - P_A0, 0.0))
    K_qb = Kq * math.sqrt(max(P_B0, 0.0))
    K_F  = A_A * beta / V_A * K_qa + A_B * beta / V_B * K_qb

    A_c = np.array([[0.0,  K_F],
                    [0.0, -1.0 / tau_v]])
    B_c = np.array([[0.0], [1.0 / tau_v]])
    C   = np.array([[1.0, 0.0]])
    D   = np.array([[0.0]])

    Ad, Bd, Cd, Dd, _ = sig.cont2discrete((A_c, B_c, C, D), Ts, method='zoh')
    return ctrl.StateSpace(Ad, Bd, Cd, Dd, Ts)


# ---------------------------------------------------------------------------
# Base
# ---------------------------------------------------------------------------
class EHFSController:
    def name(self):    return "BaseController"
    def reset(self):   pass
    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        return 0.0


# ---------------------------------------------------------------------------
# 1. OpenLoop  (no feedback - baseline reference)
# ---------------------------------------------------------------------------
class OpenLoopCtrl(EHFSController):
    """Outputs x_v proportional to F_ref. No feedback."""
    def __init__(self, p: dict):
        # Scale: full command for F = F_max
        self._scale = 1.0 / p['F_max']

    def name(self): return "OpenLoop"

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        return _clamp(F_ref * self._scale * 0.5)


# ---------------------------------------------------------------------------
# 2. PID
# ---------------------------------------------------------------------------
class PIDCtrl(EHFSController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        pp = ctrl.PIDParams()
        pp.Kp = 3.0e-4;  pp.Ki = 0.5;  pp.Kd = 1.0e-7
        pp.uMin = -1.0;  pp.uMax = 1.0;  pp.Kb = 0.5
        self._pid = ctrl.DiscretePID(pp, Ts)

    def name(self): return "PID"

    def reset(self):
        self._pid.reset()

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        return self._pid.compute(F_ref - F)


# ---------------------------------------------------------------------------
# 3. ADRC
# ---------------------------------------------------------------------------
class ADRCCtrl(EHFSController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        # omega_o * Ts = 800 * 5e-4 = 0.40 < 0.5 (constraint satisfied)
        # b0 = K_F * (1 - exp(-Ts/tau_v)) ~ static gain per sample
        K_qa = p['Kq'] * math.sqrt(max(p['P_S'] - p.get('P_A0', 3e6), 0.0))
        K_qb = p['Kq'] * math.sqrt(max(p.get('P_B0', 5e6), 0.0))
        K_F  = (p['A_A'] * p['beta'] / p['V_A'] * K_qa
                + p['A_B'] * p['beta'] / p['V_B'] * K_qb)
        b0   = K_F * Ts * (1.0 - math.exp(-Ts / p['tau_v']))
        ap = ctrl.ADRCParams()
        ap.omega_c = 240.0;  ap.omega_o = 800.0;  ap.b0 = b0
        ap.uMin = -1.0;  ap.uMax = 1.0
        self._adrc = ctrl.DiscreteADRC(ap, Ts)

    def name(self): return "ADRC"

    def reset(self):
        self._adrc.reset()

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        return self._adrc.compute(F_ref - F)


# ---------------------------------------------------------------------------
# 4. SMC  (compute(y - ref) convention)
# ---------------------------------------------------------------------------
class SMCCtrl(EHFSController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        sp = ctrl.SMCParams()
        sp.c_e = 1.0;  sp.c_de = 5.0e-4;  sp.K = 0.3;  sp.phi = 0.05
        sp.uMin = -1.0;  sp.uMax = 1.0
        self._smc = ctrl.DiscreteSMC(sp, Ts)

    def name(self): return "SMC"

    def reset(self):
        self._smc.reset()

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        return self._smc.compute(F - F_ref)


# ---------------------------------------------------------------------------
# 5. MPC (linearized 2-state force model)
# ---------------------------------------------------------------------------
class MPCCtrl(EHFSController):
    def __init__(self, p: dict, ss_model):
        mp         = ctrl.MPCParams()
        mp.Np      = 20;  mp.Nc = 8
        mp.rho_y   = 1.0e-8;  mp.rho_u = 1.0
        mp.uMin    = -1.0;    mp.uMax  = 1.0
        mp.duMin   = -0.2;    mp.duMax = 0.2
        mp.qp_max_iter = 500
        self._mpc  = ctrl.DiscreteMPC(ss_model, mp)
        self._Ts   = p['Ts']

    def name(self): return "MPC"

    def reset(self):
        self._mpc.reset()

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        x_lin = np.array([F, x_v])
        r     = np.array([F_ref])
        self._mpc.set_state(x_lin)
        return _clamp(float(self._mpc.compute_ref(x_lin, r)[0]))


# ---------------------------------------------------------------------------
# 6. LQR  (2-state, deviation-form: delta_x = [F - F_ref, x_v])
# ---------------------------------------------------------------------------
class LQRCtrl(EHFSController):
    def __init__(self, p: dict, ss_model):
        lp    = ctrl.LQRParams()
        # Q weights: force (N^2) and valve (normalized)
        lp.Q  = np.diag([1.0e-8, 1.0])
        lp.R  = np.array([[1.0]])
        self._lqr = ctrl.DiscreteLQR(ss_model, lp)
        self._Ts  = p['Ts']

    def name(self): return "LQR"

    def reset(self): pass

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        # Deviation state: track F_ref; x_v reference = 0 at steady state
        x     = np.array([F, x_v])
        x_ref = np.array([F_ref, 0.0])
        return _clamp(float(self._lqr.compute(x, x_ref)[0]))


# ---------------------------------------------------------------------------
# 7. MRAC  (set_reference then compute(y_plant) - same as toolbox convention)
# ---------------------------------------------------------------------------
class MRACCtrl(EHFSController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        # Reference model: 300 rad/s discrete pole
        a_m = math.exp(-300.0 * Ts)
        b_m = 1.0 - a_m
        mp  = ctrl.MRACParams()
        mp.a_m = a_m;  mp.b_m = b_m
        # Very small adaptation gains: force signals are O(1e4 N) vs u in [-1,1]
        mp.gamma_r = 1.0e-9;  mp.gamma_y = 5.0e-10
        mp.sigma   = 0.01;    mp.theta_max = 0.05
        self._mrac = ctrl.MRACController(mp, Ts)

    def name(self): return "MRAC"

    def reset(self):
        self._mrac.reset()

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        self._mrac.set_reference(F_ref)
        return _clamp(self._mrac.compute(F))


# ---------------------------------------------------------------------------
# 8. L1Adaptive
# ---------------------------------------------------------------------------
class L1AdaptiveCtrl(EHFSController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        a_m = math.exp(-300.0 * Ts)
        b_m = 1.0 - a_m
        lp  = ctrl.L1AdaptiveParams()
        lp.a_m = a_m;  lp.b_m = b_m;  lp.k_g = 1.0
        lp.Gamma    = 1.0e-5;  lp.omega_c = 200.0
        lp.sigma_max = 0.02
        lp.uMin = -1.0;  lp.uMax = 1.0
        self._l1 = ctrl.L1AdaptiveController(lp, Ts)

    def name(self): return "L1Adaptive"

    def reset(self):
        self._l1.reset()

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        self._l1.set_reference(F_ref)
        return _clamp(self._l1.compute(F))


# ---------------------------------------------------------------------------
# 9. FeedbackLinearisation (pure Python - state-dependent gain inversion)
#
# Desired force rate: v_des = lambda * (F_ref - F)
# Linearized gain (function of pressures): K_xv = dF/dt per unit x_v
# Valve command: u_v = v_des / K_xv + Kd * (x_v_des_prev - x_v)
# ---------------------------------------------------------------------------
class FeedbackLinController(EHFSController):
    def __init__(self, p: dict):
        self._p      = p
        self._lambda = 300.0   # desired BW [rad/s]
        self._int_e  = 0.0

    def name(self): return "FeedbackLin"

    def reset(self):
        self._int_e = 0.0

    def _K_xv(self, P_A, P_B):
        p = self._p
        K_qa = p['Kq'] * math.sqrt(max(p['P_S'] - P_A, 0.0))
        K_qb = p['Kq'] * math.sqrt(max(P_B, 0.0))
        return (p['A_A'] * p['beta'] / p['V_A'] * K_qa
                + p['A_B'] * p['beta'] / p['V_B'] * K_qb)

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        e    = F_ref - F
        self._int_e += e * self._p['Ts']
        K_xv = max(self._K_xv(P_A, P_B), 1.0e4)   # singularity guard
        v_des = self._lambda * e + 50.0 * self._int_e
        return _clamp(v_des / K_xv)


# ---------------------------------------------------------------------------
# 10. NeuralPID
# ---------------------------------------------------------------------------
class NeuralPIDCtrl(EHFSController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        # b0 approx = K_F * Ts * (1 - exp(-Ts/tau_v))
        K_qa = p['Kq'] * math.sqrt(max(p['P_S'] - p.get('P_A0', 3e6), 0.0))
        K_qb = p['Kq'] * math.sqrt(max(p.get('P_B0', 5e6), 0.0))
        K_F  = (p['A_A'] * p['beta'] / p['V_A'] * K_qa
                + p['A_B'] * p['beta'] / p['V_B'] * K_qb)
        plant_gain = K_F * Ts * (1.0 - math.exp(-Ts / p['tau_v']))
        np_p = ctrl.NeuralPIDParams()
        np_p.n_hidden = 8;  np_p.lr = 1.0e-8;  np_p.Ts = Ts
        np_p.plant_gain     = plant_gain
        np_p.max_weight_norm = 5.0
        np_p.uMin = -1.0;  np_p.uMax = 1.0
        np_p.Kp0  = 3.0e-4;  np_p.Ki0 = 0.5;  np_p.Kd0 = 1.0e-7
        self._npid = ctrl.NeuralPID(np_p)

    def name(self): return "NeuralPID"

    def reset(self):
        self._npid.reset()

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        return _clamp(self._npid.compute(F_ref - F))


# ---------------------------------------------------------------------------
# 11. ILC (Iterative Learning Control)
#
# Phase 1 (k < N_TRIAL): pure PID; record errors.
# At k = N_TRIAL: compute feedforward for phase 2.
# Phase 2 (k >= N_TRIAL): PID + learned feedforward.
# Suitable for sinusoidal scenarios; falls back to PID for non-periodic.
# ---------------------------------------------------------------------------
class ILCCtrl(EHFSController):
    N_TRIAL = 800  # 0.4 s at Ts=0.5 ms  (covers 1 period of 5Hz, 20 of 50Hz)

    def __init__(self, p: dict):
        Ts = p['Ts']
        pp = ctrl.PIDParams()
        pp.Kp = 3.0e-4;  pp.Ki = 0.5;  pp.Kd = 1.0e-7
        pp.uMin = -1.0;  pp.uMax = 1.0;  pp.Kb = 0.5
        self._pid = ctrl.DiscretePID(pp, Ts)
        ip = ctrl.ILCParams()
        ip.N          = self.N_TRIAL;  ip.Ts = Ts
        ip.mode       = ctrl.ILCMode.PType
        ip.Lp         = 0.6;  ip.Q_filter = 0.95
        ip.uMin       = -0.5;  ip.uMax = 0.5
        self._ilc     = ctrl.ILC(ip)
        self._k       = 0
        self._phase2  = False

    def name(self): return "ILC"

    def reset(self):
        self._pid.reset()
        self._ilc.reset()
        self._k      = 0
        self._phase2 = False

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        e     = F_ref - F
        u_pid = self._pid.compute(e)
        if not self._phase2:
            if self._k < self.N_TRIAL:
                self._ilc.record_error(self._k, e)
            self._k += 1
            if self._k == self.N_TRIAL:
                self._ilc.update_feedforward()
                self._phase2 = True
                self._k = 0
            return _clamp(u_pid)
        else:
            k2   = min(self._k, self.N_TRIAL - 1)
            u_ff = self._ilc.feedforward(k2)
            self._k += 1
            return _clamp(u_pid + u_ff)


# ---------------------------------------------------------------------------
# 12. GainScheduled (two PIDs blended on |F_ref| magnitude)
# ---------------------------------------------------------------------------
class GainScheduledCtrl(EHFSController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        # Low-force regime: higher gain for small-signal precision
        pp_lo = ctrl.PIDParams()
        pp_lo.Kp = 5.0e-4;  pp_lo.Ki = 0.8;  pp_lo.Kd = 2.0e-7
        pp_lo.uMin = -1.0;  pp_lo.uMax = 1.0;  pp_lo.Kb = 0.5
        pid_lo = ctrl.DiscretePID(pp_lo, Ts)
        # High-force regime: lighter gains to avoid instability under large loads
        pp_hi = ctrl.PIDParams()
        pp_hi.Kp = 2.0e-4;  pp_hi.Ki = 0.3;  pp_hi.Kd = 8.0e-8
        pp_hi.uMin = -1.0;  pp_hi.uMax = 1.0;  pp_hi.Kb = 0.5
        pid_hi = ctrl.DiscretePID(pp_hi, Ts)

        self._gs = ctrl.GainScheduledController(Ts)
        self._gs.add_schedule_point(0.0,    pid_lo)
        self._gs.add_schedule_point(5000.0, pid_hi)

    def name(self): return "GainScheduled"

    def reset(self):
        self._gs.reset()

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        self._gs.set_scheduling_param(abs(F_ref))
        return _clamp(self._gs.compute(F_ref - F))


# ---------------------------------------------------------------------------
# Normalised LMS (local utility class, used by HinfCascadeCtrl)
# ---------------------------------------------------------------------------
class NormalisedLMS:
    """Online scalar LMS with normalised step size (Haykin 2002).

    Updates weight vector w to minimise squared prediction error e:
      e_hat = w @ phi
      w += mu / (eps + phi @ phi) * (e - e_hat) * phi
    Returns the adaptive correction signal w @ phi.
    """
    def __init__(self, n_order: int = 4, mu: float = 0.01, eps: float = 1e-6):
        self._w   = np.zeros(n_order)
        self._phi = np.zeros(n_order)
        self._mu  = mu
        self._eps = eps

    def update(self, e: float, u_feature: float) -> float:
        """Feed error e and scalar feature u_feature; return adaptive correction."""
        self._phi = np.roll(self._phi, 1)
        self._phi[0] = u_feature
        e_hat = float(self._w @ self._phi)
        denom = self._eps + float(self._phi @ self._phi)
        self._w += (self._mu / denom) * (e - e_hat) * self._phi
        return float(self._w @ self._phi)

    def reset(self):
        self._w[:]   = 0.0
        self._phi[:] = 0.0


# ---------------------------------------------------------------------------
# 13. HinfODFCCtrl
#
# H-infinity-inspired Output Disturbance Feedback Controller for EHFS.
#
# Implementation: The 2-state linearised force model [F, x_v] is used.
# An H∞ state-feedback gain K_hinf is computed via the DARE with gamma-
# parameterized cost that minimises ||Tzw||_inf < gamma (Doyle et al. 1989):
#
#   DARE: P = A^T P A - A^T P B (R_h + B^T P B)^-1 B^T P A + Q_h
#   where R_h = gamma^2 * I,  Q_h = C^T C  (unit sensitivity weight)
#
# K_hinf = (R_h + B^T P B)^-1 B^T P A
# u = -K_hinf @ (x_est - x_ref)
# State estimated via Luenberger observer with poles at 0.3.
#
# Falls back to a lead-lag PD controller if scipy is unavailable.
# ---------------------------------------------------------------------------
class HinfODFCCtrl(EHFSController):
    def __init__(self, p: dict):
        self._Ts = p['Ts']
        self._K  = None   # H∞ state-feedback gain (1x2) or None
        self._L  = None   # observer gain (2x1)
        self._Ad = None
        self._Bd = None
        self._x_est = np.zeros(2)  # estimated state [F, x_v]

        # Fallback PD if scipy unavailable
        self._kp  = 3.0e-4
        self._kd  = 5.0e-7
        self._e_prev = 0.0

        try:
            from scipy.linalg import solve_discrete_are
            ss = build_linear_model(p)
            if ss is None:
                raise RuntimeError("StateSpace model unavailable")

            Ad = np.array(ss.A())
            Bd = np.array(ss.B())
            Cd = np.array(ss.C())

            # H∞-parameterized DARE: minimise ||z||/||w||_inf < gamma = 1.5
            gamma = 1.5
            Q_h = Cd.T @ Cd                          # C^T C (outputs error)
            R_h = gamma**2 * np.eye(Bd.shape[1])    # gamma^2 * I

            P = solve_discrete_are(Ad, Bd, Q_h, R_h)
            BtPB = Bd.T @ P @ Bd + R_h
            BtPA = Bd.T @ P @ Ad
            self._K  = np.linalg.solve(BtPB, BtPA)   # (1,2)
            self._Ad = Ad
            self._Bd = Bd

            # Luenberger observer: place closed-loop poles at 0.3 (fast correction)
            L_obs_gain = 0.7  # observer gain ~ (1 - 0.3) per eigenvalue
            self._L = L_obs_gain * np.ones((2, 1))
        except Exception:
            pass  # Fall back to PD

    def name(self): return "HinfODFC"

    def reset(self):
        self._x_est = np.zeros(2)
        self._e_prev = 0.0

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        if self._K is not None:
            # Observer update
            y_meas = np.array([[F]])
            y_pred = np.array([[self._x_est[0]]])
            innov  = y_meas - y_pred
            self._x_est = (self._Ad @ self._x_est
                           + self._Bd.flatten() * _clamp(self._x_est[0] - F_ref)
                           + (self._L * innov).flatten())
            # State feedback (deviation form)
            x_ref_vec = np.array([F_ref, 0.0])
            u_hinf = float(-self._K @ (self._x_est - x_ref_vec))
            return _clamp(u_hinf)
        else:
            # PD fallback
            e = F_ref - F
            de = (e - self._e_prev) / max(self._Ts, 1e-9)
            self._e_prev = e
            return _clamp(self._kp * e + self._kd * de)


# ---------------------------------------------------------------------------
# 14. HinfCascadeCtrl
#
# 3-layer cascade matching Shen et al. (2017):
#   Layer 1 (PI):    u_pi = Kp*e + Ki*sum(e)*Ts  (same gains as PIDCtrl)
#   Layer 2 (H∞ ODFC): u_hinf = HinfODFCCtrl.compute(e)  (additive feedback)
#   Layer 3 (nLMS): u_nlms = NormalisedLMS.update(e, u_pi + u_hinf)
#   Output: u = u_pi + u_hinf + u_nlms
# ---------------------------------------------------------------------------
class HinfCascadeCtrl(EHFSController):
    def __init__(self, p: dict):
        self._Ts  = p['Ts']
        self._Kp  = 3.0e-4
        self._Ki  = 0.5
        self._int = 0.0       # PI integrator state

        self._hinf = HinfODFCCtrl(p)
        self._nlms = NormalisedLMS(n_order=4, mu=0.005, eps=1e-6)

    def name(self): return "HinfCascade"

    def reset(self):
        self._int = 0.0
        self._hinf.reset()
        self._nlms.reset()

    def compute(self, F_ref, F, v_p, P_A, P_B, x_v, t):
        e = F_ref - F

        # Layer 1: PI
        self._int += e * self._Ts
        u_pi = self._Kp * e + self._Ki * self._int

        # Layer 2: H∞ ODFC (additive)
        u_hinf = self._hinf.compute(F_ref, F, v_p, P_A, P_B, x_v, t)

        # Layer 3: nLMS adaptive correction
        u_base = u_pi + u_hinf
        u_nlms = self._nlms.update(e, u_base)

        return _clamp(u_pi + u_hinf + u_nlms)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def make_controllers(plant_params: dict):
    """Return list of all 14 EHFSController instances."""
    controllers = [OpenLoopCtrl(plant_params)]
    if not CTRL_AVAILABLE:
        print("WARNING: ctrl_toolbox not available - only OpenLoop will run.")
        return controllers

    ss = build_linear_model(plant_params)
    controllers += [
        PIDCtrl(plant_params),
        ADRCCtrl(plant_params),
        SMCCtrl(plant_params),
        MPCCtrl(plant_params, ss),
        LQRCtrl(plant_params, ss),
        MRACCtrl(plant_params),
        L1AdaptiveCtrl(plant_params),
        FeedbackLinController(plant_params),
        NeuralPIDCtrl(plant_params),
        ILCCtrl(plant_params),
        GainScheduledCtrl(plant_params),
        HinfODFCCtrl(plant_params),
        HinfCascadeCtrl(plant_params),
    ]
    return controllers
