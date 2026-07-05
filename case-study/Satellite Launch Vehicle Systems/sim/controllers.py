"""
controllers.py
Twelve attitude controllers for the Satellite Launch Vehicle (SLV) pitch-plane
case study, after Nair, Selvaganesan & Lalithambika (2016).

All controllers share the interface:
    name()  -> str
    reset() -> None
    compute(ref, theta, theta_dot, t) -> delta        # gimbal command [rad]

ref, theta are attitude [rad]; theta_dot is pitch rate [rad/s]; t is time [s].
The plant is OPEN-LOOP UNSTABLE (pole at +sqrt(mu_alpha)) and TIME VARYING, so
every practical controller needs rate/derivative action to stabilise it.

Sign convention: the plant has theta_ddot = mu_alpha*theta + mu_c*delta with
mu_c > 0 (positive control gain), so error-driven controllers use the standard
attitude-tracking convention e = ref - theta with POSITIVE gains (increasing e
commands a positive gimbal that accelerates theta toward the reference). SMC uses
its native e = theta - ref surface convention (CONTRIBUTING.md#sign-conventions).

The paper's proposed method is a Lyapunov-based adaptive PID in an MRAC structure;
it is realised here as "AdaptivePID" - a fixed stabilising PD baseline augmented
by an MRAC adaptation term (the augmentation architecture of refs [10], [11]),
since ctrl_toolbox's MRACController is a first-order model-reference law that
cannot stabilise a 2nd-order unstable plant unaided.

ctrl_toolbox is imported with a graceful fallback.
"""

import math
import os
import sys

import numpy as np

_THIS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_THIS)))
sys.path.insert(0, _ROOT)
try:
    import _setup_bindings  # noqa: F401
    import ctrl_toolbox as ctrl
    _HAVE_CTRL = True
except Exception as _e:  # pragma: no cover
    print("WARNING: ctrl_toolbox not available (%s) - only OpenLoop will run." % _e)
    _HAVE_CTRL = False


# ---------------------------------------------------------------------------
# Design-point linear model (shared by LQR, MPC): worst-case transonic point.
# ---------------------------------------------------------------------------
def _design_state_space(p: dict):
    """Continuous [[0,1],[mu_a,0]], B=[[0],[mu_c]] at the transonic design point,
    ZOH-discretised to a ctrl.StateSpace (2 states, 1 in, 1 out=theta)."""
    import scipy.signal as sig
    mu_a = p.get("mu_alpha_design", 4.8)
    mu_c = p.get("mu_c_design", 7.0)
    Ts = p.get("Ts", 0.05)
    A = np.array([[0.0, 1.0], [mu_a, 0.0]])
    B = np.array([[0.0], [mu_c]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])
    Ad, Bd, Cd, Dd, _ = sig.cont2discrete((A, B, C, D), Ts, method="zoh")
    return ctrl.StateSpace(Ad, Bd, Cd, Dd, Ts)


def _clip(u, umax):
    return max(-umax, min(umax, u))


# ---------------------------------------------------------------------------
# Base
# ---------------------------------------------------------------------------
class SLVController:
    def name(self): return "Base"
    def reset(self): pass
    def compute(self, ref, theta, theta_dot, t): return 0.0


# ---------------------------------------------------------------------------
# 1. OpenLoop - no gimbal deflection (uncontrolled unstable baseline)
# ---------------------------------------------------------------------------
class OpenLoopCtrl(SLVController):
    def name(self): return "OpenLoop"
    def compute(self, ref, theta, theta_dot, t): return 0.0


# ---------------------------------------------------------------------------
# 2. PID - fixed-gain attitude autopilot, e = ref - theta
# ---------------------------------------------------------------------------
def _mk_pid(Kp, Ki, Kd, ur, Ts):
    pp = ctrl.PIDParams()
    pp.Kp = Kp; pp.Ki = Ki; pp.Kd = Kd
    pp.uMin = -ur; pp.uMax = ur; pp.Kb = 0.5
    return ctrl.DiscretePID(pp, Ts)


class PIDCtrl(SLVController):
    def __init__(self, p):
        self._pid = _mk_pid(3.0, 0.8, 0.9, p["delta_max"], p["Ts"])

    def name(self): return "PID"
    def reset(self): self._pid.reset()

    def compute(self, ref, theta, theta_dot, t):
        return self._pid.compute(ref - theta)


# ---------------------------------------------------------------------------
# 3. GainScheduledPD - PD gains scheduled on time (paper's baseline: gains
#    stored as functions of flight time, raised through the transonic band).
# ---------------------------------------------------------------------------
class GainScheduledPDCtrl(SLVController):
    def __init__(self, p):
        ur = p["delta_max"]; Ts = p["Ts"]
        self._gs = ctrl.GainScheduledController(Ts)
        self._gs.add_schedule_point(0.0,   _mk_pid(2.2, 0.0, 0.75, ur, Ts))
        self._gs.add_schedule_point(50.0,  _mk_pid(3.6, 0.0, 1.05, ur, Ts))
        self._gs.add_schedule_point(100.0, _mk_pid(2.6, 0.0, 0.85, ur, Ts))

    def name(self): return "GainScheduledPD"
    def reset(self):
        pass

    def compute(self, ref, theta, theta_dot, t):
        self._gs.set_scheduling_param(t)
        return self._gs.compute(ref - theta)


# ---------------------------------------------------------------------------
# 4. GainScheduledPID - as above with integral action.
# ---------------------------------------------------------------------------
class GainScheduledPIDCtrl(SLVController):
    def __init__(self, p):
        ur = p["delta_max"]; Ts = p["Ts"]
        self._gs = ctrl.GainScheduledController(Ts)
        self._gs.add_schedule_point(0.0,   _mk_pid(2.2, 0.6, 0.75, ur, Ts))
        self._gs.add_schedule_point(50.0,  _mk_pid(3.6, 1.0, 1.05, ur, Ts))
        self._gs.add_schedule_point(100.0, _mk_pid(2.6, 0.7, 0.85, ur, Ts))

    def name(self): return "GainScheduledPID"
    def reset(self):
        pass

    def compute(self, ref, theta, theta_dot, t):
        self._gs.set_scheduling_param(t)
        return self._gs.compute(ref - theta)


# ---------------------------------------------------------------------------
# 5. AdaptivePID (PROPOSED) - Lyapunov MRAC augmentation of a stabilising PD.
# ---------------------------------------------------------------------------
class AdaptivePIDCtrl(SLVController):
    def __init__(self, p):
        ur = p["delta_max"]; Ts = p["Ts"]
        # Stabilising PID baseline; the Lyapunov MRAC term is a gentle trim that
        # sharpens tracking through the transonic band without destabilising the
        # nominal loop (so nominal performance stays close to the tuned PID).
        self._pd = _mk_pid(3.0, 0.8, 0.9, ur, Ts)
        mp = ctrl.MRACParams()
        mp.a_m = 0.99; mp.b_m = 0.01            # 1st-order reference model
        mp.gamma_r = 5.0e-4; mp.gamma_y = 5.0e-4  # Lyapunov adaptation gains (>0, positive-gain plant)
        mp.sigma = 0.05; mp.theta_max = 1.0
        mp.uMin = -0.3 * ur; mp.uMax = 0.3 * ur   # augmentation authority
        self._mrac = ctrl.MRACController(mp, Ts)

    def name(self): return "AdaptivePID"
    def reset(self):
        self._pd.reset(); self._mrac.reset()

    def compute(self, ref, theta, theta_dot, t):
        u_pd = self._pd.compute(ref - theta)
        self._mrac.set_reference(ref)
        u_ad = self._mrac.compute(theta)
        return u_pd + u_ad


# ---------------------------------------------------------------------------
# 6. L1Adaptive - fast adaptive augmentation of the same PD baseline
#    (ref [5] uses an L1 architecture for a flexible launch vehicle).
# ---------------------------------------------------------------------------
class L1AdaptiveCtrl(SLVController):
    def __init__(self, p):
        ur = p["delta_max"]; Ts = p["Ts"]
        self._pd = _mk_pid(3.0, 0.8, 0.9, ur, Ts)
        lp = ctrl.L1AdaptiveParams()
        lp.a_m = 0.99; lp.b_m = 0.01; lp.k_g = 1.0
        lp.Gamma = 2.0e3; lp.omega_c = 3.0; lp.sigma_max = 0.5
        lp.uMin = -0.3 * ur; lp.uMax = 0.3 * ur
        self._l1 = ctrl.L1AdaptiveController(lp, Ts)

    def name(self): return "L1Adaptive"
    def reset(self):
        self._pd.reset(); self._l1.reset()

    def compute(self, ref, theta, theta_dot, t):
        u_pd = self._pd.compute(ref - theta)
        self._l1.set_reference(ref)
        u_ad = self._l1.compute(theta)
        return u_pd + u_ad


# ---------------------------------------------------------------------------
# 7. LQR - full-state feedback on the transonic design model, u = -K(x - x_ref)
# ---------------------------------------------------------------------------
class LQRCtrl(SLVController):
    def __init__(self, p, ss):
        lp = ctrl.LQRParams()
        lp.Q = np.diag([80.0, 1.0])
        lp.R = np.array([[1.0]])
        self._lqr = ctrl.DiscreteLQR(ss, lp)
        self._ur = p["delta_max"]

    def name(self): return "LQR"

    def compute(self, ref, theta, theta_dot, t):
        x = np.array([theta, theta_dot])
        x_ref = np.array([ref, 0.0])
        u = self._lqr.compute(x, x_ref, np.zeros(1))
        return _clip(float(u[0]), self._ur)


# ---------------------------------------------------------------------------
# 8. LeadLag - classical lead compensator (phase lead to stabilise the RHP pole)
# ---------------------------------------------------------------------------
class LeadLagCtrl(SLVController):
    def __init__(self, p):
        lp = ctrl.LeadLagParams()
        lp.continuous_zero = 2.5      # lead zero [rad/s]
        lp.continuous_pole = 25.0     # lead pole [rad/s]
        lp.gain = 16.0
        self._ll = ctrl.DiscreteLeadLag(lp, p["Ts"])
        self._ur = p["delta_max"]

    def name(self): return "LeadLag"
    def reset(self): self._ll.reset()

    def compute(self, ref, theta, theta_dot, t):
        return _clip(self._ll.compute(ref - theta), self._ur)


# ---------------------------------------------------------------------------
# 9. LQG (H2 output-feedback) - Kalman state estimator + LQR feedback. Unlike the
#    full-state LQR (#7) this reconstructs [theta, theta_dot] from the attitude
#    measurement alone, i.e. an OUTPUT-feedback optimal controller - the H2/LQG
#    candidate the selection matrix lists for this unstable MIMO-ish plant row.
#    (The first-/second-order SMC family sets up a divergent discrete bang-bang
#    limit cycle on this open-loop-unstable plant, so it is not part of the roster.)
# ---------------------------------------------------------------------------
class LQGCtrl(SLVController):
    def __init__(self, p, ss):
        lp = ctrl.LQRParams()
        lp.Q = np.diag([80.0, 1.0])
        lp.R = np.array([[1.0]])
        Q_noise = np.diag([1.0e-4, 1.0e-2])   # process-noise covariance
        R_noise = np.array([[1.0e-4]])        # attitude-sensor noise covariance
        self._lqg = ctrl.DiscreteLQG(ss, lp, Q_noise, R_noise)
        self._ur = p["delta_max"]
        self._u_prev = 0.0

    def name(self): return "LQG"
    def reset(self):
        self._lqg.reset(); self._u_prev = 0.0

    def compute(self, ref, theta, theta_dot, t):
        u = self._lqg.step(np.array([theta]), np.array([self._u_prev]),
                           np.array([ref, 0.0]))
        self._u_prev = _clip(float(u[0]), self._ur)
        return self._u_prev


# ---------------------------------------------------------------------------
# 10. ADRC - active disturbance rejection (ESO lumps mu_alpha*theta as the
#     "total disturbance" and cancels it, b0 = mu_c design value).
# ---------------------------------------------------------------------------
class ADRCCtrl(SLVController):
    def __init__(self, p):
        ap = ctrl.ADRCParams()
        # omega_o * Ts must stay below ~0.5 for discrete-ESO stability
        # (Ts=0.05 -> omega_o < 10); b0 = design control effectiveness mu_c.
        ap.omega_c = 3.0; ap.omega_o = 8.0; ap.b0 = p.get("mu_c_design", 8.0)
        ap.uMin = -p["delta_max"]; ap.uMax = p["delta_max"]
        self._adrc = ctrl.DiscreteADRC(ap, p["Ts"])

    def name(self): return "ADRC"
    def reset(self): self._adrc.reset()

    def compute(self, ref, theta, theta_dot, t):
        return self._adrc.compute(ref - theta)


# ---------------------------------------------------------------------------
# 11. MPC - constrained receding-horizon control on the design model
#     (hard gimbal limit |delta| <= 8 deg, paper Sec. 4).
# ---------------------------------------------------------------------------
class MPCCtrl(SLVController):
    def __init__(self, p, ss):
        ur = p["delta_max"]; Ts = p["Ts"]
        mp = ctrl.MPCParams()
        mp.Np = 20; mp.Nc = 5
        mp.rho_y = 1.0; mp.rho_u = 1.0e-2
        mp.uMin = -ur; mp.uMax = ur
        mp.duMin = -ur; mp.duMax = ur
        mp.qp_max_iter = 300
        self._mpc = ctrl.DiscreteMPC(ss, mp)

    def name(self): return "MPC"
    def reset(self): self._mpc.reset()

    def compute(self, ref, theta, theta_dot, t):
        x = np.array([theta, theta_dot])
        self._mpc.set_state(x)
        u = self._mpc.compute_ref(x, np.array([ref]))
        return float(u[0])


# ---------------------------------------------------------------------------
# 12. SelfTuningRegulator - online RLS identification + pole placement
#     (directly targets the paper's time-varying-plant theme).
# ---------------------------------------------------------------------------
class STRCtrl(SLVController):
    def __init__(self, p):
        sp = ctrl.STRParams()
        sp.na = 2; sp.nb = 2
        sp.mode = ctrl.STRMode.PolePlacement
        sp.desired_poles = [0.90, 0.90, 0.85]   # size na + nb - 1 = 3
        sp.lambda_ = 0.98
        sp.bMin = 1.0e-4
        sp.uMin = -p["delta_max"]; sp.uMax = p["delta_max"]
        sp.probeAmplitude = 2.0e-3
        self._str = ctrl.SelfTuningRegulator(sp, p["Ts"])

    def name(self): return "SelfTuningRegulator"
    def reset(self): self._str.reset()

    def compute(self, ref, theta, theta_dot, t):
        self._str.set_reference(ref)
        return self._str.compute(theta)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def make_controllers(p: dict):
    """Return the 12-controller SLV roster."""
    roster = [OpenLoopCtrl()]
    if not _HAVE_CTRL:
        print("WARNING: ctrl_toolbox not available - only OpenLoop will run.")
        return roster

    ss = _design_state_space(p)
    roster += [
        PIDCtrl(p),
        GainScheduledPDCtrl(p),
        GainScheduledPIDCtrl(p),
        AdaptivePIDCtrl(p),
        L1AdaptiveCtrl(p),
        LQRCtrl(p, ss),
        LeadLagCtrl(p),
        LQGCtrl(p, ss),
        ADRCCtrl(p),
        MPCCtrl(p, ss),
        STRCtrl(p),
    ]
    return roster
