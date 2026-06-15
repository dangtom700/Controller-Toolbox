"""
12 controllers for the Nonlinear Surface Ship Manoeuvring case study.
All controllers implement: name(), reset(), compute(x_ref, plant_state, t) -> [n_rps, delta_rad]

x_ref       = (xd, yd, xd_dot, yd_dot, xd_ddot, yd_ddot)
plant_state = [u, v, r, psi, x, y]
"""

import os
import sys
import math

import numpy as np

_THIS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(_THIS)))
sys.path.insert(0, _ROOT)
import _setup_bindings  # noqa: F401

try:
    import ctrl_toolbox as ctrl
    CTRL_AVAILABLE = True
except ImportError:
    CTRL_AVAILABLE = False

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _wrap(angle):
    while angle >  math.pi: angle -= 2.0*math.pi
    while angle < -math.pi: angle += 2.0*math.pi
    return angle

def _heading_from_ref(xd_dot, yd_dot):
    if abs(xd_dot) < 1e-9 and abs(yd_dot) < 1e-9:
        return 0.0
    return math.atan2(xd_dot, yd_dot)

def _speed_from_ref(xd_dot, yd_dot):
    return math.sqrt(xd_dot**2 + yd_dot**2)

def _cross_track_err(xe, ye, psi_d):
    return xe * math.sin(psi_d) - ye * math.cos(psi_d)

def _n_ss(plant_params):
    p = plant_params["identified_params"]
    u0 = plant_params["initial_state"]["u0"]
    return math.sqrt(-p["a1"] * u0**2 / p["a5"])


class _SpeedPI:
    def __init__(self, Ts, n_ss, n_min=0.0, n_max=10.7, Kp=2.0, Ki=0.5):
        self.Kp = Kp; self.Ki = Ki; self.Ts = Ts
        self.n_ss = n_ss; self.n_min = n_min; self.n_max = n_max
        self._integ = 0.0

    def reset(self): self._integ = 0.0

    def compute(self, u_ref, u_meas):
        e = u_ref - u_meas
        self._integ += e * self.Ts
        n = self.n_ss + self.Kp * e + self.Ki * self._integ
        return max(self.n_min, min(self.n_max, n))


def _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d):
    xe = x - xd; ye = y - yd
    cte = _cross_track_err(xe, ye, psi_d)
    return psi_d - 0.3 * math.atan(cte / 5.0)


# ---------------------------------------------------------------------------
# 1. OpenLoop
# ---------------------------------------------------------------------------
class OpenLoopCtrl:
    def __init__(self, plant_params):
        self._n_ss = _n_ss(plant_params)

    def name(self): return "OpenLoop"
    def reset(self): pass

    def compute(self, x_ref, plant_state, t):
        return self._n_ss, 0.0


# ---------------------------------------------------------------------------
# 2. PID
# ---------------------------------------------------------------------------
class PIDCtrl:
    def __init__(self, plant_params):
        Ts  = plant_params["integration"]["Ts"]
        lim = plant_params["actuator_limits"]
        self._ns = _n_ss(plant_params)
        self._Ts = Ts
        self._dmax = lim["delta_max_rad"]
        self._speed = _SpeedPI(Ts, self._ns, n_max=lim["n_max_rps"])

        self._ctrl = None
        if CTRL_AVAILABLE:
            try:
                pp = ctrl.PIDParams()
                pp.Kp = 2.0; pp.Ki = 0.4; pp.Kd = 0.6
                pp.uMin = -self._dmax; pp.uMax = self._dmax
                self._ctrl = ctrl.DiscretePID(pp, Ts)
            except Exception:
                pass

        self._Kp = 2.0; self._Ki = 0.4; self._Kd = 0.6
        self._integ = 0.0; self._prev_err = 0.0

    def name(self): return "PID"

    def reset(self):
        if self._ctrl:
            try: self._ctrl.reset()
            except Exception: pass
        self._integ = 0.0; self._prev_err = 0.0
        self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)
        psi_err = _wrap(psi - psi_ref)

        if self._ctrl:
            try:
                delta = self._ctrl.compute(-psi_err)
            except Exception:
                delta = 0.0
        else:
            self._integ += psi_err * self._Ts
            d_err = (psi_err - self._prev_err) / self._Ts
            self._prev_err = psi_err
            delta = -(self._Kp*psi_err + self._Ki*self._integ + self._Kd*d_err)
            delta = max(-self._dmax, min(self._dmax, delta))

        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# 3. SMC
# ---------------------------------------------------------------------------
class SMCCtrl:
    def __init__(self, plant_params):
        Ts  = plant_params["integration"]["Ts"]
        lim = plant_params["actuator_limits"]
        p   = plant_params["identified_params"]
        self._c2 = p["c2"]; self._c6 = p["c6"]
        self._Ts = Ts; self._dmax = lim["delta_max_rad"]
        self._Ks = 0.3; self._eta = 0.05; self._lam = 1.0
        self._integ = 0.0
        self._speed = _SpeedPI(Ts, _n_ss(plant_params), n_max=lim["n_max_rps"])

    def name(self): return "SMC"

    def reset(self):
        self._integ = 0.0
        self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)
        psi_err = _wrap(psi - psi_ref)
        self._integ += psi_err * self._Ts
        s = psi_err + self._lam * self._integ
        u_eq = -self._c2 * r / (self._c6 + 1e-12)
        u_sw = -self._Ks * math.tanh(s / self._eta) / (self._c6 + 1e-12)
        delta = -(u_eq + u_sw)
        delta = max(-self._dmax, min(self._dmax, delta))
        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# 4. ASMC (Paper's Adaptive Sliding Mode Control)
# ---------------------------------------------------------------------------
class ASMCCtrl:
    """Full cascade ASMC from Meng et al. (2025) Section 4.1."""
    def __init__(self, plant_params):
        p = plant_params["identified_params"]
        self.a1=p["a1"]; self.a2=p["a2"]; self.a3=p["a3"]; self.a4=p["a4"]
        self.a5=p["a5"]; self.a6=p["a6"]
        self.b1=p["b1"]; self.b2=p["b2"]; self.b3=p["b3"]; self.b4=p["b4"]
        self.b5=p["b5"]; self.b6=p["b6"]; self.b7=p["b7"]
        self.c1=p["c1"]; self.c2=p["c2"]; self.c3=p["c3"]
        self.c4=p["c4"]; self.c5=p["c5"]; self.c6=p["c6"]

        ap = plant_params["asmc_params"]
        self.rho=ap["rho"]; self.C=ap["C"]
        self.sv1=ap["sigma_v1"]; self.sv2=ap["sigma_v2"]
        self.su1=ap["sigma_u1"]; self.su2=ap["sigma_u2"]
        self.Kv=ap["Kv"]; self.Ku=ap["Ku"]
        self.eps_v=ap["eps_v"]; self.eps_u=ap["eps_u"]

        lim = plant_params["actuator_limits"]
        self.Ts=plant_params["integration"]["Ts"]
        self.n_max=lim["n_max_rps"]; self.n_min=lim["n_min_rps"]
        self.dmax=lim["delta_max_rad"]
        self.Omega_v = self.b7 / self.c6
        self.Omega_u = self.a6 / self.c6

        d = plant_params["disturbance"]
        self._d_u_amp=d["d_u_amp"]; self._d_u_freq=d["d_u_freq"]
        self._d_v_amp=d["d_v_amp"]; self._d_v_freq=d["d_v_freq"]

        self.reset()

    def name(self): return "ASMC"

    def reset(self):
        self._int_ve=0.0; self._int_ue=0.0
        self._av_prev=None; self._au_prev=None

    def _Fu(self, u, v, r):
        return self.a1*u*u + self.a2*v*r + self.a3*v*v + self.a4*r*r

    def _Fv(self, u, v, r):
        return (self.b1*v + self.b2*r + self.b3*abs(v)*v +
                self.b4*abs(r)*r + self.b5*abs(v)*r + self.b6*(-u*r))

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, xd_dd, yd_dd = x_ref
        u, v, r, psi, x, y = plant_state
        d_u = self._d_u_amp * math.sin(self._d_u_freq * t)
        d_v = self._d_v_amp * math.cos(self._d_v_freq * t)

        xe = x - xd; ye = y - yd
        w  = math.sqrt(xe*xe + ye*ye + self.C)
        vx = xd_d - self.rho*xe/w
        vy = yd_d - self.rho*ye/w
        alpha_u =  vx*math.cos(psi) + vy*math.sin(psi)
        alpha_v = -vx*math.sin(psi) + vy*math.cos(psi)

        Ts = self.Ts
        if self._av_prev is None:
            av_dot = 0.0; au_dot = 0.0
        else:
            av_dot = (alpha_v - self._av_prev) / Ts
            au_dot = (alpha_u - self._au_prev) / Ts
        self._av_prev = alpha_v; self._au_prev = alpha_u

        ve = v - alpha_v; ue = u - alpha_u
        self._int_ve += ve * Ts; self._int_ue += ue * Ts
        sv = self.sv1*ve + self.sv2*self._int_ve
        su = self.su1*ue + self.su2*self._int_ue

        Fv = self._Fv(u, v, r)
        tau_r  = (1.0/self.Omega_v)*(av_dot - Fv - d_v - (self.sv2/self.sv1)*ve)
        tau_r -= self.Kv * math.tanh(sv / self.eps_v)

        Fu = self._Fu(u, v, r)
        tau_u  = au_dot - Fu - self.Omega_u*tau_r - d_u - (self.su2/self.su1)*ue
        tau_u -= self.Ku * math.tanh(su / self.eps_u)

        delta = tau_r / (self.c6 + 1e-12)
        delta = max(-self.dmax, min(self.dmax, delta))
        n = min(self.n_max, math.sqrt(max(0.0, tau_u) / (self.a5 + 1e-12)))
        return n, delta


# ---------------------------------------------------------------------------
# 5. MPC
# ---------------------------------------------------------------------------
class MPCCtrl:
    def __init__(self, plant_params):
        Ts  = plant_params["integration"]["Ts"]
        p   = plant_params["identified_params"]
        lim = plant_params["actuator_limits"]
        self._Ts = Ts; self._dmax = lim["delta_max_rad"]
        self._speed = _SpeedPI(Ts, _n_ss(plant_params), n_max=lim["n_max_rps"])
        self._ctrl = None

        if CTRL_AVAILABLE:
            try:
                c2 = p["c2"]; c6 = p["c6"]
                from scipy.linalg import expm
                Ac = np.array([[0.0, 1.0], [0.0, c2]])
                Bc = np.array([[0.0], [c6]])
                Ad = expm(Ac * Ts)
                Bd = (np.eye(2) + Ac * Ts / 2.0) @ Bc * Ts
                Cd = np.array([[1.0, 0.0]])
                Dd = np.zeros((1, 1))
                ss = ctrl.StateSpace(Ad, Bd, Cd, Dd, Ts)
                mp = ctrl.MPCParams()
                mp.Np = 20; mp.Nc = 5
                mp.rho_y = 10.0; mp.rho_u = 0.1
                mp.uMin = -self._dmax; mp.uMax = self._dmax
                self._mpc = ctrl.DiscreteMPC(ss, mp)
                self._ctrl = self._mpc
            except Exception:
                self._ctrl = None

        self._Kp = 2.5; self._Ki = 0.3; self._Kd = 0.7
        self._integ = 0.0; self._prev_err = 0.0

    def name(self): return "MPC"

    def reset(self):
        if self._ctrl:
            try: self._ctrl.reset()
            except Exception: pass
        self._integ = 0.0; self._prev_err = 0.0
        self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)
        psi_err = _wrap(psi - psi_ref)

        if self._ctrl:
            try:
                x_mpc = np.array([psi - psi_ref, r])
                u_vec = self._ctrl.compute_ref(x_mpc, np.array([0.0]))
                delta = float(np.squeeze(u_vec))
            except Exception:
                delta = 0.0
        else:
            self._integ += psi_err * self._Ts
            d_err = (psi_err - self._prev_err) / self._Ts
            self._prev_err = psi_err
            delta = -(self._Kp*psi_err + self._Ki*self._integ + self._Kd*d_err)

        delta = max(-self._dmax, min(self._dmax, delta))
        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# 6. LQR
# ---------------------------------------------------------------------------
class LQRCtrl:
    def __init__(self, plant_params):
        Ts  = plant_params["integration"]["Ts"]
        p   = plant_params["identified_params"]
        lim = plant_params["actuator_limits"]
        self._Ts = Ts; self._dmax = lim["delta_max_rad"]
        self._speed = _SpeedPI(Ts, _n_ss(plant_params), n_max=lim["n_max_rps"])

        c2 = p["c2"]; c6 = p["c6"]
        try:
            from scipy.linalg import expm, solve_discrete_are
            Ac = np.array([[0.0, 1.0], [0.0, c2]])
            Bc = np.array([[0.0], [c6]])
            Ad = expm(Ac * Ts)
            Bd = (np.eye(2) + Ac * Ts / 2.0) @ Bc * Ts
            Q = np.diag([16.0, 4.0])
            R = np.array([[3.0]])
            P = solve_discrete_are(Ad, Bd, Q, R)
            self._K = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
        except Exception:
            self._K = np.array([[4.0, 0.8]])

    def name(self): return "LQR"
    def reset(self): self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)
        psi_err = _wrap(psi - psi_ref)
        z     = np.array([psi_err, r])
        delta = float(-(self._K @ z)[0])
        delta = max(-self._dmax, min(self._dmax, delta))
        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# 7. MRAC
# ---------------------------------------------------------------------------
class MRACCtrl:
    def __init__(self, plant_params):
        Ts  = plant_params["integration"]["Ts"]
        lim = plant_params["actuator_limits"]
        p   = plant_params["identified_params"]
        self._Ts = Ts; self._dmax = lim["delta_max_rad"]
        self._speed = _SpeedPI(Ts, _n_ss(plant_params), n_max=lim["n_max_rps"])
        self._ctrl = None

        if CTRL_AVAILABLE:
            try:
                mp = ctrl.MRACParams()
                mp.gamma_r = 0.5; mp.gamma_y = 0.5
                mp.a_m = -0.15; mp.b_m = 0.15
                mp.uMin = -lim["delta_max_rad"]; mp.uMax = lim["delta_max_rad"]
                self._ctrl = ctrl.MRACController(mp, Ts)
            except Exception:
                pass

        # MIT-rule fallback
        self._ym = 0.0; self._theta_r = 1.0; self._theta_y = 0.0
        self._gamma = 0.3; self._am = -0.15

    def name(self): return "MRAC"

    def reset(self):
        if self._ctrl:
            try: self._ctrl.reset()
            except Exception: pass
        self._ym = 0.0; self._theta_r = 1.0; self._theta_y = 0.0
        self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)

        if self._ctrl:
            try:
                self._ctrl.set_reference(psi_ref)
                delta = self._ctrl.compute(psi)
            except Exception:
                delta = 0.0
        else:
            Ts = self._Ts
            em = self._ym - psi
            self._theta_r += self._gamma * em * psi_ref * Ts
            self._theta_y += self._gamma * em * psi * Ts
            delta = self._theta_r * psi_ref - self._theta_y * psi
            self._ym += Ts * (self._am * self._ym + (-self._am) * psi_ref)

        delta = max(-self._dmax, min(self._dmax, delta))
        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# 8. L1 Adaptive
# ---------------------------------------------------------------------------
class L1AdaptiveCtrl:
    def __init__(self, plant_params):
        Ts  = plant_params["integration"]["Ts"]
        lim = plant_params["actuator_limits"]
        p   = plant_params["identified_params"]
        self._Ts = Ts; self._dmax = lim["delta_max_rad"]
        self._speed = _SpeedPI(Ts, _n_ss(plant_params), n_max=lim["n_max_rps"])
        self._ctrl = None

        if CTRL_AVAILABLE:
            try:
                lp = ctrl.L1AdaptiveParams()
                lp.a_m = -0.15; lp.b_m = 0.15
                lp.omega_c = 0.25; lp.Gamma = 10.0; lp.sigma_max = 5.0
                lp.uMin = -lim["delta_max_rad"]; lp.uMax = lim["delta_max_rad"]
                self._ctrl = ctrl.L1AdaptiveController(lp, Ts)
            except Exception:
                pass

        # Fallback
        self._sigma = 0.0; self._u_lp = 0.0; self._xm = 0.0
        self._am = -0.15; self._bm = 0.15; self._Gamma = 10.0; self._oc = 0.25

    def name(self): return "L1Adaptive"

    def reset(self):
        if self._ctrl:
            try: self._ctrl.reset()
            except Exception: pass
        self._sigma = 0.0; self._u_lp = 0.0; self._xm = 0.0
        self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)

        if self._ctrl:
            try:
                self._ctrl.set_reference(psi_ref)
                delta = self._ctrl.compute(psi)
            except Exception:
                delta = 0.0
        else:
            Ts = self._Ts
            err_pred = self._xm - psi
            self._sigma += self._Gamma * err_pred * Ts
            self._sigma = max(-5.0, min(5.0, self._sigma))
            alpha_lp = math.exp(-self._oc * Ts)
            u_raw = self._bm * psi_ref - self._sigma
            self._u_lp = alpha_lp * self._u_lp + (1 - alpha_lp) * u_raw
            delta = self._u_lp
            self._xm += Ts * (self._am * self._xm + self._bm * psi_ref)

        delta = max(-self._dmax, min(self._dmax, delta))
        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# 9. GainScheduled
# ---------------------------------------------------------------------------
class GainScheduledCtrl:
    """3-point gain-scheduled PID scheduled on |psi_err|."""
    def __init__(self, plant_params):
        Ts  = plant_params["integration"]["Ts"]
        lim = plant_params["actuator_limits"]
        self._Ts = Ts; self._dmax = lim["delta_max_rad"]
        self._speed = _SpeedPI(Ts, _n_ss(plant_params), n_max=lim["n_max_rps"])
        self._ctrl = None

        if CTRL_AVAILABLE:
            try:
                gsc = ctrl.GainScheduledController(Ts)
                for p_val, Kp, Ki, Kd in [(0.15, 3.0, 0.5, 1.0),
                                           (0.40, 2.5, 0.3, 0.8),
                                           (0.80, 1.8, 0.1, 0.5)]:
                    pp = ctrl.PIDParams()
                    pp.Kp = Kp; pp.Ki = Ki; pp.Kd = Kd
                    pp.uMin = -lim["delta_max_rad"]; pp.uMax = lim["delta_max_rad"]
                    gsc.add_schedule_point(p_val, ctrl.DiscretePID(pp, Ts))
                self._ctrl = gsc
            except Exception:
                pass

        # Fallback lookup table
        self._brackets = [(0.0, 0.15, 3.0, 0.5, 1.0),
                          (0.15, 0.40, 2.5, 0.3, 0.8),
                          (0.40, 1e9,  1.8, 0.1, 0.5)]
        self._integ = 0.0; self._prev_err = 0.0

    def name(self): return "GainScheduled"

    def reset(self):
        if self._ctrl:
            try: self._ctrl.reset()
            except Exception: pass
        self._integ = 0.0; self._prev_err = 0.0
        self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)
        psi_err = _wrap(psi - psi_ref)
        sched   = abs(psi_err)

        if self._ctrl:
            try:
                self._ctrl.set_scheduling_param(sched)
                delta = self._ctrl.compute(-psi_err)
            except Exception:
                delta = 0.0
        else:
            Kp = Ki = Kd = 2.5, 0.3, 0.8
            for lo, hi, Kp, Ki, Kd in self._brackets:
                if lo <= sched < hi:
                    break
            self._integ += psi_err * self._Ts
            d_err = (psi_err - self._prev_err) / self._Ts
            self._prev_err = psi_err
            delta = -(Kp*psi_err + Ki*self._integ + Kd*d_err)

        delta = max(-self._dmax, min(self._dmax, delta))
        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# 10. ADRC
# ---------------------------------------------------------------------------
class ADRCCtrl:
    """2nd-order ADRC on heading; omega_o=1.5, Ts=0.08 -> omega_o*Ts=0.12 < 0.5."""
    def __init__(self, plant_params):
        Ts  = plant_params["integration"]["Ts"]
        lim = plant_params["actuator_limits"]
        p   = plant_params["identified_params"]
        self._Ts = Ts; self._dmax = lim["delta_max_rad"]
        self._speed = _SpeedPI(Ts, _n_ss(plant_params), n_max=lim["n_max_rps"])
        self._b0 = p["c6"]   # control gain: delta -> yaw-rate
        self._ctrl = None

        if CTRL_AVAILABLE:
            try:
                ap = ctrl.ADRCParams()
                ap.b0 = p["c6"]
                ap.omega_o = 1.5   # omega_o * Ts = 0.12 < 0.5
                ap.omega_c = 0.3
                ap.uMin = -lim["delta_max_rad"]
                ap.uMax  =  lim["delta_max_rad"]
                self._ctrl = ctrl.DiscreteADRC(ap, Ts)
            except Exception:
                pass

        # Pure-Python 2nd-order ADRC ESO fallback
        self._z1 = 0.0; self._z2 = 0.0; self._z3 = 0.0
        self._oc = 0.3; self._oo = 1.5

    def name(self): return "ADRC"

    def reset(self):
        if self._ctrl:
            try: self._ctrl.reset()
            except Exception: pass
        self._z1 = 0.0; self._z2 = 0.0; self._z3 = 0.0
        self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)
        err = psi_ref - psi

        if self._ctrl:
            try:
                delta = self._ctrl.compute(err)
            except Exception:
                delta = 0.0
        else:
            Ts = self._Ts; oo = self._oo; oc = self._oc; b0 = self._b0
            beta1 = 2*oo; beta2 = oo**2; beta3 = oo**3/3.0
            e1 = self._z1 - psi
            self._z1 += Ts*(self._z2 - beta1*e1)
            self._z2 += Ts*(self._z3 - beta2*e1 + b0*self._z3)
            self._z3 += Ts*(-beta3*e1)
            u0 = oc**2*(psi_ref - self._z1) - 2*oc*self._z2
            delta = (u0 - self._z3) / (b0 + 1e-12)

        delta = max(-self._dmax, min(self._dmax, delta))
        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# 11. NeuralPID
# ---------------------------------------------------------------------------
class NeuralPIDCtrl:
    def __init__(self, plant_params):
        Ts  = plant_params["integration"]["Ts"]
        lim = plant_params["actuator_limits"]
        p   = plant_params["identified_params"]
        self._Ts = Ts; self._dmax = lim["delta_max_rad"]
        self._speed = _SpeedPI(Ts, _n_ss(plant_params), n_max=lim["n_max_rps"])
        plant_gain = p["c6"] * Ts  # psi response per delta per step
        self._ctrl = None

        if CTRL_AVAILABLE:
            try:
                np_p = ctrl.NeuralPIDParams()
                np_p.Kp0 = 2.0; np_p.Ki0 = 0.3; np_p.Kd0 = 0.6
                np_p.lr = 1e-4; np_p.plant_gain = plant_gain
                np_p.n_hidden = 8; np_p.Ts = Ts
                np_p.uMin = -lim["delta_max_rad"]; np_p.uMax = lim["delta_max_rad"]
                self._ctrl = ctrl.NeuralPID(np_p)
            except Exception:
                pass

        self._Kp = 2.0; self._Ki = 0.3; self._Kd = 0.6
        self._integ = 0.0; self._prev_err = 0.0
        self._lr = 1e-4; self._pg = plant_gain

    def name(self): return "NeuralPID"

    def reset(self):
        if self._ctrl:
            try: self._ctrl.reset()
            except Exception: pass
        self._integ = 0.0; self._prev_err = 0.0
        self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)
        psi_err = _wrap(psi - psi_ref)

        if self._ctrl:
            try:
                delta = self._ctrl.compute(-psi_err)
            except Exception:
                delta = 0.0
        else:
            Ts = self._Ts
            self._integ += psi_err * Ts
            d_err = (psi_err - self._prev_err) / Ts
            grad = self._pg * psi_err
            self._Kp = max(0.1, self._Kp - self._lr * grad * psi_err)
            self._Ki = max(0.0, self._Ki - self._lr * grad * self._integ)
            self._Kd = max(0.0, self._Kd - self._lr * grad * d_err)
            self._prev_err = psi_err
            delta = -(self._Kp*psi_err + self._Ki*self._integ + self._Kd*d_err)

        delta = max(-self._dmax, min(self._dmax, delta))
        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# 12. ILC
# ---------------------------------------------------------------------------
class ILCCtrl:
    """P-type ILC outer loop + PID inner loop for heading tracking."""
    def __init__(self, plant_params, N_steps=2500):
        Ts  = plant_params["integration"]["Ts"]
        lim = plant_params["actuator_limits"]
        p   = plant_params["identified_params"]
        self._Ts = Ts; self._N = N_steps; self._dmax = lim["delta_max_rad"]
        self._speed = _SpeedPI(Ts, _n_ss(plant_params), n_max=lim["n_max_rps"])
        self._Lp = 0.5
        self._ff = [0.0] * N_steps
        self._k  = 0
        self._Kp = 2.0; self._Ki = 0.4; self._Kd = 0.6
        self._integ = 0.0; self._prev_err = 0.0
        self._ctrl = None

        if CTRL_AVAILABLE:
            try:
                ip = ctrl.ILCParams()
                ip.mode = ctrl.ILCMode.PType
                ip.Lp = self._Lp; ip.N = N_steps; ip.Ts = Ts
                ip.uMin = -lim["delta_max_rad"]; ip.uMax = lim["delta_max_rad"]
                self._ctrl = ctrl.ILC(ip)
            except Exception:
                pass

    def name(self): return "ILC"

    def reset(self):
        if self._ctrl:
            try: self._ctrl.reset()
            except Exception: pass
        self._ff = [0.0] * self._N; self._k = 0
        self._integ = 0.0; self._prev_err = 0.0
        self._speed.reset()

    def compute(self, x_ref, plant_state, t):
        xd, yd, xd_d, yd_d, *_ = x_ref
        u, v, r, psi, x, y = plant_state
        psi_d   = _heading_from_ref(xd_d, yd_d)
        psi_ref = _psi_cmd(xd, yd, xd_d, yd_d, x, y, psi_d)
        psi_err = _wrap(psi - psi_ref)
        k = min(self._k, self._N - 1)

        if self._ctrl:
            try:
                delta = self._ctrl.compute(-psi_err)
            except Exception:
                delta = 0.0
        else:
            Ts = self._Ts
            self._integ += psi_err * Ts
            d_err = (psi_err - self._prev_err) / Ts
            self._prev_err = psi_err
            fb = -(self._Kp*psi_err + self._Ki*self._integ + self._Kd*d_err)
            ff = self._ff[k]
            self._ff[k] = ff + self._Lp * (-psi_err)
            delta = fb + ff

        self._k += 1
        delta = max(-self._dmax, min(self._dmax, delta))
        n = self._speed.compute(max(0.5, _speed_from_ref(xd_d, yd_d)), u)
        return n, delta


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def make_controllers(plant_params):
    return [
        OpenLoopCtrl(plant_params),
        PIDCtrl(plant_params),
        SMCCtrl(plant_params),
        ASMCCtrl(plant_params),
        MPCCtrl(plant_params),
        LQRCtrl(plant_params),
        MRACCtrl(plant_params),
        L1AdaptiveCtrl(plant_params),
        GainScheduledCtrl(plant_params),
        ADRCCtrl(plant_params),
        NeuralPIDCtrl(plant_params),
        ILCCtrl(plant_params),
    ]
