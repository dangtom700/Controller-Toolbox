"""
controllers.py
Twelve controllers for the Aircraft Engine Thermal Management (Intermediate
Circulation Loop) case study.

All controllers share the interface:
  name()    -> str
  reset()   -> None
  compute(refs, plant_state, t, outs) -> (u1, u2)   # u1=m1_dot, u2=m2_dot [kg/s^2]

refs        = (Thout1_ref, Thout2_ref) [K]
plant_state = (TT, m1, m2)
outs        = (Thout1, Thout2, m0, Tmix, TTi)   - precomputed plant.outputs()

Each output is driven by its own independent branch flow: Thout1 by m1
(AHE1), Thout2 by m2 (AHE2) - see plant module docstring, simplification #5,
for why (m1, m2) are independent states rather than the paper's (m0, m1).

Sign convention (see README "Implementation Notes"): increasing flow rate
(m1 or m2) always REDUCES the corresponding air outlet temperature (more
cold water capacity -> more heat pulled from the air) - this is a
negative-static-gain process, the same situation documented for the Solar
Cooker case study (CLAUDE.md "Solar Cooker sign convention"). Following
that precedent:
  - PID / ADRC / SMC : fed e = y - ref (positive when too hot), POSITIVE
    gains - increasing e increases the commanded flow-rate-of-change.
  - MRAC / L1Adaptive : standard set_reference(r) + compute(y) convention,
    NEGATIVE adaptation gains (gamma_r, gamma_y < 0), exactly mirroring the
    "Solar Cooker MRAC gammas must be negative" rule.
  - NeuralPID : standard compute(r - y) convention, NEGATIVE plant_gain
    (mirrors Solar Cooker's NeuralPID plant_gain = -0.002*Ts).

ctrl_toolbox is imported with a graceful fallback.
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

from aircraft_engine_thermal_management_plant import AETMPlant


# ---------------------------------------------------------------------------
# Trim point + linearization (shared by LQR, MPC, SmithPredictor)
# ---------------------------------------------------------------------------
def find_trim(p: dict) -> dict:
    """Run the plant open-loop at constant nominal commands (u=0, i.e. fixed
    flow rates) under condition-1 boundary values until TT settles."""
    plant = AETMPlant(p)
    plant.reset([p.get("TT0", 380.0), p.get("m1_0", 0.25), p.get("m2_0", 0.25)])
    plant.set_boundary_fn(lambda t: (723.0, 633.0, 0.48, 0.37))
    Ts = p["Ts"]
    N = int(round(900.0 / Ts))
    t = 0.0
    for _ in range(N):
        plant.step((0.0, 0.0), t)
        t += Ts
    TT, m1, m2 = plant.state()
    Thout1, Thout2, m0, Tmix, TTi = plant.outputs(t)
    return {"x": [TT, m1, m2], "y": [Thout1, Thout2], "t": t}


def linearize(p: dict, trim: dict):
    """Numeric Jacobian of the plant at the trim point.

    Returns (A, B, C) for x_dot = A*x_dev + B*u, y_dev = C*x_dev, where
    x_dev = x - x_trim and y_dev = y - y_trim (u_trim = 0 since at trim the
    flow rates are constant, m1_dot = m2_dot = 0).
    """
    plant = AETMPlant(p)
    plant.set_boundary_fn(lambda t: (723.0, 633.0, 0.48, 0.37))
    x0 = np.array(trim["x"])
    t0 = trim["t"]
    n_buf = plant._tt_buf.maxlen

    def _set_state(x):
        plant._x = list(x)
        from collections import deque
        plant._tt_buf = deque([x[0]] * n_buf, maxlen=n_buf)

    def f(x, u):
        _set_state(x)
        return np.array(plant._derivs(list(x), list(u), t0))

    def h(x):
        _set_state(x)
        Thout1, Thout2, _, _, _ = plant.outputs(t0, list(x))
        return np.array([Thout1, Thout2])

    n, m = 3, 2
    eps_x = np.array([0.5, 0.01, 0.01])
    eps_u = np.array([1e-4, 1e-4])
    A = np.zeros((n, n))
    B = np.zeros((n, m))
    C = np.zeros((2, n))

    for i in range(n):
        dx = np.zeros(n); dx[i] = eps_x[i]
        A[:, i] = (f(x0 + dx, [0.0, 0.0]) - f(x0 - dx, [0.0, 0.0])) / (2 * eps_x[i])
        C[:, i] = (h(x0 + dx) - h(x0 - dx)) / (2 * eps_x[i])
    u0 = np.array([0.0, 0.0])
    for j in range(m):
        du = np.zeros(m); du[j] = eps_u[j]
        B[:, j] = (f(x0, u0 + du) - f(x0, u0 - du)) / (2 * eps_u[j])

    return A, B, C


def build_linear_model(p: dict):
    """Trim + ZOH-discretized deviation-form StateSpace (3 states, 2 in, 2 out)."""
    trim = find_trim(p)
    if not CTRL_AVAILABLE:
        return None, trim
    try:
        import scipy.signal as sig
    except ImportError:
        return None, trim
    A, B, C = linearize(p, trim)
    D = np.zeros((2, 2))
    Ad, Bd, Cd, Dd, _ = sig.cont2discrete((A, B, C, D), p["Ts"], method="zoh")
    return ctrl.StateSpace(Ad, Bd, Cd, Dd, p["Ts"]), trim


def output_sensitivity(p: dict, trim: dict):
    """Static sensitivities dThout1/dm1, dThout2/dm2 at trim (state indices
    1 and 2 are m1, m2 respectively), used to scale controller gains and
    the NeuralPID/Smith-predictor internal models."""
    _, _, C = linearize(p, trim)
    dT1_dm1 = float(C[0, 1])
    dT2_dm2 = float(C[1, 2])
    return dT1_dm1, dT2_dm2


# ---------------------------------------------------------------------------
# Base
# ---------------------------------------------------------------------------
class AETMController:
    def name(self):  return "Base"
    def reset(self): pass
    def compute(self, refs, plant_state, t, outs): return (0.0, 0.0)


# ---------------------------------------------------------------------------
# 1. OpenLoop - hold flow rates constant (no automation, baseline)
# ---------------------------------------------------------------------------
class OpenLoopCtrl(AETMController):
    def name(self): return "OpenLoop"
    def compute(self, refs, plant_state, t, outs):
        return (0.0, 0.0)


# ---------------------------------------------------------------------------
# 2. PID - decoupled dual loop, e = y - ref convention (negative-gain plant)
# ---------------------------------------------------------------------------
class PIDCtrl(AETMController):
    def __init__(self, p: dict):
        Ts = p["Ts"]; ur = p["u_rate_max"]
        # m1/m2 are themselves integrators of u (Eq. 14), so this loop is
        # already "type 1" from u to Thout - Kp alone gives zero steady-
        # state error to a constant reference. Ki kept tiny (handles slow
        # disturbance drift only); Kd small (a 60s transport delay makes
        # derivative action on recent samples unreliable - classic
        # "no D with large delay" rule of thumb).
        pp1 = ctrl.PIDParams()
        pp1.Kp = 4.0e-5; pp1.Ki = 1.0e-7; pp1.Kd = 2.0e-4
        pp1.uMin = -ur; pp1.uMax = ur; pp1.Kb = 0.5
        self._pid1 = ctrl.DiscretePID(pp1, Ts)   # e1 -> u1 (m1_dot)

        pp0 = ctrl.PIDParams()
        pp0.Kp = 4.0e-5; pp0.Ki = 1.0e-7; pp0.Kd = 2.0e-4
        pp0.uMin = -ur; pp0.uMax = ur; pp0.Kb = 0.5
        self._pid0 = ctrl.DiscretePID(pp0, Ts)   # e2 -> u2 (m2_dot)

    def name(self): return "PID"

    def reset(self):
        self._pid0.reset(); self._pid1.reset()

    def compute(self, refs, plant_state, t, outs):
        Thout1, Thout2 = outs[0], outs[1]
        e1 = Thout1 - refs[0]
        e2 = Thout2 - refs[1]
        u1 = self._pid1.compute(e1)
        u0 = self._pid0.compute(e2)
        return (u1, u0)


# ---------------------------------------------------------------------------
# 3. ADRC - decoupled dual loop, e = y - ref convention
# ---------------------------------------------------------------------------
class ADRCCtrl(AETMController):
    def __init__(self, p: dict):
        Ts = p["Ts"]; ur = p["u_rate_max"]
        # omega_o * Ts < 0.5 required; Ts=0.5 -> omega_o < 1.0. Bandwidth
        # also kept well below 1/tau_d (~0.017 rad/s) so the ESO does not
        # chase the 60s transport delay.
        ap1 = ctrl.ADRCParams()
        ap1.omega_c = 0.015; ap1.omega_o = 0.04; ap1.b0 = 5.0e-5
        ap1.uMin = -ur; ap1.uMax = ur
        self._adrc1 = ctrl.DiscreteADRC(ap1, Ts)

        ap0 = ctrl.ADRCParams()
        ap0.omega_c = 0.015; ap0.omega_o = 0.04; ap0.b0 = 5.0e-5
        ap0.uMin = -ur; ap0.uMax = ur
        self._adrc0 = ctrl.DiscreteADRC(ap0, Ts)

    def name(self): return "ADRC"

    def reset(self):
        self._adrc0.reset(); self._adrc1.reset()

    def compute(self, refs, plant_state, t, outs):
        Thout1, Thout2 = outs[0], outs[1]
        e1 = Thout1 - refs[0]
        e2 = Thout2 - refs[1]
        u1 = self._adrc1.compute(e1)
        u0 = self._adrc0.compute(e2)
        return (u1, u0)


# ---------------------------------------------------------------------------
# 4. SMC - decoupled dual loop; native compute(y - ref) convention
# ---------------------------------------------------------------------------
class SMCCtrl(AETMController):
    def __init__(self, p: dict):
        Ts = p["Ts"]; ur = p["u_rate_max"]
        sp1 = ctrl.SMCParams()
        sp1.c_e = 2.0e-4; sp1.c_de = 1.0e-3; sp1.K = 1.5e-2; sp1.phi = 0.01
        sp1.uMin = -ur; sp1.uMax = ur
        self._smc1 = ctrl.DiscreteSMC(sp1, Ts)

        sp0 = ctrl.SMCParams()
        sp0.c_e = 2.0e-4; sp0.c_de = 1.0e-3; sp0.K = 1.5e-2; sp0.phi = 0.01
        sp0.uMin = -ur; sp0.uMax = ur
        self._smc0 = ctrl.DiscreteSMC(sp0, Ts)

    def name(self): return "SMC"

    def reset(self):
        self._smc0.reset(); self._smc1.reset()

    def compute(self, refs, plant_state, t, outs):
        Thout1, Thout2 = outs[0], outs[1]
        u1 = self._smc1.compute(Thout1 - refs[0])
        u0 = self._smc0.compute(Thout2 - refs[1])
        return (u1, u0)


# ---------------------------------------------------------------------------
# 5. LQR - full 3-state regulator toward the nominal trim point
#
# Limitation (documented): LQR regulates the state toward the FIXED trim
# (TT_trim, m1_trim, m2_trim); since the trim was chosen so Thout1/Thout2
# ~= 400K there, it tracks the nominal reference but does not re-solve the
# trim if the setpoint changes (s05) or under sustained disturbance - unlike
# the paper's MPC-with-feedback-correction, plain LQR has no integral action.
# ---------------------------------------------------------------------------
class LQRCtrl(AETMController):
    def __init__(self, p: dict, ss_model, trim: dict):
        lqr_p = ctrl.LQRParams()
        lqr_p.Q = np.diag([1.0, 1.0, 1.0])
        lqr_p.R = np.diag([1.0e4, 1.0e4])
        self._lqr = ctrl.DiscreteLQR(ss_model, lqr_p)
        self._x_trim = np.array(trim["x"])

    def name(self): return "LQR"

    def compute(self, refs, plant_state, t, outs):
        x_dev = np.array(plant_state) - self._x_trim
        u_dev = self._lqr.compute(x_dev, np.zeros(3))
        return (float(u_dev[0]), float(u_dev[1]))


# ---------------------------------------------------------------------------
# 6. MPC - deviation-form constrained MPC tracking [Thout1_ref, Thout2_ref]
# (the paper's headline method: constrained MPC with feedback correction)
# ---------------------------------------------------------------------------
class MPCCtrl(AETMController):
    def __init__(self, p: dict, ss_model, trim: dict):
        ur = p["u_rate_max"]
        mp = ctrl.MPCParams()
        mp.Np = 15; mp.Nc = 5
        mp.rho_y = 1.0; mp.rho_u = 1.0e-6
        mp.uMin = -ur; mp.uMax = ur
        mp.duMin = -ur; mp.duMax = ur
        mp.qp_max_iter = 300
        self._mpc = ctrl.DiscreteMPC(ss_model, mp)
        self._x_trim = np.array(trim["x"])
        self._y_trim = np.array(trim["y"])

    def name(self): return "MPC"

    def reset(self):
        self._mpc.reset()

    def compute(self, refs, plant_state, t, outs):
        x_dev = np.array(plant_state) - self._x_trim
        y_ref_dev = np.array(refs) - self._y_trim
        self._mpc.set_state(x_dev)
        u = self._mpc.compute_ref(x_dev, y_ref_dev)
        return (float(u[0]), float(u[1]))


# ---------------------------------------------------------------------------
# 7. MRAC - dual loop, set_reference(r) + compute(y); negative gammas
# (negative-gain plant - mirrors the Solar Cooker MRAC precedent)
# ---------------------------------------------------------------------------
class MRACCtrl(AETMController):
    def __init__(self, p: dict):
        Ts = p["Ts"]
        mp1 = ctrl.MRACParams()
        mp1.a_m = 0.98; mp1.b_m = 0.02
        mp1.gamma_r = -1.0e-7; mp1.gamma_y = -1.0e-7
        mp1.sigma = 0.01; mp1.theta_max = 0.05
        self._mrac1 = ctrl.MRACController(mp1, Ts)

        mp0 = ctrl.MRACParams()
        mp0.a_m = 0.98; mp0.b_m = 0.02
        mp0.gamma_r = -1.0e-7; mp0.gamma_y = -1.0e-7
        mp0.sigma = 0.01; mp0.theta_max = 0.05
        self._mrac0 = ctrl.MRACController(mp0, Ts)

    def name(self): return "MRAC"

    def reset(self):
        self._mrac0.reset(); self._mrac1.reset()

    def compute(self, refs, plant_state, t, outs):
        Thout1, Thout2 = outs[0], outs[1]
        self._mrac1.set_reference(refs[0])
        u1 = self._mrac1.compute(Thout1)
        self._mrac0.set_reference(refs[1])
        u0 = self._mrac0.compute(Thout2)
        return (u1, u0)


# ---------------------------------------------------------------------------
# 8. L1Adaptive - dual loop, set_reference(r) + compute(y)
# ---------------------------------------------------------------------------
class L1AdaptiveCtrl(AETMController):
    def __init__(self, p: dict):
        Ts = p["Ts"]; ur = p["u_rate_max"]
        lp1 = ctrl.L1AdaptiveParams()
        lp1.a_m = 0.98; lp1.b_m = 0.02; lp1.k_g = -1.0
        lp1.Gamma = 5.0e3; lp1.omega_c = 0.05; lp1.sigma_max = 0.05
        lp1.uMin = -ur; lp1.uMax = ur
        self._l1_1 = ctrl.L1AdaptiveController(lp1, Ts)

        lp0 = ctrl.L1AdaptiveParams()
        lp0.a_m = 0.98; lp0.b_m = 0.02; lp0.k_g = -1.0
        lp0.Gamma = 5.0e3; lp0.omega_c = 0.05; lp0.sigma_max = 0.05
        lp0.uMin = -ur; lp0.uMax = ur
        self._l1_0 = ctrl.L1AdaptiveController(lp0, Ts)

    def name(self): return "L1Adaptive"

    def reset(self):
        self._l1_0.reset(); self._l1_1.reset()

    def compute(self, refs, plant_state, t, outs):
        Thout1, Thout2 = outs[0], outs[1]
        self._l1_1.set_reference(refs[0])
        u1 = self._l1_1.compute(Thout1)
        self._l1_0.set_reference(refs[1])
        u0 = self._l1_0.compute(Thout2)
        return (u1, u0)


# ---------------------------------------------------------------------------
# 9. GainScheduled - 2-point schedule on TT (gentle below, aggressive near
# the anti-boiling margin), one instance per loop
# ---------------------------------------------------------------------------
class GainScheduledCtrl(AETMController):
    def __init__(self, p: dict):
        Ts = p["Ts"]; ur = p["u_rate_max"]

        def _mk_pid(kp):
            pp = ctrl.PIDParams()
            pp.Kp = kp; pp.Ki = 1.0e-7; pp.Kd = 2.0e-4
            pp.uMin = -ur; pp.uMax = ur; pp.Kb = 0.5
            return ctrl.DiscretePID(pp, Ts)

        self._gs1 = ctrl.GainScheduledController(Ts)
        self._gs1.add_schedule_point(380.0, _mk_pid(3.0e-5))
        self._gs1.add_schedule_point(480.0, _mk_pid(8.0e-5))

        self._gs0 = ctrl.GainScheduledController(Ts)
        self._gs0.add_schedule_point(380.0, _mk_pid(3.0e-5))
        self._gs0.add_schedule_point(480.0, _mk_pid(8.0e-5))

    def name(self): return "GainScheduled"

    def compute(self, refs, plant_state, t, outs):
        TT = plant_state[0]
        Thout1, Thout2 = outs[0], outs[1]
        self._gs1.set_scheduling_param(TT)
        u1 = self._gs1.compute(Thout1 - refs[0])
        self._gs0.set_scheduling_param(TT)
        u0 = self._gs0.compute(Thout2 - refs[1])
        return (u1, u0)


# ---------------------------------------------------------------------------
# 10. SmithPredictor - delay-compensated dual loop (the headline disturbance
# of this paper is the 60s transport delay; Smith Predictor is the classic
# dead-time compensator). Internal model: 2-state [m_dev, Thout_dev]
# approximation, m_dev_dot = u; Thout_dev_dot = (g*m_dev - Thout_dev)/tau_lag.
# ---------------------------------------------------------------------------
class SmithPredictorCtrl(AETMController):
    def __init__(self, p: dict, sens):
        Ts = p["Ts"]; ur = p["u_rate_max"]
        m0_nom = p.get("m1_0", 0.25) + p.get("m2_0", 0.25)
        tau_lag = p["mT"] / m0_nom               # tank residence time estimate
        delay_steps = max(1, int(round(p["tau_d"] / Ts)))
        dT1_dm1, dT2_dm2 = sens

        def _model(g):
            Ac = np.array([[0.0, 0.0], [g / tau_lag, -1.0 / tau_lag]])
            Bc = np.array([[1.0], [0.0]])
            Cc = np.array([[0.0, 1.0]])
            Dc = np.array([[0.0]])
            import scipy.signal as sig
            Ad, Bd, Cd, Dd, _ = sig.cont2discrete((Ac, Bc, Cc, Dc), Ts, method="zoh")
            return ctrl.StateSpace(Ad, Bd, Cd, Dd, Ts)

        pp1 = ctrl.PIDParams()
        pp1.Kp = 4.0e-5; pp1.Ki = 1.0e-7; pp1.Kd = 2.0e-4
        pp1.uMin = -ur; pp1.uMax = ur; pp1.Kb = 0.5
        self._sp1 = ctrl.SmithPredictor(ctrl.DiscretePID(pp1, Ts), _model(dT1_dm1), delay_steps)

        pp0 = ctrl.PIDParams()
        pp0.Kp = 4.0e-5; pp0.Ki = 1.0e-7; pp0.Kd = 2.0e-4
        pp0.uMin = -ur; pp0.uMax = ur; pp0.Kb = 0.5
        self._sp0 = ctrl.SmithPredictor(ctrl.DiscretePID(pp0, Ts), _model(dT2_dm2), delay_steps)

    def name(self): return "SmithPredictor"

    def reset(self):
        self._sp0.reset(); self._sp1.reset()

    def compute(self, refs, plant_state, t, outs):
        Thout1, Thout2 = outs[0], outs[1]
        u1 = self._sp1.compute(Thout1 - refs[0])
        u0 = self._sp0.compute(Thout2 - refs[1])
        return (u1, u0)


# ---------------------------------------------------------------------------
# 11. NeuralPID - dual loop, standard compute(r - y) convention, negative
# plant_gain (mirrors Solar Cooker's NeuralPID precedent)
# ---------------------------------------------------------------------------
class NeuralPIDCtrl(AETMController):
    def __init__(self, p: dict, sens):
        Ts = p["Ts"]; ur = p["u_rate_max"]
        dT1_dm1, dT2_dm2 = sens

        np1 = ctrl.NeuralPIDParams()
        np1.n_hidden = 4; np1.lr = 1.0e-9; np1.Ts = Ts
        np1.plant_gain = dT1_dm1 * Ts
        np1.uMin = -ur; np1.uMax = ur
        np1.Kp0 = 4.0e-5; np1.Ki0 = 1.0e-7; np1.Kd0 = 2.0e-4
        self._npid1 = ctrl.NeuralPID(np1)

        np0 = ctrl.NeuralPIDParams()
        np0.n_hidden = 4; np0.lr = 1.0e-9; np0.Ts = Ts
        np0.plant_gain = dT2_dm2 * Ts
        np0.uMin = -ur; np0.uMax = ur
        np0.Kp0 = 4.0e-5; np0.Ki0 = 1.0e-7; np0.Kd0 = 2.0e-4
        self._npid0 = ctrl.NeuralPID(np0)

    def name(self): return "NeuralPID"

    def compute(self, refs, plant_state, t, outs):
        Thout1, Thout2 = outs[0], outs[1]
        u1 = self._npid1.compute(refs[0] - Thout1)
        u0 = self._npid0.compute(refs[1] - Thout2)
        return (u1, u0)


# ---------------------------------------------------------------------------
# 12. ILC - two-phase iterative learning (PID feedback, then PID + learned
# feedforward), applied to the m1/Thout1 loop; m2/Thout2 loop uses plain PID.
# ---------------------------------------------------------------------------
class ILCCtrl(AETMController):
    N_TRIAL = 400  # 200 s at Ts=0.5

    def __init__(self, p: dict):
        Ts = p["Ts"]; ur = p["u_rate_max"]
        self._ur = ur

        pp1 = ctrl.PIDParams()
        pp1.Kp = 4.0e-5; pp1.Ki = 1.0e-7; pp1.Kd = 2.0e-4
        pp1.uMin = -ur; pp1.uMax = ur; pp1.Kb = 0.5
        self._pid1 = ctrl.DiscretePID(pp1, Ts)

        ip = ctrl.ILCParams()
        ip.N = self.N_TRIAL; ip.Ts = Ts
        ip.mode = ctrl.ILCMode.PType
        ip.Lp = 0.5; ip.Q_filter = 0.95
        ip.uMin = -ur * 0.5; ip.uMax = ur * 0.5
        self._ilc = ctrl.ILC(ip)

        pp0 = ctrl.PIDParams()
        pp0.Kp = 4.0e-5; pp0.Ki = 1.0e-7; pp0.Kd = 2.0e-4
        pp0.uMin = -ur; pp0.uMax = ur; pp0.Kb = 0.5
        self._pid0 = ctrl.DiscretePID(pp0, Ts)

        self._k = 0
        self._phase2 = False

    def name(self): return "ILC"

    def reset(self):
        self._pid0.reset(); self._pid1.reset(); self._ilc.reset()
        self._k = 0; self._phase2 = False

    def compute(self, refs, plant_state, t, outs):
        Thout1, Thout2 = outs[0], outs[1]
        e1 = Thout1 - refs[0]
        u1_pid = self._pid1.compute(e1)
        if not self._phase2:
            if self._k < self.N_TRIAL:
                self._ilc.record_error(self._k, e1)
            self._k += 1
            if self._k == self.N_TRIAL:
                self._ilc.update_feedforward()
                self._phase2 = True
                self._k = 0
            u1 = u1_pid
        else:
            k2 = min(self._k, self.N_TRIAL - 1)
            u1_ff = self._ilc.feedforward(k2)
            self._k += 1
            u1 = max(-self._ur, min(self._ur, u1_pid + u1_ff))

        u0 = self._pid0.compute(Thout2 - refs[1])
        return (u1, u0)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def make_controllers(plant_params: dict):
    """Return a list of all 12 AETMController instances."""
    ctrl_list = [OpenLoopCtrl()]
    if not CTRL_AVAILABLE:
        print("WARNING: ctrl_toolbox not available - only OpenLoop will run.")
        return ctrl_list

    ss_model, trim = build_linear_model(plant_params)
    sens = output_sensitivity(plant_params, trim)

    ctrl_list += [
        PIDCtrl(plant_params),
        ADRCCtrl(plant_params),
        SMCCtrl(plant_params),
        LQRCtrl(plant_params, ss_model, trim),
        MPCCtrl(plant_params, ss_model, trim),
        MRACCtrl(plant_params),
        L1AdaptiveCtrl(plant_params),
        GainScheduledCtrl(plant_params),
        SmithPredictorCtrl(plant_params, sens),
        NeuralPIDCtrl(plant_params, sens),
        ILCCtrl(plant_params),
    ]
    return ctrl_list
