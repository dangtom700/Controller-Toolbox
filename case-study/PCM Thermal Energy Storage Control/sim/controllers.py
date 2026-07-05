"""
controllers.py
Twelve controllers for the PCM-HP thermal-energy-storage case study.

Interface:
    name()  -> str
    reset() -> None
    compute(soc_ref, soc, T_o, t) -> u        # normalised compressor speed in [0,1]

The plant is a stable SoC integrator whose input u (compressor speed) has a
POSITIVE static gain to SoC, driven by the building load as a disturbance. The
nominal operating point is u0 ~= 0.5 (HP cooling balances the load), so controllers
without their own integral action are biased by u0 and their outputs clamped to
[0,1]. Sign convention: e = ref - soc with positive gains (more compressor ->
more charging -> higher SoC); DiscreteSMC is fed its native e = soc - ref.

The paper's proposed method is an MPC expert whose optimal actions train neural
imitation-learning agents; MPC is the roster's proposed controller and NeuralPID
stands in for the learned imitation policy (BC/GAIL).

ctrl_toolbox is imported with a graceful fallback.
"""

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

U0 = 0.5   # nominal operating compressor speed (cooling ~ load)

R_MAX = 2900.0


def _clip01(u):
    return max(0.0, min(1.0, u))


def load_feedforward(T_o, P_load):
    """Normalised compressor speed u_ff whose HP cooling Q_hp(r,T_o) matches the
    current building load P_load (invert the paper's quadratic Q_hp map, Eq. 2).

    The SoC integrator's true hold-point is u ~= u_ff (not 0), so the model-based
    controllers (MPC/GPC/LQR/ScenarioMPC), whose linear model assumes u=0 holds
    SoC, use this as a feedforward and only supply the tracking correction on top.
    """
    a = -7.0e-7
    b = 5.33e-3
    c = -2.07e-1 - 1.37e-1 * T_o + 1.24e-3 * T_o * T_o - P_load
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return 1.0                      # load exceeds max cooling -> full speed
    r = (-b + (disc ** 0.5)) / (2.0 * a)   # smaller positive root (a < 0)
    return max(0.0, min(1.0, r / R_MAX))


def _soc_state_space(p):
    """1-state SoC integrator model SoC[k+1] = SoC[k] + B*u, ZOH already discrete."""
    B = p.get("B_soc", 0.30)
    A = np.array([[1.0]])
    Bd = np.array([[B]])
    C = np.array([[1.0]])
    D = np.array([[0.0]])
    return ctrl.StateSpace(A, Bd, C, D, p.get("Ts", 1.0))


# ---------------------------------------------------------------------------
class PCMController:
    def name(self): return "Base"
    def reset(self): pass
    def compute(self, ref, soc, T_o, P_load, t): return U0


# 1. OpenLoop - hold the nominal compressor speed (no storage management)
class OpenLoopCtrl(PCMController):
    def name(self): return "OpenLoop"
    def compute(self, ref, soc, T_o, P_load, t): return U0


# 2. PID
def _mk_pid(Kp, Ki, Kd, Ts, umin=0.0, umax=1.0):
    pp = ctrl.PIDParams()
    pp.Kp = Kp; pp.Ki = Ki; pp.Kd = Kd
    pp.uMin = umin; pp.uMax = umax; pp.Kb = 0.5
    return ctrl.DiscretePID(pp, Ts)


class PIDCtrl(PCMController):
    def __init__(self, p):
        self._pid = _mk_pid(2.5, 0.6, 0.0, p["Ts"])

    def name(self): return "PID"
    def reset(self): self._pid.reset()

    def compute(self, ref, soc, T_o, P_load, t):
        return _clip01(self._pid.compute(ref - soc))


# 3. MPC (PROPOSED) - constrained receding-horizon expert (paper's headline method)
class MPCCtrl(PCMController):
    def __init__(self, p, ss):
        mp = ctrl.MPCParams()
        mp.Np = 12; mp.Nc = 4                 # 12-h horizon (paper Sec. 2.2.2)
        mp.rho_y = 1.0; mp.rho_u = 5.0e-2
        mp.uMin = -0.5; mp.uMax = 0.5      # deviation about the load feedforward
        mp.duMin = -0.4; mp.duMax = 0.4
        mp.qp_max_iter = 200
        self._mpc = ctrl.DiscreteMPC(ss, mp)

    def name(self): return "MPC"
    def reset(self): self._mpc.reset()

    def compute(self, ref, soc, T_o, P_load, t):
        x = np.array([soc])
        self._mpc.set_state(x)
        u_dev = float(self._mpc.compute_ref(x, np.array([ref]))[0])
        return _clip01(load_feedforward(T_o, P_load) + u_dev)


# 4. NeuralPID - stands in for the paper's imitation-learning policy (BC / GAIL)
class NeuralPIDCtrl(PCMController):
    def __init__(self, p):
        np_ = ctrl.NeuralPIDParams()
        np_.n_hidden = 6; np_.lr = 1.0e-5; np_.Ts = p["Ts"]
        np_.plant_gain = p.get("B_soc", 0.30)
        np_.uMin = 0.0; np_.uMax = 1.0
        np_.Kp0 = 2.5; np_.Ki0 = 0.6; np_.Kd0 = 0.0
        self._npid = ctrl.NeuralPID(np_)

    def name(self): return "NeuralPID"
    def reset(self):
        pass

    def compute(self, ref, soc, T_o, P_load, t):
        return _clip01(self._npid.compute(ref - soc))


# 5. GainScheduled - PID gains scheduled on outdoor temperature
class GainScheduledCtrl(PCMController):
    def __init__(self, p):
        Ts = p["Ts"]
        self._gs = ctrl.GainScheduledController(Ts)
        self._gs.add_schedule_point(18.0, _mk_pid(1.8, 0.4, 0.0, Ts))
        self._gs.add_schedule_point(28.0, _mk_pid(2.6, 0.7, 0.0, Ts))
        self._gs.add_schedule_point(36.0, _mk_pid(3.4, 0.9, 0.0, Ts))

    def name(self): return "GainScheduled"
    def reset(self):
        pass

    def compute(self, ref, soc, T_o, P_load, t):
        self._gs.set_scheduling_param(T_o)
        return _clip01(self._gs.compute(ref - soc))


# 6. LQR - 1-state optimal feedback with u0 feedforward
class LQRCtrl(PCMController):
    def __init__(self, p, ss):
        lp = ctrl.LQRParams()
        lp.Q = np.array([[40.0]])
        lp.R = np.array([[1.0]])
        self._lqr = ctrl.DiscreteLQR(ss, lp)

    def name(self): return "LQR"

    def compute(self, ref, soc, T_o, P_load, t):
        u_ff = load_feedforward(T_o, P_load)
        u = self._lqr.compute(np.array([soc]), np.array([ref]), np.array([u_ff]))
        return _clip01(float(u[0]))


# 7. FractionalOrderPID
class FOPIDCtrl(PCMController):
    def __init__(self, p):
        fp = ctrl.FOPIDParams()
        fp.Kp = 2.2; fp.Ki = 0.5; fp.Kd = 0.1
        fp.lam = 0.9; fp.mu = 0.6
        fp.wb = 1.0e-2; fp.wh = 5.0; fp.N = 4
        fp.uMin = 0.0; fp.uMax = 1.0; fp.Kaw = 0.5
        self._fo = ctrl.FractionalOrderPID(fp, p["Ts"])

    def name(self): return "FOPID"
    def reset(self): self._fo.reset()

    def compute(self, ref, soc, T_o, P_load, t):
        return _clip01(self._fo.compute(ref - soc))


# 8. SMC - boundary-layer sliding mode, native e = soc - ref, biased by u0
class SMCCtrl(PCMController):
    def __init__(self, p):
        sp = ctrl.SMCParams()
        sp.c_e = 3.0; sp.c_de = 0.5; sp.K = 0.5; sp.phi = 0.4
        sp.uMin = -0.5; sp.uMax = 0.5
        self._smc = ctrl.DiscreteSMC(sp, p["Ts"])

    def name(self): return "SMC"
    def reset(self): self._smc.reset()

    def compute(self, ref, soc, T_o, P_load, t):
        return _clip01(U0 + self._smc.compute(soc - ref))


# 9. ADRC - active disturbance rejection (load is the estimated disturbance)
class ADRCCtrl(PCMController):
    def __init__(self, p):
        ap = ctrl.ADRCParams()
        # omega_o * Ts < 0.5 (Ts = 1 h) -> omega_o < 0.5.
        ap.omega_c = 0.15; ap.omega_o = 0.35; ap.b0 = p.get("B_soc", 0.30)
        ap.uMin = -0.5; ap.uMax = 0.5
        self._adrc = ctrl.DiscreteADRC(ap, p["Ts"])

    def name(self): return "ADRC"
    def reset(self): self._adrc.reset()

    def compute(self, ref, soc, T_o, P_load, t):
        return _clip01(load_feedforward(T_o, P_load) + self._adrc.compute(ref - soc))


# 10. GPC - generalized predictive control (CARIMA, integral action)
class GPCCtrl(PCMController):
    def __init__(self, p, ss):
        gp = ctrl.GPCParams()
        gp.Np = 12; gp.Nu = 3
        gp.rho_y = 1.0; gp.rho_u = 5.0e-2; gp.alpha = 0.0
        gp.uMin = -0.5; gp.uMax = 0.5      # deviation about the load feedforward
        gp.duMin = -0.4; gp.duMax = 0.4
        gp.qp_max_iter = 200
        self._gpc = ctrl.GeneralizedPredictiveController(ss, gp)

    def name(self): return "GPC"
    def reset(self): self._gpc.reset()

    def compute(self, ref, soc, T_o, P_load, t):
        u_dev = float(self._gpc.compute_ref(soc, ref))
        return _clip01(load_feedforward(T_o, P_load) + u_dev)


# 11. LeadLag - classical lead compensator, biased by u0
class LeadLagCtrl(PCMController):
    def __init__(self, p):
        lp = ctrl.LeadLagParams()
        lp.continuous_zero = 0.05
        lp.continuous_pole = 0.6
        lp.gain = 3.0
        self._ll = ctrl.DiscreteLeadLag(lp, p["Ts"])

    def name(self): return "LeadLag"
    def reset(self): self._ll.reset()

    def compute(self, ref, soc, T_o, P_load, t):
        return _clip01(load_feedforward(T_o, P_load) + self._ll.compute(ref - soc))


# 12. ScenarioMPC - stochastic MPC (the paper adds Gaussian forecast noise to
#     the prediction horizon; scenario sampling hedges that uncertainty)
class ScenarioMPCCtrl(PCMController):
    def __init__(self, p, ss):
        sp = ctrl.ScenarioMPCParams()
        sp.Np = 12; sp.Nu = 3
        sp.Q = np.array([[1.0]]); sp.R = np.array([[5.0e-2]])
        sp.Sigma_w = np.array([[4.0e-4]])   # forecast-uncertainty process noise
        sp.N_samples = 12
        sp.uMin = np.array([-0.5]); sp.uMax = np.array([0.5])  # deviation about feedforward
        sp.Ts = p["Ts"]; sp.qp_max_iter = 200; sp.seed = 1
        self._smpc = ctrl.ScenarioMPC(ss, sp)

    def name(self): return "ScenarioMPC"
    def reset(self): self._smpc.reset()

    def compute(self, ref, soc, T_o, P_load, t):
        x = np.array([soc])
        self._smpc.set_state(x)
        u_dev = float(self._smpc.compute_ref(x, np.array([ref]))[0])
        return _clip01(load_feedforward(T_o, P_load) + u_dev)


# ---------------------------------------------------------------------------
def make_controllers(p: dict):
    """Return the 12-controller PCM roster."""
    roster = [OpenLoopCtrl()]
    if not _HAVE_CTRL:
        print("WARNING: ctrl_toolbox not available - only OpenLoop will run.")
        return roster

    ss = _soc_state_space(p)
    roster += [
        PIDCtrl(p),
        MPCCtrl(p, ss),
        NeuralPIDCtrl(p),
        GainScheduledCtrl(p),
        LQRCtrl(p, ss),
        FOPIDCtrl(p),
        SMCCtrl(p),
        ADRCCtrl(p),
        GPCCtrl(p, ss),
        LeadLagCtrl(p),
        ScenarioMPCCtrl(p, ss),
    ]
    return roster
