"""
controllers.py
Sixteen controllers for the FOWT+WEC power-extraction case study.

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
# 9. L1Adaptive (bounded-transient adaptive tracking of x_rel)
# ---------------------------------------------------------------------------
class L1AdaptiveCtrl(WaveController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        lp = ctrl.L1AdaptiveParams()
        lp.a_m = 0.85; lp.b_m = 0.15; lp.k_g = 1.0
        lp.Gamma = 20.0; lp.omega_c = 0.5; lp.sigma_max = 200.0
        lp.uMin = -p['F_max']; lp.uMax = p['F_max']
        self._l1 = ctrl.L1AdaptiveController(lp, Ts)

    def name(self): return "L1Adaptive"

    def reset(self):
        self._l1.reset()

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, _ = plant_state
        self._l1.set_reference(x_ref)
        return self._l1.compute(x_rel)


# ---------------------------------------------------------------------------
# 10. ILC (Iterative Learning Control - two-phase within one run)
#
# Phase 1 (k < N_TRIAL): pure PID feedback; record errors.
# At k = N_TRIAL: update_feedforward() computes trial-1 feedforward.
# Phase 2: PID + ILC feedforward.
# ---------------------------------------------------------------------------
class ILCCtrl(WaveController):
    N_TRIAL = 300  # 150 s at Ts=0.5

    def __init__(self, p: dict):
        Ts = p['Ts']
        self._F_max = p['F_max']
        pp = ctrl.PIDParams()
        pp.Kp = 5.0e4; pp.Ki = 1.0e3; pp.Kd = 2.0e3
        pp.uMin = -self._F_max; pp.uMax = self._F_max; pp.Kb = 0.5
        self._pid = ctrl.DiscretePID(pp, Ts)
        ip = ctrl.ILCParams()
        ip.N = self.N_TRIAL; ip.Ts = Ts
        ip.mode = ctrl.ILCMode.PType
        ip.Lp = 0.6; ip.Q_filter = 0.95
        ip.uMin = -self._F_max * 0.5; ip.uMax = self._F_max * 0.5
        self._ilc = ctrl.ILC(ip)
        self._k = 0
        self._phase2 = False

    def name(self): return "ILC"

    def reset(self):
        self._pid.reset()
        self._ilc.reset()
        self._k = 0
        self._phase2 = False

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, _ = plant_state
        e = x_ref - x_rel
        u_pid = self._pid.compute(e)
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
            k2 = min(self._k, self.N_TRIAL - 1)
            u_ff = self._ilc.feedforward(k2)
            self._k += 1
            return u_pid + u_ff


# ---------------------------------------------------------------------------
# 11. DynaCtrl (Dyna MBRL: PID + SINDy error-dynamics model)
# ---------------------------------------------------------------------------
class DynaCtrl(WaveController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        pp = ctrl.PIDParams()
        pp.Kp = 5.0e4; pp.Ki = 1.0e3; pp.Kd = 2.0e3
        pp.uMin = -p['F_max']; pp.uMax = p['F_max']; pp.Kb = 0.5
        pid = ctrl.DiscretePID(pp, Ts)
        dp = ctrl.DynaParams()
        dp.Ts = Ts; dp.n_collect = 80; dp.n_refit_every = 50
        self._dyna = ctrl.DynaController(dp, pid)

    def name(self): return "DynaCtrl"

    def reset(self):
        self._dyna.reset()

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, _ = plant_state
        return self._dyna.compute(x_ref - x_rel)


# ---------------------------------------------------------------------------
# 12. CEMCtrl (Cross-Entropy Method MPC on 4-state linearized FOWT model)
#
# "Bad fit" demo: CEM uses a deterministic linear model for a wave-excited
# system; shows how stochastic MPC degrades with model/plant mismatch.
# ---------------------------------------------------------------------------
class CEMCtrl(WaveController):
    def __init__(self, p: dict, ss_model):
        Ts = p['Ts']
        A = ss_model.A; B = ss_model.B

        def _f(x, u):
            return A @ x + B @ u

        # Output matrix: track x_rel (state index 2)
        C = np.array([[0.0, 0.0, 1.0, 0.0]])

        cp = ctrl.CEMParams()
        cp.Np = 8; cp.N_samples = 40; cp.n_iter = 3
        cp.elite_frac = 0.1
        cp.Q = 1.0e4; cp.R = 1.0e-8
        cp.uMin = -p['F_max']; cp.uMax = p['F_max']
        cp.sigma_init = p['F_max'] * 0.1; cp.seed = 11
        self._cem = ctrl.CEMController(cp, _f, C, Ts)

    def name(self): return "CEMCtrl"

    def reset(self):
        self._cem.reset()

    def compute(self, x_ref, plant_state, t):
        z, zd, xr, xrd = plant_state
        x = np.array([z, zd, xr, xrd])
        r = np.array([x_ref])
        self._cem.set_state(x)
        self._cem.set_reference(r)
        return float(self._cem.compute_ref(x, r)[0])


# ---------------------------------------------------------------------------
# 13. ScenarioMPCCtrl (Scenario MPC - 4-state, stochastic robustness)
# ---------------------------------------------------------------------------
class ScenarioMPCCtrl(WaveController):
    def __init__(self, p: dict, ss_model):
        Ts = p['Ts']
        F_max = p['F_max']
        sp = ctrl.ScenarioMPCParams()
        sp.Np = 8; sp.Nu = 3; sp.Ts = Ts
        sp.Q = np.array([[1.0e4]])
        sp.R = np.array([[1.0e-8]])
        sp.Sigma_w = np.eye(4) * 0.01
        sp.N_samples = 20; sp.seed = 17
        sp.uMin = np.array([-F_max]); sp.uMax = np.array([F_max])
        sp.qp_max_iter = 200; sp.qp_tol = 1e-4
        self._smpc = ctrl.ScenarioMPC(ss_model, sp)

    def name(self): return "ScenarioMPC"

    def reset(self):
        self._smpc.reset()

    def compute(self, x_ref, plant_state, t):
        z, zd, xr, xrd = plant_state
        x = np.array([z, zd, xr, xrd])
        self._smpc.set_state(x)
        self._smpc.set_reference(np.array([x_ref]))
        return float(self._smpc.compute_control()[0])


# ---------------------------------------------------------------------------
# 14. KoopmanMPCCtrl (EDMD + DiscreteMPC on 4-state FOWT)
# ---------------------------------------------------------------------------
class KoopmanMPCCtrl(WaveController):
    _WARMUP = 80

    def __init__(self, p: dict):
        self._p  = p
        self._Ts = p['Ts']
        self._reset_internals()

    def _make_pid(self):
        pp = ctrl.PIDParams()
        pp.Kp = 5.0e4; pp.Ki = 1.0e3; pp.Kd = 2.0e3
        pp.uMin = -self._p['F_max']; pp.uMax = self._p['F_max']; pp.Kb = 0.5
        return ctrl.DiscretePID(pp, self._Ts)

    def _make_edmd(self):
        kp = ctrl.KoopmanEDMDParams()
        kp.n_state = 4; kp.n_input = 1
        kp.dict = ctrl.KoopmanDict.PolyDeg1; kp.tikhonov = 1e-6
        return ctrl.KoopmanEDMD(kp)

    def _reset_internals(self):
        self._pid    = self._make_pid()
        self._edmd   = self._make_edmd()
        self._mpc    = None
        self._step   = 0
        self._x_prev = None
        self._u_prev = 0.0
        self._fitted = False

    def name(self): return "KoopmanMPC"

    def reset(self):
        self._reset_internals()

    def compute(self, x_ref, plant_state, t):
        z, zd, xr, xrd = plant_state
        x_now = np.array([z, zd, xr, xrd])
        e = x_ref - xr

        if self._step > 0 and not self._fitted and self._x_prev is not None:
            self._edmd.add_snapshot(
                self._x_prev, np.array([self._u_prev]), x_now)

            if self._edmd.snapshot_count() >= self._WARMUP:
                try:
                    ss_proj = self._edmd.fit_projected()
                    C_new = np.array([[0.0, 0.0, 1.0, 0.0]])
                    D_new = np.array([[0.0]])
                    ss_fixed = ctrl.StateSpace(
                        ss_proj.A, ss_proj.B, C_new, D_new, self._Ts)
                    F_max = self._p['F_max']
                    mp = ctrl.MPCParams()
                    mp.Np = 8; mp.Nc = 3
                    mp.rho_y = 1.0e4; mp.rho_u = 1.0e-8
                    mp.uMin = -F_max; mp.uMax = F_max
                    mp.duMin = -1.0e5; mp.duMax = 1.0e5
                    mp.qp_max_iter = 200
                    self._mpc = ctrl.DiscreteMPC(ss_fixed, mp)
                    self._fitted = True
                except Exception:
                    pass

        if self._fitted and self._mpc is not None:
            try:
                self._mpc.set_state(x_now)
                u = float(self._mpc.compute_ref(x_now, np.array([x_ref]))[0])
            except Exception:
                u = self._pid.compute(e)
        else:
            u = self._pid.compute(e)

        self._x_prev = x_now.copy()
        self._u_prev = u
        self._step  += 1
        return u


# ---------------------------------------------------------------------------
# 15. ESNCtrl (Echo State Network predictor + model-inversion correction)
#
# Pre-trained on simplified 2nd-order WEC dynamics in __init__.
# At each step: predict x_rel_next using (F_PTO_norm, x_rel) as input;
# add correction proportional to predicted tracking error.
# ---------------------------------------------------------------------------
class ESNCtrl(WaveController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        F_max = p['F_max']
        M_w = p['M_w']; K_w = p['K_w']; B_w = p['B_w']
        self._F_max = F_max

        ep = ctrl.ESNParams()
        ep.n_res = 60; ep.n_in = 2; ep.n_out = 1
        ep.spectral_radius = 0.9; ep.sparsity = 0.85
        ep.alpha = 0.3; ep.ridge = 1e-4; ep.washout = 20; ep.seed = 7
        self._esn = ctrl.EchoStateNetwork(ep)

        # Pre-train on simplified WEC dynamics (ignoring FOWT coupling):
        # xrel_ddot approx = -(K_w*xr + B_w*xrd + F_pto) / M_w
        # Euler-forward: xrd_next = xrd + Ts*xrel_ddot, xr_next = xr + Ts*xrd
        rng = np.random.default_rng(7)
        xr = 0.0; xrd = 0.0
        for _ in range(300):
            f_pto = float(rng.uniform(-F_max * 0.5, F_max * 0.5))
            xrd_next = xrd + Ts * (-(K_w * xr + B_w * xrd + f_pto) / M_w)
            xr_next  = xr + Ts * xrd_next
            f_norm   = f_pto / F_max
            self._esn.step_reservoir(np.array([f_norm, xr]))
            self._esn.add_training_target(np.array([xr_next]))
            xr = xr_next; xrd = xrd_next
        self._esn.fit_readout()
        self._esn.reset()
        self._u_prev = 0.0

    def name(self): return "ESNCtrl"

    def reset(self):
        self._esn.reset()
        self._u_prev = 0.0

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, _ = plant_state
        f_norm_prev = self._u_prev / self._F_max
        pred_next = float(self._esn.predict(
            np.array([f_norm_prev, x_rel]))[0])
        e = x_ref - x_rel
        pred_err = x_ref - pred_next
        # Proportional tracking + prediction-based correction
        u = 5.0e4 * e + 2.0e4 * pred_err
        u = float(np.clip(u, -self._F_max, self._F_max))
        self._u_prev = u
        return u


# ---------------------------------------------------------------------------
# 16. CBFCtrl (Control Barrier Function safety filter wrapping PID)
#
# Constrains WEC arm velocity: h(xrel_dot) = v_max - xrel_dot >= 0.
# Nominal: PID tracking x_rel. CBF modulates F_PTO to enforce velocity limit.
# f0 and g use approximate 1D WEC dynamics (2nd-order effect; "bad fit").
# ---------------------------------------------------------------------------
class CBFCtrl(WaveController):
    def __init__(self, p: dict):
        Ts = p['Ts']
        F_max = p['F_max']
        M_w = p['M_w']
        self._v_max = 0.5  # m/s WEC arm velocity limit

        pp = ctrl.PIDParams()
        pp.Kp = 5.0e4; pp.Ki = 1.0e3; pp.Kd = 2.0e3
        pp.uMin = -F_max; pp.uMax = F_max; pp.Kb = 0.5
        pid = ctrl.DiscretePID(pp, Ts)

        cbf_p = ctrl.CBFParams()
        cbf_p.alpha = 1.5
        cbf_p.uMin = -F_max; cbf_p.uMax = F_max

        v_max = self._v_max
        g_val = -1.0 / M_w  # F_PTO contribution to xrel_ddot

        self._cbf = ctrl.CBFSafetyFilter(
            pid,
            lambda x: v_max - x,       # h(xrel_dot) = v_max - xrel_dot
            lambda x: -1.0,             # dh/dx = -1
            lambda x: 0.0,              # f0: approximate drift as 0
            lambda x: g_val,            # g: -1/M_w
            cbf_p,
            Ts
        )

    def name(self): return "CBFSafety"

    def reset(self):
        self._cbf.reset()

    def compute(self, x_ref, plant_state, t):
        _, _, x_rel, xrel_dot = plant_state
        # CBF state = xrel_dot (the velocity to constrain)
        self._cbf.set_state(xrel_dot)
        return self._cbf.compute(x_ref - x_rel)


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def make_controllers(plant_params: dict):
    """Return a list of all 16 WaveController instances."""
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
        L1AdaptiveCtrl(plant_params),
        ILCCtrl(plant_params),
        DynaCtrl(plant_params),
        CEMCtrl(plant_params, ss),
        ScenarioMPCCtrl(plant_params, ss),
        KoopmanMPCCtrl(plant_params),
        ESNCtrl(plant_params),
        CBFCtrl(plant_params),
    ]
    return ctrl_list
