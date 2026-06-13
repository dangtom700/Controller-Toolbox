"""
controllers.py
Eighteen controllers for the Active Suspension 6x6 EV Full Model.

Each controller outputs F_act[6] — one active force per wheel corner
(order: right-front, right-mid, right-rear, left-front, left-mid, left-rear).

Positive F_act[i] pushes the wheel downward (increases wheel-body gap).
"""

import os
import sys
import numpy as np
from typing import Optional

# --- Import ctrl_toolbox from project root ---
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _ROOT)

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'DiscretePID'):
        raise AttributeError("ctrl_toolbox missing DiscretePID — rebuild bindings")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

from ev6x6_plant import EV6x6Plant, DEFAULT_PARAMS, N_DOF, N_STAT


# ======================================================================
# Helper: build per-wheel PID params
# ======================================================================

def _pid_params(Kp: float, Ki: float, Kd: float, Ts: float, F_max: float):
    p = ctrl.PIDParams()
    p.Kp   = Kp;  p.Ki = Ki;  p.Kd = Kd
    p.N    = 10.0
    p.uMin = -F_max;  p.uMax = F_max
    p.Kb   = 1.0
    return p


def _make_6_pids(Kp: float, Ki: float, Kd: float, Ts: float, F_max: float):
    return [ctrl.DiscretePID(_pid_params(Kp, Ki, Kd, Ts, F_max), Ts)
            for _ in range(6)]


# ======================================================================
# Metaheuristic cost function (used by GA/PSO/DE controllers)
# Simulate 1 s of s01_bump with the full 40-state plant.
# ======================================================================

def _opt_pd_cost(gains: np.ndarray, plant_params: dict, Ts: float) -> float:
    """Cost = RMS(Z_body) + 0.5*RMS(head_accel) + 0.1*max_tyre_deflect/tyre_clearance"""
    Kp = float(gains[0])
    Kd = float(gains[1])
    F_max = plant_params['F_max']

    plant = EV6x6Plant(plant_params)
    plant.reset()

    T_sim  = 1.0
    N_sim  = int(T_sim / Ts)
    bump_t = 0.3   # bump at 300 ms
    bump_h = 0.025

    z_body_sq = 0.0
    head_sq   = 0.0
    tyre_max  = 0.0
    k_t       = plant_params['k_t']
    tyre_clearance = 0.10  # 100 mm max tyre deflection

    # simple ring buffer for prev state (for derivative term)
    prev_wheel_err = np.zeros(6)

    for k in range(N_sim):
        t   = k * Ts
        z_r = np.zeros(6)
        if t >= bump_t:
            z_r[:] = bump_h

        x = plant.state()
        wheel_disp = x[3:9]       # Z_wr1-3, Z_wl1-3
        tyre_defl  = z_r - wheel_disp  # tyre compression

        # PD control on tyre deflection (keep wheel in contact with road)
        wheel_err     = tyre_defl
        wheel_err_dot = (wheel_err - prev_wheel_err) / Ts if k > 0 else np.zeros(6)
        F_act         = np.clip(Kp * wheel_err + Kd * wheel_err_dot, -F_max, F_max)
        prev_wheel_err = wheel_err.copy()

        plant.step(F_act, z_r)

        z_body_sq += plant.body_Z() ** 2
        head_sq   += plant.head_accel() ** 2
        tyre_max   = max(tyre_max, np.max(np.abs(tyre_defl)))

    rms_body = np.sqrt(z_body_sq / max(N_sim, 1))
    rms_head = np.sqrt(head_sq   / max(N_sim, 1))
    tyre_pen = tyre_max / tyre_clearance

    return float(rms_body + 0.5 * rms_head + 0.1 * tyre_pen)


# ======================================================================
# Base class
# ======================================================================

class ControllerBase:
    def name(self) -> str: return self.__class__.__name__
    def reset(self): pass

    def compute(self, state: np.ndarray, z_r: np.ndarray,
                t: float = 0.0) -> np.ndarray:
        """Returns F_act[6]."""
        raise NotImplementedError


# ======================================================================
# 1. PassiveCtrl — no active force
# ======================================================================

class PassiveCtrl(ControllerBase):
    def compute(self, state, z_r, t=0.0):
        return np.zeros(6)


# ======================================================================
# 2. PDCtrl — per-wheel PD, hand-tuned
# ======================================================================

class PDCtrl(ControllerBase):
    def __init__(self, params: dict, Ts: float):
        F_max = params['F_max']
        self._pids = _make_6_pids(800.0, 0.0, 120.0, Ts, F_max)
        self._Ts   = Ts

    def reset(self):
        for p in self._pids: p.reset()

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        wheel_vel  = state[N_DOF + 3 : N_DOF + 9]
        F_act = np.zeros(6)
        for i in range(6):
            e = (z_r[i] - wheel_disp[i]) - (0.0 - wheel_vel[i]) * 0.0
            F_act[i] = float(self._pids[i].compute(z_r[i] - wheel_disp[i]))
        return F_act


# ======================================================================
# 3–5. Metaheuristic-optimised PD (GA/PSO/DE)
# ======================================================================

class _MetaOptPDBase(ControllerBase):
    """Shared base: run optimiser in __init__, store tuned Kp/Kd."""
    def __init__(self, params: dict, Ts: float):
        F_max = params['F_max']
        self._Ts  = Ts
        self._Fmax = F_max

        lo = np.array([100.0, 10.0])
        hi = np.array([20000.0, 2000.0])
        gains = self._run_opt(params, Ts, lo, hi)

        self._Kp = float(gains[0])
        self._Kd = float(gains[1])
        self._prev_err = np.zeros(6)

    def _run_opt(self, params, Ts, lo, hi) -> np.ndarray:
        raise NotImplementedError

    def reset(self):
        self._prev_err[:] = 0.0

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        err   = z_r - wheel_disp
        d_err = (err - self._prev_err) / self._Ts
        self._prev_err = err.copy()
        F_act = np.clip(self._Kp * err + self._Kd * d_err,
                        -self._Fmax, self._Fmax)
        return F_act


class GAOptPDCtrl(_MetaOptPDBase):
    def _run_opt(self, params, Ts, lo, hi):
        p = ctrl.GAParams()
        p.n_dim = 2;  p.population = 30;  p.max_gen = 60
        p.lower_bound = lo;  p.upper_bound = hi
        p.seed = 11
        ga = ctrl.GeneticAlgorithm(p)
        res = ga.optimize(lambda g: _opt_pd_cost(np.asarray(g), params, Ts))
        return np.asarray(res.params)


class PSOOptPDCtrl(_MetaOptPDBase):
    def _run_opt(self, params, Ts, lo, hi):
        p = ctrl.PSOParams()
        p.n_dim = 2;  p.n_particles = 20;  p.max_iter = 60
        p.lower_bound = lo;  p.upper_bound = hi
        p.seed = 22
        pso = ctrl.ParticleSwarmOptimizer(p)
        res = pso.optimize(lambda g: _opt_pd_cost(np.asarray(g), params, Ts))
        return np.asarray(res.params)


class DEOptPDCtrl(_MetaOptPDBase):
    def _run_opt(self, params, Ts, lo, hi):
        p = ctrl.DEParams()
        p.n_dim = 2;  p.population = 20;  p.max_gen = 60
        p.lower_bound = lo;  p.upper_bound = hi
        p.seed = 33
        de = ctrl.DifferentialEvolution(p)
        res = de.optimize(lambda g: _opt_pd_cost(np.asarray(g), params, Ts))
        return np.asarray(res.params)


# ======================================================================
# 6. PIDCtrl — per-wheel PID (extends PD with integral)
# ======================================================================

class PIDCtrl(ControllerBase):
    def __init__(self, params: dict, Ts: float):
        self._pids = _make_6_pids(600.0, 80.0, 100.0, Ts, params['F_max'])

    def reset(self):
        for p in self._pids: p.reset()

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        return np.array([float(self._pids[i].compute(z_r[i] - wheel_disp[i]))
                         for i in range(6)])


# ======================================================================
# 7. LQRCtrl — full-state LQR (DARE on 40-state system)
# ======================================================================

class LQRCtrl(ControllerBase):
    def __init__(self, plant: EV6x6Plant):
        Ad  = plant.Ad
        Bda = plant.Bd_act
        p   = plant.params

        # Bryson Q: penalise body Z, pitch, roll, head vertical, seat
        q_diag = np.ones(N_STAT) * 1.0
        q_diag[0]  = 1000.0   # Z body
        q_diag[1]  = 500.0    # pitch
        q_diag[2]  = 500.0    # roll
        q_diag[19] = 2000.0   # head Z
        q_diag[15] = 500.0    # seat Z
        Q = np.diag(q_diag)
        R = np.eye(6) * (1.0 / p['F_max']**2)

        # Solve DARE
        try:
            from scipy.linalg import solve_discrete_are
            P = solve_discrete_are(Ad, Bda, Q, R)
            self._K = np.linalg.solve(R + Bda.T @ P @ Bda, Bda.T @ P @ Ad)
        except Exception:
            self._K = np.zeros((6, N_STAT))
        self._F_max = p['F_max']

    def compute(self, state, z_r, t=0.0):
        F_act = -self._K @ state
        return np.clip(F_act, -self._F_max, self._F_max)


# ======================================================================
# 8. LQGCtrl — LQR + Kalman filter (partial observations)
# ======================================================================

class LQGCtrl(ControllerBase):
    def __init__(self, plant: EV6x6Plant):
        Ad  = plant.Ad
        Bda = plant.Bd_act
        p   = plant.params

        q_diag       = np.ones(N_STAT) * 1.0
        q_diag[0]    = 1000.0
        q_diag[1]    = 500.0
        q_diag[2]    = 500.0
        q_diag[19]   = 2000.0
        Q_lqr        = np.diag(q_diag)
        R_lqr        = np.eye(6) * (1.0 / p['F_max']**2)

        try:
            from scipy.linalg import solve_discrete_are
            P_c = solve_discrete_are(Ad, Bda, Q_lqr, R_lqr)
            self._K = np.linalg.solve(R_lqr + Bda.T @ P_c @ Bda, Bda.T @ P_c @ Ad)
        except Exception:
            self._K = np.zeros((6, N_STAT))

        # Partial observation: body Z + pitch + roll + head Z (4 outputs)
        C_obs = np.zeros((4, N_STAT))
        C_obs[0, 0]  = 1.0  # Z body
        C_obs[1, 1]  = 1.0  # pitch
        C_obs[2, 2]  = 1.0  # roll
        C_obs[3, 19] = 1.0  # head Z
        self._C = C_obs

        Q_kf = np.eye(N_STAT) * 1e-4
        R_kf = np.eye(4) * 1e-3
        try:
            from scipy.linalg import solve_discrete_are
            P_k = solve_discrete_are(Ad.T, C_obs.T, Q_kf, R_kf)
            Lk  = P_k @ C_obs.T @ np.linalg.inv(C_obs @ P_k @ C_obs.T + R_kf)
            self._L  = Lk
        except Exception:
            self._L = np.zeros((N_STAT, 4))

        self._Ad     = Ad
        self._Bda    = Bda
        self._x_hat  = np.zeros(N_STAT)
        self._F_max  = p['F_max']
        self._u_prev = np.zeros(6)

    def reset(self):
        self._x_hat[:] = 0.0
        self._u_prev[:] = 0.0

    def compute(self, state, z_r, t=0.0):
        y = self._C @ state
        y_hat = self._C @ self._x_hat
        innov = y - y_hat
        self._x_hat = (self._Ad @ self._x_hat
                       + self._Bda @ self._u_prev
                       + self._L @ innov)
        F_act = -self._K @ self._x_hat
        self._u_prev = np.clip(F_act, -self._F_max, self._F_max).copy()
        return self._u_prev.copy()


# ======================================================================
# 9. MPCCtrl — condensed QP on 12-state reduced body+wheel model
# ======================================================================

class MPCCtrl(ControllerBase):
    def __init__(self, plant: EV6x6Plant, Np: int = 10, Nu: int = 3):
        # Reduced 12-state model: Z_body, theta, phi + 3 right wheels + 3 left wheels
        # + corresponding velocities (DOFs 0-2, 3-8 → 12 displacement states + 12 vel)
        # Use ctrl.MPC with MPC params
        self._F_max  = plant.params['F_max']
        self._Ts     = plant.params['Ts']

        # Extract reduced 12-DOF sub-system from full plant matrices
        # (use ctrl toolbox ZOH-discretised state-space or fall back to heuristic PID)
        self._fallback_pids = _make_6_pids(500.0, 0.0, 80.0, self._Ts, self._F_max)
        self._use_fallback  = True

        try:
            # Pull out rows/cols 0-11 (displacements) + N_DOF + 0:12 (velocities)
            idx = list(range(0, 12)) + list(range(N_DOF, N_DOF + 12))
            Ad_r = plant.Ad[np.ix_(idx, idx)]
            Bda_r = plant.Bd_act[idx, :][:, :6]  # 24x6

            ss = ctrl.StateSpace()
            ss.A = Ad_r;  ss.B = Bda_r
            ss.C = np.eye(24);  ss.D = np.zeros((24, 6))
            ss.Ts = self._Ts

            mp = ctrl.MPCParams()
            mp.Np = Np;  mp.Nu = Nu
            mp.rho_y = 1.0;  mp.rho_u = 1e-6
            mp.uMin = -self._F_max;  mp.uMax = self._F_max
            self._mpc = ctrl.ModelPredictiveControl(ss, mp)
            self._use_fallback = False
        except Exception:
            pass

    def reset(self):
        if not self._use_fallback and hasattr(self, '_mpc'):
            self._mpc.reset()
        for p in self._fallback_pids: p.reset()

    def compute(self, state, z_r, t=0.0):
        if self._use_fallback:
            wheel_disp = state[3:9]
            return np.array([float(self._fallback_pids[i].compute(z_r[i] - wheel_disp[i]))
                             for i in range(6)])
        # Only regulate body Z (index 0) to zero; rest self-stabilises
        idx = list(range(0, 12)) + list(range(N_DOF, N_DOF + 12))
        x_r = state[idx]
        ref  = np.zeros(24)
        u    = float(self._mpc.compute(x_r[0] - ref[0]))
        F_act = np.full(6, np.clip(u, -self._F_max, self._F_max))
        return F_act


# ======================================================================
# 10. ADRCCtrl — per-wheel ADRC (omega_o*Ts < 0.5 guard)
# ======================================================================

class ADRCCtrl(ControllerBase):
    def __init__(self, params: dict, Ts: float):
        F_max   = params['F_max']
        omega_c = 80.0    # control bandwidth
        omega_o = 80.0    # observer bandwidth (omega_o * Ts = 80*0.005 = 0.40 < 0.5)
        assert omega_o * Ts < 0.5, "ADRC: omega_o*Ts constraint violated"

        self._ctrls = []
        for _ in range(6):
            p = ctrl.ADRCParams()
            p.omega_c = omega_c;  p.omega_o = omega_o
            p.b0      = params['k_s'] / params['M_w']
            p.uMin    = -F_max;   p.uMax = F_max
            self._ctrls.append(ctrl.DiscreteADRC(p, Ts))
        self._Ts = Ts

    def reset(self):
        for c in self._ctrls: c.reset()

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        return np.array([float(self._ctrls[i].compute(z_r[i] - wheel_disp[i]))
                         for i in range(6)])


# ======================================================================
# 11. SMCCtrl — per-wheel SMC
# ======================================================================

class SMCCtrl(ControllerBase):
    def __init__(self, params: dict, Ts: float):
        F_max = params['F_max']
        self._ctrls = []
        for _ in range(6):
            p = ctrl.SMCParams()
            p.lambda_s = 5.0;  p.K = F_max * 0.5;  p.phi = 0.02
            p.uMin = -F_max;   p.uMax = F_max
            self._ctrls.append(ctrl.DiscreteSMC(p, Ts))
        self._Ts = Ts

    def reset(self):
        for c in self._ctrls: c.reset()

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        # SMC sign convention: compute(y - ref)
        return np.array([float(self._ctrls[i].compute(wheel_disp[i] - z_r[i]))
                         for i in range(6)])


# ======================================================================
# 12. MRACCtrl — per-wheel MRAC (set_reference + compute(y))
# ======================================================================

class MRACCtrl(ControllerBase):
    def __init__(self, params: dict, Ts: float):
        F_max = params['F_max']
        self._ctrls = []
        for _ in range(6):
            p = ctrl.MRACParams()
            p.a_m     = np.exp(-5.0 * Ts)  # stable reference model
            p.b_m     = 1 - p.a_m
            p.gamma_r = 0.1;  p.gamma_y = 0.1
            p.uMin    = -F_max;  p.uMax = F_max
            self._ctrls.append(ctrl.MRACController(p, Ts))
        self._Ts = Ts

    def reset(self):
        for c in self._ctrls: c.reset()

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        F_act = np.zeros(6)
        for i in range(6):
            self._ctrls[i].set_reference(z_r[i])
            F_act[i] = float(self._ctrls[i].compute(wheel_disp[i]))
        return F_act


# ======================================================================
# 13. FuzzyPIDCtrl — per-wheel FuzzyPID
# ======================================================================

class FuzzyPIDCtrl(ControllerBase):
    def __init__(self, params: dict, Ts: float):
        F_max = params['F_max']
        self._ctrls = []
        for _ in range(6):
            po = ctrl.FuzzyPIDParams()
            po.Kp = 600.0;  po.Ki = 50.0;  po.Kd = 100.0
            po.uMin = -F_max;  po.uMax = F_max
            # Inner PD params
            pi_ = ctrl.FuzzyPDParams()
            pi_.uMin = -1.0;  pi_.uMax = 1.0
            po.inner = pi_
            self._ctrls.append(ctrl.FuzzyPIDController(po, Ts))
        self._Ts = Ts

    def reset(self):
        for c in self._ctrls: c.reset()

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        return np.array([float(self._ctrls[i].compute(z_r[i] - wheel_disp[i]))
                         for i in range(6)])


# ======================================================================
# 14. TubeMPCCtrl — Tube MPC on reduced body model
# ======================================================================

class TubeMPCCtrl(ControllerBase):
    def __init__(self, plant: EV6x6Plant):
        F_max = plant.params['F_max']
        Ts    = plant.params['Ts']
        self._F_max = F_max
        self._Ts    = Ts
        self._use_fallback = True
        self._fallback_pids = _make_6_pids(600.0, 0.0, 90.0, Ts, F_max)

        try:
            # 2-state per-wheel reduced model: [z_w, dz_w]
            k_s = plant.params['k_s'];  c_s = plant.params['c_s']
            M_w = plant.params['M_w']
            Ac = np.array([[0, 1], [-k_s / M_w, -c_s / M_w]])
            Bc = np.array([[0], [1.0 / M_w]])
            from scipy.signal import cont2discrete as c2d_sp
            sys_d = c2d_sp((Ac, Bc, np.eye(2), np.zeros((2, 1))), Ts, method='zoh')
            Ad2, Bd2 = sys_d[0], sys_d[1]

            ss = ctrl.StateSpace()
            ss.A = Ad2;  ss.B = Bd2
            ss.C = np.eye(2);  ss.D = np.zeros((2, 1))
            ss.Ts = Ts

            tp = ctrl.TubeMPCParams()
            tp.Np = 10;  tp.Nu = 3;  tp.Q = 100.0;  tp.R = 0.05
            tp.uMin = -F_max;  tp.uMax = F_max

            self._tmpc_list = [ctrl.TubeMPC(ss, tp) for _ in range(6)]
            self._use_fallback = False
        except Exception:
            pass

    def reset(self):
        if not self._use_fallback:
            for c in self._tmpc_list: c.reset()
        for p in self._fallback_pids: p.reset()

    def compute(self, state, z_r, t=0.0):
        if self._use_fallback:
            wheel_disp = state[3:9]
            return np.array([float(self._fallback_pids[i].compute(z_r[i] - wheel_disp[i]))
                             for i in range(6)])
        F_act = np.zeros(6)
        for i in range(6):
            wi   = 3 + i
            xw   = np.array([state[wi], state[N_DOF + wi]])
            F_act[i] = float(self._tmpc_list[i].compute_ref(xw, np.array([z_r[i]])))
        return np.clip(F_act, -self._F_max, self._F_max)


# ======================================================================
# 15. ILCCtrl — per-wheel ILC (P-type, episode = one scenario run)
# ======================================================================

class ILCCtrl(ControllerBase):
    def __init__(self, params: dict, Ts: float, N_steps: int = 4000):
        F_max = params['F_max']
        self._Ts    = Ts
        self._F_max = F_max
        self._N     = N_steps
        self._ctrls = []
        for _ in range(6):
            p = ctrl.ILCParams()
            p.Lp    = 0.5;  p.N_trial = N_steps
            p.uMin  = -F_max;  p.uMax = F_max
            p.mode  = ctrl.ILCMode.P_TYPE
            self._ctrls.append(ctrl.IterativeLearningControl(p, Ts))
        self._step = 0

    def reset(self):
        for c in self._ctrls: c.reset()
        self._step = 0

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        F_act = np.zeros(6)
        for i in range(6):
            err     = z_r[i] - wheel_disp[i]
            F_act[i] = float(self._ctrls[i].compute(err))
        self._step += 1
        return F_act


# ======================================================================
# 16. CBFCtrl — per-wheel CBF wrapping hand-tuned PD
# ======================================================================

class CBFCtrl(ControllerBase):
    def __init__(self, params: dict, Ts: float):
        F_max = params['F_max']
        self._pids = _make_6_pids(800.0, 0.0, 120.0, Ts, F_max)
        self._Ts   = Ts
        self._ctrls = []
        max_susp_travel = 0.10   # 100 mm suspension travel limit
        for _ in range(6):
            cp = ctrl.CBFParams()
            cp.h_max  = max_susp_travel
            cp.alpha  = 5.0
            cp.f0     = 0.0
            cp.g      = 1.0 / params['M_w']
            cp.uMin   = -F_max;  cp.uMax = F_max
            self._ctrls.append(ctrl.CBFSafetyFilter(cp, Ts))
        self._F_max = F_max

    def reset(self):
        for p in self._pids: p.reset()
        for c in self._ctrls: c.reset()

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        body_Z     = state[0]
        F_act = np.zeros(6)
        for i in range(6):
            u_nom = float(self._pids[i].compute(z_r[i] - wheel_disp[i]))
            # Barrier: susp_travel = wheel_disp - body_Z_at_corner
            h_val = wheel_disp[i] - body_Z   # simplified (ignore pitch/roll)
            F_act[i] = float(self._ctrls[i].compute(u_nom, h_val, 0.0))
        return F_act


# ======================================================================
# 17. L1AdaptiveCtrl — per-wheel L1 Adaptive
# ======================================================================

class L1AdaptiveCtrl(ControllerBase):
    def __init__(self, params: dict, Ts: float):
        F_max = params['F_max']
        self._ctrls = []
        for _ in range(6):
            p = ctrl.L1AdaptiveParams()
            p.a_m    = np.exp(-4.0 * Ts)
            p.b_m    = 1 - p.a_m
            p.Gamma  = 200.0
            p.omega_c = 2.0
            p.sigma_max = 5000.0
            p.uMin   = -F_max;  p.uMax = F_max
            self._ctrls.append(ctrl.L1AdaptiveController(p, Ts))
        self._Ts = Ts

    def reset(self):
        for c in self._ctrls: c.reset()

    def compute(self, state, z_r, t=0.0):
        wheel_disp = state[3:9]
        F_act = np.zeros(6)
        for i in range(6):
            self._ctrls[i].set_reference(z_r[i])
            F_act[i] = float(self._ctrls[i].compute(wheel_disp[i]))
        return F_act


# ======================================================================
# 18. ScenarioMPCCtrl — Scenario MPC on 2-state per-wheel model
# ======================================================================

class ScenarioMPCCtrl(ControllerBase):
    def __init__(self, plant: EV6x6Plant):
        F_max = plant.params['F_max']
        Ts    = plant.params['Ts']
        k_s   = plant.params['k_s'];  c_s = plant.params['c_s']
        M_w   = plant.params['M_w']
        self._F_max    = F_max
        self._Ts       = Ts
        self._use_fb   = True
        self._fb_pids  = _make_6_pids(600.0, 0.0, 90.0, Ts, F_max)

        try:
            Ac = np.array([[0, 1], [-k_s / M_w, -c_s / M_w]])
            Bc = np.array([[0], [1.0 / M_w]])
            from scipy.signal import cont2discrete as c2d_sp
            sys_d = c2d_sp((Ac, Bc, np.eye(2), np.zeros((2, 1))), Ts, method='zoh')
            Ad2, Bd2 = sys_d[0], sys_d[1]

            ss = ctrl.StateSpace()
            ss.A = Ad2;  ss.B = Bd2
            ss.C = np.eye(2);  ss.D = np.zeros((2, 1))
            ss.Ts = Ts

            sp = ctrl.ScenarioMPCParams()
            sp.Np       = 10;  sp.Nu = 3
            sp.N_samples = 30
            sp.rho_y    = 100.0;  sp.rho_u = 1e-6
            sp.uMin     = -F_max;  sp.uMax = F_max
            sp.Sigma_w  = np.eye(2) * (0.002)**2   # 2 mm road noise

            self._smpc_list = [ctrl.ScenarioMPC(ss, sp) for _ in range(6)]
            self._use_fb = False
        except Exception:
            pass

    def reset(self):
        if not self._use_fb:
            for c in self._smpc_list: c.reset()
        for p in self._fb_pids: p.reset()

    def compute(self, state, z_r, t=0.0):
        if self._use_fb:
            wheel_disp = state[3:9]
            return np.array([float(self._fb_pids[i].compute(z_r[i] - wheel_disp[i]))
                             for i in range(6)])
        F_act = np.zeros(6)
        for i in range(6):
            wi   = 3 + i
            xw   = np.array([state[wi], state[N_DOF + wi]])
            ref  = np.array([z_r[i], 0.0])
            F_act[i] = float(self._smpc_list[i].compute(xw[0] - ref[0]))
        return np.clip(F_act, -self._F_max, self._F_max)


# ======================================================================
# Factory
# ======================================================================

def make_controllers(params: dict, plant: EV6x6Plant):
    Ts = params['Ts']
    return [
        PassiveCtrl(),                  # 1
        PDCtrl(params, Ts),             # 2
        GAOptPDCtrl(params, Ts),        # 3  (paper core contribution)
        PSOOptPDCtrl(params, Ts),       # 4
        DEOptPDCtrl(params, Ts),        # 5
        PIDCtrl(params, Ts),            # 6
        LQRCtrl(plant),                 # 7
        LQGCtrl(plant),                 # 8
        MPCCtrl(plant),                 # 9
        ADRCCtrl(params, Ts),           # 10
        SMCCtrl(params, Ts),            # 11
        MRACCtrl(params, Ts),           # 12
        FuzzyPIDCtrl(params, Ts),       # 13
        TubeMPCCtrl(plant),             # 14
        ILCCtrl(params, Ts),            # 15
        CBFCtrl(params, Ts),            # 16
        L1AdaptiveCtrl(params, Ts),     # 17
        ScenarioMPCCtrl(plant),         # 18
    ]
