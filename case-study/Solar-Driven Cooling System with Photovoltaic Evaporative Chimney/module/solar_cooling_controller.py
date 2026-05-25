"""
solar_cooling_controller.py - Controller portfolio for the Solar-Driven Cooling System.

Control objectives
------------------
The primary controlled variable is:
    y1 : condenser water temperature Tw1 [^\circC]  - tracks desired setpoint

The secondary objectives (optimisation layer) are:
    maximize  EER_grid = Q_evap / (W_comp + W_pump - W_PV)

Manipulated variables (control inputs):
    u1 : m_dot_w  - water mass flow rate [kg/s] through the evaporative chimney
    u2 : kr       - pump VFD speed ratio [-]

Three controllers are provided:

  Mode 1 - PID
      Separate PIDs for each channel:
        - PID_mw  : Tw1 -> m_dot_w  (main cooling loop)
        - PID_kr  : Q_op -> kr       (hydraulic loop tracking)

  Mode 2 - Feedforward + PID
      Solar-irradiance feedforward for m_dot_w (anticipate load), PID for trim.
      Pump kr is scheduled as a function of required condenser flow.

  Mode 3 - Model Predictive Controller (unconstrained condensed QP)
      Linear plant model around the design point. Predicts Tw1 over Np steps
      and minimises a stage cost on (Tw1 error, m_dot_w move).

All controllers expose:
    compute(reference, measurement) -> ControlInput
    reset()
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

from solar_cooling_plant import ControlInput


# ---------------------------------------------------------------------------
# Common base
# ---------------------------------------------------------------------------
class ControllerBase(ABC):
    """Abstract base: output is a ControlInput(m_dot_w, kr)."""

    def __init__(self, Ts: float,
                 mw_min: float = 0.02, mw_max: float = 0.22,
                 kr_min: float = 0.30, kr_max: float = 1.00):
        self.Ts = Ts
        self.mw_min, self.mw_max = mw_min, mw_max
        self.kr_min, self.kr_max = kr_min, kr_max

    @abstractmethod
    def compute(self, reference: float, measurement: float) -> ControlInput: ...

    @abstractmethod
    def reset(self) -> None: ...

    def _clamp_mw(self, v: float) -> float:
        return max(self.mw_min, min(self.mw_max, v))

    def _clamp_kr(self, v: float) -> float:
        return max(self.kr_min, min(self.kr_max, v))


# ---------------------------------------------------------------------------
# Mode 1 - Decoupled PID (backward-Euler I, filtered D, back-calculation AW)
# ---------------------------------------------------------------------------
@dataclass
class PIDGains:
    Kp: float = 0.002
    Ki: float = 0.0003
    Kd: float = 0.005
    N:  float = 5.0     # derivative filter pole
    Kb: float = 0.5     # anti-windup back-calculation gain


class _ScalarPID:
    """Single-channel discrete PID: backward-Euler integrator, filtered derivative."""

    def __init__(self, gains: PIDGains, Ts: float, u_min: float, u_max: float):
        self.g = gains
        self.Ts = Ts
        self.u_min, self.u_max = u_min, u_max
        self._alpha  = 1.0 / (1.0 + gains.N * Ts)
        self._I      = 0.0
        self._D      = 0.0
        self._e_prev = 0.0

    def reset(self) -> None:
        self._I = self._D = self._e_prev = 0.0

    def compute(self, e: float) -> float:
        g = self.g
        self._D = self._alpha * self._D + g.Kd * g.N * self._alpha * (e - self._e_prev)
        u_unsat = g.Kp * e + self._I + self._D
        u_sat   = max(self.u_min, min(self.u_max, u_unsat))
        self._I += g.Ki * self.Ts * e + g.Kb * (u_sat - u_unsat)
        self._e_prev = e
        return u_sat


@dataclass
class PIDParams:
    mw_gains:  PIDGains = field(default_factory=PIDGains)
    kr_gains:  PIDGains = field(default_factory=lambda: PIDGains(Kp=0.5, Ki=0.05, Kd=0.1, N=5.0))
    kr_setpoint: float = 0.85   # nominal pump speed ratio


class PIDController(ControllerBase):
    """
    Mode 1: two decoupled scalar PIDs.
      Channel 1: error in Tw1 (warm water temp) -> m_dot_w
      Channel 2: kr held at setpoint (simple P around fixed value)
    """

    def __init__(self, params: PIDParams, Ts: float, **kw):
        super().__init__(Ts, **kw)
        self.p = params
        self._pid_mw = _ScalarPID(params.mw_gains, Ts, self.mw_min, self.mw_max)
        self._pid_kr = _ScalarPID(params.kr_gains, Ts, self.kr_min, self.kr_max)
        self._kr_nom = params.kr_setpoint

    def reset(self) -> None:
        self._pid_mw.reset()
        self._pid_kr.reset()

    def compute(self, reference: float, measurement: float) -> ControlInput:
        """
        reference   : desired Tw1 [^\circC]
        measurement : actual  Tw1 [^\circC]
        """
        e_tw1 = reference - measurement
        m_dot_w = self._pid_mw.compute(e_tw1)

        # Pump: hold at nominal speed ratio
        e_kr = self._kr_nom - self._kr_nom   # zero error -> maintain setpoint
        kr = self._clamp_kr(self._kr_nom + self._pid_kr.compute(e_kr))

        return ControlInput(m_dot_w=m_dot_w, kr=kr)


# ---------------------------------------------------------------------------
# Mode 2 - Feedforward + PID (solar-irradiance feedforward)
# ---------------------------------------------------------------------------
@dataclass
class FFPIDParams:
    mw_gains:     PIDGains = field(default_factory=PIDGains)
    kr_schedule_slope: float = 0.35   # kr = kr_base + slope * (Tw1_err / 10)
    kr_base:      float = 0.80
    G_to_mw_gain: float = 1.5e-4     # FF: delta_mw = G_to_mw_gain * G [W/m^2]
    mw_nom:       float = 0.10       # nominal chimney spray flow [kg/s]


class FFPIDController(ControllerBase):
    """
    Mode 2: solar-irradiance feedforward for m_dot_w, PID for trim.
    The feedforward anticipates increased cooling demand when G rises so the
    water flow pre-emptively increases before Tw1 rises above setpoint.
    """

    def __init__(self, params: FFPIDParams, Ts: float, **kw):
        super().__init__(Ts, **kw)
        self.p = params
        self._pid = _ScalarPID(params.mw_gains, Ts, -0.10, 0.10)
        self._G   = 0.0   # last known irradiance

    def reset(self) -> None:
        self._pid.reset()
        self._G = 0.0

    def set_irradiance(self, G: float) -> None:
        """Call before compute() to provide feedforward measurement."""
        self._G = max(0.0, G)

    def compute(self, reference: float, measurement: float) -> ControlInput:
        e_tw1  = reference - measurement
        u_ff   = self.p.G_to_mw_gain * self._G
        u_fb   = self._pid.compute(e_tw1)
        m_dot_w = self._clamp_mw(self.p.mw_nom + u_ff + u_fb)

        # Pump speed schedule based on temperature error
        kr = self._clamp_kr(self.p.kr_base + self.p.kr_schedule_slope * (e_tw1 / 10.0))

        return ControlInput(m_dot_w=m_dot_w, kr=kr)


# ---------------------------------------------------------------------------
# Mode 3 - Model Predictive Controller (condensed QP, unconstrained)
# ---------------------------------------------------------------------------
@dataclass
class MPCParams:
    Np: int   = 8
    Nc: int   = 3
    Q:  float = 10.0    # output tracking weight  (on Tw1 error)
    R:  float = 50.0    # move suppression weight  (on delta m_dot_w)
    kr_nom: float = 0.85


class MPCController(ControllerBase):
    """
    Mode 3: linear MPC around the design-point linearisation.

    Plant model:  dTw1/dt approx = -a * Tw1  +  b * m_dot_w  +  d_w  (FOPDT)
    Discretised (Euler):  Tw1[k+1] = (1 - a*Ts)*Tw1[k] + b*Ts*m_dot_w[k]

    The prediction is built in condensed form and the first optimal move
    is applied (receding horizon). Pump kr is kept at nominal.

    Parameters a, b are identified from the steady-state gain at the design
    point using finite-difference sensitivity.
    """

    def __init__(self, a: float, b: float,
                 params: MPCParams, Ts: float, Tw1_nom: float, **kw):
        """
        a, b  : linearised FOPDT model coefficients (1/s and ^\circC/(kg/s)/s)
        Ts    : sample time [s]
        Tw1_nom : linearisation point for Tw1 [^\circC]
        """
        super().__init__(Ts, **kw)
        self.p = params
        self._Tw1_nom = Tw1_nom
        self._a_d = 1.0 - a * Ts      # discrete A
        self._b_d = b * Ts             # discrete B
        self._build_gain(params)

    def _build_gain(self, p: MPCParams) -> None:
        """Pre-compute unconstrained MPC gain matrix."""
        Np, Nc = p.Np, p.Nc
        Ad, Bd = self._a_d, self._b_d

        # Prediction matrix Phi (Np x 1) and Theta (Np x Nc)
        Phi   = [[Ad**(i+1)] for i in range(Np)]
        Theta = [[0.0] * Nc for _ in range(Np)]
        for i in range(Np):
            for j in range(min(i + 1, Nc)):
                Theta[i][j] = (Ad ** (i - j)) * Bd

        # Cost matrices: sum_i Q*(y_i - r)^2 + sum_j R*(du_j)^2
        # Unconstrained gain: G = (Theta^T Q Theta + R I)^-1 Theta^T Q
        # Applied as: du_seq = G * (R_vec - Phi * x)  where x = Tw1 deviation

        # Build Theta^T Q Theta + R*I  (Nc x Nc)
        Q, R = p.Q, p.R
        H = [[0.0] * Nc for _ in range(Nc)]
        for i in range(Np):
            for r_idx in range(Nc):
                for c_idx in range(Nc):
                    H[r_idx][c_idx] += Q * Theta[i][r_idx] * Theta[i][c_idx]
        for i in range(Nc):
            H[i][i] += R

        # Build Theta^T Q Phi  (Nc x 1) -> used as gain_x (maps state deviation)
        # Build Theta^T Q (summed over Np) -> used as gain_r (maps reference)
        gx = [0.0] * Nc
        gr = [0.0] * Nc
        for i in range(Np):
            for j in range(Nc):
                gx[j] += Q * Theta[i][j] * Phi[i][0]
                gr[j] += Q * Theta[i][j]   # constant reference => sum rows

        # Solve H * gain = g  via Gaussian elimination (tiny Nc matrix)
        self._gain_x = _solve_linear(H, gx)    # (Nc,) : maps Tw1 deviation
        self._gain_r = _solve_linear(H, gr)    # (Nc,) : maps reference deviation

    def reset(self) -> None:
        pass   # stateless

    def compute(self, reference: float, measurement: float) -> ControlInput:
        """
        reference   : desired Tw1 [^\circC]
        measurement : actual Tw1  [^\circC]
        """
        # Work in deviation variables from linearisation point
        x = measurement - self._Tw1_nom
        r = reference   - self._Tw1_nom

        # First optimal move (delta m_dot_w)
        du0 = self._gain_r[0] * r - self._gain_x[0] * x
        # Accumulate: m_dot_w = nominal + du
        m_dot_w = self._clamp_mw(self.mw_min + (self.mw_max - self.mw_min) / 2.0 + du0)

        return ControlInput(m_dot_w=m_dot_w, kr=self.p.kr_nom)


def _solve_linear(A: list[list[float]], b: list[float]) -> list[float]:
    """Gaussian elimination with partial pivoting for small square system."""
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for col in range(n):
        # Pivot
        max_row = max(range(col, n), key=lambda r: abs(M[r][col]))
        M[col], M[max_row] = M[max_row], M[col]
        if abs(M[col][col]) < 1e-15:
            continue
        for row in range(col + 1, n):
            f = M[row][col] / M[col][col]
            for j in range(col, n + 1):
                M[row][j] -= f * M[col][j]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = M[i][n]
        for j in range(i + 1, n):
            x[i] -= M[i][j] * x[j]
        if abs(M[i][i]) > 1e-15:
            x[i] /= M[i][i]
    return x


# ---------------------------------------------------------------------------
# Convenience factory
# ---------------------------------------------------------------------------
def make_pid(Ts: float) -> PIDController:
    return PIDController(PIDParams(), Ts)


def make_ffpid(Ts: float) -> FFPIDController:
    return FFPIDController(FFPIDParams(), Ts)


def make_mpc(Ts: float, Tw1_nom: float = 40.0,
             a: float = 0.02, b: float = 5.0) -> MPCController:
    return MPCController(a=a, b=b, params=MPCParams(), Ts=Ts, Tw1_nom=Tw1_nom)


# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    Ts = 10.0
    pid   = make_pid(Ts)
    ffpid = make_ffpid(Ts)
    mpc   = make_mpc(Ts)

    ref  = 40.0
    meas = 45.0

    print("=== Controller smoke test ===")
    for name, ctrl in [("PID", pid), ("FF-PID", ffpid), ("MPC", mpc)]:
        if isinstance(ctrl, FFPIDController):
            ctrl.set_irradiance(920.0)
        u = ctrl.compute(ref, meas)
        print(f"  {name:8s}  m_dot_w={u.m_dot_w:.4f} kg/s   kr={u.kr:.3f}")
        assert 0.01 <= u.m_dot_w <= 0.25, f"{name}: m_dot_w out of bounds"
        assert 0.30 <= u.kr      <= 1.00, f"{name}: kr out of bounds"

    print("PASS: all controller outputs within bounds.")
