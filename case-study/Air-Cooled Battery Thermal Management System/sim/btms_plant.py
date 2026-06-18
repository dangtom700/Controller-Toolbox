"""
btms_plant.py
Transient heat transfer plant model for the Air-Cooled BTMS J-U-L case study.

Models N=9 prismatic battery cells in a parallel air-cooled pack with three
switchable flow patterns (J-type, U-type, L-type).  Equations follow Zhang et al.
(2026), Applied Thermal Engineering 298, 130921 (Eqs. 13-16).

State vector (19 elements):
  [0:9]   Tb[i]        battery cell temperatures [K]
  [9:19]  Ta_out[j]    air outlet temperature of parallel channel j [K]

Integration: forward Euler at Ts=1.0 s.  Air-channel dynamics are quasi-static
(tau_air ~ 3 ms << Ts), so Ta_out is solved algebraically each step via
fixed-point iteration to avoid numerical instability.
"""

import math


# ---------------------------------------------------------------------------
# Physical constants / geometry
# ---------------------------------------------------------------------------
N_CELLS = 9        # batteries in x-direction
N_CH    = N_CELLS + 1  # parallel channels (10)

# Battery cell (16 mm x 151 mm x 65 mm)
W_BAT   = 0.016    # battery width in x-direction [m]
L_BAT   = 0.151    # depth into 2D plane [m]
H_BAT   = 0.065    # height = flow-direction channel length [m]
V_BAT   = W_BAT * L_BAT * H_BAT   # 1.578e-4 m^3

# Channel widths [m] -- 2 end channels 2.5 mm, 8 interior 3.1 mm
W_CH = [2.5e-3, 3.1e-3, 3.1e-3, 3.1e-3, 3.1e-3,
        3.1e-3, 3.1e-3, 3.1e-3, 3.1e-3, 2.5e-3]

# Air channel volume [m^3]: w_ch * H_bat * L_bat
V_CH = [W_CH[j] * H_BAT * L_BAT for j in range(N_CH)]

# Heat transfer area per battery-channel interface [m^2]
S_HX = H_BAT * L_BAT   # 0.065 * 0.151 = 9.815e-3 m^2

# Air properties (at ~310 K, constant coefficients)
RHO_A = 1.165    # kg/m^3
CP_A  = 1005.0   # J/(kg*K)
MU_A  = 1.86e-5  # Pa*s (not used directly but kept for reference)

# Battery thermal properties
RHO_B  = 1542.9  # kg/m^3
CP_B   = 1337.0  # J/(kg*K)
M_B    = RHO_B * V_BAT  # thermal mass numerator

# Heat generation model: phi_b = (I^2*R - I*Tb*dUe_dT) / V_b
DUE_DT = -0.22e-3   # V/K (entropic coefficient)
I_1C   = 7.0        # A  (7 Ah cell, 5C = 35 A)

# Electrical resistance vs SOC (Eq. 2 from paper)
def _resistance(soc: float) -> float:
    s = max(0.0, min(1.0, soc))
    return (0.00705
            - 0.01853 * s
            + 0.05894 * s**2
            - 0.09151 * s**3
            + 0.06579 * s**4
            - 0.01707 * s**5)


# ---------------------------------------------------------------------------
# Flow-pattern flow distributions
# ---------------------------------------------------------------------------
# Normalized channel flow weights for each pattern.
# J-type: exit on right  -> high flow right
# U-type: exit on left   -> high flow left
# L-type: exits both     -> high flow middle
_W_J = [0.40, 0.56, 0.68, 0.78, 0.88, 0.98, 1.08, 1.20, 1.36, 1.80]
_W_U = [1.80, 1.36, 1.20, 1.08, 0.98, 0.88, 0.78, 0.68, 0.56, 0.40]
_W_L = [0.60, 0.80, 1.00, 1.15, 1.25, 1.25, 1.15, 1.00, 0.80, 0.60]

def _normalize_flow(weights, Q_total):
    s = sum(weights)
    return [w / s * Q_total for w in weights]


# Minimum-flow channel h (from CFD, Table 3 / text of paper)
# J: ch0 (leftmost) h_ref=43.4; U: ch9 (rightmost) h_ref=34.2; L: ch0 h_ref=32.6
_H_REF = {'J': (43.4, 0), 'U': (34.2, 9), 'L': (32.6, 0)}


def _build_h_profile(pattern: str, Q_ch: list) -> list:
    """Scale h[j] from reference channel using turbulent Re scaling: h ~ Q^0.8."""
    h_ref, ref_idx = _H_REF[pattern]
    Q_ref = Q_ch[ref_idx]
    h = [h_ref * (Q_ch[j] / max(Q_ref, 1e-12)) ** 0.8 for j in range(N_CH)]
    return h


# ---------------------------------------------------------------------------
# LMTD helper (log-mean temperature difference, constant wall temperature)
# ---------------------------------------------------------------------------
def _lmtd(T_wall: float, T_in: float, T_out: float) -> float:
    """
    LMTD for one-side constant-temperature HX.
    Returns the effective driving temperature difference [K].
    """
    dT1 = T_wall - T_in
    dT2 = T_wall - T_out
    eps = 1e-8
    if dT1 < eps and dT2 < eps:
        return 0.0
    dT1 = max(dT1, eps)
    dT2 = max(dT2, eps)
    if abs(dT1 - dT2) < 1e-9:
        return (dT1 + dT2) * 0.5
    return (dT1 - dT2) / math.log(dT1 / dT2)


# ---------------------------------------------------------------------------
# BTMSPlant
# ---------------------------------------------------------------------------
class BTMSPlant:
    """
    Transient heat transfer model for BTMS J-U-L.

    Usage:
      plant = BTMSPlant(plant_params)
      plant.set_pattern('J')
      for k in range(N_steps):
          phi_b = plant.heat_gen(t, soc)
          plant.step(phi_b, Ts)
          DeltaT = plant.delta_T()
    """

    def __init__(self, p: dict):
        self.Ts      = float(p.get('Ts', 1.0))
        self.Q_in    = float(p.get('Q_in', 0.015))
        self.T_in    = float(p.get('T_in', 298.15))
        self.T_0     = float(p.get('T_0', 298.15))
        self.I_1C    = float(p.get('I_1C', 7.0))
        self._pattern = 'J'
        self._Tb     = [self.T_0] * N_CELLS
        self._Ta_out = [self.T_in] * N_CH
        self._n_switches = 0
        self._update_flow_params()

    # ------------------------------------------------------------------
    # Pattern management
    # ------------------------------------------------------------------
    def set_pattern(self, pattern: str):
        if pattern not in ('J', 'U', 'L'):
            raise ValueError(f"Unknown flow pattern: {pattern}")
        if pattern != self._pattern:
            self._n_switches += 1
        self._pattern = pattern
        self._update_flow_params()

    def _update_flow_params(self):
        if self._pattern == 'J':
            self._Q_ch = _normalize_flow(_W_J, self.Q_in)
        elif self._pattern == 'U':
            self._Q_ch = _normalize_flow(_W_U, self.Q_in)
        else:
            self._Q_ch = _normalize_flow(_W_L, self.Q_in)
        self._h_ch = _build_h_profile(self._pattern, self._Q_ch)

    # ------------------------------------------------------------------
    # Heat generation
    # ------------------------------------------------------------------
    def heat_gen(self, t: float, c_rate: float, phi_override: float = -1.0) -> float:
        """
        Heat generation rate per cell [W/m^3].
        phi_override: if > 0, use directly (for varying-condition scenarios).
        """
        if phi_override > 0.0:
            return phi_override
        soc = max(0.0, 1.0 - c_rate * t / 3600.0)
        R   = _resistance(soc)
        I   = c_rate * self.I_1C
        T_b_avg = sum(self._Tb) / N_CELLS
        phi = (I * I * R - I * T_b_avg * DUE_DT) / V_BAT
        return max(phi, 0.0)

    # ------------------------------------------------------------------
    # Quasi-static air temperature solver
    # ------------------------------------------------------------------
    def _solve_Ta_out(self, h: list, Q: list) -> list:
        """
        Solve Ta_out[j] quasi-statically (dTa/dt=0 in Eq.14) using
        fixed-point iteration.  Convergence is fast (3-5 iterations typical).
        """
        Ta = list(self._Ta_out)   # current as initial guess
        T_in = self.T_in
        Tb   = self._Tb

        for _ in range(30):
            Ta_new = list(Ta)
            for j in range(N_CH):
                # Left contribution: battery j is on the right of channel j
                dT_left = (_lmtd(Tb[j], T_in, Ta[j])
                            if j < N_CELLS else 0.0)
                # Right contribution: battery j-1 is on the left of channel j
                dT_right = (_lmtd(Tb[j - 1], T_in, Ta[j])
                             if j > 0 else 0.0)

                q_flux = h[j] * S_HX * (dT_left + dT_right)
                denom  = RHO_A * CP_A * max(Q[j], 1e-12)
                Ta_new[j] = T_in + q_flux / denom
                # Clamp: Ta_out cannot exceed max battery temperature
                Ta_new[j] = min(Ta_new[j], max(Tb) + 1.0)
                Ta_new[j] = max(Ta_new[j], T_in)

            # Check convergence
            err = max(abs(Ta_new[j] - Ta[j]) for j in range(N_CH))
            Ta = Ta_new
            if err < 1e-6:
                break
        return Ta

    # ------------------------------------------------------------------
    # Time-step integration (Euler)
    # ------------------------------------------------------------------
    def step(self, phi_b: float, Ts: float = None):
        """Advance the plant by one time step Ts [s]."""
        if Ts is None:
            Ts = self.Ts
        h  = self._h_ch
        Q  = self._Q_ch
        T_in = self.T_in
        Tb   = self._Tb

        # Solve air temperatures quasi-statically
        Ta = self._solve_Ta_out(h, Q)
        self._Ta_out = Ta

        # Battery ODE (Eq. 13)
        Tb_new = list(Tb)
        for i in range(N_CELLS):
            dT_left  = _lmtd(Tb[i], T_in, Ta[i])
            dT_right = _lmtd(Tb[i], T_in, Ta[i + 1])
            dTb_dt = (phi_b * V_BAT
                      - h[i]     * S_HX * dT_left
                      - h[i + 1] * S_HX * dT_right) / (M_B * CP_B)
            Tb_new[i] = Tb[i] + Ts * dTb_dt

        self._Tb = Tb_new

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------
    def temperatures(self):
        return list(self._Tb)

    def air_temperatures(self):
        return list(self._Ta_out)

    def delta_T(self) -> float:
        """Temperature difference of battery pack [K] (Eq. 3)."""
        return max(self._Tb) - min(self._Tb)

    def T_max(self) -> float:
        return max(self._Tb)

    def T_min(self) -> float:
        return min(self._Tb)

    def x_hotspot_norm(self) -> float:
        """Normalized x-position of hottest cell [0=left, 1=right]."""
        idx = self._Tb.index(max(self._Tb))
        return idx / max(N_CELLS - 1, 1)

    def pattern(self) -> str:
        return self._pattern

    def n_switches(self) -> int:
        return self._n_switches

    def reset(self, T_init: float = None):
        T0 = T_init if T_init is not None else self.T_0
        self._Tb      = [T0] * N_CELLS
        self._Ta_out  = [self.T_in] * N_CH
        self._n_switches = 0
        self._pattern  = 'J'
        self._update_flow_params()
