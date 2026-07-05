"""
pcm_thermal_energy_storage_control_plant.py
Control-oriented model of a Phase-Change-Material (PCM) thermal-energy-storage
buffer coupled to a variable-speed Heat Pump (HP) for building cooling, after
Chen, Marotta, Palomba, Ohlson Timoudas & Wang (2026), "Model predictive control
guided imitation learning for optimal control of PCM thermal energy storage",
Applied Thermal Engineering 295, 130741.

Architecture (paper Fig. 3 / Eq. 3): the PCM store is a buffer that serves the
building cooling load; the HP charges the store. The single controlled state is
the store State of Charge (SoC).

State:   SoC in [0, 1]           - PCM store state of charge (dimensionless)
Input:   u   in [0, 1]           - normalised HP compressor speed (r = 2900*u rpm)
Disturb: T_o  [degC]             - outdoor temperature (forecast)
         P_load [kW]             - building cooling load
         price  [-]              - normalised electricity price (for cost, not dynamics)

Reduced-order HP maps (paper Eqs. 1-2, quadratic polynomial regression fitted to
experimental data, R^2 ~ 0.9997):
    e_hp(r,T_o) = 1.5e-2 + 1.2e-4 r - 9.0e-3 T_o - 1.5e-7 r^2
                  + 2.3e-5 r T_o + 1.1e-3 T_o^2          [kWh electricity]
    Q_hp(r,T_o) = -2.07e-1 + 5.33e-3 r - 1.37e-1 T_o
                  - 7.0e-7 r^2 + 1.24e-3 T_o^2           [kW cooling]

SoC dynamics (charge = HP cooling minus building load, normalised by store
capacity, with the paper's charge/discharge hysteresis, 32 kWh charging /
27 kWh discharging):
    SoC[k+1] = SoC[k] + Ts * (Q_hp - P_load) / C,   C = C_charge if Q_hp >= P_load
                                                          else C_discharge

Model simplifications (documented per project convention, see README):
  1. The paper's headline controller is an MPC expert whose actions train two
     neural imitation-learning agents (Behavior Cloning, GAIL) against a
     high-fidelity Modelica/FMU co-simulation. That FMU is not distributed; this
     study uses the paper's own reduced-order HP maps (Eqs. 1-2) as the plant and
     a lumped-capacity SoC integrator - the same reduced model the paper's MPC
     expert itself uses internally. The imitation-learning agents are represented
     by the roster's neural controller (NeuralPID).
  2. The economic load-shifting objective (min sum price*e_hp) is realised in the
     tracking harness as a SoC reference schedule that pre-charges the store
     during low-price valleys (paper Fig. 14: twice-daily charging ~2-5 am and
     ~12-3 pm); controllers are compared on how well they realise that schedule
     (IAE on SoC) and on the resulting electricity cost (logged).
  3. Time step Ts = 1 h (paper's day-ahead-market resolution). SoC is clamped to
     [0, 1] (Eq. 3c); u to [0, 1] (Eq. 3g, r in [0, 2900] rpm).

Integrator: forward Euler at Ts = 1 h (matches the paper's hourly discretisation).
"""

import math

R_MAX = 2900.0   # max compressor speed [rpm] (paper Eq. 3g, N_max)


def hp_electricity(r: float, T_o: float) -> float:
    """e_hp(r, T_o) [kWh], paper Eq. 1 (clamped non-negative)."""
    e = (1.5e-2 + 1.2e-4 * r - 9.0e-3 * T_o - 1.5e-7 * r * r
         + 2.3e-5 * r * T_o + 1.1e-3 * T_o * T_o)
    return max(0.0, e)


def hp_cooling(r: float, T_o: float) -> float:
    """Q_hp(r, T_o) [kW], paper Eq. 2 (clamped non-negative)."""
    q = (-2.07e-1 + 5.33e-3 * r - 1.37e-1 * T_o
         - 7.0e-7 * r * r + 1.24e-3 * T_o * T_o)
    return max(0.0, q)


class PCMPlant:
    """PCM-HP cooling store: single state SoC in [0, 1]."""

    def __init__(self, params: dict):
        p = params
        self.Ts = p.get("Ts", 1.0)                    # [h]
        self.C_charge = p.get("C_charge", 32.0)        # [kWh]
        self.C_discharge = p.get("C_discharge", 27.0)  # [kWh]
        self.SoC0 = p.get("SoC0", 0.5)
        self._soc = self.SoC0

    def reset(self, soc0=None):
        self._soc = self.SoC0 if soc0 is None else float(soc0)

    def state(self):
        return self._soc

    def output(self) -> float:
        return self._soc

    def step(self, u: float, T_o: float, P_load: float):
        """Advance one Ts step. u = normalised compressor speed in [0,1].
        Returns (Q_hp, e_hp, r)."""
        u = max(0.0, min(1.0, u))
        r = R_MAX * u
        Q = hp_cooling(r, T_o)
        e = hp_electricity(r, T_o)
        C = self.C_charge if Q >= P_load else self.C_discharge
        self._soc += self.Ts * (Q - P_load) / C
        self._soc = max(0.0, min(1.0, self._soc))
        return Q, e, r


def _occupancy(h: float) -> float:
    """Fractional building occupancy by hour-of-day (working hours 8-19)."""
    return 1.0 if 8.0 <= h < 19.0 else 0.2


def make_profiles(scenario: dict):
    """Return (T_o_fn, P_load_fn, price_fn, soc_ref_fn), each t[h] -> value.

    Daily profiles keyed on hour-of-day (t % 24), amplitudes from the scenario so
    scenarios can express a mild day, a heatwave, price volatility, etc.
    """
    To_mean = scenario.get("To_mean", 26.0)
    To_amp = scenario.get("To_amp", 8.0)
    load_base = scenario.get("load_base", 1.2)
    load_day = scenario.get("load_day", 2.2)
    load_temp = scenario.get("load_temp", 0.18)
    price_base = scenario.get("price_base", 0.5)
    price_amp = scenario.get("price_amp", 0.45)
    soc_lo = scenario.get("soc_lo", 0.30)
    soc_hi = scenario.get("soc_hi", 0.85)

    def T_o(t):
        h = t % 24.0
        return To_mean + To_amp * math.sin(2.0 * math.pi * (h - 9.0) / 24.0)

    def P_load(t):
        h = t % 24.0
        temp_drive = load_temp * max(0.0, T_o(t) - 22.0)
        return load_base + load_day * _occupancy(h) + temp_drive

    def price(t):
        # Two low-price valleys (night ~2-5 h, midday ~12-15 h), morning/evening peaks.
        h = t % 24.0
        shape = (math.cos(2.0 * math.pi * (h - 8.0) / 24.0)
                 + 0.5 * math.cos(4.0 * math.pi * (h - 8.0) / 24.0))
        return price_base + price_amp * 0.5 * (shape + 1.0)

    def soc_ref(t):
        # Pre-charge schedule: full before the working-hour peak, discharge through
        # it, recharge in the midday valley, discharge through the evening peak.
        h = t % 24.0
        pts = [(0, soc_lo + 0.15), (6, soc_hi), (11, soc_lo + 0.1),
               (15, soc_hi - 0.05), (21, soc_lo), (24, soc_lo + 0.15)]
        for (h0, v0), (h1, v1) in zip(pts[:-1], pts[1:]):
            if h0 <= h <= h1:
                a = (h - h0) / (h1 - h0)
                return v0 + a * (v1 - v0)
        return soc_lo

    return T_o, P_load, price, soc_ref
