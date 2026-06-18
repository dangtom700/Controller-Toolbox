"""
aircraft_engine_thermal_management_plant.py
Reduced control-oriented model of the Intermediate Circulation Loop of an
aircraft engine Fuel Thermal Management System (FTMS), after He, Zhang,
Zhang & Chen (2026), "Model predictive control for aircraft engine thermal
management under nonlinear heat transfer and time delay", Aerospace Science
and Technology 169, 111491.

Topology (paper Fig. 1 / Fig. 6): a water tank feeds a main loop (flow rate
m0) that splits into branch 1 (m1, through AHE1) and branch 2 (m2 = m0-m1,
through AHE2). Both air-water heat exchangers (AHE1, AHE2) pick up heat from
hot bleed air; the two branches remerge and pass through a third exchanger
(OHE) where the heat is dumped into the fuel before the water returns to the
tank. tau_d is the paper's identified "equivalent delay time of the system"
(Section 4.3): the lumped fluid-transport delay between the tank and the two
AHE water inlets.

States:        x = [TT, m1, m2]
  TT  - water tank temperature [K]
  m1  - branch-1 mass flow rate [kg/s]   (through AHE1)
  m2  - branch-2 mass flow rate [kg/s]   (through AHE2)
  (m0 = m1 + m2, Eq. 6 - recovered algebraically, not a separate state)
Inputs:        u = [u1, u2] = [m1_dot, m2_dot]  - flow-rate rates of change
               [kg/s^2] (paper Eq. 14: "the time derivative of mass flow
               rates is designated as the direct control input"); the plant
               integrates u directly, m1_dot = u1, m2_dot = u2.
Disturbances:  Thin1(t), Thin2(t)  - AHE1/AHE2 hot (air) inlet temperatures [K]
               mh1(t),  mh2(t)    - AHE1/AHE2 hot (air) side mass flow [kg/s]
Outputs:       Thout1, Thout2      - AHE1/AHE2 air-side outlet temperatures [K]
               (paper Eq. 13: the controlled variables)

Model simplifications (relative to the paper - documented per project
convention, see README "Model Simplifications"):
  1. Heat exchangers: the paper derives a "thermal resistance R" form of the
     counter-flow exchanger (Eq. 7) via the heat-current/entransy method.
     The PDF extraction of that stacked-fraction equation is not reliably
     recoverable (lossy OCR). This case study instead uses the textbook
     effectiveness-NTU counter-flow model (Kays & London), which is the
     standard, numerically-verified closed form for the *same* underlying
     physical heat exchanger (a counter-flow exchanger of conductance K*A
     is K*A is K*A regardless of which equivalent algebraic form is used to
     express it).
  2. Transport delay: the paper derives per-pipeline delays (tau1, tau2,
     taua, taub, ...) and then explicitly collapses them to one "equivalent"
     delay (tau2 ~= tau1, "this paper temporarily takes 60 s as the
     equivalent delay time of the system", Section 4.3). This model applies
     that single lumped delay directly to the tank temperature seen at each
     AHE's water inlet, matching the paper's own end-state simplification
     (Eq. 25/26).
  3. Actuator: u is used exactly as the paper defines it (Eq. 14) - the
     direct time derivative of the flow rates, integrated by the plant.
     A physical rate limit (u_rate_max) bounds |u| as a stand-in for the
     pump/valve's finite slew rate.
  4. The "forced cooling" / "valve shutdown" rule-based overrides described
     in the paper (Section 3.3, Fig. 8) are implemented as a supervisor
     wrapped around every controller in simulation_runner.py, not inside
     the plant - this matches the paper's own framing of it as a layer
     above the predictive controller ("a hybrid control architecture...
     overrides the MPC when temperatures approach the critical limit").
  5. State choice: the paper's own state is (TT, m0, m1) with m2 = m0 - m1
     a *dependent* quantity. Driving Thout1 (via m1) and Thout2 (via m0)
     with two independent SISO loops on that state choice means the loop
     that raises m1 to cool AHE1 simultaneously *steals* flow from branch 2
     (m2 = m0 - m1 falls), and a hard m1 <= m0 saturation can drive m2 to
     zero and freeze AHE2 entirely - an artefact of the state choice, not
     a real physical limitation. This model instead uses the independent
     branch flows (m1, m2) as states (m0 = m1 + m2 recovered wherever
     needed) - identical heat-exchanger/tank physics, but each SISO loop
     now has its own actuator with no shared-resource lock between them.

Integration: 4th-order Runge-Kutta at Ts (states TT, m1, m2); the lumped
transport delay is applied via a ring buffer of past TT samples.
"""

import math
from collections import deque


def _counterflow_effectiveness(Gh: float, Gc: float, KA: float) -> float:
    """Effectiveness-NTU model for a counter-flow heat exchanger.

    Gh, Gc: hot/cold heat-capacity rates [W/K]; KA: conductance [W/K].
    Returns epsilon in [0, 1) such that Q = epsilon * min(Gh,Gc) * (Thin - Tcin).
    """
    if Gh <= 1e-9 or Gc <= 1e-9:
        return 0.0
    c_min = min(Gh, Gc)
    c_max = max(Gh, Gc)
    cr = c_min / c_max
    ntu = KA / c_min
    if cr > 0.9999:
        return ntu / (1.0 + ntu)
    expo = math.exp(-ntu * (1.0 - cr))
    denom = 1.0 - cr * expo
    if denom < 1e-12:
        return 1.0
    return (1.0 - expo) / denom


class AETMPlant:
    """Intermediate Circulation Loop plant: x = [TT, m1, m2]."""

    def __init__(self, params: dict):
        p = params
        self.Ts = p.get("Ts", 0.5)

        self.mT = p.get("mT", 11.22)                  # tank dynamic capacity [kg]
        self.cp_water = p.get("cp_water", 3100.0)      # [J/kg/K]
        self.cp_air = p.get("cp_air", 1050.0)          # [J/kg/K]
        self.cp_fuel = p.get("cp_fuel", 2300.0)        # [J/kg/K]

        self.KA1 = p.get("KA1", 713.7)                 # AHE1 conductance [W/K]
        self.KA2 = p.get("KA2", 1276.1)                # AHE2 conductance [W/K]
        self.KA3 = p.get("KA3", 11409.2)               # OHE  conductance [W/K]

        self.tau_d = p.get("tau_d", 60.0)              # lumped transport delay [s]
        self.u_rate_max = p.get("u_rate_max", 0.02)    # max |m_dot| [kg/s^2]

        self.m1_min = p.get("m1_min", 0.05)
        self.m1_max = p.get("m1_max", 0.45)
        self.m2_min = p.get("m2_min", 0.05)
        self.m2_max = p.get("m2_max", 0.45)

        self.mh3 = p.get("mh3", 1.30)                   # fuel flow through OHE [kg/s]
        self.Tcin3 = p.get("Tcin3", 310.0)              # fuel inlet temp to OHE [K]

        TT0 = p.get("TT0", 380.0)
        m1_0 = p.get("m1_0", 0.25)
        m2_0 = p.get("m2_0", 0.25)
        self._x = [TT0, m1_0, m2_0]

        n_buf = max(1, int(round(self.tau_d / self.Ts)))
        self._tt_buf = deque([TT0] * n_buf, maxlen=n_buf)

        # Boundary-condition function: t -> (Thin1, Thin2, mh1, mh2[, Tcin3])
        self._bc_fn = lambda t: (723.0, 633.0, 0.48, 0.37)

    def set_boundary_fn(self, bc_fn):
        """bc_fn(t) -> (Thin1, Thin2, mh1, mh2) [K, K, kg/s, kg/s]."""
        self._bc_fn = bc_fn

    def reset(self, x0=None):
        if x0 is None:
            self._x = [380.0, 0.25, 0.25]
        else:
            self._x = list(x0)
        n_buf = self._tt_buf.maxlen
        self._tt_buf = deque([self._x[0]] * n_buf, maxlen=n_buf)

    def state(self):
        return tuple(self._x)

    def tt_delayed(self) -> float:
        return self._tt_buf[0]

    def outputs(self, t: float, x=None):
        """Return (Thout1, Thout2, m0, Tmix, TTi) at the given state/time."""
        if x is None:
            x = self._x
        TT, m1, m2 = x
        m0 = m1 + m2
        Thin1, Thin2, mh1, mh2 = self._bc_fn(t)
        TT_d = self.tt_delayed()

        Gh1 = mh1 * self.cp_air
        Gc1 = m1 * self.cp_water
        eps1 = _counterflow_effectiveness(Gh1, Gc1, self.KA1)
        if Gh1 > 1e-9:
            Thout1 = Thin1 - eps1 * (Thin1 - TT_d)
        else:
            Thout1 = Thin1
        Twater_out1 = TT_d + (eps1 * Gh1 / max(Gc1, 1e-9)) * (Thin1 - TT_d) if Gc1 > 1e-9 else TT_d

        Gh2 = mh2 * self.cp_air
        Gc2 = m2 * self.cp_water
        eps2 = _counterflow_effectiveness(Gh2, Gc2, self.KA2)
        if Gh2 > 1e-9:
            Thout2 = Thin2 - eps2 * (Thin2 - TT_d)
        else:
            Thout2 = Thin2
        Twater_out2 = TT_d + (eps2 * Gh2 / max(Gc2, 1e-9)) * (Thin2 - TT_d) if Gc2 > 1e-9 else TT_d

        if m0 > 1e-6:
            Tmix = (m1 * Twater_out1 + m2 * Twater_out2) / m0
        else:
            Tmix = TT_d

        Gh3 = m0 * self.cp_water
        Gc3 = self.mh3 * self.cp_fuel
        eps3 = _counterflow_effectiveness(Gh3, Gc3, self.KA3)
        TTi = Tmix - eps3 * (Tmix - self.Tcin3)

        return Thout1, Thout2, m0, Tmix, TTi

    def _derivs(self, x, u, t):
        TT, m1, m2 = x
        u1, u2 = u
        _, _, _, _, TTi = self.outputs(t, x)
        m0 = m1 + m2

        TT_dot = m0 * (TTi - TT) / self.mT
        m1_dot = u1
        m2_dot = u2
        return [TT_dot, m1_dot, m2_dot]

    def step(self, u, t: float):
        """Advance one Ts step (RK4); u = [m1_dot_cmd, m2_dot_cmd] [kg/s^2],
        rate-limited to +/- u_rate_max. Returns the new state tuple."""
        u1 = max(-self.u_rate_max, min(self.u_rate_max, u[0]))
        u2 = max(-self.u_rate_max, min(self.u_rate_max, u[1]))
        u_sat = (u1, u2)

        dt = self.Ts
        x = self._x
        k1 = self._derivs(x, u_sat, t)
        x2 = [xi + dt / 2 * ki for xi, ki in zip(x, k1)]
        k2 = self._derivs(x2, u_sat, t + dt / 2)
        x3 = [xi + dt / 2 * ki for xi, ki in zip(x, k2)]
        k3 = self._derivs(x3, u_sat, t + dt / 2)
        x4 = [xi + dt * ki for xi, ki in zip(x, k3)]
        k4 = self._derivs(x4, u_sat, t + dt)

        new_x = [xi + dt / 6 * (k1i + 2 * k2i + 2 * k3i + k4i)
                 for xi, k1i, k2i, k3i, k4i in zip(x, k1, k2, k3, k4)]
        new_x[1] = max(self.m1_min, min(self.m1_max, new_x[1]))
        new_x[2] = max(self.m2_min, min(self.m2_max, new_x[2]))
        self._x = new_x
        self._tt_buf.append(new_x[0])
        return tuple(self._x)


def make_boundary_fn(scenario: dict):
    """Build bc_fn(t) -> (Thin1, Thin2, mh1, mh2) for the given scenario.

    Scenario "profile" types:
      constant     - fixed (Thin1, Thin2, mh1, mh2) for the whole run.
      condition_switch - piecewise schedule of (t_start, Thin1, Thin2, mh1, mh2).
      ramp         - linear ramp between (t0,val0) and (t1,val1) per channel,
                     held constant before/after.
    """
    profile = scenario.get("profile", "constant")

    if profile == "constant":
        Thin1 = scenario["Thin1"]
        Thin2 = scenario["Thin2"]
        mh1 = scenario["mh1"]
        mh2 = scenario["mh2"]

        def bc_fn(t):
            return Thin1, Thin2, mh1, mh2
        return bc_fn

    if profile == "condition_switch":
        segs = scenario["segments"]  # list of {t_start, Thin1, Thin2, mh1, mh2}

        def bc_fn(t):
            seg = segs[0]
            for s in segs:
                if t >= s["t_start"]:
                    seg = s
                else:
                    break
            return seg["Thin1"], seg["Thin2"], seg["mh1"], seg["mh2"]
        return bc_fn

    if profile == "ramp":
        r = scenario["ramp"]
        t0, t1 = r["t0"], r["t1"]

        def _interp(v0, v1, t):
            if t <= t0:
                return v0
            if t >= t1:
                return v1
            frac = (t - t0) / (t1 - t0)
            return v0 + frac * (v1 - v0)

        Thin1_0, Thin1_1 = r["Thin1_0"], r["Thin1_1"]
        Thin2_0, Thin2_1 = r["Thin2_0"], r["Thin2_1"]
        mh1_0, mh1_1 = r["mh1_0"], r["mh1_1"]
        mh2_0, mh2_1 = r["mh2_0"], r["mh2_1"]

        def bc_fn(t):
            return (_interp(Thin1_0, Thin1_1, t), _interp(Thin2_0, Thin2_1, t),
                    _interp(mh1_0, mh1_1, t), _interp(mh2_0, mh2_1, t))
        return bc_fn

    # Fallback: nominal condition-1 boundary values
    return lambda t: (723.0, 633.0, 0.48, 0.37)
