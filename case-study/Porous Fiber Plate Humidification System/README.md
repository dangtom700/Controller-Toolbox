# Porous Fiber Plate Humidification System

**Reference:** Ye, Yan & Ni, "Theoretical calculation and experimental analysis on humidification
performance of high-performance water-absorbing porous fiber plates," *Applied Thermal Engineering*
245 (2024) 122877. Harbin Institute of Technology.

---

## Context and Motivation

Indoor air in severe cold regions (e.g., Harbin, China) drops to 10-25 % RH in winter as dry
outdoor air infiltrates heated buildings. Low humidity impairs occupant health, enables virus
transmission, and degrades work performance. The paper introduces a **porous polyester-fiber
plate** (UNITIKA) that wicks water from a pan below via capillary action and evaporates it into
passing airflow - no top spray, no moving parts, compatible with miniaturised fan-coil units.

The control challenge: humidification capacity H [g/h] is a nonlinear function of fan speed,
inlet air temperature, and inlet relative humidity (all three change together as outdoor
conditions vary). A controller that assumes a fixed linear gain will overshoot in dry cold
conditions and undershoot in mild damp ones.

---

## Plant Model

Two coupled subsystems integrated at each discrete timestep (Ts = 30 s).

### Subsystem 1 - Humidifier Physics (nonlinear algebraic)

Converts fan speed `u_fan` [m/s] and inlet air state (`Ta` [K], `phi_in` [-]) to
humidification rate H [g/h]. Follows the laminar flat-plate criterion correlation.

```
Wet-bulb temperature (plate surface):  Tp = wetBulb(Ta, phi_in)   [K]
Mean boundary-layer temperature:        Tm = (Ta + Tp) / 2

Water-vapour diffusivity:   D  = 2.2e-5 * (Tm / 273)^1.5         [m^2/s]
Air kinematic viscosity:    nu = 1.328e-5 + 9.6e-8 * (Tm - 293)  [m^2/s]
Schmidt number:             Sc = nu / D

Gap velocity (CFD-corrected):  u_gap = 1.25 * u_fan               [m/s]
Reynolds number:               Re    = u_gap * l_plate / nu
Sherwood number (laminar):     Sh    = 0.664 * sqrt(Re) * cbrt(Sc)
Mass-transfer coefficient:     hm    = Sh * D / l_plate            [m/s]

Vapour partial pressures:
  Pps = Psat(Tp)              [Pa]   (saturated at plate surface)
  Pa  = phi_in * Psat(Ta)    [Pa]   (actual in inlet air)

Vapour density difference:  delta_rho = Pps/(Rw*Tp) - Pa/(Rw*Ta)  [kg/m^3]
Mass flux:                  mw = hm * A_plates * delta_rho          [kg/s]
Humidification capacity:    H  = max(0, mw * 3.6e6)                [g/h]
```

**Validated range:** Ta = 35-45 ^\circC, phi_in = 0.15-0.30, u_fan = 1.0-3.5 m/s.
Peak H approx = 266 g/h at (Ta = 45 ^\circC, phi = 15 %, u = 3.5 m/s). Theoretical errors < 10 %
after CFD gap-velocity correction (C_gap = 1.25).

### Subsystem 2 - Room Humidity Dynamics (first-order ODE, Euler)

Well-mixed room, V_room = 50 m^3 (small office), forward Euler at Ts = 30 s.

```
d(omega_room)/dt = [H_gs + G_occ_gs + m_dot_inf*(omega_out - omega_room)] / (rho_a * V_room)

where:
  H_gs        = H [g/h] / 3600          [g/s]   humidifier output
  G_occ_gs    = 50 * n_occ / 3600       [g/s]   occupant generation
  m_dot_inf   = rho_a * V_room * ACH / 3600      infiltration mass flow [kg/s]
  omega_out   = specific humidity of outdoor air [g/kg]
```

Room time constant: tau_room = rho_a * V_room / m_dot_inf approx = 7200 s (2 hours).
Forward Euler at Ts = 30 s is accurate (Ts / tau_room approx = 0.004).

**Guard:** `omega_room = max(omega_room, 0)` prevents unphysical negative specific humidity.

### Sensor Model

2-step FIFO delay on phi_room output, modelling the 60 s transport lag of a
duct-mounted RH sensor. Controllers receive the **delayed** phi_measured; only
SmithPredictor compensates for this explicitly.

### Control Inputs

| Symbol | Description | Valid Range |
|--------|-------------|-------------|
| u_fan | Fan speed (duct airflow velocity) | [1.0, 3.5] m/s |
| Ta_sp | Inlet air heater setpoint | [30, 50] ^\circC |

The primary manipulated variable is `u_fan`. `Ta_sp` is a secondary input; single-input
controllers leave it at the default 40 ^\circC.

### Key Plant Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| l_plate | 0.10 m | Plate depth in flow direction |
| A_plates | 0.40 m^2 | Total plate surface (20 plates * 2 sides * 0.1 * 0.1 m) |
| C_gap | 1.25 | Gap-to-duct velocity ratio (CFD, paper Table 2) |
| V_room | 50 m^3 | Room volume |
| T_room | 22 ^\circC | Room dry-bulb (fixed, isothermal assumption) |
| ACH | 0.5 h^-^1 | Infiltration air changes per hour |
| Ts | 30 s | Controller sampling period |
| Sensor delay | 2 steps (60 s) | RH sensor transport lag |

All parameters loaded from `config/plant_params.json`.

### Disturbances

| Symbol | Description | Winter typical |
|--------|-------------|----------------|
| T_out | Outdoor dry-bulb temperature | -20 to -3 ^\circC |
| phi_out | Outdoor relative humidity | 0.20 - 0.55 |
| n_occ | Occupant count | 0 - 4 persons |

---

## Scenarios

| ID | Description | T_out [^\circC] | phi_out | Setpoint phi_room | Stress |
|----|-------------|-----------|---------|-------------------|--------|
| s01_design | Nominal winter operation | -10 | 0.35 | 0.45 | Baseline |
| s02_cold_snap | Severe cold dry air | -20 | 0.20 | 0.45 | Maximum humidification demand; u_fan likely saturates |
| s03_setpoint_step | Setpoint step 0.30->0.50 at t=900 s | -10 | 0.35 | 0.30->0.50 | Tracking speed vs. overshoot |
| s04_occupancy | 4 occupants enter at t=600 s, leave at t=2400 s | -10 | 0.35 | 0.45 | Moisture disturbance rejection |
| s05_mild_humid | Mild outdoor air; risk of over-humidification | -3 | 0.55 | 0.45 | Controller must reduce fan to u_min |

**Total runs: 10 controllers * 5 scenarios = 50**

s02 is the hardest case: phi_out = 0.20 drives maximum humidification demand; u_fan
saturates at 3.5 m/s. Controllers without feedforward or integral anti-windup exhibit
sustained error. s05 is the complementary stress test: phi_room already near setpoint,
fan must run at minimum; integrator windup can cause overshoot into condensation range.

---

## Controller Roster

Each controller subclasses `humid::ControllerBase`. Its `compute(phi_measured, ref_phi)`
returns a `ControlInput {u_fan, Ta_sp}`. Error convention: `e = ref_phi - phi_measured`.
All PID-based controllers scale the error to percentage (`e_pct = e * 100`) to keep
gains in a numerically sensible range; PID gains are therefore in units of m/s per % RH.

| # | Name | lib/ Algorithm(s) | Key Parameters | Design Notes |
|---|------|--------------------|----------------|--------------|
| 1 | PID | `DiscretePID` | Kp=0.10, Ki=1.39e-5, Kd=0; e in % | Conservative PI on phi_room error; Ta_sp=40 ^\circC |
| 2 | PID_AW | `DiscretePID` | Same as #1; Kb=0.3 | Demonstrates faster anti-windup back-calculation. Do NOT wrap in `AntiWindupWrapper` (built-in Kb). |
| 3 | FFPID | `DiscretePID` | Kp=0.06, Ki=8.0e-6; +/-0.5 m/s authority | FF: estimates indoor equiv. of outdoor phi; PID trims residual error |
| 4 | Cascade | `DiscretePID` (outer) | Outer: Kp=12 g/h/%, Ki=1.67e-3; H_nom=275 g/h | Outer PI: phi_error -> H_ref [g/h]; inner: physics inversion H_ref -> u_fan via (H/H_nom)^2 |
| 5 | GainSched | `GainScheduledController` | 3 points: phi=[30,45,60]%; NearestNeighbor | Gains scale with phi_room: Kp=[0.14,0.10,0.07]; scheduling variable = phi_measured (%) |
| 6 | Smith | `SmithPredictor` | FOPDT A=0.99583, B=0.04917; d=2 steps | Compensates 60 s sensor dead-time; inner PI: Kp=0.12, Ki=1.67e-5 |
| 7 | ADRC | `DiscreteADRC` | omega_o=0.015, omega_c=0.003; b0=1.639e-3 | omega_o*Ts=0.45 < 0.5 (check); ESO lumps infiltration + occupancy as unknown disturbance |
| 8 | MPC | `DiscreteMPC` | FOPDT A=0.99583, B=0.04917; Np=20, Nc=5 | Deviation around phi_nom=45%; u in [-1.25, +1.25] m/s from kFanMid; du in +/-0.1 m/s |
| 9 | MRAC | `MRACController` | a_m=0.9512, b_m=0.0488; gamma_r=gamma_y=0.005; sigma=0.005 | tau_m=600 s; `compute(phi*100)` NOT error; call `setReference(ref*100)` each step |
| 10 | GPC_RLS | `GeneralizedPredictiveController` + `RecursiveLeastSquares` | Np=15, Nu=4; lambda=0.97; 60-step warmup | RLS na=1, nb=1; update every 20 steps; `rls.update(y_pct, u_fan)` output first |

### Key Implementation Notes

- **ADRC omega_o:** Must satisfy `omega_o * Ts < 0.5` (strict). With Ts=30 s,
  `omega_o = 0.015` -> `omega_o*Ts = 0.45`. Do not exceed `omega_o = 0.016` without
  reducing Ts.
- **MRAC sign convention:** `compute(y_plant)` receives the raw phi_room measurement
  (scaled to %) - not the error. Call `setReference(phi_sp * 100)` each step.
- **GPC_RLS accessor:** Use `rls.params()` (not `theta()`); call `rls.update(y, u)` -
  output first, input second (TK26-3).
- **Smith Predictor dead-time:** 2 steps at Ts=30 s = 60 s total delay, matching the
  plant sensor FIFO buffer length.
- **Cascade inversion:** H scales as u_fan^0.5 (through Re^0.5 in Sh). The inversion
  `u_fan = kFanMid * (H_ref/H_nom)^2` exploits this; valid near the nominal operating
  point (u approx = 2.25 m/s, H_nom approx = 275 g/h).
- **s05 lower-bound clamp:** When phi_room exceeds setpoint, fan should go to
  u_min = 1.0 m/s (not zero - the coil still requires airflow for heating). Controllers
  with `uMin = 1.0` in their PID/MPC parameters handle this correctly.

---

## Metrics

Each run prints and logs:

```
phi_final=%  IAE=%*s  ISE=%^2*s  u_mean=m/s  H_mean=g/h  overshoot=%  settling_s=s
```

CSV logs written to `case-study/Porous Fiber Plate Humidification System/logs/`.

Primary ranking metric: **IAE** (integral absolute error on phi_room [%]).
Secondary: overshoot (%) and fan energy proxy (u_mean).

---

## Build and Run

```bash
conda run -n soft_robotics -- python run.py
```

The `humidification_sim` target is built by `compile.bat` and run automatically by
`run.py`. Expected: 50 runs (10 controllers * 5 scenarios).

Individual run:

```bash
build\case-study\"Porous Fiber Plate Humidification System"\humidification_sim.exe
```

Logs written to `case-study/Porous Fiber Plate Humidification System/logs/`.
