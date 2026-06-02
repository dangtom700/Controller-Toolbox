# Porous Fiber Plate Humidification System

**Reference:** Ye, Yan & Ni, "Theoretical calculation and experimental analysis on humidification
performance of high-performance water-absorbing porous fiber plates," *Applied Thermal Engineering*
245 (2024) 122877. Harbin Institute of Technology.

---

## Context and Motivation

Indoor air in severe cold regions (e.g., Harbin, China) drops to 10–25 % RH in winter as dry
outdoor air infiltrates heated buildings. Low humidity impairs occupant health, enables virus
transmission, and degrades work performance. The paper introduces a **porous polyester-fiber
plate** (UNITIKA) that wicks water from a pan below via capillary action and evaporates it into
passing airflow - no top spray, no moving parts, compatible with miniaturized fan coil units.

The control challenge: humidification capacity H [g/h] is a nonlinear function of fan speed,
inlet air temperature, and inlet relative humidity (all three change together as outdoor
conditions vary). A controller that assumes a fixed linear gain will overshoot in dry cold
conditions and undershoot in mild damp ones.

---

## Plant Model

Two coupled subsystems integrated at each discrete timestep (Ts = 30 s):

### Subsystem 1 - Humidifier Physics (nonlinear algebraic, from paper)

Converts fan speed `u_fan` [m/s] and inlet air state (`Ta` [K], `phi_in` [-]) to humidification
rate `H` [g/h]. The derivation follows the laminar flat-plate criterion correlation.

```
// Wet-bulb approximation: plate surface = wet-bulb temperature of inlet air
Tp  = wetBulb(Ta, phi_in)          // [K]
Tm  = (Ta + Tp) / 2                // mean boundary-layer temperature [K]

// Water-vapour diffusivity in air (Chapman-Enskog, re-fitted for T range)
D   = D0 * (Tm / T0)^1.5          // D0 = 2.2e-5 m^2/s, T0 = 273 K

// Air kinematic viscosity and Prandtl/Schmidt numbers at Tm
nu  = air_viscosity(Tm)
Sc  = nu / D
Pr  = air_prandtl(Tm)

// Velocity in the inter-plate gap is higher than the duct velocity (CFD-corrected)
u_gap = C_gap * u_fan              // C_gap = 1.25 (from paper Table 2 regression)

// Dimensionless numbers (laminar flat-plate, Re < 1e5 confirmed)
Re  = u_gap * l / nu               // l = 0.1 m (plate depth in flow direction)
Sh  = 0.664 * sqrt(Re) * cbrt(Sc) // Sherwood (mass-transfer Nusselt analogy)
Nu  = 0.664 * sqrt(Re) * cbrt(Pr) // Nusselt (heat transfer)

// Convective mass-transfer coefficient
hm  = Sh * D / l                   // [m/s]

// Vapour partial pressures (Antoine equation)
Pps = Psat(Tp)                     // saturated vapour pressure at plate surface [Pa]
Pa  = phi_in * Psat(Ta)            // actual vapour pressure of inlet air [Pa]

// Mass flux and humidification capacity
mw  = hm * (M / Rw) * (Pps/Tp - Pa/Ta) * A   // A = 0.4 m^2 (20 plates x 2 sides x 0.1x0.1)
H   = mw * 3.6e6                   // [g/h]   (mw in kg/s -> g/h)

// Convective heat transfer (used for energy balance check)
h   = Nu * lambda(Tm) / l
Qc  = h * A * LMTD(Ta, Te, Tp)    // [W]; Te from enthalpy balance
```

**Saturation equation (Antoine form used in paper):**
```
Psat(T_K) = 610.78 * exp(17.2694 * (T_K - 273.15) / (T_K - 35.85))    [Pa]
```

**Validated range:** Ta = 35–45 ^\circC, phi_in = 0.15–0.30, u_fan = 1.0–3.5 m/s.
Peak H = 266.4 g/h at (Ta=45 ^\circC, phi=15 %, u=3.5 m/s). Theoretical errors < 10 %
after CFD velocity correction.

### Subsystem 2 - Room Humidity Dynamics (first-order ODE)

A well-mixed room with volume V_room = 50 m^3 (4 * 4 * 3 m, typical small office).

```
Moisture balance (specific humidity omega [g/kg dry air]):

  V_room * rho_a * d(omega_room)/dt =
      m_dot_fan * (omega_out_fan - omega_room)   // fan coil discharge
    + m_dot_infil * (omega_out - omega_room)     // envelope infiltration
    + G_occ                                      // occupant moisture generation

where:
  m_dot_fan  = rho_a * Q_fan = rho_a * A_duct * u_fan  // A_duct = 0.0256 m^2
  omega_out_fan = omega_in + H / (m_dot_fan * 1000)    // humidifier adds H g/h
  m_dot_infil   = rho_a * V_room * ACH / 3600          // ACH = 0.5 h^-1
  G_occ        = 50 [g/h per person] * n_occ           // scenario-dependent
```

Relative humidity recovered from omega_room via inverse psychrometric relation at T_room.

### Control Inputs

| Symbol | Description | Valid Range |
|--------|-------------|-------------|
| u_fan | Fan speed (fan coil airflow) | [1.0, 3.5] m/s |
| Ta_sp | Inlet air heater setpoint | [30, 50] ^\circC |

The primary manipulated variable is `u_fan`. `Ta_sp` is a secondary input used by cascade
and MPC controllers; it is held at 40 ^\circC for single-input controllers.

### Key Plant Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| l | 0.10 m | Plate length in flow direction |
| A | 0.40 m^2 | Total plate surface area (20 plates * 2 sides * 0.1 * 0.1 m) |
| C_gap | 1.25 | Gap-to-duct velocity ratio (CFD, Table 2) |
| D0 | 2.2 * 10^-^5 m^2/s | Diffusivity of water vapour in air at 273 K |
| M | 0.018 kg/mol | Molar mass of water vapour |
| Rw | 461.89 J/(kg.K) | Gas constant for water vapour |
| A_duct | 0.0256 m^2 | Fan coil duct cross-section |
| V_room | 50 m^3 | Room volume |
| ACH | 0.5 h^-^1 | Infiltration air changes per hour |
| Ts | 30 s | Controller sampling period |

All parameters loaded from `config/plant_params.json`.

### Disturbances

| Symbol | Description | Winter typical |
|--------|-------------|----------------|
| T_out | Outdoor dry-bulb temperature | -20 to -5 ^\circC |
| phi_out | Outdoor relative humidity | 0.15 – 0.45 |
| n_occ | Occupant count | 0 – 4 persons |

---

## Scenarios

| ID | Description | T_out [^\circC] | phi_out | Setpoint phi_room | Notes |
|----|-------------|-----------|---------|-------------------|-------|
| s01_design | Nominal winter operation | -10 | 0.35 | 0.45 | Baseline comparison |
| s02_cold_snap | Severe cold dry air | -20 | 0.20 | 0.45 | Maximum humidification demand |
| s03_setpoint_step | Setpoint step 30 %->50 % at t=900 s | -10 | 0.35 | 0.30->0.50 | Tracking speed vs. overshoot |
| s04_occupancy | 4 occupants enter at t=600 s, leave at t=2400 s | -10 | 0.35 | 0.45 | Moisture disturbance rejection |
| s05_mild_humid | Mild outdoor air; risk of over-humidification | -3 | 0.55 | 0.45 | Controller must reduce fan speed |

**Total runs: 10 controllers * 5 scenarios = 50**

s02 (cold/dry) is the hardest: phi_out = 0.20 pushes the humidifier toward its maximum
capacity (u_fan must saturate at 3.5 m/s). Controllers without anti-windup or feedforward
will exhibit sustained error. s05 (mild/humid) is the complementary stress test: phi_in
already near setpoint, so the controller must command near-zero fan speed; controllers with
integrator windup may overshoot into condensation territory.

---

## Controller Roster

Each controller subclasses `HumidificationControllerBase`. Its `compute(phi_measured, ref_phi)`
returns `u_fan [m/s]`. The heater setpoint `Ta_sp` is updated via `setHeaterSetpoint(double)`;
single-input controllers leave it at the default 40 ^\circC.

| # | Name | lib/ Algorithm(s) | Design Notes |
|---|------|--------------------|--------------|
| 1 | PID | `DiscretePID` | PI on phi_room error (Kp=8, Ki=0.004); fan-speed output; Ta_sp=40 ^\circC |
| 2 | PID_AW | `DiscretePID` (built-in Kb AW) | Same as #1 but back-calculation AW Kb=0.1; exploits DiscretePID's native Kb; do NOT wrap in AntiWindupWrapper |
| 3 | FeedforwardPID | `DiscretePID` + static FF | FF from outdoor conditions: u_ff = f(T_out, phi_out, phi_sp); residual error closed by PI |
| 4 | CascadePID | `DiscretePID` * 2 | Inner loop: Ta tracks Ta_sp (fast, Ts=5 s); outer loop: phi_room -> Ta_sp setpoint |
| 5 | GainScheduled | `GainScheduledController` | Gain table indexed by phi_in (inlet RH); 3 operating points: phi_in=[0.15,0.25,0.35]; each point: DiscretePID with tuned gains |
| 6 | SmithPredictor | `SmithPredictor` | 2-step transport delay (60 s RH sensor lag in duct); inner PI Kp=8, Ki=0.004; FOPDT model tau=700s, K=12 %/(m/s) |
| 7 | ADRC | `DiscreteADRC` | b0=12, omega_o=0.03 rad/s (omega_o*Ts=0.90 < 0.5? No: 0.03*30=0.9 - use omega_o=0.015, Ts=30: 0.015*30=0.45 < 0.5 (check)); ESO lumps infiltration + occupancy disturbance |
| 8 | MPC | `DiscreteMPC` | FOPDT linearized at nominal (tau=700 s, K_fan=12 %/(m/s), K_heat=0.8 %/^\circC); Np=20, Nc=6; box constraints u_fan\in[1,3.5], Ta\in[30,50]; soft output constraint phi\in[0.35,0.65] |
| 9 | MRAC | `MRACController` | Reference model a_m=0.96, b_m=0.04 (matches tau=700 s at Ts=30 s); sigma-modification adapts to gain shift between s01/s02/s05; compute(y_plant) NOT error |
| 10 | GPC_RLS | `GeneralizedPredictiveController` + `RecursiveLeastSquares` | Np=15, Nu=4; RLS na=1, nb=1, lambda=0.97; 60-step warmup at fixed PI; updates every 15 steps; adapts as K_hum drifts with outdoor conditions |

### Key Implementation Notes

- **ADRC omega_o:** Must satisfy `omega_o * Ts < 0.5` (backward-Euler LADRC stability bound).
  With Ts=30 s, use `omega_o = 0.015` rad/s -> `omega_o * Ts = 0.45`. Do not increase above
  `0.016` without reducing Ts.
- **GainScheduledController design_fn:** Returns `shared_ptr<IController>`. `DiscreteLQR` is
  **not** an `IController` - wrap gain matrices in a `DiscretePID` if using LQR-derived gains
  (per case-study tribal knowledge).
- **SmithPredictor delay:** 2 steps at Ts=30 s = 60 s total delay. The inner model uses FOPDT
  parameters identified from the plant step response at the nominal operating point.
- **MRAC sign convention:** `compute(y_plant)` receives the raw phi_room measurement, not the
  error. Call `setReference(phi_sp)` each step before `compute()`.
- **s05 low-demand clamp:** When phi_room already exceeds phi_sp, u_fan output should be
  clamped to u_min=1.0 m/s (fan off is not modelled; the coil still needs airflow for heating).
  Controllers without explicit lower-bound anti-windup may wind down the integrator into negative
  territory.
- **RLS accessor:** Use `rls.params()` (not `theta()`); call `rls.update(phi_measured, u_fan)`
  - output first, input second.

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

## Implementation Plan

### Phase 1 - Plant model (`plant/HumidificationPlant.{h,cpp}`)

1. Implement `psychro::Psat(T_K)` (Antoine), `psychro::wetBulb(Ta, phi)`, `psychro::air_viscosity(T)`, `psychro::air_prandtl(T)`, `psychro::air_lambda(T)` as a small inline header.
2. Implement `HumidifierPhysics::computeH(u_fan, Ta_K, phi_in)` - returns H [g/h].
3. Implement `RoomModel::step(u_fan, Ta_K, H, phi_out, T_out, n_occ)` - Euler integration of moisture balance, returns phi_room.
4. Wrap both in `HumidificationPlant` with `step(u_fan, Ta_sp)` interface and sensor model (30 s RH delay, +/-1.5 % additive noise matching paper instrument accuracy).
5. Unit-test: reproduce paper Table data - at (Ta=40 ^\circC, phi=0.20, u=2.0 m/s) expect H approx = 140 g/h +/- 20 % (within paper theoretical error band).

### Phase 2 - Controller base and roster (`controllers/`)

6. Define `HumidificationControllerBase` (abstract): `compute(phi, ref)` -> u_fan; `setHeaterSetpoint(double)` no-op by default.
7. Implement controllers 1–10 in separate `.cpp` files, each constructing the relevant `lib/` object(s) in its constructor.
8. Add `config/plant_params.json` and one JSON per scenario under `config/scenarios/`.

### Phase 3 - Runner (`main.cpp`)

9. Loop over all scenario * controller pairs; write one CSV row per timestep; print per-run metrics summary.
10. Controller count hard-coded at 10 - bump the constant when adding controllers.
11. Build target name: `humidification_sim`. Add to `compile.bat` and `CMakeLists.txt`.

### Phase 4 - run.py integration

12. Add `("humidification_sim", 50, ...)` to run.py's case-study registry (50 = 10 * 5).
13. Add a smoke entry to `bindings/smoke_test.py` if any new pybind11 bindings are introduced (none planned for Phase 1–3).

### Phase 5 - Catch2 regression test

14. Add `[humidification]` tag tests in `tests/test_catch2_advanced.cpp`:
    - Physics model correctness (reproduce paper peak H within 15 %).
    - Room model steady-state (phi_room converges to feedforward equilibrium).
    - PID closed-loop: phi_room reaches 45 % +/- 3 % within 1800 s for s01 nominal scenario.

---

## Open Questions / Design Decisions

| # | Question | Recommendation |
|---|----------|---------------|
| Q1 | Use a fan-coil heat balance (T_room dynamics) or isothermal room? | Start isothermal (T_room=22 ^\circC fixed); add T_room ODE in a future pass if cascade PID needs it |
| Q2 | Discretize room ODE with Euler or RK4? | Forward Euler at Ts=30 s is sufficient (tauapprox =700 s >> Ts) |
| Q3 | Include condensation guard (phi_room > 0.70 risk)? | Hard-clamp u_fan to 0 at phi_room=0.68 as a safety floor in HumidificationPlant |
| Q4 | Should GainScheduledController use LinearBlend or Nearest mode? | Nearest for simplicity; LinearBlend bumpless (T5 open item) not yet implemented |
| Q5 | How to handle the 60 s RH sensor delay in non-Smith controllers? | Plant outputs the delayed signal by default; SmithPredictor gets the true signal as inner feedback |

---

## File Layout (target)

```
case-study/Porous Fiber Plate Humidification System/
|-- README.md                          (this file)
|-- TheoreticalCalculation...pdf       (move here from case-study/)
|-- config/
|   |-- plant_params.json
|   |-- scenarios/
|       |-- s01_design.json
|       |-- s02_cold_snap.json
|       |-- s03_setpoint_step.json
|       |-- s04_occupancy.json
|       |-- s05_mild_humid.json
|-- plant/
|   |-- HumidificationPlant.h
|   |-- HumidificationPlant.cpp
|   |-- psychrometrics.h               (Psat, wetBulb, air properties)
|-- controllers/
|   |-- HumidificationControllerBase.h
|   |-- PIDHumidCtrl.cpp
|   |-- PID_AWHumidCtrl.cpp
|   |-- FFPIDHumidCtrl.cpp
|   |-- CascadePIDHumidCtrl.cpp
|   |-- GainScheduledHumidCtrl.cpp
|   |-- SmithPredictorHumidCtrl.cpp
|   |-- ADRCHumidCtrl.cpp
|   |-- MPCHumidCtrl.cpp
|   |-- MRACHumidCtrl.cpp
|   |-- GPC_RLSHumidCtrl.cpp
|-- main.cpp
|-- CMakeLists.txt
|-- logs/                              (created at runtime)
```
