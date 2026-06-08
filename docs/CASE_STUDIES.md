# Case Studies — Implementation Tracker

Tracks the implementation status of every case study in `case-study/`.
Run `conda run -n soft_robotics -- python run.py` for live pass/fail counts.

---

## Status Legend

| Symbol | Meaning |
|--------|---------|
| ✅ C++ | Built executable, registered in `CMakeLists.txt` + `compile.bat`, runs in Phase 4 |
| ✅ Python | `sim/main.py` present, runs in Phase 6 via conda |
| 🔲 Stub | `README.md` spec only — `sim/` not present, not registered, not built |

---

## C++ Built Studies (8)


### Boiler Control ✅ C++

| Field | Value |
|-------|-------|
| **Plant** | Bell-Åström 3×3 MIMO boiler-turbine (drum pressure, power output, drum water level) |
| **Reference** | Åström & Bell 2000 |
| **Controllers** | 27 |
| **Scenarios** | 8 |
| **Total runs** | 216 (27×8) |
| **Build target** | `boiler_sim` |
| **Logs** | `case-study/Boiler Control/logs/` |

**Controller roster:** PID, LQR, LQG, MPC, SMC, ESC, ADRC, Lead-Lag+PID, SmithPredictor, GPC-RLS, EKF-LQR, UKF-LQR, FuzzyPID, FuzzySup-MPC, ControllerStack ×3, Repetitive, MRAC, H-infinity, AdaptiveSmithPredictor, NonlinearMPC, FeedbackLinearisation, MHE-LQR, LPV-GainScheduled, SubspaceID-LQG, AutoGainScheduler-LQR.

**Key caveat:** `omega_o` for ADRC reduced to 0.45 (0.45×Ts=0.45×1.0=0.45 < 0.5). `computeY3` guards against division by zero with `x3_safe = max(x3, 1.0)`.

---

### Tug Boat Numerical Simulation ✅ C++

| Field | Value |
|-------|-------|
| **Plant** | 3-DOF marine vessel (surge, sway, yaw), 6-state MIMO + thrust allocation matrix |
| **Reference** | Li et al. 2026, Ocean Engineering 357 |
| **Controllers** | 18 |
| **Scenarios** | 4 |
| **Total runs** | 72 (18×4) |
| **Build target** | `tug_sim` |
| **Logs** | `case-study/Tug Boat Numerical Simulation/logs/` |

**Controller roster:** PID, KF-PID, SMC, MPC, ESC, FuzzyPID, FuzzySup-MPC, ADRC, Repetitive, MIMO LQR, LQG, per-axis TubeMPC, EKF-LQR, MRAC, AutoGainScheduler-LQR, NonlinearMPC, L1Adaptive (3-axis a_m=0.97), ScenarioMPC (3-axis Np=60, Nu=5, N_samples=30).

---

### Solar-Driven Cooling System ✅ C++

| Field | Value |
|-------|-------|
| **Plant** | Algebraic SISO solar cooling loop + PV evaporative chimney |
| **Reference** | Ruiz et al. 2024 |
| **Controllers** | 14 |
| **Scenarios** | 5 |
| **Total runs** | 70 (14×5) |
| **Build target** | `solar_cooling_sim` |
| **Logs** | `case-study/Solar-Driven Cooling System .../logs/` |

**Controller roster:** PID, ADRC, SMC, LQR, MPC, MRAC, FuzzyPID, TubeMPC, GPC-RLS, ILC (two-phase N_trial=180), NeuralPID, L1Adaptive, DynaCtrl (PID+SINDy MBRL), CEM.

**Key caveat:** Pump efficiency formula is `ratio*(2-ratio)` (standard parabolic BEP), not `ratio*(1-ratio)`. Negative plant gain: `m_dot_w = kMwNom - ctrl.compute(e)`. CEM uses B=-b·Ts (negative) in deviation state form.

---

### Porous Fiber Plate Humidification System ✅ C++

| Field | Value |
|-------|-------|
| **Plant** | Laminar flat-plate Sherwood evaporative model + first-order room moisture ODE + 2-step sensor delay |
| **Reference** | Ye et al. 2024 |
| **Controllers** | 15 |
| **Scenarios** | 5 |
| **Total runs** | 75 (15×5) |
| **Build target** | `humidification_sim` |
| **Logs** | `case-study/Porous Fiber Plate Humidification System/logs/` |

**Controller roster:** PID, PID_AW, FFPID, Cascade, GainSched, Smith, ADRC, MPC, MRAC, GPC_RLS, DynaMBRL, ILC (two-phase N_trial=80), NeuralPID (lr=1e-6, delta-fan), L1Adaptive (a_m=0.9512, absolute fan), CBFSafety (barrier φ_max=0.80, g=1.639e-5).

---

### Active Suspension Mathematical Modeling and Optimization 2025 ✅ C++

| Field | Value |
|-------|-------|
| **Plant** | 2-DOF quarter-car (z_s, dz_s, z_u, dz_u), 4-state RK4, F_act ±2000 N |
| **Reference** | Abdulwahab et al. 2025 (Alexandria Engineering Journal) |
| **Controllers** | 15 |
| **Scenarios** | 5 |
| **Total runs** | 75 (15×5) |
| **Build target** | `susp_sim` |
| **Logs** | `case-study/Active Suspension Mathematical Modeling and Optimization 2025/logs/` |

**Controller roster:** Passive, PID, ADRC, SMC, LQR (Bryson), LQG, MPC (2-state body SS), MRAC, FuzzyPID, TubeMPC, ILC (two-phase N_trial=1000), CBFSafety (barrier on dz_s), L1Adaptive (a_m=exp(-4·Ts)), ScenarioMPC (Np=10, Nu=3, Sigma_w=wheel noise), DynaCtrl (PID+SINDy MBRL).

**Scenarios:** s1_step_bump, s2_sine_resonance, s3_rough_road, s4_speed_bump, s5_compound.

**CSV columns:** `t, z_r, z_s, dz_s, z_u, dz_u, F_act, defl_susp, defl_tyre, accel_body`.

---

### Non-Inverting Buck-Boost Converter ✅ C++

| Field | Value |
|-------|-------|
| **Plant** | Averaged 2-state (i_L, v_C) RK4 at 50 kHz (Ts=20 µs), mode hysteresis ±0.1 V |
| **Reference** | Almasi et al. 2017, ISA Transactions 67, 515–527 |
| **Controllers** | 12 |
| **Scenarios** | 5 |
| **Total runs** | 60 (12×5) |
| **Build target** | `buck_boost_sim` |
| **Logs** | `case-study/Non-Inverting Buck-Boost Converter/logs/` |

**Controller roster:** OpenLoop, PI-Buck, PI-Boost, TLCS-ClassicPI, FuzzyPD, FuzzyPID-Buck, FuzzyPID-Boost, TLCS-FuzzyPI, GainScheduled (V_in/V_ref scheduling), ADRC, MPC, LQR.

**Scenarios:** s01_buck (8 V), s02_boost (15 V), s03_crossing_up (8→15 V), s04_crossing_down (15→4 V), s05_full (8→15→4 V).

**CSV columns:** `t, v_in, v_ref, v_out, i_L, d, mode, error`.

**Key caveats:**
- Folder name must be ASCII hyphens (U+002D) — CMake fails on Unicode dashes.
- Mode hysteresis: BUCK→BOOST when `V_ref > V_in + 0.1`; BOOST→BUCK when `V_ref < V_in - 0.1`.
- FuzzyPID inner bounds for buck-boost must be `±1.0` (not `[0,1]`) to allow overshoot suppression.
- TLCS bumpless transfer: call `inactive_ctrl.bumplessInit(d_active, e)` every step.

---

### Solar Cooker with Reflector and Absorber ✅ C++

| Field | Value |
|-------|-------|
| **Plant** | 2-state absorber+pot ODE ([T_abs, T_coll] °C) with PCM effective heat capacity; RK4 N_SUBSTEPS=10, Ts=30 s |
| **Reference** | Tarhan & Sari 2010 (Energy Conversion and Management) |
| **Controllers** | 12 |
| **Scenarios** | 5 |
| **Total runs** | 60 (12×5) |
| **Build target** | `solar_cooker_sim` |
| **Logs** | `case-study/Solar Cooker with Reflector and Absorber/logs/` |

**Controller roster:** OpenLoop, PID, ADRC, MPC, FuzzyPID, SMC, GainScheduled, MRAC, L1Adaptive, NeuralPID, DynaCtrl, ScenarioMPC.

**Scenarios:** s01_clear_sky_tracking, s02_wind_disturbance, s03_pcm_charge_discharge, s04_cloudy_morning, s05_overtemp_protection.

**CSV columns:** `t, T_ref, T_pot, T_abs, G_b, T_amb, V_wind, phi_pcm, f_shade, Q_absorbed, iae_cumulative`.

**Key caveats:**
- **Sign convention:** Direct-acting — `e = T_pot - T_ref`; output `f_shade ∈ [0,1]` (negative-gain: more shade → less solar gain → lower T_pot). Error-based controllers produce positive output when T_pot > T_ref.
- **PCM model:** Effective heat capacity `C_eff = m_abs·cp_abs + m_pcm·λ_pcm/ΔT_pcm` inside melting band `[T_melt − ΔT/2, T_melt + ΔT/2]`; outside this band `C_eff = m_abs·cp_abs`. Avoids implicit solve in RK4.
- **RK4 sub-stepping:** N_SUBSTEPS=10 inner 3 s steps per Ts=30 s sample.
- **ADRC:** ω_o=0.013, ω_c=0.004, `ω_o·Ts = 0.39 < 0.5` ✓.
- **MPC:** FOPDT `a = exp(−Ts/600)`, `b = −K·(1−a)` (negative, reflects negative-gain plant). Deviation form around T_pot_nom=95°C.
- **NeuralPID:** `plant_gain = −0.002·Ts` (negative to match gradient direction for negative-gain plant).
- **SMC:** `compute(T_pot − T_ref)` — matches DiscreteSMC convention `compute(y − ref)`.
- **Overtemp guard:** Simulation runner forces `f_shade = 1.0` when `T_abs ≥ T_abs_max − 1.0°C` (T_abs_max=280°C).

---

### Solar Ocean Thermal Energy Conversion System ✅ C++

| Field | Value |
|-------|-------|
| **Plant** | 2-state collector+tank ODE ([T_h, T_coll] °C) + algebraic ORC map; Forward Euler, Ts=30 s |
| **Reference** | Gao et al. 2024 (Applied Thermal Engineering 245, 122776) |
| **Controllers** | 12 |
| **Scenarios** | 5 |
| **Total runs** | 60 (12×5) |
| **Build target** | `sotec_sim` |
| **Logs** | `case-study/Solar Ocean Thermal Energy Conversion System/logs/` |

**Controller roster:** OpenLoop, PID, ADRC, MPC, LQR, FuzzyPID, MRAC, L1Adaptive, GainScheduled, ScenarioMPC, DynaCtrl, NeuralPID.

**Scenarios:** s01_mppt_steady (G=800 W/m²), s02_irradiance_step (G: 800→400 at t=1800 s), s03_setpoint_step (T_h_ref: 54→72°C), s04_high_irradiance (G=1050 W/m²), s05_solar_ramp (sinusoidal G).

**CSV columns:** `t, T_h_ref, T_h, T_coll, T_c, G_b, m_dot_f_cmd, m_dot_wf_cmd, W_net, P_inlet, eta_th, delta_T_super, iae_cumulative`.

**Key caveats:**
- **Dual output:** All controllers return `CtrlOutput{m_dot_f, m_dot_wf}`. Single-output SISO controllers regulate T_h via m_dot_f; m_dot_wf is always set by pressure-constrained feedforward: `m_dot_wf = 0.9·(P_inlet_max − a0 − a1·T_h)/a2`.
- **Hard constraint:** `P_inlet = a0 + a1·T_h + a2·m_dot_wf ≤ 1.38 MPa` enforced by `m_dot_wf_max_safe(T_h)` before every plant step and ORC computation.
- **Sign convention:** Positive-gain plant — more m_dot_f → more collector heat → higher T_h. Error `e = T_h_ref − T_h`. MRAC/L1Adaptive use `setReference(T_h_ref)` then `compute(T_h)` → outputs absolute m_dot_f.
- **ADRC:** ω_o=0.013, ω_c=0.004, `ω_o·Ts = 0.39 < 0.5` ✓. b0 = fopdt_b(Ts) = K·(1−a), K=30°C/(kg/s), τ=600 s.
- **MPC:** FOPDT deviation form around T_h_nom=63°C; `a=exp(−Ts/600)`, `b=K·(1−a)>0` (positive-gain).
- **LQR:** 2-state Bryson design; gain K stored as `Eigen::MatrixXd` (1×2). `DiscreteLQR` used in constructor only to compute K_ — not stored as member.
- **ScenarioMPC:** 2-state model; Q is 1×1 (output weight, pp=1 since C=[1,0]), Sigma_w is 2×2 (state noise, n=2).
- **GainScheduled:** 3 schedule points at T_h = 50°C (aggressive), 63°C (nominal), 72°C (soft).
- **NeuralPID:** Deviation output; `m_dot_f = clamp(M_F_NOM + npid.compute(e), 0.1, 0.5)`.

---

---

## Python-Only Studies (2)

Discovered by `run.py` Phase 6 via `case-study/*/sim/main.py`. Not in `CMakeLists.txt` or `compile.bat`. Each `sim/` module sets `_ROOT` 4 levels up from `sim/` to locate `build/bindings`.

### Vertical Drill String Mathematical Review 2025 ✅ Python

| Field | Value |
|-------|-------|
| **Plant** | 2-DOF torsional drill string with Stribeck bit friction (stick-slip); states [phi (rad), omega_b (rad/s)], input = omega_t (top-drive rad/s) |
| **Integration** | RK4, Ts = 0.1 s |
| **Controllers** | 17 |
| **Scenarios** | 5 |
| **Total runs** | 85 (17×5) |
| **Entry point** | `case-study/Vertical Drill String Mathematical Review 2025/sim/main.py` |
| **Logs** | `case-study/Vertical Drill String Mathematical Review 2025/logs/` |

**Controller roster:** OpenLoop, PID, ADRC (ω_o=3.0, ω_o·Ts=0.30 < 0.5 ✓), SMC, LQR (full-state), MPC (ZOH linearised at ω_nom=10), MRAC, GainScheduled (2 PIDs blended on |ω_ref|), L1Adaptive, NeuralPID, ILC (two-phase, N_trial=300), DynaCtrl (PID+SINDy MBRL), CEMCtrl (CEM-MPC linear model), ScenarioMPC (stochastic MPC), KoopmanMPC (EDMD+MPC warmup), ESNCtrl (reservoir predictor), CBFSafety (barrier h=ω_max−ω_b).

**Scenarios:** s01_step_ref (0→10 rad/s at t=5 s), s02_slow_ramp (2→18 over 60 s), s03_stick_slip (0→4 rad/s, low-speed), s04_high_speed (0→20 rad/s), s05_reversal (10→−5 at t=25 s).

**CSV columns:** `time, omega_ref, omega_b, phi, omega_t, error, iae_cumulative`.

**Key caveats:**
- LQR equilibrium: `x_ref = [phi_ss(omega_ref), omega_ref]` where `phi_ss = T_bit(omega_ref)/k_t`; total control = `u_ss + lqr.compute(x, x_ref)[0]`.
- `DiscretePID(pp, Ts)` — Ts is a constructor arg, NOT a field of `PIDParams`.
- `GainScheduledController(Ts)` — Ts required in constructor.
- MRAC/L1: `set_reference(r)` then `compute(y_plant)`.

---

### Multi-Body Floating Wind-Wave Platform ✅ Python

| Field | Value |
|-------|-------|
| **Plant** | 4-state FOWT heave + WEC arm (z, zdot, x_rel, xrel_dot) driven by sinusoidal wave F_wave(t); input = F_PTO |
| **Integration** | RK4, Ts = 0.5 s |
| **WEC resonance** | T = 10 s (K_w = 7.896×10⁴ N/m) |
| **Controllers** | 16 |
| **Scenarios** | 5 |
| **Total runs** | 80 (16×5) |
| **Entry point** | `case-study/Multi-Body Floating Wind-Wave Platform/sim/main.py` |
| **Logs** | `case-study/Multi-Body Floating Wind-Wave Platform/logs/` |

**Controller roster:** Passive (B_opt damping), Reactive (cancel stiffness + optimal damping), PID, ADRC (ω_o=0.8, ω_o·Ts=0.40 < 0.5 ✓), SMC, LQR (4-state), MPC (ZOH linearised), MRAC, L1Adaptive, ILC (two-phase, N_trial=300), DynaCtrl (PID+SINDy MBRL), CEMCtrl (CEM-MPC 4-state), ScenarioMPC (stochastic 4-state), KoopmanMPC (EDMD+MPC warmup), ESNCtrl (reservoir predictor), CBFSafety (barrier h=v_max−ẋ_rel).

**Scenarios:** s01_regular_waves (T=10 s, H=2 m), s02_storm_waves (T=12 s, H=5 m), s03_short_period (T=7 s, H=1 m), s04_irregular_wave (bi-chromatic 10 s+6 s), s05_freq_change (T steps 10→14 s at t=150 s).

**CSV columns:** `time, x_ref, z, zdot, x_rel, xrel_dot, F_pto, power, fowt_rms_cumul`.

**Key caveat:** ADRC ω_o constraint at Ts=0.5 s requires ω_o < 1.0.

---

## Spec-Only Stubs (9)

`README.md` (or PDF) present; no `sim/`, not registered, not built. To promote a stub to a C++ study: add `sim/{include,src}/`, a per-study `CMakeLists.txt`, an `add_subdirectory` line in `case-study/CMakeLists.txt`, and the target in `compile.bat`. To promote to Python-only: add `sim/main.py` following the Drill String pattern.

> **Recently graduated (Part 43):** Solar Cooker with Reflector and Absorber → `solar_cooker_sim` (60 runs). Solar Ocean Thermal Energy Conversion System → `sotec_sim` (60 runs).

---

### Electrostatic MEMS with Tilted Micro-Pillars 🔲 Stub

| Field | Value |
|-------|-------|
| **Plant** | Electrostatic actuator with tilted micro-pillars; nonlinear capacitance force model |
| **Readiness** | High — full plant equations and controller roster in README |
| **Target runs** | 10 controllers × 5 scenarios = 50 |
| **Pattern** | C++ study following Active Suspension template |

- **Status:** Full controller roster + plant equations in README. All that is needed is `sim/{include,src}/` implementation.
- **Blocker:** None — spec is complete.

---

### Bubble Column Bioreactor CO2 Biodiesel 🔲 Stub

| Field | Value |
|-------|-------|
| **Plant** | Bubble column reactor for CO₂-to-biodiesel conversion; gas-liquid mass transfer dynamics |
| **Readiness** | Low — thin spec, plant ODE not yet defined |
| **Target runs** | TBD |
| **Pattern** | C++ or Python-only study |

- **Status:** Thin spec. Plant model (mass transfer coefficients, gas hold-up, lipid accumulation ODE) needs to be designed before the controller roster can be built.
- **Blocker:** Plant model design required first.

---

### High-Altitude Aerial Firefighting Bag Drop 🔲 Stub

| Field | Value |
|-------|-------|
| **Plant** | Aerodynamic drop trajectory model; wind disturbances, altitude, bag release timing |
| **Readiness** | Low — thin spec, non-standard control problem |
| **Target runs** | TBD |
| **Pattern** | Monte-Carlo Python study (not a classic tracking loop) |

- **Status:** Thin spec. The task is stochastic trajectory planning (optimal drop timing under uncertain wind), not a reference-tracking feedback loop. Controller metrics would be CEP (circular error probable), not IAE.
- **Blocker:** Problem formulation and plant model need to be defined. Consider CEM/ScenarioMPC as the primary algorithms.

---

### Floating Nuclear Power Plant Ice Load Sensing 🔲 Stub

| Field | Value |
|-------|-------|
| **Plant** | Floating platform under ice loading; structural dynamics + mooring forces |
| **Readiness** | Low — thin spec, estimation-focused not tracking-focused |
| **Target runs** | TBD |
| **Pattern** | C++ or Python estimation study (EKF/MHE showcase) |

- **Status:** Thin spec. The focus is ice-load **estimation** (EKF, MHE, particle filter) rather than output tracking. Controller metrics would be estimation RMSE and latency, not IAE.
- **Blocker:** Plant model and sensor model need to be defined. Estimation loop is non-standard for this codebase's IController interface.

---

### Dust Control of Ultrasonic Dry Fog Nozzle 🔲 Stub

| Field | Value |
|-------|-------|
| **Plant** | Ultrasonic dry fog atomiser; droplet collision model; output = dust suppression efficiency η |
| **Reference** | Wang et al. 2026, Powder Technology 476, 122382 |
| **Readiness** | Medium — README present with full plant equations |
| **Target runs** | TBD (suggested: 10 controllers × 5 scenarios = 50) |
| **Pattern** | Python-only study (quasi-static efficiency model; no fast dynamics) |

- **Status:** README written (Part 41 rewrite). Plant: Sauter diameter `d_32 = C1·(σ/(ρ_l·f²))^(1/3)`, Stokes-number collision efficiency `η_c`, overall efficiency `η = 1 − exp(−K·n_d·A_d·η_c·L)`. Control variable: water flow rate + ultrasonic frequency to maximise η while minimising water consumption.
- **Blocker:** Control problem formulation — the plant is a static efficiency map, not an ODE tracking loop. Needs a dynamic disturbance model (particle concentration vs. time) to become a closed-loop study.

---

### Modular Convection-Enhanced Evaporation System 🔲 Stub

| Field | Value |
|-------|-------|
| **Plant** | Falling-film brine evaporator; states [T_w, T_a, ω_a, C_s, m_w] per module |
| **Reference** | Kaddoura et al. 2021, Desalination 510, 115057 |
| **Readiness** | Medium — README present with state equations |
| **Target runs** | TBD (suggested: 10 controllers × 5 scenarios = 50) |
| **Pattern** | Python-only study |

- **Status:** README written (Part 41 rewrite). Plant couples water-film temperature, air temperature, air humidity ratio, salt concentration, and film flow rate across stacked modules. Disturbances: ambient temperature, inlet brine salinity, air flow rate.
- **Blocker:** Multi-state ODE needs to be discretised and validated numerically before a controller roster can be built. Suggested primary controllers: PID (brine concentration), MPC (multi-variable air + liquid management), MRAC.

---

### Separate Meter-In Separate Meter-Out Hydraulic System 🔲 Stub

| Field | Value |
|-------|-------|
| **Plant** | Double-acting hydraulic cylinder with independent meter-in/meter-out proportional valves; states [x_p, v_p, P_A, P_B] |
| **Reference** | Chen et al. 2018, Control Engineering Practice 72, 138–150 |
| **Readiness** | Medium — README present; original C++ `sim/` was deleted (commit 37a17ef) |
| **Target runs** | TBD (suggested: 10 controllers × 5 scenarios = 50) |
| **Pattern** | C++ study (physical hydraulic dynamics, fast Ts) |

- **Status:** README written (Part 41 rewrite). Plant: bulk-modulus pressure dynamics for both chambers, spool-position-dependent valve flow, nonlinear friction. Paper's primary contribution is indirect adaptive robust dynamic surface control.
- **Blocker:** `sim/` was deleted in commit `37a17ef` ("remove a case study for now"). To restore: implement `sim/{include,src}/` following the Active Suspension template. Original implementation history is in `37a17ef^` if needed.

---

### Tracking Control of Electro-Hydraulic Force Servo Systems 🔲 Stub

| Field | Value |
|-------|-------|
| **Plant** | Servo-valve → hydraulic cylinder → load cell force; states [F, x_p, v_p, P_A, P_B, x_v] |
| **Reference** | Shen et al. 2017, ISA Transactions 67, 356–370 |
| **Readiness** | Medium — README present with plant equations |
| **Target runs** | TBD (suggested: 10 controllers × 5 scenarios = 50) |
| **Pattern** | C++ study |

- **Status:** README written (Part 41 rewrite). Plant: 4/3 servo valve with spool dynamics, double-acting actuator with compliance load (`k_L`), force measurement via load cell. Paper algorithms: PI + H∞ ODFC + nLMS adaptive feedforward. The H∞/ODFC component requires `DiscreteHinf` from `lib/`.
- **Blocker:** H∞ offline feedback control design (ODFC) depends on a plant model identified offline — requires a system identification step before the C++ simulation can be built.

---

### Data-Driven Sliding Mode Control of Soft Robot 2024 🔲 Stub

| Field | Value |
|-------|-------|
| **Plant** | Continuum soft robot module (205 mm); cable tendons + McKibben muscles; end-effector 3D position via SINDYc model |
| **Reference** | Papageorgiou et al. 2024, Control Engineering Practice 144, 105836 |
| **Readiness** | Medium — README present with SINDYc identification procedure |
| **Target runs** | TBD (suggested: 10 controllers × 5 scenarios = 50) |
| **Pattern** | Python-only study (SINDy identified model; 40 Hz Ts=0.025 s) |

- **Status:** README written (Part 41 rewrite). No first-principles ODE — plant model is a SINDYc sparse polynomial identified from data: `ẋ = Ξ·Θ(x, u)`. Control algorithms: data-driven Super-Twisting SMC (STSMC), SMC, MPC. SINDy class in `lib/` is available for the identification step.
- **Blocker:** Simulated training data needs to be generated from a surrogate (or hand-crafted) nonlinear model before SINDYc can be run and the identified model used in closed loop.

---

## Implementation Notes (All Studies)

- `DiscreteLQR` is **not** an `IController`. For `GainScheduledController` `design_fn`, use `makeLQRController(sys, lqr_params, state_fn)` which returns `shared_ptr<IController>`.
- `LQRWeightTuner::brysonMethod` is in `lib/ControllerTuner.h` — include it explicitly in case-study TUs that do not use the umbrella `ControllerToolbox.h`.
- `RecursiveLeastSquares`: accessor is `params()` (not `theta()`); `update(y, u)` is **output first, input second**.
- `compile.bat` lists every C++ target explicitly. A missing target silently runs a stale `.exe`. Current C++ case-study targets: `boiler_sim, tug_sim, solar_cooling_sim, humidification_sim, susp_sim, buck_boost_sim, solar_cooker_sim, sotec_sim`.
- Each `main.cpp` hard-codes the controller count — bump it when adding a controller.
