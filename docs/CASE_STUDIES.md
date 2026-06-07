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

## C++ Built Studies (6)


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

## Spec-Only Stubs (4)

`README.md` present; no `sim/`, not registered, not built.

### Electrostatic MEMS with Tilted Micro-Pillars 🔲 Stub

- **Status:** Full controller roster + plant equations in README. Target: 10 × 5 = 50 runs.
- **Pattern:** Implement as C++ study following Active Suspension template.

### Bubble Column Bioreactor CO2 Biodiesel 🔲 Stub

- **Status:** Thin spec. Plant model needs to be designed before controller roster can be built.

### High-Altitude Aerial Firefighting Bag Drop 🔲 Stub

- **Status:** Thin spec. Monte-Carlo drop pattern — not a classic reference-tracking loop; task is stochastic trajectory planning.

### Floating Nuclear Power Plant Ice Load Sensing 🔲 Stub

- **Status:** Thin spec. Focus is ice-load **estimation** (EKF/MHE showcase), not output tracking.

---

## Implementation Notes (All Studies)

- `DiscreteLQR` is **not** an `IController`. For `GainScheduledController` `design_fn`, use `makeLQRController(sys, lqr_params, state_fn)` which returns `shared_ptr<IController>`.
- `LQRWeightTuner::brysonMethod` is in `lib/ControllerTuner.h` — include it explicitly in case-study TUs that do not use the umbrella `ControllerToolbox.h`.
- `RecursiveLeastSquares`: accessor is `params()` (not `theta()`); `update(y, u)` is **output first, input second**.
- `compile.bat` lists every C++ target explicitly. A missing target silently runs a stale `.exe`. Current C++ case-study targets: `boiler_sim, tug_sim, solar_cooling_sim, humidification_sim, susp_sim, buck_boost_sim`.
- Each `main.cpp` hard-codes the controller count — bump it when adding a controller.
