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

## C++ Built Studies (9)


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

### Separate Meter In Separate Meter Out ✅ C++

| Field | Value |
|-------|-------|
| **Plant** | SMISMO hydraulic cylinder: 8 states [x_L, v_L, P_1, P_2, xv_1, dxv_1, xv_2, dxv_2]; two independent PDCVs (4-quadrant orifice flow), 2nd-order spool dynamics, identified Stribeck friction; RK4 Ts=1 ms, 4 substeps |
| **Reference** | Chen et al. 2018 (Control Engineering Practice 72, 138–150) + Liu et al. 2009 (IEEE/ASME AIM, 227–232) |
| **Controllers** | 12 |
| **Scenarios** | 5 |
| **Total runs** | 60 (12×5) |
| **Build target** | `smismo_sim` |
| **Logs** | `case-study/Separate Meter In Separate Meter Out/logs/` |

**Controller roster:** PID, CascadePID, LQR, LQG, MPC, ADRC (ω_o=200, ω_o·Ts=0.2 < 0.5 ✓), SMC, FeedbackLinearisation, TubeMPC, L1Adaptive, GainScheduled (on v_L), NonlinearMPC (RTI, internal 10 ms model step).

**Scenarios:** s01_resistive_step (+500 N), s02_overrunning (−800 N, backpressure braking), s03_sine_tracking (paper trajectory `0.25+0.25·sin(πt/2−π/2)`), s04_load_step (paper 500 N disturbance at t=9 s), s05_energy_compare (E = ∫P_s·Q_s dt metric).

**CSV columns:** `t, x_ref, x_p, v_p, P1_bar, P2_bar, u1, u2, F_ext, Q_s_lpm, energy_J, iae_cumulative`.

**Key caveats:**
- **Dual-loop structure (Liu Fig. 10):** controllers output one scalar working-side command u_ctrl [V]; the shared `ValveAllocator` does mode selection (±0.05 V hysteresis) and regulates the off-side chamber to P_bd=20 bar with flow-matching feedforward + PI.
- **Valve normalisation:** `Q_i = xv_i·Q_nom_i·sqrt(DP_i/3.5 MPa)` with normalised spool xv ∈ [−1,1] (equivalent to the Cd·W·x_v form; Q_nom2/Q_nom1 = k_v2/k_v1 = 0.67).
- **Working-side gain for tuning:** v/u ≈ 0.14 (m/s)/V, velocity lag τ_v ≈ 25 ms → 2-state design model shared by LQR/LQG/MPC/TubeMPC.
- **Friction:** Chen Stribeck law is discontinuous at v=0; regularised linearly inside |v| < 5e-3 m/s.
- **sqrt(DP) regularisation:** orifice uses `dp/sqrt(|dp|+1e3)` (signed, smooth at 0, allows reverse anti-cavitation flow).
- **Backpressure is the cavitation guard:** the overrunning s02 scenario stays cavitation-free because the off-side PI holds 20 bar — do not lower P_bd below ~5 bar.

---

## Python-Only Studies (6)

Discovered by `run.py` Phase 6 via `case-study/*/sim/main.py`. Not in `CMakeLists.txt` or `compile.bat`. Each `sim/` module sets `_ROOT` 3–4 levels up from `sim/` to locate `build/bindings`.

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

### Tracking Control of Electro-Hydraulic Force Servo Systems ✅ Python

| Field | Value |
|-------|-------|
| **Plant** | 5-state EHFS: [P_A, P_B, x_v, v_p, x_p]; 4/3 servo valve with dead-band, double-acting cylinder, Coulomb + viscous friction, spring load k_L |
| **Reference** | Shen et al. 2017, ISA Transactions 67, 356–370 |
| **Integration** | RK4, Ts = 0.5 ms, N_SUBSTEPS = 4 |
| **Controllers** | 12 |
| **Scenarios** | 5 |
| **Total runs** | 60 (12×5) |
| **Entry point** | `case-study/Tracking Control of Electro-Hydraulic Force Servo Systems/sim/main.py` |
| **Logs** | `case-study/Tracking Control of Electro-Hydraulic Force Servo Systems/logs/` |


**Controller roster:** OpenLoop, PID (Kp=3e-4, Ki=0.5), ADRC (ω_o=800, ω_o·Ts=0.40 < 0.5 ✓, b0=K_F·Ts·(1−e^{−Ts/τ_v})), SMC (compute(F−F_ref)), MPC (2-state [F, x_v] ZOH, Np=20), LQR (2-state deviation-form, Q=diag(1e-8,1)), MRAC (a_m=e^{−300·Ts}, γ_r=1e-9), L1Adaptive (ω_c=200, Γ=1e-5), FeedbackLin (K_xv(P_A,P_B) inversion, λ=300 rad/s), NeuralPID (n_h=8, lr=1e-8), ILC (P-type N_trial=800, Lp=0.6), GainScheduled (2 PIDs on |F_ref|, breakpoint at 5 kN).

**Scenarios:** s01_sine_50hz (±500 N, 50 Hz — high-bandwidth test), s02_sine_5hz (±5 kN, 5 Hz — high-amplitude), s03_step (0→8 kN at t=0.05 s — transient), s04_stiffness_change (±2 kN @ 2 Hz; k_L 3e6→2e7 N/m at t=0.5 s — specimen yielding), s05_earthquake (chirp 0.5→15 Hz ±8 kN — broadband/HiL fidelity).

**CSV columns:** `t, F_ref, F, x_p (mm), v_p (mm/s), P_A_bar, P_B_bar, u_v, phase_error_deg, iae_cumulative`.

**Key caveats:**
- **ADRC ω_o constraint:** ω_o·Ts = 800·5e-4 = 0.40 < 0.5 ✓. b0 computed from linearised flow gain K_F at nominal pressures.
- **FeedbackLin gain guard:** K_xv clamped to ≥ 1e4 near valve closure to prevent division by zero.
- **MRAC/L1 scaling:** force signals are O(1e4 N) vs. u_v ∈ [−1,1]; adaptation gains are very small (γ_r=1e-9) to avoid instability.
- **Dynamic stiffness (s04):** `plant.k_L` is overridden per step in `simulation_runner.py`. Adaptive controllers (MRAC, L1, FeedbackLin) outperform fixed-gain designs here.
- **Cavitation guard:** P_A, P_B clamped to ≥ 0 after each RK4 substep.
- **ILC:** converges on periodic s01/s02; behaves as PID on non-periodic s03/s05.

---

### High-Altitude Aerial Firefighting Bag Drop ✅ Python

| Field | Value |
|-------|-------|
| **Plant** | 3D translational bag trajectory: [x, y, z, vx, vy, vz]; aerodynamic drag + gravity + wind disturbance |
| **Reference** | Sun et al. 2025, Results in Engineering 27, 105940 |
| **Integration** | RK4, Ts = 0.05 s |
| **"Controllers"** | 12 drop planners |
| **Scenarios** | 5 |
| **Total runs** | 60 (12×5) |
| **Primary metric** | CEP [m] (50th-percentile radial impact error) — NOT IAE |
| **Entry point** | `case-study/High-Altitude Aerial Firefighting Bag Drop/sim/main.py` |
| **Logs** | `case-study/High-Altitude Aerial Firefighting Bag Drop/logs/` |

**Planner roster:** NominalDrop (no correction), WindOffset (simulation-based drift compensation), IterativeRefinement (ILC-style cross-run refinement), MCPredictor (internal 12-sample MC centroid), GPSurrogate (GP on 3×3 wind grid — uses `ctrl.GaussianProcess`), BayesOptDrop (BayesianOptimizer over (x,y) offset — uses `ctrl.BayesianOptimizer`), SIRParticleFilter (60-particle SIR, drift-sensitivity correction), KalmanWind (1-step Kalman on wind estimate), RobustMinMax (worst-case sigma-margin correction), AdaptiveRLS (recursive LS wind-drift model, cross-run adaptation), EnsembleConsensus (weighted average of WindOffset+MC+Robust), ProfileAdaptive (altitude-scaled safety margin).

**Scenarios:** s01_no_wind (h=90 m, σ=0.3), s02_crosswind (h=90 m, wy=5 m/s, σ=1.0), s03_headwind (h=90 m, wx=−8 m/s, σ=0.8), s04_turbulent (h=90 m, wx=2/wy=3 m/s, σ=3.0), s05_high_altitude (h=120 m, wx=3/wy=4 m/s, σ=1.5).

**CSV columns:** `sample_id, x_impact, y_impact, t_flight, wx_true, wy_true, error_x, error_y, radial_error`. One row per Monte Carlo sample. Summary row (CEP, P95, L_pat, W_pat) appended at end of each CSV.

**Key caveats:**
- **Non-standard problem:** This is a Monte Carlo drop-planning study, not a feedback tracking loop. CEP replaces IAE as the primary figure of merit. Pattern width (95th-pct |y_impact|) and pattern length (80th−20th pct x_impact) are secondary metrics.
- **Drag coupling:** Forward speed (V_aircraft=50 m/s) dominates the drag force; lateral drift is NOT wy·t_fall — it is much smaller (≈ wy/(D/m) × lag). All planners that use wind compensation must use `_drift_sensitivity()` (two reference simulations) rather than the linear t_fall approximation.
- **SIR over-correction in no-wind:** SIRParticleFilter adds ~2 m CEP in no-wind because it applies a correction based on a noisy sensor reading even when true wind is zero. This is expected and realistic.
- **BayesOpt budget:** Uses n_init=4 + maxIter=10 = 14 evaluations, each with 6 MC trajectories. Total inner simulations ≈ 84 per run. Still fast (Ts=0.05 s, fall time ≈4-6 s).
- **GP training:** GPSurrogate trains on a 3×3 wind grid per run (9 points). At near-zero wind (s01), the GP correction is near zero, matching NominalDrop. At high wind the GP is slightly less accurate than MCPredictor because 9 training points are sparse.
- **_ROOT path:** `_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(sim_dir)))` — 3 levels up from `sim/` to project root. This differs from the 4-level path in DrillString/WindWave (which go through an extra level).

---

### Air-Cooled Battery Thermal Management System ✅ Python

| Field | Value |
|-------|-------|
| **Plant** | 1-D transient heat transfer model (Zhang et al. 2026): N=9 prismatic cells, 10 parallel channels, J/U/L flow-pattern switching; states [Tb×9, Ta_out×10]; air quasi-static, batteries forward-Euler |
| **Reference** | Zhang et al. 2026, Applied Thermal Engineering 298, 130921 |
| **Integration** | Forward Euler, Ts = 1.0 s (air channels quasi-static, fixed-point iteration each step) |
| **Controllers** | 12 |
| **Scenarios** | 5 |
| **Total runs** | 60 (12×5) |
| **Primary metric** | IAE of ΔT [K·s] (temperature non-uniformity integral) |
| **Entry point** | `case-study/Air-Cooled Battery Thermal Management System/sim/main.py` |
| **Logs** | `case-study/Air-Cooled Battery Thermal Management System/logs/` |

**Controller roster:** OpenLoop (fixed J), SelfAdaptive (paper's position rule, ΔT_thresh=0.92 K), PID (threshold adaptation), ADRC (ω_o=0.3, ω_o·Ts=0.30 < 0.5 ✓), SMC (DiscreteSMC on ΔT), MPC (1-step lookahead over J/U/L, native Python), LQR (1-state ΔT model), MRAC (a_m=exp(-1/120), τ=120 s), L1Adaptive (Γ=0.05, ω_c=0.08), NeuralPID (online gains, lr=5e-4), GainScheduled (2 PIDs on SOC), ILC (P-type, N_trial=720, one discharge cycle).

**Scenarios:** s01_5C_discharge (constant 5C, 720 s — paper Section 4.1), s02_varying_conditions (random 5–8×10⁴ W/m³, 3600 s — paper Section 4.2), s03_2C_steady (mild 2C, 1800 s), s04_high_rate_pulse (7C→2C at t=300 s, 900 s), s05_battery_aging (5C, R×1.5 aged cell, 720 s).

**CSV columns:** `t, DeltaT_ref, DeltaT, T_max, T_min, T_avg, flow_pattern, phi_b_kWm3, soc, n_switches, iae_cumulative`.

**Key caveats:**
- **Controller output maps to threshold:** ctrl_toolbox controllers output u ∈ [-0.5, 0.5] K; effective threshold = max(0.1, ΔT_lim + u). Only the paper's position rule (xmax ≤ L/4 → U, xmax ≤ 3L/4 → L, else J) determines which pattern to activate.
- **h scaling:** h[j] = h_ref × (Q[j]/Q_ref)^0.8 from the minimum-flow reference channel (J: 43.4, U: 34.2, L: 32.6 W/m²/K at respective reference channels).
- **Air quasi-static:** τ_air ~ 3 ms ≪ Ts = 1 s; Ta_out solved by fixed-point iteration each step (forward Euler on Ta would be unstable at Ts=1 s).
- **MPC native Python:** 1-step lookahead evaluates all 3 patterns via `copy.deepcopy` of the plant; no linearisation needed.
- **ADRC constraint:** ω_o·Ts = 0.3·1.0 = 0.30 < 0.5 ✓.

---

### Nonlinear Surface Ship Manoeuvring Control ✅ Python

| Field | Value |
|-------|-------|
| **Plant** | 3-DOF MMG manoeuvring model: states [u, v, r, psi, x, y]; 19 SRUKF-identified parameters from MARIN SIMMAN 2020 free-running tests; propeller + rudder inputs; RK4, Ts = 0.08 s |
| **Reference** | Meng et al. 2025, Ocean Engineering 321, 120432 |
| **Integration** | RK4, Ts = 0.08 s (12.5 Hz matching MARIN data rate) |
| **Controllers** | 12 |
| **Scenarios** | 5 |
| **Total runs** | 60 (12×5) |
| **Primary metric** | Mean position error [m] and IAE [m·s] |
| **Entry point** | `case-study/Nonlinear Surface Ship Manoeuvring Control/sim/main.py` |
| **Logs** | `case-study/Nonlinear Surface Ship Manoeuvring Control/logs/` |

**Controller roster:** OpenLoop (n=n_ss, delta=0), PID (heading PID + cross-track correction), SMC (1st-order sliding surface on psi_err), ASMC (paper's full cascade Adaptive SMC with disturbance feedforward — paper result), MPC (DiscreteMPC, linearized [psi,r] ZOH model, Np=20, Nc=5), LQR (Bryson DARE on [psi_e,r], Q=diag(16,4), R=3), MRAC (MRACController, a_m=−0.15, gamma=0.5), L1Adaptive (L1AdaptiveController, Gamma=10, omega_c=0.25), GainScheduled (3-point schedule on |psi_err|, p=[0.15,0.40,0.80] rad), ADRC (DiscreteADRC 2nd-order, b0=c6=0.4045, omega_o=1.5), NeuralPID (online gain adaptation, lr=1e-4), ILC (P-type, Lp=0.5, N=2500; first trial = zero feedforward).

**Scenarios:** s01_sine_trajectory (xd=20sin(0.07t), yd=t — paper Fig. 7), s02_straight_line (xd=0, yd=2t — course-keeping), s03_circle (R=25 m, omega=0.06 rad/s), s04_zigzag (S-curve xd=20sin(pi*t/30)), s05_disturbance (paper scenario + paper disturbance model).

**CSV columns:** `time, xd, yd, x, y, u, v, r, psi_deg, psi_d_deg, n_rps, delta_deg, pos_error, iae_cumulative`.

**Key caveats:**
- **ASMC dominates:** ASMC achieves ~90% lower IAE than PID on s01 (paper scenario). It includes exact disturbance feedforward so it also outperforms on s05 (with disturbances active).
- **ADRC constraint:** omega_o=1.5, Ts=0.08 → omega_o·Ts = 0.12 < 0.5 ✓.
- **alpha_v_dot approximation:** ASMC uses backward finite differences for virtual-control derivatives; one-step delay is acceptable for ship time constants (> 1 s).
- **ILC first-trial behavior:** ctrl_toolbox ILC runs in pure-feedforward mode; first trial feedforward = 0, so heading control is inactive. Speed loop (n command) still operates.
- **Equilibrium:** at u_ss=2 m/s, n_ss = sqrt(-a1*u_ss^2 / a5) = 10.72 rev/s = 643 RPM (verify: a1=−0.023, a5=0.0008).
- **GainScheduledController:** uses `add_schedule_point(p, ctrl)` + `set_scheduling_param(p)` before `compute(error)` — NOT the `addBracket` design-fn pattern used by C++ case studies.
- **Speed loop:** PI with n_ss bias tracks u_ref = speed of desired trajectory; Kp=2, Ki=0.5.

---

## Spec-Only Stubs (5)

`README.md` (or PDF) present; no `sim/`, not registered, not built. To promote a stub to a C++ study: add `sim/{include,src}/`, a per-study `CMakeLists.txt`, an `add_subdirectory` line in `case-study/CMakeLists.txt`, and the target in `compile.bat`. To promote to Python-only: add `sim/main.py` following the Drill String pattern.

> **Recently graduated (Part 43):** Solar Cooker with Reflector and Absorber → `solar_cooker_sim` (60 runs). Solar Ocean Thermal Energy Conversion System → `sotec_sim` (60 runs).
> **Recently graduated (Part 44):** Separate Meter In Separate Meter Out → `smismo_sim` (60 runs), reimplemented from both source PDFs (the pre-37a17ef sim was deleted).
> **Recently graduated (Part 45):** Tracking Control of Electro-Hydraulic Force Servo Systems → Python-only study (60 runs, 12 controllers × 5 scenarios).
> **Recently graduated (Part 46):** High-Altitude Aerial Firefighting Bag Drop → Python-only study (60 runs, 12 planners × 5 scenarios). Monte Carlo drop pattern analysis; primary metric is CEP [m] rather than IAE.
> **Recently graduated (Part 48):** Air-Cooled Battery Thermal Management System → Python-only study (60 runs, 12 controllers × 5 scenarios). Analytical transient heat transfer model with J/U/L flow-pattern switching; primary metric IAE of ΔT.
> **Recently graduated (Part 49):** Nonlinear Surface Ship Manoeuvring Control → Python-only study (60 runs, 12 controllers × 5 scenarios). Previously listed as MATLAB/Simulink-only stub; implemented in pure Python using SRUKF-identified Table 5 parameters. ASMC achieves ~90% lower IAE than PID on the paper trajectory.

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
- `compile.bat` lists every C++ target explicitly. A missing target silently runs a stale `.exe`. Current C++ case-study targets: `boiler_sim, tug_sim, solar_cooling_sim, humidification_sim, susp_sim, buck_boost_sim, solar_cooker_sim, sotec_sim, smismo_sim`.
- Each `main.cpp` hard-codes the controller count — bump it when adding a controller.
