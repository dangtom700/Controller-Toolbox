# Controller Toolbox - Compact Reference: Parts 26-50

**Covers:** Parts 26-50 (2026-05-31 through 2026-06-11)
**Full history:** See git log; original reports archived in `docs/cumulative_bug_report.md` (now reset for Part 51+).
**Purpose:** Quick-reference for algorithm inventory, critical caveats, and tribal knowledge
accumulated over Parts 26-50. Use this when the full cumulative report is too large to read;
read `docs/cumulative_bug_report.md` (Part 51+) for active issues.

---

## 1. Project State (as of Part 50)

- **C++ executables / Catch2 tests:** UNVERIFIED — run `conda run -n soft_robotics -- python run.py` to confirm current counts (estimated ~174 C++ passing, 0 failing)
- **Python examples:** UNVERIFIED (estimated ~103 passing, 0 failing)
- **Case study C++ runs:** Boiler 216 + Tug 72 + Solar 70 + Humid 75 + ActiveSusp 75 + BuckBoost 60 + SolarCooker 60 + SOTEC 60 + SMISMO 60 = **748 C++ runs** (9 studies × scenarios)
- **Python-only studies (Phase 6):** DrillString 85 + WindWave 80 + EHFS 60 + Firefighting 60 + BTMS 60 + SurfaceShip 60 = **405 runs** (6 studies, Parts 38-49)
- **`bug_report.txt`:** Expected 0 blocks after a clean run; safe_phrases list at 37+ entries suppresses all known benign messages
- **test_smismo_regression:** Recreated in Part 44 (6 tests against new SMISMO sim); UNVERIFIED until next clean build
- **Part 50:** Planning pass only — no new code. Phase 2 roadmap (E1-E4, H1-H4, D1-D2) documented in `docs/ALGORITHM_ROADMAP_PHASE2.md`

---

## 2. Algorithm Additions (Parts 26-50)

No new `lib/` algorithms were added in Parts 45-50 (case studies and planning only).
All modules below were added in Parts 26-44 and are fully implemented, pybind11-bound, smoke-tested, and have Catch2 tests.

### New Data-Driven / ML Controllers (Parts 30-33)

| Class | Header | Part | Notes |
|-------|--------|------|-------|
| `DeePC` | DeePC.h | 30 | ADMM Hankel-QP (Coulson 2019); `CTRL_REGISTER_FEATURE(deepc)` removed — false registration (B2 fix, Part 39) |
| `ILCController` | IterativeLearningControl.h | 31 | P-type, D-type, norm-optimal; stores previous-trial u & e arrays |
| `SINDy` / `SINDyModel` | SINDy.h | 31 | STLS sparse regression; PolyDeg1/2/3+Trig library; training data must have varied `u` |
| `KoopmanEDMD` | KoopmanEDMD.h | 31 | EDMD lift → `ctrl::StateSpace`; `A.rows()==nLifted-n_input` |
| `L1AdaptiveController` | L1AdaptiveController.h | 31 | State predictor + LP-filtered adaptation; `compute(y_plant)` NOT error |
| `CBFSafetyFilter` | CBFSafetyFilter.h | 31 | 1D analytical QP wrapping any `IController`; approximate (ignores plant coupling) |
| `GaussianProcess` | GaussianProcess.h | 31 | SE kernel, Cholesky inference, fixed-budget FIFO eviction (N≤200) |
| `EchoStateNetwork` | EchoStateNetwork.h | 31 | Spectral-radius-scaled W_res; ridge-regression readout; `reset()` preserves `W_out_` / `fitted_` |
| `NeuralPID` | NeuralPID.h | 31 | 3→n_h→3 fully-connected, online backprop; softplus gains; 3→8→3 default |
| `CEMController` | CEMController.h | 31 | Elite-sample stochastic rollout MPC; warm-start mu; `computeRef(x, y_ref)` |
| `DynaController` | DynaController.h | 33 | Sutton Dyna MBRL wrapping any `IController`; SINDy error-dynamics fit; `modelRollout(e0, u_seq)` |
| `ScenarioMPC` | ScenarioMPC.h | 33 | N_s-scenario noise-averaged QP; H constant/precomputed per episode; mirrors TubeMPC API |
| `BayesianOptimizer` | BayesianOptimizer.h | 33 | GP surrogate + UCB/EI acquisition; header-only; shares `TunerResult`/`CostFn` with AutoTuner |

### New Infrastructure Modules (Parts 33-34)

| Class | Header | Part | Notes |
|-------|--------|------|-------|
| `ControllerRegistry` | ControllerRegistry.h | 33 | Meyers-singleton self-registration; `CTRL_REGISTER_FEATURE(name)` macro in headers (not .cpp) |
| `ControllerRegistrations` | ControllerRegistrations.h | 33 | Pre-M2 centralised entries; include AFTER all other lib/ headers in ControllerToolbox.h |
| `ControllerMonitor` | ControllerMonitor.h | 33 | CUSUM + EWMA SPC charts as `IControllerObserver`; ADRC emits `"eso"` z-vector, SMC emits `"surface"` |
| `LQRAdapter` / `makeLQRController()` | DiscreteLQR.h | 34 | `makeLQRController()` factory wraps DiscreteLQR + LQRAdapter into `shared_ptr<IController>`; use for `design_fn` callbacks instead of constructing DiscreteLQR directly |
| `ComputationalDelayWrapper` | ComputationalDelayWrapper.h | 34 | Header-only one-sample actuator delay decorator; NaN hold; `CTRL_REGISTER_FEATURE(computational_delay)` |

### Infrastructure Fixes (Part 34)

- **MIMO nu-gap (T2):** `GapMetric::subspaceDist()` via normalised-graph thin SVD; `nuGap()` now dispatches `chordalDist` (SISO) or `subspaceDist` (MIMO)
- **MHE state constraints (T4):** `MHEParams::xMin`/`xMax` box constraints on arrival state x_0; applied to `z[0:n]` FISTA block
- **LinearBlend bumpless (T5):** `GainScheduledController` calls `bumplessInit()` on any controller newly entering an active bracket
- **`compare_controllers.py` (T7):** `tools/compare_controllers.py` auto-discovers `case-study/*/logs/*.csv`; flags `--study/--scenario/--sort/--wide`

---

## 3. Critical Caveats / Tribal Knowledge (Parts 26-50)

These supplement `compact_bug_report_parts_1-25.md` Section 3. Both files must be read.

### New Sign Conventions (Parts 26-44)
```
L1AdaptiveController:  compute(y_plant) NOT compute(error) — same pattern as MRACController
CEMController:         computeRef(x, y_ref) — same pattern as NonlinearMPC/TubeMPC
DynaController:        wraps any IController; compute() delegates to inner controller
```

### API Gotchas (Parts 26-44)
```
makeLQRController():   factory wraps DiscreteLQR+LQRAdapter → shared_ptr<IController>;
                       use INSTEAD of constructing DiscreteLQR directly in design_fn callbacks
ComputationalDelayWrapper: output initialised to 0.0; first compute() returns 0 (held value),
                           not the fresh inner output — warm up one step before trusting output
ControllerRegistrations.h: must be included AFTER all other lib/ headers in ControllerToolbox.h
                            (Meyers-singleton map_() must exist before any addFeature() call)
CTRL_REGISTER_FEATURE macro: place in headers (not .cpp files); inline const bool fires per-include
DeePC:                CTRL_REGISTER_FEATURE(deepc) was removed — DeePC has no runtime registration
```

### Case-Study Caveats (Parts 26-44)
```
Solar Cooker MRAC gammas:  gamma_r = -0.01, gamma_y = -0.01 (NEGATIVE for negative-gain plant)
                           Positive gammas drive theta negative → u clamps to 0 → no control
Solar Cooker sign:         e = T_pot - T_ref (direct-acting); more f_shade → less solar → lower T_pot
Solar Cooker ADRC:         omega_o=0.013, Ts=30s → omega_o*Ts=0.39 < 0.5 (check)
Solar Cooker NeuralPID:    plant_gain = -0.002*Ts (negative, matches plant sign)
S-OTEC hard constraint:    P_inlet = a0+a1*T_h+a2*m_dot_wf <= 1.38 MPa; clamp before every plant step
S-OTEC ADRC:               omega_o=0.013, Ts=30s → omega_o*Ts=0.39 < 0.5 (check)
SMISMO backpressure:       P_bd=20 bar is cavitation guard for overrunning scenario;
                           do not lower P_bd below ~5 bar
SMISMO sign / gains:       SMC uses compute(y-ref); working-side v/u ~ 0.14 (m/s)/V, tau_v ~ 25 ms
SMISMO ADRC:               omega_o=200, Ts=1ms → omega_o*Ts=0.20 < 0.5 (check)
Buck-Boost FuzzyPID:       inner FuzzyPDParams.uMin/uMax must be +/-1.0 (loose);
                           using [0,1] for inner bounds kills overshoot suppression
Buck-Boost mode hysteresis: BUCK→BOOST when V_ref > V_in+0.1V; BOOST→BUCK when V_ref < V_in-0.1V
TLCS bumpless:             call inactive_ctrl.bumplessInit(d_active, e) every step (not just at switch)
```

### Python Binding Patterns (Parts 26-44)
```
Python MRAC / L1Adaptive: ctrl.set_reference(r) then ctrl.compute(y_plant) — NOT compute(r-y)
GainScheduledController:  constructor needs Ts: ctrl.GainScheduledController(Ts)
Python LQR reference:     compute x_ref=[phi_ss, omega_ref]; u = u_ss + lqr.compute(x, x_ref)[0]
MPCParams snake_case:     qp_max_iter (NOT camelCase qpMaxIter) in Python
NumPy 2.x:                float(np.squeeze(arr)) not float(arr) on 1-D arrays
```

### Python-Only Case Study Patterns (Parts 38-49)
```
ADRC omega_o constraint applies at all Ts values:
  Drill String (Ts=0.1s):  omega_o=3.0 → omega_o*Ts=0.30 < 0.5 (check)
  Wind-Wave (Ts=0.5s):     omega_o=0.8 → omega_o*Ts=0.40 < 0.5 (check)
  EHFS (Ts=0.5ms):         omega_o=800 → omega_o*Ts=0.40 < 0.5 (check)
  Surface Ship (Ts=0.08s): omega_o=1.5 → omega_o*Ts=0.12 < 0.5 (check)
Firefighting primary metric: CEP (50th-pctile radial error) replaces IAE
  _drift_sensitivity() NOT wy*t_fall — forward-speed drag coupling reduces lateral
  drift to ~40% of linear estimate at V=50 m/s
  SIRParticleFilter adds ~2 m CEP in zero-wind (noisy sensor applies correction at sigma=0.3)
Surface Ship ASMC:         paper cascade — outer ASMC for psi error with disturbance FF;
                           inner P loop for yaw rate r. 19 SRUKF-identified params (Meng 2025 Table 5)
Python-only study binding path: _ROOT = dirname(dirname(dirname(abspath(__file__))))
  (3 levels up from sim/ to project root)
```

---

## 4. Case Studies (Parts 26-50)

### C++ Studies (9 total, registered in CMakeLists.txt + compile.bat)

| Study | Target | Controllers | Runs | Key plant |
|-------|--------|-------------|------|-----------|
| Active Suspension | `susp_sim` | 15 | 75 | 2-DOF quarter-car, 4-state RK4; F_act ±2000 N |
| Non-Inverting Buck-Boost | `buck_boost_sim` | 12 | 60 | Averaged 2-state RK4 @ 50 kHz; mode hysteresis ±0.1 V |
| Solar Cooker | `solar_cooker_sim` | 12 | 60 | 2-state absorber+pot ODE + PCM effective-C, RK4 (Ts=30s) |
| S-OTEC | `sotec_sim` | 12 | 60 | 2-state collector+tank ODE, algebraic ORC, Forward Euler (Ts=30s) |
| SMISMO | `smismo_sim` | 12 | 60 | 8-state RK4 (Ts=1ms, 4 substeps), dual PDCVs + Stribeck friction |

*(Boiler 216, Tug 72, Solar-Driven Cooling 70, Humidification 75 were pre-Part 26)*

### Python-Only Studies (6 total, discovered by Phase 6 of run.py)

| Study | Part | Controllers | Runs | Key plant |
|-------|------|-------------|------|-----------|
| Vertical Drill String | 38 | 17 | 85 | 2-DOF torsional, Stribeck friction, RK4 (Ts=0.1s) |
| Multi-Body Wind-Wave | 38 | 16 | 80 | 4-state FOWT heave + WEC arm, RK4 (Ts=0.5s) |
| EH Force Servo (EHFS) | 45 | 12 | 60 | 5-state [P_A, P_B, x_v, v_p, x_p], RK4 (Ts=0.5ms) |
| Firefighting Bag Drop | 46 | 12 planners | 60 | 3D trajectory [x,y,z,vx,vy,vz], RK4 (Ts=0.05s); metric=CEP |
| Air-Cooled BTMS | 48 | 12 | 60 | 1-D transient HX, N=9 cells, J/U/L flow switching, Euler (Ts=1s) |
| Surface Ship Manoeuvring | 49 | 12 | 60 | 3-DOF MMG model, 19 SRUKF params, RK4 (Ts=0.08s) |

---

## 5. Open Items (Part 50 exit)

Phase 2 algorithm roadmap (E1-E4, H1-H4, D1-D2) is the primary active work. See `docs/ALGORITHM_ROADMAP_PHASE2.md` for full detail.

| ID | Description | Priority |
|----|-------------|----------|
| ~~**E1**~~ | ~~`GreyBoxEstimator`~~ — **Done Part 52** | HIGH |
| ~~**E2**~~ | ~~`RecursiveGreyBoxEstimator`~~ — **Done Part 52** | HIGH |
| ~~**E3**~~ | ~~GP Residual Model~~ — **Done Part 52** | MED |
| **E4** | MHE Polytopic Constraints — extend MHE with `C_ineq`/`d_ineq` beyond current box constraints | MED |
| **H1** | `HybridModel` base class — `IPlantModel` with `f_phys + f_data` | MED |
| **H2** | `HybridMPC` — `NonlinearMPC` variant using `HybridModel` for prediction | MED |
| **H3** | RL-MPC stitching Python example | LOW |
| **H4** | `HybridModelTrainer` — GP/NN hyperopt for `f_data` component | LOW |
| **D1** | Mismatch Detector — CUSUM on KF/MHE innovation sequence | LOW |
| **D2** | Digital Twin Lite Python app | LOW |
| **C2** | 6 spec-only stubs remain (BEMS + MEMS have no blocker; others need plant design) | MED |
| **B36-3** | Unify NaN-guard: `ctrl::sanitize()` dead code; ~12 controllers inline different guards | MED |
| R1 | Edge-case contract matrix: NaN-in, saturation, non-stabilizable for every controller family | MED |
| T3 | Full DK-iteration with vector-fitting rational D(jω) | LOW |
| B36-2 | `ex79_registry_monitor`: M3 telemetry demo mis-wired (monitor observes nothing) | LOW |
| REL | Rebuild `ctrl_toolbox.pyd` in Release to silence stale-.pyd QP warnings | LOW |
| M4 | `template<typename Scalar>` leaf algorithms IF embedded float target becomes real | Backlog |

---

## 6. Phase 2 Roadmap Summary (Part 50)

**Strategic direction:** Model Estimation → Hybrid Models → Digital Twin deployment.
Full implementation detail in `docs/ALGORITHM_ROADMAP_PHASE2.md`.

**Dependency order:**
```
E1 (GreyBoxEstimator) ──► E3 (GP Residual) ──► H4 (HybridModelTrainer)
                      └──► H1 (HybridModel) ──► H2 (HybridMPC) ──► H3 (RL-MPC)
E2 (RecursiveGreyBox) — wraps existing UKF, independent
E4 (MHE constraints) — independent FISTA extension
D1 (Mismatch Detector) — independent CUSUM extension
D2 (Digital Twin Lite) ──── requires E1 + D1
```

**Estimated effort:** ~17-21 days (part-time over 5-6 weeks).

**Deferred (not Phase 2):** FMU import/export, CasADi symbolic AD, full RL framework, control co-design.

---

## 7. Build & Run Reference

```bash
# Run everything (clean + compile + test C++ + test Python + case studies)
conda run -n soft_robotics -- python run.py

# Run individual Python scripts
conda run -n soft_robotics -- python examples/python/exNN_name.py

# Rebuild Python bindings
cmake -S . -B build -DCTRL_BUILD_PYTHON_BINDINGS=ON -G Ninja
cmake --build build --target ctrl_toolbox
conda run -n soft_robotics -- python bindings/smoke_test.py
```

Build is always sequential (no `--parallel`). Do NOT use `cmake --build --parallel`.

Expected passing: C++ ~174 | Python ~103 (UNVERIFIED — run run.py to confirm).
Log: `run_YYYYMMDD_HHMMSS.log` written after every run.py.

---

## 8. Key File Paths

```
lib/ControllerToolbox.h            Umbrella include (single #include for users)
lib/GradientProjectionQP.h         FISTA solver (header-only, shared by MPC/GPC/MHE/NMPC/TubeMPC)
lib/IController.h                  Base interface: compute/reset/sampleTime/bumplessInit/isHealthy
lib/IControllerObserver.h          Observer (has virtual onState(key, vec) since Part 33)
lib/ControllerRegistry.h           Meyers-singleton self-registration (Part 33)
lib/ControllerMonitor.h            CUSUM + EWMA SPC observer (Part 33)
lib/ComputationalDelayWrapper.h    One-sample delay decorator (Part 34)
lib/hal/HAL.h                      HAL umbrella (SimScheduler + FreeRTOS/Zephyr stubs)
tests/test_catch2_advanced.cpp     Main Catch2 suite (~95 cases)
tests/test_stability_margins.cpp   Stability margins regression (3 cases)
tests/test_boiler_regression.cpp   Boiler case study regression (6 cases, Part 33)
tests/test_smismo_regression.cpp   SMISMO case study regression (6 cases, Part 44 — recreated)
tests/test_solar_regression.cpp    Solar case study regression (6 cases, Part 33)
tests/test_humid_regression.cpp    Humid case study regression (6 cases, Part 33)
tools/compare_controllers.py       IAE/ISE table across all case-study CSVs (Part 34)
docs/compact_bug_report_parts_1-25.md   Archived tribal knowledge (Parts 1-25)
docs/compact_bug_report_parts_26-50.md  Archived tribal knowledge (Parts 26-50) ← this file
docs/cumulative_bug_report.md      Active issues (Part 51+)
docs/DOCUMENTATION.md              API reference
docs/ALGORITHM_ROADMAP_PHASE2.md   Phase 2 implementation plan (E1-E4, H1-H4, D1-D2)
CONTRIBUTING.md                    Coding conventions + 8-step checklist
prompt/prompt_enhanced.txt         Full session handoff (deep reference)
```

---

*Compact report covers Parts 26-50. Active issues tracked in `docs/cumulative_bug_report.md` (Part 51+).*
