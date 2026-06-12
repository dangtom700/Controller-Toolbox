# Controller Toolbox -- Project Master State Document

**Project:** Discrete-Time Controller Toolbox (C++20 / pybind11 / Catch2)
**Current Part:** 49 (Surface Ship Manoeuvring Python-only study -- complete 2026-06-11)
**Maintained by:** Claude Code (Senior Principal Engineer role)
**Update cadence:** End of every major iteration (new algorithm, case study, or binding pass)

---

## 1. Baseline Health (Part 49 Exit State)

| Suite | Passing | Failing | Notes |
|-------|---------|---------|-------|
| C++ (Catch2 + binaries) | ~174 UNVERIFIED | 0 | Await next clean `run.py` to confirm. test_smismo_regression recreated Part 44 (6 tests new sim). |
| Python examples | ~103 UNVERIFIED | 0 | ex70–ex102 added through Part 33 |
| Case studies C++ | 748 runs UNVERIFIED | 0 | 9 studies × scenarios (see Section 3) |
| Case studies Python | 480 runs UNVERIFIED | 0 | 6 Python-only studies × scenarios (Phase 6) |
| Runtime warnings | ~0 in bug_report.txt | -- | 37-entry safe_phrases list suppresses all known benign messages |

**Verify with:** `conda run -n soft_robotics -- python run.py`

---

## 2. Algorithm Inventory (~85 `lib/` modules -- updated through Part 49)

> Parts 26-33 added 13 new `lib/` algorithms (DeePC, ILC, SINDy, KoopmanEDMD, L1Adaptive,
> CBFSafetyFilter, GaussianProcess, EchoStateNetwork, NeuralPID, CEMController, DynaController,
> ScenarioMPC, BayesianOptimizer) plus 5 infrastructure modules (ControllerRegistry,
> ControllerRegistrations, ControllerMonitor, LQRAdapter/makeLQRController, ComputationalDelayWrapper).
> Parts 34-49 added case studies only; no new `lib/` algorithms.

### Core Controllers
| Class | Header | Sign convention |
|-------|--------|----------------|
| `DiscretePID` | DiscretePID.h | `compute(r - y)` |
| `DiscreteMPC` | DiscreteMPC.h | `compute(e)` |
| `DiscreteLQR` / `LQRAdapter` | DiscreteLQR.h | state feedback; `makeLQRController()` factory |
| `DiscreteLQG` | DiscreteLQG.h | `compute(e)` |
| `DiscreteSMC` | DiscreteSMC.h | `compute(y - ref)` <- reversed convention |
| `DiscreteADRC` | DiscreteADRC.h | `compute(r - y)`; omega_o*Ts < 0.5 required |
| `DiscreteLeadLag` | DiscreteLeadLag.h | `compute(e)` |
| `ExtremumSeeker` | ExtremumSeeker.h | `compute(J)` -- NOT error |
| `SmithPredictor` | SmithPredictor.h | `compute(r - y)` |
| `AdaptiveSmithPredictor` | AdaptiveSmithPredictor.h | `compute(r - y)` |
| `FeedbackLinearisation` | FeedbackLinearisation.h | `compute(e)` + `setState(x)` |
| `MRACController` | MRACController.h | `compute(y_plant)` -- NOT error |
| `RepetitiveController` | RepetitiveController.h | `compute(e)` |
| `GPC` | GeneralizedPredictiveControl.h | velocity-form |
| `NonlinearMPC` | NonlinearMPC.h | `computeRef(x, y_ref)` |
| `TubeMPC` | TubeMPC.h | `computeRef(x, y_ref)` |
| `FeedforwardController` | FeedforwardController.h | `compute(r)` |
| `AntiWindupWrapper` | AntiWindupWrapper.h | do NOT wrap DiscretePID |

### Estimators
| Class | Header | Notes |
|-------|--------|-------|
| `KalmanFilter` | KalmanFilter.h | `covariance()` = post-update P |
| `ExtendedKalmanFilter` | ExtendedKalmanFilter.h | analytical/numerical Jacobians |
| `UnscentedKalmanFilter` | UnscentedKalmanFilter.h | alpha = sqrt((n+kappa)/n) |
| `MovingHorizonEstimator` | MovingHorizonEstimator.h | box constraints on w + x_0 (Part 34) |
| `ParticleFilter` | ParticleFilter.h | SIR; RMSE 4-10 is normal |

### System ID
| Class | Header | Notes |
|-------|--------|-------|
| `RecursiveLeastSquares` | RecursiveLeastSquares.h | ARX with forgetting; `params()` not `theta()` |
| `FOPDTIdentifier` | FOPDTIdentifier.h | K, tau, theta + IMC-PID |
| `SOPDTIdentifier` | SOPDTIdentifier.h | K, tau1, tau2, theta |
| `SubspaceID` | SubspaceID.h | N4SID / MOESP |
| `LPVSystemID` | LPVSystemID.h | input is (nxN) column-major |

### Gain Scheduling Pipeline
`GapMetric` (SISO+MIMO since Part 34) -> `LinearModelCluster` -> `GainScheduledController` -> `AutoGainScheduler`

### Tuning
`RelayAutoTuner`, `StepResponseTuner`, `LQRWeightTuner`, `MPCHorizonTuner`,
`ZieglerNicholsTuner`, `CohenCoonTuner`, `LoopShapingTuner`, `KalmanWeightTuner`,
`AutoTuner` (CMA-ES), `TunerSuite`, `BayesianOptimizer` (GP surrogate + UCB/EI)

### Optional Modules (all ON by default)
`FuzzyLogic` (Mamdani/TS), `DiscreteHinf` (DGKF 2-Riccati), `EKF`, `UKF`,
`SubspaceID`, `FunctionApproximator` (Taylor + Pade)

### Data-driven and ML algorithms (Parts 30-33)
| Class | Header | Notes |
|-------|--------|-------|
| `DeePC` | DeePC.h | ADMM Hankel-QP (Coulson 2019); `CTRL_REGISTER_FEATURE(deepc)` removed (B2 fix) |
| `ILCController` | IterativeLearningControl.h | P-type + norm-optimal; stores prev-trial u & e |
| `SINDy` / `SINDyModel` | SINDy.h | STLS sparse ID; training must have varied `u` |
| `KoopmanEDMD` | KoopmanEDMD.h | EDMD lift -> `StateSpace`; `A.rows() == nLifted - n_input` |
| `L1AdaptiveController` | L1AdaptiveController.h | LP-filtered MRAC; `compute(y_plant)` not error |
| `CBFSafetyFilter` | CBFSafetyFilter.h | 1D analytical QP safety wrapper |
| `GaussianProcess` | GaussianProcess.h | SE kernel + Cholesky; fixed-budget FIFO eviction |
| `EchoStateNetwork` | EchoStateNetwork.h | `reset()` preserves `W_out_` / `fitted_` |
| `NeuralPID` | NeuralPID.h | 3→n_h→3 online backprop; softplus gains |
| `CEMController` | CEMController.h | Elite-sample stochastic MPC; `computeRef(x, y_ref)` |
| `DynaController` | DynaController.h | Sutton Dyna MBRL; `modelRollout(e0, u_seq)` |
| `ScenarioMPC` | ScenarioMPC.h | N_s-scenario noise-averaged QP; H constant per episode |

### Infrastructure (Part 33-34)
| Class | Header | Notes |
|-------|--------|-------|
| `ControllerRegistry` | ControllerRegistry.h | Meyers singleton; `CTRL_REGISTER_FEATURE` macro in headers |
| `ControllerMonitor` | ControllerMonitor.h | CUSUM + EWMA SPC as `IControllerObserver` |
| `LQRAdapter` / `makeLQRController()` | DiscreteLQR.h | Factory returns `shared_ptr<IController>` for design_fn |
| `ComputationalDelayWrapper` | ComputationalDelayWrapper.h | One-sample delay decorator; first output = 0 |

### Infrastructure (always)
`ControllerStack`, `ControllerTraits`, `MetricsAnalyzer`, `SystemAnalysis`,
`BalancedTruncation`, `ZeroPhaseTrackingFilter`, `LinearisationHelper`,
`GradientProjectionQP` (FISTA -- shared solver)

---

## 3. File Structure Map

```
Controller Toolbox/
├── lib/                      Core library headers + sources (~85 modules)
│   ├── ControllerToolbox.h   Umbrella include
│   ├── IController.h         Abstract base (virtual name() + notifyObserverState())
│   ├── IControllerObserver.h Observer (virtual onState(key, vec) since Part 33)
│   ├── Features.h            Delegates to ControllerRegistry::all() (Part 33)
│   ├── ControllerRegistry.h  Meyers-singleton self-registration (Part 33)
│   ├── ControllerRegistrations.h  Pre-M2 centralized entries (include LAST in umbrella)
│   ├── ControllerMonitor.h   CUSUM + EWMA SPC observer (Part 33)
│   ├── ComputationalDelayWrapper.h  One-sample delay decorator (Part 34)
│   ├── GradientProjectionQP.h FISTA (header-only)
│   └── hal/                  HAL: SimScheduler, FreeRTOS/Zephyr stubs
├── bindings/                 pybind11 C++ + smoke_test.py
│   ├── controllers_bindings.cpp
│   ├── estimation_bindings.cpp
│   └── smoke_test.py
├── tests/                    Catch2 + standalone test programs
│   ├── test_catch2_advanced.cpp  (main suite, ~95 cases)
│   ├── test_stability_margins.cpp (3 cases)
│   ├── test_boiler_regression.cpp  (6 cases -- Part 33)
│   ├── test_smismo_regression.cpp  (6 cases -- Part 44, RECREATED from scratch)
│   ├── test_solar_regression.cpp   (6 cases -- Part 33)
│   └── test_humid_regression.cpp   (6 cases -- Part 33)
├── examples/                 C++ examples (ex01-ex79)
│   └── python/               Python examples (ex01-ex102)
├── tools/
│   └── compare_controllers.py  IAE/ISE table across all case-study CSVs (Part 34)
├── case-study/               Full physics case studies
│   │
│   ├── [C++ built -- registered in CMakeLists.txt + compile.bat]
│   ├── Boiler Control/                    27 ctrl, boiler_sim, 216 runs
│   ├── Tug Boat Numerical Simulation/     18 ctrl, tug_sim, 72 runs
│   ├── Solar-Driven Cooling .../          14 ctrl, solar_cooling_sim, 70 runs
│   ├── Porous Fiber Plate Humidification/ 15 ctrl, humidification_sim, 75 runs
│   ├── Active Suspension .../             15 ctrl, susp_sim, 75 runs
│   ├── Non-Inverting Buck-Boost Converter/ 12 ctrl, buck_boost_sim, 60 runs
│   ├── Solar Cooker .../                  12 ctrl, solar_cooker_sim, 60 runs
│   ├── Solar Ocean Thermal Energy .../    12 ctrl, sotec_sim, 60 runs
│   ├── Separate Meter In Separate Meter Out/  12 ctrl, smismo_sim, 60 runs
│   │
│   ├── [Python-only -- discovered by Phase 6 via case-study/*/sim/main.py]
│   ├── Vertical Drill String .../         17 ctrl, sim/main.py, 85 runs
│   ├── Multi-Body Floating Wind-Wave .../  16 ctrl, sim/main.py, 80 runs
│   ├── Tracking Control of EH Force .../   12 ctrl, sim/main.py, 60 runs
│   ├── High-Altitude Aerial Firefighting/  12 planners, sim/main.py, 60 runs
│   ├── Air-Cooled Battery Thermal .../     12 ctrl, sim/main.py, 60 runs
│   └── Nonlinear Surface Ship .../         12 ctrl, sim/main.py, 60 runs
│
├── docs/
│   ├── PROJECT_MASTER_STATE.md       <- this file
│   ├── compact_bug_report_parts_1-25.md   (archived: Parts 1-25 tribal knowledge)
│   ├── compact_bug_report_parts_26-50.md  (archived: Parts 26-44 tribal knowledge)
│   ├── cumulative_bug_report.md      (Part 51+ active issues)
│   ├── ALGORITHM_ROADMAP_PHASE2.md   (Phase 2 implementation plan: E1-E4, H1-H4, D1-D2)
│   ├── DOCUMENTATION.md              (API reference)
│   └── CASE_STUDIES.md               (case study documentation)
├── prompt/prompt_enhanced.txt        Full session handoff
├── run.py                            Master build + test runner (6 phases)
├── compile.bat                       Windows quick-compile
├── CMakeLists.txt
└── CLAUDE.md                         Session guide (law of the project)
```

---

## 4. Configuration Specs

### Build toolchain
| Item | Value |
|------|-------|
| Language | C++20 |
| Math library | Eigen 3.4+ |
| Build system | CMake + Ninja |
| Python env | `soft_robotics` conda env |
| Bindings | pybind11, target `ctrl_toolbox` |
| Test framework | Catch2 (v3) |
| Build type | **Release** via `compile.bat` (`-DCMAKE_BUILD_TYPE=Release`). NOTE: `DiscreteHinf.cpp` requires Release on MinGW (Debug hits PE/COFF "too many sections" limit). |

### CMake key flags
```cmake
CTRL_BUILD_PYTHON_BINDINGS=ON
CTRL_HAS_ADVANCED_KALMAN   # EKF + UKF
CTRL_HAS_SUBSPACE          # N4SID
CTRL_HAS_FUZZY             # FuzzyLogic
CTRL_HAS_HINF              # DiscreteHinf
CTRL_HAS_FUNCTION_APPROX   # FunctionApproximator
```

### Critical parameter constraints
```
DiscreteADRC:         omega_o * Ts < 0.5  (backward Euler stability — applies at ALL Ts values)
UKF alpha:            sqrt((n+kappa)/n)   (alpha=1 -> negative Wc0)
TubeMPC K:            u_tube = K*(x-x_nom); negate MATLAB lqr(): K = -K_lqr
LPVSystemID:          identifyLPV expects (nxN) column-major
phaseAt():            returns RADIANS (not degrees)
ComputationalDelay:   first compute() returns 0 — warm up one step before trusting output
```

---

## 5. Active API Endpoints / Public Interface

All classes expose `IController` base: `compute(double)`, `reset()`, `sampleTime()`, `bumplessInit(double)`.

Key non-virtual extras:
- `GainScheduledController::lastOutput()` -- not a virtual override
- `TubeMPC::computeRef(x, y_ref)` -- different signature than IController
- `NonlinearMPC::computeRef(x, y_ref)` -- same
- `makeLQRController()` -- free function, returns `shared_ptr<IController>` wrapping DiscreteLQR+LQRAdapter

Python binding rule: all `IController` subclasses need
`py::class_<T, ctrl::IController, std::shared_ptr<T>>` (not bare `shared_ptr<T>`).

---

## 6. Open Items (Part 49+)

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| **E1** | `GreyBoxEstimator` — Levenberg-Marquardt param estimation for user ODE | HIGH | Open |
| **E2** | `RecursiveGreyBoxEstimator` — augmented-state UKF wrapper | HIGH | Open |
| **E3** | GP Residual Model — `GaussianProcess` extension with uncertainty output | MED | Open |
| **E4** | MHE Inequality Constraints — extend MHE with `C_ineq`/`d_ineq` | MED | Open |
| **H1** | `HybridModel` base class — `IPlantModel` with `f_phys + f_data` | MED | Open |
| **H2** | `HybridMPC` — `NonlinearMPC` using `HybridModel` for prediction | MED | Open |
| **H3** | RL-MPC stitching Python example | LOW | Open |
| **H4** | `HybridModelTrainer` — hyperopt for `f_data` component | LOW | Open |
| **D1** | Mismatch Detector — CUSUM on KF/MHE innovation | LOW | Open |
| **D2** | Digital Twin Lite Python app | LOW | Open |
| **C2** | 6 spec-only stubs. BEMS + MEMS have no blocker. | MED | Open |
| **B36-3** | Unify NaN-guard across controller fleet | MED | Open |
| R1 | Edge-case contract matrix tests for every controller family | MED | Open |
| T3 | Full DK-iteration with vector-fitting rational D(jω) | LOW | Open |
| B36-2 | `ex79_registry_monitor` monitors nothing (M3 telemetry mis-wired) | LOW | Open |
| REL | Rebuild `ctrl_toolbox.pyd` in Release | LOW | Open |
| M4 | `template<typename Scalar>` leaf algorithms for embedded float target | Backlog | Open |
| **P26-CS** | Meter-in-meter-out hydraulic case study | HIGH | **DONE (Part 26)** |
| **T1** | Case-study regression tests (Boiler/Solar/Humid) | HIGH | **DONE (Part 33)** |
| **T1-SMISMO** | SMISMO regression tests | HIGH | **DONE (Part 44, recreated)** |
| **M2** | Self-registration registry (`ControllerRegistry`) | HIGH | **DONE (Part 33)** |
| **M3** | `onState()` telemetry + `ControllerMonitor` SPC | MED | **DONE (Part 33)** |
| **A1-A11** | ML/DD algorithm batch (DeePC through DynaController) | HIGH | **DONE (Parts 30-33)** |
| **SMPC** | ScenarioMPC stochastic QP | MED | **DONE (Part 33)** |
| **BO** | BayesianOptimizer GP surrogate | MED | **DONE (Part 33)** |
| **G2** | LQRAdapter factory (`makeLQRController()`) | MED | **DONE (Part 34)** |
| **G3** | ComputationalDelayWrapper | MED | **DONE (Part 34)** |
| **T2** | MIMO nu-gap (subspace chordal distance) | LOW | **DONE (Part 34)** |
| **T4** | MHE state constraints | LOW | **DONE (Part 34)** |
| **T5** | LinearBlend bumpless transfer | LOW | **DONE (Part 34)** |
| **T7** | `compare_controllers.py` IAE/ISE table | LOW | **DONE (Part 34)** |
| **C2-partial** | Active Susp + BuckBoost + DrillString + WindWave + EHFS + Firefighting + BTMS + SurfaceShip case studies | HIGH | **DONE (Parts 37-49)** |

---

## 7. Architectural Decisions Log

| Date | Decision | Rationale |
|------|----------|-----------|
| Pre-Part 1 | Discrete-time only; no continuous-time classes | Embedded deployment target; avoids ODE solver overhead |
| Pre-Part 1 | Single umbrella `ControllerToolbox.h` | Zero-friction include for downstream users |
| Part 1-5 | `IController` interface with `compute(double)` | Uniform swap-in-swap-out for benchmarking |
| Part 10 | `GradientProjectionQP` shared FISTA solver | Deduplication: MPC, GPC, MHE, NMPC, TubeMPC all share one QP core |
| Part 12 | `phaseAt()` fixed to return radians | Was wrongly documented as degrees; breaking fix applied |
| Part 15 | pybind11 `shared_ptr` as 3rd template arg for all IController subclasses | Required for Python GC interop; any bare class causes segfault on GC |
| Part 20 | `AntiWindupWrapper` must NOT wrap `DiscretePID` | PID has built-in Kb anti-windup; double-wrapping corrupts integrator |
| Part 25 | HAL headers commented out of umbrella by default | Avoids RTOS include pollution for desktop users |
| Part 25 | `TubeMPC K` sign: negate MATLAB `lqr()` output | MATLAB returns positive gain for min-norm convention; toolbox uses u = K*(x-x_nom) |
| Part 26 | Case-study controllers wrap `lib/` algos behind per-study `ControllerBase`, NOT `ctrl::IController` | Each plant has different I/O signature; uniform scalar IController forces lossy adapters |
| Part 26 | `AutoGS`/`GainScheduled` `design_fn` uses `makeLQRController()`, not raw `DiscreteLQR` | `DiscreteLQR` is not an `IController`; factory provides `shared_ptr<IController>` |
| Part 33 | `ControllerRegistrations.h` must be included AFTER all other lib/ headers | Meyers-singleton `map_()` must exist before any `addFeature()` call |
| Part 33 | `CTRL_REGISTER_FEATURE` macro placed in headers (not .cpp files) | `inline const bool` fires per-include; `.cpp` placement risks dead-strip in static archives |
| Part 33 | Case-study regression tests use `late_rmse < early_rmse * threshold`, not absolute IAE | Avoids baselines that rot when operating points change; catches divergence and sign flips |
| Part 34 | `makeLQRController()` factory pattern established | Closes G2 gap: LQR can now be used anywhere an `IController` is expected |
| Part 34 | `ComputationalDelayWrapper` warm-up required | First output is 0 (held initialisation); callers must warm up one step before trusting output |
| Part 39 | `CTRL_REGISTER_FEATURE(deepc)` removed from `ControllerRegistrations.h` | DeePC has no runtime feature implementation; false registration was misleading |
| Part 43 | Solar Cooker MRAC gammas must be negative for negative-gain plant | Positive gammas drive theta negative, clamping u to 0; plant free-heats past setpoint |
| Part 44 | SMISMO backpressure P_bd=20 bar is cavitation guard | Overrunning scenario causes pressure drop below P_bd; do not lower below ~5 bar |
| Part 46 | CEP replaces IAE as primary metric for firefighting planner studies | IAE undefined for open-loop trajectory planners; CEP (50th-pctile radial error) is the engineering metric |
| Part 46 | `_drift_sensitivity()` not `wy*t_fall` for lateral drift compensation | Forward-speed drag coupling reduces lateral drift to ~40% of linear estimate at V=50 m/s |
| Part 49 | MMG model SRUKF-identified parameters used directly | Paper (Meng 2025 Table 5) provides 19 identified parameters; re-identification from scratch unnecessary |

---

## 8. Next Immediate Steps (Phase 2)

The A1-A11 algorithm set is complete and the case-study roster has reached 9 C++ + 6 Python-only studies. Phase 2 targets model estimation and hybrid models. Priority order:

1. **Verify baseline** before any work: `conda run -n soft_robotics -- python run.py`
   → Establish current UNVERIFIED counts as the new baseline; write actual numbers here.

2. **[E1] `GreyBoxEstimator`** — highest-value first, ~2-3 days.
   - New `lib/GreyBoxEstimator.{h,cpp}` following the 8-step checklist.
   - Reuse `AutoTuner` cost-fn pattern; use `LinearisationHelper::jacobianX` for sensitivity equations.
   - Levenberg-Marquardt via Eigen `NumericalDiff`; bounded parameter support.
   - Example: estimate R and C of a thermal model from step-response data.
   - 2+ `[greybox_estimator]` Catch2 tests.

3. **[E2] `RecursiveGreyBoxEstimator`** — ~1-2 days.
   - Wraps existing `UnscentedKalmanFilter`; augmented state `[x; p]`.
   - Provide `augmentStateAndParams(x, p)` helper.
   - Example: track motor friction coefficient drift over lifetime.

4. **[E4] MHE Inequality Constraints** — shortest path, ~1-2 days.
   - Extend `MovingHorizonEstimator` with `MHEParams::C_ineq`, `d_ineq` (polytopic state constraints).
   - Reuse existing FISTA box-projection mechanism.

5. **[C2] BEMS Python-only study** — no blocker per CLAUDE.md.

---

## 9. Update Protocol

At the end of every major iteration, update **sections 1, 6, 7, and 8** of this document:
- Section 1: new passing counts (always run `run.py` first to get actual numbers)
- Section 6: close finished items, add new discoveries
- Section 7: append new architectural decisions
- Section 8: rewrite next steps for the current phase

Also update `docs/cumulative_bug_report.md` (Part 45+ active log) and `prompt/prompt_enhanced.txt` (full session handoff).

---

*Last updated: 2026-06-11 | Part 49 complete (Surface Ship Manoeuvring Python-only study)*
