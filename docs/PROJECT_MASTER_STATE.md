# Controller Toolbox -- Project Master State Document

> **Known stale as of 2026-06-22:** this document described Bouyancy-Driven Airship in
> Vertical Plane as scaffolded-only/never-implemented; it now has a full `sim/`
> implementation (verified directly against the filesystem and `case-study/CMakeLists.txt`/
> `compile.bat`). Treat structural counts below as a snapshot, not current truth -- see
> `docs/handoff.md` for verified current state and `docs/case_study_status.md` (auto-generated)
> for actual case-study status.

**Project:** Discrete-Time Controller Toolbox (C++20 / pybind11 / Catch2)
**Current Part:** 66 (ROB-1 robustness analysis extended to all 10 C++ case studies + `generate_report.py` empty-data crash fix -- complete 2026-06-20)
**Maintained by:** Claude Code (Senior Principal Engineer role)
**Update cadence:** End of every major iteration (new algorithm, case study, or binding pass)

---

## 1. Baseline Health (Part 66 Exit State)

These are **structural counts** (files / TEST_CASE() declarations / CSV runs on disk),
verified directly against the working tree on 2026-06-18 (Catch2/example/Python counts
have not been re-verified since; no new `lib/` files were added in Parts 64-66, so they
should be unchanged). They are NOT a verified pass/fail report -- run
`conda run -n soft_robotics -- python run.py` for that. Treat every count below as
**UNVERIFIED pass status** until a clean `run.py` confirms it.

| Suite | Structural count | Notes |
|-------|-------------------|-------|
| C++ Catch2 TEST_CASE() | 332 across 16 files | `test_catch2_advanced.cpp` (main suite) alone has grown to 236 -- the long-standing "~95 main suite" / "~174 total" figures in CLAUDE.md and prior revisions of this doc are stale undercounts, not a regression. |
| C++ example programs | 86 files (`examples/ex*.cpp`) | |
| Python example scripts | 110 files (`examples/python/ex*.py`) | Numbering has intentional duplicates (e.g. two `ex23_*.py`, two `ex89_*.py`) from different feature batches; both run. |
| Case studies C++ (10 studies) | 1488 runs | Boiler 216, Tug 72, Solar 70, Humidification 75, ActiveSuspension 90 (18 ctrl), BuckBoost 60, SolarCooker 60, SOTEC 60, SMISMO 65 (13 ctrl), Stewart 720 (12 ctrl x 60 sea-state configs, Part 61). **All 10 now also have a `*_robustness` target** (fault sweep + Monte Carlo + WCET over the real nonlinear sim) -- 3 since Part 64 (ActiveSuspension, Humidification, Solar Cooling), the other 7 since Part 66 (Boiler, Tug, BuckBoost, SolarCooker, SOTEC, SMISMO, Stewart). `docs/case_study_status.md` now shows all 10 as `Complete` (was `On-going` for 7 of them pre-Part-66). |
| Case studies Python-only (8 studies, official roster) | 565 runs | DrillString 85, WindWave 80, EHFS 70 (14 ctrl), Firefighting 60, BTMS 60, SurfaceShip 60, EV6x6 90 (18 ctrl), AircraftEngine 60 (12 ctrl, promoted Part 63; gained a `run_wcet_profile()` hook + `config/analysis.json` in Part 66). |
| Case studies, remaining undocumented (Part 62 discovery) | see Section 6 / `docs/case_study_status.md` | 31 total directories under `case-study/`; 18 are reflected in CLAUDE.md's/README's roster tables (10 C++ + 8 Python-only, the 8th being `Aircraft Engine Thermal Management`, promoted Part 63). 7 are "Open placeholder" (scaffolded by `tools/new_case_study.py`, never implemented; one of these -- `Bouyancy-Driven Airship in Vertical Plane` -- gained a `HANDOFF_PROMPT.md` implementation plan in Part 66, still no code); 6 are "Not started" (PDF/README only, no `sim/`; one of these -- `Hybrid-Driven Tendon-Pneumatic Soft Manipulator` -- also gained a `HANDOFF_PROMPT.md` implementation plan in Part 66, still no `sim/`). |
| Runtime warnings | `bug_report.txt` expected 0 blocks | `safe_phrases` list in `run.py` suppresses all known benign messages. |

**Verify with:** `conda run -n soft_robotics -- python run.py`
**Case-study status (auto-generated, do not hand-edit):** `docs/case_study_status.md`, regenerate via `python tools/case_study_tracker.py`.

---

## 2. Algorithm Inventory (~90 `lib/` modules -- updated through Part 63; no new `lib/`
algorithms in Parts 64-66, which were case-study-level robustness/analysis/documentation
work only -- see Section 6)

> Parts 26-33 added 13 new `lib/` algorithms (DeePC, ILC, SINDy, KoopmanEDMD, L1Adaptive,
> CBFSafetyFilter, GaussianProcess, EchoStateNetwork, NeuralPID, CEMController, DynaController,
> ScenarioMPC, BayesianOptimizer) plus 5 infrastructure modules (ControllerRegistry,
> ControllerRegistrations, ControllerMonitor, LQRAdapter/makeLQRController, ComputationalDelayWrapper).
> Parts 51-55 added DAE architecture (P1/P2/P3), GreyBoxEstimator (E1), RecursiveGreyBoxEstimator (E2),
> GPResidualModel (E3), MHE inequality constraints (E4), HybridModel/HybridMPC/HybridModelTrainer (H1-H4),
> MismatchDetector (D1), BasicPID<Scalar>/BasicSMC<Scalar> (M4), GeneticAlgorithm, ParticleSwarmOptimizer,
> DifferentialEvolution (C3 from Part 55). Parts 56-60 added CI/CD overhaul, cross-platform scripts,
> case study tracker, and ROS2 adapter (no new lib/ algorithms). Part 61 added the 10th C++ case study
> (6-DOF Stewart) and fixed a `NeuralPID` softplus overflow bug (no new lib/ algorithms). Part 62
> upgraded `case_study_tracker.py` to 4-tier status detection and reconciled project documentation
> (no new lib/ algorithms). Part 63 promoted `Aircraft Engine Thermal Management` into the
> official case-study roster (documentation only - no new lib/ algorithms). Direct count:
> `lib/*.h` = 84 headers, `lib/*.cpp` = 59 sources (flat, no
> subdirectories except `lib/embedded/` and `lib/hal/`) -- the "~90 modules" figure used throughout
> CLAUDE.md/README is an approximate module count (some headers bundle multiple param structs), not
> a literal file count; the two are in the same ballpark and both are fine to use loosely.

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
| `NeuralPID` | NeuralPID.h | 3->n_h->3 online backprop; softplus gains; numerically-stable two-branch softplus (Part 61 fix) |
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

### Model Estimation (Parts 51-54)
| Class | Header | Notes |
|-------|--------|-------|
| `GreyBoxEstimator` | GreyBoxEstimator.h | Levenberg-Marquardt ODE param fit; `fit(x0,U,Y)` -> `Result{params,cost,converged}` |
| `RecursiveGreyBoxEstimator` | RecursiveGreyBoxEstimator.h | Augmented-state UKF; `step(y,u)` online; requires `CTRL_ENABLE_ADVANCED_KALMAN` |
| `GPResidualModel` | GPResidualModel.h | GP on model-plant mismatch; `predictWithUncertainty(xf,model_pred)` |
| `MismatchDetector` | MismatchDetector.h | CUSUM on KF/MHE innovation; `enableMismatchDetection()` on KF and MHE |
| `DAESystem` | PlantModel.h | Index-1 semi-explicit DAE; `consistentInit`, `dae2ode`, `c2d(DAESystem)` |

### Metaheuristic Optimisers (Part 55)
| Class | Header | Notes |
|-------|--------|-------|
| `GeneticAlgorithm` | GeneticAlgorithm.h | BLX-alpha crossover, tournament selection, elitism; `optimize(cost_fn)` -> `TunerResult` |
| `ParticleSwarmOptimizer` | ParticleSwarmOptimizer.h | Clerc-Kennedy w=0.729; V_max = v_frac*(upper-lower) |
| `DifferentialEvolution` | DifferentialEvolution.h | DE/rand/1/bin; population >= 4 required |

### Embedded Subset (Part 54/57E)
| Class | Header | Notes |
|-------|--------|-------|
| `BasicPID<Scalar>` | BasicPID.h | Header-only, no Eigen, no virtual; `compute(r-y)`; `Ts` in Params |
| `BasicSMC<Scalar>` | BasicSMC.h | Header-only, no Eigen; `K` is switching gain; `compute(r-y)` |
| `DiscreteIntegrator<Scalar>` | embedded/DiscreteIntegrator.h | Backward Euler; `integrate(x)`, `reset()`, `value()` |
| `FixedRateFilter<Scalar,N>` | embedded/FixedRateFilter.h | Compile-time order IIR; `filter(x)` |
| `RingBuffer<T,N>` | embedded/RingBuffer.h | Fixed FIFO; `push()`, `pop()`, `peek(i)` |

### Infrastructure (always)
`ControllerStack`, `ControllerTraits`, `MetricsAnalyzer`, `SystemAnalysis`,
`BalancedTruncation`, `ZeroPhaseTrackingFilter`, `LinearisationHelper`,
`GradientProjectionQP` (FISTA -- shared solver)

---

## 3. File Structure Map

```
Controller Toolbox/
|-- lib/                      Core library: 84 headers + 59 .cpp, FLAT (no lib/control/
|   |                          or lib/estimation/ subdirectories -- one file per class)
|   |-- ControllerToolbox.h   Umbrella include
|   |-- IController.h         Abstract base (virtual name() + notifyObserverState())
|   |-- IControllerObserver.h Observer (virtual onState(key, vec) since Part 33)
|   |-- Features.h            Delegates to ControllerRegistry::all() (Part 33)
|   |-- ControllerRegistry.h  Meyers-singleton self-registration (Part 33)
|   |-- ControllerRegistrations.h  Pre-M2 centralized entries (include LAST in umbrella)
|   |-- ControllerMonitor.h   CUSUM + EWMA SPC observer (Part 33)
|   |-- ComputationalDelayWrapper.h  One-sample delay decorator (Part 34)
|   |-- GradientProjectionQP.h FISTA (header-only)
|   |-- embedded/              BasicPID, BasicSMC, DiscreteIntegrator, FixedRateFilter, RingBuffer
|   |-- hal/                  HAL: SimScheduler, FreeRTOS/Zephyr stubs
|-- bindings/                 pybind11 C++ + smoke_test.py
|   |-- module.cpp            PYBIND11_MODULE entry point; flat `ctrl_toolbox` module (no submodules)
|   |-- plantmodel_bindings.cpp / controllers_bindings.cpp / estimation_bindings.cpp
|   |-- advanced_bindings.cpp / analysis_bindings.cpp
|   |-- smoke_test.py
|-- tests/                    Catch2 + standalone test programs: 332 TEST_CASE() across 16 files
|   |-- test_catch2_advanced.cpp   (main suite, 236 cases)
|   |-- test_stability_margins.cpp, test_catch2_pilot.cpp, test_autoscheduling.cpp,
|   |   test_humidification.cpp, test_embedded_subset.cpp
|   |-- test_{boiler,buck_boost,humid,smismo,solar,solar_cooker,sotec,stewart,susp,tugsim}_regression.cpp
|-- examples/                 86 C++ examples (ex01-ex82, some numbers reused across feature batches)
|   |-- python/               110 Python examples (ex01-ex102+, same reuse pattern)
|-- scripts/                  tune_all.cpp / simulate_all.cpp / realtime_all.cpp (generic example-plant
|   |                          benchmarking, NOT case studies) + create_controller.py, deploy.py,
|   |                          fix_examples_bindings.py, generate_test_data.py, visualize.py
|-- benchmark/                bench_controllers.cpp -- per-step latency microbenchmark for every
|   |                          controller; opt-in via -DCTRL_BUILD_BENCHMARKS=ON, off by default
|-- cheatsheet/                Reference notes: controller_list.md, controller_categories.md,
|   |                          controller-tuning-reference.md, tuning_methods.md,
|   |                          control_design_pipeline.md, system_identification.md (+ armax.md/
|   |                          fopdt.md/n4sid.md subfolder), advanced_model_estimation.md,
|   |                          embedded_and_realtime.md, mismatch_detection.md, model_evaluation.md,
|   |                          phase2_hybrid_modeling.md
|-- tools/
|   |-- new_case_study.py        scaffolds a case-study framework (C++ or Python) from a source PDF
|   |-- case_study_tracker.py    scans case-study/*/ (skips case-study/common/, Part 66); writes
|   |                             docs/case_study_status.md (TRK-1, Part 62 rewrite)
|   |-- metrics.py / compare_controllers.py / monte_carlo.py / mc_plots.py
|   |-- fault_injector.py / fault_sweep.py / fault_plots.py
|   |-- anova.py / wcet_report.py / model_validation.py / mu_analysis.py / mu_plots.py
|   |-- generate_report.py       self-contained HTML report per study (RPT-1)
|   |-- generate_all_reports.bat drives run_analysis.py + generate_report.py across every
|   |                             case-study/*/ with a logs/ dir, then refreshes case_study_tracker.py;
|   |                             logs to tools/generate_all_reports.log (gitignored)
|   |-- study_protocol.py        documentation-as-code: the run_single/run_with_fault/grey_box_model
|                                  hook contract every Python case study's sim/main.py must honour
|-- case-study/               31 directories total -- status is AUTO-TRACKED, do not hand-list here.
|   |                          common/RobustnessStats.h (Part 64): header-only FaultSpec/MetricStats
|   |                          shared by every C++ study's robustness_main.cpp; not a case study
|   |                          itself (excluded from the tracker, see tools/case_study_tracker.py above).
|   |                          See docs/case_study_status.md (regenerate: `python tools/case_study_tracker.py`).
|   |                          4-tier status: Complete / On-going / Open placeholder / Not started.
|   |                          As of Part 66: 18 official-roster studies (10 C++ + 8 Python-only, all
|   |                          reflected in CLAUDE.md's/README's roster tables), of which all 10 C++
|   |                          studies now show as Complete (gained fault_sweep/mc_summary/wcet_summary
|   |                          coverage Part 64+66 -- see Section 1); the 8 Python-only studies remain
|   |                          On-going. 7 Open placeholder (scaffolded by tools/new_case_study.py,
|   |                          never implemented), 6 Not started (PDF/README only). Full roster +
|   |                          tribal knowledge per study: CLAUDE.md "Case Studies" section and each
|   |                          study's own README.md.
|-- docs/
|   |-- PROJECT_MASTER_STATE.md       <- this file
|   |-- compact_bug_report_parts_1-25.md   (archived: Parts 1-25 tribal knowledge)
|   |-- compact_bug_report_parts_26-50.md  (archived: Parts 26-44 tribal knowledge)
|   |-- cumulative_bug_report.md      (Part 51+ active issues)
|   |-- ALGORITHM_ROADMAP_PHASE2.md   (Phase 2 implementation plan: E1-E4, H1-H4, D1-D2)
|   |-- DOCUMENTATION.md              (API reference)
|   |-- deployment.md                 (RT/RTOS integration, troubleshooting)
|   |-- control_strategies_deep_dive.md
|   |-- case_study_status.md          (auto-generated case-study status tracker, 4-tier as of Part 62)
|   |-- archived/                     audit_report.md, roadmap_deployment_frontend.md, test_update.md
|                                      -- moved here from docs/ root; CLAUDE.md's "Done in Part 57/57B"
|                                      narrative entries still cite the pre-move docs/ path, which was
|                                      correct AT THE TIME (historical journal, not re-edited).
|-- prompt/                   Task-specific prompt TEMPLATES (not session history -- prompt_enhanced.txt
|   |                          was removed Part 62, decluttering; it is not coming back):
|   |-- make_case_study_cpp.md      step-by-step: PDF -> production C++ case study
|   |-- make_case_study_python.md   step-by-step: PDF -> production Python-only case study
|   |-- audit_project.md            structural-audit persona/procedure
|   |-- project_enrichment.md       strategic-advisor persona for library evolution questions
|   |-- fresh_restart.md            "case-study co-pilot" onboarding persona (NOTE: its directory-
|   |                                layout assumptions, e.g. lib/control/, are WRONG for this repo --
|   |                                see docs/case_study_copilot_reference.md for the corrected version)
|   |-- handoff_prompt61.md         most recent dated session handoff (supersedes earlier handoffs)
|-- ros2/ctrl_toolbox_ros2/           ROS 2 Humble ament_cmake package (DIST-3, Part 60)
|   |-- include/ctrl_toolbox_ros2/controller_node.hpp   ControllerNode<T> lifecycle node template
|   |-- example/pid_temperature_node.cpp
|   |-- CMakeLists.txt + package.xml + README.md
|-- run.py                            Master build + test runner (8 phases, cross-platform Part 60)
|-- compile.bat / compile.sh          Windows / Linux-macOS sequential build
|-- setup.ps1 / setup.sh              Windows / Linux-macOS bootstrap
|-- CMakeLists.txt
|-- CLAUDE.md                         Session guide (law of the project)
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
CTRL_BUILD_BENCHMARKS      # benchmark/bench_controllers.cpp; OFF by default
```

### Critical parameter constraints
```
DiscreteADRC:         omega_o * Ts < 0.5  (backward Euler stability - applies at ALL Ts values)
UKF alpha:            sqrt((n+kappa)/n)   (alpha=1 -> negative Wc0)
TubeMPC K:            u_tube = K*(x-x_nom); negate MATLAB lqr(): K = -K_lqr
LPVSystemID:          identifyLPV expects (nxN) column-major
phaseAt():            returns RADIANS (not degrees)
ComputationalDelay:   first compute() returns 0 - warm up one step before trusting output
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
Python import is flat: `import ctrl_toolbox as ctrl` -- no submodule hierarchy.

---

## 6. Open Items (Part 66+)

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| **ROB-1** | Robustness analysis (fault sweep + Monte Carlo + WCET via `case-study/common/RobustnessStats.h`) for all 10 C++ case studies | MED | **DONE for C++ (Part 64 + 66)** -- still open for the ~21 Python-only/not-yet-implemented studies |
| **GR-1** | `tools/generate_report.py` `_section_comparison()` raised `KeyError` on any study with zero parseable run rows (every Open-placeholder/Not-started scaffold) -- indexed `df[[...]]` before the `df.empty` guard it fell through to | LOW | **DONE (Part 66)** |
| **AE-WCET** | Aircraft Engine Thermal Management `run_wcet_profile()` + `config/analysis.json`, bringing it in line with every other Python study's `tools/run_analysis.py` hook contract | LOW | **DONE (Part 66)** |
| **D2** | Digital Twin Lite Python app (FastAPI/Streamlit, GreyBoxEstimator-backed) | LOW | Open |
| **C2** | 11 spec-only/undocumented case studies, re-audited Part 62 via the new tracker: 7 are "Open placeholder" (`Bouyancy-Driven Airship in Vertical Plane`, `Differential Drive Robot Tracking`, `Dual-Arm IAUV Motion Planning`, `PCM Thermal Energy Storage Control`, `Residential Building Comfort SMPC`, `Satellite Launch Vehicle Systems`, `Underwater Glider Trajectory Tracking` -- scaffolded by `tools/new_case_study.py`, plant/controllers never implemented); 6 are "Not started" (`Building Energy Management System`, `Data-Driven Sliding Mode Control of Soft Robot 2024`, `Heavy-Duty Parallel-Serial Hydraulic Manipulator VDC`, `Hybrid-Driven Tendon-Pneumatic Soft Manipulator`, `Underwater Robotic Manipulator Trajectory Tracking`, `Unmanned Surface Vehicle Wave-Predictive Attitude Control` -- PDF/README only, no `sim/`). | MED | Open |
| **C2-NEW** | `Aircraft Engine Thermal Management` -- **Done Part 63**: promoted into the official roster. Added to README.md's and CLAUDE.md's Python-only studies tables (17 -> 18 studies, 7 -> 8 Python-only); controller roster + 4 tribal-knowledge gotchas added to CLAUDE.md. No code changes needed -- already fully implemented and already auto-discovered by `run.py` Phase 6. | -- | Done |
| **REL** | Rebuild `ctrl_toolbox.pyd` in Release to silence stale-.pyd QP warnings | LOW | Open |
| **B36-3** | Unify NaN-guard across controller fleet | MED | Open |
| **R1** | Edge-case contract matrix tests for every controller family | MED | Open |
| **T3** | Full DK-iteration with vector-fitting rational D(jomega) | LOW | Open |
| **TRK-2** | `case_study_tracker.py` 4-tier status detection (Complete / On-going / Open placeholder / Not started) replacing the old file-existence-only 3-tier scheme, which mis-classified untouched `new_case_study.py` scaffolds as "On-going" because the placeholder plant+controller still run and produce real-looking `logs/*.csv` | MED | **DONE (Part 62)** |
| **DOC-1** | Reconcile `docs/PROJECT_MASTER_STATE.md` (was 12 Parts behind CLAUDE.md) and fix stale references repo-wide: `prompt/prompt_enhanced.txt` (removed Part 62, doesn't exist), `docs/audit_report.md` / `docs/roadmap_deployment_frontend.md` (moved to `docs/archived/`), missing mentions of `scripts/`, `benchmark/`, `cheatsheet/` | MED | **DONE (Part 62)** |
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
| **C2-partial** | Active Susp + BuckBoost + DrillString + WindWave + EHFS + Firefighting + BTMS + SurfaceShip + EV6x6 + Stewart case studies | HIGH | **DONE (Parts 37-61)** |
| **DIST-1..5, ANA-1..7, RPT-1, PLT-1, TRK-1, DIST-3** | Distribution/analysis/cross-platform/ROS2 tracks | -- | **DONE (Parts 57E-60)** -- see CLAUDE.md for the full breakdown |

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
| Part 61 | Stewart Platform hinge geometry needs different base/platform half-angles | Equal half-angles make every leg a pure rotation of every other leg -- a genuine Jacobian singularity for vertical load at the home pose, verified empirically across phase/delta combinations |
| Part 62 | `case_study_tracker.py` status detection keys off literal placeholder-template fingerprints, not file existence | A freshly-scaffolded study's placeholder plant (`x' = -a*x + b*u`) and `OpenLoop` controller actually run, producing real-looking `logs/*.csv` -- file-existence checks alone would misclassify an untouched scaffold as "On-going" |
| Part 62 | `docs/PROJECT_MASTER_STATE.md`'s hand-maintained case-study tree (Section 3) replaced with a pointer to `docs/case_study_status.md` | The hand-maintained version had drifted 12 Parts out of date (missing the Stewart study, 8 undiscovered directories, stale controller/run counts); the auto-generated tracker is the single source of truth going forward |
| Part 63 | Promoted `Aircraft Engine Thermal Management` into the official 18-study roster rather than leaving it as a flagged-but-uncounted discovery | It was already fully implemented (real plant + 12 controllers, no placeholder fingerprint) and already auto-discovered by `run.py` Phase 6 - the only gap was documentation, and README.md's intro paragraph had already half-promoted it before this reconciliation, suggesting the promotion was simply left unfinished rather than intentionally deferred |
| Part 64 | Case-study robustness coverage uses a new, independent `case-study/common/RobustnessStats.h` (perturbs real physical plant parameters, reruns the actual nonlinear closed-loop sim) instead of wiring in the existing `lib/RobustnessAnalysis.h` (linearized `StateSpace` Monte Carlo) | `lib/RobustnessAnalysis.h`'s linearized-model Monte Carlo isn't meaningful for the SMC/ADRC/Fuzzy/GA-tuned nonlinear controllers most case studies actually use; `docs/robust_implementation_plan.md`'s own proposed case-study integration path (via `ctrl.monteCarloAnalysis` + a `grey_box_model()` hook) was deliberately not followed for this reason |
| Part 66 | `tools/case_study_tracker.py` excludes `case-study/common/` from its directory scan | `common/` is a shared header-only tooling directory (Part 64's `RobustnessStats.h`), not a case study -- it has no `sim/` and was being mis-classified as "Not started" |

---

## 8. Next Immediate Steps

1. **Verify baseline** before any further work: `conda run -n soft_robotics -- python run.py`
   -> Establish current pass/fail counts as the new baseline; the counts in Section 1 are
   structural (file/CSV counts), not verified pass status.
2. **Extend ROB-1 to the Python-only studies** (now done for all 10 C++ studies as of Part 66):
   the 8 official-roster Python-only studies and any of the placeholder/not-started studies
   that get implemented next still have no fault-sweep/Monte-Carlo/WCET coverage beyond what
   `tools/run_analysis.py`'s existing `config/analysis.json` hook already provides per study.
3. **Triage the 7 "Open placeholder" + 6 "Not started" studies** (C2 above): for each, either
   implement real plant dynamics + controller roster (follow `prompt/make_case_study_cpp.md` or
   `prompt/make_case_study_python.md`), or decide to leave them as future work and note that
   explicitly in CLAUDE.md's stub tracking instead of leaving them silently undocumented.
   `Bouyancy-Driven Airship in Vertical Plane` and `Hybrid-Driven Tendon-Pneumatic Soft
   Manipulator` each have a fully-resolved `HANDOFF_PROMPT.md` plan (Part 66) ready to execute --
   start there before re-deriving anything from their PDFs.
4. **D2 Digital Twin Lite** remains the only LOW-priority item with no implementation yet.
5. Re-run `python tools/case_study_tracker.py` after any case-study work to keep
   `docs/case_study_status.md` current -- it is the authoritative, low-maintenance status source;
   avoid re-introducing a hand-maintained case-study table anywhere else in the docs.

---

## 9. Update Protocol

At the end of every major iteration, update **sections 1, 6, 7, and 8** of this document:
- Section 1: new passing counts (always run `run.py` first to get actual numbers)
- Section 6: close finished items, add new discoveries
- Section 7: append new architectural decisions
- Section 8: rewrite next steps for the current phase

Also update `docs/cumulative_bug_report.md` (Part 45+ active log) and `CLAUDE.md` (the
canonical session guide). `prompt/` holds task-specific templates (e.g. `make_case_study_cpp.md`,
`audit_project.md`), not a running session-history file -- there is no single "full handoff" file
to update; `prompt/handoff_prompt61.md` is a dated snapshot, not a living document.

---

*Last updated: 2026-06-20 | Part 66 complete (ROB-1 robustness analysis extended to all 10 C++ case studies; `generate_report.py` empty-data crash fixed; Aircraft Engine WCET hook added)*
