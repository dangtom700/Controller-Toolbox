# Controller Toolbox -- Project Master State Document

**Project:** Discrete-Time Controller Toolbox (C++20 / pybind11 / Catch2)
**Current Part:** 33 (T1a–T1d + A11 + SMPC + BO + M2 + M3 -- complete 2026-06-03)
**Maintained by:** Claude Code (Senior Principal Engineer role)
**Update cadence:** End of every major iteration (new algorithm, case study, or binding pass)

---

## 1. Baseline Health (Part 33 Exit State)

| Suite | Passing | Failing | Notes |
|-------|---------|---------|-------|
| C++ (Catch2 + binaries) | ~163 | 0 | test_catch2_advanced (~95) + pilot (5) + autoscheduling (9) + stability_margins (3) + tugsim_regression (1) + integration (6) + boiler/smismo/solar/humid regression (4×6=24) + test_humidification (1) + example binaries |
| Python examples | ~103 | 0 | ex99–ex102 added (Dyna/MBRL, ScenarioMPC, BayesBO, Registry+Monitor) |
| Case studies | 417 runs | 0 | Boiler 216 (27×8), SMISMO 42 (14×3), Solar 45 (9×5), Tug 64 (16×4), Humid 50 (10×5) |
| Runtime warnings | ~0 in bug_report.txt | -- | 36-entry safe_phrases list suppresses all known benign messages |

**Verify with:** `conda run -n soft_robotics -- python run.py`

---

## 2. Algorithm Inventory (~75 `lib/` modules -- updated through Part 33)

> Part 26 added 24 case-study controller wrappers. Parts 30–33 added 13 new `lib/` algorithms (DeePC, ILC, SINDy, KoopmanEDMD, L1Adaptive, CBFSafetyFilter, GaussianProcess, EchoStateNetwork, NeuralPID, CEMController, DynaController, ScenarioMPC, BayesianOptimizer) plus 3 infrastructure modules (ControllerRegistry, ControllerRegistrations, ControllerMonitor).
> Every case-study controller composes one or more modules below behind a
> plant-specific `ControllerBase` (see section 3 and CLAUDE.md "Case Studies").

### Core Controllers
| Class | Header | Sign convention |
|-------|--------|----------------|
| `DiscretePID` | DiscretePID.h | `compute(r - y)` |
| `DiscreteMPC` | DiscreteMPC.h | `compute(e)` |
| `DiscreteLQR` / `LQRAdapter` | DiscreteLQR.h | state feedback |
| `DiscreteLQG` | DiscreteLQG.h | `compute(e)` |
| `DiscreteSMC` | DiscreteSMC.h | `compute(y - ref)` <- reversed |
| `DiscreteADRC` | DiscreteADRC.h | `compute(r - y)` |
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
| `MovingHorizonEstimator` | MovingHorizonEstimator.h | box constraints on w |
| `ParticleFilter` | ParticleFilter.h | SIR; RMSE 4-10 is normal |

### System ID
| Class | Header | Notes |
|-------|--------|-------|
| `RecursiveLeastSquares` | RecursiveLeastSquares.h | ARX with forgetting |
| `FOPDTIdentifier` | FOPDTIdentifier.h | K, tau, theta + IMC-PID |
| `SOPDTIdentifier` | SOPDTIdentifier.h | K, tau1, tau2, theta |
| `SubspaceID` | SubspaceID.h | N4SID / MOESP |
| `LPVSystemID` | LPVSystemID.h | input is (nxN) column-major |

### Gain Scheduling Pipeline
`GapMetric` -> `LinearModelCluster` -> `GainScheduledController` -> `AutoGainScheduler`

### Tuning
`RelayAutoTuner`, `StepResponseTuner`, `LQRWeightTuner`, `MPCHorizonTuner`,
`ZieglerNicholsTuner`, `CohenCoonTuner`, `LoopShapingTuner`, `KalmanWeightTuner`,
`AutoTuner` (CMA-ES), `TunerSuite`

### Optional Modules (all ON by default)
`FuzzyLogic` (Mamdani/TS), `DiscreteHinf` (DGKF 2-Riccati), `EKF`, `UKF`,
`SubspaceID`, `FunctionApproximator` (Taylor + Pade)

### Data-driven and ML algorithms (Parts 30–33)
| Class | Header | Notes |
|-------|--------|-------|
| `DeePC` | DeePC.h | ADMM Hankel-QP (Coulson 2019) |
| `ILCController` | IterativeLearningControl.h | P-type + norm-optimal |
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
| `BayesianOptimizer` | BayesianOptimizer.h | GP surrogate + UCB/EI; header-only |

### Infrastructure (Part 33)
| Class | Header | Notes |
|-------|--------|-------|
| `ControllerRegistry` | ControllerRegistry.h | Meyers singleton; `CTRL_REGISTER_FEATURE` macro |
| `ControllerMonitor` | ControllerMonitor.h | CUSUM + EWMA SPC as `IControllerObserver` |

### Infrastructure (always)
`ControllerStack`, `ControllerTraits`, `MetricsAnalyzer`, `SystemAnalysis`,
`BalancedTruncation`, `ZeroPhaseTrackingFilter`, `LinearisationHelper`,
`GradientProjectionQP` (FISTA -- shared solver)

---

## 3. File Structure Map

```
Controller Toolbox/
├── lib/                      Core library headers + sources
│   ├── ControllerToolbox.h   Umbrella include
│   ├── IController.h         Abstract base (now has virtual name() + notifyObserverState())
│   ├── IControllerObserver.h  Observer (now has virtual onState(key, vec))
│   ├── Features.h            Delegates to ControllerRegistry::all() (Part 33)
│   ├── ControllerRegistry.h  Meyers-singleton self-registration (Part 33)
│   ├── ControllerRegistrations.h  Pre-M2 centralized registrations (Part 33)
│   ├── ControllerMonitor.h   CUSUM + EWMA SPC observer (Part 33)
│   ├── GradientProjectionQP.h FISTA (header-only)
│   └── hal/                  HAL: SimScheduler, FreeRTOS/Zephyr stubs
├── bindings/                 pybind11 C++ + smoke_test.py
│   ├── controllers_bindings.cpp
│   ├── estimation_bindings.cpp
│   └── smoke_test.py
├── tests/                    Catch2 + standalone test programs
│   ├── test_catch2_advanced.cpp  (main suite, ~95 cases including Part 33)
│   ├── test_stability_margins.cpp (3 cases)
│   ├── test_boiler_regression.cpp  (6 cases -- Part 33)
│   ├── test_smismo_regression.cpp  (6 cases -- Part 33)
│   ├── test_solar_regression.cpp   (6 cases -- Part 33)
│   └── test_humid_regression.cpp   (6 cases -- Part 33)
├── examples/                 79 C++ examples (ex01-ex79)
│   └── python/               102 Python examples (ex01-ex102)
├── case-study/               Full physics case studies
│   ├── Boiler Control/                   27 controllers, boiler_sim, 216 runs
│   ├── Tug Boat Numerical Simulation/    16 controllers, tug_sim, 64 runs
│   ├── meter in meter out control/       14 controllers, smismo_sim, 42 runs (SMISMO)
│   ├── Solar-Driven Cooling System/      9 controllers, solar_cooling_sim, 45 runs
│   └── Porous Fiber Plate Humidification System/  10 controllers, humidification_sim, 50 runs
├── docs/
│   ├── PROJECT_MASTER_STATE.md       <- this file
│   ├── compact_bug_report_parts_1-25.md  (archived tribal knowledge)
│   ├── cumulative_bug_report.md      (Part 26+ active issues)
│   ├── DOCUMENTATION.md              (API reference)
│   └── CHANGELOG.md
├── prompt/prompt_enhanced.txt        Full session handoff
├── run.py                            Master build + test runner
├── compile.bat                       Windows quick-compile
├── CMakeLists.txt
└── CLAUDE.md                         Session guide (this project's law)
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
| Build type | **Release** via `compile.bat` (`-DCMAKE_BUILD_TYPE=Release`). NOTE: `DiscreteHinf.cpp` requires Release on MinGW (Debug hits the PE/COFF "too many sections" limit). The stale `ctrl_toolbox.pyd` is still a Debug build -- rebuild it in Release to silence the 15 QP warnings. |

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
DiscreteADRC:   omega_o * Ts < 0.5       (backward Euler stability)
UKF alpha:      sqrt((n+kappa)/n)        (alpha=1 -> negative Wc0)
TubeMPC K:      u_tube = K*(x-x_nom); negate MATLAB lqr(): K = -K_lqr
LPVSystemID:    identifyLPV expects (nxN) column-major
phaseAt():      returns RADIANS (not degrees)
```

---

## 5. Active API Endpoints / Public Interface

All classes expose `IController` base: `compute(double)`, `reset()`, `sampleTime()`, `bumplessInit(double)`.

Key non-virtual extras:
- `GainScheduledController::lastOutput()` -- not a virtual override
- `TubeMPC::computeRef(x, y_ref)` -- different signature than IController
- `NonlinearMPC::computeRef(x, y_ref)` -- same

Python binding rule: all `IController` subclasses need
`py::class_<T, ctrl::IController, std::shared_ptr<T>>` (not bare `shared_ptr<T>`).

---

## 6. Open Items (Part 34+)

Priorities are driven by the **Part 26 senior review** (top of `docs/cumulative_bug_report.md`).

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| **P26-CS** | Meter-in-meter-out hydraulic case study (14 controllers) | HIGH | **DONE (Part 26)** |
| **T1** | Case-study regression tests (Boiler/SMISMO/Solar/Humid) | HIGH | **DONE (Part 33)** |
| **M2** | Self-registration registry (`ControllerRegistry`) | HIGH | **DONE (Part 33)** |
| **M3** | `onState()` telemetry + `ControllerMonitor` SPC | MED | **DONE (Part 33)** |
| **A1–A11** | ML/DD algorithm batch (DeePC through DynaController) | HIGH | **DONE (Parts 30–33)** |
| **SMPC** | ScenarioMPC stochastic QP | MED | **DONE (Part 33)** |
| **BO** | BayesianOptimizer GP surrogate | MED | **DONE (Part 33)** |
| **R1** | NaN-guard helper + edge-case contract matrix (only ADRC fails safe) | MED | Open |
| G1/T4 | MHE state constraints (linear inequalities) -- "missing 50%" of MHE | Low | Open |
| G1/T2 | MIMO nu-gap (blocks AutoGS on Boiler/Tug MIMO plants) | Low | Open |
| T3 | Full DK-iteration with vector-fitting rational D(jw) | Low | Open |
| T5 | GainScheduledController bumpless for LinearBlend mode | Low | Open |
| T7 | tools/compare_controllers.py IAE/ISE table (case studies emit CSV now) | Low | Open |
| REL | Rebuild ctrl_toolbox.pyd in Release mode (silence QP warns) | Low | Open |
| M4 | `template<typename Scalar>` leaf algorithms IF embedded float target is real | Backlog | Open |
| -- | FrequencyResponseID (Levy's method) / SDREController / RL bridge / ROS2 / FMU | Backlog | Open |

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
| Part 26 | Case-study controllers wrap `lib/` algos behind a per-study `ControllerBase`, NOT `ctrl::IController` | Each plant has a different I/O signature (MIMO force vector, valve increment, coupled spool cmd); a uniform scalar `IController` would force lossy adapters |
| Part 26 | `AutoGS`/`GainScheduled` `design_fn` wraps an LQR gain in a `DiscretePID`, not a raw `DiscreteLQR` | `DiscreteLQR` is not an `IController`; the scheduler needs `shared_ptr<IController>` (review G2/M1) |
| Part 26 | Senior review (R/M/G/T findings) is the new prioritisation source of truth | Captured forward-looking debt before it compounded across 60+ algorithms |
| Part 33 | `ControllerRegistrations.h` must be included AFTER all other lib/ headers in ControllerToolbox.h | Meyers-singleton `map_()` must exist before any `addFeature()` call; pre-M2 controllers have no self-registration |
| Part 33 | `CTRL_REGISTER_FEATURE` macro placed in headers (not .cpp files) | `inline const bool` fires per-include in headers; `.cpp` placement risks dead-strip in static archives |
| Part 33 | Case-study regression tests use `late_rmse < early_rmse * threshold`, not absolute IAE | Avoids baselines that rot when operating points change; catches divergence, sign flips, crashes |

---

## 8. Next Immediate Steps (Part 34)

The major Part 26 senior-review debt is closed. Remaining open items in priority order:

1. **[R1]** NaN-guard helper `ctrl::sanitize(double, fallback)` + edge-case contract matrix.
   Apply `sanitize()` at every `compute()` boundary; add Catch2 tests asserting graceful
   handling of `NaN` input, sustained saturation (anti-windup bound), and non-stabilizable
   plant (`isHealthy()==false`) for every controller family.
2. **Verify baseline** before any work: `conda run -n soft_robotics -- python run.py`
   → ~163 C++ | ~103 Python, case studies 216/42/45/64/50, `bug_report.txt` 0 blocks.
3. **[REL]** Rebuild `ctrl_toolbox.pyd` in Release — silences 15 stale-.pyd QP warnings,
   unlocks any Python examples that still have SKIP guards.
4. Low-priority backlog: G1/T2 (MIMO nu-gap), T3 (DK-iteration rational D), T5 (LinearBlend
   bumpless), T7 (compare_controllers.py IAE/ISE table).

---

## 9. Update Protocol

At the end of every major iteration, update **sections 1, 6, 7, and 8** of this document:
- Section 1: new passing counts
- Section 6: close finished items, add new discoveries
- Section 7: append new architectural decisions
- Section 8: rewrite next steps

Also update `docs/cumulative_bug_report.md` (Part 26+ living issue log) and `prompt/prompt_enhanced.txt` (full session handoff).

---

*Last updated: 2026-06-03 | Part 33 complete (T1a–T1d + A11 + SMPC + BO + M2 + M3/SPC)*
