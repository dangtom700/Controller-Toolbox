# Controller Toolbox -- Project Master State Document

**Project:** Discrete-Time Controller Toolbox (C++20 / pybind11 / Catch2)
**Current Part:** 26 (case-study expansion + senior review -- complete 2026-05-31)
**Maintained by:** Claude Code (Senior Principal Engineer role)
**Update cadence:** End of every major iteration (new algorithm, case study, or binding pass)

---

## 1. Baseline Health (Part 26 Exit State)

| Suite | Passing | Failing | Notes |
|-------|---------|---------|-------|
| C++ (Catch2 + binaries) | 90 | 0 | test_catch2_advanced (65) + pilot (5) + autoscheduling (9) + stability_margins (3) + tugsim_regression (1) + integration (6) + case-study/example binaries |
| Python examples | 88 | 0 | 16 SKIPped (Release rebuild needed for QP warn suppression) |
| Case studies | 367 runs | 0 | Boiler 216 (27x8), SMISMO 42 (14x3), Solar 45 (9x5), Tug 64 (16x4) |
| Runtime warnings | ~25 | -- | 15 old .pyd, 4 TC-REG-05, 2 c2d, 4 benign Tug PBH; all expected |

**Verify with:** `conda run -n soft_robotics -- python run.py`

---

## 2. Algorithm Inventory (~55 `lib/` modules -- unchanged in Part 26)

> Part 26 added **24 case-study controller wrappers**, not new `lib/` algorithms.
> Every case-study controller composes one or more of the modules below behind a
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

### Infrastructure
`ControllerStack`, `ControllerTraits`, `MetricsAnalyzer`, `SystemAnalysis`,
`BalancedTruncation`, `ZeroPhaseTrackingFilter`, `LinearisationHelper`,
`GradientProjectionQP` (FISTA -- shared solver)

---

## 3. File Structure Map

```
Controller Toolbox/
├── lib/                      Core library headers + sources
│   ├── ControllerToolbox.h   Umbrella include
│   ├── IController.h         Abstract base
│   ├── Features.h            Runtime feature flags
│   ├── GradientProjectionQP.h FISTA (header-only)
│   └── hal/                  HAL: SimScheduler, FreeRTOS/Zephyr stubs
├── bindings/                 pybind11 C++ + smoke_test.py
│   ├── controllers_bindings.cpp
│   ├── estimation_bindings.cpp
│   └── smoke_test.py
├── tests/                    Catch2 + standalone test programs
│   ├── test_catch2_advanced.cpp  (main suite, 65 cases)
│   ├── test_stability_margins.cpp (3 cases)
│   └── ...
├── examples/                 69 C++ examples (ex01-ex69+)
│   └── python/               Python mirror examples
├── case-study/               Full physics case studies (Part 26: all rosters expanded)
│   ├── Boiler Control/                   27 controllers, boiler_sim, 216 runs
│   ├── Tug Boat Numerical Simulation/    16 controllers, tug_sim, 64 runs
│   ├── meter in meter out control/       14 controllers, smismo_sim, 42 runs (SMISMO)
│   └── Solar-Driven Cooling System/      9 controllers, solar_cooling_sim, 45 runs
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

## 6. Open Items (Part 27+)

Priorities are now driven by the **Part 26 senior review** (top of
`docs/cumulative_bug_report.md`). The review tags are R/M/G/T.

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| **P26-CS** | Meter-in-meter-out hydraulic case study (14 controllers) | HIGH | **DONE (Part 26)** |
| **T1** | Case-study regression tests (Boiler/SMISMO/Solar -- 50/65 ctrls untested) | HIGH | Open |
| **M2** | Self-registration registry to replace hand-maintained `Features.h` map | HIGH | Open |
| **M3** | `onState()` telemetry hook on `IControllerObserver` (nonlinear debug) | MED | Open |
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

---

## 8. Next Immediate Steps (Part 27)

The case-study expansion is done. Next work is the review's high-priority debt:

1. **[T1]** Add case-study regression tests -- `tests/test_boiler_regression.cpp`,
   `test_smismo_regression.cpp`, `test_solar_regression.cpp`, asserting per-controller
   IAE / settling thresholds. Mirror `tests/test_tugsim_regression.cpp`. (50 of 65
   case-study controllers currently have no correctness check beyond "exe exited 0".)
2. **[M2]** Self-registration registry: `CTRL_REGISTER(Type)` macro + static factory map
   to replace the hand-maintained `lib/Features.h` unordered_map and shrink the 8-file
   "add a controller" checklist.
3. **[M3]** Add `IControllerObserver::onState(key, VectorXd)`; emit ESO/sliding-surface/
   QP-iteration internals from the nonlinear controllers.
4. **[REL]** Rebuild `ctrl_toolbox.pyd` in Release -- resolves 15 QP warnings, unblocks
   16 SKIPped Python examples.
5. **Verify baseline** before any work: `conda run -n soft_robotics -- python run.py`
   -> 90 C++ | 88 Python, case studies 216/42/45/64.

---

## 9. Update Protocol

At the end of every major iteration, update **sections 1, 6, 7, and 8** of this document:
- Section 1: new passing counts
- Section 6: close finished items, add new discoveries
- Section 7: append new architectural decisions
- Section 8: rewrite next steps

Also update `docs/cumulative_bug_report.md` (Part 26+ living issue log) and `prompt/prompt_enhanced.txt` (full session handoff).

---

*Last updated: 2026-05-31 | Part 26 complete (case-study expansion + senior review)*
