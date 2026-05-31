# Controller Toolbox — Project Master State Document

**Project:** Discrete-Time Controller Toolbox (C++20 / pybind11 / Catch2)
**Current Part:** 26 (started 2026-05-30)
**Maintained by:** Claude Code (Senior Principal Engineer role)
**Update cadence:** End of every major iteration (new algorithm, case study, or binding pass)

---

## 1. Baseline Health (Part 25 Exit State)

| Suite | Passing | Failing | Notes |
|-------|---------|---------|-------|
| C++ (Catch2) | 89 | 0 | test_catch2_advanced (65) + pilot (5) + autoscheduling (9) + stability_margins (3) + tugsim_regression (1) + integration (6) |
| Python examples | 88 | 0 | 16 SKIPped (Release rebuild needed for QP warn suppression) |
| Runtime warnings | 21 | — | 15 old .pyd, 4 TC-REG-05, 2 c2d; all expected |

**Verify with:** `conda run -n soft_robotics -- python run.py`

---

## 2. Algorithm Inventory (55 modules as of Part 25)

### Core Controllers
| Class | Header | Sign convention |
|-------|--------|----------------|
| `DiscretePID` | DiscretePID.h | `compute(r - y)` |
| `DiscreteMPC` | DiscreteMPC.h | `compute(e)` |
| `DiscreteLQR` / `LQRAdapter` | DiscreteLQR.h | state feedback |
| `DiscreteLQG` | DiscreteLQG.h | `compute(e)` |
| `DiscreteSMC` | DiscreteSMC.h | `compute(y - ref)` ← reversed |
| `DiscreteADRC` | DiscreteADRC.h | `compute(r - y)` |
| `DiscreteLeadLag` | DiscreteLeadLag.h | `compute(e)` |
| `ExtremumSeeker` | ExtremumSeeker.h | `compute(J)` — NOT error |
| `SmithPredictor` | SmithPredictor.h | `compute(r - y)` |
| `AdaptiveSmithPredictor` | AdaptiveSmithPredictor.h | `compute(r - y)` |
| `FeedbackLinearisation` | FeedbackLinearisation.h | `compute(e)` + `setState(x)` |
| `MRACController` | MRACController.h | `compute(y_plant)` — NOT error |
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
| `LPVSystemID` | LPVSystemID.h | input is (n×N) column-major |

### Gain Scheduling Pipeline
`GapMetric` → `LinearModelCluster` → `GainScheduledController` → `AutoGainScheduler`

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
`GradientProjectionQP` (FISTA — shared solver)

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
├── examples/                 69 C++ examples (ex01–ex69+)
│   └── python/               Python mirror examples
├── case-study/               Full physics case studies
│   ├── Boiler Control/
│   ├── Tug Boat Numerical Simulation/
│   ├── meter in meter out control/   ← Part 26 candidate
│   └── Solar-Driven Cooling System/
├── docs/
│   ├── PROJECT_MASTER_STATE.md       ← this file
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
| Build type | **Debug** (Release pending — silences 15 QP warns) |

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
UKF alpha:      sqrt((n+kappa)/n)        (alpha=1 → negative Wc0)
TubeMPC K:      u_tube = K*(x-x_nom); negate MATLAB lqr(): K = -K_lqr
LPVSystemID:    identifyLPV expects (n×N) column-major
phaseAt():      returns RADIANS (not degrees)
```

---

## 5. Active API Endpoints / Public Interface

All classes expose `IController` base: `compute(double)`, `reset()`, `sampleTime()`, `bumplessInit(double)`.

Key non-virtual extras:
- `GainScheduledController::lastOutput()` — not a virtual override
- `TubeMPC::computeRef(x, y_ref)` — different signature than IController
- `NonlinearMPC::computeRef(x, y_ref)` — same

Python binding rule: all `IController` subclasses need
`py::class_<T, ctrl::IController, std::shared_ptr<T>>` (not bare `shared_ptr<T>`).

---

## 6. Open Items (Part 26+)

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| **P26-CS** | Meter-in-meter-out hydraulic case study (C++ sim + controller sweep) | HIGH | In progress |
| T2 | MIMO nu-gap (subspace chordal distance) | Low | Open |
| T3 | Full DK-iteration with vector-fitting rational D(jw) | Low | Open |
| T4 | MHE state constraints (linear inequalities) | Low | Open |
| T5 | GainScheduledController bumpless for LinearBlend mode | Low | Open |
| T7 | tools/compare_controllers.py IAE/ISE benchmark table | Low | Open |
| REL | Rebuild ctrl_toolbox.pyd in Release mode (silence QP warns) | Low | Open |
| — | FrequencyResponseID (Levy's method) | Backlog | Open |
| — | SDREController (pointwise DARE) | Backlog | Open |
| — | RL bridge (gym.Env Python wrapper) | Backlog | Open |
| — | ROS 2 hardware interface | Backlog | Open |
| — | FMU / co-simulation support | Backlog | Open |

---

## 7. Architectural Decisions Log

| Date | Decision | Rationale |
|------|----------|-----------|
| Pre-Part 1 | Discrete-time only; no continuous-time classes | Embedded deployment target; avoids ODE solver overhead |
| Pre-Part 1 | Single umbrella `ControllerToolbox.h` | Zero-friction include for downstream users |
| Part 1–5 | `IController` interface with `compute(double)` | Uniform swap-in-swap-out for benchmarking |
| Part 10 | `GradientProjectionQP` shared FISTA solver | Deduplication: MPC, GPC, MHE, NMPC, TubeMPC all share one QP core |
| Part 12 | `phaseAt()` fixed to return radians | Was wrongly documented as degrees; breaking fix applied |
| Part 15 | pybind11 `shared_ptr` as 3rd template arg for all IController subclasses | Required for Python GC interop; any bare class causes segfault on GC |
| Part 20 | `AntiWindupWrapper` must NOT wrap `DiscretePID` | PID has built-in Kb anti-windup; double-wrapping corrupts integrator |
| Part 25 | HAL headers commented out of umbrella by default | Avoids RTOS include pollution for desktop users |
| Part 25 | `TubeMPC K` sign: negate MATLAB `lqr()` output | MATLAB returns positive gain for min-norm convention; toolbox uses u = K*(x-x_nom) |

---

## 8. Next Immediate Steps (Part 26)

1. **[P26-CS]** Complete the meter-in-meter-out hydraulic plant model in `case-study/meter in meter out control/sim/`
   - Files exist: `smismo_plant.{h,cpp}`, `smismo_controllers.{h,cpp}`, `smismo_main.cpp`
   - Logs exist for 6 controllers × 3 scenarios (18 CSV files)
   - Missing: validate simulation output, generate plots, write case-study README
2. **[REL]** Rebuild `ctrl_toolbox.pyd` in Release — resolves 15 QP warnings, unblocks 16 SKIPped Python examples
3. **Verify baseline** before any Part 26 work: `conda run -n soft_robotics -- python run.py` → 89 C++ | 88 Python

---

## 9. Update Protocol

At the end of every major iteration, update **sections 1, 6, 7, and 8** of this document:
- Section 1: new passing counts
- Section 6: close finished items, add new discoveries
- Section 7: append new architectural decisions
- Section 8: rewrite next steps

Also update `docs/cumulative_bug_report.md` (Part 26+ living issue log) and `prompt/prompt_enhanced.txt` (full session handoff).

---

*Last updated: 2026-05-30 | Part 26 start*
