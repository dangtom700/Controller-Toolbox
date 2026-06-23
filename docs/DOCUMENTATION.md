# Controller Toolbox - Technical Documentation

*Discrete-time control library in modern C++20 (Eigen 3.4+).
Target audience: control engineers and software developers familiar with discrete-time control theory who want to integrate, extend, or deploy the library.*

---

## Table of Contents

1. [Setup and Environment](#1-setup-and-environment)
2. [Compilation Guide](#2-compilation-guide)
3. [Project Structure](#3-project-structure)
4. [Usage Guide](#4-usage-guide)
5. [Class Reference](#5-class-reference)
   - 5.1 [Core Types](#51-core-types-iplantmodel)
   - 5.2 [Controllers](#52-controllers) (incl. Fuzzy Logic Module)
   - 5.3 [Estimators, Identification & Optimisation](#53-estimators--identification)
          (incl. GainScheduledController, GapMetric, LPVSystemID, NonlinearMPC, AdaptiveSmithPredictor, AutoTuner, AntiWindupWrapper, TubeMPC, ParticleFilter)
   - 5.4 [Tuning Layer](#54-tuning-layer)
   - 5.5 [Composition & Orchestration](#55-composition--orchestration)
   - 5.6 [Analysis & Metrics](#56-analysis--metrics)
   - 5.7 [Real-Time Utilities & HAL](#57-real-time-utilities--hal)
   - 5.8 [Data-Driven & ML Controllers](#58-data-driven--ml-controllers-parts-3134)
          (ILC, SINDy, KoopmanEDMD, L1Adaptive, CBFSafetyFilter, GaussianProcess, EchoStateNetwork, NeuralPID, CEMController, DynaController, ScenarioMPC, BayesianOptimizer, ControllerRegistry, ControllerMonitor, ComputationalDelayWrapper)
6. [Deployment Cross-References](#6-deployment-cross-references)

---

## 1. Setup and Environment

### 1.1 Required Dependencies

| Component | Version | Purpose |
|-----------|---------|---------|
| C++ compiler | C++20 (GCC >= 10, Clang >= 12, MSVC >= 19.29) | Source language |
| CMake | >= 3.16 | Build system |
| Eigen | >= 3.4 (`find_package(Eigen3 3.4 REQUIRED)`) | Linear algebra |
| Doxygen | optional | API documentation target (`make docs`) |

Eigen must be discoverable by CMake. On Windows install via vcpkg (`vcpkg install eigen3`) or conda-forge; on Linux use the distribution package (`libeigen3-dev`) or build from source.

### 1.2 Optional Python Tooling

The `examples/python/` directory contains companion scripts using `python-control` for cross-validation. Create the environment from [examples/python/environment.yml](../examples/python/environment.yml):

```bash
conda env create -f examples/python/environment.yml
conda activate soft_robotics
```

This installs `python=3.11`, `numpy`, `scipy`, `matplotlib`, `pandas`, `scikit-learn`, and the `control` package.

### 1.3 Repository Layout (Top Level)

```
controller/
|-- CMakeLists.txt          # Root build, subdir aggregator
|-- lib/                    # Library sources (build target: controller_toolbox)
|-- examples/               # Single-file demos (ex01..ex88 C++) + examples/python/ (Python)
|-- case-study/             # 9 C++ built + 7 Python-only implemented studies, plus spec-only
|--                          #   stubs/scaffolds tracked in docs/case_study_status.md
|-- tests/                  # CTest-driven unit + integration tests
|-- tools/                  # Analysis pipeline (compare_controllers, monte_carlo, fault_sweep, ...)
|-- scripts/                # tune_all / simulate_all / realtime_all batch tools
|-- ros2/                   # ROS2 thin wrapper (ctrl_toolbox_ros2, ControllerNode<T> lifecycle node)
|-- cheatsheet/             # Markdown reference notes (tuning, identification)
|-- setup.ps1               # Windows bootstrap (MSYS2 + conda + bindings)
|-- setup.sh                # Linux/macOS bootstrap (mirrors setup.ps1)
|-- compile.bat             # Windows full sequential build (~120 targets)
|-- compile.sh              # Linux/macOS full sequential build (mirrors compile.bat)
|-- run.py                  # 7-phase test runner (non-ASCII check, compile, bindings, C++, Python,
|--                          #   case studies, case-study status/report)
|-- docs/                   # Documentation, roadmap, audit reports, deployment.md, case study status
```

---

## 2. Compilation Guide

### 2.1 Standard Configure-Build

From the repository root:

```bash
# Windows (one-shot bootstrap + bindings + smoke test):
.\setup.ps1

# Linux / macOS:
./setup.sh

# Manual configure + build (bindings only):
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTRL_BUILD_PYTHON_BINDINGS=ON -G Ninja
cmake --build build --target ctrl_toolbox
```

**Important:** Do **not** use `cmake --build --parallel`. The build system requires sequential target ordering; parallel builds produce linker failures on some platforms. Use `compile.bat` (Windows) or `compile.sh` (Linux/macOS) to build all ~120 targets in the correct dependency order.

This produces the static library `build/lib/libcontroller_toolbox.a` (or `.lib` on Windows) and every example/test/script executable. The root [CMakeLists.txt](../CMakeLists.txt) aggregates: `lib/`, `tests/`, `examples/`, `scripts/`, `case-study/` (benchmarks are intentionally excluded).

### 2.2 Running Tests

```bash
cd build && ctest --output-on-failure
```

Many test executables are registered in [tests/CMakeLists.txt](../tests/CMakeLists.txt) (21 `add_executable` targets as of this writing): the Catch2 suites (`test_catch2_advanced`, `test_catch2_pilot`, `test_stability_margins`, `test_autoscheduling`), the legacy hand-rolled suites (`test_controllers`, `test_tuners_extended`, `test_integration`), the embedded-subset suite (`test_embedded_subset`), and the per-study regression suites. Filter by name or Catch2 tag, e.g. `ctest -R test_catch2_advanced` or `build/tests/test_catch2_advanced.exe [smc]`. Run `conda run -n soft_robotics -- python run.py` for the canonical full pass.

### 2.3 Linking Against the Library

The library publishes `lib/` as its include root, so consumers write `#include "ControllerToolbox.h"` (the umbrella header at [lib/ControllerToolbox.h](../lib/ControllerToolbox.h)) and link `controller_toolbox`:

```cmake
target_link_libraries(your_target PRIVATE controller_toolbox)
target_compile_features(your_target PRIVATE cxx_std_20)
```

Eigen is propagated as a `PUBLIC` dependency of `controller_toolbox`, so the consumer does not need to link it explicitly.

### 2.4 Real-Time Build Flags

For production / RTOS targets, see [deployment.md Section 2](deployment.md#2-real-time-integration). Suggested flags:

```
-O2 -fno-exceptions -fno-rtti -fstack-usage
```

### 2.5 Doxygen API Output (Optional)

If `Doxygen` is found, the root `CMakeLists.txt` registers a `docs` target:

```bash
cmake --build build --target docs
```

---

## 3. Project Structure

### 3.1 Library (`lib/`)

| Header | Component(s) |
|--------|--------------|
| [ControllerToolbox.h](../lib/ControllerToolbox.h) | Umbrella include - pulls in every public header |
| [IController.h](../lib/IController.h) | Abstract controller interface (`IController`) |
| [IControllerObserver.h](../lib/IControllerObserver.h) | Observer/telemetry callback interface |
| [Features.h](../lib/Features.h) | `ctrl::features()` - runtime optional-module discovery |
| [PlantModel.h](../lib/PlantModel.h) | `TransferFunction`, `StateSpace`, `tf2ss`, `c2d`, `ssStep`, `ssStepCopy` |
| [DiscretePID.h](../lib/DiscretePID.h) | PID with derivative filter + anti-windup (DoM, 2-DOF) |
| [DiscreteLQR.h](../lib/DiscreteLQR.h) | Infinite-horizon LQR, DARE solver, `LQRAdapter` |
| [DiscreteMPC.h](../lib/DiscreteMPC.h) | Condensed receding-horizon QP, `setPlant()` for adaptive MPC |
| [DiscreteLQG.h](../lib/DiscreteLQG.h) | LQR + Kalman output-feedback combo |
| [DiscreteSMC.h](../lib/DiscreteSMC.h) | First-order sliding mode with boundary layer |
| [DiscreteADRC.h](../lib/DiscreteADRC.h) | 2nd-order LADRC (ESO + PD) |
| [DiscreteLeadLag.h](../lib/DiscreteLeadLag.h) | Tustin-discretised lead-lag biquad |
| [SmithPredictor.h](../lib/SmithPredictor.h) | Dead-time compensator wrapper (integer + fractional Pade delay) |
| [ExtremumSeeker.h](../lib/ExtremumSeeker.h) | Perturbation-based ESC |
| [RepetitiveController.h](../lib/RepetitiveController.h) | Internal-model repetitive control (IMP with Q-filter) |
| [FeedforwardController.h](../lib/FeedforwardController.h) | Static / dynamic feedforward (reference model + gain) |
| [GeneralizedPredictiveControl.h](../lib/GeneralizedPredictiveControl.h) | GPC (CARIMA predictor, RLS online adaptation, `setPlant()`) |
| [DiscreteHinf.h](../lib/DiscreteHinf.h) | H-infinity (gamma iteration, mixed sensitivity) + mu-synthesis DK-iteration |
| [FuzzyLogic.h](../lib/FuzzyLogic.h) | Mamdani/TS inference engine, `FuzzyPD`, `FuzzyPID`, `FuzzySupervisor` |
| [KalmanFilter.h](../lib/KalmanFilter.h) | Standalone linear Kalman filter (predict / update / step) |
| [ExtendedKalmanFilter.h](../lib/ExtendedKalmanFilter.h) | EKF with user-supplied Jacobians or numerical differentiation |
| [UnscentedKalmanFilter.h](../lib/UnscentedKalmanFilter.h) | UKF with scaled sigma-point parametrisation (alpha, beta, kappa) |
| [MovingHorizonEstimator.h](../lib/MovingHorizonEstimator.h) | MHE via condensed QP; box constraints on process noise |
| [FOPDTIdentifier.h](../lib/FOPDTIdentifier.h) | FOPDT step-response identification (graphical + golden-section) |
| [SOPDTIdentifier.h](../lib/SOPDTIdentifier.h) | SOPDT step-response identification + Rivera 1986 IMC-PID tuning |
| [RecursiveLeastSquares.h](../lib/RecursiveLeastSquares.h) | Online ARX identification, `toTransferFunction`, `toStateSpace` |
| [SubspaceID.h](../lib/SubspaceID.h) | N4SID subspace identification, `suggestOrder` |
| [FunctionApproximator.h](../lib/FunctionApproximator.h) | Pade delay filter, polynomial approximators |
| [GradientProjectionQP.h](../lib/GradientProjectionQP.h) | Projected gradient QP solver (shared by MPC, GPC, MHE) |
| [ControllerStack.h](../lib/ControllerStack.h) | Supervisory / Additive / Weighted composition |
| [ControllerTuner.h](../lib/ControllerTuner.h) | Per-family tuners (Relay, FOPDT, Bryson, MPC, ...) |
| [TunerSuite.h](../lib/TunerSuite.h) | Unified runtime-dispatched tuner with soft warnings |
| [ControllerTraits.h](../lib/ControllerTraits.h) | Compile-time traits for tuner <-> controller compatibility |
| [MetricsAnalyzer.h](../lib/MetricsAnalyzer.h) | Time-domain step-response metrics |
| [SystemAnalysis.h](../lib/SystemAnalysis.h) | Poles, margins, H-infinity norm, Lyapunov |
| [AtomicParamBuffer.h](../lib/AtomicParamBuffer.h) | Lock-free param double-buffer for RT updates |
| [hal/HAL.h](../lib/hal/HAL.h) | `ISensor`, `IActuator`, `SimPlant`, `SimSensor`, `SimActuator` |

### 3.2 Examples (`examples/`)

~89 single-file C++ programs (`ex01_*` through `ex88_*`) plus `example_pid_feedback`. Each demonstrates one controller, composition pattern, identification method, or corrector architecture. See [examples/CMakeLists.txt](../examples/CMakeLists.txt) for the full enumeration. The most recent additions cover the robustness/synthesis growth in v2 (e.g. `ex88_h2_synthesis.cpp` for `DiscreteH2`).

**C++ example groups:**

| Range | Theme |
|-------|-------|
| ex01–ex22 | Core controllers: PID, LQR, MPC, SMC, ADRC, ESC, Lead-Lag, Smith Predictor, LQG, stacks |
| ex23–ex26 | Fuzzy Logic: FuzzyPD, FuzzyPID, FuzzySupervisor+MPC, TS gain scheduling |
| ex27–ex31 | Advanced: function approximator, GPC, repetitive control, EKF, subspace ID |
| ex32–ex41 | SOPDT ID, MHE, rational mu-synthesis, cascade/feedforward/smith/ESC/UKF/LPV |
| ex42–ex54 | Corrector patterns: Cascade, Additive, Observer+SF, Supervisory/bumpless transfer |
| ex55–ex59 | Extensions: LinearisationHelper, FeedbackLinearisation, MRAC, BalancedTruncation, ZPETC |
| ex60–ex69 | Part 20–30: GapMetric/clustering, LPV ID, AutoGainScheduler, NonlinearMPC, AdaptiveSP, AutoTuner, AntiWindup, TubeMPC, ParticleFilter, DeePC |
| ex70–ex79 | Part 31–33 ML/DD: ILC, SINDy, KoopmanEDMD, L1Adaptive, CBFSafety, GP+ESN+NeuralPID, DynaMBRL, ScenarioMPC, BayesianTuner, RegistryMonitor |
| ex80–ex82 | Part 52–55: GreyBoxEstimator, HybridMPC, Metaheuristics (GA/PSO/DE) |

**Python examples** (`examples/python/`, `ex01_*` through `ex102_*`): NumPy/python-control cross-validation and pybind11 binding demonstrations for every C++ class. Includes RL-MPC stitching, all ML/DD controllers, DAE/hybrid-model workflows, and metaheuristic optimisers.

### 3.3 Case Studies (`case-study/`)

Each case study pairs a nonlinear plant simulator with a roster of controllers, sweeps every controller across several scenarios, and writes CSV telemetry to `logs/` for post-processing. C++ studies build as self-contained executables; Python-only studies run via `sim/main.py` (discovered automatically by `run.py` Phase 6). The auto-generated status table is at [`docs/case_study_status.md`](case_study_status.md).

#### C++ built (9) — registered in `case-study/CMakeLists.txt` + `compile.bat`

| Study | Plant | Ctrls | Scenarios | Runs | Target |
|---|---|---|---|---|---|
| [`Boiler Control/`](../case-study/Boiler%20Control/) | Bell-Åström 3×3 MIMO boiler-turbine | 27 | 8 | 216 | `boiler_sim` |
| [`Tug Boat Numerical Simulation/`](../case-study/Tug%20Boat%20Numerical%20Simulation/) | 3-DOF tug, 6-state MIMO + thrust allocation | 18 | 4 | 72 | `tug_sim` |
| [`Solar-Driven Cooling .../`](../case-study/Solar-Driven%20Cooling%20System%20with%20Photovoltaic%20Evaporative%20Chimney/) | Algebraic SISO solar cooling + PV chimney | 14 | 5 | 70 | `solar_cooling_sim` |
| [`Porous Fiber Plate Humidification System/`](../case-study/Porous%20Fiber%20Plate%20Humidification%20System/) | Flat-plate evaporative humidifier + room ODE + 2-step sensor delay | 15 | 5 | 75 | `humidification_sim` |
| [`Active Suspension Mathematical Modeling and Optimization 2025/`](../case-study/Active%20Suspension%20Mathematical%20Modeling%20and%20Optimization%202025/) | 2-DOF quarter-car, 4-state RK4, F_act ±2000 N | 18 | 5 | 90 | `susp_sim` |
| [`Non-Inverting Buck-Boost Converter/`](../case-study/Non-Inverting%20Buck-Boost%20Converter/) | Averaged 2-state buck-boost, RK4 at 50 kHz, mode hysteresis ±0.1 V | 12 | 5 | 60 | `buck_boost_sim` |
| [`Solar Cooker with Reflector and Absorber/`](../case-study/Solar%20Cooker%20with%20Reflector%20and%20Absorber/) | 2-state absorber+pot ODE, PCM effective-C, RK4 (Ts=30 s) | 12 | 5 | 60 | `solar_cooker_sim` |
| [`Solar Ocean Thermal Energy Conversion System/`](../case-study/Solar%20Ocean%20Thermal%20Energy%20Conversion%20System/) | 2-state collector+tank ODE, ORC algebraic map, Euler (Ts=30 s) | 12 | 5 | 60 | `sotec_sim` |
| [`Separate Meter In Separate Meter Out/`](../case-study/Separate%20Meter%20In%20Separate%20Meter%20Out/) | SMISMO hydraulic cylinder, 8-state RK4 (Ts=1 ms), dual PDCVs + Stribeck friction | 13 | 5 | 65 | `smismo_sim` |

**Boiler-Turbine (27):** PID, LQR, LQG, MPC, SMC, ESC, ADRC, Lead-Lag+PID, Smith Predictor, GPC-RLS, EKF-LQR, UKF-LQR, FuzzyPID, FuzzySup-MPC, three ControllerStack compositions, Repetitive, MRAC, H-infinity, AdaptiveSP, NonlinearMPC, FeedbackLin, MHE-LQR, LPV-GS, SubspaceID-LQG, AutoGainScheduler-LQR.

**Tug Boat (18):** PID, KF-PID, SMC, MPC, ESC, FuzzyPID, FuzzySup-MPC, ADRC, Repetitive, MIMO-LQR, LQG, per-axis TubeMPC, EKF-LQR, MRAC, AutoGainScheduler-LQR, NonlinearMPC, L1Adaptive, ScenarioMPC.

**Active Suspension (18):** Passive, PID, ADRC, SMC, LQR (Bryson), LQG, MPC (2-state body SS), MRAC, FuzzyPID, TubeMPC, ILC, CBFSafety, L1Adaptive, ScenarioMPC, DynaCtrl, GAOptPID, PSOOptPID, DEOptPID.

**Buck-Boost (12):** OpenLoop, PI-Buck, PI-Boost, TLCS-ClassicPI, FuzzyPD, FuzzyPID-Buck, FuzzyPID-Boost, TLCS-FuzzyPI, GainScheduled, ADRC, MPC, LQR.

**SMISMO (13):** PID, CascadePID, LQR, LQG, MPC, ADRC, SMC, FeedbackLin, TubeMPC, L1Adaptive, GainScheduled, NonlinearMPC, DOBEnergyCtrl (adaptive supply-pressure DOB).

#### Python-only (7) — `sim/main.py`, run by Phase 6 of `run.py`

These studies use `ctrl_toolbox` Python bindings directly; no C++ compilation is needed. Run individually with `conda run -n soft_robotics -- python sim/main.py` from the study directory, or automatically via `run.py` Phase 6.

| Study | Plant | Ctrls | Scenarios | Runs |
|---|---|---|---|---|
| [`Vertical Drill String Mathematical Review 2025/`](../case-study/Vertical%20Drill%20String%20Mathematical%20Review%202025/) | 2-DOF torsional model, Stribeck friction, RK4 (Ts=0.1 s) | 17 | 5 | 85 |
| [`Multi-Body Floating Wind-Wave Platform/`](../case-study/Multi-Body%20Floating%20Wind-Wave%20Platform/) | 4-state FOWT heave + WEC arm, sinusoidal wave forcing, RK4 (Ts=0.5 s) | 16 | 5 | 80 |
| [`Tracking Control of Electro-Hydraulic Force Servo Systems/`](../case-study/Tracking%20Control%20of%20Electro-Hydraulic%20Force%20Servo%20Systems/) | 5-state EHFS [P_A, P_B, x_v, v_p, x_p], servo valve + cylinder, RK4 (Ts=0.5 ms) | 14 | 5 | 70 |
| [`High-Altitude Aerial Firefighting Bag Drop/`](../case-study/High-Altitude%20Aerial%20Firefighting%20Bag%20Drop/) | 3D bag trajectory, drag+gravity+wind, RK4 (Ts=0.05 s); primary metric = CEP | 12 | 5 | 60 |
| [`Air-Cooled Battery Thermal Management System/`](../case-study/Air-Cooled%20Battery%20Thermal%20Management%20System/) | 1-D transient HX, N=9 cells, J/U/L flow switching, Forward Euler (Ts=1 s) | 12 | 5 | 60 |
| [`Nonlinear Surface Ship Manoeuvring Control/`](../case-study/Nonlinear%20Surface%20Ship%20Manoeuvring%20Control/) | 3-DOF MMG model, 19 SRUKF-identified params, [u,v,r,ψ,x,y], RK4 (Ts=0.08 s) | 12 | 5 | 60 |
| [`Active Suspension 6x6 EV Full Model/`](../case-study/Active%20Suspension%206x6%20EV%20Full%20Model/) | 40-state 20-DOF (body + 6 wheels + motors + 5-DOF human biodynamic), ZOH (Ts=5 ms) | 18 | 5 | 90 |

**Drill String (17):** OpenLoop, PID, ADRC, SMC, LQR, MPC, MRAC, GainScheduled, L1Adaptive, NeuralPID, ILC, DynaCtrl, CEMCtrl, ScenarioMPC, KoopmanMPC, ESNCtrl, CBFSafety.

**Wind-Wave (16):** Passive, Reactive, PID, ADRC, SMC, LQR, MPC, MRAC, L1Adaptive, ILC, DynaCtrl, CEMCtrl, ScenarioMPC, KoopmanMPC, ESNCtrl, CBFSafety.

**EHFS (14):** OpenLoop, PID, ADRC, SMC, MPC, LQR, MRAC, L1Adaptive, FeedbackLin, NeuralPID, ILC, GainScheduled, HinfODFCCtrl, HinfCascadeCtrl.

**EV 6×6 (18):** Passive, PD, GAOptPD, PSOOptPD, DEOptPD, PID, LQR, LQG, MPC, ADRC, SMC, MRAC, FuzzyPID, TubeMPC, ILC, CBF, L1Adaptive, ScenarioMPC.

**Ship Manoeuvring (12):** OpenLoop, PID, SMC, ASMC, MPC, LQR, MRAC, L1Adaptive, GainScheduled, ADRC, NeuralPID, ILC.

#### Spec-only stubs and unfilled scaffolds — `README.md`/PDF present, no real `sim/` content, not built

The exact count drifts as new studies are proposed; see [`docs/case_study_status.md`](case_study_status.md) for the live auto-generated status table (regenerate via `tools/case_study_tracker.py`) and `CLAUDE.md` Section "Spec-only stubs" for per-study tribal knowledge. Note: the tracker's "On-going" status only checks for `sim/` + `logs/` + `config/` presence, not real content — a few directories created by `tools/new_case_study.py` pass that check while still containing only placeholder dynamics and a single `OpenLoop` controller. Check the study's own `README.md` before treating it as a finished study.

All C++ targets are listed in `compile.bat` / `compile.sh`; Python-only studies are **not** in `CMakeLists.txt` or the compile scripts. The standalone boiler demo [ex21_boiler_turbine_case_study.cpp](../examples/ex21_boiler_turbine_case_study.cpp) exercises the boiler plant without the full multi-controller sweep.

### 3.4 Tests (`tests/`)

- `test_controllers.cpp` — per-class unit tests (custom `test_framework.h` harness)
- `test_tuners_extended.cpp` — tuner suite tests (covers all 8 strategies)
- `test_integration.cpp` — end-to-end closed-loop tests (c2d+MPC, N4SID+GPC adaptive pipeline)
- `test_catch2_advanced.cpp` — main Catch2 v3 regression suite (~95 test cases). Tags include: `[pid]`, `[lqr]`, `[mpc]`, `[smc]`, `[adrc]`, `[mhe]`, `[mhe_constraints]`, `[mhe_polytopic]`, `[lqr_factory]`, `[delay_wrapper]`, `[gain_scheduled]`, `[mimo_nugap]`, `[ilc]`, `[sindy]`, `[koopman]`, `[l1adaptive]`, `[cbf]`, `[gp]`, `[esn]`, `[neuralpid]`, `[cem]`, `[dyna]`, `[scenario_mpc]`, `[bayesian_optimizer]`, `[registry]`, `[monitor]`, `[deepc]`, `[grey_box]`, `[recursive_grey_box]`, `[gp_residual]`, `[mismatch_detector]`, `[basic_pid]`, `[basic_smc]`, `[dae_system]`, `[dae_c2d]`, `[dae_ekf]`, `[genetic_algorithm]`, `[pso]`, `[de]`, `[repetitive]`, `[extremum_seeker]`
- `test_catch2_pilot.cpp` — pilot Catch2 v3 tests (5 cases): LQRAdapter MIMO `computeVec()`, EKF scaled-epsilon Jacobian, PID DoM derivative suppression, 2DOF b_weight overshoot, observer telemetry
- `test_stability_margins.cpp` — 3 `[stability_margins]` cases
- `test_embedded_subset.cpp` — 13 tests, links only Catch2 (no Eigen); verifies `BasicPID<float>`, `BasicSMC<float>`, `DiscreteIntegrator`, `FixedRateFilter`, `RingBuffer` have zero Eigen dependency
- Per-study regression tests: `test_boiler_regression`, `test_tugsim_regression`, `test_solar_regression`, `test_humid_regression`, `test_susp_regression`, `test_buck_boost_regression`, `test_solar_cooker_regression`, `test_sotec_regression`, `test_smismo_regression` (6 cases each)
- `test_framework.h` — lightweight assertion macros for the custom harness

**Current totals:** run `conda run -n soft_robotics -- python run.py` to get the live count. All counts are unverified until a clean run confirms them.

### 3.5 Scripts (`scripts/`)

- `tune_all.cpp` - runs every tuner against canonical plants
- `simulate_all.cpp` - closed-loop simulation matrix
- `realtime_all.cpp` - RT timing benchmark (links `pthread` on POSIX)
- `create_controller.py`, `generate_test_data.py` - Python helpers

### 3.6 Cheatsheets (`cheatsheet/`)

Quick-reference notes covering: tuning methods, controller categories, controller list, system identification (FOPDT, ARMAX, N4SID), model evaluation, and the full control-design pipeline.

---

## 4. Usage Guide

### 4.1 Minimum Closed-Loop Example (SISO PID)

```cpp
#include "ControllerToolbox.h"

const double Ts = 0.01;
ctrl::TransferFunction G({0.0048, 0.0047}, {1.0, -1.81, 0.819}, Ts);
ctrl::StateSpace sys = ctrl::tf2ss(G);

ctrl::PIDParams pp; pp.Kp = 1.0; pp.Ki = 0.1; pp.Kd = 0.05; pp.N = 100.0;
ctrl::DiscretePID pid(pp, Ts);

Eigen::VectorXd x = Eigen::VectorXd::Zero(sys.stateSize());
double r = 1.0, y = 0.0;
for (int k = 0; k < 500; ++k) {
    double u = pid.compute(r - y);
    Eigen::VectorXd uv(1); uv << u;
    y = ctrl::ssStep(sys, x, uv)(0);
}
```

### 4.2 Auto-Tuning Workflow (Relay -> PID)

```cpp
ctrl::RelayTunerConfig cfg; cfg.relayAmplitude = 1.0; cfg.cyclesRequired = 3;
ctrl::RelayAutoTuner tuner(cfg, Ts);

while (!tuner.isDone()) {
    double u = tuner.step(y);                 // run inside your sim/plant loop
    // step plant with u, update y
}
ctrl::PIDParams pp = tuner.computePIDParams(ctrl::PIDTuningRule::TyreusLuyben);
ctrl::DiscretePID pid(pp, Ts);
```

### 4.3 LQR + Kalman = LQG (Output Feedback)

```cpp
ctrl::LQRParams lqr_p = ctrl::LQRWeightTuner::brysonMethod(xmax, umax);
Eigen::MatrixXd Qn = 1e-4 * Eigen::MatrixXd::Identity(n, n);
Eigen::MatrixXd Rn = 0.01 * Eigen::MatrixXd::Identity(p, p);

ctrl::DiscreteLQG lqg(plant, lqr_p, Qn, Rn);
Eigen::VectorXd du = lqg.step(y_noisy, u_prev, x_ref);
```

### 4.4 MPC with Online Re-Linearisation (Adaptive MPC)

```cpp
auto rec = ctrl::MPCHorizonTuner::recommend(plant, Ts);
ctrl::MPCParams mp; mp.Np = rec.Np; mp.Nc = rec.Nc;
mp.rho_y = rec.rho_y; mp.rho_u = rec.rho_u;
ctrl::DiscreteMPC mpc(plant, mp);

for (int k = 0; k < N; ++k) {
    if (operatingPointChanged()) mpc.setPlant(reLinearise(x_now));   // hot-swap model
    Eigen::VectorXd du = mpc.computeRef(x_now, r_ref);
    // apply du, advance plant
}
```

### 4.5 Fuzzy PID and Fuzzy Supervisor

```cpp
// -- FuzzyPID (HVAC, motor, marine - any axis needing smooth nonlinear P+D+I) --
ctrl::FuzzyPIDParams fp;
fp.pd.e_scale  = 2.0;      // error value considered "large"  (plant-specific units)
fp.pd.de_scale = 0.2;      // error rate considered "large"   (units/s)
fp.pd.u_scale  = 3.0;      // maps fuzzy output [-1,1] to output units
fp.pd.uMin = 0.0; fp.pd.uMax = 3.0;
fp.Ki  = 0.006;   fp.Kb = 1.0;    // integral gain + anti-windup
fp.uMin = 0.0;    fp.uMax = 3.0;
ctrl::FuzzyPID fuzzy(fp, Ts);

double u = fuzzy.compute(r - y);   // call once per sample step

// -- FuzzySupervisor (adaptive MPC re-linearisation trigger) ------------------
ctrl::SupervisorParams sp;
sp.e_threshold      = 5.0;   // |error| at which "Large" fires
sp.trend_threshold  = 0.5;   // d|e|/dt at which "Increasing" fires
sp.signal_threshold = 0.5;   // fuzzy output level that triggers action
sp.cooldown_steps   = 20;    // steps before re-triggering is allowed
ctrl::FuzzySupervisor supervisor(sp, Ts);

ctrl::SupervisorDecision dec = supervisor.update(std::abs(r - y));
if (dec.relinearize)
    mpc.setPlant(reLinearise(current_state));   // adapt MPC model

// -- Custom FuzzySystem (Takagi-Sugeno gain scheduling) -----------------------
ctrl::FuzzySystem sys;
sys.params.inference = ctrl::InferenceMethod::TakagiSugeno;
sys.params.defuzz    = ctrl::DefuzzMethod::WeightedAverage;

ctrl::LinguisticVariable v; v.name="theta"; v.lo=0; v.hi=1.0;
v.terms.push_back({"Near", ctrl::mfGaussian(0.0, 0.15)});
v.terms.push_back({"Far",  ctrl::mfGaussian(0.5, 0.15)});
sys.addInput(v);

ctrl::LinguisticVariable vout; vout.name="weight"; vout.lo=0; vout.hi=1.0;
vout.terms.push_back({"w_hi", ctrl::mfSingleton(1.0)});
vout.terms.push_back({"w_lo", ctrl::mfSingleton(0.0)});
sys.addOutput(vout);

ctrl::Rule r1; r1.antecedents.push_back({0,0}); r1.consequent_term_idx=0; sys.addRule(r1);
ctrl::Rule r2; r2.antecedents.push_back({0,1}); r2.consequent_term_idx=1; sys.addRule(r2);

double w = sys.evaluate({std::abs(theta)});   // weight for gain-scheduled blend
```

### 4.6 Composed Controllers (`ControllerStack`)

```cpp
auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, Ts);
stack->addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "PID");
stack->addController(std::make_shared<ctrl::DiscreteSMC>(sp, Ts), "SMC",
    1.0, [](double e, double /*last_out*/){ return std::abs(e) > 5.0; });  // SMC only when error large
double u = stack->compute(error);
```

**Condition callback signature:** `std::function<bool(double error, double last_output)>` -- both arguments are always passed; use `(double e, double)` to ignore `last_output`.

**Corrector patterns supported:**
- **Cascade** (`Additive` mode): inner loop runs at fast rate, outer provides setpoint -- see `ex42_pid_inner_mpc_outer.cpp`.
- **Additive** (`Additive` mode): `u_total = u_primary + u_corrector` -- see `ex47_esc_additive_pid.cpp`.
- **Observer+SF** (manual, not via stack): estimator provides `setState()` to feedback law -- see `ex50_ekf_mpc.cpp`.
- **Supervisory / Bumpless** (`Supervisory` mode): condition switches between controllers -- see `ex54_bumpless_transfer.cpp`.

### 4.7 Background Tuning, RT Reads

```cpp
ctrl::AtomicParamBuffer<ctrl::PIDParams> buf(pp_initial);

// Real-time thread:
auto p = buf.read();                        // seqlock-protected copy - no blocking
pid.setParams(p);
double u = pid.compute(error);

// Background thread (tuner):
buf.publish(newly_computed_params);         // atomic swap
```

---

## 5. Class Reference

> Each entry lists: **Purpose**, **Inputs / parameters**, **Outputs / returns**, and **key methods**. Method signatures are abbreviated for brevity - see the header for full declarations.

<a name="51-core-types-iplantmodel"></a>
### 5.1 Core Types ([PlantModel.h](../lib/PlantModel.h))

#### `TransferFunction`
- **Purpose:** SISO discrete-time TF in `z^-^1` form: `H(z^-^1) = num / den`.
- **Inputs:** `num` (vector `{b0,...,bm}`), `den` (monic vector `{1,a1,...,an}`), `Ts` (sample time, seconds).
- **Throws:** `std::invalid_argument` if `den[0] != 1`.
- **Methods:** `order() -> int`.

#### `StateSpace`
- **Purpose:** Discrete-time SS model `x[k+1] = A x + B u`, `y = C x + D u`.
- **Inputs:** Eigen `MatrixXd` A (n*n), B (n*m), C (p*n), D (p*m), `Ts`.
- **Methods:** `stateSize()`, `inputSize()`, `outputSize()` - all return `int`.

#### `StateSpace tf2ss(const TransferFunction&)`
SISO TF -> controllable canonical SS conversion.

#### `Eigen::VectorXd ssStep(const StateSpace&, Eigen::Ref<Eigen::VectorXd> x, const Eigen::VectorXd& u)`
- **Returns:** `y[k] = C x + D u`. **Side effect:** updates `x` in-place to `x[k+1]`.

#### `std::pair<Eigen::VectorXd, Eigen::VectorXd> ssStepCopy(const StateSpace&, const Eigen::VectorXd& x, const Eigen::VectorXd& u)`
- **Returns:** `{y[k], x[k+1]}` without modifying `x`. Semantically identical to `ssStep` but non-mutating.
- **When to use:** Preferred from Python bindings (pybind11 cannot bind `Eigen::Ref<>` out-params to NumPy arrays); also useful in functional-style simulation loops where mutation of `x` is undesirable.

#### `std::unordered_map<std::string, bool> ctrl::features()` ([Features.h](../lib/Features.h))
- **Purpose:** Runtime discovery of which optional modules were compiled in.
- **Returns:** Map with keys `"hinf"`, `"subspace"`, `"fuzzy"`, `"function_approx"`, `"advanced_kalman"` each mapping to `true` if the corresponding `CTRL_HAS_*` flag was defined at compile time.
- **Typical use:** Python bindings and plugin-discovery code that cannot inspect compile-time `#if` guards.

#### `IController` ([IController.h](../lib/IController.h))
- **Purpose:** Abstract base for all SISO controllers; uniform interface for stacking and tuning.
- **Pure-virtual:** `compute(double signal) -> double`, `reset()`, `sampleTime() const -> double`.
- **Convention:** `signal` is the **error** `e = r - y` for tracking controllers, the **plant output** for optimisation controllers (ESC) and observer-based controllers (ADRC, LQG).
- **Sign-convention introspection (`signConvention()`):** Virtual, returns a `SignConvention` enum (`Unspecified` by default; audited overrides return `TrackingErrorRMinusY`, `TrackingErrorYMinusR`, `PlantOutput`, `CostSignal`, or `Other`). Lets a caller query a controller's `compute()` convention at runtime instead of only from docs - the library intentionally does **not** enforce one convention across controllers. See [forensic_reconstruction.md Phase 0.3](forensic_reconstruction.md#phase-0) for the full per-controller mapping.
- **Observer (telemetry):** `attachObserver(IControllerObserver*)` stores a non-owning raw pointer -- caller must ensure the observer outlives the controller. `attachObserver(std::shared_ptr<IControllerObserver>)` co-owns the observer, safe for Python bindings and dynamically-allocated observers. `detachObserver()` releases both.

---

### 5.2 Controllers

#### `DiscretePID` ([DiscretePID.h](../lib/DiscretePID.h))
- **Purpose:** Backward-Euler PID with filtered derivative and back-calculation anti-windup.
- **Parameters (`PIDParams`):** `Kp, Ki, Kd, N` (derivative filter coefficient), `uMin/uMax` (saturation), `Kb` (anti-windup gain).
- **Returns:** Scalar control `u[k]` (saturated).
- **Methods:** `compute(error)`, `reset()`, `setParams(p)`, `params()`, `lastOutput()`.
- **Integral law (backward Euler):** `I[k] = I[k-1] + Ki.Ts.e[k] + Kb.(u_sat[k] - u_unsat[k])`. The current-step error `e[k]` is included in both `I[k]` and `u[k]` before the state is advanced, matching true backward Euler discretisation.
- **Constraints:** `Ki*Ts < 2*Kp` for discrete stability (see [deployment.md Section 1](deployment.md#1-parameter-constraints-by-controller)).

#### `DiscreteLQR` ([DiscreteLQR.h](../lib/DiscreteLQR.h))
- **Purpose:** Optimal full-state feedback `u = -K*(x - x_ref) + u_ff` with DARE solved offline via value iteration.
- **Parameters (`LQRParams`):** `Q` (n*n, PSD state cost), `R` (m*m, PD control cost).
- **Inputs:** `compute(x, x_ref = \emptyset, u_ff = \emptyset)` - full state vector required.
- **Returns:** `Eigen::VectorXd u[k]` (size m).
- **Methods:** `gainMatrix()`, `riccatiSolution()`, `dareConverged()`, `dareIterations()`, `sampleTime()`.
- **Helper:** `LQRAdapter` (Part 34) - wraps `DiscreteLQR` as `IController`. Two constructors: reference-capture (`LQRAdapter(lqr, state_fn)`) and owning (`LQRAdapter(owned_lqr, state_fn)`). Free function `makeLQRController(sys, lqr_params, state_fn)` creates a `shared_ptr<IController>`-compatible instance in one call — use this for `AutoGainScheduler`/`GainScheduledController` `design_fn` callbacks. MIMO note: `compute()` returns `u[0]` only; use `computeVec()` for the full vector.

#### `DiscreteMPC` ([DiscreteMPC.h](../lib/DiscreteMPC.h))
- **Purpose:** Condensed receding-horizon QP with hard box constraints on `Deltau` and `u`.
- **Parameters (`MPCParams`):** `Np` (prediction horizon), `Nc` (control horizon, `Nc <= Np`), `rho_y` (output weight), `rho_u` (move-suppression weight), `uMin/uMax`, `duMin/duMax`.
- **Inputs:** `computeRef(x_current, r_ref)` - current state and stacked reference.
- **Returns:** Optimal Deltau vector (first move of the receding horizon).
- **Prediction formula:** `Y = F.x + G_u.u_prev + Phi.DeltaU`, where `F(i) = C.A^(i+1)`, `G_u(i) = Sigma_{j=0}^{i} C.A^j.B` (cumulative step response, accounts for u_prev baseline), and `Phi(i,j) = C.A^(i-j).B`. All three matrices are pre-built in the constructor.
- **Methods:** `computeRef(...)`, `setPlant(plant)` (online re-linearisation), `setState(x)`, `setParams(p)`, `compute(error)` (SISO convenience).
- **Performance:** Condensed matrices F, G_u, Phi, H are pre-built in the constructor; per-step cost is one LDLT solve.

#### `DiscreteLQG` ([DiscreteLQG.h](../lib/DiscreteLQG.h))
- **Purpose:** LQR on Kalman-estimated state - output-feedback optimal control (separation principle).
- **Constructor:** `(plant, LQRParams, Q_noise, R_noise, P0 = I)`.
- **Inputs:** `step(y, u_prev, x_ref) -> u[k]` for MIMO; SISO scalar overload `compute(double y_scalar)` needs prior `setReference()` + `setUPrev()`. `compute()` honours the hold-last-on-NaN contract (returns the previous `u` on non-finite input).
- **Returns:** Control vector `u[k]`. Internal state estimate via `stateEstimate()`.
- **Methods:** `step(...)`, `compute(...)`, `setReference(x_ref)`, `setUPrev(u)`, `reset()`, `gainMatrix()`.
- **D != 0 note:** If the plant's D matrix is non-zero, a `std::cerr` warning is printed at construction. The Kalman innovation `y - C.x^ - D.u` must use `u[k-1]` (one step stale) because `u[k]` has not yet been computed at update time. Accuracy degrades proportionally to D's magnitude. Set D = 0 in the model for accurate filtering.

#### `DiscreteSMC` ([DiscreteSMC.h](../lib/DiscreteSMC.h))
- **Purpose:** First-order SMC with sliding surface `s = c_e.e + c_de.(e - e_prev)` and saturation `sat(s/phi)` to reduce chattering.
- **Sign convention (IMPORTANT):** `compute()` takes `e = y - r` (the **reverse** of DiscretePID's `e = r - y`). The sliding law `u = -K.sat(s/phi)` requires `s`, and therefore the error, to grow with `y - r` for a positive-gain plant. `signConvention()` returns `TrackingErrorYMinusR`. (Same convention for `SuperTwistingSMC`.) See `CONTRIBUTING.md#sign-conventions`.
- **Parameters (`SMCParams`):** `c_e`, `c_de`, `K` (switching gain), `phi` (boundary layer thickness), `uMin/uMax`.
- **Inputs:** `compute(error)` with `error = y - r`.
- **Returns:** Saturated control `u[k]`.
- **Methods:** `compute(e)`, `slidingSurface()`, `setParams(p)`, `reset()`.
- **`c_de` sizing:** `c_de` absorbs the sample time `Ts`. To match a continuous-time slope `lambda` [1/s], set `c_de = lambda . Ts`. Passing a continuous-time `lambda` directly as `c_de` over-weights the rate term by a factor of `1/Ts`.

#### `DiscreteADRC` ([DiscreteADRC.h](../lib/DiscreteADRC.h))
- **Purpose:** Bandwidth-parameterised 2nd-order Linear ADRC - ESO estimates total disturbance, PD law cancels it.
- **Parameters (`ADRCParams`):** `omega_o` (observer BW), `omega_c` (controller BW), `b0` (approximate input gain), `uMin/uMax`.
- **Inputs:** `compute(y)` (plant output, **not error**); set reference first via `setReference(r)` or use `computeTracking(y, r)`.
- **Returns:** Scalar control. Internal ESO state available via `esoState() -> Vector3d {z1, z2, z3}`.
- **Stability:** Requires `omega_o * Ts < 2` (forward-Euler limit).

#### `DiscreteLeadLag` ([DiscreteLeadLag.h](../lib/DiscreteLeadLag.h))
- **Purpose:** Tustin-discretised first-order compensator `C(s) = K.(s + z_c)/(s + p_c)`. Lead if `p > z`; lag if `p < z`.
- **Parameters (`LeadLagParams`):** `continuousZero z_c`, `continuousPole p_c`, `gain K`.
- **Inputs:** `compute(u)` - typically the error or plant output to filter.
- **Returns:** Filtered output `y[k]` via `y = b0.u[k] + b1.u[k-1] - a1.y[k-1]`.
- **Methods:** `compute(u)`, `setParams(p)`, `phaseAt(omega_rad_s)`.

#### `SmithPredictor` ([SmithPredictor.h](../lib/SmithPredictor.h))
- **Purpose:** Dead-time compensator wrapping any `IController`; replaces feedback delay with internal-model prediction.
- **Constructor:** `(shared_ptr<IController> inner, StateSpace delayModel, int delaySteps)` - `delayModel` is the delay-free plant.
- **Inputs:** `compute(error)` - closed-loop error `r - y`.
- **Returns:** Inner controller's output, with the modified error including the Smith correction term.
- **Methods:** `innerController()` for runtime re-tuning. Delay buffer is a fixed-size circular buffer (no RT allocation).
- **D feedthrough:** The internal model output `yhat[k] = C.x^ + D.u_prev` uses the previous control input `u[k-1]` for the feedthrough term. `u_prev_` is initialised to zero and updated each step.

#### `ExtremumSeeker` ([ExtremumSeeker.h](../lib/ExtremumSeeker.h))
- **Purpose:** Perturbation-based optimiser - injects dither, demodulates output, integrates gradient to climb to the extremum of an unknown static cost surface.
- **Parameters (`ExtremumSeekerParams`):** `perturbAmp`, `perturbFreq`, `lpfCutoff`, `hpfCutoff`, `integGain`, `seekMinimum` (true -> min, false -> max).
- **Inputs:** `compute(signal)` - `signal` is the **plant output / cost**, **not** an error.
- **Returns:** Plant input `u = theta + dither` (absolute, not deviation).
- **Methods:** `currentEstimate()` -> integrator state theta.
- **Convergence:** ESC does not declare convergence; user must implement a stagnation window.

#### `RepetitiveController` ([RepetitiveController.h](../lib/RepetitiveController.h))
- **Purpose:** Internal Model Principle (IMP) controller for periodic reference/disturbance rejection. Stores one period of correction signal in a circular buffer and blends it with current error via a Q-filter.
- **Parameters (`RepetitiveParams`):** `periodSteps` (period length in samples), `gain`, `qCutoff` (Q-filter cutoff, stability robustness knob).
- **Inputs:** `compute(error)` - standard error `e = r - y`.
- **Returns:** Control correction `u_rc[k]` to add to a primary controller's output.
- **Methods:** `compute(e)`, `reset()`, `bufferSize()`.

#### `FeedforwardController` ([FeedforwardController.h](../lib/FeedforwardController.h))
- **Purpose:** Reference model feedforward - computes `u_ff = Kff * r` (static) or filters the reference through a model-inverse transfer function. Use additively with a feedback controller: `u = u_feedback + u_ff`.
- **Parameters (`FeedforwardParams`):** `gain` (static gain), optional `referenceModel` (`StateSpace` for dynamic FF).
- **Inputs:** `compute(reference)` - the setpoint signal.
- **Returns:** Feedforward control signal.

#### `GeneralizedPredictiveController` ([GeneralizedPredictiveControl.h](../lib/GeneralizedPredictiveControl.h))
- **Purpose:** GPC based on the CARIMA (Controlled AutoRegressive Integrated Moving Average) process model. Supports online model adaptation via RLS using `setPlant()`.
- **Parameters (`GPCParams`):** `Np` (prediction horizon), `Nu` (control horizon), `rho_y` (output weight), `rho_u` (move-suppression weight), `uMin/uMax`.
- **Inputs:** `computeRef(y_current, r_ref)` -- current plant output and reference.
- **Returns:** Optimal delta-u (first move of the receding horizon).
- **Methods:** `computeRef(y, r)`, `setPlant(plant)` (hot-swap for adaptive MPC), `augmentedState()`, `reset()`.
- **Prediction model:** CARIMA `A(q^-1) * Delta * y = B(q^-1) * u + e`; augmented state includes integrator state; Ga (CARIMA step-response matrix) differs from standard MPC Phi by a `C*B` correction term.
- **Difference from MPC:** GPC operates on `Delta u` (increments) without requiring an initial state `x`; adaptive to unknown plants via RLS feedback.

#### `DiscreteHinf` ([DiscreteHinf.h](../lib/DiscreteHinf.h))
- **Purpose:** H-infinity synthesis via gamma-bisection. Also supports mixed-sensitivity design (`mixedSensitivity()`) and mu-synthesis via full DK-iteration (`solveMuSyn()`).
- **Parameters (`HinfParams`):** `gammaInit` (initial gamma upper bound), `gammaTol` (bisection tolerance), `maxIter`.
- **`solve(plant, W1, W2, W3)` -> `HinfResult`:** Solves the standard weighted H-infinity problem. `HinfResult` fields: `controller` (`StateSpace`), `achievedGamma` (actual H-infinity norm), `converged`.
- **`mixedSensitivity(plant, W1, W2, W3)` -> `HinfResult`:** Convenience wrapper for `[W1*S; W2*KS; W3*T]` mixed-sensitivity design.
- **`solveMuSyn(plant, params)` -> `MuSynResult`:** Full DK-iteration mu-synthesis. `MuSynParams` fields: `maxDKIter`, `useRationalD` (if true, fits first-order rational D_j(z) per frequency channel). `MuSynResult` includes `controller`, `dFilters_L`, `dFilters_R` (rational D filters per channel), `muHistory` (mu upper bound per iteration).

#### `DiscreteH2` ([DiscreteH2.h](../lib/DiscreteH2.h)) *(requires `CTRL_HAS_HINF`)*
- **Purpose:** Discrete-time H2-optimal (LQG) dynamic output-feedback synthesis. Minimises the closed-loop H2 norm `||F_l(P, K)||_2` via the LQG separation principle (a control DARE + a dual filter DARE, each reduced to a no-cross-term DARE and solved by `DiscreteLQR::solveDARE`).
- **Plant format:** Reuses `GeneralisedPlant` from [DiscreteHinf.h](../lib/DiscreteHinf.h). **Regularity assumptions (enforced by throwing):** `D11 = 0`, `D22 = 0`, `D12` full column rank, `D21` full row rank. Because most `MixedSensitivity::build()` plants have `D11 != 0`, pair `DiscreteH2` with a hand-built generalised plant (see [examples/ex88_h2_synthesis.cpp](../examples/ex88_h2_synthesis.cpp)).
- **`solve(P, params = {}) -> H2Result`:** Fields `feasible`, `achievedH2Norm`, `Ts`, controller matrices `Ak/Bk/Ck/Dk` (Dk always 0), Riccati diagnostics `X/Y`, `dareConvX/dareConvY`. Check `feasible` before constructing.
- **Controller:** `DiscreteH2(result)` implements `IController`: `compute(y)` (SISO; `signal` is the measurement `y`, **not** the tracking error - same convention as DiscreteHinf), `computeVec(y)` (MIMO), `reset()`, plus read-only accessors `Ak()/Bk()/Ck()/Dk()`, `controllerState()`, `achievedH2Norm()`. Honours the hold-last-on-NaN contract.

#### Fuzzy Logic Module ([FuzzyLogic.h](../lib/FuzzyLogic.h))

The fuzzy module provides a self-contained Mamdani / Takagi-Sugeno inference engine together with three ready-to-use `IController` wrappers. All classes live in the `ctrl` namespace and are included via `ControllerToolbox.h`.

##### Membership function factories
| Function | Signature | Shape |
|----------|-----------|-------|
| `mfTriangular` | `(a, c, b)` | Triangle peaking at `c`, zero at `a` and `b` |
| `mfTrapezoidal` | `(a, b, c, d)` | Trapezoid: ramps up `[a,b]`, flat `[b,c]`, ramps down `[c,d]` |
| `mfGaussian` | `(mean, sigma)` | `exp(-0.5*((x-mean)/sigma)^2)` |
| `mfSingleton` | `(value)` | 1 only at `x == value` (for TS consequents) |
| `mfShoulderLeft` | `(a, b)` | 1 for `x <= a`, linear 1->0 from `a` to `b` |
| `mfShoulderRight` | `(a, b)` | 0 for `x <= a`, linear 0->1 from `a` to `b` |

All factories return `ctrl::MF = std::function<double(double)>` capturing parameters by value.

##### `LinguisticVariable`
- Holds `name`, universe `[lo, hi]`, and a `std::vector<LinguisticTerm>` (name + MF pairs).
- `fuzzify(x)` -> `std::vector<double>` of membership degrees (clamped to `[0,1]`).
- `termIndex(name)` -> int (-1 if not found).

##### `FuzzySystem`
- **Purpose:** Core inference engine supporting both Mamdani and TS inference on a single output variable.
- **Build:** `addInput(var)`, `addOutput(var)`, `addRule(rule)`.
- **Evaluate:** `evaluate(inputs) -> double` - fuzzifies all inputs, fires all rules (product AND, max aggregation), defuzzifies.
- **Params (`FuzzySystemParams`):** `inference` (Mamdani/TakagiSugeno), `defuzz` (CoG/WeightedAverage), `cog_resolution` (101 default), `uMin/uMax`.
- **CoG defuzz:** discrete grid of `cog_resolution` points over the output universe; each point takes the max over all clipped output MFs; weighted centroid.
- **TS/WeightedAverage defuzz:** strength-weighted sum over the peak location of each output term (grid-searched at 51 points).
- **Rule format (`Rule`):** `antecedents` (vector of `{input_idx, term_idx}`), `consequent_term_idx`, optional `weight` (default 1.0). AND connector is product t-norm.

##### `FuzzyPD` (implements `IController`)
- **Purpose:** Convenience Mamdani PD controller for one axis; builds its own `FuzzySystem` automatically from the canonical 5-term partition `{NL, NS, ZE, PS, PL}` with 25-rule diagonal rule table.
- **Parameters (`FuzzyPDParams`):** `e_scale` (normalises error to `[-1,1]`), `de_scale` (normalises error rate), `u_scale` (scales output back to physical units), `uMin/uMax`.
- **Inputs:** `compute(error)` - normalises `e` and `de = (e - e_prev)/Ts`, runs inference, scales and clamps output.
- **Methods:** `compute(e)`, `reset()`, `setParams(p)`, `params()`, `lastOutput()`, `sampleTime()`.

##### `FuzzyPID` (implements `IController`)
- **Purpose:** `FuzzyPD` block + crisp backward-Euler integral with back-calculation anti-windup.
- **Architecture:** `u = clamp(u_PD + integral, uMin, uMax)` where the integral accumulates `Ki*Ts*e` and receives the standard back-calculation correction `Kb*(u_sat - u_unsat)`.
- **Parameters (`FuzzyPIDParams`):** contains `FuzzyPDParams pd` plus `Ki`, `Kb`, `uMin/uMax` (outer saturation, may differ from `pd.uMin/uMax`).
- **Methods:** `compute(e)`, `reset()`, `setParams(p)`, `bumplessInit(u_target, error)`, `lastOutput()`.

##### `FuzzySupervisor`
- **Purpose:** Monitors a closed-loop error channel and returns a `SupervisorDecision` indicating whether the underlying controller's linearised plant model should be refreshed.
- **Inputs:** `update(abs_error)` - called once per step with the scalar absolute error for this axis.
- **Output (`SupervisorDecision`):** `relinearize_signal` [0,1], `relinearize` (bool, thresholded with cooldown), `error_norm`, `trend`.
- **Internal logic:** 9-rule Mamdani system with 2 inputs (normalised error magnitude, normalised error trend) and 1 output (re-linearisation signal). A configurable cooldown prevents rapid oscillatory triggering.
- **Parameters (`SupervisorParams`):** `e_threshold` (error at which term "Large" fires), `trend_threshold` (d|e|/dt at which "Increasing" fires), `signal_threshold` (threshold for boolean `relinearize`), `cooldown_steps`.
- **Typical integration:** call `mpc.setPlant(reLinearise(nu))` when `dec.relinearize == true`.

---

### 5.3 Estimators & Identification

#### `KalmanFilter` ([KalmanFilter.h](../lib/KalmanFilter.h))
- **Purpose:** Linear discrete Kalman filter (predict / update) with Joseph-form covariance update.
- **Constructor:** `(plant, Q_noise, R_noise, P0 = I)`.
- **Methods:** `predict(u)`, `update(y, u_current)`, `state()`, `covariance()`, `reset()`.
- **`step()` overloads:**
  - `step(y, u_prev)` -- combined predict+update; `u_current` defaults to `u_prev` (correct for `D = 0` plants).
  - `step(y, u_prev, u_current)` -- plain-reference overload; explicit current input for `D != 0` plants. Preferred from Python bindings (the default overload uses `std::optional<std::reference_wrapper<...>>` which pybind11 cannot auto-convert).
- **Floor:** `R_noise` has an automatic floor of `1e-12` per diagonal element to avoid division by zero.

#### `ExtendedKalmanFilter` ([ExtendedKalmanFilter.h](../lib/ExtendedKalmanFilter.h))
- **Purpose:** EKF for nonlinear state estimation. Linearises the dynamics and measurement functions around the current estimate at each step.
- **Constructor:** `(n, p, f, h, Fjac, Hjac, Q, R, Ts)` -- `n` states, `p` outputs, nonlinear functions `f(x,u)` and `h(x,u)`, Jacobian functions `Fjac(x,u)` and `Hjac(x,u)` (or pass `nullptr` for numerical differentiation with scaled epsilon).
- **Methods:** `predict(u)`, `update(y, u)`, `step(y, u_prev)`, `state()`, `covariance()`, `setState(x)`, `reset()`.
- **Numerical Jacobian:** When `Fjac = nullptr`, finite differences use `eps = max(1e-5 * |x_i|, 1e-8)` per element to handle heterogeneous state magnitudes (see [test_catch2_pilot.cpp] P12-17 fix).

#### `UnscentedKalmanFilter` ([UnscentedKalmanFilter.h](../lib/UnscentedKalmanFilter.h))
- **Purpose:** UKF using the unscented transform (2n+1 sigma points) for higher-order nonlinear estimation accuracy without Jacobians.
- **Constructor:** `(n, p, f, h, Q, R, Ts, MatrixXd(), alpha, beta, kappa)`.
- **Methods:** `predict(u)`, `update(y, u)`, `state()`, `covariance()`, `setState(x)`, `reset()`.
- **Alpha/kappa guidance:** For n=2, kappa=0: use `alpha >= 1/sqrt(n) = 0.707` to avoid negative `Wc0 = 1 - alpha^2 + beta`. With `alpha=1.0, beta=2.0, kappa=0`: `lambda=0`, `Wm0=0`, `Wc0=2 > 0`. This is the recommended configuration for 2-state systems.

#### `MovingHorizonEstimator` ([MovingHorizonEstimator.h](../lib/MovingHorizonEstimator.h))
- **Purpose:** MHE via condensed QP -- the dual of MPC for state estimation. Optimises the state trajectory over a moving window of N measurements with box constraints on process noise.
- **Constructor:** `(plant, Q_noise, R_noise, params)`.
- **Parameters (`MHEParams`):** `N` (horizon length), `wMin/wMax` (process noise bounds), `qpMaxIter`, `qpTol`.
- **Methods:** `initialize(x0, P0)`, `estimate(y, u)` -> `VectorXd x_hat`, `setHorizon(N)`, `setWeightMatrices(Q, R)`, `reset()`, `state()`, `lastConverged()`, `lastQpIters()`, `sampleTime()`.
- **Decision variable:** `z = [x_0; w_0; ...; w_{N-1}]` of size `n*(N+1)`. Condensed matrices `Psi_`, `Gamma_u_`, `C_bar_` are pre-built; per-step cost is one `GradientProjectionQP` solve.
- **Horizon ramp-up:** First N-1 calls use an effective horizon shorter than N; arrival cost `P0inv` weights the initial state uncertainty.
- **Backend:** Shared `GradientProjectionQP` solver (same as MPC and GPC); zero heap allocation after construction.

#### `FOPDTIdentifier` ([FOPDTIdentifier.h](../lib/FOPDTIdentifier.h))
- **Purpose:** Identifies a First-Order Plus Dead-Time (FOPDT) model from open-loop step response data.
- **Methods:** `identify(t, y, stepMag, method)` -> `FOPDTModel {K, tau, theta, fitRMSE}`; `evaluate(model, t, stepMag)` -> simulated response.
- **Methods:** Graphical (ZN tangent + 63.2% crossing) or optimization (golden-section on theta and tau).
- **Integration:** Result feeds directly into `StepResponseTuner::computePIDParams()` for IMC, ZN, Cohen-Coon, AMIGO tuning.

#### `SOPDTIdentifier` ([SOPDTIdentifier.h](../lib/SOPDTIdentifier.h))
- **Purpose:** Identifies a Second-Order Plus Dead-Time (SOPDT) model from open-loop step response data. Convention: `tau1 >= tau2`.
- **Constructor:** `(t_data, y_data, stepMag)`.
- **Methods:** `identify(method)` -> `SOPDTModel {K, tau1, tau2, theta, fitRMSE}`; `evaluate(model, t, stepMag)` -> simulated response; `static imcTuning(model, lambdaC)` -> `PIDParams`.
- **`SOPDTMethod`:** `Graphical` or `Optimization`.
- **Graphical algorithm:** ZN tangent -> `theta`; 63.2% crossing -> `tau_sum = tau1+tau2`; 28.3% crossing ratio `r` interpolated between FOPDT limit (r~0.332) and critically-damped SOPDT limit (r~0.530) to split into `tau1` and `tau2`.
- **Optimization:** Nested golden-section search on `(theta, tau1, tau2)` minimising RMSE.
- **IMC-PID (Rivera 1986):** `tau_eq = tau1+tau2`, `Kp = tau_eq / (K*(lambdaC + theta/2))`, `Ti = tau_eq`, `Td = tau1*tau2/tau_eq`.

#### `MRACController` ([MRACController.h](../lib/MRACController.h))
- **Purpose:** Discrete-time Model Reference Adaptive Control -- SISO, first-order reference model. Forces plant output to track y_m[k+1] = a_m*y_m + b_m*r via two-parameter Lyapunov adaptation with σ-modification and Euclidean projection.
- **Constructor:** `(MRACParams, Ts)`.
- **Convention:** `compute(y_plant)` takes plant output (ADRC convention, not error). Call `setReference(r)` before each `compute()`.
- **Adaptation law:** `θ[k+1] = θ[k] - Ts*(γ*e_m*φ + σ*θ[k])` with projection: if ‖θ‖ > theta_max -> θ <- θ*theta_max/‖θ‖.
- **Parameters (`MRACParams`):** `a_m`, `b_m` (reference model), `gamma_r`, `gamma_y` (rates), `sigma` (σ-modification, 0=off), `theta_max` (projection bound), `uMin/uMax`.
- **Methods:** `compute(y)`, `setReference(r)`, `reset()`, `theta_r()`, `theta_y()`, `modelOutput()`, `modelError()`.
- **Feasibility:** Minimum-phase plant required; gain sign must match γ_r, γ_y sign; persistent excitation needed for θ convergence.

#### `FeedbackLinearisationController` ([FeedbackLinearisation.h](../lib/FeedbackLinearisation.h))
- **Purpose:** Exact feedback linearisation for SISO affine-in-control systems ẋ = f(x) + g(x)*u, relative degree 1. Cancels nonlinear terms algebraically; inner IController drives the resulting virtual integrator.
- **Constructor:** `(DriftFn f, GainFn g, shared_ptr<IController> inner, FLParams, Ts)`.
- **Control law:** `u[k] = clamp((v - f(x[k], u[k-1])) / g_eff(x[k], u[k-1]), uMin, uMax)` where v = inner->compute(error).
- **Critical requirement:** `setState(x)` must be called before each `compute()` with the current measured or estimated plant state.
- **Parameters (`FLParams`):** `uMin`, `uMax`, `regularisationEps` (minimum |g| before clamping; preserves sign).
- **Feasibility:** Relative degree 1, minimum-phase zeros, g(x) ≠ 0 across operating region.
- **Relative degree 2 note:** For angle-output systems (pendulum), the inner PID must be tuned for a virtual double integrator; example gains: Kp=9, Ki=5, Kd=5 for poles {-1, -2±j}. See `ex56_feedback_linearisation.cpp`.

#### `RecursiveLeastSquares` ([RecursiveLeastSquares.h](../lib/RecursiveLeastSquares.h))
- **Purpose:** Online ARX parameter estimation using exponential forgetting (`lambda` factor).
- **Constructor:** `(na, nb, nk, lambda, P0_scale)` -- output order, input order, delay, forgetting factor, initial covariance scale.
- **Methods:** `update(y, u)` -> current ARX coefficients; `reset()`, `toTransferFunction()`, `toStateSpace()`.
- **Typical use:** Feed identified `StateSpace` into `GPC::setPlant()` for adaptive GPC (see `ex28_gpc_adaptive.cpp`).

#### `LinearisationHelper` ([LinearisationHelper.h](../lib/LinearisationHelper.h))
- **Purpose:** Numerical Jacobians and ZOH linearisation of continuous-time nonlinear models at a given operating point. Central-difference with step `h_i = ε*max(|x_i|, 1)` for heterogeneous state magnitudes.
- **Key functions:**
  - `jacobianX(f, x0, u0, eps=1e-4)` -> MatrixXd ∂f/∂x (n evaluations of f).
  - `jacobianU(f, x0, u0, eps=1e-4)` -> MatrixXd ∂f/∂u (m evaluations).
  - `lineariseAtPoint(f, x0, u0, Ts)` -> discrete StateSpace (ZOH, C=I, D=0).
  - `lineariseAtPoint(f, h, x0, u0, Ts)` -> discrete StateSpace with custom output h(x,u).
- **Note:** The `StateFunc`/`MeasFunc` type aliases are identical to those in `ExtendedKalmanFilter.h` -- safe to include both (C++ redeclaration of identical alias is valid).
- **Typical use:** Compute A, B at operating point -> `c2d(ZOH)` -> `DiscreteLQR` design -> apply gain to nonlinear plant.

#### `BalancedTruncation` ([BalancedTruncation.h](../lib/BalancedTruncation.h))
- **Purpose:** Moore (1981) model order reduction via balanced realisation. Replaces an n-state stable system with an r-state approximation preserving the most energetically significant modes. Provides a-priori H∞ error bound.
- **Functions:**
  - `balancedTruncate(sys, r)` -> `TruncationResult {reduced, hankelSingularValues, errorBound, isStable}`.
  - `suggestOrder(result, tol=0.01)` -> int -- smallest r such that error bound < tol x total_norm.
- **Algorithm:** Solve gramians via `SystemAnalysis::solveDiscreteLyapunov` -> Cholesky of P_c -> `SelfAdjointEigenSolver` of M = L_c'*P_o*L_c -> sort HSVs descending -> balanced transformation T_r = L_c*U*Σ^{-½} -> truncate.
- **Error bound:** ‖G - G_r‖∞ ≤ 2*Σᵢ₌ᵣ₊₁ⁿ σᵢ. Verified: actual DC gain deviation is always within this bound.
- **Constraint:** O(n⁶) Lyapunov solver -- use for n ≤ 10. Larger systems require Bartels-Stewart (not yet implemented).
- **Correct usage:** Design controller on `result.reduced`, then apply to the full-order plant. The error bound quantifies the performance degradation. See `ex58_balanced_truncation.cpp`.

#### `ZeroPhaseTrackingFilter` ([ZeroPhaseTrackingFilter.h](../lib/ZeroPhaseTrackingFilter.h))
- **Purpose:** ZPETC (Tomizuka 1987) feedforward prefilter. Inverts the minimum-phase part of a plant's numerator causally, normalises DC gain to 1, and produces zero phase error for min-phase zeros.
- **Functions:**
  - `transmissionZeros(sys)` -> `vector<complex<double>>` -- finite eigenvalues of the system matrix pencil `[[A-λI, B],[C, D]]` via `GeneralizedEigenSolver`.
  - `designZPETC(plant)` -> `ZPETCResult {filter, dcAmplitudeError, hasNMPZeros, zeros, nmpZeros}`.
- **Composite response:** G(z)*G_ff(z) = B⁻(z)/B⁻(1). For min-phase plants: G*G_ff = z^{-d} (unit magnitude, pure delay). For NMP: unit DC gain, amplitude error away from DC.
- **Key implementation detail:** The evaluation function `evalTF` must store C as `MatrixXcd` not `VectorXcd` -- Eigen's implicit reshape transposed a (1xn) row into an (nx1) column, producing a x50 error. Fixed: use matrix products `C_c * zIA.solve(B_c) + D_c`.
- **Python binding disambiguation:** `suggest_order` is overloaded -- VectorXd (SubspaceID) dispatches from `advanced_bindings.cpp`; TruncationResult (BalancedTruncation) from `analysis_bindings.cpp`. Wrapped in lambdas to resolve the C++ overload ambiguity.

#### `SubspaceID` ([SubspaceID.h](../lib/SubspaceID.h))
- **Purpose:** N4SID subspace identification of MIMO state-space models from PRBS or arbitrary excitation data.
- **Methods:** `n4sid(y, u, n, i)` -> `StateSpace`; `suggestOrder(y, u, maxOrder)` -> recommended model order from singular value elbow detection.
- **Inputs:** `y` (p x N output data matrix), `u` (m x N input data matrix), `n` (model order), `i` (block-row factor, typically n <= i <= 2n).

#### `FreqDomainIdentifier` ([FreqDomainIdentifier.h](../lib/FreqDomainIdentifier.h))
- **Purpose:** Frequency-domain SISO identification via Levy's method (1959) - fits a discrete `TransferFunction` directly to complex frequency-response samples (the inverse of `SystemAnalysis::getFrequencyResponse`). This is the classical single-shot fit behind MATLAB `invfreqz`.
- **Method:** `static fitLevy(freqs, response, num_order, den_order, Ts) -> FreqDomainFitResult { tf, rmse, full_rank }`.
- **Inputs:** `freqs` [rad/s] and complex `response` `H(e^{j.omega.Ts})` (same length); numerator/denominator orders; `Ts`. Throws if lengths differ or the system is underdetermined (`freqs.size() < num_order + 1 + den_order`).
- **Note:** Levy's fit carries a known high-frequency bias; for SK-iteration / real-pole magnitude fitting see [VectorFitting.h](../lib/VectorFitting.h).

#### `GainScheduledController` ([GainScheduledController.h](../lib/GainScheduledController.h)) *Part 20+23*
- **Purpose:** IController wrapper that interpolates between a sorted list of (p, IController) schedule points.
- **Modes:** `NearestNeighbor` (hard-switch; calls `bumplessInit` on the incoming controller when the active index changes -- Part 23 fix); `LinearBlend` (weighted average of adjacent controllers).
- **Methods:** `addSchedulePoint(p, ctrl)`, `setSchedulingParam(p)`, `compute(error)`, `lastOutput()` (not override).
- **Note:** `lastOutput()` is NOT a virtual override (no virtual lastOutput() in IController base).
- **LQR pattern:** Use `LQRAdapter` (which IS an IController) with a state-capturing lambda as the schedule point controller.

#### `GapMetric` ([GapMetric.h](../lib/GapMetric.h)) *Part 20*
- **Purpose:** Nu-gap upper bound (SISO chordal metric) for measuring plant model distance.
- **Functions:** `nuGap(P1, P2, freq_points=200)` -> scalar in [0,1]; `nuGapMatrix(models)` -> N x N symmetric distance matrix; `freqResponseGrid(sys, omega)`.
- **Limitation:** SISO only; throws `invalid_argument` for MIMO plants.

#### `LinearModelCluster` ([LinearModelCluster.h](../lib/LinearModelCluster.h)) *Part 20*
- **Purpose:** Single-linkage agglomerative clustering of plant models by nu-gap distance.
- **Functions:** `clusterByGap(gapMatrix, threshold)` -> `ClusterResult {labels, representatives, maxIntraGap, numClusters, threshold}`; `suggestGapThreshold(gapMatrix)`.

#### `LPVSystemID` ([LPVSystemID.h](../lib/LPVSystemID.h)) *Part 20*
- **Purpose:** Polynomial LPV system identification via QR regression.
- **Functions:** `identifyLPV(X, U, Y, sched, degree, Ts)` -> `LPVModel`; `identifyLPVFromIO(U, Y, sched, n_states)` (uses n4sid first).
- **Layout:** X is (n x N) column-major (each column = one time step). U is (m x N), Y is (p x N). **CRITICAL: NOT (N x n) row-major.**
- **LPVModel:** `frozen(p)` -> `StateSpace`; `evalA(p)`, `evalB(p)`.

#### `AutoGainScheduler` ([AutoGainScheduler.h](../lib/AutoGainScheduler.h)) *Part 20*
- **Purpose:** Pipeline: nonlinear plant -> equilibrium grid -> linearise -> gap cluster -> design controllers -> assemble GainScheduledController.
- **Functions:** `findEquilibrium(f, u_eq, x0)` (Newton-Raphson); `buildAutoGainScheduler(f, p_min, p_max, density, u_eq_fn, x0_fn, design_fn, Ts)`.
- **CRITICAL:** `design_fn` lambda must have trailing return type `-> std::shared_ptr<IController>` (GCC cannot deduce shared_ptr<Derived> -> shared_ptr<Base>).

#### `NonlinearMPC` ([NonlinearMPC.h](../lib/NonlinearMPC.h)) *Part 22*
- **Purpose:** Nonlinear MPC via Real-Time Iteration (RTI, Diehl 2005). Discrete-time dynamics `x[k+1] = f(x[k], u[k])`.
- **Params:** `NMPCParams {Np, Nu, rho_y, rho_u, uMin, uMax, qpMaxIter, qpTol, Ts, n_states, n_inputs, n_outputs}`.
- **Usage:** `setState(x)`, `setReference(y_ref)`, `computeRef(x, y_ref)` (MIMO) or `compute(error)` (SISO).
- **Algorithm:** linearise along warm-started trajectory, build time-varying condensed QP (Theta matrix), solve via FISTA. Theta built with `Phi = Phi * A_list[k]` (right-multiply, k descending from j to 0).
- **Note:** `lastOutput()` is NOT a virtual override.

#### `AdaptiveSmithPredictor` ([AdaptiveSmithPredictor.h](../lib/AdaptiveSmithPredictor.h)) *Part 22*
- **Purpose:** SmithPredictor with online dead-time estimation via cross-correlation `R_uy(tau) = sum u[k-tau]*y[k]`. Rebuilds SP when delay estimate changes.
- **Params:** `AdaptiveSPParams {maxDelaySteps, estimateInterval, bufferLen}`.
- **Usage:** `setPlantOutput(y)` before `compute(r-y)` each step (if not called, uses `y approx= -error` for r=0 regulation).

#### `AutoTuner` ([AutoTuner.h](../lib/AutoTuner.h)) *Part 22, header-only*
- **Purpose:** CMA-ES black-box optimizer for controller parameter tuning (minimises arbitrary cost function).
- **Params:** `AutoTunerParams {n, sigma0, maxIter, tol, lower, upper}`. **Result:** `TunerResult {params, cost, nEvals, nGens, converged}`.
- **Usage:** `AutoTuner tuner(atp, seed); auto result = tuner.tune(cost_fn, x0);`
- **Box constraints:** implemented by clipping samples to [lower, upper].

#### `AntiWindupWrapper` ([AntiWindupWrapper.h](../lib/AntiWindupWrapper.h)) *Part 24, header-only*
- **Purpose:** Generic anti-windup decorator for any IController using the Hanus (1987) conditioning technique. Prevents integrator windup in controllers without built-in saturation handling (DiscreteLQG, DiscreteHinf, GPC, custom).
- **Algorithm:** At each step: `e_in[k] = e[k] + Kb*(u_sat[k-1]-u_raw[k-1])` is passed to the inner controller; integral is bounded to `~ uMax + e/Kb` in steady saturation (vs unbounded without wrapper).
- **Ctor:** `AntiWindupWrapper(shared_ptr<IController> inner, uMin, uMax, Kb=1.0)`.
- **Key methods:** `compute(error)`, `isSaturated()`, `saturationError()`, `setActualOutput(u_applied)`, `lastOutput()`, `bumplessInit(u_target, error)`.
- **Do NOT use on DiscretePID** -- PID already has back-calculation via `PIDParams::Kb`. Double-wrapping applies conditioning twice.
- **Python:** `ctrl.AntiWindupWrapper(inner, uMin, uMax, Kb=1.0)`.

#### `TubeMPC` ([TubeMPC.h](../lib/TubeMPC.h)) *Part 26*
- **Purpose:** Robust MPC with mRPI tube guarantee for bounded additive disturbances `w[k] ∈ W` (Mayne, Seron & Rakovic 2005). The actual state stays inside `Z = {e : |e_i| ≤ z_max_i}` around the nominal trajectory for all admissible disturbances.
- **Parameters (`TubeMPCParams`):** `Np`, `Nu`, `Q` (state cost), `R` (control cost), `K` (stabilising feedback — must make A+B*K stable), `wMax` (disturbance bound per state), `uMin/uMax`, `Ts`.
- **Offline (ctor):** computes mRPI set, tightens input constraints by K*Z, builds condensed QP matrices.
- **Online:** `setState(x)`, `computeRef(x, y_ref)` applies composite law `u = K*(x - x_nom) + V*[0]`.
- **Cave:** `y_ss ≈ Q/(Q+R)*r` without integral action; for ~0.5% error use `Q=10, R=0.05`. MATLAB `lqr()` sign: pass `K = -K_lqr` (negate).

#### `ParticleFilter` ([ParticleFilter.h](../lib/ParticleFilter.h))
- **Purpose:** SIR (Bootstrap) particle filter for nonlinear / non-Gaussian state estimation (Gordon, Salmond & Smith 1993). Use over EKF/UKF when the posterior is multimodal or heavy-tailed.
- **Parameters:** `n_particles`, process-noise `Q`, measurement-noise `R`, `resample_threshold` (ESS fraction, typically 0.5).
- **Usage:** provide `state_fn(x, u)` (dynamics) + `obs_fn(x, u)` (measurement model); call `update(u, y)` → `estimate()` returns weighted mean.
- **Benchmark:** Kitagawa `y = x²/20 + noise` — RMSE 4–10 is normal (bimodal posterior).

---

### 5.4 Tuning Layer

#### `RelayAutoTuner` ([ControllerTuner.h](../lib/ControllerTuner.h))
- **Purpose:** Astrom-Hagglund relay-feedback test -> extracts ultimate gain `Ku` and period `Tu`.
- **Config (`RelayTunerConfig`):** `relayAmplitude`, `hysteresis`, `cyclesRequired`.
- **Usage:** Drive `step(y)` until `isDone()`; then `computePIDParams(rule, lambda)` returns `PIDParams`.
- **Rules:** `ZieglerNichols`, `TyreusLuyben`, `IMC`, `AMIGO`.

#### `StepResponseTuner`
- **Purpose:** Open-loop FOPDT identification (`K, tau, theta`) from step response data; produces PID gains via IMC.
- **Methods:** `identify(t, y, stepMag)` -> `FOPDTModel`; `computePIDParams(model, Ts, rule, lambda)`.

#### `LQRWeightTuner`
- `brysonMethod(xmax, umax)` - Bryson's rule: `Q = diag(1/xmax^2)`, `R = diag(1/umax^2)`.
- `polePlacementHint(plant, desiredPoles, maxIter)` - iterative pole shaping into `LQRParams`.

#### `MPCHorizonTuner`
- `recommend(plant, Ts, rho_y, rho_u)` -> `Recommendation { Np, Nc, rho_y, rho_u, estimatedSettlingTime }`.
- `estimateSettlingTime(plant, maxSteps)` - used to size `Np`.

#### `ZieglerNicholsTuner`, `CohenCoonTuner`, `LoopShapingTuner`, `KalmanWeightTuner`
Standalone heuristics; each exposes `tuneImpl(...)` (unchecked) and `tuneFor<C>(...)` (template wrapper enforcing compile-time `ControllerTraits<C>` compatibility - produces actionable error messages for incompatible types).

#### `TunerSuite` ([TunerSuite.h](../lib/TunerSuite.h))
- **Purpose:** Unified front-end dispatching to the eight tuning families with **runtime soft-warnings** (IDEAL -> no warning; SOFT -> diagnostic + `result.warned == true`; FALLBACK -> default params + `success == false`).
- **Methods:** `relayZN`, `imcPID`, `cohenCoon`, `bryson`, `kalmanNoise`, `mpcHorizon`, `loopShaping`, `optimise` (Nelder-Mead ISE/ITAE black-box).
- **Helpers:** `makeISECost`, `makeITAECost` - factory for cost functions used by `optimise`.

#### `ControllerTraits<C>` ([ControllerTraits.h](../lib/ControllerTraits.h))
- **Purpose:** Compile-time mapping from controller type to supported tuners (booleans `supports_heuristic_pid`, `supports_lqr_tuning`, `supports_mpc_tuning`, `supports_freq_tuning`, `supports_kalman_tuning`).
- **Use:** `tuneFor<C>` static asserts fire with a diagnostic naming the correct alternative tuner.

---

<a name="55-composition--orchestration"></a>
### 5.5 Composition & Orchestration

#### `ControllerStack` ([ControllerStack.h](../lib/ControllerStack.h))
- **Purpose:** Multi-controller orchestrator with three modes.
  - **Supervisory** - first entry whose `activationCondition(error, lastOutput)` returns `true` is used; others idle. Use for fallbacks and bumpless transfer. For continuous gain scheduling prefer `GainScheduledController`.
  - **Additive** - outputs of all enabled entries are summed. Use for inner/outer cascades.
  - **Weighted** - `u = Sigma w_i.u_i(e)`. Use for fuzzy blending.
- **Methods:** `addController(ptr, name, weight, condition)`, `removeController(name)`, `setActive(name, bool)`, `setWeight(name, w)`, `compute(error)`, `activeControllerName()`, `entries()`.

---

<a name="56-analysis--metrics"></a>
### 5.6 Analysis & Metrics

#### `MetricsAnalyzer` ([MetricsAnalyzer.h](../lib/MetricsAnalyzer.h))
- **Purpose:** Extract time-domain metrics from step-response data.
- **Method:** `calculate(t_data, y_data, reference, finalValueWindow) -> TimeDomainMetrics { riseTime, settlingTime, peakOvershoot, steadyStateError }`.

#### `SystemAnalysis` ([SystemAnalysis.h](../lib/SystemAnalysis.h))
- **Purpose:** Frequency-domain and stability analysis utilities (static methods).
- **Methods:** `getPoles(sys)`, `isDiscreteStable(sys)`, `solveDiscreteLyapunov(A, Q)`, `getFrequencyResponse(sys, freqs)`, `getSingularValues(sys, freqs)` (singular values of `G(e^{j.omega.Ts})` per frequency, descending; SISO and MIMO - for SISO equals `|getFrequencyResponse()|`), `calculateMargins(sys) -> StabilityMargins { gainMarginDb, phaseMarginDeg, wCrossoverGain, wCrossoverPhase }`, `calculateHInfinityNorm(sys)` (grid approximation - treat as lower bound).

#### Robustness analysis ([RobustnessAnalysis.h](../lib/RobustnessAnalysis.h), [MuAnalysis.h](../lib/MuAnalysis.h), [WorstCaseSearch.h](../lib/WorstCaseSearch.h), [LyapunovRobustness.h](../lib/LyapunovRobustness.h))
- **`RobustnessAnalysis`:** Monte-Carlo closed-loop robustness - spawns perturbed plants and aggregates stability / margin / sensitivity statistics. (Roadmap Phase 1.)
- **`MuAnalysis`:** Structured singular value (mu) D-scaling upper bound; `peakMu`, `robustStabilityRadius`. (Phase 3.)
- **`WorstCaseSearch`:** CMA-ES worst-case parameter search over plant uncertainty. Free functions `findWorstCaseSensitivity(...)` (maximise `||S||_inf`), `findWorstCaseIAE(...)` (maximise step IAE), and generic `findWorstCase(plant_factory, metric_fn, ...)`. Search runs in normalised coordinates; returns `WorstCaseResult { worst_params, worst_cost, converged, n_evals }`. (Phase 4.)
- **`LyapunovRobustness`:** `findCommonLyapunov(vertices, Q = {}, params = {}) -> LyapunovResult { found, P, residual, iterations }` - searches for a single `P > 0` proving quadratic stability of a polytopic system over arbitrary switching. Heuristic sum-and-project (no SDP solver); `found` is the authoritative pass/fail. (Phase 5.)

---

<a name="57-real-time-utilities--hal"></a>
### 5.7 Real-Time Utilities & HAL

#### `AtomicParamBuffer<Params>` ([AtomicParamBuffer.h](../lib/AtomicParamBuffer.h))
- **Purpose:** Seqlock-based double-buffered parameter handoff between a background tuner and the real-time control thread. Single-writer / single-reader, data-race-free under the C++ memory model.
- **Constraint:** `Params` must be `std::is_trivially_copyable<Params>::value == true` (plain-old-data struct).
- **API:**
  - `Params read()` - RT thread. Returns a copy protected by the seqlock. Spins only if a `publish()` is mid-flight (typically zero retries).
  - `void publish(const Params& p)` - background thread. Writes to inactive slot, then atomically promotes it. Increments seqlock counter twice.
  - `Params latest()` - background thread only; non-seqlock-protected peek at the active slot.

#### HAL ([lib/hal/HAL.h](../lib/hal/HAL.h))
Bundles `ISensor`, `IActuator`, `SimPlant`, `SimSensor`, `SimActuator` for closed-loop simulation against a `StateSpace` plant. Suitable as a stand-in for real hardware drivers when developing/testing.

```cpp
ctrl::SimPlant    plant(sys);
ctrl::SimSensor   sensor(plant);
ctrl::SimActuator actuator(plant, -10.0, +10.0);
for (int k = 0; k < N; ++k) {
    double y = sensor.read();
    double u = pid.compute(r - y);
    actuator.write(u);    // steps plant
}
```

---

### 5.8 Data-Driven & ML Controllers (Parts 31–34)

All algorithms below are in `lib/`, included by [ControllerToolbox.h](../lib/ControllerToolbox.h), and have pybind11 bindings + Catch2 tests.

#### `ILCController` ([IterativeLearningControl.h](../lib/IterativeLearningControl.h)) *Part 31*
- **Purpose:** Iterative Learning Control — learns a feedforward correction that eliminates repeating tracking errors across fixed-duration trials (Bristow 2006).
- **Modes:** `P_type` (`u_{j+1} = u_j + L_p*e_j`), `D_type` (adds derivative term), `NormOptimal` (minimises `||e_{j+1}||²_R + ||Δu_j||²_Q` using the Markov-parameter matrix G).
- **Usage:** `newTrial()` → run the trial → `endTrial(e_vec)` → `getNextInput(k)` for the feedforward at step k.

#### `SINDy` + `SINDyModel` ([SINDy.h](../lib/SINDy.h)) *Part 31*
- **Purpose:** Sparse Identification of Nonlinear Dynamics — builds a sparse polynomial/trig equation of motion from state-derivative data via STLS regression (Brunton 2016).
- **Library options:** `PolyDeg1`, `PolyDeg2`, `PolyDeg3`, `PolyDeg1Trig` (adds sin/cos columns).
- **Usage:** `fit(X_dot, X, U)` → `SINDyModel`; `model.stateFunc()` returns a `StateFunc` compatible with `NonlinearMPC`, `CEMController`, and `DynaController`.

#### `KoopmanEDMD` ([KoopmanEDMD.h](../lib/KoopmanEDMD.h)) *Part 31*
- **Purpose:** Extended Dynamic Mode Decomposition — lifts a nonlinear system to a high-dimensional linear representation via a dictionary of basis functions (Williams 2015).
- **Dictionaries:** `PolyDeg1`, `PolyDeg2`, `RBF` (radial basis), and combinations.
- **Methods:** `fit(X, U, Y)` → `StateSpace` (full lifting); `fitProjected()` → `StateSpace` restricted to the original state coordinates. The output drops directly into `DiscreteMPC` or `DiscreteLQR`.

#### `L1AdaptiveController` ([L1AdaptiveController.h](../lib/L1AdaptiveController.h)) *Part 31*
- **Purpose:** L1 adaptive control — state predictor + low-pass-filtered adaptation law; guarantees bounded transient performance independent of adaptation gain (Hovakimyan & Cao 2010).
- **Usage (Python/C++):** `set_reference(r)` then `compute(y_plant)` — **not** `compute(r - y)`.
- **Key params:** `a_m` (model pole), `b_m` (model gain), `Gamma` (adaptation rate), `omega_c` (LP filter cutoff), `sigma_max` (projection bound). Constraint: `a_m` must give a stable reference model.

#### `CBFSafetyFilter` ([CBFSafetyFilter.h](../lib/CBFSafetyFilter.h)) *Part 31*
- **Purpose:** Control Barrier Function safety filter — 1D analytical QP wrapper that minimally modifies any `IController`'s output to keep the system inside a safe set `{x : h(x) ≥ 0}` (Ames et al. 2017).
- **Usage:** provide `h_fn(x)` and `grad_h_fn(x)` (scalar + gradient); wraps any `shared_ptr<IController>`.
- **1D analytical solve:** avoids a full QP; closed-form projection onto the CBF halfspace.

#### `GaussianProcess` ([GaussianProcess.h](../lib/GaussianProcess.h)) *Part 31*
- **Purpose:** Gaussian Process Regression with squared-exponential kernel and Cholesky inference (Rasmussen & Williams 2006). Fixed-budget online mode evicts oldest point at `N_max`.
- **Methods:** `train(X, y)`, `predict(x*)` → `{mean, variance}`. Kernel params: `sigma_f` (signal std), `ell` (length scale), `sigma_n` (noise std).
- **Note:** Not an `IController`; used as a surrogate for GP-MPC or inside `BayesianOptimizer`.

#### `EchoStateNetwork` ([EchoStateNetwork.h](../lib/EchoStateNetwork.h)) *Part 31*
- **Purpose:** Echo State Network / Reservoir Computing — fixed random reservoir, trained readout only via ridge regression (Jaeger 2001). Identifies nonlinear dynamics without backprop.
- **Methods:** `train(U, Y, washout)` fits `W_out`; `step(u)` → `y_hat`. Returns a `StateFunc` for use in predictive controllers.
- **Key param:** `spectral_radius` — must be `< 1` for echo state property (typically 0.9).

#### `NeuralPID` ([NeuralPID.h](../lib/NeuralPID.h)) *Part 31*
- **Purpose:** Online neural PID — 3-layer network `[e, ė, ∫e] → [Kp, Ki, Kd]` adapts weights each step via backprop through the linearised plant Jacobian.
- **Params (`NeuralPIDParams`):** `n_hidden`, `lr`, `Ts`, `plant_gain`, `max_weight_norm`, `uMin/uMax`, `Kp0/Ki0/Kd0`. **Note:** `Ts` IS a field of `NeuralPIDParams` (unlike `PIDParams` where Ts is a constructor arg).
- **Constructor:** `NeuralPID(params)` — single-arg, no separate Ts.

#### `CEMController` ([CEMController.h](../lib/CEMController.h)) *Part 31*
- **Purpose:** Cross-Entropy Method MPC — derivative-free stochastic rollout optimisation. Samples N action sequences, keeps elite set, refits Gaussian, warm-starts with previous solution.
- **Params:** `N_samples` (typically 50), `elite_frac` (0.1), `n_iter`, `Np`, `sigma_init`.
- **Usage:** provide `state_fn` (dynamics) + `cost_fn` (per-step cost); call `compute(error)`.

#### `DynaController` ([DynaController.h](../lib/DynaController.h)) *Part 33*
- **Purpose:** Model-based RL (Dyna, Sutton 1991) — wraps any base `IController`, accumulates transition data, fits a SINDy error-dynamics model, exposes `modelRollout()` for synthetic planning.
- **Usage:** `compute(error)` delegates to the wrapped policy + learns in the background once `min_data_points` transitions are collected. Call `modelRollout(u_seq, x0)` from Python to improve the policy on synthetic data.

#### `ScenarioMPC` ([ScenarioMPC.h](../lib/ScenarioMPC.h)) *Part 33*
- **Purpose:** Scenario-based stochastic MPC (Calafiore & Campi 2006) — averages the QP cost over N_s sampled Gaussian noise trajectories. More conservative than `DiscreteMPC`, less conservative than `TubeMPC`.
- **Params:** `N_s` (scenario count), `Sigma_w` (process-noise covariance), plus the usual `MPCParams` fields. API mirrors `DiscreteMPC`.

#### `BayesianOptimizer` ([BayesianOptimizer.h](../lib/BayesianOptimizer.h)) *Part 33, header-only*
- **Purpose:** Bayesian Optimization for expensive controller parameter tuning — GP surrogate + UCB or EI acquisition (Srinivas 2010). Use instead of `AutoTuner` (CMA-ES) when each cost evaluation is costly (hardware test, long simulation).
- **Acquisition:** `UCB` (exploration weight `kappa`) or `EI` (expected improvement `xi`).
- **Shared types with AutoTuner:** `TunerResult`, `CostFn` — plug into the same cost wrappers.

#### `ControllerRegistry` + `ControllerMonitor` ([ControllerRegistry.h](../lib/ControllerRegistry.h), [ControllerMonitor.h](../lib/ControllerMonitor.h)) *Part 33*
- **Registry:** Meyers-singleton self-registration. Each algorithm header places `CTRL_REGISTER_FEATURE(name)` at its bottom; `ctrl::features()` then returns the live map. `CTRL_HAS_*` compile-time flags set conditional entries.
- **Monitor:** Attaches to any `IController` as an `IControllerObserver`. Runs CUSUM (mean-shift detection, params `k`/`h`) and EWMA (drift detection, params `lambda`/`L`) SPC charts on the output stream. Fires a configurable `alarm_cb(chart_name, value)` on fault. Also listens to `onState(key, vec)` — ADRC emits `"eso"` z-vector, SMC emits `"surface"`.

#### `ComputationalDelayWrapper` ([ComputationalDelayWrapper.h](../lib/ComputationalDelayWrapper.h)) *Part 34, header-only*
- **Purpose:** One-sample actuator delay decorator — models the realistic digital loop where computation at step k cannot reach the actuator until step k+1.
- **Behaviour:** `u_out[k] = u_inner[k-1]` (first call returns 0, the held initial value). Shifts Nyquist phase margin by −π; use this to expose that margin during tuning, not to fix it.
- **Note:** Output is initialised to 0.0; warm up one step before trusting the output.

---

## 6. Deployment Cross-References

For production deployment, parameter-stability constraints, RTOS integration, and troubleshooting recipes, consult [deployment.md](deployment.md):

- **Section 1** Per-controller parameter constraints (PID `Ki.Ts < 2.Kp`, ADRC `omega_o.Ts < 2`, MPC Hessian conditioning, ...).
- **Section 2** Real-time integration: zero-allocation checklist, stack-size estimates, RTOS scheduling.
- **Section 3** Troubleshooting: DARE non-convergence, MPC LDLT failure, Kalman divergence, ADRC ESO instability, Smith-predictor delay mismatch, NaN propagation.
- **Section 4** Quick-start parameter tables for an unknown SISO plant.

For tuning workflow choices and history, see [cheatsheet/tuning_methods.md](../cheatsheet/tuning_methods.md) and [cheatsheet/controller-tuning-reference.md](../cheatsheet/controller-tuning-reference.md). For system identification, see [cheatsheet/system_identification.md](../cheatsheet/system_identification.md) and the FOPDT / ARMAX / N4SID sub-notes.

For a source-only architectural reconstruction, the v1 -> v2 breaking-changes / deprecation log, and the full per-controller sign-convention mapping, see [forensic_reconstruction.md](forensic_reconstruction.md).

---

*End of documentation.*
