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
          (incl. GainScheduledController, GapMetric, LPVSystemID, NonlinearMPC, AdaptiveSmithPredictor, AutoTuner, AntiWindupWrapper)
   - 5.4 [Tuning Layer](#54-tuning-layer)
   - 5.5 [Composition & Orchestration](#55-composition--orchestration)
   - 5.6 [Analysis & Metrics](#56-analysis--metrics)
   - 5.7 [Real-Time Utilities & HAL](#57-real-time-utilities--hal)
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

The `examples/python/` directory contains companion scripts using `python-control` for cross-validation. Create the environment from [examples/python/environment.yml](examples/python/environment.yml):

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
|-- examples/               # Single-file demos (ex01..ex22) + advanced cpp/ folder
|-- case-study/             # 4 physics studies: boiler-turbine, SMISMO hydraulic, tug boat, solar cooling
|-- tests/                  # CTest-driven unit + integration tests
|-- scripts/                # tune_all / simulate_all / realtime_all batch tools
|-- cheatsheet/             # Markdown reference notes (tuning, identification)
|-- DEPLOYMENT.md           # Real-time / RTOS deployment guide (must-read for prod)
|-- bug_report.md           # Internal code-review log
```

---

## 2. Compilation Guide

### 2.1 Standard Configure-Build

From the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

This produces the static library `build/lib/libcontroller_toolbox.a` (or `.lib` on Windows) and every example/test/script executable. The root [CMakeLists.txt](CMakeLists.txt) aggregates: `lib/`, `tests/`, `examples/`, `scripts/`, `case-study/` (benchmarks are intentionally excluded).

### 2.2 Running Tests

```bash
cd build && ctest --output-on-failure
```

Three test targets are registered in [tests/CMakeLists.txt](tests/CMakeLists.txt): `controller_tests`, `tuner_tests`, `integration_tests`.

### 2.3 Linking Against the Library

The library publishes `lib/` as its include root, so consumers write `#include "ControllerToolbox.h"` (the umbrella header at [lib/ControllerToolbox.h](lib/ControllerToolbox.h)) and link `controller_toolbox`:

```cmake
target_link_libraries(your_target PRIVATE controller_toolbox)
target_compile_features(your_target PRIVATE cxx_std_20)
```

Eigen is propagated as a `PUBLIC` dependency of `controller_toolbox`, so the consumer does not need to link it explicitly.

### 2.4 Real-Time Build Flags

For production / RTOS targets, see [DEPLOYMENT.md Section 2](DEPLOYMENT.md#2-real-time-integration). Suggested flags:

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
| [ControllerToolbox.h](lib/ControllerToolbox.h) | Umbrella include - pulls in every public header |
| [IController.h](lib/IController.h) | Abstract controller interface (`IController`) |
| [IControllerObserver.h](lib/IControllerObserver.h) | Observer/telemetry callback interface |
| [Features.h](lib/Features.h) | `ctrl::features()` - runtime optional-module discovery |
| [PlantModel.h](lib/PlantModel.h) | `TransferFunction`, `StateSpace`, `tf2ss`, `c2d`, `ssStep`, `ssStepCopy` |
| [DiscretePID.h](lib/DiscretePID.h) | PID with derivative filter + anti-windup (DoM, 2-DOF) |
| [DiscreteLQR.h](lib/DiscreteLQR.h) | Infinite-horizon LQR, DARE solver, `LQRAdapter` |
| [DiscreteMPC.h](lib/DiscreteMPC.h) | Condensed receding-horizon QP, `setPlant()` for adaptive MPC |
| [DiscreteLQG.h](lib/DiscreteLQG.h) | LQR + Kalman output-feedback combo |
| [DiscreteSMC.h](lib/DiscreteSMC.h) | First-order sliding mode with boundary layer |
| [DiscreteADRC.h](lib/DiscreteADRC.h) | 2nd-order LADRC (ESO + PD) |
| [DiscreteLeadLag.h](lib/DiscreteLeadLag.h) | Tustin-discretised lead-lag biquad |
| [SmithPredictor.h](lib/SmithPredictor.h) | Dead-time compensator wrapper (integer + fractional Pade delay) |
| [ExtremumSeeker.h](lib/ExtremumSeeker.h) | Perturbation-based ESC |
| [RepetitiveController.h](lib/RepetitiveController.h) | Internal-model repetitive control (IMP with Q-filter) |
| [FeedforwardController.h](lib/FeedforwardController.h) | Static / dynamic feedforward (reference model + gain) |
| [GeneralizedPredictiveControl.h](lib/GeneralizedPredictiveControl.h) | GPC (CARIMA predictor, RLS online adaptation, `setPlant()`) |
| [DiscreteHinf.h](lib/DiscreteHinf.h) | H-infinity (gamma iteration, mixed sensitivity) + mu-synthesis DK-iteration |
| [FuzzyLogic.h](lib/FuzzyLogic.h) | Mamdani/TS inference engine, `FuzzyPD`, `FuzzyPID`, `FuzzySupervisor` |
| [KalmanFilter.h](lib/KalmanFilter.h) | Standalone linear Kalman filter (predict / update / step) |
| [ExtendedKalmanFilter.h](lib/ExtendedKalmanFilter.h) | EKF with user-supplied Jacobians or numerical differentiation |
| [UnscentedKalmanFilter.h](lib/UnscentedKalmanFilter.h) | UKF with scaled sigma-point parametrisation (alpha, beta, kappa) |
| [MovingHorizonEstimator.h](lib/MovingHorizonEstimator.h) | MHE via condensed QP; box constraints on process noise |
| [FOPDTIdentifier.h](lib/FOPDTIdentifier.h) | FOPDT step-response identification (graphical + golden-section) |
| [SOPDTIdentifier.h](lib/SOPDTIdentifier.h) | SOPDT step-response identification + Rivera 1986 IMC-PID tuning |
| [RecursiveLeastSquares.h](lib/RecursiveLeastSquares.h) | Online ARX identification, `toTransferFunction`, `toStateSpace` |
| [SubspaceID.h](lib/SubspaceID.h) | N4SID subspace identification, `suggestOrder` |
| [FunctionApproximator.h](lib/FunctionApproximator.h) | Pade delay filter, polynomial approximators |
| [GradientProjectionQP.h](lib/GradientProjectionQP.h) | Projected gradient QP solver (shared by MPC, GPC, MHE) |
| [ControllerStack.h](lib/ControllerStack.h) | Supervisory / Additive / Weighted composition |
| [ControllerTuner.h](lib/ControllerTuner.h) | Per-family tuners (Relay, FOPDT, Bryson, MPC, ...) |
| [TunerSuite.h](lib/TunerSuite.h) | Unified runtime-dispatched tuner with soft warnings |
| [ControllerTraits.h](lib/ControllerTraits.h) | Compile-time traits for tuner <-> controller compatibility |
| [MetricsAnalyzer.h](lib/MetricsAnalyzer.h) | Time-domain step-response metrics |
| [SystemAnalysis.h](lib/SystemAnalysis.h) | Poles, margins, H-infinity norm, Lyapunov |
| [AtomicParamBuffer.h](lib/AtomicParamBuffer.h) | Lock-free param double-buffer for RT updates |
| [hal/HAL.h](lib/hal/HAL.h) | `ISensor`, `IActuator`, `SimPlant`, `SimSensor`, `SimActuator` |

### 3.2 Examples (`examples/`)

54 single-file C++ programs (`ex01_*` through `ex54_*`) plus `example_pid_feedback`. Each demonstrates one controller, composition pattern, identification method, or corrector architecture. See [examples/CMakeLists.txt](examples/CMakeLists.txt) for the full enumeration.

**Example groups:**

| Range | Theme |
|-------|-------|
| ex01-ex22 | Core controllers: PID, LQR, MPC, SMC, ADRC, ESC, Lead-Lag, Smith Predictor, LQG, stacks |
| ex23-ex26 | Fuzzy Logic: FuzzyPD, FuzzyPID, FuzzySupervisor+MPC, TS gain scheduling |
| ex27-ex31 | Advanced: function approximator, GPC, repetitive control, EKF, subspace ID |
| ex32-ex41 | Part 18 algorithms: SOPDT ID, MHE, rational mu-synthesis, cascade/feedforward/smith/ESC/UKF/LPV |
| ex42-ex54 | Corrector patterns: Cascade (PID+MPC, SMC+LQR, Fuzzy+PID, ADRC+PID, LeadLag+RC), Additive (ESC+PID, Fuzzy+SMC, LeadLag+I), Observer+SF (EKF+MPC, UKF+SMC, DOB+PI, MHE+MPC), Supervisory (bumpless transfer) |
| ex55-ex56 | E2/E4 extensions: LinearisationHelper (Van der Pol + CSTR), FeedbackLinearisation (cubic drift + pendulum) |
| ex57-ex59 | E3/E1/E5 extensions: MRAC with gain jump, BalancedTruncation (4th-order plant), ZPETC (min-phase + NMP) |

**Python examples** (`examples/python/`, `ex01_*` through `ex75_*`): NumPy/python-control cross-validation and pybind11 binding demonstrations for every C++ class. Python examples 61-70 mirror corrector patterns; 71-75 mirror the new algorithm extensions.

### 3.3 Case Studies (`case-study/`)

Four self-contained physics studies exercise the library end-to-end. Each pairs a nonlinear plant simulator with a roster of controllers that **wrap** the `lib/` algorithms behind a study-specific `ControllerBase` (not `ctrl::IController` directly, because each plant has a different I/O signature), then sweeps every controller across several scenarios and writes CSV telemetry for post-processing. Counts below are the Part 26 state.

| Study | Plant | Controllers | Scenarios | Runs | Build target |
|---|---|---|---|---|---|
| [`Boiler Control/`](case-study/Boiler%20Control/) | Bell-Astrom 3x3 MIMO boiler-turbine, 3 operating points | 27 | 8 | 216 | `boiler_sim` |
| [`Meter In Meter Out Control/`](case-study/Meter%20In%20Meter%20Out%20Control/) | SMISMO 9-state hydraulic actuator (separate meter-in/meter-out spools) | 14 | 3 | 42 | `smismo_sim` |
| [`Tug Boat Numerical Simulation/`](case-study/Tug%20Boat%20Numerical%20Simulation/) | 3-DOF marine vessel (Li et al. 2026, Ocean Engineering 357), 6-state MIMO + thrust allocation | 16 | 4 | 64 | `tug_sim` |
| [`Solar-Driven Cooling .../`](case-study/Solar-Driven%20Cooling%20System%20with%20Photovoltaic%20Evaporative%20Chimney/) | Algebraic SISO solar cooling + PV evaporative chimney (Ruiz et al. 2024) | 9 | 5 | 45 | `solar_cooling_sim` |

**Boiler-Turbine** controllers span PID, LQR, LQG, MPC, SMC, ESC, ADRC, Lead-Lag+PID, Smith Predictor, GPC-RLS, EKF-LQR, UKF-LQR, FuzzyPID, FuzzySup-MPC, three `ControllerStack` compositions, Repetitive, MRAC, H-infinity, Adaptive Smith Predictor, plus Part 26 additions: NonlinearMPC, Feedback Linearisation, MHE-LQR, LPV gain-scheduled, SubspaceID-LQG, and automated (gap-metric) gain-scheduled LQR.

**Tug Boat** controllers: PID, KF-PID, SMC, MPC, ESC, FuzzyPID, FuzzySup-MPC, ADRC, Repetitive, plus Part 26 additions: 6-state MIMO LQR, LQG, per-axis TubeMPC, EKF-LQR, MRAC, automated gain-scheduled LQR, and NonlinearMPC. Uses `FuzzyLogic`, `KalmanFilter`, `DiscreteMPC`, `DiscreteSMC`, `ExtremumSeeker`, `DiscreteLQR/LQG`, `TubeMPC`, `ExtendedKalmanFilter`, `NonlinearMPC`, and `PlantModel`.

**SMISMO** and **Solar** rosters are documented in their respective `sim/include/*controllers.h` headers. The example wrapper [ex21_boiler_turbine_case_study.cpp](examples/ex21_boiler_turbine_case_study.cpp) is the standalone boiler demo; the full multi-controller sweeps live under the `case-study/` subdirectories and build as the targets in the table above (all listed in `compile.bat`).

> **Note:** only the Tug study currently has a Catch2 regression test (`tests/test_tugsim_regression.cpp`). Adding equivalent IAE/settling-threshold tests for Boiler, SMISMO, and Solar is the top open task (see the Part 26 review in `docs/cumulative_bug_report.md`, finding T1).

### 3.4 Tests (`tests/`)

- `test_controllers.cpp` - per-class unit tests (custom `test_framework.h` harness)
- `test_tuners_extended.cpp` - tuner suite tests (covers all 8 strategies)
- `test_integration.cpp` - end-to-end closed-loop tests (c2d+MPC, N4SID+GPC adaptive pipeline)
- `test_catch2_advanced.cpp` - Catch2 v3 regression suite (**51 test cases, 189 assertions**): GPC tracking, LQR convergence, SMC sign convention, ADRC double-integrator, n4sid identification, EKF/UKF, repetitive control, H-infinity, SOPDTIdentifier [sopdt], MovingHorizonEstimator [mhe], **LinearisationHelper** [linearisation], **FeedbackLinearisationController** [fl], **MRACController** [mrac], **BalancedTruncation** [btm], **ZeroPhaseTrackingFilter** [zpetc]
- `test_catch2_pilot.cpp` - Catch2 v3 pilot tests (5 test cases, 21 assertions): LQRAdapter MIMO `computeVec()`, EKF scaled-epsilon Jacobian, PID DoM derivative suppression, 2DOF b_weight overshoot reduction, observer telemetry wiring
- `test_framework.h` - lightweight assertion macros for the custom harness

**Current totals (2026-05-28):** 78 C++ executables pass | 79 Python examples pass | 0 failures.

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
### 5.1 Core Types ([PlantModel.h](lib/PlantModel.h))

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

#### `std::unordered_map<std::string, bool> ctrl::features()` ([Features.h](lib/Features.h))
- **Purpose:** Runtime discovery of which optional modules were compiled in.
- **Returns:** Map with keys `"hinf"`, `"subspace"`, `"fuzzy"`, `"function_approx"`, `"advanced_kalman"` each mapping to `true` if the corresponding `CTRL_HAS_*` flag was defined at compile time.
- **Typical use:** Python bindings and plugin-discovery code that cannot inspect compile-time `#if` guards.

#### `IController` ([IController.h](lib/IController.h))
- **Purpose:** Abstract base for all SISO controllers; uniform interface for stacking and tuning.
- **Pure-virtual:** `compute(double signal) -> double`, `reset()`, `sampleTime() const -> double`.
- **Convention:** `signal` is the **error** `e = r - y` for tracking controllers, the **plant output** for optimisation controllers (ESC) and observer-based controllers (ADRC, LQG).
- **Observer (telemetry):** `attachObserver(IControllerObserver*)` stores a non-owning raw pointer -- caller must ensure the observer outlives the controller. `attachObserver(std::shared_ptr<IControllerObserver>)` co-owns the observer, safe for Python bindings and dynamically-allocated observers. `detachObserver()` releases both.

---

### 5.2 Controllers

#### `DiscretePID` ([DiscretePID.h](lib/DiscretePID.h))
- **Purpose:** Backward-Euler PID with filtered derivative and back-calculation anti-windup.
- **Parameters (`PIDParams`):** `Kp, Ki, Kd, N` (derivative filter coefficient), `uMin/uMax` (saturation), `Kb` (anti-windup gain).
- **Returns:** Scalar control `u[k]` (saturated).
- **Methods:** `compute(error)`, `reset()`, `setParams(p)`, `params()`, `lastOutput()`.
- **Integral law (backward Euler):** `I[k] = I[k-1] + Ki.Ts.e[k] + Kb.(u_sat[k] - u_unsat[k])`. The current-step error `e[k]` is included in both `I[k]` and `u[k]` before the state is advanced, matching true backward Euler discretisation.
- **Constraints:** `Ki*Ts < 2*Kp` for discrete stability (see [DEPLOYMENT.md Section 1](DEPLOYMENT.md)).

#### `DiscreteLQR` ([DiscreteLQR.h](lib/DiscreteLQR.h))
- **Purpose:** Optimal full-state feedback `u = -K*(x - x_ref) + u_ff` with DARE solved offline via value iteration.
- **Parameters (`LQRParams`):** `Q` (n*n, PSD state cost), `R` (m*m, PD control cost).
- **Inputs:** `compute(x, x_ref = \emptyset, u_ff = \emptyset)` - full state vector required.
- **Returns:** `Eigen::VectorXd u[k]` (size m).
- **Methods:** `gainMatrix()`, `riccatiSolution()`, `dareConverged()`, `dareIterations()`, `sampleTime()`.
- **Helper:** `LQRAdapter` - wraps LQR as `IController` for use inside `ControllerStack`.

#### `DiscreteMPC` ([DiscreteMPC.h](lib/DiscreteMPC.h))
- **Purpose:** Condensed receding-horizon QP with hard box constraints on `Deltau` and `u`.
- **Parameters (`MPCParams`):** `Np` (prediction horizon), `Nc` (control horizon, `Nc <= Np`), `rho_y` (output weight), `rho_u` (move-suppression weight), `uMin/uMax`, `duMin/duMax`.
- **Inputs:** `computeRef(x_current, r_ref)` - current state and stacked reference.
- **Returns:** Optimal Deltau vector (first move of the receding horizon).
- **Prediction formula:** `Y = F.x + G_u.u_prev + Phi.DeltaU`, where `F(i) = C.A^(i+1)`, `G_u(i) = Sigma_{j=0}^{i} C.A^j.B` (cumulative step response, accounts for u_prev baseline), and `Phi(i,j) = C.A^(i-j).B`. All three matrices are pre-built in the constructor.
- **Methods:** `computeRef(...)`, `setPlant(plant)` (online re-linearisation), `setState(x)`, `setParams(p)`, `compute(error)` (SISO convenience).
- **Performance:** Condensed matrices F, G_u, Phi, H are pre-built in the constructor; per-step cost is one LDLT solve.

#### `DiscreteLQG` ([DiscreteLQG.h](lib/DiscreteLQG.h))
- **Purpose:** LQR on Kalman-estimated state - output-feedback optimal control (separation principle).
- **Constructor:** `(plant, LQRParams, Q_noise, R_noise, P0 = I)`.
- **Inputs:** `step(y, u_prev, x_ref) -> u[k]` for MIMO; SISO scalar overload `compute(double y_scalar)` needs prior `setReference()` + `setUPrev()`.
- **Returns:** Control vector `u[k]`. Internal state estimate via `stateEstimate()`.
- **Methods:** `step(...)`, `compute(...)`, `setReference(x_ref)`, `setUPrev(u)`, `reset()`, `gainMatrix()`.
- **D != 0 note:** If the plant's D matrix is non-zero, a `std::cerr` warning is printed at construction. The Kalman innovation `y - C.x^ - D.u` must use `u[k-1]` (one step stale) because `u[k]` has not yet been computed at update time. Accuracy degrades proportionally to D's magnitude. Set D = 0 in the model for accurate filtering.

#### `DiscreteSMC` ([DiscreteSMC.h](lib/DiscreteSMC.h))
- **Purpose:** First-order SMC with sliding surface `s = c_e.e + c_de.(e - e_prev)` and saturation `sat(s/phi)` to reduce chattering.
- **Parameters (`SMCParams`):** `c_e`, `c_de`, `K` (switching gain), `phi` (boundary layer thickness), `uMin/uMax`.
- **Inputs:** `compute(error)`.
- **Returns:** Saturated control `u[k]`.
- **Methods:** `compute(e)`, `slidingSurface()`, `setParams(p)`, `reset()`.
- **`c_de` sizing:** `c_de` absorbs the sample time `Ts`. To match a continuous-time slope `lambda` [1/s], set `c_de = lambda . Ts`. Passing a continuous-time `lambda` directly as `c_de` over-weights the rate term by a factor of `1/Ts`.

#### `DiscreteADRC` ([DiscreteADRC.h](lib/DiscreteADRC.h))
- **Purpose:** Bandwidth-parameterised 2nd-order Linear ADRC - ESO estimates total disturbance, PD law cancels it.
- **Parameters (`ADRCParams`):** `omega_o` (observer BW), `omega_c` (controller BW), `b0` (approximate input gain), `uMin/uMax`.
- **Inputs:** `compute(y)` (plant output, **not error**); set reference first via `setReference(r)` or use `computeTracking(y, r)`.
- **Returns:** Scalar control. Internal ESO state available via `esoState() -> Vector3d {z1, z2, z3}`.
- **Stability:** Requires `omega_o * Ts < 2` (forward-Euler limit).

#### `DiscreteLeadLag` ([DiscreteLeadLag.h](lib/DiscreteLeadLag.h))
- **Purpose:** Tustin-discretised first-order compensator `C(s) = K.(s + z_c)/(s + p_c)`. Lead if `p > z`; lag if `p < z`.
- **Parameters (`LeadLagParams`):** `continuousZero z_c`, `continuousPole p_c`, `gain K`.
- **Inputs:** `compute(u)` - typically the error or plant output to filter.
- **Returns:** Filtered output `y[k]` via `y = b0.u[k] + b1.u[k-1] - a1.y[k-1]`.
- **Methods:** `compute(u)`, `setParams(p)`, `phaseAt(omega_rad_s)`.

#### `SmithPredictor` ([SmithPredictor.h](lib/SmithPredictor.h))
- **Purpose:** Dead-time compensator wrapping any `IController`; replaces feedback delay with internal-model prediction.
- **Constructor:** `(shared_ptr<IController> inner, StateSpace delayModel, int delaySteps)` - `delayModel` is the delay-free plant.
- **Inputs:** `compute(error)` - closed-loop error `r - y`.
- **Returns:** Inner controller's output, with the modified error including the Smith correction term.
- **Methods:** `innerController()` for runtime re-tuning. Delay buffer is a fixed-size circular buffer (no RT allocation).
- **D feedthrough:** The internal model output `yhat[k] = C.x^ + D.u_prev` uses the previous control input `u[k-1]` for the feedthrough term. `u_prev_` is initialised to zero and updated each step.

#### `ExtremumSeeker` ([ExtremumSeeker.h](lib/ExtremumSeeker.h))
- **Purpose:** Perturbation-based optimiser - injects dither, demodulates output, integrates gradient to climb to the extremum of an unknown static cost surface.
- **Parameters (`ExtremumSeekerParams`):** `perturbAmp`, `perturbFreq`, `lpfCutoff`, `hpfCutoff`, `integGain`, `seekMinimum` (true -> min, false -> max).
- **Inputs:** `compute(signal)` - `signal` is the **plant output / cost**, **not** an error.
- **Returns:** Plant input `u = theta + dither` (absolute, not deviation).
- **Methods:** `currentEstimate()` -> integrator state theta.
- **Convergence:** ESC does not declare convergence; user must implement a stagnation window.

#### `RepetitiveController` ([RepetitiveController.h](lib/RepetitiveController.h))
- **Purpose:** Internal Model Principle (IMP) controller for periodic reference/disturbance rejection. Stores one period of correction signal in a circular buffer and blends it with current error via a Q-filter.
- **Parameters (`RepetitiveParams`):** `periodSteps` (period length in samples), `gain`, `qCutoff` (Q-filter cutoff, stability robustness knob).
- **Inputs:** `compute(error)` - standard error `e = r - y`.
- **Returns:** Control correction `u_rc[k]` to add to a primary controller's output.
- **Methods:** `compute(e)`, `reset()`, `bufferSize()`.

#### `FeedforwardController` ([FeedforwardController.h](lib/FeedforwardController.h))
- **Purpose:** Reference model feedforward - computes `u_ff = Kff * r` (static) or filters the reference through a model-inverse transfer function. Use additively with a feedback controller: `u = u_feedback + u_ff`.
- **Parameters (`FeedforwardParams`):** `gain` (static gain), optional `referenceModel` (`StateSpace` for dynamic FF).
- **Inputs:** `compute(reference)` - the setpoint signal.
- **Returns:** Feedforward control signal.

#### `GeneralizedPredictiveController` ([GeneralizedPredictiveControl.h](lib/GeneralizedPredictiveControl.h))
- **Purpose:** GPC based on the CARIMA (Controlled AutoRegressive Integrated Moving Average) process model. Supports online model adaptation via RLS using `setPlant()`.
- **Parameters (`GPCParams`):** `Np` (prediction horizon), `Nu` (control horizon), `rho_y` (output weight), `rho_u` (move-suppression weight), `uMin/uMax`.
- **Inputs:** `computeRef(y_current, r_ref)` -- current plant output and reference.
- **Returns:** Optimal delta-u (first move of the receding horizon).
- **Methods:** `computeRef(y, r)`, `setPlant(plant)` (hot-swap for adaptive MPC), `augmentedState()`, `reset()`.
- **Prediction model:** CARIMA `A(q^-1) * Delta * y = B(q^-1) * u + e`; augmented state includes integrator state; Ga (CARIMA step-response matrix) differs from standard MPC Phi by a `C*B` correction term.
- **Difference from MPC:** GPC operates on `Delta u` (increments) without requiring an initial state `x`; adaptive to unknown plants via RLS feedback.

#### `DiscreteHinf` ([DiscreteHinf.h](lib/DiscreteHinf.h))
- **Purpose:** H-infinity synthesis via gamma-bisection. Also supports mixed-sensitivity design (`mixedSensitivity()`) and mu-synthesis via full DK-iteration (`solveMuSyn()`).
- **Parameters (`HinfParams`):** `gammaInit` (initial gamma upper bound), `gammaTol` (bisection tolerance), `maxIter`.
- **`solve(plant, W1, W2, W3)` -> `HinfResult`:** Solves the standard weighted H-infinity problem. `HinfResult` fields: `controller` (`StateSpace`), `achievedGamma` (actual H-infinity norm), `converged`.
- **`mixedSensitivity(plant, W1, W2, W3)` -> `HinfResult`:** Convenience wrapper for `[W1*S; W2*KS; W3*T]` mixed-sensitivity design.
- **`solveMuSyn(plant, params)` -> `MuSynResult`:** Full DK-iteration mu-synthesis. `MuSynParams` fields: `maxDKIter`, `useRationalD` (if true, fits first-order rational D_j(z) per frequency channel). `MuSynResult` includes `controller`, `dFilters_L`, `dFilters_R` (rational D filters per channel), `muHistory` (mu upper bound per iteration).

#### Fuzzy Logic Module ([FuzzyLogic.h](lib/FuzzyLogic.h))

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

#### `KalmanFilter` ([KalmanFilter.h](lib/KalmanFilter.h))
- **Purpose:** Linear discrete Kalman filter (predict / update) with Joseph-form covariance update.
- **Constructor:** `(plant, Q_noise, R_noise, P0 = I)`.
- **Methods:** `predict(u)`, `update(y, u_current)`, `state()`, `covariance()`, `reset()`.
- **`step()` overloads:**
  - `step(y, u_prev)` -- combined predict+update; `u_current` defaults to `u_prev` (correct for `D = 0` plants).
  - `step(y, u_prev, u_current)` -- plain-reference overload; explicit current input for `D != 0` plants. Preferred from Python bindings (the default overload uses `std::optional<std::reference_wrapper<...>>` which pybind11 cannot auto-convert).
- **Floor:** `R_noise` has an automatic floor of `1e-12` per diagonal element to avoid division by zero.

#### `ExtendedKalmanFilter` ([ExtendedKalmanFilter.h](lib/ExtendedKalmanFilter.h))
- **Purpose:** EKF for nonlinear state estimation. Linearises the dynamics and measurement functions around the current estimate at each step.
- **Constructor:** `(n, p, f, h, Fjac, Hjac, Q, R, Ts)` -- `n` states, `p` outputs, nonlinear functions `f(x,u)` and `h(x,u)`, Jacobian functions `Fjac(x,u)` and `Hjac(x,u)` (or pass `nullptr` for numerical differentiation with scaled epsilon).
- **Methods:** `predict(u)`, `update(y, u)`, `step(y, u_prev)`, `state()`, `covariance()`, `setState(x)`, `reset()`.
- **Numerical Jacobian:** When `Fjac = nullptr`, finite differences use `eps = max(1e-5 * |x_i|, 1e-8)` per element to handle heterogeneous state magnitudes (see [test_catch2_pilot.cpp] P12-17 fix).

#### `UnscentedKalmanFilter` ([UnscentedKalmanFilter.h](lib/UnscentedKalmanFilter.h))
- **Purpose:** UKF using the unscented transform (2n+1 sigma points) for higher-order nonlinear estimation accuracy without Jacobians.
- **Constructor:** `(n, p, f, h, Q, R, Ts, MatrixXd(), alpha, beta, kappa)`.
- **Methods:** `predict(u)`, `update(y, u)`, `state()`, `covariance()`, `setState(x)`, `reset()`.
- **Alpha/kappa guidance:** For n=2, kappa=0: use `alpha >= 1/sqrt(n) = 0.707` to avoid negative `Wc0 = 1 - alpha^2 + beta`. With `alpha=1.0, beta=2.0, kappa=0`: `lambda=0`, `Wm0=0`, `Wc0=2 > 0`. This is the recommended configuration for 2-state systems.

#### `MovingHorizonEstimator` ([MovingHorizonEstimator.h](lib/MovingHorizonEstimator.h))
- **Purpose:** MHE via condensed QP -- the dual of MPC for state estimation. Optimises the state trajectory over a moving window of N measurements with box constraints on process noise.
- **Constructor:** `(plant, Q_noise, R_noise, params)`.
- **Parameters (`MHEParams`):** `N` (horizon length), `wMin/wMax` (process noise bounds), `qpMaxIter`, `qpTol`.
- **Methods:** `initialize(x0, P0)`, `estimate(y, u)` -> `VectorXd x_hat`, `setHorizon(N)`, `setWeightMatrices(Q, R)`, `reset()`, `state()`, `lastConverged()`, `lastQpIters()`, `sampleTime()`.
- **Decision variable:** `z = [x_0; w_0; ...; w_{N-1}]` of size `n*(N+1)`. Condensed matrices `Psi_`, `Gamma_u_`, `C_bar_` are pre-built; per-step cost is one `GradientProjectionQP` solve.
- **Horizon ramp-up:** First N-1 calls use an effective horizon shorter than N; arrival cost `P0inv` weights the initial state uncertainty.
- **Backend:** Shared `GradientProjectionQP` solver (same as MPC and GPC); zero heap allocation after construction.

#### `FOPDTIdentifier` ([FOPDTIdentifier.h](lib/FOPDTIdentifier.h))
- **Purpose:** Identifies a First-Order Plus Dead-Time (FOPDT) model from open-loop step response data.
- **Methods:** `identify(t, y, stepMag, method)` -> `FOPDTModel {K, tau, theta, fitRMSE}`; `evaluate(model, t, stepMag)` -> simulated response.
- **Methods:** Graphical (ZN tangent + 63.2% crossing) or optimization (golden-section on theta and tau).
- **Integration:** Result feeds directly into `StepResponseTuner::computePIDParams()` for IMC, ZN, Cohen-Coon, AMIGO tuning.

#### `SOPDTIdentifier` ([SOPDTIdentifier.h](lib/SOPDTIdentifier.h))
- **Purpose:** Identifies a Second-Order Plus Dead-Time (SOPDT) model from open-loop step response data. Convention: `tau1 >= tau2`.
- **Constructor:** `(t_data, y_data, stepMag)`.
- **Methods:** `identify(method)` -> `SOPDTModel {K, tau1, tau2, theta, fitRMSE}`; `evaluate(model, t, stepMag)` -> simulated response; `static imcTuning(model, lambdaC)` -> `PIDParams`.
- **`SOPDTMethod`:** `Graphical` or `Optimization`.
- **Graphical algorithm:** ZN tangent -> `theta`; 63.2% crossing -> `tau_sum = tau1+tau2`; 28.3% crossing ratio `r` interpolated between FOPDT limit (r~0.332) and critically-damped SOPDT limit (r~0.530) to split into `tau1` and `tau2`.
- **Optimization:** Nested golden-section search on `(theta, tau1, tau2)` minimising RMSE.
- **IMC-PID (Rivera 1986):** `tau_eq = tau1+tau2`, `Kp = tau_eq / (K*(lambdaC + theta/2))`, `Ti = tau_eq`, `Td = tau1*tau2/tau_eq`.

#### `MRACController` ([MRACController.h](lib/MRACController.h))
- **Purpose:** Discrete-time Model Reference Adaptive Control -- SISO, first-order reference model. Forces plant output to track y_m[k+1] = a_m*y_m + b_m*r via two-parameter Lyapunov adaptation with σ-modification and Euclidean projection.
- **Constructor:** `(MRACParams, Ts)`.
- **Convention:** `compute(y_plant)` takes plant output (ADRC convention, not error). Call `setReference(r)` before each `compute()`.
- **Adaptation law:** `θ[k+1] = θ[k] - Ts*(γ*e_m*φ + σ*θ[k])` with projection: if ‖θ‖ > theta_max -> θ <- θ*theta_max/‖θ‖.
- **Parameters (`MRACParams`):** `a_m`, `b_m` (reference model), `gamma_r`, `gamma_y` (rates), `sigma` (σ-modification, 0=off), `theta_max` (projection bound), `uMin/uMax`.
- **Methods:** `compute(y)`, `setReference(r)`, `reset()`, `theta_r()`, `theta_y()`, `modelOutput()`, `modelError()`.
- **Feasibility:** Minimum-phase plant required; gain sign must match γ_r, γ_y sign; persistent excitation needed for θ convergence.

#### `FeedbackLinearisationController` ([FeedbackLinearisation.h](lib/FeedbackLinearisation.h))
- **Purpose:** Exact feedback linearisation for SISO affine-in-control systems ẋ = f(x) + g(x)*u, relative degree 1. Cancels nonlinear terms algebraically; inner IController drives the resulting virtual integrator.
- **Constructor:** `(DriftFn f, GainFn g, shared_ptr<IController> inner, FLParams, Ts)`.
- **Control law:** `u[k] = clamp((v - f(x[k], u[k-1])) / g_eff(x[k], u[k-1]), uMin, uMax)` where v = inner->compute(error).
- **Critical requirement:** `setState(x)` must be called before each `compute()` with the current measured or estimated plant state.
- **Parameters (`FLParams`):** `uMin`, `uMax`, `regularisationEps` (minimum |g| before clamping; preserves sign).
- **Feasibility:** Relative degree 1, minimum-phase zeros, g(x) ≠ 0 across operating region.
- **Relative degree 2 note:** For angle-output systems (pendulum), the inner PID must be tuned for a virtual double integrator; example gains: Kp=9, Ki=5, Kd=5 for poles {-1, -2±j}. See `ex56_feedback_linearisation.cpp`.

#### `RecursiveLeastSquares` ([RecursiveLeastSquares.h](lib/RecursiveLeastSquares.h))
- **Purpose:** Online ARX parameter estimation using exponential forgetting (`lambda` factor).
- **Constructor:** `(na, nb, nk, lambda, P0_scale)` -- output order, input order, delay, forgetting factor, initial covariance scale.
- **Methods:** `update(y, u)` -> current ARX coefficients; `reset()`, `toTransferFunction()`, `toStateSpace()`.
- **Typical use:** Feed identified `StateSpace` into `GPC::setPlant()` for adaptive GPC (see `ex28_gpc_adaptive.cpp`).

#### `LinearisationHelper` ([LinearisationHelper.h](lib/LinearisationHelper.h))
- **Purpose:** Numerical Jacobians and ZOH linearisation of continuous-time nonlinear models at a given operating point. Central-difference with step `h_i = ε*max(|x_i|, 1)` for heterogeneous state magnitudes.
- **Key functions:**
  - `jacobianX(f, x0, u0, eps=1e-4)` -> MatrixXd ∂f/∂x (n evaluations of f).
  - `jacobianU(f, x0, u0, eps=1e-4)` -> MatrixXd ∂f/∂u (m evaluations).
  - `lineariseAtPoint(f, x0, u0, Ts)` -> discrete StateSpace (ZOH, C=I, D=0).
  - `lineariseAtPoint(f, h, x0, u0, Ts)` -> discrete StateSpace with custom output h(x,u).
- **Note:** The `StateFunc`/`MeasFunc` type aliases are identical to those in `ExtendedKalmanFilter.h` -- safe to include both (C++ redeclaration of identical alias is valid).
- **Typical use:** Compute A, B at operating point -> `c2d(ZOH)` -> `DiscreteLQR` design -> apply gain to nonlinear plant.

#### `BalancedTruncation` ([BalancedTruncation.h](lib/BalancedTruncation.h))
- **Purpose:** Moore (1981) model order reduction via balanced realisation. Replaces an n-state stable system with an r-state approximation preserving the most energetically significant modes. Provides a-priori H∞ error bound.
- **Functions:**
  - `balancedTruncate(sys, r)` -> `TruncationResult {reduced, hankelSingularValues, errorBound, isStable}`.
  - `suggestOrder(result, tol=0.01)` -> int -- smallest r such that error bound < tol x total_norm.
- **Algorithm:** Solve gramians via `SystemAnalysis::solveDiscreteLyapunov` -> Cholesky of P_c -> `SelfAdjointEigenSolver` of M = L_c'*P_o*L_c -> sort HSVs descending -> balanced transformation T_r = L_c*U*Σ^{-½} -> truncate.
- **Error bound:** ‖G - G_r‖∞ ≤ 2*Σᵢ₌ᵣ₊₁ⁿ σᵢ. Verified: actual DC gain deviation is always within this bound.
- **Constraint:** O(n⁶) Lyapunov solver -- use for n ≤ 10. Larger systems require Bartels-Stewart (not yet implemented).
- **Correct usage:** Design controller on `result.reduced`, then apply to the full-order plant. The error bound quantifies the performance degradation. See `ex58_balanced_truncation.cpp`.

#### `ZeroPhaseTrackingFilter` ([ZeroPhaseTrackingFilter.h](lib/ZeroPhaseTrackingFilter.h))
- **Purpose:** ZPETC (Tomizuka 1987) feedforward prefilter. Inverts the minimum-phase part of a plant's numerator causally, normalises DC gain to 1, and produces zero phase error for min-phase zeros.
- **Functions:**
  - `transmissionZeros(sys)` -> `vector<complex<double>>` -- finite eigenvalues of the system matrix pencil `[[A-λI, B],[C, D]]` via `GeneralizedEigenSolver`.
  - `designZPETC(plant)` -> `ZPETCResult {filter, dcAmplitudeError, hasNMPZeros, zeros, nmpZeros}`.
- **Composite response:** G(z)*G_ff(z) = B⁻(z)/B⁻(1). For min-phase plants: G*G_ff = z^{-d} (unit magnitude, pure delay). For NMP: unit DC gain, amplitude error away from DC.
- **Key implementation detail:** The evaluation function `evalTF` must store C as `MatrixXcd` not `VectorXcd` -- Eigen's implicit reshape transposed a (1xn) row into an (nx1) column, producing a x50 error. Fixed: use matrix products `C_c * zIA.solve(B_c) + D_c`.
- **Python binding disambiguation:** `suggest_order` is overloaded -- VectorXd (SubspaceID) dispatches from `advanced_bindings.cpp`; TruncationResult (BalancedTruncation) from `analysis_bindings.cpp`. Wrapped in lambdas to resolve the C++ overload ambiguity.

#### `SubspaceID` ([SubspaceID.h](lib/SubspaceID.h))
- **Purpose:** N4SID subspace identification of MIMO state-space models from PRBS or arbitrary excitation data.
- **Methods:** `n4sid(y, u, n, i)` -> `StateSpace`; `suggestOrder(y, u, maxOrder)` -> recommended model order from singular value elbow detection.
- **Inputs:** `y` (p x N output data matrix), `u` (m x N input data matrix), `n` (model order), `i` (block-row factor, typically n <= i <= 2n).

#### `GainScheduledController` ([GainScheduledController.h](lib/GainScheduledController.h)) *Part 20+23*
- **Purpose:** IController wrapper that interpolates between a sorted list of (p, IController) schedule points.
- **Modes:** `NearestNeighbor` (hard-switch; calls `bumplessInit` on the incoming controller when the active index changes -- Part 23 fix); `LinearBlend` (weighted average of adjacent controllers).
- **Methods:** `addSchedulePoint(p, ctrl)`, `setSchedulingParam(p)`, `compute(error)`, `lastOutput()` (not override).
- **Note:** `lastOutput()` is NOT a virtual override (no virtual lastOutput() in IController base).
- **LQR pattern:** Use `LQRAdapter` (which IS an IController) with a state-capturing lambda as the schedule point controller.

#### `GapMetric` ([GapMetric.h](lib/GapMetric.h)) *Part 20*
- **Purpose:** Nu-gap upper bound (SISO chordal metric) for measuring plant model distance.
- **Functions:** `nuGap(P1, P2, freq_points=200)` -> scalar in [0,1]; `nuGapMatrix(models)` -> N x N symmetric distance matrix; `freqResponseGrid(sys, omega)`.
- **Limitation:** SISO only; throws `invalid_argument` for MIMO plants.

#### `LinearModelCluster` ([LinearModelCluster.h](lib/LinearModelCluster.h)) *Part 20*
- **Purpose:** Single-linkage agglomerative clustering of plant models by nu-gap distance.
- **Functions:** `clusterByGap(gapMatrix, threshold)` -> `ClusterResult {labels, representatives, maxIntraGap, numClusters, threshold}`; `suggestGapThreshold(gapMatrix)`.

#### `LPVSystemID` ([LPVSystemID.h](lib/LPVSystemID.h)) *Part 20*
- **Purpose:** Polynomial LPV system identification via QR regression.
- **Functions:** `identifyLPV(X, U, Y, sched, degree, Ts)` -> `LPVModel`; `identifyLPVFromIO(U, Y, sched, n_states)` (uses n4sid first).
- **Layout:** X is (n x N) column-major (each column = one time step). U is (m x N), Y is (p x N). **CRITICAL: NOT (N x n) row-major.**
- **LPVModel:** `frozen(p)` -> `StateSpace`; `evalA(p)`, `evalB(p)`.

#### `AutoGainScheduler` ([AutoGainScheduler.h](lib/AutoGainScheduler.h)) *Part 20*
- **Purpose:** Pipeline: nonlinear plant -> equilibrium grid -> linearise -> gap cluster -> design controllers -> assemble GainScheduledController.
- **Functions:** `findEquilibrium(f, u_eq, x0)` (Newton-Raphson); `buildAutoGainScheduler(f, p_min, p_max, density, u_eq_fn, x0_fn, design_fn, Ts)`.
- **CRITICAL:** `design_fn` lambda must have trailing return type `-> std::shared_ptr<IController>` (GCC cannot deduce shared_ptr<Derived> -> shared_ptr<Base>).

#### `NonlinearMPC` ([NonlinearMPC.h](lib/NonlinearMPC.h)) *Part 22*
- **Purpose:** Nonlinear MPC via Real-Time Iteration (RTI, Diehl 2005). Discrete-time dynamics `x[k+1] = f(x[k], u[k])`.
- **Params:** `NMPCParams {Np, Nu, rho_y, rho_u, uMin, uMax, qpMaxIter, qpTol, Ts, n_states, n_inputs, n_outputs}`.
- **Usage:** `setState(x)`, `setReference(y_ref)`, `computeRef(x, y_ref)` (MIMO) or `compute(error)` (SISO).
- **Algorithm:** linearise along warm-started trajectory, build time-varying condensed QP (Theta matrix), solve via FISTA. Theta built with `Phi = Phi * A_list[k]` (right-multiply, k descending from j to 0).
- **Note:** `lastOutput()` is NOT a virtual override.

#### `AdaptiveSmithPredictor` ([AdaptiveSmithPredictor.h](lib/AdaptiveSmithPredictor.h)) *Part 22*
- **Purpose:** SmithPredictor with online dead-time estimation via cross-correlation `R_uy(tau) = sum u[k-tau]*y[k]`. Rebuilds SP when delay estimate changes.
- **Params:** `AdaptiveSPParams {maxDelaySteps, estimateInterval, bufferLen}`.
- **Usage:** `setPlantOutput(y)` before `compute(r-y)` each step (if not called, uses `y approx= -error` for r=0 regulation).

#### `AutoTuner` ([AutoTuner.h](lib/AutoTuner.h)) *Part 22, header-only*
- **Purpose:** CMA-ES black-box optimizer for controller parameter tuning (minimises arbitrary cost function).
- **Params:** `AutoTunerParams {n, sigma0, maxIter, tol, lower, upper}`. **Result:** `TunerResult {params, cost, nEvals, nGens, converged}`.
- **Usage:** `AutoTuner tuner(atp, seed); auto result = tuner.tune(cost_fn, x0);`
- **Box constraints:** implemented by clipping samples to [lower, upper].

#### `AntiWindupWrapper` ([AntiWindupWrapper.h](lib/AntiWindupWrapper.h)) *Part 24, header-only*
- **Purpose:** Generic anti-windup decorator for any IController using the Hanus (1987) conditioning technique. Prevents integrator windup in controllers without built-in saturation handling (DiscreteLQG, DiscreteHinf, GPC, custom).
- **Algorithm:** At each step: `e_in[k] = e[k] + Kb*(u_sat[k-1]-u_raw[k-1])` is passed to the inner controller; integral is bounded to `~ uMax + e/Kb` in steady saturation (vs unbounded without wrapper).
- **Ctor:** `AntiWindupWrapper(shared_ptr<IController> inner, uMin, uMax, Kb=1.0)`.
- **Key methods:** `compute(error)`, `isSaturated()`, `saturationError()`, `setActualOutput(u_applied)`, `lastOutput()`, `bumplessInit(u_target, error)`.
- **Do NOT use on DiscretePID** -- PID already has back-calculation via `PIDParams::Kb`. Double-wrapping applies conditioning twice.
- **Python:** `ctrl.AntiWindupWrapper(inner, uMin, uMax, Kb=1.0)`.

---

### 5.4 Tuning Layer

#### `RelayAutoTuner` ([ControllerTuner.h](lib/ControllerTuner.h))
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

#### `TunerSuite` ([TunerSuite.h](lib/TunerSuite.h))
- **Purpose:** Unified front-end dispatching to the eight tuning families with **runtime soft-warnings** (IDEAL -> no warning; SOFT -> diagnostic + `result.warned == true`; FALLBACK -> default params + `success == false`).
- **Methods:** `relayZN`, `imcPID`, `cohenCoon`, `bryson`, `kalmanNoise`, `mpcHorizon`, `loopShaping`, `optimise` (Nelder-Mead ISE/ITAE black-box).
- **Helpers:** `makeISECost`, `makeITAECost` - factory for cost functions used by `optimise`.

#### `ControllerTraits<C>` ([ControllerTraits.h](lib/ControllerTraits.h))
- **Purpose:** Compile-time mapping from controller type to supported tuners (booleans `supports_heuristic_pid`, `supports_lqr_tuning`, `supports_mpc_tuning`, `supports_freq_tuning`, `supports_kalman_tuning`).
- **Use:** `tuneFor<C>` static asserts fire with a diagnostic naming the correct alternative tuner.

---

<a name="55-composition--orchestration"></a>
### 5.5 Composition & Orchestration

#### `ControllerStack` ([ControllerStack.h](lib/ControllerStack.h))
- **Purpose:** Multi-controller orchestrator with three modes.
  - **Supervisory** - first entry whose `activationCondition(error, lastOutput)` returns `true` is used; others idle. Use for fallbacks and bumpless transfer. For continuous gain scheduling prefer `GainScheduledController`.
  - **Additive** - outputs of all enabled entries are summed. Use for inner/outer cascades.
  - **Weighted** - `u = Sigma w_i.u_i(e)`. Use for fuzzy blending.
- **Methods:** `addController(ptr, name, weight, condition)`, `removeController(name)`, `setActive(name, bool)`, `setWeight(name, w)`, `compute(error)`, `activeControllerName()`, `entries()`.

---

<a name="56-analysis--metrics"></a>
### 5.6 Analysis & Metrics

#### `MetricsAnalyzer` ([MetricsAnalyzer.h](lib/MetricsAnalyzer.h))
- **Purpose:** Extract time-domain metrics from step-response data.
- **Method:** `calculate(t_data, y_data, reference, finalValueWindow) -> TimeDomainMetrics { riseTime, settlingTime, peakOvershoot, steadyStateError }`.

#### `SystemAnalysis` ([SystemAnalysis.h](lib/SystemAnalysis.h))
- **Purpose:** Frequency-domain and stability analysis utilities (static methods).
- **Methods:** `getPoles(sys)`, `isDiscreteStable(sys)`, `solveDiscreteLyapunov(A, Q)`, `getFrequencyResponse(sys, freqs)`, `calculateMargins(sys) -> StabilityMargins { gainMarginDb, phaseMarginDeg, wCrossoverGain, wCrossoverPhase }`, `calculateHInfinityNorm(sys)` (grid approximation - treat as lower bound).

---

<a name="57-real-time-utilities--hal"></a>
### 5.7 Real-Time Utilities & HAL

#### `AtomicParamBuffer<Params>` ([AtomicParamBuffer.h](lib/AtomicParamBuffer.h))
- **Purpose:** Seqlock-based double-buffered parameter handoff between a background tuner and the real-time control thread. Single-writer / single-reader, data-race-free under the C++ memory model.
- **Constraint:** `Params` must be `std::is_trivially_copyable<Params>::value == true` (plain-old-data struct).
- **API:**
  - `Params read()` - RT thread. Returns a copy protected by the seqlock. Spins only if a `publish()` is mid-flight (typically zero retries).
  - `void publish(const Params& p)` - background thread. Writes to inactive slot, then atomically promotes it. Increments seqlock counter twice.
  - `Params latest()` - background thread only; non-seqlock-protected peek at the active slot.

#### HAL ([lib/hal/HAL.h](lib/hal/HAL.h))
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

## 6. Deployment Cross-References

For production deployment, parameter-stability constraints, RTOS integration, and troubleshooting recipes, consult [DEPLOYMENT.md](DEPLOYMENT.md):

- **Section 1** Per-controller parameter constraints (PID `Ki.Ts < 2.Kp`, ADRC `omega_o.Ts < 2`, MPC Hessian conditioning, ...).
- **Section 2** Real-time integration: zero-allocation checklist, stack-size estimates, RTOS scheduling.
- **Section 3** Troubleshooting: DARE non-convergence, MPC LDLT failure, Kalman divergence, ADRC ESO instability, Smith-predictor delay mismatch, NaN propagation.
- **Section 4** Quick-start parameter tables for an unknown SISO plant.

For tuning workflow choices and history, see [cheatsheet/tuning_methods.md](cheatsheet/tuning_methods.md) and [cheatsheet/controller-tuning-reference.md](cheatsheet/controller-tuning-reference.md). For system identification, see [cheatsheet/system_identification.md](cheatsheet/system_identification.md) and the FOPDT / ARMAX / N4SID sub-notes.

---

*End of documentation.*
