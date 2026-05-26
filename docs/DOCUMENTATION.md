# Controller Toolbox - Technical Documentation

*Discrete-time control library in modern C++17 (Eigen 3.4+).
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
   - 5.3 [Estimators](#53-estimators)
   - 5.4 [Tuning Layer](#54-tuning-layer)
   - 5.5 [Composition & Orchestration](#55-composition--orchestration)
   - 5.6 [Analysis & Metrics](#56-analysis--metrics)
   - 5.7 [Real-Time Utilities & HAL](#57-real-time-utilities--hal)
6. [Deployment Cross-References](#6-deployment-cross-references)

---

<a name="1-setup-and-environment"></a>
## 1. Setup and Environment

### 1.1 Required Dependencies

| Component | Version | Purpose |
|-----------|---------|---------|
| C++ compiler | C++17 (GCC >= 9, Clang >= 10, MSVC >= 19.20) | Source language |
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
|-- case-study/             # Boiler-turbine multivariable case study
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
target_compile_features(your_target PRIVATE cxx_std_17)
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
| [IController.h](lib/IController.h) | Abstract controller interface |
| [PlantModel.h](lib/PlantModel.h) | `TransferFunction`, `StateSpace`, `tf2ss`, `ssStep` |
| [DiscretePID.h](lib/DiscretePID.h) | PID with derivative filter + anti-windup |
| [DiscreteLQR.h](lib/DiscreteLQR.h) | Infinite-horizon LQR, DARE solver, `LQRAdapter` |
| [DiscreteMPC.h](lib/DiscreteMPC.h) | Condensed receding-horizon QP, `setPlant()` for adaptive MPC |
| [DiscreteLQG.h](lib/DiscreteLQG.h) | LQR + Kalman output-feedback combo |
| [DiscreteSMC.h](lib/DiscreteSMC.h) | First-order sliding mode with boundary layer |
| [DiscreteADRC.h](lib/DiscreteADRC.h) | 2nd-order LADRC (ESO + PD) |
| [DiscreteLeadLag.h](lib/DiscreteLeadLag.h) | Tustin-discretised lead-lag biquad |
| [SmithPredictor.h](lib/SmithPredictor.h) | Dead-time compensator wrapper |
| [ExtremumSeeker.h](lib/ExtremumSeeker.h) | Perturbation-based ESC |
| [FuzzyLogic.h](lib/FuzzyLogic.h) | Mamdani/TS inference engine, `FuzzyPD`, `FuzzyPID`, `FuzzySupervisor` |
| [KalmanFilter.h](lib/KalmanFilter.h) | Standalone linear KF |
| [ControllerStack.h](lib/ControllerStack.h) | Supervisory / Additive / Weighted composition |
| [ControllerTuner.h](lib/ControllerTuner.h) | Per-family tuners (Relay, FOPDT, Bryson, MPC, ...) |
| [TunerSuite.h](lib/TunerSuite.h) | Unified runtime-dispatched tuner with soft warnings |
| [ControllerTraits.h](lib/ControllerTraits.h) | Compile-time traits for tuner \leftrightarrow controller compatibility |
| [MetricsAnalyzer.h](lib/MetricsAnalyzer.h) | Time-domain step-response metrics |
| [SystemAnalysis.h](lib/SystemAnalysis.h) | Poles, margins, Hinf, Lyapunov |
| [AtomicParamBuffer.h](lib/AtomicParamBuffer.h) | Lock-free param double-buffer for RT updates |
| [hal/HAL.h](lib/hal/HAL.h) | `ISensor`, `IActuator`, `SimPlant`, `SimSensor`, `SimActuator` |

### 3.2 Examples (`examples/`)

27 single-file programs numbered `ex01_*` through `ex26_*` plus `example_pid_feedback`. Each demonstrates one controller or composition pattern (see [examples/CMakeLists.txt](examples/CMakeLists.txt) for the full enumeration). The `examples/cpp/` subdirectory contains MIMO / coupled-plant scenarios (`mimo_known`, `mimo_unknown`, `siso_coupled`, `siso_unknown`).

The fuzzy examples `ex23`-`ex26` cover four distinct application domains:

| Example | Application | Fuzzy element |
|---------|-------------|---------------|
| `ex23_fuzzy_pd_temperature` | HVAC zone temperature | `FuzzyPD` vs PID |
| `ex24_fuzzy_pid_dc_motor` | DC motor speed ramp | `FuzzyPID` anti-windup |
| `ex25_fuzzy_supervisor_mpc` | pH neutralisation (nonlinear) | `FuzzySupervisor` driving adaptive MPC |
| `ex26_fuzzy_ts_gain_scheduler` | Inverted pendulum balancing | Hand-built TS `FuzzySystem` blending two PIDs |

Companion Python scripts in `examples/python/` (ex23-ex26) generate identical data via self-contained Python implementations and produce comparison plots saved to `examples/data/`.

The Python folder `examples/python/` mirrors many of the C++ demos for cross-validation against `python-control`.

### 3.3 Case Study (`case-study/`)

**Boiler-Turbine:** [boiler_turbine_case_study.cpp](case-study/boiler_turbine_case_study.cpp) - a 3*3 nonlinear Bell-Astrom boiler-turbine model with linearisation, then LQR / MPC / LQG / PID / SMC / Extremum-Seeker comparison across three operating points (Low / Medium / High Load). See [case-study/verdict_boiler_turbine.md](case-study/verdict_boiler_turbine.md) for the analysis verdict.

**Tug Boat Numerical Simulation:** [`case-study/Tug Boat Numerical Simulation/`](case-study/Tug%20Boat%20Numerical%20Simulation/) - a 3-DOF marine vessel simulation (Li et al. 2026, Ocean Engineering 357) running 7 controllers across 4 environmental scenarios (28 runs total). Controllers: PID, KF-PID, SMC, MPC, ESC, FuzzyPID, FuzzySup-MPC. Uses the full Toolbox including `FuzzyLogic`, `KalmanFilter`, `DiscreteMPC`, `DiscreteSMC`, `ExtremumSeeker`, and `PlantModel`. Outputs CSV telemetry for Python post-processing. See the planning documents in that folder for mathematical model detail.

### 3.4 Tests (`tests/`)

- `test_controllers.cpp` - per-class unit tests
- `test_tuners_extended.cpp` - tuner suite tests (covers all 8 strategies)
- `test_integration.cpp` - end-to-end closed-loop tests
- `test_framework.h` - lightweight assertion macros

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
                     1.0, [](double e, double){ return std::abs(e) > 5.0; });   // SMC only when error large
double u = stack->compute(error);
```

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

#### `IController` ([IController.h](lib/IController.h))
- **Purpose:** Abstract base for all SISO controllers; uniform interface for stacking and tuning.
- **Pure-virtual:** `compute(double signal) -> double`, `reset()`, `sampleTime() const -> double`.
- **Convention:** `signal` is the **error** `e = r - y` for tracking controllers, the **plant output** for optimisation controllers (ESC) and observer-based controllers (ADRC, LQG).

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

### 5.3 Estimators

#### `KalmanFilter` ([KalmanFilter.h](lib/KalmanFilter.h))
- **Purpose:** Linear discrete Kalman filter (predict / update) with Joseph-form covariance update.
- **Constructor:** `(plant, Q_noise, R_noise, P0 = I)`.
- **Methods:** `predict(u)`, `update(y, u_current)`, `step(y, u_prev)` (one-call), `state()`, `covariance()`, `reset()`.
- **Floor:** `R_noise` has an automatic floor of `1e-12` per diagonal element to avoid division by zero.

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
  - **Supervisory** - first entry whose `activationCondition(error, lastOutput)` returns `true` is used; others idle. Use for gain scheduling, fallbacks.
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
