# Controller Toolbox

A discrete-time C++20 control library with PID, LQR, LQG, MPC, GPC, ADRC, SMC, H-infinity, Lead-Lag, Smith Predictor, Repetitive Control, Feedforward, Extremum Seeking, Kalman/EKF/UKF/MHE filtering, Fuzzy Logic inference, SOPDT/FOPDT identification, RLS and N4SID system identification, plus an integrated tuner suite and analysis layer.

Thirty-plus controller implementations, nine tuning families, frequency- and time-domain analysis, corrector-pattern composition (Cascade / Additive / Observer+SF / Supervisory), a lock-free parameter buffer for RT updates, and a hardware abstraction layer for simulation.

Full pybind11 Python bindings expose every class to NumPy-aware Python scripts. 69 C++ example programs and 88 Python example scripts cover every controller, tuning method, identification approach, corrector pattern, and new algorithm extension. Four end-to-end physics case studies (boiler-turbine, hydraulic SMISMO, tug boat, solar cooling) exercise the full controller stack on nonlinear plants.

---

## Quick Start

### Native build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Requires **C++20**, **CMake >= 3.16**, and **Eigen >= 3.4**.

### Run all examples

```bash
conda activate soft_robotics
python run.py
```

`run.py` compiles every C++ target sequentially, runs each executable, then runs all Python examples and reports a pass/fail summary.

### Docker build

```bash
docker build -t controller-toolbox .
docker run --rm controller-toolbox          # runs the test suite
docker run --rm controller-toolbox ex02_ss_lqr   # runs a specific example
```

See [Docker Usage](#docker-usage) for more.

---

## Minimal Example

```cpp
#include "ControllerToolbox.h"

const double Ts = 0.01;
ctrl::TransferFunction G({0.0048, 0.0047}, {1.0, -1.81, 0.819}, Ts);
ctrl::StateSpace sys = ctrl::tf2ss(G);

ctrl::PIDParams pp;
pp.Kp = 1.0; pp.Ki = 0.1; pp.Kd = 0.05; pp.N = 100.0;
ctrl::DiscretePID pid(pp, Ts);

Eigen::VectorXd x = Eigen::VectorXd::Zero(sys.stateSize());
double r = 1.0, y = 0.0;
for (int k = 0; k < 500; ++k) {
    double u = pid.compute(r - y);
    Eigen::VectorXd uv(1); uv << u;
    y = ctrl::ssStep(sys, x, uv)(0);
}
```

---

## Documentation

| Document | Purpose |
|---|---|
| [docs/DOCUMENTATION.md](docs/DOCUMENTATION.md) | Full API reference, class-by-class breakdown, usage workflows |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | Parameter constraints, RT/RTOS integration, troubleshooting recipes |
| [docs/TEST_UPDATE.md](docs/TEST_UPDATE.md) | Test suite history, regression coverage, sign-convention notes |
| [docs/CONTROL_STRATEGIES_DEEP_DIVE.md](docs/CONTROL_STRATEGIES_DEEP_DIVE.md) | Taxonomy of strategy families, plant model interference, decision framework, proposed extensions |
| [cheatsheet/](cheatsheet/) | Tuning methods, controller categories, system identification notes |
| [case-study/](case-study/) | Four full physics studies (boiler-turbine, hydraulic SMISMO, tug boat, solar cooling) -- see "Case Studies" below |

---

## Repository Layout

```
|-- lib/             # Library sources -> target: controller_toolbox
|-- examples/        # ex01..ex54 single-file C++ demos (corrector patterns, new algorithms)
|-- examples/python/ # ex01..ex70 Python companion scripts and binding demos
|-- case-study/      # 4 physics studies: boiler-turbine, SMISMO hydraulic, tug boat, solar cooling
|-- tests/           # CTest-driven unit + integration tests (Catch2 v3)
|-- bindings/        # pybind11 binding source files
|-- scripts/         # tune_all / simulate_all / realtime_all
|-- cheatsheet/      # Reference notes
|-- docs/            # Documentation & deployment guides
```

---

## Case Studies

Four self-contained physics studies under [case-study/](case-study/) exercise the
library end-to-end. Each pairs a nonlinear plant simulator with a roster of
controllers that wrap the `lib/` algorithms, then sweeps every controller across
several scenarios and writes CSV telemetry for post-processing.

| Study | Plant | Controllers | Scenarios x Runs |
|---|---|---|---|
| [Boiler Control](case-study/Boiler%20Control/) | Bell-Astrom 3x3 MIMO boiler-turbine | 27 | 8 -> 216 |
| [Meter In Meter Out Control](case-study/Meter%20In%20Meter%20Out%20Control/) | SMISMO 9-state hydraulic actuator | 14 | 3 -> 42 |
| [Tug Boat Numerical Simulation](case-study/Tug%20Boat%20Numerical%20Simulation/) | 3-DOF tug, 6-state MIMO + thrust allocation | 16 | 4 -> 64 |
| [Solar-Driven Cooling System](case-study/Solar-Driven%20Cooling%20System%20with%20Photovoltaic%20Evaporative%20Chimney/) | Algebraic SISO solar cooling + PV evaporative chimney | 9 | 5 -> 45 |

Controllers span the full stack: PID, LQR, LQG, MPC, GPC-RLS, SMC, ADRC, Fuzzy-PID,
Smith Predictor, MRAC, H-infinity, TubeMPC, NonlinearMPC, Feedback Linearisation,
EKF-LQR, MHE-LQR, SubspaceID-LQG, and gain-scheduled (manual, LPV, and automated
gap-metric) variants, depending on the plant.

---

## Controller Inventory

| Category | Implementations |
|---|---|
| **Classical** | PID (backward-Euler, anti-windup, DoM, 2-DOF, b_weight), Lead-Lag, Smith Predictor (integer + fractional Pade), Feedforward, Repetitive Control |
| **Optimal** | LQR (DARE doubling), LQG (LQR+KF), MPC (condensed QP + box constraints), GPC (CARIMA+RLS adaptive), H-infinity (gamma bisection, mixed sensitivity, DK mu-synthesis with rational D) |
| **Robust / Nonlinear** | SMC (saturation boundary layer), ADRC (2nd-order LADRC + ESO), Extremum Seeker |
| **Adaptive** | MRACController (Lyapunov + sigma-modification + Euclidean projection), GPC::setPlant (RLS adaptive) |
| **Nonlinear** | FeedbackLinearisationController (affine-in-control SISO, relative degree 1; DriftFn+GainFn) |
| **Intelligent** | FuzzyPD, FuzzyPID, FuzzySupervisor (Mamdani & Takagi-Sugeno) |
| **Composition** | ControllerStack (Supervisory, Additive, Weighted) - cascade, observer+SF, bumpless transfer |
| **Estimators** | KalmanFilter, EKF (analytical/numerical Jacobians), UKF (sigma-point), MovingHorizonEstimator (condensed QP) |
| **Identification** | FOPDTIdentifier, SOPDTIdentifier + Rivera 1986 IMC, RecursiveLeastSquares, SubspaceID (N4SID) |
| **Model utilities** | LinearisationHelper (jacobianX/U, lineariseAtPoint ZOH), BalancedTruncation (Hinf bound), ZeroPhaseTrackingFilter (ZPETC + transmissionZeros) |

---

## Docker Usage

The included [`Dockerfile`](Dockerfile) uses a two-stage build:

- **Stage 1 (builder)** - Debian Bookworm slim + CMake + g++ + libeigen3-dev, compiles every target with the root [`CMakeLists.txt`](CMakeLists.txt).
- **Stage 2 (runtime)** - Slim image containing only the compiled binaries, ready to run examples, tests, or your own application.

### Build the image

```bash
docker build -t controller-toolbox .
```

### Run the test suite (default `CMD`)

```bash
docker run --rm controller-toolbox
```

### Run any example or script

```bash
docker run --rm controller-toolbox ex07_lqg_kalman
docker run --rm controller-toolbox boiler_turbine_case_study
docker run --rm controller-toolbox simulate_all
```

### Interactive shell for development

```bash
docker run --rm -it --entrypoint /bin/bash controller-toolbox
```

### Mount your own source for in-container builds

```bash
docker run --rm -it -v "$(pwd):/work" -w /work \
    --entrypoint /bin/bash controller-toolbox:builder \
    -c "cmake -S . -B build && cmake --build build"
```

(Use the `builder` stage tag - see the Dockerfile for details.)

---

## Tested Compilers

| Compiler | Version | Status |
|---|---|---|
| GCC | 9, 11, 12, 13 | OK |
| Clang | 10, 14, 16 | OK |
| MSVC | 19.20+ (VS 2019/2022) | OK |

---

## Project Status

**Current test counts (2026-05-28):** 78 C++ executables pass | 79 Python examples pass | 0 failures.
test_catch2_advanced: 51 test cases, 189 assertions. All Catch2 tags: [qp][mpc][gpc][kalman][lqr][smc][adrc][leadlag][hinf][ekf][ukf][rc][rls][n4sid][c2d][stack][observer][sopdt][mhe][linearisation][fl][mrac][btm][zpetc].

**Part 18 additions (2026-05-28):**
- `SOPDTIdentifier` - graphical (ZN tangent + 28.3%/63.2% crossings) and optimization (nested golden-section) SOPDT step-response identification; Rivera 1986 IMC-PID tuning for SOPDT models.
- `MovingHorizonEstimator` - condensed QP dual of MPC; reuses `GradientProjectionQP`; box constraints on process noise; horizon ramp-up from 1 to N.
- `DiscreteHinf::solveMuSyn` extended with full rational D-scaling (first-order D_j(z) fit per channel, plant state augmentation).
- 23 new C++ examples (ex32-ex54) and 21 new Python examples (ex50-ex70) covering corrector patterns: Cascade, Additive, Observer+SF, Supervisory.

**Extension session (2026-05-28, E1-E5 all implemented):**
- `MRACController` - Lyapunov MRAC with sigma-modification and parameter projection; ADRC-convention `compute(y_plant)`. Examples: ex57/ex73.
- `FeedbackLinearisationController` - exact FL for SISO affine-in-control; DriftFn+GainFn; `setState(x)` required each step. Examples: ex56/ex72.
- `LinearisationHelper` - `jacobianX/U` (central-diff, scaled epsilon); `lineariseAtPoint` ZOH. Examples: ex55/ex71.
- `BalancedTruncation` - `balancedTruncate(sys, r)` with Hinf error bound 2.Sigmasigma_i; `suggestOrder`. Examples: ex58/ex74.
- `ZeroPhaseTrackingFilter` - ZPETC (Tomizuka 1987); `transmissionZeros` via `GeneralizedEigenSolver`. Examples: ex59/ex75.

**Earlier additions:**
- `FuzzyLogic` module (2026-05-23): Mamdani and Takagi-Sugeno inference engines, `FuzzyPD`, `FuzzyPID`, `FuzzySupervisor` - see [docs/DOCUMENTATION.md Section 5.2](docs/DOCUMENTATION.md).
- `ExtendedKalmanFilter`, `UnscentedKalmanFilter`, `RepetitiveController`, `GeneralizedPredictiveController`, `SubspaceID`, `DiscreteHinf`, `FOPDTIdentifier`, `SOPDTIdentifier`, `MovingHorizonEstimator`, `FeedforwardController`, `GradientProjectionQP`.
- Five math corrections (2026-05-22): MPC condensed prediction formula, PID backward-Euler integral law, LQG D!=0 staleness warning, Smith Predictor feedthrough, SMC `c_de` Ts-absorption.

Details in [docs/cumulative_bug_report.md](docs/cumulative_bug_report.md). Real-time deployment guidance in [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md).
