# Controller Toolbox

A discrete-time C++20 control library with PID, LQR, LQG, MPC, GPC, ADRC, SMC, H-infinity, Lead-Lag, Smith Predictor, Repetitive Control, Feedforward, Extremum Seeking, Kalman/EKF/UKF/MHE filtering, Fuzzy Logic inference, SOPDT/FOPDT identification, RLS and N4SID system identification, plus an integrated tuner suite, analysis layer, and Index-1 DAE utilities.

~85 controller implementations, nine tuning families, frequency- and time-domain analysis, corrector-pattern composition (Cascade / Additive / Observer+SF / Supervisory), a lock-free parameter buffer for RT updates, and a hardware abstraction layer for simulation.

Full pybind11 Python bindings expose every class to NumPy-aware Python scripts. C++ example programs and 100+ Python example scripts cover every controller, tuning method, identification approach, corrector pattern, and algorithm extension. Fifteen end-to-end physics case studies (nine C++: boiler-turbine, tug boat, solar cooling, porous-plate humidification, active suspension, buck-boost converter, solar cooker, solar OTEC, hydraulic SMISMO; six Python-only: drill string, wind-wave platform, electro-hydraulic force servo, aerial firefighting bag drop, battery thermal management, surface ship manoeuvring) exercise the full controller stack on nonlinear plants.

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
| [case-study/](case-study/) | Eleven full physics studies (9 C++ + 2 Python-only) -- see "Case Studies" below; per-study tracker in [docs/CASE_STUDIES.md](docs/CASE_STUDIES.md) |

---

## Repository Layout

```
|-- lib/             # Library sources -> target: controller_toolbox
|-- examples/        # ex01..ex79 single-file C++ demos (corrector patterns, new algorithms)
|-- examples/python/ # ex01..ex102 Python companion scripts and binding demos
|-- case-study/      # 15 physics studies (9 C++ + 6 Python-only) + spec-only stubs
|-- tests/           # CTest-driven unit + integration tests (Catch2 v3)
|-- bindings/        # pybind11 binding source files
|-- scripts/         # tune_all / simulate_all / realtime_all
|-- cheatsheet/      # Reference notes
|-- docs/            # Documentation & deployment guides
|-- tools/           # compare_controllers.py (IAE/ISE table across case-study CSVs)
```

---

## Case Studies

Fifteen self-contained physics studies under [case-study/](case-study/) exercise the
library end-to-end. Each pairs a nonlinear plant simulator with a roster of
controllers that wrap the `lib/` algorithms, then sweeps every controller across
several scenarios and writes CSV telemetry for post-processing. Per-study status,
rosters, and caveats are tracked in [docs/CASE_STUDIES.md](docs/CASE_STUDIES.md).

**C++ studies (built by `compile.bat`, run in Phase 4):**

| Study | Plant | Controllers | Scenarios x Runs |
|---|---|---|---|
| [Boiler Control](case-study/Boiler%20Control/) | Bell-Astrom 3x3 MIMO boiler-turbine | 27 | 8 -> 216 |
| [Tug Boat Numerical Simulation](case-study/Tug%20Boat%20Numerical%20Simulation/) | 3-DOF tug, 6-state MIMO + thrust allocation | 18 | 4 -> 72 |
| [Solar-Driven Cooling System](case-study/Solar-Driven%20Cooling%20System%20with%20Photovoltaic%20Evaporative%20Chimney/) | Algebraic SISO solar cooling + PV evaporative chimney | 14 | 5 -> 70 |
| [Porous Fiber Plate Humidification System](case-study/Porous%20Fiber%20Plate%20Humidification%20System/) | Laminar flat-plate evaporative humidifier + room ODE | 15 | 5 -> 75 |
| [Active Suspension](case-study/Active%20Suspension%20Mathematical%20Modeling%20and%20Optimization%202025/) | 2-DOF quarter-car (4-state RK4) | 15 | 5 -> 75 |
| [Non-Inverting Buck-Boost Converter](case-study/Non-Inverting%20Buck-Boost%20Converter/) | Averaged 2-state converter, 50 kHz, mode hysteresis | 12 | 5 -> 60 |
| [Solar Cooker with Reflector and Absorber](case-study/Solar%20Cooker%20with%20Reflector%20and%20Absorber/) | 2-state absorber+pot ODE with PCM effective-C | 12 | 5 -> 60 |
| [Solar Ocean Thermal Energy Conversion](case-study/Solar%20Ocean%20Thermal%20Energy%20Conversion%20System/) | 2-state collector+tank ODE + algebraic ORC map | 12 | 5 -> 60 |
| [Separate Meter In Separate Meter Out](case-study/Separate%20Meter%20In%20Separate%20Meter%20Out/) | SMISMO hydraulic cylinder, 8-state RK4, dual PDCVs + Stribeck friction | 12 | 5 -> 60 |

**Python-only studies (discovered by `run.py` Phase 6 via `sim/main.py`):**

| Study | Plant | Controllers | Scenarios x Runs |
|---|---|---|---|
| [Vertical Drill String](case-study/Vertical%20Drill%20String%20Mathematical%20Review%202025/) | 2-DOF torsional model, Stribeck bit friction (stick-slip) | 17 | 5 -> 85 |
| [Multi-Body Floating Wind-Wave Platform](case-study/Multi-Body%20Floating%20Wind-Wave%20Platform/) | 4-state FOWT heave + WEC arm, sinusoidal wave forcing | 16 | 5 -> 80 |
| [Tracking Control of Electro-Hydraulic Force Servo Systems](case-study/Tracking%20Control%20of%20Electro-Hydraulic%20Force%20Servo%20Systems/) | 5-state EHFS [P_A, P_B, x_v, v_p, x_p], servo valve + cylinder, RK4 Ts=0.5ms | 12 | 5 -> 60 |
| [High-Altitude Aerial Firefighting Bag Drop](case-study/High-Altitude%20Aerial%20Firefighting%20Bag%20Drop/) | 3D bag trajectory [x,y,z,vx,vy,vz], drag+gravity+wind, RK4 Ts=0.05s | 12 | 5 -> 60 |
| [Air-Cooled Battery Thermal Management System](case-study/Air-Cooled%20Battery%20Thermal%20Management%20System/) | 1-D transient HX, N=9 cells, 10 channels, J/U/L flow-pattern switching, Euler Ts=1s | 12 | 5 -> 60 |
| [Nonlinear Surface Ship Manoeuvring Control](case-study/Nonlinear%20Surface%20Ship%20Manoeuvring%20Control/) | 3-DOF MMG model, 19 SRUKF-identified params (Meng 2025), [u,v,r,ψ,x,y], RK4 Ts=0.08s | 12 | 5 -> 60 |

Controllers span the full stack: PID, LQR, LQG, MPC, GPC-RLS, SMC, ADRC, Fuzzy-PID,
Smith Predictor, MRAC, H-infinity, TubeMPC, ScenarioMPC, NonlinearMPC, Feedback
Linearisation, EKF-LQR, MHE-LQR, SubspaceID-LQG, L1Adaptive, ILC, NeuralPID,
DynaMBRL, CEM-MPC, Koopman-MPC, ESN, CBF safety filtering, ASMC, and gain-scheduled
(manual, LPV, and automated gap-metric) variants, depending on the plant.
Several spec-only stubs (README only, no `sim/`) remain as outstanding work --
see [docs/CASE_STUDIES.md](docs/CASE_STUDIES.md).

---

## Controller Inventory

| Category | Implementations |
|---|---|
| **Classical** | PID (backward-Euler, anti-windup, DoM, 2-DOF, b_weight), Lead-Lag, Smith Predictor (integer + fractional Pade), Feedforward, Repetitive Control |
| **Optimal** | LQR (DARE doubling), LQG (LQR+KF), MPC (condensed QP + box constraints), GPC (CARIMA+RLS adaptive), H-infinity (gamma bisection, mixed sensitivity, DK mu-synthesis with rational D) |
| **Robust / Nonlinear** | SMC (saturation boundary layer), ADRC (2nd-order LADRC + ESO), Extremum Seeker |
| **Adaptive** | MRACController (Lyapunov + sigma-modification + Euclidean projection), L1AdaptiveController (state predictor + LP-filtered adaptation), GPC::setPlant (RLS adaptive) |
| **Nonlinear** | FeedbackLinearisationController (affine-in-control SISO, relative degree 1; DriftFn+GainFn), NonlinearMPC (RTI, user-supplied StateFunc) |
| **Intelligent** | FuzzyPD, FuzzyPID, FuzzySupervisor (Mamdani & Takagi-Sugeno) |
| **Composition** | ControllerStack (Supervisory, Additive, Weighted) - cascade, observer+SF, bumpless transfer; ComputationalDelayWrapper, AntiWindupWrapper, CBFSafetyFilter |
| **Estimators** | KalmanFilter, EKF (analytical/numerical Jacobians + DAE algebraic projection), UKF (sigma-point), MovingHorizonEstimator (condensed QP + state box constraints) |
| **Identification** | FOPDTIdentifier, SOPDTIdentifier + Rivera 1986 IMC, RecursiveLeastSquares, SubspaceID (N4SID), SINDy (STLS sparse regression), KoopmanEDMD (PolyDeg/RBF dictionary) |
| **ML / Data-driven** | GaussianProcess (SE kernel, Cholesky, fixed-budget), EchoStateNetwork (spectral-radius reservoir), NeuralPID (3→n_h→3 online backprop), CEMController (elite-sample stochastic MPC), DynaController (Sutton Dyna MBRL + SINDy model), ILC (P-type / D-type / norm-optimal), DeePC (ADMM data-enabled predictive control), ScenarioMPC, BayesianOptimizer |
| **Model utilities** | LinearisationHelper (jacobianX/U, lineariseAtPoint ZOH), BalancedTruncation (Hinf bound), ZeroPhaseTrackingFilter (ZPETC + transmissionZeros), GapMetric (chordal SISO + subspace MIMO) |
| **DAE utilities** | `DAESystem` (Index-1 semi-explicit, `f`/`g`/`h` functors), `consistentInit` (Newton-Raphson), `dae2ode` (Euler+Newton discrete step function), `c2d(DAESystem)` (algebraic elimination + ZOH/Tustin) |

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

**Baseline (Part 51 - 2026-06-12, UNVERIFIED until next clean `run.py`):**
C++ case studies: Boiler 216/216 . Tug 72/72 . Solar 70/70 . Humidification 75/75 .
ActiveSuspension 75/75 . BuckBoost 60/60 . SolarCooker 60/60 . SOTEC 60/60 . SMISMO 60/60.
Python-only (Phase 6): DrillString 85/85 . WindWave 80/80 . EHFS 60/60 .
Firefighting 60/60 . BTMS 60/60 . SurfaceShip 60/60.
`bug_report.txt`: 0 blocks expected after a clean run (`safe_phrases` list in `run.py` suppresses all known benign messages).

**Part 51 (2026-06-12) — DAE architecture (P1/P2/P3):**
- `DAESystem` struct (`lib/PlantModel.h/.cpp`): Index-1 semi-explicit DAE with `f` (differential), `g` (algebraic), `h` (output) functors; `n_diff`/`n_alg`/`Ts` fields.
- `consistentInit`: Newton-Raphson (LDLT) solving `g(x1,x2,u)=0` for `x2`.
- `dae2ode`: discrete step function via forward Euler on `x1` + Newton projection on `x2`.
- `c2d(DAESystem, x1_op, x2_op, u_op, Ts, method)`: Index-1 algebraic elimination (`A_red = A11 - A12*G2⁻¹*G1`), then ZOH/Tustin. Throws `runtime_error` if G2 singular.
- DAE-aware EKF (`lib/ExtendedKalmanFilter.h/.cpp`): `setAlgebraicConstraint(g, n_diff, n_alg)` projects `x2` block via Newton after each `update()`; covariance projected as `P = J_proj*P*J_proj'`.
- pybind11 bindings: `DAESystem`, `consistent_init`, `dae2ode`, `dae_c2d`, `set_algebraic_constraint` on EKF.
- 7 Catch2 tests: `[dae_system]` ×3, `[dae_c2d]` ×2, `[dae_ekf]` ×2.

**Part 49 (2026-06-11) — Nonlinear Surface Ship Manoeuvring Control (Python-only):**
- 3-DOF MMG model, 19 SRUKF-identified parameters (Meng 2025 Table 5), [u,v,r,ψ,x,y], RK4 Ts=0.08s.
- 12 controllers × 5 scenarios = 60 runs. Includes ASMC (paper cascade + disturbance FF), MPC (ZOH [ψ,r] linearisation), LQR (Bryson DARE), MRAC, L1Adaptive, ADRC, GainScheduled, NeuralPID, ILC.

**Part 48 (2026-06-11) — Air-Cooled Battery Thermal Management System (Python-only):**
- 1-D transient HX model: N=9 cells, 10 channels, J/U/L flow-pattern switching, Forward Euler Ts=1s.
- 12 controllers × 5 scenarios = 60 runs.

**Part 46 (2026-06-10) — High-Altitude Aerial Firefighting Bag Drop (Python-only):**
- 3D trajectory [x,y,z,vx,vy,vz], drag+gravity+wind, RK4 Ts=0.05s. Primary metric: CEP (50th-percentile radial error). N_mc=30 Monte-Carlo trajectories per planner/scenario.
- 12 planners × 5 scenarios = 60 runs.

**Part 45 (2026-06-10) — Electro-Hydraulic Force Servo Systems (Python-only):**
- 5-state EHFS [P_A, P_B, x_v, v_p, x_p], servo valve + cylinder, RK4 Ts=0.5ms.
- 12 controllers × 5 scenarios = 60 runs. Includes FeedbackLinearisation, LQR, ADRC (omega_o=800, omega_o*Ts=0.40<0.5), GainScheduled.

**Part 44 (2026-06-10) — SMISMO C++ case study reimplemented:**
- Separate Meter In Separate Meter Out hydraulic cylinder (Chen 2018 + Liu 2009). 8-state RK4, dual PDCV spool dynamics, Stribeck friction, 20 bar backpressure regulation.
- 12 controllers × 5 scenarios = 60 runs, target `smismo_sim`. Recreated `test_smismo_regression.cpp`.

Details in [docs/cumulative_bug_report.md](docs/cumulative_bug_report.md). Real-time deployment guidance in [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md).
