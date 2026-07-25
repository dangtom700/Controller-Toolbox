# Controller Toolbox

A discrete-time C++20 control library with PID, LQR, LQG, MPC, GPC, ADRC, SMC, H-infinity, Lead-Lag, Smith Predictor, Repetitive Control, Feedforward, Extremum Seeking, Kalman/EKF/UKF/MHE filtering, Fuzzy Logic inference, SOPDT/FOPDT identification, RLS and N4SID system identification, Grey-Box / Recursive Grey-Box parameter estimation, GP residual models, DAE utilities, GA/PSO/DE metaheuristic optimisers, and a ROS 2 lifecycle node adapter.

~125 controller and estimation implementations, nine tuning families, frequency- and time-domain analysis, corrector-pattern composition (Cascade / Additive / Observer+SF / Supervisory), a lock-free parameter buffer for RT updates, flat-C code generation for step-based controllers, an analysis pipeline (Monte Carlo, fault injection, ANOVA, WCET, mu-analysis, HTML reports), and a hardware abstraction layer for simulation.

Full pybind11 Python bindings expose every class to NumPy-aware Python scripts. 126 C++ and 149 Python example scripts cover every controller, tuning method, identification approach, corrector pattern, and algorithm extension. Twenty-one end-to-end physics case studies (eleven C++: boiler-turbine, tug boat, solar cooling, porous-plate humidification, active suspension 2-DOF, buck-boost converter, solar cooker, solar OTEC, hydraulic SMISMO, 6-DOF Stewart platform, bouyancy-driven airship; ten Python-only: drill string, wind-wave platform, electro-hydraulic force servo, aerial firefighting bag drop, battery thermal management, surface ship manoeuvring, active suspension 40-state 6x6 EV, aircraft engine thermal management, PCM thermal energy storage, satellite launch vehicle) exercise the full controller stack on nonlinear plants, plus one MATLAB-native study (Boiler Control MATLAB) built on the R2026a toolboxes.

Current status, verified counts, and open work: [docs/PROJECT_MASTER_STATE.md](docs/PROJECT_MASTER_STATE.md). Live case-study status: [docs/case_study_status.md](docs/case_study_status.md) (auto-generated).

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
| [docs/index.md](docs/index.md) | Full documentation map -- every committed doc, organized by task |
| [docs/handoff.md](docs/handoff.md) | Onboarding tribal knowledge -- current, verified; read before trusting any status doc |
| [docs/DOCUMENTATION.md](docs/DOCUMENTATION.md) | Full API reference, class-by-class breakdown, usage workflows |
| [docs/deployment.md](docs/deployment.md) | Parameter constraints, RT/RTOS integration, troubleshooting recipes |
| [docs/archived/test_update.md](docs/archived/test_update.md) | Test suite history, regression coverage, sign-convention notes |
| [docs/control_strategies_deep_dive.md](docs/control_strategies_deep_dive.md) | Taxonomy of strategy families, plant model interference, decision framework, proposed extensions |
| [cheatsheet/](cheatsheet/) | Tuning methods, controller categories, system identification notes |
| [case-study/](case-study/) | Twenty-one complete physics studies (11 C++ + 10 Python-only) + 1 MATLAB-native + 10 stubs -- see "Case Studies" below; auto-tracked status in [docs/case_study_status.md](docs/case_study_status.md) |

---

## Repository Layout

```
|-- lib/             # Library sources -> target: controller_toolbox (119 .h + 89 .cpp, ~125 modules)
|-- lib/embedded/    # Header-only embedded subset (BasicPID, BasicSMC, DiscreteIntegrator, ...)
|-- examples/        # 126 single-file C++ demos
|-- examples/python/ # 149 Python companion scripts and binding demos
|-- case-study/      # 21 complete physics studies (11 C++ + 10 Python-only)
|                    #   + Boiler Control MATLAB (MATLAB-native) + 10 spec-only/placeholder stubs
|-- tests/           # CTest-driven unit + integration tests (Catch2 v3)
|-- bindings/        # pybind11 binding source files + smoke_test.py
|-- ros2/            # ROS 2 package: ctrl_toolbox_ros2 (ControllerNode<T> lifecycle adapter)
|-- scripts/         # tune_all / simulate_all / realtime_all
|-- cheatsheet/      # Reference notes (controller categories, tuning, embedded, DAE, ...)
|-- docs/            # Documentation & deployment guides
|-- tools/           # Analysis pipeline: metrics, compare_controllers, monte_carlo, fault_injector,
|--                  #   fault_sweep, anova, wcet_report, model_validation, mu_analysis,
|--                  #   generate_report, case_study_tracker
|-- cmake/           # CMake config templates + vcpkg port files (DIST-1)
```

---

## Case Studies

Twenty-one self-contained physics studies under [case-study/](case-study/) exercise the
library end-to-end. Each pairs a nonlinear plant simulator with a roster of
controllers that wrap the `lib/` algorithms, then sweeps every controller across
several scenarios and writes CSV telemetry for post-processing. Per-study status and
links are auto-tracked in [docs/case_study_status.md](docs/case_study_status.md)
(regenerate via `tools/case_study_tracker.py`); rosters and tribal knowledge live in
`CLAUDE.md`'s Case Studies section and in each study's own `README.md`.

**C++ studies (built by `compile.bat`/`compile.sh`, run in Phase 5):**

| Study | Plant | Controllers | Scenarios x Runs |
|---|---|---|---|
| [Boiler Control](case-study/Boiler%20Control/) | Bell-Astrom 3x3 MIMO boiler-turbine | 27 | 8 -> 216 |
| [Tug Boat Numerical Simulation](case-study/Tug%20Boat%20Numerical%20Simulation/) | 3-DOF tug, 6-state MIMO + thrust allocation | 18 | 4 -> 72 |
| [Solar-Driven Cooling System](case-study/Solar-Driven%20Cooling%20System%20with%20Photovoltaic%20Evaporative%20Chimney/) | Algebraic SISO solar cooling + PV evaporative chimney | 14 | 5 -> 70 |
| [Porous Fiber Plate Humidification System](case-study/Porous%20Fiber%20Plate%20Humidification%20System/) | Laminar flat-plate evaporative humidifier + room ODE | 15 | 5 -> 75 |
| [Active Suspension](case-study/Active%20Suspension%20Mathematical%20Modeling%20and%20Optimization%202025/) | 2-DOF quarter-car (4-state RK4) | 18 | 5 -> 90 |
| [Non-Inverting Buck-Boost Converter](case-study/Non-Inverting%20Buck-Boost%20Converter/) | Averaged 2-state converter, 50 kHz, mode hysteresis | 12 | 5 -> 60 |
| [Solar Cooker with Reflector and Absorber](case-study/Solar%20Cooker%20with%20Reflector%20and%20Absorber/) | 2-state absorber+pot ODE with PCM effective-C | 12 | 5 -> 60 |
| [Solar Ocean Thermal Energy Conversion](case-study/Solar%20Ocean%20Thermal%20Energy%20Conversion%20System/) | 2-state collector+tank ODE + algebraic ORC map | 12 | 5 -> 60 |
| [Separate Meter In Separate Meter Out](case-study/Separate%20Meter%20In%20Separate%20Meter%20Out/) | SMISMO hydraulic cylinder, 8-state RK4, dual PDCVs + Stribeck friction | 13 | 5 -> 65 |
| [6-DOF Stewart Platform Vessel Motion Simulator](case-study/6-DOF%20Stewart%20Platform%20Vessel%20Motion%20Simulator/) | 6-UPU Stewart platform, 12-state per-rod spring-mass-damper, closed-form IK+Jacobian, Douglas sea-state CFD-input stand-in | 12 | 60 -> 720 |
| [Bouyancy-Driven Airship in Vertical Plane](case-study/Bouyancy-Driven%20Airship%20in%20Vertical%20Plane/) | 6-state liberated-center airship, moving-mass + net-lift actuation, RK4 (Ts=0.05s) | 12 | 5 -> 60 |

**Python-only studies (discovered by `run.py` Phase 7 via `sim/main.py`):**

| Study | Plant | Controllers | Scenarios x Runs |
|---|---|---|---|
| [Vertical Drill String](case-study/Vertical%20Drill%20String%20Mathematical%20Review%202025/) | 2-DOF torsional model, Stribeck bit friction (stick-slip) | 17 | 5 -> 85 |
| [Multi-Body Floating Wind-Wave Platform](case-study/Multi-Body%20Floating%20Wind-Wave%20Platform/) | 4-state FOWT heave + WEC arm, sinusoidal wave forcing | 16 | 5 -> 80 |
| [Tracking Control of Electro-Hydraulic Force Servo Systems](case-study/Tracking%20Control%20of%20Electro-Hydraulic%20Force%20Servo%20Systems/) | 5-state EHFS [P_A, P_B, x_v, v_p, x_p], servo valve + cylinder, RK4 Ts=0.5ms | 14 | 5 -> 70 |
| [High-Altitude Aerial Firefighting Bag Drop](case-study/High-Altitude%20Aerial%20Firefighting%20Bag%20Drop/) | 3D bag trajectory [x,y,z,vx,vy,vz], drag+gravity+wind, RK4 Ts=0.05s | 12 | 5 -> 60 |
| [Air-Cooled Battery Thermal Management System](case-study/Air-Cooled%20Battery%20Thermal%20Management%20System/) | 1-D transient HX, N=9 cells, 10 channels, J/U/L flow-pattern switching, Euler Ts=1s | 12 | 5 -> 60 |
| [Nonlinear Surface Ship Manoeuvring Control](case-study/Nonlinear%20Surface%20Ship%20Manoeuvring%20Control/) | 3-DOF MMG model, 19 SRUKF-identified params (Meng 2025), [u,v,r,ψ,x,y], RK4 Ts=0.08s | 12 | 5 -> 60 |
| [Active Suspension 6x6 EV Full Model](case-study/Active%20Suspension%206x6%20EV%20Full%20Model/) | 40-state 20-DOF (body+wheels+motors+5-DOF human biodynamic), ZOH Ts=0.005s, 6 actuators | 18 | 5 -> 90 |
| [PCM Thermal Energy Storage Control](case-study/PCM%20Thermal%20Energy%20Storage%20Control/) | Phase-change-material store + variable-speed heat pump, price-driven load shifting | 12 | 5 -> 60 |
| [Satellite Launch Vehicle Systems](case-study/Satellite%20Launch%20Vehicle%20Systems/) | Pitch-plane rigid-body SLV, aerodynamically unstable + time-varying | 12 | 5 -> 60 |
| [Aircraft Engine Thermal Management](case-study/Aircraft%20Engine%20Thermal%20Management/) | 3-state FTMS intermediate circulation loop [TT, m1, m2], effectiveness-NTU heat exchangers, 60s lumped transport delay, RK4 Ts=0.5s | 12 | 5 -> 60 |

**MATLAB-native study (run by hand, not by `run.py`):**

| Study | Plant | Controllers | Scenarios x Runs |
|---|---|---|---|
| [Boiler Control MATLAB](case-study/Boiler%20Control%20MATLAB/) | Nonlinear Bell-Astrom boiler-turbine, MATLAB twin of the C++ study | 27 | 8 -> 216 |

Built directly on the R2026a toolbox stack (`quadprog` MPC/GPC/NMPC, `mixsyn` H-infinity,
`n4sid` subspace-ID LQG, EKF/UKF, gain scheduling) rather than on `lib/` -- see
[MATLAB/HANDOFF.md](MATLAB/HANDOFF.md) §0 for the decision record. It is fully self-contained
(its own `config/`, `matlab/`, `logs/`) and never writes into the C++ study's `logs/`. Run with
`matlab -batch "addpath('case-study/Boiler Control MATLAB/matlab'); run_all()"`. It is tracked by
hand: `tools/case_study_tracker.py` is scoped to the C++ and Python studies that `run.py` drives,
and skips MATLAB-native studies by design.

Controllers span the full stack: PID, LQR, LQG, MPC, GPC-RLS, SMC, ADRC, Fuzzy-PID,
Smith Predictor, MRAC, H-infinity, TubeMPC, ScenarioMPC, NonlinearMPC, Feedback
Linearisation, EKF-LQR, MHE-LQR, SubspaceID-LQG, L1Adaptive, ILC, NeuralPID,
DynaMBRL, CEM-MPC, Koopman-MPC, ESN, CBF safety filtering, ASMC, and gain-scheduled
(manual, LPV, and automated gap-metric) variants, depending on the plant.
Several spec-only stubs (README/PDF only, no real `sim/` content) remain as outstanding
work -- see [docs/case_study_status.md](docs/case_study_status.md) for the live list and
`CLAUDE.md`'s "Spec-only / placeholder stubs" section for per-study notes. A few directories
scaffolded by `tools/new_case_study.py` still contain only placeholder dynamics and a single
`OpenLoop` controller; `tools/case_study_tracker.py` reports these as "Open placeholder"
(distinct from "On-going", which means real plant+controller code) -- check the study's own
`README.md` before treating any study as complete.

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
| **ML / Data-driven** | GaussianProcess (SE kernel, Cholesky, fixed-budget), EchoStateNetwork (spectral-radius reservoir), NeuralPID (3->n_h->3 online backprop), CEMController (elite-sample stochastic MPC), DynaController (Sutton Dyna MBRL + SINDy model), ILC (P-type / D-type / norm-optimal), DeePC (ADMM data-enabled predictive control), ScenarioMPC, BayesianOptimizer, GeneticAlgorithm (BLX-alpha crossover + elitism), ParticleSwarmOptimizer (Clerc-Kennedy), DifferentialEvolution (DE/rand/1/bin) |
| **Model Estimation** | GreyBoxEstimator (Levenberg-Marquardt ODE param fit, RK4 sensitivity), RecursiveGreyBoxEstimator (augmented-state UKF, online), GPResidualModel (learn model-plant mismatch as GP), MismatchDetector (CUSUM on KF/MHE innovation) |
| **Model utilities** | LinearisationHelper (jacobianX/U, lineariseAtPoint ZOH), BalancedTruncation (Hinf bound), ZeroPhaseTrackingFilter (ZPETC + transmissionZeros), GapMetric (chordal SISO + subspace MIMO), HybridModel + HybridMPC + HybridModelTrainer (physical ODE + data correction) |
| **DAE utilities** | `DAESystem` (Index-1 semi-explicit, `f`/`g`/`h` functors), `consistentInit` (Newton-Raphson), `dae2ode` (Euler+Newton discrete step function), `c2d(DAESystem)` (algebraic elimination + ZOH/Tustin), `setAlgebraicConstraint` on EKF (post-update Newton projection) |
| **Embedded subset** | `BasicPID<Scalar>`, `BasicSMC<Scalar>` (header-only, no Eigen, no virtual dispatch, MCU-safe); `DiscreteIntegrator`, `FixedRateFilter`, `RingBuffer` in `lib/embedded/` |
| **ROS 2 adapter** | `ctrl_toolbox_ros2::ControllerNode<T>` - lifecycle node wrapping any `ctrl::IController`; topics `~/setpoint`, `~/measurement`, `~/control_output`; factory pattern for param-driven construction |

---

## Tested Compilers

| Compiler | Version | Status |
|---|---|---|
| GCC | 9, 11, 12, 13 | OK |
| Clang | 10, 14, 16 | OK |
| MSVC | 19.20+ (VS 2019/2022) | OK |

---

## Project Status

**Baseline (Part 63 - 2026-06-18, UNVERIFIED until next clean `run.py`):**
C++ case studies: Boiler 216/216 . Tug 72/72 . Solar 70/70 . Humidification 75/75 .
ActiveSuspension 90/90 (18 ctrl) . BuckBoost 60/60 . SolarCooker 60/60 . SOTEC 60/60 . SMISMO 65/65 (13 ctrl) .
Stewart 720/720 (12 ctrl x 60 sea-state configs).
Python-only (Phase 7): DrillString 85/85 . WindWave 80/80 . EHFS 70/70 (14 ctrl) .
Firefighting 60/60 . BTMS 60/60 . SurfaceShip 60/60 . EV6x6 90/90 (18 ctrl) .
AircraftEngine 60/60 (12 ctrl).
`bug_report.txt`: 0 blocks expected after a clean run.

**Part 63 (2026-06-18) - Aircraft Engine Thermal Management promoted into the official roster:**
- `case-study/Aircraft Engine Thermal Management/` was already fully implemented (12
  controllers x 5 scenarios, real FTMS intermediate-circulation-loop plant) but had never
  been added to this README's tables or counts -- discovered via a `tools/case_study_tracker.py`
  rewrite (Part 62) that re-scanned every `case-study/*/` directory. No code changes were
  needed: it's a pure Python-only study already auto-discovered by `run.py` Phase 7.
- Study roster (12): OpenLoop, PID, ADRC, SMC, LQR, MPC, MRAC, L1Adaptive, GainScheduled,
  SmithPredictor, NeuralPID, ILC. Negative-static-gain plant (mirrors the documented Solar
  Cooker sign-convention precedent) -- see the study's own README "Implementation Notes".

**Part 61 (2026-06-18) - 6-DOF Stewart Platform Vessel Motion Simulator (C++ case study):**
- `stewart_sim` - closed-form inverse kinematics + velocity Jacobian for a 6-UPU hinge
  layout; 12-state (6 rods) spring-mass-damper actuator dynamics, RK4 at Ts=5ms.
- Standalone CFD-input stand-in (`cfd_input_model.{h,cpp}`), architecturally separated
  from the plant: 10 Douglas Sea Scale states x 3 wave directions x 2 swell flags = 60
  configs built at runtime, calibrated against the source paper's 3 actual (Hs,T)/(Hs,
  amplitude) data points, with workspace-margin scaling to keep every trajectory within
  the paper's Table 1 limits. 12 controllers x 60 configs = 720 runs.
- Fixed a `lib/NeuralPID.cpp` overflow bug (naive softplus instead of the existing
  numerically-stable helper) surfaced by this case study's larger gain seeds.

**Part 60 (2026-06-16) - DIST-3 ROS 2 wrapper + CI/CD overhaul + cross-platform run.py:**
- `ros2/ctrl_toolbox_ros2/` - ROS 2 Humble `ament_cmake` package; `ControllerNode<T>` lifecycle node template wrapping any `ctrl::IController`.
- CI/CD consolidated from 8 -> 3 workflow files (`documentation.yml`, `benchmarks.yml`, `cross-platform-cicd.yml`). Tag-triggered release + PyPI publish jobs folded into the unified workflow.
- `tests/test_embedded_subset.cpp` API aligned with Part 54 (`BasicPID`/`BasicSMC` single-arg constructor, `Ts` in Params, `sp.K` not `sp.eta`).
- `run.py` cross-platform: Phase 3 dispatches `compile.sh` (bash) on Linux/macOS; Phase 4 cmake via `conda run`; Phase 5 finds executables by no-extension + executable-bit on Linux.

**Part 55 (2026-06-13) - GA/PSO/DE + controller gap audit (C3-C6):**
- `GeneticAlgorithm`, `ParticleSwarmOptimizer`, `DifferentialEvolution` added to `lib/` + Python bindings.
- Active Suspension 2-DOF expanded 15 -> 18 controllers (+ GAOptPID, PSOOptPID, DEOptPID); 90 runs.
- SMISMO expanded 12 -> 13 controllers (+ DOBEnergyCtrl with dynamic supply pressure); 65 runs.
- EHFS expanded 12 -> 14 controllers (+ HinfODFCCtrl, HinfCascadeCtrl); 70 runs.
- Active Suspension 6x6 EV Full Model added as a new Python-only study: 40-state 20-DOF, 18 controllers, 90 runs.

Details in [docs/cumulative_bug_report.md](docs/cumulative_bug_report.md). Real-time deployment guidance in [docs/deployment.md](docs/deployment.md).
