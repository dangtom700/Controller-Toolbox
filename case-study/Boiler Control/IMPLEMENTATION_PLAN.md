# Boiler Control - Full Numerical Simulation: Implementation Plan

> **Goal:** Restructure the Boiler Control case study into a modular multi-file simulation
> matching the Tug Boat Numerical Simulation architecture, and expand it to exercise
> every controller design available in Controller Toolbox.

---

## 1. Background - What Already Exists

The current `boiler_turbine_case_study.cpp` is a single monolithic file (~830 lines) that:

- Implements the Bell & Astrom (1987) nonlinear boiler-turbine plant  
- Linearises around 3 operating points (Low / Medium / High Load)  
- Runs 6 controller experiments per operating point: LQR, MPC, LQG+Kalman, PID, SMC, ESC  

**What is missing relative to the full toolbox and the tug boat sim pattern:**

| Toolbox component | Current status |
|---|---|
| `DiscretePID` | ✅ (decentralised SISO, fixed gains) |
| `DiscreteLQR` | ✅ |
| `DiscreteLQG` / `KalmanFilter` | ✅ |
| `DiscreteMPC` | ✅ |
| `DiscreteSMC` | ✅ |
| `ExtremumSeeker` | ✅ |
| `DiscreteADRC` | ❌ missing |
| `DiscreteLeadLag` | ❌ missing |
| `SmithPredictor` | ❌ missing |
| `GeneralizedPredictiveControl` (GPC) | ❌ missing |
| `ExtendedKalmanFilter` (EKF) | ❌ missing |
| `UnscentedKalmanFilter` (UKF) | ❌ missing |
| `RecursiveLeastSquares` (RLS) | ❌ missing |
| `ControllerStack` (Supervisory / Additive / Weighted) | ❌ missing |
| `FuzzyPID` / `FuzzySupervisor` | ❌ missing |
| `RepetitiveController` | ❌ missing |
| `SubspaceID` (offline MIMO system ID) | ❌ missing |
| Modular multi-file architecture | ❌ monolithic |
| JSON scenario / config files | ❌ hardcoded |
| Structured telemetry / CSV logging | ❌ ad-hoc per function |
| Load-tracking (setpoint changes, not just regulation) | ❌ only perturbation rejection |

---

## 2. Target Architecture

```
case-study/Boiler Control/
|-- CMakeLists.txt                   (updated - links controller_toolbox)
|-- IMPLEMENTATION_PLAN.md           (this file)
|-- config/
|   |-- plant_params.json            (all physical constants, Ts, valve bounds)
|   |-- scenarios/
|       |-- s01_lowload_regulation.json
|       |-- s02_medload_regulation.json
|       |-- s03_highload_regulation.json
|       |-- s04_lowload_loadstep.json
|       |-- s05_medload_loadstep.json
|       |-- s06_highload_loadstep.json
|       |-- s07_multiop_transition.json
|-- logs/                            (auto-created at runtime)
|-- sim/
|   |-- include/
|   |   |-- boiler_plant.h           (BoilerTurbine class + operating points)
|   |   |-- linearizer.h             (linearize() + LinearStateSpace struct)
|   |   |-- controllers.h            (ControllerBase + all controller declarations)
|   |   |-- simulation_runner.h      (ScenarioConfig + runSimulation())
|   |   |-- telemetry_logger.h       (BoilerLogger - CSV + metrics)
|   |-- src/
|       |-- boiler_plant.cpp
|       |-- linearizer.cpp
|       |-- controllers.cpp          (all controller implementations)
|       |-- simulation_runner.cpp
|       |-- telemetry_logger.cpp
|       |-- main.cpp
```

This mirrors the tug boat layout exactly:  
`plant_parameters` -> `boiler_plant` + `linearizer`  
`environment` -> (no equivalent; disturbances are scenario-driven)  
`physics_plant` -> `boiler_plant` (nonlinear ODE stepper)  
`controllers` -> `controllers`  
`simulation_runner` -> `simulation_runner`  
`telemetry_logger` -> `telemetry_logger`

---

## 3. Plant Model (`boiler_plant`)

Retain the existing Bell-Astrom nonlinear ODE exactly as-is:

```
States  x = [x1, x2, x3]
         x1 = drum pressure   [bar]
         x2 = electric power  [MW]
         x3 = water level deviation [cm]

Inputs  u = [u1, u2, u3]
         u1 = fuel flow valve    [0, 1]
         u2 = steam control valve [0, 1]
         u3 = feedwater valve     [0, 1]

Outputs y = [y1, y2, y3]
         y1 = drum pressure  (= x1)
         y2 = electric power (= x2)
         y3 = boiler efficiency proxy (nonlinear function of x, u)
```

Valve rate limits: |Deltau1| <= 0.007/step, |Deltau2| <= 0.02/step, |Deltau3| <= 0.05/step.

**Three operating points** (unchanged):

| Label | x1 [bar] | x2 [MW] | x3 [cm] | u1    | u2    | u3    |
|-------|----------|---------|---------|-------|-------|-------|
| A (Low Load)  | 75.6 | 15.3 | 508.97 | 0.119 | 0.381 | 0.123 |
| B (Med Load)  | 97.2 | 50.5 | 469.51 | 0.270 | 0.621 | 0.340 |
| C (High Load) | 140  | 128  | 323.68 | 0.596 | 0.894 | 0.788 |

---

## 4. Lineariser (`linearizer`)

The Jacobian linearisation already in the code is correct. Two additions:

1. **ZOH discretisation** - replace the current Euler forward (`Ad = I + Ts.Ac`) with
   `ctrl::c2d(plant_c, Ts, ctrl::C2dMethod::ZOH)` via the toolbox for accuracy.
2. **Gain matrix for output feedback** - compute `Nbar` feedforward matrix
   for setpoint tracking experiments (not just regulation).

Returns a `LinearStateSpace` holding `ctrl::StateSpace` (double, not float).

---

## 5. Scenario System (`config/scenarios/*.json`)

Each scenario JSON specifies:

```json
{
  "id":            "s01_lowload_regulation",
  "description":   "Perturbation rejection at Low Load operating point",
  "operating_point": "A",
  "mode":          "regulation",
  "dx0":           [5.0, 3.0, -10.0],
  "setpoint_y":    [0.0, 0.0, 0.0],
  "duration_s":    3600,
  "Ts":            1.0
}
```

For load-step scenarios:

```json
{
  "id":          "s04_lowload_loadstep",
  "description": "Step demand: Low Load -> 120% rated power",
  "operating_point": "A",
  "mode":        "tracking",
  "setpoint_y":  [10.0, 20.0, 0.0],
  "duration_s":  3600,
  "Ts":          1.0
}
```

Seven scenarios total:
- s01-s03: perturbation regulation at each operating point (same Deltax0 as existing code)
- s04-s06: load-step tracking at each operating point
- s07: multi-operating-point transition (A -> B midway through simulation)

---

## 6. Controller Suite

All controllers share this interface (mirroring tug boat pattern):

```cpp
class ControllerBase {
public:
    virtual Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                                    const Eigen::Vector3d& y) = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;
};
```

`ref` = desired output deviation `[dy1, dy2, dy3]`  
`y`   = measured output deviation `[y1-y1_op, y2-y2_op, y3-y3_op]`  
Output = control increment `du = [du1, du2, du3]`; absolute valve = `u0 + du`, clamped to [0,1].

---

### 6.1 PID - Decentralised SISO (existing, kept)

Three independent `ctrl::DiscretePID` loops: `e_y1 -> du1`, `e_y2 -> du2`, `e_y3 -> du3`.

**Tuning:** `StepResponseTuner::identify()` on each diagonal channel of the linearised model,
then `StepResponseTuner::computePIDParams(..., PIDTuningRule::IMC)`.  
Anti-windup: `Kb = 1/Ti`.

---

### 6.2 LQR - Full-State Feedback (existing, refactored)

`ctrl::DiscreteLQR` with Bryson weights:  
`xmax = [5, 10, 1]` (pressure, power, level),  
`umax = [0.3, 0.3, 0.1]` (valve deviations).

For tracking scenarios: compute feedforward gain  
`Nbar = -(C.(A_cl - I)^-^1.B)^-^1` to eliminate steady-state error.

---

### 6.3 MPC - Condensed MIMO QP (existing, refactored)

`ctrl::DiscreteMPC` with `computeRef()` for tracking.

Horizon selection: `ctrl::MPCHorizonTuner::recommend()`, capped at Np=20, Nc=5
(integrating water-level mode would otherwise request Np=5000).

Separate `uMin`/`uMax` = +/-0.5 (valve increment bounds), with absolute valve clamped
after each step.

---

### 6.4 LQG - LQR + Linear Kalman Filter (existing, refactored)

`ctrl::DiscreteLQG` on the nonlinear plant with measurement noise:

| Channel | sigma_meas |
|---------|--------|
| y1 (pressure) | 0.5 bar |
| y2 (power)    | 1.0 MW  |
| y3 (efficiency) | 5.0 % |

Process noise covariance Q = 1e-4 . I.

---

### 6.5 SMC - Decentralised Sliding Mode (existing, refactored)

Three `ctrl::DiscreteSMC` loops. Parameters from existing code:
`c_e=1.0, c_de=0.2, K=0.05, phi=0.3`.

For tracking scenarios, reference is the non-zero setpoint deviation.

---

### 6.6 ESC - Extremum Seeking on Efficiency (existing, refactored)

`ctrl::ExtremumSeeker` on `u3` to maximise `y3` (efficiency proxy) while
`u1, u2` are regulated by PID to hold `y1, y2` at setpoint.

Parameters: `perturbAmp=0.005`, `perturbFreq=0.02 Hz`, `seekMinimum=false`.

---

### 6.7 ADRC - Active Disturbance Rejection Control (new)

Three `ctrl::DiscreteADRC` loops (one per output axis).

ADRC treats cross-coupling between the three boiler channels as "total disturbance"
observed by the Extended State Observer (ESO). This makes it naturally robust to the
nonlinear coupling terms without needing a full plant model.

**Tuning per axis:**

| Axis | omega_o (ESO BW) | omega_c (ctrl BW) | b0 (input gain estimate) |
|------|-------------|--------------|--------------------------|
| y1 (pressure) | 0.10 rad/s | 0.02 rad/s | 0.9 (Bc[0,0]) |
| y2 (power)    | 0.05 rad/s | 0.01 rad/s | 0.073.x1^(9/8) |
| y3 (efficiency) | 0.05 rad/s | 0.01 rad/s | Dc[2,2] |

b0 estimated from the diagonal of Bc and Dc at each operating point.

**Why ADRC here:** The boiler is open-loop unstable in some regimes and has significant
load-dependent gain variation - exactly the scenario ADRC was designed for.

---

### 6.8 Lead-Lag Compensator (new)

One `ctrl::DiscreteLeadLag` per output channel, applied as pre-filter to PID
(Lead-Lag + PID cascade).

**Design procedure:**  
1. Identify FOPDT from linearised diagonal channel step response.  
2. Compute crossover frequency omega_c for 45^\circ phase margin.  
3. `ctrl::LoopShapingTuner::tuneImpl()` -> `ctrl::LeadLagParams`.  
4. Cascade output into a proportional gain (no integral; integral is added by the
   downstream PID to handle steady-state).

This demonstrates frequency-domain loop-shaping as an alternative to pole-placement.

---

### 6.9 Smith Predictor (new)

The boiler-turbine exhibits a significant computation/transport delay through the
steam path. Model the identified effective delay as `d_steps = round(theta / Ts)`
using FOPDT theta from step response.

`ctrl::SmithPredictor` wrapping a `ctrl::DiscretePID` inner loop:

```cpp
auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
ctrl::SmithPredictor sp(inner, model_ss, d_steps);
```

Applied per output axis. Particularly relevant for y3 (efficiency) which has the
longest effective response time.

---

### 6.10 Generalized Predictive Control (GPC) (new)

`ctrl::GeneralizedPredictiveControl` - a self-tuning GPC that uses `ctrl::RecursiveLeastSquares`
to continuously estimate an ARX model online from closed-loop I/O data, then recomputes
the GPC control law each step.

This is the only controller in the suite that performs **online system identification**.

**Configuration:**

```
ARX order:   na=2, nb=2, nc=1   (2nd-order MISO per channel)
Prediction horizon:  Ny=10
Control horizon:     Nu=3
Forgetting factor:   lambda=0.98      (tracks slow operating point drift)
```

Why it matters: as the boiler transitions between load regimes (s07 scenario), the ARX model
automatically adapts, avoiding the need to pre-specify operating-point-dependent gains.

---

### 6.11 Extended Kalman Filter + LQR (EKF-LQR) (new)

Replace the linear Kalman filter in the LQG design with `ctrl::ExtendedKalmanFilter`.

The EKF linearises the **nonlinear** Bell-Astrom plant at each step using the
analytical Jacobian from `linearize()` (already implemented), rather than relying on
a fixed linearisation at a single operating point.

State dimension: 3 (x1, x2, x3).  
Measurement: y = [x1, x2, y3(x,u)] - full-state + nonlinear efficiency output.

The EKF-LQR pairing delivers near-optimal regulation even during large transients where
the linear KF degrades.

---

### 6.12 Unscented Kalman Filter + LQR (UKF-LQR) (new)

`ctrl::UnscentedKalmanFilter` - sigma-point propagation through the nonlinear plant.
No Jacobian required. Useful for validating EKF accuracy and for regimes where the
linearised Jacobian is ill-conditioned (near operating point C where x1=140 bar pushes
against model validity).

Same LQR outer loop as LQG. Noise parameters identical to LQG for fair comparison.

---

### 6.13 Fuzzy PID (new)

Three `ctrl::FuzzyPID` loops (one per output channel).

Each `FuzzyPID` runs the toolbox's built-in 25-rule Mamdani PD block with an external
integral accumulator and back-calculation anti-windup.

**Scaling parameters per axis:**

| Axis | e_scale | de_scale | u_scale |
|------|---------|----------|---------|
| y1 (pressure, bar) | 10.0 | 2.0 | 0.3 |
| y2 (power, MW)     | 20.0 | 5.0 | 0.3 |
| y3 (efficiency, %) |  0.05| 0.01| 0.1 |

Anti-windup: `Kb = 0.8` (same as DC motor example).

**Motivation:** The boiler nonlinearity means PD gain requirements differ between
small-error regulation and large-error recovery. The fuzzy surface provides smooth
gain-scheduling without explicit lookup tables.

---

### 6.14 Fuzzy Supervisor + MPC (new)

`ctrl::FuzzySupervisor` monitors the output error magnitude and its trend.
When it fires, it triggers re-linearisation of the MPC internal model at the current
operating state, exactly mirroring the tug boat `FuzzySupervised_MPC`.

**Supervisor parameters:**

```
e_threshold      = 5.0    (5 bar / 5 MW / 0.05 efficiency)
trend_threshold  = 0.5    (divergence rate)
signal_threshold = 0.5
cooldown_steps   = 120    (120 s between re-triggers)
```

At re-linearisation: call `linearize(current_op, Ts)` using the instantaneous
`(x1, x2, x3)` as the operating point, push to each `ctrl::DiscreteMPC::setPlant()`.

**What this tests:** Whether on-demand re-linearisation outperforms the fixed-model
MPC during the `s07_multiop_transition` scenario.

---

### 6.15 Supervisory Stack - SMC -> LQR (new)

`ctrl::ControllerStack` in `StackMode::Supervisory`:

- **Large error** (||e|| > 5): `ctrl::DiscreteSMC` handles fast nonlinear regime.
- **Small error** (||e|| <= 5): `ctrl::DiscreteLQR` takes over for near-optimal regulation.

Activation condition per axis independently evaluated.  
Demonstrates bumpless transfer via `bumplessInit()`.

---

### 6.16 Additive Stack - PID + Lead-Lag (new)

`ctrl::ControllerStack` in `StackMode::Additive`:

- Base: `ctrl::DiscretePID` (steady-state tracking, Ki != 0)
- Supplement: `ctrl::DiscreteLeadLag` (transient phase kick)

Weight of Lead-Lag component faded from 1.0 -> 0.0 over the first 300 seconds
(transient only).

---

### 6.17 Weighted Stack - Load-Dependent Blending (new)

`ctrl::ControllerStack` in `StackMode::Weighted`:

Two controllers with weights that depend on current drum pressure x1:
- `w_pid = (x1 - 75) / 65`   (PID dominates at high load)
- `w_lqr = 1 - w_pid`        (LQR dominates at low load)

Weight is updated each step, demonstrating live gain scheduling via the Weighted stack.

---

### 6.18 Repetitive Controller (new)

`ctrl::RepetitiveController` wrapping a `ctrl::DiscretePID` inner loop.

Relevant for the s07 scenario where the load profile follows a periodic
demand cycle (once the transition is complete). The repetitive layer learns
the period-specific feedforward correction, reducing IAE below pure PID.

Period: 600 steps (10 minutes at Ts=1 s).

---

## 7. Telemetry Logger (`telemetry_logger`)

`BoilerLogger` mirrors `TelemetryLogger` from the tug sim:

```cpp
struct BoilerTickData {
    double   t;
    double   y1, y2, y3;       // measured outputs (absolute)
    double   u1, u2, u3;       // applied valve positions (absolute)
    double   du1, du2, du3;    // valve increments
    double   ref_y1, ref_y2, ref_y3;  // setpoints (absolute)
    double   e1, e2, e3;       // tracking errors
};
```

Per-run CSV filename: `logs/run_<scenario_id>_<controller_name>.csv`

Accumulated metrics (computed on flush):

| Metric | Formula |
|--------|---------|
| IAE_y1/y2/y3 | \int|e_i| dt |
| ISE_y1/y2/y3 | \inte_i^2 dt |
| E_valve | Sigma(du1^2+du2^2+du3^2).Ts (control effort) |
| max_overshoot | max(y_i - ref_i) per channel |

Printed to stdout at end of each run (same one-liner format as tug sim).

---

## 8. Simulation Runner (`simulation_runner`)

```cpp
void runSimulation(const ScenarioConfig& scenario,
                   ControllerBase& controller,
                   const std::string& log_dir);
```

Loop structure:

```
for k = 0..N_steps:
    y  = bt.measureOutputs()            // nonlinear plant outputs
    dy = y - y_op                       // deviation from operating point
    du = controller.compute(ref_dy, dy) // controller in deviation space
    u  = clamp(u_op + du, 0, 1)        // absolute valve, clamped
    bt.applyValveRateLimits(u)         // |Deltau| per step constraint
    bt.update()                         // nonlinear ODE step
    logger.log(...)
logger.flush()
print metrics
```

For `mode == "tracking"`: `ref_dy = scenario.setpoint_y` (non-zero constant).  
For `mode == "regulation"`: `ref_dy = [0,0,0]`, initial state perturbed by `dx0`.

---

## 9. Main Entry Point (`main.cpp`)

```
main [base_dir]
|-- Load plant_params.json
|-- Enumerate scenarios/ -> sorted vector
|-- Build controller vector (one instance each):
|   PID, LQR, MPC, LQG, SMC, ESC,
|   ADRC, LeadLag+PID, SmithPredictor,
|   GPC+RLS, EKF-LQR, UKF-LQR,
|   FuzzyPID, FuzzySup-MPC,
|   SupervisoryStack, AdditiveStack, WeightedStack,
|   RepetitiveController
|-- Run all (scenario * controller) pairs
|-- Print summary table
```

Total matrix: 7 scenarios * 18 controllers = **126 simulation runs**.

---

## 10. CMakeLists.txt (updated)

```cmake
include(FetchContent)
FetchContent_Declare(
    nlohmann_json_single
    URL      https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp
    DOWNLOAD_NO_EXTRACT TRUE
    DOWNLOAD_DIR "${CMAKE_CURRENT_BINARY_DIR}/json_include"
)
FetchContent_MakeAvailable(nlohmann_json_single)

set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

set(SIM_SRCS
    ${SIM_SRC}/boiler_plant.cpp
    ${SIM_SRC}/linearizer.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/simulation_runner.cpp
    ${SIM_SRC}/telemetry_logger.cpp
    ${SIM_SRC}/main.cpp
)

add_executable(boiler_sim ${SIM_SRCS})
target_include_directories(boiler_sim PRIVATE
    ${SIM_INC}
    "${CMAKE_CURRENT_BINARY_DIR}/json_include"
)
target_link_libraries(boiler_sim PRIVATE controller_toolbox)
target_compile_features(boiler_sim PRIVATE cxx_std_17)
```

The old `boiler_turbine_case_study` target is **removed** from the parent
`case-study/CMakeLists.txt` (replaced by the subdirectory's `boiler_sim`).

---

## 11. Implementation Order

| Phase | Files | Notes |
|-------|-------|-------|
| P1 | `boiler_plant.h/.cpp`, `linearizer.h/.cpp` | Port existing classes; switch to ZOH via `c2d()` |
| P2 | `telemetry_logger.h/.cpp` | Straightforward port from tug logger |
| P3 | `simulation_runner.h/.cpp` | Scenario JSON loading + loop |
| P4 | `config/plant_params.json`, `config/scenarios/*.json` | 7 scenario files |
| P5 | `controllers.h/.cpp` - Group A | Port PID, LQR, LQG, MPC, SMC, ESC (existing logic, new wrapper) |
| P6 | `controllers.h/.cpp` - Group B | ADRC, LeadLag+PID, SmithPredictor |
| P7 | `controllers.h/.cpp` - Group C | GPC+RLS, EKF-LQR, UKF-LQR |
| P8 | `controllers.h/.cpp` - Group D | FuzzyPID, FuzzySup-MPC |
| P9 | `controllers.h/.cpp` - Group E | SupervisoryStack, AdditiveStack, WeightedStack, RepetitiveController |
| P10 | `main.cpp`, `CMakeLists.txt` | Wire everything; build + test |

---

## 12. Validation Criteria

Each controller is considered correctly implemented when, on the **regulation scenarios**
at operating point B (medium load), it satisfies:

| Controller | Acceptance criterion |
|------------|---------------------|
| PID | y1 error < 1 bar, y2 error < 2 MW after 600 s |
| LQR / LQG / EKF / UKF | dx -> 0 within 300 s |
| MPC / GPC | dx -> 0 within 200 s; no valve saturation sustained > 30 s |
| SMC | Settling within 400 s; chattering amplitude < 0.01 on du |
| ADRC | Comparable settling to SMC; no manual Jacobian needed |
| ESC | y3 increases or stays within 2% of optimum within 1000 s |
| FuzzyPID | y1, y2 errors < 2* PID IAE |
| Stacks | Active controller switches <= 5 times per run |
| RepetitiveController | IAE in 2nd period < 50% IAE in 1st period |

---

## 13. Key Design Decisions

1. **All controllers work in deviation space** - outputs from the controller are
   `du`, not absolute `u`. Absolute valve is always `u = clamp(u_op + du, 0, 1)`.
   This means the same controller code works for both regulation (ref=0) and tracking
   (ref=setpoint deviation) without modification.

2. **Valve rate limiting is enforced inside `boiler_plant`, not inside controllers.**
   Controllers see the effect of rate-limited inputs on the next measurement but
   don't need to model it themselves.

3. **The nonlinear plant is used for all closed-loop simulations** (not the linearised
   model). The linearised model is only used for controller design (LQR, MPC, LQG weights,
   ADRC b0, Lead-Lag frequency design, Smith Predictor model). This keeps the simulation
   realistic and exposes model-plant mismatch.

4. **ZOH discretisation replaces Euler** in the lineariser. This gives more accurate
   discrete eigenvalues especially for the faster pressure dynamics.

5. **The GPC+RLS controller does not use the linearised model** at all - it identifies
   its own ARX model online from closed-loop data, making it the most self-sufficient
   controller in the suite.
