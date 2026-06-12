# Controller Toolbox — Algorithm Roadmap: Phase 2

**Created:** 2026-06-11 (Part 50 planning)
**Status:** Planning — no code written yet
**Scope:** DAE Representation (P1-P3) → Model Estimation (E1-E4) → Hybrid Models (H1-H4) → Deployment (D1-D2)

---

## Motivation

The A1-A11 data-driven / ML algorithm set is complete (Parts 30-33). The toolbox can now
learn nonlinear dynamics (SINDy, KoopmanEDMD), adapt online (MRAC, L1Adaptive, NeuralPID),
and plan under uncertainty (ScenarioMPC, CEMController). The gap that remains is the bridge
between a physics-based plant model and real plant behaviour:

- **Grey-box estimation** enables users to calibrate their physics models from data, rather
  than treating every parameter as a tuning knob.
- **GP residual models** quantify the mismatch between physics and reality, providing
  uncertainty bounds that risk-aware controllers (MPC, TubeMPC) can act on.
- **Hybrid models** combine physics with learned residuals, giving the interpretability of
  first-principles design with the accuracy of data-driven correction.
- **Digital twin deployment** closes the loop: the model is kept alive with real data,
  mismatch is detected automatically, and re-tuning is suggested without manual inspection.

This is the highest-ROI development path after the current case-study roster.

A prerequisite that unlocks all three phases is **DAE (Differential-Algebraic Equation)
support**. Several existing case studies already have hidden algebraic equations (S-OTEC's
ORC map, SMISMO's valve flows, EHFS pressure-balance equations) that are currently
resolved by direct substitution before the integrator. A proper `DAESystem` representation
preserves the algebraic structure, enabling more natural plant formulation, exact constraint
enforcement in MHE, and a cleaner `GreyBoxEstimator` API.

---

## Dependency Graph

```
P1 (DAESystem + dae2ode)
  │
  ├─► P2 (c2d for DAE)          enables linearise-and-discretise of DAE plants
  └─► P3 (DAE-aware EKF)        algebraic projection after each UKF/EKF update step
  │
  ├─► E1 (GreyBoxEstimator) ───────────────────────────► E3 (GP Residual)
  │     │                                                        │
  │     └─► E2 (RecursiveGreyBox, wraps UKF)                    └─► H4 (HybridModelTrainer)
  │                                                                       │
  ├─► E4 (MHE Inequality Constraints) — independent                       │
  │                                                               H1 (HybridModel base class)
  │                                                                       │
  └─────────────────────────────────────────────────────────► H2 (HybridMPC) ──► H3 (RL-MPC)

D1 (Mismatch Detector) — independent CUSUM extension
D2 (Digital Twin Lite) ──── requires E1 + D1
```

**Recommended implementation order:** P1 → P2 → P3 → E1 → E2 → E4 → E3 → H1 → H2 → D1 → H4 → H3 → D2

---

## Phase 0: DAE Model Representation

**Scope:** Index-1 semi-explicit DAE only. Index ≥ 2 requires the Pantelides algorithm
and is a research-grade effort outside this project's focus. Index-1 covers the overwhelming
majority of control-relevant systems (constrained mechanisms, power systems, process plants
with equilibrium stages, electrical circuits).

**Semi-explicit Index-1 form:**
```
x1' = f(x1, x2, u)    ← differential states (integrated by RK4 / Euler)
 0  = g(x1, x2, u)    ← algebraic constraints (solved at each step, not integrated)
 y  = h(x1, x2, u)    ← outputs
```

---

### P1 — DAESystem Struct + `dae2ode()` Converter

**Goal:** Add a `DAESystem` data structure to `PlantModel.h` and a `dae2ode()` function
that reduces it to a standard ODE by eliminating `x2` via a Newton solve on `g`. The
resulting ODE can then be used with every existing integrator and controller unchanged.

**New struct in `PlantModel.h`:**
```cpp
struct DAESystem {
    // x1' = f(x1, x2, u)
    using DiffFunc  = std::function<VectorXd(const VectorXd& x1,
                                             const VectorXd& x2,
                                             double u)>;
    // 0 = g(x1, x2, u)
    using AlgFunc   = std::function<VectorXd(const VectorXd& x1,
                                             const VectorXd& x2,
                                             double u)>;
    // y = h(x1, x2, u)
    using OutputFunc = std::function<VectorXd(const VectorXd& x1,
                                              const VectorXd& x2,
                                              double u)>;

    DiffFunc   f;         // differential equations
    AlgFunc    g;         // algebraic constraints
    OutputFunc h;         // output map
    int        n_diff;    // number of differential states
    int        n_alg;     // number of algebraic states
    double     Ts;
};
```

**`dae2ode()` converter:**
```cpp
// Returns a StateFunc (x' = F(x_aug, u)) usable by any RK4/Euler integrator.
// x_aug = [x1; x2]; x2 is updated by Newton solve on g at each step.
std::function<VectorXd(const VectorXd&, double)>
dae2ode(const DAESystem& dae,
        int newton_max_iter = 20,
        double newton_tol   = 1e-9);
```

The Newton solve at each step: given `x1[k+1]` (after integrating `f`), find `x2[k+1]`
such that `g(x1[k+1], x2[k+1], u) = 0` via Newton-Raphson with `LinearisationHelper`
numerical Jacobian `∂g/∂x2`.

**Consistent initialisation helper:**
```cpp
// Finds x2_init such that g(x1_init, x2_init, u0) = 0.
VectorXd consistentInit(const DAESystem& dae,
                        const VectorXd& x1_init,
                        double u0,
                        const VectorXd& x2_guess);
```

**Reused components:**
- `PlantModel.h` — add `DAESystem` struct and free functions alongside existing `StateSpace`
- `LinearisationHelper::jacobianX` — provides `∂g/∂x2` for Newton solve (no new dependency)

**Effort estimate:** ~250 lines (struct + Newton + consistent init + binding + 3 tests)

**Example use case:** CSTR reactor with fast/slow dynamics. Algebraic constraint: mass
balance `g = F_in*C_in - F_out*C - r(C, T) = 0` (equilibrium assumption for a fast species).
`dae2ode()` eliminates `C` algebraically, handing a clean ODE to the MPC prediction step.

**Case studies that benefit immediately:**
- S-OTEC: ORC algebraic map (`P_inlet`, `W_net`, `eta_th`) currently substituted manually
- SMISMO: valve flow algebraic equations currently inlined
- EHFS: pressure-balance algebraic equations currently inlined

**Catch2 test plan (`[dae_system]`):**
1. Index-1 DAE with known analytic solution — `dae2ode()` trajectory matches ODE solution within Newton tolerance
2. `consistentInit()` — residual `‖g(x1, x2, u)‖ < tol` at the returned `x2`
3. Singular or near-singular `∂g/∂x2` — Newton fails gracefully (return last iterate, set flag)

---

### P2 — `c2d()` Overload for DAE → Discrete-Time StateSpace

**Goal:** Extend `c2d()` to accept a `DAESystem`, linearise it at an operating point,
eliminate the algebraic states analytically, and return a discrete-time `StateSpace`.
This is the natural path for any controller that needs a linear prediction model (MPC, LQR, LQG).

**New overload:**
```cpp
// Linearise DAE at (x1_op, x2_op, u_op), eliminate x2, then c2d ZOH/Tustin.
StateSpace c2d(const DAESystem& dae,
               const VectorXd& x1_op,
               const VectorXd& x2_op,
               double u_op,
               double Ts,
               C2dMethod method = C2dMethod::ZOH);
```

**Linearisation steps:**
1. Compute Jacobians `A11 = ∂f/∂x1`, `A12 = ∂f/∂x2`, `G1 = ∂g/∂x1`, `G2 = ∂g/∂x2`
   via `LinearisationHelper` central differences.
2. Eliminate `x2` analytically: `δx2 = -G2⁻¹ G1 δx1` (Index-1 assumption: `G2` invertible).
3. Reduced ODE: `A_red = A11 - A12 G2⁻¹ G1`, `B_red = B1 - A12 G2⁻¹ B2`.
4. Apply existing `c2d(StateSpace, Ts, method)` to `(A_red, B_red, C_red, D_red)`.

**Reused components:** `LinearisationHelper`; existing `c2d(StateSpace, ...)` for the final ZOH/Tustin step.

**Effort estimate:** ~150 lines (linearise + eliminate + call existing c2d + 2 tests)

**Catch2 test plan (`[dae_c2d]`):**
1. DAE with known linear reduction — discrete eigenvalues match manual calculation
2. Non-invertible `G2` — throws `std::runtime_error("DAE index > 1 at operating point")`

---

### P3 — DAE-Aware EKF / UKF Extension

**Goal:** Extend `ExtendedKalmanFilter` (and optionally `UnscentedKalmanFilter`) with an
algebraic projection step. After each predict/update cycle, the algebraic states `x2` are
re-solved from `g(x1, x2, u) = 0` so they remain consistent with the differential states
even when noise moves them off the constraint manifold.

**New method on `ExtendedKalmanFilter`:**
```cpp
// Attach a DAE algebraic constraint for post-update projection.
// After every update(), x2 block of the state is re-solved via Newton.
void setAlgebraicConstraint(DAESystem::AlgFunc g,
                            int n_diff, int n_alg,
                            double newton_tol = 1e-9);
```

**Implementation:** Override the post-update step to call `consistentInit()` on the
`x2` block using the updated `x1` estimate. Covariance projection: `P = J_proj * P * J_proj'`
where `J_proj = [I; -G2⁻¹ G1]` is the constraint Jacobian.

**Reused components:** `ExtendedKalmanFilter` (extend, not replace); `consistentInit()` from P1.

**Effort estimate:** ~150 lines (+ 2 tests + binding update)

**Example use case:** State estimation for an EHFS hydraulic cylinder where pressure states
must satisfy the algebraic continuity equation at every step. Without projection, the KF
estimate drifts off the pressure-balance manifold within ~10 steps.

**Catch2 test plan (`[dae_ekf]`):**
1. DAE system with known steady state — estimate converges and `‖g(x1, x2, u)‖ < tol` holds after each update
2. No constraint set (default) — EKF behaves identically to existing EKF (no regression)

---

## Phase 1: Model Estimation

### E1 — GreyBoxEstimator

**Goal:** Non-linear least-squares parameter estimation for a user-supplied ODE `f(x, u, p)`.
The user provides the dynamics and data; the estimator finds `p` that minimises the
sum-of-squared prediction errors over the data horizon.

**Class signature (sketch):**
```cpp
struct GreyBoxParams {
    VectorXd p0;            // initial parameter guess
    VectorXd p_lb, p_ub;    // bounds
    int max_iter = 50;
    double tol = 1e-6;
    double lambda = 1e-3;   // Levenberg-Marquardt damping
};

struct GreyBoxResult {
    VectorXd p_opt;
    double cost;
    int iters;
    bool converged;
};

class GreyBoxEstimator {
public:
    using DynamicsFunc = std::function<VectorXd(const VectorXd& x,
                                                double u,
                                                const VectorXd& p)>;
    GreyBoxEstimator(DynamicsFunc f, double Ts, const GreyBoxParams& params);
    GreyBoxResult estimate(const MatrixXd& X,  // (n x N) state trajectory
                           const VectorXd& U,  // (N-1) input sequence
                           const VectorXd& p0);
};
```

**Reused components:**
- `AutoTuner` cost-function pattern (`CostFn = std::function<double(VectorXd)>`)
- `LinearisationHelper::jacobianX` for numerical sensitivity `∂f/∂p` (central differences)

**Effort estimate:** ~300 lines (+ ~150 lines test + ~100 lines Python binding)

**Example use case:** Estimate thermal resistance R and capacitance C of a building wall
from 24 hours of temperature + heater power data. One-liner: `estimator.estimate(X, U, p0)`.

**Catch2 test plan (`[greybox_estimator]`):**
1. Scalar first-order system `x_dot = -a*x + b*u`; inject known `[a, b]`, add noise, verify recovery within 1%
2. Bounded parameters: p_lb violated in initialisation → clamped correctly
3. Convergence flag: non-identifiable system (constant input) → `converged=false` or large cost

---

### E2 — RecursiveGreyBoxEstimator

**Goal:** Online parameter tracking via augmented-state UKF. Augments the plant state `x`
with parameters `p` as slowly-varying states (`dp/dt ≈ 0` + small process noise `Q_p`).
Handles slowly drifting parameters (friction, thermal resistance over lifetime).

**Class signature (sketch):**
```cpp
class RecursiveGreyBoxEstimator {
public:
    using DynamicsFunc = std::function<VectorXd(const VectorXd& x_aug,
                                                double u)>;
    RecursiveGreyBoxEstimator(DynamicsFunc f_aug, double Ts,
                               int n_states, int n_params,
                               const MatrixXd& Q, const MatrixXd& R);
    void update(double y, double u);
    VectorXd stateEstimate() const;
    VectorXd paramEstimate() const;
};
```

**Helper:** `augmentStateAndParams(f, n_states, n_params)` — wraps a standard
`f(x, u, p)` into an augmented `f_aug([x; p], u)` ready for UKF propagation.

**Reused components:** `UnscentedKalmanFilter` (existing); augmentation is a wrapper.

**Effort estimate:** ~150 lines (mostly boilerplate around UKF constructor)

**Example use case:** Motor friction coefficient `mu_k` drifts with temperature and wear.
Online estimator tracks `[omega, mu_k]` simultaneously without stopping the motor.

**Catch2 test plan (`[recursive_greybox]`):**
1. Step in parameter (friction increase at t=5s) — estimator converges within 3 time constants
2. Augmented state has correct dimension `n_states + n_params`

---

### E3 — GP Residual Model (extend GaussianProcess)

**Goal:** Learn the model-plant mismatch `ε(t) = y_true(t) − y_model(t)` as a Gaussian
Process indexed by `(x, u)`. The trained residual provides a predictive mean correction
and variance (uncertainty) usable as a constraint slack in `NonlinearMPC` / `TubeMPC`.

**New methods on `GaussianProcess` (sketch):**
```cpp
// Fits residual GP: inputs = [x; u] flattened, targets = y_true - y_model
void residualFit(const MatrixXd& XU_data, const VectorXd& residuals);

struct GPPrediction {
    double mean;
    double variance;
};
GPPrediction predictWithUncertainty(const VectorXd& xu) const;
```

**Reused components:** Existing `GaussianProcess` SE kernel, Cholesky solver, FIFO budget.
The change is purely additive (new public methods; no breaking changes).

**Effort estimate:** ~200 lines (new methods + updated binding + 2 new tests)

**Example use case:** CSTR reactor where first-principles model captures 80% of behaviour.
GP learns the remaining 20% (unknown reaction term). MPC uses `predictWithUncertainty` to
tighten constraints in high-variance regions and loosen them where the model is confident.

**Catch2 test plan (`[gp_residual]`):**
1. Synthetic residual `sin(x)` — GP mean matches after training, variance small near training points
2. Extrapolation — variance grows away from training data

---

### E4 — MHE Inequality Constraints (extend MovingHorizonEstimator)

**Goal:** Add polytopic state constraints `C_ineq * x_0 <= d_ineq` to the arrival cost
block in `MovingHorizonEstimator`. This allows hard bounds on state estimates (e.g.,
concentration ≥ 0, temperature ≤ T_max) without a separate projection step.

**New fields in `MHEParams`:**
```cpp
MatrixXd C_ineq;  // (m x n), empty = no constraint
VectorXd d_ineq;  // (m,)
```

**Implementation:** Apply `C_ineq * z[0:n] <= d_ineq` as a polytopic projection inside
the existing FISTA loop. For box constraints (diagonal `C_ineq`), use direct clipping;
for general polytopic, add a lightweight projected-gradient step.

**Reused components:** `GradientProjectionQP` box-projection; `MovingHorizonEstimator` FISTA loop.

**Effort estimate:** ~100 lines (+ 2 tests)

**Note:** Box constraints (`xMin`/`xMax`) were already added in Part 34. This extends to
general polytopic `C_ineq` for applications like simplex constraints (mole fractions sum to 1).

**Catch2 test plan (`[mhe_polytopic]`):**
1. Simplex constraint (x1 + x2 = 1, both ≥ 0) — estimate stays on simplex
2. Hard lower bound on concentration — estimate never goes negative despite noisy measurements

---

## Phase 2: Hybrid Models

### H1 — HybridModel Base Class

**Goal:** Abstract interface `IPlantModel` (new interface, does not implement `IController`)
that represents `xdot = f_phys(x, u, p) + f_data(x, u)`. The data component is optional
and swappable at runtime.

**Interface sketch:**
```cpp
class IPlantModel {
public:
    virtual ~IPlantModel() = default;
    virtual VectorXd dynamics(const VectorXd& x, double u) const = 0;
    virtual int stateSize() const = 0;
};

class HybridModel : public IPlantModel {
public:
    using PhysicsFunc = std::function<VectorXd(const VectorXd& x, double u)>;
    using DataFunc    = std::function<VectorXd(const VectorXd& x, double u)>;

    HybridModel(PhysicsFunc f_phys, int n_states);
    void setDataModel(DataFunc f_data);       // attach learned residual
    void clearDataModel();                    // revert to physics-only
    VectorXd dynamics(const VectorXd& x, double u) const override;
    int stateSize() const override;
};
```

**Effort estimate:** ~100 lines (header-only; no .cpp needed)

**Example use case:** Quadrotor with nominal rigid-body dynamics (physics) + GP residual
that corrects for ground effect and rotor asymmetry (data). The same `HybridModel` object
is used by both `HybridMPC` and the digital twin dashboard.

---

### H2 — HybridMPC

**Goal:** `NonlinearMPC` variant that uses a `HybridModel` for rollout predictions.
The data component can be updated every `N_update` steps as new data arrives.

**Class sketch:**
```cpp
class HybridMPC : public NonlinearMPC {
public:
    HybridMPC(std::shared_ptr<HybridModel> model,
              const NonlinearMPCParams& params);
    void updateDataModel(HybridModel::DataFunc f_data);
    // compute() inherited from NonlinearMPC; prediction uses HybridModel::dynamics
};
```

**Reused components:** `NonlinearMPC` prediction loop (override `rollout()` to call
`HybridModel::dynamics` instead of the user-supplied StateFunc).

**Effort estimate:** ~200 lines (+ 2 tests + Python binding)

---

### H3 — RL-MPC Stitching (Python example only)

**Goal:** A Python example showing how a small DQN or PPO policy can adjust `HybridMPC`
parameters (specifically `rho_y` multiplier or reference offset) based on tracking error
and control effort, without modifying the C++ core.

**Implementation:** Pure Python. Uses `ctrl_toolbox` bindings + PyTorch (2-layer, <10k params).
The policy observes `[e(t), |u(t)|, e_rms_window]` and outputs a discrete multiplier for `rho_y`.
The MPC runs with adjusted parameters each step.

**File:** `examples/python/ex103_rl_mpc.py`

**Effort estimate:** ~200 lines Python

**Dependency:** H2 (HybridMPC) for the MPC it tunes; PyTorch for the policy.

---

### H4 — HybridModelTrainer

**Goal:** Train the `f_data` component of a `HybridModel`. Supports two backends:
(a) GP marginal-likelihood hyperparameter optimisation (for GP residuals from E3),
(b) `EchoStateNetwork` offline ridge regression (for ESN residuals).

**Class sketch:**
```cpp
class HybridModelTrainer {
public:
    // GP backend: maximise log marginal likelihood via L-BFGS
    static DataFunc trainGP(const MatrixXd& XU_data,
                             const VectorXd& residuals,
                             GaussianProcess& gp);

    // ESN backend: offline fit and return forward-pass lambda
    static DataFunc trainESN(const MatrixXd& XU_data,
                              const VectorXd& residuals,
                              EchoStateNetwork& esn);
};
```

**Reused components:** `GaussianProcess::residualFit()` (E3); `EchoStateNetwork` (existing).

**Effort estimate:** ~250 lines (+ 2 tests)

---

## Phase 3: Deployment / Validation

### D1 — Mismatch Detector (extend KF/MHE)

**Goal:** Real-time CUSUM on the KF/MHE innovation sequence. When the innovation exceeds
a threshold for a sustained number of steps, `mismatchDetected()` returns `true` and
optionally triggers a re-estimation callback.

**New method on `KalmanFilter` and `MovingHorizonEstimator`:**
```cpp
struct MismatchDetectorParams {
    double k_cusum = 0.5;    // CUSUM slack (half-sigma)
    double h_threshold = 5.0; // detection threshold (sigma)
    int window = 20;
};

void enableMismatchDetection(const MismatchDetectorParams& p);
bool mismatchDetected() const;
double mismatchScore() const;
```

**Reused components:** `ControllerMonitor` CUSUM implementation pattern (extract into
a standalone `CUSUMDetector` helper class, then call from both KF and MHE).

**Effort estimate:** ~100 lines (+ 2 tests)

---

### D2 — Digital Twin Lite (Python application)

**Goal:** A self-contained Python reference application demonstrating the full Phase 1-2
pipeline: run a simulation, log plant + model predictions, detect mismatch, periodically
re-estimate parameters, visualise the results.

**Stack:** FastAPI (REST API + WebSocket push) + Plotly Dash (frontend) + `ctrl_toolbox`
bindings. No new C++ required.

**File structure:**
```
tools/digital_twin/
├── app.py            FastAPI + Dash entry point
├── twin.py           DigitalTwin class: runs GreyBoxEstimator + MismatchDetector
├── dashboard.py      Plotly Dash layout + callbacks
└── README.md
```

**Effort estimate:** ~300 lines Python

**Dependency:** E1 (GreyBoxEstimator), D1 (MismatchDetector), H2 (HybridMPC optional).

---

## What Is Explicitly Deferred

| Feature | Reason for deferral |
|---------|---------------------|
| DAE Index ≥ 2 (Pantelides algorithm, BLT ordering) | Research-grade effort; Index-1 (P1-P3) covers all current case-study plants |
| FMU import/export (libfmilib) | Heavy external dependency; low immediate value vs. effort |
| CasADi symbolic auto-differentiation | Adds a large dependency; `LinearisationHelper` numerical Jacobians are sufficient for moderate n |
| Full RL framework (Stable-Baselines3 integration) | H3 example covers the use case; no C++ RL core needed |
| Control co-design (joint plant + controller optimisation) | Separate research track; not needed to complete Phase 2 |
| Sparse GP (inducing-point approximation) | E3 GP residual uses the fixed-budget FIFO eviction already in `GaussianProcess`; sparse GP is only needed for N > 500, not a current bottleneck |

---

## Implementation Checklist (per algorithm)

Each new `lib/` algorithm must follow the 8-step checklist from `CLAUDE.md`:

```
1. lib/ClassName.{h,cpp} — implement; call notifyObserver() at end of compute()
2. lib/CMakeLists.txt — add ClassName.cpp to CTRL_CORE_SOURCES
3. lib/ControllerToolbox.h — add #include "ClassName.h"
4. lib/Features.h — add {"feature_name", true} entry
5. bindings/*_bindings.cpp — add pybind11 class with std::shared_ptr<T> 3rd arg
6. bindings/smoke_test.py — add assertion
7. tests/test_catch2_advanced.cpp — add 2+ Catch2 tests with [tag]
8. examples/exNN.cpp + examples/python/exNN.py + update CMakeLists.txt + compile.bat
```

For Python-only features (H3, D2): steps 1-4 and 7-8 (C++ side) are skipped.
For extensions to existing classes (E3, E4, D1, P2, P3): only the modified files need updating,
not the full 8-step checklist — but Catch2 tests are always required.
For `DAESystem` (P1): struct + free functions go in `PlantModel.h`/`PlantModel.cpp` alongside
the existing `TransferFunction` and `StateSpace` — no new lib/ file needed.

---

## Estimated Timeline

| Phase | Items | Effort | Notes |
|-------|-------|--------|-------|
| Phase 0 | P1 (`DAESystem` + `dae2ode`) | ~3 days | First; unlocks cleaner plant formulation in all later phases |
| Phase 0 | P2 (`c2d` for DAE) | ~1-2 days | After P1; short — reuses existing `c2d` + `LinearisationHelper` |
| Phase 0 | P3 (DAE-aware EKF) | ~1-2 days | After P1; extends existing EKF |
| Phase 1 | E1, E2, E4 | ~5-7 days | After P1; E3 after E1 is validated |
| Phase 1 | E3 | ~2-3 days | After E1 confirms GP residual concept |
| Phase 2 | H1, H2 | ~3-4 days | Requires E3 for meaningful data component |
| Phase 2 | H4 | ~2 days | Requires E3 + H1 |
| Phase 3 | D1 | ~1-2 days | Independent; can be done in parallel with H1 |
| Phase 2 | H3 | ~2 days | Requires H2; Python only |
| Phase 3 | D2 | ~2-3 days | Requires E1 + D1 |
| **Total** | 13 items | **~23-28 days** | Focused development, part-time over 6-7 weeks |

---

*This document is the planning reference for Part 50+. Update it as items are implemented.*
