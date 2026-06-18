# Phase 2 - Grey-Box Estimation and Hybrid Modeling

### Parameter Identification, Online Tracking, Residual Learning, and Hybrid Model Predictive Control

---

## Introduction

Classical black-box identification (ARX, N4SID, GP-NARX) treats the plant as an unknown mapping from inputs to outputs and estimates all parameters from data alone. Grey-box methods occupy the middle ground: the physical structure of the ODE is known (or partially known), but one or more parameters are uncertain. This approach is orders of magnitude more data-efficient than pure black-box identification because the structure constrains the solution space, and the resulting model inherits physical interpretability-mass, stiffness, time constant, and heat-transfer coefficient remain identifiable even from short experiments.

The Phase 2 algorithms in this toolbox extend that principle through four layers:

| Layer | ID | Class | Role |
|---|---|---|---|
| Batch parameter fitting | E1 | `GreyBoxEstimator` | Levenberg-Marquardt ODE fitting from a fixed dataset |
| Online parameter tracking | E2 | `RecursiveGreyBoxEstimator` | Augmented-state UKF; updates parameters each sample |
| Residual learning | E3 | `GPResidualModel` | GP correction for systematic model-plant mismatch |
| Hybrid prediction | H1/H4 | `HybridModel` + `HybridModelTrainer` | Physical ODE + trained data-driven correction |
| Hybrid MPC | H2 | `HybridMPC` | Nonlinear MPC using `HybridModel` as the prediction engine |

---

## Part I - Batch Grey-Box Parameter Estimation

---

## E1. GreyBoxEstimator

**Purpose.** Identify unknown parameters $p$ in a user-supplied continuous-time ODE
$$\dot{x} = f(x, u, p), \quad y = h(x, p)$$
from a batch of measured input-output trajectories. Integration uses fourth-order Runge-Kutta; the Jacobian of the output with respect to parameters is approximated by central finite differences; Levenberg-Marquardt (LM) iterates until convergence or the maximum iteration count.

**Non-obvious API facts.**

- `fit()` returns `GreyBoxResult{params, cost, iterations, converged}` - always check `converged`.
- Parameters are box-constrained via projection after each LM step; pass `p_min` / `p_max` in `GreyBoxParams`.
- The ODE `f` and measurement `h` are `std::function<VectorXd(VectorXd, VectorXd, VectorXd)>` - capture plant constants by value in the lambda.
- `predict(x0, U)` returns an `(n_out * N)` matrix of simulated outputs (not a trajectory structure).
- Finite-difference step is scaled per parameter: `delta_p[i] = fd_step * max(1, |p[i]|)` - avoids near-zero step for small parameters.

**Minimal C++ usage.**
```cpp
#include "GreyBoxEstimator.h"

ctrl::GreyBoxEstimator::Params gp;
gp.p_init   = Eigen::Vector2d{0.5, 2.0};   // initial guess [k, c]
gp.p_min    = Eigen::Vector2d{0.01, 0.01};
gp.p_max    = Eigen::Vector2d{10.0, 10.0};
gp.max_iter = 100;
gp.tol      = 1e-8;
gp.n_substeps = 4;   // RK4 sub-steps per sample interval

auto f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u,
            const Eigen::VectorXd& p) -> Eigen::VectorXd {
    // mass-spring-damper: xdot = [x1; (-k*x0 - c*x1 + u0)/m]
    return Eigen::Vector2d{x(1), (-p(0)*x(0) - p(1)*x(1) + u(0)) / 1.0};
};
auto h = [](const Eigen::VectorXd& x, const Eigen::VectorXd& p) -> Eigen::VectorXd {
    return x.head(1);  // observe position only
};

ctrl::GreyBoxEstimator est(f, h, 2, 1, 1, Ts, gp);
auto res = est.fit(x0, U_batch, Y_batch);

if (res.converged)
    std::cout << "k=" << res.params(0) << " c=" << res.params(1) << "\n";
```

**Python usage.**
```python
import ctrl_toolbox as ctrl
import numpy as np

params = ctrl.GreyBoxParams()
params.p_init = np.array([0.5, 2.0])
params.p_min  = np.array([0.01, 0.01])
params.p_max  = np.array([10.0, 10.0])
params.max_iter = 100

def ode(x, u, p):
    return np.array([x[1], (-p[0]*x[0] - p[1]*x[1] + u[0])])

def meas(x, p):
    return x[:1]

est = ctrl.GreyBoxEstimator(ode, meas, n_state=2, n_in=1, n_out=1, Ts=Ts, params=params)
result = est.fit(x0, U, Y)
print(result.params, result.converged, result.cost)
```

**When to use GreyBoxEstimator.**
- You have a physics-based ODE with 1-20 unknown scalar parameters.
- A one-time batch experiment provides enough data.
- You want bounded, physically interpretable estimates (not neural network weights).
- Data length is short (tens to hundreds of samples) - black-box methods would overfit.

**Pitfalls.**
- LM is a local optimizer. Use multiple restarts with different `p_init` if the cost landscape is multi-modal.
- If `n_substeps` is too low, RK4 integration error exceeds parameter uncertainty and LM converges to wrong values. Use `n_substeps >= 4` for Ts/tau > 0.1.
- Box constraints are enforced only by projection, not in the LM step direction. Parameters can temporarily violate bounds during a step; the projection clamps them before the next cost evaluation.

---

## E2. RecursiveGreyBoxEstimator

**Purpose.** Track slowly time-varying ODE parameters $p(k)$ online, one sample at a time, without storing the full dataset. The state is augmented to $z = [x; p]$; the process model integrates the ODE for $x$ (RK4) and holds $p$ constant plus Gaussian diffusion noise $Q_\text{param}$; the UKF update step corrects both $x$ and $p$ from the current measurement $y$.

**Non-obvious API facts.**

- Constructor requires `CTRL_ENABLE_ADVANCED_KALMAN` (same guard as `UnscentedKalmanFilter`) - ensure the CMake flag is ON.
- `alpha` (UKF spread parameter) should be 0.1-0.3 for augmented state dimensions >= 3. The default `alpha=1` used for standard UKF gives negative weights $W_{c,0}$ and can produce NaN.
- `step(y, u)` returns void; call `stateEstimate()`, `paramEstimate()`, and `covariance()` separately.
- `Q_param` in `RecursiveGreyBoxParams` controls how fast parameters are allowed to drift - too large tracks noise, too small misses real changes. Start with `Q_param = diag(1e-4 * p_nominal^2)`.
- `initialize(x0, P0_state)` sets the initial state; parameter covariance is set separately in `params.P0_param`.

**Minimal C++ usage.**
```cpp
#include "RecursiveGreyBoxEstimator.h"

ctrl::RecursiveGreyBoxEstimator::Params rp;
rp.p_init     = Eigen::Vector2d{3.0, 0.5};
rp.Q_param    = 1e-5 * Eigen::Matrix2d::Identity();
rp.P0_param   = 0.1  * Eigen::Matrix2d::Identity();
rp.R_meas     = 1e-3 * Eigen::MatrixXd::Identity(1,1);
rp.alpha      = 0.15;

ctrl::RecursiveGreyBoxEstimator rge(f, h, 2, 1, 1, Ts, rp);
rge.initialize(x0, 0.01 * Eigen::Matrix2d::Identity());

for (int k = 0; k < N; ++k) {
    rge.step(y_meas[k], u[k]);
    auto p_est = rge.paramEstimate();
    auto x_est = rge.stateEstimate();
}
```

**Python usage.**
```python
params = ctrl.RecursiveGreyBoxParams()
params.p_init   = np.array([3.0, 0.5])
params.Q_param  = 1e-5 * np.eye(2)
params.P0_param = 0.1  * np.eye(2)
params.R_meas   = 1e-3 * np.eye(1)
params.alpha    = 0.15

rge = ctrl.RecursiveGreyBoxEstimator(ode, meas, 2, 1, 1, Ts, params)
rge.initialize(x0, 0.01 * np.eye(2))

for k in range(N):
    rge.step(y[k], u[k])
p_track = rge.param_estimate()
```

**When to use RecursiveGreyBoxEstimator.**
- Plant parameters drift slowly over time (thermal expansion, wear, fouling, batch-to-batch variation).
- Online adaptation is required - no offline re-identification window.
- You have a physics ODE and want online parameter tracking rather than a black-box adaptive filter.

---

## E3. GPResidualModel

**Purpose.** Learn the systematic model-plant mismatch
$$\epsilon(x_\text{feat}) = y_\text{true} - y_\text{model}(x_\text{feat})$$
as a Gaussian Process, then apply the GP correction at prediction time:
$$\hat{y}_\text{corrected}(x_\text{feat}) = y_\text{model}(x_\text{feat}) + \mu_\text{GP}(x_\text{feat})$$

The posterior variance $\sigma_\text{GP}^2$ is available for risk-aware MPC constraint tightening.

**Non-obvious API facts.**

- `predictWithUncertainty(xf, model_pred)` returns `GPResidualPrediction{mean_total, gp_mean, variance}`.
- When unfitted: returns `{model_pred, 0.0, 0.0}` - safe fallback, no exception.
- Feature vector `xf` must be the same dimension passed to all `addResidualPoint` calls.
- `residualFit(X_feat, Y_true, model_fn)` calls `model_fn(xf)` internally and stores $\epsilon$; the underlying GP uses the SE kernel from `GaussianProcess`.
- Budget eviction: the internal `GaussianProcess` uses fixed-budget eviction (oldest point dropped when capacity is reached) - set `GPResidualParams.gp.budget` appropriately.

**Minimal Python usage.**
```python
gp_res = ctrl.GPResidualModel(ctrl.GPResidualParams())

# Batch fit
def model_fn(xf):
    return float(nominal_model.predict(xf))

gp_res.residual_fit(X_feat, Y_true, model_fn)

# Online correction at run time
pred = gp_res.predict_with_uncertainty(x_feat, nominal_output)
y_corrected  = pred.mean_total
uncertainty  = pred.variance     # use for constraint tightening
```

---

## Part II - Hybrid Model Stack

---

## H1. HybridModel

**Purpose.** Represent a plant whose dynamics are the sum of a known physical ODE and an unknown data-driven correction:
$$x_{k+1} = f_\text{phys}(x_k, u_k) + f_\text{data}(x_k, u_k)$$

`HybridModel` stores a `PhysFunc` (physical step) and a `DataFunc` (correction, initially zero). The data function is installed by `HybridModelTrainer` or updated online by `HybridMPC`.

**Key methods.**

| Method | Description |
|---|---|
| `predictPhys(x, u)` | Physical model only; used by the trainer to compute residuals |
| `predict(x, u)` | Physical + data correction; used by the MPC rollout |
| `setDataModel(DataFunc)` | Install (or replace) the trained correction function |
| `hasDataModel()` | True after the first training pass |

**Typical workflow.**
```
HybridModel model(phys_step_fn, n_state, n_input);
// model.predict == model.predictPhys until trained

HybridModelTrainer trainer(tp);
auto result = trainer.trainHybridModel(model, X_obs, U_obs, X_next_obs);
// model.predict now includes GP/Ridge/ESN correction
```

---

## H4. HybridModelTrainer

**Purpose.** Compute residuals $\delta x_k = x_{k+1}^\text{obs} - f_\text{phys}(x_k, u_k)$ from a batch of observed transitions and fit a data-driven function to those residuals. Three training methods are available:

| Method | Class | Pros | Cons |
|---|---|---|---|
| `Ridge` | Eigen ridge regression | Fastest, always converges, $O(N)$ predict | Linear features only; underfits strong nonlinearities |
| `GP` | One GP per state dimension | Nonlinear + posterior variance | $O(N^3)$ training; budget-limited |
| `ESN` | `EchoStateNetwork` | Captures temporal dependencies | Stateless at predict time (reservoir reset); hyperparameter-sensitive |

**Non-obvious API facts.**

- The constructor now takes an explicit default: `HybridModelTrainer()` (no params, uses `Params{}` defaults) or `HybridModelTrainer(const Params&)`. This split was necessary to work around a GCC/MinGW bug with nested-struct default arguments.
- For GP method: `params.gp` (a `GaussianProcess::Params`) is applied identically to all state dimensions.
- For ESN method: `params.esn.n_in` and `params.esn.n_out` are **overridden** to match the feature/state dimensions - do not set them manually.
- `Result::train_rmse` is evaluated on the training data (not held-out); always validate separately with `validate()`.

**Minimal C++ usage.**
```cpp
#include "HybridModelTrainer.h"

ctrl::HybridModelTrainer::Params tp;
tp.method       = ctrl::HybridModelTrainer::Method::GP;
tp.gp.length_scale = 0.5;
tp.gp.budget    = 200;

ctrl::HybridModelTrainer trainer(tp);
auto res = trainer.trainHybridModel(model, X_obs, U_obs, X_next_obs);
std::cout << "RMSE=" << res.train_rmse << "  method=" << res.method << "\n";

double val_rmse = trainer.validate(model, X_val, U_val, X_next_val);
```

**Python usage.**
```python
tp = ctrl.HybridTrainerParams()
tp.method = ctrl.HybridTrainerMethod.GP
tp.gp.length_scale = 0.5

trainer = ctrl.HybridModelTrainer(tp)
result  = trainer.train_hybrid_model(model, X_obs, U_obs, X_next_obs)
print(result.train_rmse, result.converged)

val_rmse = trainer.validate(model, X_val, U_val, X_next_val)
```

---

## H2. HybridMPC

**Purpose.** Nonlinear MPC that uses a `HybridModel` as its internal prediction engine, with the option to update the data correction every $N_\text{update}$ steps using newly collected data. This closes the loop between online data collection and MPC prediction quality.

**Key differences from `NonlinearMPC`.**

| Property | `NonlinearMPC` | `HybridMPC` |
|---|---|---|
| Internal model | User-supplied `StateFunc` | `HybridModel` (phys + data) |
| Data correction | No | Ridge regression, updated online every $N_\text{update}$ steps |
| Prediction error | Fixed by model quality | Decreases as data accumulates |
| State dimension | Any | Must match `HybridModel` |

**Non-obvious API facts.**

- `compute(x, r)` returns $u$ as a scalar (SISO) or vector (MIMO); the internal rollout calls `model.predict()` which already includes the data correction.
- Online update is triggered automatically when the circular buffer reaches `params.n_collect` samples. Set `params.n_refit_every` to control how often refitting is triggered within the collection window.
- Ridge `lambda` for online update is set via `params.ridge_lambda` (separate from `HybridModelTrainer::Params` used for offline training).
- The physical model step function inside `HybridModel` must be deterministic and stateless (pure function of `(x, u)`); do not capture mutable state.

**Minimal C++ usage.**
```cpp
#include "HybridMPC.h"

ctrl::HybridMPCParams mp;
mp.Np           = 15;
mp.Nu           = 5;
mp.rho_y        = 10.0;
mp.rho_u        = 0.1;
mp.u_min        = -1.0;
mp.u_max        =  1.0;
mp.n_collect    = 50;     // collect 50 steps before first refit
mp.n_refit_every = 25;
mp.ridge_lambda = 1e-4;

ctrl::HybridMPC mpc(model, Ts, mp);

for (int k = 0; k < N_sim; ++k) {
    double u = mpc.compute(x_current, r_ref);
    // plant step...
    mpc.addSample(x_current, u, x_next);   // feed to online buffer
}
```

---

## Part III - Workflow: From Data to Hybrid MPC

The recommended end-to-end workflow combines all five algorithms:

```
Step 1: Identify nominal parameters offline
        GreyBoxEstimator.fit(x0, U_id, Y_id)
        -> p_nominal, converged check

Step 2: Quantify residual (model-plant mismatch)
        HybridModelTrainer(GP).trainHybridModel(model, X, U, X_next)
        -> f_data installed in HybridModel

Step 3: Deploy HybridMPC with online refinement
        HybridMPC.compute(x, r)  [online, updates f_data every n_collect steps]

Step 4: Track parameter drift (parallel path)
        RecursiveGreyBoxEstimator.step(y, u)  [runs every sample]
        -> if param drift detected, re-run Step 1 and reset HybridModel

Step 5: Risk-aware constraint tightening (optional)
        GPResidualModel.predictWithUncertainty(xf, y_model)
        -> variance used to tighten MPC state constraints
```

---

## Comparison and Selection Guide

| Scenario | Recommended approach |
|---|---|
| Short batch experiment, known ODE structure, static parameters | `GreyBoxEstimator` (E1) |
| Parameters drift slowly over operation | `RecursiveGreyBoxEstimator` (E2) alongside fixed MPC |
| Physical ODE underfits due to unmodelled nonlinearity | `HybridModel` + `HybridModelTrainer` (H1/H4) with GP correction |
| Black-box data only (no physics knowledge) | Standard `NonlinearMPC` or `DynaController` |
| Want calibrated uncertainty for constraint tightening | `GPResidualModel` (E3) feeding constraint bounds to `HybridMPC` |
| Online closed-loop adaptation with physics prior | `HybridMPC` (H2) with Ridge online update |
| Both online adaptation and uncertainty quantification | `HybridMPC` (H2, Ridge) + parallel `GPResidualModel` (E3) |

---

*See also:* `system_identification.md` (classical parametric structures), `advanced_model_estimation.md` (black-box ML methods), `mismatch_detection.md` (real-time CUSUM-based change detection on KF/MHE innovations).
