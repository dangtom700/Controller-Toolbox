# Controller Toolbox - Cumulative Bug Report (Part 51+)

**Active issues start at Part 51.** Earlier history is archived in two compact references:
- [`docs/compact_bug_report_parts_1-25.md`](compact_bug_report_parts_1-25.md) — Parts 1-25 (2026-05-19 through 2026-05-30)
- [`docs/compact_bug_report_parts_26-50.md`](compact_bug_report_parts_26-50.md) — Parts 26-50 (2026-05-31 through 2026-06-11)

Read both compact files for tribal knowledge before making any changes to controllers or case studies.

---

## Open Issues Log (Part 51+)

*(Append dated entries below as work proceeds.)*

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| **P1** | `DAESystem` struct + `dae2ode()` — Index-1 semi-explicit DAE; Newton solve on `g` | HIGH | **Done (Part 51)** |
| **P2** | `c2d()` overload for DAE — linearise + algebraic elimination + ZOH/Tustin | MED | **Done (Part 51)** |
| **P3** | DAE-aware EKF — post-update algebraic projection via `consistentInit()` | MED | **Done (Part 51)** |
| **E1** | `GreyBoxEstimator` — non-linear param estimation via Levenberg-Marquardt | HIGH | **Done (Part 52)** |
| **E2** | `RecursiveGreyBoxEstimator` — augmented-state UKF for online param tracking | HIGH | **Done (Part 52)** |
| **E3** | GP Residual Model — extend `GaussianProcess` with uncertainty output | MED | **Done (Part 52)** |
| **E4** | MHE Polytopic Constraints — extend MHE with `C_ineq`/`d_ineq` | MED | **Done (Part 53)** |
| **H1** | `HybridModel` base class — `IPlantModel` with `f_phys + f_data` | MED | **Done (Part 53)** |
| **H2** | `HybridMPC` — `NonlinearMPC` variant using `HybridModel` | MED | **Done (Part 53)** |
| **H3** | RL-MPC stitching Python example | LOW | **Done (Part 53)** |
| **H4** | `HybridModelTrainer` — hyperopt for `f_data` component | LOW | **Done (Part 53)** |
| **D1** | Mismatch Detector — CUSUM on KF/MHE innovation | LOW | **Done (Part 54)** |
| **D2** | Digital Twin Lite Python app | LOW | Open |
| **C2** | 8 spec-only stubs (BEMS + MEMS no blocker; others need plant design) | MED | Open |
| **B36-3** | Unify NaN-guard across controller fleet | MED | **Done (Part 53)** |
| R1 | Edge-case contract matrix tests for every controller family | MED | **Done (Part 53)** |
| T3 | Full DK-iteration with vector-fitting rational D(jω) | LOW | **Done (Part 53)** |
| B36-2 | `ex79_registry_monitor` monitors nothing (M3 telemetry mis-wired) | LOW | **Done (Part 39, confirmed Part 53)** |
| REL | Rebuild `ctrl_toolbox.pyd` in Release | LOW | Open |
| M4 | `template<typename Scalar>` leaf algorithms for embedded float target | Backlog | **Done (Part 54)** |

---

*(New parts appended below as work proceeds.)*

---

## Part 54 — D1 (MismatchDetector), M4 (BasicPID/BasicSMC) — 2026-06-12

**D1 — `MismatchDetector`** (`lib/MismatchDetector.h`, header-only)

- Wraps `CUSUMChart` (from `ControllerMonitor.h`) to run real-time CUSUM on the
  normalised innovation of a `KalmanFilter` or `MovingHorizonEstimator`.
- `MismatchDetectorParams`: `sigma` (in-control innovation RMS), `k_cusum` (slack, default 0.5),
  `h_threshold` (alarm level, default 5.0).
- `update(double)` feeds a scalar innovation (absolute value used internally to detect
  both upward and downward shifts in magnitude).
- `update(VectorXd&)` computes `‖innov‖/sqrt(p)` and feeds to scalar CUSUM.
- `detected()`: sticky bool — stays `true` until `reset()` is called.
- `score()`: current CUSUM statistic `max(C+, C-)`.
- `CTRL_REGISTER_FEATURE(mismatch_detector)`.

**D1 — Extensions to `KalmanFilter`** (`lib/KalmanFilter.{h,cpp}`)

- `enableMismatchDetection(params)`: attaches a `MismatchDetector` member; feeds
  `innov = y - C*x_pred - D*u` into CUSUM after every `update()` call.
- `mismatchDetected()`, `mismatchScore()`, `resetMismatchDetector()` accessors.
- Private: `std::optional<MismatchDetector> mismatch_det_`.

**D1 — Extensions to `MovingHorizonEstimator`** (`lib/MovingHorizonEstimator.{h,cpp}`)

- Same API: `enableMismatchDetection()`, `mismatchDetected()`, `mismatchScore()`,
  `resetMismatchDetector()`.
- Feeds `y_hist_[N] - C*x_est_` (one-step-ahead residual) into CUSUM after each
  `estimate()` call.

**D1 — Bindings / tests**

- `estimation_bindings.cpp`: `enable_mismatch_detection(sigma, k_cusum, h_threshold)`,
  `mismatch_detected()`, `mismatch_score()`, `reset_mismatch_detector()` on both
  `KalmanFilter` and `MovingHorizonEstimator`.
- `smoke_test.py`: D1 block — creates KF, enables detection, asserts no alarm on zero
  steps and `mismatch_score()` is float, checks registry.
- `tests/test_catch2_advanced.cpp`: 7 new `[mismatch_detector]` tests:
  (1) no alarm on white-noise innovation, (2) sustained shift triggers detection,
  (3) reset clears alarm, (4) vector innovation fires alarm, (5) KF disabled by default,
  (6) KF fires on wrong model, (7) MHE fires on mismatched model.

**M4 — `BasicPID<Scalar>` and `BasicSMC<Scalar>`** (`lib/BasicPID.h`, `lib/BasicSMC.h`,
header-only)

- `BasicPID<Scalar>`: backward-Euler ISA PID with derivative filter (alpha = 1/(1+N*Ts)),
  back-calculation anti-windup (Kb), and saturation. No IController base, no Eigen, no
  virtual dispatch. Suitable for `float` on MCU targets. Methods: `compute(error)`,
  `reset()`, `setParams()`, `lastOutput()`, `integrator()`.
- `BasicSMC<Scalar>`: first-order SMC with boundary-layer saturation
  `u = -K * clamp(s/phi, -1, 1)` where `s = c_e*e + c_de*(e - e_prev)`. Same embedded-target
  constraints. Methods: `compute(error)`, `reset()`, `setParams()`, `lastOutput()`,
  `slidingSurface(error)`.
- Both added to `lib/ControllerToolbox.h` umbrella include.
- `CTRL_REGISTER_FEATURE(basic_pid)` / `CTRL_REGISTER_FEATURE(basic_smc)` for feature checks.
- No Python bindings (template types; not useful via pybind11 `shared_ptr<IController>` path).
- `smoke_test.py`: checks `registry_has('basic_pid')` and `registry_has('basic_smc')`.
- `tests/test_catch2_advanced.cpp`: 7 new tests — 4 `[basic_pid]` (step response, float
  saturation, reset, anti-windup bounded integrator) + 3 `[basic_smc]` (convergence,
  float saturation, reset reproducibility).

---

## Part 53 — Hybrid Models (H1-H4), E4, T3, B36-2/B36-3, R1 — 2026-06-12

**E4 — MHE Polytopic Inequality Constraints** (`lib/MovingHorizonEstimator.{h,cpp}`)

- `MHEParams::C_ineq` (m_c × n) and `d_ineq` (m_c): enforce `C_ineq * x_0 <= d_ineq` on
  arrival state after the FISTA solve via Hildreth's cyclic half-space projections.
- `ineq_proj_iters` (default 20): number of Hildreth sweeps. Box constraints (`xMin`/`xMax`) are
  re-applied after polytope projection so both are simultaneously satisfied.
- `projectX0Polytope()` private helper: no-op when `C_ineq` is empty; otherwise iterates
  half-space projections then re-clips to box.
- 3 `[mhe_polytopic]` Catch2 tests: half-space upper bound, simplex-coupled constraint,
  equivalence with xMax for pure box case.
- `estimation_bindings.cpp`: `C_ineq`, `d_ineq`, `ineq_proj_iters` exposed on `MHEParams`.
- `smoke_test.py`: E4 block asserts `C_ineq`/`d_ineq` shape round-trips correctly.

**H1 — `HybridModel`** (`lib/HybridModel.h`, header-only)

- `IPlantModel` abstract interface: `dynamics(x, u)`, `stateSize()`, `outputSize()`.
- `HybridModel` concrete class: `f_phys(x, u)` (physics, required) + optional `f_data(x, u)`
  (data-driven correction). `setDataModel()` / `clearDataModel()` swappable at runtime.
- RK4 `predict(x0, U, Ts)` helper on `IPlantModel`.

**H2 — `HybridMPC`** (`lib/HybridMPC.{h,cpp}`)

- Inherits `NonlinearMPC`; overrides the prediction rollout to call `HybridModel::dynamics`.
- `updateDataModel(DataFunc)`: hot-swaps the data correction without rebuilding the QP.
- `N_update` parameter: data model refreshed from new observations every N steps.

**H3 — RL-MPC Stitching** (`examples/python/ex101_rl_mpc_stitching.py`, Python-only)

- Lightweight DQN-style policy (<10 k params, numpy-only) that adjusts `rho_y` of `HybridMPC`
  in real time on a spring-mass-damper plant.
- Policy state: `[error, error_dot]`; actions: 4 discrete `rho_y` multipliers.
- Demonstrates H2 + H3 integration without PyTorch dependency.

**H4 — `HybridModelTrainer`** (`lib/HybridModelTrainer.{h,cpp}`)

- `trainGP(XU, residuals, gp)` → `DataFunc`: fits `GaussianProcess` residual and returns a
  lambda capturing the trained GP's `predict()` method.
- `trainESN(XU, residuals, esn)` → `DataFunc`: offline ridge-regression on `EchoStateNetwork`
  and returns the ESN forward-pass lambda.
- `trainRidge(XU, residuals, lambda_reg)` → `DataFunc`: lightweight fallback using Eigen
  ridge (no external dependency).

**T3 — VectorFitting + full DK-iteration** (`lib/VectorFitting.{h,cpp}`)

- Gustavsen SK iterative rational fitting of complex frequency response data → poles + residues.
- `solveMuSyn` in `DiscreteHinf`: `dFitOrder > 1` switches from first-order D-scaling to
  vector-fit rational D(jω), enabling full DK-iteration for structured-uncertainty mu-synthesis.

**B36-2 — `ex79_registry_monitor` fix** (Part 39, confirmed Part 53)

- `shared_ptr` monitor was copy-constructed before attachment; fixed to create `mon_ptr` first
  so callback and observer share a single instance. Observer now actually fires.

**B36-3 — NaN-guard hold-last fleet contract** (`lib/IController.h` + 7 controllers)

- `sanitize()` removed from contract. Hold-last NaN behaviour added to:
  ExtremumSeeker, MRAC, TubeMPC (scalar path), ScenarioMPC (scalar path),
  RepetitiveController, SmithPredictor, DiscreteHinf.
- `IController.h` documents the hold-last contract.
- `[nan_guard]` Catch2 tags added across all affected controllers.

**R1 — Contract matrix tests** (`tests/test_catch2_advanced.cpp`)

- `[nan_guard]` + `[health_contract]` extended to all controller families.
- Saturation-bounded-integral and non-stabilizable `isHealthy()` coverage added.

**M4 — BasicPID / BasicSMC (CLAIMED done in CLAUDE.md — NOT VERIFIED)**

- CLAUDE.md states `lib/BasicPID.h` and `lib/BasicSMC.h` (header-only `BasicPID<Scalar>` /
  `BasicSMC<Scalar>` for embedded float usage) were created in Part 53.
- **Files do NOT exist** in the repository as of the Part 53 audit (2026-06-12).
- Marked Open in the issues table. Must be implemented before closing M4.

---

## Part 51 — DAE Architecture (P1/P2/P3) — 2026-06-12

**P1 — `DAESystem` + `consistentInit` + `dae2ode`** (`lib/PlantModel.h/.cpp`)

- `DAESystem` struct: `f` (differential), `g` (algebraic), `h` (output), `n_diff`, `n_alg`, `Ts`.
- `consistentInit(dae, x1_init, u0, x2_guess)`: Newton-Raphson (LDLT) solving `g=0` for `x2`; up to 20 iters, tol=1e-9.
- `dae2ode(dae)`: returns discrete step function `x_aug_next = F(x_aug, u)`. Forward Euler for `x1`, Newton projection for `x2` at both current and next `x1`. `Ts` must be set on `DAESystem`.
- Three static central-difference Jacobian helpers (`algJacX1`, `algJacX2`, `algJacU`) in `PlantModel.cpp`.
- `CTRL_REGISTER_FEATURE(dae_system)` added after `namespace ctrl`.

**P2 — `c2d(DAESystem, x1_op, x2_op, u_op, Ts, method)`** (`lib/PlantModel.h/.cpp`)

- Index-1 algebraic elimination: `A_red = A11 - A12*G2⁻¹*G1`, `B_red = B1 - A12*G2⁻¹*B2` where all Jacobians are computed numerically via `algJac*` helpers.
- Checks `rcond(G2) > 1e-12`; throws `std::runtime_error("c2d(DAESystem): G2 is singular — DAE is not Index-1 at operating point.")` otherwise.
- Output matrix built from `h` Jacobians (or identity w.r.t. `x1` if `h` not set).
- Dispatches to existing `c2d(StateSpace, Ts, method)` for ZOH/Tustin.
- Python binding registered as `dae_c2d` (avoids `py::overload_cast` ambiguity with existing `c2d`).

**P3 — DAE-aware EKF projection** (`lib/ExtendedKalmanFilter.h/.cpp`)

- `setAlgebraicConstraint(g_alg, n_diff, n_alg, tol=1e-9)`: attaches algebraic constraint function; validates `n_diff + n_alg == n_states_`.
- `hasAlgebraicConstraint()`: bool accessor.
- `projectAlgebraicStates(u)`: called at end of `update()` when constraint is set. Newton-Raphson on `x2` block using `numericalJacobian`; then covariance projection `P = J_proj * P * J_proj'` where `J_proj = [[I, 0]; [-G2⁻¹G1, 0]]`.
- SISO assumption: `u_scalar = u(0)` (consistent with `DAESystem::AlgFunc` signature).
- Independent `AlgConstraintFn` type alias in EKF (does not depend on `PlantModel.h`).

**Bindings / tests**

- `plantmodel_bindings.cpp`: `DAESystem` class with `set_f/set_g/set_h`, `consistent_init`, `dae2ode`, `dae_c2d`.
- `estimation_bindings.cpp`: `set_algebraic_constraint`, `has_algebraic_constraint` on `ExtendedKalmanFilter`.
- `smoke_test.py`: 4 DAE assertions (`consistent_init`, `dae2ode`, `dae_c2d`, `registry_has('dae_system')`).
- `tests/test_catch2_advanced.cpp`: 7 Catch2 tests — `[dae_system]` ×3, `[dae_c2d]` ×2, `[dae_ekf]` ×2.

---

## Part 52 — Model Estimation E1/E2/E3 — 2026-06-12

**E1 — `GreyBoxEstimator`** (`lib/GreyBoxEstimator.{h,cpp}`)

- Batch Levenberg-Marquardt for user ODE `f(x,u,p)` and measurement `h(x,p)`.
- RK4 integration with `rk4_steps` substeps per `Ts`; central finite-difference Jacobian
  (step `eps_i = eps_jac * max(|p_i|, 1)` per parameter); box-constrained params via projection.
- LM normal equations: `(J'J + lambda*diag(J'J)) dp = -J'r`; accept/reject; `lambda *= nu` on
  reject, `lambda /= nu` on accept. Convergence: `max|J'r| < tol_grad`.
- `fit(x0, U, Y)` returns `Result{params, cost, iterations, converged}`.
- `predict(x0, U)` returns `Y_hat (n_y x N)` using current `p_est_`.

**E2 — `RecursiveGreyBoxEstimator`** (`lib/RecursiveGreyBoxEstimator.{h,cpp}`)

- Augmented state `z = [x; p]`; `f_aug` integrates ODE (RK4) for `x` and holds `p` constant
  with diffusion `Q_param`. `h_aug(z, u) = h(z.head(n_x), z.tail(n_p))`.
- UKF created at `initialize()` with `P0_aug = blkdiag(P0_state, P0_param)`.
- Compiled only under `CTRL_ENABLE_ADVANCED_KALMAN` (same guard as UKF/EKF).
- Default `alpha=0.1` (not 1e-3) — augmented `n_aug = n_state + n_param >= 3`.

**E3 — `GPResidualModel`** (`lib/GPResidualModel.{h,cpp}`)

- Composition over existing `GaussianProcess`; stores residuals `epsilon = y_true - y_model`.
- `addResidualPoint(xf, y_true, y_model)`: appends `(xf, epsilon)` to GP dataset.
- `residualFit(X_feat, Y_true, model_fn)`: batch version — resets GP, adds all points, calls `fit()`.
- `predictWithUncertainty(xf, model_pred)`: returns `{model_pred + gp_mean, gp_mean, gp_variance}`.
  Returns `{model_pred, 0.0, 0.0}` before first `fit()` call.

**Bindings / tests**

- `estimation_bindings.cpp`: `GreyBoxParams`, `GreyBoxResult`, `GreyBoxEstimator`; `RecursiveGreyBoxParams`,
  `RecursiveGreyBoxEstimator` (inside `CTRL_HAS_ADVANCED_KALMAN` guard).
- `controllers_bindings.cpp`: `GPResidualParams`, `GPResidualPrediction`, `GPResidualModel`.
  `residual_fit` wraps `model_fn` via `py::object` lambda capture.
- `smoke_test.py`: 3 assertion blocks (E1 fit, E2 step, E3 residual + batch).
- `tests/test_catch2_advanced.cpp`: 8 new Catch2 tests — `[grey_box]` ×3, `[recursive_grey_box]` ×2,
  `[gp_residual]` ×3. Bug fix: `Eigen::Vector1d` does not exist — replaced with explicit `VectorXd(1)`.
- `examples/ex80_grey_box_estimator.cpp`; `ex97_grey_box_estimator.py`, `ex98_recursive_grey_box.py`,
  `ex99_gp_residual_model.py`.
