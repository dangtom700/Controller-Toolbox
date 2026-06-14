# Controller Toolbox — AI-Guided Code Audit

**Date:** 2026-06-13  
**Auditor:** Claude Sonnet 4.6 — read-only structural audit, five parallel agent passes  
**Branch:** `main`  
**Scope:** `lib/` (headers + implementations), `tests/`, `examples/`, `bindings/`, `hal/`, `tools/`, `.github/workflows/`, root build files  
**Total findings:** 1 Critical · 13 High · 24 Medium · 45 Low · 0 Security

> **How to read this report.** Every finding includes a severity tag (`[CRITICAL]`/`[HIGH]`/`[MEDIUM]`/`[LOW]`), a confidence tag (`[Confidence: High/Medium/Low]`), the exact file path and approximate line number, a description of the problem, and a suggested fix. No finding duplicates the three currently open issue IDs (**D2**, **C2**, **REL**) from `docs/cumulative_bug_report.md`.

---

## Project State Snapshot

### Currently Open Issues (from `docs/cumulative_bug_report.md`)

| ID  | Description | Priority |
|-----|-------------|----------|
| D2  | Digital Twin Lite — FastAPI/Streamlit dashboard | LOW |
| C2  | 8 spec-only case-study stubs remain (BEMS, MEMS, etc.) | MED |
| REL | Rebuild `ctrl_toolbox.pyd` in Release to silence QP warnings | LOW |

All other tracked items (P1–P3, E1–E4, H1–H4, D1, M4, C3–C6, B36-x, R1, T1–T7, A1–A11, G2–G3, M2–M3) are confirmed closed.

### Test Baseline *(unverified until a clean `run.py` run)*

| Suite | Target |
|-------|--------|
| C++ Catch2 | ~174 tests |
| Python examples | ~100 scripts |
| C++ case studies (9) | Boiler 216, Tug 72, Solar 70, Humid 75, ActiveSusp 90, BuckBoost 60, SolarCooker 60, SOTEC 60, SMISMO 65 |
| Python-only studies (7) | DrillString 85, WindWave 80, EHFS 70, FirefightingBagDrop 60, BTMS 60, SurfaceShip 60, EV6x6 90 |

### TODO / FIXME / HACK Comments
Full scan of all `lib/` headers and implementation files returned **zero** occurrences. The codebase is clean of inline tracked debt markers.

### God Classes and Deep Inheritance
No class with more than 20 public methods was found. The closest are `GainScheduledController` (~12) and `DiscreteMPC` (~11). No inheritance chain deeper than 2 levels was found.

---

## Category 1 — Code Correctness & Safety

---

### `[CRITICAL]` `[Confidence: High]`  
**`lib/DeePC.h` / `lib/DeePC.cpp` — Neither file exists**

`CLAUDE.md` Part 30 describes a full 8-step implementation of `ctrl::DeePC` including `lib/DeePC.{h,cpp}`. Part 39 contradicts this, noting feature registration was removed because "DeePC has no implementation." A filesystem search confirms both files are absent. The C++ example `examples/ex69_deepc.cpp` will fail to link. Any Python example still referencing `ctrl.DeePC` will fail at attribute lookup. There is no `[deepc]` Catch2 test section.

**Fix:** Either implement `lib/DeePC.{h,cpp}` per the Part 30 spec and add it to `lib/CMakeLists.txt` and `lib/ControllerToolbox.h`, or remove `examples/ex69_deepc.cpp` and any Python example referencing DeePC and add DeePC to the `C2` spec-stub list.

---

### `[HIGH]` `[Confidence: High]`  
**`lib/DifferentialEvolution.h` ~line 133 — Potential infinite loop in boundary reflection**

```cpp
while (v[j] < p_.lower[j] || v[j] > p_.upper[j]) {
    if (v[j] < p_.lower[j]) v[j] = 2.0 * p_.lower[j] - v[j];
    if (v[j] > p_.upper[j]) v[j] = 2.0 * p_.upper[j] - v[j];
}
```

When `lower[j] == upper[j]` (zero-width bound) or when a large mutation places `v[j]` simultaneously outside both bounds, the two reflection clauses alternate indefinitely. No iteration limit exists. The optimizer hangs permanently with no diagnostic output.

**Fix:** Cap the loop at a fixed number of iterations (e.g. 20), then hard-clamp:
```cpp
int reflect_iters = 0;
while ((v[j] < p_.lower[j] || v[j] > p_.upper[j]) && ++reflect_iters < 20) {
    if (v[j] < p_.lower[j]) v[j] = 2.0 * p_.lower[j] - v[j];
    if (v[j] > p_.upper[j]) v[j] = 2.0 * p_.upper[j] - v[j];
}
v[j] = std::clamp(v[j], p_.lower[j], p_.upper[j]);
```

---

### `[HIGH]` `[Confidence: High]`  
**`lib/DiscreteLQR.h` ~line 186 — MIMO truncation warning suppressed in Release builds**

The one-shot `warned_mimo_` diagnostic that notifies callers their MIMO LQR output is being silently truncated to `u[0]` is inside `#ifndef NDEBUG`. In Release builds, a caller using a MIMO plant receives only the first control output with no indication that remaining outputs are discarded. This is silent data loss in production.

**Fix:** Remove the `#ifndef NDEBUG` guard. The warning fires at most once per instance (gated by `warned_mimo_`), so the runtime cost is negligible.

---

### `[HIGH]` `[Confidence: High]`  
**`lib/ExtendedKalmanFilter.cpp` ~line 126 — Unchecked LDLT solve in DAE covariance projection**

`projectAlgebraicStates()` re-factorises `G2` for the covariance projection update without calling `.info()` to verify the factorisation succeeded. If `G2` is ill-conditioned (possible for stiff DAE systems), the projection silently populates the covariance matrix with garbage values, corrupting all subsequent state estimates without any observable failure signal.

**Fix:** Check the factorisation result before using it:
```cpp
auto ldlt = G2.ldlt();
if (ldlt.info() != Eigen::Success) {
    dae_projection_failed_ = true;
    return; // leave P unchanged
}
P_ = J_proj * P_ * J_proj.transpose();
```

---

### `[HIGH]` `[Confidence: High]`  
**`lib/L1AdaptiveController.h` ~line 112 — Dead private member `use_compute_y_`**

`bool use_compute_y_ = false;` is declared private, never assigned, and never read anywhere in the header or (verified) in the implementation. The surrounding comment references "flag to distinguish compute(y) vs compute(error) calls" — indicating an abandoned feature branch whose state was partially removed and whose field was left behind.

**Fix:** Remove the field entirely. The dead state misleads readers about intended dual-mode dispatch.

---

### `[HIGH]` `[Confidence: High]`  
**`lib/HybridModel.h` ~line 169 — `dynamicsFunc()` returns a lambda capturing raw `this`**

The non-static `dynamicsFunc()` method returns a `std::function` whose lambda implicitly captures `this` as a raw pointer. A comment acknowledges callers must ensure the `HybridModel` outlives the returned lambda. Any `NonlinearMPC` or `ExtendedKalmanFilter` that stores the lambda and is then destroyed after the `HybridModel` triggers silent undefined behaviour with no diagnostic.

**Fix:** Deprecate `dynamicsFunc()` in favour of the existing `makeDynamicsFunc(shared_ptr<HybridModel>)` static method, which captures a `shared_ptr` and extends the object lifetime. Add `[[deprecated("use HybridModel::makeDynamicsFunc(shared_ptr<HybridModel>)")]]` to the instance method.

---

### `[HIGH]` `[Confidence: High]`  
**`lib/NonlinearMPC.cpp` ~lines 115–129 — Per-step heap allocation in `buildAndSolve()`**

Every `computeRef()` call (the real-time control step) allocates `x_traj` (a `std::vector<VectorXd>` of size Np+1), `A_list` (size Np), and `B_list` (size Np) on the heap, plus full `MatrixXd` temporaries for each Jacobian call. For Np=20 and n=5 this is O(Np × n²) allocations per control cycle at the real-time rate.

**Fix:** Pre-allocate `x_traj_`, `A_list_`, `B_list_` as member vectors sized once in `init()` or `setHorizon()`; fill them in-place inside `buildAndSolve()` without re-allocation.

---

### `[HIGH]` `[Confidence: High]`  
**`lib/MovingHorizonEstimator.cpp` ~lines 111–178 — Per-step matrix allocation in `estimate()`**

Every `estimate()` call allocates approximately 14 local `Eigen::MatrixXd` / `VectorXd` objects on the heap: `Psi_eff`, `Gamma_u_eff`, `C_bar_eff`, `Y_hist`, `U_hist_eff`, `H_prior`, `R_bar`, `f_eff`, `lb_eff`, `ub_eff`, and more. The full-horizon variants (`Psi_`, `C_bar_`, etc.) are correctly cached as members, but the effective-horizon ramp-up path recreates them from scratch on every step.

**Fix:** Pre-allocate all matrices at full-horizon size in `buildCondensedMatrices()`; use `Eigen::Block` views inside `estimate()` to slice to `eff_N` without copying. Once `eff_N == N` the cached full-horizon matrices can be reused with zero new allocation.

---

### `[HIGH]` `[Confidence: High]`  
**`lib/GradientProjectionQP.h` ~line 97 — `VectorXd` heap allocation on every QP solve**

`Eigen::VectorXd y = x;` creates a heap-allocated copy at the start of every FISTA solve call. Since every MPC / TubeMPC / ScenarioMPC / GPC `compute()` invokes the QP solver, this is one unavoidable heap allocation per control step for every QP-based controller. The comment "zero-allocation hot-path (iterations)" is misleading — only the inner FISTA iterations are allocation-free; the outer call is not.

**Fix:** Accept a pre-allocated workspace buffer via `Eigen::Ref<Eigen::VectorXd> workspace`, or store `y` as a `mutable` member of a solver context struct that callers instantiate once.

---

### `[HIGH]` `[Confidence: High]`  
**`lib/ScenarioMPC.cpp` ~lines 55–90 — Per-step allocation in the N_samples × Np loop**

`computeControl()` allocates `VectorXd x_noise(n_)` and `VectorXd z(n_)` inside the N_samples outer loop on every control step, plus `VectorXd R_stacked(Np*pp_)` per call. For N_samples=30, Np=10, n=2 this is more than 60 allocations per control step.

**Fix:** Pre-allocate `x_noise_`, `z_work_`, and `R_stacked_` as member vectors; size them in the constructor and reuse each step.

---

### `[HIGH]` `[Confidence: High]`  
**`lib/TubeMPC.cpp` ~lines 45–55 — Per-step output-stacked vector allocation**

`computeControl()` allocates `VectorXd R_stacked(Np*pp_)` and `VectorXd Qy_err(Np*pp_)` on every call despite their sizes being fixed at construction time (`Np` and `pp_` do not change at runtime).

**Fix:** Promote both to member variables and allocate them once in the constructor.

---

### `[HIGH]` `[Confidence: High]`  
**`lib/KalmanFilter.cpp` ~lines 36–56 — Four matrix temporaries per `update()` call**

`update()` allocates `R_safe` (p×p copy of `R_`), `S` (p×p), `Kf` (n×p), and `IKC` (n×n) on the heap every call. For a 1 kHz embedded controller with n=4, p=2 this is four small heap allocations per step.

**Fix:** Pre-allocate `R_safe_`, `S_`, `Kf_`, `IKC_` as member matrices sized in the constructor; use `.noalias()` for in-place products.

---

### `[HIGH]` `[Confidence: High]`  
**`bindings/controllers_bindings.cpp` ~line 388 — `DiscreteLQG` missing `std::shared_ptr<T>` holder**

`py::class_<ctrl::DiscreteLQG>` is registered without a `shared_ptr` holder type. Any Python code that stores a `DiscreteLQG` inside a `ControllerStack` or passes it to a C++ API expecting `shared_ptr<IController>` will produce a `std::bad_cast` exception or a use-after-free crash at runtime.

**Fix:**
```cpp
py::class_<ctrl::DiscreteLQG, ctrl::IController, std::shared_ptr<ctrl::DiscreteLQG>>(m, "DiscreteLQG")
```

---

### `[HIGH]` `[Confidence: High]`  
**`bindings/controllers_bindings.cpp` (multiple locations) — 8 ML / optimizer classes missing `std::shared_ptr<T>` holder**

`AutoTuner`, `BayesianOptimizer`, `GeneticAlgorithm`, `ParticleSwarmOptimizer`, `DifferentialEvolution`, `KoopmanEDMD`, `SINDy`, `GaussianProcess`, `EchoStateNetwork`, and `ILC` are all bound without `std::shared_ptr<T>` as the third template argument. Any attempt to pass these objects into C++ APIs that hold a `shared_ptr` raises:  
`RuntimeError: Unable to load a custom holder type from a default-holder instance.`

**Fix:** Add `std::shared_ptr<T>` as the third template argument to all affected `py::class_<>` registrations. This is a one-line change per class.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/DiscreteLQG.h` — `DiscreteLQG` does not inherit `IController`**

`DiscreteLQG` exposes `compute(double y)`, `reset()`, and `sampleTime()` but does not inherit `IController`. It therefore cannot be placed in a `ControllerStack`, passed to `AutoTuner`, `GainScheduledController`, `AntiWindupWrapper`, or any function accepting `shared_ptr<IController>`. A `ControllerTraits<DiscreteLQG>` specialisation exists, implying tuner compatibility, but the class cannot fulfil the base interface contract. The Python binding (see HIGH finding above) compounds this by also lacking the correct holder.

**Fix (option A):** Add `: public IController`, implement the hold-last NaN-guard `compute()` contract, and note the non-standard sign convention (`compute(y)` not `compute(error)`) in the override comment.  
**Fix (option B):** Document clearly in the class Doxygen that `DiscreteLQG` is deliberately not an `IController`, and specify the recommended composition pattern for users who need it in a stack.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/PlantModel.h` — `consistentInit()` returns last Newton iterate with no convergence signal**

The function `Eigen::VectorXd consistentInit(...)` returns the last Newton-Raphson iterate with no convergence flag and no `[[nodiscard]]` attribute. A caller that does not manually check the residual will silently use an unconverged algebraic initial condition, corrupting the entire subsequent simulation without any observable error.

**Fix:** Change the return type to a small struct:
```cpp
struct ConsistentInitResult {
    Eigen::VectorXd x;
    bool converged;
    double residual;
};
[[nodiscard]] ConsistentInitResult consistentInit(...);
```

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/VectorFitting.cpp` ~line 246 — Complex poles silently stripped to real parts**

`es.eigenvalues()(k).real()` discards the imaginary component of companion-matrix eigenvalues. For real input data with resonant poles (which produce complex-conjugate pairs), this produces incorrect real-only poles. `enforcePoleStability` then clips these, compounding the error. The caller receives a silently wrong rational fit.

**Fix:** Handle conjugate pairs explicitly: for `|imag| > threshold`, store both `real ± j*imag` as a pair and add both to the pole list. For the real-only path, assert `|imag| < threshold` and emit a `std::clog` warning otherwise.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/BayesianOptimizer.h` ~line 230 — `maximiseAcquisition()` declared `const` but mutates RNG state**

`maximiseAcquisition()` is declared `const` and advances `rng_` which is declared `mutable` to paper over this. The method has a meaningful side effect (advancing the random state) and should not be `const`. The identical pattern exists in `lib/ScenarioMPC.h` ~line 226 where `rng_` and `normal_` are `mutable` but their actual caller `computeControl()` is non-`const`.

**Fix:** Remove `const` from `maximiseAcquisition()`; remove `mutable` from `rng_` in both `BayesianOptimizer` and `ScenarioMPC`. This corrects the false signal that these methods are pure observers.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/HybridMPC.cpp` ~lines 25–29 — Unbounded training data buffer growth**

`addStateObservation()` appends to `feat_data_` and `resid_data_` on every step without a capacity bound. `refitDataModel()` resets the refit counter but does not trim the vectors. Over a long simulation both vectors grow without limit, causing steadily increasing memory usage and a progressively more expensive `GPResidualModel::residualFit()` call.

**Fix:** Cap at a configurable `max_buffer` size (default: `hp_.min_observations * 2`) using FIFO eviction (erase front or use a circular index), consistent with how `GaussianProcess` manages its own budget via `n_max`.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/GainScheduledController.h` ~line 232 — `lowerIndex()` helper relies on unstated non-empty invariant**

The private helper `lowerIndex()` calls `schedule_.front()` and `schedule_.back()` without guarding against an empty `schedule_`. The outer `compute()` method does guard `if (schedule_.empty()) return 0.0;`, but `lowerIndex()` relies on the caller maintaining this invariant — an assumption that is one refactor away from becoming a silent out-of-bounds crash.

**Fix:** Add `assert(!schedule_.empty())` at the top of `lowerIndex()` to make the invariant explicit and catchable in debug builds.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/CEMController.cpp` ~line 103 — Silent zero return on unconfigured call**

When `x_.size() == 0 || r_.size() == 0`, `compute()` silently returns `0.0`. No diagnostic is emitted, even in debug builds. A user who forgets to call `setState()` or `setReference()` before the control loop gets zero outputs for the entire run with no error indication.

**Fix:** Add a debug diagnostic:
```cpp
#ifndef NDEBUG
if (x_.size() == 0 || r_.size() == 0) {
    std::clog << "[CEMController] WARNING: compute() called before setState/setReference — returning 0\n";
}
#endif
```

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/MovingHorizonEstimator.cpp` ~line 212 — General eigenvalue solver used on symmetric positive-definite matrix**

The effective-horizon path calls `H_eff.eigenvalues().real().maxCoeff()` which invokes `Eigen::EigenSolver` — the general complex-eigenvalue algorithm — on `H_eff`, a symmetric positive-definite matrix. This performs unnecessary complex arithmetic and discards the imaginary parts.

**Fix:**
```cpp
double L_eff = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(H_eff)
                   .eigenvalues().maxCoeff();
```
This is strictly faster, numerically cleaner, and guaranteed to return real eigenvalues.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/GaussianProcess.cpp` ~line 24 — O(N) `erase` at front in budget eviction path**

`addPoint()` performs `X_.erase(X_.begin())` when the budget is full — an O(N) operation that shifts all N training vectors on every online update. For `n_max=200` at 1 kHz this is 200 `VectorXd` moves per second.

**Fix:** Replace `std::vector<VectorXd> X_` with a ring-buffer backed by a pre-allocated `std::vector` and a head/tail index, or use `std::deque<VectorXd>` where `pop_front()` is O(1).

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/CEMController.cpp` ~lines 61–70 — Per-step vector allocation in `computeRef()`**

Every `computeRef()` call allocates `samples` (a `std::vector<VectorXd>` of size N_samples × Np), `costs` (N_samples), `new_mu` (Np), and `u_k(1)` for every time step in every rollout (N_samples × Np allocations per control step for the inner rollout alone).

**Fix:** Pre-allocate `samples_`, `costs_`, `new_mu_`, and `u_k_` as member vectors sized in the constructor; index into them each step without re-allocation.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/ParticleFilter.cpp` ~lines 140–145 — Per-resample heap allocation in `resample()`**

`resample()` allocates `new_particles` (a `std::vector<VectorXd>` of size N) and `cdf` (a `std::vector<double>` of size N) every time the ESS threshold fires. For N=1000 particles running at 100 Hz this is a real-time hazard.

**Fix:** Pre-allocate `resample_buf_` and `cdf_` as class members sized in the constructor; fill them in-place and swap with the live particle buffer.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/NeuralPID.cpp` ~lines 76–108 — Per-step gradient matrix temporaries**

Every `compute()` call allocates 7 heap objects: the hidden activation vector, `dW2`, `db2`, `dJ_dh`, `dJ_dpre1`, `dW1`, and `db1`. Eigen's expression templates generate these as temporaries from outer-product expressions. At 1 kHz with n_hidden=10 this is ~70 small allocations per second.

**Fix:** Promote all gradient matrices and vectors to member variables; resize once in the constructor. Replace all outer-product expressions with `.noalias() =`:
```cpp
dW2_.noalias() = dJ_dpre_ * h_.transpose();
dW1_.noalias() = dJ_dpre1_ * z_.transpose();
```

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/CEMController.h` ~line 79 — `Eigen::MatrixXd C` passed by value in constructor**

The output matrix `C` is taken by value in the constructor signature, triggering an unnecessary heap-allocated copy on every `CEMController` construction.

**Fix:** Change to `const Eigen::MatrixXd& C` or use move semantics: `Eigen::MatrixXd C` with `C_(std::move(C))` in the initialiser list.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/EchoStateNetwork.cpp` ~line 80, `lib/SINDy.cpp` ~line 116, `lib/KoopmanEDMD.cpp` ~line 132 — `push_back` without `reserve` in data-collection loops**

All three call `push_back` on training data vectors (`states_`, `targets_`, `theta_rows_`, `xdot_rows_`, `psi_k_`, `psi_k1_`, `x_k1_`) inside per-step loops without prior `reserve()`. Over thousands of snapshots this causes O(log N) vector reallocations and the associated copy cascades.

**Fix:** Expose a `reserveSnapshots(int N)` method on each class; call `reserve(N)` before the training loop begins. Document in the Python binding docstring.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/DiscreteHinf.cpp` — `VectorXd` temporary allocated every step in `computeVec()`**

`computeVec()` contains `Eigen::VectorXd u = Ck_ * xk_ + Dk_ * y` — one heap allocation per control step for the output vector.

**Fix:** Add a member `Eigen::VectorXd u_work_` sized in the constructor; replace with:
```cpp
u_work_.noalias() = Ck_ * xk_ + Dk_ * y;
```

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/SINDy.cpp` — `SINDyModel::predict()` constructs a full `SINDy` object per call**

`SINDyModel::predict()` instantiates a full `SINDy helper(p_tmp)` object on every prediction call, running the complete library setup from scratch. In a CEM rollout (N_samples=50 × Np=20) this is 1000 full `SINDy` constructions per control step.

**Fix:** Cache the evaluated library matrix at `SINDy::fit()` time inside `SINDyModel` and use a direct matrix-vector multiply in `predict()`. Alternatively, store a `mutable` pre-allocated helper and reset it rather than reconstructing.

---

### `[MEDIUM]` `[Confidence: Medium]`  
**`lib/RecursiveGreyBoxEstimator.cpp` — O(2n_aug+1) `VectorXd` allocations per estimator step**

The `f_aug` lambda called inside the UKF predict step allocates `VectorXd x(n_state)` and `VectorXd z_next(n_aug)` for each of the 2n_aug+1 sigma points. For n_state=4, n_param=3, n_aug=7 this is 15 small heap allocations per estimator step.

**Fix:** Pre-allocate `x_scratch_` and `z_scratch_` scratch buffers in the closure at `initialize()` time and capture them by reference into the lambda.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/GaussianProcess.h` ~line 93 — Member `L_chol_` is `Eigen::LDLT`, not Cholesky**

The member is declared `Eigen::LDLT<MatrixXd>` but named `L_chol_` and the comment says "Cholesky of training covariance." `LDLT` is a diagonal-pivoting rank-revealing decomposition, not the standard lower-triangular Cholesky (`LLT`). The mismatch misleads readers about numerical stability properties.

**Fix:** Rename the member to `K_ldlt_` and update the comment to: "LDLT decomposition of training covariance matrix (used for numerically stable solve on near-singular kernels)."

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/MismatchDetector.h` ~line 94 — Zero-size innovation fallback is undocumented**

When `innov.size() == 0`, the code feeds `0 / sqrt(1) = 0` to the CUSUM, silently producing no alarm update. This defensive path is undocumented; callers have no way to know that a zero-size innovation is silently absorbed rather than flagged.

**Fix:** Add a comment directly above the guard:
```cpp
// Zero-size innovation: treated as zero normalised residual (no CUSUM update).
// This occurs when the estimator has not yet received an observation.
```

---

### `[MEDIUM]` `[Confidence: High]`  
**`bindings/controllers_bindings.cpp` ~lines 1872–1879 — Stale TODO Fuzzy block duplicates real bindings**

A second `#if defined(CTRL_HAS_FUZZY)` block at the bottom of the file contains only TODO comments and `(void)m;`. The Fuzzy bindings are already fully and correctly implemented in `bindings/advanced_bindings.cpp`. This dead block suppresses the "unused variable" warning that would otherwise signal a missing binding, and misleads readers into thinking Fuzzy work is incomplete.

**Fix:** Delete the entire dead block at lines 1872–1879 of `controllers_bindings.cpp`.

---

### `[MEDIUM]` `[Confidence: Medium]`  
**`bindings/advanced_bindings.cpp` ~lines 194–197 — `reference_internal` on `FuzzySystem` member vectors risks dangling reference**

`FuzzySystem::input_var(i)` and `output_var(i)` are bound with `py::return_value_policy::reference_internal`. A subsequent `add_input()` or `add_output()` call may trigger a reallocation of the internal `std::vector`, leaving the returned Python object pointing to freed memory.

**Fix:** Use `py::return_value_policy::copy` to return a value copy of `LinguisticVariable`. Alternatively add a docstring warning that `add_input`/`add_output` must not be called after taking a variable reference.

---

### `[MEDIUM]` `[Confidence: High]`  
**`bindings/estimation_bindings.cpp` ~lines 436–441 — Dead SubspaceID TODO stub block**

A `#if defined(CTRL_HAS_SUBSPACE) // TODO: bind ...` block inside `bind_estimation` is a stale comment-only block. The actual SubspaceID bindings are correctly located in `bind_advanced`. This dead block creates confusion about the canonical location of the bindings.

**Fix:** Remove the TODO comment block from `estimation_bindings.cpp`.

---

### `[MEDIUM]` `[Confidence: High]`  
**`.github/workflows/ubuntu.yml` + `windows.yml` — Python bindings never built or tested in CI**

Neither CI workflow passes `-DCTRL_BUILD_PYTHON_BINDINGS=ON` to CMake or runs `bindings/smoke_test.py`. The entire Python binding layer is untested in CI. Binding regressions (the missing `shared_ptr` holders found above, a breaking pybind11 version bump) are completely invisible.

**Fix:** Add a CI step in at least the Ubuntu workflow:
```yaml
- name: Build Python bindings
  run: |
    cmake -S . -B build -DCTRL_BUILD_PYTHON_BINDINGS=ON -G Ninja
    cmake --build build --target ctrl_toolbox
- name: Run smoke test
  run: python bindings/smoke_test.py
```

---

### `[MEDIUM]` `[Confidence: High]`  
**All four `.github/workflows/` files — No static analysis or sanitizer runs**

None of the four CI workflows run `clang-tidy`, `cppcheck`, or any sanitizer (ASan/UBSan). For an Eigen-heavy ~85-algorithm library under active development, this leaves undefined behaviour, unused-variable accumulation, and style drift unchecked by automation.

**Fix:** Add a `clang-tidy` step to `ubuntu.yml`:
```yaml
- name: Static analysis
  run: run-clang-tidy -p build/ -quiet
```
Or add a separate `sanitize.yml` that builds with `-fsanitize=address,undefined` and runs the full Catch2 suite.

---

## Category 2 — Test Coverage & Quality

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/LPVSystemID` — Zero Catch2 tests**

`LPVSystemID` has a documented non-obvious API convention (`identifyLPV` takes an `n × N` column-major matrix, not the `N × n` row-major form) that has caused bugs in the past (recorded in CLAUDE.md tribal knowledge). No `[lpv]` tag appears anywhere in any test file.

**Fix:** Add at least two `[lpv]` Catch2 tests: one verifying correct identification on synthetic column-major data, and one deliberately using the wrong orientation and asserting the result diverges — to document the convention through failure.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/FunctionApproximator` — Zero Catch2 tests**

`FunctionApproximator` (RBFN + polynomial approximation) is used internally by other controllers. A silent regression in its training or prediction path propagates undetected into all dependent controllers.

**Fix:** Add `[func_approx]` Catch2 tests covering construction, training on a known 1D function (e.g. `sin(x)`), and prediction accuracy within a tolerance.

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/MetricsAnalyzer` — Zero Catch2 tests**

`MetricsAnalyzer` computes IAE, ISE, ITAE, settling time, and overshoot — the primary performance metrics used by every case study's comparative table. A silent off-by-one in the integration step or a wrong settling-time threshold would corrupt every case-study comparison without any test catching it.

**Fix:** Add `[metrics]` Catch2 tests verifying IAE/ISE/ITAE/settling-time on a known analytic step response where ground truth is calculable (e.g. a first-order step with known time constant).

---

### `[MEDIUM]` `[Confidence: High]`  
**`lib/ZeroPhaseTrackingFilter` — No test of the phase-correction property**

The test suite covers `designZPETC` and `transmissionZeros`, but no test instantiates `ZeroPhaseTrackingFilter` and verifies that a sinusoid passed through the filter has near-zero phase shift — the filter's entire purpose.

**Fix:** Add a `[zero_phase]` Catch2 test: drive a known-frequency sinusoid through the filter and assert the peak-to-peak phase error is below 1°.

---

### `[LOW]` `[Confidence: High]`  
**Tests — No standalone `[pid]` test section**

`DiscretePID` is exercised indirectly via `[anti_windup]`, `[health_contract]`, and integration tests, but no focused `[pid]` section tests PID-specific behaviours: derivative-on-measurement vs. `compute()`, `Kb` anti-windup activation, the `N`-filter decay rate, or the `b_weight` two-degree-of-freedom setpoint weight.

**Fix:** Add a `[pid]` section with 3–4 targeted tests for these behaviours.

---

### `[LOW]` `[Confidence: High]`  
**Tests — `RepetitiveController` missing Catch2 edge-case coverage**

Only legacy hand-rolled tests exist for `RepetitiveController`. There is no Catch2 coverage for: `periodSteps < 1` throwing an exception, NaN input triggering hold-last output, or `setParams` with a changed period correctly resetting the internal buffer.

**Fix:** Add a `[repetitive]` Catch2 section covering these three edge cases.

---

### `[LOW]` `[Confidence: High]`  
**Tests — `DiscretePID::computeDoM` has no comparative test**

`computeDoM()` (derivative-on-measurement variant) exists and is demonstrated in examples, but no test asserts it produces strictly lower peak overshoot than `compute()` on the same step input — which is the sole justification for the variant's existence.

**Fix:** Add a `[pid][dom]` test running both modes on the same simulated plant and asserting that DoM peak overshoot is strictly lower.

---

### `[LOW]` `[Confidence: Medium]`  
**Tests — `ExtremumSeeker` has no convergence test**

Existing tests only verify that outputs are finite. No test confirms the ESC actually locates the optimum of a known cost function such as `J(θ) = (θ − 2)²` within tolerance after N steps.

**Fix:** Add an `[extremum_seeker]` convergence test with a closed-form quadratic cost and an assertion on the final `θ` estimate.

---

### `[LOW]` `[Confidence: High]`  
**`tests/test_catch2_advanced.cpp` ~line 2998 — Vague test name: `"DynaController wraps inner controller"`**

The test body calls `compute()` once and checks only `std::isfinite()`. The name promises validation of inner-controller delegation, but the test would pass even if `DynaController::compute()` always returned `0.0`.

**Fix:** Rename to `"DynaController single compute() returns finite bounded output"` and add an assertion that the output is within the expected range for the given input error.

---

## Category 3 — Documentation & Comments

---

### `[LOW]` `[Confidence: High]`  
**`lib/KoopmanEDMD.h` ~line 26 — `DiscreteLQG` listed twice in class Doxygen (copy-paste error)**

The class-level Doxygen example reads: "…with `DiscreteMPC`, `DiscreteLQR`, `DiscreteLQG`, and `DiscreteLQG`." The final entry is a copy-paste duplicate. The intended fourth controller is likely `DiscreteHinf`.

**Fix:** Replace the second `DiscreteLQG` with `DiscreteHinf`.

---

### `[LOW]` `[Confidence: High]`  
**`lib/DiscreteSMC.h` — `slidingSurface()` returns `s_prev_` (previous step), not current**

`slidingSurface() const` returns `s_prev_` which is the surface value at step k−1. Every other accessor that callers expect to return "the current value" does so immediately after `compute()`. A caller querying `slidingSurface()` after `compute(error)` receives a stale one-step-old value with no indication.

**Fix:** Either rename to `previousSlidingSurface()` to match the actual semantics, or restructure `compute()` to store the current surface in a `surface_` member and return that from `slidingSurface()`.

---

### `[LOW]` `[Confidence: Medium]`  
**`lib/ParticleFilter.h` ~line 220 — Comment says "log-sum-exp weights" but stores raw probabilities**

`///< Normalised log-sum-exp weights (N)` on `Eigen::VectorXd w_` is contradictory. The `state()` implementation (`x_hat = Σ wᵢ * xᵢ`) confirms that raw normalised probability weights are stored, not log-space values.

**Fix:** Change the comment to: `///< Normalised probability weights (N), each in [0, 1], sum = 1.`

---

### `[LOW]` `[Confidence: High]`  
**`lib/SystemAnalysis.h` — O(n⁶) complexity not in `solveDiscreteLyapunov()` Doxygen**

The O(n⁶) complexity warning appears only in the file-level `@par Complexity note` section, not in the `solveDiscreteLyapunov()` function's own Doxygen block. Users calling the function directly without reading the file header will not see the constraint.

**Fix:** Add `@note O(n⁶) via Kronecker vectorisation. Suitable only for n ≤ 10.` to the function's own Doxygen block.

---

### `[LOW]` `[Confidence: High]`  
**`lib/DiscreteADRC.h` — `compute(error)` requires prior `setReference()` with no runtime assertion**

If `compute(error)` is called without `setReference(r)`, the controller silently treats `r_ref_ = 0`. The contract is only in a `@warning` tag. No default value is documented in the constructor comment, and no assertion guards the call order, unlike the analogous `MRACController` pattern.

**Fix:** Document the default: `r_ref_ = 0.0` in the constructor comment. Optionally add a `#ifndef NDEBUG` assertion on first call if no reference has been set.

---

### `[LOW]` `[Confidence: Medium]`  
**`lib/AutoGainScheduler.h` — `findEquilibrium()` 1000× tolerance fallback emits no diagnostic**

After the Newton-Raphson loop fails to converge, a fallback accepts residuals up to `tol * 1e3` without any warning to the caller. A poorly converged equilibrium produces an inaccurate linearisation point and a poorly tuned scheduled controller — silently.

**Fix:** Emit a `std::clog` warning stating the actual residual and that the loose-tolerance fallback was used. Consider throwing `std::runtime_error` instead to make failure non-ignorable.

---

### `[LOW]` `[Confidence: High]`  
**`lib/EchoStateNetwork.h` ~line 119 — `W_out_` declared size contradicts architecture documentation**

The header documents "readout uses reservoir state `r` only (not the extended `[r; u]` form)" but `W_out_` is declared at `n_out × (n_res + n_in)`. `fitReadout()` presumably resizes to `n_res` columns, but the discrepancy between declaration and documentation misleads readers about the actual matrix dimensions.

**Fix:** Audit `fitReadout()` to confirm the resized dimensions; then either correct the declaration to `n_out × n_res` or update the architecture comment to explain why the wider pre-allocation exists.

---

### `[LOW]` `[Confidence: Medium]`  
**`lib/DiscreteMPC.h` — `lastQPConverged()` and `isHealthy()` lack `[[nodiscard]]`**

Silently ignoring the QP convergence flag is a latent correctness hazard. Both methods return values that callers should not discard. The same applies to `GeneralizedPredictiveControl.h::lastQPConverged()`.

**Fix:** Add `[[nodiscard]]` to all four method declarations.

---

## Category 4 — Design & Architecture

---

### `[LOW]` `[Confidence: High]`  
**`lib/BayesianOptimizer.h` ~line 183 and `lib/ScenarioMPC.h` ~line 226 — Unnecessary `mutable` on RNG members**

In both classes, `rng_` (and `normal_` in `ScenarioMPC`) are declared `mutable`. Their actual call sites (`tune()`, `computeControl()`) are already non-`const`. The `mutable` is therefore superfluous and provides a false signal that these members are used from `const` paths. (Root cause: the `const`-incorrectness of `maximiseAcquisition()` described in Category 1 above.)

**Fix:** After removing `const` from the calling methods, remove `mutable` from both RNG members.

---

No circular include chains, no inheritance depth greater than 2 levels, and no god classes (maximum ~12 public methods per class) were found across all `lib/` headers.

---

## Category 5 — Performance & Efficiency

Performance findings are consolidated in Category 1 above where they overlap with correctness concerns. Additional lower-severity performance items follow.

---

### `[LOW]` `[Confidence: High]`  
**`lib/DiscreteSMC.cpp` ~line 39, `lib/DiscreteMPC.cpp` ~line 190, `lib/GeneralizedPredictiveControl.cpp` ~line 174 — 1-element `VectorXd` heap-allocated per step for observer notification**

`notifyObserverState("surface", Eigen::VectorXd::Constant(1, s))` constructs a temporary 1-element `VectorXd` on the heap every control step, even when no observer is attached. Multiplied across all SMC and MPC instances in a simulation, this adds measurable allocation pressure.

**Fix (two parts):**  
1. Pre-allocate `Eigen::VectorXd notify_buf_{1}` as a member; assign `notify_buf_(0) = s` before calling `notifyObserverState`.  
2. Wrap all `notifyObserverState` calls in `if (observer_)` to skip both buffer fill and virtual dispatch when no observer is registered.

---

### `[LOW]` `[Confidence: High]`  
**`lib/VectorFitting.cpp` ~line 244 — `EigenSolver` object reconstructed every SK iteration**

`Eigen::EigenSolver<MatrixXd> es;` is declared inside the SK iteration loop, causing full solver object construction and destruction on every iteration.

**Fix:** Move the declaration outside the loop; call `es.compute(companion, false)` inside.

---

### `[LOW]` `[Confidence: Medium]`  
**`lib/UnscentedKalmanFilter.cpp` ~lines 56–68 — `sigmaPoints()` returns large matrix by value per step**

`sigmaPoints()` returns an `n × (2n+1)` matrix by value and is called once per `step()`. For n=10 this is a 10×21 matrix allocated per filter update. NRVO may elide the copy on modern compilers, but storing the sigma-point matrix as a pre-allocated member eliminates the allocation entirely.

**Fix:** Add `Eigen::MatrixXd X_sigma_` as a member sized in `init(n)`; fill it in-place inside `sigmaPoints()` via an output-parameter form or by making `sigmaPoints()` fill the member directly.

---

### `[LOW]` `[Confidence: Medium]`  
**`lib/AdaptiveSmithPredictor.cpp` ~line 93 — Deque never shrinks if `bufferLen == 0`**

`compute()` calls `push_back` on `u_hist_` and `y_hist_` unconditionally; if `bufferLen == 0` the `pop_front` guard never fires and both deques grow indefinitely.

**Fix:** Guard the push/pop pair:
```cpp
if (bufferLen > 0) {
    u_hist_.push_back(u);
    if (u_hist_.size() > static_cast<size_t>(bufferLen)) u_hist_.pop_front();
}
```

---

### `[LOW]` `[Confidence: High]`  
**`lib/EchoStateNetwork.cpp` — `reservoirStateFunc()` copies entire ESN by value**

`reservoirStateFunc()` returns `[self = *this](...){}` — a by-value copy of the entire `EchoStateNetwork` object including `W_res_` (n_res×n_res). For n_res=200 this copies approximately 320 KB into each returned lambda.

**Fix:** Capture `this` by raw pointer (if lifetime is guaranteed by the caller), or accept a `std::shared_ptr<EchoStateNetwork>` and capture that:
```cpp
auto self = shared_from_this(); // requires EchoStateNetwork to inherit enable_shared_from_this
return [self](const VectorXd& u) { return self->step(u); };
```

---

### `[LOW]` `[Confidence: High]`  
**`lib/KoopmanEDMD.cpp` — `liftPoly()` allocates two `VectorXd` per call**

`liftPoly()` allocates `VectorXd psi(n_lifted_)` and `VectorXd z(n+m)` on every call. In any prediction or rollout loop, this method is called at every time step.

**Fix:** Promote `psi` and `z` to `mutable` member vectors sized at construction; fill them in-place on each call.

---

### `[LOW]` `[Confidence: High]`  
**`lib/DynaController.cpp` ~lines 75–77 — Three 1-element `VectorXd` per `addTransition()` call**

`addTransition()` constructs three `Eigen::VectorXd(1)` per call during the online training phase.

**Fix:** Maintain persistent 1-element member vectors `x_scalar_`, `u_scalar_`, `xnext_scalar_` and assign values before appending.

---

### `[LOW]` `[Confidence: High]`  
**`lib/SINDy.cpp` STLS inner loop — `active` index vector and `Theta_a` sub-matrix allocated per STLS iteration**

The STLS inner loop allocates a new `std::vector<int> active` and a new `Eigen::MatrixXd Theta_a` for each state/iteration combination. For large problems or frequent re-fitting (as done by `DynaController`) this adds measurable allocation pressure.

**Fix:** Pre-allocate `active_` (capacity: n_terms) and `Theta_a_` (max size: K × n_terms) before the STLS loop; use a fill-count index rather than resizing.

---

### `[LOW]` `[Confidence: Low]`  
**`lib/FeedforwardController.h` ~line 89 — 1-element `VectorXd` allocated per step for SISO plants**

`compute(double r)` broadcasts the scalar into `VectorXd rv(model_.inputSize())` via `rv.fill(r)` on every call. For SISO plants (the common case) this is a 1-element heap allocation per step.

**Fix:** Pre-allocate `VectorXd rv_` as a member; assign `rv_(0) = r` each step.

---

## Category 6 — Python Bindings

High-severity binding findings are already listed in Category 1. Additional lower-severity items follow.

---

### `[LOW]` `[Confidence: High]`  
**`bindings/controllers_bindings.cpp` ~line 1675 — `reference_internal` on `DynaController::inner_controller()` without `keep_alive`**

Returns a `shared_ptr<IController>` with `py::return_value_policy::reference_internal` but no `py::keep_alive` annotation. If Python stores the returned inner controller beyond the `DynaController`'s lifetime, a use-after-free is possible.

**Fix:** Return by copy to ensure the `shared_ptr` reference count is incremented:
```cpp
.def("inner_controller", &ctrl::DynaController::innerController,
     py::return_value_policy::copy)
```

---

### `[LOW]` `[Confidence: High]`  
**`bindings/smoke_test.py` — Six bound classes / functions not asserted**

`ComputationalDelayWrapper`, `CUSUMChart`, `EWMAChart`, `SmithPredictor`, `ExtremumSeeker`, and `make_lqr_controller` are all registered in `*_bindings.cpp` files but absent from `smoke_test.py`. Binding regressions for any of these pass Phase 3 of `run.py` undetected.

**Fix:** Add:
```python
assert hasattr(ctrl, 'ComputationalDelayWrapper'), "ComputationalDelayWrapper not found"
assert hasattr(ctrl, 'CUSUMChart'),                "CUSUMChart not found"
assert hasattr(ctrl, 'EWMAChart'),                 "EWMAChart not found"
assert hasattr(ctrl, 'SmithPredictor'),            "SmithPredictor not found"
assert hasattr(ctrl, 'ExtremumSeeker'),            "ExtremumSeeker not found"
assert hasattr(ctrl, 'make_lqr_controller'),       "make_lqr_controller not found"
```

---

### `[LOW]` `[Confidence: Medium]`  
**`bindings/smoke_test.py` ~line 836 — `GreyBoxEstimator.predict()` shape assertion too loose**

`assert _Y_hat.shape == (1, _N_gb)` passes for any (1, 40) output even if the transposition semantics changed silently.

**Fix:**
```python
assert _Y_hat.shape[0] == n_outputs and _Y_hat.shape[1] == _N_gb, \
    f"predict() shape mismatch: got {_Y_hat.shape}"
```

---

### `[LOW]` `[Confidence: High]`  
**28 Python example files — MinGW DLL path inlined instead of using `_setup_bindings.py`**

The `C:\msys64\mingw64\bin` DLL discovery block is copy-pasted into ~28 files rather than using the existing `_setup_bindings.py` utility. Any change to the default MinGW install path requires editing all 28 files.

**Fix:** Replace the inline block in each affected file with `import _setup_bindings`. For files in `case-study/` subdirectories, use an absolute-path import:
```python
import importlib.util, pathlib
_sb = importlib.util.spec_from_file_location(
    "_setup_bindings",
    pathlib.Path(__file__).parent.parent.parent / "_setup_bindings.py")
importlib.util.module_from_spec(_sb); _sb.loader.exec_module(importlib.util.module_from_spec(_sb))
```

---

## Category 7 — Version Control & Project Health

---

### `[LOW]` `[Confidence: High]`  
**`.github/workflows/doc.yml` ~line 44 — Third-party action not pinned to SHA**

`uses: peaceiris/actions-gh-pages@v3` pins to a mutable major-version tag. The action author can silently update `v3` at any time, introducing supply-chain risk to the documentation deployment job.

**Fix:** Pin to the full commit SHA of the `v3.x.x` release (e.g. `peaceiris/actions-gh-pages@4f9cc6e`). GitHub's security hardening guide recommends this for all third-party actions.

---

### `[LOW]` `[Confidence: High]`  
**`.github/workflows/ubuntu.yml` + `windows.yml` — Build jobs use default broad `GITHUB_TOKEN` permissions**

The `build` jobs carry the default broad `GITHUB_TOKEN` permissions (read + write on contents, packages, etc.) unless the repository has restricted org-level defaults. Following the principle of least privilege, build jobs that only read code and produce artifacts should declare minimal permissions.

**Fix:** Add to each `build` job:
```yaml
permissions:
  contents: read
```

---

### `[LOW]` `[Confidence: High]`  
**`.gitignore` — Multiple common artifact patterns missing**

The following are absent and will appear as untracked files after a build or IDE session:

| Missing pattern | Produces from |
|-----------------|---------------|
| `*.pyd` | Python binding build outside `/build` |
| `*.exe` | Stray compiled examples |
| `CMakeCache.txt` | CMake configure run |
| `CMakeFiles/` | CMake configure run |
| `.idea/` | JetBrains IDEs (only `/.vscode` is excluded) |
| `case-study/**/logs/` | CSV telemetry from case-study runs |

The `/build` exclusion covers most of these for standard in-tree builds, but out-of-tree builds and IDE sessions will leave untracked files.

**Fix:** Append to `.gitignore`:
```
*.pyd
*.exe
CMakeCache.txt
CMakeFiles/
.idea/
case-study/**/logs/
```

---

### `[LOW]` `[Confidence: High]`  
**`.gitignore` ~line 15 — `/tools` gitignored but `tools/` scripts are committed**

`/tools` appears in `.gitignore`, but the files in `tools/` (e.g. `extract_text.py`) are tracked by git. Any new file added to `tools/` will be silently excluded from `git add -A`, making it easy to accidentally leave new scripts untracked.

**Fix:** Remove `/tools` from `.gitignore` (if all `tools/` files should be tracked), or add a negation to include specific patterns: `!/tools/*.py`.

---

### `[LOW]` `[Confidence: Medium]`  
**`CMakeLists.txt` ~line 23 — Unconditional `M_PI` macro redefinition**

`add_compile_definitions(M_PI=3.14159265358979323846)` is applied globally and unconditionally. On Linux with GCC and `_GNU_SOURCE` active, `<cmath>` already defines `M_PI`, triggering `-Wmacro-redefined` warnings across all translation units. The MSVC guard a few lines above already enables `M_PI` via `_USE_MATH_DEFINES`, making the unconditional definition below it redundant on Windows as well.

**Fix:**
```cmake
if(NOT MSVC)
    add_compile_definitions(M_PI=3.14159265358979323846)
endif()
```

---

### `[LOW]` `[Confidence: High]`  
**`examples/CMakeLists.txt` ~line 6 — Examples declare `cxx_std_17` but require C++20 features**

The examples CMake helper declares `target_compile_features(${NAME} PRIVATE cxx_std_17)`, but multiple examples use C++20 features — `std::numbers::pi` in `ex70_ilc.cpp`, structured bindings in several others. This works today because the library propagates `cxx_std_20` via its `PUBLIC` interface, but will break for anyone trying to build examples standalone.

**Fix:** Change the helper to `target_compile_features(${NAME} PRIVATE cxx_std_20)`.

---

### `[LOW]` `[Confidence: High]`  
**`tools/extract_text.py` ~lines 6, 19 — Two path-handling defects**

1. `os.walk("case-study")` resolves relative to the current working directory. If the script is run from any directory other than the repository root (e.g. `cd tools && python extract_text.py`), it silently walks zero files.  
   **Fix:** `os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'case-study')`

2. `file.replace(".pdf", ".txt")` corrupts the output path if `.pdf` appears anywhere in the parent directory name.  
   **Fix:** `os.path.splitext(file)[0] + ".txt"`

---

## Category 8 — Security

**Full scan across `lib/`, `examples/`, `bindings/`, `tools/`, `hal/`, `scripts/`:**

| Check | Result |
|-------|--------|
| `strcpy` / `sprintf` / `gets` / `strcat` | **None found** |
| `system()` / `popen()` with untrusted input | **None found** |
| Python `eval()` / `exec()` on external data | **None found** |
| `subprocess` with `shell=True` on untrusted input | **None found** (all uses are list-form) |
| Hardcoded passwords or API keys | **None found** |
| `GITHUB_TOKEN` in workflows | Correct GitHub Actions pattern — not a secret leak |

One security-adjacent finding:

### `[LOW]` `[Confidence: Medium]`  
**`scripts/deploy.py` ~line 350 — Non-stable `hash()` used as an integrity field**

`str(hash(json.dumps(params, sort_keys=True)))` uses Python's built-in `hash()` which is non-deterministic across process restarts (randomised by `PYTHONHASHSEED`) and varies between 32-bit and 64-bit interpreters. The field name `params_hash` implies reproducible integrity verification, which this cannot provide.

**Fix:**
```python
import hashlib
params_hash = hashlib.sha256(
    json.dumps(params, sort_keys=True).encode()
).hexdigest()[:16]
```

---

## HAL Concurrency Findings

---

### `[LOW]` `[Confidence: Medium]`  
**`lib/hal/SimPlant.h` ~line 71 — `state()` returns unprotected reference**

`state()` returns `const Eigen::VectorXd&` to the private member `x_` without holding `mu_` (the class mutex). Both `setState()` and `step()` hold `mu_` when writing `x_`. A concurrent `setState()` or `step()` call while a caller holds the reference returned by `state()` creates a data race in multi-threaded HIL scenarios.

**Fix:** Return by value under the mutex:
```cpp
Eigen::VectorXd state() const {
    std::lock_guard<std::mutex> lock(mu_);
    return x_;
}
```
Or document explicitly that `state()` is not thread-safe.

---

### `[LOW]` `[Confidence: High]`  
**`lib/hal/ZephyrScheduler.h` ~line 77 — Sub-millisecond period silently rounds to zero**

`K_MSEC(period_ns_ / 1'000'000ULL)` rounds any period below 1 ms to `K_MSEC(0)`, which creates an immediate one-shot Zephyr timer rather than a periodic one. A Ts of 500 µs — perfectly normal for fast inner control loops — silently produces wrong scheduling behaviour.

**Fix:** Add a guard in the constructor:
```cpp
if (period_ns_ < 1'000'000ULL)
    throw std::logic_error(
        "ZephyrScheduler: period < 1 ms not supported (K_MSEC resolution limit)");
```

---

### `[LOW]` `[Confidence: Medium]`  
**`lib/hal/FreeRTOSScheduler.h` ~line 128 — Tick counters are not atomic**

`ticks_` and `overruns_` are plain `uint64_t` members incremented from `timerCallback()` (the FreeRTOS timer-service task) and read from `tickCount()` / `overrunCount()` potentially from another thread. No atomic or mutex protection is present, creating a data race.

**Fix:** Declare both as `std::atomic<uint64_t>`:
```cpp
std::atomic<uint64_t> ticks_{0}, overruns_{0};
```
Use `.fetch_add(1, std::memory_order_relaxed)` in the callback.

---

### `[LOW]` `[Confidence: Medium]`  
**`lib/AtomicParamBuffer.h` ~line 85 — Spin-loop lacks CPU pause hint**

The `read()` spin loop (`while (s0 & 1u) continue;`) busy-waits without a CPU pause instruction. On x86 this wastes power and can delay store-buffer retirement in the memory ordering pipeline.

**Fix:**
```cpp
while (s0 & 1u) {
#if defined(__x86_64__) || defined(_M_X64)
    _mm_pause();
#else
    std::this_thread::yield();
#endif
}
```

---

## Consolidated Finding Index

| # | Sev | Conf | Location | Category | Summary |
|---|-----|------|----------|----------|---------|
| 1 | CRIT | High | `lib/DeePC.h` (absent) | Correctness | DeePC implementation files missing entirely |
| 2 | HIGH | High | `lib/DifferentialEvolution.h:133` | Correctness | Infinite loop on zero-width bounds in reflection |
| 3 | HIGH | High | `lib/DiscreteLQR.h:186` | Correctness | MIMO truncation warning disabled in Release |
| 4 | HIGH | High | `lib/ExtendedKalmanFilter.cpp:126` | Correctness | Unchecked LDLT in DAE covariance projection |
| 5 | HIGH | High | `lib/L1AdaptiveController.h:112` | Correctness | Dead private member `use_compute_y_` |
| 6 | HIGH | High | `lib/HybridModel.h:169` | Correctness | `dynamicsFunc()` captures raw `this` — dangling pointer |
| 7 | HIGH | High | `lib/NonlinearMPC.cpp:115` | Performance | Per-step O(Np×n²) allocation in `buildAndSolve()` |
| 8 | HIGH | High | `lib/MovingHorizonEstimator.cpp:111` | Performance | Per-step matrix allocation in `estimate()` |
| 9 | HIGH | High | `lib/GradientProjectionQP.h:97` | Performance | `VectorXd` allocation on every QP solve |
| 10 | HIGH | High | `lib/ScenarioMPC.cpp:55` | Performance | Per-step allocation in N_samples×Np loop |
| 11 | HIGH | High | `lib/TubeMPC.cpp:45` | Performance | Per-step output-stacked vector allocation |
| 12 | HIGH | High | `lib/KalmanFilter.cpp:36` | Performance | Four matrix temporaries per `update()` call |
| 13 | HIGH | High | `bindings/controllers_bindings.cpp:388` | Bindings | `DiscreteLQG` missing `shared_ptr<T>` holder |
| 14 | HIGH | High | `bindings/controllers_bindings.cpp` (×8+) | Bindings | 8 ML/optimizer classes missing `shared_ptr<T>` holder |
| 15 | MED | High | `lib/DiscreteLQG.h` | Correctness | `DiscreteLQG` does not inherit `IController` |
| 16 | MED | High | `lib/PlantModel.h` | Correctness | `consistentInit()` no convergence signal |
| 17 | MED | High | `lib/VectorFitting.cpp:246` | Correctness | Complex poles silently stripped to real parts |
| 18 | MED | High | `lib/BayesianOptimizer.h:230` | Correctness | `maximiseAcquisition()` `const`-incorrect via `mutable` |
| 19 | MED | High | `lib/HybridMPC.cpp:28` | Correctness | Unbounded training data buffer growth |
| 20 | MED | High | `lib/GainScheduledController.h:232` | Correctness | `lowerIndex()` unchecked empty-vector access |
| 21 | MED | High | `lib/CEMController.cpp:103` | Correctness | Silent zero return on unconfigured call |
| 22 | MED | High | `lib/MovingHorizonEstimator.cpp:212` | Correctness | General eigenvalue solver on SPD matrix |
| 23 | MED | High | `lib/GaussianProcess.cpp:24` | Performance | O(N) `vector::erase` at front in budget eviction |
| 24 | MED | High | `lib/CEMController.cpp:61` | Performance | Per-step vector allocation in `computeRef()` |
| 25 | MED | High | `lib/ParticleFilter.cpp:140` | Performance | Per-resample heap allocation in `resample()` |
| 26 | MED | High | `lib/NeuralPID.cpp:101` | Performance | Per-step gradient matrix temporaries |
| 27 | MED | High | `lib/CEMController.h:79` | Performance | `MatrixXd C` passed by value in constructor |
| 28 | MED | High | `lib/EchoStateNetwork.cpp:80` + SINDy + Koopman | Performance | `push_back` without `reserve` in data-collection loops |
| 29 | MED | High | `lib/DiscreteHinf.cpp` | Performance | `VectorXd` temporary per step in `computeVec()` |
| 30 | MED | High | `lib/SINDy.cpp` (`SINDyModel::predict`) | Performance | Full `SINDy` object constructed per prediction call |
| 31 | MED | Med | `lib/RecursiveGreyBoxEstimator.cpp` | Performance | O(2n_aug+1) `VectorXd` per estimator step |
| 32 | MED | High | `lib/GaussianProcess.h:93` | Docs | `L_chol_` is `LDLT`, not Cholesky; comment contradicts |
| 33 | MED | High | `lib/MismatchDetector.h:94` | Docs | Zero-size innovation fallback undocumented |
| 34 | MED | High | `bindings/controllers_bindings.cpp:1872` | Bindings | Stale TODO Fuzzy block is dead code |
| 35 | MED | Med | `bindings/advanced_bindings.cpp:194` | Bindings | `reference_internal` on `FuzzySystem` member vector |
| 36 | MED | High | `bindings/estimation_bindings.cpp:436` | Bindings | SubspaceID TODO stub block is dead code |
| 37 | MED | High | `ubuntu.yml` + `windows.yml` | CI | Python bindings never built or tested in CI |
| 38 | MED | High | All 4 workflow files | CI | No static analysis or sanitizer runs |
| 39 | MED | High | `lib/` (tests) | Tests | `LPVSystemID` has zero Catch2 tests |
| 40 | MED | High | `lib/` (tests) | Tests | `FunctionApproximator` has zero Catch2 tests |
| 41 | MED | High | `lib/` (tests) | Tests | `MetricsAnalyzer` has zero Catch2 tests |
| 42 | MED | High | `lib/` (tests) | Tests | `ZeroPhaseTrackingFilter` class untested |
| 43 | LOW | High | `lib/KoopmanEDMD.h:26` | Docs | `DiscreteLQG` listed twice in Doxygen (copy-paste) |
| 44 | LOW | High | `lib/ComputationalDelayWrapper.h:98` | Correctness | `lastOutput()` returns next-to-be-output, not last output |
| 45 | LOW | High | `lib/EchoStateNetwork.cpp:57` | Correctness | Dead `extendedState()` method |
| 46 | LOW | High | `lib/EchoStateNetwork.h:119` | Docs | `W_out_` declared size contradicts architecture doc |
| 47 | LOW | High | `lib/EchoStateNetwork.cpp` | Performance | `reservoirStateFunc()` copies entire ESN by value (~320 KB) |
| 48 | LOW | High | `lib/KoopmanEDMD.cpp` | Performance | `liftPoly()` allocates two `VectorXd` per call |
| 49 | LOW | High | `lib/DiscreteSMC.cpp:39` + MPC + GPC | Performance | 1-element `VectorXd` heap-allocated per step for observer |
| 50 | LOW | High | `lib/VectorFitting.cpp:244` | Performance | `EigenSolver` reconstructed every SK iteration |
| 51 | LOW | Med | `lib/UnscentedKalmanFilter.cpp:56` | Performance | `sigmaPoints()` returns large matrix by value per step |
| 52 | LOW | High | `lib/ScenarioMPC.h:226` + `BayesianOptimizer.h:183` | Design | Unnecessary `mutable` on RNG members |
| 53 | LOW | High | `lib/AdaptiveSmithPredictor.cpp:93` | Correctness | Deque never shrinks if `bufferLen == 0` |
| 54 | LOW | High | `lib/DiscreteSMC.h` | Docs | `slidingSurface()` returns step k−1, not current |
| 55 | LOW | Med | `lib/ParticleFilter.h:220` | Docs | Comment says "log-sum-exp" but stores raw probabilities |
| 56 | LOW | High | `lib/AutoGainScheduler.h` | Correctness | `findEquilibrium()` 1000× tolerance fallback has no diagnostic |
| 57 | LOW | Med | `lib/AtomicParamBuffer.h:85` | Performance | Spin-loop lacks `_mm_pause()` / `yield()` |
| 58 | LOW | High | `lib/DiscreteADRC.h` | Docs | `compute()` requires `setReference()` with no runtime assertion |
| 59 | LOW | Low | `lib/FeedforwardController.h:89` | Performance | 1-element `VectorXd` per step for SISO plants |
| 60 | LOW | Med | `lib/DiscreteMPC.h` | Correctness | `lastQPConverged()` / `isHealthy()` lack `[[nodiscard]]` |
| 61 | LOW | High | `lib/SystemAnalysis.h` | Docs | O(n⁶) complexity not in `solveDiscreteLyapunov()` Doxygen |
| 62 | LOW | High | `lib/DynaController.cpp:75` | Performance | Three 1-element `VectorXd` per `addTransition()` call |
| 63 | LOW | High | `lib/SINDy.cpp` (STLS) | Performance | `active` and `Theta_a` allocated per STLS iteration |
| 64 | LOW | High | Tests | Tests | No standalone `[pid]` section |
| 65 | LOW | High | Tests | Tests | `RepetitiveController` missing Catch2 edge cases |
| 66 | LOW | High | Tests | Tests | `DiscretePID::computeDoM` has no comparative test |
| 67 | LOW | Med | Tests | Tests | `ExtremumSeeker` has no convergence test |
| 68 | LOW | High | `test_catch2_advanced.cpp:2998` | Tests | Vague test name — `DynaController` only checks `isfinite` |
| 69 | LOW | High | `bindings/controllers_bindings.cpp:1675` | Bindings | `reference_internal` on `inner_controller()` lacks `keep_alive` |
| 70 | LOW | High | `bindings/smoke_test.py` | Bindings | 6 bound classes/functions not asserted in smoke test |
| 71 | LOW | Med | `bindings/smoke_test.py:836` | Bindings | `GreyBoxEstimator.predict()` shape assertion too loose |
| 72 | LOW | High | 28 Python files | Bindings | MinGW DLL path inlined 28× instead of `import _setup_bindings` |
| 73 | LOW | High | `examples/CMakeLists.txt:6` | Build | Examples declare `cxx_std_17` but require C++20 |
| 74 | LOW | High | `doc.yml:44` | CI | Third-party action not pinned to SHA |
| 75 | LOW | High | Build job definitions | CI | Build jobs use default broad `GITHUB_TOKEN` permissions |
| 76 | LOW | High | `.gitignore` | Project | Missing `*.pyd`, `*.exe`, `CMakeCache.txt`, `.idea/`, `case-study/**/logs/` |
| 77 | LOW | High | `.gitignore:15` | Project | `/tools` gitignored but `tools/` scripts are committed |
| 78 | LOW | Med | `CMakeLists.txt:23` | Build | Unconditional `M_PI` redefinition triggers `-Wmacro-redefined` on GCC |
| 79 | LOW | High | `tools/extract_text.py:6,19` | Tools | Relative path + wrong extension-strip method |
| 80 | LOW | Med | `scripts/deploy.py:350` | Security-adj. | Non-stable `hash()` used as integrity field |
| 81 | LOW | Med | `lib/hal/SimPlant.h:71` | HAL | `state()` not mutex-protected — data race risk |
| 82 | LOW | High | `lib/hal/ZephyrScheduler.h:77` | HAL | Sub-ms period silently rounds to zero |
| 83 | LOW | Med | `lib/hal/FreeRTOSScheduler.h:128` | HAL | Tick counters not atomic — data race from timer callback |
| 84 | LOW | Med | `lib/AtomicParamBuffer.h:85` | HAL | Spin-loop lacks CPU pause hint |

---

*Security scan across all directories returned **zero** findings for unsafe C functions (`strcpy`, `sprintf`, `gets`, `strcat`), hardcoded credentials, `eval`/`exec` on external data, or `shell=True` subprocess calls with untrusted input.*

*TODO / FIXME / HACK comment count across all `lib/` files: **zero**.*

*God classes (>20 public methods): **none found**.*

*Inheritance depth >2 levels: **none found**.*

*Circular `#include` dependencies: **none found**.*
