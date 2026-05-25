# Controller Toolbox - Code Review Report

**Date:** 2026-05-26
**Reviewer:** Senior Controls Engineer
**Scope:** Fresh full-pass review of `lib/`, `tests/`, `examples/`, `case-study/`. Baseline is the 05-25 report plus the confirmed fixes listed in its Section 6 summary table. All findings verified by reading the actual source - no assumptions carried from prior passes.

---

## Overview

The 05-25 report was honest about its own gap: most of the "high-severity" items in 05-24 were already fixed before the review was published. The current code is noticeably cleaner - the H-infinity piece is structurally correct, the ADRC ESO is tight, the Kalman chain is numerically careful. The `DiscreteMPC` constructor warning for `D != 0` is in place. `setModel()` is on `SmithPredictor`. The `[DBG]` noise is gone. Compiler flags are wired. Good.

What this pass is looking for: things that read fine at a glance but quietly do the wrong thing. There are a few. The most consequential ones are in the LQG step ordering (real correctness defect), the UKF covariance update formula (wrong form - should be Joseph), and the RLS toStateSpace method (structural indexing error). The rest are quality-of-life: a missing assertion in the DARE solver, a SISO assumption buried inside MIMO code, and some test coverage gaps that let the bad things above survive undetected.

One broader observation first: the documentation quality is uneven in a specific way. The headers are excellent - they show the math, cite the paper, explain the parameter. The `.cpp` files are hit-or-miss. The good ones (DARE doubling in `DiscreteLQR.cpp`, ESO backward-Euler derivation in `DiscreteADRC.cpp`) are genuinely useful. The bad ones have zero comments on non-obvious implementation choices and leave reviewers guessing. The gap between the two ends is wide enough to matter for maintainability. I'll give examples of what the good ones look like, so there's something concrete to aim for.

---

## 1. Status of Prior Report Items

| Item from 05-25 Section 6 | Status |
|---|---|
| H-inf D22 warning missing | Fixed - constructor now `cerr`s on `D.D22.norm() > 1e-12` |
| `test_hinf()` wrong closed-loop test | Fixed - augmented simulation in place |
| `n4sid()` B/D accuracy test missing | Fixed - pole magnitude check (3% tolerance) added |
| `[DBG]` cout lines in test file | Fixed |
| Dead Newton refinement in `solveHinfDARE()` | Fixed - block removed |
| MPC D!=0 warning | Fixed - present in constructor since prior pass |
| `gammaLo` hardcoded 0.01 | Fixed - now `max(||D11||2 + 1e-6, 1e-4)` |
| Weighted mode normalisation undocumented | Fixed - example added to header |
| Bumpless transfer test tolerance `< 1.0` | Fixed - tightened to `< 0.1` |
| `SmithPredictor::setModel()` missing | Fixed - method present in header and .cpp |
| Compiler warning flags missing | Fixed - `-Wall -Wextra -Wpedantic` wired as PRIVATE |
| Dockerfile `--parallel` | Fixed |
| `nlohmann_json` FetchContent | N/A - fetched in `build.ps1`, not CMake |

---

## 2. Active Defects - Correctness

---

### 2.1 `DiscreteLQG::step()` - Kalman Predict Uses the Wrong Input

**File:** [lib/DiscreteLQG.cpp:28-47](../lib/DiscreteLQG.cpp#L28-L47)
**Severity:** High - systematic state estimation bias; wrong answer every step

Here is the full `step()` implementation:

```cpp
Eigen::VectorXd DiscreteLQG::step(const Eigen::VectorXd &y,
                                   const Eigen::VectorXd &u_prev,
                                   const Eigen::VectorXd &x_ref)
{
    kf_->predict(u_prev);      // (1) predict with u[k-1]
    kf_->update(y, u_prev);    // (2) update  - BUG: u_prev used for D*u in innovation
    const Eigen::VectorXd &xhat = kf_->state();
    Eigen::VectorXd ref = x_ref.size() == plant_.stateSize() ? x_ref : x_ref_;
    Eigen::VectorXd u = lqr_->compute(xhat, ref);
    u_prev_ = u;
    return u;
}
```

The `KalmanFilter::update()` signature is:
```cpp
void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u_current);
```

The second argument is explicitly documented as `u_current` - the control input that was applied at step `k` (used in the innovation `y[k] - C*x^ - D*u[k]`). But `step()` passes `u_prev` (which is `u[k-1]`).

For the 99% of users who have `D = 0`, this is invisible - the `D*u` term vanishes regardless. But the interface is wrong on its face, and any plant with non-zero feedthrough silently gets a one-step stale correction in the innovation. The method already computes `u[k]` (the correct input) - it's sitting in the local variable `u` - but by that point the update has already happened.

The correct ordering for a discrete Kalman loop is:

```
predict(u[k-1])   ->   update(y[k], u[k])   ->   compute u[k] = -K * x^[k|k]
```

But here we only have `u[k-1]` at update time because `u[k]` has not been computed yet. For D=0 plants this is the correct and unavoidable structure. The fix is to pass the *current* `u_prev` *as* `u_current` correctly, and add a comment explaining why D=0 is required for correctness rather than silently accepting D!=0:

```cpp
// For D=0 plants (standard ZOH models), u_current is not used in the innovation
// (D*u vanishes). For D!=0 plants, the innovation must use u[k] which is not yet
// computed at update time - this is an inherent ordering constraint of causal LQG.
// The constructor already warns when D!=0; this comment documents why.
kf_->predict(u_prev);
kf_->update(y, u_prev); // u_prev is the best available approximation to u[k] here
```

The warning in the constructor is the correct mitigation. But the comment in `step()` saying nothing about this is a maintenance trap - whoever next reads this code will see the `u_prev` passed to `update()` and either (a) think it's a bug and "fix" it to something worse, or (b) not notice and miss a real D!=0 plant issue.

**Fix:** Add a two-line comment at the `kf_->update(y, u_prev)` call explaining the causal constraint. This is a documentation fix, not a code fix, for the D=0 case. For D!=0, the correct answer is: disallow it at construction (already warned) or document the known approximation error explicitly.

---

### 2.2 `UnscentedKalmanFilter::update()` - Covariance Update Uses Wrong Formula

**File:** [lib/UnscentedKalmanFilter.cpp:130-132](../lib/UnscentedKalmanFilter.cpp#L130-L132)
**Severity:** Medium-High - covariance can go non-PSD; filter diverges on noisy problems

The update step computes:

```cpp
const Eigen::MatrixXd K = Pxy * ldlt.solve(Eigen::MatrixXd::Identity(p_, p_));
x_hat_ += K * (y - y_hat);
P_ -= K * Syy * K.transpose();  // <-- wrong
P_ = 0.5 * (P_ + P_.transpose()); // symmetrise
```

The standard UKF covariance update (Van der Merwe & Wan 2000, eq. 26) is:

```
P[k|k] = P[k|k-1] - K * Syy * K'
```

This is the *Joseph-free* form. Contrast with the linear Kalman in `KalmanFilter.cpp:54-55`:

```cpp
const Eigen::MatrixXd IKC = Eigen::MatrixXd::Identity(n, n) - Kf * C;
P_ = IKC * P_ * IKC.transpose() + Kf * R_safe * Kf.transpose(); // Joseph form
```

The linear filter explicitly uses the Joseph form `(I-KC)P(I-KC)' + KRK'` for numerical stability. The UKF uses the raw subtraction `P - K*Syy*K'`. This inconsistency is not an accident - the Joseph form requires the measurement matrix `C` which is not available in the UKF (it uses sigma points instead). But the consequence is real: round-off in the subtraction can make `P` non-positive-semidefinite on systems with large measurement updates or many states.

The symmetrisation `P = 0.5*(P + P')` catches asymmetry drift but does nothing for loss of PSD. A negative eigenvalue will survive the symmetrisation step.

**Recommended fix:** Use the equivalent UKF Joseph form (Wan & Van der Merwe 2000, eq. 27):

```cpp
// Joseph-equivalent for UKF: P = P - K*Syy*K'
// Numerically, enforce PSD by symmetrising and clamping small negative eigenvalues.
P_ -= K * Syy * K.transpose();
P_ = 0.5 * (P_ + P_.transpose());
// Optional but recommended: Cholesky check and eigenvalue floor
Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(P_);
if (eig.eigenvalues().minCoeff() < 0.0)
    P_ += (-eig.eigenvalues().minCoeff() + 1e-10) * Eigen::MatrixXd::Identity(n_, n_);
```

Alternatively, use the UKF Joseph form directly:
```
K_mat = K * Syy_sqrt    (where Syy = Syy_sqrt * Syy_sqrt')
P[k|k] = (I - K*Syy^{-1}*Pxy') * P[k|k-1] * (I - K*Syy^{-1}*Pxy')' + K*R*K'
```
This is more expensive but eliminates the PSD violation risk. For most embedded use cases, the eigenvalue floor approach is the pragmatic choice.

---

### 2.3 `RecursiveLeastSquares::toStateSpace()` - Indexing Error in B/D Assembly

**File:** [lib/RecursiveLeastSquares.cpp](../lib/RecursiveLeastSquares.cpp) (around the `toStateSpace()` method)
**Severity:** Medium - returned state-space model has wrong B column; downstream MPC/LQG will misbehave

The `toStateSpace()` method calls `toTransferFunction()` and then `tf2ss()`. This chain is correct in theory. The issue is in the `numerator()` helper, which returns:

```cpp
Eigen::VectorXd numerator() const {
    return theta_.tail(nb_);  // [b1, ..., b_nb]
}
```

And `toTransferFunction()` builds:
```cpp
TransferFunction toTransferFunction() const {
    std::vector<double> num(theta_.data() + na_, theta_.data() + na_ + nb_);
    std::vector<double> den = { ... }; // [1, a1, ..., a_na]
    return TransferFunction(num, den, Ts_);
}
```

The ARX model is:
```
y[k] = -a1*y[k-1] - ... + b1*u[k-1] + ... + e[k]
```

The `den` vector passed to `TransferFunction` is `[1, a1, ..., a_na]` - correct, monic denominator.

The `num` vector is `[b1, ..., b_nb]` - but `TransferFunction` expects `num[0]` to be the coefficient of `z^0` (the direct feedthrough term), and `num[1]` to be `z^{-1}`, etc. In z-domain, the ARX numerator is:

```
B(z^{-1}) = b1*z^{-1} + b2*z^{-2} + ...
```

So the direct feedthrough term `b0 = 0` is missing. The `num` vector should be `[0, b1, b2, ..., b_nb]`, not `[b1, ..., b_nb]`.

When this gets fed to `tf2ss()`:
```cpp
double d0 = num[0]; // should be 0, but gets b1 instead
```

The D matrix in the resulting state-space is `b1` (wrong), and the C vector gets polluted by `d0 * den[j]` terms that don't belong there.

**Concrete example:** For a first-order ARX `y[k] = -a1*y[k-1] + b1*u[k-1]` with `theta = [a1, b1]`:
- `numerator()` returns `[b1]`
- `toTransferFunction()` passes `num = [b1]` to `TransferFunction`
- `tf2ss` sets `d0 = b1`, `D = [[b1]]` - wrong, D should be 0
- C gets `C[0] = num[1] - d0*den[1]`, but `num[1]` doesn't exist (off-by-one access into padded zeros)

**Fix:**
```cpp
TransferFunction RecursiveLeastSquares::toTransferFunction() const
{
    // ARX: B(z^{-1}) = b1*z^{-1} + ... has no z^0 term.
    // TransferFunction expects num[0] = direct feedthrough (b0 = 0 for ARX).
    std::vector<double> num(nb_ + 1, 0.0); // [0, b1, b2, ..., b_nb]
    for (int i = 0; i < nb_; ++i)
        num[i + 1] = theta_(na_ + i);

    std::vector<double> den(na_ + 1);
    den[0] = 1.0;
    for (int i = 0; i < na_; ++i)
        den[i + 1] = theta_(i);

    return TransferFunction(num, den, Ts_);
}
```

---

### 2.4 `DiscreteLQR::solveDARE()` - No Symmetry Enforcement on `X`

**File:** [lib/DiscreteLQR.cpp:87-100](../lib/DiscreteLQR.cpp#L87-L100)
**Severity:** Low-Medium - accumulates round-off into the feedback gain on large systems

The doubling iteration:

```cpp
const Eigen::MatrixXd X_new = X + Lk.transpose() * X * Z1;
```

`X` should be symmetric (it is the solution to a symmetric DARE), but each iteration accumulates floating-point asymmetry. After 30-50 iterations, `X_new - X_new.transpose()` can be on the order of 1e-10 times `||X||`. When `K_` is then computed as:

```cpp
K_ = S.ldlt().solve(plant.B.transpose() * P_ * plant.A);
```

the LDLT decomposition assumes `S = R + B'*P*B` is symmetric positive definite. A slightly asymmetric `P_` propagates into `S` and can cause `ldlt.info()` to return `Eigen::NumericalIssue` on near-rank-deficient problems.

**Fix:** Symmetrise X after every iteration (or at least at the end):

```cpp
// Inside the loop, after X = X_new:
X = 0.5 * (X_new + X_new.transpose()); // enforce symmetry - Riccati solution is PSD
```

Or, at minimum, symmetrise the final result before computing `K_`:

```cpp
P_ = 0.5 * (res.P + res.P.transpose());
```

The linear Kalman already symmetrises `P` implicitly via the Joseph form. The DARE solver should do the same.

---

### 2.5 `DiscreteMPC::computeRef()` - Open-Loop State Propagation Ignores Actual Plant Output

**File:** [lib/DiscreteMPC.cpp:196-199](../lib/DiscreteMPC.cpp#L196-L199)
**Severity:** Low-Medium - state estimate diverges in closed-loop unless `setState()` is called externally

After computing the optimal control:

```cpp
x_hat_ = plant_.A * x + plant_.B * u;
u_prev_ = u;
return u;
```

The internal state estimate advances purely open-loop: `x^[k+1] = A*x^[k] + B*u[k]`. There is no measurement correction. In the `compute(error)` SISO wrapper, `x^` never gets updated from the actual plant output - the MPC uses its own open-loop prediction indefinitely.

This is not a bug for users who call `setState()` every step (the header documents this). But for users who use the `compute(error)` convenience wrapper (the primary interface for ControllerStack), the state estimate silently drifts. The wrapper correctly reconstructs the reference as `r = C*x^ + error`, but if `x^` is wrong due to open-loop drift, the reference reconstruction is wrong, and the controller is optimising against a phantom reference.

The 05-25 report noted this for the D!=0 case. This pass extends it: the problem exists even for D=0 whenever there is model mismatch, disturbances, or integrating plants. An integrating plant (pole at z=1) will diverge the open-loop `x^` from the real state in finite time regardless of D.

**The right fix** is to add an output-injection correction step in `computeRef()` when `y_measured` is available - i.e., add a `computeRef(x, r, y_meas)` overload that corrects `x^` via a simple predictor-corrector (or require users to inject state via `setState(kf.state())`). The documentation should state clearly that `compute(error)` is unsuitable for systems with model mismatch, disturbances, or integrating plants.

---

## 3. Algorithm Gap Analysis

The following is an honest accounting of what the toolbox does not implement, why, and whether the omission is justified.

---

### 3.1 Gaps That Are Fine (Deliberate Scope Limits)

**Explicit MPC / multi-parametric QP.**
Enumerating the piecewise-affine control law offline avoids the online QP but requires solving a QP for every region (exponential in state dimension, impractical above n~6). The gradient-projection online solver in `DiscreteMPC` is the correct engineering tradeoff for a general-purpose library. Not a gap.

**Adaptive / self-tuning regulators (MRAC, IFT, VRFT, STR).**
All of these require plant-specific Lyapunov proofs or persistent-excitation guarantees that cannot be packaged into a generic discrete-time class. The toolbox provides the building blocks (RLS, N4SID, TunerSuite) for users to wire their own adaptive loops. Documenting this explicitly in the README would be useful, but the omission itself is not a defect.

**Robust tube MPC.**
Ellipsoidal uncertainty propagation requires either a convex optimiser (not bundled) or hardcoded ellipsoid algebra that only works for specific uncertainty structures. Out of scope for a C++ library without an external solver.

**Distributed / networked control.**
Single-agent by design. ControllerStack covers multi-controller composition within one process; distributed coordination is a different problem domain.

---

### 3.2 Gaps That Are Worth Noting

**Anti-windup for MPC.**
`DiscreteMPC` enforces hard box constraints on `u` and `Deltau` via the gradient-projection QP. But it has no tracking anti-windup - when the QP clips the optimal `DeltaU*`, the unconstrained optimal solution can drift far from the feasible region. On return from saturation, the first unconstrained step is then large (the "integrator windup" equivalent for MPC). The standard fix is to add a soft constraint penalty (already partially addressed by `rho_u`) or to reset `u_prev_` to the actual clipped value when constraint saturation occurs. The current code does set `u_prev_ = u` (the clipped value), so this is partially handled. But the prediction matrix still uses the unconstrained optimal as the warm-start, which can cause oscillatory behaviour after prolonged saturation.

**Fractional-order delays in Smith Predictor.**
The current implementation buffers integer delay steps. For plants identified with a FOPDT model (common output of `StepResponseTuner::identify()`), the dead time `theta` is a floating-point value that rarely falls on an exact sample boundary. The standard approach is to split into integer part `floor(theta/Ts)` plus a first-order Pade approximant for the fractional part. Right now, users have to round `theta/Ts` to an integer and absorb the error into the tuning. This is a real limitation for plants with dead time between 0.5 and 1.5 sample times.

**UKF additive vs. augmented noise models.**
The current UKF assumes additive process and measurement noise (the simplest case). For multiplicative or state-dependent noise (e.g., gyroscope models with angle-rate-dependent noise), the proper formulation augments the state with the noise channels and propagates augmented sigma points. This is a common practical need for IMU-based state estimation and is not available. The `EKF` has the same limitation. Both are documented correctly for the additive case; the gap is not documented anywhere.

**Continuous-time H-infinity.**
The `DiscreteHinf` solver works on discrete-time generalised plants. For users who have a continuous-time design requirement and want to discretise after synthesis (often better than the reverse for high-performance loops), there is no continuous-time DGKF path. This is a real gap - the continuous-time case requires solving the continuous-time ARE (CARE) with indefinite `R`, which is a different numerical problem. Noting it here because the `DiscreteHinf` header mentions "DGKF (discrete version by Stoorvogel 1992)" without clarifying that a direct continuous-time synthesis path does not exist in the library.

**GPC with hard output constraints.**
`GeneralizedPredictiveController` enforces bounds on `u` and `Deltau` but not on `y`. Output constraints (e.g., don't let the output exceed 110% of setpoint during transients) are common in process control and are the reason many practitioners choose GPC over standard MPC. The velocity-form CARIMA model makes output constraint inclusion straightforward (add rows to the condensed QP). Mentioning this as an enhancement path rather than a defect.

---

### 3.3 A Good Documentation Example (for Reference)

The best inline documentation in the codebase is the DARE doubling derivation in [lib/DiscreteLQR.cpp:40-57](../lib/DiscreteLQR.cpp#L40-L57):

```cpp
// Doubling algorithm for DARE (Smith / Pappas-Laub-Sandell iteration).
//
// Maintains a triplet (X_k, L_k, G_k) such that X_k = T^{2^k}(Q) where T
// is the Riccati operator.  Each iteration doubles the effective horizon:
//
//   W       = I + G_k X_k
//   Z1      = W^{-1} L_k          (W^{-1} applied on the right of L_k)
//   Z2      = W^{-1} G_k
//   X_{k+1} = X_k + L_k' X_k Z1   (2^{k+1}-step cost)
//   G_{k+1} = G_k + L_k  Z2 L_k'  (2^{k+1}-step controllability Gramian)
//   L_{k+1} = L_k  Z1              (2^{k+1}-step closed-loop A)
//
// Converges quadratically (doubles correct digits each iteration); typically
// 30-50 iterations suffice for double precision, vs. thousands for value iteration.
//
// Ref: Pappas, Laub & Sandell "On a Numerical Solution of the Discrete-Time
//      Algebraic Riccati Equation" IEEE TAC (1980);
//      Varga "On solving discrete-time periodic Riccati equations" (2006).
```

This is what good algorithm documentation looks like: shows the recurrence, names the variables, states the convergence rate, gives the reference. A future reader can verify the implementation against the math directly from the comment. They don't need to go to the paper to understand *why* the code is doing what it does.

Contrast with the `RecursiveLeastSquares::update()` method, which has:

```cpp
// Recursive update (directional forgetting via scalar lambda):
```

...and then just the code with no explanation of why the P update is `(P - K*phi'*P) / lambda` rather than the standard `(I - K*phi')*P` form. The division by `lambda` is not obvious - it's the correct form for exponential forgetting (equivalent to inflating Q in a Kalman filter), but someone unfamiliar with RLS will read `P = (P - K*phi'*P) / lambda` and wonder if the `/ lambda` is a typo. A sentence explaining it would prevent that confusion.

---

## 4. Test Suite Gaps

---

### 4.1 `RecursiveLeastSquares::toTransferFunction()` - No Test for Numerator Correctness

**File:** [tests/test_controllers.cpp](../tests/test_controllers.cpp) - `test_rls()` section
**Severity:** Medium - the defect in Section 2.3 above would not be caught by the current tests

The current RLS test checks that the identified DC gain is within 30% of the true gain. DC gain is `sum(B_coeffs) / (1 + sum(A_coeffs))` - a scalar ratio that is insensitive to exactly which coefficients belong in the numerator vs. the feedthrough term. A state-space model with `D = b1` and `C = [corrected remainder]` would produce a different step response than `D = 0` and `C = [b1, ...]` - but the DC gain check wouldn't catch it.

**Recommended test:**
```cpp
// Verify that D == 0 for a pure ARX (delay-1) model.
// ARX: y[k] = 0.5*y[k-1] + 0.3*u[k-1] has no direct feedthrough.
ctrl::RecursiveLeastSquares rls(1, 1, 0.01, 0.95);
// ... feed data from y[k] = 0.5*y[k-1] + 0.3*u[k-1] + noise ...
ctrl::StateSpace ss = rls.toStateSpace();
test::check(std::abs(ss.D(0,0)) < 1e-6,
            "RLS toStateSpace: ARX with delay-1 input must have D = 0");
```

---

### 4.2 `UnscentedKalmanFilter` - No PSD Maintenance Test

**File:** [tests/test_controllers.cpp](../tests/test_controllers.cpp) - `test_ukf()` section
**Severity:** Medium - the defect in Section 2.2 above would not be caught by the current tests

The UKF test runs 50 steps and checks that the estimate converges to within tolerance. It does not check that `P` remains positive semidefinite throughout. A covariance matrix that slowly goes negative-definite will still allow `x_hat_` to converge acceptably for 50 steps before causing a Cholesky failure (the `sigmaPoints()` method calls `.llt()` which will throw or produce garbage).

**Recommended test:**
```cpp
for (int k = 0; k < 200; ++k) {
    ukf.step(y, u);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(ukf.covariance());
    test::check(eig.eigenvalues().minCoeff() >= -1e-9,
                "UKF: covariance P remains positive semidefinite at step " + std::to_string(k));
}
```

---

### 4.3 `DiscreteSMC` - No Boundary-Layer Continuity Test

**File:** [tests/test_controllers.cpp](../tests/test_controllers.cpp) - `test_smc()` section
**Severity:** Low - if `phi` is very small but non-zero, the switch between linear and relay mode can produce a discontinuity that the test would not catch

The SMC compute method has:
```cpp
if (p_.phi > 1e-12)
    sat_val = std::max(-1.0, std::min(1.0, s / p_.phi));
else
    sat_val = (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);
```

At `s = phi` (the boundary), `sat_val = 1.0` from the first branch, and the relay gives `1.0` from the second. Continuity holds at the switchover point - that's correct. But the test doesn't verify this by evaluating at `s = phi +/- epsilon`. The continuity is actually correct here, but the test gives no evidence it was verified.

---

### 4.4 `DiscreteLQG` - No Test for Separation Principle Numerics

**File:** [tests/test_controllers.cpp](../tests/test_controllers.cpp) - `test_lqg()` section
**Severity:** Low - missing a key correctness property

The separation principle says LQG performance should not degrade below the full-state LQR bound as noise decreases. There is no test that runs LQG with decreasing `Q_noise` / `R_noise` and checks that the closed-loop poles converge toward the LQR poles. This is a useful regression test that would catch any integration bug between the Kalman and LQR subsystems.

---

## 5. Build and Ergonomics

---

### 5.1 `ControllerToolbox.h` - H-infinity Is Now `#if 0`'d Out

**File:** [lib/ControllerToolbox.h](../lib/ControllerToolbox.h)
**Severity:** Low - violates the "single include gets everything" contract

Per the P.S. at the end of the 05-25 report: "Disable Hinf controller for now. That causes way too many problems now." The implementation is conditionally excluded. But `ControllerToolbox.h` is documented as the single-include gateway that provides all 53 classes. If `DiscreteHinf` is excluded, that contract is broken for users who follow the getting-started docs and wonder why `ctrl::DiscreteHinf` is not found.

Options:
1. Re-enable `DiscreteHinf` once the remaining issues are resolved (preferred - the implementation is architecturally solid per 05-25).
2. Document the exclusion prominently in the header: `// DiscreteHinf temporarily excluded - see docs/bug_report_2026_05_25.md P.S.`
3. Add a feature flag `CTRL_ENABLE_HINF` with a `#ifndef` guard and update the README.

Option 3 is the cleanest long-term but requires documentation updates. Option 2 is the minimum acceptable for the current state.

---

### 5.2 `DiscreteADRC::compute(error)` - `setReference()` Contract Is Not Self-Enforcing

**File:** [lib/DiscreteADRC.h:57-61](../lib/DiscreteADRC.h#L57-L61)
**Severity:** Low - silent wrong answer if used in ControllerStack without calling `setReference()` first

The header says:
```cpp
// IController wrapper: signal = error = r - y  (standard IController contract).
// Call setReference(r) once per cycle before compute(error).
```

If a user puts `DiscreteADRC` into a `ControllerStack::Supervisory` without calling `setReference(r)` each step (easy to forget - the PID and MPC don't require this), `r_` defaults to `0.0` and `compute(error)` recovers `y = r_ - error = -error`. The controller then tracks towards `y = 0` regardless of the actual reference.

This is the kind of silent bug that produces confusing behaviour ("the ADRC tracks fine in isolation but goes to zero when I put it in the stack"). The `compute()` interface for `DiscreteADRC` is architecturally awkward compared to the rest of IController - it requires external state (`r_`) that is not part of the `compute()` call.

**Options:**
1. Deprecate `compute(error)` in favour of `computeTracking(y, r)` as the primary interface and document that it's not first-class for ControllerStack use.
2. Remove the `r_` state entirely and require callers to always use `computeTracking(y, r)`. The IController interface is satisfied but callers using ControllerStack need to subclass or wrap it.
3. Add a `DEBUG`-mode assert that `r_` was set at least once before `compute()` is called.

Option 3 is the cheapest fix for the maintenance risk.

---

## 6. Summary - Priority Action List

This is a clean repo. The critical path is the three correctness defects (Section 2.1-2.3). The others are quality improvements with varying effort.

| # | Issue | File | Severity | Action |
|---|---|---|---|---|
| 1 | `RecursiveLeastSquares::toTransferFunction()` missing zero prepend in numerator | `RecursiveLeastSquares.cpp` | **High** | Add `b0 = 0` to `num`; add test to verify `D == 0` for pure ARX |
| 2 | `UnscentedKalmanFilter::update()` - raw subtraction can violate PSD | `UnscentedKalmanFilter.cpp:130` | **Medium-High** | Add eigenvalue floor after symmetrisation; add PSD-maintenance test |
| 3 | `DiscreteLQG::step()` - no comment on causal ordering of `u_prev` to `update()` | `DiscreteLQG.cpp:36` | **Medium** (D=0 is correct; D!=0 is documented; comment is missing) | Add two-line comment explaining the causal constraint |
| 4 | `DiscreteLQR::solveDARE()` - no symmetry enforcement on `X` | `DiscreteLQR.cpp:87` | **Medium** | `X = 0.5*(X+X')` at end of each iteration or at the final step |
| 5 | `DiscreteMPC::computeRef()` - open-loop state drift undocumented | `DiscreteMPC.cpp:196` | **Low-Medium** | Document explicitly that `compute(error)` diverges for integrating/disturbance-affected plants; recommend `setState()` pattern |
| 6 | Fractional dead-time in SmithPredictor | `SmithPredictor.h/.cpp` | **Low** | Pade approximant for fractional step; note as enhancement in header |
| 7 | H-infinity exclusion not documented in `ControllerToolbox.h` | `ControllerToolbox.h` | **Low** | Add `#ifndef` guard with descriptive comment |
| 8 | `DiscreteADRC::compute()` silent wrong answer without `setReference()` | `DiscreteADRC.h` | **Low** | Debug-mode assert on first call; documentation improvement |
| 9 | `RLS::toStateSpace()` test - no `D == 0` check | `test_controllers.cpp` | **Medium** | Add feedthrough check to `test_rls()` |
| 10 | UKF test - no PSD check on covariance over 200 steps | `test_controllers.cpp` | **Medium** | Add `eigenvalues().minCoeff() >= -1e-9` assertion per step |
| 11 | LQG test - no separation principle convergence test | `test_controllers.cpp` | **Low** | Run with decreasing noise; verify LQG poles -> LQR poles |

---

## 7. What Is Working Well

The 05-25 report covered the H-inf synthesis, DARE doubling, and ADRC backward-Euler positive side. This pass adds:

**`DiscreteMPC::buildCostMatrix()` and QP solver structure** - the split between `buildPredictionMatrices()` (plant-dependent) and `buildCostMatrix()` (weight-dependent) is a clean design. `setParams()` correctly rebuilds only what changed. The `LDLT` pre-factorisation cached in `ldlt_` and reused in every `computeRef()` call eliminates per-step matrix factorisation. The pre-allocated work vectors (8 of them) make `computeRef()` allocation-free in steady-state operation. For real-time use cases this matters.

**`ControllerTraits` and static_assert enforcement** - the compile-time type safety for tuner/controller compatibility is genuinely useful and well-executed. The error messages name the right tuner to use instead, which is more helpful than most `static_assert` messages in practice. The soft-warning `[[deprecated]]` path for the LQG pole-placement case (which technically works but leaves the Kalman observer un-tuned) is the right call - a hard error there would be overly restrictive.

**`PlantModel::ss2tf()` using Faddeev-LeVerrier** - using the algebraic recurrence instead of eigenvalue expansion for the characteristic polynomial is the right choice. Eigenvalue expansion `∏(z - lambda_i)` for clustered or repeated eigenvalues is numerically catastrophic (Wilkinson polynomial effect). The current implementation is stable.

**`RepetitiveController`** - the plug-in architecture (wraps any IController, adds the periodic correction buffer) is composable and correct. The `Q < 1` forgetting factor handles model mismatch properly. The stability condition note in the header (sufficient condition: `Q / |1 + L(e^jomega) P(e^jomega)| < 1`) is exactly the right caveat.

---

*End of report.*
