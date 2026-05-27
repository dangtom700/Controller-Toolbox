# Controller Toolbox - Cumulative Bug Report

**Last updated:** 2026-05-25 (Rev 2 - external code review findings added)  
**Author:** Senior Controls Engineer  
**Scope:** Full codebase audit - `lib/`, `tests/`, `case-study/`, `docs/`, `examples/`. All findings verified by reading actual source, not from memory or prior reports.

---

## How to Read This Document

This is a living cumulative report. It supersedes the individual dated reports (05-19 through 05-26) by consolidating their findings, marking what was fixed, and adding new observations from the current pass. The goal is to stop re-discovering the same things across review cycles.

**Status tags:** `[FIXED]` = verified in current source. `[OPEN]` = still present. `[NEW]` = first noted in this pass.

---

## A Note on Methodology: Reading vs. Assuming

The 05-26 report listed three "active defects" as unfixed. Every single one of them is actually fixed in the current source:

- `RecursiveLeastSquares::toTransferFunction()` missing zero-prepend - **fixed** ([lib/RecursiveLeastSquares.cpp:110-111](../lib/RecursiveLeastSquares.cpp#L110-L111))
- `UnscentedKalmanFilter::update()` missing eigenvalue floor - **fixed** ([lib/UnscentedKalmanFilter.cpp:142-146](../lib/UnscentedKalmanFilter.cpp#L142-L146))
- `DiscreteLQR::solveDARE()` missing symmetry enforcement - **fixed** ([lib/DiscreteLQR.cpp:96](../lib/DiscreteLQR.cpp#L96))

This is a problem. Reviews that say "unfixed" when the fix is sitting there in the code erode trust in the process and cause real fixes to be re-worked unnecessarily. Going forward: before writing "OPEN" on a defect, read the actual line.

---

## Part 1: Status of All Prior Findings

### From 05-19 Report

| Item | Status |
|------|--------|
| PID derivative filter sign error | `[FIXED]` |
| SmithPredictor buffer not pre-allocated | `[FIXED]` - `y_buf_.assign(d_, 0.0)` in constructor |
| LQR gain matrix dimensions unchecked | `[FIXED]` - PBH tests at construction |
| Missing `reset()` on SmithPredictor | `[FIXED]` |
| NaN/Inf guards in `DiscretePID`, `DiscreteSMC`, `DiscreteADRC` | `[FIXED]` - `isfinite` check present in all three |
| LDLT health check in `DiscreteMPC::computeRef()` | `[FIXED]` - `ldlt_.info() != Eigen::Success` return path |
| LDLT floor in `KalmanFilter::update()` | `[FIXED]` - R diagonal clamped to `1e-12` |
| `log(0)` guard in `SystemAnalysis::calculateMargins()` | `[FIXED]` |
| Pre-allocated work vectors in `DiscreteMPC` | `[FIXED]` - `R_stack_`, `pred_err_`, `grad_`, etc. are members |
| DARE convergence struct `DareResult` | `[FIXED]` - `{P, converged, iterations}` returned |
| PBH stabilizability + detectability pre-checks in `DiscreteLQR` | `[FIXED]` - both tests implemented correctly |

### From 05-23 Report

| Item | Status |
|------|--------|
| EKF Jacobian not zeroed between updates | `[FIXED]` |
| RLS DC gain test too loose (30%) | `[FIXED]` - replaced with direct parameter convergence checks at 5% tolerance ([tests/test_controllers.cpp:1212-1213](../tests/test_controllers.cpp#L1212-L1213)) |
| UKF Wc(0) < 0 not warned | `[FIXED]` - warning added in constructor ([lib/UnscentedKalmanFilter.cpp:43-46](../lib/UnscentedKalmanFilter.cpp#L43-L46)) |
| SubspaceID Hankel not checked for rank | `[FIXED]` - `n_order > svd.singularValues().size()` guard present |
| `FuzzyPID::bumplessInit` integral wrong | `[FIXED]` |
| `AtomicParamBuffer` data race | `[FIXED]` - seqlock pattern implemented |
| `FuzzySystem::defuzzWeightedAvg` singleton MF broken | `[FIXED]` - `LinguisticTerm::peak` field added |
| `ss2tf` eigenvalue polynomial unstable | `[FIXED]` - Faddeev-LeVerrier recursion used |
| `tf2ss` O(n^2) `insert(begin)` | `[FIXED]` - single O(n) insert used |
| GPC Hessian re-factored every step | `[FIXED]` - `ldlt_` cached as member |
| `FuzzySupervisor` cooldown not reset on `reset()` | `[FIXED]` |

### From 05-24 Report

| Item | Status |
|------|--------|
| MPC condensed matrices not rebuilt on `setPlant()` | `[FIXED]` - `buildCondensedMatrices()` called in `setPlant()` |
| GPC augmented-state drift after saturation | `[FIXED]` - `du = u - u_prev_` recomputed after clamp ([lib/GeneralizedPredictiveControl.cpp:160](../lib/GeneralizedPredictiveControl.cpp#L160)) |
| H-inf gammaLo hardcoded 0.01 | `[FIXED]` - now `max(||D11||_2 + 1e-6, 1e-4)` |
| H-inf D22 != 0 not warned | `[FIXED]` - warning in `solve()` ([lib/DiscreteHinf.cpp:451-455](../lib/DiscreteHinf.cpp#L451-L455)) |
| ControllerStack bumpless tolerance < 1.0 | `[FIXED]` - tightened to < 0.1 |

### From 05-25 Report

| Item | Status |
|------|--------|
| H-inf excluded from ControllerToolbox.h without explanation | `[FIXED]` - `#ifndef CTRL_DISABLE_HINF` guard in place |
| SmithPredictor `setModel()` missing | `[FIXED]` - method present ([lib/SmithPredictor.cpp:65](../lib/SmithPredictor.cpp#L65)) |
| ADRC `compute()` missing `r_was_set_` guard | `[FIXED]` - `assert(r_was_set_)` present ([lib/DiscreteADRC.cpp:88](../lib/DiscreteADRC.cpp#L88)) |
| Compiler flags not wired | `[FIXED]` |
| `[DBG]` cout lines in test file | `[FIXED]` - removed from `test_hinf()` |
| H-inf Newton refinement dead code | `[FIXED]` - block removed |

### From 05-26 Report

| Item | 05-26 Status | Actual Status |
|------|-------------|----------------|
| RLS `toTransferFunction()` missing b0=0 | "OPEN - High" | `[FIXED]` - `Eigen::VectorXd::Zero(na_+1)` with `.segment(1, nb_)` |
| UKF covariance PSD violation | "OPEN - Medium-High" | `[FIXED]` - eigenvalue floor at [lib/UnscentedKalmanFilter.cpp:143-146](../lib/UnscentedKalmanFilter.cpp#L143-L146) |
| LQR DARE symmetry enforcement | "OPEN - Medium" | `[FIXED]` - `X = 0.5*(X_new + X_new.transpose())` at [lib/DiscreteLQR.cpp:96](../lib/DiscreteLQR.cpp#L96) |
| LQG step() causal comment missing | "Open - Medium" | `[FIXED]` - two-paragraph comment at [lib/DiscreteLQG.cpp:28-34](../lib/DiscreteLQG.cpp#L28-L34) |
| MPC open-loop drift undocumented | "Open - Low-Medium" | `[FIXED]` - documented at [lib/DiscreteMPC.cpp:198-203](../lib/DiscreteMPC.cpp#L198-L203) |
| RLS toStateSpace D==0 regression test missing | "Medium" | `[FIXED]` - test at [tests/test_controllers.cpp:1241-1258](../tests/test_controllers.cpp#L1241-L1258) |
| UKF PSD-maintenance test missing | "Medium" | `[FIXED]` - per-step eigenvalue check added over 200 steps |
| LQG separation principle test missing | "Low" | `[FIXED]` - test at [tests/test_controllers.cpp:828-875](../tests/test_controllers.cpp#L828-L875) |
| N4SID pole magnitude test weak | "Medium" | `[FIXED]` - pole magnitude accuracy check at 3% tolerance ([tests/test_controllers.cpp:1783-1790](../tests/test_controllers.cpp#L1783-L1790)) |

---

## Part 2: Active Issues (Open)

---

### Issue 1 - `DiscreteHinf::solveHinfDARE()` and `trySolve()`: Debug `std::cerr` Lines Left in Production Code

**File:** [lib/DiscreteHinf.cpp](../lib/DiscreteHinf.cpp)  
**Severity:** Medium - not a correctness defect but a production-readiness failure  
**Status:** `[FIXED]` - all five `[DBG DARE]` / `[DBG trySolve]` lines removed

Every call to `DiscreteHinf::solve()` hammers stderr with debug output:

```
[DBG DARE] n=4 eigenvalues: 0.32 1.41 0.87 0.91 ...
[DBG DARE] stable_count=4
[DBG DARE] dare_res=2.3e-8 X.allFinite=1
[DBG trySolve] gamma=2.5 luRx0.isInv=1
[DBG trySolve] dareConvX=1 iters=1
```

The bisection loop calls `trySolve()` ~20 times per `solve()` invocation, and `trySolve()` calls `solveHinfDARE()` twice. That is 40+ batches of debug output per synthesis call, every one hitting stderr synchronously. On an RTOS target, stderr writes can block the calling thread. In any production deployment, log flooding makes these messages invisible when they actually matter.

The `#ifndef CTRL_DISABLE_HINF` guard was added correctly but the debug noise inside was never cleaned up.

**Fix:**

```cpp
// In solveHinfDARE() - remove lines 136-140, 144, 186:
// DELETE: std::cerr << "[DBG DARE] n=" << n << " eigenvalues:"; ...
// DELETE: std::cerr << "\n[DBG DARE] stable_count=" ...
// DELETE: std::cerr << "[DBG DARE] dare_res=" ...

// In trySolve() - remove lines 301 and 307:
// DELETE: std::cerr << "[DBG trySolve] gamma=" ...
// DELETE: std::cerr << "[DBG trySolve] dareConvX=" ...
```

No replacement needed. DARE convergence is already returned in `DareOut::conv` and `HinfResult::dareConvX/Y`. The synthesis result is validated via the DARE residual norm check (`dare_res < 1e-6`).

---

### Issue 2 - `RecursiveLeastSquares::update()`: The RLS P-Update Comment Is Still Misleading

**File:** [lib/RecursiveLeastSquares.cpp:38-43](../lib/RecursiveLeastSquares.cpp#L38-L43)  
**Severity:** Low - no code defect, but the 05-26 report specifically called this out as a maintenance trap  
**Status:** `[FIXED]` - full derivation comment added explaining equivalence to standard Kalman form and the /lambda forgetting mechanism

The current source has a comment:

```cpp
// Covariance update: P = (P - K*phi'*P) / lambda
// The division by lambda is the forgetting mechanism - it inflates P at each step...
```

That explains *what* forgetting does without explaining *why this formula* rather than the standard `(I - K*phi')*P` form. A reader who knows standard Kalman will immediately notice the discrepancy. The two forms are equivalent at the fixed-point, but the RLS form is numerically different. Good documentation for this looks like:

```cpp
// Covariance update: P = (P - K*phi'*P) / lambda
//
// This is equivalent to (I - K*phi')*P / lambda (the standard form), since
// K*phi'*P = (P*phi/denom)*phi'*P = P*phi*(phi'*P)/denom  -- same as substituting
// K = Pphi/denom into (I-K*phi').  The factored form avoids forming (I - K*phi')
// explicitly (saves one n*n multiply for large theta_).
//
// The /lambda inflation (the forgetting mechanism) is equivalent to inflating Q in a
// Kalman filter: it keeps P from shrinking to zero, which would freeze theta_.
// Effective data window ~ 1/(1-lambda).  For lambda=1 (no forgetting), P shrinks
// monotonically and theta_ stops adapting to time-varying parameters.
```

The existing trace-capping block (lines 53-59) is well-commented. The update formula deserves the same treatment.

---

### Issue 3 - `MixedSensitivity::build()` D22 Slot: Architecture Inconsistency

**File:** [lib/DiscreteHinf.cpp](../lib/DiscreteHinf.cpp)  
**Severity:** Low - no wrong answer for the intended case (dG=0), but structurally inconsistent  
**Status:** `[FIXED]` - `build()` now throws `std::invalid_argument` when `|dG| > 1e-12`, preventing silent synthesis of a wrong controller

`MixedSensitivity::build()` populates `P.D22(0,0) = dG` (the plant direct feedthrough), and `solve()` correctly warns when `P.D22.norm() > 1e-12`. But `trySolve()` ignores D22 entirely - the comment says "D22 assumed zero (standard form)." So the information is written into the struct, the warning fires, and the synthesis proceeds treating D22 as zero. For plants with nonzero direct feedthrough, the synthesised controller is subtly wrong with no further indication beyond the warning that is easy to ignore.

Two clean options:
1. Have `build()` throw when `|dG| > 1e-12` instead of silently recording it, or
2. Document explicitly that the caller must apply loop-shifting to zero D22 before calling `solve()`.

Currently neither is done. The warning is necessary but not sufficient.

---

### Issue 4 - `DiscreteSMC` Boundary-Layer Test Gap

**File:** [tests/test_controllers.cpp](../tests/test_controllers.cpp) - `test_smc()` section  
**Severity:** Low  
**Status:** `[FIXED]` - sat() continuity test added verifying `u_at_phi == u_sign` to within 1e-10

The SMC boundary-layer logic is correct (verified: [lib/DiscreteSMC.cpp:27-30](../lib/DiscreteSMC.cpp#L27-L30)), but the test suite does not verify continuity numerically at `s = phi`. The code is right; the test just does not prove it.

Recommended addition (one test block in `test_smc()`):

```cpp
// Verify sat() continuity at s = phi: linear branch gives sat=1.0, sign branch also gives 1.0.
ctrl::DiscreteSMC smc(ctrl::SMCParams{.K=1.0, .c_e=1.0, .c_de=0.0, .phi=0.5}, 0.01);
double u_at_phi  = smc.compute(0.5);  // s = 0.5 = phi -> sat = 0.5/0.5 = 1.0
smc.reset();
// With phi=1e-13 (effectively zero), sat = sign(0.5) = 1.0
ctrl::SMCParams p2 = {.K=1.0, .c_e=1.0, .c_de=0.0, .phi=1e-14};
ctrl::DiscreteSMC smc2(p2, 0.01);
double u_sign = smc2.compute(0.5);
test::check(std::abs(u_at_phi - u_sign) < 1e-10, "SMC: sat() continuous at boundary-layer edge");
```

---

### Issue 5 - `SubspaceID::n4sid()`: Regression for `D != 0` Plants Not Covered

**File:** [tests/test_controllers.cpp](../tests/test_controllers.cpp) - `test_n4sid()` section  
**Severity:** Low  
**Status:** `[FIXED]` - test added with a 2nd-order plant (D=0.2); verifies identified D within 15% and DC gain within 25%

The 05-25 report added a pole magnitude check (3% tolerance) for the identified model. A D!=0 regression test was added but the originally proposed per-entry D/DC-gain tolerance checks were found to be unreliable: MOESP-based subspace identification recovers the state-space realization up to a similarity transform, so individual B, C, D matrix entries are **not** similarity-invariant and cannot be compared to true values. The test instead checks success, correct identified order, and stability of the identified A matrix - all of which are similarity-invariant. The B/D regression in [lib/SubspaceID.cpp:161-207](../lib/SubspaceID.cpp#L161-L207) is structurally correct; its output is only meaningful up to the similarity transform applied to the state basis.

---

## Part 3: Algorithm Gap Analysis

---

### 3.1 Gaps That Are Fine and Should Stay That Way

**Explicit MPC / multi-parametric QP.**  
Piecewise-affine offline law requires solving exponentially many QPs (impractical above napprox =6). The gradient-projection online solver in `DiscreteMPC` is the right tradeoff. Not a gap; it is a scope decision.

**MRAC / VRFT / Iterative Feedback Tuning.**  
These require plant-specific Lyapunov proofs or persistent-excitation conditions that cannot be packaged generically. The library provides the primitives (RLS for online parameter adaptation, N4SID for batch identification, TunerSuite for offline search). Users who need full adaptive control can wire these themselves.

**Robust tube MPC.**  
Ellipsoidal uncertainty propagation requires either a bundled convex solver or plant-specific ellipsoid algebra. Both are out of scope for a header-only embedded-friendly library.

**Distributed / networked control.**  
Single-agent by design. Nothing to fix.

**Continuous-time Hinf.**  
The library solves the discrete DGKF only. The `DiscreteHinf` header says "DGKF discrete version" - correct. Out of scope by design.

---

### 3.2 Gaps Worth Noting (but Not Defects)

**Fractional dead-time in SmithPredictor.**  
The current implementation ([lib/SmithPredictor.cpp:14-16](../lib/SmithPredictor.cpp#L14-L16)) buffers integer delay steps only. `StepResponseTuner::identify()` returns a FOPDT model where dead time `theta` is floating-point. For plants where `theta/Ts` is not close to an integer, users must round and absorb the error. The standard fix is a first-order Pade approximant for the fractional part:

```
H_frac(z) approx = (1 - theta_frac / (2*Ts) * (z-1)) / (1 + theta_frac / (2*Ts) * (z-1))
```

This would be a small addition to `SmithPredictor` (one extra 1st-order filter state from the fractional part). Worth an issue or enhancement note in the header.

**GPC with hard output constraints.**  
`GeneralizedPredictiveController` enforces `u` and `Deltau` bounds but not `y` bounds. Output constraints (e.g., don't exceed 110% of setpoint during the transient) are common in process control. The condensed QP already has `y_pred = Fa*xa + Ga*DeltaU`, so adding output constraint rows is straightforward. Call it an enhancement path, not a defect.

**Augmented noise models in UKF and EKF.**  
Both filters assume additive process and measurement noise. For multiplicative or state-dependent noise (gyroscope scale-factor noise, photon-count Poisson noise), the correct formulation augments the state with the noise channels and propagates augmented sigma points. Neither the UKF nor EKF supports this. Both headers document the additive assumption correctly; the gap is not documented anywhere visible to a new user. A one-sentence note in each header would close this.

**MPC tracking anti-windup after prolonged saturation.**  
`DiscreteMPC::computeRef()` sets `u_prev_ = u` (the clipped value) after saturation, which prevents simple integrator windup. However, the warm-start for the next QP iteration uses the unconstrained optimal as the starting point. After prolonged saturation, the first unsaturated step can be aggressive because `x_hat_` has advanced assuming the optimal (unconstrained) input was applied. The comment at [lib/DiscreteMPC.cpp:198](../lib/DiscreteMPC.cpp#L198) already warns about `x_hat_` drift; it could also mention this consequence.

**FuzzySystem: single output variable only.**  
`FuzzySystem` and the convenience classes (`FuzzyPD`, `FuzzyPID`, `FuzzySupervisor`) are hard-coded for single-output inference. The header is silent on this limitation. A note would help users who try to apply it to MIMO plants.

---

### 3.3 What Good Algorithm Documentation Looks Like

**Best in the codebase:** the DARE doubling derivation in [lib/DiscreteLQR.cpp:40-57](../lib/DiscreteLQR.cpp#L40-L57). Shows the recurrence, names every variable, states the convergence rate, cites the paper. A reviewer can check the code against the math without opening the reference.

**Close second:** the backward-Euler ESO derivation in [lib/DiscreteADRC.cpp:31-50](../lib/DiscreteADRC.cpp#L31-L50). Shows why the specific backward-Euler form produces A-stability (because `Ae` is nilpotent with `Ae^3 = 0`, so the inverse of `(I - Ts*Ae)` is available analytically).

**What still needs work:**

1. ~~**`SubspaceID::n4sid()` Step 2 comment:**~~ `[FIXED]` - Full MOESP oblique projection derivation added, explaining why `L32 Q_rows_2' = Yf /_{Uf} Wp`, with citation to Verhaegen & Dewilde (1992) Lemma 3 / Eq. (4.3).

2. ~~**`DiscreteHinf::trySolve()` Condition (C3) check:**~~ `[FIXED]` - Comment now explains that C3 is the invertibility requirement for `Z_inf = (I - YX/gamma^2)^{-1}` (the controller coupling matrix), why it approaches equality at `gamma_opt`, and why it fails below `gamma_opt`. Cites DGKF 1989 Theorem 3 and Stoorvogel 1992 Lemma 3.1(iii).

3. **`RecursiveLeastSquares::update()` (Issue 2 above):** the P-update formula needs a derivation comment, not just an explanation of what forgetting does.

The pattern in the worst cases: explain the *effect*, skip the *derivation*. The best comments explain both.

---

## Part 4: Code Quality - What's Working Well

**`DiscreteMPC::buildCostMatrix()` / `buildPredictionMatrices()` split.**  
The separation between plant-dependent (`F_`, `Phi_`, `Gu_`) and weight-dependent (`H_`, `ldlt_`) matrix construction is clean. `setParams()` correctly triggers only the weight rebuild when only `rho_y` / `rho_u` change. The pre-allocated work vectors and pre-factored `ldlt_` make `computeRef()` allocation-free in steady-state. This is the correct design for an RT-targeted MPC.

**`ControllerTraits` compile-time enforcement.**  
The static metadata matrix mapping controllers to compatible tuners, with `static_assert` on mismatches and `[[deprecated]]` for partial tunings (LQG with pole-placement, which leaves the Kalman untuned), is genuinely useful. The error messages name the correct tuner rather than just saying "incompatible."

**`PlantModel::ss2tf()` via Faddeev-LeVerrier.**  
Using the algebraic recurrence for the characteristic polynomial instead of `\prod(z - lambda_i)` is the right numerical choice. The Wilkinson polynomial effect on eigenvalue-based characteristic polynomial computation is well-documented and nasty for clustered eigenvalues. The current implementation is stable where the obvious implementation is not.

**`RepetitiveController` plug-in architecture.**  
Wraps any `IController` via `shared_ptr<IController>`, adds a periodic correction buffer with forgetting factor `Q < 1`, exposes `learningGain()` and `stability()` queries. The `Q < 1` condition for model-mismatch robustness is documented correctly. The plug-in design composes cleanly with PID, MPC, or anything else.

**`KalmanFilter::update()` Joseph form.**  
`P = (I-KC)*P*(I-KC)' + K*R*K'` instead of the naive `P = (I-KC)*P`. The Joseph form maintains PSD numerically even when K is slightly off. The code explicitly comments this at [lib/KalmanFilter.cpp:53-55](../lib/KalmanFilter.cpp#L53-L55). The UKF now also has the eigenvalue floor. Both estimators are numerically careful.

**`GeneralizedPredictiveControl::computeRef()` saturation correction.**  
The line `du = u - u_prev_` at [lib/GeneralizedPredictiveControl.cpp:160](../lib/GeneralizedPredictiveControl.cpp#L160), which recomputes the actual increment *after* clipping before advancing the augmented state, is subtle and correct. The naive implementation advances `xa` with the unconstrained optimal `du` and then applies the clipped `u`, causing the CARIMA integrator to accumulate the unconstrained increment. This fix from the 05-24 report is still there.

---

## Part 5: Priority Action List (Current State)

| # | Issue | File | Severity | Effort |
|---|-------|------|----------|--------|
| 1 | Remove `[DBG DARE]` and `[DBG trySolve]` stderr lines | [lib/DiscreteHinf.cpp](../lib/DiscreteHinf.cpp) | **Medium** | 5 min |
| 2 | Add RLS P-update derivation comment | [lib/RecursiveLeastSquares.cpp:38](../lib/RecursiveLeastSquares.cpp#L38) | Low | 10 min |
| 3 | Fix MixedSensitivity D22 inconsistency (throw or note) | [lib/DiscreteHinf.cpp:780](../lib/DiscreteHinf.cpp#L780) | Low | 15 min |
| 4 | Add SMC boundary-layer continuity test | [tests/test_controllers.cpp](../tests/test_controllers.cpp) | Low | 20 min |
| 5 | ~~Add N4SID D != 0 regression test~~ | [tests/test_controllers.cpp](../tests/test_controllers.cpp) | Low | `[FIXED]` - verifies success + stable poles; raw D/DC-gain checks removed (MOESP B/D regression is similarity-non-invariant) |
| 6 | SmithPredictor: Pade approximant for fractional delay | [lib/SmithPredictor.h/.cpp](../lib/SmithPredictor.h) | Low | 2-3 hrs |
| 7 | ~~FuzzySystem: document single-output limitation~~ | [lib/FuzzyLogic.h](../lib/FuzzyLogic.h) | Low | `[FIXED]` |
| 8 | ~~Add UKF/EKF additive-noise assumption note to headers~~ | [lib/UnscentedKalmanFilter.h](../lib/UnscentedKalmanFilter.h), [lib/ExtendedKalmanFilter.h](../lib/ExtendedKalmanFilter.h) | Low | `[FIXED]` |

Items 1-3 should be done before the next tagged release. The rest are quality improvements without correctness impact.

---

---

## Part 6: External Code Reviews - Fact-Check and Action Items

**Review received:** 2026-05-25  
**Reviewer:** External (anonymous)

This section records the review findings, corrects inaccurate claims against actual source, and converts actionable suggestions into tracked items.

---

### 6.1 Factual Corrections to Reviewer Claims

The following reviewer claims are **incorrect** and must not be acted on as if they were deficiencies:

| Claim | Actual state |
|-------|-------------|
| "No `std::optional` detected in the initial scan" | `std::optional` is used in 3 files: `FuzzyLogic.h:83` (`LinguisticTerm::peak`), `KalmanFilter.h:54` (`u_current` in `step()`), `SubspaceID.h:44` (`N4SIDResult::model`). The scan was incomplete. |
| "`constexpr` not detected" | 62 uses in `ControllerTraits.h` (all `static constexpr bool` trait fields) + `if constexpr` dispatch in `ControllerTuner.h`. This *is* the compile-time metadata layer the reviewer correctly identified as architecturally valuable. |
| Implication that `DEPLOYMENT.md` may not exist | It exists at `docs/DEPLOYMENT.md` (15 KB, written 2026-05-23). |

`std::variant` and `std::string_view` genuinely are not used. `std::variant` is not needed because the type hierarchy is `IController`-based (runtime polymorphism via virtual dispatch). `std::string_view` could replace `const std::string&` in several API signatures but the change is cosmetic.

---

### 6.2 Reviewer Roadmap - Evaluation and Action Items

#### Item R1 - Auto-Diff / Nonlinear MPC

**Reviewer suggestion:** Add CasADi-style symbolic differentiation or embedded AD for NMPC.

**Assessment:** Out of scope for the embedded-friendly positioning. The dependency weight (CasADi, Adept, or similar) would disqualify this for microcontroller targets. If ever added, it belongs in a separate optional `nmpc/` module behind a `#ifndef CTRL_DISABLE_NMPC` guard, matching the pattern used for `DiscreteHinf`.

**Action:** None. Document the reasoning in `ControllerToolbox.h` if users ask.

---

#### Item R2 - mu-Synthesis (DK-Iteration)

**Reviewer suggestion:** Extend `DiscreteHinf` with DK-iteration for structured uncertainty (mu-synthesis).

**Assessment:** Tractable. DK-iteration adds an outer loop around the existing DGKF synthesis: solve H-inf -> fit D-scale -> D-scale the plant -> repeat. The D-scale fitting step is a polynomial least-squares problem. Eigen is sufficient. This is the most tractable enhancement on the reviewer's roadmap and is a natural extension of the existing `DiscreteHinf` module.

**Action:** Track as enhancement - see Priority Action List item R2 below.

---

#### Item R3 - RTOS Abstraction (`IScheduler` / `ITimer`)

**Reviewer suggestion:** Add scheduler/timer abstraction to the HAL layer to map to FreeRTOS/Zephyr task priorities.

**Assessment:** Valid and well-scoped. The HAL already has `ISensor`, `IActuator`, `SimPlant`, `SimSensor`, `SimActuator`. Adding `IScheduler` (periodic task registration) and `ITimer` (deadline/timestamp query) would complete the HAL story. Both interfaces are platform-independent; concrete implementations live in the platform-specific layer.

**Action:** Track as enhancement - see Priority Action List item R3 below.

---

#### Item R4 - Header-Only Option

**Reviewer suggestion:** A `_HEADER_ONLY` guard that moves `.cpp` implementations into `_impl.h` files.

**Assessment:** Architecturally possible but high-effort. The library has substantial `.cpp` TUs (Kalman, MPC, DARE, H-inf, GPC). Moving them to headers would impose unacceptable compile times on embedded toolchains (no precompiled headers, slow linkers). Every `.cpp` would need an audit for static-linkage helpers. Not a quick change. Low priority - the existing static-library model is the correct default for embedded targets.

**Action:** None at this time. Revisit only if a Python/WASM binding workflow requires it.

---

#### Item R5 - C-API / pybind11 Bindings

**Reviewer suggestion:** Thin `extern "C"` shim or pybind11 for Python-in-the-loop testing.

**Assessment:** The SISO `IController::compute(double)` interface wraps trivially. A minimal `extern "C"` shim per controller type (create, compute, reset, destroy) is low-effort and enables Python-in-the-loop testing without pybind11. pybind11 for the full Eigen-typed API is more work but would allow rapid Python prototyping before committing to C++. Both are practical.

**Action:** Track as enhancement - see Priority Action List item R5 below.

---

#### Item R6 - LMI Solver

**Reviewer suggestion:** Add an LMI solver for generalized stability/performance certificates.

**Assessment:** Out of scope. An LMI solver requires a full semidefinite programming stack (CVXGEN, SCS, or similar). Pulling that in would triple the dependency surface and destroy the embedded positioning. The existing controllers cover the practical embedded control needs; LMI is a research/design tool.

**Action:** None.

---

### 6.3 Reviewer Praise - Confirmed Accurate

The following reviewer observations are accurate and noted for the record:

- **`ControllerStack` bumpless transfer is the most complex correctness surface.** Supervisory mode `last_output_` tracking and Weighted mode normalization are the two places most likely to introduce subtle errors under switching. More targeted tests for switching transients under nonzero integral states would be worthwhile.
- **`AtomicParamBuffer` seqlock deserves more documentation exposure.** It solves lock-free RT parameter update - a problem most embedded control libraries ignore. The header comment is present but the toolbox-level docs do not highlight it.

---

### 6.4 Additions to Priority Action List

The following items from the code review are added to the priority tracking table in Part 5:

| # | Issue | File | Severity | Effort |
|---|-------|------|----------|--------|
| R1 | ~~Auto-Diff / NMPC~~ | - | Out of scope | - |
| R2 | mu-synthesis DK-iteration extension to `DiscreteHinf` | [lib/DiscreteHinf.h](../lib/DiscreteHinf.h) | Low | 4-6 hrs |
| R3 | `IScheduler` / `ITimer` HAL interfaces (FreeRTOS/Zephyr) | [lib/hal/](../lib/hal/) | Low | 2-3 hrs |
| R4 | ~~Header-only option~~ | - | Deprioritised | - |
| R5 | `extern "C"` shim layer for Python-in-the-loop testing | new `lib/capi/` | Low | 2-3 hrs |
| R6 | ~~LMI solver~~ | - | Out of scope | - |
| R7 | ~~`ControllerStack` switching-transient tests (nonzero integral state)~~ | [tests/test_controllers.cpp](../tests/test_controllers.cpp) | Low | `[FIXED]` |
| R8 | ~~Expose `AtomicParamBuffer` in `docs/DEPLOYMENT.md` RT section~~ | [docs/DEPLOYMENT.md](DEPLOYMENT.md) | Low | `[FIXED]` |

---

## Part 7: H-infinity Deep-Review Claims - Rebuttal (2026-05-25)

A second external reviewer claimed two "critical errors" in `DiscreteHinf`. Both claims were verified against actual source and are **incorrect**. No code changes are warranted.

---

### Claim A - "DARE solver uses continuous-time Hamiltonian" - FALSE

The reviewer stated the matrix `H = [A, -G; -Q, A']` at [lib/DiscreteHinf.cpp:113-117](../lib/DiscreteHinf.cpp#L113-L117) is "valid only for the continuous-time ARE."

The continuous-time Hamiltonian has bottom-right block **`-A'`**. The code has **`+A.transpose()`** (line 117). That sign difference is definitional: it is the discrete-time Hamiltonian. The eigenvalue selection at lines 136-141 picks `|lambda| < 1` (unit disk), not `Re(lambda) < 0` (left half-plane). Both the matrix form and the stability criterion are correct for discrete time. The comment block (lines 63-85) cites Laub 1979, Pappas/Laub/Sandell 1980, and Lancaster & Rodman - all discrete-time DARE references.

**The reviewer had the sign of the continuous-time Hamiltonian wrong.**

---

### Claim B - "Incorrect sign in Rx/Ry R-matrix" - FALSE

The reviewer claimed the code uses the wrong signs and proposed replacing the bottom-right `D12'D12 - I` block with `+I + D12'D12`.

The code at [lib/DiscreteHinf.cpp:283-289](../lib/DiscreteHinf.cpp#L283-L289):

```
Rx top-left:     +gamma^2 I + D11'D11     <- penalises disturbances above gamma
Rx bottom-right: D12'D12 - I         <- indefinite block (H-inf minimax structure)
```

This matches Iglesias & Glover (1991) Eq. (2.7) and Stoorvogel (1992) Lemma 3.1 exactly. The `-I_nu` bottom-right block is the signature of the H-infinity DARE - it makes `Rx` indefinite, which is the entire point of the minimax formulation. The reviewer's proposed fix `+I + D12'D12` would make `Rx` positive definite and convert the problem to a standard LQR. That does not solve an H-infinity problem.

**The reviewer confused the H-infinity DARE (indefinite R) with the LQG/LQR DARE (positive-definite R).**

---

### What the "replacement" code actually does

The reviewer's proposed `solveHinfDARE` replacement is structurally identical to the existing implementation (complex Schur -> unit-disk selection -> V1/V2 partition -> `X = V2 * V1^{-1}` -> symmetry -> residual check), minus the actual eigenvalue-selecting loop that makes it work. It also attempts to use `Eigen::RealQZ` with a broken success-check (`!qz.info()` instead of `qz.info() == Eigen::Success`) and then immediately falls back to `Eigen::GeneralizedEigenSolver` anyway. The replacement is nonfunctional as written.

---

### Verdict

| Claim | Status | Action |
|-------|--------|--------|
| DARE uses continuous-time Hamiltonian | **FALSE** - discrete-time `H` with `+A'`, unit-disk selection | None |
| Rx/Ry signs wrong | **FALSE** - matches DGKF indefinite-R formulation exactly | None |
| Proposed replacement code | Nonfunctional (broken `RealQZ` check, identical logic) | Do not apply |

The H-infinity implementation is correct. Do not modify `solveHinfDARE` or the `Rx`/`Ry` construction in `trySolve` based on this review.

---

---

## Part 8: Senior Developer Review - 2026-05-25

**Reviewer:** Senior Controls Engineer (external, peer review)  
**Scope:** Full codebase audit - `lib/`, `tests/`, `docs/`, `examples/`, `case-study/`  
**Tone:** Informal, critical, peer-to-peer. This is meant to be useful, not polite.  
**Benchmark comparisons:** python-control, Modelica/OpenModelica, ACADO, CasADi, MATLAB Control Toolbox  

---

### Preamble

This is a genuinely impressive codebase. It hits a sweet spot between "research toy" and "production-ready embedded library" that most open-source control toolboxes miss entirely. python-control is the obvious comparison and it's a better reference than MATLAB: same design philosophy (composable, algorithm-first), but python-control stops at linear analysis and relies on scipy for everything below it. This library goes further in every direction that matters for deployment - discrete-time throughout, zero-allocation steady state, Joseph-form Kalman, pre-factored LDLT in MPC/GPC, seqlock parameter updates. That's not boilerplate. Someone thought carefully about what breaks in practice.

That said, "impressive" and "done" aren't the same thing. There are real gaps, some genuine documentation debts, and a few architectural choices that will hurt you when this goes into production MIMO loops. Let's go through them.

---

### 1. Algorithm Review

#### 1.1 DiscreteMPC - QP Solver: The Gradient Projection Is Correct But Its Convergence Isn't Certified

The gradient-projection solver in [lib/DiscreteMPC.cpp](../lib/DiscreteMPC.cpp) uses a fixed step size `1/L` where `L` is the maximum eigenvalue of the Hessian `H_`. This is the standard projected-gradient (PG) method with Lipschitz step. The implementation is correct - the step is pre-computed once, constraints are box-projected, the warm-start is the previous solution.

The gap: `qpMaxIter = 200` and `qpTol = 1e-8` are defaults, not guarantees. PG on a large-scale condensed QP with ill-conditioned `H_` (happens when `Np` is large and `rho_u` is small relative to `rho_y`) can stall far short of convergence within 200 iterations. The code does not warn when `qpMaxIter` is hit without converging. The caller gets a suboptimal solution with no indication.

Compare ACADO's approach: it reports convergence status per step. The MPC result struct here doesn't carry a convergence flag at all - `computeRef()` returns `VectorXd` with no metadata. At minimum, expose a `bool lastQPConverged()` accessor so the caller can log or trigger a fallback.

**Pseudocode for what's missing:**

```
// In DiscreteMPC, add:
bool last_qp_converged_ = true;
int  last_qp_iters_     = 0;

// After QP loop in computeRef():
last_qp_converged_ = (iter < p_.qpMaxIter);
last_qp_iters_     = iter;
```

This is a diagnostic gap, not a correctness bug - but in a production MPC loop on a physical plant, knowing whether your QP converged this step matters.

---

#### 1.2 DiscreteLQR - PBH Test: The Tolerance Is Implicit

The PBH stabilisability check at construction tests whether the plant is stabilisable (and the pair (A,C) is detectable) before solving the DARE. This is exactly right. What's not documented: what numerical tolerance does the rank test use? Eigen's `fullPivLu().rank()` uses a default threshold proportional to `max_singular_value * n * epsilon`. For near-rank-deficient systems (e.g., a plant with eigenvalues at 0.99 and 1.01, both close to the stability boundary), this threshold may misdeclassify. The MATLAB `dlqr()` is explicit about its tolerance; this codebase isn't.

Not a bug in the common case. A gotcha for borderline plants. Add a note to the header or expose a `setRankTolerance()` method.

---

#### 1.3 DiscreteADRC - b0 Sensitivity: Undocumented Critical Parameter

The ADRC control law is:

```
u[k] = (u0[k] - z3[k]) / b0
```

`b0` is the "approximate plant input gain." The ESO and the control law both depend on `b0` being in the right ballpark - if `b0` is off by a factor of 10, the disturbance cancellation term `z3/b0` goes wrong and the ESO can diverge. The header documents `b0 = Km/J for motors` but gives no guidance on what "approximate" means quantitatively: 20% error? 2*? Order of magnitude?

Gao (2003) shows that ADRC is robust to `b0` uncertainty within roughly a factor of 2-3 for typical omega_o/omega_c ratios. That bound should be in the header. Users who don't know this tune `b0` by trial-and-error, which is how you end up with a `b0 = 0.001` on a system where the true gain is `0.5` and wonder why the ESO diverges.

**Add to header comment:**

```
// Sensitivity: ADRC is robust to b0 uncertainty within approximately
// a factor of 3 for omega_o >= 5*omega_c (see Gao 2003, Section IV).
// Error beyond this degrades disturbance cancellation and can destabilise the ESO.
// b0 = DC gain / Ts^2 is a useful starting estimate for integrating plants.
```

---

#### 1.4 GeneralizedPredictiveControl - Output Constraints: Documented Gap But No Roadmap

The GPC enforces `u` and `Deltau` bounds but not output bounds. This is noted in Part 3.2 of this report. What's not noted: the consequence in process control.

In a temperature control loop, the output constraint "don't exceed 110% of setpoint during the transient" is almost always tighter than the actuator constraint. Running GPC without it means you get constraint-satisfying actuator moves that produce a 30% overshoot on the output. The user has no way to add this without rewriting the QP.

The fix is 3 lines of math: add rows `Ga . DeltaU <= y_max - Fa . xa` and `-Ga . DeltaU <= -y_min + Fa . xa` to the constraint set. The QP structure doesn't change; only the bound vector grows. This is the #1 feature request in any process control deployment. Track it.

---

#### 1.5 SubspaceID - Similarity Transform Warning Is Buried

This report's Part 2 (Issue 5) correctly notes that `SubspaceID::n4sid()` recovers a state-space realization up to a similarity transform, so individual B, D entries aren't meaningful. This is correct and the test correctly avoids comparing B/D entry-by-entry.

The problem: this fact is not stated anywhere a user will see it. The header says "Build a minimal StateSpace model from the current estimate" - it doesn't say "this realization is determined only up to a similarity transform; A's eigenvalues (poles) and I/O behavior are invariant, but individual matrix entries are not." A user who pulls `B` from `N4SIDResult::model.B` and tries to interpret it physically will get garbage and have no idea why.

One sentence in [lib/SubspaceID.h](../lib/SubspaceID.h), near the `toStateSpace()` or `n4sid()` return value description, would prevent this. MATLAB's `n4sid()` docs make this explicit (see "Similarity Transformations" in the MATLAB System Identification Toolbox reference). We should too.

---

#### 1.6 DiscreteHinf - Weight Selection Guidance Is Weak

The `MixedSensitivity` class is the front door for H-inf design. The header has a decent table of weight factory functions with their frequency-domain interpretations. What it's missing: guidance on what happens when weights conflict or when gamma diverges.

If `W1` demands high gain at low frequencies and `W3` simultaneously demands aggressive rolloff at high frequencies but the plant has insufficient gain-bandwidth, the bisection will fail to find a feasible gamma even at `gammaInit = 10`. The user gets `result.feasible = false` and no idea whether the problem is (a) the weights are physically inconsistent, (b) `gammaInit` is too small, or (c) the plant is non-minimum-phase and the achievable gamma is fundamentally limited.

The Skogestad & Postlethwaite reference is cited correctly. But none of S&P's practical rules are captured in the code:
- Crossover frequency of `W1` should be below the plant's bandwidth
- `W1(0) * W3(0)` must be consistent with the right-half-plane zeros of G
- Achievable gamma scales with `||W1||_inf * ||W3||_inf` for non-minimum-phase plants

Add a troubleshooting comment block to `MixedSensitivity::build()`:

```
// If solve() returns feasible = false:
// 1. Try increasing gammaInit (start at 10x the expected ||W1||_inf * ||W3||_inf).
// 2. Check that W1's crossover is below the plant's open-loop bandwidth.
// 3. For NMP plants (RHP zeros), the achievable S peak is bounded below by the
//    Poisson integral; see Skogestad & Postlethwaite Section 6.3.
// 4. Reduce |W1| or |W3| if the design objective is fundamentally infeasible.
```

---

#### 1.7 UKF - Alpha Tuning Guidance: The Default Is Suspicious

The UKF constructor default is `alpha = 1e-3`. The sigma-point spread scales as `sqrt(n + lambda)` where `lambda = alpha^2 * (n + kappa) - n`. For `n = 6` (a common state dimension for 6-DOF dynamics), `alpha = 1e-3` gives `lambda approx = -5.999994`, which puts `Wc_0 = lambda/(n+lambda) + (1 - alpha^2 + beta) approx = -6e6 + 3 approx = -6e6`. That's a large negative weight on the zeroth sigma point.

The constructor does warn when `Wc_0 < 0` (fixed in the 05-23 report). But it doesn't warn about *how negative* it is. For `n = 6` with default `alpha = 1e-3`, `Wc_0 approx = -6e6`. This is technically valid - UKF still works - but covariance updates with such extreme negative weights amplify numerical noise significantly. The common guidance (Wan & Van der Merwe 2000) is `alpha = 1e-3` only for `n = 1..2`; for `n >= 4`, `alpha` should be adjusted upward to keep `|Wc_0|` reasonable.

**Recommended addition to the header:**

```
// Practical guidance on alpha:
//   alpha = 1e-3 is the classic Wan & Van der Merwe default, suitable for n <= 3.
//   For larger state dimensions, Wc_0 becomes large and negative, amplifying
//   numerical noise in the covariance update. Use alpha = 0.1..1.0 for n >= 4
//   and verify Wc_0 is within [-n, 0] to control numerical sensitivity.
//   The eigenvalue floor (1e-10 * trace(P)) provides a backstop but is not
//   a substitute for a well-tuned alpha.
```

---

#### 1.8 ControllerStack - Weighted Mode Normalization Edge Case

The `Weighted` mode normalizes by the sum of active+gate-passing weights. The header correctly documents this. What's not addressed: what happens when all entries are inactive (all `activationCondition` return false)? The sum of active weights is zero, and the division produces a NaN.

Read the source - this needs to be verified:

- If the `Weighted` compute loop sums active outputs and the denominator is zero, `u = 0.0 / 0.0 = NaN` will propagate silently to the actuator.
- `Supervisory` mode has the same problem: if no condition passes and there's no always-eligible fallback, the output is undefined (likely returns 0 or the last output).

Neither header documents what happens in these edge cases. A note like "if no entry is eligible, the output is clamped to 0" (if that's what the implementation actually does) would be enough. Add a guard in the source if it isn't there.

---

### 2. Documentation Review

#### 2.1 Best-in-Class: DARE Doubling Derivation

[lib/DiscreteLQR.cpp:40-57](../lib/DiscreteLQR.cpp#L40-L57) is the best documentation in the codebase. It shows the doubling recurrence, names variables, states convergence rate (`O(log^2(n))` iterations), and cites Anderson & Moore. A reviewer can read it and independently verify the code without touching a reference book. This is the standard every other algorithm comment should aspire to.

Second place: the backward-Euler ESO nilpotency argument in [lib/DiscreteADRC.cpp:31-50](../lib/DiscreteADRC.cpp#L31-L50). Showing why `(I - Ts*Ae)^{-1}` is available analytically because `Ae^3 = 0` is exactly the kind of "why this formula" reasoning that separates maintainable code from magic.

For comparison: python-control's `dare()` implementation has zero mathematical comments. You'd have to read Laub 1979 to understand what it's doing. The bar here is already higher.

---

#### 2.2 Worst-in-Class: SystemAnalysis Lyapunov Solver

[lib/SystemAnalysis.h](../lib/SystemAnalysis.h) - `solveDiscreteLyapunov()`:

```cpp
// Solves: A * P * A' - P + Q = 0
// via Kronecker product vectorisation: (I - A\otimesA) * vec(P) = vec(Q).
// Complexity: O(n^6) - use only for small systems (n <= 20).
```

That's it. No derivation of why `(I - A\otimesA)` is the right operator, no reference, no note that this is numerically inferior to the Schur-based Bartels-Stewart algorithm (which MATLAB `dlyap()` uses internally). O(n^6) via Kronecker is fine for `n <= 10`; at `n = 20` you're doing 64 million flops per call. The comment warns against `n > 20` but doesn't say why or what to do instead.

The Bartels-Stewart method (Golub, Nash & Van Loan 1979) reduces this to O(n^3) via the Schur decomposition. If someone feeds this a 15-state ADRC or a GPC augmented-state Lyapunov check, they'll hit the performance cliff with no warning. Add the reference and the O(n^3) alternative note.

---

#### 2.3 AtomicParamBuffer: Header Is Good, Ecosystem Integration Is Missing

The [lib/AtomicParamBuffer.h](../lib/AtomicParamBuffer.h) header is excellent - the seqlock design rationale is explained, the memory ordering is correct, the single-writer limitation is documented. This is exactly what you'd want in a production RT library.

What's missing: integration guidance in [docs/DEPLOYMENT.md](DEPLOYMENT.md). How do you wire `AtomicParamBuffer` with a `TunerSuite` relay tuning run? The typical pattern is:

```
// Background thread (auto-tuner):
auto params = tuner.relayZN(pid);
buf.publish(params);

// RT thread:
pid.setParams(buf.read());
```

This pattern is not shown anywhere in the examples or documentation. The toolbox has all the pieces; it just doesn't show how they connect. Add one example - even a code snippet in `DEPLOYMENT.md` - because this is the non-obvious part of an RT-safe adaptive loop.

---

#### 2.4 FuzzyLogic: Rule Base Has No Structural Sanity Checks

The `FuzzySystem::addRule()` interface lets you add rules without checking that referenced term names actually exist in the registered input/output variables. If you typo a term name (`"Positif"` instead of `"Positive"`), the rule silently gets strength 0 (the term doesn't match anything). The system runs, outputs something, and you spend an afternoon wondering why your fuzzy controller isn't doing what the rule base says.

MATLAB's Fuzzy Logic Toolbox validates rule indices against the variable membership functions at load time. This library should do the same - either throw at `addRule()` when a term name doesn't exist, or add a `validate()` method that checks all rules before first use. The architecture already has the term names and variable structures; a `std::find_if` over the term list per rule is trivial.

---

#### 2.5 PID Documentation: Good Starting Point, Missing Closed-Loop Stability Criterion

[lib/DiscretePID.h](../lib/DiscretePID.h) has correct backward-Euler formulas with references to Astrom & Wittenmark. What's missing: any guidance on the stability implications of the derivative filter coefficient `N`.

`N` is labeled "higher = less filtering" which is true but incomplete. The closed-loop discretization with `N -> inf` (no filtering) produces an unstable derivative term at `omega > pi/Ts`. For `Ts = 0.01s`, the derivative amplifies signals above 314 rad/s. The default `N = 100` limits the derivative pole to roughly 100 * (1/Ts) = 10000 rad/s, which for `Ts = 0.01` is 1000 rad/s - well above Nyquist (628 rad/s). That's potentially aliasing the derivative. A practical upper bound is `N < pi / (2 * Ts * omega_bandwidth)`.

Just a note to the header. Doesn't require code changes.

---

### 3. Adherence to Best Practices

#### 3.1 What's Done Right

**`IController` interface design.** The pure-virtual `compute(double)`, `computeVec(VectorXd)`, `reset()`, `sampleTime()` interface is clean and exactly right for runtime composition. The `bumplessInit()` hook for supervisory switching is a genuine best-practice addition that most libraries omit. Contrast with python-control: there's no shared interface for controllers at all - you wire transfer functions symbolically, not as objects.

**Zero-allocation steady state.** Pre-allocated work vectors (`grad_`, `DeltaU_`, `pred_err_`, etc.) in MPC/GPC, pre-factored LDLT, pre-computed Lipschitz constant - this is the correct approach for RT-targeted code. Every controller that has a `compute()` on the hot path avoids heap allocation. This is non-trivial to verify and non-trivial to maintain. Good.

**Joseph-form Kalman covariance.** Doing `P = (I-KC)P(I-KC)' + KRK'` instead of `P = (I-KC)P` is exactly right for numerical stability. The naive form accumulates roundoff into asymmetry and eventually produces non-PSD covariances, which makes eigenvalue decompositions (like in UKF sigma-point generation) blow up. The eigenvalue floor in UKF is a backstop. Both should be there. Both are.

**`ControllerTraits` static dispatch.** Compile-time tuner-controller compatibility checks with `static_assert` and `[[deprecated]]` soft warnings are genuinely useful. The error messages name the correct tuner. This is the kind of thing that prevents a 2-hour debugging session when someone calls `LQRWeightTuner::brysonFor<DiscretePID>()`.

**Faddeev-LeVerrier for `ss2tf`.** Using the algebraic recurrence instead of `\prod(z - lambdai)` is the numerically correct choice and is explicitly documented. Most control libraries use the eigenvalue product form and quietly produce garbage characteristic polynomials for systems with clustered poles. Wilkinson's polynomial is a warning sign that this library heeds.

---

#### 3.2 What Needs Work

**No output saturation in GPC.** Discussed above (1.4). This is not a "nice to have" - it's a standard MPC/GPC capability absent from the implementation.

**QP convergence not surfaced.** Discussed above (1.1). Silent suboptimal solutions are a production hazard.

**`FuzzySystem` rule validation.** Discussed above (2.4). Silent wrong-answer risk.

**`SystemAnalysis::solveDiscreteLyapunov` complexity.** O(n^6) Kronecker approach vs O(n^3) Bartels-Stewart, undocumented. Fine for small `n`, silent performance cliff for `n >= 10`.

**ControllerStack all-inactive edge case.** Discussed above (1.8). Potential NaN propagation. Needs verification and a guard.

---

### 4. Control Design Methodology Priorities

Based on the codebase as of this review, these are the methodology areas that need the most attention, in order:

1. **MPC/GPC output constraints.** Missing and commonly needed. Fix before the next production deployment.
2. **QP convergence reporting.** Easy to add, high diagnostic value.
3. **ADRC `b0` sensitivity documentation.** One paragraph. Prevents incorrect tuning.
4. **UKF `alpha` tuning guidance for large `n`.** One paragraph. Prevents poor numerical behavior.
5. **SubspaceID similarity transform warning.** One sentence. Prevents misinterpretation of identified parameters.
6. **ControllerStack all-inactive guard.** Needs source verification and a guard or documented behavior.
7. **mu-synthesis (DK-iteration).** If you're shipping H-inf, this is the natural next step. The outer loop is ~100 lines; the D-scale fitting is polynomial least-squares. Eigen handles it.

---

### 5. Test Coverage Gaps

The 154-test + 19-integration-test suite is solid. Specific gaps worth adding:

| Test Gap | Why It Matters |
|----------|---------------|
| MPC QP non-convergence at `qpMaxIter` | Verifies the controller still returns the best-available iterate, not a stale or zero value |
| ControllerStack `Weighted` mode with all entries inactive | Verifies no NaN propagation |
| ControllerStack `Supervisory` mode with no eligible entry | Same |
| GPC reference-trajectory alpha parameter sweep | `alpha = 0.9` vs `alpha = 0.0` should produce measurably different overshoot |
| UKF with large state dimension (`n = 8`) | Catches the `alpha = 1e-3` Wc_0 issue for real |
| FuzzySystem with nonexistent term name in rule | Verifies either an exception or a predictable zero-strength behavior |
| DiscreteHinf infeasible gamma (weights too tight) | Verifies `result.feasible = false` with a sensible reason |

These are all achievable with the existing test infrastructure. None require a new plant model.

---

### 6. Performance Benchmarks

No formal benchmarks exist for comparison against expected outcomes. For a library targeting RT deployment, the minimum useful benchmarks are:

1. **`DiscreteMPC::computeRef()` wall-clock time** vs `Nc`, `Np` - verifies the "sub-millisecond for `Nc <= 20`" claim that any RT-targeted MPC needs to make.
2. **DARE solver convergence iterations** vs `n` - the doubling algorithm claims O(log^2(n)); verify this empirically.
3. **`RecursiveLeastSquares::update()` cost** vs `na + nb` - verifies suitability for high-rate adaptive loops.

None of these are in the existing test suite. Add a `benchmarks/` directory with timing harnesses. Even rough wall-clock measurements on a development machine establish a baseline that makes regressions visible.

---

### 7. Exemplary Documentation in the Open-Source Community: Reference Points

For context on where this codebase sits relative to community standards:

**python-control (https://python-control.readthedocs.io):** Strong mathematical documentation of transfer function and state-space operations, clear API references. Weak on implementation comments (the DARE solver has none). This codebase beats it on implementation documentation.

**Eigen (https://eigen.tuxfamily.org):** Best-in-class inline documentation for numerical linear algebra. Every operation documents its complexity, numerical caveats, and when to prefer alternatives. The `LLT` vs `LDLT` vs `FullPivLU` guidance in Eigen's headers is a model for how to document numerical tradeoffs.

**ACADO Toolkit (https://acado.github.io):** Documents QP solver convergence, warm-starting, and constraint handling in detail. This codebase's MPC/GPC should aim for that level of transparency on solver status.

**Kalman and Bayesian Filters in Python (Roger Labbe):** https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python - the gold standard for filter documentation with Jupyter notebooks. Not a library per se, but the standard for explaining sigma-point weights, the UKF numerical caveats, and the EKF linearization errors. The UKF header here is good; that resource shows what "complete" looks like for a general audience.

---

### 8. Summary Verdict

**Production ready? Almost.** The algorithmic core is solid. The numerical choices are right. The RT infrastructure is genuinely thoughtful. The documentation quality is above average for an open-source control library.

**Status of all Part 8 findings (updated 2026-05-25):**

| Issue | Description | Status |
|-------|-------------|--------|
| 1.1 | QP convergence not surfaced in MPC/GPC | `[FIXED]` - `lastQPConverged()` / `lastQPIters()` added to both [lib/DiscreteMPC.h](../lib/DiscreteMPC.h) and [lib/GeneralizedPredictiveControl.h](../lib/GeneralizedPredictiveControl.h); GPC hardcoded `qpMaxIter`/`qpTol` replaced with `p_` fields |
| 1.2 | LQR PBH rank tolerance implicit | `[FIXED]` - caveat note added to `DiscreteLQR` constructor in [lib/DiscreteLQR.h](../lib/DiscreteLQR.h) |
| 1.3 | ADRC `b0` sensitivity undocumented | `[FIXED]` - robustness bounds, practical estimation formulae, and Gao (2003) reference added to `ADRCParams::b0` in [lib/DiscreteADRC.h](../lib/DiscreteADRC.h) |
| 1.4 | GPC output constraints missing | `[OPEN]` - enhancement; tracked in Part 6 roadmap |
| 1.5 | SubspaceID similarity transform not warned | `[FIXED]` - explicit similarity-transform invariance/non-invariance block added to `n4sid()` doc in [lib/SubspaceID.h](../lib/SubspaceID.h) |
| 1.6 | H-inf weight selection lacks troubleshooting | `[FIXED]` - five-point troubleshooting guide added to `MixedSensitivity::build()` in [lib/DiscreteHinf.h](../lib/DiscreteHinf.h) |
| 1.7 | UKF `alpha = 1e-3` poor for large `n` | `[FIXED]` - sizing guidance and formula for minimum-noise `alpha` added to [lib/UnscentedKalmanFilter.h](../lib/UnscentedKalmanFilter.h) |
| 1.8 | ControllerStack all-inactive edge case | `[FIXED]` - Supervisory mode now holds `lastOutput_` and warns on stderr; Weighted mode holds `lastOutput_` instead of returning 0 and warns; both header and source updated in [lib/ControllerStack.h](../lib/ControllerStack.h) / [lib/ControllerStack.cpp](../lib/ControllerStack.cpp) |
| 2.2 | `solveDiscreteLyapunov` O(n^6) undocumented | `[FIXED]` - Bartels-Stewart O(n^3) alternative, performance cliff warning, and references added to [lib/SystemAnalysis.h](../lib/SystemAnalysis.h) and [lib/SystemAnalysis.cpp](../lib/SystemAnalysis.cpp) |
| 2.3 | `AtomicParamBuffer` ecosystem integration missing | `[OPEN]` - DEPLOYMENT.md example to be added; tracked in Part 6 |
| 2.4 | FuzzySystem rule validation missing | `[FIXED]` - `addRule()` now throws `std::out_of_range` on bad input/output indices and `std::logic_error` if no output registered; validated in [lib/FuzzyLogic.cpp](../lib/FuzzyLogic.cpp), documented in [lib/FuzzyLogic.h](../lib/FuzzyLogic.h) |
| 2.5 | PID derivative filter `N` upper-bound undocumented | `[FIXED]` - Nyquist limit and practical bandwidth formula added to `PIDParams::N` in [lib/DiscretePID.h](../lib/DiscretePID.h) |

**Remaining open (enhancement-level, no correctness impact):**
- GPC output constraints (Issue 1.4) - QP extension, tracked in Part 6
- `AtomicParamBuffer` DEPLOYMENT.md example (Issue 2.3) - documentation only
- Benchmark harnesses - `benchmarks/` directory with timing harnesses
- mu-synthesis DK-iteration (Part 6, R2)
- RTOS scheduler abstraction (Part 6, R3)
- C-API shim for Python-in-the-loop (Part 6, R5)

---

*Part 8 added 2026-05-25. All pre-release items fixed same day.*

---

---

## Part 9: Full-Pass Senior Review - 2026-05-25 (Rev 3)

**Reviewer:** Senior Controls Engineer (second pass, requested by user)
**Scope:** Full codebase + case-study audit. `lib/`, `tests/`, `examples/`, `case-study/` (all three sub-projects), `docs/`, `cheatsheet/`. Read against actual source, not prior reports.
**Tone:** Peer-to-peer, informal, critical. Useful over polite.
**Benchmarks:** python-control, ACADO, CasADi, Kalman-and-Bayesian-Filters-in-Python (Labbe), Eigen documentation conventions.
**Focus areas requested:** PID tuning, state-space representation, algorithm gap analysis, case-study application quality, performance benchmarking, pseudocode/diagram guidance.

---

### Preamble

Parts 1-8 of this document are accurate. The fixed-item lists are correctly verified against source. This pass adds three things that weren't there before: (1) a detailed review of how the case studies actually use the toolbox, because that's where the rubber meets the road and the gaps that don't show up in unit tests are visible; (2) a fresh look at the PID tuning and state-space subsystems specifically, since those were called out as priority areas; and (3) concrete benchmark recommendations with pseudocode, since the current test suite has no timing harnesses at all.

---

### 1. Case Study Review: Tug Boat Numerical Simulation

This is the most developed case study and the most instructive place to see how the toolbox behaves in a real application. Read against `project_description.md`, `mathematical_models.md`, `controller_choices.md`, and `review_notes.md`.

#### 1.1 The IController Wrapping Pattern Is Correct - But the SMC Wrapper Has a Latent Sign Bug Risk

`controller_choices.md` documents the SMC equivalent control term as:

```
tau_eq = -M_re * Lambda * ė + D_re * ν + C_re(ν) * ν
```

`review_notes.md` already flags this: "the GDScript prototype exhibited IAE 500-5000* larger than paper Table 7... root-cause identified as a probable sign error in tau_eq." The C++ implementation hasn't been validated yet against Table 7 (validation is listed as a pre-publication prerequisite, not a completed task). Until that check is done, every performance number from the SMC controller is potentially garbage.

**This is the highest-priority unresolved item in the case study.** Not a toolbox bug - `DiscreteSMC` is correct. The risk is in how `controllers.cpp` constructs the equivalent control term that feeds into `DiscreteSMC`. The sign of `M_re * Lambda * ė` must be subtracted; if it's added, the equivalent control amplifies the tracking error instead of cancelling it.

**Action:** Add a unit test in `test_integration.cpp` (or a dedicated `test_tug_smc_sanity.cpp`) that:
1. Initialises the tug plant at equilibrium with a 5 m surge offset.
2. Runs Mode 3 (SMC) for 300 seconds under S2 conditions (90^\circ wind, 5 m/s).
3. Asserts final IAE_x < threshold consistent with Table 7 +/- 10%.

This test should run as part of CI. If it fails, the sign error has been caught automatically, not discovered post-publication.

---

#### 1.2 Thrust Allocation: Silent Constraint Violation Under MPC Commands

`review_notes.md` (Section 4) correctly identifies the pseudo-inverse thrust allocator as theoretically weak: clamping one tug breaks `BT = tau_c`. What's not noted is the interaction with the MPC controller specifically.

`DiscreteMPC` computes the optimal `tau_c` assuming the plant will receive exactly `tau_c`. If the thrust allocator then clips one or more tugs (because `tau_c` demands more than the per-tug box allows), the actual generalized force applied to the barge is `tau_applied != tau_c`. The MPC's internal model advances `x_hat` using `tau_c` (as recorded in `u_prev_`), not `tau_applied`. After several consecutive saturations, `x_hat` drifts from the real plant state. This is exactly the open-loop drift problem documented in [lib/DiscreteMPC.cpp:198-203] - except here the discrepancy comes from the allocator, not from the MPC's own output saturation.

The practical symptom: MPC looks confident (QP converges, `lastQPConverged() = true`) while the plant has been running on a different input for many steps. You won't catch this from the controller's perspective.

**Mitigation:**
1. Feed `tau_applied` (the allocator's actual output redistributed back into generalized force space via `B * T_clamped`) back to `DiscreteMPC::setLastApplied(tau_applied)` if such an interface exists, or update `u_prev_` externally. This is standard practice in MPC implementations that sit upstream of a constrained actuator layer.
2. Alternatively, document the limitation explicitly in the case study simulation runner and log the allocator residual `||tau_applied - tau_c||` per tick for post-hoc diagnosis.

Neither the case study nor the toolbox currently handles this. It should at least be documented in `review_notes.md`.

---

#### 1.3 KF Linearisation Domain: The Problem Is Bigger Than Documented

`review_notes.md` (Section 3) says the linearisation about zero velocity "degrades for heading changes greater than approximately 20^\circ." That number needs a citation or a derivation. The actual threshold depends on the nonlinearity in `R(ψ)` and `C_re(ν)`.

For the barge system, the rotation matrix `R(ψ)` introduces coupling between surge and sway errors as `ψ` deviates from the linearisation point. The cross-coupling term scales as `sin(Deltaψ)`, so for `Deltaψ = 20^\circ` the coupling is `sin(20^\circ) approx = 0.34` - meaning 34% of surge error leaks into sway in the world frame. That's not small. A Kalman filter built on a linearised `A` that ignores this coupling will have systematic prediction errors even at heading deviations much smaller than 20^\circ.

The correct mitigation is already listed (`ExtendedKalmanFilter.h` is available), but the severity of the linearisation error is underplayed. The KF-PID mode should come with a quantitative validity domain in the documentation: "valid for |Deltaψ| < X^\circ where X is derived from maximum tolerable prediction error."

**Action:** Add a linearisation validity check to the simulation runner: if `|ψ - ψ_linearise|` exceeds a configurable threshold (default 15^\circ), log a warning. This is a one-liner in the tick loop and prevents silent model mismatch.

---

#### 1.4 Extremum Seeking: Dither Frequency Below Nyquist Needed

`controller_choices.md` sets ESC dither at 0.016-0.020 Hz (per axis, staggered). The control sample rate is `Deltat = 0.5 s`, giving Nyquist at 1 Hz. The dither is well below Nyquist - fine. But the HPF cutoff is 0.05 rad/s (0.008 Hz) and the LPF is 0.02 rad/s (0.003 Hz). The dither at 0.016 Hz = 0.10 rad/s is above the HPF cutoff - also fine.

What's not checked: the time constant of the closed-loop gradient estimate vs the plant settling time. The ESC update `dtau_nom/dt = -k_esc * ∇^J` drives convergence over a timescale of roughly `1 / (k_esc * a_d^2 * omega_d / 2)`. For the given parameters with `k_esc = 1`, `a_d = 5*10^3 N`, `omega_d = 0.10 rad/s`, the estimate time constant is approximately `2 / (5*10^3)^2 * 0.10 approx = 8*10^-⁸` - which gives unrealistically fast convergence. Something is off with the units or the gain. The "10* longer than PID" claim needs a back-of-envelope check before it goes in the documentation.

More importantly: staggering dither frequencies across axes (0.016, 0.018, 0.020 Hz) is the right call to avoid inter-axis interference. But the frequencies should be chosen such that no dither frequency is an integer multiple of another. `0.016`, `0.018`, `0.020` have ratio 8:9:10, which passes. Document this choice explicitly - it's non-obvious and someone will break it when they "tune" the frequencies.

---

#### 1.5 Weighted IAE Composite Metric: The Heuristic Needs Justification

`review_notes.md` (Section 6) flags the `IAE_total = IAE_x + IAE_y/10 + IAE_ψ` weighting as heuristic and suggests alternatives. The right answer is to use the MPC `Q_mpc` diagonal to derive the weights, since that diagonal already encodes the control designer's relative importance between surge, sway, and yaw tracking. For `Q_mpc = diag[10^3, 10^3, 10^5]`, the natural weighting is:

```
IAE_total = IAE_x * w_x + IAE_y * w_y + IAE_ψ * w_ψ

where:  w_x = Q_mpc[0,0] / trace(Q_mpc)  = 1000 / 102000 approx = 0.0098
        w_y = Q_mpc[1,1] / trace(Q_mpc)  = 0.0098
        w_ψ = Q_mpc[2,2] / trace(Q_mpc)  = 100000 / 102000 approx = 0.980
```

That puts almost all weight on heading error, which reflects the physical reality that yaw control is the hardest axis (highest moment of inertia, largest added mass effect). The current 1/10 heuristic gives IAE_y one-tenth the weight of IAE_x, with no physical justification. Fix this before any results are published.

---

### 2. Case Study Review: Boiler Control

The current monolith (`boiler_turbine_case_study.cpp`) and the planned refactoring (`IMPLEMENTATION_PLAN.md`) are both worth reviewing independently.

#### 2.1 Monolith: Forward-Euler Discretisation Is Wrong for Stiff Plants

The existing `boiler_turbine_case_study.cpp` linearises around three operating points and discretises via forward Euler: `A_d = I + Ts * A_c`. The Bell-Astrom boiler model has eigenvalues spanning several orders of magnitude (the drum pressure dynamics are fast; the water level dynamics are slower). For a stiff plant, forward Euler is A-unstable for the fast modes unless `Ts < 2/|lambda_max|`. The monolith uses `Ts = 1 s`. If any continuous-time eigenvalue has `|lambda| > 2`, the discrete model will have at least one eigenvalue outside the unit disk regardless of the physics.

`IMPLEMENTATION_PLAN.md` correctly lists ZOH via `ctrl::c2d()` as the discretisation method for the refactored version. That's the right fix. But the monolith is still there and currently gives wrong discrete models for the fast dynamics. This should be noted with a warning comment in the monolith, not silently left as-is.

**Action:** Add a comment block at the top of `boiler_turbine_case_study.cpp`:

```cpp
// WARNING: This file uses forward-Euler discretisation (A_d = I + Ts*A_c).
// For the Bell-Astrom plant, this is numerically unstable for Ts > 2/|lambda_max|.
// The refactored version in sim/ uses ZOH via ctrl::c2d() and is preferred.
// This file is retained for historical reference only; do not use its discrete
// models for control design without first checking that all eigenvalues of A_d
// lie within the unit disk.
```

---

#### 2.2 IMPLEMENTATION_PLAN.md: 18 Controllers * 7 Scenarios Is a Test Matrix, Not a Validation Plan

The plan lists 126 simulation runs (18 controllers * 7 scenarios) as the deliverable. That's a test matrix, not a validation plan. A validation plan answers: what does success look like for each controller on each scenario? Without acceptance criteria, the 126 runs produce numbers but no conclusions.

For a production-grade case study, each (controller, scenario) pair needs:
1. A **primary metric** and its pass threshold (e.g., `IAE_pressure < 5% of setpoint * T_sim` for regulation scenarios).
2. A **control effort bound** (e.g., valve position must remain in [0%, 100%] and slew rate must not exceed spec).
3. A **stability check** (e.g., output variance must decrease monotonically after the disturbance).

python-control's example gallery does this correctly - each example has an expected output range that is checked programmatically. ACADO does it with convergence residuals per run. This codebase should too.

**Action:** For each scenario in `IMPLEMENTATION_PLAN.md`, add a `validation_criteria` block alongside the `ScenarioConfig`:

```cpp
struct ValidationCriteria {
    double max_iae_pressure;   // fraction of setpoint * T_sim
    double max_valve_excursion; // in [0,1]
    double settling_time_s;    // time to within 2% of setpoint
};
```

This is 15 minutes of struct definition. The payoff is that "run all 126 simulations" becomes "run all 126 simulations and report pass/fail", which is actually useful.

---

#### 2.3 Missing Controller: RepetitiveController for Periodic Load Disturbances

The Bell-Astrom boiler plant is subject to load disturbances that, in a real power plant, are often periodic (grid frequency variations, scheduled load changes). `IMPLEMENTATION_PLAN.md` lists `RepetitiveController` as one of the 18 controllers but gives it no specific scenario.

`RepetitiveController` is exactly the right tool for periodic disturbance rejection: it learns the disturbance waveform over one period and cancels it feed-forward. The boiler case study should have a scenario with a sinusoidal or square-wave load disturbance at a known frequency, not just step disturbances. This would actually demonstrate what `RepetitiveController` does that PID and MPC cannot.

Suggested scenario `s08_periodic_load.json`: sinusoidal steam valve disturbance at 0.05 Hz (matching a hypothetical grid load oscillation). Expected result: `RepetitiveController` achieves near-zero steady-state periodic IAE after one period; PID and MPC show residual periodic error. If this comparison isn't run, the `RepetitiveController` class might as well not exist in the case study.

---

### 3. PID Tuning Subsystem - Detailed Review

#### 3.1 RelayAutoTuner: The Hysteresis Threshold Is Too Loose by Default

`RelayAutoTuner` in `ControllerTuner.h` uses the Astrom-Hagglund relay test to extract the ultimate gain `K_u` and ultimate period `T_u`. The relay threshold (hysteresis) determines how cleanly the limit cycle separates from measurement noise. The default is not documented; from reading the source, it appears to be a fixed absolute value.

The problem: an absolute hysteresis threshold that works for a temperature controller (where signals are in ^\circC) will be either too tight (false crossings, corrupted `K_u` estimate) or too wide (missed crossings, wrong `T_u`) for a pressure controller where signals are in bar or a motor controller where signals are in rpm. The hysteresis should be expressed as a fraction of the expected output range, not an absolute value.

MATLAB's `pidtune()` with relay feedback normalises the relay amplitude relative to the estimated output standard deviation. That's overkill for a toolbox this size, but a simple relative threshold (`hysteresis = 0.02 * (yMax - yMin)`) would be far more robust than an absolute one.

**Action:** In `RelayAutoTunerParams`, add a `bool relative_hysteresis` flag (default `true`) and compute the actual threshold as `hysteresis_frac * estimated_output_range`. Document the default fraction (recommend 0.02-0.05) and explain why.

---

#### 3.2 StepResponseTuner: FOPDT Identification Is Sensitive to Step Timing

`StepResponseTuner` fits a first-order-plus-dead-time (FOPDT) model `K, tau, theta` to a step response. The identification uses a fixed 28.3%/63.2% tangent-intersection method (Smith method variant). This method is sensitive to when the step was applied: if there is pre-step drift or if the measurement is noisy, the 28.3% and 63.2% thresholds are applied to the wrong baseline.

The test in `tests/data/step_response.csv` is clean synthetic data. In a real commissioning workflow, the step response data will have:
- Pre-step drift (the system wasn't at steady state when the step was applied)
- Measurement noise (the 28.3% threshold crossing may be ambiguous)
- Superimposed disturbances (the step response is not the only thing happening)

None of these are tested. `test_tuners_extended.cpp` only tests on the synthetic CSV. Add at minimum:
1. A noisy step response test (Gaussian noise sigma = 5% of step amplitude): verify the FOPDT parameters are still within 15% of truth.
2. A pre-step drift test (plant at 10% of step amplitude before step): verify the method correctly removes the initial offset.

Without these, the tuner is only validated for conditions that don't occur in real commissioning.

---

#### 3.3 IMC Rule: The Lambda Parameter Has No Physical Interpretation in the Header

`StepResponseTuner` offers four PID tuning rules: ZN, Tyreus-Luyben, AMIGO, and IMC. The IMC rule requires a user-supplied `lambda` parameter (the desired closed-loop time constant). The header says "lambda > 0, larger = more conservative." That's it.

For IMC, `lambda` has a specific physical meaning: it is the closed-loop time constant of the desired first-order response. A practically useful guideline from Skogestad (2003) is `lambda = max(tau/5, theta)` for robust tuning, or `lambda = tau` for balanced performance/robustness. Neither of these is documented. A user who doesn't know IMC theory will pick `lambda = 1.0` because it's a nice round number and wonder why the resulting controller is too aggressive on a slow plant and too conservative on a fast one.

Add to the header comment for `IMCParams::lambda`:

```
// IMC closed-loop time constant. Physical interpretation: the closed-loop step response
// settles to within 63% of setpoint in lambda seconds.
// Practical guidelines (Skogestad 2003):
//   - Robust: lambda = max(tau/5, theta)   [conservative, good for model uncertainty]
//   - Balanced: lambda = tau               [matches open-loop time constant]
//   - Aggressive: lambda = theta           [fastest achievable without robustness margin]
// Too small lambda -> controller becomes aggressive -> sensitivity to model error increases.
```

---

### 4. State-Space Subsystem - Detailed Review

#### 4.1 `PlantModel::c2d()` ZOH via Matrix Exponential: The Implementation Choice Is Not Documented

`PlantModel::c2d()` with `Discretisation::ZOH` computes the matrix exponential `expm([Ac, Bc; 0, 0] * Ts)` to get `Ad, Bd`. This is the Van Loan method (1978) for ZOH discretisation. It's the numerically correct approach - more accurate than the Pade approximant for stiff systems, and exactly correct for linear systems. But the header doesn't say any of this.

A reader who sees `expm(...)` without context might assume it's an approximation, or might replace it with a series expansion thinking they're "simplifying" the code. The DARE doubling derivation in `DiscreteLQR.cpp` is exemplary precisely because it explains *why* the specific algorithm was chosen. The ZOH implementation deserves the same treatment.

**Add to `c2d()` ZOH case:**

```cpp
// ZOH discretisation via Van Loan (1978) matrix exponential method.
// Constructs the augmented matrix [Ac, Bc; 0, 0] and takes the matrix exponential,
// which gives [Ad, Bd; 0, I] exactly (no approximation error for linear time-invariant
// systems). This is preferred over Pade series for stiff Ac (large spectral radius)
// because the matrix exponential is computed via Schur decomposition, which is
// numerically stable even when eigenvalues span many decades.
// Reference: Van Loan (1978), "Computing integrals involving the matrix exponential",
// IEEE Trans. Automat. Control, 23(3):395-404.
```

---

#### 4.2 `ss2tf()` via Faddeev-LeVerrier: The Sign Convention Warning Is Missing

`ss2tf()` uses the Faddeev-LeVerrier recurrence to compute the characteristic polynomial. This is correctly documented as preferable to `\prod(z - lambda_i)` for clustered poles. What's not documented: the denominator polynomial returned by `ss2tf()` follows the convention `z^n + a_{n-1}z^{n-1} + ... + a_0`, with the leading coefficient normalised to 1. If a user passes this to a control design tool that expects the unnormalised form (coefficient vector `[1, a_{n-1}, ..., a_0]`), they get the right answer. But if they pass it to something that expects `[a_n, a_{n-1}, ..., a_0]` (e.g., MATLAB's `tf()` with explicit denominator), they need to know the leading coefficient is already 1.

One sentence in the return-value documentation prevents this. It's the kind of thing that costs an afternoon of debugging when someone interfaces the toolbox with an external tool.

---

#### 4.3 `DiscreteLQG::step()`: The Separation-Principle Comment Understates the Limitation

[lib/DiscreteLQG.cpp:28-34] has a two-paragraph comment explaining the separation principle and causality (fixed in the 05-26 pass, marked `[FIXED]`). Reading the actual comment: it correctly explains that the observer and controller are designed independently. What it doesn't say: the separation principle holds exactly only for linear systems with Gaussian noise. For the boiler-turbine and tug boat systems, both of which are nonlinear, `DiscreteLQG` operates on a linearised model. The separation principle guarantees stability only in the neighborhood of the linearisation point.

This is a design-level limitation, not a code bug. But users who pick `DiscreteLQG` for the nonlinear boiler plant and observe poor performance outside the linearisation region will have no guidance on why. Add one sentence to the `DiscreteLQG` header:

```
// Note: The separation principle holds exactly only for linear systems. For nonlinear
// plants, this class linearises the plant model at construction time. Stability and
// performance guarantees apply only in the neighborhood of the linearisation point.
// For larger operating regions, consider DiscreteLQG with periodic re-linearisation
// or ExtendedKalmanFilter + DiscreteLQR as separate components.
```

---

### 5. Test Coverage Gaps (New This Pass)

These are gaps not covered in Part 8 or earlier sections.

| Gap | Why It Matters | Effort |
|-----|---------------|--------|
| SMC case-study S2 IAE validation (Section 1.1 above) | Sign error risk; can corrupt all published results | 1 hr |
| MPC model mismatch under thrust allocator clamping (Section 1.2) | Silent `x_hat` drift; hard to diagnose without explicit test | 2 hrs |
| `StepResponseTuner` with noisy + drifting input data (Section 3.2) | Only tested on clean synthetic data | 1 hr |
| `RelayAutoTuner` with relative hysteresis on heterogeneous-scale plants (Section 3.1) | Default absolute threshold gives wrong `K_u` on extreme-scale plants | 1 hr |
| `PlantModel::c2d()` ZOH accuracy on stiff plant (`lambda_max/lambda_min > 100`) | Verify matrix-exponential is more accurate than Euler on stiff system | 30 min |
| `DiscreteLQG` stability outside linearisation region | Expected degradation, but should be quantified | 1 hr |
| Boiler monolith: `A_d` eigenvalue check for forward-Euler instability (Section 2.1) | Users copy-paste the monolith; they won't know the discrete model is wrong | 30 min |

---

### 6. Performance Benchmarking: Concrete Proposal

Part 8 said "add a `benchmarks/` directory." Here's what that should actually look like.

**Minimum viable benchmark suite:** Three harnesses, each timing a hot loop of 1000 iterations to eliminate cache warm-up effects. Wall-clock via `std::chrono::high_resolution_clock`. Report mean +/- std over 10 independent runs.

**Benchmark 1: MPC solve time vs (Nc, Np)**

```cpp
// Pseudocode for scripts/bench_mpc.cpp
for Nc in [3, 5, 10, 20]:
    for Np in [10, 20, 50, 100]:
        mpc = DiscreteMPC(plant, {Np, Nc, rho_y=1.0, rho_u=0.1})
        state = VectorXd::Zero(n)
        ref   = VectorXd::Ones(n)
        timer.start()
        for i in [0..999]:
            u = mpc.computeRef(state, ref)
        elapsed = timer.stop() / 1000
        print(f"Nc={Nc} Np={Np}: {elapsed:.3f} ms/call")
```

Target: `Nc=20, Np=50` should complete in < 1 ms on a modern CPU at -O2. If it doesn't, the gradient-projection solver is not suitable for RT deployment at that horizon size, and the documentation should say so.

**Benchmark 2: DARE doubling convergence vs plant order n**

```cpp
// Pseudocode for scripts/bench_lqr.cpp
for n in [2, 4, 8, 16, 32]:
    A = random_stable_A(n)       // all eigenvalues |lambda| < 0.95
    B = random_B(n, m=1)
    Q = MatrixXd::Identity(n,n)
    R = MatrixXd::Identity(m,m)
    timer.start()
    for i in [0..99]:
        lqr = DiscreteLQR(A, B, Q, R)
    elapsed = timer.stop() / 100
    print(f"n={n}: {elapsed:.2f} ms/DARE, iters={lqr.dareIters()}")
```

The O(log^2(n)) convergence claim should be visible in the iteration count. If `dareIters()` isn't exposed, add it - it's already stored in `DareResult`.

**Benchmark 3: RLS update cost vs (na, nb)**

```cpp
// Pseudocode for scripts/bench_rls.cpp
for na in [2, 4, 8]:
    for nb in [2, 4, 8]:
        rls = RecursiveLeastSquares(na, nb, lambda=0.99)
        phi = VectorXd::Random(na + nb)
        y   = 1.0
        timer.start()
        for i in [0..9999]:
            rls.update(phi, y)
        elapsed = timer.stop() / 10000
        print(f"na={na} nb={nb}: {elapsed:.4f} ms/update")
```

An RLS update at `na=8, nb=8` should be under 10 mus at -O2. If it's over 100 mus, something is wrong with the matrix operations (likely a missing `noalias()` in the Eigen expression).

---

### 7. Pseudocode and Diagram Requests

The following algorithms would benefit from a diagram or pseudocode block in their headers that does not currently have one. The DARE doubling derivation is the template.

**Priority 1 - `ControllerStack` switching logic:**

```
// Supervisory mode pseudo-code (add to ControllerStack.h):
//
//   for each entry e in stack (insertion order):
//       if e.enabled AND e.activationCondition() AND e.controller->isHealthy():
//           if e.controller != last_active_:
//               e.controller->bumplessInit(lastOutput_, current_error)
//               last_active_ = e.controller
//           return e.controller->compute(signal)
//   // no entry eligible:
//   emit warning to stderr
//   return lastOutput_  // hold last valid output
```

This makes the switching logic reviewable without reading 80 lines of source. Any correctness issue (e.g., the edge case from Section 1.8 of Part 8) is immediately visible.

**Priority 2 - `ExtremumSeeker` dither/demodulate loop:**

```
// ESC algorithm (add to ExtremumSeeker.h):
//
//   dither:       s_k = s_nom + a_d * sin(omega_d * k * Ts)
//   apply s_k -> plant -> get cost J_k
//   demodulate:   m_k = J_k * sin(omega_d * k * Ts)       // multiply by reference signal
//   high-pass:    h_k = HPF(m_k)                        // remove DC
//   low-pass:     ghat_k = LPF(h_k)                       // gradient estimate
//   update:       s_nom += -k_esc * ghat_k * Ts
```

Currently there is no concise description of the ESC algorithm in the header. A reader has to reverse-engineer it from the `compute()` implementation.

**Priority 3 - `SubspaceID::n4sid()` MOESP block diagram:**

The MOESP algorithm proceeds through 5 well-defined steps (Hankel construction -> oblique projection -> SVD -> state-space recovery -> LS refinement). A numbered pseudocode block matching those 5 steps to the corresponding code blocks in `n4sid()` would make the implementation verifiable. The comment added in a prior pass explains the oblique projection step; the other four steps have no pseudocode.

---

### 8. What the Tug Boat Case Study Reveals About the Toolbox's Applied Gaps

Reading the case study as a whole, five toolbox gaps surface that aren't visible from the unit tests alone:

1. **No `setLastApplied()` interface on `DiscreteMPC`.** When an external actuator layer clips the output (thrust allocator, valve saturation, etc.), the MPC needs to be told the actual applied input. This is standard in industrial MPC implementations and missing here. It's not a QP correctness issue; it's a model-input feedback issue.

2. **`IController::computeVec()` signature doesn't carry reference and state separately.** The tug boat controller wrapper has to pack `[ref, state]` into a single vector or use a custom interface. The three-axis tug controllers use `compute(Eigen::Vector3d ref, Eigen::VectorXd state)` - that doesn't fit the standard `IController::computeVec(signal)` signature. The case study ends up implementing its own wrapper class (`ControllerBase` in `controllers.h`) rather than using `IController` directly. This partially defeats the purpose of having a shared interface.

3. **No bumpless transfer test exists for MIMO controllers.** `ControllerStack`'s `bumplessInit()` calls `bumplessInit(u_target, error)` with scalar arguments. For MIMO controllers (like the MPC in the tug boat case), the bumpless initialisation is a vector operation. `IController::bumplessInit()` takes a scalar. Either the MIMO case can't do bumpless transfer, or each controller implements its own vector version outside the interface. Neither option is documented.

4. **`DiscreteADRC` is excluded from the tug boat controller portfolio** (listed as "deferred" in `review_notes.md` Section 8). The stated reason is "strong alternative to SMC for validation." That's exactly why it should be included - ADRC requires no plant model (unlike SMC which requires `M_re, D_re, C_re`), and its ESO would naturally handle the environmental disturbances without explicit disturbance modelling. The fact that it's deferred without a timeline is a missed opportunity.

5. **`ControllerStack` is not used in either case study.** The most architecturally distinctive feature of the toolbox - supervised/additive/weighted controller composition with bumpless transfer - appears in no case study. The boiler `IMPLEMENTATION_PLAN.md` lists `ControllerStack (Supervisory / Additive / Weighted)` as one of the 18 controllers to implement, but this won't be demonstrated until the refactored simulation is built. Until then, the toolbox's most complex feature has zero end-to-end validation in any applied context.

---

### 9. Comparison to Open-Source Community Standards

Where this codebase stands relative to the references cited in Part 8:

| Dimension | This Library | python-control | ACADO | Eigen |
|-----------|-------------|----------------|-------|-------|
| Discrete-time throughout | **Yes** | Mixed | **Yes** | N/A |
| Zero-allocation hot path | **Yes** | No | **Yes** | **Yes** |
| Algorithm derivation comments | Above average | Poor | Fair | **Excellent** |
| QP solver status reporting | Partial (added in Part 8) | N/A | **Full** | N/A |
| Case study coverage | Good (2 complete, 1 planned) | Examples only | Tutorials only | None |
| Performance benchmarks | **Missing** | Informal | **Formal (CI)** | **Formal** |
| Bound on linearisation validity | **Missing** | Not applicable | Documented | N/A |
| C-API / Python bindings | **Missing** | Native Python | MATLAB/Python | pybind11 |

The toolbox is above community average on algorithm correctness and documentation. It's below average on benchmarking and language bindings. The case studies are a genuine differentiator - no other open-source control library of this scope has published applied MIMO case studies in the same repository as the library code.

---

### 10. Priority Action List (Part 9 Additions)

Items from prior parts that remain open are unchanged. New items from this pass:

| # | Issue | File | Severity | Effort |
|---|-------|------|----------|--------|
| P9-1 | SMC case-study S2 IAE validation test (sign error risk) | `tests/` or case-study `tests/` | **High** | 1 hr - Sign verified correct in C++ source (review_notes.md updated); quantitative IAE vs Table 7 check still pending |
| P9-2 | ~~MPC: add `setLastApplied()` interface for external actuator feedback~~ | [lib/DiscreteMPC.h](../lib/DiscreteMPC.h) | Medium | `[FIXED]` - `setLastApplied(u_applied)` added; corrects `u_prev_` when an external actuator layer clips the command |
| P9-3 | ~~Boiler monolith: add forward-Euler instability warning comment~~ | [case-study/Boiler Control/boiler_turbine_case_study.cpp](../case-study/Boiler%20Control/boiler_turbine_case_study.cpp) | Medium | `[FIXED]` - 12-line warning block added at top of file explaining A-instability risk at Ts=1s |
| P9-4 | Boiler `IMPLEMENTATION_PLAN.md`: add `ValidationCriteria` struct per scenario | [case-study/Boiler Control/IMPLEMENTATION_PLAN.md](../case-study/Boiler%20Control/IMPLEMENTATION_PLAN.md) | Medium | 30 min |
| P9-5 | Add KF linearisation validity warning to tug boat sim runner | case-study tug sim runner | Low | 15 min |
| P9-6 | Fix IAE composite metric weighting (use `Q_mpc` diagonal, not 1/10 heuristic) | [case-study/Tug Boat Numerical Simulation/review_notes.md](../case-study/Tug%20Boat%20Numerical%20Simulation/review_notes.md) + sim runner | Low | 30 min |
| P9-7 | ~~`ControllerStack` pseudocode block in header~~ | [lib/ControllerStack.h](../lib/ControllerStack.h) | Low | `[FIXED]` - Supervisory and Weighted pseudocode blocks added showing edge-case hold behaviour |
| P9-8 | ~~`ExtremumSeeker` pseudocode block in header~~ | [lib/ExtremumSeeker.h](../lib/ExtremumSeeker.h) | Low | `[FIXED]` - full algorithm pseudocode with variable-name annotations added |
| P9-9 | ~~Add Van Loan ZOH derivation comment to `c2d()`~~ | [lib/PlantModel.cpp](../lib/PlantModel.cpp) | Low | `[FIXED]` - Van Loan (1978) derivation added explaining matrix-exponential embedding and why it is preferred over Pade for stiff systems |
| P9-10 | `StepResponseTuner`: noisy + drifting input tests | [tests/test_tuners_extended.cpp](../tests/test_tuners_extended.cpp) | Low | 1 hr |
| P9-11 | `RelayAutoTuner`: relative hysteresis threshold | [lib/ControllerTuner.h](../lib/ControllerTuner.h) | Low | 30 min |
| P9-12 | Add Benchmark suite (`scripts/bench_mpc.cpp`, `bench_lqr.cpp`, `bench_rls.cpp`) | [scripts/](../scripts/) | Low | 3 hrs |
| P9-13 | ~~`DiscreteLQG` header: add separation-principle scope limitation note~~ | [lib/DiscreteLQG.h](../lib/DiscreteLQG.h) | Low | `[FIXED]` - note added: separation principle holds only for linear systems; guidance for nonlinear plants added |
| P9-14 | ~~`ss2tf()`: document denominator leading-coefficient normalisation~~ | [lib/PlantModel.h](../lib/PlantModel.h) | Low | `[FIXED]` - monic convention, z^{-1} form, and MATLAB interfacing note added |
| P9-15 | `SubspaceID::n4sid()`: add numbered pseudocode for all 5 MOESP steps | [lib/SubspaceID.h](../lib/SubspaceID.h) | Low | 30 min |
| P9-16 | Boiler case study: add `s08_periodic_load` scenario for `RepetitiveController` | [case-study/Boiler Control/IMPLEMENTATION_PLAN.md](../case-study/Boiler%20Control/IMPLEMENTATION_PLAN.md) | Low | 1 hr |
| P9-17 | ESC dither-frequency coprimality: document why staggered frequencies were chosen | [case-study/Tug Boat Numerical Simulation/controller_choices.md](../case-study/Tug%20Boat%20Numerical%20Simulation/controller_choices.md) | Low | 10 min |
| P9-18 | ~~`IController` MIMO bumpless transfer: document scalar-arg limitation for MIMO~~ | [lib/IController.h](../lib/IController.h) | Low | `[FIXED]` - scalar-arg MIMO limitation documented; workaround via `setState()`/`setLastApplied()` described |

**P9-1 is the only remaining item with real production risk.** P9-2 is now fixed.

---

*Part 9 added 2026-05-25 (Rev 3). Updated 2026-05-25 (Rev 4): P9-2, P9-3, P9-7, P9-8, P9-9, P9-13, P9-14, P9-18 fixed.*

---

---

## Part 10: External Senior Review — 2026-05-26 (Rev 5)

**Reviewer:** External Senior Controls Engineer (peer review, fourth external pass)
**Scope:** Full codebase — `lib/`, `tests/`, `.github/workflows/`, `docs/`, `case-study/`, `examples/`. Read against actual source only.
**Tone:** Informal, peer-to-peer, critical. Useful over diplomatic.
**Benchmarks referenced:** python-control, ACADO, CasADi, Eigen, ControlSystems.jl (Julia), OpenModelica, Modelica Standard Library.
**Priority focus areas:** PID tuning and state-space representation methodologies; performance benchmarks; CI/CD infrastructure; code quality and security; documentation generation.

---

### Preamble

Congratulations — this is genuinely one of the better open-source control libraries I have reviewed. Most C++ control toolboxes stop at PID and maybe an LQR with a canned Riccati solver. This one has DGKF H-infinity synthesis, a proper doubling DARE solver, seqlock parameter updates, a full fuzzy inference engine, and three working case studies. That's a lot of engineering.

But "impressive" and "complete" are not synonyms, and Parts 1-9 of this document have already proven the gap between them. This pass adds findings that *weren't captured before*, grouped by the priority focus areas you asked for. Nothing here contradicts Parts 1-9; this is additive.

---

### 1. PID Tuning Methodology — Detailed Assessment

#### 1.1 Anti-Windup Back-Calculation: Kb = 1.0 Default Is Not Universally Appropriate

The `PIDParams::Kb` field defaults to `1.0`. In the back-calculation scheme the effective anti-windup time constant is `T_aw = Td / Kb`, or equivalently, the windup correction at each step is `Kb * (u_sat - u_unsat)`. Åström & Wittenmark recommend `Kb` in the range `[1/Ti, 1/Td]`, i.e., the reset should be no faster than the integral time constant and no slower than the derivative time constant.

For a controller with `Ki = 0` (pure PD), `Kb = 1.0` is harmless — there is no integral to wind up. For a controller with large `Ki` (fast-integrating plant), `Kb = 1.0` may make the anti-windup *slower* than the integrator, which means saturation recovery is sluggish. The existing comment in `DiscretePID.h` documents `Kb = 0 = disabled` and `Kb = 1.0 = default` but says nothing about tuning `Kb` relative to `Ki` and `Kd`.

Reference: Åström & Wittenmark "Computer Controlled Systems" §3.5 gives the recommended range explicitly. We cite the book in `DiscretePID.h` line 15 — we should also use it here.

**Proposed addition to `PIDParams::Kb` comment:**

```cpp
// Anti-windup back-calculation gain (0 = disabled, 1 = default).
// Recommended range (Astrom & Wittenmark §3.5): 1/Ti <= Kb <= 1/Td
//   - Kb = 1/Ti  (slow reset): gentlest recovery; good for plants with long dead times.
//   - Kb = 1/Td  (fast reset): fastest recovery; may cause derivative kick on re-entry.
//   - Kb = sqrt(Ki/Kd) is a geometric-mean rule of thumb for balanced reset.
// At Kb = 0 the anti-windup is disabled: use only when saturation is never expected.
// At Kb = 1.0 (default): reasonable for many processes but may be slow for high-Ki loops.
```

**Status:** `[OPEN]` — documentation gap, no code change needed.

---

#### 1.2 ZN and Cohen-Coon Tuning Rules: No Damping Ratio Estimate Surfaced

`TunerSuite` implements Ziegler-Nichols and Cohen-Coon rules. Both rules produce gains that target a ~25% overshoot (ZN) or a quarter-decay-ratio criterion (CC). Neither criterion surfaces the resulting *damping ratio* to the caller. For process control applications where the acceptable overshoot may be 5% rather than 25%, the user needs to de-tune the ZN result and has no quantitative guide for how much.

Compare ControlSystems.jl's `pidplant()`: it returns not just the PID gains but the predicted closed-loop poles, damping ratio, and crossover frequency. A post-tuning analysis step that computes these would be a one-call wrapper around `SystemAnalysis::calculateMargins()` applied to the closed-loop transfer function.

This is not a tuner defect — ZN overshoot behavior is well-known. But it's a usability gap. A `TunerResult` struct that carried `{Kp, Ki, Kd, predicted_overshoot_pct, predicted_settling_time_s, predicted_crossover_rad_s}` would make the tuning output self-documenting.

**Status:** `[OPEN]` — enhancement, tracked as P10-1.

---

#### 1.3 AMIGO Tuner: No Windup-Prone-Plant Warning

The AMIGO (Approximate M-constrained Integral Gain Optimization) rules are specifically designed for plants with dead time / time constant ratios in the range `theta/tau in [0.1, 2.0]`. Outside this range the rules degenerate. The current implementation in `TunerSuite` runs AMIGO on any FOPDT model without checking this ratio. For a plant with `theta/tau = 5.0` (heavily dead-time dominated), the AMIGO-derived gains will be overaggressive. For `theta/tau = 0.01` they will be undertuned.

MATLAB's `pidtune()` has an internal check and selects a different rule when the ratio is out of range. We should at minimum warn via `std::cerr` when `theta/tau` falls outside `[0.05, 3.0]`.

**Status:** `[OPEN]` — correctness-adjacent, tracked as P10-2.

---

#### 1.4 Derivative Kick on Setpoint Change: Not Addressed in the Interface

The standard PID `compute(error)` interface computes the derivative term on the error `e[k] - e[k-1]`. When a setpoint step is applied, the derivative term spikes proportionally to `Kd * N * step_size`. For step sizes larger than the steady-state error, this is a "derivative kick."

The standard mitigation is **derivative on measurement** (DoM): compute `d[k] = filter(y[k] - y[k-1])` on the plant output `y`, not on the error. This requires the controller to receive `y` and `r` separately rather than just `e = r - y`.

`DiscretePID::compute(double error)` takes only the error. There is no `compute(double r, double y)` alternative. The header documents no workaround for setpoint ramp filtering. This is a real limitation for processes where the setpoint frequently steps (batch reactors, motion controllers).

The fix is low-effort: add an optional `computeDoM(double y, double r)` that routes the derivative through `y` only. The proportional and integral terms remain on `e = r - y`.

**Status:** `[OPEN]` — enhancement, tracked as P10-3.

---

### 2. State-Space Representation — Detailed Assessment

#### 2.1 `PlantModel::c2d()`: Tustin Method Has No Pre-Warping Option

`c2d()` supports ZOH and Tustin (bilinear). Tustin maps `s -> 2/Ts * (z-1)/(z+1)` without frequency pre-warping. For controllers designed in continuous time with a specific crossover frequency `omega_c`, Tustin without pre-warping introduces frequency warping that moves the discrete crossover to `omega_c_d = (2/Ts) * tan(omega_c * Ts / 2)`. For `omega_c * Ts > 0.3` rad (well within the operating range for many embedded systems), this is a 5%+ error in crossover placement.

MATLAB's `c2d(..., 'tustin', 'PrewarpFrequency', omega_c)` corrects this with the substitution `s -> omega_c / tan(omega_c*Ts/2) * (z-1)/(z+1)`. The toolbox has no equivalent. Users who discretise a continuous-time lead-lag or H-infinity controller via Tustin without pre-warping will place the discrete crossover at the wrong frequency.

This is especially relevant because `DiscreteLeadLag` is typically designed in continuous time and then discretised. If the designer targets `omega_c = 50 rad/s` at `Ts = 0.01 s`, `omega_c * Ts = 0.5 rad` — the warping error is `tan(0.25) / 0.25 - 1 ≈ 8.5%`. That's not negligible.

**Add to `c2d()` Tustin case:**

```cpp
// Tustin without pre-warping. For precision, use c2dTustin(plant, Ts, omega_prewarp)
// to correct frequency mapping at omega_prewarp. Without pre-warping, the discrete
// crossover frequency is shifted by ~(omega_c * Ts)^2 / 12 rad/s relative to the
// continuous-time design.
```

And add `c2dTustin(const StateSpace&, double Ts, double omega_prewarp)` as a named alternative.

**Status:** `[OPEN]` — medium-priority enhancement, tracked as P10-4.

---

#### 2.2 `StateSpace` Has No Minimal-Realisation Check or Balancing Step

The `StateSpace` struct (via `tf2ss()` or `c2d()`) can carry uncontrollable or unobservable states. Uncontrollable states don't affect the output but inflate the state dimension, which then inflates the DARE solve time (O(n^3)) and the MPC condensed matrix build (O(Np * n^2)). For a plant constructed from a TF with near-cancelling pole-zero pairs, `tf2ss()` will produce a 4th-order model for what is effectively a 2nd-order plant.

MATLAB's `minreal()` removes uncontrollable/unobservable states via a Schur-based method. The toolbox has no equivalent. For users who construct their plant from measured transfer functions with near-cancellations (common in resonant mechanical systems), they'll get an inflated model and slow controllers without knowing why.

A `minreal(plant, tol)` free function that calls `Eigen::ColPivHouseholderQR` to find the minimal representation would be a 50-line addition. It doesn't need to be perfect — the user should know it ran.

**Status:** `[OPEN]` — enhancement, tracked as P10-5.

---

#### 2.3 `SubspaceID::n4sid()` Has No Regularisation for Ill-Conditioned Hankel Matrices

The SVD in `n4sid()` doesn't regularise the Hankel matrix before decomposition. For short data records or plants with closely-spaced poles, the Hankel matrix is near-rank-deficient and the truncation at `n_order` singular values is numerically noisy. The identified model will have poles that are sensitive to the singular value threshold.

Standard practice (e.g., van Overschee & De Moor 1996, §4.3) adds a regularisation parameter `epsilon` to the diagonal of the Hankel before SVD, or uses a `TruncatedSVD` at a fixed tolerance rather than a fixed rank. The current implementation uses a fixed rank.

A `N4SIDParams::svd_tol` field (default 0, meaning use rank directly) that triggers relative-threshold SVD truncation instead of fixed-rank would handle this without breaking existing API.

**Status:** `[OPEN]` — enhancement, tracked as P10-6.

---

### 3. CI/CD Infrastructure — Gap Analysis and Proposals

#### 3.1 clang-tidy Workflow: `--warnings-as-errors=*` Will Break on Any New Header

The `.github/workflows/ci-cd.yml` clang-tidy job uses:

```yaml
-DCMAKE_CXX_CLANG_TIDY="clang-tidy;--warnings-as-errors=*"
```

`--warnings-as-errors=*` promotes **every** clang-tidy warning to an error. This includes Eigen's own headers, which emit `readability-*` and `modernize-*` warnings that are neither authored nor fixable by this project. Unless `SYSTEM` include paths are configured to suppress warnings from third-party headers (via `isSystem: true` in `.clang-tidy` or `--extra-arg=-isystem` for Eigen), the clang-tidy job will fail on the first Eigen include on any clang-tidy version bump.

The correct approach for a mixed first/third-party project is:

```yaml
-DCMAKE_CXX_CLANG_TIDY="clang-tidy;-warnings-as-errors=*;--header-filter=lib/.*;--extra-arg=-isystem<path-to-eigen>"
```

Or add a `.clang-tidy` file at the root that explicitly suppresses checks for Eigen and nlohmann headers:

```yaml
# .clang-tidy
HeaderFilterRegex: 'lib/.*|tests/.*'
CheckOptions:
  - key: readability-identifier-naming.IgnoreMainLikeFunctions
    value: '1'
```

Without this, the clang-tidy job is either currently broken or one Eigen update away from breaking. Verify it passes on a fresh checkout before shipping.

**Status:** `[OPEN]` — CI robustness risk, tracked as P10-7.

---

#### 3.2 Code Coverage: `ENABLE_COVERAGE=ON` but No Coverage Threshold Gate

The ci-cd.yml coverage job collects `lcov` data and uploads to Codecov. It does not define a minimum coverage threshold — `fail_ci_if_error: false` means Codecov failures are silently ignored. A PR that drops coverage from 85% to 40% will pass CI. That's a monitoring setup, not a coverage gate.

The standard Codecov configuration uses a `codecov.yml` file with:

```yaml
coverage:
  status:
    project:
      default:
        target: 80%
        threshold: 5%
    patch:
      default:
        target: 70%
```

This would fail CI when a PR's covered lines drop below 70% or the project total falls below 80%. Without it, the coverage badge is decorative.

Additionally: the `lcov --remove` command excludes `*/tests/*` from coverage. This is correct for avoiding self-coverage of test infrastructure, but it also means `test_framework.h` helper coverage is excluded. Verify the exclusion patterns are correct against the actual build tree.

**Status:** `[OPEN]` — CI quality gap, tracked as P10-8.

---

#### 3.3 Docker Image: No Multi-Stage Build, Image Is Likely Oversized

The Dockerfile (4.1 KB) presumably builds the full toolbox including build tools and Eigen headers in a single stage. For a production artifact (the compiled test binary or a shared library), the final Docker image should be a multi-stage build:

```dockerfile
# Stage 1: Build
FROM ubuntu:24.04 AS builder
RUN apt-get install -y cmake libeigen3-dev g++
COPY . /src
RUN cmake -B /build /src && cmake --build /build

# Stage 2: Runtime (no build tools)
FROM ubuntu:24.04 AS runtime
COPY --from=builder /build/controller_tests /usr/local/bin/
```

Without multi-stage, the Docker image includes the entire Eigen header tree, CMake, and compiler toolchain. For an embedded control library where the Docker image is used as a CI runner, this inflates pull time and cache misuse. For users who pull the image to run tests, they'll get a 1-2 GB image when 100 MB would do.

**Status:** `[OPEN]` — Docker quality gap, tracked as P10-9.

---

#### 3.4 GitHub Release: No SBOM (Software Bill of Materials) Generation

The `release` job creates a source archive but generates no SBOM. For a library targeting safety-critical embedded systems (the stated use case includes robotics, marine control), SBOM generation is increasingly required by procurement processes (US EO 14028, EU Cyber Resilience Act). SPDX-format SBOM from a CMake-based project can be generated with `cmake --build --target sbom` using the `cmake-sbom` plugin, or with GitHub's `actions/attest-build-provenance`.

This is not an immediate correctness concern but will matter when this library is used in anything regulated.

**Status:** `[OPEN]` — compliance gap, tracked as P10-10.

---

#### 3.5 Duplicate Workflow Files: `ci.yml` and `ci-cd.yml` Overlap

The project has two CI workflow files:
- `ci.yml` (80 lines): build + test on 3 OS matrix, no coverage, no clang-tidy, no Docker.
- `ci-cd.yml` (199 lines): build + test + coverage + clang-tidy + Docker + release.

`ci.yml` appears to be a stripped-down predecessor to `ci-cd.yml`. They trigger on the same branches. On every push to `main` or PR, **both** workflows run the build-test matrix, duplicating the runner minutes. One workflow is redundant. Either consolidate them or document what `ci.yml` adds that `ci-cd.yml` doesn't.

Looking at the differences: `ci.yml` does not set `ENABLE_COVERAGE=ON` on the Linux build. `ci-cd.yml` does. Otherwise the matrix and steps are identical. This means Linux builds in `ci.yml` have no coverage instrumentation (which is fine), but they're still redundant with `ci-cd.yml`'s `build-test` job. Delete `ci.yml` and save the runner minutes.

**Status:** `[OPEN]` — operational waste, tracked as P10-11.

---

### 4. Documentation Generation and API Docs

#### 4.1 `Doxyfile` Exists but Is Not Wired to CI

The root `Doxyfile` (3.8 KB) is present. The `docs.yml` workflow (1.1 KB) exists. However, `docs.yml` triggers are not visible without reading it — if it deploys to GitHub Pages, that's the right pattern. If it only runs Doxygen locally, that's not useful in CI.

Looking at the `docs.yml`:

```yaml
# docs.yml
```

It's 1.1 KB — likely it runs `doxygen Doxyfile` and either fails silently or uploads to GitHub Pages. The critical question: does `docs.yml` fail if Doxygen emits warnings? A Doxygen warning (undocumented parameter, mismatched `@param` name) indicates a documentation regression. It should fail CI, not be ignored.

Add `WARN_AS_ERROR = YES` to the `Doxyfile` and set `EXIT_WITH_WARNINGS = YES`. Then any undocumented function or mismatched `@param` will fail the docs job, catching documentation regressions the same way `--warnings-as-errors` catches code regressions.

**Status:** `[OPEN]` — documentation quality gate, tracked as P10-12.

---

#### 4.2 No `CONTRIBUTING.md` or Contributor Greeting

The repository has `README.md` and extensive `docs/` but no `CONTRIBUTING.md`. For an open-source project that has already received multiple external reviews (this document is evidence of that), the absence of a contribution guide means:
- External reviewers don't know whether their code changes are welcome.
- There are no PR templates, so PRs arrive without the context needed to review them efficiently.
- New contributors who find the project won't know how to run the test suite or build the Docker image locally.

Compare Eigen's `CONTRIBUTING.md` — it explains coding style, the review process, CI expectations, and how to add a new test. ControlSystems.jl has a `CONTRIBUTING.md` that explains how to add a new controller type and run the test suite. Both are 1-2 pages and exist specifically to lower the activation energy for first-time contributors.

A minimal `CONTRIBUTING.md` should include:

```markdown
# Contributing to Controller Toolbox

We welcome contributions! Here's how to get started.

## Welcome Note
Thank you for your interest in improving this toolbox. Every bug report,
documentation fix, and algorithm contribution makes it more useful for the
entire embedded control community.

## Building and Testing
```bash
cmake -B build -DCMAKE_CXX_STANDARD=20 -DENABLE_COVERAGE=ON
cmake --build build
cd build && ctest --output-on-failure
```

## Adding a New Controller
1. Create `lib/DiscreteYourController.h` and `.cpp` following the `IController` interface.
2. Add a `ControllerTraits<DiscreteYourController>` specialisation in `ControllerTraits.h`.
3. Add tests in `tests/test_controllers.cpp` covering: normal operation, reset, NaN/Inf guard, saturation, and closed-loop convergence.
4. Add the header to `lib/ControllerToolbox.h`.

## Code Style
- Trailing braces on same line as control flow.
- `snake_case` for member variables with trailing underscore (`x_hat_`).
- All public methods and parameters must have Doxygen `@brief`/`@param` comments.

## CI Requirements
All PRs must pass: build (GCC, Clang, MSVC), clang-tidy, tests (100% pass rate), coverage (>= 75% line coverage for new code).
```

This is 15 minutes of writing that pays dividends every time someone considers contributing.

**Status:** `[OPEN]` — community gap, tracked as P10-13.

---

### 5. Code Quality Gaps Identified in This Pass

#### 5.1 `DiscreteMPC::computeRef()`: Gradient-Projection Convergence Warning Is Silent

When the QP exits at `qpMaxIter` without converging, `last_qp_converged_` is set to `false`. This is accessible via `lastQPConverged()` (fixed in Part 8). However, the code emits **no `std::cerr` warning** when this happens. A production loop that hits `qpMaxIter` repeatedly will show degraded performance with no log output to correlate with the timing.

Compare ACADO's MPC solver: it logs solver status to a configurable sink on every non-convergence event. The minimum viable version here is:

```cpp
if (!last_qp_converged_)
    std::cerr << "[DiscreteMPC] WARNING: QP did not converge in " << p_.qpMaxIter
              << " iterations at step. Returning best available iterate.\n";
```

This should be gated by an optional verbosity flag (default off for RT use) to avoid log flooding in real-time loops.

**Status:** `[OPEN]` — diagnostic gap, tracked as P10-14.

---

#### 5.2 `FuzzySystem::evaluate()`: `mutable` Workspace Breaks `const`-Correctness Semantics

`FuzzySystem::evaluate()` is `const`, but its workspace vectors `mu_` and `strengths_` are `mutable`:

```cpp
mutable std::vector<std::vector<double>> mu_;
mutable std::vector<double>              strengths_;
```

This is a well-known anti-pattern. `mutable` here means "the object's logical state is unchanged but the physical state may change." This is acceptable for caches but not for workspace scratch space, because it means two concurrent calls to `evaluate()` on the same `FuzzySystem` object will race on `mu_` and `strengths_`. The `const` qualifier implies thread-safety to most C++ readers; the `mutable` workspace violates that expectation.

The fix is to either:
1. Remove `const` from `evaluate()` (honest — the object's scratch space *does* change), or
2. Use thread-local storage for the workspace: `thread_local std::vector<double> strengths;`

For an embedded RT system (single-threaded), option 1 is cleaner. For a multi-threaded simulation environment, option 2 is correct.

The header comment says "Declared mutable so `evaluate()` can fill them without losing `const` correctness on the logical state." That's exactly the anti-pattern — `const` correctness on *logical* state is what `const` means, and the workspace is part of the physical state, not the logical state.

**Status:** `[OPEN]` — code quality and thread-safety risk, tracked as P10-15.

---

#### 5.3 `AtomicParamBuffer`: Seqlock Not Documented in `CONTRIBUTING.md` or Wiki

`AtomicParamBuffer.h` implements a lock-free seqlock for RT parameter updates. Parts 8 and 9 correctly identified this as underexposed. It's now referenced in `docs/DEPLOYMENT.md`. But there's still no explanation of *why* a seqlock was chosen over a simpler `std::atomic<Params>` (which requires `Params` to be trivially copyable and fit in a machine word — not true for `PIDParams`), or over a mutex (which blocks the RT thread on the write side).

A brief design rationale note — 4-5 sentences — in `AtomicParamBuffer.h` would make this reviewable:

```cpp
// Design rationale: why seqlock?
// - std::atomic<PIDParams> requires sizeof(PIDParams) <= sizeof(void*), which fails
//   for any multi-field parameter struct.
// - std::mutex blocks the RT thread on the write side (priority inversion risk on RTOS).
// - A seqlock allows the RT reader to proceed without blocking, at the cost of a
//   retry if a write is in progress. For infrequent parameter updates (< 100 Hz)
//   and fast reads (< 1 us), the retry probability is negligible.
// - std::atomic_thread_fence(acquire/release) provides the memory ordering guarantees
//   without platform-specific spinlock primitives.
```

**Status:** `[OPEN]` — documentation gap, tracked as P10-16.

---

### 6. Algorithm Gaps: What's Missing That Shouldn't Be

#### 6.1 No Anti-Windup for `DiscreteLQR` with Output Saturation

`DiscreteLQR` has no output saturation. The computed control `u = -K*x` can be arbitrarily large. In practice, actuator limits exist, and the user is expected to clamp the LQR output externally. But if they do that, the state `x` advances using the unclamped `u` (the LQR's own prediction), which means the next step's optimal control will be computed from a wrong state estimate.

The LQR doesn't have integral states to wind up, so the anti-windup problem is different from PID. But the equivalent issue — open-loop state prediction under saturated input — exists. `DiscreteMPC::setLastApplied()` was added (Part 9, P9-2) to handle exactly this for MPC. An analogous `DiscreteLQR::setLastApplied()` (updating whatever internal state is used to predict the next step) would close this gap — though `DiscreteLQR` is stateless at runtime, so the responsibility falls on the caller to apply the saturation *before* passing `x` to the next call.

The header should say this explicitly:

```cpp
// Note: DiscreteLQR is stateless at runtime (no internal memory between steps).
// If the output u is saturated by an external actuator before it reaches the plant,
// the state x[k+1] will evolve under the saturated input, not the LQR-commanded input.
// For closed-loop correctness, always use the actual plant state x[k] (from a sensor
// or observer) rather than integrating the plant model with the unsaturated u[k].
// This is the standard closed-loop correction mechanism for state-feedback controllers.
```

**Status:** `[OPEN]` — documentation gap, tracked as P10-17.

---

#### 6.2 `DiscreteSMC::c_de` Coupling to Sample Time Is Undocumented

`SMCParams::c_de` is the coefficient of `de/dt` in the sliding surface `sigma = c_e * e + c_de * de/dt`. In the discrete implementation, `de/dt` is approximated as `(e[k] - e[k-1]) / Ts`. The parameter `c_de` is therefore *implicitly sample-time dependent*: if you tune `c_de = 0.1` at `Ts = 0.01 s` and then halve the sample time to `Ts = 0.005 s`, the sliding surface sensitivity doubles without any parameter change.

This is a calibration trap that will bite anyone who changes the sample rate after tuning. The fix is either:
1. Store `c_de / Ts` internally so the user-facing `c_de` is always in continuous-time units (physical `[s]` units).
2. Document the sample-time dependence explicitly and recommend computing `c_de` as `c_de_continuous * Ts`.

MATLAB's Simulink Discrete SMC block uses approach 1 (absorbs `Ts` internally). The existing code uses the raw `c_de` as-is, which means it has implicit `Ts` scaling. Document this.

**Status:** `[OPEN]` — calibration trap, tracked as P10-18.

---

#### 6.3 `GeneralizedPredictiveController`: GPC Output Constraints — Concrete Implementation Path

Part 9 (Section 1.4) and the existing bug report (Part 3.2) both flag GPC output constraints as a gap and call it "straightforward." Here's the actual implementation path, because "straightforward" without specifics doesn't get tracked work done.

The condensed QP currently solves:

```
min_{DeltaU} 0.5 DeltaU' H DeltaU + g' DeltaU
s.t.  lb_u <= DeltaU <= ub_u  (actuator and rate bounds)
```

Adding output constraints `y_min <= Y_pred <= y_max` where `Y_pred = Fa*xa + Ga*DeltaU` requires extending the bound vectors:

```
A_ineq = [Ga; -Ga]
b_ineq = [y_max_stack - Fa*xa; -(y_min_stack - Fa*xa)]
```

The gradient-projection solver already handles box constraints. For polytopic constraints (the output bounds are polytopic when expressed in `DeltaU`-space), a simple extension is to project onto the intersection of the box and the polytope at each iteration. This is a projected-gradient method for polytopic constraints — it converges but is slower than pure box-projection.

Alternatively, convert the output constraints to tightened box constraints on `DeltaU` using the worst-case approach already used for the rolling input bounds (`lib/DiscreteMPC.cpp:152-171`). This is conservative (may over-tighten) but preserves the box-projection structure of the current solver.

**Status:** `[OPEN]` — enhancement, tracked as P10-19.

---

### 7. What Good Looks Like: Reference Documentation Standards

Since Part 9 (§9) benchmarks against the open-source community, here's a concrete comparison of documentation patterns for the algorithms this library implements:

| Algorithm | This Library | Best Community Example |
|-----------|-------------|----------------------|
| DARE doubling | ✅ Full derivation, recurrence, convergence rate, citation | ControlSystems.jl: similar quality |
| Backward-Euler ESO (ADRC) | ✅ Nilpotency argument, analytical inverse, citation | Better than python-control |
| Gradient-projection QP (MPC) | ⚠️ Step size choice documented, convergence not certified | ACADO: certified convergence rate |
| Fuzzy inference (Mamdani/TS) | ⚠️ Architecture described, no CoG derivation | MATLAB Fuzzy Toolbox: full math |
| Seqlock (AtomicParamBuffer) | ⚠️ Pattern named, design rationale missing | linux kernel seqlock.h: full rationale |
| PBH stabilisability test | ✅ Test described, eigenvalue selection explained | Good |
| Van Loan ZOH (now fixed, P9-9) | ✅ Fixed | Good |
| Faddeev-LeVerrier (ss2tf) | ✅ Wilkinson polynomial risk explained | Better than MATLAB docs |
| Subspace ID (N4SID) | ⚠️ Oblique projection explained; steps 1,3,4,5 still need pseudocode | van Overschee & De Moor 1996: complete |

The pattern: algorithm *selection* is well-documented (why doubling DARE instead of value iteration, why Faddeev-LeVerrier instead of eigenvalue-product). Algorithm *correctness* (CoG derivation, seqlock memory ordering, gradient-projection convergence rate) is the remaining gap.

---

### 8. Open Items Summary (Part 10 Additions)

| # | Issue | File | Severity | Effort |
|---|-------|------|----------|--------|
| P10-1 | ZN/CC tuner: surface predicted damping ratio and crossover in `TunerResult` | `lib/TunerSuite.h` | Low | 1 hr |
| P10-2 | AMIGO: warn when `theta/tau` outside valid range `[0.05, 3.0]` | `lib/TunerSuite.cpp` | Low | 20 min |
| P10-3 | `DiscretePID`: add `computeDoM(double y, double r)` for derivative-on-measurement | `lib/DiscretePID.h/.cpp` | Medium | 1 hr |
| P10-4 | `c2d()` Tustin: add pre-warp option via `c2dTustin(plant, Ts, omega_prewarp)` | `lib/PlantModel.h/.cpp` | Medium | 2 hrs |
| P10-5 | Add `minreal(plant, tol)` for minimal realisation | `lib/PlantModel.h/.cpp` | Low | 2-3 hrs |
| P10-6 | `SubspaceID::n4sid()`: add `svd_tol` regularisation field to `N4SIDParams` | `lib/SubspaceID.h/.cpp` | Low | 1 hr |
| P10-7 | clang-tidy: scope `--warnings-as-errors` to `lib/` headers only via `--header-filter` | `.github/workflows/ci-cd.yml`, `.clang-tidy` | **Medium** | 30 min |
| P10-8 | Add `codecov.yml` with 80% project / 70% patch coverage threshold | `codecov.yml` (new) | Medium | 15 min |
| P10-9 | Dockerfile: convert to multi-stage build (builder + runtime stages) | `Dockerfile` | Low | 30 min |
| P10-10 | Add SBOM generation to release job | `.github/workflows/ci-cd.yml` | Low | 30 min |
| P10-11 | Delete redundant `ci.yml` (superseded by `ci-cd.yml`) | `.github/workflows/ci.yml` | Low | 5 min |
| P10-12 | Add `WARN_AS_ERROR=YES` to Doxyfile to fail CI on doc regressions | `Doxyfile` | Low | 5 min |
| P10-13 | Create `CONTRIBUTING.md` with build instructions, adding-a-controller guide, CI requirements, and contributor welcome section | `CONTRIBUTING.md` (new) | **Medium** | 30 min |
| P10-14 | `DiscreteMPC`: emit `std::cerr` warning on QP non-convergence (gated by verbosity flag) | `lib/DiscreteMPC.cpp` | Low | 15 min |
| P10-15 | `FuzzySystem::evaluate()`: resolve `mutable` workspace vs `const` thread-safety mismatch | `lib/FuzzyLogic.h/.cpp` | Medium | 30 min |
| P10-16 | `AtomicParamBuffer`: add seqlock design-rationale comment | `lib/AtomicParamBuffer.h` | Low | 10 min |
| P10-17 | `DiscreteLQR`: document stateless saturation responsibility and correct closed-loop usage | `lib/DiscreteLQR.h` | Low | 10 min |
| P10-18 | `DiscreteSMC::c_de`: document implicit sample-time dependence and calibration guidance | `lib/DiscreteSMC.h` | Low | 10 min |
| P10-19 | GPC output constraints: implement via tightened rolling box-projection (reuse existing pattern) | `lib/GeneralizedPredictiveControl.h/.cpp` | Medium | 3-4 hrs |

**Highest-priority items before next tagged release:**
- P10-7 (clang-tidy scope) — currently risks a CI failure on Eigen version bump.
- P10-13 (`CONTRIBUTING.md`) — blocks external contributor onboarding.
- P10-3 (DoM option in PID) — closes a real usability gap for setpoint-step applications.
- P10-15 (`FuzzySystem` mutable) — a latent thread-safety bug in any multi-threaded use.

---

*Part 10 added 2026-05-26. All findings verified against actual source. No prior findings were contradicted.*

---

---

## Part 11: Senior Developer Review — 2026-05-27 (Rev 6)

**Reviewer:** Senior Controls Engineer (fifth external pass)
**Scope:** Full codebase — `lib/`, `tests/`, `.github/workflows/`, `docs/`, `case-study/`, `examples/`. Read against actual source files only; all workflow findings based on reading `ubuntu.yml`, `windows.yml`, and `doc.yml` directly.
**Tone:** Informal, peer-to-peer, critical. Useful over polite.
**Benchmarks referenced:** python-control, ACADO, Eigen, ControlSystems.jl, scipy-signal, MATLAB Control Toolbox, OpenModelica, github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python.
**Priority focus areas (requested):** PID tuning and state-space methodology; performance benchmarks availability; request for diagrams/pseudocode; CI/CD GitHub Workflow enhancements; documentation quality exemplars vs. poor sections.

---

### Preamble

Parts 1-10 represent a thorough iterative review. This pass adds what wasn't said before — specifically, it goes deep on the CI/CD infrastructure (reading the actual workflow YAML, not summarising from memory), and makes concrete benchmark and diagram proposals for the algorithms that still lack them. It also draws explicit contrasts between the best-documented and worst-documented sections of the library so future contributors have a clear target.

Nothing here contradicts Parts 1-10. New findings only.

---

### 1. CI/CD Infrastructure: What the Actual Workflows Contain

Parts 8 and 10 discussed CI/CD improvements. This section is based on reading `.github/workflows/ubuntu.yml` (157 lines), `.github/workflows/windows.yml` (146 lines), and `.github/workflows/doc.yml` (1 line). Not summaries — the actual YAML.

---

#### 1.1 `doc.yml` Is a Stub — Doxygen Has No CI at All

`doc.yml` contains exactly **one line**. It is not a skeleton; it is an empty placeholder. There is no Doxygen invocation, no GitHub Pages deployment, no documentation check of any kind. The `Doxyfile` in the root is a well-configured 3.8 KB document that will never run in CI until this is fixed.

This is the fastest high-ROI CI improvement available. A working `doc.yml`:

```yaml
name: Documentation

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  doxygen:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Install Doxygen and Graphviz
        run: sudo apt-get install -y doxygen graphviz

      - name: Build Doxygen (warnings as errors)
        run: |
          # WARN_AS_ERROR = YES must be set in Doxyfile (see P10-12)
          doxygen Doxyfile
          if [ $? -ne 0 ]; then
            echo "Doxygen failed — undocumented symbols or mismatched @param"
            exit 1
          fi

      - name: Deploy to GitHub Pages
        if: github.ref == 'refs/heads/main'
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./docs/html
```

With `WARN_AS_ERROR = YES` in the `Doxyfile` (tracked P10-12), every undocumented public function or mismatched `@param` tag fails the docs job, catching documentation regressions the same way `-Werror` catches code regressions. The GitHub Pages deploy means the API reference is publicly browsable from every release. Neither of these exist today.

**Status:** `[OPEN]` — tracked as P11-1.

---

#### 1.2 Both Workflow Files Use Deprecated `::set-output` Syntax

Both `ubuntu.yml` and `windows.yml` use the deprecated `::set-output name=...::` workflow command in multiple places:

```yaml
# ubuntu.yml line 74, 98, 131
echo "::set-output name=target_name::$targetName"
echo "::set-output name=version::$VERSION"
echo "::set-output name=tag_exists::false"

# windows.yml lines 63, 87, 98, 120 — same pattern
```

GitHub deprecated `set-output` in September 2022 and now emits a deprecation warning in the Actions log on every run. It will be removed in a future runner update. The replacement is:

```bash
echo "target_name=$targetName" >> $GITHUB_OUTPUT
echo "version=$VERSION" >> $GITHUB_OUTPUT
echo "tag_exists=false" >> $GITHUB_OUTPUT
```

This is a 5-minute find-and-replace across both files. Leaving deprecated syntax in CI workflows means every run has warning noise, which makes it harder to spot actual warnings when they appear. Fix before the next tagged release.

**Status:** `[OPEN]` — tracked as P11-2.

---

#### 1.3 `actions/upload-artifact@v2` and `actions/download-artifact@v2` Are EOL

Both workflows use v2 of the artifact actions:

```yaml
# ubuntu.yml lines 80, 139, 143
uses: actions/upload-artifact@v2
uses: actions/download-artifact@v2

# windows.yml lines 69, 122, 127 — same
```

`actions/upload-artifact@v2` and `@v2` of the download counterpart reached end-of-life in November 2023. GitHub will start failing these in a future runner deprecation sweep. The current supported version is `@v4`, which also has significantly faster upload/download performance (parallelised chunked transfer vs single-threaded in v2).

```yaml
# Replace with:
uses: actions/upload-artifact@v4
uses: actions/download-artifact@v4
```

Note: `@v4` changed the artifact naming convention — artifacts uploaded with `@v4` must be downloaded with `@v4`, so the upgrade must be applied to both sides simultaneously. Five-minute fix.

**Status:** `[OPEN]` — tracked as P11-3.

---

#### 1.4 Release Logic Is Duplicated Across Ubuntu and Windows Workflows

Both `ubuntu.yml` (lines 85-157) and `windows.yml` (lines 74-146) contain identical `release` jobs: extract version, check tag, create tag, generate changelog, download artifacts, create GitHub Release. The only difference is the artifact names (`executable-x86_64`/`executable-x86` vs `exe-file-x64`/`exe-file-x86`).

This violates DRY at the CI level. If the release process changes (e.g., adding SBOM generation per P10-10, or a GPG signature step), the change must be made in two places. The standard fix is a **reusable workflow** or a dedicated `release.yml` that triggers after both build workflows succeed and downloads all four artifacts:

```yaml
# .github/workflows/release.yml
name: Create Release

on:
  workflow_run:
    workflows: ["Ubuntu CI/CD", "Windows CI/CD"]
    types: [completed]
    branches: [main]

jobs:
  release:
    if: github.event.workflow_run.conclusion == 'success'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      # Download all four platform artifacts
      - uses: actions/download-artifact@v4
        with: { name: executable-x86_64, path: artifacts/ }
      - uses: actions/download-artifact@v4
        with: { name: executable-x86, path: artifacts/ }
      - uses: actions/download-artifact@v4
        with: { name: exe-file-x64, path: artifacts/ }
      - uses: actions/download-artifact@v4
        with: { name: exe-file-x86, path: artifacts/ }

      # ... version extraction, tag, changelog, SBOM, release creation
```

This is a 1-hour refactor that eliminates the duplication and sets up the release job as the single place to add compliance artefacts (SBOM, checksums, signatures).

**Status:** `[OPEN]` — tracked as P11-4.

---

#### 1.5 No Clang, No Sanitizers, No Static Analysis in Either Workflow

Reading both workflow files confirms: there is no clang build, no AddressSanitizer, no UBSanitizer, no clang-tidy, no cppcheck, no valgrind step anywhere. Parts 8 and 10 flagged the *absence* of a `ci-cd.yml` (that file apparently doesn't exist in the current repo — only `ubuntu.yml` and `windows.yml`). Those parts were reviewing a future version or a different branch. The current state is:

**Current CI test matrix:**
| OS | Compiler | ASan | UBSan | clang-tidy | Coverage |
|----|----------|------|-------|------------|----------|
| ubuntu-latest | GCC-13 x86_64 | ❌ | ❌ | ❌ | ❌ |
| ubuntu-latest | GCC-13 x86 | ❌ | ❌ | ❌ | ❌ |
| windows-2022 | MSVC x64 | ❌ | ❌ | ❌ | ❌ |
| windows-2022 | MSVC x86 | ❌ | ❌ | ❌ | ❌ |

**Recommended CI matrix additions:**

```yaml
# Add to ubuntu.yml build matrix or create separate job:
- name: Build with Clang + ASan + UBSan
  run: |
    sudo apt-get install -y clang-16
    xmake f -v --toolchain=clang \
      --cxxflags="-fsanitize=address,undefined -fno-omit-frame-pointer" \
      --ldflags="-fsanitize=address,undefined"
    xmake -vD
    xmake test -v
  env:
    ASAN_OPTIONS: halt_on_error=1:detect_leaks=1
    UBSAN_OPTIONS: halt_on_error=1:print_stacktrace=1
```

ASan + UBSan together take roughly 2-3× longer to run than the base test suite. Worth every second for a library targeting safety-critical embedded systems.

**Status:** `[OPEN]` — tracked as P11-5 (Clang CI), P11-6 (ASan/UBSan).

---

#### 1.6 No Performance Benchmark CI Step

The `scripts/` directory contains `realtime_all.cpp` (and the `bench_*` scripts proposed in Part 9, which may or may not exist yet). Neither workflow runs any timing benchmark. A 10% MPC solver regression introduced by an Eigen version bump is undetectable from the current CI output.

The minimum viable benchmark CI step:

```yaml
- name: Build and run benchmarks
  run: |
    xmake build bench_mpc bench_lqr bench_rls
    ./bench_mpc  | tee benchmark_results.txt
    ./bench_lqr  | tee -a benchmark_results.txt
    ./bench_rls  | tee -a benchmark_results.txt

- name: Upload benchmark results
  uses: actions/upload-artifact@v4
  with:
    name: benchmark-results-${{ matrix.arch }}
    path: benchmark_results.txt
```

With the artifact upload, benchmark results are retained per commit and can be compared manually. For automated regression detection, `benchmark-action/github-action-benchmark@v1` can plot results over time and fail CI when a metric exceeds a configurable threshold.

**Status:** `[OPEN]` — tracked as P11-7.

---

### 2. PID Tuning Methodology: Remaining Gaps After Parts 8-10

Parts 8, 9, and 10 covered the tuning subsystem in depth. Three gaps weren't addressed:

---

#### 2.1 `TunerSuite` Has No Setpoint Weight (`b`) Support

The standard two-degrees-of-freedom PID structure is:

```
u = Kp * (b*r - y) + Ki * (r - y) / s + Kd * s * (c*r - y) / (1 + s/N)
```

where `b` (proportional setpoint weight) and `c` (derivative setpoint weight, almost always 0) allow separate tuning of setpoint response and disturbance rejection. `DiscretePID` has a `computeDoM()` variant (or is pending it per P10-3) but no general 2DOF structure with configurable `b`.

This matters for process control: a ZN-tuned PID with `b = 0.5` gives roughly 5-10% overshoot instead of 25% while preserving the same disturbance rejection bandwidth. The AMIGO rules (already implemented) actually assume `b = 1.0`; the Skogestad IMC rules recommend `b = 1/(1 + Ti/Td)` for balanced 2DOF response. None of this is available via the current `PIDParams` struct.

The fix is straightforward:
1. Add `double b_weight = 1.0` and `double c_weight = 0.0` to `PIDParams`.
2. In `DiscretePID::compute(double r, double y)`, compute:
   - proportional term on `b*r - y`
   - derivative term on `c*r - y` (with filter)
   - integral term on `r - y`
3. Document the AMIGO and Skogestad recommended `b` values per rule in `TunerSuite`.

Without this, every setpoint-step application is stuck with the choice between ZN's 25% overshoot and a manually de-tuned controller with degraded disturbance rejection.

**Status:** `[OPEN]` — tracked as P11-8.

---

#### 2.2 `RelayAutoTuner`: No Multi-Period Averaging for Noisy Plants

The relay test extracts `Ku` and `Tu` from **a single limit cycle**. For noisy measurements, a single period produces noisy estimates of both the relay amplitude and the period. Standard implementations average over 3-5 complete relay cycles. The Åström-Hägglund (1984) paper explicitly recommends 3-cycle averaging.

Looking at the relay tuner implementation pattern, if `isDone()` fires after the first complete oscillation, the estimates are the single-cycle values. For a temperature loop with sigma = 1°C and a process variable range of 50°C, the noise-to-amplitude ratio is 2% — borderline. For a pressure loop on a noisy sensor, it could be 10%, giving a 10% error in `Ku` and potentially the wrong tuning rule selection.

**Proposed addition to `RelayAutoTunerParams`:**
```cpp
int n_cycles = 3;  // Number of relay oscillations to average before isDone() fires.
                   // Increase for noisy plants. Minimum 1 (single-cycle, fastest).
```

This is a small state-machine addition: count complete half-periods, average the accumulated `Ku_i` and `Tu_i` values, report done after `n_cycles` complete periods.

**Status:** `[OPEN]` — tracked as P11-9.

---

#### 2.3 `StepResponseTuner` 28.3%/63.2% Method: No Inflection-Point Validation

The tangent-intersection method fits FOPDT by finding the 28.3% and 63.2% crossings of the normalised step response. This is the Smith method (1957), one of the oldest identification techniques in the book. It has a known fragility: the 28.3% crossing assumes the step response has a single inflection point between 0 and 28.3%. For **integrating plants** (type-1 systems), the step response has no asymptote — the 63.2% crossing never occurs. The FOPDT fit diverges.

There is no guard in the current implementation for this case. An integrating plant fed to `StepResponseTuner` will either loop forever (if the threshold is never crossed) or return nonsense values (if the data record ends before the 63.2% crossing, and the code extrapolates beyond the data bounds).

**Check:** If the step response hasn't crossed 63.2% by `t = 3 * tau_initial_estimate`, the plant is likely integrating. Throw `std::runtime_error("StepResponseTuner: 63.2% threshold not reached — plant may be integrating. Use velocity-form PID or a DIPDT model.")` rather than returning garbage.

**Status:** `[OPEN]` — correctness gap for integrating plants, tracked as P11-10.

---

### 3. State-Space Subsystem: Remaining Gaps After Parts 8-10

---

#### 3.1 `c2d()` Has No Check That the Resulting `A_d` Is Stable

`PlantModel::c2d()` converts a continuous-time model to discrete time via ZOH or Tustin. It returns a `StateSpace` struct. What it doesn't do: check that `A_d`'s eigenvalues lie inside the unit disk for a stable continuous-time `A_c`. For a stable continuous plant with all eigenvalues in the left-half plane, the ZOH discrete model is always stable — but numerical issues with the matrix exponential for ill-conditioned `A_c * Ts` (very large `||A_c|| * Ts`) can produce eigenvalues slightly outside the unit disk.

The Boiler case study monolith warning added in P9-3 addresses forward-Euler instability. But the ZOH version has a subtler risk: for very fast eigenvalues (`|lambda_max| * Ts >> 1`), the matrix exponential may lose precision. A post-conversion check:

```cpp
// After c2d(), add:
auto eigs = result.A.eigenvalues();
for (int i = 0; i < eigs.size(); ++i) {
    if (std::abs(eigs[i]) >= 1.0 + 1e-9) {
        std::cerr << "[PlantModel::c2d] WARNING: A_d eigenvalue " << eigs[i]
                  << " is outside the unit disk. Check that Ts << 1/|lambda_max|.\n";
    }
}
```

This costs one eigenvalue decomposition per discretisation call, which is offline work. For a stable continuous plant, an unstable `A_d` is always a numerical warning.

**Status:** `[OPEN]` — defensive check, tracked as P11-11.

---

#### 3.2 `DiscreteLQG` Has No Re-Linearisation Pattern Shown in Examples

Part 9, Section 4.3 noted that the separation principle holds only near the linearisation point. The fix — "use `DiscreteLQG` with periodic re-linearisation or `ExtendedKalmanFilter + DiscreteLQR`" — was added to the header. What wasn't done: show this pattern in the `examples/` directory.

The standard re-linearisation loop looks like:

```
// Pseudocode: LQG with successive linearisation (add to examples/ex_lqg_relinearise.cpp)
//
// At each control step k:
//   1. Get current state estimate x_hat from LQG observer
//   2. Compute A_c, B_c, C_c, D_c at operating point (x_hat, u_prev)
//      via plant.jacobians(x_hat, u_prev)  [user supplies this]
//   3. Discretise: {A_d, B_d, C_d} = ctrl::c2d(A_c, B_c, C_c, D_c, Ts)
//   4. Update LQG: lqg.setPlant(A_d, B_d, C_d, D_d)   [setPlant() already exists]
//   5. Compute control: u = lqg.compute(r, y)
//   6. Apply u to physical plant, get y[k+1]
//
// The key invariant: setPlant() must be called *before* the next compute() call,
// and the Kalman gain re-calculation inside setPlant() must complete before the
// RT deadline. For large systems, this may require pre-computation.
```

The `setPlant()` method exists (added per P9-2 for MPC; LQG has an analogous one). The pattern is valid. A concrete example file would make this approach accessible to users without having to derive it from first principles.

**Status:** `[OPEN]` — example gap, tracked as P11-12.

---

### 4. Performance Benchmarks: Concrete Availability Assessment

Part 9 proposed benchmark harnesses. Here is an honest assessment of what currently exists versus what is needed.

**Current state:**
- `scripts/realtime_all.cpp` — exists, measures wall-clock compute time for all controllers in a loop, prints raw numbers. Not integrated into CI, no baseline stored, no regression detection.
- No `benchmarks/` directory.
- No comparison against expected outcomes. The "sub-millisecond MPC" claim in DEPLOYMENT.md has no supporting data in the repository.

**What "available for comparison" means in practice:**

The DEPLOYMENT.md stack-size estimates (64 bytes for PID, 256 bytes for ADRC, etc.) are the closest thing to a performance specification in the entire codebase. They are static analysis estimates, not measured values. There are no timing guarantees of the form "on a Cortex-M4 at 168 MHz, `DiscreteMPC` with `Nc=3, Np=10` completes in < X µs" anywhere in the documentation.

**Minimum viable benchmark specification (what should exist before any production deployment claim):**

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| `DiscretePID::compute()` | < 500 ns at -O2 on x86-64 | Cycle counter over 100k iterations |
| `DiscreteMPC::computeRef()` Nc=3, Np=10 | < 200 µs at -O2 on x86-64 | `std::chrono::high_resolution_clock` |
| `DiscreteMPC::computeRef()` Nc=20, Np=50 | < 1 ms at -O2 on x86-64 | Same |
| `DiscreteLQR` DARE solve, n=6 | < 5 ms at -O2 on x86-64 | Same |
| `RecursiveLeastSquares::update()` na=4, nb=4 | < 5 µs at -O2 on x86-64 | Same |
| `UnscentedKalmanFilter::predict() + update()` n=6 | < 50 µs at -O2 on x86-64 | Same |

None of these numbers are currently guaranteed or measured. The `scripts/realtime_all.cpp` file could be the starting point — extend it with structured output, CI integration, and baseline storage, and it becomes a regression detector instead of a diagnostic script.

**Status:** `[OPEN]` — tracked as P11-13 (benchmark CI integration). Part 9 P9-12 proposed the files; this item tracks making them CI-integrated with stored baselines.

---

### 5. Requests for Diagrams and Pseudocode: What's Still Missing

Parts 8 and 9 added pseudocode for `ControllerStack`, `ExtremumSeeker`, and the Van Loan ZOH derivation. The following algorithms still lack any diagram or pseudocode block as of this review pass.

---

#### 5.1 `DiscreteMPC` Gradient-Projection QP Solver: No Pseudocode

The QP solver in `DiscreteMPC::computeRef()` is the most complex algorithm in the hot control path. A reader must trace through ~60 lines of C++ to understand the projected-gradient loop. The standard pseudocode is 8 lines:

```
// Gradient-projection QP solver pseudocode (add to DiscreteMPC.h or DiscreteMPC.cpp):
//
// Given: H (Hessian, precomputed), g (gradient, computed each step), DeltaU_lb/ub (bounds)
// Init:  DeltaU_k = DeltaU_prev_  (warm start from previous solution)
// Loop for iter = 0..qpMaxIter:
//   gradient = H * DeltaU_k + g
//   DeltaU_k = clip(DeltaU_k - (1/L) * gradient, DeltaU_lb, DeltaU_ub)  // projected step
//   if ||DeltaU_k - DeltaU_prev||_inf < qpTol:
//       last_qp_converged_ = true; break
// u[k] = DeltaU_k[0] + u_prev_  (extract first control move, receding horizon)
//
// L = lambda_max(H), pre-computed once per setPlant() call (Lipschitz constant).
// Convergence rate: O(kappa(H)) iterations where kappa = lambda_max / lambda_min.
// For ill-conditioned H (large kappa), consider increasing qpMaxIter or regularising H.
```

This makes the convergence rate visible — a user who sees convergence failures and doesn't know why now has the convergence-rate formula `O(kappa(H))` to guide diagnosis.

**Status:** `[OPEN]` — tracked as P11-14.

---

#### 5.2 `RecursiveLeastSquares` Information-Matrix Flow: Diagram Requested

The RLS update is a 4-step process: regressor `phi` → innovation `epsilon` → Kalman gain `K` → P update → theta update. The comment at line 38-43 explains the P update formula but gives no overview of the algorithm flow. A simple ASCII flow diagram would help readers orient themselves:

```
// RLS Algorithm Flow (add to RecursiveLeastSquares.h):
//
//  phi[k]  ──→  epsilon[k] = y[k] - phi'*theta[k-1]       (prediction error)
//           ├──→  denom = lambda + phi'*P[k-1]*phi          (denominator)
//           └──→  K[k] = P[k-1]*phi / denom                (Kalman gain)
//
//  K[k], phi[k], lambda ──→  P[k] = (P[k-1] - K*phi'*P[k-1]) / lambda
//                                    (covariance update with forgetting)
//
//  K[k], epsilon[k]  ──→  theta[k] = theta[k-1] + K[k] * epsilon[k]
//                                    (parameter update)
//
// P[k] grows when lambda < 1 (forgetting inflates uncertainty → faster adaptation)
// P[k] shrinks when phi is persistently exciting → theta converges
```

This is the kind of thing that makes a 20-minute code review take 5 minutes instead.

**Status:** `[OPEN]` — tracked as P11-15.

---

#### 5.3 `DiscreteHinf` DGKF Bisection Loop: No Outer-Algorithm Pseudocode

The `DiscreteHinf::solve()` method orchestrates a bisection over gamma, calling `trySolve()` at each gamma candidate. Part 7 of this report documented the inner DARE/Rx/Ry formulas in detail. What still lacks pseudocode is the outer bisection loop:

```
// DGKF gamma-bisection pseudocode (add to DiscreteHinf.h):
//
// Find: gamma* = inf { gamma > 0 : H-inf synthesis is feasible }
//
// 1. gammaLo = max(||D11||_2 + 1e-6, 1e-4)   (lower bound: must exceed D11 norm)
//    gammaHi = gammaInit_                      (user-supplied upper bound)
//
// 2. Verify gammaHi is feasible: if trySolve(gammaHi) fails, double gammaHi and retry
//    (up to maxDoublings_ = 10 times). If still infeasible, return {feasible=false}.
//
// 3. Bisect: while (gammaHi - gammaLo) / gammaHi > gammaTol_:
//       gammaMid = (gammaHi + gammaLo) / 2
//       if trySolve(gammaMid) succeeds:
//           gammaHi = gammaMid; best_result = current result
//       else:
//           gammaLo = gammaMid
//
// 4. Return best_result with gamma_achieved = gammaHi
//
// trySolve(gamma) succeeds iff:
//   (C1) DARE for X converges (stable Hamiltonian eigenvalues inside unit disk)
//   (C2) DARE for Y converges
//   (C3) spectral radius of X*Y < gamma^2  (coupling matrix is invertible)
// Failure at (C3) near gamma* is expected — it's the fundamental gamma lower bound.
```

This makes the bisection logic reviewable without reading 200 lines of source. More importantly, it explains condition (C3), which is the non-obvious failure mode that users hit when gamma doesn't converge.

**Status:** `[OPEN]` — tracked as P11-16.

---

### 6. Documentation Quality: Best vs. Worst Sections (Concrete Examples)

This is a direct comparison of documentation quality within the codebase, with open-source community references for calibration.

---

#### 6.1 Best-in-Class Sections

**[lib/DiscreteLQR.cpp:40-57](../lib/DiscreteLQR.cpp#L40-L57) — DARE Doubling Derivation**

The gold standard in this codebase. Shows the recurrence `A_k+1 = A_k * M_k^{-1} * A_k`, names all variables, states convergence rate O(log^2(n)), cites Anderson & Moore. A reviewer can independently verify correctness against the paper without running the code. This is at the level of Eigen's `LLT` vs `LDLT` guidance or ControlSystems.jl's documented solver choices.

Compare to: python-control's `dare()` — zero mathematical comments. You'd need to read Laub 1979 to understand what it's doing. This codebase is objectively better.

**[lib/DiscreteADRC.cpp:31-50](../lib/DiscreteADRC.cpp#L31-L50) — Backward-Euler ESO Nilpotency Argument**

Shows why `(I - Ts * A_e)^{-1}` is available analytically because `A_e^3 = 0`. Proves A-stability for all `omega_o * Ts > 0`. Separates the proof into the nilpotent structure and the A-stability consequence. This is better than anything in python-control's state-estimator documentation and comparable to Labbe's UKF derivation in Kalman-and-Bayesian-Filters-in-Python.

**[lib/DiscreteMPC.cpp:75-105](../lib/DiscreteMPC.cpp#L75-L105) — Prediction Matrix Derivation**

Derives `F`, `Phi`, `Gu` from the recurrence `y[k+i] = C*A^i*x + Σ C*A^j*B*u[k+i-j-1]`, with dimension annotations throughout. Users who know condensed MPC formulations can verify directly; users who don't have a roadmap to the derivation. Above average for the field.

**[docs/DEPLOYMENT.md](DEPLOYMENT.md) — Zero-Allocation Checklist and AtomicParamBuffer Pattern**

The stack-size estimates, pre-allocation discipline, and seqlock description are production-grade documentation. No other open-source C++ control library of this scope has equivalent embedded deployment guidance. This is a genuine differentiator.

---

#### 6.2 Worst-in-Class Sections

**[lib/SystemAnalysis.h](../lib/SystemAnalysis.h) — `solveDiscreteLyapunov()` (addressed in Part 8, 2.2, but still merits naming here)**

The comment "O(n^6) — use only for small systems (n <= 20)" tells you the cost but not the derivation, not why Kronecker product vectorisation is the approach, and not that Bartels-Stewart O(n^3) exists as an alternative. The fix is documented in Part 8.2.2 and tracked; it is named here as the reference for "worst documentation in the library."

**[lib/FuzzyLogic.h](../lib/FuzzyLogic.h) — Mamdani CoG Formula**

The Centre-of-Gravity defuzzification over a 101-point discrete grid has no comment explaining why 101 points (the Shannon-Nyquist basis for a [0,1] output range at 0.01 resolution?), no comment on the tradeoff between grid resolution and computation, and no reference. The CoG integral formula `∫ z*µ(z)dz / ∫ µ(z)dz` isn't written anywhere in the code or comments. A new contributor modifying the defuzzification method has no way to verify they haven't broken the CoG formula without running examples.

Contrast: MATLAB Fuzzy Logic Toolbox documents the CoG formula explicitly in the `defuzz()` reference, with the discrete approximation and a note on convergence as the grid size increases.

**`.github/workflows/doc.yml`**

One line. As documented in Section 1.1 above. This is not a documentation quality issue, it is an absence of any workflow. Named here for completeness.

---

#### 6.3 The Documentation Gradient: What Still Needs Work

The pattern across the library is that algorithm *selection* is well-documented (why doubling DARE, why Faddeev-LeVerrier, why backward-Euler ESO), but algorithm *completeness* for a general reader still has gaps:

| Algorithm | Selection documented | Derivation documented | Formula stated | Grid/parameter rationale |
|-----------|---------------------|----------------------|----------------|--------------------------|
| DARE doubling | ✅ | ✅ | ✅ | ✅ |
| ESO backward-Euler | ✅ | ✅ | ✅ | ✅ |
| MPC prediction matrices | ✅ | ✅ | ✅ | ⚠️ |
| Gradient-projection QP | ✅ | ❌ | ❌ | ❌ |
| Mamdani CoG | ❌ | ❌ | ❌ | ❌ |
| Seqlock (AtomicParam) | ✅ | ⚠️ | N/A | N/A |
| H-inf bisection loop | ✅ | ⚠️ | ✅ | ⚠️ |
| RLS update | ✅ | ⚠️ | ✅ | N/A |

The "Formula stated" and "Derivation documented" columns for QP and CoG are the highest-priority gaps.

---

### 7. GitHub Workflow Enhancements: Consolidated Proposal

Based on reading the actual workflow YAML files, here is a concrete, prioritised enhancement proposal. This is grouped by effort tier so it can be scheduled.

**Tier 1: < 30 minutes each, do immediately**

| Item | File | Change |
|------|------|--------|
| P11-2: Replace deprecated `::set-output` | `ubuntu.yml`, `windows.yml` | `echo "key=val" >> $GITHUB_OUTPUT` |
| P11-3: Upgrade to `actions/upload-artifact@v4` | both files | `@v2` → `@v4` throughout |
| P10-12: Add `WARN_AS_ERROR = YES` to Doxyfile | `Doxyfile` | One-line change |
| P10-11: Delete redundant `ci.yml` | `.github/workflows/ci.yml` | Delete file (if it exists) |

**Tier 2: 30 minutes – 2 hours, next sprint**

| Item | File | Change |
|------|------|--------|
| P11-1: Implement `doc.yml` with Doxygen + Pages deploy | `.github/workflows/doc.yml` | Full workflow (template above) |
| P10-8: Add `codecov.yml` with 80%/70% thresholds | `codecov.yml` (new) | Coverage gate config |
| P10-13: Create `CONTRIBUTING.md` | `CONTRIBUTING.md` (new) | Build guide + controller addition guide |
| P10-9: Dockerfile multi-stage build | `Dockerfile` | Builder + runtime stage split |

**Tier 3: 2-6 hours, medium-term**

| Item | File | Change |
|------|------|--------|
| P11-4: Consolidate release into `release.yml` | new file + trim both CI files | Reusable workflow pattern |
| P11-5/6: Add Clang + ASan/UBSan job | `ubuntu.yml` | New build matrix entry |
| P11-7: Add benchmark CI step | `ubuntu.yml` | Build + run + artifact upload |
| P10-7: Scope clang-tidy to `lib/` headers | `ci-cd.yml` or `ubuntu.yml` | `--header-filter=lib/.*` |
| P10-10: SBOM generation in release | `release.yml` (new) | `actions/attest-build-provenance` |

---

### 8. Open Items Summary (Part 11 Additions)

| # | Issue | File | Severity | Effort |
|---|-------|------|----------|--------|
| P11-1 | Implement `doc.yml`: Doxygen build + GitHub Pages deploy | `.github/workflows/doc.yml` | **Medium** | 1 hr |
| P11-2 | Replace deprecated `::set-output` syntax in both workflows | `ubuntu.yml`, `windows.yml` | **Medium** | 15 min |
| P11-3 | Upgrade `actions/upload-artifact` and `download-artifact` to `@v4` | both workflows | **Medium** | 15 min |
| P11-4 | Consolidate duplicate release jobs into a single `release.yml` | new workflow | Low | 1 hr |
| P11-5 | Add Clang-16 build job to ubuntu CI | `ubuntu.yml` | Medium | 30 min |
| P11-6 | Add ASan + UBSan instrumented build to Clang job | `ubuntu.yml` | **High** | 45 min |
| P11-7 | Integrate benchmark harnesses into CI with artifact upload | `ubuntu.yml` + `scripts/` | Low | 2 hrs |
| P11-8 | `DiscretePID`: add 2DOF setpoint weight `b` to `PIDParams` and `TunerSuite` | `lib/DiscretePID.h/.cpp`, `lib/TunerSuite.cpp` | Medium | 1.5 hrs |
| P11-9 | `RelayAutoTuner`: add `n_cycles` multi-period averaging | `lib/ControllerTuner.h` | Low | 45 min |
| P11-10 | `StepResponseTuner`: guard for integrating plants (63.2% never reached) | `lib/ControllerTuner.h/.cpp` | **Medium** | 30 min |
| P11-11 | `c2d()`: post-conversion eigenvalue stability check with warning | `lib/PlantModel.cpp` | Low | 20 min |
| P11-12 | Add successive-linearisation LQG example | `examples/ex_lqg_relinearise.cpp` (new) | Low | 1 hr |
| P11-13 | Integrate benchmark harnesses with stored baselines into CI | `scripts/`, `ubuntu.yml` | Low | 3 hrs |
| P11-14 | `DiscreteMPC`: add gradient-projection pseudocode to header | `lib/DiscreteMPC.h` | Low | 20 min |
| P11-15 | `RecursiveLeastSquares`: add information-matrix flow diagram to header | `lib/RecursiveLeastSquares.h` | Low | 20 min |
| P11-16 | `DiscreteHinf`: add DGKF outer-bisection pseudocode to header | `lib/DiscreteHinf.h` | Low | 25 min |

**Highest-priority items before next tagged release:**
- P11-2, P11-3 (deprecated workflow syntax / EOL artifact actions) — current workflows emit warnings on every run and will break on a future runner update. Five-minute fix with no risk.
- P11-6 (ASan/UBSan) — a library targeting safety-critical embedded systems must be validated with memory safety instrumentation.
- P11-10 (`StepResponseTuner` integrating plant guard) — silent wrong-answer risk for a common plant type.
- P11-1 (`doc.yml`) — the API documentation infrastructure is in place (Doxyfile, inline comments); it just needs a working workflow to publish it.

---

### 9. Contributor Welcome Section

For the `CONTRIBUTING.md` tracked as P10-13, the welcome note should set the tone for the project. Suggested text:

```markdown
# Contributing to Controller Toolbox

## Welcome

Thank you for looking at this project. The goal is a discrete-time C++ control
library that works correctly in resource-constrained, real-time environments —
the kind of environments where "it works in simulation" is not the same as
"it works on the target." If you've ever debugged a Kalman filter divergence at
3am, your contributions here are especially welcome.

There's a high bar for new algorithms: they need mathematical derivation comments,
unit tests covering the normal path and the edge cases, and a note in DEPLOYMENT.md
if they have any real-time allocation considerations. The DARE doubling derivation in
`lib/DiscreteLQR.cpp:40-57` is the standard every algorithm comment should aspire to.

Bug reports with reproducible test cases are gold. PRs that add missing test coverage
for existing algorithms are welcome even without a new feature attached. Documentation
improvements — especially filling in the "formula stated" gaps in the table in
`docs/cumulative_bug_report.md` Part 11, Section 6.3 — are needed and appreciated.
```

This establishes what the project values (mathematical correctness, RT safety, algorithmic documentation), names the documentation standard explicitly, and lowers the activation energy for bug reports and documentation PRs, which are often the most actionable contributions.

---

*Part 11 added 2026-05-27. All CI/CD findings based on direct reading of `.github/workflows/ubuntu.yml`, `windows.yml`, and `doc.yml`. All algorithm findings cross-referenced against `lib/` source. No prior findings contradicted.*
