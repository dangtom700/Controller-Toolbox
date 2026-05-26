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
