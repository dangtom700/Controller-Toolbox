# Controller Toolbox – Cumulative Bug Report

**Last updated:** 2026-05-25  
**Author:** Senior Controls Engineer  
**Scope:** Full codebase audit — `lib/`, `tests/`, `case-study/`, `docs/`, `examples/`. All findings verified by reading actual source, not from memory or prior reports.

---

## How to Read This Document

This is a living cumulative report. It supersedes the individual dated reports (05-19 through 05-26) by consolidating their findings, marking what was fixed, and adding new observations from the current pass. The goal is to stop re-discovering the same things across review cycles.

**Status tags:** `[FIXED]` = verified in current source. `[OPEN]` = still present. `[NEW]` = first noted in this pass.

---

## A Note on Methodology: Reading vs. Assuming

The 05-26 report listed three "active defects" as unfixed. Every single one of them is actually fixed in the current source:

- `RecursiveLeastSquares::toTransferFunction()` missing zero-prepend — **fixed** ([lib/RecursiveLeastSquares.cpp:110-111](../lib/RecursiveLeastSquares.cpp#L110-L111))
- `UnscentedKalmanFilter::update()` missing eigenvalue floor — **fixed** ([lib/UnscentedKalmanFilter.cpp:142-146](../lib/UnscentedKalmanFilter.cpp#L142-L146))
- `DiscreteLQR::solveDARE()` missing symmetry enforcement — **fixed** ([lib/DiscreteLQR.cpp:96](../lib/DiscreteLQR.cpp#L96))

This is a problem. Reviews that say "unfixed" when the fix is sitting there in the code erode trust in the process and cause real fixes to be re-worked unnecessarily. Going forward: before writing "OPEN" on a defect, read the actual line.

---

## Part 1: Status of All Prior Findings

### From 05-19 Report

| Item | Status |
|------|--------|
| PID derivative filter sign error | `[FIXED]` |
| SmithPredictor buffer not pre-allocated | `[FIXED]` – `y_buf_.assign(d_, 0.0)` in constructor |
| LQR gain matrix dimensions unchecked | `[FIXED]` – PBH tests at construction |
| Missing `reset()` on SmithPredictor | `[FIXED]` |
| NaN/Inf guards in `DiscretePID`, `DiscreteSMC`, `DiscreteADRC` | `[FIXED]` – `isfinite` check present in all three |
| LDLT health check in `DiscreteMPC::computeRef()` | `[FIXED]` – `ldlt_.info() != Eigen::Success` return path |
| LDLT floor in `KalmanFilter::update()` | `[FIXED]` – R diagonal clamped to `1e-12` |
| `log(0)` guard in `SystemAnalysis::calculateMargins()` | `[FIXED]` |
| Pre-allocated work vectors in `DiscreteMPC` | `[FIXED]` – `R_stack_`, `pred_err_`, `grad_`, etc. are members |
| DARE convergence struct `DareResult` | `[FIXED]` – `{P, converged, iterations}` returned |
| PBH stabilizability + detectability pre-checks in `DiscreteLQR` | `[FIXED]` – both tests implemented correctly |

### From 05-23 Report

| Item | Status |
|------|--------|
| EKF Jacobian not zeroed between updates | `[FIXED]` |
| RLS DC gain test too loose (30%) | `[FIXED]` – replaced with direct parameter convergence checks at 5% tolerance ([tests/test_controllers.cpp:1212-1213](../tests/test_controllers.cpp#L1212-L1213)) |
| UKF Wc(0) < 0 not warned | `[FIXED]` – warning added in constructor ([lib/UnscentedKalmanFilter.cpp:43-46](../lib/UnscentedKalmanFilter.cpp#L43-L46)) |
| SubspaceID Hankel not checked for rank | `[FIXED]` – `n_order > svd.singularValues().size()` guard present |
| `FuzzyPID::bumplessInit` integral wrong | `[FIXED]` |
| `AtomicParamBuffer` data race | `[FIXED]` – seqlock pattern implemented |
| `FuzzySystem::defuzzWeightedAvg` singleton MF broken | `[FIXED]` – `LinguisticTerm::peak` field added |
| `ss2tf` eigenvalue polynomial unstable | `[FIXED]` – Faddeev-LeVerrier recursion used |
| `tf2ss` O(n²) `insert(begin)` | `[FIXED]` – single O(n) insert used |
| GPC Hessian re-factored every step | `[FIXED]` – `ldlt_` cached as member |
| `FuzzySupervisor` cooldown not reset on `reset()` | `[FIXED]` |

### From 05-24 Report

| Item | Status |
|------|--------|
| MPC condensed matrices not rebuilt on `setPlant()` | `[FIXED]` – `buildCondensedMatrices()` called in `setPlant()` |
| GPC augmented-state drift after saturation | `[FIXED]` – `du = u - u_prev_` recomputed after clamp ([lib/GeneralizedPredictiveControl.cpp:160](../lib/GeneralizedPredictiveControl.cpp#L160)) |
| H-inf gammaLo hardcoded 0.01 | `[FIXED]` – now `max(||D11||_2 + 1e-6, 1e-4)` |
| H-inf D22 != 0 not warned | `[FIXED]` – warning in `solve()` ([lib/DiscreteHinf.cpp:451-455](../lib/DiscreteHinf.cpp#L451-L455)) |
| ControllerStack bumpless tolerance < 1.0 | `[FIXED]` – tightened to < 0.1 |

### From 05-25 Report

| Item | Status |
|------|--------|
| H-inf excluded from ControllerToolbox.h without explanation | `[FIXED]` – `#ifndef CTRL_DISABLE_HINF` guard in place |
| SmithPredictor `setModel()` missing | `[FIXED]` – method present ([lib/SmithPredictor.cpp:65](../lib/SmithPredictor.cpp#L65)) |
| ADRC `compute()` missing `r_was_set_` guard | `[FIXED]` – `assert(r_was_set_)` present ([lib/DiscreteADRC.cpp:88](../lib/DiscreteADRC.cpp#L88)) |
| Compiler flags not wired | `[FIXED]` |
| `[DBG]` cout lines in test file | `[FIXED]` – removed from `test_hinf()` |
| H-inf Newton refinement dead code | `[FIXED]` – block removed |

### From 05-26 Report

| Item | 05-26 Status | Actual Status |
|------|-------------|----------------|
| RLS `toTransferFunction()` missing b0=0 | "OPEN – High" | `[FIXED]` – `Eigen::VectorXd::Zero(na_+1)` with `.segment(1, nb_)` |
| UKF covariance PSD violation | "OPEN – Medium-High" | `[FIXED]` – eigenvalue floor at [lib/UnscentedKalmanFilter.cpp:143-146](../lib/UnscentedKalmanFilter.cpp#L143-L146) |
| LQR DARE symmetry enforcement | "OPEN – Medium" | `[FIXED]` – `X = 0.5*(X_new + X_new.transpose())` at [lib/DiscreteLQR.cpp:96](../lib/DiscreteLQR.cpp#L96) |
| LQG step() causal comment missing | "Open – Medium" | `[FIXED]` – two-paragraph comment at [lib/DiscreteLQG.cpp:28-34](../lib/DiscreteLQG.cpp#L28-L34) |
| MPC open-loop drift undocumented | "Open – Low-Medium" | `[FIXED]` – documented at [lib/DiscreteMPC.cpp:198-203](../lib/DiscreteMPC.cpp#L198-L203) |
| RLS toStateSpace D==0 regression test missing | "Medium" | `[FIXED]` – test at [tests/test_controllers.cpp:1241-1258](../tests/test_controllers.cpp#L1241-L1258) |
| UKF PSD-maintenance test missing | "Medium" | `[FIXED]` – per-step eigenvalue check added over 200 steps |
| LQG separation principle test missing | "Low" | `[FIXED]` – test at [tests/test_controllers.cpp:828-875](../tests/test_controllers.cpp#L828-L875) |
| N4SID pole magnitude test weak | "Medium" | `[FIXED]` – pole magnitude accuracy check at 3% tolerance ([tests/test_controllers.cpp:1783-1790](../tests/test_controllers.cpp#L1783-L1790)) |

---

## Part 2: Active Issues (Open)

---

### Issue 1 — `DiscreteHinf::solveHinfDARE()` and `trySolve()`: Debug `std::cerr` Lines Left in Production Code

**File:** [lib/DiscreteHinf.cpp:136-144](../lib/DiscreteHinf.cpp#L136-L144) and [lib/DiscreteHinf.cpp:301,307](../lib/DiscreteHinf.cpp#L301-L307)  
**Severity:** Medium — not a correctness defect but a production-readiness failure  
**Status:** `[OPEN]`

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
// In solveHinfDARE() — remove lines 136-140, 144, 186:
// DELETE: std::cerr << "[DBG DARE] n=" << n << " eigenvalues:"; ...
// DELETE: std::cerr << "\n[DBG DARE] stable_count=" ...
// DELETE: std::cerr << "[DBG DARE] dare_res=" ...

// In trySolve() — remove lines 301 and 307:
// DELETE: std::cerr << "[DBG trySolve] gamma=" ...
// DELETE: std::cerr << "[DBG trySolve] dareConvX=" ...
```

No replacement needed. DARE convergence is already returned in `DareOut::conv` and `HinfResult::dareConvX/Y`. The synthesis result is validated via the DARE residual norm check (`dare_res < 1e-6`).

---

### Issue 2 — `RecursiveLeastSquares::update()`: The RLS P-Update Comment Is Still Misleading

**File:** [lib/RecursiveLeastSquares.cpp:38-43](../lib/RecursiveLeastSquares.cpp#L38-L43)  
**Severity:** Low — no code defect, but the 05-26 report specifically called this out as a maintenance trap  
**Status:** `[OPEN]`

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

### Issue 3 — `MixedSensitivity::build()` D22 Slot: Architecture Inconsistency

**File:** [lib/DiscreteHinf.cpp:778-786](../lib/DiscreteHinf.cpp#L778-L786)  
**Severity:** Low — no wrong answer for the intended case (dG=0), but structurally inconsistent  
**Status:** `[OPEN]`

`MixedSensitivity::build()` populates `P.D22(0,0) = dG` (the plant direct feedthrough), and `solve()` correctly warns when `P.D22.norm() > 1e-12`. But `trySolve()` ignores D22 entirely — the comment says "D22 assumed zero (standard form)." So the information is written into the struct, the warning fires, and the synthesis proceeds treating D22 as zero. For plants with nonzero direct feedthrough, the synthesised controller is subtly wrong with no further indication beyond the warning that is easy to ignore.

Two clean options:
1. Have `build()` throw when `|dG| > 1e-12` instead of silently recording it, or
2. Document explicitly that the caller must apply loop-shifting to zero D22 before calling `solve()`.

Currently neither is done. The warning is necessary but not sufficient.

---

### Issue 4 — `DiscreteSMC` Boundary-Layer Test Gap

**File:** [tests/test_controllers.cpp](../tests/test_controllers.cpp) — `test_smc()` section  
**Severity:** Low  
**Status:** `[OPEN]` — carried from 05-26 Section 4.3

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

### Issue 5 — `SubspaceID::n4sid()`: Regression for `D != 0` Plants Not Covered

**File:** [tests/test_controllers.cpp](../tests/test_controllers.cpp) — `test_n4sid()` section  
**Severity:** Low  
**Status:** `[OPEN]`

The 05-25 report added a pole magnitude check (3% tolerance) for the identified model. N4SID identification of a plant with `D != 0` is still not tested. The B and D regression ([lib/SubspaceID.cpp:161-187](../lib/SubspaceID.cpp#L161-L187)) solves them separately, which is correct, but the D regression can pick up B content if the regressor is ill-conditioned. For D=0 plants (the common case), this is never triggered.

Recommended addition: one test with a SISO plant `H(z) = 0.3 z^{-1} + 0.1 z^{-2}` with `D = 0.2`, verify the identified `D` is within 10% of 0.2 and `B` is within 10% of the true B.

---

## Part 3: Algorithm Gap Analysis

---

### 3.1 Gaps That Are Fine and Should Stay That Way

**Explicit MPC / multi-parametric QP.**  
Piecewise-affine offline law requires solving exponentially many QPs (impractical above n≈6). The gradient-projection online solver in `DiscreteMPC` is the right tradeoff. Not a gap; it is a scope decision.

**MRAC / VRFT / Iterative Feedback Tuning.**  
These require plant-specific Lyapunov proofs or persistent-excitation conditions that cannot be packaged generically. The library provides the primitives (RLS for online parameter adaptation, N4SID for batch identification, TunerSuite for offline search). Users who need full adaptive control can wire these themselves.

**Robust tube MPC.**  
Ellipsoidal uncertainty propagation requires either a bundled convex solver or plant-specific ellipsoid algebra. Both are out of scope for a header-only embedded-friendly library.

**Distributed / networked control.**  
Single-agent by design. Nothing to fix.

**Continuous-time H∞.**  
The library solves the discrete DGKF only. The `DiscreteHinf` header says "DGKF discrete version" — correct. Out of scope by design.

---

### 3.2 Gaps Worth Noting (but Not Defects)

**Fractional dead-time in SmithPredictor.**  
The current implementation ([lib/SmithPredictor.cpp:14-16](../lib/SmithPredictor.cpp#L14-L16)) buffers integer delay steps only. `StepResponseTuner::identify()` returns a FOPDT model where dead time `theta` is floating-point. For plants where `theta/Ts` is not close to an integer, users must round and absorb the error. The standard fix is a first-order Padé approximant for the fractional part:

```
H_frac(z) ≈ (1 - theta_frac / (2*Ts) * (z-1)) / (1 + theta_frac / (2*Ts) * (z-1))
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

1. **`SubspaceID::n4sid()` Step 2 comment (line 89):** explains the LQ step mechanically ("thin LQ decomposition via QR of Z'") but does not explain *why* the `L32` block equals the oblique projection of `Yf` onto `Wp || Uf`. The MOESP derivation (Verhaegen & Dewilde 1992) makes this clear; the comment should cite the equation number.

2. **`DiscreteHinf::trySolve()` Condition (C3) check (line 358):** the code computes `rho(Xs * Ys) < gamma^2` but the comment just says "spectral radius of X * Y < gamma^2." A reader needs to know *why* this condition is required (it is the coupling condition from DGKF Theorem 3.1). One sentence would close this.

3. **`RecursiveLeastSquares::update()` (Issue 2 above):** the P-update formula needs a derivation comment, not just an explanation of what forgetting does.

The pattern in the worst cases: explain the *effect*, skip the *derivation*. The best comments explain both.

---

## Part 4: Code Quality — What's Working Well

**`DiscreteMPC::buildCostMatrix()` / `buildPredictionMatrices()` split.**  
The separation between plant-dependent (`F_`, `Phi_`, `Gu_`) and weight-dependent (`H_`, `ldlt_`) matrix construction is clean. `setParams()` correctly triggers only the weight rebuild when only `rho_y` / `rho_u` change. The pre-allocated work vectors and pre-factored `ldlt_` make `computeRef()` allocation-free in steady-state. This is the correct design for an RT-targeted MPC.

**`ControllerTraits` compile-time enforcement.**  
The static metadata matrix mapping controllers to compatible tuners, with `static_assert` on mismatches and `[[deprecated]]` for partial tunings (LQG with pole-placement, which leaves the Kalman untuned), is genuinely useful. The error messages name the correct tuner rather than just saying "incompatible."

**`PlantModel::ss2tf()` via Faddeev-LeVerrier.**  
Using the algebraic recurrence for the characteristic polynomial instead of `∏(z - λ_i)` is the right numerical choice. The Wilkinson polynomial effect on eigenvalue-based characteristic polynomial computation is well-documented and nasty for clustered eigenvalues. The current implementation is stable where the obvious implementation is not.

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
| 5 | Add N4SID D != 0 regression test | [tests/test_controllers.cpp](../tests/test_controllers.cpp) | Low | 30 min |
| 6 | SmithPredictor: Padé approximant for fractional delay | [lib/SmithPredictor.h/.cpp](../lib/SmithPredictor.h) | Low | 2-3 hrs |
| 7 | FuzzySystem: document single-output limitation | [lib/FuzzyLogic.h](../lib/FuzzyLogic.h) | Low | 10 min |
| 8 | Add UKF/EKF additive-noise assumption note to headers | [lib/UnscentedKalmanFilter.h](../lib/UnscentedKalmanFilter.h), [lib/ExtendedKalmanFilter.h](../lib/ExtendedKalmanFilter.h) | Low | 10 min |

Items 1-3 should be done before the next tagged release. The rest are quality improvements without correctness impact.

---

## Part 6: What to Do Differently in the Next Review

1. **Verify before declaring open.** Before writing "OPEN" on a defect from a prior report, read the file at the line number. This cycle lost credibility by listing three already-fixed issues as active defects.

2. **Separate documentation gaps from code defects.** The 05-26 report mixed these and it made the severity table misleading. A missing comment is not the same severity as a wrong formula.

3. **Maintain this document instead of writing a new one per cycle.** The value of a cumulative report is tracking what's been tried, what was found to be wrong, and what actually got fixed. Spinning up a new report each time discards that history.

4. **When calling out documentation gaps, write the intended comment.** The 05-26 report said "add a two-line comment explaining the causal constraint" — fine. But writing the actual two lines in the report would have taken 30 seconds and removed all ambiguity.

---

*End of cumulative report.*
