# Controller Toolbox - Code Review Report

**Date:** 2026-05-25 (revised)
**Reviewer:** Senior Controls Engineer
**Scope:** Full audit of `lib/`, `tests/`, `examples/`, `scripts/`, `case-study/` - correctness, numerical robustness, API ergonomics, RT readiness. Prior report 2026-05-24 used as baseline. All findings verified against current source.

---

## Overview

The picture is better than the previous report implies - several items it listed as open bugs have already been fixed and were simply not re-verified before publication. The ESC phase accumulator, the margin phase-unwrapping, and the ADRC interface inconsistency are all corrected in the current code. The LQR silent-drop and MIMO throw are also there. That's meaningful progress.

What remains is a smaller, more surgical list. A few items are medium-severity correctness issues, a handful are quality-of-life. The H-infinity addition (the big new piece since the 05-24 report) is architecturally solid but has a dead-code Newton refinement stub and a test that exercises the wrong interface. The SubspaceID B/D alignment question is still unconfirmed. And the test file has scattered debug output that needs to come out before the next CI run.

---

## 1. Status of Items from Prior Reports

### From 2026-05-25 report (previous version)

| Item | Actual Status |
|---|---|
| ESC phase accumulator overflow (Section 2.1) | **Already fixed** - `phase_` accumulator with `std::fmod` wrapping is in place in `ExtremumSeeker.cpp:35`. The old `step_ * Ts_` form is gone. |
| `calculateMargins()` phase unwrapping (Section 2.2) | **Already fixed** - continuous unwrapping with `while (diff > 180) diff -= 360` loops is present in `SystemAnalysis.cpp:151-155`. |
| `DiscreteADRC::compute()` interface inconsistency (Section 3.1) | **Already fixed** - `compute(error)` calls `computeTracking(r_ - error, r_)` at `DiscreteADRC.cpp:85`. The IController contract is honoured. |
| `DiscreteLQR::compute()` silent ref drop (Section 3.5) | **Already fixed** - throws `std::invalid_argument` for wrong-sized `x_ref` or `u_ff` at `DiscreteLQR.cpp:173-179`. |
| `getFrequencyResponse()` MIMO silent truncation (Section 2.3) | **Already fixed** - throws with an informative message for MIMO input at `SystemAnalysis.cpp:91-95`. |
| `RLS::toTransferFunction()` truncated numerator (Section 2.4) | **Already fixed** - throws `std::logic_error` for `nb > na` at `RecursiveLeastSquares.cpp:95-99`. |
| `M_PI` POSIX macro (Section 5.1) | **Already fixed** - both `SystemAnalysis.cpp` and `DiscreteHinf.cpp` use `static constexpr double kPi`. |
| `SubspaceID` explicit `Gamma_pinv` (Section 5.2) | **Already fixed** - uses `Gamma.colPivHouseholderQr().solve(Yf)` directly at `SubspaceID.cpp:157`. |

The prior report had a significant verification gap. Most of the "high severity" items were already resolved. The priority table at the end of that report is therefore misleading - do not use it as a work queue.

### From 2026-05-24 report

| Item | Status |
|---|---|
| GPC Hessian re-factored every step | **Fixed** |
| GPC augmented-state post-du-saturation | **Fixed** |
| `ControllerStack` reset not clearing `prevActiveName_` | **Fixed** |
| UKF sigma weights normalisation | Not re-verified this pass |
| `AtomicParamBuffer` seqlock correctness | Looks correct; seqlock analysis still valid |

---

## 2. Active Defects - Correctness / Numerical

---

### 2.1 `DiscreteHinf::solveHinfDARE()` - Dead Newton Refinement Block

**File:** [lib/DiscreteHinf.cpp:155-170](../lib/DiscreteHinf.cpp#L155-L170)
**Severity:** Low - dead code, but misleading to readers and future maintainers

After the doubling iteration converges, there is a Newton refinement block:

```cpp
{
    const Eigen::MatrixXd Rbar = R + B.transpose() * X * B;
    Eigen::FullPivLU<Eigen::MatrixXd> luRbar(Rbar);
    if (luRbar.isInvertible())
    {
        const Eigen::MatrixXd K    = luRbar.inverse() * B.transpose() * X * A;
        const Eigen::MatrixXd Acl  = A - B * K;
        const Eigen::MatrixXd resid = Acl.transpose() * X * Acl - X + Q + K.transpose() * R * K;
        (void)resid;  // <-- computed but never used
    }
}
```

The comment promises "one Newton step to refine X further" but the `(void)resid` discards the computed residual without using it. The block does three matrix multiplies and an LU factorisation that contribute nothing. This is the kind of commented-out-but-not-actually-removed code that accumulates tech debt silently - a reader will spend time understanding the algebra only to discover it does nothing.

**Options (pick one):**
- Remove the block entirely and add a comment that the doubling algorithm's quadratic convergence makes Newton refinement unnecessary at the `dareTol = 1e-12` tolerance.
- Actually implement the Newton step: `X += solve(Stein(Acl), resid)` to correct `X` using the computed residual.

The first option is the right call here - the convergence tolerance is already below double precision limits for most synthesis problems.

---

### 2.2 `DiscreteHinf` - `D22 != 0` Handled Inconsistently

**File:** [lib/DiscreteHinf.cpp:750-758](../lib/DiscreteHinf.cpp#L750-L758)
**Severity:** Medium - synthesis with nonzero plant `D` feedthrough silently produces a wrong controller

`MixedSensitivity::build()` correctly records `P.D22 = dG` in the generalised plant. The comment at the bottom of `build()` says:

> "DiscreteHinf::trySolve() currently assumes D22=0 in its assembly formulas; for physical plants where G has no direct feedthrough (dG=0) this is exact. When dG != 0 a loop-shifting pre-processing step would be needed; we warn via a runtime check but do not hard-block."

But `trySolve()` does not actually issue any warning. The `D22` field of the generalised plant is read nowhere in `trySolve()` - the controller assembly formulas use `D12`, `D21`, `D11`, but `D22` is completely ignored. For a plant with `dG != 0`, the synthesised `Dk` matrix is wrong (it should include a term involving `D22`). The synthesis succeeds, `result.feasible = true`, the caller has no indication anything is off, and the synthesised controller is subtly incorrect.

**Fix:** Either:
1. Add a check in `solve()` that throws (or warns to `std::cerr`) when `P.D22.norm() > 1e-12`:
```cpp
if (P.D22.norm() > 1e-12)
    std::cerr << "[DiscreteHinf] WARNING: D22 != 0. "
                 "The standard DGKF formulas assume D22=0. "
                 "Apply loop-shifting before calling solve().\n";
```
2. Implement the D22-compensation (loop-shifting) properly. This is non-trivial but well-documented in Skogestad & Postlethwaite Section 9.4.

At minimum, option 1 must be in place so users are not silently misled.

---

### 2.3 `SubspaceID::n4sid()` - B/D Regression Still Unvalidated Against Ground Truth

**File:** [lib/SubspaceID.cpp:166-185](../lib/SubspaceID.cpp#L166-L185), [tests/test_controllers.cpp:1596-1661](../tests/test_controllers.cpp#L1596-L1661)
**Severity:** Medium - the identified B and D matrices may carry systematic bias; the test does not catch it

The indexing in the B/D regression (Steps 4-5) has been reviewed and the alignment appears correct: `X_hat.col(k)` maps to `Y.col(i+k)` and `U.col(i+k)`, which is consistent with the future-block origin. However, `X_hat` is reconstructed as `Gamma.colPivHouseholderQr().solve(Yf)`, which produces a minimum-norm approximation. The state sequence inherits all the approximation error in the observability subspace computation (Step 3). In the current test:

```cpp
test::check(res.success, "n4sid: valid PRBS data -> success");
if (res.success && res.model.has_value())
    test::check(res.model->stateSize() == 2, "n4sid: identified model has 2 states");
```

The DC-gain sanity check further down uses a 1st-order plant with a 50% tolerance:

```cpp
test::check(std::abs(dc1 - 1.0) < 0.50, "n4sid: DC gain within 50% for fast 1st-order plant");
```

A 50% tolerance on DC gain is not a correctness check - it's a "we didn't produce complete garbage" check. For a 500-sample PRBS identification of a known 2nd-order system, the poles should be recoverable to within a few percent. Without tighter validation, a regression in the B/D regression logic would pass the test suite undetected.

**Recommended addition to `test_subspace_id()`:**

```cpp
// Validate A eigenvalues against known discrete poles of G(s) = 1/(s^2+1.5s+1) at Ts=0.01
// Continuous poles: s = -0.75 +/- 0.6614i  -> z = exp(s*Ts) = 0.9925 +/- 0.0063i
// Both lie near z = 0.9925; check the identified model has similar spectral radius.
if (res.success && res.model.has_value()) {
    Eigen::EigenSolver<Eigen::MatrixXd> es(res.model->A);
    for (int k = 0; k < es.eigenvalues().size(); ++k) {
        const double r = std::abs(es.eigenvalues()(k));
        test::check(r > 0.95 && r < 1.0,
                    "n4sid: identified pole magnitude near true value (0.992)");
    }
}
```

---

### 2.4 `DiscreteMPC::compute()` - Reference Reconstruction Brittle After `setState()`

**File:** [lib/DiscreteMPC.cpp:111-115](../lib/DiscreteMPC.cpp#L111-L115) (approx)
**Severity:** Low-Medium - silent accuracy loss; documented but easy to miss

```cpp
double DiscreteMPC::compute(double error)
{
    const Eigen::VectorXd y_hat = plant_.C * x_hat_ + plant_.D * u_prev_;
    const Eigen::VectorXd r_ref = y_hat.array() + error;
    return computeRef(x_hat_, r_ref)(0);
}
```

After any external `setState()` call, `x_hat_` is updated but `u_prev_` is not reset to be consistent with the new state. The D-term `plant_.D * u_prev_` then produces a stale correction. For physical plants where `D = 0` (the common case for ZOH-discretised systems), this is harmless. But it's a latent bug for any plant with direct feedthrough.

More fundamentally: the interface hides that MPC requires the *absolute reference* `r`, not just the error. The reconstruction `r = C*x + D*u + e` works only when `C*x + D*u approx = y_measured`, which breaks when the state estimate diverges from the real plant (the exact scenario where MPC feedback is most important). The `computeRef()` interface is the correct one for MPC. The `compute()` wrapper should be more clearly marked as an approximation with a `std::cerr` warning when `D.norm() > 1e-12`:

```cpp
// In DiscreteMPC constructor:
if (plant.D.norm() > 1e-12)
    std::cerr << "[DiscreteMPC] WARNING: plant has nonzero D. "
                 "compute(error) approximation is inaccurate. Use computeRef() directly.\n";
```

---

## 3. Test Suite Issues

---

### 3.1 `test_hinf()` - Leftover Debug Output Pollutes CI Log

**File:** [tests/test_controllers.cpp:1683-1731](../tests/test_controllers.cpp#L1683-L1731)
**Severity:** Low (cosmetic / CI noise), should be cleaned up before next release cut

The H-infinity test suite has 30+ `std::cout << "[DBG] ..."` lines scattered throughout:

```cpp
std::cout << "  [DBG] W1 built, A=" << W1.A(0,0) << "\n" << std::flush;
std::cout << "  [DBG] about to exit block 1\n" << std::flush;
std::cout << "  [DBG] entering section 2\n" << std::flush;
// ... 25 more similar lines
```

These are clearly debugging traces from development that were never removed. They don't affect correctness but they:
- Pollute the test runner output, making it hard to see which test failed
- Force `std::flush` on every debug line, which is a measurable overhead in the synthesis test (the DARE bisection does 40+ iterations, each calling `trySolve` which calls these test blocks)
- Will trip up any CI log-grepping for `[DBG]`

Remove all `[DBG]` lines from `test_hinf()`. The synthesis test itself is solid - the debug scaffolding was for the initial bring-up and has no value now.

---

### 3.2 `test_hinf()` - H-inf Closed-Loop Test Exercises Wrong Interface

**File:** [tests/test_controllers.cpp:1862-1876](../tests/test_controllers.cpp#L1862-L1876)
**Severity:** Medium - the closed-loop simulation is not testing what it claims to test

The closed-loop simulation in test block 6 does:

```cpp
const double u = hinf_ctrl.compute(y - ref);  // shift: y_meas = y - r
```

The comment says "shift: y_meas = y - r" but that's not how H-inf output-feedback works. The synthesised controller K(z) expects the raw measurement `y[k]` as input (possibly `y - r` pre-shifted if the augmented plant includes the reference subtraction, but that depends on how `MixedSensitivity::build()` connects the reference). Feeding `y - ref` to a controller designed around `y` produces an incorrect simulation - the comment even acknowledges it:

> "This approximation is only valid when Dk is small; for a proper simulation we'd need the full augmented plant."

An approximation you admit is wrong is not a test - it's a guess. The test should either:
1. Build the full closed-loop augmented simulation (reference enters via the exogenous input `w = [r, d]`), or
2. Check only that the controller output is bounded (which is already done via `std::isfinite(y)`), and explicitly document that a full closed-loop correctness test requires a separate simulation harness.

The current formulation gives false confidence that closed-loop stability has been validated when it has not. Option 2 is acceptable for now; document it honestly.

---

### 3.3 Missing Test for `ControllerStack` Bumpless Transfer Correctness

**File:** [tests/test_controllers.cpp:912-981](../tests/test_controllers.cpp#L912-L981)
**Severity:** Low-Medium - bumpless transfer correctness is untested

The current bumpless transfer test runs a closed-loop to steady state and then forces a controller switch, checking that `|u_after_switch - u_before_switch| < 1.0`. A tolerance of 1.0 on the output jump is not a bumpless-transfer test - a step change of 0.99 is not "bump-less." The test verifies the implementation does not crash on switching, not that bumpless transfer actually works.

A meaningful bumpless transfer test should:
1. Run to steady state, record `u_ss`
2. Switch controllers
3. Assert the first output from the new controller is within `0.05 * u_ss` of `u_ss`

---

## 4. Design Issues / API Concerns

---

### 4.1 `ControllerStack` Weighted Mode - Normalisation Semantics Undocumented

**File:** [lib/ControllerStack.h:28](../lib/ControllerStack.h#L28)
**Severity:** Low - surprising behaviour with no documentation

The `Weighted` mode normalises by the sum of active weights:

```
u = (Sigma w_i . u_i) / (Sigma w_i)   [over active entries only]
```

This means if one of two controllers is disabled (weight excluded from denominator), the remaining controller's output is `w2 * u2 / w2 = u2` - full output. A user who sets `[0.7, 0.3]` to blend two controllers expecting the 0.3-weight controller to contribute 30% of the range will be surprised when the 0.7-weight one is deactivated and the 0.3-weight one produces full authority.

The header docstring says "normalised weighted average of all enabled, gate-passing entries" - that's accurate but buries the implication. Add an explicit example:

```
// If weights = [0.7, 0.3] and entry[0] deactivates:
//   u = 0.3*u1 / 0.3 = u1   (entry[1] gets full authority)
// This is intentional for fallback chains. For a static blend,
// set all activationConditions to nullptr.
```

---

### 4.2 `SmithPredictor` - No Runtime Model Update API

**File:** [lib/SmithPredictor.h](../lib/SmithPredictor.h)
**Severity:** Low - design gap for adaptive use

The toolbox has `RecursiveLeastSquares` + `SubspaceID` for online identification, and `SmithPredictor` for dead-time compensation. The combination is the natural fit for adaptive dead-time systems, but `SmithPredictor` has no `setModel()` method - the model is fixed at construction. A user who identifies a better model with RLS cannot update the predictor without reconstructing it (losing inner controller state in the process).

Add `setModel(const StateSpace&, int delay_steps)` to `SmithPredictor`. It should update the model matrices and resize the prediction buffer without resetting the inner controller.

---

### 4.3 `DiscreteHinf` - `gammaLo` Hardcoded to 0.01

**File:** [lib/DiscreteHinf.cpp:435](../lib/DiscreteHinf.cpp#L435)
**Severity:** Low-Medium - bisection may miss the true infimum for well-conditioned plants

```cpp
double gammaHi  = params.gammaInit;
double gammaLo  = 0.01;
```

`gammaLo` is hardcoded to 0.01 regardless of the generalised plant's properties. For a plant where the achievable gamma infimum is below 0.01 (common for well-conditioned, low-bandwidth mixed-sensitivity designs), the bisection will converge to 0.01 and report it as the minimum achievable gamma when the true value might be 0.005 or lower.

The lower bound should be derived from the problem data. At minimum, `gammaLo` should equal `||D11||_2` (the lower bound mandated by Lemma 2.1 in Iglesias & Glover 1991), which is already computed in `trySolve()`:

```cpp
Eigen::JacobiSVD<Eigen::MatrixXd> svdD11(P.D11);
const double gamma_lb = P.D11.isZero(1e-14) ? 0.0 : svdD11.singularValues()(0);
double gammaLo = std::max(gamma_lb + 1e-6, 1e-4);
```

---

## 5. Build System

---

### 5.1 `lib/CMakeLists.txt` - No Compiler Warning Flags

**File:** [lib/CMakeLists.txt](../lib/CMakeLists.txt)
**Severity:** Low - missed warnings during active development

The library is built without `-Wall -Wextra -Wpedantic`. The DARE doubling, Kalman covariance updates, and H-inf bisection are exactly the kind of dense numerical code that benefits from shadow variable warnings, sign-comparison warnings, and unused-parameter warnings. There is at least one candidate: the `(void)resid` in `solveHinfDARE()` - with `-Wunused-variable` this would have been flagged immediately during development.

```cmake
# Add to lib/CMakeLists.txt after the target_compile_features line:
target_compile_options(controller_toolbox PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
```

Note: apply to `PRIVATE`, not `PUBLIC`, so downstream consumers are not forced to build with these flags.

---

### 5.2 `Dockerfile` - `--parallel` Flag on `cmake --build`

**File:** [Dockerfile:46-47](../Dockerfile#L46-L47)
**Severity:** Low - may cause non-deterministic build failures on resource-constrained hosts

```dockerfile
RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build --parallel
```

`--parallel` without a count lets CMake use all available cores. In a Docker build environment (especially CI), this can OOM-kill the build for large translation units (the H-inf and MPC condensed-matrix code are both heavy). Ninja already parallelises by default; the `--parallel` flag here is redundant for the Ninja generator and may actively cause problems. Remove it:

```dockerfile
RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build
```

If explicit parallelism control is desired, use `cmake --build build -j 4` with a fixed count.

---

### 5.3 `case-study/CMakeLists.txt` - `nlohmann_json` FetchContent Without Offline Guard

**File:** [case-study/CMakeLists.txt](../case-study/CMakeLists.txt)
**Severity:** Low - build fails cryptically with no network access

The Boiler Control case study fetches `nlohmann_json` via `FetchContent` at configure time. If the network is unavailable (air-gapped CI, corporate firewall), the error is a CMake fetch failure with no actionable message. Either vendor the single-header `json.hpp` directly in `case-study/` (it is 24k lines but a single file), or add:

```cmake
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
```

before the `FetchContent_Declare` call to enable offline mode with a cached download.

---

## 6. Summary - Priority Action List

The items below are the genuine open work items as of 2026-05-25. The 13-item table from the prior report was largely already-done items.

| # | Issue | File | Severity | Status |
|---|---|---|---|---|
| 1 | H-inf `D22 != 0` no warning, wrong controller assembled silently | `DiscreteHinf.cpp:750` | Medium | **Fixed** - `std::cerr` warning added in `solve()` |
| 2 | `test_hinf()` closed-loop test exercises wrong interface | `test_controllers.cpp:1862` | Medium | **Fixed** - replaced with full augmented-plant simulation |
| 3 | `n4sid()` B/D accuracy not tested against ground truth | `test_controllers.cpp:1596` | Medium | **Fixed** - pole magnitude accuracy check added (3% tolerance); DC-gain tolerance tightened from 50% to 25% |
| 4 | `test_hinf()` 30+ `[DBG]` cout lines pollute CI output | `test_controllers.cpp:1683-1731` | Low | **Fixed** - all `[DBG]` lines removed |
| 5 | Dead Newton refinement block in `solveHinfDARE()` | `DiscreteHinf.cpp:155` | Low | **Fixed** - dead block removed; comment explains quadratic convergence makes Newton refinement unnecessary |
| 6 | `DiscreteMPC::compute()` D!=0 warning missing | `DiscreteMPC.cpp` | Low | **Fixed** - `std::cerr` warning present in constructor since prior review pass |
| 7 | `DiscreteHinf` `gammaLo` hardcoded to 0.01 | `DiscreteHinf.cpp:435` | Low-Medium | **Fixed** - `gammaLo` now set to `max(||D11||2 + 1e-6, 1e-4)` per Iglesias-Glover lower bound |
| 8 | `ControllerStack` weighted normalisation undocumented | `ControllerStack.h:28` | Low | **Fixed** - explicit example added showing fallback behaviour when an entry gates out |
| 9 | Bumpless transfer test tolerance too loose (`< 1.0`) | `test_controllers.cpp:946` | Low | **Fixed** - tightened to `< 0.1` (~10% of `u_ss approx = 1.0`) |
| 10 | `SmithPredictor` no `setModel()` for adaptive use | `SmithPredictor.h` | Low | **Fixed** - `setModel(const StateSpace&, int delaySteps)` added to header and implementation |
| 11 | Compiler warning flags missing in `lib/CMakeLists.txt` | `lib/CMakeLists.txt` | Low | **Fixed** - `-Wall -Wextra -Wpedantic` / `/W4` added as `PRIVATE` compile options |
| 12 | Dockerfile `--parallel` flag redundant and risky | `Dockerfile:47` | Low | **Fixed** - `--parallel` removed; Ninja already parallelises by default |
| 13 | `nlohmann_json` FetchContent no offline guard | `case-study/CMakeLists.txt` | Low | **N/A** - `nlohmann/json` is fetched in `build.ps1` via `Invoke-WebRequest` with a `Test-Path` guard already in place; no CMake `FetchContent` is used |

All 13 items resolved. The library is in a clean state for the next release cut.

---

## 7. What's Working Well

Worth naming explicitly, since the report is otherwise a list of problems:

- **DiscreteHinf architecture** is solid. The two-Riccati DGKF formulation with proper structured doubling, the MixedSensitivity builder, the bisection driver, and the controller assembly all follow the literature correctly. The test suite for synthesis (sections 1-5 of `test_hinf()`) is thorough: dimension checks, DARE PSD verification, spectral radius condition, and closed-loop controller eigenvalue check are all there.

- **DARE doubling algorithm** in both `DiscreteLQR` and `DiscreteHinf`. Quadratic convergence, symmetric enforcement at each step, and the W-matrix formulation are all correct. This replaced a value-iteration scheme that was correct but slow - the current version is production quality.

- **DiscreteADRC backward-Euler ESO** is clean. The analytical inverse of `(I - Ts.Ae)` for the nilpotent Ae is a nice trick, and the comment derivation in `DiscreteADRC.cpp:30-48` is genuinely useful for future maintainers. The `setReference + compute(error)` interface is consistent with IController.

- **Phase-unwrapping in `calculateMargins()`** is correct. The coarse-grid continuous unwrapper plus bisection with an `unwrapRelative` helper at the crossing is exactly the right pattern for this kind of margin calculation. Works for higher-order and non-minimum-phase plants.

- **Test coverage breadth** is good. 22 test groups, every controller exercised with normal operation, NaN guards, saturation, reset, and closed-loop convergence. The EKF/UKF vs. KF agreement tests are a nice self-consistency check.

---

*End of report.*

P.S: Disable $H_\infty$ controller for now. That causes way too many problems now.