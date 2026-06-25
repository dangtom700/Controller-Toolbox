# Design: Correlation-Based Identification, Nelder-Mead Simplex, and Generalized SK Fitting

**Date:** 2026-06-24
**Status:** Approved, not yet implemented

## Motivation

`docs/ALGORITHM_ROADMAP_PHASE3.md`'s Phase 1 sequences SI2 (Correlation-Based Identification),
MO2 (Nelder-Mead Simplex), and FD1 (Generalized SK Iteration) first, as the three smallest/
lowest-risk items in the entire 32-item roadmap (~150-200 lines each). They are bundled into one
spec purely for that reason — independent subject matter, independent of each other, but sharing
no design content worth three separate docs: each is a free-function or header-only utility, none
is an `IController`, and each drops into (or alongside) an existing contract with at most a small,
well-scoped extension.

## Scope

- **SI2** is non-parametric, classical impulse-response estimation — a sanity-check step before
  committing to a parametric ARX/state-space structure. Single-input/single-output only; no
  MIMO cross-correlation.
- **MO2** is unconstrained direct search (no box bounds in this version) — `AutoTuner`/
  `GeneticAlgorithm`/etc. already cover the box-constrained case; Nelder-Mead's whole value
  proposition here is "give it one point, no bounds, no population sizing."
- **FD1** generalizes `FreqDomainIdentifier::fitLevy` to iteratively reweight against the
  complex response (Sanathanan-Koerner style), not a from-scratch fitting algorithm. It reuses
  Levy's linear-system shape; the denominator-order convention follows `fitLevy`'s
  `(num_order, den_order)` pair rather than the roadmap sketch's single `n_poles` (see Components
  below) — `fitSK` is extending `fitLevy`'s general-TransferFunction representation, not
  `VectorFitting`'s strictly-proper pole/residue representation, so it needs both orders.

## Components

### 1. `lib/CorrelationID.h` / `.cpp` — standalone, no shared base

Classical non-parametric impulse-response estimation via cross-correlation:
`g_hat(k) = R_uy(k) / R_uu(0)`, accurate when `u` is near-white (PRBS is the standard choice).

```cpp
struct CorrelationIDParams {
    int  max_lag       = 50;     // impulse response length to estimate
    bool whiten_input  = false;  // optional input pre-whitening (documented limitation if off
                                  // and the input isn't white - see Testing plan item 3)
};

struct CorrelationIDResult {
    Eigen::VectorXd impulse_response;  // g_hat(0..max_lag)
    Eigen::VectorXd autocorr_u;        // R_uu(0..max_lag)
    Eigen::VectorXd crosscorr_uy;      // R_uy(0..max_lag)
};

class CorrelationID {
public:
    static CorrelationIDResult identify(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                                         double Ts, const CorrelationIDParams &params = {});
    static Eigen::VectorXd generatePRBS(int length, int n_bits, unsigned seed = 42);
};
```

`generatePRBS` produces a maximal-length LFSR-based pseudo-random binary sequence (period
`2^n_bits - 1`, values in `{-1, +1}`) — the standard near-white test input for `identify()`.
`identify()` throws `std::invalid_argument` if `u`/`y` differ in length or `max_lag >= u.size()`.

No reuse needed or claimed — confirmed nothing in `lib/` already does cross-correlation impulse-
response estimation; this is intentionally the simplest item in the roadmap.

### 2. `lib/NelderMead.h` — header-only, no shared base

```cpp
struct NelderMeadParams {
    int    n_dim;
    int    max_iter = 500;
    double tol       = 1e-8;       // simplex-spread convergence
    double alpha = 1.0, gamma = 2.0, rho = 0.5, sigma = 0.5; // reflect/expand/contract/shrink
    unsigned seed = 42;             // initial-simplex perturbation
};

class NelderMead {
public:
    using CostFn = AutoTuner::CostFn;   // shared contract, zero new types
    explicit NelderMead(const NelderMeadParams &p);
    TunerResult optimize(const CostFn &cost, const Eigen::VectorXd &x0);
};
```

Standard Nelder-Mead-with-restart: build an initial simplex around `x0` (`x0` plus `n_dim` unit
perturbations scaled by 5% of `|x0|` or a small absolute floor when `x0` has zero components),
then reflect/expand/contract/shrink each iteration. On simplex-collapse detection (max edge
length below a numerical floor before `tol`/`max_iter` is reached), restart once from the best
vertex with a fresh perturbed simplex rather than silently returning a degenerate result (roadmap
test plan item 3).

Reuses `AutoTuner::CostFn`/`TunerResult` (`AutoTuner.h:78-97`, confirmed public) directly, the
same way `GeneticAlgorithm`/`ParticleSwarmOptimizer`/`DifferentialEvolution` already do
(`using CostFn = AutoTuner::CostFn;`, header-only, no `.cpp`, no `CTRL_CORE_SOURCES` entry) — the
cleanest reuse claim in Phase 1, confirmed with zero modification needed to `AutoTuner.h`.

### 3. `lib/SKFit.h` / `.cpp` — standalone, no shared base; also extends `FreqDomainIdentifier.h`

```cpp
struct SKFitResult {
    TransferFunction     model;
    std::vector<double>  iterCost;   // RMSE per SK iteration, for convergence diagnostics
    bool                 converged;
};

class SKFit {
public:
    // num_order/den_order match FreqDomainIdentifier::fitLevy's convention (D's constant
    // term fixed to 1) - SKFit is an iteratively-reweighted Levy fit, not a pole/residue fit.
    static SKFitResult fitSK(const std::vector<double> &freqs,
                              const std::vector<std::complex<double>> &response,
                              int num_order, int den_order, double Ts,
                              int max_iter = 20, double tol = 1e-4);
};
```

**Reuse decision (resolves the roadmap's overstated reuse claim):** `FreqDomainIdentifier::
fitLevy` (`FreqDomainIdentifier.cpp:8-95`) is a single monolithic function — it builds the real-
stacked linear system, solves, and computes RMSE all inline, with no row-weighting parameter and
no separable sub-step to call into. Rather than duplicating ~40 lines of that linear-algebra
block into `SKFit.cpp`, **`FreqDomainIdentifier` gets a small additive refactor**: extract the
system-build step into a new public static helper with an optional per-sample weight vector
(empty = unweighted, i.e. today's behavior exactly):

```cpp
// Added to FreqDomainIdentifier (FreqDomainIdentifier.h/.cpp) - no behavior change for
// existing fitLevy callers (weights defaults to empty = all-ones).
static void buildLevySystem(const std::vector<double> &freqs,
                             const std::vector<std::complex<double>> &response,
                             int num_order, int den_order, double Ts,
                             const std::vector<double> &weights,   // empty = unweighted
                             Eigen::MatrixXd &Phi, Eigen::VectorXd &y);
```

`fitLevy` itself is rewritten to call `buildLevySystem(..., {}, Phi, y)` then solve/score exactly
as today (pure refactor, regression-tested against the existing `[freq_domain_id]`
suite to confirm zero behavior change). `SKFit::fitSK` then:
1. Iteration 0: call `buildLevySystem` unweighted, solve via QR — identical to a plain `fitLevy`
   call (this is the documented "already-good fitLevy result" baseline for test plan item 3).
2. Each subsequent iteration: evaluate the previous iteration's fitted denominator
   `D_prev(zinv_k)` at every sample frequency, set `weight_k = 1 / max(|D_prev(zinv_k)|, eps_d)`
   (`eps_d = 1e-9`, guards near-zero denominator magnitude), call `buildLevySystem` with those
   weights, solve, record RMSE into `iterCost`.
3. Stop at `max_iter` or when the coefficient vector's change is below `tol`; `converged` reflects
   which.

This is the classical Sanathanan-Koerner reweighting (minimizes `|N_k - H_k*D_k|^2 /
|D_prev,k|^2` by weighting the *linear* residual rows by `1/|D_prev,k|`, which squares correctly
inside the least-squares solve) applied to Levy's general numerator/denominator representation
instead of `VectorFitting`'s real-pole/magnitude-only one. `VectorFitting::buildSKSystem`
(`VectorFitting.h:99-105`) is confirmed `private` and real-pole/magnitude-specific — not reused,
only its convergence-check *pattern* (max-iter/tol on coefficient displacement) is mirrored.

## Explicitly out of scope (this phase)

- **MO2 box-bounds / population-hybrid variants** — Nelder-Mead's value here is specifically
  "no bounds, no population sizing needed"; a bounded variant would just be `AutoTuner` again.
- **MO2 simplex-size adaptive restart strategies beyond the single restart-on-collapse** — a
  single restart from the best vertex is sufficient to satisfy the roadmap's test plan; more
  elaborate restart heuristics are not needed for this use case (quick 2-3 parameter retunes).
- **SI2 MIMO / multivariable correlation** — SISO only, matching the roadmap's scope exactly.
- **SI2 frequency-domain variant of correlation ID** — roadmap explicitly flags this as a
  possible v2 extension reusing `FreqDomainIdentifier`'s FFT utilities; not built now.
- **FD1 complex-conjugate-pole bookkeeping** — that is FD2 (Phase 3), a materially bigger lift;
  `fitSK` keeps Levy's general-coefficient representation (poles can come out anywhere, including
  unstable, exactly like `fitLevy` today — no stability enforcement is added by SK reweighting).

## Implementation checklist

**CorrelationID** (lighter non-`IController` utility-class checklist):
1. `lib/CorrelationID.h`/`.cpp` + `CTRL_REGISTER_FEATURE(correlation_id)`
2. `lib/CMakeLists.txt` — add `CorrelationID.cpp` to `CTRL_CORE_SOURCES`
3. `lib/ControllerToolbox.h` — add `#include "CorrelationID.h"`
4. `bindings/estimation_bindings.cpp` — bind `CorrelationIDParams`/`CorrelationIDResult` as
   plain structs, `identify`/`generatePRBS` as static methods
5. `bindings/smoke_test.py` — `generate_prbs()` + `identify()` smoke assertion
6. `examples/ex92_correlation_id.cpp` + `examples/python/ex109_correlation_id.py`
7. `tests/test_catch2_advanced.cpp` — tests under `[correlation_id]`
8. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` — add `ex92_correlation_id`

**NelderMead** (header-only optimizer checklist, mirrors `GeneticAlgorithm.h`):
1. `lib/NelderMead.h` + `CTRL_REGISTER_FEATURE(nelder_mead)` (no `.cpp`, no `CTRL_CORE_SOURCES`
   entry — confirmed `GeneticAlgorithm`/`ParticleSwarmOptimizer`/`DifferentialEvolution` are all
   header-only with no `CTRL_CORE_SOURCES` entry)
2. `lib/ControllerToolbox.h` — add `#include "NelderMead.h"` near the other optimizers
3. `bindings/controllers_bindings.cpp` — bind alongside `GeneticAlgorithm` (confirmed binding
   home at `controllers_bindings.cpp`, using the `py::object cost_py` lambda-wrap pattern for
   `CostFn`, per `CONTRIBUTING.md`'s binding rule for `std::function` parameters)
4. `bindings/smoke_test.py` — Rosenbrock-style smoke assertion
5. `examples/ex93_nelder_mead.cpp` + `examples/python/ex110_nelder_mead.py`
6. `tests/test_catch2_advanced.cpp` — tests under `[nelder_mead]`
7. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` — add `ex93_nelder_mead`

**SKFit** (lighter non-`IController` utility-class checklist, plus the `FreqDomainIdentifier`
refactor):
1. `lib/FreqDomainIdentifier.h`/`.cpp` — add `buildLevySystem`, rewrite `fitLevy` to call it
   (pure refactor; re-run existing `[frequency_domain_identification]` tests to confirm zero
   behavior change before proceeding)
2. `lib/SKFit.h`/`.cpp` + `CTRL_REGISTER_FEATURE(sk_fit)`
3. `lib/CMakeLists.txt` — add `SKFit.cpp` to `CTRL_CORE_SOURCES`
4. `lib/ControllerToolbox.h` — add `#include "SKFit.h"` near `FreqDomainIdentifier.h`
5. `bindings/estimation_bindings.cpp` — bind alongside `FreqDomainIdentifier`
6. `bindings/smoke_test.py` — smoke assertion
7. `examples/ex94_sk_complex_fit.cpp` + `examples/python/ex111_sk_complex_fit.py`
8. `tests/test_catch2_advanced.cpp` — tests under `[sk_complex_fit]`
9. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` — add `ex94_sk_complex_fit`

## Testing plan

**`[correlation_id]`**
1. Known linear system driven by PRBS — recovered impulse response matches the analytic one
   within the noise floor.
2. `generatePRBS` — verified near-white autocorrelation (single dominant peak at lag 0, low
   sidelobes at nonzero lags).
3. Colored (non-PRBS) input without whitening — result is visibly biased vs. the analytic
   impulse response (a regression test documenting the known limitation, not a bug).
4. Mismatched `u`/`y` lengths or `max_lag >= u.size()` — throws `std::invalid_argument`.

**`[nelder_mead]`**
1. Rosenbrock function (2D) — converges to the known minimum `(1,1)` within tolerance.
2. Quadratic bowl — converges in fewer evaluations than `AutoTuner`/CMA-ES on the same problem
   (documents the "why use this" case for cheap low-dimensional problems).
3. Degenerate simplex collapse (e.g. a flat-cost region) — detected and restarted once, returns
   a valid (non-degenerate) result rather than a collapsed point.

**`[sk_complex_fit]`**
1. Synthetic complex response with known poles, lightly damped — SK-reweighted fit has lower
   RMSE than a one-shot `fitLevy` call on the same data.
2. Convergence — `iterCost` is non-increasing iteration-over-iteration (allow equal, not strictly
   decreasing, since a converged fit can plateau).
3. Already-good `fitLevy` result (low-order, low-damping system) — SK iteration's final RMSE is
   not worse than iteration 0's (the unweighted baseline), confirming SK doesn't regress easy
   cases.
4. Regression: `buildLevySystem({}, ...)` (unweighted) reproduces `fitLevy`'s pre-refactor
   numerical result exactly on the existing `[freq_domain_id]` test fixtures.
