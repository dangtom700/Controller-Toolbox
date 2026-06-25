# Design: H-Infinity Filter

**Date:** 2026-06-24
**Status:** Approved, not yet implemented

## Motivation

`docs/algorithm_backlog.md`'s Advanced Estimation & Filtering category flags this gap directly:
`DiscreteHinf` today is a *controller* only — there is no Hinf-optimal *filter* anywhere in
`lib/`, despite the dual two-Riccati DGKF machinery already existing for the control side. An
Hinf filter bounds the worst-case ratio of estimation-error energy to disturbance/noise energy
for *any* bounded disturbance, instead of assuming Gaussian noise the way `KalmanFilter` does —
valuable when noise is bounded-but-non-Gaussian (e.g. impact disturbances on a vibration sensor)
and a guaranteed worst-case bound matters more than an average-case optimum.

This is the highest *technical*-risk item in Phase 1's first batch (after RC1): the roadmap's
"reuse `DiscreteLQR::solveDARE`" claim is wrong (that solver assumes positive-definite `R`, the
Kalman/H2 case — the H-infinity filter's Riccati equation has an *indefinite* `R`, like the
control side). The right reusable primitive is `DiscreteHinf::solveHinfDARE`
(`DiscreteHinf.h:408-412`), confirmed `private static`, plus a feasibility/bisection condition
that must be derived independently since the filter-only problem has no DGKF coupling condition.

## Scope

- SISO and MIMO state estimation (no restriction on `ny`/`nu` channel counts — `solveHinfDARE`
  is dimension-general).
- Steady-state (infinite-horizon, fixed-gain) design, matching `DiscreteHinf`'s own pattern: one
  `solve()` call produces a fixed filter gain `L`, then `predict()`/`update()` apply it every
  step with no per-step Riccati recursion at runtime (cheaper, real-time-safe, and the natural
  reading of the roadmap's `HinfFilterResult` holding a fixed `L`/`P` rather than time-varying
  ones).
- Estimates the **full state** `x` (the performance/estimation-error channel `z = x`, i.e. `L =
  I` in the estimation-error weighting). Estimating an arbitrary linear combination `z = L*x`
  (`L != I`) is a straightforward generalization but not needed by any current case study —
  deferred (see Out of scope).
- Process/measurement noise are modeled as **independent** bounded disturbances entering through
  separate `Qw`/`Rv` weighting matrices, mirroring `KalmanFilter`'s `(plant, Q_noise, R_noise)`
  constructor shape exactly (the roadmap's own `solve(plant, Qw, Rv, params)` signature already
  implies this). Correlated process/measurement noise (a nonzero `D` cross-term in the bordered
  Riccati) is out of scope — `KalmanFilter` doesn't support it either, so this is parity, not a
  regression.

## Components

### `lib/HinfFilter.h` / `.cpp` — standalone estimator, no shared base (mirrors `KalmanFilter`'s API shape)

```cpp
struct HinfFilterParams {
    double gammaInit = 10.0;
    double gammaTol   = 1e-3;
    int    maxIter    = 60;     // bisection iterations, matches HinfParams::maxIter
    double dareTol    = 1e-12;  // forwarded to solveHinfDARE's residual check
    int    dareMaxIter = 200;
};

struct HinfFilterResult {
    bool   feasible      = false;
    double achievedGamma = 0.0;
    Eigen::MatrixXd L;      // filter gain (n x ny)
    Eigen::MatrixXd P;      // bordered Riccati solution Y_inf (n x n)
    double Ts = 0.0;
};

class HinfFilter {
public:
    explicit HinfFilter(const HinfFilterResult &result);   // throws if !result.feasible

    // plant: (A, B, C, D, Ts) - same StateSpace used everywhere else in lib/.
    // Qw: process-noise weighting (n x n, PSD).  Rv: measurement-noise weighting (ny x ny, PD).
    static HinfFilterResult solve(const StateSpace &plant,
                                   const Eigen::MatrixXd &Qw, const Eigen::MatrixXd &Rv,
                                   const HinfFilterParams &params = {});

    void predict(const Eigen::VectorXd &u);   // x_ = A*x_ + B*u
    void update(const Eigen::VectorXd &y);    // x_ += L*(y - C*x_)
    const Eigen::VectorXd &state() const;
    void reset();                              // x_ = 0
};
```

**Mathematical formulation (Simon, "Optimal State Estimation," 2006, Ch. 11 — the steady-state
bordered-Riccati H-infinity predictor):**

Estimating the full state (`z = x`) adds a *fictitious* second measurement channel `I*x` to the
real measurement `y = C*x + v`, weighted with a **negative** `gamma^2` block — exactly the same
indefinite-`R` trick `DiscreteHinf`'s control DARE already uses (`gamma2*[I 0; 0 -I]`,
`DiscreteHinf.cpp:215`), just on the filter side instead of the control side:

```
Cbar = [C; I_n]                          // (ny+n) x n
Rbar = blockdiag(Rv, -gamma^2 * I_n)      // (ny+n) x (ny+n), indefinite by construction

Bordered Riccati:  P = A P A' + Qw - A P Cbar' (Rbar + Cbar P Cbar')^-1 Cbar P A'
```

This is **exactly** the equation `solveHinfDARE(Aa, Ba, Qa, Ra)` solves (`Aa'XAa - X -
Aa'XBa(Ra+Ba'XBa)^-1 Ba'XAa + Qa = 0`, i.e. `X = Aa'XAa + Qa - Aa'XBa(...)Ba'XAa`) once `Aa = A'`,
`Ba = Cbar'` are substituted (so `Aa' = A`, `Ba' = Cbar`):

```cpp
DareResult dr = DiscreteHinf::solveHinfDARE(A.transpose(), Cbar.transpose(), Qw, Rbar,
                                             params.dareTol, params.dareMaxIter);
// dr.P is Y_inf == P above.
```

**Feasibility condition** (the filter-only analogue of DGKF's `rho(X_inf*Y_inf) < gamma^2`
coupling condition — derived independently since there's no control-side Riccati to couple
against here): `dr.converged` (DARE residual check, reused as-is from `solveHinfDARE`) **and**
`Y_inf` positive semi-definite **and** `(I - gamma^-2 * Y_inf)` positive definite (equivalently:
spectral radius of `Y_inf` strictly less than `gamma^2`) — this is the standard H-infinity
filtering feasibility bound (Simon Ch. 11, Theorem 11.1: the recursion is well-posed iff `P_k -
gamma^2 I` stays negative definite at every step; for the steady-state solution this collapses to
a single eigenvalue check on `Y_inf`).

**Filter gain** (uses only the *real* measurement channel `C`/`Rv` — the fictitious `I*x`
channel only shapes the Riccati recursion, it is never an observable signal, matching Simon's
derivation):

```
L_inf = A * Y_inf * C' * (Rv + C * Y_inf * C')^-1
```

`predict()`/`update()` then apply the fixed `(A, B, C, L_inf)` every step exactly like
`KalmanFilter::predict()`/`update()`'s method shape, but with no covariance recursion at runtime
(steady-state gain baked in at `solve()` time) — consistent with `DiscreteHinf`'s own
solve-once-then-fixed-gain-runtime pattern.

**Bisection driver** mirrors `DiscreteHinf::solve()`'s structure exactly
(`DiscreteHinf.cpp:478-531`): `gammaLo = 1e-4` (no `D11`-based lower bound exists here — there is
no performance feedthrough term in this simpler problem), `gammaHi = params.gammaInit` (doubled
up to 10x if infeasible at the initial value), then standard bisection on `[gammaLo, gammaHi]`
calling a `trySolve(plant, Qw, Rv, gamma, dareTol, dareMaxIter, out)`-shaped helper (the filter
analogue of `DiscreteHinf::trySolve`) until the bracket narrows below `gammaTol` or `maxIter` is
exhausted.

**Required change to `DiscreteHinf.h`/`.cpp`:** move the `solveHinfDARE` declaration
(`DiscreteHinf.h:408-412`) from the `private:` section to `public:` (pure access-specifier
change, zero behavior change, zero signature change) so `HinfFilter::solve()` can call it. This
is the same precedent already set when `DiscreteLQR::solveDARE` was made public specifically so
`DiscreteH2` could reuse it — confirmed in `DiscreteLQR.h`'s own doc comment justifying that
exact move.

## Explicitly out of scope (this phase)

- **Time-varying / finite-horizon recursive Riccati** — only the steady-state fixed-gain design
  is built, matching `DiscreteHinf`'s own scope; a finite-horizon variant (recomputing `P`/`L`
  every step) would need a from-scratch recursive driver and isn't needed by any current
  use case.
- **Arbitrary estimated output `z = L*x` (`L != I`)** — only direct full-state estimation
  (`L = I`) is built; generalizing `Cbar`'s second block from `I_n` to an arbitrary `L` is a
  small, well-understood extension if a future case study needs it.
- **Correlated process/measurement noise** — `Qw`/`Rv` are independent block-diagonal weights,
  parity with `KalmanFilter`'s own constructor shape, not a new restriction.
- **Mu-synthesis / DK-iteration analogue for filtering** — `DiscreteHinf::solveMuSyn` has no
  filtering counterpart in this phase; out of scope.

## Implementation checklist

(Lighter non-`IController` estimator checklist, same shape as `KalmanFilter`/`ParticleFilter`.)

1. `lib/DiscreteHinf.h` — move `solveHinfDARE`'s declaration from `private:` to `public:` (no
   `.cpp` change needed beyond the declaration's access specifier). Re-run the existing
   `DiscreteHinf`/Hinf-tagged Catch2 tests to confirm zero behavior change from this move alone.
2. `lib/HinfFilter.h`/`.cpp` + `CTRL_REGISTER_FEATURE(hinf_filter)`
3. `lib/CMakeLists.txt` — add `HinfFilter.cpp` to `CTRL_CORE_SOURCES`
4. `lib/ControllerToolbox.h` — add `#include "HinfFilter.h"` near `DiscreteHinf.h`/`DiscreteH2.h`
   (inside the `CTRL_ENABLE_HINF` feature-flag region, since it depends on `DiscreteHinf`'s
   Riccati machinery)
5. `bindings/advanced_bindings.cpp` — bind alongside `DiscreteHinf`/`GeneralisedPlant`
6. `bindings/smoke_test.py` — construct via `solve()`, call `predict()`/`update()`, confirm
   `state()` returns a finite vector
7. `examples/ex95_hinf_filter.cpp` — a vibration-sensor-style scenario: known plant + bounded
   *non-Gaussian* (e.g. uniform-impact) disturbance, comparing `HinfFilter`'s worst-case bound
   against `KalmanFilter`'s (still-stable-but-no-guarantee) estimate on the same data
   + `examples/python/ex112_hinf_filter.py`
8. `tests/test_catch2_advanced.cpp` — tests under `[hinf_filter]`
9. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` — add `ex95_hinf_filter`

## Testing plan

**`[hinf_filter]`**
1. Known plant + simulated bounded adversarial (non-Gaussian, e.g. worst-case impulsive) noise
   sequence — the actual estimation-error-energy-to-disturbance-energy ratio over the simulation
   stays below `achievedGamma`, confirming the worst-case bound holds empirically.
2. Comparison against `KalmanFilter` on Gaussian noise — `HinfFilter` is more conservative
   (higher steady-state error variance) but stable; RMS error within a documented expected
   factor of the KF's (the roadmap's own framing: conservative-but-stable, not a free lunch).
3. Infeasible gamma (`gammaInit` too tight, e.g. set below the achievable bound) — `solve()`
   returns `feasible=false`, not a garbage `L`/`P`. Confirm via deliberately setting
   `gammaInit` to a value where the eigenvalue-of-`Y_inf`-vs-`gamma^2` check fails.
4. Constructing `HinfFilter` from an infeasible `HinfFilterResult` throws `std::invalid_argument`
   (mirrors `DiscreteHinf`'s own constructor contract).
5. Non-finite `predict()`/`update()` input — regression-test against `lib/`'s NaN-hold-last
   convention: confirm the chosen behavior explicitly (hold last state, do not propagate NaN)
   since `HinfFilter` is an estimator class, not an `IController`, and the convention must be
   stated rather than assumed.
