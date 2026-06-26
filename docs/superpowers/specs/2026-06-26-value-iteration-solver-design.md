# Design: ValueIterationSolver (Dynamic Programming / Value Iteration)

**Date:** 2026-06-26
**Status:** Approved, not yet implemented
**Roadmap item:** OC2 (`docs/ALGORITHM_ROADMAP_PHASE3.md`, Phase 4), first in the recommended
Phase 4 order (OC2 -> OC4 -> DT1 -> DT2 -> DT3 -> RC2).

## Motivation

No grid-based dynamic-programming solver exists anywhere in `lib/` today
(`docs/algorithm_backlog.md`'s Optimal Control section: "No DP solver exists; would need a
discretized state-space grid."). For low-dimensional optimal-control problems (`n <= 3-4` states,
curse-of-dimensionality-limited) where a globally optimal policy is wanted and MPC's local,
continuous optimization isn't required or trusted (e.g. a poor MPC initial guess on a nonlinear
swing-up problem), the toolbox currently has no answer. `ValueIterationSolver` fills that gap with
a classical value-iteration sweep over a discretized state grid.

## Scope

**In scope:**
- A standalone `ValueIterationSolver` class: regular-grid value iteration for finite state
  dimension `n` and finite control dimension `m` (vector control, not SISO-only - see decision
  log below), discounted-infinite-horizon formulation.
- Multilinear (2^n-corner) interpolation for both the internal Bellman backup (reading `V_old` at
  a continuous next-state) and the public `policy()`/`value()` query methods.
- A documented, non-blocking safety warning for runaway grid sizes.

**Out of scope (this phase) - see "Explicitly out of scope" below for the full list and reasoning.**

## Decision log (resolved during brainstorming)

These were explicit forks discussed with the user before finalizing the design; recorded here so
the rationale survives independently of the conversation that produced it.

1. **Vector control, not SISO-only.** The roadmap's draft API used a scalar `double u`. This
   design generalizes to `Eigen::VectorXd u` (m-dim control), at the cost of an `n_grid_u^m`
   action-grid blow-up per Bellman backup. Chosen over staying SISO-only because the toolbox's
   other optimal-control classes (`DiscreteLQR`, `DiscreteMPC`) are already MIMO-capable, and
   `DPGridParams::u_min`/`u_max` are vectors in the roadmap's own draft struct, implying mixed
   intent.
2. **Standalone solver only - no `IController` wrapper in this phase.** The roadmap phrased the
   wrapper as a possible follow-on ("could be wrapped... which would be the first table-driven
   controller"), not committed work, and the effort estimate (~350 lines/3 tests) doesn't budget
   for a wrapper's own bindings/tests/example. Deferred to a future roadmap item if a concrete
   consumer (e.g. a `ControllerStack` entry) needs it.
3. **Out-of-grid next-states use a fixed penalty cost, not clamping.** When a Bellman backup's
   `f(x,u)` lands outside `[x_min, x_max]`, the cost-to-go term is replaced by
   `DPGridParams::out_of_grid_penalty` instead of clamping the next-state to the grid boundary and
   interpolating there. This actively discourages the policy from selecting actions that leave the
   discretized region, which clamping (silently saturating without cost) would not do.
4. **Runaway grid size: warn, don't block.** Mirrors `DiscreteLQR`'s DARE-non-convergence
   pattern (print once to stderr, use the best available result) rather than throwing - consistent
   with the toolbox's "queryable, not enforced" philosophy. No hard cap on grid size.
5. **Synchronous (Jacobi-style) sweeps, not Gauss-Seidel.** Each sweep computes `V_new` entirely
   from the previous sweep's `V_old`, then swaps, rather than updating `V` in place. Gauss-Seidel
   converges in fewer sweeps with half the memory, but its convergence is sweep-order-dependent,
   which conflicts with the monotonic-delta-decrease test below and is harder to reason about
   deterministically. Mirrors `GradientProjectionQP`'s separate old/new buffer + explicit delta
   pattern.

## Components

### `lib/ValueIterationSolver.h` / `.cpp`

```cpp
namespace ctrl
{

struct DPGridParams
{
    Eigen::VectorXd x_min, x_max;        ///< n-dim state bounds.
    Eigen::VectorXi n_grid_per_dim;      ///< n-dim state grid resolution.
    Eigen::VectorXd u_min, u_max;        ///< m-dim control bounds.
    int    n_grid_u = 11;                ///< Per-dimension action grid resolution
                                          ///< (uniform across all m control dimensions).
    double discount = 0.99;
    int    max_iter = 500;
    double tol      = 1e-6;
    double out_of_grid_penalty = 1e6;    ///< Surrogate cost-to-go substituted for V(x') when a
                                          ///< Bellman backup's next-state x' = f(x,u) leaves
                                          ///< [x_min, x_max]. Units match the value function
                                          ///< (cost-to-go), not the raw stage cost.
};

class ValueIterationSolver
{
public:
    using StageCost  = std::function<double(const Eigen::VectorXd& x, const Eigen::VectorXd& u)>;
    using DynamicsFn = std::function<Eigen::VectorXd(const Eigen::VectorXd& x, const Eigen::VectorXd& u)>;

    ValueIterationSolver(DynamicsFn f, StageCost cost, const DPGridParams& params);

    /// Run synchronous value iteration to convergence (or max_iter). Populates the value
    /// table and the cached per-grid-point policy table used by policy()/value().
    void solve();

    /// Multilinear-interpolated best action at an arbitrary state x (clamped into grid bounds).
    Eigen::VectorXd policy(const Eigen::VectorXd& x) const;

    /// Multilinear-interpolated value-to-go at an arbitrary state x (clamped into grid bounds).
    double value(const Eigen::VectorXd& x) const;

    bool   converged()  const;  ///< true if the Bellman-residual max-norm fell below tol.
    int    iterations() const;  ///< Sweeps actually performed.
    double finalDelta() const;  ///< Final Bellman-residual max-norm (for diagnostics).

private:
    // Grid construction, multi-index <-> linear-index helpers (shared between the state grid
    // and the action grid), multilinear interpolation core, one stderr warning guard for
    // runaway grid sizes.
};

} // namespace ctrl
```

### Algorithm

**Grid construction.** Per state dimension `d`, build `n_grid_per_dim[d]` evenly spaced points
between `x_min[d]` and `x_max[d]` (`Eigen::VectorXd::LinSpaced`). The value table `V_` is a flat
`Eigen::VectorXd` of size `N_x = prod(n_grid_per_dim)`; the cached policy table is `N_x` entries of
an `m`-dim `Eigen::VectorXd` each. A shared multi-index <-> linear-index helper (mixed-radix
encode/decode) is used for both the state grid and, with `n_grid_u` substituted as a uniform
per-dimension count, the `m`-dim action grid (`N_u = n_grid_u^m` candidates).

**One sweep:**
```
for each state grid point x_i (linear index 0..N_x-1):
    best_cost = +inf
    for each action grid point u_j (linear index 0..N_u-1):
        x_next = f(x_i, u_j)
        if x_next within [x_min, x_max]:
            v_next = interpolate(V_old, x_next)      # multilinear, 2^n corners
        else:
            v_next = out_of_grid_penalty
        candidate = cost(x_i, u_j) + discount * v_next
        if candidate < best_cost:
            best_cost = candidate
            best_action = u_j
    V_new[i] = best_cost
    policy_table[i] = best_action

delta = max_i |V_new[i] - V_old[i]|
V_old, V_new = V_new, V_old   # swap (or copy)
```
`solve()` repeats this until `delta < tol` or `max_iter` sweeps are exhausted, recording
`converged()`/`iterations()`/`finalDelta()` accordingly (same accessor-after-the-fact style as
`DiscreteLQR::dareConverged()`/`dareIterations()`, since `solve()` is `void`).

**Multilinear interpolation.** For a query point `x`, locate the surrounding grid cell per
dimension (`floor`-index + fractional offset), then blend the `2^n` corner values with the
product-of-linear-weights formula (standard multilinear/bilinear-generalized interpolation).
Used both internally (reading `V_old` at a Bellman backup's next-state) and externally
(`value()`). `policy()` blends the cached per-corner *actions* the same way - analogous to
`GainScheduledController`'s `LinearBlend` mode, which already blends neighboring controllers'
outputs in this codebase, just generalized from 1-D to n-D.

**External-query clamping vs. internal penalty.** `policy()`/`value()` clamp an out-of-bounds
query `x` into `[x_min, x_max]` before interpolating - there is no "next stage" to penalize for an
arbitrary external query, so clamping (a safe, well-defined fallback) is the right behavior there.
This is independent of the `out_of_grid_penalty` mechanism, which only fires inside `solve()`'s
Bellman backup when evaluating a *dynamics-generated* next-state.

**Runaway grid size warning.** At construction, if `N_x * N_u` exceeds a documented threshold
(20,000,000), print one stderr warning ("grid is large: ~control of N sweeps may be slow/expensive")
and proceed - no hard cap, no thrown exception.

**Reused components:** None directly (no grid-based DP exists in `lib/` today). The multi-index
helper and multilinear interpolation are new, from-scratch code. `GradientProjectionQP`'s
old/new-buffer-plus-delta sweep shape and `DiscreteLQR`'s accessor-based convergence reporting
are the structural precedents being mirrored.

## Explicitly out of scope (this phase)

- **`IController` wrapper** around `policy()` (e.g. a `ValueIterationController`) - no concrete
  consumer is scoped yet; revisit as its own roadmap item if one appears.
- **Policy iteration** or any solver variant other than synchronous value iteration.
- **Continuous/refined action search** (e.g. golden-section refinement around the best grid
  action) - actions stay grid-discretized at `n_grid_u` resolution per dimension.
- **A hard cap on grid size** - warn-only, per the decision log.
- **GPU/multi-threaded sweeps.**
- **Python-side convenience** beyond direct bindings (no plotting helpers, no auto-grid-sizing
  heuristics).

## Implementation checklist

1. `lib/ValueIterationSolver.h` + `.cpp` - the struct/class above;
   `CTRL_REGISTER_FEATURE(value_iteration)` at the bottom of the header.
2. `lib/CMakeLists.txt` - append `ValueIterationSolver.cpp` to `CTRL_CORE_SOURCES` (always-on,
   no new `CTRL_ENABLE_*` flag - matches the `NSGA2`/`ComplexVectorFit` precedent of recent
   non-`IController` solver additions).
3. `lib/ControllerToolbox.h` - add the umbrella include.
4. `bindings/controllers_bindings.cpp` - bind `DPGridParams` and `ValueIterationSolver` (the
   file that already hosts `NSGA2` and `GradientProjectionQP`, neither of which is an
   `IController` either - this file is the established home for standalone optimization-style
   solvers, not just `IController` subclasses).
5. `bindings/smoke_test.py` - minimal smoke check (construct, `solve()`, query `policy()`/`value()`).
6. `examples/ex115_value_iteration_solver.cpp` (+ optional `examples/python/ex132_value_iteration_solver.py`) -
   a small 2-state mechanical system (pendulum swing-up per the roadmap's example use case) where
   the globally optimal grid policy is demonstrated. (`ex113`/`ex113_gp_mpc` and `ex114`/`ex114_complex_vector_fit`,
   plus Python `ex130`/`ex131`, are already reserved by in-progress Phase 3 work as of 2026-06-26 -
   C++ and Python examples are numbered independently, so the next free slots are `ex115` (C++) and
   `ex132` (Python). Re-check `examples/CMakeLists.txt` and `examples/python/` for the true next
   number immediately before registering, since this is a moving target while Phase 3 work lands.)
7. `tests/test_catch2_advanced.cpp` - the 4 tests below, tagged `[value_iteration]`.
8. `examples/CMakeLists.txt`, `compile.bat`, `compile.sh` - register the new example.

## Testing plan (`[value_iteration]`)

1. **LQR-equivalent problem** - linear dynamics + quadratic cost on a fine grid - converges to a
   policy matching `DiscreteLQR`'s gain within grid-resolution error.
2. **Convergence** - `finalDelta()` (Bellman-residual max-norm) decreases monotonically across
   sweeps toward `tol`, confirming the synchronous-sweep contraction property.
3. **Grid-resolution sensitivity** - a coarser grid has bounded, documented accuracy loss
   relative to a finer grid - confirms expected curse-of-dimensionality behavior, not a bug.
4. **Out-of-grid penalty** - a dynamics/cost pairing where one action would drive the state
   outside `[x_min, x_max]` is correctly disfavored by the resulting policy relative to a
   within-bounds alternative.
