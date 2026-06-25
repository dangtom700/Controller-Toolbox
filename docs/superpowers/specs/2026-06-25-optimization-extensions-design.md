# Design: NSGA-II Multi-Objective Optimization and Constrained Tuning

**Date:** 2026-06-25
**Status:** Approved, not yet implemented

## Motivation

Phase 2's two optimization items both extend the tuning layer (`AutoTuner`, `GeneticAlgorithm`,
`ParticleSwarmOptimizer`, `DifferentialEvolution`, `NelderMead`) rather than the controller fleet:

- **MO1** adds a multi-objective optimizer (NSGA-II) returning a Pareto front instead of a single
  best point — every existing metaheuristic in `lib/` is single-objective.
- **MO3** adds general nonlinear-constraint support (`g(theta) <= 0`) on top of *any* existing
  `CostFn`-based optimizer via an exterior-penalty wrapper, since today's optimizers only support
  box bounds on the parameters themselves.

`lib/GeneticAlgorithm.h` was read in full before writing this spec: `optimize()` is one monolithic
method with `tournamentSelect()` as the only private helper — there is no separated,
publicly-reusable surface for tournament selection / BLX-alpha crossover / Gaussian mutation that
`NSGA2` could literally call into. The roadmap's "reuses `GeneticAlgorithm`'s operators,
unchanged" claim is reinterpreted here as: `NSGA2` is a **self-contained sibling class**
re-implementing the *same style* of operators (binary tournament — the NSGA-II paper's own
convention, vs. `GeneticAlgorithm`'s tournament-3 — BLX-alpha crossover, Gaussian mutation), not a
literal call into `GeneticAlgorithm`. This does not change the effort estimate (the roadmap's own
~400 lines already implies a new implementation, not a thin wrapper).

## Scope

- **MO1**: real-valued (box-bounded) parameter vectors only, matching every other metaheuristic in
  `lib/`. Selection uses NSGA-II's standard binary tournament + crowded-comparison operator (Deb,
  Pratap, Agarwal & Meyarivan, *IEEE TEC* 2002) — the canonical algorithm, not a variant.
- **MO3**: the exterior (quadratic) penalty method only — not an augmented-Lagrangian or
  interior-point method. Matches the roadmap's own framing ("penalty methods only transform the
  cost function") and keeps this a pure wrapper with no new optimizer-specific state.

## Components

### 1. `lib/NSGA2.h` / `.cpp` — standalone optimizer, mirrors `GeneticAlgorithm`'s API shape

```cpp
struct NSGA2Params {
    int n_dim, n_objectives;
    int population = 100, max_gen = 200;
    double crossover = 0.9, mutation = 0.1, alpha = 0.3;   // alpha: BLX-alpha blend factor
    Eigen::VectorXd lower, upper;     // required, size n_dim (NSGA2 has no unbounded mode)
    unsigned seed = 42;
};

struct ParetoResult {
    Eigen::MatrixXd front_params;      // one row per non-dominated solution (rank 0 of final pop)
    Eigen::MatrixXd front_objectives;  // corresponding objective vectors
    int nGens;
    int nEvals;
};

using MultiCostFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;  // minimize, every component

class NSGA2 {
public:
    explicit NSGA2(const NSGA2Params &p);
    ParetoResult optimize(const MultiCostFn &cost);
};
```

**Algorithm (Deb et al. 2002, full NSGA-II generational loop):**
```
1. Initialise population P0 (size N) uniformly in [lower, upper]; evaluate objectives.
2. For gen = 1..max_gen:
   a. Fast non-dominated sort of P_gen-1 -> fronts F1, F2, ... (assigns rank[i])
      - p dominates q iff obj(p)[m] <= obj(q)[m] for all m, and obj(p)[m] < obj(q)[m] for some m.
      - domination_count[q] = #individuals dominating q; front_0 = {q : domination_count[q]==0}.
      - Iteratively peel: for p in front_k, decrement domination_count[q] for q in p's dominated
        set; any q reaching 0 joins front_(k+1). O(M.N^2) for M objectives, N population.
   b. Crowding-distance assignment within each front (per-objective: sort ascending, boundary
      points get +infinity, interior points accumulate
      (obj_m[i+1]-obj_m[i-1])/(obj_m_max-obj_m_min), 0 contribution when max==min).
   c. Binary tournament selection using the crowded-comparison operator: i beats j iff
      rank[i] < rank[j], OR (rank[i]==rank[j] AND crowding[i] > crowding[j]).
   d. BLX-alpha crossover + Gaussian mutation (same formulas as `GeneticAlgorithm`'s, reused at
      the *style* level per Scope) produce N offspring; evaluate their objectives.
   e. Combine P_gen-1 (size N) + offspring (size N) -> size 2N; repeat steps a-b on the combined
      set; build P_gen (size N) by adding whole fronts (lowest rank first) until the next front
      would overflow N, then fill the remainder from that front sorted by descending crowding
      distance (the standard NSGA-II elitist replacement).
3. After max_gen: `front_params`/`front_objectives` = rank-0 front of the final population.
```

**Degenerate `n_objectives=1` behavior:** with one objective, "non-domination" collapses to "is
the (tied-)minimum," so `front_params` may be a single row — the test plan compares the *best*
(minimum-objective) row against `GeneticAlgorithm::optimize`'s result, not the front as a
collection (a one-element front isn't a meaningful "tradeoff curve," it's just confirming NSGA2
doesn't break in the single-objective limit).

### 2. `lib/ConstrainedTuning.h` / `.cpp` — free function, pure wrapper around any `CostFn`-based optimizer

```cpp
struct ConstrainedTuneParams {
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> constraints;  // feasible iff all entries <= 0
    double penalty_init   = 10.0;
    double penalty_growth = 10.0;
    int    outer_iters     = 5;
    double feasTol         = 1e-4;   // constraints() <= feasTol counts as feasible for `converged`
};

// optimizerRun: adapts any optimizer's call shape to (CostFn, x0) -> TunerResult, e.g.
//   [&](const AutoTuner::CostFn& c, const Eigen::VectorXd& x0){ return tuner.tune(c, x0); }
//   [&](const AutoTuner::CostFn& c, const Eigen::VectorXd&  ){ return ga.optimize(c); }      // GA ignores x0
TunerResult tuneConstrained(std::function<TunerResult(const AutoTuner::CostFn &, const Eigen::VectorXd &)> optimizerRun,
                             const AutoTuner::CostFn &objective,
                             const ConstrainedTuneParams &params,
                             const Eigen::VectorXd &x0);
```

**Algorithm (exterior quadratic penalty, Nocedal & Wright Ch. 17):**
```
mu = penalty_init; x = x0; totalEvals = 0; totalGens = 0
for outer = 1..outer_iters:
    penalizedCost(theta) = objective(theta) + mu * sum_i( max(0, constraints(theta)(i))^2 )
    result = optimizerRun(penalizedCost, x)
    x = result.params; totalEvals += result.nEvals; totalGens += result.nGens
    mu *= penalty_growth
finalCost = objective(x)                              // report the TRUE objective, not the penalized one
feasible  = constraints(x).maxCoeff() <= feasTol       // (or constraints(x).size()==0 -> trivially feasible)
return TunerResult{ x, finalCost, totalEvals, totalGens, /*converged=*/ result.converged && feasible }
```
No new optimizer-specific state — `tuneConstrained` only ever calls `optimizerRun` with a
transformed cost function, exactly the "composes with everything, changes nothing inside any
optimizer" property the roadmap wants.

## Explicitly out of scope (this phase)

- **MO1 constraint handling** — `NSGA2` is unconstrained (box bounds only); combining it with
  `MO3`'s penalty wrapper for a constrained multi-objective problem is possible by construction
  (wrap `MultiCostFn`'s scalarized form) but not built/tested here.
- **MO1 reference-point / many-objective (NSGA-III-style) variants** — only the classical 2002
  crowding-distance NSGA-II, matching the roadmap's "ZDT1" 2-objective test plan.
- **MO3 augmented-Lagrangian or interior-point constraint handling** — exterior penalty only, per
  Scope.

## Implementation checklist

(Both are extensions/new standalone optimizer classes — lighter checklist, no `IController`
involved.)

**MO1:**
1. `lib/NSGA2.h`/`.cpp` + `CTRL_REGISTER_FEATURE(nsga2)`
2. `lib/CMakeLists.txt` — add `NSGA2.cpp`
3. `lib/ControllerToolbox.h` — `#include "NSGA2.h"` near `GeneticAlgorithm.h`
4. `bindings/controllers_bindings.cpp` — bind `NSGA2Params`, `ParetoResult`, `NSGA2` alongside
   `GeneticAlgorithm`/`NelderMead`
5. `bindings/smoke_test.py` — optimize a tiny 2-objective problem, assert `front_params.shape[0] >= 1`
6. `examples/ex105_nsga2.cpp` + `examples/python/ex122_nsga2.py` — PID settling-time-vs-control-effort
   tradeoff (the roadmap's own example use case)
7. `tests/test_catch2_advanced.cpp` — tests under `[nsga2]`
8. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` — add `ex105_nsga2`

**MO3:**
1. `lib/ConstrainedTuning.h`/`.cpp` + `CTRL_REGISTER_FEATURE(constrained_tuning)`
2. `lib/CMakeLists.txt` — add `ConstrainedTuning.cpp`
3. `lib/ControllerToolbox.h` — `#include "ConstrainedTuning.h"` near `AutoTuner.h`
4. `bindings/controllers_bindings.cpp` — bind `ConstrainedTuneParams`, `tune_constrained`
   (Python lambda wraps `AutoTuner`/`GeneticAlgorithm` instances on the Python side)
5. `bindings/smoke_test.py` — constrained quadratic with `AutoTuner` as the backend, assert
   the result respects the constraint within `feasTol`
6. `examples/ex106_constrained_tuning.cpp` + `examples/python/ex123_constrained_tuning.py` —
   MPC weight tuning subject to a closed-loop spectral-radius constraint (the roadmap's own
   example use case, using `SystemAnalysis`'s existing utilities for the constraint function)
7. `tests/test_catch2_advanced.cpp` — tests under `[constrained_tuning]`
8. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` — add `ex106_constrained_tuning`

## Testing plan

**`[nsga2]`**
1. ZDT1 (classic 2-objective benchmark) — recovered front matches the known analytic Pareto-front
   shape (`f2 = 1 - sqrt(f1)`) within tolerance.
2. Front diversity — crowding-distance selection keeps the final front spread across the
   objective range, not clustered at one point (measured via e.g. the spacing metric or simply
   checking `front_objectives`'s range coverage).
3. Degenerate `n_objectives=1` — best (minimum-objective) row of `front_params` matches
   `GeneticAlgorithm::optimize`'s result to equivalent quality on the same scalar problem.
4. Non-dominated sort correctness — a small hand-constructed population with known dominance
   relationships sorts into the expected fronts (a direct unit test of step 2a, independent of
   the full generational loop).

**`[constrained_tuning]`**
1. Constrained quadratic with a known analytic constrained optimum (e.g. minimize `(x-2)^2`
   subject to `x <= 1`) — `tuneConstrained` wrapping `AutoTuner` converges to `x=1`, not the
   unconstrained `x=2`.
2. Infeasible initial point (`x0` violating the constraint) — penalty growth still drives the
   final `x` into the feasible region by the last outer iteration.
3. Wraps `AutoTuner` and `GeneticAlgorithm` interchangeably (via the two different
   `optimizerRun` adapter lambdas shown above) — both backends converge to a consistent
   constrained optimum on the same problem.
4. Already-feasible `x0` with a loose constraint — result matches the unconstrained optimum
   (the penalty term contributes ~0 throughout, regression-testing that the wrapper doesn't
   distort already-feasible problems).
