# Design: LPSolver (Linear-Programming-Based Control)

**Date:** 2026-06-27
**Status:** Approved, implementing
**Roadmap item:** OC4 (`docs/ALGORITHM_ROADMAP_PHASE3.md`, Phase 4), second in the recommended
Phase 4 order (OC2 -> **OC4** -> DT1 -> DT2 -> DT3 -> RC2). OC2 (`ValueIterationSolver`) is done.

## Motivation

`docs/algorithm_backlog.md`'s Optimal Control section: "No LP solver exists in `lib/` (only
`GradientProjectionQP` for QP)." Min-time control and L1/Linf-cost MPC are naturally linear
programs, not quadratic ones — `GradientProjectionQP` (FISTA over a box-constrained QP) cannot
express either a linear cost or a general inequality constraint, so there is currently no path to
an L1-cost MPC variant in this toolbox.

## Pre-implementation audit finding (load-bearing correction to the roadmap sketch)

The roadmap's "Reused components" note claims `LPSolver` reuses "`GradientProjectionQP`'s
box-constraint-projection pattern as a starting point for the active-set working-set updates."
Verified against `lib/GradientProjectionQP.h`: that solver's only constraint handling is
`cwiseMax(lb).cwiseMin(ub)` — clamp-to-box. It has no general inequality (`A_ineq x <= b_ineq`)
machinery, and FISTA is a first-order method that does not produce exact LP vertices (the entire
point of solving an LP). **`LPSolver` is therefore a from-scratch two-phase simplex, not an
extension of `GradientProjectionQP`.** What carries over from that file is only convention (header-
only free-standing solver, caller-facing result struct, LDLT-style guarded numerics), not algorithm
or code. Effort is revised upward from the roadmap's ~400-line estimate to ~550-650 lines across
solver + MPC + tests, reflecting a real two-phase simplex (Phase-1 artificial-variable feasibility
search + Phase-2 optimization) rather than a QP-solver extension.

## Scope

**In scope:**
- `LPSolver::solve()` — a standalone, stateless two-phase simplex solver for general bounded-
  variable LPs (`A_ineq`, `A_eq`, and box `lb`/`ub`, all optional/required as documented below).
- `LPMPC` — a SISO linear MPC `IController` that casts an L1-cost (absolute tracking error +
  absolute control-move) receding-horizon problem into an LP each step and solves it via
  `LPSolver`.

**Out of scope (this phase):**
- MIMO `LPMPC` (vector output/input) — deferred; see decision log.
- Min-time (free-horizon) control — the roadmap's headline example is not a fixed-size LP without
  a horizon reformulation; not attempted here (see decision log).
- A persistent, allocation-free simplex workspace reused across `LPMPC::compute()` calls — the
  solver allocates its tableau fresh per call (see decision log: RT contract).

## Decision log (resolved before implementation)

1. **Bounded variables via shift-and-augment, not native bounded-variable simplex.** Every
   `LPProblem` variable has finite `lb`/`ub` (no `-inf`/`+inf` sentinel — this toolbox's existing
   convention is `+-1e9` for "effectively unbounded", e.g. `BacksteppingParams::uMin/uMax`).
   Rather than implement the more intricate revised-simplex-with-upper-bounding tableau, each
   variable `x_i` is shifted to `y_i = x_i - lb_i >= 0`, and the upper bound becomes an explicit
   extra inequality row `y_i <= ub_i - lb_i`. This roughly doubles the row count (`n` extra rows)
   but keeps the simplex itself in the textbook "all variables >= 0" form, which is far easier to
   get right and to verify. Chosen over the more compact bounded-variable simplex because
   correctness review is cheaper and this toolbox's problem sizes (MPC horizons, not large-scale
   LPs) don't need the extra efficiency.
2. **Uniform artificial-variable Phase 1 (Big-M-free two-phase), not Big-M.** Every constraint row
   (after sign-normalizing so RHS >= 0) gets one artificial variable; Phase 1 minimizes their sum.
   Big-M is avoided because it introduces a magic constant that must dominate the true costs
   without causing numerical ill-conditioning — a known footgun. Two-phase has no such constant.
3. **Bland's rule (smallest-index), not Dantzig's largest-coefficient rule, for entering/leaving
   variable selection.** Bland's rule is provably cycle-free; Dantzig's rule is faster in practice
   but can cycle on degenerate LPs (which MPC's box-bound-row augmentation makes more likely, since
   many rows can be simultaneously tight). Given the roadmap's own test plan requires "infeasible
   LP — doesn't loop forever," cycle-freedom is a correctness requirement, not a nice-to-have.
4. **`LPProblem` includes `A_eq`/`b_eq`, not just `A_ineq`/`b_ineq` as the roadmap sketch had.**
   Once a real two-phase simplex exists, equality rows are nearly free (they already need an
   artificial variable; they just skip the slack column). Adding them now avoids an API break
   later. `LPMPC` itself does not end up needing `A_eq` (see below) — it's included for the
   solver's general-purpose API completeness, validated by the Catch2 tests directly.
5. **`LPResult` has a 4-way `LPStatus` enum (`Optimal`/`Infeasible`/`Unbounded`/`IterationLimit`),
   not a bare `bool feasible`.** The roadmap sketch's `bool feasible` cannot distinguish
   infeasible from unbounded, which are different failure modes a caller needs to react to
   differently. Per the box-bound contract (decision 1), a well-formed `LPProblem` is always a
   bounded polytope, so `Unbounded` should be unreachable in practice for problems that respect the
   contract — kept as a defensive status, not a primary code path; see the analysis in the solver's
   own doc comment for why this is provably rare.
6. **`LPMPC` is SISO-only (single input, single output), not MIMO like `DiscreteMPC`.** Going
   MIMO means the L1-epigraph row count and the rolling cumulative-bound tightening (ported from
   `DiscreteMPC::computeRef`) both need vector bookkeeping per channel. Given the audit above
   already revised effort upward once, staying SISO keeps the lift bounded and matches the
   roadmap's own "min-time control of an actuator-limited system" example, which is inherently
   single-actuator. Constructor throws `std::invalid_argument` if the plant isn't 1-input/1-output.
7. **`LPMPC::compute()`/`computeRef()` is explicitly NOT a zero-allocation hot path.** Unlike
   `DiscreteMPC` (which pre-allocates all FISTA workspace vectors at construction per
   `docs/deployment.md`'s Zero-Allocation Checklist), `LPSolver::solve()` builds its tableau
   (`Eigen::MatrixXd`) fresh on every call, because the row/column count is a function of the
   *current* RHS sign pattern (which rows get sign-flipped before adding artificials) and isn't
   worth tracking as a stable, reusable layout for a first version. This is a documented,
   deliberate limitation (mirrors `DiscreteH2`'s and other classes' explicit `@warning`-style
   scoping in this codebase) rather than a silent RT-contract violation. `DiscreteMPC` remains the
   recommended choice for hard-RT loops where an L2 cost is acceptable.
8. **Min-time control (the roadmap's headline LP-MPC example) is not implemented.** Min-time is a
   free-horizon problem (minimize N such that the target is reached), not a fixed-size LP — making
   it one requires a feasibility-search-over-increasing-N outer loop that is a distinct, separable
   piece of work. `LPMPC`'s actual cost (L1 tracking + L1 move-suppression, fixed horizon) is the
   other naturally-LP formulation the roadmap names ("L1/Linf-cost MPC are naturally LPs") and is
   implemented in full; min-time is left for a future item if a concrete case-study consumer needs
   it.

## Components

### `lib/LPSolver.h` (header-only)

```cpp
namespace ctrl {

enum class LPStatus { Optimal, Infeasible, Unbounded, IterationLimit };

struct LPProblem {
    Eigen::VectorXd c;          // minimize c'x
    Eigen::MatrixXd A_ineq;     // (m_ineq x n); A_ineq.rows() == 0 is valid (no inequalities)
    Eigen::VectorXd b_ineq;     // A_ineq * x <= b_ineq
    Eigen::MatrixXd A_eq;       // (m_eq x n); A_eq.rows() == 0 is valid (no equalities)
    Eigen::VectorXd b_eq;       // A_eq * x == b_eq
    Eigen::VectorXd lb, ub;     // required, finite (n); this toolbox's "unbounded" convention is +-1e9
};

struct LPResult {
    LPStatus        status = LPStatus::IterationLimit;
    Eigen::VectorXd x;       // meaningful only when status == Optimal
    double          cost = 0.0;
    int             iters = 0;
};

class LPSolver {
public:
    static LPResult solve(const LPProblem& problem, int maxIter = 200, double tol = 1e-8);
};

} // namespace ctrl
```

### Algorithm

1. **Shift to nonnegative variables:** `y = x - lb`, `y >= 0`, cost unchanged (`c'x = c'lb + c'y`,
   the constant term is dropped from the LP and re-added when reporting `cost`).
2. **Augment with box rows:** append `n` rows `y_i <= ub_i - lb_i` to the inequality block.
3. **Standardize:** every inequality row gets a slack column (`+1` coefficient); every row
   (inequality-with-slack and equality alike) is sign-normalized so its RHS is `>= 0`, then gets
   one artificial-variable column (`+1` coefficient, identity block by construction).
4. **Phase 1:** minimize the sum of artificials via Bland's-rule simplex (smallest-index entering
   column with negative reduced cost; minimum-ratio leaving row, smallest-basis-index tie-break).
   If the optimal Phase-1 objective exceeds `tol`, return `Infeasible` immediately.
5. **Phase 2:** recompute the reduced-cost row from scratch against the real cost vector and the
   current basis (`row0[j] = c2_j - sum_i c2[basis[i]] * T[i][j]`), then run Bland's-rule simplex
   again, with artificial columns permanently excluded from the entering-column search (they
   naturally get pivoted out of the basis if a real variable's ratio test ever selects their row;
   no special-casing needed — this is verified via the standard simplex feasibility-preservation
   argument: every non-pivot row's RHS update is `old - (entering coeff)*(min ratio)`, which is
   nonnegative by construction of the ratio test, so a degenerate zero-valued artificial can never
   be driven negative by another pivot).
6. **Extract:** read basic-variable values off the final tableau's RHS column, map back via
   `x = lb + y`, recompute `cost = c.dot(x)` directly (not from the tableau's internal tracking, to
   keep the reported cost trivially correct regardless of any sign-convention subtlety inside the
   tableau).

**On `Unbounded` being structurally rare:** since every `y_i` has its own explicit box row in the
augmented system, a column cannot have "no positive entries in any row" in the *original* tableau;
after pivoting, the *current* tableau's column for that variable can in principle still leave no
eligible ratio-test row, but the LP's overall feasible region is provably bounded (a subset of the
box), so `Unbounded` cannot be the LP's true status — it's defensive code for a state that should
not arise for a `LPProblem` that honors the box-bound contract, and is differentiated from
`IterationLimit` purely so a caller can immediately recognize a malformed problem (e.g. a `b_ineq`
that contradicts the stated `lb`/`ub`) rather than silently treating it as a `tol`/`maxIter` tuning
issue.

**Reused components:** None at the algorithm level (see audit finding above) — `GradientProjectionQP.h`
contributes only the "header-only, stateless, struct-in/struct-out" convention. `NelderMead.h`/
`NSGA2.h` contribute the "standalone non-`IController` optimizer, `CTRL_REGISTER_FEATURE` at file
bottom, bound in `controllers_bindings.cpp` (the established home for non-`IController` solvers,
per the `ValueIterationSolver` precedent)" structural pattern.

### `lib/LPMPC.h` / `.cpp`

```cpp
namespace ctrl {

struct LPMPCParams {
    int    Np = 10, Nc = 3;
    double rho_y = 1.0, rho_u = 0.1;
    double uMin = -1e9, uMax = 1e9, duMin = -1e9, duMax = 1e9;
    int    lpMaxIter = 200;
    double lpTol     = 1e-8;
};

class LPMPC : public IController {
public:
    LPMPC(const StateSpace& plant, const LPMPCParams& params);  // throws if plant isn't SISO
    double compute(double error) override;
    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }
    double computeRef(const Eigen::VectorXd& x_current, double r_ref);
    void   reset() override;
    double sampleTime() const override { return Ts_; }
    void   setParams(const LPMPCParams& p);
    void   setPlant(const StateSpace& plant);
    void   setState(const Eigen::VectorXd& x) { x_hat_ = x; }
    void   setLastApplied(double u_applied) { u_prev_ = u_applied; }
    [[nodiscard]] bool lastLPConverged() const { return last_lp_converged_; }
    int    lastLPIters() const { return last_lp_iters_; }
    [[nodiscard]] bool isHealthy() const override { return last_lp_converged_; }
};

} // namespace ctrl
```

**Formulation** — condensed prediction `Yhat = F*x + Gu*u_prev + Phi*DeltaU` (identical `F`/`Phi`/
`Gu` formulas to `DiscreteMPC::buildPredictionMatrices`, specialized to `m=p=1`), L1 cost via
epigraph slacks:

```
minimize    rho_y * sum(t_y) + rho_u * sum(t_u)
subject to  Phi*DeltaU - t_y <=  rhs1         rhs1 = r_stack - F*x - Gu*u_prev
           -Phi*DeltaU - t_y <= -rhs1
            DeltaU - t_u <= 0
           -DeltaU - t_u <= 0
            t_y >= 0, t_u >= 0
            DeltaU in [lb_j, ub_j]   (rolling worst-case tightened box bounds, identical
                                      port of DiscreteMPC::computeRef's cumMin_/cumMax_ loop —
                                      this is the one piece that *is* a direct line-for-line
                                      port, since cumulative-u-bound-as-tightened-DeltaU-box is
                                      QP/LP-agnostic)
```

No `A_eq` rows are needed (cumulative-`u` bounds are folded into `DeltaU`'s box, exactly as
`DiscreteMPC` already does for its QP) — `LPMPC` exercises `LPSolver`'s inequality+box path only;
`A_eq` is validated solely by `LPSolver`'s own direct Catch2 tests.

`Phi`, `F`, `Gu`, and the LP's structural zero/identity blocks are built once at construction
(mirrors `DiscreteMPC::buildCondensedMatrices`); each `compute()`/`computeRef()` call only
recomputes `rhs1` and the `DeltaU` box-bound tightening before calling `LPSolver::solve()`.

## Explicitly out of scope (this phase)

- MIMO `LPMPC` (decision 6).
- Min-time / free-horizon LP-MPC (decision 8).
- A reusable, allocation-free simplex workspace inside `LPMPC`'s hot path (decision 7).
- Revised (sparse) simplex — dense `Eigen::MatrixXd` tableau only; fine for MPC-scale horizons,
  not intended for large-scale LP use.

## Implementation checklist

1. `lib/LPSolver.h` — header-only, `CTRL_REGISTER_FEATURE(lp_solver)` at file bottom.
2. `lib/LPMPC.h` + `lib/LPMPC.cpp` — `lib/CMakeLists.txt`: append `LPMPC.cpp` to
   `CTRL_CORE_SOURCES`.
3. `lib/ControllerToolbox.h` — umbrella includes for both, inserted after the
   `EventTriggeredWrapper.h` line (next free slot in the always-on include block).
4. `bindings/controllers_bindings.cpp` — bind `LPProblem`/`LPResult`/`LPStatus`/`LPSolver` (the
   `NSGA2`/`NelderMead`/`ValueIterationSolver` non-`IController`-solver home) and
   `LPMPCParams`/`LPMPC` (`shared_ptr<LPMPC>` 3rd template arg, per the `IController` rule).
5. `bindings/smoke_test.py` — minimal smoke checks for both.
6. `examples/ex118_lp_solver.cpp`, `examples/ex119_lp_mpc.cpp` (+ Python
   `examples/python/ex135_lp_solver.py`, `ex136_lp_mpc.py`) — next free slots as of 2026-06-27
   (`ex117`/`ex134` are the last reserved by the prior scope-triage-cleanup session).
7. `tests/test_catch2_advanced.cpp` — tests below, tagged `[lp_solver]` / `[lp_mpc]`.
8. `examples/CMakeLists.txt`, `compile.bat`, `compile.sh` — register both new examples.
9. `docs/cumulative_bug_report.md` — new Part section (next: Part 70).
10. `docs/ALGORITHM_ROADMAP_PHASE3.md` status table — OC4 Open -> Done.
11. `docs/algorithm_backlog.md` — move the "Linear-programming-based control" line to "Already
    done".

## Testing plan

`[lp_solver]` (`LPSolver` direct, cross-validated against hand-computed textbook optima):
1. Known 2-variable LP with a textbook vertex solution (e.g. maximize-as-minimize-negated profit
   problem) — matches the known optimum exactly within tolerance.
2. Equality-constrained LP (`A_eq` nonempty) — recovers a known solution, confirming the `A_eq`
   path independent of `LPMPC` (which doesn't exercise it).
3. Infeasible LP (contradictory box + inequality) — returns `LPStatus::Infeasible` within
   `maxIter`, never loops forever.
4. Degenerate/redundant constraint row (one row a linear multiple of another) — still reaches
   `Optimal` (regression guard for the "artificial parked at zero in the basis" case analyzed
   above).
5. `maxIter` deliberately set too low on a problem that needs more pivots — returns
   `LPStatus::IterationLimit`, not a crash or a silently wrong `Optimal`.

`[lp_mpc]` (`LPMPC`, cross-checked against `DiscreteMPC` where the comparison is meaningful):
1. Step-reference tracking on a stable first-order plant — settles near the reference, `u` stays
   within `[uMin, uMax]` throughout.
2. `DeltaU` move-suppression — increasing `rho_u` measurably reduces peak `|DeltaU|` versus a low-
   `rho_u` run on the same plant/reference (the L1 analogue of `DiscreteMPC`'s `rho_u` behavior).
3. Actuator saturation — a reference outside `[uMin, uMax]`'s reachable output still respects the
   hard bound every step (confirms the ported cumulative-tightening box-bound logic).
4. NaN input — `compute()` holds `u_prev_`, matching the fleet NaN-guard contract.
5. `lastLPConverged()`/`isHealthy()` — false after an artificially starved `lpMaxIter`, true in the
   normal case (mirrors `DiscreteMPC::lastQPConverged()`'s contract).
