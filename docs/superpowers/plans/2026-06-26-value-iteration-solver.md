# ValueIterationSolver (OC2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `ctrl::ValueIterationSolver` - grid-based dynamic programming / value iteration for
low-dimensional, discounted-infinite-horizon optimal control - per
`docs/superpowers/specs/2026-06-26-value-iteration-solver-design.md` (roadmap item OC2, Phase 4).

**Architecture:** A standalone (non-`IController`) class. Construction builds the state grid and
validates sizes; `solve()` runs synchronous (Jacobi-style) value-iteration sweeps with multilinear
(2^n-corner) interpolation, vector control (`m`-dim action grid swept exhaustively per backup),
and a fixed `out_of_grid_penalty` substituted for the cost-to-go whenever a Bellman backup's
next-state leaves the grid. `policy()`/`value()` are `const` multilinear-interpolated queries
against the converged tables, clamping an out-of-bounds query into the grid first.

**Tech Stack:** C++20, Eigen (`Eigen::VectorXd`/`VectorXi`), `std::function` callbacks, pybind11
bindings, Catch2 v3 tests.

## Global Constraints

- New `.cpp` goes into `CTRL_CORE_SOURCES` in `lib/CMakeLists.txt` - **always-on, no new
  `CTRL_ENABLE_*` flag** (matches the `NSGA2`/`ComplexVectorFit` precedent).
- Feature self-registers via `CTRL_REGISTER_FEATURE(value_iteration)` at file scope, after
  `namespace ctrl { ... }`, requiring `#include "ControllerRegistry.h"`.
- Python bindings use **snake_case** method/field names; this class is bound in
  `bindings/controllers_bindings.cpp` (the established home for standalone optimization-style
  solvers such as `NSGA2`/`GradientProjectionQP`, not just `IController` subclasses).
- `std::function` constructor parameters bind directly via `pybind11/functional.h`'s automatic
  conversion (no manual `py::object` lambda-wrapping needed) - confirmed working precedent:
  `NonlinearIMC::ModelFn`/`InverseModelFn` in the same binding file, same TU
  (`bindings/controllers_bindings.cpp:1-4` already includes `pybind11/eigen.h` and
  `pybind11/functional.h`).
- C++ examples print `PASS`/`FAIL` to stdout and `return 0`/`1`; Python examples mirror the C++
  example, use `import _setup_bindings`, skip gracefully (`SKIP:` + `sys.exit(0)`) if
  `ctrl_toolbox` or the new class isn't built yet, and print `[PASS]`/`[FAIL]` + `sys.exit(0/1)`.
- Example numbering is two independent sequences (C++ `exNNN`, Python `exNNN`); the next free
  slots as of 2026-06-26 are **`ex115`** (C++, `examples/ex113_gp_mpc.cpp` and
  `examples/ex114_complex_vector_fit.cpp` are already reserved by concurrent in-progress work)
  and **`ex132`** (Python, `ex130`/`ex131` already reserved). **Re-verify both numbers are still
  free immediately before Task 6/7** (`ls examples/ex11*.cpp examples/python/ex13*.py`) since
  this is a moving target while other Phase 3/4 work lands concurrently.
- This repo's established workflow for this kind of change (per prior session feedback) is:
  write every file's full content first, then do **one** full build + test + example + smoke-test
  pass at the end, rather than a compile cycle after every task - C++ rebuilds are too slow to
  pay for per-task. Tasks 1-7 below are pure file-writing; Task 8 is the single verification pass.
  If Task 8 surfaces a genuine implementation bug (not a typo), fix it and re-run Task 8's checks
  - don't reopen Tasks 1-7 as separate review gates.
- Do not `git commit` anything in this plan unless explicitly asked - the user owns commits.

---

### Task 1: `lib/ValueIterationSolver.h`

**Files:**
- Create: `lib/ValueIterationSolver.h`

**Interfaces:**
- Produces: `ctrl::DPGridParams` (struct, fields below), `ctrl::ValueIterationSolver` (class),
  `ctrl::ValueIterationSolver::StageCost` = `std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>`,
  `ctrl::ValueIterationSolver::DynamicsFn` = `std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>`.
  Public methods consumed by every later task: constructor `ValueIterationSolver(DynamicsFn, StageCost, const DPGridParams&)`,
  `void solve()`, `Eigen::VectorXd policy(const Eigen::VectorXd&) const`, `double value(const Eigen::VectorXd&) const`,
  `bool converged() const`, `int iterations() const`, `double finalDelta() const`.

- [ ] **Step 1: Write the header**

```cpp
#pragma once
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

/**
 * @file ValueIterationSolver.h
 * @brief Grid-based dynamic programming / value iteration for low-dimensional optimal control.
 *
 * Classical value iteration over a discretized, regular state-space grid, for
 * discounted-infinite-horizon optimal control problems where a globally optimal -- not just
 * locally optimal -- policy is wanted and the state dimension is low enough (n <= 3-4) for grid
 * discretization to be tractable. For higher-dimensional problems, use the MPC family
 * (DiscreteMPC, NonlinearMPC), which scales with horizon length rather than grid volume.
 *
 * **Algorithm:** synchronous (Jacobi-style) sweeps -- each sweep computes a full new value table
 * from the previous sweep's table, then swaps -- with multilinear (2^n-corner) interpolation
 * for reading the value function at a dynamics-generated next-state that falls between grid
 * points. Both the state grid (n-dim) and the action grid (m-dim, vector control) use a shared
 * mixed-radix multi-index <-> linear-index encoding.
 *
 * **Out-of-grid next-states:** when a Bellman backup's f(x,u) leaves [x_min, x_max], the
 * cost-to-go term is replaced by DPGridParams::out_of_grid_penalty rather than clamping --
 * this actively discourages the policy from selecting actions that drive the state out of the
 * discretized region. policy()/value() queries (not internal backups) instead clamp an
 * out-of-bounds query point into the grid before interpolating, since there is no "next stage"
 * to penalize for an arbitrary external query.
 *
 * @see docs/superpowers/specs/2026-06-26-value-iteration-solver-design.md
 * @see Bellman, R. "Dynamic Programming" (1957).
 */

namespace ctrl
{

/**
 * @brief Grid and solver parameters for ValueIterationSolver.
 */
struct DPGridParams
{
    Eigen::VectorXd x_min, x_max;      ///< State bounds (n * 1 each); every x_max entry must exceed x_min.
    Eigen::VectorXi n_grid_per_dim;    ///< State grid resolution per dimension (n * 1, each >= 2).
    Eigen::VectorXd u_min, u_max;      ///< Control bounds (m * 1 each); every u_max entry must exceed u_min.
    int    n_grid_u = 11;              ///< Action grid resolution per control dimension (>= 2, uniform across all m dims).
    double discount = 0.99;           ///< Discount factor gamma in (0, 1].
    int    max_iter = 500;            ///< Maximum value-iteration sweeps.
    double tol      = 1e-6;           ///< Convergence tolerance on the Bellman-residual max-norm.
    double out_of_grid_penalty = 1e6; ///< Surrogate cost-to-go for next-states outside [x_min, x_max].
};

/**
 * @brief Grid-based value iteration for discounted-infinite-horizon optimal control.
 *
 * Stateless after solve(): policy()/value() are const queries against the converged tables.
 * Construction is cheap (grid sizing + validation only); solve() performs the (potentially slow)
 * offline value-iteration sweeps -- mirrors DiscreteLQR's split between a cheap constructor and
 * its (also offline, also potentially-non-converging) DARE solve.
 */
class ValueIterationSolver
{
public:
    using StageCost  = std::function<double(const Eigen::VectorXd &x, const Eigen::VectorXd &u)>;
    using DynamicsFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &x, const Eigen::VectorXd &u)>;

    /**
     * @brief Construct the grid and store the dynamics/cost callbacks; does not solve.
     *
     * Prints a one-time warning to stderr if the combined state-grid x action-grid point count
     * exceeds 20,000,000 (a runaway-size guard, not a hard cap -- construction still succeeds).
     *
     * @param f      Discrete-time dynamics x[k+1] = f(x[k], u[k]).
     * @param cost   Stage cost g(x[k], u[k]) to minimize.
     * @param params Grid bounds/resolution and solver tolerances.
     * @throws std::invalid_argument If x_min/x_max/n_grid_per_dim or u_min/u_max have mismatched
     *         or zero sizes, any n_grid_per_dim entry or n_grid_u is < 2, or any x_max/u_max
     *         entry does not exceed the matching x_min/u_min entry.
     */
    ValueIterationSolver(DynamicsFn f, StageCost cost, const DPGridParams &params);

    /**
     * @brief Run synchronous value iteration to convergence (or params.max_iter sweeps).
     *
     * Resets the value table to zero and re-solves from scratch every call (deterministic,
     * repeatable -- safe to call again after changing nothing, or after externally mutating
     * the callbacks' captured state).
     */
    void solve();

    /**
     * @brief Multilinear-interpolated best action at state @p x.
     * @param x State query point (n * 1); clamped into [x_min, x_max] if outside.
     * @return Interpolated control action (m * 1).
     */
    Eigen::VectorXd policy(const Eigen::VectorXd &x) const;

    /**
     * @brief Multilinear-interpolated value-to-go at state @p x.
     * @param x State query point (n * 1); clamped into [x_min, x_max] if outside.
     * @return Interpolated V(x).
     */
    double value(const Eigen::VectorXd &x) const;

    /** @brief @c true if the last solve() call's Bellman residual fell below params.tol. */
    bool   converged()  const { return converged_; }

    /** @brief Number of sweeps the last solve() call actually performed. */
    int    iterations() const { return iterations_; }

    /** @brief Final Bellman-residual max-norm from the last solve() call. */
    double finalDelta() const { return final_delta_; }

private:
    DynamicsFn   f_;
    StageCost    cost_;
    DPGridParams params_;

    int      n_states_;
    int      n_inputs_;
    long long n_state_grid_;   ///< prod(n_grid_per_dim).
    long long n_action_grid_;  ///< n_grid_u ^ n_inputs_.

    Eigen::VectorXd              V_;        ///< Value table, flattened (n_state_grid_ * 1).
    std::vector<Eigen::VectorXd> policy_;   ///< Cached best action per grid point (n_state_grid_ entries, each m * 1).

    bool   converged_   = false;
    int    iterations_  = 0;
    double final_delta_ = std::numeric_limits<double>::infinity();

    // Mixed-radix multi-index <-> linear-index helpers (shared shape for state and action grids).
    static long long      totalGridPoints(const Eigen::VectorXi &counts);
    static Eigen::VectorXi linearToMultiIndex(long long linear, const Eigen::VectorXi &counts);
    static long long      multiIndexToLinear(const Eigen::VectorXi &multiIndex, const Eigen::VectorXi &counts);

    Eigen::VectorXd stateAt(const Eigen::VectorXi &multiIndex) const;
    Eigen::VectorXd actionAt(const Eigen::VectorXi &multiIndex) const;
    Eigen::VectorXd clampToGrid(const Eigen::VectorXd &x) const;

    // (linear state-grid index, multilinear weight) pairs for an already-clamped query point.
    std::vector<std::pair<long long, double>> interpolationCorners(const Eigen::VectorXd &xClamped) const;
    double          interpolateValue(const Eigen::VectorXd &xClamped) const;
    Eigen::VectorXd interpolatePolicy(const Eigen::VectorXd &xClamped) const;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(value_iteration)
```

- [ ] **Step 2: Move on to Task 2** - this header has no standalone test (it's declarations only);
  correctness is exercised by the Catch2 tests in Task 4 once Task 2's `.cpp` exists. Per the
  Global Constraints note above, do not attempt to compile yet.

---

### Task 2: `lib/ValueIterationSolver.cpp`

**Files:**
- Create: `lib/ValueIterationSolver.cpp`

**Interfaces:**
- Consumes: every declaration from Task 1's `lib/ValueIterationSolver.h` (exact names above).
- Produces: working definitions for all of them. Nothing later depends on anything beyond
  Task 1's public interface.

- [ ] **Step 1: Write the implementation**

```cpp
#include "ValueIterationSolver.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace ctrl
{

ValueIterationSolver::ValueIterationSolver(DynamicsFn f, StageCost cost, const DPGridParams &params)
    : f_(std::move(f)), cost_(std::move(cost)), params_(params)
{
    n_states_ = static_cast<int>(params_.x_min.size());
    n_inputs_ = static_cast<int>(params_.u_min.size());

    if (n_states_ == 0 || params_.x_max.size() != n_states_ || params_.n_grid_per_dim.size() != n_states_)
        throw std::invalid_argument(
            "ValueIterationSolver: x_min/x_max/n_grid_per_dim must be non-empty and equal size.");
    if (n_inputs_ == 0 || params_.u_max.size() != n_inputs_)
        throw std::invalid_argument("ValueIterationSolver: u_min/u_max must be non-empty and equal size.");
    for (int d = 0; d < n_states_; ++d)
        if (params_.n_grid_per_dim(d) < 2)
            throw std::invalid_argument("ValueIterationSolver: every n_grid_per_dim entry must be >= 2.");
    if (params_.n_grid_u < 2)
        throw std::invalid_argument("ValueIterationSolver: n_grid_u must be >= 2.");
    if (!(params_.x_max.array() > params_.x_min.array()).all())
        throw std::invalid_argument("ValueIterationSolver: every x_max entry must exceed the matching x_min entry.");
    if (!(params_.u_max.array() > params_.u_min.array()).all())
        throw std::invalid_argument("ValueIterationSolver: every u_max entry must exceed the matching u_min entry.");

    n_state_grid_  = totalGridPoints(params_.n_grid_per_dim);
    n_action_grid_ = 1;
    for (int d = 0; d < n_inputs_; ++d) n_action_grid_ *= params_.n_grid_u;

    constexpr long long kGridWarnThreshold = 20'000'000LL;
    if (n_state_grid_ * n_action_grid_ > kGridWarnThreshold)
    {
        std::fprintf(stderr,
            "[ValueIterationSolver] large grid: %lld state points x %lld action points "
            "(warn threshold %lld) - solve() may be slow. Consider coarsening "
            "n_grid_per_dim/n_grid_u.\n",
            n_state_grid_, n_action_grid_, kGridWarnThreshold);
    }

    V_.setZero(n_state_grid_);
    policy_.assign(static_cast<size_t>(n_state_grid_), Eigen::VectorXd::Zero(n_inputs_));
}

long long ValueIterationSolver::totalGridPoints(const Eigen::VectorXi &counts)
{
    long long total = 1;
    for (int d = 0; d < counts.size(); ++d) total *= counts(d);
    return total;
}

Eigen::VectorXi ValueIterationSolver::linearToMultiIndex(long long linear, const Eigen::VectorXi &counts)
{
    Eigen::VectorXi idx(counts.size());
    for (int d = 0; d < counts.size(); ++d)
    {
        idx(d) = static_cast<int>(linear % counts(d));
        linear /= counts(d);
    }
    return idx;
}

long long ValueIterationSolver::multiIndexToLinear(const Eigen::VectorXi &multiIndex, const Eigen::VectorXi &counts)
{
    long long linear = 0;
    long long stride = 1;
    for (int d = 0; d < counts.size(); ++d)
    {
        linear += static_cast<long long>(multiIndex(d)) * stride;
        stride *= counts(d);
    }
    return linear;
}

Eigen::VectorXd ValueIterationSolver::stateAt(const Eigen::VectorXi &multiIndex) const
{
    Eigen::VectorXd x(n_states_);
    for (int d = 0; d < n_states_; ++d)
    {
        const double t = static_cast<double>(multiIndex(d)) / (params_.n_grid_per_dim(d) - 1);
        x(d) = params_.x_min(d) + t * (params_.x_max(d) - params_.x_min(d));
    }
    return x;
}

Eigen::VectorXd ValueIterationSolver::actionAt(const Eigen::VectorXi &multiIndex) const
{
    Eigen::VectorXd u(n_inputs_);
    for (int d = 0; d < n_inputs_; ++d)
    {
        const double t = static_cast<double>(multiIndex(d)) / (params_.n_grid_u - 1);
        u(d) = params_.u_min(d) + t * (params_.u_max(d) - params_.u_min(d));
    }
    return u;
}

Eigen::VectorXd ValueIterationSolver::clampToGrid(const Eigen::VectorXd &x) const
{
    return x.cwiseMax(params_.x_min).cwiseMin(params_.x_max);
}

std::vector<std::pair<long long, double>>
ValueIterationSolver::interpolationCorners(const Eigen::VectorXd &xClamped) const
{
    Eigen::VectorXi i0(n_states_);
    Eigen::VectorXd frac(n_states_);
    for (int d = 0; d < n_states_; ++d)
    {
        const int n        = params_.n_grid_per_dim(d);
        const double span  = params_.x_max(d) - params_.x_min(d);
        const double t     = (xClamped(d) - params_.x_min(d)) / span * (n - 1);
        const int i0d      = std::clamp(static_cast<int>(std::floor(t)), 0, n - 2);
        i0(d)   = i0d;
        frac(d) = t - i0d;
    }

    const long long nCorners = 1LL << n_states_;
    std::vector<std::pair<long long, double>> corners;
    corners.reserve(static_cast<size_t>(nCorners));
    for (long long c = 0; c < nCorners; ++c)
    {
        Eigen::VectorXi corner = i0;
        double weight = 1.0;
        for (int d = 0; d < n_states_; ++d)
        {
            if ((c >> d) & 1) { corner(d) += 1; weight *= frac(d); }
            else                weight *= (1.0 - frac(d));
        }
        corners.emplace_back(multiIndexToLinear(corner, params_.n_grid_per_dim), weight);
    }
    return corners;
}

double ValueIterationSolver::interpolateValue(const Eigen::VectorXd &xClamped) const
{
    double result = 0.0;
    for (const auto &[linear, weight] : interpolationCorners(xClamped))
        result += weight * V_(linear);
    return result;
}

Eigen::VectorXd ValueIterationSolver::interpolatePolicy(const Eigen::VectorXd &xClamped) const
{
    Eigen::VectorXd result = Eigen::VectorXd::Zero(n_inputs_);
    for (const auto &[linear, weight] : interpolationCorners(xClamped))
        result += weight * policy_[static_cast<size_t>(linear)];
    return result;
}

void ValueIterationSolver::solve()
{
    V_.setZero(n_state_grid_);
    policy_.assign(static_cast<size_t>(n_state_grid_), Eigen::VectorXd::Zero(n_inputs_));

    Eigen::VectorXd V_new(n_state_grid_);
    const Eigen::VectorXi actionCounts = Eigen::VectorXi::Constant(n_inputs_, params_.n_grid_u);

    converged_   = false;
    iterations_  = 0;
    final_delta_ = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < params_.max_iter; ++iter)
    {
        double maxDelta = 0.0;

        for (long long i = 0; i < n_state_grid_; ++i)
        {
            const Eigen::VectorXd x_i = stateAt(linearToMultiIndex(i, params_.n_grid_per_dim));

            double bestCost = std::numeric_limits<double>::infinity();
            Eigen::VectorXd bestAction = Eigen::VectorXd::Zero(n_inputs_);

            for (long long j = 0; j < n_action_grid_; ++j)
            {
                const Eigen::VectorXd u_j    = actionAt(linearToMultiIndex(j, actionCounts));
                const Eigen::VectorXd x_next = f_(x_i, u_j);

                const bool inBounds = (x_next.array() >= params_.x_min.array()).all() &&
                                       (x_next.array() <= params_.x_max.array()).all();
                const double v_next = inBounds ? interpolateValue(x_next) : params_.out_of_grid_penalty;
                const double candidate = cost_(x_i, u_j) + params_.discount * v_next;

                if (candidate < bestCost)
                {
                    bestCost   = candidate;
                    bestAction = u_j;
                }
            }

            V_new(i) = bestCost;
            policy_[static_cast<size_t>(i)] = bestAction;
            maxDelta = std::max(maxDelta, std::abs(bestCost - V_(i)));
        }

        V_.swap(V_new);
        iterations_  = iter + 1;
        final_delta_ = maxDelta;

        if (maxDelta < params_.tol)
        {
            converged_ = true;
            break;
        }
    }
}

Eigen::VectorXd ValueIterationSolver::policy(const Eigen::VectorXd &x) const
{
    return interpolatePolicy(clampToGrid(x));
}

double ValueIterationSolver::value(const Eigen::VectorXd &x) const
{
    return interpolateValue(clampToGrid(x));
}

} // namespace ctrl
```

- [ ] **Step 2: Move on to Task 3.**

---

### Task 3: Build wiring

**Files:**
- Modify: `lib/CMakeLists.txt:98` (immediately after the `GPMPC.cpp` line, inside `CTRL_CORE_SOURCES`)
- Modify: `lib/ControllerToolbox.h:157` (immediately after the `ComplexVectorFit.h` include line)

**Interfaces:**
- Consumes: `lib/ValueIterationSolver.h`/`.cpp` from Tasks 1-2 (file names only).
- Produces: nothing new - this just makes Task 1/2's files reachable from `#include "ControllerToolbox.h"` and part of the `controller_toolbox` build target.

- [ ] **Step 1: Add the source file to `lib/CMakeLists.txt`**

Find this in `lib/CMakeLists.txt` (the `CTRL_CORE_SOURCES` list, currently ending in `GPMPC.cpp`):

```cmake
    FTCSupervisor.cpp
    GPMPC.cpp
)
```

Change to:

```cmake
    FTCSupervisor.cpp
    GPMPC.cpp
    ValueIterationSolver.cpp
)
```

- [ ] **Step 2: Add the umbrella include to `lib/ControllerToolbox.h`**

Find this line (currently the last entry before the `// Optional modules` comment block):

```cpp
#include "ComplexVectorFit.h"        ///< ComplexVectorFit - complex-conjugate-pole Vector Fitting (Phase 3 FD2).
```

Add immediately after it:

```cpp
#include "ValueIterationSolver.h"    ///< ValueIterationSolver - grid-based dynamic programming / value iteration (Phase 4 OC2).
```

- [ ] **Step 3: Move on to Task 4.**

---

### Task 4: Catch2 tests

**Files:**
- Modify: `tests/test_catch2_advanced.cpp` (append at end of file; current EOF is the `GPMPC`
  test group ending around line 9638 - re-check with a search for the last `TEST_CASE` before
  inserting, since this file is being concurrently modified by other in-progress work)

**Interfaces:**
- Consumes: `ctrl::DPGridParams`, `ctrl::ValueIterationSolver` (Tasks 1-2); `ctrl::DiscreteLQR`,
  `ctrl::LQRParams`, `ctrl::ssStepCopy`, and the file's existing `makeDoubleIntegrator()` helper
  (already defined at `tests/test_catch2_advanced.cpp:86`) and `Ts` constant.
- Produces: 4 new `TEST_CASE`s tagged `[value_iteration]`, run automatically by the existing
  `test_catch2_advanced` Catch2 executable (no `tests/CMakeLists.txt` change needed).

- [ ] **Step 1: Append the four test cases**

Add at the end of `tests/test_catch2_advanced.cpp`:

```cpp
// -----------------------------------------------------------------------------
// ValueIterationSolver - grid-based DP / value iteration (Phase 4 OC2)
// -----------------------------------------------------------------------------

TEST_CASE("ValueIterationSolver matches DiscreteLQR's gain on a double-integrator "
          "(LQR-equivalent problem)", "[value_iteration]")
{
    auto plant = makeDoubleIntegrator();
    ctrl::LQRParams lqr_p;
    lqr_p.Q = 10.0 * Eigen::Matrix2d::Identity();
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);
    ctrl::DiscreteLQR lqr(plant, lqr_p);
    REQUIRE(lqr.dareConverged());

    auto f = [&plant](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return ctrl::ssStepCopy(plant, x, u).second;
    };
    auto cost = [&lqr_p](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return x.dot(lqr_p.Q * x) + u.dot(lqr_p.R * u);
    };

    ctrl::DPGridParams gp;
    gp.x_min = Eigen::Vector2d(-1.0, -1.0);
    gp.x_max = Eigen::Vector2d( 1.0,  1.0);
    gp.n_grid_per_dim = Eigen::Vector2i(61, 61);
    gp.u_min = Eigen::VectorXd::Constant(1, -5.0);
    gp.u_max = Eigen::VectorXd::Constant(1,  5.0);
    gp.n_grid_u = 41;
    gp.discount = 0.99;
    gp.max_iter = 300;
    gp.tol      = 1e-5;

    ctrl::ValueIterationSolver vi(f, cost, gp);
    vi.solve();
    REQUIRE(vi.converged());

    const Eigen::VectorXd x_test = (Eigen::VectorXd(2) << 0.5, 0.0).finished();
    const Eigen::VectorXd u_lqr  = lqr.compute(x_test);
    const Eigen::VectorXd u_vi   = vi.policy(x_test);

    REQUIRE_THAT(u_vi(0), WithinAbs(u_lqr(0), 0.5)); // grid-resolution tolerance
    REQUIRE(std::isfinite(vi.value(x_test)));
}

TEST_CASE("ValueIterationSolver's Bellman residual decreases monotonically across sweep counts",
          "[value_iteration]")
{
    ctrl::DPGridParams gp;
    gp.x_min = Eigen::Vector2d(-1.0, -1.0);
    gp.x_max = Eigen::Vector2d( 1.0,  1.0);
    gp.n_grid_per_dim = Eigen::Vector2i(21, 21);
    gp.u_min = Eigen::VectorXd::Constant(1, -3.0);
    gp.u_max = Eigen::VectorXd::Constant(1,  3.0);
    gp.n_grid_u = 9;
    gp.discount = 0.95;
    gp.tol      = 1e-9; // tight enough that none of the iter counts below actually converge

    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(2);
        xn(0) = x(0) + 0.1 * x(1);
        xn(1) = x(1) + 0.1 * u(0);
        return xn;
    };
    auto cost = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return x.squaredNorm() + 0.1 * u.squaredNorm();
    };

    double prevDelta = std::numeric_limits<double>::infinity();
    for (int iters : {5, 10, 15, 20, 25})
    {
        gp.max_iter = iters;
        ctrl::ValueIterationSolver vi(f, cost, gp);
        vi.solve();
        REQUIRE_FALSE(vi.converged());
        REQUIRE(vi.finalDelta() <= prevDelta + 1e-12);
        prevDelta = vi.finalDelta();
    }
}

TEST_CASE("ValueIterationSolver's policy error shrinks as the grid is refined "
          "(curse-of-dimensionality, not a bug)", "[value_iteration]")
{
    auto plant = makeDoubleIntegrator();
    ctrl::LQRParams lqr_p;
    lqr_p.Q = 10.0 * Eigen::Matrix2d::Identity();
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);
    ctrl::DiscreteLQR lqr(plant, lqr_p);

    auto f = [&plant](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return ctrl::ssStepCopy(plant, x, u).second;
    };
    auto cost = [&lqr_p](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return x.dot(lqr_p.Q * x) + u.dot(lqr_p.R * u);
    };

    const Eigen::VectorXd x_test = (Eigen::VectorXd(2) << 0.5, 0.0).finished();
    const double u_lqr = lqr.compute(x_test)(0);

    auto policyErrorAt = [&](int n_grid) {
        ctrl::DPGridParams gp;
        gp.x_min = Eigen::Vector2d(-1.0, -1.0);
        gp.x_max = Eigen::Vector2d( 1.0,  1.0);
        gp.n_grid_per_dim = Eigen::Vector2i(n_grid, n_grid);
        gp.u_min = Eigen::VectorXd::Constant(1, -5.0);
        gp.u_max = Eigen::VectorXd::Constant(1,  5.0);
        gp.n_grid_u = 41;
        gp.discount = 0.99;
        gp.max_iter = 300;
        gp.tol      = 1e-5;

        ctrl::ValueIterationSolver vi(f, cost, gp);
        vi.solve();
        return std::abs(vi.policy(x_test)(0) - u_lqr);
    };

    const double err_coarse = policyErrorAt(11);
    const double err_fine   = policyErrorAt(61);

    REQUIRE(err_fine < err_coarse);
}

TEST_CASE("ValueIterationSolver's out_of_grid_penalty determines whether an out-of-bounds "
          "action is selected over a within-bounds alternative", "[value_iteration]")
{
    // cost(x,u) = -x*u rewards (negative cost) actions that push x further from zero in its
    // current direction. At x=0.9 the cheapest-looking actions all leave the grid; this isolates
    // out_of_grid_penalty's effect on the first sweep, where V_old == 0 everywhere (hand-verified:
    // with penalty=0.5, escaping via u=2 costs -1.8 + 0.95*0.5 = -1.325, beating the best
    // in-bounds action u=0 at cost 0; with penalty=1e6, escaping costs ~949998, losing badly).
    auto f = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(1);
        xn(0) = x(0) + u(0);
        return xn;
    };
    auto cost = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return -x(0) * u(0);
    };

    ctrl::DPGridParams gp;
    gp.x_min = Eigen::VectorXd::Constant(1, -1.0);
    gp.x_max = Eigen::VectorXd::Constant(1,  1.0);
    gp.n_grid_per_dim = Eigen::VectorXi::Constant(1, 21); // spacing 0.1
    gp.u_min = Eigen::VectorXd::Constant(1, -2.0);
    gp.u_max = Eigen::VectorXd::Constant(1,  2.0);
    gp.n_grid_u = 9;     // spacing 0.5: -2,-1.5,...,2
    gp.discount = 0.95;
    gp.max_iter = 1;     // exactly one sweep: V_old == 0 everywhere, hand-verifiable
    gp.tol      = 1e-12;

    const Eigen::VectorXd x_query = Eigen::VectorXd::Constant(1, 0.9); // exactly on a grid point

    gp.out_of_grid_penalty = 0.5;
    ctrl::ValueIterationSolver vi_weak(f, cost, gp);
    vi_weak.solve();
    REQUIRE(vi_weak.policy(x_query)(0) > 1.5); // picks the escaping action (u=2)

    gp.out_of_grid_penalty = 1e6;
    ctrl::ValueIterationSolver vi_strong(f, cost, gp);
    vi_strong.solve();
    REQUIRE(vi_strong.policy(x_query)(0) > -0.5);
    REQUIRE(vi_strong.policy(x_query)(0) <  0.5); // settles on u~=0, the cheapest in-bounds action
}
```

- [ ] **Step 2: Move on to Task 5.**

---

### Task 5: Python bindings

**Files:**
- Modify: `bindings/controllers_bindings.cpp:2537` (immediately after the `NonlinearIMC` binding
  block's final `.def("set_state", ...)` line, before the `// Fuzzy bindings are fully
  implemented...` comment)

**Interfaces:**
- Consumes: `ctrl::DPGridParams`, `ctrl::ValueIterationSolver`, `ctrl::ValueIterationSolver::DynamicsFn`,
  `ctrl::ValueIterationSolver::StageCost` (Task 1).
- Produces: Python-visible `ctrl.DPGridParams` and `ctrl.ValueIterationSolver` with snake_case
  methods `solve()`, `policy(x)`, `value(x)`, `converged()`, `iterations()`, `final_delta()`, used
  by Tasks 6/7's smoke test and Python example.

- [ ] **Step 1: Insert the binding block**

Find this in `bindings/controllers_bindings.cpp` (the end of the `NonlinearIMC` block):

```cpp
        .def("compute",     &ctrl::NonlinearIMC::compute, py::arg("error"))
        .def("reset",       &ctrl::NonlinearIMC::reset)
        .def("sample_time", &ctrl::NonlinearIMC::sampleTime)
        .def("name",        &ctrl::NonlinearIMC::name)
        .def("set_state",   &ctrl::NonlinearIMC::setState, py::arg("x"));

    // Fuzzy bindings are fully implemented in advanced_bindings.cpp (bind_advanced).
```

Insert a new block between those two pieces:

```cpp
        .def("compute",     &ctrl::NonlinearIMC::compute, py::arg("error"))
        .def("reset",       &ctrl::NonlinearIMC::reset)
        .def("sample_time", &ctrl::NonlinearIMC::sampleTime)
        .def("name",        &ctrl::NonlinearIMC::name)
        .def("set_state",   &ctrl::NonlinearIMC::setState, py::arg("x"));

    // -----------------------------------------------------------------------
    // ValueIterationSolver (Phase 4 OC2)
    // -----------------------------------------------------------------------
    py::class_<ctrl::DPGridParams>(m, "DPGridParams",
        "Grid and solver parameters for ValueIterationSolver.")
        .def(py::init<>())
        .def_readwrite("x_min",               &ctrl::DPGridParams::x_min)
        .def_readwrite("x_max",               &ctrl::DPGridParams::x_max)
        .def_readwrite("n_grid_per_dim",      &ctrl::DPGridParams::n_grid_per_dim)
        .def_readwrite("u_min",               &ctrl::DPGridParams::u_min)
        .def_readwrite("u_max",               &ctrl::DPGridParams::u_max)
        .def_readwrite("n_grid_u",            &ctrl::DPGridParams::n_grid_u)
        .def_readwrite("discount",            &ctrl::DPGridParams::discount)
        .def_readwrite("max_iter",            &ctrl::DPGridParams::max_iter)
        .def_readwrite("tol",                 &ctrl::DPGridParams::tol)
        .def_readwrite("out_of_grid_penalty", &ctrl::DPGridParams::out_of_grid_penalty);

    py::class_<ctrl::ValueIterationSolver>(m, "ValueIterationSolver", R"doc(
Grid-based dynamic programming / value iteration for low-dimensional (n<=3-4 states)
discounted-infinite-horizon optimal control, where a globally optimal policy is wanted and
MPC's local, continuous optimization isn't required or trusted.

Usage
-----
>>> gp = ctrl.DPGridParams()
>>> gp.x_min = np.array([-1.0, -1.0]); gp.x_max = np.array([1.0, 1.0])
>>> gp.n_grid_per_dim = np.array([41, 41])
>>> gp.u_min = np.array([-5.0]); gp.u_max = np.array([5.0])
>>> vi = ctrl.ValueIterationSolver(dynamics_fn, cost_fn, gp)
>>> vi.solve()
>>> u = vi.policy(x)
)doc")
        .def(py::init<ctrl::ValueIterationSolver::DynamicsFn,
                      ctrl::ValueIterationSolver::StageCost,
                      const ctrl::DPGridParams &>(),
             py::arg("dynamics_fn"), py::arg("cost_fn"), py::arg("params"))
        .def("solve",       &ctrl::ValueIterationSolver::solve)
        .def("policy",      &ctrl::ValueIterationSolver::policy, py::arg("x"))
        .def("value",       &ctrl::ValueIterationSolver::value, py::arg("x"))
        .def("converged",   &ctrl::ValueIterationSolver::converged)
        .def("iterations",  &ctrl::ValueIterationSolver::iterations)
        .def("final_delta", &ctrl::ValueIterationSolver::finalDelta);

    // Fuzzy bindings are fully implemented in advanced_bindings.cpp (bind_advanced).
```

- [ ] **Step 2: Add the smoke test**

Append to `bindings/smoke_test.py` (after the `NARMAXIdentifier` block, before the final
`print('\nAll smoke tests passed.')` line):

```python
# ---------------------------------------------------------------------------
# ValueIterationSolver (Phase 4 OC2) smoke test
# ---------------------------------------------------------------------------
assert hasattr(ctrl, 'ValueIterationSolver'), "ValueIterationSolver not bound"
assert ctrl.registry_has('value_iteration'), "value_iteration not registered"


def _vi_dynamics(x, u):
    return np.array([x[0] + 0.1 * x[1], x[1] + 0.1 * u[0]])


def _vi_cost(x, u):
    return float(x @ x + 0.1 * (u @ u))


_vi_gp = ctrl.DPGridParams()
_vi_gp.x_min = np.array([-1.0, -1.0])
_vi_gp.x_max = np.array([1.0, 1.0])
_vi_gp.n_grid_per_dim = np.array([11, 11])
_vi_gp.u_min = np.array([-2.0])
_vi_gp.u_max = np.array([2.0])
_vi_gp.n_grid_u = 5
_vi_gp.discount = 0.95
_vi_gp.max_iter = 50
_vi_gp.tol = 1e-3

_vi = ctrl.ValueIterationSolver(_vi_dynamics, _vi_cost, _vi_gp)
_vi.solve()
_u_vi = _vi.policy(np.array([0.5, 0.0]))
assert np.all(np.isfinite(_u_vi)), "ValueIterationSolver policy not finite"
_v_vi = _vi.value(np.array([0.5, 0.0]))
assert np.isfinite(_v_vi), "ValueIterationSolver value not finite"
print('ValueIterationSolver smoke test passed.')

print('\nAll smoke tests passed.')
```

(Remove the old standalone `print('\nAll smoke tests passed.')` line that previously ended the
file - there must be exactly one, at the very end.)

- [ ] **Step 3: Move on to Task 6.**

---

### Task 6: C++ example

**Files:**
- Create: `examples/ex115_value_iteration_solver.cpp` (re-verify `ex115` is still free first -
  see Global Constraints)
- Modify: `examples/CMakeLists.txt` (register the example, after the `ex113_gp_mpc`/`ex114_complex_vector_fit`
  entries - re-check exact current last line first, since other work is landing concurrently)
- Modify: `compile.bat` and `compile.sh` (add `ex115_value_iteration_solver` to the sequential
  target list, after whatever the current last `exNNN` entry is)

**Interfaces:**
- Consumes: `ctrl::DPGridParams`, `ctrl::ValueIterationSolver` (Task 1), `#include "ControllerToolbox.h"`.
- Produces: an executable target `ex115_value_iteration_solver` that prints `PASS`/`FAIL`.

- [ ] **Step 1: Write the example**

```cpp
/**
 * @file ex115_value_iteration_solver.cpp
 * @brief Phase 4 (OC2): Grid-based dynamic programming / value iteration - pendulum swing-up.
 *
 * Solves the classical undamped-pendulum swing-up problem (start hanging down at theta=0,
 * reach upright theta=pi) via discretized-state-space value iteration rather than a locally
 * linearizing controller - demonstrating a globally optimal policy on a low-dimensional (n=2)
 * nonlinear system. Pendulum parameters (ml2=1.0, mgl=9.8) match examples/ex98_passivity_based.cpp.
 *
 * @see docs/superpowers/specs/2026-06-26-value-iteration-solver-design.md
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01, ml2 = 1.0, mgl = 9.8;

    auto f = [Ts, ml2, mgl](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(2);
        const double theta_ddot = (u(0) - mgl * std::sin(x(0))) / ml2;
        xn(1) = x(1) + Ts * theta_ddot;
        xn(0) = x(0) + Ts * xn(1);
        return xn;
    };
    auto cost = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return (1.0 + std::cos(x(0))) + 0.05 * x(1) * x(1) + 0.001 * u(0) * u(0);
    };

    ctrl::DPGridParams gp;
    gp.x_min = Eigen::Vector2d(-M_PI, -8.0);
    gp.x_max = Eigen::Vector2d( M_PI,  8.0);
    gp.n_grid_per_dim = Eigen::Vector2i(41, 41);
    gp.u_min = Eigen::VectorXd::Constant(1, -15.0);
    gp.u_max = Eigen::VectorXd::Constant(1,  15.0);
    gp.n_grid_u = 15;
    gp.discount = 0.97;
    gp.max_iter = 400;
    gp.tol      = 1e-4;
    gp.out_of_grid_penalty = 1e5;

    ctrl::ValueIterationSolver vi(f, cost, gp);
    vi.solve();
    std::printf("solve(): converged=%d iterations=%d finalDelta=%.6g\n",
                vi.converged() ? 1 : 0, vi.iterations(), vi.finalDelta());

    Eigen::VectorXd state = Eigen::VectorXd::Zero(2); // hanging straight down
    for (int k = 0; k < 1500; ++k)
    {
        const Eigen::VectorXd u = vi.policy(state);
        state = f(state, u);
    }

    std::printf("Final state: theta=%.4f theta_dot=%.4f (target theta=pi=%.4f)\n",
                state(0), state(1), M_PI);

    const bool ok = vi.converged() && state.allFinite() &&
                    std::cos(state(0)) < -0.9 && std::fabs(state(1)) < 1.5;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
```

- [ ] **Step 2: Register it in `examples/CMakeLists.txt`**

Find the line registering the most recently added example (currently `ex114_complex_vector_fit`
or `ex113_gp_mpc` - confirm the actual last `add_example(...)` line in the main list before the
`# MIMO / advanced examples` section, since this is a moving target). Add immediately after it:

```cmake
# Phase 4 (Algorithm Roadmap Phase 3): OC2 dynamic programming / value iteration
add_example(ex115_value_iteration_solver)
```

- [ ] **Step 3: Register it in `compile.bat`**

In the `for %%T in ( ... )` target list, add a new line `    ex115_value_iteration_solver`
immediately after the current last `exNNN` entry.

- [ ] **Step 4: Register it in `compile.sh`**

In the matching target list (mirrors `compile.bat`), add `    ex115_value_iteration_solver`
immediately after the current last `exNNN` entry.

- [ ] **Step 5: Move on to Task 7.**

---

### Task 7: Python example (optional but included for parity with other Phase 3/4 items)

**Files:**
- Create: `examples/python/ex132_value_iteration_solver.py` (re-verify `ex132` is still free
  first - see Global Constraints)

**Interfaces:**
- Consumes: `ctrl.DPGridParams`, `ctrl.ValueIterationSolver` (Task 5's bindings).
- Produces: a standalone script printing `[PASS]`/`[FAIL]` and exiting 0/1 (or `SKIP:` + exit 0
  if the binding isn't built).

- [ ] **Step 1: Write the example**

```python
"""
ex132_value_iteration_solver.py

Phase 4 (OC2): Grid-based dynamic programming / value iteration - pendulum swing-up.

Mirrors ex115_value_iteration_solver.cpp -- solves the same undamped-pendulum swing-up problem
(start hanging down at theta=0, reach upright theta=pi) via discretized-state-space value
iteration.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ValueIterationSolver'):
        raise AttributeError("ValueIterationSolver not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts, ml2, mgl = 0.01, 1.0, 9.8


def f_dyn(x, u):
    theta_ddot = (u[0] - mgl * np.sin(x[0])) / ml2
    theta_dot_next = x[1] + Ts * theta_ddot
    theta_next = x[0] + Ts * theta_dot_next
    return np.array([theta_next, theta_dot_next])


def cost_fn(x, u):
    return float((1.0 + np.cos(x[0])) + 0.05 * x[1] ** 2 + 0.001 * u[0] ** 2)


gp = ctrl.DPGridParams()
gp.x_min = np.array([-np.pi, -8.0])
gp.x_max = np.array([np.pi, 8.0])
gp.n_grid_per_dim = np.array([41, 41])
gp.u_min = np.array([-15.0])
gp.u_max = np.array([15.0])
gp.n_grid_u = 15
gp.discount = 0.97
gp.max_iter = 400
gp.tol = 1e-4
gp.out_of_grid_penalty = 1e5

vi = ctrl.ValueIterationSolver(f_dyn, cost_fn, gp)
vi.solve()
print(f"solve(): converged={vi.converged()} iterations={vi.iterations()} "
      f"finalDelta={vi.final_delta():.6g}")

state = np.zeros(2)  # hanging straight down
for _ in range(1500):
    u = vi.policy(state)
    state = f_dyn(state, u)

print(f"Final state: theta={state[0]:.4f} theta_dot={state[1]:.4f} (target theta=pi={np.pi:.4f})")

ok = bool(vi.converged() and np.all(np.isfinite(state)) and
          np.cos(state[0]) < -0.9 and abs(state[1]) < 1.5)
print("[PASS]" if ok else "[FAIL]")
sys.exit(0 if ok else 1)
```

- [ ] **Step 2: Move on to Task 8.**

---

### Task 8: Full build, test, and smoke verification (the single checkpoint for this plan)

**Files:** none (verification only).

**Interfaces:** none - this task only runs what Tasks 1-7 produced.

- [ ] **Step 1: Configure and build the core library + Catch2 tests + Python bindings**

Run (adjust shell per the Windows/MSYS2 UCRT64 toolchain note in `CLAUDE.md` section 2 if linker
errors mention missing `libgcc`/`libstdc++`/`libwinpthread` symbols):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTRL_BUILD_PYTHON_BINDINGS=ON
cmake --build build --target controller_toolbox
cmake --build build --target test_catch2_advanced
cmake --build build --target ctrl_toolbox
cmake --build build --target ex115_value_iteration_solver
```

Expected: all four targets build with no errors. If there's a compile error, fix it in the
relevant Task's file (don't paper over it) and rebuild just that target before moving on. If a
fix isn't obvious within a couple of attempts, record the exact error and what was tried in
`docs/cumulative_bug_report.md` rather than continuing to iterate blindly, per this repo's usual
practice for build issues that resist a quick fix.

- [ ] **Step 2: Run the new Catch2 tests**

```bash
ctest --test-dir build -R test_catch2_advanced --output-on-failure
```

(Or, if iterating only on the new cases: run the test binary directly with the tag filter, e.g.
`build/tests/test_catch2_advanced.exe "[value_iteration]"` on Windows or the equivalent
extensionless path on Linux/macOS.)

Expected: all 4 `[value_iteration]` cases pass, and nothing else regresses. If the LQR-matching
test (Task 4, test 1) or the grid-refinement test (test 3) fails on tolerance (not on a crash or
NaN), it's most likely the grid resolution or the `0.5` tolerance needing adjustment, not a logic
bug - widen the grid or the tolerance slightly and re-run before suspecting the algorithm.

- [ ] **Step 3: Run the C++ example**

```bash
build/examples/ex115_value_iteration_solver
```

(Path may be `build/examples/Release/ex115_value_iteration_solver.exe` depending on generator.)

Expected: prints `solve(): converged=1 ...`, then the final pendulum state, then `PASS`. If it
prints `FAIL`, check the printed final `theta`/`theta_dot` against the target (`theta=pi`) - if
it's close but just outside tolerance, loosen the `0.9`/`1.5` thresholds or increase the 1500-step
simulation horizon in `examples/ex115_value_iteration_solver.cpp`; if `theta` isn't moving from 0
at all, suspect the action bounds or grid bounds first (an `out_of_grid_penalty` dominating
everywhere is the most likely root cause).

- [ ] **Step 4: Run the Python smoke test and example**

```bash
conda run -n soft_robotics -- python bindings/smoke_test.py
conda run -n soft_robotics -- python examples/python/ex132_value_iteration_solver.py
```

Expected: smoke test prints `ValueIterationSolver smoke test passed.` followed by
`All smoke tests passed.`; the example prints the same `solve()`/final-state diagnostics as the
C++ version followed by `[PASS]`.

- [ ] **Step 5: Run the full `run.py` validation**

```bash
conda run -n soft_robotics -- python run.py
```

This is the canonical "is everything passing" check (7 phases). Per established practice for this
repo, let this run to completion (it's long); if it fails, `bug_report.txt` will identify exactly
which phase/target. Do not declare this work done without a clean `run.py` pass - report results
back rather than guessing.
