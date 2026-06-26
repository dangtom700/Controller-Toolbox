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
