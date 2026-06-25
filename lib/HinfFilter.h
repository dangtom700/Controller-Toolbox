#pragma once
#include "DiscreteHinf.h"
#include "PlantModel.h"
#include "Features.h"
#include <Eigen/Dense>

/**
 * @file HinfFilter.h
 * @brief Discrete-time H-infinity-optimal state filter (Phase 3 EF1).
 *
 * The estimation dual of `DiscreteHinf`'s two-Riccati controller synthesis: bounds the
 * worst-case ratio of estimation-error energy to disturbance/noise energy, for *any*
 * bounded disturbance - instead of `KalmanFilter`'s Gaussian-noise assumption.
 *
 * **Formulation (Simon, "Optimal State Estimation," 2006, Ch. 11 - the steady-state
 * bordered-Riccati H-infinity predictor):** estimating the full state (z = x) adds a
 * fictitious second "measurement" channel `I*x` to the real measurement `y = C*x + v`,
 * weighted with a *negative* gamma^2 block - the same indefinite-R trick `DiscreteHinf`'s
 * control DARE already uses, applied to the filter side instead:
 * @code
 *   Cbar = [C; I_n]                          // (ny+n) x n
 *   Rbar = blockdiag(Rv, -gamma^2 * I_n)      // (ny+n) x (ny+n), indefinite by construction
 *   Y = A*Y*A' + Qw - A*Y*Cbar'*(Rbar + Cbar*Y*Cbar')^-1*Cbar*Y*A'   (bordered Riccati)
 * @endcode
 * Solved via `DiscreteHinf::solveHinfDARE(A', Cbar', Qw, Rbar)` - the symplectic-pencil
 * indefinite-R Riccati solver, exactly the dual of the control-side solve.
 *
 * Feasibility: the DARE residual converges *and* `Y` is positive semi-definite *and*
 * `Y`'s spectral radius is strictly below `gamma^2` (Simon Ch. 11, Theorem 11.1) - the
 * filter-only analogue of DGKF's `rho(X_inf*Y_inf) < gamma^2` coupling condition.
 *
 * The filter gain uses only the *real* measurement channel: `L = A*Y*C' * (Rv+C*Y*C')^-1`
 * (the fictitious `I*x` channel only shapes the Riccati recursion, it is never an
 * observable signal).
 *
 * @see docs/superpowers/specs/2026-06-24-hinf-filter-design.md
 */

namespace ctrl {

/** @brief Parameters for HinfFilter::solve()'s gamma bisection. */
struct HinfFilterParams
{
    double gammaInit   = 10.0;  ///< Initial upper bound for gamma bisection.
    double gammaTol    = 1e-3;  ///< Stop bisection when the gamma bracket is narrower than this.
    int    maxIter     = 60;    ///< Maximum bisection iterations.
    double dareTol     = 1e-12; ///< Forwarded to solveHinfDARE's residual convergence check.
    int    dareMaxIter = 200;
};

/**
 * @brief Result of HinfFilter::solve().
 *
 * Carries the plant's own StateSpace alongside the synthesized gain/Riccati solution -
 * `predict()`/`update()` need the plant's (A, B, C) at runtime to actually simulate/observe
 * the state, unlike `DiscreteHinf`'s controller (which is a self-contained dynamical system
 * not needing the plant matrices once synthesized).
 */
struct HinfFilterResult
{
    bool   feasible      = false; ///< True when synthesis succeeded.
    double achievedGamma = 0.0;   ///< Smallest feasible gamma found by bisection.
    Eigen::MatrixXd L;            ///< Filter gain (n x ny).
    Eigen::MatrixXd P;            ///< Bordered Riccati solution Y (n x n).
    StateSpace      plant;        ///< The plant solve() was called against (needed at runtime).
};

/**
 * @brief Discrete-time H-infinity state filter (steady-state, fixed gain).
 */
class HinfFilter
{
public:
    /**
     * @brief Construct from a completed synthesis result.
     * @param result A HinfFilterResult with feasible == true.
     * @throws std::invalid_argument If `result.feasible` is false.
     */
    explicit HinfFilter(const HinfFilterResult &result);

    /**
     * @brief Solve the discrete-time H-infinity filtering problem.
     *
     * Bisects gamma from `[1e-4, params.gammaInit]` until the bracket is narrower than
     * `params.gammaTol` (doubling `gammaInit` up to 10 times first if it isn't itself
     * feasible) - the same structure as `DiscreteHinf::solve()`'s gamma bisection.
     *
     * @param plant  Plant StateSpace (A, B, C, D, Ts); D is not used by this filter.
     * @param Qw     Process-noise weighting (n x n, PSD).
     * @param Rv     Measurement-noise weighting (ny x ny, PD).
     * @param params Synthesis parameters.
     * @return HinfFilterResult; check `feasible` before constructing a HinfFilter.
     * @throws std::invalid_argument If the plant has no state or no measurement channel.
     */
    static HinfFilterResult solve(const StateSpace &plant,
                                   const Eigen::MatrixXd &Qw, const Eigen::MatrixXd &Rv,
                                   const HinfFilterParams &params = {});

    /** @brief Prediction step: x_ = A*x_ + B*u. Holds the last state on a non-finite @p u. */
    void predict(const Eigen::VectorXd &u);

    /** @brief Update step: x_ += L*(y - C*x_). Holds the last state on a non-finite @p y. */
    void update(const Eigen::VectorXd &y);

    /** @brief Current state estimate. */
    const Eigen::VectorXd &state() const { return x_; }

    /** @brief Reset the state estimate to zero. */
    void reset();

    /** @brief Achieved H-infinity bound gamma from synthesis. */
    double achievedGamma() const { return gamma_; }

private:
    Eigen::MatrixXd A_, B_, C_, L_, P_;
    Eigen::VectorXd x_;
    double gamma_ = 0.0;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(hinf_filter)
