#pragma once
#include <Eigen/Dense>
#include "PlantModel.h"

/**
 * @file BalancedTruncation.h
 * @brief Model order reduction via balanced truncation for discrete-time systems.
 *
 * Balanced truncation (Moore 1981) replaces a high-order state-space model with a
 * low-order approximation that preserves the most energetically significant modes.
 *
 * **Algorithm:**
 * 1. Solve the discrete Lyapunov equations for the controllability gramian Pc and
 *    observability gramian Po.
 * 2. Compute the balanced realisation via Cholesky + SVD of Pc^{1/2} * Po * Pc^{1/2}.
 * 3. The Hankel singular values sigma_i (descending) quantify how much each balanced mode
 *    contributes to input-output energy.
 * 4. Truncate to order r by keeping only the r largest-sigma states.
 *
 * **Hinf error bound:** ||G - Gᵣ||inf <= 2 . Sigma_i=ᵣ₊1^n sigma_i
 *
 * @par Complexity note
 * Uses ctrl::SystemAnalysis::solveDiscreteLyapunov which is O(n⁶) via Kronecker
 * vectorisation. Suitable for n <= 10. For n > 10, the Bartels-Stewart algorithm
 * (O(n^3)) is strongly preferred (not currently in this toolbox).
 *
 * @par Typical workflow
 * @code
 *   ctrl::StateSpace full = ...;   // n=6 FEM model
 *
 *   // Inspect Hankel singular values
 *   ctrl::TruncationResult res = ctrl::balancedTruncate(full, 2);
 *   std::cout << "HSVs: " << res.hankelSingularValues.transpose() << "\n";
 *   std::cout << "Error bound for r=2: " << res.errorBound << "\n";
 *
 *   int r_suggest = ctrl::suggestOrder(res, 0.01);  // tol = 1 % of DC gain
 *   ctrl::TruncationResult best = ctrl::balancedTruncate(full, r_suggest);
 *
 *   // Design LQR on reduced model; apply gain to full-order plant
 *   ctrl::DiscreteLQR lqr(best.reduced, lqr_params);
 * @endcode
 *
 * @see Moore, "Principal component analysis in linear systems", IEEE TAC (1981).
 * @see Dullerud & Paganini, "A Course in Robust Control Theory", Ch. 4.
 */

namespace ctrl
{

/** @brief Result of a balanced truncation operation. */
struct TruncationResult
{
    StateSpace      reduced;               ///< Order-r state-space approximation.
    Eigen::VectorXd hankelSingularValues;  ///< All n Hankel singular values (descending).
    double          errorBound;            ///< Hinf error bound = 2.Sigma_i=ᵣ₊1^n sigma_i.
    bool            isStable;             ///< True if all poles of @c reduced are strictly inside UC.
};

/**
 * @brief Compute the balanced truncation of a discrete-time system to order r.
 *
 * @param sys Strictly stable discrete-time state-space model (all |eigenvalues(A)| < 1).
 *            Throws if the system is not stable or if Pc is not positive definite.
 * @param r   Target reduced order (1 <= r < n).
 * @return    TruncationResult with reduced model, Hankel singular values, and error bound.
 * @throws std::invalid_argument If sys is not strictly stable, r is out of range, or
 *   Pc is numerically singular (system has near-uncontrollable modes).
 */
TruncationResult balancedTruncate(const StateSpace &sys, int r);

/**
 * @brief Suggest a reduced order based on a relative tolerance on the error bound.
 *
 * Returns the smallest r such that the Hinf error bound for the (r+1..n) truncated modes
 * is less than @p tol * ||G||inf_approx, where ||G||inf is approximated as 2.Sigma_i sigma_i (tight
 * bound for high-sigma systems).
 *
 * @param res   Result from balancedTruncate (any r, since all HSVs are returned).
 * @param tol   Relative tolerance (default 0.01 = 1 %). Must be in (0, 1).
 * @return      Suggested order, clamped to [1, n-1].
 */
int suggestOrder(const TruncationResult &res, double tol = 0.01);

} // namespace ctrl
