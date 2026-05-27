#pragma once
#include <Eigen/Dense>

/**
 * @file GradientProjectionQP.h
 * @brief Header-only gradient-projection solver for box-constrained convex QPs.
 *
 * Solves:
 * @code
 *   min_{x}   ½ xᵀHx + gᵀx
 *   s.t.      lb ≤ x ≤ ub
 * @endcode
 *
 * **Algorithm:** Projected gradient descent with constant step α = 1/L, where L = λmax(H)
 * is the Lipschitz constant of the gradient ∇f = Hx + g. Warm-started from the clamped
 * unconstrained optimum x₀ = clamp(−H⁻¹g, lb, ub).
 *
 * **Zero-allocation hot path:** All temporaries are passed in by the caller as pre-sized
 * scratch vectors. DiscreteMPC and GeneralizedPredictiveController use this to eliminate
 * per-step heap allocation.
 *
 * **Convergence:** The iteration terminates when ‖x_new − x_old‖∞ < tol. For typical MPC
 * horizons (Nc ≤ 10, well-conditioned H), this converges in O(10) iterations.
 *
 * @see Camacho & Bordons, "Model Predictive Control" (2007), Ch. 3.
 * @see Bertsekas, "Nonlinear Programming" (1999), §2.3 (projected gradient).
 */

namespace ctrl
{

/**
 * @brief Result of a gradient-projection QP solve.
 */
struct QPSolveResult
{
    bool converged; ///< @c true if ‖Δx‖∞ < tol before maxIter.
    int  iters;     ///< Actual number of gradient-projection iterations performed.
};

/**
 * @brief Solve a box-constrained QP via gradient projection (zero-allocation).
 *
 * On entry, @p x must be pre-sized to the problem dimension n (its value is ignored — the
 * warm start comes from the LDLT solve). On exit, @p x holds the solution.
 *
 * @param H       Positive-definite Hessian (n × n).
 * @param g       Linear cost vector (n × 1).
 * @param lb      Lower bound (n × 1). May equal @p ub for equality constraints.
 * @param ub      Upper bound (n × 1).
 * @param ldlt    Pre-factored LDLT decomposition of @p H. Must be valid (info() == Success).
 *                Refreshed externally when H changes (e.g., after setParams()).
 * @param L       Lipschitz constant λmax(H) — precomputed step size 1/L.
 * @param maxIter Maximum gradient-projection iterations.
 * @param tol     Convergence tolerance ‖Δx‖∞.
 * @param x       [in/out] Solution vector (pre-allocated to size n).
 * @param tmp1    Scratch vector (pre-allocated to size n). Contents are overwritten.
 * @param tmp2    Scratch vector (pre-allocated to size n). Contents are overwritten.
 * @return QPSolveResult with convergence flag and iteration count.
 *
 * @pre @p H, @p g, @p lb, @p ub, @p x, @p tmp1, @p tmp2 all have the same size n.
 * @pre @p L > 0 (undefined behaviour if L ≤ 0).
 * @pre @p ldlt.info() == Eigen::Success (callers check before invoking).
 */
inline QPSolveResult solveGradientProjectionQP(
    const Eigen::MatrixXd                  &H,
    const Eigen::VectorXd                  &g,
    const Eigen::VectorXd                  &lb,
    const Eigen::VectorXd                  &ub,
    const Eigen::LDLT<Eigen::MatrixXd>     &ldlt,
    double                                  L,
    int                                     maxIter,
    double                                  tol,
    Eigen::VectorXd                        &x,
    Eigen::VectorXd                        &tmp1,
    Eigen::VectorXd                        &tmp2)
{
    // Warm-start: clamped unconstrained optimum x₀ = clamp(−H⁻¹g, lb, ub)
    x = (-ldlt.solve(g)).cwiseMax(lb).cwiseMin(ub);

    const double alpha = 1.0 / L;
    QPSolveResult result{false, 0};

    for (int iter = 0; iter < maxIter; ++iter)
    {
        tmp1.noalias() = H * x + g;                              // gradient at x
        tmp2           = (x - alpha * tmp1).cwiseMax(lb).cwiseMin(ub); // projected step

        const double delta = (tmp2 - x).cwiseAbs().maxCoeff();
        x = tmp2;

        if (delta < tol)
        {
            result.converged = true;
            result.iters     = iter + 1;
            return result;
        }
    }

    result.iters = maxIter;
    return result;
}

} // namespace ctrl
