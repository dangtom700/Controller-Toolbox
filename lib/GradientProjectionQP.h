#pragma once
#include <Eigen/Dense>
#include <cmath>

/**
 * @file GradientProjectionQP.h
 * @brief Header-only FISTA solver for box-constrained convex QPs.
 *
 * Solves:
 * @code
 *   min_{x}   1/2 x'Hx + g'x
 *   s.t.      lb <= x <= ub
 * @endcode
 *
 * **Algorithm:** FISTA (Fast Iterative Shrinkage-Thresholding Algorithm,
 * Beck & Teboulle 2009) with step alpha = 1/L, L = lambda_max(H).
 *
 * Convergence rate: O(1/k^2) objective error, i.e., O(1/sqrt(kappa)) per decade
 * of tolerance compared to O(1/kappa) for plain projected gradient descent.
 * For a Hessian condition number kappa, FISTA converges in O(sqrt(kappa)) iterations
 * vs O(kappa) for PGD -- typically 10-100x fewer iterations on ill-conditioned MPC
 * Hessians (large prediction horizon, weight ratio rho_y/rho_u >> 1).
 *
 * **Warm start:** x_0 = clamp(-H^{-1}g, lb, ub) (unconstrained optimum projected).
 *
 * **Zero-allocation hot path (iterations):** All per-iteration work uses pre-sized
 * scratch vectors passed by the caller. One VectorXd of size n is allocated per
 * QP solve (for the FISTA momentum point y); this is not per-iteration.
 *
 * @par FISTA update equations
 * @code
 *   y_0    = x_0  (warm start)
 *   t_0    = 1
 *   x_{k+1} = proj( y_k - (1/L)(Hy_k + g), lb, ub )
 *   t_{k+1} = (1 + sqrt(1 + 4*t_k^2)) / 2
 *   beta_k  = (t_k - 1) / t_{k+1}
 *   y_{k+1} = x_{k+1} + beta_k * (x_{k+1} - x_k)
 * @endcode
 *
 * @see Beck, A. & Teboulle, M. (2009). A fast iterative shrinkage-thresholding
 *      algorithm for linear inverse problems. SIAM J. Imag. Sci. 2(1):183-202.
 * @see Camacho & Bordons, "Model Predictive Control" (2007), Ch. 3.
 */

namespace ctrl
{

/**
 * @brief Result of a gradient-projection QP solve.
 */
struct QPSolveResult
{
    bool converged; ///< @c true if the FISTA iterate satisfied ||x_{k+1}-x_k||_inf < tol.
    int  iters;     ///< Actual number of FISTA iterations performed.
};

/**
 * @brief Solve a box-constrained QP via FISTA (zero-allocation per iteration).
 *
 * On entry, @p x must be pre-sized to the problem dimension n (its value is ignored -
 * the warm start comes from the LDLT solve). On exit, @p x holds the solution.
 *
 * @param H       Positive-definite Hessian (n * n).
 * @param g       Linear cost vector (n * 1).
 * @param lb      Lower bound (n * 1).
 * @param ub      Upper bound (n * 1).
 * @param ldlt    Pre-factored LDLT decomposition of @p H.
 * @param L       Lipschitz constant lambda_max(H). Step size alpha = 1/L.
 * @param maxIter Maximum FISTA iterations.
 * @param tol     Convergence tolerance ||x_{k+1} - x_k||_inf.
 * @param x       [in/out] Solution vector (pre-allocated to size n).
 * @param tmp1    Scratch vector (pre-allocated to size n). Contents are overwritten.
 * @param tmp2    Scratch vector (pre-allocated to size n). Contents are overwritten.
 * @param y_fista FISTA extrapolation workspace (pre-allocated to size n). Contents are overwritten.
 * @return QPSolveResult {converged, iters}.
 *
 * @pre @p H, @p g, @p lb, @p ub, @p x, @p tmp1, @p tmp2, @p y_fista all have the same size n.
 * @pre @p L > 0.
 * @pre @p ldlt.info() == Eigen::Success.
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
    Eigen::Ref<Eigen::VectorXd>             x,
    Eigen::Ref<Eigen::VectorXd>             tmp1,    // gradient workspace
    Eigen::Ref<Eigen::VectorXd>             tmp2,    // x_new workspace
    Eigen::Ref<Eigen::VectorXd>             y_fista) // FISTA momentum point (pre-allocated)
{
    // Warm start: clamped unconstrained optimum
    x = (-ldlt.solve(g)).cwiseMax(lb).cwiseMin(ub);

    // FISTA momentum point initialised from warm-started x (no allocation)
    y_fista = x;

    const double alpha = 1.0 / L;
    double t           = 1.0;
    QPSolveResult result{false, 0};

    for (int iter = 0; iter < maxIter; ++iter)
    {
        // Gradient at momentum point y_fista
        tmp1.noalias() = H * y_fista + g;

        // Projected gradient step: x_new = proj(y_fista - alpha*grad, lb, ub)
        tmp2 = (y_fista - alpha * tmp1).cwiseMax(lb).cwiseMin(ub);

        // Convergence check on ||x_new - x_k||_inf
        const double delta = (tmp2 - x).cwiseAbs().maxCoeff();

        // FISTA momentum update
        const double t_new  = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * t * t));
        const double beta   = (t - 1.0) / t_new;

        // y_fista_{k+1} = x_new + beta * (x_new - x_k)
        y_fista.noalias() = tmp2 + beta * (tmp2 - x);

        // Advance iterates
        x = tmp2;
        t = t_new;

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
