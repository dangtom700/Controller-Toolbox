#include "GreyBoxEstimator.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

GreyBoxEstimator::GreyBoxEstimator(OdeFn ode, MeasFn meas, const Params& par)
    : f_(std::move(ode)), h_(std::move(meas)), par_(par)
{
    if (par_.p0.size() == 0)
        throw std::invalid_argument("GreyBoxEstimator: p0 must be non-empty");
    p_est_ = clipParams(par_.p0);
}

// ---- RK4 integration (rk4_steps substeps per Ts) ----------------------------

Eigen::VectorXd GreyBoxEstimator::rk4Step(const Eigen::VectorXd& x,
                                            const Eigen::VectorXd& u,
                                            const Eigen::VectorXd& p) const
{
    const double h = par_.Ts / par_.rk4_steps;
    Eigen::VectorXd xk = x;
    for (int s = 0; s < par_.rk4_steps; ++s) {
        const Eigen::VectorXd k1 = f_(xk,                u, p);
        const Eigen::VectorXd k2 = f_(xk + 0.5*h*k1,    u, p);
        const Eigen::VectorXd k3 = f_(xk + 0.5*h*k2,    u, p);
        const Eigen::VectorXd k4 = f_(xk + h*k3,        u, p);
        xk += (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
    }
    return xk;
}

// ---- Simulate output trajectory given p -------------------------------------

Eigen::MatrixXd GreyBoxEstimator::simulateY(const Eigen::VectorXd& x0,
                                              const Eigen::MatrixXd& U,
                                              const Eigen::VectorXd& p) const
{
    const int N   = static_cast<int>(U.cols());
    const int n_y = static_cast<int>(h_(x0, p).size());
    Eigen::MatrixXd Y_hat(n_y, N);
    Eigen::VectorXd x = x0;
    for (int k = 0; k < N; ++k) {
        Y_hat.col(k) = h_(x, p);
        x = rk4Step(x, U.col(k), p);
    }
    return Y_hat;
}

// ---- Residual: (Y_meas - Y_hat) stacked column-major into one vector --------

Eigen::VectorXd GreyBoxEstimator::computeResidual(const Eigen::VectorXd& x0,
                                                    const Eigen::MatrixXd& U,
                                                    const Eigen::MatrixXd& Y,
                                                    const Eigen::VectorXd& p) const
{
    const Eigen::MatrixXd diff = Y - simulateY(x0, U, p);
    return Eigen::Map<const Eigen::VectorXd>(diff.data(), diff.size());
}

// ---- Central-difference Jacobian d(residual)/d(p) ---------------------------

Eigen::MatrixXd GreyBoxEstimator::computeJacobianFD(const Eigen::VectorXd& x0,
                                                      const Eigen::MatrixXd& U,
                                                      const Eigen::MatrixXd& Y,
                                                      const Eigen::VectorXd& p) const
{
    const int n_p = static_cast<int>(p.size());
    const int n_r = static_cast<int>(Y.size());  // n_y * N

    Eigen::MatrixXd J(n_r, n_p);
    Eigen::VectorXd p_fwd = p, p_bwd = p;

    for (int i = 0; i < n_p; ++i) {
        const double eps = par_.eps_jac * std::max(std::abs(p(i)), 1.0);
        p_fwd(i) = p(i) + eps;
        p_bwd(i) = p(i) - eps;
        J.col(i) = (computeResidual(x0, U, Y, p_fwd) -
                    computeResidual(x0, U, Y, p_bwd)) / (2.0 * eps);
        p_fwd(i) = p(i);
        p_bwd(i) = p(i);
    }
    return J;
}

// ---- Box-clip parameter vector to [lower, upper] ----------------------------

Eigen::VectorXd GreyBoxEstimator::clipParams(const Eigen::VectorXd& p) const
{
    if (par_.lower.size() == 0)
        return p;
    const int n_p = static_cast<int>(p.size());
    Eigen::VectorXd q = p;
    for (int i = 0; i < n_p; ++i)
        q(i) = std::min(std::max(q(i), par_.lower(i)), par_.upper(i));
    return q;
}

// ---- Levenberg-Marquardt main loop ------------------------------------------

GreyBoxEstimator::Result
GreyBoxEstimator::fit(const Eigen::VectorXd& x0,
                       const Eigen::MatrixXd& U,
                       const Eigen::MatrixXd& Y)
{
    const int n_p = static_cast<int>(par_.p0.size());
    Eigen::VectorXd p_cur = clipParams(par_.p0);
    double lambda = par_.lm_lambda0;

    Eigen::VectorXd r = computeResidual(x0, U, Y, p_cur);
    double cost = 0.5 * r.squaredNorm();
    bool converged = false;
    int iter = 0;

    for (; iter < par_.max_iter; ++iter) {
        const Eigen::MatrixXd J = computeJacobianFD(x0, U, Y, p_cur);
        const Eigen::VectorXd g = J.transpose() * r;  // gradient

        if (g.cwiseAbs().maxCoeff() < par_.tol_grad) {
            converged = true;
            break;
        }

        // Damped normal equations: (J'J + lambda*I) dp = -g
        const Eigen::MatrixXd H = J.transpose() * J;
        const Eigen::MatrixXd A = H + lambda * Eigen::MatrixXd::Identity(n_p, n_p);
        const Eigen::VectorXd dp = A.ldlt().solve(-g);

        const Eigen::VectorXd p_new = clipParams(p_cur + dp);
        const Eigen::VectorXd r_new = computeResidual(x0, U, Y, p_new);
        const double cost_new = 0.5 * r_new.squaredNorm();

        if (cost_new < cost) {
            p_cur  = p_new;
            r      = r_new;
            cost   = cost_new;
            lambda = std::max(lambda / par_.lm_nu, 1e-10);
        } else {
            lambda = std::min(lambda * par_.lm_nu, 1e10);
        }
    }

    p_est_ = p_cur;
    fitted_ = true;
    return {p_cur, cost, iter, converged};
}

// ---- Predict output trajectory using current estimated params ----------------

Eigen::MatrixXd GreyBoxEstimator::predict(const Eigen::VectorXd& x0,
                                            const Eigen::MatrixXd& U) const
{
    return simulateY(x0, U, p_est_);
}

} // namespace ctrl
