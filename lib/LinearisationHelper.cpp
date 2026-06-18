#include "LinearisationHelper.h"
#include <cmath>

namespace ctrl
{

// ---------------------------------------------------------------------------
// Internal helper: central-difference Jacobian of a 1-argument function
// ---------------------------------------------------------------------------
static Eigen::MatrixXd numJacImpl(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &func,
    const Eigen::VectorXd &x0,
    double eps_scale)
{
    const Eigen::VectorXd f0 = func(x0);
    const int nx = static_cast<int>(x0.size());
    const int ny = static_cast<int>(f0.size());

    Eigen::MatrixXd J(ny, nx);
    Eigen::VectorXd xp = x0, xm = x0;

    for (int i = 0; i < nx; ++i)
    {
        const double h = eps_scale * std::max(std::abs(x0(i)), 1.0);
        xp(i) = x0(i) + h;
        xm(i) = x0(i) - h;
        J.col(i) = (func(xp) - func(xm)) / (2.0 * h);
        xp(i) = x0(i);
        xm(i) = x0(i);
    }
    return J;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

Eigen::MatrixXd jacobianX(const StateFunc &func,
                           const Eigen::VectorXd &x0,
                           const Eigen::VectorXd &u0,
                           double eps_scale)
{
    auto f_fixed_u = [&](const Eigen::VectorXd &x) { return func(x, u0); };
    return numJacImpl(f_fixed_u, x0, eps_scale);
}

Eigen::MatrixXd jacobianU(const StateFunc &func,
                           const Eigen::VectorXd &x0,
                           const Eigen::VectorXd &u0,
                           double eps_scale)
{
    auto f_fixed_x = [&](const Eigen::VectorXd &u) { return func(x0, u); };
    return numJacImpl(f_fixed_x, u0, eps_scale);
}

StateSpace lineariseAtPoint(const StateFunc &f_continuous,
                             const MeasFunc  &h,
                             const Eigen::VectorXd &x0,
                             const Eigen::VectorXd &u0,
                             double Ts,
                             double eps_scale)
{
    // Continuous-time Jacobians
    const Eigen::MatrixXd A_c = jacobianX(f_continuous, x0, u0, eps_scale);
    const Eigen::MatrixXd B_c = jacobianU(f_continuous, x0, u0, eps_scale);

    // Measurement Jacobians  (reuse numJacImpl treating h as a StateFunc)
    auto hx = [&](const Eigen::VectorXd &x) { return h(x, u0); };
    auto hu = [&](const Eigen::VectorXd &u) { return h(x0, u); };
    const Eigen::MatrixXd C_c = numJacImpl(hx, x0, eps_scale);
    const Eigen::MatrixXd D_c = numJacImpl(hu, u0, eps_scale);

    // Build continuous-time model (Ts = 0 signals continuous-time to c2d)
    StateSpace sys_c(A_c, B_c, C_c, D_c, 0.0);

    // Discretise via ZOH
    return c2d(sys_c, Ts, C2dMethod::ZOH);
}

StateSpace lineariseAtPoint(const StateFunc &f_continuous,
                             const Eigen::VectorXd &x0,
                             const Eigen::VectorXd &u0,
                             double Ts,
                             double eps_scale)
{
    // Identity output: y = x  ->  C = I_n, D = 0_{n*m}
    const int n = static_cast<int>(x0.size());

    MeasFunc h_identity = [n](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        return x.head(n);
    };

    return lineariseAtPoint(f_continuous, h_identity, x0, u0, Ts, eps_scale);
}

} // namespace ctrl
