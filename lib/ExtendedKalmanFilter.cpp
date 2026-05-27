#include "ExtendedKalmanFilter.h"

namespace ctrl
{

    ExtendedKalmanFilter::ExtendedKalmanFilter(
        int n, int p,
        StateFunc f, MeasFunc h,
        JacobianFn Fjac, JacobianFn Hjac,
        const Eigen::MatrixXd &Q_noise,
        const Eigen::MatrixXd &R_noise,
        double Ts,
        const Eigen::MatrixXd &P0)
        : n_(n), p_(p),
          f_(std::move(f)), h_(std::move(h)),
          Fjac_(std::move(Fjac)), Hjac_(std::move(Hjac)),
          Q_(Q_noise), R_(R_noise), Ts_(Ts)
    {
        x_hat_ = Eigen::VectorXd::Zero(n);
        P_ = P0.rows() == n ? P0 : Eigen::MatrixXd::Identity(n, n);
    }

    void ExtendedKalmanFilter::predict(const Eigen::VectorXd &u)
    {
        // Linearise f around x^[k|k] to get the state Jacobian F = df/dx.
        // F is used only for the covariance propagation P = F*P*F' + Q; the
        // state itself is propagated through the exact nonlinear function f.
        const Eigen::MatrixXd F = Fjac_(x_hat_, u);
        x_hat_ = f_(x_hat_, u);
        P_ = F * P_ * F.transpose() + Q_;
    }

    void ExtendedKalmanFilter::update(const Eigen::VectorXd &y,
                                      const Eigen::VectorXd &u)
    {
        // Linearise h around x^[k+1|k] (the post-predict estimate) to get H = dh/dx.
        // H is evaluated at the predicted state, which is the current x_hat_ after predict().
        const Eigen::MatrixXd H = Hjac_(x_hat_, u);

        Eigen::MatrixXd R_safe = R_;
        R_safe.diagonal() = R_safe.diagonal().cwiseMax(1e-12);

        const Eigen::MatrixXd S = H * P_ * H.transpose() + R_safe;
        const auto ldlt = S.ldlt();
        if (ldlt.info() != Eigen::Success)
            return;

        const Eigen::MatrixXd K =
            P_ * H.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(p_, p_));

        x_hat_.noalias() += K * (y - h_(x_hat_, u));

        // Joseph form: P = (I-KH).P.(I-KH)' + K.R.K'
        const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n_, n_) - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R_safe * K.transpose();
    }

    void ExtendedKalmanFilter::step(const Eigen::VectorXd &y,
                                    const Eigen::VectorXd &u_prev)
    {
        predict(u_prev);
        update(y, u_prev);
    }

    void ExtendedKalmanFilter::reset()
    {
        x_hat_.setZero();
        P_ = Eigen::MatrixXd::Identity(n_, n_);
    }

    Eigen::MatrixXd ExtendedKalmanFilter::numericalJacobian(
        const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &func,
        const Eigen::VectorXd &x,
        double eps_scale)
    {
        // Central-difference: J(:,i) = (f(x+h_i*ei) - f(x-h_i*ei)) / (2*h_i)
        // h_i = eps_scale * max(|x_i|, 1) gives O(eps_scale^2) truncation error regardless
        // of state magnitude — essential for heterogeneous state vectors (positions, angles,
        // velocities may differ by orders of magnitude).
        const Eigen::VectorXd f0 = func(x);
        const int nx = x.size();
        const int ny = f0.size();
        Eigen::MatrixXd J(ny, nx);
        Eigen::VectorXd xp = x, xm = x;
        for (int i = 0; i < nx; ++i)
        {
            const double h = eps_scale * std::max(std::abs(x(i)), 1.0);
            xp(i) = x(i) + h;
            xm(i) = x(i) - h;
            J.col(i) = (func(xp) - func(xm)) / (2.0 * h);
            xp(i) = x(i);
            xm(i) = x(i);
        }
        return J;
    }

} // namespace ctrl
