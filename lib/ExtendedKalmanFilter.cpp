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
        : n_states_(n), n_outputs_(p),
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
        //
        // Stability assumption: f_ must be a well-conditioned discrete-time map.
        // If f_ is internally a forward-Euler step at the control Ts, accuracy and
        // stability degrade for stiff plants (e.g. hydraulic actuators with high bulk
        // modulus, electrical systems with fast electrical time constants).  For such
        // plants, implement f_ with RK4 or a fixed-step ODE solver at a sub-step Ts/N.
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
            P_ * H.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(n_outputs_, n_outputs_));

        x_hat_.noalias() += K * (y - h_(x_hat_, u));

        // Joseph form: P = (I-KH).P.(I-KH)' + K.R.K'
        const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n_states_, n_states_) - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R_safe * K.transpose();

        // P3: project algebraic states onto constraint manifold (if constraint attached)
        if (alg_set_)
            projectAlgebraicStates(u);
    }

    // ---------------------------------------------------------------------------
    // setAlgebraicConstraint - P3: attach DAE algebraic projection
    // ---------------------------------------------------------------------------
    void ExtendedKalmanFilter::setAlgebraicConstraint(
        AlgConstraintFn g_alg, int n_diff, int n_alg, double newton_tol)
    {
        if (n_diff + n_alg != n_states_)
            throw std::invalid_argument(
                "EKF::setAlgebraicConstraint: n_diff + n_alg must equal EKF state dimension.");
        if (n_diff <= 0 || n_alg <= 0)
            throw std::invalid_argument(
                "EKF::setAlgebraicConstraint: both n_diff and n_alg must be > 0.");
        alg_g_     = std::move(g_alg);
        n_diff_alg_ = n_diff;
        n_alg_     = n_alg;
        alg_tol_   = newton_tol;
        alg_set_   = true;
    }

    // ---------------------------------------------------------------------------
    // projectAlgebraicStates - Newton-project x2 block, then project covariance.
    //
    // Newton iterations: x2 <- x2 - G2^{-1} * g(x1, x2, u)
    // Covariance projection: P <- J_proj * P * J_proj'
    //   where J_proj = [[I_{n1}, 0]; [-G2^{-1}*G1, 0_{n2}]]
    // ---------------------------------------------------------------------------
    void ExtendedKalmanFilter::projectAlgebraicStates(const Eigen::VectorXd &u)
    {
        const int  n1      = n_diff_alg_;
        const int  n2      = n_alg_;
        const double u_s   = u.size() > 0 ? u(0) : 0.0; // SISO scalar input

        Eigen::VectorXd x1 = x_hat_.head(n1);
        Eigen::VectorXd x2 = x_hat_.tail(n2);

        // Newton-Raphson: solve g(x1, x2, u) = 0 for x2
        for (int iter = 0; iter < 20; ++iter)
        {
            const Eigen::VectorXd g_val = alg_g_(x1, x2, u_s);
            if (g_val.norm() < alg_tol_) break;

            // Central-difference dg/dx2 (n2 x n2)
            const Eigen::MatrixXd G2 = numericalJacobian(
                [&](const Eigen::VectorXd &x2v) { return alg_g_(x1, x2v, u_s); }, x2);

            const auto ldlt = G2.ldlt();
            if (ldlt.info() != Eigen::Success) break;
            x2 -= ldlt.solve(g_val);
        }
        x_hat_.tail(n2) = x2;

        // Covariance projection: J_proj = [[I, 0]; [-G2^{-1}*G1, 0]]
        // Numerically compute G1 = dg/dx1 and G2 = dg/dx2 at the projected point
        const Eigen::MatrixXd G1 = numericalJacobian(
            [&](const Eigen::VectorXd &x1v) { return alg_g_(x1v, x2, u_s); }, x1); // n2 x n1
        const Eigen::MatrixXd G2 = numericalJacobian(
            [&](const Eigen::VectorXd &x2v) { return alg_g_(x1, x2v, u_s); }, x2); // n2 x n2

        auto ldlt_g2 = G2.ldlt();
        if (ldlt_g2.info() != Eigen::Success) {
            dae_projection_failed_ = true;
            return; // leave P_ unchanged rather than corrupt it with a garbage solve
        }
        Eigen::MatrixXd G2inv_G1 = ldlt_g2.solve(G1); // n2 x n1

        Eigen::MatrixXd J_proj = Eigen::MatrixXd::Zero(n1 + n2, n1 + n2);
        J_proj.topLeftCorner(n1, n1)    = Eigen::MatrixXd::Identity(n1, n1);
        J_proj.bottomLeftCorner(n2, n1) = -G2inv_G1;
        // bottomRightCorner remains zero: x2 variance explained by x1 variance via constraint

        P_ = J_proj * P_ * J_proj.transpose();
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
        P_ = Eigen::MatrixXd::Identity(n_states_, n_states_);
    }

    Eigen::MatrixXd ExtendedKalmanFilter::numericalJacobian(
        const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &func,
        const Eigen::VectorXd &x,
        double eps_scale)
    {
        // Central-difference: J(:,i) = (f(x+h_i*ei) - f(x-h_i*ei)) / (2*h_i)
        // h_i = eps_scale * max(|x_i|, 1) gives O(eps_scale^2) truncation error regardless
        // of state magnitude - essential for heterogeneous state vectors (positions, angles,
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
