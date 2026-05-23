#include "RecursiveLeastSquares.h"

namespace ctrl
{

    RecursiveLeastSquares::RecursiveLeastSquares(int na, int nb, double Ts,
                                                 double lambda, double P0_scale)
        : na_(na), nb_(nb), ntheta_(na + nb), Ts_(Ts), lambda_(lambda), P0_scale_(P0_scale), k_(0)
    {
        theta_ = Eigen::VectorXd::Zero(ntheta_);
        P_ = P0_scale * Eigen::MatrixXd::Identity(ntheta_, ntheta_);
        y_buf_ = Eigen::VectorXd::Zero(na);
        u_buf_ = Eigen::VectorXd::Zero(nb);
    }

    double RecursiveLeastSquares::update(double y, double u)
    {
        // Build regressor φ[k] = [-y[k-1],...,-y[k-na], u[k-1],...,u[k-nb]]
        // using the current (pre-update) buffers
        Eigen::VectorXd phi(ntheta_);
        for (int i = 0; i < na_; ++i) phi(i)       = -y_buf_(i);
        for (int i = 0; i < nb_; ++i) phi(na_ + i) =  u_buf_(i);

        // Prediction error
        const double e = y - phi.dot(theta_);

        // Kalman gain: K = P.φ / (lambda + φ'.P.φ)
        const Eigen::VectorXd Pphi = P_ * phi;
        const double denom = lambda_ + phi.dot(Pphi);
        const Eigen::VectorXd K = Pphi / denom;

        // Parameter update
        theta_ += K * e;

        // Covariance update
        P_ = (P_ - K * Pphi.transpose()) / lambda_;

        // Symmetrise to suppress drift
        P_ = 0.5 * (P_ + P_.transpose());

        // Shift buffers: push current y,u to front (index 0 = most recent lag)
        for (int i = na_ - 1; i > 0; --i) y_buf_(i) = y_buf_(i - 1);
        if (na_ > 0) y_buf_(0) = y;
        for (int i = nb_ - 1; i > 0; --i) u_buf_(i) = u_buf_(i - 1);
        if (nb_ > 0) u_buf_(0) = u;

        ++k_;
        return e;
    }

    void RecursiveLeastSquares::reset()
    {
        theta_.setZero();
        P_ = P0_scale_ * Eigen::MatrixXd::Identity(ntheta_, ntheta_);
        y_buf_.setZero();
        u_buf_.setZero();
        k_ = 0;
    }

    Eigen::VectorXd RecursiveLeastSquares::denominator() const
    {
        // [1, a1, a2, ..., ana]
        Eigen::VectorXd den(na_ + 1);
        den(0) = 1.0;
        den.tail(na_) = theta_.head(na_);
        return den;
    }

    Eigen::VectorXd RecursiveLeastSquares::numerator() const
    {
        // [b1, b2, ..., bnb]
        return theta_.tail(nb_);
    }

    TransferFunction RecursiveLeastSquares::toTransferFunction() const
    {
        // num is [0, b1,...,bnb] padded/trimmed to length na_+1 to align degrees
        Eigen::VectorXd num_e = Eigen::VectorXd::Zero(na_ + 1);
        const int copy_len = std::min(nb_, na_);
        num_e.segment(1, copy_len) = theta_.segment(na_, copy_len);

        const Eigen::VectorXd den_e = denominator();

        // Convert Eigen vectors to std::vector<double> for TransferFunction ctor
        std::vector<double> num_v(num_e.data(), num_e.data() + num_e.size());
        std::vector<double> den_v(den_e.data(), den_e.data() + den_e.size());
        return TransferFunction(num_v, den_v, Ts_);
    }

    StateSpace RecursiveLeastSquares::toStateSpace() const
    {
        return tf2ss(toTransferFunction());
    }

} // namespace ctrl
