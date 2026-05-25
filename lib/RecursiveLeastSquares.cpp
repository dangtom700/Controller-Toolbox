#include "RecursiveLeastSquares.h"
#include <stdexcept>

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
        // Build regressor phi[k] = [-y[k-1],...,-y[k-na], u[k-1],...,u[k-nb]]
        // using the current (pre-update) buffers. Negative sign on y terms because
        // the ARX model is y[k] = -a1*y[k-1] - ... + b1*u[k-1] + ..., so theta = [a1,...,b1,...].
        Eigen::VectorXd phi(ntheta_);
        for (int i = 0; i < na_; ++i) phi(i)       = -y_buf_(i);
        for (int i = 0; i < nb_; ++i) phi(na_ + i) =  u_buf_(i);

        // A-posteriori prediction error: e = y - phi'*theta_{k-1}
        const double e = y - phi.dot(theta_);

        // RLS Kalman gain: K = P*phi / (lambda + phi'*P*phi)
        // Denominator is always > 0 (lambda > 0, phi'*P*phi >= 0, P is PSD).
        const Eigen::VectorXd Pphi = P_ * phi;
        const double denom = lambda_ + phi.dot(Pphi);
        const Eigen::VectorXd K = Pphi / denom;

        // Parameter update: theta = theta + K*e  (standard gradient-descent step)
        theta_ += K * e;

        // Covariance update: P = (P - K*phi'*P) / lambda
        // The division by lambda is the forgetting mechanism - it inflates P at each step,
        // giving more weight to recent data. Equivalent to using a sliding data window of
        // effective length ~ 1/(1-lambda). Without it (lambda=1), this is standard LS and
        // P shrinks monotonically (no tracking of time-varying parameters).
        P_ = (P_ - K * Pphi.transpose()) / lambda_;

        // Symmetrise to suppress numerical drift
        P_ = 0.5 * (P_ + P_.transpose());

        // Trace capping: when lambda < 1 (forgetting active), scalar exponential
        // forgetting inflates P isotropically - including unexcited directions -
        // causing parameter drift.  Capping trace(P) at P0_scale * ntheta prevents
        // runaway covariance growth without introducing directional bias.
        // Equivalent to "variable-trace" forgetting from Fortescue et al. (1981).
        if (lambda_ < 1.0)
        {
            const double tr     = P_.trace();
            const double tr_max = P0_scale_ * static_cast<double>(ntheta_);
            if (tr > tr_max)
                P_ *= tr_max / tr;
        }

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
        // ARX models require nb <= na so the transfer function B(z)/A(z) is proper.
        // If nb > na the extra B coefficients cannot be represented in the returned
        // TransferFunction without adding improper (non-causal) terms.
        if (nb_ > na_)
            throw std::logic_error(
                "RecursiveLeastSquares::toTransferFunction: nb (" + std::to_string(nb_) +
                ") > na (" + std::to_string(na_) + "). ARX models require nb <= na. "
                "Increase na or reduce nb.");

        // ARX B-polynomial: B(z^{-1}) = b1*z^{-1} + b2*z^{-2} + ...
        // There is no z^0 (direct feedthrough) term - input delay of at least one step.
        // TransferFunction expects num[0] = b0 (the z^0 coefficient). We must prepend 0.
        // Without this: tf2ss would set D = b1 and corrupt the C vector.
        Eigen::VectorXd num_e = Eigen::VectorXd::Zero(na_ + 1);
        num_e.segment(1, nb_) = theta_.segment(na_, nb_); // [0, b1, b2, ..., b_nb, 0, ...]

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
