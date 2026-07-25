#include "DisturbanceObserverController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

DisturbanceObserverController::DisturbanceObserverController(std::shared_ptr<IController> inner,
                                                             const StateSpace &nominal,
                                                             const DOBParams &params,
                                                             double Ts)
    : inner_(std::move(inner)), nom_(nominal), p_(params), Ts_(Ts)
{
    if (!inner_)
        throw std::invalid_argument("DisturbanceObserverController: inner controller must not be null");
    if (Ts_ <= 0.0)
        throw std::invalid_argument("DisturbanceObserverController: Ts must be > 0");
    if (p_.omega_q <= 0.0)
        throw std::invalid_argument("DisturbanceObserverController: omega_q must be > 0");
    if (p_.qOrder != 1 && p_.qOrder != 2)
        throw std::invalid_argument("DisturbanceObserverController: qOrder must be 1 or 2");
    if (std::abs(p_.gainDC) < 1e-12)
        throw std::invalid_argument("DisturbanceObserverController: |gainDC| must be >= 1e-12");
    if (p_.dMin >= p_.dMax)
        throw std::invalid_argument("DisturbanceObserverController: dMin must be < dMax");
    if (p_.uMin >= p_.uMax)
        throw std::invalid_argument("DisturbanceObserverController: uMin must be < uMax");
    if (nom_.inputSize() != 1 || nom_.outputSize() != 1)
        throw std::invalid_argument("DisturbanceObserverController: nominal model must be SISO");
    if (nom_.stateSize() < 1)
        throw std::invalid_argument("DisturbanceObserverController: nominal model must have >= 1 state");

    // ZOH-discretised unity-DC-gain low-pass: Q(s) = omega_q / (s + omega_q).
    q_a_ = std::exp(-p_.omega_q * Ts_);
    q_b_ = 1.0 - q_a_;

    x_ = Eigen::VectorXd::Zero(nom_.stateSize());
    xn_ = Eigen::VectorXd::Zero(nom_.stateSize());
    yv_ = Eigen::VectorXd::Zero(1);
}

double DisturbanceObserverController::compute(double error)
{
    if (!std::isfinite(error))
        return u_prev_; // hold last command; observer and inner loop both stall

    // Without a real measurement fall back to y ~= -error (valid at r = 0).
    const double y = has_y_ ? y_meas_ : -error;

    // -- Nominal model output driven by the previously APPLIED command --------
    yv_.noalias() = nom_.C * x_;
    y_nom_ = yv_(0) + nom_.D(0, 0) * u_prev_;

    // -- Q-filter the innovation ---------------------------------------------
    const double innov = y - y_nom_;
    q1_ = q_a_ * q1_ + q_b_ * innov;
    double d_out = q1_;
    if (p_.qOrder == 2)
    {
        q2_ = q_a_ * q2_ + q_b_ * q1_;
        d_out = q2_;
    }

    // Output-referred disturbance -> input-referred via the nominal DC gain.
    d_hat_ = std::clamp(d_out / p_.gainDC, p_.dMin, p_.dMax);

    // -- Inner feedback plus cancellation ------------------------------------
    const double u_fb = inner_->compute(error);
    double u = std::clamp(u_fb - d_hat_, p_.uMin, p_.uMax);
    if (!std::isfinite(u))
        u = u_prev_;

    // -- Advance the nominal model with the command actually applied ---------
    xn_.noalias() = nom_.A * x_;
    xn_ += nom_.B.col(0) * u;
    x_.swap(xn_);

    u_prev_ = u;

    notify_buf_(0) = d_hat_;
    notifyObserverState("d_hat", notify_buf_);
    notifyObserver(u, error);
    return u;
}

void DisturbanceObserverController::reset()
{
    inner_->reset();
    q1_ = 0.0;
    q2_ = 0.0;
    x_.setZero();
    y_meas_ = 0.0;
    has_y_ = false;
    y_nom_ = 0.0;
    d_hat_ = 0.0;
    u_prev_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
