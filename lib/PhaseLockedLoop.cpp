#include "PhaseLockedLoop.h"
#include <cmath>
#include <stdexcept>

namespace ctrl
{

PhaseLockedLoop::PhaseLockedLoop(const PLLParams &params, double Ts)
    : p_(params), Ts_(Ts)
{
    if (p_.nominalFreqHz <= 0.0)
        throw std::invalid_argument("PhaseLockedLoop: nominalFreqHz must be positive");
    if (p_.nominalFreqHz >= 1.0 / (2.0 * Ts_))
        throw std::invalid_argument("PhaseLockedLoop: nominalFreqHz must be below the Nyquist frequency");
    reset();
}

void PhaseLockedLoop::step(double sample)
{
    if (!std::isfinite(sample))
        return;

    // SOGI quadrature generator (forward-Euler), tuned to the current frequency estimate:
    //   dx1/dt = w_hat*x2 + sogiK*w_hat*(v_in - x1)
    //   dx2/dt = -w_hat*x1
    const double x1_old = x1_;
    const double x2_old = x2_;
    x1_ = x1_old + Ts_ * (w_hat_ * x2_old + p_.sogiK * w_hat_ * (sample - x1_old));
    x2_ = x2_old + Ts_ * (-w_hat_ * x1_old);

    // Error signal: v_q = x1*cos(theta_hat) - x2*sin(theta_hat) = A*sin(theta_true-theta_hat),
    // vanishing at lock. See the header's derivation note for why this pairing/sign (not the
    // naive cos/-sin one) is the one that's actually correct.
    const double cosT = std::cos(theta_hat_);
    const double sinT = std::sin(theta_hat_);
    const double v_q = x1_ * cosT - x2_ * sinT;

    // PI loop filter, direct (no extra negation - see derivation note).
    integral_ += p_.Ki * v_q * Ts_;
    const double delta_w = p_.Kp * v_q + integral_;

    // NCO: corrected frequency drives the phase integrator, wrapped to [-pi, pi).
    w_hat_ = 2.0 * M_PI * p_.nominalFreqHz + delta_w;
    theta_hat_ += w_hat_ * Ts_;
    theta_hat_ = std::atan2(std::sin(theta_hat_), std::cos(theta_hat_));

    // Lock detection: |v_q| small relative to amplitude for kLockCountRequired consecutive samples.
    const double amp = std::sqrt(x1_ * x1_ + x2_ * x2_);
    if (std::fabs(v_q) < 0.02 * amp)
    {
        if (lockCounter_ < kLockCountRequired) ++lockCounter_;
    }
    else
    {
        lockCounter_ = 0;
    }
}

double PhaseLockedLoop::amplitude() const
{
    return std::sqrt(x1_ * x1_ + x2_ * x2_);
}

void PhaseLockedLoop::reset()
{
    x1_ = x2_ = 0.0;
    integral_ = 0.0;
    theta_hat_ = 0.0;
    w_hat_ = 2.0 * M_PI * p_.nominalFreqHz;
    lockCounter_ = 0;
}

} // namespace ctrl
