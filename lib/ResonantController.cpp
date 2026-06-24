#include "ResonantController.h"
#include <cmath>
#include <stdexcept>

namespace ctrl
{

ResonantController::ResonantController(const ResonantParams &params, double Ts)
    : p_(params), Ts_(Ts)
{
    if (p_.targetFreqHz <= 0.0)
        throw std::invalid_argument("ResonantController: targetFreqHz must be positive");
    if (p_.dampingRadPerSec <= 0.0)
        throw std::invalid_argument("ResonantController: dampingRadPerSec must be positive");
    if (p_.targetFreqHz >= 1.0 / (2.0 * Ts_))
        throw std::invalid_argument("ResonantController: targetFreqHz must be below the Nyquist frequency");
    computeCoeffs();
    reset();
}

// Non-ideal resonant filter G_RC(s) = 2*Kr*wc*s / (s^2 + 2*wc*s + w0^2), discretised via Tustin
// with prewarping at w0 so the digital resonance peak lands exactly at targetFreqHz:
//
//   K          = 2/Ts
//   w0         = 2*pi*targetFreqHz
//   w0_warped  = K*tan(w0*Ts/2)                  (prewarp)
//   a0         = w0_warped^2
//   a1c        = 2*dampingRadPerSec
//   b1c        = 2*Kr*dampingRadPerSec
//   D0         = K^2 + a1c*K + a0
//
//   b0 =  b1c*K / D0
//   b2 = -b0           (numerator of the bilinear-transformed system is odd: b1c*K*(z^2-1))
//   a1 =  (2*a0 - 2*K^2) / D0
//   a2 =  (K^2 - a1c*K + a0) / D0
//
//   u[k] = b0*e[k] + b2*e[k-2] - a1*u[k-1] - a2*u[k-2]   (the b1 term is exactly zero)
void ResonantController::computeCoeffs()
{
    const double K  = 2.0 / Ts_;
    const double w0 = 2.0 * M_PI * p_.targetFreqHz;
    const double w0_warped = K * std::tan(w0 * Ts_ / 2.0);
    const double a0  = w0_warped * w0_warped;
    const double a1c = 2.0 * p_.dampingRadPerSec;
    const double b1c = 2.0 * p_.Kr * p_.dampingRadPerSec;
    const double D0  = K * K + a1c * K + a0;

    b0_ = (b1c * K) / D0;
    b2_ = -b0_;
    a1_ = (2.0 * a0 - 2.0 * K * K) / D0;
    a2_ = (K * K - a1c * K + a0) / D0;
}

double ResonantController::compute(double error)
{
    if (!std::isfinite(error))
        return u_prev1_;

    double u = b0_ * error + b2_ * e_prev2_ - a1_ * u_prev1_ - a2_ * u_prev2_;
    if (u > p_.uMax) u = p_.uMax;
    if (u < p_.uMin) u = p_.uMin;

    e_prev2_ = e_prev1_;
    e_prev1_ = error;
    u_prev2_ = u_prev1_;
    u_prev1_ = u;
    return u;
}

void ResonantController::reset()
{
    e_prev1_ = e_prev2_ = 0.0;
    u_prev1_ = u_prev2_ = 0.0;
}

void ResonantController::setParams(const ResonantParams &p)
{
    p_ = p;
    computeCoeffs();
}

} // namespace ctrl
