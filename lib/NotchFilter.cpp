#include "NotchFilter.h"
#include <cmath>
#include <stdexcept>

namespace ctrl
{

NotchFilter::NotchFilter(const NotchFilterParams &params, double Ts)
    : p_(params), Ts_(Ts)
{
    if (p_.centerFreqHz <= 0.0)
        throw std::invalid_argument("NotchFilter: centerFreqHz must be positive");
    if (p_.Q <= 0.0)
        throw std::invalid_argument("NotchFilter: Q must be positive");
    if (p_.centerFreqHz >= 1.0 / (2.0 * Ts_))
        throw std::invalid_argument("NotchFilter: centerFreqHz must be below the Nyquist frequency");
    computeCoeffs();
    reset();
}

void NotchFilter::computeCoeffs()
{
    const double omega = 2.0 * M_PI * p_.centerFreqHz * Ts_;
    const double alpha = std::sin(omega) / (2.0 * p_.Q);
    const double cos_omega = std::cos(omega);
    const double a0 = 1.0 + alpha;

    b0_ = 1.0 / a0;
    b1_ = (-2.0 * cos_omega) / a0;
    b2_ = 1.0 / a0;
    a1_ = (-2.0 * cos_omega) / a0;
    a2_ = (1.0 - alpha) / a0;
}

double NotchFilter::apply(double x)
{
    if (!std::isfinite(x))
        return y_prev1_;

    const double y = b0_ * x + b1_ * x_prev1_ + b2_ * x_prev2_
                    - a1_ * y_prev1_ - a2_ * y_prev2_;

    x_prev2_ = x_prev1_;
    x_prev1_ = x;
    y_prev2_ = y_prev1_;
    y_prev1_ = y;
    return y;
}

void NotchFilter::reset()
{
    x_prev1_ = x_prev2_ = 0.0;
    y_prev1_ = y_prev2_ = 0.0;
}

void NotchFilter::setParams(const NotchFilterParams &p)
{
    p_ = p;
    computeCoeffs();
}

} // namespace ctrl
