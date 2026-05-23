#include "DiscreteADRC.h"
#include <algorithm>
#include <iostream>

namespace ctrl
{

    DiscreteADRC::DiscreteADRC(const ADRCParams &params, double sampleTime)
        : p_(params), Ts_(sampleTime), r_(0.0), u_prev_(0.0)
    {
        // Forward-Euler ESO is stable only when omega_o * Ts < 2.
        // Violating this pushes the observer poles outside the unit circle.
        if (params.omega_o * sampleTime >= 2.0)
        {
            std::cerr << "[DiscreteADRC] WARNING: ESO bandwidth omega_o=" << params.omega_o
                      << " with Ts=" << sampleTime
                      << " violates Forward-Euler stability (omega_o * Ts must be < 2). "
                      << "Reduce omega_o or decrease the sample time.\n";
        }
        updateGains();
        reset();
    }

    void DiscreteADRC::updateGains()
    {
        beta1_ = 3.0 * p_.omega_o;
        beta2_ = 3.0 * p_.omega_o * p_.omega_o;
        beta3_ = p_.omega_o * p_.omega_o * p_.omega_o;
    }

    void DiscreteADRC::setParams(const ADRCParams &p)
    {
        p_ = p;
        updateGains();
    }

    // ESO forward-Euler update (Gao 2003 bandwidth-parameterisation):
    //   epsilon    = y - z₁
    //   z₁  += Ts.(z₂ + beta₁.epsilon)
    //   z₂  += Ts.(z₃ + beta₂.epsilon + b₀.u)
    //   z₃  += Ts.beta₃.epsilon
    //
    // PD + disturbance cancellation:
    //   u₀   = omega_c^2.(r - z₁) - 2.omega_c.z₂
    //   u    = (u₀ - z₃) / b₀
    double DiscreteADRC::computeTracking(double y, double r)
    {
        if (!std::isfinite(y) || !std::isfinite(r))
            return u_prev_;

        // ESO observation error
        const double eps = y - z_(0);

        // Observer update (forward Euler)
        const double z1n = z_(0) + Ts_ * (z_(1) + beta1_ * eps);
        const double z2n = z_(1) + Ts_ * (z_(2) + beta2_ * eps + p_.b0 * u_prev_);
        const double z3n = z_(2) + Ts_ * beta3_ * eps;
        z_ << z1n, z2n, z3n;

        // PD setpoint control
        const double u0 = p_.omega_c * p_.omega_c * (r - z_(0)) - 2.0 * p_.omega_c * z_(1);

        // Active disturbance cancellation
        double u = (u0 - z_(2)) / p_.b0;
        u = std::max(p_.uMin, std::min(p_.uMax, u));

        u_prev_ = u;
        return u;
    }

    double DiscreteADRC::compute(double y)
    {
        return computeTracking(y, r_);
    }

    void DiscreteADRC::reset()
    {
        z_.setZero();
        u_prev_ = 0.0;
        r_ = 0.0;
    }

} // namespace ctrl
