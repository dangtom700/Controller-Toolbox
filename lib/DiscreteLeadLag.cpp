#include "DiscreteLeadLag.h"
#include <cmath>

namespace ctrl
{

    DiscreteLeadLag::DiscreteLeadLag(const LeadLagParams &params, double sampleTime)
        : p_(params), Ts_(sampleTime)
    {
        computeCoeffs();
        reset();
    }

    // Tustin bilinear discretisation of C(s) = K.(s+z_c)/(s+p_c):
    //
    //   b₀ = K.(2/Ts + z_c) / (2/Ts + p_c)
    //   b₁ = K.(z_c - 2/Ts) / (2/Ts + p_c)
    //   a₁ = (p_c - 2/Ts)   / (2/Ts + p_c)
    //
    //   y[k] = b₀.u[k] + b₁.u[k-1] - a₁.y[k-1]
    void DiscreteLeadLag::computeCoeffs()
    {
        const double two_over_Ts = 2.0 / Ts_;
        const double denom = two_over_Ts + p_.continuousPole;
        b0_ = p_.gain * (two_over_Ts + p_.continuousZero) / denom;
        b1_ = p_.gain * (p_.continuousZero - two_over_Ts) / denom;
        a1_ = (p_.continuousPole - two_over_Ts) / denom;
    }

    double DiscreteLeadLag::compute(double u)
    {
        if (!std::isfinite(u))
            return y_prev_;

        const double y = b0_ * u + b1_ * u_prev_ - a1_ * y_prev_;
        u_prev_ = u;
        y_prev_ = y;
        return y;
    }

    void DiscreteLeadLag::reset()
    {
        u_prev_ = 0.0;
        y_prev_ = 0.0;
    }

    void DiscreteLeadLag::setParams(const LeadLagParams &p)
    {
        p_ = p;
        computeCoeffs();
    }

    // Phase of C(jomega) = K.(jomega+z_c)/(jomega+p_c)
    // ∠C(jomega) = atan2(omega, z_c) - atan2(omega, p_c)  [radians]
    double DiscreteLeadLag::phaseAt(double omega) const
    {
        return std::atan2(omega, p_.continuousZero) - std::atan2(omega, p_.continuousPole);
    }

} // namespace ctrl
