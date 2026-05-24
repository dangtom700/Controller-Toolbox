#include "DiscreteSMC.h"
#include <algorithm>
#include <cmath>

namespace ctrl
{

    DiscreteSMC::DiscreteSMC(const SMCParams &params, double sampleTime)
        : p_(params), Ts_(sampleTime)
    {
        reset();
    }

    // Sliding surface: s[k] = c_e.e[k] + c_de.(e[k] - e[k-1])
    //
    // Boundary layer saturation avoids chattering:
    //   |s| <= phi  ->  continuous PD law (sat = s/phi)
    //   |s| > phi  ->  full relay switching  (sat = sign(s))
    double DiscreteSMC::compute(double error)
    {
        if (!std::isfinite(error))
            return u_prev_;

        const double s = p_.c_e * error + p_.c_de * (error - e_prev_);

        double sat_val;
        if (p_.phi > 1e-12)
            sat_val = std::max(-1.0, std::min(1.0, s / p_.phi));
        else
            sat_val = (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);

        double u = -p_.K * sat_val;
        u = std::max(p_.uMin, std::min(p_.uMax, u));

        e_prev_ = error;
        s_prev_ = s;
        u_prev_ = u;
        return u;
    }

    void DiscreteSMC::reset()
    {
        e_prev_ = 0.0;
        s_prev_ = 0.0;
        u_prev_ = 0.0;
    }

    // ---------------------------------------------------------------------------
    // SuperTwistingSMC
    // ---------------------------------------------------------------------------
    SuperTwistingSMC::SuperTwistingSMC(const SuperTwistingParams &params, double sampleTime)
        : p_(params), Ts_(sampleTime)
    {
        reset();
    }

    // Discrete super-twisting update:
    //   s[k]   = c_e.e[k] + c_de.(e[k] - e[k-1])
    //   u[k]   = v_ - K1.|s|^{1/2}.sign(s)          (power term - chattering-free)
    //   v_new  = v_ - K2.Ts.sign(s)                  (integral term - Euler)
    //
    // The integrator v_ provides the equivalent of a sliding-mode integral that
    // removes steady-state error without a boundary layer.
    double SuperTwistingSMC::compute(double error)
    {
        if (!std::isfinite(error))
            return std::max(p_.uMin, std::min(p_.uMax, v_));

        const double s = p_.c_e * error + p_.c_de * (error - e_prev_);

        const double sign_s = (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);
        const double sqrt_s = std::sqrt(std::abs(s));

        // Control output
        double u = v_ - p_.K1 * sqrt_s * sign_s;
        u = std::max(p_.uMin, std::min(p_.uMax, u));

        // Integrator update (Euler - keep K2*Ts small for accuracy)
        v_ -= p_.K2 * Ts_ * sign_s;

        e_prev_ = error;
        s_prev_ = s;
        return u;
    }

    void SuperTwistingSMC::reset()
    {
        e_prev_ = 0.0;
        s_prev_ = 0.0;
        v_      = 0.0;
    }

} // namespace ctrl
