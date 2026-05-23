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
    //   |s| <= φ  ->  continuous PD law (sat = s/φ)
    //   |s| > φ  ->  full relay switching  (sat = sign(s))
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

} // namespace ctrl
