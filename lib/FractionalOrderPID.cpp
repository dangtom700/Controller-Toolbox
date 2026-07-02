#include "FractionalOrderPID.h"
#include <algorithm>
#include <cmath>

namespace ctrl
{

    // -------------------------------------------------------------------------
    // FractionalDifferintegrator - Oustaloup recursive approximation of s^{alpha}
    // -------------------------------------------------------------------------
    //
    // Continuous Oustaloup filter (Oustaloup et al. 2000), for k = -N .. N:
    //   zero_k = wb.(wh/wb)^{ (k + N + 0.5(1 - alpha)) / (2N + 1) }
    //   pole_k = wb.(wh/wb)^{ (k + N + 0.5(1 + alpha)) / (2N + 1) }
    //   K      = wh^{alpha}
    //   s^{alpha} ~= K . prod_k (s + zero_k) / (s + pole_k)
    //
    // Each first-order section (s + z)/(s + p) is discretised with the bilinear (Tustin)
    // transform s = c.(1 - z^-1)/(1 + z^-1), c = 2/Ts:
    //   H(z) = ( (c + z) + (z - c) z^-1 ) / ( (c + p) + (p - c) z^-1 )
    // Normalising the leading denominator coefficient to 1 gives the difference equation
    //   y[k] = b0.x[k] + b1.x[k-1] - a1.y[k-1].
    void FractionalDifferintegrator::build(double alpha, double wb, double wh, int N,
                                           double sampleTime)
    {
        if (N < 1)          N  = 1;
        if (!(wb > 0.0))    wb = 1e-6;
        if (!(wh > wb))     wh = wb * 1e4;
        const double Ts = (sampleTime > 0.0) ? sampleTime : 1.0;

        sec_.clear();
        sec_.reserve(static_cast<std::size_t>(2 * N + 1));

        const double logr = std::log(wh / wb);
        const double denom = static_cast<double>(2 * N + 1);
        const double c = 2.0 / Ts;

        for (int k = -N; k <= N; ++k)
        {
            const double kk = static_cast<double>(k + N);
            const double z  = wb * std::exp(logr * (kk + 0.5 * (1.0 - alpha)) / denom); // zero
            const double p  = wb * std::exp(logr * (kk + 0.5 * (1.0 + alpha)) / denom); // pole

            const double a0 = c + p;
            Section s;
            s.b0 = (c + z) / a0;
            s.b1 = (z - c) / a0;
            s.a1 = (p - c) / a0;
            s.x1 = 0.0;
            s.y1 = 0.0;
            sec_.push_back(s);
        }

        K_ = std::pow(wh, alpha);
    }

    double FractionalDifferintegrator::compute(double x)
    {
        // Hold-last on a non-finite input: a NaN would otherwise propagate into every
        // section's state (y1) and poison the IIR cascade permanently.
        if (!std::isfinite(x))
            return sec_.empty() ? 0.0 : sec_.back().y1;

        // Overall gain first, then cascade the (unit-high-frequency-gain) sections.
        double v = K_ * x;
        for (Section &s : sec_)
        {
            const double y = s.b0 * v + s.b1 * s.x1 - s.a1 * s.y1;
            s.x1 = v;
            s.y1 = y;
            v = y;
        }
        return v;
    }

    void FractionalDifferintegrator::reset()
    {
        for (Section &s : sec_)
        {
            s.x1 = 0.0;
            s.y1 = 0.0;
        }
    }

    // -------------------------------------------------------------------------
    // FractionalOrderPID
    // -------------------------------------------------------------------------
    FractionalOrderPID::FractionalOrderPID(const FOPIDParams &params, double sampleTime)
        : p_(params), Ts_(sampleTime)
    {
        // s^{-lambda} integrator and s^{mu} differentiator over the same Oustaloup band.
        fi_.build(-p_.lambda, p_.wb, p_.wh, p_.N, Ts_);
        fd_.build(p_.mu, p_.wb, p_.wh, p_.N, Ts_);
        reset();
    }

    // u[k] = Kp.e + Ki.(D^{-lambda} e) + Kd.(D^{mu} e), saturated, with back-calculation
    // anti-windup fed into the fractional-integral branch input.
    double FractionalOrderPID::compute(double error)
    {
        if (!std::isfinite(error))
            return u_prev_;

        // Anti-windup: bias the integrator input by the previous saturation error.
        const double i_in    = error + p_.Kaw * aw_prev_;
        const double i_branch = fi_.compute(i_in);
        const double d_branch = fd_.compute(error);

        const double u_unsat = p_.Kp * error + p_.Ki * i_branch + p_.Kd * d_branch;
        const double u_sat   = std::max(p_.uMin, std::min(p_.uMax, u_unsat));

        aw_prev_ = u_sat - u_unsat; // 0 when unsaturated
        u_prev_  = u_sat;

        notifyObserver(u_sat, error);
        return u_sat;
    }

    void FractionalOrderPID::reset()
    {
        fi_.reset();
        fd_.reset();
        u_prev_  = 0.0;
        aw_prev_ = 0.0;
        notifyObserverReset();
    }

    void FractionalOrderPID::setParams(const FOPIDParams &params)
    {
        p_ = params;
        fi_.build(-p_.lambda, p_.wb, p_.wh, p_.N, Ts_);
        fd_.build(p_.mu, p_.wb, p_.wh, p_.N, Ts_);
        reset();
    }

} // namespace ctrl
