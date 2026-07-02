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
        // M3 telemetry: sliding surface value
        notify_buf_(0) = s;
        notifyObserverState("surface", notify_buf_);
        notifyObserver(u, error);
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
        notifyObserver(u, error);
        return u;
    }

    void SuperTwistingSMC::reset()
    {
        e_prev_ = 0.0;
        s_prev_ = 0.0;
        v_      = 0.0;
    }

    // ---------------------------------------------------------------------------
    // NonsingularTerminalSMC
    // ---------------------------------------------------------------------------
    NonsingularTerminalSMC::NonsingularTerminalSMC(const NonsingularTerminalSMCParams &params,
                                                   double sampleTime)
        : p_(params), Ts_(sampleTime)
    {
        reset();
    }

    // Terminal sliding surface with finite-time convergence (discrete form):
    //   de = e[k] - e[k-1]                                   (raw per-step change)
    //   s  = c_e.e + (1/beta).|de|^{gamma}.sign(de),   1 < gamma < 2
    // Using the raw difference (not de/Ts) keeps the surface Ts-robust; beta absorbs the
    // sample-time scaling, matching SMCParams::c_de.
    // Reaching law (sat-smoothed, nonsingular - only positive powers of |de|):
    //   u  = -( K.sat(s/phi) + eta.s )
    double NonsingularTerminalSMC::compute(double error)
    {
        if (!std::isfinite(error))
            return u_prev_;

        const double de = error - e_prev_;
        // |de|^{gamma}.sign(de); guarded so a zero change contributes nothing (and never a
        // negative/singular power). std::pow(0, gamma) = 0 for gamma > 0, but branch anyway to
        // avoid sign(0) ambiguity.
        double term = 0.0;
        const double abs_de = std::abs(de);
        if (abs_de > 1e-12)
            term = std::pow(abs_de, p_.gamma) * ((de > 0.0) ? 1.0 : -1.0);

        const double beta = (std::abs(p_.beta) > 1e-12) ? p_.beta : 1e-12;
        const double s = p_.c_e * error + (1.0 / beta) * term;

        double sat_val;
        if (p_.phi > 1e-12)
            sat_val = std::max(-1.0, std::min(1.0, s / p_.phi));
        else
            sat_val = (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);

        double u = -(p_.K * sat_val + p_.eta * s);
        u = std::max(p_.uMin, std::min(p_.uMax, u));

        e_prev_ = error;
        s_prev_ = s;
        u_prev_ = u;
        notify_buf_(0) = s;
        notifyObserverState("surface", notify_buf_);
        notifyObserver(u, error);
        return u;
    }

    void NonsingularTerminalSMC::reset()
    {
        e_prev_ = 0.0;
        s_prev_ = 0.0;
        u_prev_ = 0.0;
        notifyObserverReset();
    }

    // ---------------------------------------------------------------------------
    // AdaptiveSMC
    // ---------------------------------------------------------------------------
    AdaptiveSMC::AdaptiveSMC(const AdaptiveSMCParams &params, double sampleTime)
        : p_(params), Ts_(sampleTime)
    {
        reset();
    }

    // First-order sliding surface with an online-adapted switching gain:
    //   s      = c_e.e + c_de.(e - e_prev)
    //   u      = -K.sat(s/phi)
    //   K[k+1] = clamp( K + Ts.gamma.(|s| - epsilon), Kmin, Kmax )
    // The gain grows while |s| exceeds the dead-band and relaxes inside it, so no a-priori
    // disturbance bound is required.
    double AdaptiveSMC::compute(double error)
    {
        if (!std::isfinite(error))
            return u_prev_;

        const double s = p_.c_e * error + p_.c_de * (error - e_prev_);

        double sat_val;
        if (p_.phi > 1e-12)
            sat_val = std::max(-1.0, std::min(1.0, s / p_.phi));
        else
            sat_val = (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);

        double u = -K_ * sat_val;
        u = std::max(p_.uMin, std::min(p_.uMax, u));

        // Gain adaptation (Euler integration of Kdot = gamma.(|s| - epsilon)).
        K_ += Ts_ * p_.gamma * (std::abs(s) - p_.epsilon);
        K_ = std::max(p_.Kmin, std::min(p_.Kmax, K_));

        e_prev_ = error;
        s_prev_ = s;
        u_prev_ = u;
        notify_buf_(0) = s;
        notifyObserverState("surface", notify_buf_);
        notifyObserver(u, error);
        return u;
    }

    void AdaptiveSMC::reset()
    {
        e_prev_ = 0.0;
        s_prev_ = 0.0;
        u_prev_ = 0.0;
        K_      = p_.K0;
        notifyObserverReset();
    }

} // namespace ctrl
