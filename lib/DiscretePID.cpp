#include "DiscretePID.h"
#include <algorithm>
#include <cmath>

namespace ctrl
{

    DiscretePID::DiscretePID(const PIDParams &params, double sampleTime)
        : p_(params), Ts_(sampleTime)
    {
        reset();
    }

    // Discrete PID - backward Euler integral + backward-Euler filtered derivative.
    //
    // Derivative filter recurrence (derived from C_D(s) = Kd.N.s/(s+N), backward Euler):
    //   alpha     = 1 / (1 + N.Ts)
    //   d[k]  = alpha.d[k-1] + Kd.N.alpha.(e[k] - e[k-1])
    //
    // Anti-windup back-calculation (Astrom & Wittenmark):
    //   I[k+1] = I[k] + Ki.Ts.e[k] + Kb.(u_sat[k] - u_unsat[k])
    double DiscretePID::compute(double error)
    {
        // Hold last output on bad measurement rather than corrupting integral state
        if (!std::isfinite(error))
            return u_prev_;

        // Filtered derivative
        const double alpha = 1.0 / (1.0 + p_.N * Ts_);
        const double d_new = alpha * deriv_ + p_.Kd * p_.N * alpha * (error - e_prev_);

        // Integral: backward Euler increment for this step
        const double ki_update = p_.Ki * Ts_ * error;

        // Unsaturated output - uses I[k] = I[k-1] + ki_update (backward Euler)
        const double u_unsat = p_.Kp * error + (integral_ + ki_update) + d_new;

        // Output saturation
        const double u_sat = std::max(p_.uMin, std::min(p_.uMax, u_unsat));

        // Integral state update with anti-windup back-calculation
        integral_ += ki_update + p_.Kb * (u_sat - u_unsat);

        // State updates
        deriv_ = d_new;
        e_prev_ = error;
        u_prev_ = u_sat;

        return u_sat;
    }

    void DiscretePID::reset()
    {
        integral_ = 0.0;
        deriv_ = 0.0;
        e_prev_ = 0.0;
        y_prev_ = 0.0;
        u_prev_ = 0.0;
    }

    // Back-calculate the integral state so that compute(error) approx = u_target.
    // Derivation: u_target approx = Kp*error + integral + Kd*N*(error-e_prev)/(1+N*Ts)
    // Solving for integral: integral = u_target - Kp*error - deriv (zeroed here)
    // On the very first compute() call after this, the output will be close to u_target.
    void DiscretePID::bumplessInit(double u_target, double error)
    {
        deriv_    = 0.0;
        e_prev_   = error;
        u_prev_   = u_target;
        integral_ = u_target - p_.Kp * error;  // absorb P term; D starts from rest
    }

    // Derivative on measurement: same as compute() but routes the derivative filter
    // through -y (plant output) instead of e = r - y. This eliminates the derivative
    // spike that occurs when the setpoint r steps.
    //
    // Derivation:
    //   Standard: d[k] = alpha*d[k-1] + Kd*N*alpha*(e[k] - e[k-1])
    //   DoM:      d[k] = alpha*d[k-1] - Kd*N*alpha*(y[k] - y[k-1])   <- derivative of -y only
    //   (sign correct: for a step up in r, y increases, d[k] goes negative = less overshoot)
    //
    // Equivalent MATLAB: Discrete PID Controller block, DerivativeFilter on 'Measurement'.
    double DiscretePID::computeDoM(double y, double r)
    {
        const double error = r - y;

        if (!std::isfinite(error) || !std::isfinite(y))
            return u_prev_;

        // Derivative filter on -y (not on error)
        const double alpha = 1.0 / (1.0 + p_.N * Ts_);
        const double d_new = alpha * deriv_ - p_.Kd * p_.N * alpha * (y - y_prev_);

        // Integral: backward Euler on error (unchanged)
        const double ki_update = p_.Ki * Ts_ * error;

        // Unsaturated output
        const double u_unsat = p_.Kp * error + (integral_ + ki_update) + d_new;

        // Output saturation
        const double u_sat = std::max(p_.uMin, std::min(p_.uMax, u_unsat));

        // Integral state update with anti-windup back-calculation
        integral_ += ki_update + p_.Kb * (u_sat - u_unsat);

        // State updates
        deriv_  = d_new;
        e_prev_ = error;
        y_prev_ = y;
        u_prev_ = u_sat;

        return u_sat;
    }

} // namespace ctrl
