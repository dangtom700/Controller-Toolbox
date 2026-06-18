#pragma once
#include "ControllerRegistry.h"
#include <cmath>

/**
 * @file BasicPID.h
 * @brief Header-only template PID controller for embedded targets.
 *
 * BasicPID<Scalar> provides the same ISA parallel PID algorithm as DiscretePID
 * without virtual dispatch, Eigen, or std::function overhead.  The Scalar
 * template parameter allows float usage on MCUs with 32-bit FPU (no 64-bit FP).
 *
 * **Algorithm (backward Euler, same as DiscretePID):**
 * @code
 *   d[k]     = alpha * d[k-1] + Kd*N*alpha * (e[k] - e[k-1])   // derivative filter
 *   u_raw[k] = Kp*e[k] + I[k] + d[k]                           // unsaturated output
 *   u[k]     = clamp(u_raw, uMin, uMax)                         // saturate
 *   I[k+1]   = I[k] + Ki*Ts*e[k] + Kb*(u[k] - u_raw[k])       // back-calc anti-windup
 * @endcode
 * where alpha = 1 / (1 + N*Ts).
 *
 * **Convention:** compute(error) where error = r - y (same as DiscretePID).
 *
 * **Differences from DiscretePID:**
 * - No IController base - no virtual table, no observer pattern.
 * - No Eigen dependency - pure arithmetic on Scalar.
 * - No setReference() - caller passes the error directly.
 * - Ts stored in Params for compile-time visibility (can be constexpr).
 *
 * **Typical embedded usage (float):**
 * @code
 *   ctrl::BasicPIDParams<float> p;
 *   p.Kp = 2.0f; p.Ki = 0.5f; p.Ts = 0.001f; p.uMin = -1.0f; p.uMax = 1.0f;
 *   ctrl::BasicPID<float> pid(p);
 *   // In control loop:
 *   float u = pid.compute(setpoint - measurement);
 * @endcode
 *
 * @see DiscretePID.h - full-featured double-precision version with IController.
 */

namespace ctrl {

/**
 * @brief Tuning parameters for BasicPID<Scalar>.
 *
 * All fields are plain aggregates; no Eigen or std::function.
 */
template<typename Scalar = double>
struct BasicPIDParams
{
    Scalar Kp  = Scalar(1);      ///< Proportional gain.
    Scalar Ki  = Scalar(0);      ///< Integral gain.
    Scalar Kd  = Scalar(0);      ///< Derivative gain.
    Scalar N   = Scalar(100);    ///< Derivative filter coefficient [rad/s].
    Scalar Kb  = Scalar(1);      ///< Anti-windup back-calculation gain (0 = disabled).
    Scalar uMin = Scalar(-1e9);  ///< Output saturation lower limit.
    Scalar uMax = Scalar( 1e9);  ///< Output saturation upper limit.
    Scalar Ts  = Scalar(0.01);   ///< Sample time [s].
};

/**
 * @brief Header-only template PID controller for embedded/float targets.
 *
 * @tparam Scalar  Numeric type: double (default) or float for MCU targets.
 */
template<typename Scalar = double>
class BasicPID
{
public:
    using Params = BasicPIDParams<Scalar>;

    /** @brief Construct with tuning parameters. */
    explicit BasicPID(const Params& p = Params{}) : p_(p)
    {
        reset();
    }

    /**
     * @brief Compute control output from tracking error.
     * @param error  Tracking error e[k] = r[k] - y[k].
     * @return Saturated control output u[k].
     */
    Scalar compute(Scalar error)
    {
        const Scalar alpha  = Scalar(1) / (Scalar(1) + p_.N * p_.Ts);
        const Scalar d      = alpha * d_prev_ + p_.Kd * p_.N * alpha * (error - e_prev_);
        const Scalar u_raw  = p_.Kp * error + I_ + d;
        const Scalar u      = clamp(u_raw, p_.uMin, p_.uMax);

        I_      += p_.Ki * p_.Ts * error + p_.Kb * (u - u_raw);
        e_prev_  = error;
        d_prev_  = d;
        u_prev_  = u;
        return u;
    }

    /** @brief Zero integrator, previous error, derivative state, and output. */
    void reset()
    {
        I_      = Scalar(0);
        e_prev_ = Scalar(0);
        d_prev_ = Scalar(0);
        u_prev_ = Scalar(0);
    }

    /** @brief Hot-update parameters without resetting state. */
    void setParams(const Params& p) { p_ = p; }

    /** @brief Read-only access to current parameters. */
    const Params& params() const { return p_; }

    /** @brief Last computed output (held across calls). */
    Scalar lastOutput() const { return u_prev_; }

    /** @brief Current integrator state. */
    Scalar integrator() const { return I_; }

private:
    Params p_;
    Scalar I_      = Scalar(0);
    Scalar e_prev_ = Scalar(0);
    Scalar d_prev_ = Scalar(0);
    Scalar u_prev_ = Scalar(0);

    static Scalar clamp(Scalar v, Scalar lo, Scalar hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(basic_pid)
