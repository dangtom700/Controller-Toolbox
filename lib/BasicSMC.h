#pragma once
#include "ControllerRegistry.h"

/**
 * @file BasicSMC.h
 * @brief Header-only template sliding mode controller for embedded targets.
 *
 * BasicSMC<Scalar> implements first-order discrete SMC with boundary-layer
 * saturation - the same algorithm as DiscreteSMC - without virtual dispatch,
 * Eigen, or std::function overhead.  Suitable for float usage on MCUs.
 *
 * **Algorithm:**
 * @code
 *   s[k]  = c_e * e[k] + c_de * (e[k] - e[k-1])    // sliding surface
 *   sat   = clamp(s[k] / phi, -1, 1)                // boundary-layer saturation
 *   u[k]  = clamp(-K * sat, uMin, uMax)             // output
 * @endcode
 *
 * **Convention:** compute(error) where error = r - y (error = r[k] - y[k]).
 *
 * **Typical embedded usage (float):**
 * @code
 *   ctrl::BasicSMCParams<float> p;
 *   p.c_e = 1.0f; p.c_de = 0.05f; p.K = 3.0f; p.phi = 0.2f;
 *   p.uMin = -1.0f; p.uMax = 1.0f;
 *   ctrl::BasicSMC<float> smc(p);
 *   float u = smc.compute(setpoint - measurement);
 * @endcode
 *
 * @see DiscreteSMC.h - full-featured version with IController and SuperTwisting variant.
 */

namespace ctrl {

/**
 * @brief Tuning parameters for BasicSMC<Scalar>.
 */
template<typename Scalar = double>
struct BasicSMCParams
{
    Scalar c_e  = Scalar(1.0);   ///< Error weight in the sliding surface.
    Scalar c_de = Scalar(0.1);   ///< Error-rate weight (discrete; absorbs Ts; see DiscreteSMC).
    Scalar K    = Scalar(5.0);   ///< Switching gain. Larger = more robust, more chattering.
    Scalar phi  = Scalar(0.5);   ///< Boundary-layer thickness. Larger = smoother, slower.
    Scalar uMin = Scalar(-1e9);  ///< Output saturation lower limit.
    Scalar uMax = Scalar( 1e9);  ///< Output saturation upper limit.
};

/**
 * @brief Header-only template SMC for embedded/float targets.
 *
 * @tparam Scalar  Numeric type: double (default) or float for MCU targets.
 */
template<typename Scalar = double>
class BasicSMC
{
public:
    using Params = BasicSMCParams<Scalar>;

    /** @brief Construct with tuning parameters. */
    explicit BasicSMC(const Params& p = Params{}) : p_(p)
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
        const Scalar s    = p_.c_e * error + p_.c_de * (error - e_prev_);
        const Scalar sat  = clamp(s / p_.phi, Scalar(-1), Scalar(1));
        const Scalar u    = clamp(-p_.K * sat, p_.uMin, p_.uMax);

        e_prev_ = error;
        u_prev_ = u;
        return u;
    }

    /** @brief Zero previous error and output. */
    void reset()
    {
        e_prev_ = Scalar(0);
        u_prev_ = Scalar(0);
    }

    /** @brief Hot-update parameters without resetting state. */
    void setParams(const Params& p) { p_ = p; }

    /** @brief Read-only access to current parameters. */
    const Params& params() const { return p_; }

    /** @brief Last computed output. */
    Scalar lastOutput() const { return u_prev_; }

    /**
     * @brief Current sliding surface value s[k] given the last stored error and new error.
     * @param error  Current error e[k].
     */
    Scalar slidingSurface(Scalar error) const
    {
        return p_.c_e * error + p_.c_de * (error - e_prev_);
    }

private:
    Params p_;
    Scalar e_prev_ = Scalar(0);
    Scalar u_prev_ = Scalar(0);

    static Scalar clamp(Scalar v, Scalar lo, Scalar hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(basic_smc)
