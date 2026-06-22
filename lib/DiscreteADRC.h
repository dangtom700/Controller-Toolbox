#pragma once
#include "IController.h"
#include <Eigen/Dense>

/**
 * @file DiscreteADRC.h
 * @brief Active Disturbance Rejection Controller - 2nd-order Linear ADRC (LADRC).
 *
 * Models the controlled plant as a double integrator with unknown total disturbance:
 * @code
 *   y = f(y, ydot, d, t) + b0.u     (b0 is an approximate input gain)
 * @endcode
 *
 * A 3rd-order Extended State Observer (ESO) estimates:
 * - z1 approx = y   (output)
 * - z2 approx = ydot   (rate)
 * - z3 approx = f(...) (total disturbance - lumped unknown dynamics + external)
 *
 * **ESO (backward Euler, bandwidth-parameterised at omega0):**
 * @code
 *   epsilon[k]    = y[k] - z1[k]
 *   z3[k+1] = z3[k] + Ts.beta3.epsilon[k]
 *   z2[k+1] = z2[k] + Ts.(beta2.epsilon[k] + b0.u[k]) + Ts.z3[k+1]
 *   z1[k+1] = z1[k] + Ts.beta1.epsilon[k]              + Ts.z2[k+1]
 * @endcode
 * where beta1 = 3omega0, beta2 = 3omega0^2, beta3 = omega0^3 (pole at (s + omega0)^3).
 * This scheme is A-stable for all omega0.Ts > 0.
 *
 * **PD + disturbance cancellation (bandwidth omegac):**
 * @code
 *   u0[k] = omegac^2.(r[k] - z1[k]) - 2omegac.z2[k]
 *   u[k]  = (u0[k] - z3[k]) / b0
 * @endcode
 *
 * @see Han, "From PID to Active Disturbance Rejection Control" (2009).
 * @see Gao, "Scaling and bandwidth-parameterisation based controller tuning" (2003).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for DiscreteADRC.
 */
struct ADRCParams
{
    double omega_o = 20.0; ///< ESO bandwidth [rad/s]. Typically 3-10* omega_c.
    double omega_c =  5.0; ///< Controller bandwidth [rad/s].

    /**
     * @brief Approximate plant input gain b0.
     *
     * The control law u = (u0 - z3)/b0 cancels the estimated total disturbance z3;
     * if b0 is badly wrong, cancellation degrades and the ESO may diverge.
     *
     * Robustness: ADRC tolerates b0 uncertainty within roughly a factor of 3 for
     * omega0 >= 5.omegac (Gao 2003, Section IV). Beyond that, disturbance cancellation degrades.
     *
     * Practical starting estimates:
     * - Motors:             b0 = Km / J  (torque constant / inertia)
     * - Integrating plants: b0 = DC_gain / Ts^2
     * - Unknown plants:     b0 = 1/tau where tau is the open-loop time constant
     *
     * Verify by running open-loop: the step response slope at t = 0^+ approx = b0.u.
     */
    double b0 = 1.0;

    double uMin = -1e9; ///< Lower output saturation limit.
    double uMax =  1e9; ///< Upper output saturation limit.
};

/**
 * @brief Discrete-time 2nd-order Linear Active Disturbance Rejection Controller.
 */
class DiscreteADRC : public IController
{
public:
    /**
     * @brief Construct with tuning parameters and fixed sample time.
     * @param params     ADRC tuning (bandwidths, b0, output limits).
     * @param sampleTime Sample period Ts [s].
     */
    explicit DiscreteADRC(const ADRCParams &params, double sampleTime);

    /**
     * @brief Full interface: compute u[k] from plant output y and setpoint r.
     *
     * Prefer this overload over compute(error) whenever y and r are independently available.
     *
     * @param y Current plant output y[k].
     * @param r Current setpoint r[k].
     * @return Saturated control output u[k].
     */
    double computeTracking(double y, double r);

    /**
     * @brief IController wrapper: compute u[k] from tracking error e[k] = r - y.
     *
     * @warning Call setReference(r) **once per cycle before** calling compute(error).
     *          If setReference() is not called, r_ defaults to 0 and the controller will
     *          drive the plant output to zero regardless of the intended setpoint -
     *          a silent incorrect answer that is hard to diagnose inside a ControllerStack.
     *
     * Internally recovers y = r_ - error and delegates to computeTracking(y, r_).
     *
     * @param error Tracking error e[k] = r[k] - y[k].
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::Other; }

    /**
     * @brief Set the reference for the next compute(error) call.
     * @param r Setpoint r[k].
     */
    void setReference(double r) { r_ = r; }

    /** @brief Reset ESO state and stored inputs to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Hot-update tuning parameters and recompute observer gains.
     * @param p New ADRC parameters.
     */
    void setParams(const ADRCParams &p);

    /** @brief Read-only access to current parameters. */
    const ADRCParams &params() const { return p_; }

    /**
     * @brief Current ESO state estimates [z1, z2, z3].
     * @return Vector: z1 approx = y, z2 approx = ydot, z3 approx = total disturbance.
     */
    const Eigen::Vector3d &esoState() const { return z_; }

private:
    ADRCParams p_;
    double Ts_;
    double r_;            ///< Stored reference (set via setReference(); defaults to 0).
    Eigen::Vector3d z_;   ///< ESO state [z1, z2, z3].
    double u_prev_;       ///< u[k-1] used in ESO state update.
    double beta1_, beta2_, beta3_; ///< Observer gains derived from omega_o.

    /** @brief Recompute beta1, beta2, beta3 from the current omega_o. */
    void updateGains();
};

} // namespace ctrl
