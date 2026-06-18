#pragma once
#include "IController.h"

/**
 * @file DiscretePID.h
 * @brief Discrete-time PID controller with derivative filter and back-calculation anti-windup.
 *
 * Discretisation: backward Euler for both the integral and derivative filter.
 *
 * ISA standard form:
 * @code
 *   u[k] = Kp.e[k] + I[k] + D[k]
 * @endcode
 *
 * Derivative filter (backward Euler, equivalent to MATLAB 'Filter coefficient' N):
 * @code
 *   d[k] = (1/(1+N.Ts)).d[k-1] + (Kd.N/(1+N.Ts)).(e[k] - e[k-1])
 * @endcode
 *
 * Anti-windup via back-calculation:
 * @code
 *   I[k] = I[k-1] + Ki.Ts.e[k] + Kb.(u_sat[k] - u_unsat[k])
 * @endcode
 *
 * @see Astrom & Wittenmark, "Computer Controlled Systems" Ch. 3.
 * @see MATLAB pid(), pidtune(), Simulink Discrete PID Controller block.
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for DiscretePID. All gains follow MATLAB's parallel PID form.
 */
struct PIDParams
{
    double Kp = 1.0; ///< Proportional gain.
    double Ki = 0.0; ///< Integral gain (Ki = Kp / Ti).
    double Kd = 0.0; ///< Derivative gain (Kd = Kp . Td).

    /**
     * @brief Derivative filter coefficient N [rad/s].
     *
     * Higher N = less filtering (sharper derivative). The derivative pole is placed at
     * z = 1/(1 + N.Ts) in the backward-Euler discretisation, corresponding to a
     * continuous-time pole at s = -N. This attenuates derivative action above N rad/s.
     *
     * Constraints:
     * - Nyquist limit: N < pi/Ts (e.g., N < 314 rad/s for Ts = 0.01 s).
     * - Practical limit: N < pi/(2.Ts.omegac) where omegac is the closed-loop bandwidth.
     *
     * The default N = 100 suits many process-control loops; reduce if derivative is noisy.
     */
    double N = 100.0;

    double uMin = -1e9; ///< Lower output saturation limit.
    double uMax =  1e9; ///< Upper output saturation limit.

    /**
     * @brief Anti-windup back-calculation gain (0 = disabled, 1 = default).
     *
     * Recommended tuning range (Astrom & Wittenmark Section 3.5): Ki/Kp <= Kb <= Kp/Kd
     * - `Kb = Ki/Kp` - slow reset; good for dead-time-dominated plants.
     * - `Kb = Kp/Kd` - fast reset; may cause an actuator spike on re-entry.
     * - `Kb = sqrt(Ki.Kp/Kd)` - geometric-mean rule of thumb for balanced reset.
     * - `Kb = 0` - disables anti-windup entirely; use only when saturation cannot occur.
     * - `Kb = 1.0` (default) - reasonable for many processes; may be slow for high-Ki loops.
     */
    double Kb = 1.0;

    /**
     * @brief Two-degree-of-freedom proportional setpoint weight b \in [0, 1].
     *
     * Applied only by computeDoM(y, r): the proportional term acts on (b.r - y) while the
     * integral term acts on (r - y). This decouples setpoint-tracking bandwidth from
     * disturbance-rejection bandwidth without changing any gains.
     *
     * Tuning guide (Skogestad 2003 / Astrom & Hagglund 2006 Section 8.4):
     * - `b = 1.0` (default): standard 1DOF - maximum setpoint gain.
     * - `b = 0.5`: reduces overshoot ~10-15 % vs b = 1 for ZN-tuned loops.
     * - `b = 0.0`: setpoint feed-forward entirely through integrator; no proportional kick.
     *
     * @note Has no effect on compute(error) because that overload receives only the
     *       pre-computed error (r - y) and cannot separate r from y.
     */
    double b_weight = 1.0;
};

/**
 * @brief Discrete-time PID controller.
 *
 * Implements the ISA parallel form with backward-Euler discretisation, a first-order
 * derivative low-pass filter (coefficient N), and back-calculation anti-windup. An optional
 * derivative-on-measurement variant (computeDoM()) avoids the derivative kick when the
 * setpoint steps.
 *
 * @see PIDParams for tuning parameter descriptions.
 */
class DiscretePID : public IController
{
public:
    /**
     * @brief Construct with tuning parameters and a fixed sample time.
     * @param params    PID tuning parameters (gains, limits, anti-windup, filter).
     * @param sampleTime Sample period Ts [s] > 0.
     */
    explicit DiscretePID(const PIDParams &params, double sampleTime);

    /**
     * @brief Compute u[k] from tracking error e[k] = r[k] - y[k].
     * @param error Current tracking error.
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

    /**
     * @brief Derivative-on-Measurement variant - avoids derivative kick on setpoint steps.
     *
     * The derivative filter is applied to -y (plant output) rather than to the error e = r - y.
     * Proportional and integral terms still use the full error e = r - y.
     * The 2DOF setpoint weight PIDParams::b_weight is applied here: the proportional term
     * acts on (b.r - y) rather than (r - y).
     *
     * Prefer this overload when:
     * - The setpoint changes frequently (batch reactors, motion controllers).
     * - Large setpoint steps cause unacceptable actuator spikes.
     *
     * Equivalent MATLAB: Discrete PID block with 'Derivative on measurement' selected.
     *
     * @param y Current plant output y[k].
     * @param r Current setpoint r[k].
     * @return Saturated control output u[k].
     */
    double computeDoM(double y, double r);

    /** @brief Reset all internal states (integral, derivative filter, previous values). */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Hot-update tuning parameters at runtime.
     *
     * Safe for gradual parameter changes (bumpless). Abrupt large changes (e.g., switching
     * Kp by an order of magnitude) may cause a transient bump.
     * @param params New PID parameters.
     */
    void setParams(const PIDParams &params) { p_ = params; }

    /** @brief Read-only access to current tuning parameters. */
    const PIDParams &params() const { return p_; }

    /** @brief Control output produced at the previous sample (u[k-1]). */
    double lastOutput() const { return u_prev_; }

    /**
     * @brief True when back-calculation anti-windup is active (Kb != 0).
     *
     * AntiWindupWrapper checks this at construction to prevent double-correction.
     * Set PIDParams::Kb = 0 before wrapping with AntiWindupWrapper.
     */
    bool hasInternalAntiWindup() const override { return p_.Kb != 0.0; }

    /**
     * @brief Bumpless initialisation - sets integral so compute(error) returns u_target.
     *
     * Called by ControllerStack (Supervisory mode) on activation to avoid an output bump.
     *
     * @param u_target Desired initial output.
     * @param error    Current tracking error that will be passed to compute().
     */
    void bumplessInit(double u_target, double error) override;

private:
    PIDParams p_;
    double Ts_;
    double integral_; ///< Accumulated integral I[k].
    double deriv_;    ///< Filtered derivative state d[k-1].
    double e_prev_;   ///< Previous error e[k-1].
    double y_prev_;   ///< Previous measurement y[k-1] (derivative-on-measurement).
    double u_prev_;   ///< Previous saturated output u[k-1] (anti-windup).
};

} // namespace ctrl
