#pragma once
#include "Features.h"

/**
 * @file NotchFilter.h
 * @brief Discrete biquad notch filter (Bristow-Johnson "Audio EQ Cookbook" formulation).
 *
 * Attenuates a single frequency band in any sampled signal - no setpoint/error semantics, so
 * this is a standalone filter, not an IController. Apply it to a measurement, an actuator
 * command, or any other signal that needs a known resonance/disturbance frequency suppressed.
 *
 * **Digital design** (computed directly from the warped digital center frequency - the
 * standard "cookbook" approach, no separate continuous-to-discrete step):
 * @code
 *   omega = 2*pi*centerFreqHz*Ts
 *   alpha = sin(omega) / (2*Q)
 *   b0 =  1,            b1 = -2*cos(omega),  b2 =  1
 *   a0 =  1 + alpha,     a1 = -2*cos(omega),  a2 =  1 - alpha
 *   (normalise b0,b1,b2,a1,a2 by dividing through by a0)
 *
 *   y[k] = b0.x[k] + b1.x[k-1] + b2.x[k-2] - a1.y[k-1] - a2.y[k-2]
 * @endcode
 *
 * The numerator has exact zeros at z = e^{+-j*omega} (on the unit circle), so a sinusoid fed
 * in at exactly centerFreqHz is attenuated to (numerically) zero in steady state - verified
 * numerically in docs/superpowers/plans/2026-06-24-resonant-notch-pll-controllers.md.
 *
 * @see Bristow-Johnson, "Cookbook formulae for audio EQ biquad filter coefficients".
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for NotchFilter.
 */
struct NotchFilterParams
{
    double centerFreqHz; ///< f0 to attenuate [Hz].
    double Q;             ///< Quality factor = f0 / bandwidth. Higher Q = narrower notch.
};

/**
 * @brief Discrete biquad notch filter.
 */
class NotchFilter
{
public:
    /**
     * @brief Construct the filter and precompute biquad coefficients.
     * @param params Center frequency and Q.
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument if centerFreqHz <= 0, Q <= 0, or centerFreqHz is at/above
     *         the Nyquist frequency (1/(2*Ts)).
     */
    NotchFilter(const NotchFilterParams &params, double Ts);

    /**
     * @brief Filter one sample.
     * @param x Input sample x[k].
     * @return Filtered output y[k].
     */
    double apply(double x);

    /** @brief Reset previous inputs/outputs to zero. */
    void reset();

    /** @brief Hot-update parameters and recompute biquad coefficients. */
    void setParams(const NotchFilterParams &p);

    /** @brief Read-only access to current parameters. */
    const NotchFilterParams &params() const { return p_; }

private:
    NotchFilterParams p_;
    double Ts_;
    double b0_ = 0.0, b1_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0;
    double x_prev1_ = 0.0, x_prev2_ = 0.0;
    double y_prev1_ = 0.0, y_prev2_ = 0.0;

    void computeCoeffs();
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(notch_filter)
