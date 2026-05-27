#pragma once
#include "IController.h"

/**
 * @file DiscreteLeadLag.h
 * @brief Discrete Lead, Lag, or Lead-Lag compensator (Tustin-discretised first-order IIR).
 *
 * **Continuous form:** C(s) = K·(s + zc) / (s + pc)
 * - **Lead** (pc > zc > 0): adds positive phase, improves phase margin and closed-loop speed.
 * - **Lag** (0 < pc < zc):  adds low-frequency gain, improves steady-state accuracy.
 *
 * **Tustin discretisation** (s = 2(z−1)/(Ts(z+1))):
 * @code
 *   C(z) = (b0 + b1·z⁻¹) / (1 + a1·z⁻¹)
 *
 *   b0 = K·(2/Ts + zc) / (2/Ts + pc)
 *   b1 = K·(zc − 2/Ts) / (2/Ts + pc)
 *   a1 = (pc − 2/Ts)   / (2/Ts + pc)
 * @endcode
 *
 * Difference equation: y[k] = b0·u[k] + b1·u[k−1] − a1·y[k−1]
 *
 * @see Franklin, Powell & Emami-Naeini, "Feedback Control of Dynamic Systems".
 * @see MATLAB c2d() with 'tustin' method.
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for DiscreteLeadLag.
 */
struct LeadLagParams
{
    double continuousZero = 1.0;  ///< Zero location zc [rad/s].
    double continuousPole = 10.0; ///< Pole location pc [rad/s].
    double gain           = 1.0;  ///< DC gain K.
};

/**
 * @brief Discrete Lead-Lag compensator.
 *
 * Implemented as a first-order IIR filter derived from the Tustin-discretised continuous
 * transfer function C(s) = K·(s + zc)/(s + pc).
 */
class DiscreteLeadLag : public IController
{
public:
    /**
     * @brief Construct and precompute IIR coefficients.
     * @param params     Lead/lag zero, pole, and gain parameters.
     * @param sampleTime Sample period Ts [s].
     */
    DiscreteLeadLag(const LeadLagParams &params, double sampleTime);

    /**
     * @brief Filter the input signal (typically the tracking error or plant output).
     * @param u Input signal u[k].
     * @return Filtered output y[k].
     */
    double compute(double u) override;

    /** @brief Reset previous input and output to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Hot-update parameters and recompute IIR coefficients.
     * @param p New lead-lag parameters.
     */
    void setParams(const LeadLagParams &p);

    /** @brief Read-only access to current parameters. */
    const LeadLagParams &params() const { return p_; }

    /**
     * @brief Phase contribution at a given frequency (continuous-domain approximation).
     *
     * Returns the phase angle of C(jω) = K·(jω + zc)/(jω + pc) in degrees.
     * Useful for verifying that the compensator provides the intended phase lead/lag.
     *
     * @param omega_rad_s Frequency [rad/s].
     * @return Phase shift [rad]. Positive = lead, negative = lag.
     */
    double phaseAt(double omega_rad_s) const;

private:
    LeadLagParams p_;
    double Ts_;
    double b0_, b1_, a1_; ///< Tustin IIR coefficients.
    double u_prev_;       ///< Previous input u[k−1].
    double y_prev_;       ///< Previous output y[k−1].

    /** @brief Recompute b0, b1, a1 from the current parameters. */
    void computeCoeffs();
};

} // namespace ctrl
