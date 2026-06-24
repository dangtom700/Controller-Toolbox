#pragma once
#include "Features.h"

/**
 * @file PhaseLockedLoop.h
 * @brief Single-input SOGI-PLL: tracks the phase and frequency of one sampled sinusoid.
 *
 * Produces an estimate (phase, frequency, amplitude), not a control action, so this is a
 * standalone estimator - same philosophy as KalmanFilter, no shared base with IController.
 *
 * **Pipeline per sample** (Ciobotaru, Teodorescu & Blaabjerg, "A New Single-Phase PLL
 * Structure Based on Second Order Generalized Integrator", IEEE PESC 2006):
 * 1. SOGI quadrature signal generator (forward-Euler discretised, tuned to the current
 *    frequency estimate w_hat) manufactures states x1, x2 from the single input sample. In
 *    steady state x1 ~= A*sin(theta_true), x2 ~= A*cos(theta_true) (verified numerically -
 *    this is the opposite role/sign pairing a naive continuous-domain phasor derivation
 *    suggests; see the implementation plan this class was specified from for the full
 *    derivation and the bug it caught).
 * 2. Park-transform-style error signal v_q = x1*cos(theta_hat) - x2*sin(theta_hat)
 *    (= A*sin(theta_true - theta_hat)), which vanishes when theta_hat == theta_true.
 * 3. A PI loop filter on v_q (direct, no extra sign flip) produces a frequency correction.
 * 4. The corrected frequency drives an NCO (phase integrator), wrapped to [-pi, pi).
 *
 * @see Ciobotaru, Teodorescu & Blaabjerg (2006).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for PhaseLockedLoop.
 */
struct PLLParams
{
    double nominalFreqHz;        ///< Expected/center frequency [Hz].
    double Kp;                   ///< PI loop-filter proportional gain.
    double Ki;                   ///< PI loop-filter integral gain.
    double sogiK = 1.41421356;   ///< SOGI damping gain (default sqrt(2)).
};

/**
 * @brief Single-input SOGI-based phase-locked loop.
 */
class PhaseLockedLoop
{
public:
    /**
     * @brief Construct the PLL and initialise its internal state at the nominal frequency.
     * @param params Nominal frequency and PI loop-filter gains.
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument if nominalFreqHz <= 0, or nominalFreqHz is at/above the
     *         Nyquist frequency (1/(2*Ts)).
     */
    PhaseLockedLoop(const PLLParams &params, double Ts);

    /**
     * @brief Advance the PLL by one sample.
     * @param sample Measured sinusoid value v[k]. A non-finite sample is skipped entirely -
     *               the estimate holds at its last value and internal state is unchanged.
     */
    void step(double sample);

    /** @brief Current phase estimate theta_hat [rad], wrapped to [-pi, pi). */
    double phase() const { return theta_hat_; }

    /** @brief Current frequency estimate [Hz]. */
    double frequencyHz() const { return w_hat_ / (2.0 * M_PI); }

    /** @brief Estimated input amplitude sqrt(x1^2 + x2^2) - a free byproduct of the SOGI. */
    double amplitude() const;

    /** @brief True once |v_q| has stayed below a small fraction of the amplitude for long enough. */
    bool locked() const { return lockCounter_ >= kLockCountRequired; }

    /** @brief Reset all internal state; frequency estimate returns to nominalFreqHz. */
    void reset();

    /** @brief Read-only access to current parameters. */
    const PLLParams &params() const { return p_; }

private:
    static constexpr int kLockCountRequired = 50;

    PLLParams p_;
    double Ts_;
    double x1_ = 0.0, x2_ = 0.0; ///< SOGI quadrature-generator states.
    double integral_ = 0.0;      ///< PI loop-filter integrator state.
    double theta_hat_ = 0.0;     ///< Phase estimate [rad].
    double w_hat_ = 0.0;         ///< Frequency estimate [rad/s].
    int    lockCounter_ = 0;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(phase_locked_loop)
