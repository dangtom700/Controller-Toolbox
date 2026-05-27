#pragma once
#include "IController.h"
#include <cmath>

/**
 * @file ExtremumSeeker.h
 * @brief Perturbation-based Extremum Seeking Controller (ESC).
 *
 * Injects a sinusoidal dither into the plant operating point, demodulates the plant output
 * to extract a gradient estimate, then integrates the gradient to converge to the extremum
 * of an unknown static (or slowly varying) cost surface - **no explicit model required**.
 *
 * **Algorithm per sample k:**
 * @code
 *   phase[k] += 2pi.fp.Ts                          // phase accumulator (wrapped mod 2pi)
 *   u[k]      = theta[k] + a.sin(phase[k])            // operating point + dither
 *
 *   // High-pass filter (backward Euler, removes DC/slow drift):
 *   yh[k]   = alphah.(yh[k-1] + y[k] - y[k-1]),   alphah = 1/(1 + 2pi.fhpf.Ts)
 *
 *   // Demodulate - extract gradient signal:
 *   xi[k]    = yh[k].sin(phase[k])
 *
 *   // Low-pass filter (backward Euler, smooths gradient):
 *   ghat[k]    = ghat[k-1] + alphal.(xi[k] - ghat[k-1]),    alphal = 2pi.flpf.Ts/(1 + 2pi.flpf.Ts)
 *
 *   // Gradient-descent integration:
 *   theta[k+1]  = theta[k] + sign_dir.kint.Ts.ghat[k]      // sign_dir = -1 (min) or +1 (max)
 * @endcode
 *
 * @par Separation of timescales requirement
 * For correct gradient estimation: plant settling time ≪ 1/fp ≪ 1/flpf.
 * The plant must respond quasi-statically to the dither frequency, and the LPF must cleanly
 * extract the gradient from the demodulated signal. Violating this causes a biased gradient
 * estimate and slow or no convergence.
 *
 * @see Ariyur & Krstic, "Real-Time Optimisation by Extremum-Seeking Control" (2003).
 * @see Tan et al., "On non-local stability of ESC" (2006).
 * @see MATLAB/Simulink Extremum Seeking block (R2020b+).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for ExtremumSeeker.
 */
struct ExtremumSeekerParams
{
    double perturbAmp  = 0.1;  ///< Dither amplitude a. Larger = faster convergence, more perturbation.
    double perturbFreq = 1.0;  ///< Dither frequency fp [Hz]. Must be well above the closed-loop bandwidth.
    double lpfCutoff   = 0.1;  ///< Low-pass filter cutoff flpf [Hz] for gradient smoothing.
    double hpfCutoff   = 0.05; ///< High-pass filter cutoff fhpf [Hz] for DC removal.
    double integGain   = 1.0;  ///< Gradient integrator gain kint.
    bool   seekMinimum = true; ///< @c true to seek the minimum; @c false to seek the maximum.
};

/**
 * @brief Perturbation-based Extremum Seeking Controller.
 *
 * IController::compute() takes the **plant output** (cost/performance metric) rather than
 * a tracking error, and returns the **plant input** (operating-point estimate + dither).
 */
class ExtremumSeeker : public IController
{
public:
    /**
     * @brief Construct with tuning parameters and fixed sample time.
     * @param params     ESC tuning (dither amplitude/frequency, filter cutoffs, integrator gain).
     * @param sampleTime Sample period Ts [s].
     */
    ExtremumSeeker(const ExtremumSeekerParams &params, double sampleTime);

    /**
     * @brief Advance one step of the ESC algorithm.
     *
     * @param signal Plant output y[k] (cost or performance metric, **not** tracking error).
     * @return Plant input u[k] = operating-point estimate theta[k] + dither a.sin(phase[k]).
     */
    double compute(double signal) override;

    /** @brief Reset phase, integrator, filter states, and previous output. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Hot-update tuning parameters.
     * @param p New ESC parameters.
     */
    void setParams(const ExtremumSeekerParams &p) { p_ = p; }

    /** @brief Read-only access to current parameters. */
    const ExtremumSeekerParams &params() const { return p_; }

    /**
     * @brief Current operating-point estimate theta (without dither component).
     *
     * Converges to the extremum of the cost surface as the algorithm runs.
     */
    double currentEstimate() const { return theta_; }

private:
    ExtremumSeekerParams p_;
    double Ts_;
    double phase_;      ///< Dither phase accumulator [rad], wrapped to [0, 2pi).
    double theta_;      ///< Operating-point integrator state.
    double hpf_state_;  ///< HPF IIR state (backward Euler, first order).
    double lpf_state_;  ///< LPF IIR state (backward Euler, first order).
    double y_prev_;     ///< y[k-1] for the HPF difference term.
};

} // namespace ctrl
