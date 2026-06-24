#pragma once
#include "IController.h"
#include "Features.h"

/**
 * @file ResonantController.h
 * @brief Discrete-time non-ideal (finite-Q) resonant controller for single-harmonic rejection.
 *
 * **Continuous form** (avoids the phase singularity of the textbook infinite-gain resonant
 * term by using a finite damping bandwidth wc):
 * @code
 *   G_RC(s) = 2*Kr*wc*s / (s^2 + 2*wc*s + w0^2)      [w0 = 2*pi*targetFreqHz]
 * @endcode
 *
 * Discretised via Tustin with frequency prewarping at w0, so the digital resonance peak lands
 * exactly at targetFreqHz despite bilinear-transform frequency warping:
 * @code
 *   w0_warped = (2/Ts)*tan(w0*Ts/2)
 * @endcode
 *
 * By the exact bilinear-transform correspondence H_d(e^{j*theta}) = G(j*K*tan(theta/2)) (K=2/Ts),
 * the steady-state gain at theta = w0*Ts is exactly Kr (real, zero phase) - verified analytically
 * and numerically (see docs/superpowers/plans/2026-06-24-resonant-notch-pll-controllers.md).
 *
 * Composes with a base controller through ControllerStack(StackMode::Additive) - add one
 * ResonantController per target harmonic alongside the base controller; the stack sums outputs.
 *
 * @see Yepes, Freijedo, Lopez & Doval-Gandoy, "High-Performance Digital Resonant Controllers
 *      Implemented With Two Integrators", IEEE Trans. Power Electronics (2011).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for ResonantController.
 */
struct ResonantParams
{
    double targetFreqHz;     ///< f0 - the harmonic frequency to reject/track [Hz].
    double dampingRadPerSec; ///< wc - bandwidth/peak-width parameter [rad/s]. Smaller = narrower/higher peak.
    double Kr;               ///< Resonant gain - the exact steady-state gain at targetFreqHz.
    double uMin = -1e9;      ///< Output saturation lower limit.
    double uMax =  1e9;      ///< Output saturation upper limit.
};

/**
 * @brief Discrete-time single-harmonic resonant controller.
 *
 * Inherits from IController so it composes through ControllerStack(Additive) alongside a base
 * controller (PID, etc.), one instance per target harmonic.
 */
class ResonantController : public IController
{
public:
    /**
     * @brief Construct the resonant controller and precompute biquad coefficients.
     * @param params Resonant tuning parameters (target frequency, damping, gain, limits).
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument if targetFreqHz <= 0, dampingRadPerSec <= 0, or
     *         targetFreqHz is at/above the Nyquist frequency (1/(2*Ts)).
     */
    ResonantController(const ResonantParams &params, double Ts);

    /**
     * @brief Compute the resonant correction for the current tracking error.
     * @param error Tracking error e[k] = r[k] - y[k].
     * @return Resonant correction u[k], clamped to [uMin, uMax].
     */
    double compute(double error) override;

    /** @brief This controller's signal convention is e = r - y. */
    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }

    /** @brief Reset the biquad's internal state (previous inputs/outputs) to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Hot-update parameters and recompute biquad coefficients. */
    void setParams(const ResonantParams &p);

    /** @brief Read-only access to current parameters. */
    const ResonantParams &params() const { return p_; }

private:
    ResonantParams p_;
    double Ts_;
    double b0_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0; ///< Biquad coeffs (b1 is exactly 0).
    double e_prev1_ = 0.0, e_prev2_ = 0.0;             ///< e[k-1], e[k-2].
    double u_prev1_ = 0.0, u_prev2_ = 0.0;             ///< u[k-1], u[k-2].

    void computeCoeffs();
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(resonant_controller)
