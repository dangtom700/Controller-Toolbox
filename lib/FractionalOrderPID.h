#pragma once
#include "IController.h"
#include "ControllerRegistry.h"
#include <vector>

/**
 * @file FractionalOrderPID.h
 * @brief Fractional-order PID controller (PI^{lambda} D^{mu}) with Oustaloup-approximated
 *        fractional operators.
 *
 * Control law (Podlubny 1999):
 * @code
 *   u[k] = Kp.e[k] + Ki.(D^{-lambda} e)[k] + Kd.(D^{mu} e)[k],   0 < lambda, mu < 1
 * @endcode
 *
 * The two fractional operators s^{-lambda} (integrator) and s^{mu} (differentiator) are each
 * realised by an Oustaloup recursive filter: a band-limited product of (2N+1) first-order
 * sections that approximates s^{alpha} over [wb, wh]. Discretised with the bilinear (Tustin)
 * transform, this becomes a fixed-order IIR cascade - bounded memory, allocation-free in the
 * hot loop (RT-friendly), and with finite DC gain so the "integrator" branch cannot wind up
 * unboundedly the way a pure 1/s would.
 *
 * @see Podlubny, "Fractional-Order Systems and PI^{lambda}D^{mu} Controllers", IEEE TAC (1999).
 * @see Oustaloup, Levron, Mathieu & Nanot, "Frequency-band complex noninteger differentiator",
 *      IEEE TCAS-I 47 (2000).
 * @see Monje, Chen, Vinagre, Xue & Feliu, "Fractional-order Systems and Controls" (2010).
 */

namespace ctrl
{

/**
 * @brief Band-limited approximation of the fractional operator s^{alpha} (Oustaloup filter).
 *
 * Approximates s^{alpha} for a real order @p alpha in (-1, 1) over the frequency band
 * [wb, wh] using a product of (2N+1) first-order zero/pole sections, each discretised by the
 * bilinear transform. alpha > 0 realises a fractional differentiator; alpha < 0 a fractional
 * integrator. Used as the building block of @ref FractionalOrderPID but useful standalone.
 *
 * At the geometric band centre omega = sqrt(wb.wh) the magnitude |s^{alpha}| = omega^{alpha}
 * is matched closely; accuracy degrades toward the band edges and improves with larger N.
 */
class FractionalDifferintegrator
{
public:
    FractionalDifferintegrator() = default;

    /**
     * @brief Build the Oustaloup approximation.
     * @param alpha      Fractional order in (-1, 1) (positive = differentiator).
     * @param wb         Lower band edge [rad/s] (> 0).
     * @param wh         Upper band edge [rad/s] (> wb).
     * @param N          Order parameter; the filter has 2N+1 sections (N >= 1).
     * @param sampleTime Sample period Ts [s] for the bilinear discretisation.
     */
    FractionalDifferintegrator(double alpha, double wb, double wh, int N, double sampleTime)
    {
        build(alpha, wb, wh, N, sampleTime);
    }

    /** @brief (Re)build the section bank. Resets all internal states. */
    void build(double alpha, double wb, double wh, int N, double sampleTime);

    /** @brief Advance one sample: returns the fractional operator applied to @p x. */
    double compute(double x);

    /** @brief Zero all section states. */
    void reset();

    /** @brief Overall static gain K = wh^{alpha} of the Oustaloup approximation. */
    double gain() const { return K_; }

    /** @brief Number of first-order sections (2N+1). */
    std::size_t sectionCount() const { return sec_.size(); }

private:
    /// One bilinear-discretised first-order section y = b0.x + b1.x1 - a1.y1 (a0 normalised to 1).
    struct Section
    {
        double b0 = 1.0, b1 = 0.0, a1 = 0.0;
        double x1 = 0.0, y1 = 0.0;
    };
    std::vector<Section> sec_; ///< Cascade of 2N+1 sections (sized at build()).
    double K_ = 1.0;           ///< Overall gain wh^{alpha}.
};

/**
 * @brief Tuning parameters for FractionalOrderPID.
 */
struct FOPIDParams
{
    double Kp = 1.0;      ///< Proportional gain.
    double Ki = 0.0;      ///< Fractional-integral gain (weights the D^{-lambda} branch).
    double Kd = 0.0;      ///< Fractional-derivative gain (weights the D^{mu} branch).

    double lambda = 1.0;  ///< Integral order in (0, 1]. lambda = 1 -> band-limited classical I.
    double mu     = 1.0;  ///< Derivative order in (0, 1]. mu = 1 -> band-limited classical D.

    double wb = 0.01;     ///< Lower Oustaloup band edge [rad/s].
    double wh = 100.0;    ///< Upper Oustaloup band edge [rad/s].
    int    N  = 4;        ///< Oustaloup order (2N+1 sections per operator).

    double uMin = -1e9;   ///< Lower output saturation limit.
    double uMax =  1e9;   ///< Upper output saturation limit.

    /**
     * @brief Anti-windup back-calculation gain on the fractional-integral branch (0 disables).
     *
     * On saturation the excess (u_sat - u_unsat) is fed back into the s^{-lambda} branch input,
     * unwinding it. The Oustaloup integrator already has finite DC gain, so windup is bounded
     * even with Kaw = 0; Kaw > 0 simply speeds recovery.
     */
    double Kaw = 1.0;
};

/**
 * @brief Discrete fractional-order PID controller (PI^{lambda} D^{mu}).
 *
 * A superset of the classical PID: the integral and derivative actions use non-integer orders
 * lambda and mu, giving an extra two degrees of freedom that can improve robustness on
 * diffusion-dominated / long-memory plants (thermal, electrochemical) where integer-order PID
 * struggles. Reduces to a (band-limited) classical PID at lambda = mu = 1.
 *
 * @see FOPIDParams for tuning parameters.
 */
class FractionalOrderPID : public IController
{
public:
    /**
     * @brief Construct with tuning parameters and a fixed sample time.
     * @param params     FOPID parameters (gains, fractional orders, Oustaloup band).
     * @param sampleTime Sample period Ts [s] > 0.
     */
    explicit FractionalOrderPID(const FOPIDParams &params, double sampleTime);

    /**
     * @brief Compute u[k] from tracking error e[k] = r[k] - y[k] (same convention as DiscretePID).
     * @param error Current tracking error.
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }

    /** @brief Reset both fractional operators, anti-windup state, and last output. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Hot-update tuning parameters. Rebuilds the Oustaloup filters and resets their
     *        internal states (fractional orders / band may have changed).
     */
    void setParams(const FOPIDParams &params);

    /** @brief Read-only access to current parameters. */
    const FOPIDParams &params() const { return p_; }

    /** @brief Control output produced at the previous sample (u[k-1]). */
    double lastOutput() const { return u_prev_; }

    /** @brief True when back-calculation anti-windup is active (Kaw != 0). */
    bool hasInternalAntiWindup() const override { return p_.Kaw != 0.0; }

private:
    FOPIDParams p_;
    double Ts_;
    FractionalDifferintegrator fi_; ///< s^{-lambda} fractional integrator.
    FractionalDifferintegrator fd_; ///< s^{mu} fractional differentiator.
    double u_prev_;                 ///< Previous saturated output u[k-1].
    double aw_prev_;                ///< Previous saturation error (u_sat - u_unsat) for anti-windup.
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(fractional_order_pid)
