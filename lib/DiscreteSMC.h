#pragma once
#include "IController.h"
#include "ControllerRegistry.h"

/**
 * @file DiscreteSMC.h
 * @brief Discrete-time Sliding Mode Controllers - first-order SMC, super-twisting (2nd-order),
 *        nonsingular terminal (finite-time), and adaptive-gain SMC.
 *
 * @see Utkin, "Sliding Modes in Control and Optimization" (1992).
 * @see Edwards & Spurgeon, "Sliding Mode Control: Theory and Applications" (1998).
 * @see Levant, "Sliding order and sliding accuracy in SMC", IJRNLC (1993).
 * @see Moreno & Osorio, "A Lyapunov approach to super-twisting", IEEE TAC (2012).
 * @see Feng, Yu & Man, "Non-singular terminal sliding mode control of rigid manipulators",
 *      Automatica 38 (2002).
 * @see Plestan, Shtessel, Bregeault & Poznyak, "New methodologies for adaptive sliding mode
 *      control", Int. J. Control 83 (2010).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for DiscreteSMC.
 */
struct SMCParams
{
    /**
     * @brief Error weight in the sliding surface.
     *
     * Together with c_de, determines the sliding surface bandwidth. Setting sigma = 0 and
     * substituting the discrete derivative gives the continuous convergence rate:
     * @code
     *   omega_s = c_e . Ts / c_de   [rad/s]
     * @endcode
     * Choose c_e and c_de so that omega_s matches the desired closed-loop bandwidth. Larger
     * c_e relative to c_de -> faster convergence but higher sensitivity to noise in e.
     */
    double c_e  = 1.0;

    /**
     * @brief Error-rate weight in the sliding surface (discrete, stores lambda.Ts).
     *
     * @par Sample-time dependence (calibration trap)
     * The discrete derivative is approximated as (e[k] - e[k-1]) / Ts. Therefore c_de is
     * implicitly Ts-dependent: halving Ts doubles the surface sensitivity without any
     * parameter change.
     *
     * **Convention:** c_de stores the **discrete** coefficient, i.e., c_de already absorbs Ts.
     * To convert from a continuous-time slope lambda [1/s]:
     * @code
     *   c_de = lambda * Ts
     * @endcode
     * Recalculate c_de whenever Ts changes.
     *
     * Larger c_de -> faster convergence to the sliding surface but more chattering.
     */
    double c_de = 0.1;

    double K    = 5.0;   ///< Switching gain. Larger = more robust, more chattering.
    double phi  = 0.5;   ///< Boundary-layer thickness. Larger = smoother, slower.
    double uMin = -1e9;  ///< Output saturation lower limit.
    double uMax =  1e9;  ///< Output saturation upper limit.
};

/**
 * @brief Discrete first-order Sliding Mode Controller with boundary-layer saturation.
 *
 * **Sliding surface:**
 * @code
 *   s[k] = c_e.e[k] + c_de.(e[k] - e[k-1])
 * @endcode
 *
 * **Control law (sat replaces sign to reduce chattering):**
 * @code
 *   u[k] = -K.sat(s[k] / phi)
 *   sat(x) = x          if |x| <= 1   (linear PD inside the boundary layer)
 *   sat(x) = sign(x)    if |x| > 1   (relay switching outside)
 * @endcode
 *
 * Setting phi -> 0 recovers ideal relay SMC with chattering.
 * Setting phi large gives a soft PD approximation.
 */
class DiscreteSMC : public IController
{
public:
    /**
     * @brief Construct with tuning parameters and fixed sample time.
     * @param params     SMC parameters (surface weights, gain, boundary layer, limits).
     * @param sampleTime Sample period Ts [s].
     */
    explicit DiscreteSMC(const SMCParams &params, double sampleTime);

    /**
     * @brief Compute u[k] from tracking error e[k] = y[k] - r[k] (sign reversed from PID -
     *        see CONTRIBUTING.md#sign-conventions). The sliding law u = -K*sat(s/phi) requires
     *        s, and therefore @p error, to grow with y - r for a positive-gain plant.
     * @param error Current tracking error, e = y - r.
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorYMinusR; }

    /** @brief Reset previous error, sliding surface, and output. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Hot-update tuning parameters.
     * @param p New SMC parameters.
     */
    void setParams(const SMCParams &p) { p_ = p; }

    /** @brief Read-only access to current parameters. */
    const SMCParams &params() const { return p_; }

    /**
     * @brief Sliding surface value from the previous sample s[k-1].
     *
     * Returns `s_prev_`, the surface computed during the last `compute()` call.
     * Query immediately after `compute()` to read the surface used to produce the
     * most recent output. |slidingSurface()| < phi indicates operation inside the
     * boundary layer (linear PD regime).
     */
    double slidingSurface() const { return s_prev_; }

private:
    SMCParams p_;
    double Ts_;
    double e_prev_; ///< Previous error e[k-1].
    double s_prev_; ///< Previous sliding surface s[k-1].
    double u_prev_; ///< Previous output u[k-1].
    mutable Eigen::VectorXd notify_buf_{Eigen::VectorXd::Constant(1, 0.0)};
};

// -----------------------------------------------------------------------------

/**
 * @brief Tuning parameters for SuperTwistingSMC.
 */
struct SuperTwistingParams
{
    double c_e  = 1.0;  ///< Error weight in the sliding surface (same convention as SMCParams).
    double c_de = 0.1;  ///< Error-rate weight (discrete; stores lambda.Ts - see SMCParams::c_de).
    double K1   = 3.0;  ///< Power-term gain (|s|^{1/2} component).
    double K2   = 5.0;  ///< Integral gain (sign(s) component). Must satisfy K2 > K1^2 / 4.
    double uMin = -1e9; ///< Output saturation lower limit.
    double uMax =  1e9; ///< Output saturation upper limit.
};

/**
 * @brief Discrete 2nd-order Sliding Mode Controller (super-twisting algorithm).
 *
 * Eliminates chattering without a boundary layer while retaining finite-time convergence
 * and robustness to matched disturbances bounded by a known Lipschitz constant.
 *
 * **Continuous-time super-twisting law (Levant 1993):**
 * @code
 *   u[k]   = v[k] - K1.|s|^{1/2}.sign(s)
 *   v.[k]   = -K2.sign(s)
 * @endcode
 *
 * **Discrete approximation (Euler integration of v):**
 * @code
 *   s[k]   = c_e.e[k] + c_de.(e[k] - e[k-1])
 *   u[k]   = v[k] - K1.|s[k]|^{1/2}.sign(s[k])
 *   v[k+1] = v[k] - K2.Ts.sign(s[k])
 * @endcode
 *
 * **Gain selection (Moreno & Osorio 2008):** K1 > 0 and K2 > K1^2/4 is sufficient for
 * finite-time convergence in continuous time. In discrete time, keep K2.Ts << K1 to
 * limit discretisation error.
 */
class SuperTwistingSMC : public IController
{
public:
    /**
     * @brief Construct with tuning parameters and fixed sample time.
     * @param params     Super-twisting parameters.
     * @param sampleTime Sample period Ts [s].
     */
    explicit SuperTwistingSMC(const SuperTwistingParams &params, double sampleTime);

    /**
     * @brief Compute u[k] from tracking error e[k] = y[k] - r[k] (sign reversed from PID -
     *        same convention as DiscreteSMC, see CONTRIBUTING.md#sign-conventions).
     * @param error Current tracking error, e = y - r.
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorYMinusR; }

    /** @brief Reset previous error, sliding surface, and integrator state v. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Hot-update tuning parameters.
     * @param p New super-twisting parameters.
     */
    void setParams(const SuperTwistingParams &p) { p_ = p; }

    /** @brief Read-only access to current parameters. */
    const SuperTwistingParams &params() const { return p_; }

    /**
     * @brief Current sliding surface value s[k] (for diagnostics).
     * @return s[k] = c_e.e[k] + c_de.(e[k] - e[k-1]).
     */
    double slidingSurface() const { return s_prev_; }

private:
    SuperTwistingParams p_;
    double Ts_;
    double e_prev_; ///< Previous error e[k-1].
    double s_prev_; ///< Previous sliding surface s[k-1].
    double v_;      ///< Integrator state of the super-twisting algorithm.
};

// -----------------------------------------------------------------------------

/**
 * @brief Tuning parameters for NonsingularTerminalSMC.
 */
struct NonsingularTerminalSMCParams
{
    double c_e = 1.0;   ///< Error weight in the sliding surface (same role as SMCParams::c_e).

    /**
     * @brief Terminal-surface coefficient beta > 0.
     *
     * The terminal sliding surface is `s = c_e.e + (1/beta).|de|^{gamma}.sign(de)`, where
     * `de = e[k] - e[k-1]` is the per-step error change. On the surface (s = 0) the error
     * decays to zero in *finite* time (unlike a linear surface's asymptotic convergence).
     *
     * **Discrete convention:** as with SMCParams::c_de, the coefficient absorbs the sample
     * time - beta scales the raw difference `de` (not `de/Ts`), so it implicitly carries a
     * `Ts^{gamma}` factor. Recompute beta if Ts changes. Larger beta -> stronger terminal pull.
     */
    double beta = 1.0;

    /**
     * @brief Fractional power gamma = p/q in the open interval (1, 2).
     *
     * Values in (1, 2) keep the control law nonsingular (only positive powers of |de| appear),
     * while still yielding finite-time convergence because 1/gamma is in (0.5, 1). gamma -> 1
     * recovers a near-linear surface (asymptotic); gamma -> 2 gives the strongest terminal pull.
     */
    double gamma = 1.5;

    double K   = 5.0;   ///< Switching gain (robust reaching term). Larger = more robust.
    double eta = 1.0;   ///< Proportional reaching gain (adds -eta.s for a smoother approach).
    double phi = 0.5;   ///< Boundary-layer thickness (sat replaces sign to limit chattering).
    double uMin = -1e9; ///< Output saturation lower limit.
    double uMax =  1e9; ///< Output saturation upper limit.
};

/**
 * @brief Discrete nonsingular terminal Sliding Mode Controller (finite-time convergence).
 *
 * Fills the finite-time gap between first-order @ref DiscreteSMC (asymptotic linear surface)
 * and @ref SuperTwistingSMC (2nd-order). The nonsingular terminal surface adds a fractional
 * power of the error rate so that, once on the surface, the tracking error reaches zero in
 * finite time rather than only exponentially.
 *
 * **Terminal sliding surface (Feng/Yu/Man 2002, discrete form):**
 * @code
 *   de[k] = e[k] - e[k-1]                                        (per-step error change)
 *   s[k]  = c_e.e[k] + (1/beta).|de[k]|^{gamma}.sign(de[k]),     1 < gamma < 2
 * @endcode
 *
 * **Model-free reaching law (sat-smoothed, nonsingular):**
 * @code
 *   u[k] = -( K.sat(s[k]/phi) + eta.s[k] )
 * @endcode
 *
 * The fractional term is only ever raised to a *positive* power of |de|, so no singularity
 * arises (that is the "nonsingular" property). Using the raw difference `de` (rather than
 * `de/Ts`) keeps the surface Ts-robust - beta absorbs the sample-time scaling, matching the
 * discrete convention of SMCParams::c_de. Setting gamma -> 1 degrades gracefully to a
 * first-order-like surface.
 */
class NonsingularTerminalSMC : public IController
{
public:
    /**
     * @brief Construct with tuning parameters and fixed sample time.
     * @param params     Terminal-SMC parameters.
     * @param sampleTime Sample period Ts [s].
     */
    explicit NonsingularTerminalSMC(const NonsingularTerminalSMCParams &params, double sampleTime);

    /**
     * @brief Compute u[k] from tracking error e[k] = y[k] - r[k] (sign reversed from PID -
     *        same convention as DiscreteSMC, see CONTRIBUTING.md#sign-conventions).
     * @param error Current tracking error, e = y - r.
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorYMinusR; }

    /** @brief Reset previous error, sliding surface, and output. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Hot-update tuning parameters. */
    void setParams(const NonsingularTerminalSMCParams &p) { p_ = p; }

    /** @brief Read-only access to current parameters. */
    const NonsingularTerminalSMCParams &params() const { return p_; }

    /** @brief Sliding surface value from the previous sample s[k-1] (diagnostics). */
    double slidingSurface() const { return s_prev_; }

private:
    NonsingularTerminalSMCParams p_;
    double Ts_;
    double e_prev_; ///< Previous error e[k-1].
    double s_prev_; ///< Previous sliding surface s[k-1].
    double u_prev_; ///< Previous output u[k-1].
    mutable Eigen::VectorXd notify_buf_{Eigen::VectorXd::Constant(1, 0.0)};
};

// -----------------------------------------------------------------------------

/**
 * @brief Tuning parameters for AdaptiveSMC.
 */
struct AdaptiveSMCParams
{
    double c_e  = 1.0;  ///< Error weight in the sliding surface (same convention as SMCParams).
    double c_de = 0.1;  ///< Error-rate weight (discrete; stores lambda.Ts - see SMCParams::c_de).

    /**
     * @brief Gain adaptation rate gamma > 0.
     *
     * The switching gain integrates `Kdot = gamma.(|s| - epsilon)`, so K rises while the
     * trajectory is off the sliding surface (|s| > epsilon) and relaxes once inside the
     * dead-band. Larger gamma = faster gain growth (quicker rejection of an unknown-bound
     * disturbance) but more overshoot in K.
     */
    double gamma = 5.0;

    double epsilon = 0.02; ///< Dead-band on |s|; K stops growing (and slowly decays) below it.
    double K0   = 0.5;     ///< Initial switching gain (adaptation starts here on reset()).
    double Kmin = 0.0;     ///< Lower clamp on the adaptive gain.
    double Kmax = 1e3;     ///< Upper clamp on the adaptive gain (safety ceiling).
    double phi  = 0.5;     ///< Boundary-layer thickness.
    double uMin = -1e9;    ///< Output saturation lower limit.
    double uMax =  1e9;    ///< Output saturation upper limit.
};

/**
 * @brief Discrete adaptive-gain Sliding Mode Controller (no a-priori disturbance bound).
 *
 * Classic first-order SMC requires the switching gain K to exceed the (usually unknown) bound
 * of the matched disturbance. This variant *adapts* K online, so the designer need not know the
 * bound: K grows until the trajectory is confined to the sliding boundary layer, then holds.
 * The library previously lacked this; the Nonlinear Surface Ship case study had to hand-roll an
 * "ASMC" - this promotes the technique to a first-class controller.
 *
 * **Sliding surface:**
 * @code
 *   s[k] = c_e.e[k] + c_de.(e[k] - e[k-1])
 * @endcode
 *
 * **Gain adaptation (Plestan et al. 2010, discretised) and control:**
 * @code
 *   u[k]   = -K[k].sat(s[k]/phi)
 *   K[k+1] = clamp( K[k] + Ts.gamma.(|s[k]| - epsilon), Kmin, Kmax )
 * @endcode
 */
class AdaptiveSMC : public IController
{
public:
    /**
     * @brief Construct with tuning parameters and fixed sample time.
     * @param params     Adaptive-SMC parameters.
     * @param sampleTime Sample period Ts [s].
     */
    explicit AdaptiveSMC(const AdaptiveSMCParams &params, double sampleTime);

    /**
     * @brief Compute u[k] from tracking error e[k] = y[k] - r[k] (sign reversed from PID -
     *        same convention as DiscreteSMC, see CONTRIBUTING.md#sign-conventions).
     * @param error Current tracking error, e = y - r.
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorYMinusR; }

    /** @brief Reset previous error, sliding surface, output, and adaptive gain to K0. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Hot-update tuning parameters (does not reset the current adaptive gain). */
    void setParams(const AdaptiveSMCParams &p) { p_ = p; }

    /** @brief Read-only access to current parameters. */
    const AdaptiveSMCParams &params() const { return p_; }

    /** @brief Sliding surface value from the previous sample s[k-1] (diagnostics). */
    double slidingSurface() const { return s_prev_; }

    /** @brief Current value of the online-adapted switching gain K[k]. */
    double adaptiveGain() const { return K_; }

private:
    AdaptiveSMCParams p_;
    double Ts_;
    double e_prev_; ///< Previous error e[k-1].
    double s_prev_; ///< Previous sliding surface s[k-1].
    double u_prev_; ///< Previous output u[k-1].
    double K_;      ///< Current adaptive switching gain K[k].
    mutable Eigen::VectorXd notify_buf_{Eigen::VectorXd::Constant(1, 0.0)};
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(terminal_smc)
CTRL_REGISTER_FEATURE(adaptive_smc)
