#pragma once
#include "IController.h"

/**
 * @file DiscreteSMC.h
 * @brief Discrete-time Sliding Mode Controllers — first-order SMC and super-twisting SMC.
 *
 * @see Utkin, "Sliding Modes in Control and Optimization" (1992).
 * @see Edwards & Spurgeon, "Sliding Mode Control: Theory and Applications" (1998).
 * @see Levant, "Sliding order and sliding accuracy in SMC", IJRNLC (1993).
 * @see Moreno & Osorio, "A Lyapunov approach to super-twisting", IEEE TAC (2012).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for DiscreteSMC.
 */
struct SMCParams
{
    double c_e  = 1.0;  ///< Error weight in the sliding surface.

    /**
     * @brief Error-rate weight in the sliding surface (discrete, stores λ·Ts).
     *
     * @par Sample-time dependence (calibration trap)
     * The discrete derivative is approximated as (e[k] − e[k−1]) / Ts. Therefore c_de is
     * implicitly Ts-dependent: halving Ts doubles the surface sensitivity without any
     * parameter change.
     *
     * **Convention:** c_de stores the **discrete** coefficient, i.e., c_de already absorbs Ts.
     * To convert from a continuous-time slope λ [1/s]:
     * @code
     *   c_de = lambda * Ts
     * @endcode
     * Recalculate c_de whenever Ts changes.
     *
     * Larger c_de → faster convergence to the sliding surface but more chattering.
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
 *   s[k] = c_e·e[k] + c_de·(e[k] − e[k−1])
 * @endcode
 *
 * **Control law (sat replaces sign to reduce chattering):**
 * @code
 *   u[k] = −K·sat(s[k] / φ)
 *   sat(x) = x          if |x| ≤ 1   (linear PD inside the boundary layer)
 *   sat(x) = sign(x)    if |x| > 1   (relay switching outside)
 * @endcode
 *
 * Setting φ → 0 recovers ideal relay SMC with chattering.
 * Setting φ large gives a soft PD approximation.
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
     * @brief Compute u[k] from tracking error e[k] = r[k] − y[k].
     * @param error Current tracking error.
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

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
     * @brief Current sliding surface value s[k].
     *
     * Useful for monitoring convergence. |s[k]| < φ indicates operation inside the
     * boundary layer (linear PD regime).
     */
    double slidingSurface() const { return s_prev_; }

private:
    SMCParams p_;
    double Ts_;
    double e_prev_; ///< Previous error e[k−1].
    double s_prev_; ///< Previous sliding surface s[k−1].
    double u_prev_; ///< Previous output u[k−1].
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Tuning parameters for SuperTwistingSMC.
 */
struct SuperTwistingParams
{
    double c_e  = 1.0;  ///< Error weight in the sliding surface (same convention as SMCParams).
    double c_de = 0.1;  ///< Error-rate weight (discrete; stores λ·Ts — see SMCParams::c_de).
    double K1   = 3.0;  ///< Power-term gain (|s|^{1/2} component).
    double K2   = 5.0;  ///< Integral gain (sign(s) component). Must satisfy K2 > K1² / 4.
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
 *   u[k]   = v[k] − K₁·|s|^{1/2}·sign(s)
 *   v̇[k]   = −K₂·sign(s)
 * @endcode
 *
 * **Discrete approximation (Euler integration of v):**
 * @code
 *   s[k]   = c_e·e[k] + c_de·(e[k] − e[k−1])
 *   u[k]   = v[k] − K₁·|s[k]|^{1/2}·sign(s[k])
 *   v[k+1] = v[k] − K₂·Ts·sign(s[k])
 * @endcode
 *
 * **Gain selection (Moreno & Osorio 2008):** K₁ > 0 and K₂ > K₁²/4 is sufficient for
 * finite-time convergence in continuous time. In discrete time, keep K₂·Ts ≪ K₁ to
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
     * @brief Compute u[k] from tracking error e[k] = r[k] − y[k].
     * @param error Current tracking error.
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

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
     * @return s[k] = c_e·e[k] + c_de·(e[k] − e[k−1]).
     */
    double slidingSurface() const { return s_prev_; }

private:
    SuperTwistingParams p_;
    double Ts_;
    double e_prev_; ///< Previous error e[k−1].
    double s_prev_; ///< Previous sliding surface s[k−1].
    double v_;      ///< Integrator state of the super-twisting algorithm.
};

} // namespace ctrl
