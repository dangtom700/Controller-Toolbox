#pragma once
#include "ControllerRegistry.h"
#include "DiscreteSMC.h"
#include "FuzzyLogic.h"
#include "IController.h"
#include <Eigen/Dense>
#include <string>

/**
 * @file FuzzySlidingModeController.h
 * @brief Fuzzy-scheduled sliding-mode control (FSMC) - fuzzy inference retunes the
 *        switching gain and boundary layer every step.
 *
 * Fixed-gain SMC forces a single compromise: a large switching gain K rejects matched
 * disturbances but chatters, and a wide boundary layer phi smooths the command but
 * leaves steady-state error. FSMC schedules both on how far the state is from the
 * sliding surface:
 * @code
 *   s[k] = c_e.e[k] + c_de.(e[k] - e[k-1])              (same surface as DiscreteSMC)
 *   m[k] = |FuzzyPD(s[k])| / u_scale       in [0, 1]    (25-rule Mamdani on (s, s_dot))
 *   K[k]   = clamp(K_nom  .(1 + gainSpan.m[k]), Kmin,   Kmax)
 *   phi[k] = clamp(phi_nom.(1 + phiSpan .m[k]), phiMin, phiMax)
 *   u[k]   = -K[k] . sat(s[k] / phi[k])
 * @endcode
 *
 * Far from the surface m -> 1, so K and phi both grow: fast reaching without relay
 * switching. Near the surface m -> 0 and both fall back to nominal, giving precision
 * with a narrow boundary layer. The result is lower total control variation than a
 * fixed-gain DiscreteSMC at the same tracking accuracy.
 *
 * **Sign convention: `compute()` takes e = y - r**, inherited from DiscreteSMC and the
 * reverse of DiscretePID. See CONTRIBUTING.md#sign-conventions.
 *
 * **Usage:**
 * @code
 *   ctrl::FuzzySMCParams fp;
 *   fp.smc.c_e = 1.0;  fp.smc.c_de = 5.0 * Ts;  fp.smc.K = 5.0;  fp.smc.phi = 0.1;
 *   fp.smc.uMin = -10.0;  fp.smc.uMax = 10.0;
 *   fp.fuzzy.e_scale = 1.0;    // typical |s|
 *   fp.fuzzy.de_scale = 10.0;  // typical |s_dot|
 *   fp.fuzzy.u_scale = 1.0;    // modulation universe; m = |fuzzy| / u_scale
 *   fp.Kmin = 0.5;  fp.Kmax = 20.0;  fp.phiMin = 0.01;  fp.phiMax = 1.0;
 *   ctrl::FuzzySlidingModeController fsmc(fp, Ts);
 *
 *   double u = fsmc.compute(y - r);        // note the SMC sign convention
 * @endcode
 *
 * @note `fuzzy.e_scale` normalises the **sliding surface** s (not the tracking error)
 *       and `fuzzy.de_scale` normalises s_dot = (s[k]-s[k-1])/Ts. Set them to the
 *       typical magnitudes of those signals, not of e.
 *
 * @see DiscreteSMC - the underlying reaching law; SuperTwistingSMC for the 2nd-order
 *      chattering-free alternative that needs no boundary layer.
 * @see Palm, R. (1994). Robust control by fuzzy sliding mode. Automatica 30(9), 1429-1437.
 */

namespace ctrl
{

/** @brief Tuning parameters for @ref FuzzySlidingModeController. */
struct FuzzySMCParams
{
    SMCParams smc;       ///< Nominal surface (c_e, c_de), gain K, boundary layer phi, u limits.
    FuzzyPDParams fuzzy; ///< Inference scaling; e_scale/de_scale normalise s and s_dot.

    double gainSpan = 0.8; ///< K modulation depth: K = K_nom.(1 + gainSpan.m). Must be > -1.
    double phiSpan = 0.8;  ///< phi modulation depth: phi = phi_nom.(1 + phiSpan.m). Must be > -1.

    double Kmin = 0.0;     ///< Hard lower bound on the scheduled switching gain.
    double Kmax = 1e9;     ///< Hard upper bound on the scheduled switching gain.
    double phiMin = 1e-6;  ///< Hard lower bound on the boundary layer; must be > 0.
    double phiMax = 1e9;   ///< Hard upper bound on the boundary layer.
};

/**
 * @brief Sliding-mode controller whose gain and boundary layer are fuzzy-scheduled.
 */
class FuzzySlidingModeController : public IController
{
public:
    /**
     * @brief Construct with tuning parameters and fixed sample time.
     * @param params FSMC parameters (nominal SMC, fuzzy scaling, spans, bounds).
     * @param sampleTime Sample period Ts [s].
     * @throws std::invalid_argument If Ts <= 0, fuzzy.u_scale <= 0, phiMin <= 0,
     *         Kmin >= Kmax, phiMin >= phiMax, or either span <= -1.
     */
    explicit FuzzySlidingModeController(const FuzzySMCParams &params, double sampleTime);

    // ---- IController -------------------------------------------------------

    /**
     * @brief Compute u[k] from tracking error e[k] = y[k] - r[k].
     * @param error Current tracking error, e = y - r (reversed from DiscretePID).
     * @return Saturated control output u[k].
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorYMinusR; }

    void reset() override;
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "FuzzySlidingModeController"; }

    // ---- Tuning ------------------------------------------------------------

    /**
     * @brief Hot-update tuning parameters (rebuilds the fuzzy inference system).
     * @param p New FSMC parameters.
     * @throws std::invalid_argument On the same conditions as the constructor.
     */
    void setParams(const FuzzySMCParams &p);

    /** @brief Read-only access to current parameters. */
    const FuzzySMCParams &params() const noexcept { return p_; }

    // ---- Diagnostics -------------------------------------------------------

    /** @brief Switching gain K[k] applied on the last compute() call. */
    double switchingGain() const noexcept { return K_; }
    /** @brief Boundary-layer thickness phi[k] applied on the last compute() call. */
    double boundaryLayer() const noexcept { return phi_; }
    /** @brief Sliding surface s[k] from the last compute() call. */
    double surface() const noexcept { return s_; }
    /** @brief Fuzzy modulation magnitude m[k] in [0, 1] from the last compute() call. */
    double modulation() const noexcept { return m_; }
    /** @brief Last control output u[k-1]. */
    double lastOutput() const noexcept { return u_prev_; }

private:
    /** @brief Shared validation for the constructor and setParams(). */
    static void validate(const FuzzySMCParams &p, double Ts);

    FuzzySMCParams p_;
    double Ts_;

    DiscreteSMC smc_; ///< Reaching law; its params are rewritten every step.
    FuzzyPD fuzzy_;   ///< Inference block driven by the sliding surface.

    double e_prev_ = 0.0; ///< Previous error e[k-1]; mirrors DiscreteSMC's own copy.
    double s_ = 0.0;      ///< Sliding surface s[k].
    double m_ = 0.0;      ///< Fuzzy modulation magnitude in [0, 1].
    double K_ = 0.0;      ///< Scheduled switching gain.
    double phi_ = 0.0;    ///< Scheduled boundary layer.
    double u_prev_ = 0.0; ///< Previous output u[k-1] (NaN-guard hold value).

    mutable Eigen::VectorXd notify_buf_{Eigen::VectorXd::Zero(3)};
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(fuzzy_smc)
