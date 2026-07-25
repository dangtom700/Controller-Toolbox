#pragma once
#include "ControllerRegistry.h"
#include "IController.h"
#include <Eigen/Dense>
#include <memory>
#include <string>

/**
 * @file CascadeController.h
 * @brief Series (inner/outer) cascade composition of two IControllers.
 *
 * The outer loop regulates the slow variable and its output becomes the **setpoint**
 * of the fast inner loop:
 * @code
 *   sp[k] = clamp(rateLimit(outer.compute(r_out - y_out)), spMin, spMax)
 *   u[k]  = inner.compute(sp[k] - y_in)
 * @endcode
 *
 * This is a *series* hand-off and is therefore **not** expressible with
 * ControllerStack::Additive, which sums child outputs in parallel.
 *
 * **Usage:**
 * @code
 *   auto outer = std::make_shared<ctrl::DiscretePID>(pp_outer, Ts);
 *   auto inner = std::make_shared<ctrl::DiscretePID>(pp_inner, Ts);
 *   ctrl::CascadeParams cp;  cp.spMin = -5.0;  cp.spMax = 5.0;
 *   ctrl::CascadeController casc(outer, inner, cp, Ts);
 *
 *   casc.setInnerMeasurement(y_inner);
 *   double u = casc.compute(r_outer - y_outer);
 * @endcode
 *
 * **Inner sign convention is handled automatically.** The inner error is formed as
 * `sp - y_in`, except when `inner->signConvention()` reports TrackingErrorYMinusR
 * (DiscreteSMC, SuperTwistingSMC, ...), in which case `y_in - sp` is used instead.
 *
 * **Setpoint anti-windup.** When the outer command hits spMin/spMax the outer
 * controller is back-calculated via `bumplessInit(sp_clamped, e_out)`, which stops
 * its integrator winding up against a limit the inner loop can never deliver.
 * Disable with `CascadeParams::antiWindup = false`.
 *
 * @see CONTRIBUTING.md#sign-conventions
 * @see ControllerStack - for parallel (Additive/Weighted/Supervisory) composition.
 * @see Astrom & Hagglund (2006), Advanced PID Control, Ch. 3 (cascade control).
 */

namespace ctrl
{

/** @brief Tuning parameters for @ref CascadeController. */
struct CascadeParams
{
    double spMin = -1e9;      ///< Lower clamp on the inner setpoint produced by the outer loop.
    double spMax = 1e9;       ///< Upper clamp on the inner setpoint. Must exceed spMin.
    double spRateMax = 1e9;   ///< Max |d(setpoint)/dt| [setpoint-units/s]. Large = unlimited.
    int outerDecimation = 1;  ///< Run the outer loop every N inner ticks (multi-rate cascade); >= 1.
    bool antiWindup = true;   ///< Back-calculate the outer loop while the setpoint is clamped.
};

/**
 * @brief Cascade controller: outer loop drives the inner loop's setpoint.
 */
class CascadeController : public IController
{
public:
    /**
     * @brief Construct a cascade from an outer and an inner controller.
     * @param outer  Slow/primary loop; its output is interpreted as the inner setpoint.
     * @param inner  Fast/secondary loop; its output is the plant command.
     * @param params Clamp, rate-limit, decimation and anti-windup settings.
     * @param Ts     Sample time [s] of the (fast) cascade tick.
     * @throws std::invalid_argument If either controller is null, Ts <= 0,
     *         outerDecimation < 1, or spMin >= spMax.
     */
    CascadeController(std::shared_ptr<IController> outer,
                      std::shared_ptr<IController> inner,
                      const CascadeParams &params,
                      double Ts);

    /**
     * @brief Provide the inner-loop measurement for this step.
     * @param y_inner Measured fast variable y_in[k]; call before compute().
     */
    void setInnerMeasurement(double y_inner) noexcept { y_inner_ = y_inner; }

    // ---- IController -------------------------------------------------------

    /**
     * @brief Advance both loops one step.
     * @param outer_error Outer tracking error e_out = r_out - y_out.
     * @return Plant command u[k] from the inner loop.
     */
    double compute(double outer_error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }

    void reset() override;
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "CascadeController"; }

    /** @brief Healthy only when both loops report health. */
    bool isHealthy() const override { return outer_->isHealthy() && inner_->isHealthy(); }

    /**
     * @brief Reports the *inner* controller's anti-windup status.
     *
     * AntiWindupWrapper asks this before wrapping, and the inner loop is the one that
     * produces the saturated plant command - CascadeParams::antiWindup concerns the
     * outer loop's setpoint limit, which is a different signal.
     */
    bool hasInternalAntiWindup() const override { return inner_->hasInternalAntiWindup(); }

    /** @brief Prepare the inner loop to deliver @p u_target at the current setpoint. */
    void bumplessInit(double u_target, double error) override;

    // ---- Diagnostics -------------------------------------------------------

    /** @brief Inner setpoint applied on the last compute() call (post clamp + rate limit). */
    double innerSetpoint() const noexcept { return sp_prev_; }
    /** @brief true if the last outer command was altered by the clamp or rate limit. */
    bool setpointClamped() const noexcept { return sp_clamped_; }
    /** @brief Last plant command u[k-1]. */
    double lastOutput() const noexcept { return u_prev_; }
    /** @brief Mutable access to the outer controller. */
    const std::shared_ptr<IController> &outerController() const noexcept { return outer_; }
    /** @brief Mutable access to the inner controller. */
    const std::shared_ptr<IController> &innerController() const noexcept { return inner_; }

private:
    std::shared_ptr<IController> outer_; ///< Slow loop; output = inner setpoint.
    std::shared_ptr<IController> inner_; ///< Fast loop; output = plant command.
    CascadeParams p_;
    double Ts_;

    bool inner_flip_ = false; ///< true when the inner loop wants e = y - r.
    double y_inner_ = 0.0;    ///< Latest inner measurement from setInnerMeasurement().
    double sp_prev_ = 0.0;    ///< Applied inner setpoint sp[k-1] (also the decimation hold value).
    double u_prev_ = 0.0;     ///< Previous plant command u[k-1] (NaN-guard hold value).
    bool sp_clamped_ = false; ///< Clamp/rate-limit activity flag for the last step.
    long long tick_ = 0;      ///< Inner-tick counter driving outerDecimation.

    mutable Eigen::VectorXd notify_buf_{Eigen::VectorXd::Constant(1, 0.0)};
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(cascade_controller)
