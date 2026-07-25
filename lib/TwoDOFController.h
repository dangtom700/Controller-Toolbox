#pragma once
#include "ControllerRegistry.h"
#include "IController.h"
#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <string>

/**
 * @file TwoDOFController.h
 * @brief Two-degree-of-freedom control: functional feedforward plus a feedback trim.
 *
 * @code
 *   u[k] = clamp( u_ff(r[k], d[k]) + feedback.compute(e[k]) )
 * @endcode
 *
 * The feedforward is an arbitrary user callable, which is what separates this class
 * from the two existing near-neighbours:
 *
 * | Class                          | Feedforward source                              |
 * |--------------------------------|-------------------------------------------------|
 * | `FeedforwardController`        | a designed StateSpace filter G_ff(z) applied to r |
 * | `DiscretePID` (`b_weight`)     | setpoint weighting *inside* one PID              |
 * | `TwoDOFController` (this)      | any `double(double r, double d)` callable        |
 *
 * Use this one when the feedforward comes from a physics inversion or a measured
 * exogenous signal rather than a transfer function - the dominant case in practice
 * (steady-state duty from a converter mode, trim input from an equilibrium solve,
 * compressor speed matching a measured thermal load, ...).
 *
 * **Usage:**
 * @code
 *   auto ff = [](double r, double d) { return 0.42 * r + 0.1 * d; };
 *   ctrl::TwoDOFParams tp;  tp.uMin = -5.0;  tp.uMax = 5.0;
 *   ctrl::TwoDOFController c2(pid, ff, tp, Ts);
 *
 *   c2.setReference(r);
 *   c2.setMeasuredDisturbance(d);      // optional; defaults to 0
 *   double u = c2.compute(r - y);
 * @endcode
 *
 * **Anti-windup.** When the total command saturates, the feedback controller is
 * back-calculated via `bumplessInit(u_sat - u_ff, error)` so its integrator settles on
 * the achievable trim instead of winding up against the limit. Disable with
 * `TwoDOFParams::antiWindup = false` (e.g. when the feedback path has no integrator).
 *
 * @see FeedforwardController - use that instead when you have a designed G_ff(z).
 * @see Astrom & Hagglund (2006), Advanced PID Control, Ch. 6 (2-DOF structures).
 */

namespace ctrl
{

/** @brief Feedforward callable: (reference r, measured disturbance d) -> u_ff. */
using FeedforwardFn = std::function<double(double, double)>;

/** @brief Tuning parameters for @ref TwoDOFController. */
struct TwoDOFParams
{
    double uMin = -1e9;     ///< Lower saturation limit on the total command.
    double uMax = 1e9;      ///< Upper saturation limit on the total command.
    bool antiWindup = true; ///< Back-calculate the feedback path while the total is clamped.
};

/**
 * @brief Two-degree-of-freedom controller: u = u_ff(r, d) + feedback trim.
 */
class TwoDOFController : public IController
{
public:
    /**
     * @brief Construct from a feedback controller and a feedforward callable.
     * @param feedback Feedback (trim) controller.
     * @param ff       Feedforward map (r, d) -> u_ff. Must be callable and finite-valued.
     * @param params   Saturation and anti-windup settings.
     * @param Ts       Sample time [s].
     * @throws std::invalid_argument If feedback is null, ff is empty, Ts <= 0,
     *         or uMin >= uMax.
     */
    TwoDOFController(std::shared_ptr<IController> feedback,
                     FeedforwardFn ff,
                     const TwoDOFParams &params,
                     double Ts);

    /** @brief Set the reference r[k] passed to the feedforward map. */
    void setReference(double r) noexcept { r_ = r; }

    /** @brief Set the measured disturbance d[k] passed to the feedforward map. */
    void setMeasuredDisturbance(double d) noexcept { d_ = d; }

    // ---- IController -------------------------------------------------------

    /**
     * @brief Advance the feedback loop and add the feedforward term.
     * @param error Tracking error e = r - y.
     * @return Saturated total command u[k].
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }

    void reset() override;
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "TwoDOFController"; }

    bool isHealthy() const override { return feedback_->isHealthy(); }

    /** @brief true when this class or the feedback controller handles windup. */
    bool hasInternalAntiWindup() const override
    {
        return p_.antiWindup || feedback_->hasInternalAntiWindup();
    }

    void bumplessInit(double u_target, double error) override;

    // ---- Diagnostics -------------------------------------------------------

    /** @brief Feedforward contribution u_ff from the last compute() call. */
    double feedforwardTerm() const noexcept { return u_ff_; }
    /** @brief Feedback contribution from the last compute() call. */
    double feedbackTerm() const noexcept { return u_fb_; }
    /** @brief true if the total command was saturated on the last call. */
    bool saturated() const noexcept { return saturated_; }
    /** @brief Last applied command u[k-1]. */
    double lastOutput() const noexcept { return u_prev_; }
    /** @brief Mutable access to the feedback controller. */
    const std::shared_ptr<IController> &feedbackController() const noexcept { return feedback_; }

private:
    std::shared_ptr<IController> feedback_;
    FeedforwardFn ff_;
    TwoDOFParams p_;
    double Ts_;

    double r_ = 0.0;         ///< Reference handed to the feedforward map.
    double d_ = 0.0;         ///< Measured disturbance handed to the feedforward map.
    double u_ff_ = 0.0;      ///< Last feedforward contribution.
    double u_fb_ = 0.0;      ///< Last feedback contribution.
    bool saturated_ = false; ///< Saturation flag for the last step.
    double u_prev_ = 0.0;    ///< Previous command u[k-1] (NaN-guard hold value).

    mutable Eigen::VectorXd notify_buf_{Eigen::VectorXd::Constant(1, 0.0)};
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(two_dof_controller)
