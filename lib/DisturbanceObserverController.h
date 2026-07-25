#pragma once
#include "ControllerRegistry.h"
#include "IController.h"
#include "PlantModel.h"
#include <Eigen/Dense>
#include <memory>
#include <string>

/**
 * @file DisturbanceObserverController.h
 * @brief Q-filter disturbance observer (DOB) wrapped around any IController.
 *
 * Estimates the *total* disturbance (external input plus model error) by comparing the
 * measured output against a nominal model driven by the same command, then feeds the
 * estimate forward to cancel it before the inner controller has to react:
 * @code
 *   y_nom[k] = C.x_nom[k] + D.u[k-1]
 *   d_hat[k] = Q(z) . (y[k] - y_nom[k]) / gainDC
 *   u[k]     = clamp(inner.compute(e[k]) - d_hat[k])
 *   x_nom[k+1] = A.x_nom[k] + B.u[k]
 * @endcode
 *
 * Q(z) is a unity-DC-gain low-pass (ZOH-discretised, order 1 or 2) that makes the
 * otherwise-improper nominal inverse realisable; its cutoff omega_q sets the rejection
 * bandwidth and trades disturbance rejection against measurement-noise injection.
 *
 * **Usage:**
 * @code
 *   ctrl::DOBParams dp;  dp.omega_q = 5.0;  dp.gainDC = 1.0;
 *   ctrl::DisturbanceObserverController dob(pi, sys_nom, dp, Ts);
 *
 *   dob.setPlantOutput(y);           // measured output, before compute()
 *   double u = dob.compute(r - y);
 * @endcode
 *
 * @note The nominal model is advanced with the **applied** command u[k] (the inner
 *       output *after* disturbance cancellation), which is the textbook DOB form.
 *       examples/ex52_dob_pi.cpp hand-rolls a variant that drives the nominal model
 *       with the raw PI output instead - that under-estimates d when the DOB is active.
 * @note If setPlantOutput() is never called, `y ~= -error` is assumed (the same
 *       fallback AdaptiveSmithPredictor uses); pass the real measurement whenever the
 *       reference is non-zero.
 *
 * @see Ohishi, K. et al. (1987). Microprocessor-controlled DC motor for load-insensitive
 *      position servo system. IEEE Trans. Ind. Electron. 34(1), 44-49.
 * @see DiscreteADRC - the extended-state-observer alternative (ESO is internal there).
 */

namespace ctrl
{

/** @brief Tuning parameters for @ref DisturbanceObserverController. */
struct DOBParams
{
    double omega_q = 5.0;  ///< Q-filter cutoff [rad/s]. Higher = faster rejection, more noise.
    int qOrder = 1;        ///< Q-filter order: 1 or 2 (2 = cascaded, sharper roll-off).
    double gainDC = 1.0;   ///< Nominal plant DC gain; converts an output disturbance to input units.
    double dMin = -1e9;    ///< Lower clamp on d_hat (limits the observer's authority).
    double dMax = 1e9;     ///< Upper clamp on d_hat.
    double uMin = -1e9;    ///< Lower output saturation limit.
    double uMax = 1e9;     ///< Upper output saturation limit.
};

/**
 * @brief Disturbance-observer controller: inner feedback plus d_hat cancellation.
 */
class DisturbanceObserverController : public IController
{
public:
    /**
     * @brief Construct a DOB around an inner controller and a nominal SISO model.
     * @param inner   Feedback controller providing setpoint tracking.
     * @param nominal Discrete-time SISO nominal plant G_nom(z) (use c2d() first).
     * @param params  Q-filter and limit settings.
     * @param Ts      Sample time [s].
     * @throws std::invalid_argument If inner is null, Ts <= 0, omega_q <= 0,
     *         qOrder not in {1,2}, |gainDC| < 1e-12, the model is not SISO,
     *         or dMin >= dMax / uMin >= uMax.
     */
    DisturbanceObserverController(std::shared_ptr<IController> inner,
                                  const StateSpace &nominal,
                                  const DOBParams &params,
                                  double Ts);

    /**
     * @brief Provide the measured plant output for this step.
     * @param y Measured output y[k]; call before compute().
     */
    void setPlantOutput(double y) noexcept
    {
        y_meas_ = y;
        has_y_ = true;
    }

    // ---- IController -------------------------------------------------------

    /**
     * @brief Advance the inner loop and the observer one step.
     * @param error Tracking error e = r - y.
     * @return Saturated command u[k] with the estimated disturbance removed.
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }

    void reset() override;
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "DisturbanceObserverController"; }

    bool isHealthy() const override { return inner_->isHealthy(); }
    bool hasInternalAntiWindup() const override { return inner_->hasInternalAntiWindup(); }

    // ---- Diagnostics -------------------------------------------------------

    /** @brief Latest disturbance estimate d_hat[k], in **input** units, post-clamp. */
    double disturbanceEstimate() const noexcept { return d_hat_; }
    /** @brief Nominal-model output y_nom[k] from the last compute() call. */
    double nominalOutput() const noexcept { return y_nom_; }
    /** @brief Last applied command u[k-1]. */
    double lastOutput() const noexcept { return u_prev_; }
    /** @brief Mutable access to the inner controller. */
    const std::shared_ptr<IController> &innerController() const noexcept { return inner_; }

private:
    std::shared_ptr<IController> inner_;
    StateSpace nom_; ///< Nominal SISO model G_nom(z).
    DOBParams p_;
    double Ts_;

    double q_a_ = 0.0; ///< Q-filter pole exp(-omega_q.Ts).
    double q_b_ = 0.0; ///< Q-filter gain 1 - q_a_ (unity DC gain).
    double q1_ = 0.0;  ///< First Q-filter section state.
    double q2_ = 0.0;  ///< Second Q-filter section state (qOrder == 2 only).

    Eigen::VectorXd x_;  ///< Nominal-model state x_nom[k].
    Eigen::VectorXd xn_; ///< Pre-allocated work vector for the state update.
    Eigen::VectorXd yv_; ///< Pre-allocated work vector for C.x.

    double y_meas_ = 0.0; ///< Latest measurement from setPlantOutput().
    bool has_y_ = false;  ///< false until setPlantOutput() is called at least once.
    double y_nom_ = 0.0;  ///< Last nominal-model output.
    double d_hat_ = 0.0;  ///< Last clamped disturbance estimate [input units].
    double u_prev_ = 0.0; ///< Previous applied command u[k-1] (NaN-guard hold value).

    mutable Eigen::VectorXd notify_buf_{Eigen::VectorXd::Constant(1, 0.0)};
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(disturbance_observer)
