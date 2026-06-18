#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <memory>
#include <optional>
#include <vector>

/**
 * @file SmithPredictor.h
 * @brief Smith Predictor - compensates for pure dead-time in discrete plants.
 *
 * Replaces the dead-time delay in the feedback path with a prediction from an internal model,
 * so the inner controller C(z) sees a delay-free loop.
 *
 * **Modified error delivered to the inner controller:**
 * @code
 *   e_sp[k] = (r[k] - y[k]) - (yhat_model[k] - yhat_model[k-d])
 *            = error - (current model output - d-step-delayed model output)
 * @endcode
 * Subtracting the model correction removes the delay: the inner controller C(z)
 * sees the delay-free error r[k] - yhat_model[k] as if no dead time existed.
 * Ref: Astrom & Wittenmark, CCS eq. (6.36); Smith, Chem. Eng. Prog. 53 (1957).
 *
 * **Requirements:** the @p delayModel must represent the delay-**free** plant P0(z).
 *
 * **Fractional dead-time support:**
 * If the true dead time theta is not an integer multiple of Ts, pass a first-order Pade filter
 * (from FunctionApproximator::padeDelayFilter()) for the sub-sample remainder. The filter
 * H_frac is connected in series with the delay model so the effective model is
 * P0_eff(z) = H_frac(z).P0(z), matching e^{-theta_frac.s} to first order without rounding.
 *
 * @see Smith, "Closed control of loops with dead time", Chem. Eng. Prog. 53 (1957).
 * @see Astrom & Wittenmark, "Computer Controlled Systems" Section 6.4.
 */

namespace ctrl
{

/**
 * @brief Smith Predictor wrapper for any IController inner loop.
 *
 * Three overloaded constructors support integer-only dead time, explicit fractional Pade
 * filters, and an automatic whole-dead-time constructor that splits theta = floor(theta/Ts).Ts + theta_frac.
 */
class SmithPredictor : public IController
{
public:
    /**
     * @brief Construct for an integer dead-time delay only.
     * @param inner      Base stabilising controller (e.g., DiscretePID).
     * @param delayModel State-space model of the plant **without** the dead-time delay.
     * @param delaySteps Integer dead-time length in samples d.
     */
    SmithPredictor(std::shared_ptr<IController> inner,
                   const StateSpace &delayModel,
                   int delaySteps);

    /**
     * @brief Construct with explicit fractional Pade filter for sub-sample dead time.
     * @param inner           Base stabilising controller.
     * @param delayModel      Delay-free plant model P0.
     * @param delaySteps      Integer part of dead time in samples d.
     * @param fracDelayFilter First-order StateSpace from padeDelayFilter(theta_frac, Ts).
     */
    SmithPredictor(std::shared_ptr<IController> inner,
                   const StateSpace &delayModel,
                   int delaySteps,
                   const StateSpace &fracDelayFilter);

    /**
     * @brief Convenient whole-dead-time constructor - handles fractional remainder automatically.
     *
     * Splits theta into floor(theta/Ts) integer steps and builds a first-order Pade filter for
     * the sub-sample remainder.
     *
     * @param inner      Base stabilising controller.
     * @param delayModel Delay-free plant model.
     * @param theta      Total dead time theta [s].
     * @param Ts         Sample time [s] (must match delayModel.Ts).
     */
    SmithPredictor(std::shared_ptr<IController> inner,
                   const StateSpace &delayModel,
                   double theta,
                   double Ts);

    /**
     * @brief Compute u[k] from closed-loop error e[k] = r[k] - y[k].
     * @param error Current tracking error.
     * @return Control output u[k] from the inner controller operating on the Smith-corrected error.
     */
    double compute(double error) override;

    /** @brief Reset model state, output buffer, and inner controller. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Access the wrapped inner controller for runtime tuning.
     * @return Mutable reference to the inner controller.
     */
    IController &innerController() { return *inner_; }

    /**
     * @brief Replace the delay-free plant model and dead-time length at runtime.
     *
     * Resets model state and output buffer (equivalent to reset()).
     * @param delayModel Updated delay-free model.
     * @param delaySteps New integer dead-time length in samples.
     */
    void setModel(const StateSpace &delayModel, int delaySteps);

    /**
     * @brief Replace the plant model with a fractional delay filter at runtime.
     * @param delayModel      Updated delay-free model.
     * @param delaySteps      New integer dead-time length in samples.
     * @param fracDelayFilter Updated first-order Pade filter for the sub-sample remainder.
     */
    void setModel(const StateSpace &delayModel, int delaySteps,
                  const StateSpace &fracDelayFilter);

private:
    void initBuffers();

    std::shared_ptr<IController> inner_;
    StateSpace                   model_;       ///< Delay-free plant model P0.
    int                          d_;           ///< Integer delay steps.
    double                       Ts_;
    bool                         has_frac_;    ///< @c true when a Pade filter is active.
    StateSpace                   frac_filter_; ///< H_frac state-space (1 state).
    Eigen::VectorXd              x_frac_;      ///< State of the fractional Pade filter.
    Eigen::VectorXd              x_model_;     ///< Internal model state x^.
    Eigen::VectorXd              u_prev_;      ///< u[k-1] for D.u feedthrough.
    std::vector<double>          y_buf_;       ///< Circular buffer for yhat delay (length d).
    int                          buf_head_;    ///< Write pointer into y_buf_.
};

} // namespace ctrl
