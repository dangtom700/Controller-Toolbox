#pragma once
#include "IController.h"
#include "SmithPredictor.h"
#include <deque>
#include <memory>

/**
 * @file AdaptiveSmithPredictor.h
 * @brief Smith Predictor with online dead-time estimation via cross-correlation.
 *
 * Wraps a SmithPredictor and periodically re-estimates the plant dead time by
 * computing the cross-correlation between past control inputs and plant outputs:
 * @code
 *   R_uy(tau) = sum_{k=tau}^{N-1} u[k-tau] * y[k]
 *   estimated_d = argmax_{0 <= tau <= maxDelaySteps} R_uy(tau)
 * @endcode
 *
 * This exploits the fact that for a plant with delay d samples,
 * y[k] approx = G(z) * u[k-d], so R_uy(tau) is maximised at tau = d.
 *
 * **Usage:**
 * @code
 *   ctrl::AdaptiveSPParams asp;
 *   asp.maxDelaySteps = 20; asp.estimateInterval = 100;
 *   ctrl::AdaptiveSmithPredictor asp_ctrl(inner, plant_model, 5, Ts, asp);
 *
 *   // Each step: feed the raw plant output before compute()
 *   asp_ctrl.setPlantOutput(y_meas);
 *   double u = asp_ctrl.compute(r - y_meas);
 * @endcode
 *
 * **Requirements:**
 * - setPlantOutput() should be called each step with the actual plant output
 *   for accurate cross-correlation.  If omitted, y is approximated as -error
 *   (valid for zero-setpoint regulation problems).
 * - At least (maxDelaySteps + bufferLen/2) steps must elapse before the first
 *   delay estimate is produced.
 *
 * @see Smith, O.J.M. (1957). Closed control of loops with dead time.
 * @see Astrom & Wittenmark, "Computer Controlled Systems" Section 6.4.
 */

namespace ctrl
{

/** @brief Tuning parameters for AdaptiveSmithPredictor. */
struct AdaptiveSPParams
{
    int maxDelaySteps    = 20;  ///< Maximum dead-time steps to search [0..maxDelaySteps].
    int estimateInterval = 100; ///< Re-estimate delay every N control steps.
    int bufferLen        = 200; ///< Cross-correlation buffer length (samples).
};

/**
 * @brief Smith Predictor with online dead-time estimation.
 *
 * Composes a SmithPredictor that is rebuilt whenever the delay estimate changes
 * by one or more samples.  The inner controller state is transferred to the new
 * predictor to avoid abrupt transients on re-identification.
 */
class AdaptiveSmithPredictor : public IController
{
public:
    /**
     * @brief Construct with a given inner controller and initial delay estimate.
     *
     * @param inner            Stabilising inner controller (e.g., DiscretePID).
     * @param delayModel       Delay-free plant model P0(z).
     * @param initialDelaySteps Initial dead-time estimate in samples.
     * @param Ts               Sample time [s].
     * @param params           Adaptation parameters.
     */
    AdaptiveSmithPredictor(std::shared_ptr<IController>  inner,
                           const StateSpace              &delayModel,
                           int                            initialDelaySteps,
                           double                         Ts,
                           const AdaptiveSPParams        &params = {});

    // -------------------------------------------------------------------------
    // IController interface
    // -------------------------------------------------------------------------

    /**
     * @brief Compute control output from tracking error.
     *
     * Appends the latest (u, y) pair to the correlation buffers and triggers
     * delay re-estimation every estimateInterval steps.
     *
     * @param error Tracking error e = r - y.
     * @return Control output u.
     */
    double compute(double error) override;

    /** @brief Reset buffers, inner SP, and step counter. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Last control output. */
    double lastOutput() const { return last_output_; }

    // -------------------------------------------------------------------------
    // Adaptation interface
    // -------------------------------------------------------------------------

    /**
     * @brief Feed the raw plant output y for cross-correlation.
     *
     * Call this each step BEFORE compute() to populate the y buffer.
     * If not called, the approximation y approx = -error is used (good for r = 0).
     *
     * @param y Plant output measurement.
     */
    void setPlantOutput(double y)
    {
        last_y_ = y;
        has_y_  = true;
    }

    /** @brief Current delay estimate in samples. */
    int    estimatedDelaySteps()  const { return current_delay_; }

    /** @brief Current delay estimate in seconds. */
    double estimatedDelayTime()   const
    { return static_cast<double>(current_delay_) * Ts_; }

private:
    void estimateDelay();
    void rebuildSP();

    std::shared_ptr<IController>  inner_;
    StateSpace                    delay_model_;
    AdaptiveSPParams              params_;
    double                        Ts_;
    int                           current_delay_;

    std::shared_ptr<SmithPredictor> sp_;

    // Cross-correlation buffers (oldest first)
    std::deque<double> u_hist_;
    std::deque<double> y_hist_;

    double last_y_      = 0.0;
    bool   has_y_       = false;
    double last_u_      = 0.0;
    double last_output_ = 0.0;
    int    step_count_  = 0;
};

} // namespace ctrl
