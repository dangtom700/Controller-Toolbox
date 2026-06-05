#pragma once
#include "IController.h"
#include <memory>
#include <cmath>
#include <stdexcept>

/**
 * @file ComputationalDelayWrapper.h
 * @brief One-sample computational delay wrapper for any IController.
 *
 * Real digital control loops have at least one sample of computational delay:
 * the control law computed at step k cannot reach the actuator until step k+1
 * because ADC sampling, computation, and DAC output take time.  This wrapper
 * models that delay explicitly so that simulations and frequency-domain
 * robustness analyses reflect the actual deployed loop behaviour.
 *
 * **Timing model:**
 * @code
 *   u_inner[k] = inner->compute(signal[k])   // computed this step
 *   u_out[k]   = u_inner[k-1]                // applied to the plant (one step late)
 * @endcode
 *
 * @note A one-sample delay shifts the discrete-time Nyquist frequency phase margin
 *       by -pi (180 deg), which can destabilise loops tuned without it.  Use this
 *       wrapper to expose that margin to the controller designer, not to fix it.
 *
 * @see CONTRIBUTING.md#numerical-safety-rules (Rule 8, G3)
 * @see Astrom & Wittenmark, "Computer-Controlled Systems" Section 3.3.
 */

namespace ctrl {

/**
 * @brief IController decorator that introduces exactly one sample of computational delay.
 *
 * The wrapped controller computes its output every step; the output is held for
 * one step before being returned to the caller.  The initial delayed output is
 * @p initial_output (default 0).
 */
class ComputationalDelayWrapper : public IController
{
public:
    /**
     * @brief Construct the wrapper around an existing controller.
     *
     * @param inner          Inner IController whose output will be delayed by one sample.
     * @param initial_output Value returned on the very first compute() call before the
     *                       inner controller has produced any output (default 0.0).
     * @throws std::invalid_argument If inner is null.
     */
    explicit ComputationalDelayWrapper(std::shared_ptr<IController> inner,
                                       double initial_output = 0.0)
        : inner_(std::move(inner)), u_delayed_(initial_output)
    {
        if (!inner_)
            throw std::invalid_argument(
                "ComputationalDelayWrapper: inner controller must not be null.");
    }

    /**
     * @brief Compute the delayed output.
     *
     * Returns the output computed in the *previous* step; schedules the inner
     * controller's computation for delivery next step.
     *
     * @param signal  Input to the inner controller (tracking error or plant output,
     *                depending on the wrapped controller's convention).
     * @return u[k-1] - the inner controller's output from the previous step.
     * @see CONTRIBUTING.md#sign-conventions
     */
    double compute(double signal) override
    {
        if (!std::isfinite(signal)) {
            notifyObserver(u_delayed_, signal);
            return u_delayed_;
        }
        const double u_now = inner_->compute(signal); // stored for next step
        const double u_out = u_delayed_;              // output this step's delayed value
        u_delayed_ = u_now;
        notifyObserver(u_out, signal);
        return u_out;
    }

    /**
     * @brief Reset the inner controller and clear the delayed output to 0.
     */
    void reset() override
    {
        inner_->reset();
        u_delayed_ = 0.0;
        notifyObserverReset();
    }

    /** @brief Sample time inherited from the wrapped controller. */
    double sampleTime() const override { return inner_->sampleTime(); }

    /** @brief The output that will be delivered on the next compute() call. */
    double lastOutput() const { return u_delayed_; }

    /** @brief Read-only access to the inner controller. */
    const IController& inner() const { return *inner_; }

private:
    std::shared_ptr<IController> inner_;
    double                       u_delayed_; ///< Output buffered from the previous step.
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(computational_delay)
