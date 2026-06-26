#pragma once
#include "IController.h"
#include <memory>
#include <cmath>
#include <stdexcept>

/**
 * @file EventTriggeredWrapper.h
 * @brief Aperiodic-sampling (event-triggered) decorator for any IController.
 *
 * Periodic sampling recomputes the inner controller every step regardless of whether the
 * signal has changed meaningfully. Event-triggered control instead holds the last output
 * (zero-order hold) until the incoming signal drifts more than a deadband threshold @c sigma
 * away from the value at the last triggered computation, then recomputes once. Useful when
 * communication bandwidth or computation must be budgeted (networked control, IoT sensors).
 *
 * **Trigger rule (convention-agnostic):** unlike comparing a tracking error against a
 * reference directly (which requires knowing whether @c signal is `r-y`, `y-r`, or raw `y`),
 * this wrapper only ever compares the incoming @c signal to the @c signal value recorded at
 * the last trigger - so it works unchanged regardless of the wrapped controller's
 * @ref ctrl::SignConvention.
 *
 * @see docs/superpowers/specs/2026-06-26-small-extensions-batch-design.md, item 5.
 */

namespace ctrl {

/**
 * @brief Deadband threshold for EventTriggeredWrapper.
 */
struct EventTriggeredParams
{
    double sigma = 0.1; ///< Deadband threshold on the incoming signal [engineering units].
};

/**
 * @brief IController decorator that recomputes the inner controller only when the incoming
 * signal has moved more than @c sigma since the last triggered computation.
 */
class EventTriggeredWrapper : public IController
{
public:
    /**
     * @brief Construct the wrapper around an existing controller.
     *
     * @param inner  Inner IController to gate with event-triggered sampling.
     * @param params Deadband threshold (default sigma = 0.1).
     * @throws std::invalid_argument If inner is null.
     */
    explicit EventTriggeredWrapper(std::shared_ptr<IController> inner,
                                    const EventTriggeredParams& params = {})
        : inner_(std::move(inner)), params_(params)
    {
        if (!inner_)
            throw std::invalid_argument(
                "EventTriggeredWrapper: inner controller must not be null.");
    }

    /**
     * @brief Compute the event-triggered output.
     *
     * The very first call always triggers (there is no prior reference to compare against).
     * After that, recomputes only if @p signal has moved more than @c sigma away from the
     * signal value recorded at the last trigger; otherwise returns the held output unchanged.
     *
     * @param signal Input to the inner controller (tracking error or plant output, depending
     *                on the wrapped controller's convention).
     * @return u[k] - either a freshly computed output, or the held value from the last trigger.
     */
    double compute(double signal) override
    {
        if (!std::isfinite(signal)) {
            notifyObserver(u_held_, signal);
            return u_held_;
        }

        const bool should_trigger =
            !triggered_once_ || std::abs(signal - signal_last_triggered_) > params_.sigma;

        if (should_trigger) {
            u_held_                = inner_->compute(signal);
            signal_last_triggered_ = signal;
            triggered_once_        = true;
            ++n_triggered_;
        } else {
            ++n_held_;
        }

        notifyObserver(u_held_, signal);
        return u_held_;
    }

    /**
     * @brief Reset the inner controller and clear held state.
     *
     * The next compute() call will trigger unconditionally, mirroring the first-ever call.
     */
    void reset() override
    {
        inner_->reset();
        u_held_                = 0.0;
        signal_last_triggered_ = 0.0;
        triggered_once_        = false;
        n_triggered_           = 0;
        n_held_                = 0;
        notifyObserverReset();
    }

    /** @brief Sample time inherited from the wrapped controller. */
    double sampleTime() const override { return inner_->sampleTime(); }

    /** @brief The output held since the last triggered computation. */
    double lastOutput() const { return u_held_; }

    /** @brief Number of compute() calls that triggered a real inner computation. */
    int triggerCount() const { return n_triggered_; }

    /** @brief Number of compute() calls that returned the held output unchanged. */
    int holdCount() const { return n_held_; }

    /** @brief Read-only access to the inner controller. */
    const IController& inner() const { return *inner_; }

private:
    std::shared_ptr<IController> inner_;
    EventTriggeredParams         params_;
    double u_held_                = 0.0;
    double signal_last_triggered_ = 0.0;
    bool   triggered_once_        = false;
    int    n_triggered_           = 0;
    int    n_held_                = 0;
};

} // namespace ctrl

#include "ControllerRegistry.h"
CTRL_REGISTER_FEATURE(event_triggered)
