#pragma once
#include "IScheduler.h"
#include "ITimer.h"
#include <functional>
#include <stdexcept>

/**
 * @file SimScheduler.h
 * @brief Synchronous simulation scheduler - concrete IScheduler for testing.
 *
 * Fires the registered callback deterministically via tick() or run(), without
 * any real-time thread or OS timer. This makes it trivial to drive a
 * control-loop simulation in unit tests and examples without platform
 * dependencies.
 *
 * **Usage - closed-loop simulation:**
 * @code
 *   ctrl::SimPlant    plant(model);
 *   ctrl::SimSensor   sensor(plant);
 *   ctrl::SimActuator actuator(plant, -10.0, 10.0);
 *   ctrl::DiscretePID pid(pp, Ts);
 *   ctrl::StdTimer    timer;
 *   ctrl::SimScheduler sched;
 *
 *   const double ref = 1.0;
 *   sched.setPeriodNs(static_cast<uint64_t>(Ts * 1e9));
 *   sched.setCallback([&]() {
 *       double y = sensor.read();
 *       double u = pid.compute(ref - y);
 *       actuator.write(u);
 *   });
 *   sched.start();
 *   sched.run(timer, 500);   // fire 500 ticks synchronously
 *   sched.stop();
 * @endcode
 *
 * **Thread safety:** Not thread-safe. All calls must come from the same thread.
 *
 * **Overrun detection:** tick(timer) measures real wall-clock time of the callback
 * and increments overrunCount() if it exceeds periodNs(). This is meaningful only
 * when the callback is computationally expensive relative to the tick period.
 *
 * @note For hard-RT targets, replace with a platform-specific IScheduler
 * (FreeRTOS xTimerCreate / Linux timer_create / Zephyr k_timer_init).
 *
 * @see IScheduler.h, ITimer.h, StdTimer.h
 */

namespace ctrl {

/**
 * @brief Synchronous test scheduler - fires callback immediately on tick().
 */
class SimScheduler : public IScheduler {
public:
    SimScheduler() = default;

    /**
     * @brief Set the nominal period. Used only for overrun accounting in tick().
     * @param period_ns Nominal tick period [ns].
     */
    void setPeriodNs(uint64_t period_ns) override { period_ns_ = period_ns; }

    /**
     * @brief Register the periodic callback.
     * @param cb Callable with signature @c void().
     */
    void setCallback(std::function<void()> cb) override { cb_ = std::move(cb); }

    /**
     * @brief Mark the scheduler as started. Resets tick and overrun counters.
     *
     * Does not spawn any thread or timer.
     */
    void start() override
    {
        running_   = true;
        ticks_     = 0;
        overruns_  = 0;
    }

    /**
     * @brief Mark the scheduler as stopped.
     */
    void stop() override { running_ = false; }

    /** @return @c true between start() and stop(). */
    bool isRunning() const override { return running_; }

    /** @return Configured period [ns], or 0 if not set. */
    uint64_t periodNs() const override { return period_ns_; }

    /** @return Total ticks fired since the last start() call. */
    uint64_t tickCount() const override { return ticks_; }

    /** @return Number of ticks where callback wall-time exceeded periodNs(). */
    uint64_t overrunCount() const override { return overruns_; }

    // -------------------------------------------------------------------------
    // SimScheduler-specific API (not in IScheduler)
    // -------------------------------------------------------------------------

    /**
     * @brief Fire one tick synchronously.
     *
     * Invokes the callback immediately, measures elapsed real wall-clock time,
     * and increments overrunCount() if elapsed > periodNs().
     *
     * @param timer ITimer used for overrun measurement (e.g., StdTimer).
     * @throws std::logic_error If the scheduler is not running or no callback is set.
     */
    void tick(ITimer &timer)
    {
        if (!running_)
            throw std::logic_error("SimScheduler::tick() called while not running.");
        if (!cb_)
            throw std::logic_error("SimScheduler::tick() called with no callback set.");

        const uint64_t t0 = timer.nowNs();
        cb_();
        const uint64_t elapsed = timer.elapsedNs(t0);
        if (period_ns_ > 0 && elapsed > period_ns_)
            ++overruns_;
        ++ticks_;
    }

    /**
     * @brief Fire the callback @p n_ticks times synchronously.
     *
     * Equivalent to calling tick(timer) in a loop. Stops immediately if the
     * scheduler is stopped from inside the callback.
     *
     * @param timer    ITimer for overrun measurement.
     * @param n_ticks  Number of ticks to fire.
     */
    void run(ITimer &timer, int n_ticks)
    {
        for (int i = 0; i < n_ticks && running_; ++i)
            tick(timer);
    }

    /**
     * @brief Fire the callback @p n_ticks times without overrun measurement.
     *
     * Lighter-weight path when timing accuracy is not needed.
     *
     * @param n_ticks Number of ticks to fire.
     * @throws std::logic_error If not running or no callback is set.
     */
    void run(int n_ticks)
    {
        if (!running_)
            throw std::logic_error("SimScheduler::run() called while not running.");
        if (!cb_)
            throw std::logic_error("SimScheduler::run() called with no callback set.");
        for (int i = 0; i < n_ticks && running_; ++i) {
            cb_();
            ++ticks_;
        }
    }

private:
    std::function<void()> cb_;
    uint64_t period_ns_{1'000'000}; // default: 1 ms
    bool     running_{false};
    uint64_t ticks_{0};
    uint64_t overruns_{0};
};

} // namespace ctrl
