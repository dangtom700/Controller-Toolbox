#pragma once
#include <cstdint>
#include <functional>

/**
 * @file IScheduler.h
 * @brief Abstract interface for periodic task management.
 *
 * Registers a fixed-rate callback (the control-loop tick) with the platform
 * scheduler and provides a standard way to start, stop, and query execution
 * context. Platform-specific implementations wrap:
 * - Linux RT: `timer_create(CLOCK_MONOTONIC)` + `SIGEV_THREAD_ID` + `SCHED_FIFO`.
 * - FreeRTOS: `xTimerCreate()` / `vTaskDelayUntil()`.
 * - Zephyr: `k_timer_init()` / `k_timer_start()`.
 *
 * The callback signature is `void()`. State required by the control loop
 * should be captured in a lambda or stored externally.
 *
 * **Usage — wiring a PID loop on a 1 ms tick:**
 * @code
 *   auto sched = platform::makeScheduler();   // platform-specific factory
 *   sched->setPeriodNs(1'000'000);            // 1 ms
 *   sched->setCallback([&]() {
 *       double y = sensor.read();
 *       double u = pid.compute(ref - y);
 *       actuator.write(u);
 *   });
 *   sched->start();
 *   // ... main thread does non-RT work or sleeps
 *   sched->stop();
 * @endcode
 *
 * **Thread safety:** `start()`, `stop()`, and `setCallback()` must be called
 * from the same non-RT setup thread before the loop begins. The callback runs
 * from the RT context; do not call `start()`/`stop()` from inside the callback.
 *
 * @see POSIX.1-2017 timer_create(); FreeRTOS Software Timer API.
 */

namespace ctrl {

/**
 * @brief Abstract periodic task scheduler for the control loop.
 */
class IScheduler {
public:
    virtual ~IScheduler() = default;

    /**
     * @brief Set the tick period. Must be called before start().
     * @param period_ns Tick period in nanoseconds.
     */
    virtual void setPeriodNs(uint64_t period_ns) = 0;

    /**
     * @brief Register the periodic callback. Must be called before start().
     *
     * Replaces any previously registered callback. The callback is invoked once
     * per period from the RT context; it must not block, allocate memory, or
     * call non-RT-safe APIs.
     *
     * @param cb Callback with signature `void()`.
     */
    virtual void setCallback(std::function<void()> cb) = 0;

    /**
     * @brief Start the periodic timer.
     *
     * After this call, the callback fires every `periodNs()` nanoseconds.
     * Calling `start()` when already running is a no-op.
     */
    virtual void start() = 0;

    /**
     * @brief Stop the periodic timer.
     *
     * Blocks until any in-progress callback invocation completes before returning.
     */
    virtual void stop() = 0;

    /**
     * @brief True while the scheduler is running (between start() and stop()).
     * @return Running state.
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief Period in nanoseconds as configured (0 if not set).
     * @return Configured period [ns].
     */
    virtual uint64_t periodNs() const = 0;

    /**
     * @brief Number of timer ticks fired since the last start() call.
     *
     * Useful for drift measurement and missed-tick detection.
     *
     * @return Tick count since last start().
     */
    virtual uint64_t tickCount() const = 0;

    /**
     * @brief Number of ticks where the callback execution time exceeded periodNs().
     *
     * Reset to zero on each start() call.
     *
     * @return Overrun count since last start().
     */
    virtual uint64_t overrunCount() const = 0;
};

} // namespace ctrl
