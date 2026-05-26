#pragma once
#include <cstdint>
#include <functional>

// IScheduler - abstract interface for periodic task management.
//
// Registers a fixed-rate callback (the control loop tick) with the platform
// scheduler and provides a standard way to start, stop, and query the
// current execution context.  Platform-specific implementations wrap:
//   - Linux RT:   timer_create(CLOCK_MONOTONIC) + SIGEV_THREAD_ID + SCHED_FIFO
//   - FreeRTOS:   xTimerCreate() / vTaskDelayUntil()
//   - Zephyr:     k_timer_init() / k_timer_start()
//
// The callback signature is void(void) - no arguments.  Any state needed
// by the control loop should be captured in a lambda or stored externally.
//
// Example - wiring a PID loop on a 1 ms tick:
//
//   auto sched = platform::makeScheduler();   // platform-specific factory
//   sched->setPeriodNs(1'000'000);            // 1 ms
//   sched->setCallback([&]() {
//       double y = sensor.read();
//       double u = pid.compute(ref - y);
//       actuator.write(u);
//   });
//   sched->start();
//   // ... main thread does non-RT work or sleeps
//   sched->stop();
//
// Overrun detection:
//   Pair with ITimer to measure per-tick execution time and call onOverrun()
//   if it exceeds the period.  See ITimer.h for the pattern.
//
// Thread safety: start()/stop()/setCallback() must be called from the same
// non-RT setup thread before the loop begins.  The callback runs from the RT
// context; do not call start()/stop() from inside the callback.
//
// Ref: POSIX.1-2017 timer_create(); FreeRTOS Software Timer API.
namespace ctrl {

class IScheduler {
public:
    virtual ~IScheduler() = default;

    // Set the tick period in nanoseconds.  Must be called before start().
    virtual void setPeriodNs(uint64_t period_ns) = 0;

    // Register the periodic callback.  Replaces any previously registered callback.
    // Must be called before start().  The callback is invoked once per period from
    // the RT context; it must not block, allocate memory, or call non-RT-safe APIs.
    virtual void setCallback(std::function<void()> cb) = 0;

    // Start the periodic timer.  After this call, the callback fires every period_ns.
    // Calling start() when already running is a no-op.
    virtual void start() = 0;

    // Stop the periodic timer.  Blocks until any in-progress callback invocation
    // completes before returning.
    virtual void stop() = 0;

    // True while the scheduler is running (between start() and stop()).
    virtual bool isRunning() const = 0;

    // Period in nanoseconds as configured (0 if not set).
    virtual uint64_t periodNs() const = 0;

    // Number of timer ticks fired since the last start() call.
    // Useful for drift measurement and missed-tick detection.
    virtual uint64_t tickCount() const = 0;

    // Number of ticks where the callback execution time exceeded period_ns.
    // Reset to zero on each start() call.
    virtual uint64_t overrunCount() const = 0;
};

} // namespace ctrl
