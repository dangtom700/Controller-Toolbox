#pragma once
#include <cstdint>

/**
 * @file ITimer.h
 * @brief Abstract interface for a monotonic high-resolution clock.
 *
 * Provides nanosecond-resolution timestamps for deadline checking and
 * execution-time measurement inside the control loop.
 *
 * Platform-specific implementations wrap:
 * - Linux RT: `clock_gettime(CLOCK_MONOTONIC)`.
 * - FreeRTOS: `xTaskGetTickCount() * portTICK_PERIOD_MS * 1e6`.
 * - Zephyr: `k_cycle_get_32() / sys_clock_hw_cycles_per_sec()`.
 * - Windows (test): `QueryPerformanceCounter()`.
 *
 * **Overrun detection pattern:**
 * @code
 *   void control_loop_tick(ITimer& timer, IController& ctrl, uint64_t Ts_ns) {
 *       uint64_t t0 = timer.nowNs();
 *       double u = ctrl.compute(error);
 *       actuator.write(u);
 *       uint64_t elapsed = timer.elapsedNs(t0);
 *       if (elapsed > Ts_ns)
 *           on_overrun(elapsed, Ts_ns);  // log / alarm / fallback
 *   }
 * @endcode
 *
 * @see POSIX.1-2017 clock_gettime(); FreeRTOS xTaskGetTickCount() API.
 */

namespace ctrl {

/**
 * @brief Abstract monotonic clock for deadline and execution-time measurement.
 */
class ITimer {
public:
    virtual ~ITimer() = default;

    /**
     * @brief Return the current monotonic time in nanoseconds.
     *
     * Must be callable from an ISR / RT thread without blocking.
     *
     * @return Monotonic timestamp [ns]. Wraps at UINT64_MAX (~585 years).
     */
    virtual uint64_t nowNs() const = 0;

    /**
     * @brief Return nanoseconds elapsed since a previously recorded timestamp.
     *
     * Handles 64-bit wrap-around correctly.
     *
     * @param t0 Timestamp from a prior nowNs() call.
     * @return Elapsed time [ns] = nowNs() - t0.
     */
    virtual uint64_t elapsedNs(uint64_t t0) const { return nowNs() - t0; }
};

} // namespace ctrl
