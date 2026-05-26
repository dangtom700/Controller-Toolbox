#pragma once
#include <cstdint>

// ITimer - abstract interface for a monotonic high-resolution clock.
//
// Provides nanosecond-resolution timestamps for deadline checking and
// execution-time measurement inside the control loop.  Platform-specific
// implementations wrap:
//   - Linux RT:       clock_gettime(CLOCK_MONOTONIC)
//   - FreeRTOS:       xTaskGetTickCount() * portTICK_PERIOD_MS * 1e6
//   - Zephyr:         k_cycle_get_32() / sys_clock_hw_cycles_per_sec()
//   - Windows (test): QueryPerformanceCounter()
//
// Usage pattern (overrun detection):
//
//   void control_loop_tick(ITimer& timer, IController& ctrl, double Ts_ns) {
//       uint64_t t0 = timer.nowNs();
//
//       double u = ctrl.compute(error);
//       actuator.write(u);
//
//       uint64_t elapsed = timer.nowNs() - t0;
//       if (elapsed > static_cast<uint64_t>(Ts_ns))
//           on_overrun(elapsed, Ts_ns);  // log / alarm / fallback
//   }
//
// Ref: POSIX.1-2017 clock_gettime(); FreeRTOS xTaskGetTickCount() API.
namespace ctrl {

class ITimer {
public:
    virtual ~ITimer() = default;

    // Returns the current monotonic time in nanoseconds.
    // Must be callable from an ISR / RT thread without blocking.
    virtual uint64_t nowNs() const = 0;

    // Elapsed nanoseconds since a previously recorded timestamp t0.
    // Handles 64-bit wrap-around correctly.
    virtual uint64_t elapsedNs(uint64_t t0) const { return nowNs() - t0; }
};

} // namespace ctrl
