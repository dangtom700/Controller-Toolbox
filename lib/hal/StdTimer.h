#pragma once
#include "ITimer.h"
#include <chrono>

// StdTimer - ITimer implementation using std::chrono::steady_clock.
//
// Suitable for simulation, unit tests, and non-RT Linux/Windows targets.
// NOT suitable for hard-RT use: steady_clock has no jitter guarantee.
// On Linux RT replace with a clock_gettime(CLOCK_MONOTONIC) implementation;
// on FreeRTOS replace with xTaskGetTickCount().
//
// Example:
//   ctrl::StdTimer timer;
//   uint64_t t0 = timer.nowNs();
//   ctrl.compute(error);
//   uint64_t dt = timer.elapsedNs(t0);
//   if (dt > period_ns) std::cerr << "overrun: " << dt << " ns\n";
namespace ctrl {

class StdTimer : public ITimer {
public:
    uint64_t nowNs() const override
    {
        using namespace std::chrono;
        return static_cast<uint64_t>(
            duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
    }
};

} // namespace ctrl
