#pragma once

/**
 * @file ZephyrScheduler.h
 * @brief Stub IScheduler implementation for Zephyr RTOS kernel timers.
 *
 * This file is a **porting stub**. It compiles and links only when the Zephyr
 * kernel headers are available (detected via CONFIG_ZEPHYR). On host builds
 * (CI, unit tests) the entire body is excluded so there is no link-time
 * dependency on the RTOS.
 *
 * **Integration steps:**
 *   1. Add `lib/hal/` to your Zephyr module's include directories.
 *   2. Ensure `<zephyr/kernel.h>` is on the include path (Zephyr build system
 *      provides this automatically for in-tree modules).
 *   3. Instantiate `ctrl::ZephyrScheduler` in place of `ctrl::SimScheduler`.
 *   4. Set period, callback, then call start(). The k_timer fires `cb_` from
 *      the timer ISR context (expiry function); see note below on context safety.
 *
 * **Zephyr context note:** `k_timer` expiry functions run from the system
 * timer ISR or a dedicated timer thread depending on CONFIG_SYSTEM_WORKQUEUE_*.
 * To run the control callback from a thread context (safer for floating-point
 * and controller state), submit a k_work item from the expiry function instead
 * of calling `cb_()` directly. This stub calls `cb_()` inline; adapt as needed
 * for your CONFIG_FPU and IRQ-safety requirements.
 *
 * @see IScheduler.h, SimScheduler.h (host-simulation reference implementation)
 */

#if defined(CONFIG_ZEPHYR)

#include "IScheduler.h"
#include <zephyr/kernel.h>
#include <functional>
#include <stdexcept>

namespace ctrl {

/**
 * @brief IScheduler backed by a Zephyr k_timer.
 *
 * The control callback is captured in a std::function and dispatched from the
 * k_timer expiry function. See the file-level note regarding ISR vs. thread
 * context before use in safety-critical applications.
 */
class ZephyrScheduler : public IScheduler {
public:
    ZephyrScheduler() { k_timer_init(&timer_, &ZephyrScheduler::expiryFn, nullptr); }

    ~ZephyrScheduler() { k_timer_stop(&timer_); }

    void setPeriodNs(uint64_t period_ns) override { period_ns_ = period_ns; }

    void setCallback(std::function<void()> cb) override { cb_ = std::move(cb); }

    /**
     * @brief Start the k_timer with the configured period.
     *
     * Converts the period from nanoseconds to Zephyr k_timeout_t (millisecond
     * resolution on most configurations; use K_NSEC if CONFIG_TIMER_HAS_64BIT
     * is enabled on your platform).
     */
    void start() override
    {
        if (!cb_)
            throw std::logic_error("ZephyrScheduler::start() called with no callback.");
        if (period_ns_ < 1'000'000ULL)
            throw std::logic_error("ZephyrScheduler::start(): period_ns must be >= 1 ms (1000000 ns).");

        // Store this pointer in timer user-data for the trampoline.
        k_timer_user_data_set(&timer_, this);

        running_  = true;
        ticks_    = 0;
        overruns_ = 0;

        // K_MSEC resolution; use K_NSEC on platforms that support it.
        const k_timeout_t period = K_MSEC(static_cast<int64_t>(period_ns_ / 1'000'000ULL));
        k_timer_start(&timer_, period, period);
    }

    void stop() override
    {
        k_timer_stop(&timer_);
        running_ = false;
    }

    bool     isRunning()    const override { return running_; }
    uint64_t periodNs()     const override { return period_ns_; }
    uint64_t tickCount()    const override { return ticks_; }
    uint64_t overrunCount() const override { return overruns_; }

private:
    static void expiryFn(struct k_timer *timer)
    {
        auto *self = static_cast<ZephyrScheduler *>(k_timer_user_data_get(timer));
        if (self && self->cb_)
        {
            self->cb_();
            ++self->ticks_;
        }
    }

    struct k_timer timer_;
    std::function<void()> cb_;
    uint64_t period_ns_{0};
    bool     running_{false};
    uint64_t ticks_{0};
    uint64_t overruns_{0};
};

} // namespace ctrl

#endif // CONFIG_ZEPHYR
