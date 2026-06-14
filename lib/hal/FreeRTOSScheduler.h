#pragma once

/**
 * @file FreeRTOSScheduler.h
 * @brief Stub IScheduler implementation for FreeRTOS software timers.
 *
 * This file is a **porting stub**. It compiles and links only when the FreeRTOS
 * headers are available on the include path (detected via FREERTOS_VERSION or
 * the canonical FreeRTOS.h guard macro). On host builds (CI, unit tests) the
 * entire body is excluded so there is no link-time dependency on the RTOS.
 *
 * **Integration steps:**
 *   1. Add `lib/hal/` to your RTOS target's include directories.
 *   2. Ensure `FreeRTOS.h`, `timers.h`, and `task.h` are on the include path.
 *   3. Instantiate `ctrl::FreeRTOSScheduler` in place of `ctrl::SimScheduler`.
 *   4. Set period, callback, then call start(). The internal FreeRTOS software
 *      timer fires `cb_` from the timer-service task.
 *
 * **Hard-RT note:** FreeRTOS software timers execute from the timer-service
 * task at a fixed priority (configTIMER_TASK_PRIORITY). For deterministic
 * control loops, prefer `vTaskDelayUntil()` inside a dedicated high-priority
 * task and implement IScheduler::start() to create/resume that task instead.
 * This stub uses `xTimerCreate()` as a baseline; replace if needed.
 *
 * @see IScheduler.h, SimScheduler.h (host-simulation reference implementation)
 */

#if defined(FREERTOS_VERSION) || defined(INC_FREERTOS_H)
#include <atomic>

#include "IScheduler.h"
#include <FreeRTOS.h>
#include <timers.h>
#include <functional>
#include <stdexcept>

namespace ctrl {

/**
 * @brief IScheduler backed by a FreeRTOS software timer (xTimerCreate).
 *
 * The callback is captured in a std::function and dispatched from the
 * timer-service task via a static trampoline. Only one instance may be active
 * at a time per timer handle; use separate FreeRTOSScheduler objects for
 * independent control loops.
 */
class FreeRTOSScheduler : public IScheduler {
public:
    FreeRTOSScheduler() = default;

    ~FreeRTOSScheduler()
    {
        if (timer_handle_)
            xTimerDelete(timer_handle_, portMAX_DELAY);
    }

    void setPeriodNs(uint64_t period_ns) override
    {
        period_ns_   = period_ns;
        period_ticks_ = static_cast<TickType_t>(
            (period_ns / 1'000'000ULL) / portTICK_PERIOD_MS);
        if (period_ticks_ == 0)
            period_ticks_ = 1; // minimum one tick
    }

    void setCallback(std::function<void()> cb) override { cb_ = std::move(cb); }

    /**
     * @brief Create (or update) the FreeRTOS timer and start it.
     *
     * If the timer already exists it is deleted and re-created with the current
     * period so that setPeriodNs() changes take effect cleanly.
     */
    void start() override
    {
        if (!cb_)
            throw std::logic_error("FreeRTOSScheduler::start() called with no callback.");
        if (period_ticks_ == 0)
            throw std::logic_error("FreeRTOSScheduler::start() called before setPeriodNs().");

        if (timer_handle_)
        {
            xTimerStop(timer_handle_, portMAX_DELAY);
            xTimerDelete(timer_handle_, portMAX_DELAY);
            timer_handle_ = nullptr;
        }

        timer_handle_ = xTimerCreate(
            "ctrl_sched",
            period_ticks_,
            pdTRUE,           // auto-reload (periodic)
            this,             // pvTimerID = this pointer for trampoline
            &FreeRTOSScheduler::timerCallback);

        if (!timer_handle_)
            throw std::runtime_error("FreeRTOSScheduler: xTimerCreate failed.");

        running_  = true;
        ticks_    = 0;
        overruns_ = 0;
        xTimerStart(timer_handle_, portMAX_DELAY);
    }

    void stop() override
    {
        if (timer_handle_)
            xTimerStop(timer_handle_, portMAX_DELAY);
        running_ = false;
    }

    bool     isRunning()    const override { return running_; }
    uint64_t periodNs()     const override { return period_ns_; }
    uint64_t tickCount()    const override { return ticks_; }
    uint64_t overrunCount() const override { return overruns_; }

private:
    static void timerCallback(TimerHandle_t xTimer)
    {
        auto *self = static_cast<FreeRTOSScheduler *>(pvTimerGetTimerID(xTimer));
        if (self && self->cb_)
        {
            self->cb_();
            ++self->ticks_;
            // Overrun detection: FreeRTOS software timers have no built-in
            // wall-clock measurement. Use a platform ITimer if needed.
        }
    }

    std::function<void()> cb_;
    TimerHandle_t timer_handle_{nullptr};
    uint64_t period_ns_{0};
    TickType_t period_ticks_{0};
    bool     running_{false};
    std::atomic<uint64_t> ticks_{0};
    std::atomic<uint64_t> overruns_{0};
};

} // namespace ctrl

#endif // FREERTOS_VERSION || INC_FREERTOS_H
