#pragma once
#include "ISensor.h"
#include <cmath>
#include <functional>

/**
 * @file SafeSensor.h
 * @brief ISensor decorator that enforces the isValid() contract via hold-last semantics.
 *
 * **Problem:** `ISensor::isValid()` documents that controllers should freeze when it
 * returns `false`, but no controller calls `isValid()` - it only receives a `double`.
 * A sensor glitch (encoder loss, CAN timeout, ADC saturation) can silently feed NaN,
 * 0.0, or a rail value into the estimator, corrupting Kalman covariance and MPC state
 * estimates before any guard fires.
 *
 * **Solution:** wrap any `ISensor` in `SafeSensor`. `read()` returns the last valid
 * measurement whenever the inner sensor reports `isValid() == false`, instead of
 * forwarding the bad sample. `isValid()` on SafeSensor still reflects the inner state
 * so the caller can react (log, alarm, switch to safe mode).
 *
 * **Usage:**
 * @code
 *   ctrl::SafeSensor safe(my_encoder);
 *
 *   double y = safe.read();           // hold-last on fault; never NaN
 *   if (!safe.isValid())
 *       alarm.trigger(AlarmCode::SensorFault);
 *   double u = pid.compute(ref - y);
 * @endcode
 *
 * For simulation (SimSensor), `isValid()` is always `true` so SafeSensor is a
 * transparent pass-through with negligible overhead.
 *
 * **Staleness limit:** if `max_stale_steps > 0`, SafeSensor calls `onMaxStale()` (if set)
 * after that many consecutive invalid samples, signalling that the hold value is too old
 * to be trusted. The default is unlimited (hold forever).
 */

namespace ctrl {

/**
 * @brief ISensor decorator with hold-last fault semantics and optional staleness alarm.
 */
class SafeSensor : public ISensor {
public:
    /**
     * @brief Wrap @p inner with hold-last protection.
     * @param inner           The sensor to wrap (must outlive this object).
     * @param max_stale_steps How many consecutive invalid samples before `onMaxStale()` fires.
     *                        0 = unlimited (hold forever, never fire callback).
     */
    explicit SafeSensor(ISensor& inner, int max_stale_steps = 0)
        : inner_(&inner), last_valid_(0.0), stale_count_(0),
          max_stale_(max_stale_steps)
    {}

    /**
     * @brief Return the inner sensor's reading, or the last valid value when invalid.
     *
     * Also rejects non-finite inner values (NaN, +/-Inf) even when `isValid() == true`.
     * Increments the stale counter on any rejected sample.
     *
     * @return Most recent valid (finite) measurement. Never NaN or Inf.
     */
    double read() override
    {
        if (inner_->isValid()) {
            double v = inner_->read();
            if (std::isfinite(v)) {
                last_valid_  = v;
                stale_count_ = 0;
            } else {
                ++stale_count_;
                checkStaleLimit();
            }
        } else {
            ++stale_count_;
            checkStaleLimit();
        }
        return last_valid_;
    }

    /**
     * @brief Reflects the inner sensor's validity.
     *
     * `true` = the inner sensor reports fresh data. Does NOT indicate whether
     * `last_valid_` is still trustworthy - check `staleCount()` or the
     * `onMaxStale` callback for that.
     *
     * @return `inner_->isValid()`.
     */
    bool isValid() const override { return inner_->isValid(); }

    /**
     * @brief Reset the hold value to 0, the stale counter, and the inner sensor.
     */
    void reset() override
    {
        inner_->reset();
        last_valid_  = 0.0;
        stale_count_ = 0;
    }

    /**
     * @brief Number of consecutive samples since the last valid reading.
     * @return Stale-sample count.
     */
    int staleCount() const { return stale_count_; }

    /**
     * @brief Last measurement successfully accepted from the inner sensor (finite, valid).
     * @return Last valid reading.
     */
    double lastValidReading() const { return last_valid_; }

    /**
     * @brief Register a callback invoked when stale_count reaches max_stale_steps.
     *
     * Useful for triggering alarms or switching to a safe mode without polling.
     * The callback receives the current stale count.
     *
     * @param cb Callback `void(int stale_count)`.
     */
    void setOnMaxStale(std::function<void(int stale_count)> cb)
    {
        on_max_stale_ = std::move(cb);
    }

private:
    void checkStaleLimit()
    {
        if (max_stale_ > 0 && stale_count_ == max_stale_ && on_max_stale_)
            on_max_stale_(stale_count_);
    }

    ISensor*    inner_;
    double      last_valid_;
    int         stale_count_;
    int         max_stale_;
    std::function<void(int)> on_max_stale_;
};

} // namespace ctrl
