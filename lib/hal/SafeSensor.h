#pragma once
#include "ISensor.h"
#include <cmath>
#include <functional>

// SafeSensor - ISensor decorator that enforces the isValid() contract.
//
// Problem: ISensor::isValid() documents that "controllers should freeze when
// this returns false," but no controller calls isValid() - it only receives a
// double from the caller.  A sensor glitch (encoder loss, CAN timeout, ADC
// saturation) can silently feed NaN, 0.0, or a rail value into the estimator,
// corrupting Kalman covariance and MPC state estimates before any guard fires.
//
// Solution: wrap any ISensor in SafeSensor.  read() returns the last valid
// measurement whenever the inner sensor reports isValid() == false, instead of
// forwarding the bad sample.  isValid() on SafeSensor still reflects the inner
// state so the caller can react (log, alarm, switch to safe mode).
//
// Usage:
//   // Hardware sensor with fault detection:
//   ctrl::SafeSensor safe(my_encoder);
//
//   // In the control loop:
//   double y = safe.read();             // last-valid hold on fault; never NaN
//   if (!safe.isValid())
//       alarm.trigger(AlarmCode::SensorFault);
//   double u = pid.compute(ref - y);
//
// For simulation (SimSensor), isValid() is always true so SafeSensor is a
// transparent pass-through - no overhead.
//
// Staleness limit:
//   If max_stale_steps > 0, SafeSensor calls onMaxStale() (if set) after that
//   many consecutive invalid samples, signalling that the hold value is too
//   old to be trusted.  The default is unlimited (hold forever).
namespace ctrl {

class SafeSensor : public ISensor {
public:
    // inner:           the sensor being wrapped (must outlive this object)
    // max_stale_steps: how many consecutive invalid samples before onMaxStale() fires.
    //                  0 = unlimited (hold forever, never call onMaxStale).
    explicit SafeSensor(ISensor& inner, int max_stale_steps = 0)
        : inner_(&inner), last_valid_(0.0), stale_count_(0),
          max_stale_(max_stale_steps)
    {}

    // Returns the inner sensor's reading when isValid() == true,
    // or the last valid reading when isValid() == false (hold-last).
    // Never returns NaN or Inf even when the inner sensor does.
    double read() override
    {
        if (inner_->isValid()) {
            double v = inner_->read();
            // Accept the reading only if it is finite; reject NaN/Inf silently.
            if (std::isfinite(v)) {
                last_valid_ = v;
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

    // Reflects the inner sensor's validity (true = sensor is reporting fresh data).
    // Does NOT reflect whether last_valid_ is still trustworthy - check staleCount()
    // or listen for the onMaxStale callback for that.
    bool isValid() const override { return inner_->isValid(); }

    // Resets the hold value to 0 and the stale counter.
    void reset() override
    {
        inner_->reset();
        last_valid_  = 0.0;
        stale_count_ = 0;
    }

    // Number of consecutive samples since the last valid reading.
    int staleCount() const { return stale_count_; }

    // Last value successfully accepted from the inner sensor.
    double lastValidReading() const { return last_valid_; }

    // Register a callback invoked when stale_count_ reaches max_stale_ (if > 0).
    // Useful for triggering alarms or switching to a safe mode without polling.
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
