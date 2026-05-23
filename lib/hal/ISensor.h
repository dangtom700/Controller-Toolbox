#pragma once

// ISensor - abstract interface for any measurement source.
//
// Implementations may wrap a hardware ADC, encoder, CAN bus signal, or a
// simulation model (SimSensor).  Controllers depend only on ISensor, so the
// same control code runs against hardware or a mathematical plant without
// modification.
//
// Contract:
//   read()    - returns the most recent measurement as a double.
//               Must not block longer than one sample period.
//               Returns the last valid value (never NaN) when data is stale
//               - check isValid() to detect that condition.
//   isValid() - true while the sensor data is fresh and within hardware limits.
//               Controllers should freeze or fault when this returns false.
//   reset()   - restores the sensor to its initial state (e.g. clears filters).
namespace ctrl {

class ISensor {
public:
    virtual ~ISensor() = default;

    virtual double read()           = 0;
    virtual bool   isValid() const  { return true; }
    virtual void   reset()          {}
};

} // namespace ctrl
