#pragma once

/**
 * @file ISensor.h
 * @brief Abstract interface for any measurement source.
 *
 * Implementations may wrap a hardware ADC, encoder, CAN-bus signal, or a simulation
 * model (SimSensor). Controllers depend only on ISensor, so the same control code runs
 * against hardware or a mathematical plant without modification.
 *
 * **Contract:**
 * - `read()` — returns the most recent measurement as a `double`.
 *   Must not block longer than one sample period.
 *   Returns the last valid value (never NaN) when data is stale;
 *   check `isValid()` to detect that condition.
 * - `isValid()` — `true` while sensor data is fresh and within hardware limits.
 *   Controllers should freeze or fault when this returns `false`.
 * - `reset()` — restores the sensor to its initial state (e.g., clears filters).
 */

namespace ctrl {

/**
 * @brief Abstract base class for all sensor adapters in the HAL.
 */
class ISensor {
public:
    virtual ~ISensor() = default;

    /**
     * @brief Read the most recent measurement.
     * @return Current sensor value. Must not be NaN.
     */
    virtual double read()          = 0;

    /**
     * @brief Check whether the sensor data is currently fresh and valid.
     * @return `true` if the reading from `read()` is trustworthy.
     */
    virtual bool   isValid() const { return true; }

    /**
     * @brief Reset the sensor to its initial state (clear filters, counters).
     */
    virtual void   reset()         {}
};

} // namespace ctrl
