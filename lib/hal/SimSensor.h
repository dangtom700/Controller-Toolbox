#pragma once
#include "ISensor.h"
#include "SimPlant.h"

/**
 * @file SimSensor.h
 * @brief ISensor adapter backed by a SimPlant.
 *
 * `read()` returns the plant's cached output from the last `step()` call.
 * `isValid()` is always `true` for a simulation (no hardware faults to model).
 *
 * **Usage:**
 * @code
 *   ctrl::SimPlant    plant(sys);
 *   ctrl::SimSensor   sensor(plant);
 *   ctrl::SimActuator actuator(plant);
 *
 *   double y = sensor.read();        // y[k] = plant output before this step
 *   double u = pid.compute(r - y);
 *   actuator.write(u);               // advances plant -> x[k+1]
 * @endcode
 */

namespace ctrl {

/**
 * @brief ISensor implementation that reads from a SimPlant.
 */
class SimSensor : public ISensor {
public:
    /**
     * @brief Construct the sensor adapter.
     * @param plant SimPlant instance (must outlive this object).
     */
    explicit SimSensor(SimPlant& plant) : plant_(&plant) {}

    /**
     * @brief Return the plant's cached output from the last step().
     * @return y[k].
     */
    double read()          override { return plant_->output(); }

    /**
     * @brief Always returns `true` (simulation has no hardware faults).
     * @return `true`.
     */
    bool   isValid() const override { return true; }

    /**
     * @brief No-op - plant reset is the caller's responsibility.
     */
    void   reset()         override {}

private:
    SimPlant* plant_; ///< Non-owning pointer; lifetime managed by the caller.
};

} // namespace ctrl
