#pragma once
#include "ISensor.h"
#include "SimPlant.h"

// SimSensor - ISensor adapter backed by a SimPlant.
//
// read() returns the plant's cached output from the last step().
// isValid() is always true for a simulation (no hardware faults).
//
// Usage:
//   ctrl::SimPlant  plant(sys);
//   ctrl::SimSensor sensor(plant);
//   ctrl::SimActuator actuator(plant);
//
//   double y = sensor.read();        // y[k] = plant output
//   double u = pid.compute(r - y);
//   actuator.write(u);               // advances plant -> x[k+1]
namespace ctrl {

class SimSensor : public ISensor {
public:
    explicit SimSensor(SimPlant& plant) : plant_(&plant) {}

    double read()          override { return plant_->output(); }
    bool   isValid() const override { return true; }
    void   reset()         override { /* plant reset is the caller's responsibility */ }

private:
    SimPlant* plant_; // non-owning; lifetime managed by the caller
};

} // namespace ctrl
