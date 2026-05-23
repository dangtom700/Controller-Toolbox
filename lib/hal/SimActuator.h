#pragma once
#include "IActuator.h"
#include "SimPlant.h"
#include <cmath>
#include <limits>

// SimActuator - IActuator adapter backed by a SimPlant.
//
// write(u) applies the control input to the plant:
//   - Non-finite values (NaN, +/-Inf) are rejected; the last valid output is
//     repeated and the plant is stepped with that value.  This matches the
//     "hold-last" actuator fail-safe behaviour expected from real hardware.
//   - An optional saturation limit [uMin, uMax] (defaults to +/-inf) is applied
//     before forwarding to the plant, mirroring physical actuator limits.
//
// lastOutput() returns the most recent value actually applied to the plant.
namespace ctrl {

class SimActuator : public IActuator {
public:
    explicit SimActuator(SimPlant& plant,
                         double uMin = -std::numeric_limits<double>::infinity(),
                         double uMax =  std::numeric_limits<double>::infinity())
        : plant_(&plant), u_last_(0.0), u_min_(uMin), u_max_(uMax)
    {}

    void write(double u) override
    {
        // Reject non-finite inputs - hold last good output (safe-state behaviour)
        if (!std::isfinite(u)) u = u_last_;

        // Apply actuator saturation limits
        if (u < u_min_) u = u_min_;
        if (u > u_max_) u = u_max_;

        u_last_ = u;
        plant_->step(u);
    }

    double lastOutput() const override { return u_last_; }

    void reset() override { u_last_ = 0.0; }

private:
    SimPlant* plant_;  // non-owning
    double    u_last_;
    double    u_min_;
    double    u_max_;
};

} // namespace ctrl
