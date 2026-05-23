#pragma once
#include <cmath>

// IActuator - abstract interface for any command output.
//
// Implementations may wrap a DAC, PWM channel, EtherCAT drive, or a
// simulation plant (SimActuator).  Controllers call write() to issue a
// setpoint; the HAL layer translates that into the physical command.
//
// Contract:
//   write(u)      - applies control value u to the physical or simulated plant.
//                   Non-finite values (NaN, +/-Inf) must be rejected silently;
//                   the actuator holds its last valid output instead.
//   lastOutput()  - returns the most recent value successfully written.
//                   Useful for warm-start and bumpless transfer.
//   reset()       - drives output to zero and clears internal state.
namespace ctrl {

class IActuator {
public:
    virtual ~IActuator() = default;

    virtual void   write(double u)       = 0;
    virtual double lastOutput()  const   = 0;
    virtual void   reset()               {}
};

} // namespace ctrl
