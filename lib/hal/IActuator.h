#pragma once
#include <cmath>

/**
 * @file IActuator.h
 * @brief Abstract interface for any command output.
 *
 * Implementations may wrap a DAC, PWM channel, EtherCAT drive, or a simulation
 * plant (SimActuator). Controllers call `write()` to issue a setpoint; the HAL
 * layer translates that into the physical or simulated command.
 *
 * **Contract:**
 * - `write(u)` - applies control value @p u to the physical or simulated plant.
 *   Non-finite values (NaN, +/-Inf) must be rejected silently; the actuator holds
 *   its last valid output instead.
 * - `lastOutput()` - returns the most recent value successfully written.
 *   Useful for warm-start and bumpless transfer.
 * - `reset()` - drives output to zero and clears internal state.
 */

namespace ctrl {

/**
 * @brief Abstract base class for all actuator adapters in the HAL.
 */
class IActuator {
public:
    virtual ~IActuator() = default;

    /**
     * @brief Apply a control value to the actuator.
     *
     * Non-finite values (@p u = NaN or +/-Inf) must be silently rejected and the
     * last valid output held.
     *
     * @param u Control command to apply.
     */
    virtual void   write(double u)     = 0;

    /**
     * @brief Return the most recent successfully applied control value.
     * @return Last finite value passed to write().
     */
    virtual double lastOutput() const  = 0;

    /**
     * @brief Drive output to zero and clear internal state.
     */
    virtual void   reset()             {}
};

} // namespace ctrl
