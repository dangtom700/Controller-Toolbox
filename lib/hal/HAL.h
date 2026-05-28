#pragma once

/**
 * @file HAL.h
 * @brief Hardware Abstraction Layer - convenience umbrella include.
 *
 * Pulls in all HAL interfaces and simulation adapters. Include this header
 * (or `ControllerToolbox.h`) to access the full HAL.
 *
 * **Interfaces:**
 * - ISensor    - abstract sensor (`read`, `isValid`, `reset`).
 * - IActuator  - abstract actuator (`write`, `lastOutput`, `reset`).
 * - ITimer     - abstract monotonic clock (`nowNs`, `elapsedNs`).
 * - IScheduler - abstract periodic task scheduler (`setPeriodNs`, `start`, `stop`, `overrunCount`).
 *
 * **Simulation adapters:**
 * - SimPlant    - discrete-time state-space plant simulator.
 * - SimSensor   - ISensor backed by SimPlant.
 * - SimActuator - IActuator backed by SimPlant (NaN-safe, saturating).
 * - SafeSensor  - ISensor decorator that freezes on `isValid() == false`.
 * - StdTimer    - ITimer using `std::chrono::steady_clock` (non-RT; for tests).
 *
 * **Example - closed-loop simulation with the HAL:**
 * @code
 *   ctrl::StateSpace  sys = ctrl::tf2ss(...);
 *   ctrl::SimPlant    plant(sys);
 *   ctrl::SafeSensor  sensor(plant);           // freezes last-valid reading on fault
 *   ctrl::SimActuator actuator(plant, -10.0, 10.0);
 *   ctrl::DiscretePID pid(params, Ts);
 *   ctrl::StdTimer    timer;
 *
 *   const double   r         = 1.0;
 *   const uint64_t period_ns = static_cast<uint64_t>(Ts * 1e9);
 *   for (int k = 0; k < N; ++k) {
 *       uint64_t t0 = timer.nowNs();
 *       double y = sensor.read();
 *       double u = pid.compute(r - y);
 *       actuator.write(u);                    // steps the plant
 *       uint64_t dt = timer.elapsedNs(t0);
 *       if (dt > period_ns)
 *           std::cerr << "[overrun] " << dt << " ns > " << period_ns << " ns\n";
 *   }
 * @endcode
 */

#include "ISensor.h"
#include "IActuator.h"
#include "ITimer.h"
#include "IScheduler.h"
#include "StdTimer.h"
#include "SimPlant.h"
#include "SimSensor.h"
#include "SimActuator.h"
#include "SafeSensor.h"
#include "SimScheduler.h"
