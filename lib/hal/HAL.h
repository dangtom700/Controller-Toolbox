#pragma once

// HAL.h - Hardware Abstraction Layer convenience include.
//
// Pulls in ISensor, IActuator, SimPlant, SimSensor, and SimActuator.
// Include this header (or "ControllerToolbox.h") to access the full HAL.
//
// Example - closed-loop simulation with the HAL:
//
//   #include "ControllerToolbox.h"  // or "hal/HAL.h" directly
//
//   ctrl::StateSpace sys = ctrl::tf2ss(...);
//   ctrl::SimPlant    plant(sys);
//   ctrl::SimSensor   sensor(plant);
//   ctrl::SimActuator actuator(plant, /*uMin=*/-10.0, /*uMax=*/10.0);
//   ctrl::DiscretePID pid(params, Ts);
//
//   const double r = 1.0;
//   for (int k = 0; k < N; ++k) {
//       double y = sensor.read();
//       double u = pid.compute(r - y);
//       actuator.write(u);           // steps the plant
//   }
#include "ISensor.h"
#include "IActuator.h"
#include "SimPlant.h"
#include "SimSensor.h"
#include "SimActuator.h"
