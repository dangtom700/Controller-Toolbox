#pragma once
#include "humid_plant.h"
#include "controllers.h"
#include <string>
#include <functional>

namespace humid {

// Disturbance profile function: maps simulation time [s] -> Disturbance
using DisturbFn  = std::function<Disturbance(double t)>;

// Setpoint profile function: maps time [s] -> ref_phi [-]
using SetpointFn = std::function<double(double t)>;

struct ScenarioConfig {
    std::string  id;
    std::string  description;
    DisturbFn    disturbance_fn;
    SetpointFn   setpoint_fn;
    double       phi_init;     // initial room RH [-]
    double       duration_s;
    double       Ts;
};

// Run one (scenario, controller) pair. Writes CSV to log_dir.
void runSimulation(const ScenarioConfig& scenario,
                   ControllerBase& controller,
                   const PlantParams& plant_params,
                   const std::string& log_dir);

} // namespace humid
