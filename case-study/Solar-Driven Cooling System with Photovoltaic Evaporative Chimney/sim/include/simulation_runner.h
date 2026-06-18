#pragma once
#include "solar_plant.h"
#include "controllers.h"
#include <string>
#include <functional>

namespace solar {

// Weather profile function: maps simulation time [s] -> WeatherInput
using WeatherFn = std::function<WeatherInput(double t)>;

// Setpoint profile function: maps time [s] -> ref_Tw1 [^\circC]
using SetpointFn = std::function<double(double t)>;

struct ScenarioConfig {
    std::string  id;
    std::string  description;
    WeatherFn    weather_fn;
    SetpointFn   setpoint_fn;
    double       duration_s;
    double       Ts;
};

// Run one (scenario, controller) pair. Writes CSV to log_dir.
void runSimulation(const ScenarioConfig& scenario,
                   ControllerBase& controller,
                   const PlantParams& plant_params,
                   const std::string& log_dir);

} // namespace solar
