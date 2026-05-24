#pragma once
#include "plant_parameters.h"
#include "environment.h"
#include "controllers.h"
#include "telemetry_logger.h"
#include "json.hpp"
#include <string>
#include <memory>

// Scenario configuration loaded from JSON.
struct ScenarioConfig {
    std::string id;
    std::string description;
    double wind_speed;       // m/s
    double wind_bearing_deg; // degrees
    double current_speed;    // m/s
    double current_bearing_deg;
    double Hs;               // m  (0 = use plant_params default)
    double Tp;               // s
    tug::EnvConditions toEnvConditions() const;

    double ref_x, ref_y, ref_psi_deg;  // target pose
    double duration;         // s  (0 = use plant_params default)
    uint32_t seed;

    static ScenarioConfig fromJson(const std::string& path);
};

// Runs a single (scenario, controller) pair and writes CSV to logs/.
void runSimulation(const tug::PlantParameters& plant,
                   const ScenarioConfig&        scenario,
                   tug::ControllerBase&         controller,
                   const std::string&           log_dir);
