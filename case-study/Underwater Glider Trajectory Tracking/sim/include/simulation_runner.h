#pragma once
// simulation_runner.h - one (controller, scenario) run for Underwater Glider Trajectory Tracking (TEMPLATE)
#include "underwater_glider_trajectory_tracking_plant.h"
#include "controllers.h"
#include <string>

namespace underwaterglidertrajectorytracking {

struct Scenario {
    std::string id;
    double T_sim   = 30.0;
    double ref_init  = 0.0;
    double ref_final = 1.0;
    double step_time = 1.0;
    static Scenario fromJson(const std::string& path);
};

// Run one simulation; writes CSV telemetry to <log_dir>/run_<scenario>_<ctrl>.csv.
// Returns the final cumulative IAE.
double runSimulation(const PlantParams& plant,
                     const Scenario& scen,
                     const NamedController& nc,
                     const std::string& log_dir);

}  // namespace underwaterglidertrajectorytracking
