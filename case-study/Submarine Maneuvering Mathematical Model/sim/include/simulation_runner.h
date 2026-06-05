#pragma once
#include "submarine_plant.h"
#include "controllers.h"
#include <functional>
#include <string>

namespace sub {

// Disturbance function: time [s] -> dist_Y [N] (lateral force)
using DisturbFn  = std::function<double(double t)>;

// Setpoint functions: time [s] -> reference value
using SetpointFn = std::function<double(double t)>;

// ---------------------------------------------------------------------------
// Scenario configuration
// ---------------------------------------------------------------------------
struct ScenarioConfig {
    std::string  id;
    std::string  description;
    double       duration_s;        // total simulation time [s]
    double       Ts;                // sample time [s]
    SubState     initial;           // initial plant state
    SetpointFn   psi_ref_fn;        // heading reference [rad]
    SetpointFn   z_ref_fn;          // depth reference [m]
    DisturbFn    disturbance_fn;    // lateral disturbance force [N]
};

// ---------------------------------------------------------------------------
// Per-run metrics written to CSV footer comment
// ---------------------------------------------------------------------------
struct RunMetrics {
    double iae_heading = 0.0;   // integral absolute heading error [rad.s]
    double iae_depth   = 0.0;   // integral absolute depth error [m.s]
    double max_overshoot = 0.0; // maximum heading overshoot [rad]
};

// ---------------------------------------------------------------------------
// Run one (scenario, controller) pair.  Writes CSV to log_dir.
// ---------------------------------------------------------------------------
void runSimulation(const ScenarioConfig& scenario,
                   ControllerBase& controller,
                   const PlantParams& plant,
                   const std::string& log_dir);

} // namespace sub
