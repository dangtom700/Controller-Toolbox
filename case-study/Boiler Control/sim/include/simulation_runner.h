#pragma once
#include "boiler_plant.h"
#include "telemetry_logger.h"
#include <string>
#include <memory>

// Forward declaration to avoid circular include with controllers.h
namespace boiler { class ControllerBase; }

namespace boiler {

struct ScenarioConfig {
    std::string id;
    std::string description;
    std::string operating_point;    // "A", "B", or "C"
    std::string mode;               // "regulation", "tracking", "periodic_tracking"
    Eigen::Vector3d dx0;            // initial state perturbation
    Eigen::Vector3d setpoint_dy;    // output deviation setpoint (base / constant)
    double duration_s;
    double Ts;

    // Multi-op transition fields (s07)
    bool has_transition = false;
    double transition_time_s = 0.0;
    std::string transition_to_op;

    // Periodic load disturbance fields (s08)
    // Active when mode == "periodic_tracking".
    // Setpoint = setpoint_dy + periodic_amp * sin(2*pi*periodic_freq_hz*t)
    bool            has_periodic    = false;
    Eigen::Vector3d periodic_amp    = Eigen::Vector3d::Zero();
    double          periodic_freq_hz = 0.0;

    static ScenarioConfig fromJson(const std::string& path);
};

// Run one (scenario, controller) pair and write CSV to log_dir.
void runSimulation(const ScenarioConfig& scenario,
                   ControllerBase& controller,
                   const std::string& log_dir);

} // namespace boiler
