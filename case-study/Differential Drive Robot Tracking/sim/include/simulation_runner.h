#pragma once
// simulation_runner.h - one (controller, scenario) run for the Differential Drive Robot
// Tracking case study.
//
// Three nested rates, matching the paper's multi-rate design:
//
//   outer kinematic controller   Tf = 0.03 s     compute() -> (v_cmd, w_cmd); every 5th
//                                                call -> slowTick()  (Ts_slow = 0.15 s)
//   inner PI wheel-torque layer  Ts_plant        paper's tau_R / tau_L law + anti-windup
//   plant RK4                    Ts_plant = 0.005 s
//
// eps = Tf/Ts_slow = 0.2 is the paper's time-scale separation ratio.
#include "controllers.h"
#include "differential_drive_robot_tracking_plant.h"
#include "trajectory.h"
#include <random>
#include <string>

namespace differentialdriverobottracking {

struct Scenario {
    std::string id;
    std::string description;

    std::string trajectory = "lemniscate";  ///< lemniscate | circle | diamond
    double a          = 2.0;    ///< path amplitude [m]
    double T_sim      = 30.0;   ///< simulation horizon [s]
    double time_scale = 1.0;    ///< t_path = t*time_scale; 1.0 = the paper's literal parameterisation

    // Initial pose. When start_on_path is true, (x0, y0, theta0) are IGNORED and the robot
    // starts exactly on the reference at t = 0.
    bool   start_on_path = true;
    double x0 = 0.0, y0 = 0.0, theta0 = 0.0;

    // Disturbances / non-idealities.
    double encoder_noise_std  = 0.0;   ///< std dev of additive pose measurement noise [m, rad]
    double torque_dist_amp    = 0.0;   ///< amplitude of an external wheel-torque disturbance [N m]
    double torque_dist_freq   = 0.0;   ///< frequency of that disturbance [Hz]
    double tau_max_override   = -1.0;  ///< if > 0, overrides PlantParams::tau_max [N m]
    double mass_mismatch      = 1.0;   ///< multiplies the SIMULATED plant's M_total (controllers
                                       ///  are still designed on nominal params)
    unsigned seed             = 42u;

    static Scenario fromJson(const std::string& path);
    PathType pathType() const { return pathTypeFromString(trajectory); }
};

/// Performance indices for one run - the paper's Table 2 columns.
struct RunMetrics {
    double ise         = 0.0;   ///< int (ex^2 + ey^2) dt
    double iae         = 0.0;   ///< int (|ex| + |ey|) dt
    double itae        = 0.0;   ///< int t*(|ex| + |ey|) dt
    double final_err   = 0.0;   ///< |position error| at t = T_sim [m]
    double mean_torque = 0.0;   ///< mean of max(|tau_R|, |tau_L|) [N m]
    double max_torque  = 0.0;   ///< peak |tau| over the run [N m]
    double Ks_final    = 0.0;   ///< final adaptive sliding gain (NaN if not applicable)
    double settle_time = -1.0;  ///< first t after which |e_pos| < 0.05 m holds; -1 if never
    double max_err     = 0.0;   ///< peak position error [m]
};

/// Build the plant params actually handed to the simulated Plant for this scenario
/// (applies tau_max_override and mass_mismatch). Controllers are always built from nominal.
PlantParams effectivePlantParams(const PlantParams& nominal, const Scenario& scen);

/// Run one simulation; writes CSV telemetry to <log_dir>/run_<scenario.id>_<ctrl.name()>.csv.
/// Pass an empty log_dir to skip all file I/O (used by robustness_main.cpp).
RunMetrics runSimulation(const PlantParams& plant,
                         const Scenario&    scen,
                         ControllerBase&    ctrl,
                         const std::string& log_dir);

}  // namespace differentialdriverobottracking
