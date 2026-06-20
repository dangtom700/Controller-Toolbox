#pragma once
// simulation_runner.h - one (controller, scenario) run for the airship case study.
#include "bouyancy_driven_airship_in_vertical_plan_plant.h"
#include "controllers.h"
#include <string>

namespace bouyancydrivenairshipinverticalplan {

// Reference/disturbance profile for one scenario. ref_type selects between a single step
// (s01-s04) and the Sec. 4.6 sawtooth bang-bang flight (s05) - see thetaRefAt()/m0At().
struct Scenario {
    std::string id;
    std::string description;

    double T_sim      = 60.0;   // [s]
    double theta0_deg  = 41.5;  // initial pitch angle [deg]
    double rp1_ref     = -1.15; // initial rp1 AND the LQR/MPC/FBL trim reference [m]
    double v1_0        = 0.0;   // initial forward velocity [m/s]
    double v3_0         = 0.0;  // initial heave velocity [m/s]
    double m0_initial   = 1.0;  // net-lift parameter [kg] (constant unless ref_type=sawtooth)

    std::string ref_type = "step";  // "step" or "sawtooth"

    // "step" fields
    double theta_ref_deg = 30.0;
    double step_time     = 5.0;

    // "sawtooth" fields (Sec. 4.6)
    double theta_ref_a_deg = 25.0;
    double theta_ref_b_deg = -15.0;
    double half_period     = 37.5;   // [s]
    double m0_switch_kg    = 34.66;  // +/- net-lift switch magnitude [kg]

    // s04 disturbance: one-shot pitch-rate impulse at dist_time (< 0 disables it)
    double dist_time        = -1.0;  // [s]
    double dist_q_impulse   = 0.0;   // [rad/s] added once to q at dist_time

    double thetaRefAt(double t) const;  // [rad]
    double m0At(double t) const;        // [kg]

    static Scenario fromJson(const std::string& path);
};

// Run one simulation; writes CSV telemetry to <log_dir>/run_<scenario.id>_<ctrl.name()>.csv.
// Returns the final cumulative IAE (integral |theta_ref - theta| dt).
double runSimulation(const PlantParams&    plant,
                     const Scenario&       scen,
                     ControllerBase&       ctrl,
                     const std::string&    log_dir);

}  // namespace bouyancydrivenairshipinverticalplan
