#include "humid_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#ifndef HUMID_SIM_SOURCE_DIR
#define HUMID_SIM_SOURCE_DIR "."
#endif

// ---------------------------------------------------------------------------
// Scenario disturbance profiles
// ---------------------------------------------------------------------------

static humid::Disturbance d_nominal(double /*t*/)
{
    return {263.15, 0.35, 0.0};   // T_out=-10^\circC, phi_out=0.35, 0 occupants
}

static humid::Disturbance d_cold_snap(double /*t*/)
{
    return {253.15, 0.20, 0.0};   // T_out=-20^\circC, phi_out=0.20
}

static humid::Disturbance d_setpoint_step(double /*t*/)
{
    return {263.15, 0.35, 0.0};
}

static humid::Disturbance d_occupancy(double t)
{
    // 4 occupants enter at t=600s, leave at t=2400s
    double n = (t >= 600.0 && t < 2400.0) ? 4.0 : 0.0;
    return {263.15, 0.35, n};
}

static humid::Disturbance d_mild_humid(double /*t*/)
{
    return {270.15, 0.55, 0.0};   // T_out=-3^\circC, phi_out=0.55
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    std::string base_dir = (argc > 1) ? argv[1] : HUMID_SIM_SOURCE_DIR;
    std::string cfg_path = base_dir + "/config/plant_params.json";
    std::string log_dir  = base_dir + "/logs";

    humid::PlantParams params;
    try {
        params = humid::PlantParams::fromJson(cfg_path);
    } catch (const std::exception& ex) {
        std::cerr << "ERROR loading config: " << ex.what() << '\n';
        return 1;
    }

    const double Ts = params.Ts;

    using humid::ScenarioConfig;
    using humid::DisturbFn;
    using humid::SetpointFn;

    std::vector<ScenarioConfig> scenarios = {
        {
            "s01_design",
            "Nominal winter - cold dry outdoor air",
            d_nominal,
            [](double) { return 0.45; },
            0.20, params.duration, Ts
        },
        {
            "s02_cold_snap",
            "Severe cold snap - maximum humidification demand",
            d_cold_snap,
            [](double) { return 0.45; },
            0.15, params.duration, Ts
        },
        {
            "s03_setpoint_step",
            "Setpoint step 30%->50% at t=900s",
            d_setpoint_step,
            [](double t) { return (t < 900.0) ? 0.30 : 0.50; },
            0.20, params.duration, Ts
        },
        {
            "s04_occupancy",
            "Occupancy moisture disturbance (4 persons, t=600-2400s)",
            d_occupancy,
            [](double) { return 0.45; },
            0.20, params.duration, Ts
        },
        {
            "s05_mild_humid",
            "Mild outdoor conditions - over-humidification risk",
            d_mild_humid,
            [](double) { return 0.45; },
            0.35, params.duration, Ts
        },
    };

    constexpr int kNumControllers = 10;

    std::cout << "Porous Fiber Plate Humidification System - Simulation\n";
    std::cout << "=======================================================\n";
    std::cout << "Config  : " << cfg_path  << '\n';
    std::cout << "Logs    : " << log_dir   << '\n';
    std::cout << "Ts      : " << Ts        << " s\n";
    std::cout << "Duration: " << params.duration << " s  ("
              << params.duration / 3600.0 << " h)\n";
    std::cout << "Scenarios: " << scenarios.size() << "\n";
    std::cout << "Controllers per scenario: " << kNumControllers << "\n";
    std::cout << "Total runs: " << scenarios.size() * kNumControllers << "\n\n";

    int total = static_cast<int>(scenarios.size()) * kNumControllers;
    int done  = 0;

    for (const auto& sc : scenarios) {
        std::cout << "=== " << sc.id << "  |  " << sc.description << " ===\n";

        std::vector<std::unique_ptr<humid::ControllerBase>> controllers;
        controllers.push_back(std::make_unique<humid::PIDHumidCtrl>(Ts));
        controllers.push_back(std::make_unique<humid::PID_AWHumidCtrl>(Ts));
        controllers.push_back(std::make_unique<humid::FFPIDHumidCtrl>(Ts));
        controllers.push_back(std::make_unique<humid::CascadePIDHumidCtrl>(Ts));
        controllers.push_back(std::make_unique<humid::GainSchedHumidCtrl>(Ts));
        controllers.push_back(std::make_unique<humid::SmithHumidCtrl>(Ts));
        controllers.push_back(std::make_unique<humid::ADRCHumidCtrl>(Ts));
        controllers.push_back(std::make_unique<humid::MPCHumidCtrl>(Ts));
        controllers.push_back(std::make_unique<humid::MRACHumidCtrl>(Ts));
        controllers.push_back(std::make_unique<humid::GPCRLSHumidCtrl>(Ts));

        for (auto& c : controllers) {
            try {
                runSimulation(sc, *c, params, log_dir);
            } catch (const std::exception& ex) {
                std::cerr << "\n  ERROR [" << sc.id << " | "
                          << c->name() << "]: " << ex.what() << '\n';
            }
            ++done;
        }
        std::cout << '\n';
    }

    std::cout << "Done. " << done << '/' << total
              << " runs completed. Results in: " << log_dir << '\n';
    return 0;
}
