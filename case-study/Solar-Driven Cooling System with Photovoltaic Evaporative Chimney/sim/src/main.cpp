#include "solar_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#ifndef SOLAR_SIM_SOURCE_DIR
#define SOLAR_SIM_SOURCE_DIR "."
#endif

// ---------------------------------------------------------------------------
// Scenario weather and setpoint profiles
// ---------------------------------------------------------------------------
static solar::WeatherInput weatherSteady(double /*t*/)
{
    return {920.0, 35.6, 0.35, 3.028};
}

static solar::WeatherInput weatherIrrRamp(double t)
{
    // G rises from 250 to 1000 W/m^2 over 1 hour
    double G = std::min(1000.0, 250.0 + t / 3.6);
    return {G, 30.0, 0.50, 5.0};
}

static solar::WeatherInput weatherCloud(double t)
{
    // Cloud shadow between t = 600 s and t = 1800 s
    double G = (t > 600.0 && t < 1800.0) ? 500.0 : 900.0;
    return {G, 28.0, 0.60, 4.0};
}

static solar::WeatherInput weatherStep(double /*t*/)
{
    return {750.0, 32.0, 0.45, 3.5};
}

static solar::WeatherInput weatherHighHumid(double /*t*/)
{
    return {800.0, 38.0, 0.90, 2.0};
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    std::string base_dir = (argc > 1) ? argv[1] : SOLAR_SIM_SOURCE_DIR;
    std::string cfg_path = base_dir + "/config/plant_params.json";
    std::string log_dir  = base_dir + "/logs";

    // Load plant parameters once
    solar::PlantParams params;
    try {
        params = solar::PlantParams::fromJson(cfg_path);
    } catch (const std::exception& ex) {
        std::cerr << "ERROR loading config: " << ex.what() << '\n';
        return 1;
    }

    const double Ts = params.dt;

    // --- Scenarios -----------------------------------------------------------
    using solar::ScenarioConfig;
    using solar::WeatherFn;
    using solar::SetpointFn;

    std::vector<ScenarioConfig> scenarios = {
        {
            "s01_design_steady",
            "Design-point steady-state regulation",
            weatherSteady,
            [](double) { return 40.0; },
            params.duration, Ts
        },
        {
            "s02_irradiance_ramp",
            "Rising irradiance - FF-PID anticipates load",
            weatherIrrRamp,
            [](double) { return 40.0; },
            params.duration, Ts
        },
        {
            "s03_cloudy_disturbance",
            "Cloud step disturbance then recovery",
            weatherCloud,
            [](double) { return 38.0; },
            params.duration, Ts
        },
        {
            "s04_setpoint_step",
            "Setpoint step: 42 -> 38 degC at t=1800 s",
            weatherStep,
            [](double t) { return (t < 1800.0) ? 42.0 : 38.0; },
            params.duration, Ts
        },
        {
            "s05_high_humidity",
            "High humidity reduces evaporative effectiveness",
            weatherHighHumid,
            [](double) { return 40.0; },
            params.duration, Ts
        },
    };

    // --- Controllers ---------------------------------------------------------
    // Construct fresh controllers per scenario so internal state doesn't bleed.
    // We build a factory vector instead of a pre-built vector.

    std::cout << "Solar-Driven Cooling System - Numerical Simulation\n";
    std::cout << "====================================================\n";
    std::cout << "Config  : " << cfg_path  << '\n';
    std::cout << "Logs    : " << log_dir   << '\n';
    std::cout << "Ts      : " << Ts        << " s\n";
    std::cout << "Duration: " << params.duration << " s\n";
    std::cout << "Scenarios: " << scenarios.size() << "\n";
    std::cout << "Controllers per scenario: 9  (PID, FFPID, MPC, ADRC, FuzzyPID, SmithPredictor, MRAC, ESC, GPC-RLS)\n";
    std::cout << "Total runs: " << scenarios.size() * 9 << "\n\n";

    int total = static_cast<int>(scenarios.size()) * 9;
    int done  = 0;

    for (const auto& sc : scenarios) {
        std::cout << "=== " << sc.id << "  |  " << sc.description << " ===\n";

        // Rebuild controllers fresh for each scenario
        std::vector<std::unique_ptr<solar::ControllerBase>> controllers;
        controllers.push_back(std::make_unique<solar::PIDController>(Ts));
        controllers.push_back(std::make_unique<solar::FFPIDController>(Ts));
        controllers.push_back(std::make_unique<solar::MPCController>(Ts, 40.0, 0.018, 4.5));
        controllers.push_back(std::make_unique<solar::ADRCSolarCtrl>(Ts));
        controllers.push_back(std::make_unique<solar::FuzzyPIDSolarCtrl>(Ts));
        controllers.push_back(std::make_unique<solar::SmithPredictorSolarCtrl>(Ts));
        controllers.push_back(std::make_unique<solar::MRACSolarCtrl>(Ts));
        // Wave 3 additions
        controllers.push_back(std::make_unique<solar::ESCSolarCtrl>(Ts));
        controllers.push_back(std::make_unique<solar::GPCRLSSolarCtrl>(Ts, 40.0));

        for (auto& ctrl : controllers) {
            try {
                runSimulation(sc, *ctrl, params, log_dir);
            } catch (const std::exception& ex) {
                std::cerr << "\n  ERROR [" << sc.id << " | "
                          << ctrl->name() << "]: " << ex.what() << '\n';
            }
            ++done;
        }
        std::cout << '\n';
    }

    std::cout << "Done. " << done << '/' << total
              << " runs completed. Results in: " << log_dir << '\n';
    return 0;
}
