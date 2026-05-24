#include "boiler_plant.h"
#include "linearizer.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <filesystem>
#include <algorithm>
#include <iomanip>

#ifndef BOILER_SIM_SOURCE_DIR
#define BOILER_SIM_SOURCE_DIR "."
#endif

int main(int argc, char* argv[])
{
    std::string base_dir     = (argc > 1) ? argv[1] : BOILER_SIM_SOURCE_DIR;
    std::string scenarios_dir = base_dir + "/config/scenarios";
    std::string log_dir       = base_dir + "/logs";

    std::filesystem::create_directories(log_dir);

    // -- Scenarios ------------------------------------------------------------
    std::vector<std::string> scenario_files;
    for (auto& entry : std::filesystem::directory_iterator(scenarios_dir)) {
        if (entry.path().extension() == ".json")
            scenario_files.push_back(entry.path().string());
    }
    std::sort(scenario_files.begin(), scenario_files.end());

    if (scenario_files.empty()) {
        std::cerr << "No scenario JSON files found in: " << scenarios_dir << '\n';
        return 1;
    }

    // -- Build linearised models for each operating point --------------------
    // Controllers are constructed fresh for each scenario to get the correct
    // operating-point-specific linearisation.

    // Pre-compute linearisations for all three operating points
    auto ss_A = boiler::linearize(boiler::op_A, 1.0);
    auto ss_B = boiler::linearize(boiler::op_B, 1.0);
    auto ss_C = boiler::linearize(boiler::op_C, 1.0);

    std::cout << "Boiler Control Full Numerical Simulation\n";
    std::cout << "=========================================\n";
    std::cout << "Scenarios: " << scenario_files.size() << "\n";
    std::cout << "Controllers per scenario: 18\n";
    std::cout << "Total runs: " << scenario_files.size() * 18 << "\n\n";

    int total = static_cast<int>(scenario_files.size()) * 18;
    int done  = 0;

    for (auto& sf : scenario_files) {
        boiler::ScenarioConfig scenario;
        try {
            scenario = boiler::ScenarioConfig::fromJson(sf);
        } catch (const std::exception& e) {
            std::cerr << "Skipping " << sf << ": " << e.what() << '\n';
            continue;
        }

        std::cout << "\n=== Scenario: " << scenario.id
                  << " (" << scenario.description << ") ===\n";

        // Select linearisation for this scenario's operating point
        const boiler::OperatingPoint& op =
            boiler::getOperatingPoint(scenario.operating_point);
        ctrl::StateSpace ss = boiler::linearize(op, scenario.Ts);

        // Build all 18 controllers for this operating point
        std::vector<std::unique_ptr<boiler::ControllerBase>> controllers;
        controllers.push_back(std::make_unique<boiler::PIDController>(ss, op));
        controllers.push_back(std::make_unique<boiler::LQRController>(ss, op));
        controllers.push_back(std::make_unique<boiler::LQGController>(ss, op));
        controllers.push_back(std::make_unique<boiler::MPCController>(ss, op));
        controllers.push_back(std::make_unique<boiler::SMCController>(ss, op));
        controllers.push_back(std::make_unique<boiler::ESCController>(ss, op));
        controllers.push_back(std::make_unique<boiler::ADRCController>(ss, op));
        controllers.push_back(std::make_unique<boiler::LeadLagPIDController>(ss, op));
        controllers.push_back(std::make_unique<boiler::SmithPredictorController>(ss, op));
        controllers.push_back(std::make_unique<boiler::GPCController>(ss, op));
        controllers.push_back(std::make_unique<boiler::EKFLQRController>(ss, op));
        controllers.push_back(std::make_unique<boiler::UKFLQRController>(ss, op));
        controllers.push_back(std::make_unique<boiler::FuzzyPIDController>(ss, op));
        controllers.push_back(std::make_unique<boiler::FuzzySupMPCController>(ss, op));
        controllers.push_back(std::make_unique<boiler::SupervisoryStackController>(ss, op));
        controllers.push_back(std::make_unique<boiler::AdditiveStackController>(ss, op));
        controllers.push_back(std::make_unique<boiler::WeightedStackController>(ss, op));
        controllers.push_back(std::make_unique<boiler::RepetitiveCtrl>(ss, op));

        for (auto& ctrl : controllers) {
            try {
                boiler::runSimulation(scenario, *ctrl, log_dir);
            } catch (const std::exception& e) {
                std::cerr << "\n  ERROR [" << scenario.id << " | "
                          << ctrl->name() << "]: " << e.what() << '\n';
            }
            ++done;
        }
    }

    std::cout << "\n\nDone. " << done << '/' << total
              << " runs completed. Results in: " << log_dir << '\n';
    return 0;
}
