#include "plant_parameters.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <filesystem>

int main(int argc, char* argv[])
{
    // ── Paths ────────────────────────────────────────────────────────────────
    std::string base_dir = (argc > 1) ? argv[1] : ".";
    std::string plant_json    = base_dir + "/config/plant_params.json";
    std::string scenarios_dir = base_dir + "/config/scenarios";
    std::string log_dir       = base_dir + "/logs";

    std::filesystem::create_directories(log_dir);

    // ── Load plant ───────────────────────────────────────────────────────────
    std::cout << "Loading plant parameters from: " << plant_json << '\n';
    tug::PlantParameters plant;
    try {
        plant = tug::PlantParameters::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }
    std::cout << "M_re diagonal: " << plant.M_re.diagonal().transpose() << '\n';
    std::cout << "D_re diagonal: " << plant.D_re.diagonal().transpose() << '\n';

    // ── Scenarios ─────────────────────────────────────────────────────────────
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

    // ── Controllers ───────────────────────────────────────────────────────────
    // Build them once; runner calls reset() before each run.
    std::vector<std::unique_ptr<tug::ControllerBase>> controllers;
    controllers.push_back(std::make_unique<tug::PIDController>(plant));
    controllers.push_back(std::make_unique<tug::KFPIDController>(plant));
    controllers.push_back(std::make_unique<tug::SMCController>(plant));
    controllers.push_back(std::make_unique<tug::MPCController>(plant));
    controllers.push_back(std::make_unique<tug::ESCController>(plant));
    controllers.push_back(std::make_unique<tug::FuzzyPIDController>(plant));
    controllers.push_back(std::make_unique<tug::FuzzySupervised_MPC>(plant));

    // ── Run all (scenario x controller) pairs ─────────────────────────────────
    int total = static_cast<int>(scenario_files.size() * controllers.size());
    int done  = 0;
    std::cout << "\nRunning " << total << " simulations ("
              << scenario_files.size() << " scenarios x "
              << controllers.size() << " controllers)...\n\n";

    for (auto& sf : scenario_files) {
        ScenarioConfig scenario;
        try {
            scenario = ScenarioConfig::fromJson(sf);
        } catch (const std::exception& e) {
            std::cerr << "Skipping " << sf << ": " << e.what() << '\n';
            continue;
        }

        for (auto& ctrl : controllers) {
            try {
                runSimulation(plant, scenario, *ctrl, log_dir);
            } catch (const std::exception& e) {
                std::cerr << "ERROR [" << scenario.id << " | "
                          << ctrl->name() << "]: " << e.what() << '\n';
            }
            ++done;
        }
    }

    std::cout << "\nDone. " << done << '/' << total
              << " runs completed. Results in: " << log_dir << '\n';
    return 0;
}
