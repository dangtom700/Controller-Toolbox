#include "sotec_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <filesystem>
#include <algorithm>

int main(int argc, char* argv[])
{
#ifndef SOTEC_SIM_SOURCE_DIR
#define SOTEC_SIM_SOURCE_DIR "."
#endif
    std::string base_dir   = (argc > 1) ? argv[1] : SOTEC_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";

    std::filesystem::create_directories(log_dir);

    // -- Load plant -----------------------------------------------------------
    std::cout << "Loading plant parameters from: " << plant_json << '\n';
    sotec::PlantParams plant;
    try {
        plant = sotec::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }
    std::cout << "S-OTEC plant parameters:\n"
              << "  A_coll="    << plant.A_coll    << " m^2"
              << "  eta_coll="  << plant.eta_coll  << '\n'
              << "  m_tank="    << plant.m_tank     << " kg"
              << "  Ts="        << plant.Ts         << " s\n\n";

    // -- Scenarios ------------------------------------------------------------
    std::vector<std::string> scen_files;
    for (auto& entry : std::filesystem::directory_iterator(scen_dir)) {
        if (entry.path().extension() == ".json")
            scen_files.push_back(entry.path().string());
    }
    std::sort(scen_files.begin(), scen_files.end());

    if (scen_files.empty()) {
        std::cerr << "No scenario JSON files found in: " << scen_dir << '\n';
        return 1;
    }

    // -- Controllers (12) -----------------------------------------------------
    static constexpr int N_CONTROLLERS = 12;

    std::vector<std::unique_ptr<sotec::ControllerBase>> controllers;
    controllers.push_back(std::make_unique<sotec::OpenLoopCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::PIDThCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::ADRCCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::MPCCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::LQRCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::FuzzyPIDCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::MRACCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::L1AdaptiveCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::GainScheduledCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::ScenarioMPCCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::DynaCtrl>(plant));
    controllers.push_back(std::make_unique<sotec::NeuralPIDCtrl>(plant));

    static_assert(N_CONTROLLERS == 12, "Update N_CONTROLLERS when adding/removing controllers");
    if (static_cast<int>(controllers.size()) != N_CONTROLLERS) {
        std::cerr << "Controller count mismatch: expected " << N_CONTROLLERS
                  << " got " << controllers.size() << '\n';
        return 1;
    }

    // -- Run all pairs --------------------------------------------------------
    const int total = static_cast<int>(scen_files.size() * controllers.size());
    int done  = 0;
    std::cout << "Running " << total << " simulations ("
              << scen_files.size()  << " scenarios x "
              << controllers.size() << " controllers)...\n\n";

    for (auto& sf : scen_files) {
        sotec::ScenarioConfig scenario;
        try {
            scenario = sotec::ScenarioConfig::fromJson(sf);
        } catch (const std::exception& e) {
            std::cerr << "Skipping " << sf << ": " << e.what() << '\n';
            continue;
        }

        for (auto& ctrl : controllers) {
            try {
                sotec::runSimulation(plant, scenario, *ctrl, log_dir);
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
