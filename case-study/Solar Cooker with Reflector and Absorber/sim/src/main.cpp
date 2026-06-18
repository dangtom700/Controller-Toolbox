#include "solar_cooker_plant.h"
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
#ifndef SOLAR_COOKER_SIM_SOURCE_DIR
#define SOLAR_COOKER_SIM_SOURCE_DIR "."
#endif
    std::string base_dir   = (argc > 1) ? argv[1] : SOLAR_COOKER_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";

    std::filesystem::create_directories(log_dir);

    // -- Load plant -----------------------------------------------------------
    std::cout << "Loading plant parameters from: " << plant_json << '\n';
    cooker::PlantParams plant;
    try {
        plant = cooker::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }
    std::cout << "Solar cooker parameters:\n"
              << "  A_box=" << plant.A_box << " m^2"
              << "  eta_box=" << plant.eta_box << "  eta_TBPR=" << plant.eta_TBPR << '\n'
              << "  T_melt=" << plant.T_melt << " C"
              << "  m_pcm=" << plant.m_pcm << " kg"
              << "  Ts=" << plant.Ts << " s\n\n";

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
    std::vector<std::unique_ptr<cooker::ControllerBase>> controllers;
    controllers.push_back(std::make_unique<cooker::OpenLoopCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::PIDCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::ADRCCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::MPCCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::FuzzyPIDCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::SMCCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::GainScheduledCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::MRACCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::L1AdaptiveCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::NeuralPIDCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::DynaCookerCtrl>(plant));
    controllers.push_back(std::make_unique<cooker::ScenarioMPCCookerCtrl>(plant));

    static constexpr int N_CONTROLLERS = 12;
    if (static_cast<int>(controllers.size()) != N_CONTROLLERS) {
        std::cerr << "ERROR: Expected " << N_CONTROLLERS
                  << " controllers, got " << controllers.size() << '\n';
        return 1;
    }

    // -- Run all pairs --------------------------------------------------------
    int total = static_cast<int>(scen_files.size() * controllers.size());
    int done  = 0;
    std::cout << "Running " << total << " simulations ("
              << scen_files.size()  << " scenarios x "
              << controllers.size() << " controllers)...\n\n";

    for (auto& sf : scen_files) {
        cooker::ScenarioConfig scenario;
        try {
            scenario = cooker::ScenarioConfig::fromJson(sf);
        } catch (const std::exception& e) {
            std::cerr << "Skipping " << sf << ": " << e.what() << '\n';
            continue;
        }

        for (auto& ctrl : controllers) {
            try {
                cooker::runSimulation(plant, scenario, *ctrl, log_dir);
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
