#include "smismo_plant.h"
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
#ifndef SMISMO_SIM_SOURCE_DIR
#define SMISMO_SIM_SOURCE_DIR "."
#endif
    std::string base_dir   = (argc > 1) ? argv[1] : SMISMO_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";

    std::filesystem::create_directories(log_dir);

    // -- Load plant -----------------------------------------------------------
    std::cout << "Loading plant parameters from: " << plant_json << '\n';
    smismo::PlantParams plant;
    try {
        plant = smismo::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }
    std::cout << "SMISMO plant parameters (Chen et al. 2018, Table 1):\n"
              << "  P_s="    << plant.P_s * 1e-5  << " bar"
              << "  A1="     << plant.A1 * 1e4    << " cm^2"
              << "  A2="     << plant.A2 * 1e4    << " cm^2"
              << "  m="      << plant.m           << " kg\n"
              << "  beta_e=" << plant.beta_e * 1e-6 << " MPa"
              << "  P_bd="   << plant.P_bd * 1e-5 << " bar"
              << "  Ts="     << plant.Ts * 1e3    << " ms\n\n";

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

    // -- Controllers (13) -----------------------------------------------------
    static constexpr int N_CONTROLLERS = 13;

    std::vector<std::unique_ptr<smismo::ControllerBase>> controllers;
    controllers.push_back(std::make_unique<smismo::PIDPosCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::CascadePIDCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::LQRCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::LQGCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::MPCCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::ADRCCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::SMCCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::FLCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::TubeMPCCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::L1Ctrl>(plant));
    controllers.push_back(std::make_unique<smismo::GainSchedCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::NMPCCtrl>(plant));
    controllers.push_back(std::make_unique<smismo::DOBEnergyCtrl>(plant));

    static_assert(N_CONTROLLERS == 13, "Update N_CONTROLLERS when adding/removing controllers");
    if (static_cast<int>(controllers.size()) != N_CONTROLLERS) {
        std::cerr << "Controller count mismatch: expected " << N_CONTROLLERS
                  << " got " << controllers.size() << '\n';
        return 1;
    }

    // -- Run all pairs --------------------------------------------------------
    const int total = static_cast<int>(scen_files.size() * controllers.size());
    int done = 0;
    std::cout << "Running " << total << " simulations ("
              << scen_files.size()  << " scenarios x "
              << controllers.size() << " controllers)...\n\n";

    for (auto& sf : scen_files) {
        smismo::ScenarioConfig scenario;
        try {
            scenario = smismo::ScenarioConfig::fromJson(sf);
        } catch (const std::exception& e) {
            std::cerr << "Skipping " << sf << ": " << e.what() << '\n';
            continue;
        }

        for (auto& ctrl : controllers) {
            try {
                smismo::runSimulation(plant, scenario, *ctrl, log_dir);
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
