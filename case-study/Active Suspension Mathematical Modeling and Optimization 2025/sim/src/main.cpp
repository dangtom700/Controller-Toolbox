#include "susp_plant.h"
#include "road_profile.h"
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
#ifndef SUSP_SIM_SOURCE_DIR
#define SUSP_SIM_SOURCE_DIR "."
#endif
    std::string base_dir     = (argc > 1) ? argv[1] : SUSP_SIM_SOURCE_DIR;
    std::string plant_json   = base_dir + "/config/plant_params.json";
    std::string scen_dir     = base_dir + "/config/scenarios";
    std::string log_dir      = base_dir + "/logs";

    std::filesystem::create_directories(log_dir);

    // -- Load plant -----------------------------------------------------------
    std::cout << "Loading plant parameters from: " << plant_json << '\n';
    susp::PlantParams plant;
    try {
        plant = susp::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }
    std::cout << "Quarter-car parameters:\n"
              << "  m_s=" << plant.m_s << " kg  m_u=" << plant.m_u << " kg\n"
              << "  k_s=" << plant.k_s << " N/m  c_s=" << plant.c_s << " N.s/m"
              << "  k_t=" << plant.k_t << " N/m\n"
              << "  omega_body="  << plant.omega_body  << " rad/s ("
              << plant.omega_body/(2.0*M_PI) << " Hz)\n"
              << "  omega_wheel=" << plant.omega_wheel << " rad/s ("
              << plant.omega_wheel/(2.0*M_PI) << " Hz)\n"
              << "  zeta_body="   << plant.zeta_body   << "\n\n";

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

    // -- Controllers ----------------------------------------------------------
    // Built once; runner calls reset() before each run.
    std::vector<std::unique_ptr<susp::ControllerBase>> controllers;
    controllers.push_back(std::make_unique<susp::PassiveCtrl>(plant));
    controllers.push_back(std::make_unique<susp::PIDSuspCtrl>(plant));
    controllers.push_back(std::make_unique<susp::ADRCSuspCtrl>(plant));
    controllers.push_back(std::make_unique<susp::SMCSuspCtrl>(plant));
    controllers.push_back(std::make_unique<susp::LQRSuspCtrl>(plant));
    controllers.push_back(std::make_unique<susp::LQGSuspCtrl>(plant));
    controllers.push_back(std::make_unique<susp::MPCSuspCtrl>(plant));
    controllers.push_back(std::make_unique<susp::MRACSuspCtrl>(plant));
    controllers.push_back(std::make_unique<susp::FuzzyPIDSuspCtrl>(plant));
    controllers.push_back(std::make_unique<susp::TubeMPCSuspCtrl>(plant));

    // -- Run all pairs --------------------------------------------------------
    int total = static_cast<int>(scen_files.size() * controllers.size());
    int done  = 0;
    std::cout << "Running " << total << " simulations ("
              << scen_files.size()    << " scenarios x "
              << controllers.size()   << " controllers)...\n\n";

    for (auto& sf : scen_files) {
        susp::ScenarioConfig scenario;
        try {
            scenario = susp::ScenarioConfig::fromJson(sf);
        } catch (const std::exception& e) {
            std::cerr << "Skipping " << sf << ": " << e.what() << '\n';
            continue;
        }

        for (auto& ctrl : controllers) {
            try {
                susp::runSimulation(plant, scenario, *ctrl, log_dir);
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
