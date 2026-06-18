#include "buck_boost_plant.h"
#include "input_profile.h"
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
#ifndef BUCK_BOOST_SIM_SOURCE_DIR
#define BUCK_BOOST_SIM_SOURCE_DIR "."
#endif
    std::string base_dir   = (argc > 1) ? argv[1] : BUCK_BOOST_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";

    std::filesystem::create_directories(log_dir);

    // -- Load plant -----------------------------------------------------------
    std::cout << "Loading plant parameters from: " << plant_json << '\n';
    conv::PlantParams plant;
    try {
        plant = conv::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }
    std::cout << "Buck-Boost converter parameters:\n"
              << "  L=" << plant.L*1e6 << " uH  C=" << plant.C*1e3 << " mF"
              << "  R=" << plant.R << " Ohm  V_in=" << plant.V_in_nom << " V\n"
              << "  f_s=" << plant.f_s/1000.0 << " kHz  Ts=" << plant.Ts*1e6 << " us\n\n";

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
    // 12 controllers: 4 fuzzy-family + 8 comparison
    std::vector<std::unique_ptr<conv::ControllerBase>> controllers;
    controllers.push_back(std::make_unique<conv::OpenLoopCtrl>(plant));
    controllers.push_back(std::make_unique<conv::PIBuckCtrl>(plant));
    controllers.push_back(std::make_unique<conv::PIBoostCtrl>(plant));
    controllers.push_back(std::make_unique<conv::TLCSClassicPICtrl>(plant));
    controllers.push_back(std::make_unique<conv::FuzzyPDCtrl>(plant));
    controllers.push_back(std::make_unique<conv::FuzzyPIDBuckCtrl>(plant));
    controllers.push_back(std::make_unique<conv::FuzzyPIDBoostCtrl>(plant));
    controllers.push_back(std::make_unique<conv::TLCSFuzzyPICtrl>(plant));
    controllers.push_back(std::make_unique<conv::GainScheduledCtrl>(plant));
    controllers.push_back(std::make_unique<conv::ADRCConvCtrl>(plant));
    controllers.push_back(std::make_unique<conv::MPCConvCtrl>(plant));
    controllers.push_back(std::make_unique<conv::LQRConvCtrl>(plant));

    // -- Run all pairs --------------------------------------------------------
    int total = static_cast<int>(scen_files.size() * controllers.size());
    int done  = 0;
    std::cout << "Running " << total << " simulations ("
              << scen_files.size()  << " scenarios x "
              << controllers.size() << " controllers)...\n\n";

    for (auto& sf : scen_files) {
        conv::ScenarioConfig scenario;
        try {
            scenario = conv::ScenarioConfig::fromJson(sf);
        } catch (const std::exception& e) {
            std::cerr << "Skipping " << sf << ": " << e.what() << '\n';
            continue;
        }

        for (auto& ctrl : controllers) {
            try {
                conv::runSimulation(plant, scenario, *ctrl, log_dir);
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
