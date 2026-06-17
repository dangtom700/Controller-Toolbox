// main.cpp - entry point for Residential Building Comfort SMPC (TEMPLATE). Target: residential_building_comfort_smpc_sim
#include "residential_building_comfort_smpc_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>

int main(int argc, char* argv[]) {
#ifndef RESIDENTIAL_BUILDING_COMFORT_SMPC_SIM_SOURCE_DIR
#define RESIDENTIAL_BUILDING_COMFORT_SMPC_SIM_SOURCE_DIR "."
#endif
    std::string base_dir   = (argc > 1) ? argv[1] : RESIDENTIAL_BUILDING_COMFORT_SMPC_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    residentialbuildingcomfortsmpc::PlantParams plant;
    try {
        plant = residentialbuildingcomfortsmpc::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }

    std::vector<residentialbuildingcomfortsmpc::Scenario> scenarios;
    for (const auto& entry : std::filesystem::directory_iterator(scen_dir)) {
        if (entry.path().extension() == ".json")
            scenarios.push_back(residentialbuildingcomfortsmpc::Scenario::fromJson(entry.path().string()));
    }

    auto controllers = residentialbuildingcomfortsmpc::makeControllers(plant.Ts);
    std::cout << "Residential Building Comfort SMPC: " << controllers.size() << " controllers x "
              << scenarios.size() << " scenarios\n";

    for (const auto& scen : scenarios)
        for (const auto& nc : controllers) {
            double iae = residentialbuildingcomfortsmpc::runSimulation(plant, scen, nc, log_dir);
            std::cout << "  " << scen.id << " / " << nc.name
                      << "  IAE=" << iae << '\n';
        }
    return 0;
}
