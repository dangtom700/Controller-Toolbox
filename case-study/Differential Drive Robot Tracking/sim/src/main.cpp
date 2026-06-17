// main.cpp - entry point for Differential Drive Robot Tracking (TEMPLATE). Target: differential_drive_robot_tracking_sim
#include "differential_drive_robot_tracking_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>

int main(int argc, char* argv[]) {
#ifndef DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR
#define DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR "."
#endif
    std::string base_dir   = (argc > 1) ? argv[1] : DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    differentialdriverobottracking::PlantParams plant;
    try {
        plant = differentialdriverobottracking::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }

    std::vector<differentialdriverobottracking::Scenario> scenarios;
    for (const auto& entry : std::filesystem::directory_iterator(scen_dir)) {
        if (entry.path().extension() == ".json")
            scenarios.push_back(differentialdriverobottracking::Scenario::fromJson(entry.path().string()));
    }

    auto controllers = differentialdriverobottracking::makeControllers(plant.Ts);
    std::cout << "Differential Drive Robot Tracking: " << controllers.size() << " controllers x "
              << scenarios.size() << " scenarios\n";

    for (const auto& scen : scenarios)
        for (const auto& nc : controllers) {
            double iae = differentialdriverobottracking::runSimulation(plant, scen, nc, log_dir);
            std::cout << "  " << scen.id << " / " << nc.name
                      << "  IAE=" << iae << '\n';
        }
    return 0;
}
