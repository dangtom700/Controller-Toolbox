// main.cpp - entry point for Dual-Arm IAUV Motion Planning (TEMPLATE). Target: dual_arm_iauv_motion_planning_sim
#include "dual_arm_iauv_motion_planning_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>

int main(int argc, char* argv[]) {
#ifndef DUAL_ARM_IAUV_MOTION_PLANNING_SIM_SOURCE_DIR
#define DUAL_ARM_IAUV_MOTION_PLANNING_SIM_SOURCE_DIR "."
#endif
    std::string base_dir   = (argc > 1) ? argv[1] : DUAL_ARM_IAUV_MOTION_PLANNING_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    dualarmiauvmotionplanning::PlantParams plant;
    try {
        plant = dualarmiauvmotionplanning::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }

    std::vector<dualarmiauvmotionplanning::Scenario> scenarios;
    for (const auto& entry : std::filesystem::directory_iterator(scen_dir)) {
        if (entry.path().extension() == ".json")
            scenarios.push_back(dualarmiauvmotionplanning::Scenario::fromJson(entry.path().string()));
    }

    auto controllers = dualarmiauvmotionplanning::makeControllers(plant.Ts);
    std::cout << "Dual-Arm IAUV Motion Planning: " << controllers.size() << " controllers x "
              << scenarios.size() << " scenarios\n";

    for (const auto& scen : scenarios)
        for (const auto& nc : controllers) {
            double iae = dualarmiauvmotionplanning::runSimulation(plant, scen, nc, log_dir);
            std::cout << "  " << scen.id << " / " << nc.name
                      << "  IAE=" << iae << '\n';
        }
    return 0;
}
