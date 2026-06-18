// main.cpp - entry point for Bouyancy-Driven Airship in Vertical Plane (TEMPLATE). Target: bouyancy_driven_airship_in_vertical_plan_sim
#include "bouyancy_driven_airship_in_vertical_plan_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>

int main(int argc, char* argv[]) {
#ifndef BOUYANCY_DRIVEN_AIRSHIP_IN_VERTICAL_PLAN_SIM_SOURCE_DIR
#define BOUYANCY_DRIVEN_AIRSHIP_IN_VERTICAL_PLAN_SIM_SOURCE_DIR "."
#endif
    std::string base_dir   = (argc > 1) ? argv[1] : BOUYANCY_DRIVEN_AIRSHIP_IN_VERTICAL_PLAN_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    bouyancydrivenairshipinverticalplan::PlantParams plant;
    try {
        plant = bouyancydrivenairshipinverticalplan::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }

    std::vector<bouyancydrivenairshipinverticalplan::Scenario> scenarios;
    for (const auto& entry : std::filesystem::directory_iterator(scen_dir)) {
        if (entry.path().extension() == ".json")
            scenarios.push_back(bouyancydrivenairshipinverticalplan::Scenario::fromJson(entry.path().string()));
    }

    auto controllers = bouyancydrivenairshipinverticalplan::makeControllers(plant.Ts);
    std::cout << "Bouyancy-Driven Airship in Vertical Plane: " << controllers.size() << " controllers x "
              << scenarios.size() << " scenarios\n";

    for (const auto& scen : scenarios)
        for (const auto& nc : controllers) {
            double iae = bouyancydrivenairshipinverticalplan::runSimulation(plant, scen, nc, log_dir);
            std::cout << "  " << scen.id << " / " << nc.name
                      << "  IAE=" << iae << '\n';
        }
    return 0;
}
