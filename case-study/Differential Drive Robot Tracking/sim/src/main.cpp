// main.cpp - entry point for the Differential Drive Robot Tracking case study.
// Target: differential_drive_robot_tracking_sim. 12 controllers x 5 scenarios = 60 runs.
//
// Reference: Xu, Maghsoudniazi & Maghsoudniazi, "Integrating Lyapunov based backstepping and
// neuro fuzzy logic with sliding mode control for precise trajectory tracking of differential
// drive robots", Sci. Rep. 16:11961 (2026), doi:10.1038/s41598-026-39667-1.
#include "controllers.h"
#include "differential_drive_robot_tracking_plant.h"
#include "simulation_runner.h"
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
#ifndef DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR
#define DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR "."
#endif
    using namespace differentialdriverobottracking;

    std::string base_dir   = (argc > 1) ? argv[1] : DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_dir   = base_dir + "/config/scenarios";
    std::string log_dir    = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    std::cout << "Loading plant parameters from: " << plant_json << '\n';
    PlantParams plant;
    try {
        plant = PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }

    std::cout << "DDMR: M=" << plant.M_total << " kg  r=" << plant.r_wheel
              << " m  R=" << plant.R_half_axle << " m  d=" << plant.d_com
              << " m  Kf=" << plant.Kf << " N.m.s/rad  tau_max=" << plant.tau_max << " N.m\n"
              << "Rates: plant/inner-PI " << plant.Ts_plant << " s | fast loop " << plant.Tf
              << " s | slow loop " << plant.Ts_slow << " s (eps = "
              << plant.Tf / plant.Ts_slow << ")\n\n";

    std::vector<std::string> scen_files;
    for (const auto& entry : std::filesystem::directory_iterator(scen_dir))
        if (entry.path().extension() == ".json")
            scen_files.push_back(entry.path().string());
    std::sort(scen_files.begin(), scen_files.end());

    if (scen_files.empty()) {
        std::cerr << "No scenario JSON files found in: " << scen_dir << '\n';
        return 1;
    }

    auto controllers = makeControllers(plant);
    static constexpr int N_CONTROLLERS = 12;
    if (static_cast<int>(controllers.size()) != N_CONTROLLERS) {
        std::cerr << "ERROR: expected " << N_CONTROLLERS << " controllers, got "
                  << controllers.size() << '\n';
        return 1;
    }

    const int total = static_cast<int>(scen_files.size() * controllers.size());
    int done = 0;
    std::cout << "Running " << total << " simulations (" << scen_files.size()
              << " scenarios x " << controllers.size() << " controllers)...\n\n";

    for (const auto& sf : scen_files) {
        Scenario scen;
        try {
            scen = Scenario::fromJson(sf);
        } catch (const std::exception& e) {
            std::cerr << "Skipping " << sf << ": " << e.what() << '\n';
            continue;
        }

        std::cout << "-- " << scen.id << " (" << scen.trajectory << ", a=" << scen.a
                  << " m, " << scen.T_sim << " s) --\n";

        for (auto& c : controllers) {
            try {
                const RunMetrics m = runSimulation(plant, scen, *c, log_dir);
                std::cout << std::fixed << std::setprecision(4)
                          << "  " << std::setw(14) << std::left << c->name() << std::right
                          << "  ISE=" << std::setw(9) << m.ise
                          << "  IAE=" << std::setw(9) << m.iae
                          << "  ITAE=" << std::setw(10) << m.itae
                          << "  final_err=" << std::setw(8) << m.final_err << " m"
                          << "  mean_tau=" << std::setw(8) << m.mean_torque << " N.m"
                          << "  Ks=" << std::setw(7) << m.Ks_final
                          << "  settle=" << std::setw(7) << m.settle_time << " s\n";
            } catch (const std::exception& e) {
                std::cerr << "ERROR [" << scen.id << " | " << c->name() << "]: "
                          << e.what() << '\n';
            }
            ++done;
        }
        std::cout << '\n';
    }

    std::cout << "Done. " << done << '/' << total << " runs completed. Results in: "
              << log_dir << '\n';
    return 0;
}
