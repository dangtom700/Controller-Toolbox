#include "stewart_plant.h"
#include "cfd_input_model.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
#ifndef STEWART_SIM_SOURCE_DIR
#define STEWART_SIM_SOURCE_DIR "."
#endif
    std::string base_dir    = (argc > 1) ? argv[1] : STEWART_SIM_SOURCE_DIR;
    std::string plant_json  = base_dir + "/config/plant_params.json";
    std::string matrix_json = base_dir + "/config/scenarios/sea_state_matrix.json";
    std::string log_dir     = base_dir + "/logs";

    std::filesystem::create_directories(log_dir);

    std::cout << "Loading plant parameters from: " << plant_json << '\n';
    stewart::PlantParams plant;
    try {
        plant = stewart::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }
    std::cout << "Stewart platform parameters:\n"
              << "  Ra=" << plant.Ra << " m  Rb=" << plant.Rb << " m  z0_mid=" << plant.z0_mid << " m\n"
              << "  m_rod=" << plant.m_rod << " kg  F_rod_max=" << plant.F_rod_max << " N\n"
              << "  F_eq=" << plant.F_eq << " N  m_platform=" << plant.m_platform << " kg\n"
              << "  Ts=" << plant.Ts << " s\n\n";

    auto matrix = stewart::buildSeaStateMatrix(matrix_json, plant);
    if (matrix.empty()) {
        std::cerr << "ERROR: sea state matrix is empty\n";
        return 1;
    }

    std::vector<std::unique_ptr<stewart::ControllerBase>> controllers;
    controllers.push_back(std::make_unique<stewart::PIDStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::FuzzyPIDStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::ADRCStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::SMCStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::LQRStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::MPCStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::MRACStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::L1AdaptiveStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::GainScheduledStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::TubeMPCStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::NeuralPIDStewartCtrl>(plant));
    controllers.push_back(std::make_unique<stewart::ScenarioMPCStewartCtrl>(plant));

    static constexpr int N_CONTROLLERS = 12;
    if (static_cast<int>(controllers.size()) != N_CONTROLLERS) {
        std::cerr << "ERROR: Expected " << N_CONTROLLERS
                  << " controllers, got " << controllers.size() << '\n';
        return 1;
    }

    const int total = static_cast<int>(matrix.size() * controllers.size());
    int done = 0;

    std::cout << "Running " << total << " simulations ("
              << matrix.size() << " Douglas sea-state configs x "
              << controllers.size() << " controllers)...\n";
    std::cout << "(10 states x 3 directions x 2 swell = "
              << matrix.size() << " configs - see README for the matrix derivation)\n\n";

    std::vector<stewart::RunResult> results;
    results.reserve(static_cast<size_t>(total));

    for (const auto& cfg : matrix) {
        for (auto& ctrl : controllers) {
            try {
                results.push_back(stewart::runSimulation(plant, cfg, *ctrl, log_dir));
            } catch (const std::exception& e) {
                std::cerr << "ERROR [" << cfg.id << " | " << ctrl->name() << "]: " << e.what() << '\n';
            }
            ++done;
            if (done % 60 == 0 || done == total)
                std::cout << "  [" << done << '/' << total << "] completed\n";
        }
    }

    std::cout << "\nDone. " << results.size() << '/' << total
              << " runs completed. Logs: " << log_dir << '\n';

    std::cout << "\n  " << std::left << std::setw(24) << "Scenario" << std::setw(14) << "Controller"
              << std::right << std::setw(12) << "PosRMSE[mm]" << std::setw(13) << "AttRMSE[deg]"
              << std::setw(12) << "RodRMS[mm]" << std::setw(10) << "IAE" << '\n';
    std::cout << "  " << std::string(85, '-') << '\n';
    for (const auto& r : results) {
        std::cout << "  " << std::left << std::setw(24) << r.scenario_id << std::setw(14) << r.name
                  << std::right << std::fixed << std::setprecision(3)
                  << std::setw(12) << r.rmse_pos_mm << std::setw(13) << r.rmse_att_deg
                  << std::setw(12) << r.rod_rms_mm
                  << std::setprecision(2) << std::setw(10) << r.iae << '\n';
    }

    return (results.size() == static_cast<size_t>(total)) ? 0 : 1;
}
