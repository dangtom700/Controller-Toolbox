#include "submarine_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#ifndef SUB_SIM_SOURCE_DIR
#define SUB_SIM_SOURCE_DIR "."
#endif

// ---------------------------------------------------------------------------
// Zig-zag helper: switches heading reference when heading crosses +/-band [rad]
// ---------------------------------------------------------------------------
struct ZigZagRef {
    double band;          // switch angle [rad]
    mutable double target;
    explicit ZigZagRef(double b) : band(b), target(b) {}
    double operator()(double /*t*/, double psi) const {
        if (target > 0 && psi >=  band - 0.005) target = -band;
        if (target < 0 && psi <= -band + 0.005) target =  band;
        return target;
    }
};

int main(int argc, char* argv[])
{
    std::string base_dir = (argc > 1) ? argv[1] : SUB_SIM_SOURCE_DIR;
    std::string log_dir  = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    // -- Plant (shared across all scenarios) ---------------------------------
    sub::PlantParams plant;   // default MARIN BB2 model-scale parameters

    constexpr double Ts = 0.05;  // 20 Hz, model-scale

    // -- Scenarios -----------------------------------------------------------
    // ZigZag ref objects (stateful - not reusable across controllers; see loop)
    using sub::ScenarioConfig;
    using sub::SubState;

    SubState s0_surface;   // surface: z=0
    SubState s0_depth10;   s0_depth10.z = 10.0;

    std::vector<ScenarioConfig> scenarios = {
        // s01 - 20^\circ port turn (ψ: 0 -> -0.349 rad), constant depth 0 m
        {
            "s01_turn20", "20 deg port heading change",
            300.0, Ts, s0_surface,
            [](double) { return -0.349; },           // ψ_ref = -20^\circ
            [](double) { return 0.0; },              // z_ref = 0 m
            [](double) { return 0.0; }               // no disturbance
        },
        // s02 - 90^\circ port turn (ψ: 0 -> -1.571 rad), constant depth 0 m
        {
            "s02_turn90", "90 deg port heading change",
            400.0, Ts, s0_surface,
            [](double) { return -1.571; },
            [](double) { return 0.0; },
            [](double) { return 0.0; }
        },
        // s03 - 10/10 zig-zag (+/-0.175 rad = +/-10^\circ), 300 s
        // Reference toggles when heading reaches +/-band; state per controller.
        {
            "s03_zigzag10", "10/10 zig-zag heading",
            300.0, Ts, s0_surface,
            // psi_ref_fn is time-only for simplicity; zig-zag switching
            // is approximated as: +10^\circ for first 30 s, then alternate
            [](double t) {
                int cycle = static_cast<int>(t / 30.0);
                return (cycle % 2 == 0) ? 0.175 : -0.175;
            },
            [](double) { return 0.0; },
            [](double) { return 0.0; }
        },
        // s04 - Course keeping with 150 N lateral ocean-current disturbance
        {
            "s04_disturbance", "Course keeping, 150 N current disturbance",
            300.0, Ts, s0_surface,
            [](double) { return 0.0; },              // hold heading = 0
            [](double) { return 0.0; },
            [](double t) { return (t >= 50.0) ? 150.0 : 0.0; }  // current on at t=50s
        },
        // s05 - Depth change 0 -> 20 m while holding heading = 0
        {
            "s05_depth_dive", "Depth change 0 to 20 m, heading hold",
            400.0, Ts, s0_surface,
            [](double) { return 0.0; },
            [](double t) { return (t >= 30.0) ? 20.0 : 0.0; },  // dive at t=30 s
            [](double) { return 0.0; }
        },
    };

    // -- Controllers ---------------------------------------------------------
    std::vector<std::unique_ptr<sub::ControllerBase>> controllers;
    controllers.push_back(std::make_unique<sub::PIDZNCtrl>(Ts));
    controllers.push_back(std::make_unique<sub::PIDConsCtrl>(Ts));
    controllers.push_back(std::make_unique<sub::MPCHeadingCtrl>(plant, Ts));
    controllers.push_back(std::make_unique<sub::LQRHeadingCtrl>(plant, Ts));
    controllers.push_back(std::make_unique<sub::ADRCHeadingCtrl>(Ts));
    controllers.push_back(std::make_unique<sub::SMCHeadingCtrl>(Ts));
    controllers.push_back(std::make_unique<sub::MRACHeadingCtrl>(Ts));
    controllers.push_back(std::make_unique<sub::L1HeadingCtrl>(Ts));
    controllers.push_back(std::make_unique<sub::GainSchedHeadingCtrl>(Ts));
    controllers.push_back(std::make_unique<sub::SmithHeadingCtrl>(Ts));

    const int total = static_cast<int>(scenarios.size() * controllers.size());

    std::cout << "MARIN BB2 Submarine Autopilot Simulation\n";
    std::cout << "=========================================\n";
    std::cout << "Plant : MARIN BB2 (model scale, L=" << plant.L
              << " m, U=" << plant.U << " m/s)\n";
    std::cout << "Logs  : " << log_dir << "\n";
    std::cout << "Ts    : " << Ts << " s\n";
    std::cout << "Scenarios  : " << scenarios.size() << "\n";
    std::cout << "Controllers: " << controllers.size() << "\n";
    std::cout << "Total runs : " << total << "\n\n";

    int done = 0;
    for (const auto& sc : scenarios) {
        std::cout << "=== Scenario: " << sc.id << " | " << sc.description << " ===\n";
        for (auto& c : controllers) {
            try {
                runSimulation(sc, *c, plant, log_dir);
            } catch (const std::exception& ex) {
                std::cerr << "\n  ERROR [" << sc.id << " | "
                          << c->name() << "]: " << ex.what() << '\n';
            }
            ++done;
        }
        std::cout << '\n';
    }

    std::cout << "Done. " << done << '/' << total
              << " runs completed. Results in: " << log_dir << '\n';
    return 0;
}
