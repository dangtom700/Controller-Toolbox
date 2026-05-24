#include "simulation_runner.h"
#include "physics_plant.h"
#include "thrust_allocator.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

using json = nlohmann::json;

// ── ScenarioConfig ────────────────────────────────────────────────────────────

tug::EnvConditions ScenarioConfig::toEnvConditions() const
{
    return {
        wind_speed,
        wind_bearing_deg * M_PI / 180.0,
        current_speed,
        current_bearing_deg * M_PI / 180.0,
        Hs,
        Tp
    };
}

ScenarioConfig ScenarioConfig::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open scenario: " + path);
    json cfg = json::parse(f);

    ScenarioConfig s{};
    s.id                  = cfg.value("id", "unknown");
    s.description         = cfg.value("description", "");
    s.wind_speed          = cfg.value("wind_speed_ms", 0.0);
    s.wind_bearing_deg    = cfg.value("wind_bearing_deg", 0.0);
    s.current_speed       = cfg.value("current_speed_ms", 0.0);
    s.current_bearing_deg = cfg.value("current_bearing_deg", 0.0);
    s.Hs                  = cfg.value("Hs_m", 0.0);
    s.Tp                  = cfg.value("Tp_s", 0.0);
    s.ref_x               = cfg.value("ref_x", 0.0);
    s.ref_y               = cfg.value("ref_y", 0.0);
    s.ref_psi_deg         = cfg.value("ref_psi_deg", 0.0);
    s.duration            = cfg.value("duration_s", 0.0);
    s.seed                = cfg.value("seed", 42u);
    return s;
}

// ── Simulation loop ───────────────────────────────────────────────────────────

void runSimulation(const tug::PlantParameters& plant,
                   const ScenarioConfig&        scenario,
                   tug::ControllerBase&         controller,
                   const std::string&            log_dir)
{
    const double dt       = plant.dt;
    const double duration = (scenario.duration > 0.0) ? scenario.duration : plant.duration;
    const int    N_steps  = static_cast<int>(duration / dt);

    Eigen::Vector3d ref(scenario.ref_x,
                        scenario.ref_y,
                        scenario.ref_psi_deg * M_PI / 180.0);

    // Environment and plant
    tug::Environment  env(plant, scenario.toEnvConditions(), scenario.seed);
    tug::PhysicsPlant dynplant(plant);
    tug::ThrustAllocator allocator(plant);

    controller.reset();

    // CSV log path
    std::string log_path = log_dir + "/run_" + scenario.id
                         + "_" + controller.name() + ".csv";
    tug::TelemetryLogger logger(log_path);

    std::array<double, tug::NUM_TUGS> T_prev{};
    T_prev.fill(plant.T_min);   // initialise at minimum thrust

    auto t_wall_start = std::chrono::steady_clock::now();

    for (int k = 0; k < N_steps; ++k) {
        double t = k * dt;

        auto state = dynplant.state();
        auto nu    = dynplant.nu();
        auto eta   = dynplant.eta();

        // Environmental disturbances
        Eigen::Vector3d tau_env = env.compute(t, eta, nu);

        // Controller
        Eigen::Vector3d tau_c = controller.compute(ref, state);
        tau_c = tug::saturateTau(tau_c);

        // Thrust allocation
        auto alloc = allocator.allocate(tau_c, T_prev);
        T_prev = alloc.T;

        // Achieved force (body frame)
        Eigen::Vector3d tau_main = allocator.achieved(alloc.T);

        // Plant step
        dynplant.step(tau_main, tau_env);

        // Log
        tug::TickData td;
        td.t         = t;
        td.state     = state;
        td.tau_c     = tau_c;
        td.T         = alloc.T;
        td.ref       = ref;
        td.sat_count = alloc.sat_count;
        logger.log(td);
    }

    logger.flush();

    auto t_wall_end = std::chrono::steady_clock::now();
    double wall_ms  = std::chrono::duration<double, std::milli>(
                          t_wall_end - t_wall_start).count();

    std::cout << std::fixed << std::setprecision(1)
              << "[" << scenario.id << " | " << controller.name() << "]"
              << "  IAE_x=" << logger.IAE_x()
              << "  IAE_y=" << logger.IAE_y()
              << "  IAE_psi=" << logger.IAE_psi()
              << "  E_fuel=" << logger.E_fuel()
              << "  sat=" << logger.sat_total()
              << "  wall=" << wall_ms << " ms"
              << "  -> " << log_path << '\n';
}
