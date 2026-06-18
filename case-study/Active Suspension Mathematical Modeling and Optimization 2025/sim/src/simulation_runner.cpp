#include "simulation_runner.h"
#include "susp_plant.h"
#include "road_profile.h"
#include "controllers.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace susp {

void runSimulation(const PlantParams&    plant,
                   const ScenarioConfig& scenario,
                   ControllerBase&       controller,
                   const std::string&    log_dir)
{
    const double dt       = plant.Ts;
    const double duration = scenario.duration_s > 0.0 ? scenario.duration_s : plant.duration;
    const int    N_steps  = static_cast<int>(std::lround(duration / dt));

    QuarterCarPlant dyn(plant);
    RoadProfile     road(scenario);
    controller.reset();

    // CSV output
    std::string log_path = log_dir + "/run_" + scenario.id
                         + "_" + controller.name() + ".csv";
    std::ofstream csv(log_path);
    if (!csv.is_open())
        throw std::runtime_error("Cannot open log: " + log_path);
    csv << "t,z_s,dz_s,z_u,dz_u,z_r,F_act,body_acc,susp_defl,tyre_defl\n";

    // Metrics accumulators
    double iae_zs     = 0.0;   // integral |z_s| dt
    double sum_acc2   = 0.0;   // sum of body_acc^2 (for RMS)
    double sum_force2 = 0.0;   // sum of F_act^2 (for RMS)
    double max_defl   = 0.0;   // max |suspension deflection|
    int    sat_count  = 0;     // actuator saturation events

    auto t_wall_start = std::chrono::steady_clock::now();

    for (int k = 0; k < N_steps; ++k) {
        const double t    = k * dt;
        const auto&  x    = dyn.state();
        const double z_r  = road.compute(t);

        // Controller
        double F_raw = controller.compute(x, z_r);

        // Saturation
        double F_act = std::clamp(F_raw, -plant.F_max, plant.F_max);
        if (std::abs(F_raw) > plant.F_max) ++sat_count;

        // Diagnostics before step
        const double body_acc   = dyn.bodyAccel(F_act, z_r);
        const double susp_defl  = dyn.suspDeflection();
        const double tyre_defl  = dyn.tyreDeflection(z_r);

        // Log
        csv << std::fixed << std::setprecision(6)
            << t         << ','
            << x(0)      << ','
            << x(1)      << ','
            << x(2)      << ','
            << x(3)      << ','
            << z_r       << ','
            << F_act     << ','
            << body_acc  << ','
            << susp_defl << ','
            << tyre_defl << '\n';

        // Accumulate metrics
        iae_zs     += std::abs(x(0)) * dt;
        sum_acc2   += body_acc * body_acc;
        sum_force2 += F_act * F_act;
        max_defl    = std::max(max_defl, std::abs(susp_defl));

        // Plant step
        dyn.step(F_act, z_r);
    }

    csv.close();

    auto t_wall_end = std::chrono::steady_clock::now();
    double wall_ms  = std::chrono::duration<double, std::milli>(
                          t_wall_end - t_wall_start).count();

    const double rms_acc   = std::sqrt(sum_acc2   / N_steps);
    const double rms_force = std::sqrt(sum_force2 / N_steps);

    std::cout << std::fixed << std::setprecision(4)
              << "[" << scenario.id << " | " << controller.name() << "]"
              << "  IAE_zs="    << iae_zs
              << "  RMS_acc="   << rms_acc
              << "  MaxDefl="   << max_defl
              << "  RMS_force=" << rms_force
              << "  sat="       << sat_count
              << "  wall="      << std::setprecision(1) << wall_ms << " ms"
              << "  -> "        << log_path << '\n';
}

} // namespace susp
