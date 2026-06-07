#include "simulation_runner.h"
#include "buck_boost_plant.h"
#include "input_profile.h"
#include "controllers.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace conv {

void runSimulation(const PlantParams&    plant,
                   const ScenarioConfig& scenario,
                   ControllerBase&       controller,
                   const std::string&    log_dir)
{
    const double dt       = plant.Ts;
    const double duration = scenario.duration_s > 0.0 ? scenario.duration_s : plant.duration;
    const int    N_steps  = static_cast<int>(std::lround(duration / dt));

    BuckBoostPlant dyn(plant);
    controller.reset();

    // --- CSV output ---
    std::string log_path = log_dir + "/run_" + scenario.id
                         + "_" + controller.name() + ".csv";
    std::ofstream csv(log_path);
    if (!csv.is_open())
        throw std::runtime_error("Cannot open log: " + log_path);
    csv << "t,v_in,v_ref,v_out,i_L,d,mode,error\n";

    // --- Metrics accumulators ---
    double iae     = 0.0;   // integral |error| dt
    double ise     = 0.0;   // integral error^2 dt
    double max_err = 0.0;   // peak absolute voltage error [V]
    int    sat_d   = 0;     // duty-cycle at limit (0 or 1) events

    // Mode detection (hysteresis state)
    ConvMode mode = ConvMode::BUCK;

    auto t_wall_start = std::chrono::steady_clock::now();

    for (int k = 0; k < N_steps; ++k) {
        const double t     = k * dt;
        const double V_in  = scenario.v_in;
        const double v_ref = scenario.vRefAt(t);

        // Update mode (hysteresis)
        if (v_ref > V_in + plant.hysteresis)
            mode = ConvMode::BOOST;
        else if (v_ref < V_in - plant.hysteresis)
            mode = ConvMode::BUCK;

        const Eigen::Vector2d& x   = dyn.state();
        const double           v_C = x(1);
        const double           i_L = x(0);

        // Controller -> duty cycle
        double d_raw = controller.compute(x, v_ref, V_in);
        double d     = std::clamp(d_raw, plant.d_min, plant.d_max);
        if (d_raw <= plant.d_min + 1e-9 || d_raw >= plant.d_max - 1e-9) ++sat_d;

        // CSV row
        const double err = v_ref - v_C;
        csv << std::fixed << std::setprecision(8)
            << t    << ','
            << V_in << ','
            << v_ref << ','
            << v_C  << ','
            << i_L  << ','
            << d    << ','
            << (mode == ConvMode::BOOST ? 1 : 0) << ','
            << err  << '\n';

        // Accumulate metrics
        iae     += std::abs(err) * dt;
        ise     += err * err * dt;
        max_err  = std::max(max_err, std::abs(err));

        // Plant step
        dyn.step(d, mode, V_in);
    }

    csv.close();

    auto t_wall_end = std::chrono::steady_clock::now();
    double wall_ms  = std::chrono::duration<double, std::milli>(
                          t_wall_end - t_wall_start).count();

    const double rms_err = std::sqrt(ise / duration);

    std::cout << std::fixed << std::setprecision(4)
              << "[" << scenario.id << " | " << controller.name() << "]"
              << "  IAE="     << iae
              << "  RMS_err=" << rms_err
              << "  MaxErr="  << max_err   << " V"
              << "  sat="     << sat_d
              << "  wall="    << std::setprecision(1) << wall_ms << " ms"
              << "  -> "      << log_path << '\n';
}

} // namespace conv
