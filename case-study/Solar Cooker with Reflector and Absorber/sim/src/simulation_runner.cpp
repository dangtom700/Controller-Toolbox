#include "simulation_runner.h"
#include "solar_cooker_plant.h"
#include "controllers.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace cooker
{

    void runSimulation(const PlantParams &plant,
                       const ScenarioConfig &scenario,
                       ControllerBase &controller,
                       const std::string &log_dir)
    {
        const double dt = plant.Ts;
        const double dur = scenario.duration_s > 0.0 ? scenario.duration_s : plant.duration;
        const int N_steps = static_cast<int>(std::lround(dur / dt));

        SolarCookerPlant dyn(plant);
        dyn.reset(scenario.T_abs_init, scenario.T_pot_init);
        controller.reset();

        // CSV output
        std::string log_path = log_dir + "/run_" + scenario.id + "_" + controller.name() + ".csv";
        std::ofstream csv(log_path);
        if (!csv.is_open())
            throw std::runtime_error("Cannot open log: " + log_path);

        csv << "t,T_ref,T_pot,T_abs,G_b,T_amb,V_wind,phi_pcm,f_shade,Q_absorbed,iae_cumulative\n";

        double iae = 0.0;
        int sat = 0;

        auto t_wall_start = std::chrono::steady_clock::now();

        for (int k = 0; k < N_steps; ++k)
        {
            const double t = k * dt;
            const double T_ref = scenario.T_ref;
            const Disturbance d = scenario.distAt(t);

            const Eigen::Vector2d &x = dyn.state();

            // Safety: force full shade if absorber approaches hard limit
            double f_raw = controller.compute(x, T_ref);
            if (x(0) >= plant.T_abs_max - 1.0)
                f_raw = 1.0;
            const double f_shade = std::clamp(f_raw, 0.0, 1.0);

            if (f_raw <= 0.0 + 1e-9 || f_raw >= 1.0 - 1e-9)
                ++sat;

            const double Q_abs = dyn.Q_absorbed(f_shade, d.G);
            const double phi_pcm = dyn.phiPCM();
            const double err = T_ref - x(1);

            iae += std::abs(err) * dt;

            csv << std::fixed << std::setprecision(6)
                << t << ','
                << T_ref << ','
                << x(1) << ','
                << x(0) << ','
                << d.G << ','
                << d.T_amb << ','
                << d.V_wind << ','
                << phi_pcm << ','
                << f_shade << ','
                << Q_abs << ','
                << iae << '\n';

            dyn.step(f_shade, d);
        }

        csv.close();

        auto t_wall_end = std::chrono::steady_clock::now();
        const double wall_ms = std::chrono::duration<double, std::milli>(
                                   t_wall_end - t_wall_start)
                                   .count();

        // Print final T_pot as a summary metric
        const double T_pot_final = dyn.state()(1);

        std::cout << std::fixed << std::setprecision(4)
                  << "[" << scenario.id << " | " << controller.name() << "]"
                  << "  IAE=" << iae
                  << "  T_pot_final=" << T_pot_final << " ^\\circC"
                  << "  sat=" << sat
                  << "  wall=" << std::setprecision(1) << wall_ms << " ms"
                  << "  -> " << log_path << '\n';
    }

} // namespace cooker
