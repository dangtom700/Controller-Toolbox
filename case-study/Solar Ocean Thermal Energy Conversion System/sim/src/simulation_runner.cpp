#include "simulation_runner.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <filesystem>

namespace sotec
{

    void runSimulation(const PlantParams &p,
                       const ScenarioConfig &sc,
                       ControllerBase &ctrl,
                       const std::string &log_dir)
    {
        const int N_steps = static_cast<int>(std::lround(sc.duration_s / p.Ts));

        SotecPlant plant(p);
        plant.reset(sc.T_h_init, sc.T_coll_init);
        ctrl.reset();

        std::filesystem::create_directories(log_dir);
        const std::string log_path = log_dir + "/run_" + sc.id + "_" + ctrl.name() + ".csv";
        std::ofstream csv(log_path);
        if (!csv.is_open())
            throw std::runtime_error("Cannot open log: " + log_path);

        // CSV header
        // t,T_h_ref,T_h,T_coll,T_c,G_b,m_dot_f_cmd,m_dot_wf_cmd,
        // W_net,P_inlet,eta_th,delta_T_super,iae_cumulative
        csv << "t,T_h_ref,T_h,T_coll,T_c,G_b,m_dot_f_cmd,m_dot_wf_cmd,"
               "W_net,P_inlet,eta_th,delta_T_super,iae_cumulative\n";

        double iae = 0.0;
        double ise = 0.0;
        double max_err = 0.0;
        double w_net_sum = 0.0;

        auto t_wall_start = std::chrono::steady_clock::now();

        for (int k = 0; k < N_steps; ++k)
        {
            const double t = k * p.Ts;
            const auto d = sc.distAt(t);
            const double T_h_r = sc.T_h_refAt(t);

            // Controller output
            CtrlOutput cmd = ctrl.compute(plant.state(), T_h_r, d.T_c, d.G);

            // ORC algebraic outputs at current state
            OrcState orc = plant.computeOrc(cmd.m_dot_f, cmd.m_dot_wf, d);

            // Metrics
            const double err = T_h_r - plant.T_h();
            iae += std::abs(err) * p.Ts;
            ise += err * err * p.Ts;
            max_err = std::max(max_err, std::abs(err));
            w_net_sum += orc.W_net * p.Ts;

            csv << std::fixed << std::setprecision(6)
                << t << ','
                << T_h_r << ','
                << plant.T_h() << ','
                << plant.T_coll() << ','
                << d.T_c << ','
                << d.G << ','
                << cmd.m_dot_f << ','
                << cmd.m_dot_wf << ','
                << orc.W_net << ','
                << orc.P_inlet << ','
                << orc.eta_th << ','
                << orc.delta_T_super << ','
                << iae << '\n';

            // Advance plant
            plant.step(cmd.m_dot_f, cmd.m_dot_wf, d);
        }

        csv.close();

        auto t_wall_end = std::chrono::steady_clock::now();
        double wall_ms = std::chrono::duration<double, std::milli>(
                             t_wall_end - t_wall_start)
                             .count();

        const double rms_err = std::sqrt(ise / sc.duration_s);
        const double W_net_kJ = w_net_sum / 1000.0;

        std::cout << std::fixed << std::setprecision(4)
                  << "[" << sc.id << " | " << ctrl.name() << "]"
                  << "  IAE=" << iae
                  << "  RMS=" << rms_err
                  << "  MaxErr=" << max_err << " ^\\circC"
                  << "  W_net_kJ=" << std::setprecision(1) << W_net_kJ
                  << "  wall=" << wall_ms << " ms"
                  << "  -> " << log_path << '\n';
    }

} // namespace sotec
