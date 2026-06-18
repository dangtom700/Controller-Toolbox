#include "simulation_runner.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <filesystem>

namespace smismo {

void runSimulation(const PlantParams&    p,
                   const ScenarioConfig& sc,
                   ControllerBase&       ctrl,
                   const std::string&    log_dir)
{
    const int N_steps = static_cast<int>(std::lround(sc.duration_s / p.Ts));

    SmismoPlant plant(p);
    plant.reset(sc.x_init, sc.P1_init, sc.P2_init);
    ctrl.reset();

    ValveAllocator alloc(p);
    alloc.reset();

    std::filesystem::create_directories(log_dir);
    const std::string log_path = log_dir + "/run_" + sc.id + "_" + ctrl.name() + ".csv";
    std::ofstream csv(log_path);
    if (!csv.is_open())
        throw std::runtime_error("Cannot open log: " + log_path);

    csv << "t,x_ref,x_p,v_p,P1_bar,P2_bar,u1,u2,F_ext,Q_s_lpm,energy_J,"
           "iae_cumulative\n";

    double iae     = 0.0;
    double ise     = 0.0;
    double max_err = 0.0;

    auto t_wall_start = std::chrono::steady_clock::now();

    for (int k = 0; k < N_steps; ++k) {
        const double t     = k * p.Ts;
        const double x_ref = sc.xRefAt(t);
        const double F_ext = sc.fExtAt(t);

        double u_ctrl = ctrl.compute(plant.state(), x_ref);
        if (!std::isfinite(u_ctrl)) u_ctrl = 0.0;   // NaN guard: hold valves shut

        ctrl.beforePlantStep(plant);   // DOBEnergyCtrl adjusts P_s here; no-op for others

        const ValveAllocator::Cmd cmd = alloc.allocate(u_ctrl, plant.state());

        const double err = x_ref - plant.x_L();
        iae += std::abs(err) * p.Ts;
        ise += err * err * p.Ts;
        max_err = std::max(max_err, std::abs(err));

        csv << std::fixed << std::setprecision(6)
            << t << ','
            << x_ref << ','
            << plant.x_L() << ','
            << plant.v_L() << ','
            << plant.P1() * 1e-5 << ','
            << plant.P2() * 1e-5 << ','
            << cmd.u1 << ','
            << cmd.u2 << ','
            << F_ext << ','
            << plant.Q_supply() * 60000.0 << ','   // m^3/s -> L/min
            << plant.energy() << ','
            << iae << '\n';

        plant.step(cmd.u1, cmd.u2, F_ext);
    }

    csv.close();

    auto t_wall_end = std::chrono::steady_clock::now();
    const double wall_ms = std::chrono::duration<double, std::milli>(
                               t_wall_end - t_wall_start).count();

    const double rms_err = std::sqrt(ise / sc.duration_s);

    std::cout << std::fixed << std::setprecision(5)
              << "[" << sc.id << " | " << ctrl.name() << "]"
              << "  IAE=" << iae
              << "  RMS=" << rms_err
              << "  MaxErr=" << max_err << " m"
              << "  E_kJ=" << std::setprecision(2) << plant.energy() / 1000.0
              << "  wall=" << std::setprecision(0) << wall_ms << " ms"
              << "  -> " << log_path << '\n';
}

} // namespace smismo
