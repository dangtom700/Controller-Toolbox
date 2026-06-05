#include "simulation_runner.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace sub {

void runSimulation(const ScenarioConfig& sc,
                   ControllerBase& ctrl,
                   const PlantParams& plant_base,
                   const std::string& log_dir)
{
    std::filesystem::create_directories(log_dir);
    std::string csv_path = log_dir + "/run_" + sc.id + "_" + ctrl.name() + ".csv";
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "ERROR: cannot open " << csv_path << '\n';
        return;
    }

    csv << "t,psi_ref,psi,psi_err,delta_r,r,v,z_ref,z,delta_s,theta,w,q,x,y\n";

    const int N = static_cast<int>(std::lround(sc.duration_s / sc.Ts));

    // Copy plant params and set initial disturbance
    PlantParams plant = plant_base;

    SubState s = sc.initial;
    ctrl.reset();

    std::cout << "  [" << std::setw(30) << std::left << sc.id
              << " | " << std::setw(12) << ctrl.name() << "]  ";
    std::cout.flush();

    RunMetrics met;
    const double psi_final = sc.psi_ref_fn(sc.duration_s);

    for (int k = 0; k < N; ++k) {
        const double t       = k * sc.Ts;
        const double psi_ref = sc.psi_ref_fn(t);
        const double z_ref   = sc.z_ref_fn(t);
        plant.dist_Y         = sc.disturbance_fn(t);

        // Control commands
        const double delta_r = ctrl.computeHeading(psi_ref, s.psi, s.v, s.r);
        const double delta_s = ctrl.computeDepth(z_ref, s.z, s.tht, sc.Ts);

        // Log
        const double psi_err = wrapAngle(psi_ref - s.psi);
        csv << std::fixed << std::setprecision(6)
            << t       << ',' << psi_ref  << ',' << s.psi  << ',' << psi_err
            << ','     << delta_r << ',' << s.r << ',' << s.v
            << ','     << z_ref   << ',' << s.z << ',' << delta_s
            << ','     << s.tht   << ',' << s.w  << ',' << s.q
            << ','     << s.x     << ',' << s.y  << '\n';

        // Metrics
        met.iae_heading += std::abs(psi_err)       * sc.Ts;
        met.iae_depth   += std::abs(z_ref - s.z)   * sc.Ts;
        if (std::abs(psi_err) > met.max_overshoot &&
            std::abs(wrapAngle(psi_ref - psi_final)) < 0.01) {
            met.max_overshoot = std::abs(psi_err);
        }

        // Advance plant
        s = step(s, delta_r, delta_s, plant, sc.Ts);
    }

    csv.flush();

    std::cout << "IAE_psi=" << std::fixed << std::setprecision(3)
              << met.iae_heading
              << "  IAE_z=" << std::setprecision(3) << met.iae_depth
              << '\n';
}

} // namespace sub
