#include "simulation_runner.h"
#include "controllers.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace stewart {

RunResult runSimulation(const PlantParams&    plant,
                        const SeaStateConfig& cfg,
                        ControllerBase&        controller,
                        const std::string&     log_dir)
{
    StewartPlant dyn(plant);
    dyn.reset();
    controller.reset();

    CFDInputModel cfd(cfg, plant);

    const double Ts = plant.Ts;
    const int    N  = std::max(1, static_cast<int>(std::lround(cfg.duration_s / Ts)));

    const std::string csv_path = log_dir + "/run_" + cfg.id + "_" + controller.name() + ".csv";
    std::ofstream csv(csv_path);
    if (!csv.is_open())
        throw std::runtime_error("Cannot open log: " + csv_path);

    csv << "t,z_ref,z_p,phi_ref,phi,theta_ref,theta,psi_ref,psi,x_p,y_p,"
           "L1,L2,L3,L4,L5,L6,u1,u2,u3,u4,u5,u6,iae_cumulative\n";

    // One-sample held communication delay (decision #5) - a small FIFO of
    // pending actuator commands; output is zero until the pipeline fills.
    std::deque<Vec6> delay_queue;

    double pos_sq_sum = 0.0; // sum of squared 3D position error norm [m^2]
    double att_sq_sum = 0.0; // sum of squared 3D attitude error norm [deg^2]
    double rod_sq_sum = 0.0; // sum of squared per-rod error over all rods/steps [m^2]
    double iae         = 0.0;

    const auto t_wall_start = std::chrono::steady_clock::now();

    for (int k = 0; k < N; ++k) {
        const double t = k * Ts;
        const PoseRef ref = cfd.poseAt(t);

        Vec6 L_cmd; Mat6 J;
        dyn.geometry().ikAndJacobian(ref, L_cmd, J);

        Vec6 F_load = Vec6::Zero();
        if (cfg.equipment_load) {
            // Static equilibrium (virtual work): J^T*f + wrench_ext = 0 => f = -J^-T*wrench_ext.
            // f is the rod force needed to SUPPORT the load (sign-checked against a
            // vertical 1-rod example: gravity pulling down must be met by f=+W, pushing up).
            Vec6 wrench;
            wrench << 0.0, 0.0, -(plant.F_eq + plant.m_platform * GRAVITY), 0.0, 0.0, 0.0;
            F_load = -(J.transpose().partialPivLu().solve(wrench));
        }

        const Vec6 L  = dyn.length();
        const Vec6 dL = dyn.velocity();

        const double z_ref_global = std::abs(ref.P(2) - plant.z0_mid);

        Vec6 u_cmd = controller.compute(L_cmd, L, dL, t, z_ref_global);
        for (int i = 0; i < N_RODS; ++i)
            u_cmd(i) = std::clamp(u_cmd(i), -plant.F_rod_max, plant.F_rod_max);

        delay_queue.push_back(u_cmd);
        Vec6 u_applied = Vec6::Zero();
        if (static_cast<int>(delay_queue.size()) > plant.comm_delay_samples) {
            u_applied = delay_queue.front();
            delay_queue.pop_front();
        }

        // Achieved-pose approximation (decision #4): uses the rod error BEFORE
        // this step's plant integration, consistent with the L/dL the
        // controller just acted on.
        const Vec6 e_rod = L_cmd - L;
        const Vec6 dq_err = -(J.partialPivLu().solve(e_rod));

        const double pos_err_sq = dq_err(0)*dq_err(0) + dq_err(1)*dq_err(1) + dq_err(2)*dq_err(2);
        const double att_err_sq = (dq_err(3)*RAD2DEG)*(dq_err(3)*RAD2DEG)
                                 + (dq_err(4)*RAD2DEG)*(dq_err(4)*RAD2DEG)
                                 + (dq_err(5)*RAD2DEG)*(dq_err(5)*RAD2DEG);
        pos_sq_sum += pos_err_sq;
        att_sq_sum += att_err_sq;
        rod_sq_sum += e_rod.squaredNorm();
        iae        += e_rod.lpNorm<1>() * Ts;

        const double x_p = ref.P(0) + dq_err(0);
        const double y_p = ref.P(1) + dq_err(1);
        const double z_p = ref.P(2) + dq_err(2);
        const double phi   = (ref.rpy(0) + dq_err(3)) * RAD2DEG;
        const double theta = (ref.rpy(1) + dq_err(4)) * RAD2DEG;
        const double psi   = (ref.rpy(2) + dq_err(5)) * RAD2DEG;

        csv << std::fixed << std::setprecision(6)
            << t << ',' << ref.P(2) << ',' << z_p << ','
            << (ref.rpy(0) * RAD2DEG) << ',' << phi << ','
            << (ref.rpy(1) * RAD2DEG) << ',' << theta << ','
            << (ref.rpy(2) * RAD2DEG) << ',' << psi << ','
            << x_p << ',' << y_p << ',';
        for (int i = 0; i < N_RODS; ++i) csv << L(i) << ',';
        for (int i = 0; i < N_RODS; ++i) csv << u_applied(i) << ',';
        csv << iae << '\n';

        dyn.step(u_applied, F_load);
    }

    csv.close();

    const auto t_wall_end = std::chrono::steady_clock::now();
    const double wall_ms = std::chrono::duration<double, std::milli>(t_wall_end - t_wall_start).count();

    RunResult res;
    res.name        = controller.name();
    res.scenario_id  = cfg.id;
    res.rmse_pos_mm  = std::sqrt(pos_sq_sum / N) * 1000.0;
    res.rmse_att_deg = std::sqrt(att_sq_sum / N);
    res.rod_rms_mm   = std::sqrt(rod_sq_sum / (N * N_RODS)) * 1000.0;
    res.iae          = iae;
    res.wall_ms      = wall_ms;
    res.csv          = csv_path;
    return res;
}

} // namespace stewart
