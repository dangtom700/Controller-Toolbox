// robustness_main.cpp - robustness analysis for the Differential Drive Robot Tracking case
// study: WCET profiling, Monte Carlo (physical-parameter perturbation) and fault sweep
// (sensor / actuator / setpoint faults). Reuses the real 12-controller roster and the
// nonlinear 5-state DDMR plant through a lightweight in-memory loop (no per-step file I/O),
// mirroring the Boiler Control / Tug Boat / Airship robustness_main.cpp pattern
// (see case-study/common/RobustnessStats.h).
//
// Nominal analysis scenario: s01_lemniscate - the paper's hardest benchmark - LOADED FROM
// config/scenarios/s01_lemniscate.json rather than duplicated here, so it can never drift
// from the real scenario file.
//
// Monte Carlo perturbs the robot's physical parameters (M_total, I_A, Kf, d_com) - NOT the
// sample times or actuator limits. Controllers are always built from the ORIGINAL nominal
// PlantParams (LQR/NMPC design depends on it); only the simulated Plant uses the perturbed
// copy - same convention as Boiler Control / Airship.
//
// Sensor faults perturb the measured pose fed to the error computation; actuator faults
// perturb the commanded wheel torques; setpoint faults offset the reference position.
// Output, matching the schema/location tools/generate_report.py already reads:
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)
#include "RobustnessStats.h"
#include "controllers.h"
#include "differential_drive_robot_tracking_plant.h"
#include "simulation_runner.h"
#include "trajectory.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#ifndef DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR
#define DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace ddmr = differentialdriverobottracking;

namespace {

constexpr int    kNumMcSamples = 30;
constexpr double kMcSigma      = 0.15;   // 15% relative perturbation
constexpr double kSettleBand   = 0.05;   // [m] - same band as the simulation runner

// Inner PI wheel-velocity loop, duplicated from simulation_runner.cpp's file-local WheelPI
// because that one is deliberately internal. Kept in lockstep with it by the regression test
// (see tests/test_ddmr_regression.cpp "runner and robustness inner loops agree").
struct WheelPI {
    double int_v = 0.0, int_w = 0.0;
    void reset() { int_v = 0.0; int_w = 0.0; }
    void step(const ddmr::PlantParams& p, double v_cmd, double w_cmd,
              double v_meas, double w_meas, double& tau_R, double& tau_L) {
        const double e_v = v_cmd - v_meas;
        const double e_w = w_cmd - w_meas;
        const double trial_v = int_v + e_v * p.Ts_plant;
        const double trial_w = int_w + e_w * p.Ts_plant;
        const double lin = (p.Kp_v * e_v + p.Ki_v * trial_v) * p.R_half_axle;
        const double ang = (p.Kp_w * e_w + p.Ki_w * trial_w) * p.R_half_axle;
        const double inv_r = (p.r_wheel > 1e-9) ? 1.0 / p.r_wheel : 0.0;
        double tR = inv_r * (lin + ang);
        double tL = inv_r * (lin - ang);
        if (!(std::abs(tR) > p.tau_max || std::abs(tL) > p.tau_max)) {
            int_v = trial_v;
            int_w = trial_w;
        }
        tau_R = std::clamp(tR, -p.tau_max, p.tau_max);
        tau_L = std::clamp(tL, -p.tau_max, p.tau_max);
    }
};

// Runs one (controller, plant-copy, fault) trial through the real nonlinear plant with no
// per-step file I/O. `plant_p` initialises the simulated PLANT (possibly perturbed); `scen`
// supplies the nominal trajectory (always s01_lemniscate, unperturbed). If wcet_us is
// non-null, records the wall-clock time of every controller.compute() call.
SimSummary runOnce(const ddmr::PlantParams& plant_p,
                   const ddmr::PlantParams& design_p,
                   const ddmr::Scenario&    scen,
                   ddmr::ControllerBase&    ctrl,
                   const FaultSpec&         fault,
                   std::mt19937&            rng,
                   std::vector<double>*     wcet_us = nullptr) {
    using Clock = std::chrono::steady_clock;

    SimSummary out;
    ddmr::Plant plant(plant_p);
    ddmr::Trajectory traj(scen.pathType(), scen.a, scen.time_scale,
                          design_p.v_max, design_p.w_max);

    const ddmr::RefPoint r0 = traj.at(0.0);
    if (scen.start_on_path) plant.resetPose(r0.x, r0.y, r0.theta);
    else                    plant.resetPose(scen.x0, scen.y0, scen.theta0);

    ctrl.reset();
    WheelPI inner;

    const int    n_fast    = static_cast<int>(std::lround(scen.T_sim / design_p.Tf));
    const int    sub_steps = design_p.plantSubSteps();
    const int    slow_div  = design_p.slowDivider();
    const double Tf        = design_p.Tf;

    double sum_e2 = 0.0, sum_u = 0.0, sum_u2 = 0.0;
    double e1_prev = 0.0, e2_prev = 0.0, e3_prev = 0.0;
    bool   first = true, settled = false;
    double settle_candidate = -1.0;
    int    n = 0;

    for (int k = 0; k < n_fast; ++k) {
        const double t = k * Tf;
        ddmr::RefPoint r = traj.at(t);

        // Setpoint fault: offset the reference position.
        r.x = robust::applySetpointFault(r.x, fault, t);
        r.y = robust::applySetpointFault(r.y, fault, t);

        // Sensor fault: perturb the measured pose.
        const double xm  = robust::applySensorFault(plant.X(),     fault, t, rng);
        const double ym  = robust::applySensorFault(plant.Y(),     fault, t, rng);
        const double thm = robust::applySensorFault(plant.theta(), fault, t, rng);

        const double dx = r.x - xm, dy = r.y - ym;
        const double c = std::cos(thm), s = std::sin(thm);
        ddmr::BodyError e;
        e.e1 =  c * dx + s * dy;
        e.e2 = -s * dx + c * dy;
        e.e3 = ddmr::wrapAngle(r.theta - thm);
        if (first) { e1_prev = e.e1; e2_prev = e.e2; e3_prev = e.e3; first = false; }
        e.de1 = (e.e1 - e1_prev) / Tf;
        e.de2 = (e.e2 - e2_prev) / Tf;
        e.de3 = ddmr::wrapAngle(e.e3 - e3_prev) / Tf;
        e1_prev = e.e1; e2_prev = e.e2; e3_prev = e.e3;

        Eigen::Vector2d u;
        if (wcet_us) {
            const auto t0 = Clock::now();
            u = ctrl.compute(e, r.v, r.w);
            const auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            u = ctrl.compute(e, r.v, r.w);
        }
        const double v_cmd = std::clamp(u(0), -design_p.v_max, design_p.v_max);
        const double w_cmd = std::clamp(u(1), -design_p.w_max, design_p.w_max);

        if (k > 0 && (k % slow_div) == 0) ctrl.slowTick();

        double tau_R = 0.0, tau_L = 0.0;
        for (int i = 0; i < sub_steps; ++i) {
            inner.step(plant_p, v_cmd, w_cmd, plant.v(), plant.w(), tau_R, tau_L);
            // Actuator fault: perturb the delivered torques.
            const double aR = robust::applyActuatorFault(tau_R, fault, t);
            const double aL = robust::applyActuatorFault(tau_L, fault, t);
            plant.step(aR, aL);
        }

        const double ex = r.x - plant.X();
        const double ey = r.y - plant.Y();
        const double e_pos = std::hypot(ex, ey);

        if (!std::isfinite(e_pos) || e_pos > 1e3) { out.stable = false; break; }

        out.iae += e_pos * Tf;
        sum_e2  += e_pos * e_pos;
        out.max_abs_error = std::max(out.max_abs_error, e_pos);

        const double tau_peak = std::max(std::abs(tau_R), std::abs(tau_L));
        out.max_u = std::max(out.max_u, tau_peak);
        sum_u  += tau_peak;
        sum_u2 += tau_peak * tau_peak;

        if (e_pos < kSettleBand) {
            if (!settled) { settle_candidate = t; settled = true; }
        } else {
            settled = false;
            settle_candidate = -1.0;
        }
        ++n;
    }

    if (n > 0) {
        out.rms_error  = std::sqrt(sum_e2 / n);
        out.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(n));
    }
    out.settle_time_s = settle_candidate;
    // Overshoot is not meaningful for a closed path; report peak error above the settle band.
    out.overshoot_pct = (out.max_abs_error > kSettleBand)
                          ? (out.max_abs_error - kSettleBand) / kSettleBand * 100.0
                          : 0.0;
    return out;
}

// ---------------------------------------------------------------------------
// WCET
// ---------------------------------------------------------------------------
void runWcet(const ddmr::PlantParams& plant_p, const ddmr::Scenario& scen,
             const std::string& study_dir, const std::string& log_dir) {
    std::ofstream raw(log_dir + "/wcet_nominal.csv");
    raw << "controller,step_time_us,step_index\n";
    std::ofstream summary(study_dir + "/wcet_summary.csv");
    summary << "controller,n_samples,mean_us,median_us,p99_us,wcet_us,max_us\n";

    auto controllers = ddmr::makeControllers(plant_p);
    for (auto& c : controllers) {
        std::mt19937 rng(1234u);
        std::vector<double> us;
        us.reserve(2048);
        runOnce(plant_p, plant_p, scen, *c, FaultSpec{}, rng, &us);

        for (size_t i = 0; i < us.size(); ++i)
            raw << c->name() << ',' << us[i] << ',' << i << '\n';

        std::vector<double> sorted_us = us;
        std::sort(sorted_us.begin(), sorted_us.end());
        double mean = 0.0;
        for (double v : us) mean += v;
        mean = us.empty() ? 0.0 : mean / us.size();

        const double p50 = robust::percentile(sorted_us, 0.50);
        const double p99 = robust::percentile(sorted_us, 0.99);
        const double mx  = sorted_us.empty() ? 0.0 : sorted_us.back();

        summary << c->name() << ',' << us.size() << ',' << mean << ',' << p50 << ','
                << p99 << ',' << p99 << ',' << mx << '\n';

        std::cout << std::fixed << std::setprecision(3)
                  << "  [WCET] " << std::setw(14) << std::left << c->name() << std::right
                  << " mean=" << std::setw(8) << mean << " us  p99=" << std::setw(8) << p99
                  << " us  max=" << std::setw(8) << mx << " us\n";
    }
    std::cout << "  [WCET] wrote " << log_dir << "/wcet_nominal.csv + "
              << study_dir << "/wcet_summary.csv\n";
}

// ---------------------------------------------------------------------------
// Monte Carlo
// ---------------------------------------------------------------------------
void runMonteCarlo(const ddmr::PlantParams& nominal, const ddmr::Scenario& scen,
                   const std::string& study_dir) {
    std::ofstream csv(study_dir + "/mc_summary.csv");
    csv << "controller,sample_id,iae,rms_error,settle_time_s,overshoot_pct,max_u,"
           "energy_var,stable\n";

    auto controllers = ddmr::makeControllers(nominal);
    for (auto& c : controllers) {
        std::vector<double> iaes;
        int n_stable = 0;

        for (int s = 0; s < kNumMcSamples; ++s) {
            std::mt19937 rng(static_cast<unsigned>(9000 + s));

            // Perturb only the PLANT's physical parameters; the controller was designed on
            // the nominal set and is deliberately not rebuilt.
            ddmr::PlantParams pert = nominal;
            pert.M_total *= robust::perturbFactor(kMcSigma, rng);
            pert.I_A     *= robust::perturbFactor(kMcSigma, rng);
            pert.Kf      *= robust::perturbFactor(kMcSigma, rng);
            pert.d_com   *= robust::perturbFactor(kMcSigma, rng);
            pert.M_total = std::max(0.5, pert.M_total);
            pert.I_A     = std::max(0.01, pert.I_A);
            pert.Kf      = std::max(0.01, pert.Kf);

            const SimSummary r = runOnce(pert, nominal, scen, *c, FaultSpec{}, rng);
            csv << c->name() << ',' << s << ',' << r.iae << ',' << r.rms_error << ','
                << r.settle_time_s << ',' << r.overshoot_pct << ',' << r.max_u << ','
                << r.energy_var << ',' << (r.stable ? 1 : 0) << '\n';
            if (r.stable) { iaes.push_back(r.iae); ++n_stable; }
        }

        const MetricStats st = robust::computeStats(iaes);
        std::cout << std::fixed << std::setprecision(4)
                  << "  [MonteCarlo] " << std::setw(14) << std::left << c->name() << std::right
                  << " IAE mean=" << std::setw(10) << st.mean
                  << " p95=" << std::setw(10) << st.p95
                  << " worst=" << std::setw(10) << st.worst
                  << "  stable " << n_stable << '/' << kNumMcSamples << '\n';
    }
    std::cout << "  [MonteCarlo] wrote " << study_dir << "/mc_summary.csv\n";
}

// ---------------------------------------------------------------------------
// Fault sweep
// ---------------------------------------------------------------------------
void runFaultSweep(const ddmr::PlantParams& plant_p, const ddmr::Scenario& scen,
                   const std::string& study_dir) {
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,"
           "max_u,energy_var\n";

    struct Sweep { FaultKind kind; std::vector<double> mags; };
    const std::vector<Sweep> sweeps = {
        {FaultKind::SensorBias,    {0.02, 0.05, 0.10, 0.20}},   // [m] pose bias
        {FaultKind::SensorNoise,   {0.01, 0.02, 0.05, 0.10}},   // [m] noise sigma
        {FaultKind::ActuatorLoss,  {0.10, 0.25, 0.50, 0.75}},   // fraction of torque lost
        {FaultKind::ActuatorStuck, {0.0,  2.0,  5.0,  10.0}},   // [N m] stuck torque
        {FaultKind::SetpointStep,  {0.10, 0.25, 0.50, 1.00}},   // [m] reference offset
    };

    auto controllers = ddmr::makeControllers(plant_p);
    for (auto& c : controllers) {
        for (const auto& sw : sweeps) {
            for (double mag : sw.mags) {
                FaultSpec f;
                f.kind       = sw.kind;
                f.magnitude  = mag;
                f.fault_time = scen.T_sim * 0.25;   // engage a quarter of the way in

                std::mt19937 rng(4242u);
                const SimSummary r = runOnce(plant_p, plant_p, scen, *c, f, rng);
                csv << c->name() << ',' << robust::faultKindName(sw.kind) << ',' << mag << ','
                    << r.iae << ',' << r.rms_error << ',' << r.settle_time_s << ','
                    << r.overshoot_pct << ',' << r.max_u << ',' << r.energy_var << '\n';
            }
        }
        std::cout << "  [FaultSweep] " << c->name() << " done\n";
    }
    std::cout << "  [FaultSweep] wrote " << study_dir << "/fault_sweep.csv\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string study_dir = (argc > 1) ? argv[1]
                                             : DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR;
    const std::string log_dir   = study_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    ddmr::PlantParams plant;
    ddmr::Scenario    scen;
    try {
        plant = ddmr::PlantParams::fromJson(study_dir + "/config/plant_params.json");
        scen  = ddmr::Scenario::fromJson(study_dir +
                                         "/config/scenarios/s01_lemniscate.json");
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading configuration: " << e.what() << '\n';
        return 1;
    }

    std::cout << "Differential Drive Robot Tracking - Robustness Analysis\n"
              << "======================================================\n"
              << "Nominal scenario: " << scen.id << " (" << scen.trajectory
              << ", a=" << scen.a << " m, " << scen.T_sim << " s)\n"
              << "Logs       : " << log_dir << '\n'
              << "MC samples : " << kNumMcSamples << "  (sigma=" << kMcSigma << ")\n\n";

    runWcet(plant, scen, study_dir, log_dir);
    std::cout << '\n';
    runMonteCarlo(plant, scen, study_dir);
    std::cout << '\n';
    runFaultSweep(plant, scen, study_dir);

    std::cout << "\nRobustness analysis complete.\n";
    return 0;
}
