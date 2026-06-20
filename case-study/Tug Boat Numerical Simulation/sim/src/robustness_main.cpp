#include "plant_parameters.h"
#include "physics_plant.h"
#include "environment.h"
#include "thrust_allocator.h"
#include "controllers.h"
#include "RobustnessStats.h"
#include <algorithm>
#include <array>
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

// Robustness analysis for the Tug Boat case study: WCET profiling, Monte
// Carlo (plant-parameter perturbation), and fault sweep (sensor/actuator/
// setpoint faults). Reuses the real nonlinear PhysicsPlant + Environment +
// ThrustAllocator + every controller in the roster via a lightweight
// in-memory simulation loop with no per-step file I/O.
//
// MIMO fault decision: fault the surge axis (index 0) only, holding sway/yaw
// fault-free, to keep trial count comparable to the SISO studies.
// Actuator fault applies to the controller's RAW tau_c output, before
// saturation/allocation (not post-allocation per-thruster), so the fault
// model stays comparable across controllers and independent of the
// allocator's pseudo-inverse logic.
//
// Output, matching the schema/location tools/generate_report.py already reads
// (case-study/<Study>/{mc_summary,fault_sweep,wcet_summary}.csv) so the report
// pipeline picks these up with no changes:
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)

#ifndef TUG_SIM_SOURCE_DIR
#define TUG_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace {

// Truncated relative to the default 5400s scenario length (matches S2's own
// 300s zero-input stability-test duration): keeps cost down across 30 MC
// samples x 18 controllers (several solve a QP/MPC problem per step).
constexpr double kAnalysisDuration = 300.0;  // [s]
constexpr int    kNumMcSamples     = 30;
constexpr double kMcSigma          = 0.15;   // 15% relative perturbation

// Runs one (controller, plant, fault) trial through the real nonlinear
// barge-tugboat plant with no per-step file I/O. If wcet_us is non-null,
// records the wall-clock time of every controller.compute() call.
SimSummary runOnce(const tug::PlantParameters& plant_params,
                    tug::ControllerBase&        ctrl,
                    const FaultSpec&            fault,
                    std::mt19937&               rng,
                    std::vector<double>*        wcet_us = nullptr)
{
    using Clock = std::chrono::steady_clock;

    const double dt      = plant_params.dt;
    const int    N_steps = static_cast<int>(std::lround(kAnalysisDuration / dt));

    // Nominal disturbance, matching S2 (paper baseline, Li et al. Table 5).
    const tug::EnvConditions cond{10.0, 90.0 * M_PI / 180.0, 0.5144, 90.0 * M_PI / 180.0, 2.0, 10.0};
    tug::Environment      env(plant_params, cond, 42u);
    tug::PhysicsPlant      dynplant(plant_params);
    tug::ThrustAllocator   allocator(plant_params);
    ctrl.reset();

    const Eigen::Vector3d ref_nom(0.0, 0.0, 0.0);
    std::array<double, tug::NUM_TUGS> T_prev{};
    T_prev.fill(plant_params.T_min);

    SimSummary out;
    double sum_e2      = 0.0;
    double sum_u       = 0.0;
    double sum_u2      = 0.0;
    double e0_abs      = -1.0;
    int    in_band_run = 0;
    bool   settled     = false;
    double max_y = -1e9, min_y = 1e9, final_ref = ref_nom(0);
    constexpr double kSettleBand       = 0.02;
    constexpr int    kSettleHysteresis = 10;

    for (int k = 0; k < N_steps; ++k) {
        const double t = k * dt;

        const auto& state = dynplant.state();
        const Eigen::Vector3d eta = dynplant.eta();
        const Eigen::Vector3d nu  = dynplant.nu();

        const Eigen::Vector3d tau_env = env.compute(t, eta, nu);

        Eigen::Vector3d ref = ref_nom;
        ref(0) = robust::applySetpointFault(ref_nom(0), fault, t);
        final_ref = ref(0);

        // Sensor fault: perturb only the measured world-frame x position.
        Eigen::Matrix<double, 6, 1> state_meas = state;
        state_meas(0) = robust::applySensorFault(state(0), fault, t, rng);

        Eigen::Vector3d tau_c;
        if (wcet_us) {
            auto t0 = Clock::now();
            tau_c   = ctrl.compute(ref, state_meas);
            auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            tau_c = ctrl.compute(ref, state_meas);
        }

        // Actuator fault on the surge command, before saturation/allocation
        // (same order as simulation_runner.cpp).
        tau_c(0) = robust::applyActuatorFault(tau_c(0), fault, t);
        tau_c    = tug::saturateTau(tau_c);

        auto alloc = allocator.allocate(tau_c, T_prev);
        T_prev     = alloc.T;
        const Eigen::Vector3d tau_main = allocator.achieved(alloc.T);
        ctrl.notifyApplied(tau_main);

        const double error  = ref(0) - state(0);
        out.iae             += std::abs(error) * dt;
        sum_e2               += error * error;
        out.max_abs_error     = std::max(out.max_abs_error, std::abs(error));
        out.max_u              = std::max(out.max_u, std::max({std::abs(tau_main(0)), std::abs(tau_main(1)), std::abs(tau_main(2))}));
        sum_u                   += tau_main(0);
        sum_u2                   += tau_main(0) * tau_main(0);
        max_y = std::max(max_y, state(0));
        min_y = std::min(min_y, state(0));

        bool finite_state = true;
        for (int i = 0; i < 6; ++i) finite_state = finite_state && std::isfinite(state(i));
        if (!finite_state || !std::isfinite(tau_main(0)) || std::abs(state(0)) > 2000.0 || std::abs(state(1)) > 2000.0) {
            out.stable = false;
        }

        if (k == 0) e0_abs = std::abs(error);
        if (!settled) {
            const double band = kSettleBand * std::max(e0_abs, 1e-9);
            if (std::abs(error) <= band) {
                if (++in_band_run >= kSettleHysteresis) { out.settle_time_s = t; settled = true; }
            } else {
                in_band_run = 0;
            }
        }

        dynplant.step(tau_main, tau_env);
    }

    out.rms_error  = std::sqrt(sum_e2 / N_steps);
    out.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(N_steps));
    if (!out.stable) out.iae = std::max(out.iae, 1.0e6);

    if (std::abs(final_ref) > 1e-9 && max_y > final_ref) {
        out.overshoot_pct = (max_y - final_ref) / std::abs(final_ref) * 100.0;
    }
    return out;
}

std::vector<std::unique_ptr<tug::ControllerBase>> buildControllers(const tug::PlantParameters& plant) {
    std::vector<std::unique_ptr<tug::ControllerBase>> v;
    v.push_back(std::make_unique<tug::PIDController>(plant));
    v.push_back(std::make_unique<tug::KFPIDController>(plant));
    v.push_back(std::make_unique<tug::SMCController>(plant));
    v.push_back(std::make_unique<tug::MPCController>(plant));
    v.push_back(std::make_unique<tug::ESCController>(plant));
    v.push_back(std::make_unique<tug::FuzzyPIDController>(plant));
    v.push_back(std::make_unique<tug::FuzzySupervised_MPC>(plant));
    v.push_back(std::make_unique<tug::ADRCTugCtrl>(plant));
    v.push_back(std::make_unique<tug::RepetitiveTugCtrl>(plant));
    v.push_back(std::make_unique<tug::LQRTugCtrl>(plant));
    v.push_back(std::make_unique<tug::LQGTugCtrl>(plant));
    v.push_back(std::make_unique<tug::TubeMPCTugCtrl>(plant));
    v.push_back(std::make_unique<tug::EKFLQRTugCtrl>(plant));
    v.push_back(std::make_unique<tug::MRACTugCtrl>(plant));
    v.push_back(std::make_unique<tug::AutoGSTugCtrl>(plant));
    v.push_back(std::make_unique<tug::NMPCTugCtrl>(plant));
    v.push_back(std::make_unique<tug::L1AdaptiveTugCtrl>(plant));
    v.push_back(std::make_unique<tug::ScenarioMPCTugCtrl>(plant));
    return v;
}

void runWcet(const tug::PlantParameters& plant,
             std::vector<std::unique_ptr<tug::ControllerBase>>& controllers,
             const std::string& log_dir,
             const std::string& study_dir)
{
    std::ofstream raw(log_dir + "/wcet_nominal.csv");
    raw << "controller,step_time_us,step_index\n";
    std::ofstream summary(study_dir + "/wcet_summary.csv");
    summary << "controller,n_samples,mean_us,median_us,p99_us,wcet_us,max_us\n";

    std::mt19937 rng(42);
    FaultSpec no_fault;
    for (auto& ctrl : controllers) {
        std::vector<double> wcet_us;
        runOnce(plant, *ctrl, no_fault, rng, &wcet_us);
        for (size_t k = 0; k < wcet_us.size(); ++k) {
            raw << ctrl->name() << ',' << std::fixed << std::setprecision(3)
                << wcet_us[k] << ',' << k << '\n';
        }

        double sum = 0.0;
        for (double v : wcet_us) sum += v;
        const double mean_us   = wcet_us.empty() ? 0.0 : sum / static_cast<double>(wcet_us.size());
        const double median_us = robust::percentile(wcet_us, 0.50);
        const double p99_us    = robust::percentile(wcet_us, 0.99);
        const double wcet_us_v = robust::percentile(wcet_us, 0.999);
        const double max_us    = wcet_us.empty() ? 0.0 : *std::max_element(wcet_us.begin(), wcet_us.end());
        summary << ctrl->name() << ',' << wcet_us.size() << ',' << mean_us << ',' << median_us << ','
                << p99_us << ',' << wcet_us_v << ',' << max_us << '\n';
    }
    std::cout << "  [WCET] wrote " << log_dir << "/wcet_nominal.csv + " << study_dir << "/wcet_summary.csv\n";
}

void runMonteCarlo(const tug::PlantParameters& plant,
                   std::vector<std::unique_ptr<tug::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/mc_summary.csv");
    csv << "study,controller,sample_id,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var,stable\n";

    FaultSpec no_fault;
    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        int n_unstable = 0;
        std::vector<double> iae_values;
        iae_values.reserve(kNumMcSamples);

        for (int s = 0; s < kNumMcSamples; ++s) {
            tug::PlantParameters p = plant;
            p.M_re(0, 0) *= robust::perturbFactor(kMcSigma, rng);
            p.M_re(1, 1) *= robust::perturbFactor(kMcSigma, rng);
            p.M_re(2, 2) *= robust::perturbFactor(kMcSigma, rng);
            p.D_re(0, 0) *= robust::perturbFactor(kMcSigma, rng);
            p.D_re(1, 1) *= robust::perturbFactor(kMcSigma, rng);
            p.M_re_inv    = p.M_re.inverse();

            SimSummary res = runOnce(p, *ctrl, no_fault, rng);
            if (!res.stable) ++n_unstable;
            iae_values.push_back(res.iae);

            csv << "TugBoat," << ctrl->name() << ',' << s << ','
                << res.iae << ',' << res.rms_error << ',' << res.settle_time_s << ','
                << res.overshoot_pct << ',' << res.max_u << ',' << res.energy_var << ','
                << (res.stable ? 1 : 0) << '\n';
        }

        const MetricStats stats = robust::computeStats(iae_values);
        const double instability_p = static_cast<double>(n_unstable) / kNumMcSamples;
        std::cout << std::fixed << std::setprecision(4)
                  << "  [MonteCarlo] " << std::setw(16) << std::left << ctrl->name()
                  << "  P(unstable)=" << instability_p
                  << "  IAE mean=" << stats.mean << " p95=" << stats.p95 << '\n';
    }
    std::cout << "  [MonteCarlo] wrote " << study_dir << "/mc_summary.csv\n";
}

void runFaultSweep(const tug::PlantParameters& plant,
                   std::vector<std::unique_ptr<tug::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "study,controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var\n";

    const double fault_time = 0.4 * kAnalysisDuration;
    struct Trial { FaultKind kind; std::vector<double> magnitudes; };
    const std::vector<Trial> trials = {
        {FaultKind::SensorBias,    {1.0, 5.0, 20.0}},          // [m] x-position sensor offset
        {FaultKind::SensorNoise,   {0.5, 2.0, 5.0}},           // [m] x-position sensor noise sigma
        {FaultKind::ActuatorLoss,  {0.20, 0.50, 0.80}},        // fraction of tau_x lost
        {FaultKind::ActuatorStuck, {0.0, 1.0e6, -1.0e6}},      // [N] frozen tau_x
        {FaultKind::SetpointStep,  {10.0, 30.0, 60.0}},        // [m] x-target step
    };

    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        for (const auto& trial : trials) {
            for (double mag : trial.magnitudes) {
                FaultSpec fault{trial.kind, fault_time, mag, std::numeric_limits<double>::infinity()};
                SimSummary res = runOnce(plant, *ctrl, fault, rng);
                csv << "TugBoat," << ctrl->name() << ',' << robust::faultKindName(trial.kind) << ','
                    << mag << ',' << res.iae << ',' << res.rms_error << ',' << res.settle_time_s << ','
                    << res.overshoot_pct << ',' << res.max_u << ',' << res.energy_var << '\n';
            }
        }
    }
    std::cout << "  [FaultSweep] wrote " << study_dir << "/fault_sweep.csv\n";
}

} // namespace

int main(int argc, char* argv[])
{
    std::string base_dir = (argc > 1) ? argv[1] : TUG_SIM_SOURCE_DIR;
    std::string cfg_path = base_dir + "/config/plant_params.json";
    std::string log_dir  = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    tug::PlantParameters plant;
    try {
        plant = tug::PlantParameters::fromJson(cfg_path);
    } catch (const std::exception& ex) {
        std::cerr << "ERROR loading config: " << ex.what() << '\n';
        return 1;
    }

    auto controllers = buildControllers(plant);

    std::cout << "Tug Boat Numerical Simulation - Robustness Analysis\n";
    std::cout << "=======================================================\n";
    std::cout << "Plant      : " << cfg_path << '\n';
    std::cout << "Logs       : " << log_dir << '\n';
    std::cout << "Controllers: " << controllers.size() << '\n';
    std::cout << "MC samples : " << kNumMcSamples << "  (sigma=" << kMcSigma << ")\n\n";

    runWcet(plant, controllers, log_dir, base_dir);
    runMonteCarlo(plant, controllers, base_dir);
    runFaultSweep(plant, controllers, base_dir);

    std::cout << "\nRobustness analysis complete.\n";
    return 0;
}
