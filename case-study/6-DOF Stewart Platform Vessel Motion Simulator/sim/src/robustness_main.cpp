#include "stewart_plant.h"
#include "cfd_input_model.h"
#include "controllers.h"
#include "RobustnessStats.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

// Robustness analysis for the 6-DOF Stewart Platform case study: WCET
// profiling, Monte Carlo (plant-parameter perturbation), and fault sweep
// (sensor/actuator/setpoint faults). Reuses the real nonlinear StewartPlant +
// CFDInputModel + every controller in the roster via a lightweight in-memory
// simulation loop with no per-step file I/O.
//
// By far the heaviest study (12-state-per-rod x6 rods, 12 controllers x up
// to 60 sea-states in the main sim), so this scopes down deliberately:
//   - 3 representative sea states (calm/moderate/extreme) instead of 60.
//   - kAnalysisDuration=15s regardless of sea state (vs. the matrix's own
//     10*T(Hs) law, which can reach 150s).
//   - WCET + Monte Carlo use the moderate sea state only (sea-state severity
//     is orthogonal to compute cost / plant-uncertainty testing); fault
//     sweep uses all 3 (fault robustness IS sea-state-dependent).
//
// MIMO fault decision: fault rod 0 only (all 6 rods are kinematically
// symmetric under nominal load, so rod 0 is as representative as any).
// delta_base_deg/delta_platform_deg are excluded from Monte Carlo
// perturbation entirely -- the documented architecture singularity sits at
// delta_base_deg == delta_platform_deg, and 5 other physically-meaningful
// fields are available without flirting with it.
//
// Output, matching the schema/location tools/generate_report.py already reads
// (case-study/<Study>/{mc_summary,fault_sweep,wcet_summary}.csv) so the report
// pipeline picks these up with no changes:
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)

#ifndef STEWART_SIM_SOURCE_DIR
#define STEWART_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace {

constexpr double kAnalysisDuration = 15.0;  // [s], regardless of sea state
constexpr int    kNumMcSamples     = 30;
constexpr double kMcSigma          = 0.15;  // 15% relative perturbation

stewart::SeaStateConfig calmSeaState() {
    stewart::SeaStateConfig c;
    c.douglas_state = 0; c.Hs = 0.0; c.direction = stewart::WaveDirection::Head;
    c.swell = false; c.equipment_load = true; c.duration_s = kAnalysisDuration;
    c.id = "ss00_head_noswell"; c.description = "Douglas sea state 0 (calm), Head seas, no swell";
    return c;
}
stewart::SeaStateConfig moderateSeaState() {
    stewart::SeaStateConfig c;  // matches CFDInputModel's own default SeaStateConfig{}
    c.duration_s = kAnalysisDuration;
    return c;
}
stewart::SeaStateConfig extremeSeaState() {
    stewart::SeaStateConfig c;
    c.douglas_state = 9; c.Hs = 16.0; c.direction = stewart::WaveDirection::Beam;
    c.swell = true; c.equipment_load = true; c.duration_s = kAnalysisDuration;
    c.id = "ss09_beam_swell"; c.description = "Douglas sea state 9 (phenomenal), Beam seas with swell";
    return c;
}

// Runs one (controller, plant, sea-state, fault) trial through the real
// nonlinear Stewart platform with no per-step file I/O. If wcet_us is
// non-null, records the wall-clock time of every controller.compute() call.
SimSummary runOnce(const stewart::PlantParams&    plant_params,
                    const stewart::SeaStateConfig& sea_cfg,
                    stewart::ControllerBase&        ctrl,
                    const FaultSpec&                 fault,
                    std::mt19937&                     rng,
                    std::vector<double>*              wcet_us = nullptr)
{
    using Clock = std::chrono::steady_clock;

    const double dt      = plant_params.Ts;
    const int    N_steps = static_cast<int>(std::lround(sea_cfg.duration_s / dt));

    stewart::StewartPlant dyn(plant_params);
    dyn.reset();
    ctrl.reset();

    stewart::CFDInputModel cfd(sea_cfg, plant_params);

    // One-sample held communication delay, same as simulation_runner.cpp.
    std::deque<stewart::Vec6> delay_queue;

    SimSummary out;
    double sum_e2      = 0.0;
    double sum_u       = 0.0;
    double sum_u2      = 0.0;
    double e0_abs      = -1.0;
    int    in_band_run = 0;
    bool   settled     = false;
    constexpr double kSettleBand       = 0.02;
    constexpr int    kSettleHysteresis = 10;

    for (int k = 0; k < N_steps; ++k) {
        const double t = k * dt;
        const stewart::PoseRef ref = cfd.poseAt(t);

        stewart::Vec6 L_cmd;
        stewart::Mat6 J;
        dyn.geometry().ikAndJacobian(ref, L_cmd, J);

        // Setpoint fault: perturb the rod-0 commanded length (length-domain --
        // the controller never sees a Cartesian reference directly).
        L_cmd(0) = robust::applySetpointFault(L_cmd(0), fault, t);

        stewart::Vec6 F_load = stewart::Vec6::Zero();
        if (sea_cfg.equipment_load) {
            stewart::Vec6 wrench;
            wrench << 0.0, 0.0, -(plant_params.F_eq + plant_params.m_platform * stewart::GRAVITY), 0.0, 0.0, 0.0;
            F_load = -(J.transpose().partialPivLu().solve(wrench));
        }

        const stewart::Vec6 L_true = dyn.length();
        const stewart::Vec6 dL     = dyn.velocity();

        // Sensor fault: perturb only the measured rod-0 length.
        stewart::Vec6 L_meas = L_true;
        L_meas(0) = robust::applySensorFault(L_true(0), fault, t, rng);

        const double z_ref_global = std::abs(ref.P(2) - plant_params.z0_mid);

        stewart::Vec6 u_cmd;
        if (wcet_us) {
            auto t0 = Clock::now();
            u_cmd   = ctrl.compute(L_cmd, L_meas, dL, t, z_ref_global);
            auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            u_cmd = ctrl.compute(L_cmd, L_meas, dL, t, z_ref_global);
        }

        // Actuator fault on rod 0, then the same clamp + comm-delay queue
        // ordering as simulation_runner.cpp.
        u_cmd(0) = robust::applyActuatorFault(u_cmd(0), fault, t);
        for (int i = 0; i < stewart::N_RODS; ++i)
            u_cmd(i) = std::clamp(u_cmd(i), -plant_params.F_rod_max, plant_params.F_rod_max);

        delay_queue.push_back(u_cmd);
        stewart::Vec6 u_applied = stewart::Vec6::Zero();
        if (static_cast<int>(delay_queue.size()) > plant_params.comm_delay_samples) {
            u_applied = delay_queue.front();
            delay_queue.pop_front();
        }

        const double error  = L_cmd(0) - L_true(0);
        out.iae             += std::abs(error) * dt;
        sum_e2               += error * error;
        out.max_abs_error     = std::max(out.max_abs_error, std::abs(error));

        double max_u_rods = 0.0;
        for (int i = 0; i < stewart::N_RODS; ++i) max_u_rods = std::max(max_u_rods, std::abs(u_applied(i)));
        out.max_u   = std::max(out.max_u, max_u_rods);
        sum_u        += u_applied(0);
        sum_u2        += u_applied(0) * u_applied(0);

        bool ok = true;
        for (int i = 0; i < stewart::N_RODS; ++i) {
            ok = ok && std::isfinite(L_true(i)) && std::isfinite(dL(i)) && std::isfinite(u_applied(i)) &&
                 L_true(i) > 0.1 && L_true(i) < 2.0;
        }
        if (!ok) out.stable = false;

        if (k == 0) e0_abs = std::abs(error);
        if (!settled) {
            const double band = kSettleBand * std::max(e0_abs, 1e-9);
            if (std::abs(error) <= band) {
                if (++in_band_run >= kSettleHysteresis) { out.settle_time_s = t; settled = true; }
            } else {
                in_band_run = 0;
            }
        }

        dyn.step(u_applied, F_load);
    }

    out.rms_error  = std::sqrt(sum_e2 / N_steps);
    out.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(N_steps));
    if (!out.stable) out.iae = std::max(out.iae, 1.0e6);
    // overshoot_pct: L_cmd(0) varies continuously (sinusoidal wave-driven
    // reference, not a step+hold), so classical step-overshoot is ill-defined
    // here -- left at SimSummary's default of 0.0 (same choice Active
    // Suspension makes for its always-zero setpoint).
    return out;
}

std::vector<std::unique_ptr<stewart::ControllerBase>> buildControllers(const stewart::PlantParams& plant) {
    std::vector<std::unique_ptr<stewart::ControllerBase>> v;
    v.push_back(std::make_unique<stewart::PIDStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::FuzzyPIDStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::ADRCStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::SMCStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::LQRStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::MPCStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::MRACStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::L1AdaptiveStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::GainScheduledStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::TubeMPCStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::NeuralPIDStewartCtrl>(plant));
    v.push_back(std::make_unique<stewart::ScenarioMPCStewartCtrl>(plant));
    return v;
}

void runWcet(const stewart::PlantParams& plant,
             std::vector<std::unique_ptr<stewart::ControllerBase>>& controllers,
             const std::string& log_dir,
             const std::string& study_dir)
{
    std::ofstream raw(log_dir + "/wcet_nominal.csv");
    raw << "controller,step_time_us,step_index\n";
    std::ofstream summary(study_dir + "/wcet_summary.csv");
    summary << "controller,n_samples,mean_us,median_us,p99_us,wcet_us,max_us\n";

    const stewart::SeaStateConfig sea = moderateSeaState();
    std::mt19937 rng(42);
    FaultSpec no_fault;
    for (auto& ctrl : controllers) {
        std::vector<double> wcet_us;
        runOnce(plant, sea, *ctrl, no_fault, rng, &wcet_us);
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

void runMonteCarlo(const stewart::PlantParams& plant,
                   std::vector<std::unique_ptr<stewart::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/mc_summary.csv");
    csv << "study,controller,sample_id,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var,stable\n";

    const stewart::SeaStateConfig sea = moderateSeaState();
    FaultSpec no_fault;
    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        int n_unstable = 0;
        std::vector<double> iae_values;
        iae_values.reserve(kNumMcSamples);

        for (int s = 0; s < kNumMcSamples; ++s) {
            stewart::PlantParams p = plant;
            p.m_rod      *= robust::perturbFactor(kMcSigma, rng);
            p.k_spring   *= robust::perturbFactor(kMcSigma, rng);
            p.b_damp     *= robust::perturbFactor(kMcSigma, rng);
            p.F_eq       *= robust::perturbFactor(kMcSigma, rng);
            p.m_platform *= robust::perturbFactor(kMcSigma, rng);

            SimSummary res = runOnce(p, sea, *ctrl, no_fault, rng);
            if (!res.stable) ++n_unstable;
            iae_values.push_back(res.iae);

            csv << "Stewart," << ctrl->name() << ',' << s << ','
                << res.iae << ',' << res.rms_error << ',' << res.settle_time_s << ','
                << res.overshoot_pct << ',' << res.max_u << ',' << res.energy_var << ','
                << (res.stable ? 1 : 0) << '\n';
        }

        const MetricStats stats = robust::computeStats(iae_values);
        const double instability_p = static_cast<double>(n_unstable) / kNumMcSamples;
        std::cout << std::fixed << std::setprecision(4)
                  << "  [MonteCarlo] " << std::setw(14) << std::left << ctrl->name()
                  << "  P(unstable)=" << instability_p
                  << "  IAE mean=" << stats.mean << " p95=" << stats.p95 << '\n';
    }
    std::cout << "  [MonteCarlo] wrote " << study_dir << "/mc_summary.csv\n";
}

void runFaultSweep(const stewart::PlantParams& plant,
                   std::vector<std::unique_ptr<stewart::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "study,controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var\n";

    const std::vector<stewart::SeaStateConfig> sea_states = {
        calmSeaState(), moderateSeaState(), extremeSeaState()
    };

    const double fault_time = 0.4 * kAnalysisDuration;
    struct Trial { FaultKind kind; std::vector<double> magnitudes; };
    const std::vector<Trial> trials = {
        {FaultKind::SensorBias,    {0.002, 0.005, 0.010}},     // [m] rod-0 length sensor offset
        {FaultKind::SensorNoise,   {0.0005, 0.0015, 0.003}},   // [m] rod-0 sensor noise sigma
        {FaultKind::ActuatorLoss,  {0.20, 0.50, 0.80}},        // fraction of rod-0 force lost
        {FaultKind::ActuatorStuck, {0.0, 3000.0, -3000.0}},    // [N] frozen rod-0 force
        {FaultKind::SetpointStep,  {0.01, 0.03, 0.06}},        // [m] rod-0 length-command step
    };

    for (auto& ctrl : controllers) {
        for (const auto& sea : sea_states) {
            std::mt19937 rng(42);
            const std::string ctrl_label = ctrl->name() + "_" + sea.id;
            for (const auto& trial : trials) {
                for (double mag : trial.magnitudes) {
                    FaultSpec fault{trial.kind, fault_time, mag, std::numeric_limits<double>::infinity()};
                    SimSummary res = runOnce(plant, sea, *ctrl, fault, rng);
                    csv << "Stewart," << ctrl_label << ',' << robust::faultKindName(trial.kind) << ','
                        << mag << ',' << res.iae << ',' << res.rms_error << ',' << res.settle_time_s << ','
                        << res.overshoot_pct << ',' << res.max_u << ',' << res.energy_var << '\n';
                }
            }
        }
    }
    std::cout << "  [FaultSweep] wrote " << study_dir << "/fault_sweep.csv\n";
}

} // namespace

int main(int argc, char* argv[])
{
    std::string base_dir = (argc > 1) ? argv[1] : STEWART_SIM_SOURCE_DIR;
    std::string cfg_path = base_dir + "/config/plant_params.json";
    std::string log_dir  = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    stewart::PlantParams plant;
    try {
        plant = stewart::PlantParams::fromJson(cfg_path);
    } catch (const std::exception& ex) {
        std::cerr << "ERROR loading config: " << ex.what() << '\n';
        return 1;
    }

    auto controllers = buildControllers(plant);

    std::cout << "6-DOF Stewart Platform Vessel Motion Simulator - Robustness Analysis\n";
    std::cout << "========================================================================\n";
    std::cout << "Plant      : " << cfg_path << '\n';
    std::cout << "Logs       : " << log_dir << '\n';
    std::cout << "Controllers: " << controllers.size() << '\n';
    std::cout << "MC samples : " << kNumMcSamples << "  (sigma=" << kMcSigma << ")\n";
    std::cout << "Analysis duration: " << kAnalysisDuration << " s (vs. up to 150s default)\n\n";

    runWcet(plant, controllers, log_dir, base_dir);
    runMonteCarlo(plant, controllers, base_dir);
    runFaultSweep(plant, controllers, base_dir);

    std::cout << "\nRobustness analysis complete.\n";
    return 0;
}
