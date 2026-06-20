#include "sotec_plant.h"
#include "controllers.h"
#include "RobustnessStats.h"
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

// Robustness analysis for the Solar Ocean Thermal Energy Conversion (S-OTEC)
// case study: WCET profiling, Monte Carlo (plant-parameter perturbation),
// and fault sweep (sensor/actuator/setpoint faults). Reuses the real
// SotecPlant + every controller in the roster via a lightweight in-memory
// simulation loop with no per-step file I/O.
//
// Output, matching the schema/location tools/generate_report.py already reads
// (case-study/<Study>/{mc_summary,fault_sweep,wcet_summary}.csv) so the report
// pipeline picks these up with no changes:
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)

#ifndef SOTEC_SIM_SOURCE_DIR
#define SOTEC_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace {

// Default scenario duration (Ts=30s, 5400s -> 180 steps) is already cheap
// across 30 MC samples x 12 controllers, so no truncation is needed here.
constexpr double kAnalysisDuration = 5400.0;  // [s]
constexpr int    kNumMcSamples     = 30;
constexpr double kMcSigma          = 0.15;    // 15% relative perturbation

// Runs one (controller, plant, fault) trial through the real nonlinear
// S-OTEC plant with no per-step file I/O. If wcet_us is non-null, records
// the wall-clock time of every controller.compute() call.
SimSummary runOnce(const sotec::PlantParams& plant_params,
                    sotec::ControllerBase&    ctrl,
                    const FaultSpec&          fault,
                    std::mt19937&             rng,
                    std::vector<double>*      wcet_us = nullptr)
{
    using Clock = std::chrono::steady_clock;

    const double dt      = plant_params.Ts;
    const int    N_steps = static_cast<int>(std::lround(kAnalysisDuration / dt));

    sotec::SotecPlant plant(plant_params);
    plant.reset(20.0, 20.0);
    ctrl.reset();

    // Nominal clear-sky disturbance + MPPT setpoint, matching s01_mppt_steady.json.
    const sotec::Disturbance d_nom{800.0, 20.0, 6.0};  // G, T_amb, T_c
    const double ref_nom = 63.0;

    SimSummary out;
    double sum_e2      = 0.0;
    double sum_u       = 0.0;
    double sum_u2      = 0.0;
    double e0_abs      = -1.0;
    int    in_band_run = 0;
    bool   settled     = false;
    double max_y = -1e9, min_y = 1e9, final_ref = ref_nom;
    constexpr double kSettleBand       = 0.02;
    constexpr int    kSettleHysteresis = 10;

    for (int k = 0; k < N_steps; ++k) {
        const double t = k * dt;

        const double T_h_ref = robust::applySetpointFault(ref_nom, fault, t);
        final_ref               = T_h_ref;

        const Eigen::Vector2d& x_true = plant.state();

        // Sensor fault: perturb only the measured T_h fed to the controller
        // (also feeds every controller's internal m_dot_wf feedforward, since
        // that's derived from the same state(0) argument).
        Eigen::Vector2d x_meas = x_true;
        x_meas(0) = robust::applySensorFault(x_true(0), fault, t, rng);

        sotec::CtrlOutput cmd;
        if (wcet_us) {
            auto t0 = Clock::now();
            cmd     = ctrl.compute(x_meas, T_h_ref, d_nom.T_c, d_nom.G);
            auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            cmd = ctrl.compute(x_meas, T_h_ref, d_nom.T_c, d_nom.G);
        }

        // Actuator fault on m_dot_f only (the real regulating decision);
        // m_dot_wf is left untouched -- the plant re-clamps it via
        // m_dot_wf_max_safe(T_h) regardless, so a fault there is mostly masked.
        const double mf_fault = robust::applyActuatorFault(cmd.m_dot_f, fault, t);
        const double mf       = std::clamp(mf_fault, plant_params.m_dot_f_min, plant_params.m_dot_f_max);

        const double error  = T_h_ref - x_true(0);
        out.iae            += std::abs(error) * dt;
        sum_e2              += error * error;
        out.max_abs_error    = std::max(out.max_abs_error, std::abs(error));
        out.max_u             = std::max(out.max_u, mf);
        sum_u                 += mf;
        sum_u2                += mf * mf;
        max_y = std::max(max_y, x_true(0));
        min_y = std::min(min_y, x_true(0));

        if (!std::isfinite(x_true(0)) || !std::isfinite(x_true(1)) || !std::isfinite(mf) ||
            x_true(0) > 500.0) {
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

        plant.step(mf, cmd.m_dot_wf, d_nom);
    }

    out.rms_error  = std::sqrt(sum_e2 / N_steps);
    out.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(N_steps));
    if (!out.stable) out.iae = std::max(out.iae, 1.0e6);

    if (std::abs(final_ref) > 1e-9 && max_y > final_ref) {
        out.overshoot_pct = (max_y - final_ref) / std::abs(final_ref) * 100.0;
    }
    return out;
}

std::vector<std::unique_ptr<sotec::ControllerBase>> buildControllers(const sotec::PlantParams& plant) {
    std::vector<std::unique_ptr<sotec::ControllerBase>> v;
    v.push_back(std::make_unique<sotec::OpenLoopCtrl>(plant));
    v.push_back(std::make_unique<sotec::PIDThCtrl>(plant));
    v.push_back(std::make_unique<sotec::ADRCCtrl>(plant));
    v.push_back(std::make_unique<sotec::MPCCtrl>(plant));
    v.push_back(std::make_unique<sotec::LQRCtrl>(plant));
    v.push_back(std::make_unique<sotec::FuzzyPIDCtrl>(plant));
    v.push_back(std::make_unique<sotec::MRACCtrl>(plant));
    v.push_back(std::make_unique<sotec::L1AdaptiveCtrl>(plant));
    v.push_back(std::make_unique<sotec::GainScheduledCtrl>(plant));
    v.push_back(std::make_unique<sotec::ScenarioMPCCtrl>(plant));
    v.push_back(std::make_unique<sotec::DynaCtrl>(plant));
    v.push_back(std::make_unique<sotec::NeuralPIDCtrl>(plant));
    return v;
}

void runWcet(const sotec::PlantParams& plant,
             std::vector<std::unique_ptr<sotec::ControllerBase>>& controllers,
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

void runMonteCarlo(const sotec::PlantParams& plant,
                   std::vector<std::unique_ptr<sotec::ControllerBase>>& controllers,
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
            sotec::PlantParams p = plant;
            p.eta_coll *= robust::perturbFactor(kMcSigma, rng);
            p.U_coll   *= robust::perturbFactor(kMcSigma, rng);
            p.m_tank   *= robust::perturbFactor(kMcSigma, rng);
            p.a1       *= robust::perturbFactor(kMcSigma, rng);

            SimSummary res = runOnce(p, *ctrl, no_fault, rng);
            if (!res.stable) ++n_unstable;
            iae_values.push_back(res.iae);

            csv << "SOTEC," << ctrl->name() << ',' << s << ','
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

void runFaultSweep(const sotec::PlantParams& plant,
                   std::vector<std::unique_ptr<sotec::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "study,controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var\n";

    const double fault_time = 0.4 * kAnalysisDuration;
    const double mid_mf     = 0.5 * (plant.m_dot_f_min + plant.m_dot_f_max);
    struct Trial { FaultKind kind; std::vector<double> magnitudes; };
    const std::vector<Trial> trials = {
        {FaultKind::SensorBias,    {2.0, 5.0, 8.0}},                                  // [degC] T_h sensor offset
        {FaultKind::SensorNoise,   {1.0, 2.0, 4.0}},                                  // [degC] T_h sensor noise sigma
        {FaultKind::ActuatorLoss,  {0.20, 0.50, 0.80}},                               // fraction of m_dot_f lost
        {FaultKind::ActuatorStuck, {plant.m_dot_f_min, mid_mf, plant.m_dot_f_max}},   // frozen m_dot_f
        {FaultKind::SetpointStep,  {3.0, 6.0, 9.0}},                                  // [degC] T_h_ref step
    };

    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        for (const auto& trial : trials) {
            for (double mag : trial.magnitudes) {
                FaultSpec fault{trial.kind, fault_time, mag, std::numeric_limits<double>::infinity()};
                SimSummary res = runOnce(plant, *ctrl, fault, rng);
                csv << "SOTEC," << ctrl->name() << ',' << robust::faultKindName(trial.kind) << ','
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
    std::string base_dir = (argc > 1) ? argv[1] : SOTEC_SIM_SOURCE_DIR;
    std::string cfg_path = base_dir + "/config/plant_params.json";
    std::string log_dir  = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    sotec::PlantParams plant;
    try {
        plant = sotec::PlantParams::fromJson(cfg_path);
    } catch (const std::exception& ex) {
        std::cerr << "ERROR loading config: " << ex.what() << '\n';
        return 1;
    }

    auto controllers = buildControllers(plant);

    std::cout << "Solar Ocean Thermal Energy Conversion System - Robustness Analysis\n";
    std::cout << "======================================================================\n";
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
