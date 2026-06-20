#include "buck_boost_plant.h"
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

// Robustness analysis for the Non-Inverting Buck-Boost Converter case study:
// WCET profiling, Monte Carlo (plant-parameter perturbation), and fault sweep
// (sensor/actuator/setpoint faults). Reuses the real BuckBoostPlant + every
// controller in the roster via a lightweight in-memory simulation loop with
// no per-step file I/O.
//
// Output, matching the schema/location tools/generate_report.py already reads
// (case-study/<Study>/{mc_summary,fault_sweep,wcet_summary}.csv) so the report
// pipeline picks these up with no changes:
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)

#ifndef BUCK_BOOST_SIM_SOURCE_DIR
#define BUCK_BOOST_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace {

// Default scenario duration (~0.06 s at Ts=20us, ~3000 steps) is already cheap
// across 30 MC samples x 12 controllers with only 1 control input, so no
// truncation is needed here (unlike the slower-Ts thermal studies).
constexpr double kAnalysisDuration = 0.060;  // [s]
constexpr int    kNumMcSamples     = 30;
constexpr double kMcSigma          = 0.15;   // 15% relative perturbation

// Runs one (controller, plant, fault) trial through the real nonlinear
// buck-boost plant with no per-step file I/O. If wcet_us is non-null, records
// the wall-clock time of every controller.compute() call.
SimSummary runOnce(const conv::PlantParams& plant_params,
                    conv::ControllerBase&    ctrl,
                    const FaultSpec&         fault,
                    std::mt19937&            rng,
                    std::vector<double>*     wcet_us = nullptr)
{
    using Clock = std::chrono::steady_clock;

    const double dt      = plant_params.Ts;
    const int    N_steps = static_cast<int>(std::lround(kAnalysisDuration / dt));

    conv::BuckBoostPlant dyn(plant_params);
    ctrl.reset();

    // Nominal input voltage + buck-regime target, matching s01_buck.json.
    const double V_in    = plant_params.V_in_nom;
    const double ref_nom = 8.0;
    conv::ConvMode mode  = conv::ConvMode::BUCK;

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

        const double v_ref = robust::applySetpointFault(ref_nom, fault, t);
        final_ref            = v_ref;

        // Mode hysteresis computed externally, using the FAULTED setpoint
        // (mirrors simulation_runner.cpp:50-54 - a real controller only ever
        // sees the setpoint it was given, faulted or not).
        if (v_ref > V_in + plant_params.hysteresis)      mode = conv::ConvMode::BOOST;
        else if (v_ref < V_in - plant_params.hysteresis) mode = conv::ConvMode::BUCK;

        const Eigen::Vector2d& x_true = dyn.state();

        // Sensor fault: perturb only the measured v_C fed to the controller.
        Eigen::Vector2d x_meas = x_true;
        x_meas(1) = robust::applySensorFault(x_true(1), fault, t, rng);

        double d_raw;
        if (wcet_us) {
            auto t0 = Clock::now();
            d_raw   = ctrl.compute(x_meas, v_ref, V_in);
            auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            d_raw = ctrl.compute(x_meas, v_ref, V_in);
        }

        // Actuator fault, then saturation (same order as simulation_runner.cpp).
        const double d_fault = robust::applyActuatorFault(d_raw, fault, t);
        const double d       = std::clamp(d_fault, plant_params.d_min, plant_params.d_max);

        const double v_C    = x_true(1);
        const double error  = v_ref - v_C;
        out.iae            += std::abs(error) * dt;
        sum_e2              += error * error;
        out.max_abs_error    = std::max(out.max_abs_error, std::abs(error));
        out.max_u             = std::max(out.max_u, d);
        sum_u                 += d;
        sum_u2                += d * d;
        max_y = std::max(max_y, v_C);
        min_y = std::min(min_y, v_C);

        if (!std::isfinite(x_true(0)) || !std::isfinite(v_C) || !std::isfinite(d) || v_C > 200.0) {
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

        dyn.step(d, mode, V_in);
    }

    out.rms_error  = std::sqrt(sum_e2 / N_steps);
    out.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(N_steps));
    if (!out.stable) out.iae = std::max(out.iae, 1.0e6);

    if (std::abs(final_ref) > 1e-9 && max_y > final_ref) {
        out.overshoot_pct = (max_y - final_ref) / std::abs(final_ref) * 100.0;
    }
    return out;
}

std::vector<std::unique_ptr<conv::ControllerBase>> buildControllers(const conv::PlantParams& plant) {
    std::vector<std::unique_ptr<conv::ControllerBase>> v;
    v.push_back(std::make_unique<conv::OpenLoopCtrl>(plant));
    v.push_back(std::make_unique<conv::PIBuckCtrl>(plant));
    v.push_back(std::make_unique<conv::PIBoostCtrl>(plant));
    v.push_back(std::make_unique<conv::TLCSClassicPICtrl>(plant));
    v.push_back(std::make_unique<conv::FuzzyPDCtrl>(plant));
    v.push_back(std::make_unique<conv::FuzzyPIDBuckCtrl>(plant));
    v.push_back(std::make_unique<conv::FuzzyPIDBoostCtrl>(plant));
    v.push_back(std::make_unique<conv::TLCSFuzzyPICtrl>(plant));
    v.push_back(std::make_unique<conv::GainScheduledCtrl>(plant));
    v.push_back(std::make_unique<conv::ADRCConvCtrl>(plant));
    v.push_back(std::make_unique<conv::MPCConvCtrl>(plant));
    v.push_back(std::make_unique<conv::LQRConvCtrl>(plant));
    return v;
}

void runWcet(const conv::PlantParams& plant,
             std::vector<std::unique_ptr<conv::ControllerBase>>& controllers,
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

void runMonteCarlo(const conv::PlantParams& plant,
                   std::vector<std::unique_ptr<conv::ControllerBase>>& controllers,
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
            conv::PlantParams p = plant;
            p.L        *= robust::perturbFactor(kMcSigma, rng);
            p.C        *= robust::perturbFactor(kMcSigma, rng);
            p.R        *= robust::perturbFactor(kMcSigma, rng);
            p.V_in_nom *= robust::perturbFactor(kMcSigma, rng);

            SimSummary res = runOnce(p, *ctrl, no_fault, rng);
            if (!res.stable) ++n_unstable;
            iae_values.push_back(res.iae);

            csv << "BuckBoost," << ctrl->name() << ',' << s << ','
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

void runFaultSweep(const conv::PlantParams& plant,
                   std::vector<std::unique_ptr<conv::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "study,controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var\n";

    const double fault_time = 0.4 * kAnalysisDuration;
    struct Trial { FaultKind kind; std::vector<double> magnitudes; };
    const std::vector<Trial> trials = {
        {FaultKind::SensorBias,    {0.1, 0.3, 0.6}},                       // [V] v_C sensor offset
        {FaultKind::SensorNoise,   {0.05, 0.15, 0.3}},                    // [V] v_C sensor noise sigma
        {FaultKind::ActuatorLoss,  {0.20, 0.50, 0.80}},                   // fraction of duty cycle lost
        {FaultKind::ActuatorStuck, {plant.d_min, 0.5, plant.d_max}},      // frozen duty cycle
        {FaultKind::SetpointStep,  {1.0, 3.0, 5.0}},                      // [V] v_ref step
    };

    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        for (const auto& trial : trials) {
            for (double mag : trial.magnitudes) {
                FaultSpec fault{trial.kind, fault_time, mag, std::numeric_limits<double>::infinity()};
                SimSummary res = runOnce(plant, *ctrl, fault, rng);
                csv << "BuckBoost," << ctrl->name() << ',' << robust::faultKindName(trial.kind) << ','
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
    std::string base_dir = (argc > 1) ? argv[1] : BUCK_BOOST_SIM_SOURCE_DIR;
    std::string cfg_path = base_dir + "/config/plant_params.json";
    std::string log_dir  = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    conv::PlantParams plant;
    try {
        plant = conv::PlantParams::fromJson(cfg_path);
    } catch (const std::exception& ex) {
        std::cerr << "ERROR loading config: " << ex.what() << '\n';
        return 1;
    }

    auto controllers = buildControllers(plant);

    std::cout << "Non-Inverting Buck-Boost Converter - Robustness Analysis\n";
    std::cout << "===========================================================\n";
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
