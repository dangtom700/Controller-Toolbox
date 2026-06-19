#include "solar_plant.h"
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

// Robustness analysis for the Solar-Driven Cooling case study: WCET
// profiling, Monte Carlo (plant-parameter perturbation), and fault sweep
// (sensor/actuator/setpoint faults). Reuses the real SolarCoolingPlant +
// every controller in the roster via a lightweight in-memory simulation loop
// with no per-step file I/O.
//
// Output, matching the schema/location tools/generate_report.py already reads
// (case-study/<Study>/{mc_summary,fault_sweep,wcet_summary}.csv) so the report
// pipeline picks these up with no changes:
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)

#ifndef SOLAR_SIM_SOURCE_DIR
#define SOLAR_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace {

// Truncated relative to the default 3600 s scenario: half-length is plenty
// to see the design-point regulation response, at lower cost across
// kNumMcSamples x 14 controllers (CEM/MPC/GPC-RLS solve per-step problems).
constexpr double kAnalysisDuration = 1800.0;  // [s]
constexpr int    kNumMcSamples     = 30;
constexpr double kMcSigma          = 0.15;    // 15% relative perturbation

SimSummary runOnce(const solar::PlantParams& plant_params,
                    solar::ControllerBase&    ctrl,
                    const FaultSpec&          fault,
                    std::mt19937&             rng,
                    std::vector<double>*      wcet_us = nullptr)
{
    using Clock = std::chrono::steady_clock;

    const double dt      = plant_params.dt;
    const int    N_steps = static_cast<int>(std::lround(kAnalysisDuration / dt));

    solar::SolarCoolingPlant plant(plant_params);
    ctrl.reset();

    // Nominal weather/setpoint, matching main.cpp's s01_design_steady scenario.
    const solar::WeatherInput w_nom{920.0, 35.6, 0.35, 3.028};
    const double ref_nom = 40.0;

    // Warm-start: one step at nominal to prime the controller (mirrors simulation_runner.cpp).
    solar::ControlInput u0  = {0.10, 0.85};
    solar::PlantOutput  out = plant.step(w_nom, u0);

    SimSummary summary;
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

        const double ref          = robust::applySetpointFault(ref_nom, fault, t);
        final_ref                  = ref;
        const double measured_Tw1 = robust::applySensorFault(out.Tw1_C, fault, t, rng);

        solar::ControlInput u;
        if (wcet_us) {
            auto t0 = Clock::now();
            u       = ctrl.compute(ref, measured_Tw1, w_nom.G);
            auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            u = ctrl.compute(ref, measured_Tw1, w_nom.G);
        }

        u.m_dot_w = std::clamp(robust::applyActuatorFault(u.m_dot_w, fault, t), 0.0, 0.30);

        out = plant.step(w_nom, u);
        ctrl.setLastEER(out.EER_grid);

        const double error = ref - out.Tw1_C;
        summary.iae          += std::abs(error) * dt;
        sum_e2                 += error * error;
        summary.max_abs_error  = std::max(summary.max_abs_error, std::abs(error));
        summary.max_u          = std::max(summary.max_u, u.m_dot_w);
        sum_u                   += u.m_dot_w;
        sum_u2                  += u.m_dot_w * u.m_dot_w;
        max_y = std::max(max_y, out.Tw1_C);
        min_y = std::min(min_y, out.Tw1_C);

        if (!std::isfinite(out.Tw1_C) || out.Tw1_C < -50.0 || out.Tw1_C > 150.0) {
            summary.stable = false;
        }

        if (k == 0) e0_abs = std::abs(error);
        if (!settled) {
            const double band = kSettleBand * std::max(e0_abs, 1e-9);
            if (std::abs(error) <= band) {
                if (++in_band_run >= kSettleHysteresis) { summary.settle_time_s = t; settled = true; }
            } else {
                in_band_run = 0;
            }
        }
    }

    summary.rms_error  = std::sqrt(sum_e2 / N_steps);
    summary.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(N_steps));
    if (!summary.stable) summary.iae = std::max(summary.iae, 1.0e6);

    if (std::abs(final_ref) > 1e-9) {
        if (final_ref > 0 && max_y > final_ref) {
            summary.overshoot_pct = (max_y - final_ref) / std::abs(final_ref) * 100.0;
        } else if (final_ref < 0 && min_y < final_ref) {
            summary.overshoot_pct = (final_ref - min_y) / std::abs(final_ref) * 100.0;
        }
    }
    return summary;
}

std::vector<std::unique_ptr<solar::ControllerBase>> buildControllers(double Ts) {
    std::vector<std::unique_ptr<solar::ControllerBase>> v;
    v.push_back(std::make_unique<solar::PIDController>(Ts));
    v.push_back(std::make_unique<solar::FFPIDController>(Ts));
    v.push_back(std::make_unique<solar::MPCController>(Ts, 40.0, 0.018, 4.5));
    v.push_back(std::make_unique<solar::ADRCSolarCtrl>(Ts));
    v.push_back(std::make_unique<solar::FuzzyPIDSolarCtrl>(Ts));
    v.push_back(std::make_unique<solar::SmithPredictorSolarCtrl>(Ts));
    v.push_back(std::make_unique<solar::MRACSolarCtrl>(Ts));
    v.push_back(std::make_unique<solar::ESCSolarCtrl>(Ts));
    v.push_back(std::make_unique<solar::GPCRLSSolarCtrl>(Ts, 40.0));
    v.push_back(std::make_unique<solar::ILCSolarCtrl>(Ts));
    v.push_back(std::make_unique<solar::NeuralPIDSolarCtrl>(Ts));
    v.push_back(std::make_unique<solar::L1AdaptiveSolarCtrl>(Ts));
    v.push_back(std::make_unique<solar::DynaSolarCtrl>(Ts));
    v.push_back(std::make_unique<solar::CEMSolarCtrl>(Ts, 40.0));
    return v;
}

void runWcet(const solar::PlantParams& plant,
             std::vector<std::unique_ptr<solar::ControllerBase>>& controllers,
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

void runMonteCarlo(const solar::PlantParams& plant,
                   std::vector<std::unique_ptr<solar::ControllerBase>>& controllers,
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
            solar::PlantParams p = plant;
            p.pv.eta_ref          *= robust::perturbFactor(kMcSigma, rng);
            p.evap.me_a            *= robust::perturbFactor(kMcSigma, rng);
            p.chiller.Q_evap_kW    *= robust::perturbFactor(kMcSigma, rng);
            p.hydraulics.eta_p0    *= robust::perturbFactor(kMcSigma, rng);

            SimSummary res = runOnce(p, *ctrl, no_fault, rng);
            if (!res.stable) ++n_unstable;
            iae_values.push_back(res.iae);

            csv << "SolarCooling," << ctrl->name() << ',' << s << ','
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

void runFaultSweep(const solar::PlantParams& plant,
                   std::vector<std::unique_ptr<solar::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "study,controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var\n";

    const double fault_time = 0.4 * kAnalysisDuration;
    struct Trial { FaultKind kind; std::vector<double> magnitudes; };
    const std::vector<Trial> trials = {
        {FaultKind::SensorBias,    {1.0, 3.0, 6.0}},     // [degC] Tw1 sensor offset
        {FaultKind::SensorNoise,   {0.5, 1.5, 3.0}},     // [degC] Tw1 sensor noise sigma
        {FaultKind::ActuatorLoss,  {0.20, 0.50, 0.80}},  // fraction of m_dot_w lost
        {FaultKind::ActuatorStuck, {0.02, 0.10, 0.22}},  // [kg/s] frozen m_dot_w (min/mid/max)
        {FaultKind::SetpointStep,  {2.0, 5.0, 10.0}},    // [degC] ref_Tw1 step
    };

    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        for (const auto& trial : trials) {
            for (double mag : trial.magnitudes) {
                FaultSpec fault{trial.kind, fault_time, mag, std::numeric_limits<double>::infinity()};
                SimSummary res = runOnce(plant, *ctrl, fault, rng);
                csv << "SolarCooling," << ctrl->name() << ',' << robust::faultKindName(trial.kind) << ','
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
    std::string base_dir = (argc > 1) ? argv[1] : SOLAR_SIM_SOURCE_DIR;
    std::string cfg_path = base_dir + "/config/plant_params.json";
    std::string log_dir  = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    solar::PlantParams plant;
    try {
        plant = solar::PlantParams::fromJson(cfg_path);
    } catch (const std::exception& ex) {
        std::cerr << "ERROR loading config: " << ex.what() << '\n';
        return 1;
    }

    auto controllers = buildControllers(plant.dt);

    std::cout << "Solar-Driven Cooling System - Robustness Analysis\n";
    std::cout << "===================================================\n";
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
