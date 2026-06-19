#include "susp_plant.h"
#include "road_profile.h"
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

// Robustness analysis for the Active Suspension case study: WCET profiling,
// Monte Carlo (plant-parameter perturbation), and fault sweep (sensor/actuator
// faults). Reuses the real nonlinear QuarterCarPlant + every controller in the
// roster (not a linearized StateSpace approximation, since most controllers
// here are nonlinear/ML-based) via a lightweight in-memory simulation loop with
// no per-step file I/O.
//
// Output, matching the schema/location tools/generate_report.py already reads
// (case-study/<Study>/{mc_summary,fault_sweep,wcet_summary}.csv) so the report
// pipeline picks these up with no changes:
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)

#ifndef SUSP_SIM_SOURCE_DIR
#define SUSP_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace {

// Truncated relative to the default 5 s scenario duration: enough to capture
// the step-bump transient without paying full-length cost x N_SAMPLES x 18
// controllers (several of which solve a QP per step).
constexpr double kAnalysisDuration = 2.0;   // [s]
constexpr int    kNumMcSamples     = 30;
constexpr double kMcSigma          = 0.15;  // 15% relative perturbation

susp::ScenarioConfig nominalScenario(double duration_s) {
    susp::ScenarioConfig sc;
    sc.id          = "nominal";
    sc.description = "robustness-analysis nominal road input";
    sc.road_type   = susp::RoadType::STEP;
    sc.amplitude   = 0.025;
    sc.step_time   = 0.5;
    sc.duration_s  = duration_s;
    return sc;
}

// Runs one (controller, plant, fault) trial through the real nonlinear
// quarter-car plant with no per-step file I/O. If wcet_us is non-null, records
// the wall-clock time of every controller.compute() call (sensor-fault
// preprocessing is excluded from the timed region).
SimSummary runOnce(const susp::PlantParams& plant_params,
                    susp::ControllerBase&    ctrl,
                    const FaultSpec&         fault,
                    std::mt19937&            rng,
                    std::vector<double>*     wcet_us = nullptr)
{
    using Clock = std::chrono::steady_clock;

    const double dt      = plant_params.Ts;
    const int    N_steps = static_cast<int>(std::lround(kAnalysisDuration / dt));

    susp::QuarterCarPlant dyn(plant_params);
    susp::RoadProfile     road(nominalScenario(kAnalysisDuration));
    ctrl.reset();

    SimSummary out;
    double sum_e2       = 0.0;
    double sum_u        = 0.0;
    double sum_u2        = 0.0;
    double e0_abs       = -1.0;
    int    in_band_run  = 0;
    bool   settled      = false;
    constexpr double kSettleBand      = 0.02;
    constexpr int    kSettleHysteresis = 10;

    for (int k = 0; k < N_steps; ++k) {
        const double t      = k * dt;
        const auto&  x_true = dyn.state();
        const double z_r    = road.compute(t);

        // Sensor fault: perturb only the measured z_s fed to the controller.
        Eigen::Vector4d x_meas = x_true;
        x_meas(0) = robust::applySensorFault(x_true(0), fault, t, rng);

        double F_raw;
        if (wcet_us) {
            auto t0 = Clock::now();
            F_raw   = ctrl.compute(x_meas, z_r);
            auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            F_raw = ctrl.compute(x_meas, z_r);
        }

        // Actuator fault, then saturation (same order as simulation_runner.cpp).
        const double F_fault = robust::applyActuatorFault(F_raw, fault, t);
        const double F_act   = std::clamp(F_fault, -plant_params.F_max, plant_params.F_max);

        const double error = -x_true(0);
        out.iae           += std::abs(error) * dt;
        sum_e2             += error * error;
        out.max_abs_error   = std::max(out.max_abs_error, std::abs(error));
        out.max_u           = std::max(out.max_u, std::abs(F_act));
        sum_u              += F_act;
        sum_u2             += F_act * F_act;

        if (!std::isfinite(x_true(0)) || !std::isfinite(F_act) || std::abs(x_true(0)) > 1.0) {
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

        dyn.step(F_act, z_r);
    }

    out.rms_error  = std::sqrt(sum_e2 / N_steps);
    out.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(N_steps));
    if (!out.stable) out.iae = std::max(out.iae, 1.0e6);
    // overshoot_pct: target z_s -> 0 permanently, so the tools/metrics.py
    // convention (overshoot relative to a nonzero final reference) doesn't
    // apply here; left at its SimSummary default of 0.0.
    return out;
}

std::vector<std::unique_ptr<susp::ControllerBase>> buildControllers(const susp::PlantParams& plant) {
    std::vector<std::unique_ptr<susp::ControllerBase>> v;
    v.push_back(std::make_unique<susp::PassiveCtrl>(plant));
    v.push_back(std::make_unique<susp::PIDSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::ADRCSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::SMCSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::LQRSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::LQGSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::MPCSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::MRACSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::FuzzyPIDSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::TubeMPCSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::ILCSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::CBFSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::L1AdaptiveSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::ScenarioMPCSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::DynaSuspCtrl>(plant));
    v.push_back(std::make_unique<susp::GAOptPIDCtrl>(plant));
    v.push_back(std::make_unique<susp::PSOOptPIDCtrl>(plant));
    v.push_back(std::make_unique<susp::DEOptPIDCtrl>(plant));
    return v;
}

void runWcet(const susp::PlantParams& plant,
             std::vector<std::unique_ptr<susp::ControllerBase>>& controllers,
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
        const double mean_us = wcet_us.empty() ? 0.0 : sum / static_cast<double>(wcet_us.size());
        const double median_us = robust::percentile(wcet_us, 0.50);
        const double p99_us    = robust::percentile(wcet_us, 0.99);
        const double wcet_us_v = robust::percentile(wcet_us, 0.999);
        const double max_us    = wcet_us.empty() ? 0.0 : *std::max_element(wcet_us.begin(), wcet_us.end());
        summary << ctrl->name() << ',' << wcet_us.size() << ',' << mean_us << ',' << median_us << ','
                << p99_us << ',' << wcet_us_v << ',' << max_us << '\n';
    }
    std::cout << "  [WCET] wrote " << log_dir << "/wcet_nominal.csv + " << study_dir << "/wcet_summary.csv\n";
}

void runMonteCarlo(const susp::PlantParams& plant,
                   std::vector<std::unique_ptr<susp::ControllerBase>>& controllers,
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
            susp::PlantParams p = plant;
            p.m_s *= robust::perturbFactor(kMcSigma, rng);
            p.m_u *= robust::perturbFactor(kMcSigma, rng);
            p.k_s *= robust::perturbFactor(kMcSigma, rng);
            p.c_s *= robust::perturbFactor(kMcSigma, rng);
            p.k_t *= robust::perturbFactor(kMcSigma, rng);

            SimSummary res = runOnce(p, *ctrl, no_fault, rng);
            if (!res.stable) ++n_unstable;
            iae_values.push_back(res.iae);

            csv << "ActiveSuspension," << ctrl->name() << ',' << s << ','
                << res.iae << ',' << res.rms_error << ',' << res.settle_time_s << ','
                << res.overshoot_pct << ',' << res.max_u << ',' << res.energy_var << ','
                << (res.stable ? 1 : 0) << '\n';
        }

        const MetricStats stats = robust::computeStats(iae_values);
        const double instability_p = static_cast<double>(n_unstable) / kNumMcSamples;
        std::cout << std::fixed << std::setprecision(4)
                  << "  [MonteCarlo] " << std::setw(12) << std::left << ctrl->name()
                  << "  P(unstable)=" << instability_p
                  << "  IAE mean=" << stats.mean << " p95=" << stats.p95 << '\n';
    }
    std::cout << "  [MonteCarlo] wrote " << study_dir << "/mc_summary.csv\n";
}

void runFaultSweep(const susp::PlantParams& plant,
                   std::vector<std::unique_ptr<susp::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "study,controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var\n";

    const double fault_time = 0.4 * kAnalysisDuration;
    struct Trial { FaultKind kind; std::vector<double> magnitudes; };
    // Setpoint faults don't apply here: every controller regulates to a fixed
    // internal reference of 0 with no externally-passed setpoint argument.
    const std::vector<Trial> trials = {
        {FaultKind::SensorBias,    {0.01, 0.03, 0.06}},     // [m] z_s sensor offset
        {FaultKind::SensorNoise,   {0.002, 0.006, 0.012}},  // [m] z_s sensor noise sigma
        {FaultKind::ActuatorLoss,  {0.20, 0.50, 0.80}},     // fraction of F_act lost
        {FaultKind::ActuatorStuck, {0.0, 500.0, -500.0}},   // [N] frozen F_act
    };

    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        for (const auto& trial : trials) {
            for (double mag : trial.magnitudes) {
                FaultSpec fault{trial.kind, fault_time, mag, std::numeric_limits<double>::infinity()};
                SimSummary res = runOnce(plant, *ctrl, fault, rng);
                csv << "ActiveSuspension," << ctrl->name() << ',' << robust::faultKindName(trial.kind) << ','
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
    std::string base_dir   = (argc > 1) ? argv[1] : SUSP_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string log_dir    = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    susp::PlantParams plant;
    try {
        plant = susp::PlantParams::fromJson(plant_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading plant: " << e.what() << '\n';
        return 1;
    }

    auto controllers = buildControllers(plant);

    std::cout << "Active Suspension - Robustness Analysis\n";
    std::cout << "========================================\n";
    std::cout << "Plant      : " << plant_json << '\n';
    std::cout << "Logs       : " << log_dir << '\n';
    std::cout << "Controllers: " << controllers.size() << '\n';
    std::cout << "MC samples : " << kNumMcSamples << "  (sigma=" << kMcSigma << ")\n\n";

    runWcet(plant, controllers, log_dir, base_dir);
    runMonteCarlo(plant, controllers, base_dir);
    runFaultSweep(plant, controllers, base_dir);

    std::cout << "\nRobustness analysis complete.\n";
    return 0;
}
