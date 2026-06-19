#include "humid_plant.h"
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

// Robustness analysis for the Humidification case study: WCET profiling,
// Monte Carlo (plant-parameter perturbation), and fault sweep (sensor/
// actuator/setpoint faults). Reuses the real HumidificationPlant + every
// controller in the roster via a lightweight in-memory simulation loop with
// no per-step file I/O.
//
// Output, matching the schema/location tools/generate_report.py already reads
// (case-study/<Study>/{mc_summary,fault_sweep,wcet_summary}.csv) so the report
// pipeline picks these up with no changes:
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)

#ifndef HUMID_SIM_SOURCE_DIR
#define HUMID_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace {

// Truncated relative to the default 10800 s (3 h) scenario: 1 h is plenty to
// see the fan-driven RH response (tau_room is on the order of tens of
// minutes), at far lower cost across kNumMcSamples x 15 controllers.
constexpr double kAnalysisDuration = 3600.0;  // [s]
constexpr int    kNumMcSamples     = 30;
constexpr double kMcSigma          = 0.15;    // 15% relative perturbation

SimSummary runOnce(const humid::PlantParams& plant_params,
                    humid::ControllerBase&    ctrl,
                    const FaultSpec&          fault,
                    std::mt19937&             rng,
                    std::vector<double>*      wcet_us = nullptr)
{
    using Clock = std::chrono::steady_clock;

    const double dt      = plant_params.Ts;
    const int    N_steps = static_cast<int>(std::lround(kAnalysisDuration / dt));

    humid::HumidificationPlant plant(plant_params);
    plant.reset(0.20);
    ctrl.reset();

    auto* ffpid = dynamic_cast<humid::FFPIDHumidCtrl*>(&ctrl);

    // Nominal disturbance/setpoint, matching main.cpp's s01_design scenario.
    const humid::Disturbance d_nom{263.15, 0.35, 0.0};
    const double ref_nom = 0.45;

    // Prime: one step at the initial condition (mirrors simulation_runner.cpp).
    humid::ControlInput u0{humid::kFanMid, humid::kTa_nom};
    humid::PlantOutput  out = plant.step(u0, d_nom);

    SimSummary out_summary;
    double sum_e2      = 0.0;
    double sum_u       = 0.0;
    double sum_u2      = 0.0;
    double e0_abs      = -1.0;
    int    in_band_run = 0;
    bool   settled     = false;
    double max_phi = -1e9, min_phi = 1e9, final_ref = ref_nom;
    constexpr double kSettleBand      = 0.02;
    constexpr int    kSettleHysteresis = 10;

    for (int k = 0; k < N_steps; ++k) {
        const double t = k * dt;
        if (ffpid) ffpid->setOutdoorPhi(d_nom.phi_out);

        const double ref       = robust::applySetpointFault(ref_nom, fault, t);
        final_ref               = ref;
        const double phi_meas  = robust::applySensorFault(out.phi_measured, fault, t, rng);

        humid::ControlInput u;
        if (wcet_us) {
            auto t0 = Clock::now();
            u       = ctrl.compute(phi_meas, ref);
            auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            u = ctrl.compute(phi_meas, ref);
        }

        u.u_fan = humid::clampFan(robust::applyActuatorFault(u.u_fan, fault, t));

        out = plant.step(u, d_nom);

        const double error = ref - out.phi_measured;
        out_summary.iae          += std::abs(error) * dt;
        sum_e2                     += error * error;
        out_summary.max_abs_error  = std::max(out_summary.max_abs_error, std::abs(error));
        out_summary.max_u          = std::max(out_summary.max_u, u.u_fan);
        sum_u                       += u.u_fan;
        sum_u2                      += u.u_fan * u.u_fan;
        max_phi = std::max(max_phi, out.phi_room);
        min_phi = std::min(min_phi, out.phi_room);

        if (!std::isfinite(out.phi_room) || out.phi_room < -0.5 || out.phi_room > 1.5) {
            out_summary.stable = false;
        }

        if (k == 0) e0_abs = std::abs(error);
        if (!settled) {
            const double band = kSettleBand * std::max(e0_abs, 1e-9);
            if (std::abs(error) <= band) {
                if (++in_band_run >= kSettleHysteresis) { out_summary.settle_time_s = t; settled = true; }
            } else {
                in_band_run = 0;
            }
        }
    }

    out_summary.rms_error  = std::sqrt(sum_e2 / N_steps);
    out_summary.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(N_steps));
    if (!out_summary.stable) out_summary.iae = std::max(out_summary.iae, 1.0e6);

    if (std::abs(final_ref) > 1e-9) {
        if (final_ref > 0 && max_phi > final_ref) {
            out_summary.overshoot_pct = (max_phi - final_ref) / std::abs(final_ref) * 100.0;
        } else if (final_ref < 0 && min_phi < final_ref) {
            out_summary.overshoot_pct = (final_ref - min_phi) / std::abs(final_ref) * 100.0;
        }
    }
    return out_summary;
}

std::vector<std::unique_ptr<humid::ControllerBase>> buildControllers(double Ts) {
    std::vector<std::unique_ptr<humid::ControllerBase>> v;
    v.push_back(std::make_unique<humid::PIDHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::PID_AWHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::FFPIDHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::CascadePIDHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::GainSchedHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::SmithHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::ADRCHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::MPCHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::MRACHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::GPCRLSHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::DynaHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::ILCHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::NeuralPIDHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::L1AdaptiveHumidCtrl>(Ts));
    v.push_back(std::make_unique<humid::CBFSafetyHumidCtrl>(Ts));
    return v;
}

void runWcet(const humid::PlantParams& plant,
             std::vector<std::unique_ptr<humid::ControllerBase>>& controllers,
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

void runMonteCarlo(const humid::PlantParams& plant,
                   std::vector<std::unique_ptr<humid::ControllerBase>>& controllers,
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
            humid::PlantParams p = plant;
            p.l_plate  *= robust::perturbFactor(kMcSigma, rng);
            p.A_plates *= robust::perturbFactor(kMcSigma, rng);
            p.V_room   *= robust::perturbFactor(kMcSigma, rng);
            p.ACH      *= robust::perturbFactor(kMcSigma, rng);
            p.init();  // refresh m_dot_inf / tau_room derived from V_room, ACH

            SimSummary res = runOnce(p, *ctrl, no_fault, rng);
            if (!res.stable) ++n_unstable;
            iae_values.push_back(res.iae);

            csv << "Humidification," << ctrl->name() << ',' << s << ','
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

void runFaultSweep(const humid::PlantParams& plant,
                   std::vector<std::unique_ptr<humid::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "study,controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var\n";

    const double fault_time = 0.4 * kAnalysisDuration;
    struct Trial { FaultKind kind; std::vector<double> magnitudes; };
    const std::vector<Trial> trials = {
        {FaultKind::SensorBias,    {0.02, 0.05, 0.10}},   // [fraction] phi_measured offset
        {FaultKind::SensorNoise,   {0.01, 0.03, 0.05}},   // [fraction] phi_measured noise sigma
        {FaultKind::ActuatorLoss,  {0.20, 0.50, 0.80}},   // fraction of u_fan lost
        {FaultKind::ActuatorStuck, {1.0, 2.25, 3.5}},     // [m/s] frozen u_fan (min/mid/max)
        {FaultKind::SetpointStep,  {0.05, 0.10, 0.20}},   // [fraction] ref_phi step
    };

    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        for (const auto& trial : trials) {
            for (double mag : trial.magnitudes) {
                FaultSpec fault{trial.kind, fault_time, mag, std::numeric_limits<double>::infinity()};
                SimSummary res = runOnce(plant, *ctrl, fault, rng);
                csv << "Humidification," << ctrl->name() << ',' << robust::faultKindName(trial.kind) << ','
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
    std::string base_dir = (argc > 1) ? argv[1] : HUMID_SIM_SOURCE_DIR;
    std::string cfg_path = base_dir + "/config/plant_params.json";
    std::string log_dir  = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    humid::PlantParams plant;
    try {
        plant = humid::PlantParams::fromJson(cfg_path);
    } catch (const std::exception& ex) {
        std::cerr << "ERROR loading config: " << ex.what() << '\n';
        return 1;
    }

    auto controllers = buildControllers(plant.Ts);

    std::cout << "Porous Fiber Plate Humidification - Robustness Analysis\n";
    std::cout << "=========================================================\n";
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
