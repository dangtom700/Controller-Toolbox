#include "bouyancy_driven_airship_in_vertical_plan_plant.h"
#include "controllers.h"
#include "simulation_runner.h"
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

// Robustness analysis for the Bouyancy-Driven Airship in Vertical Plane case study: WCET
// profiling, Monte Carlo (physical-parameter perturbation), and fault sweep (sensor/actuator/
// setpoint faults). Reuses the real 12-controller roster and nonlinear 6-state liberated-center
// plant via a lightweight in-memory simulation loop (no per-step file I/O), mirroring the
// Boiler Control / Tug Boat robustness_main.cpp pattern (see case-study/common/RobustnessStats.h).
//
// Nominal analysis scenario: s01_calm_step (paper's own Sec. 4.2.3 validation case, 60 s,
// 41.5 -> 30 deg step at t=5s) - loaded from config/scenarios/s01_calm_step.json rather than
// duplicated here, so it can never drift from the real scenario file.
//
// Monte Carlo perturbs the plant's physical mass/inertia parameters (m_bar, ms, J) - NOT the
// sample time or actuator/track limits. Controllers are always built from the ORIGINAL nominal
// PlantParams (their LQR/MPC/FBL trim and gain design depends on it); only the simulated Plant
// uses the perturbed copy - same convention as Boiler Control's robustness_main.cpp.
//
// Sensor faults perturb the measured theta only (the sole regulated output); actuator faults
// perturb the scalar force command u; setpoint faults step theta_ref. Output, matching the
// schema/location tools/generate_report.py already reads
// (case-study/<Study>/{mc_summary,fault_sweep,wcet_summary}.csv):
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)

#ifndef BOUYANCY_DRIVEN_AIRSHIP_IN_VERTICAL_PLAN_SIM_SOURCE_DIR
#define BOUYANCY_DRIVEN_AIRSHIP_IN_VERTICAL_PLAN_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace bda = bouyancydrivenairshipinverticalplan;

namespace {

constexpr int    kNumMcSamples = 30;
constexpr double kMcSigma      = 0.15;  // 15% relative perturbation

constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;

// Runs one (controller, plant-copy, fault) trial through the real nonlinear liberated-center
// plant with no per-step file I/O. If wcet_us is non-null, records the wall-clock time of every
// controller.compute() call. `plant_p` initialises the simulated PLANT (possibly perturbed);
// `scen` supplies the nominal reference/m0 profile (always s01_calm_step, unperturbed).
SimSummary runOnce(const bda::PlantParams& plant_p,
                    const bda::Scenario&    scen,
                    bda::ControllerBase&    ctrl,
                    const FaultSpec&        fault,
                    std::mt19937&           rng,
                    std::vector<double>*    wcet_us = nullptr)
{
    using Clock = std::chrono::steady_clock;
    using bda::THETA;

    bda::Plant plant(plant_p);
    plant.reset(scen.theta0_deg * DEG2RAD, scen.rp1_ref, scen.v1_0, scen.v3_0);
    ctrl.reset();

    const double dt      = plant_p.Ts;
    const int    N_steps = static_cast<int>(std::lround(scen.T_sim / dt));

    SimSummary out;
    double sum_e2 = 0.0, sum_u = 0.0, sum_u2 = 0.0;
    double e0_abs = -1.0;
    int    in_band_run = 0;
    bool   settled      = false;
    double max_theta_after_step = -1e18, min_theta_after_step = 1e18;
    constexpr double kSettleBand       = 0.02;
    constexpr int    kSettleHysteresis = 10;

    for (int k = 0; k < N_steps; ++k) {
        const double t             = k * dt;
        const double theta_ref_nom = scen.thetaRefAt(t);
        const double theta_ref     = robust::applySetpointFault(theta_ref_nom, fault, t);
        const double m0            = scen.m0At(t);

        const bda::State& x_true   = plant.state();
        const double theta_true    = x_true(THETA);
        const double theta_meas    = robust::applySensorFault(theta_true, fault, t, rng);

        bda::State x_meas = x_true;
        x_meas(THETA)     = theta_meas;

        double u_raw;
        if (wcet_us) {
            auto t0 = Clock::now();
            u_raw   = ctrl.compute(x_meas, theta_ref, scen.rp1_ref, m0);
            auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            u_raw = ctrl.compute(x_meas, theta_ref, scen.rp1_ref, m0);
        }

        const double u_faulted = robust::applyActuatorFault(u_raw, fault, t);
        const double u         = bda::clampU(plant_p, u_faulted);

        const double error = theta_ref - theta_true;
        out.iae            += std::abs(error) * dt;
        sum_e2               += error * error;
        out.max_abs_error     = std::max(out.max_abs_error, std::abs(error));
        out.max_u              = std::max(out.max_u, std::abs(u));
        sum_u                   += u;
        sum_u2                   += u * u;

        if (t >= scen.step_time) {
            max_theta_after_step = std::max(max_theta_after_step, theta_true);
            min_theta_after_step = std::min(min_theta_after_step, theta_true);
        }

        if (!x_true.allFinite() || !std::isfinite(u) || std::abs(theta_true) > M_PI) {
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

        plant.step(u, m0);
    }

    out.rms_error  = std::sqrt(sum_e2 / N_steps);
    out.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(N_steps));
    if (!out.stable) out.iae = std::max(out.iae, 1.0e6);

    const double theta_final_target = scen.theta_ref_deg * DEG2RAD;
    const double span               = theta_final_target - scen.theta0_deg * DEG2RAD;
    if (std::abs(span) > 1e-9) {
        if (span > 0.0)
            out.overshoot_pct = std::max(0.0, (max_theta_after_step - theta_final_target) / span * 100.0);
        else
            out.overshoot_pct = std::max(0.0, (theta_final_target - min_theta_after_step) / std::abs(span) * 100.0);
    }
    return out;
}

void runWcet(const bda::PlantParams& plant_p, const bda::Scenario& scen,
             std::vector<std::unique_ptr<bda::ControllerBase>>& controllers,
             const std::string& log_dir, const std::string& study_dir)
{
    std::ofstream raw(log_dir + "/wcet_nominal.csv");
    raw << "controller,step_time_us,step_index\n";
    std::ofstream summary(study_dir + "/wcet_summary.csv");
    summary << "controller,n_samples,mean_us,median_us,p99_us,wcet_us,max_us\n";

    std::mt19937 rng(42);
    FaultSpec no_fault;
    for (auto& ctrl : controllers) {
        std::vector<double> wcet_us;
        runOnce(plant_p, scen, *ctrl, no_fault, rng, &wcet_us);
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

void runMonteCarlo(const bda::PlantParams& plant_p, const bda::Scenario& scen,
                   std::vector<std::unique_ptr<bda::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/mc_summary.csv");
    csv << "controller,sample_id,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var,stable\n";

    FaultSpec no_fault;
    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        int n_unstable = 0;
        std::vector<double> iae_values;
        iae_values.reserve(kNumMcSamples);

        for (int s = 0; s < kNumMcSamples; ++s) {
            bda::PlantParams p_mc = plant_p;
            p_mc.m_bar *= robust::perturbFactor(kMcSigma, rng);
            p_mc.ms    *= robust::perturbFactor(kMcSigma, rng);
            p_mc.J     *= robust::perturbFactor(kMcSigma, rng);

            SimSummary res = runOnce(p_mc, scen, *ctrl, no_fault, rng);
            if (!res.stable) ++n_unstable;
            iae_values.push_back(res.iae);

            csv << ctrl->name() << ',' << s << ','
                << res.iae << ',' << res.rms_error << ',' << res.settle_time_s << ','
                << res.overshoot_pct << ',' << res.max_u << ',' << res.energy_var << ','
                << (res.stable ? 1 : 0) << '\n';
        }

        const MetricStats stats    = robust::computeStats(iae_values);
        const double instability_p = static_cast<double>(n_unstable) / kNumMcSamples;
        std::cout << std::fixed << std::setprecision(4)
                  << "  [MonteCarlo] " << std::setw(18) << std::left << ctrl->name()
                  << "  P(unstable)=" << instability_p
                  << "  IAE mean=" << stats.mean << " p95=" << stats.p95 << '\n';
    }
    std::cout << "  [MonteCarlo] wrote " << study_dir << "/mc_summary.csv\n";
}

void runFaultSweep(const bda::PlantParams& plant_p, const bda::Scenario& scen,
                   std::vector<std::unique_ptr<bda::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var\n";

    const double fault_time = 0.4 * scen.T_sim;
    struct Trial { FaultKind kind; std::vector<double> magnitudes; };
    const std::vector<Trial> trials = {
        {FaultKind::SensorBias,    {2.0 * DEG2RAD, 5.0 * DEG2RAD, 10.0 * DEG2RAD}},   // pitch sensor offset
        {FaultKind::SensorNoise,   {0.01, 0.03, 0.06}},                              // pitch sensor noise sigma [rad]
        {FaultKind::ActuatorLoss,  {0.20, 0.50, 0.80}},                              // fraction of u lost
        {FaultKind::ActuatorStuck, {0.0, 130.0, -130.0}},                            // frozen u [N] (~trim magnitude)
        {FaultKind::SetpointStep,  {5.0 * DEG2RAD, 10.0 * DEG2RAD, 20.0 * DEG2RAD}},  // extra theta_ref step [rad]
    };

    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        for (const auto& trial : trials) {
            for (double mag : trial.magnitudes) {
                FaultSpec fault{trial.kind, fault_time, mag, std::numeric_limits<double>::infinity()};
                SimSummary res = runOnce(plant_p, scen, *ctrl, fault, rng);
                csv << ctrl->name() << ',' << robust::faultKindName(trial.kind) << ','
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
    std::string base_dir   = (argc > 1) ? argv[1] : BOUYANCY_DRIVEN_AIRSHIP_IN_VERTICAL_PLAN_SIM_SOURCE_DIR;
    std::string plant_json = base_dir + "/config/plant_params.json";
    std::string scen_json  = base_dir + "/config/scenarios/s01_calm_step.json";
    std::string log_dir    = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    bda::PlantParams plant_p;
    bda::Scenario    scen;
    try {
        plant_p = bda::PlantParams::fromJson(plant_json);
        scen    = bda::Scenario::fromJson(scen_json);
    } catch (const std::exception& e) {
        std::cerr << "ERROR loading config: " << e.what() << '\n';
        return 1;
    }

    auto controllers = bda::makeControllers(plant_p);

    std::cout << "Bouyancy-Driven Airship in Vertical Plane - Robustness Analysis\n";
    std::cout << "================================================================\n";
    std::cout << "Nominal scenario: " << scen.id << " (" << scen.theta0_deg << " -> "
              << scen.theta_ref_deg << " deg)\n";
    std::cout << "Logs       : " << log_dir << '\n';
    std::cout << "Controllers: " << controllers.size() << '\n';
    std::cout << "MC samples : " << kNumMcSamples << "  (sigma=" << kMcSigma << ")\n\n";

    runWcet(plant_p, scen, controllers, log_dir, base_dir);
    runMonteCarlo(plant_p, scen, controllers, base_dir);
    runFaultSweep(plant_p, scen, controllers, base_dir);

    std::cout << "\nRobustness analysis complete.\n";
    return 0;
}
