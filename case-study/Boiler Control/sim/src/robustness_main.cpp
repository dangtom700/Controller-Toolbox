#include "boiler_plant.h"
#include "linearizer.h"
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

// Robustness analysis for the Boiler Control case study: WCET profiling,
// Monte Carlo (operating-point perturbation), and fault sweep (sensor/
// actuator/setpoint faults). Reuses the real nonlinear BoilerTurbine + every
// controller in the roster via a lightweight in-memory simulation loop with
// no per-step file I/O.
//
// Unlike every other case study, Boiler has no PlantParams struct -- the
// Bell-Astrom coefficients are hardcoded literals in BoilerTurbine::update().
// The only "parameter" data is the 3 OperatingPoint equilibria (op_A/B/C).
// Monte Carlo therefore perturbs a local copy of the nominal OperatingPoint's
// equilibrium fields (modeling "the real plant sits near a slightly
// different load point than the one the controller was linearised around"),
// NOT physics coefficients. Controllers are always built from the ORIGINAL
// op_B/ss_B; only the simulated BoilerTurbine is initialised at the
// perturbed equilibrium.
//
// MIMO fault decision: fault channel 0 (drum pressure / fuel valve) only,
// holding channels 1-2 fault-free, to keep trial count comparable to the
// SISO studies rather than a 3x channel sweep.
//
// Output, matching the schema/location tools/generate_report.py already reads
// (case-study/<Study>/{mc_summary,fault_sweep,wcet_summary}.csv) so the report
// pipeline picks these up with no changes:
//   mc_summary.csv, fault_sweep.csv, wcet_summary.csv   (study root)
//   logs/wcet_nominal.csv                                (raw per-step timing)

#ifndef BOILER_SIM_SOURCE_DIR
#define BOILER_SIM_SOURCE_DIR "."
#endif

using robust::FaultKind;
using robust::FaultSpec;
using robust::MetricStats;
using robust::SimSummary;

namespace {

// Truncated relative to the default ~3600s scenario length: captures
// pressure/power/level settling at far lower cost across 30 MC samples x 27
// controllers (several of which solve a QP/MHE per step).
constexpr double kAnalysisDuration = 600.0;  // [s]
constexpr int    kNumMcSamples     = 30;
constexpr double kMcSigma          = 0.15;   // 15% relative perturbation

// Runs one (controller, plant-equilibrium, fault) trial through the real
// nonlinear Bell-Astrom boiler-turbine with no per-step file I/O. If
// wcet_us is non-null, records the wall-clock time of every
// controller.compute() call. `op_plant` initialises the PLANT (possibly a
// perturbed equilibrium); `op_nom` is always the controller's own design
// point, used for the y0/u0 deviation reference.
SimSummary runOnce(const boiler::OperatingPoint& op_plant,
                    const boiler::OperatingPoint& op_nom,
                    boiler::ControllerBase&       ctrl,
                    const FaultSpec&               fault,
                    std::mt19937&                   rng,
                    std::vector<double>*            wcet_us = nullptr)
{
    using Clock = std::chrono::steady_clock;

    const double dt      = boiler::BoilerTurbine::Ts;
    const int    N_steps = static_cast<int>(std::lround(kAnalysisDuration / dt));

    boiler::BoilerTurbine bt;
    bt.initAt(op_plant);
    ctrl.reset();

    const Eigen::Vector3d u0(op_nom.u1, op_nom.u2, op_nom.u3);
    const Eigen::Vector3d y0(op_nom.y1, op_nom.y2, op_nom.y3);

    SimSummary out;
    double sum_e2      = 0.0;
    double sum_u       = 0.0;
    double sum_u2      = 0.0;
    double e0_abs      = -1.0;
    int    in_band_run = 0;
    bool   settled     = false;
    double max_y = -1e9, min_y = 1e9, final_ref = 0.0;
    constexpr double kSettleBand       = 0.02;
    constexpr int    kSettleHysteresis = 10;

    for (int k = 0; k < N_steps; ++k) {
        const double t = k * dt;

        // Regulation to the nominal operating point by default; SetpointStep
        // faults perturb channel 0 only.
        Eigen::Vector3d ref_dy(0.0, 0.0, 0.0);
        ref_dy(0) = robust::applySetpointFault(0.0, fault, t);
        final_ref = ref_dy(0);

        const Eigen::Vector3d y_true  = bt.measureOutputs();
        const Eigen::Vector3d dy_true = y_true - y0;

        // Sensor fault: perturb only the measured pressure deviation (channel 0).
        Eigen::Vector3d dy_meas = dy_true;
        dy_meas(0) = robust::applySensorFault(dy_true(0), fault, t, rng);

        Eigen::Vector3d du;
        if (wcet_us) {
            auto t0 = Clock::now();
            du      = ctrl.compute(ref_dy, dy_meas);
            auto t1 = Clock::now();
            wcet_us->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        } else {
            du = ctrl.compute(ref_dy, dy_meas);
        }

        // Actuator fault on the fuel-valve increment (channel 0), then the
        // same absolute-valve clamp as simulation_runner.cpp.
        du(0) = robust::applyActuatorFault(du(0), fault, t);

        Eigen::Vector3d u_abs;
        for (int i = 0; i < 3; ++i) u_abs(i) = std::clamp(u0(i) + du(i), 0.0, 1.0);

        bt.setControls(u_abs(0), u_abs(1), u_abs(2));
        bt.constrainValveRate();
        bt.update();

        const double error  = ref_dy(0) - dy_true(0);
        out.iae             += std::abs(error) * dt;
        sum_e2               += error * error;
        out.max_abs_error     = std::max(out.max_abs_error, std::abs(error));
        out.max_u              = std::max(out.max_u, std::max({std::abs(bt.du1), std::abs(bt.du2), std::abs(bt.du3)}));
        sum_u                  += bt.du1;
        sum_u2                  += bt.du1 * bt.du1;
        max_y = std::max(max_y, dy_true(0));
        min_y = std::min(min_y, dy_true(0));

        if (!std::isfinite(bt.x1) || !std::isfinite(bt.x2) || !std::isfinite(bt.x3) ||
            !std::isfinite(du(0)) || bt.x1 < 0.0 || bt.x1 > 250.0 || bt.x3 < 0.0 || bt.x3 > 1000.0) {
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
    }

    out.rms_error  = std::sqrt(sum_e2 / N_steps);
    out.energy_var = robust::varianceFromSums(sum_u, sum_u2, static_cast<double>(N_steps));
    if (!out.stable) out.iae = std::max(out.iae, 1.0e6);

    if (std::abs(final_ref) > 1e-9 && max_y > final_ref) {
        out.overshoot_pct = (max_y - final_ref) / std::abs(final_ref) * 100.0;
    }
    return out;
}

std::vector<std::unique_ptr<boiler::ControllerBase>> buildControllers(const ctrl::StateSpace& ss,
                                                                       const boiler::OperatingPoint& op) {
    std::vector<std::unique_ptr<boiler::ControllerBase>> v;
    v.push_back(std::make_unique<boiler::PIDController>(ss, op));
    v.push_back(std::make_unique<boiler::LQRController>(ss, op));
    v.push_back(std::make_unique<boiler::LQGController>(ss, op));
    v.push_back(std::make_unique<boiler::MPCController>(ss, op));
    v.push_back(std::make_unique<boiler::SMCController>(ss, op));
    v.push_back(std::make_unique<boiler::ESCController>(ss, op));
    v.push_back(std::make_unique<boiler::ADRCController>(ss, op));
    v.push_back(std::make_unique<boiler::LeadLagPIDController>(ss, op));
    v.push_back(std::make_unique<boiler::SmithPredictorController>(ss, op));
    v.push_back(std::make_unique<boiler::GPCController>(ss, op));
    v.push_back(std::make_unique<boiler::EKFLQRController>(ss, op));
    v.push_back(std::make_unique<boiler::UKFLQRController>(ss, op));
    v.push_back(std::make_unique<boiler::FuzzyPIDController>(ss, op));
    v.push_back(std::make_unique<boiler::FuzzySupMPCController>(ss, op));
    v.push_back(std::make_unique<boiler::SupervisoryStackController>(ss, op));
    v.push_back(std::make_unique<boiler::AdditiveStackController>(ss, op));
    v.push_back(std::make_unique<boiler::WeightedStackController>(ss, op));
    v.push_back(std::make_unique<boiler::RepetitiveCtrl>(ss, op));
    v.push_back(std::make_unique<boiler::MRACBoilerCtrl>(ss, op));
    v.push_back(std::make_unique<boiler::HinfBoilerCtrl>(ss, op));
    v.push_back(std::make_unique<boiler::AdaptiveSPBoilerCtrl>(ss, op));
    v.push_back(std::make_unique<boiler::NMPCBoilerCtrl>(ss, op));
    v.push_back(std::make_unique<boiler::FLBoilerCtrl>(ss, op));
    v.push_back(std::make_unique<boiler::MHELQRBoilerCtrl>(ss, op));
    v.push_back(std::make_unique<boiler::LPVGSBoilerCtrl>(ss, op));
    v.push_back(std::make_unique<boiler::SubspaceIDLQGBoilerCtrl>(ss, op));
    v.push_back(std::make_unique<boiler::AutoGSBoilerCtrl>(ss, op));
    return v;
}

void runWcet(const boiler::OperatingPoint& op_nom,
             std::vector<std::unique_ptr<boiler::ControllerBase>>& controllers,
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
        runOnce(op_nom, op_nom, *ctrl, no_fault, rng, &wcet_us);
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

void runMonteCarlo(const boiler::OperatingPoint& op_nom,
                   std::vector<std::unique_ptr<boiler::ControllerBase>>& controllers,
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
            boiler::OperatingPoint op_mc = op_nom;
            op_mc.x1 *= robust::perturbFactor(kMcSigma, rng);
            op_mc.x3 *= robust::perturbFactor(kMcSigma, rng);
            op_mc.u1 *= robust::perturbFactor(kMcSigma, rng);
            op_mc.u3 *= robust::perturbFactor(kMcSigma, rng);

            SimSummary res = runOnce(op_mc, op_nom, *ctrl, no_fault, rng);
            if (!res.stable) ++n_unstable;
            iae_values.push_back(res.iae);

            csv << "Boiler," << ctrl->name() << ',' << s << ','
                << res.iae << ',' << res.rms_error << ',' << res.settle_time_s << ','
                << res.overshoot_pct << ',' << res.max_u << ',' << res.energy_var << ','
                << (res.stable ? 1 : 0) << '\n';
        }

        const MetricStats stats = robust::computeStats(iae_values);
        const double instability_p = static_cast<double>(n_unstable) / kNumMcSamples;
        std::cout << std::fixed << std::setprecision(4)
                  << "  [MonteCarlo] " << std::setw(18) << std::left << ctrl->name()
                  << "  P(unstable)=" << instability_p
                  << "  IAE mean=" << stats.mean << " p95=" << stats.p95 << '\n';
    }
    std::cout << "  [MonteCarlo] wrote " << study_dir << "/mc_summary.csv\n";
}

void runFaultSweep(const boiler::OperatingPoint& op_nom,
                   std::vector<std::unique_ptr<boiler::ControllerBase>>& controllers,
                   const std::string& study_dir)
{
    std::ofstream csv(study_dir + "/fault_sweep.csv");
    csv << "study,controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var\n";

    const double fault_time = 0.4 * kAnalysisDuration;
    struct Trial { FaultKind kind; std::vector<double> magnitudes; };
    const std::vector<Trial> trials = {
        {FaultKind::SensorBias,    {1.0, 3.0, 6.0}},          // [bar] pressure sensor offset
        {FaultKind::SensorNoise,   {0.2, 0.5, 1.0}},          // [bar] pressure sensor noise sigma
        {FaultKind::ActuatorLoss,  {0.20, 0.50, 0.80}},       // fraction of du1 lost
        {FaultKind::ActuatorStuck, {0.0, 0.007, -0.007}},     // frozen du1 (zero / rate-limit rails)
        {FaultKind::SetpointStep,  {2.0, 5.0, 10.0}},         // [bar] pressure setpoint step
    };

    for (auto& ctrl : controllers) {
        std::mt19937 rng(42);
        for (const auto& trial : trials) {
            for (double mag : trial.magnitudes) {
                FaultSpec fault{trial.kind, fault_time, mag, std::numeric_limits<double>::infinity()};
                SimSummary res = runOnce(op_nom, op_nom, *ctrl, fault, rng);
                csv << "Boiler," << ctrl->name() << ',' << robust::faultKindName(trial.kind) << ','
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
    std::string base_dir = (argc > 1) ? argv[1] : BOILER_SIM_SOURCE_DIR;
    std::string log_dir  = base_dir + "/logs";
    std::filesystem::create_directories(log_dir);

    const boiler::OperatingPoint& op_nom = boiler::op_B;
    ctrl::StateSpace ss = boiler::linearize(op_nom, boiler::BoilerTurbine::Ts);

    auto controllers = buildControllers(ss, op_nom);

    std::cout << "Boiler Control - Robustness Analysis\n";
    std::cout << "=======================================\n";
    std::cout << "Operating point: " << op_nom.label << " (x1=" << op_nom.x1 << " bar)\n";
    std::cout << "Logs       : " << log_dir << '\n';
    std::cout << "Controllers: " << controllers.size() << '\n';
    std::cout << "MC samples : " << kNumMcSamples << "  (sigma=" << kMcSigma << ")\n\n";

    runWcet(op_nom, controllers, log_dir, base_dir);
    runMonteCarlo(op_nom, controllers, base_dir);
    runFaultSweep(op_nom, controllers, base_dir);

    std::cout << "\nRobustness analysis complete.\n";
    return 0;
}
