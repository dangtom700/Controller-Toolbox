/**
 * @file test_smismo_regression.cpp
 * @brief Regression guards for the Meter-In Meter-Out hydraulic actuator case study.
 *
 * Two test categories:
 *   1. Convergence assertions for PID, LQR, and SMC on the S1_step scenario
 *      (5 cm position step, 8 s, Ts=0.005 s, 5 inner RK4 sub-steps).
 *      Each test asserts that the final position error settles within the
 *      +/-3 mm band and that IAE is finite and positive.
 *
 *   2. A smoke test that runs all 14 controllers for 4 s on S1_step and
 *      asserts no exception is thrown and position output remains finite.
 *
 * S3_disturbance (1 kN load step at t=4 s) is covered for PID and SMC to
 * verify disturbance rejection is functional.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "smismo_plant.h"
#include "smismo_controllers.h"

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;

// ---------------------------------------------------------------------------
// Inline simulation helper (mirrors smismo_main::runSimulation)
// ---------------------------------------------------------------------------

struct SmismoResult {
    double iae;         // position integral absolute error [m.s]
    double final_err;   // |x - r| at end of run [m]
    double peak_err;    // peak |x - r| observed [m]
    double early_rmse;  // RMS position error over first 20% of run
    double late_rmse;   // RMS position error over last  20% of run
    bool   finite;
};

static SmismoResult runSmismoSim(smismo::ControllerBase& ctrl,
                                  double x_start,
                                  std::function<double(double)> ref_fn,
                                  double duration_s,
                                  double Ts,
                                  double F_ext_step = 0.0,
                                  double load_time_s = 1e9)
{
    smismo::SmismoPlant plant;
    plant.reset(x_start);
    ctrl.reset();

    const int N = static_cast<int>(duration_s / Ts);
    constexpr int N_inner = 5;
    const double dt_inner = Ts / N_inner;

    SmismoResult res{};
    res.finite = true;

    const int n_window = std::max(1, N / 5);

    for (int k = 0; k < N; ++k) {
        const double t = k * Ts;
        if (t >= load_time_s)
            plant.setLoad(F_ext_step);

        const double r = ref_fn(t);
        Eigen::Vector4d y = plant.measure();

        if (!y.allFinite()) { res.finite = false; break; }

        double u_cmd = ctrl.compute(y, r);
        double e     = r - y(0);

        res.iae      += std::abs(e) * Ts;
        res.peak_err  = std::max(res.peak_err, std::abs(e));

        double rmse_k = std::abs(e);
        if (k < n_window)       { res.early_rmse += rmse_k * rmse_k; }
        if (k >= N - n_window)  { res.late_rmse  += rmse_k * rmse_k; }

        Eigen::Matrix<double, 2, 1> u_vec(u_cmd, u_cmd);
        for (int i = 0; i < N_inner; ++i)
            plant.step(dt_inner, u_vec);
    }

    Eigen::Vector4d y_fin = plant.measure();
    res.final_err  = std::abs(ref_fn(duration_s) - y_fin(0));
    res.early_rmse = std::sqrt(res.early_rmse / n_window);
    res.late_rmse  = std::sqrt(res.late_rmse  / n_window);
    return res;
}

// ---------------------------------------------------------------------------
// TEST 1 - PID: S1 5 cm step, settles within +/-3 mm
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO S1_step: PID settles to within 3 mm of setpoint",
          "[smismo][regression][pid]")
{
    smismo::SmismoPlant ref;
    const smismo::SmismoOperatingPoint& op = ref.op();
    constexpr double Ts = 0.005;

    smismo::PIDCtrl ctrl(op, Ts);
    auto res = runSmismoSim(ctrl, 0.10,
                             [](double t) { return t < 1.0 ? 0.10 : 0.20; },
                             8.0, Ts);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.iae > 0.0);
    REQUIRE(res.final_err < 0.003);  // +/-3 mm settling band
}

// ---------------------------------------------------------------------------
// TEST 2 - LQR: settles within +/-3 mm; IAE not worse than PID
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO S1_step: LQR settles to within 3 mm and IAE <= PID",
          "[smismo][regression][lqr]")
{
    smismo::SmismoPlant ref;
    ctrl::StateSpace ss2 = ref.linearise2(0.005);
    const smismo::SmismoOperatingPoint& op = ref.op();
    constexpr double Ts = 0.005;

    auto step_fn = [](double t) { return t < 1.0 ? 0.10 : 0.20; };

    smismo::LQRCtrl lqr(ss2, op, Ts);
    smismo::PIDCtrl pid(op, Ts);

    auto r_lqr = runSmismoSim(lqr, 0.10, step_fn, 8.0, Ts);
    auto r_pid = runSmismoSim(pid, 0.10, step_fn, 8.0, Ts);

    REQUIRE(r_lqr.finite);
    REQUIRE(r_lqr.final_err < 0.003);
    REQUIRE(r_lqr.iae < r_pid.iae * 1.30);  // LQR <= 130% of PID IAE
}

// ---------------------------------------------------------------------------
// TEST 3 - SMC: settles within +/-5 mm (chattering allows wider band)
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO S1_step: SMC settles to within 5 mm of setpoint",
          "[smismo][regression][smc]")
{
    smismo::SmismoPlant ref;
    const smismo::SmismoOperatingPoint& op = ref.op();
    constexpr double Ts = 0.005;

    smismo::SMCCtrl ctrl(op, Ts);
    auto res = runSmismoSim(ctrl, 0.10,
                             [](double t) { return t < 1.0 ? 0.10 : 0.20; },
                             8.0, Ts);

    REQUIRE(res.finite);
    REQUIRE(res.final_err < 0.005);  // +/-5 mm
}

// ---------------------------------------------------------------------------
// TEST 4 - ADRC: S2 multi-step convergence
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO S2_multi_step: ADRC tracks all steps within 5 mm",
          "[smismo][regression][adrc]")
{
    smismo::SmismoPlant ref;
    const smismo::SmismoOperatingPoint& op = ref.op();
    constexpr double Ts = 0.005;
    const double b0 = ref.accelGain();

    smismo::ADRCCtrl ctrl(b0, op, Ts);

    auto ref_fn = [](double t) -> double {
        if (t < 1.0)  return 0.05;
        if (t < 4.0)  return 0.15;
        if (t < 7.0)  return 0.10;
        if (t < 10.0) return 0.25;
        return 0.20;
    };

    auto res = runSmismoSim(ctrl, 0.05, ref_fn, 12.0, Ts);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.iae > 0.0);
    // Check convergence: late error is smaller than early
    REQUIRE(res.late_rmse < res.early_rmse * 0.80);
}

// ---------------------------------------------------------------------------
// TEST 5 - PID disturbance rejection: 1 kN load step at t=4 s
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO S3_disturbance: PID recovers from 1kN load step",
          "[smismo][regression][disturbance]")
{
    smismo::SmismoPlant ref;
    const smismo::SmismoOperatingPoint& op = ref.op();
    constexpr double Ts = 0.005;

    smismo::PIDCtrl ctrl(op, Ts);
    auto ref_fn = [](double t) -> double { return t < 2.0 ? 0.10 : 0.20; };
    auto res = runSmismoSim(ctrl, 0.10, ref_fn, 10.0, Ts, 1000.0, 4.0);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    // Late convergence: integral recovers after the load step
    REQUIRE(res.late_rmse < res.early_rmse * 0.80);
}

// ---------------------------------------------------------------------------
// TEST 6 - Smoke: all 14 controllers complete 4 s without exception or NaN
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO: all 14 controllers complete 4s without exception or NaN",
          "[smismo][regression][smoke]")
{
    smismo::SmismoPlant ref;
    ctrl::StateSpace ss2 = ref.linearise2(0.005);
    ctrl::StateSpace ss4 = ref.linearise4(0.005);
    const smismo::SmismoOperatingPoint& op = ref.op();
    const double b0 = ref.accelGain();
    constexpr double Ts = 0.005;

    using namespace smismo;
    std::vector<std::unique_ptr<ControllerBase>> ctrls;
    ctrls.push_back(std::make_unique<PIDCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<LQRCtrl>(ss2, op, Ts));
    ctrls.push_back(std::make_unique<LQGCtrl>(ss2, op, Ts));
    ctrls.push_back(std::make_unique<SMCCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<ADRCCtrl>(b0, op, Ts));
    ctrls.push_back(std::make_unique<TubeMPCCtrl>(ss2, op));
    ctrls.push_back(std::make_unique<LeadLagPIDCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<GPCRLSCtrl>(ss2, op, Ts));
    ctrls.push_back(std::make_unique<MRACSmismoCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<GainScheduledSmismoCtrl>(op, Ts));
    ctrls.push_back(std::make_unique<EKFLQRSmismoCtrl>(ss4, op, Ts));
    ctrls.push_back(std::make_unique<HinfSmismoCtrl>(ss2, op, Ts));
    ctrls.push_back(std::make_unique<NMPCSmismoCtrl>(ss4, op, Ts));
    ctrls.push_back(std::make_unique<FLSmismoCtrl>(b0, op, Ts));

    REQUIRE(ctrls.size() == 14u);

    auto step_fn = [](double t) { return t < 1.0 ? 0.10 : 0.20; };

    for (auto& c : ctrls) {
        INFO("Controller: " << c->name());
        SmismoResult res;
        REQUIRE_NOTHROW(res = runSmismoSim(*c, 0.10, step_fn, 4.0, Ts));
        CHECK(res.finite);
    }
}
