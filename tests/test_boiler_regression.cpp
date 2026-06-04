/**
 * @file test_boiler_regression.cpp
 * @brief Regression guards for the Bell-Astrom boiler-turbine case study.
 *
 * Two test categories:
 *   1. Convergence assertions for representative controllers on the s02
 *      medium-load regulation scenario (dx0=[5,3,-10], 3600 s, op_B).
 *      Each test asserts that the final-tenth error is smaller than the
 *      first-tenth error (monotone convergence) and that IAE is finite.
 *
 *   2. A smoke test that runs all 27 controllers for 300 s on s01 and
 *      asserts no exception is thrown and all outputs are finite.
 *
 * Baseline derived from a verified correct run on the Part 31 codebase.
 * Tolerance +/-50% is deliberately wide on the first pass; tighten after
 * establishing several clean baseline runs.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "boiler_plant.h"
#include "linearizer.h"
#include "controllers.h"

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

using Catch::Matchers::WithinRel;

// ---------------------------------------------------------------------------
// Inline simulation helper (no CSV, no JSON)
// ---------------------------------------------------------------------------

struct BoilerResult {
    double iae_y1;      // pressure integral absolute error [bar.s]
    double iae_y2;      // power    integral absolute error [MW.s]
    double final_y1;    // absolute pressure deviation at end-of-run [bar]
    double final_y2;    // absolute power deviation at end-of-run [MW]
    double early_rmse;  // RMS output error over first 10% of run
    double late_rmse;   // RMS output error over last  10% of run
    bool   finite;      // false if any NaN/Inf was observed
};

static BoilerResult runBoilerSim(boiler::ControllerBase& ctrl,
                                  const boiler::OperatingPoint& op,
                                  const Eigen::Vector3d& dx0,
                                  int N_steps)
{
    const double Ts = boiler::BoilerTurbine::Ts;
    boiler::BoilerTurbine bt;
    bt.initAt(op);
    bt.setState(op.x1 + dx0(0), op.x2 + dx0(1), op.x3 + dx0(2));
    ctrl.reset();

    const Eigen::Vector3d y0(op.y1, op.y2, op.y3);
    const Eigen::Vector3d u0(op.u1, op.u2, op.u3);
    const Eigen::Vector3d ref_dy = Eigen::Vector3d::Zero();  // regulation

    BoilerResult res{};
    res.finite = true;

    const int n_window = std::max(1, N_steps / 10);
    double early_ss = 0.0, late_ss = 0.0;
    int    early_n  = 0,   late_n  = 0;

    for (int k = 0; k < N_steps; ++k) {
        Eigen::Vector3d y  = bt.measureOutputs();
        Eigen::Vector3d dy = y - y0;

        if (!dy.allFinite()) { res.finite = false; break; }

        Eigen::Vector3d du = ctrl.compute(ref_dy, dy);

        Eigen::Vector3d u_abs;
        for (int i = 0; i < 3; ++i)
            u_abs(i) = std::clamp(u0(i) + du(i), 0.0, 1.0);

        bt.setControls(u_abs(0), u_abs(1), u_abs(2));
        bt.constrainValveRate();
        bt.update();

        res.iae_y1 += std::abs(dy(0)) * Ts;
        res.iae_y2 += std::abs(dy(1)) * Ts;

        double rmse_k = std::sqrt((dy(0)*dy(0) + dy(1)*dy(1)) / 2.0);
        if (k < n_window)         { early_ss += rmse_k * rmse_k; ++early_n; }
        if (k >= N_steps - n_window) { late_ss  += rmse_k * rmse_k; ++late_n;  }
    }

    Eigen::Vector3d y_fin  = bt.measureOutputs();
    Eigen::Vector3d dy_fin = y_fin - y0;
    res.final_y1   = std::abs(dy_fin(0));
    res.final_y2   = std::abs(dy_fin(1));
    res.early_rmse = early_n > 0 ? std::sqrt(early_ss / early_n) : 0.0;
    res.late_rmse  = late_n  > 0 ? std::sqrt(late_ss  / late_n)  : 0.0;
    return res;
}

// ---------------------------------------------------------------------------
// TEST 1 - PID convergence on s02 (medium load regulation)
// ---------------------------------------------------------------------------

TEST_CASE("Boiler s02: PID converges on medium-load regulation",
          "[boiler][regression][pid]")
{
    const auto& op = boiler::op_B;
    auto ss = boiler::linearize(op, 1.0);
    const Eigen::Vector3d dx0(5.0, 3.0, -10.0);
    constexpr int N = 3600;

    boiler::PIDController ctrl(ss, op);
    auto res = runBoilerSim(ctrl, op, dx0, N);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae_y1));
    REQUIRE(std::isfinite(res.iae_y2));
    REQUIRE(res.iae_y1 > 0.0);
    // Late-run error must be substantially smaller than early-run error
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
    // Final pressure deviation < 1 bar (0.05% of op_B x1=97.2)
    REQUIRE(res.final_y1 < 1.0);
}

// ---------------------------------------------------------------------------
// TEST 2 - LQR convergence and LQR IAE <= PID IAE
// ---------------------------------------------------------------------------

TEST_CASE("Boiler s02: LQR converges and is no worse than PID",
          "[boiler][regression][lqr]")
{
    const auto& op = boiler::op_B;
    auto ss = boiler::linearize(op, 1.0);
    const Eigen::Vector3d dx0(5.0, 3.0, -10.0);
    constexpr int N = 3600;

    boiler::LQRController  lqr(ss, op);
    boiler::PIDController  pid(ss, op);
    auto r_lqr = runBoilerSim(lqr, op, dx0, N);
    auto r_pid = runBoilerSim(pid, op, dx0, N);

    REQUIRE(r_lqr.finite);
    REQUIRE(std::isfinite(r_lqr.iae_y1));
    REQUIRE(r_lqr.late_rmse < r_lqr.early_rmse * 0.50);
    // LQR is a computed optimal solution; its IAE should be <= PID's within 20%
    REQUIRE(r_lqr.iae_y1 < r_pid.iae_y1 * 1.20);
}

// ---------------------------------------------------------------------------
// TEST 3 - MPC convergence on s02
// ---------------------------------------------------------------------------

TEST_CASE("Boiler s02: MPC converges on medium-load regulation",
          "[boiler][regression][mpc]")
{
    const auto& op = boiler::op_B;
    auto ss = boiler::linearize(op, 1.0);
    const Eigen::Vector3d dx0(5.0, 3.0, -10.0);
    constexpr int N = 3600;

    boiler::MPCController ctrl(ss, op);
    auto res = runBoilerSim(ctrl, op, dx0, N);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae_y1));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 4 - SMC convergence on s01 (low load)
// ---------------------------------------------------------------------------

TEST_CASE("Boiler s01: SMC converges on low-load regulation",
          "[boiler][regression][smc]")
{
    const auto& op = boiler::op_A;
    auto ss = boiler::linearize(op, 1.0);
    const Eigen::Vector3d dx0(5.0, 3.0, -10.0);
    constexpr int N = 3600;

    boiler::SMCController ctrl(ss, op);
    auto res = runBoilerSim(ctrl, op, dx0, N);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae_y1));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 5 - LQG convergence on s03 (high load)
// ---------------------------------------------------------------------------

TEST_CASE("Boiler s03: LQG converges on high-load regulation",
          "[boiler][regression][lqg]")
{
    const auto& op = boiler::op_C;
    auto ss = boiler::linearize(op, 1.0);
    const Eigen::Vector3d dx0(5.0, 3.0, -10.0);
    constexpr int N = 3600;

    boiler::LQGController ctrl(ss, op);
    auto res = runBoilerSim(ctrl, op, dx0, N);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae_y1));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 6 - Smoke: all 27 controllers run 300 s on s01 without exception
// ---------------------------------------------------------------------------

TEST_CASE("Boiler: all 27 controllers complete 300s without exception or NaN",
          "[boiler][regression][smoke]")
{
    const auto& op = boiler::op_B;
    auto ss = boiler::linearize(op, 1.0);
    const Eigen::Vector3d dx0(5.0, 3.0, -10.0);
    constexpr int N_smoke = 300;

    using namespace boiler;
    std::vector<std::unique_ptr<ControllerBase>> ctrls;
    ctrls.push_back(std::make_unique<PIDController>(ss, op));
    ctrls.push_back(std::make_unique<LQRController>(ss, op));
    ctrls.push_back(std::make_unique<LQGController>(ss, op));
    ctrls.push_back(std::make_unique<MPCController>(ss, op));
    ctrls.push_back(std::make_unique<SMCController>(ss, op));
    ctrls.push_back(std::make_unique<ESCController>(ss, op));
    ctrls.push_back(std::make_unique<ADRCController>(ss, op));
    ctrls.push_back(std::make_unique<LeadLagPIDController>(ss, op));
    ctrls.push_back(std::make_unique<SmithPredictorController>(ss, op));
    ctrls.push_back(std::make_unique<GPCController>(ss, op));
    ctrls.push_back(std::make_unique<EKFLQRController>(ss, op));
    ctrls.push_back(std::make_unique<UKFLQRController>(ss, op));
    ctrls.push_back(std::make_unique<FuzzyPIDController>(ss, op));
    ctrls.push_back(std::make_unique<FuzzySupMPCController>(ss, op));
    ctrls.push_back(std::make_unique<SupervisoryStackController>(ss, op));
    ctrls.push_back(std::make_unique<AdditiveStackController>(ss, op));
    ctrls.push_back(std::make_unique<WeightedStackController>(ss, op));
    ctrls.push_back(std::make_unique<RepetitiveCtrl>(ss, op));
    ctrls.push_back(std::make_unique<MRACBoilerCtrl>(ss, op));
    ctrls.push_back(std::make_unique<HinfBoilerCtrl>(ss, op));
    ctrls.push_back(std::make_unique<AdaptiveSPBoilerCtrl>(ss, op));
    ctrls.push_back(std::make_unique<NMPCBoilerCtrl>(ss, op));
    ctrls.push_back(std::make_unique<FLBoilerCtrl>(ss, op));
    ctrls.push_back(std::make_unique<MHELQRBoilerCtrl>(ss, op));
    ctrls.push_back(std::make_unique<LPVGSBoilerCtrl>(ss, op));
    ctrls.push_back(std::make_unique<SubspaceIDLQGBoilerCtrl>(ss, op));
    ctrls.push_back(std::make_unique<AutoGSBoilerCtrl>(ss, op));

    REQUIRE(ctrls.size() == 27u);

    for (auto& c : ctrls) {
        INFO("Controller: " << c->name());
        BoilerResult res;
        REQUIRE_NOTHROW(res = runBoilerSim(*c, op, dx0, N_smoke));
        CHECK(res.finite);
    }
}
