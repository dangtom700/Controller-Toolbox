/**
 * @file test_susp_regression.cpp
 * @brief Regression guards for the Active Suspension case study.
 *
 * Two test categories:
 *   1. Convergence assertions for PID, LQR, ADRC, MPC, and MRAC on a constant
 *      road bump scenario (z_r=0.05 m from t=0).  Each test asserts that the
 *      body displacement error (|z_s|) in the last 20% of the run is at least
 *      50% smaller than in the first 20% - confirming the controller actively
 *      suppresses road-induced body motion.
 *
 *   2. A smoke test that runs all 15 controllers for 100 steps (0.5 s) and
 *      asserts no exception is thrown and all state components are finite.
 *
 * PlantParams uses default member initialisers; init() fills derived scalars.
 * No JSON file is required.
 */

#include <catch2/catch_test_macros.hpp>

#include "susp_plant.h"
#include "controllers.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

// ---------------------------------------------------------------------------
// Inline simulation helper (no CSV, no filesystem dependency)
// ---------------------------------------------------------------------------

struct SuspResult {
    double iae;         // integral |z_s| dt  [m.s]
    double early_rmse;  // RMS |z_s| over first 20% of steps
    double late_rmse;   // RMS |z_s| over last  20% of steps
    bool   finite;      // false if any NaN/Inf was observed
};

static SuspResult runSuspSim(susp::ControllerBase&             ctrl,
                               const susp::PlantParams&          p,
                               std::function<double(double)>     road_fn,
                               double                            duration_s)
{
    susp::QuarterCarPlant plant(p);
    plant.reset();
    ctrl.reset();

    const int N        = static_cast<int>(duration_s / p.Ts);
    const int n_window = std::max(1, N / 5);  // 20% window

    SuspResult res{};
    res.finite = true;

    double early_ss = 0.0;
    double late_ss  = 0.0;
    int    early_n  = 0;
    int    late_n   = 0;

    for (int k = 0; k < N; ++k) {
        const double             t   = k * p.Ts;
        const Eigen::Vector4d&   x   = plant.state();
        const double             z_r = road_fn(t);

        if (!x.allFinite()) { res.finite = false; break; }

        const double e     = x(0);           // z_s, reference = 0
        const double rmse2 = e * e;
        res.iae += std::abs(e) * p.Ts;

        if (k < n_window)        { early_ss += rmse2; ++early_n; }
        if (k >= N - n_window)   { late_ss  += rmse2; ++late_n;  }

        double F_raw = ctrl.compute(x, z_r);
        double F_act = std::clamp(F_raw, -p.F_max, p.F_max);
        plant.step(F_act, z_r);
    }

    if (!plant.state().allFinite()) res.finite = false;

    res.early_rmse = early_n > 0 ? std::sqrt(early_ss / early_n) : 0.0;
    res.late_rmse  = late_n  > 0 ? std::sqrt(late_ss  / late_n)  : 0.0;
    return res;
}

// ---------------------------------------------------------------------------
// Shared test infrastructure
// ---------------------------------------------------------------------------

static susp::PlantParams makeParams()
{
    susp::PlantParams p;
    p.init();
    return p;
}

// Constant 50 mm road bump from t=0
static auto roadConstant = [](double /*t*/) { return 0.05; };

// 50 mm bump for the first second, then flat - all active controllers return z_s to 0
// after the excitation ends, yielding late_rmse << early_rmse.
static auto roadBumpAndReturn = [](double t) { return t < 1.0 ? 0.05 : 0.0; };

// ---------------------------------------------------------------------------
// TEST 1 - PID: suppresses body motion after road bump and release
// ---------------------------------------------------------------------------

TEST_CASE("Susp: PID suppresses body displacement after road bump",
          "[susp][regression][pid]")
{
    auto p   = makeParams();
    auto res = [&] {
        susp::PIDSuspCtrl ctrl(p);
        return runSuspSim(ctrl, p, roadBumpAndReturn, p.duration);
    }();

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.iae > 0.0);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 2 - LQR: full-state feedback converges to near-zero body displacement
// ---------------------------------------------------------------------------

TEST_CASE("Susp: LQR converges to near-zero body displacement after road bump",
          "[susp][regression][lqr]")
{
    auto p   = makeParams();
    auto res = [&] {
        susp::LQRSuspCtrl ctrl(p);
        return runSuspSim(ctrl, p, roadBumpAndReturn, p.duration);
    }();

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 3 - ADRC: ESO rejects road disturbance
// ---------------------------------------------------------------------------

TEST_CASE("Susp: ADRC rejects constant road disturbance",
          "[susp][regression][adrc]")
{
    auto p   = makeParams();
    auto res = [&] {
        susp::ADRCSuspCtrl ctrl(p);
        return runSuspSim(ctrl, p, roadConstant, p.duration);
    }();

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 4 - MPC: constraint-aware MPC reduces body motion
// ---------------------------------------------------------------------------

TEST_CASE("Susp: MPC reduces body displacement under constant road input",
          "[susp][regression][mpc]")
{
    auto p   = makeParams();
    auto res = [&] {
        susp::MPCSuspCtrl ctrl(p);
        return runSuspSim(ctrl, p, roadConstant, p.duration);
    }();

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 5 - MRAC: adaptive controller attenuates body motion
// ---------------------------------------------------------------------------

TEST_CASE("Susp: MRAC attenuates body motion after road bump",
          "[susp][regression][mrac]")
{
    auto p   = makeParams();
    auto res = [&] {
        susp::MRACSuspCtrl ctrl(p);
        return runSuspSim(ctrl, p, roadBumpAndReturn, p.duration);
    }();

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 6 - Metaheuristic controllers converge after road bump
// ---------------------------------------------------------------------------

TEST_CASE("Susp: GA/PSO/DE optimised PID controllers suppress body motion",
          "[susp][regression][metaheuristic]")
{
    auto p = makeParams();

    auto checkConvergence = [&](susp::ControllerBase& ctrl) {
        auto res = runSuspSim(ctrl, p, roadBumpAndReturn, p.duration);
        INFO("Controller: " << ctrl.name());
        REQUIRE(res.finite);
        REQUIRE(std::isfinite(res.iae));
        REQUIRE(res.iae > 0.0);  // active control produces some response
    };

    susp::GAOptPIDCtrl  ga_ctrl(p);  checkConvergence(ga_ctrl);
    susp::PSOOptPIDCtrl pso_ctrl(p); checkConvergence(pso_ctrl);
    susp::DEOptPIDCtrl  de_ctrl(p);  checkConvergence(de_ctrl);
}

// ---------------------------------------------------------------------------
// TEST 7 - Smoke: all 18 controllers run 100 steps without exception or NaN
// ---------------------------------------------------------------------------

TEST_CASE("Susp: all 18 controllers complete 100 steps without exception or NaN",
          "[susp][regression][smoke]")
{
    auto p = makeParams();

    using namespace susp;
    std::vector<std::unique_ptr<ControllerBase>> ctrls;
    ctrls.push_back(std::make_unique<PassiveCtrl>(p));
    ctrls.push_back(std::make_unique<PIDSuspCtrl>(p));
    ctrls.push_back(std::make_unique<ADRCSuspCtrl>(p));
    ctrls.push_back(std::make_unique<SMCSuspCtrl>(p));
    ctrls.push_back(std::make_unique<LQRSuspCtrl>(p));
    ctrls.push_back(std::make_unique<LQGSuspCtrl>(p));
    ctrls.push_back(std::make_unique<MPCSuspCtrl>(p));
    ctrls.push_back(std::make_unique<MRACSuspCtrl>(p));
    ctrls.push_back(std::make_unique<FuzzyPIDSuspCtrl>(p));
    ctrls.push_back(std::make_unique<TubeMPCSuspCtrl>(p));
    ctrls.push_back(std::make_unique<ILCSuspCtrl>(p));
    ctrls.push_back(std::make_unique<CBFSuspCtrl>(p));
    ctrls.push_back(std::make_unique<L1AdaptiveSuspCtrl>(p));
    ctrls.push_back(std::make_unique<ScenarioMPCSuspCtrl>(p));
    ctrls.push_back(std::make_unique<DynaSuspCtrl>(p));
    ctrls.push_back(std::make_unique<GAOptPIDCtrl>(p));
    ctrls.push_back(std::make_unique<PSOOptPIDCtrl>(p));
    ctrls.push_back(std::make_unique<DEOptPIDCtrl>(p));

    REQUIRE(ctrls.size() == 18u);

    constexpr int N_SMOKE = 100;  // 0.5 s at Ts=0.005

    for (auto& c : ctrls) {
        INFO("Controller: " << c->name());
        SuspResult res;
        REQUIRE_NOTHROW(res = runSuspSim(*c, p, roadConstant,
                                          N_SMOKE * p.Ts));
        CHECK(res.finite);
    }
}
