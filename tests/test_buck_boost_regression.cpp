/**
 * @file test_buck_boost_regression.cpp
 * @brief Regression guards for the Non-Inverting Buck-Boost Converter case study.
 *
 * Two test categories:
 *   1. Convergence assertions for PI_Buck, FuzzyPID_Buck, ADRC, MPC, and
 *      TLCS_FuzzyPI on the s01 buck scenario (V_in=10V, V_ref=8V).
 *      Each test asserts that the output-voltage error in the last 10% of
 *      the run is at least 50% smaller than in the first 10%.
 *
 *   2. A smoke test that runs all 12 controllers for 300 steps (~6 ms) and
 *      asserts no exception is thrown and v_C is finite.
 *
 * The converter Ts=20 mus is very fast; 3000 steps = 60 ms contains many
 * time constants (L/R = 25 mus), so all PI/ADRC/MPC controllers settle well
 * within the test window.
 *
 * PlantParams uses default member initialisers - no JSON required.
 */

#include <catch2/catch_test_macros.hpp>

#include "buck_boost_plant.h"
#include "controllers.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

// ---------------------------------------------------------------------------
// Inline simulation helper
// ---------------------------------------------------------------------------

struct ConvResult {
    double iae;         // integral |v_ref - v_C| dt  [V.s]
    double early_rmse;  // RMS voltage error over first 10% of steps
    double late_rmse;   // RMS voltage error over last  10% of steps
    bool   finite;      // false if any NaN/Inf was observed
};

static ConvResult runConvSim(conv::ControllerBase&      ctrl,
                               const conv::PlantParams&   p,
                               double                     V_ref,
                               double                     V_in,
                               int                        N_steps)
{
    conv::BuckBoostPlant plant(p);
    plant.reset();
    ctrl.reset();

    const int n_window = std::max(1, N_steps / 10);  // 10% window

    ConvResult res{};
    res.finite = true;

    double early_ss = 0.0;
    double late_ss  = 0.0;
    int    early_n  = 0;
    int    late_n   = 0;

    // Mode state (hysteresis, mirrors simulation_runner.cpp)
    conv::ConvMode mode = conv::ConvMode::BUCK;

    for (int k = 0; k < N_steps; ++k) {
        const Eigen::Vector2d& x   = plant.state();
        const double           v_C = x(1);

        if (!std::isfinite(v_C)) { res.finite = false; break; }

        const double e     = V_ref - v_C;
        const double rmse2 = e * e;
        res.iae += std::abs(e) * p.Ts;

        if (k < n_window)          { early_ss += rmse2; ++early_n; }
        if (k >= N_steps - n_window) { late_ss  += rmse2; ++late_n;  }

        // Update mode (hysteresis, same logic as simulation_runner.cpp)
        if (V_ref > V_in + p.hysteresis)
            mode = conv::ConvMode::BOOST;
        else if (V_ref < V_in - p.hysteresis)
            mode = conv::ConvMode::BUCK;

        double d = ctrl.compute(x, V_ref, V_in);
        d        = std::clamp(d, p.d_min, p.d_max);
        plant.step(d, mode, V_in);
    }

    if (!std::isfinite(plant.state()(1))) res.finite = false;

    res.early_rmse = early_n > 0 ? std::sqrt(early_ss / early_n) : 0.0;
    res.late_rmse  = late_n  > 0 ? std::sqrt(late_ss  / late_n)  : 0.0;
    return res;
}

// ---------------------------------------------------------------------------
// Shared test parameters - s01_buck: V_in=10V, V_ref=8V
// ---------------------------------------------------------------------------

static const double V_IN    = 10.0;
static const double V_REF   = 8.0;
static const int    N_CONV  = 3000;  // 60 ms at Ts=20 mus
static const int    N_SMOKE = 300;   //  6 ms

// ---------------------------------------------------------------------------
// TEST 1 - PI_Buck: regulates v_out to 8 V in buck mode
// ---------------------------------------------------------------------------

TEST_CASE("BuckBoost s01: PI_Buck regulates output voltage to 8 V",
          "[buck_boost][regression][pi_buck]")
{
    conv::PlantParams p;
    conv::PIBuckCtrl  ctrl(p);
    auto res = runConvSim(ctrl, p, V_REF, V_IN, N_CONV);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.iae > 0.0);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 2 - FuzzyPID_Buck: Mamdani FuzzyPID converges in buck mode
// ---------------------------------------------------------------------------

TEST_CASE("BuckBoost s01: FuzzyPID_Buck converges in buck mode",
          "[buck_boost][regression][fuzzypid_buck]")
{
    conv::PlantParams       p;
    conv::FuzzyPIDBuckCtrl  ctrl(p);
    auto res = runConvSim(ctrl, p, V_REF, V_IN, N_CONV);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 3 - ADRC: ESO treats converter nonlinearity as total disturbance
// ---------------------------------------------------------------------------

TEST_CASE("BuckBoost s01: ADRC converges in buck mode",
          "[buck_boost][regression][adrc]")
{
    conv::PlantParams  p;
    conv::ADRCConvCtrl ctrl(p);
    auto res = runConvSim(ctrl, p, V_REF, V_IN, N_CONV);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 4 - MPC: DiscreteMPC on linearised buck-mode SS model converges
// ---------------------------------------------------------------------------

TEST_CASE("BuckBoost s01: MPC converges on linearised buck model",
          "[buck_boost][regression][mpc]")
{
    conv::PlantParams  p;
    conv::MPCConvCtrl  ctrl(p);
    auto res = runConvSim(ctrl, p, V_REF, V_IN, N_CONV);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 5 - TLCS_FuzzyPI: paper's main result - bumpless two-mode FuzzyPID
// ---------------------------------------------------------------------------

TEST_CASE("BuckBoost s01: TLCS_FuzzyPI converges in buck mode",
          "[buck_boost][regression][tlcs_fuzzypi]")
{
    conv::PlantParams    p;
    conv::TLCSFuzzyPICtrl ctrl(p);
    auto res = runConvSim(ctrl, p, V_REF, V_IN, N_CONV);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 6 - Smoke: all 12 controllers run 300 steps without exception or NaN
// ---------------------------------------------------------------------------

TEST_CASE("BuckBoost: all 12 controllers complete 300 steps without exception or NaN",
          "[buck_boost][regression][smoke]")
{
    conv::PlantParams p;

    using namespace conv;
    std::vector<std::unique_ptr<ControllerBase>> ctrls;
    ctrls.push_back(std::make_unique<OpenLoopCtrl>(p));
    ctrls.push_back(std::make_unique<PIBuckCtrl>(p));
    ctrls.push_back(std::make_unique<PIBoostCtrl>(p));
    ctrls.push_back(std::make_unique<TLCSClassicPICtrl>(p));
    ctrls.push_back(std::make_unique<FuzzyPDCtrl>(p));
    ctrls.push_back(std::make_unique<FuzzyPIDBuckCtrl>(p));
    ctrls.push_back(std::make_unique<FuzzyPIDBoostCtrl>(p));
    ctrls.push_back(std::make_unique<TLCSFuzzyPICtrl>(p));
    ctrls.push_back(std::make_unique<GainScheduledCtrl>(p));
    ctrls.push_back(std::make_unique<ADRCConvCtrl>(p));
    ctrls.push_back(std::make_unique<MPCConvCtrl>(p));
    ctrls.push_back(std::make_unique<LQRConvCtrl>(p));

    REQUIRE(ctrls.size() == 12u);

    for (auto& c : ctrls) {
        INFO("Controller: " << c->name());
        ConvResult res;
        REQUIRE_NOTHROW(res = runConvSim(*c, p, V_REF, V_IN, N_SMOKE));
        CHECK(res.finite);
    }
}
