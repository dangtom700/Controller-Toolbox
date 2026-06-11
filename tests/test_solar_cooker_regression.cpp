/**
 * @file test_solar_cooker_regression.cpp
 * @brief Regression guards for the Solar Cooker with Reflector and Absorber case study.
 *
 * Two test categories:
 *   1. Convergence assertions for PID, ADRC, MPC, SMC, and MRAC on a
 *      clear-sky steady scenario (G=800 W/m^2, T_ref=95 ^\circC, T_init=30 ^\circC).
 *      Each test asserts that |T_pot - T_ref| in the last 20% of the run is
 *      at least 50% smaller than in the first 20%, confirming that the
 *      controller drives the pot temperature toward the cooking setpoint.
 *
 *   2. A smoke test that runs all 12 controllers for 10 steps (300 s) and
 *      asserts no exception is thrown and T_pot is finite.
 *
 * Sign convention: e = T_pot - T_ref (direct-acting; positive = too hot).
 * Negative-gain plant: more f_shade -> less solar gain -> lower T_pot.
 *
 * PlantParams uses default member initialisers - no JSON required.
 */

#include <catch2/catch_test_macros.hpp>

#include "solar_cooker_plant.h"
#include "controllers.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

// ---------------------------------------------------------------------------
// Inline simulation helper
// ---------------------------------------------------------------------------

struct CookerResult {
    double iae;         // integral |T_pot - T_ref| dt  [^\circC.s]
    double early_rmse;  // RMS temperature error over first 20% of steps
    double late_rmse;   // RMS temperature error over last  20% of steps
    bool   finite;      // false if any NaN/Inf was observed
};

static CookerResult runCookerSim(cooker::ControllerBase&       ctrl,
                                   const cooker::PlantParams&    p,
                                   double                        T_ref,
                                   const cooker::Disturbance&    d,
                                   double                        T_abs_init,
                                   double                        T_pot_init,
                                   double                        duration_s)
{
    cooker::SolarCookerPlant plant(p);
    plant.reset(T_abs_init, T_pot_init);
    ctrl.reset();

    const int N        = static_cast<int>(duration_s / p.Ts);
    const int n_window = std::max(1, N / 5);  // 20% window

    CookerResult res{};
    res.finite = true;

    double early_ss = 0.0;
    double late_ss  = 0.0;
    int    early_n  = 0;
    int    late_n   = 0;

    for (int k = 0; k < N; ++k) {
        const Eigen::Vector2d& x     = plant.state();
        const double           T_pot = x(1);

        if (!std::isfinite(T_pot)) { res.finite = false; break; }

        const double e     = T_ref - T_pot;
        const double rmse2 = e * e;
        res.iae += std::abs(e) * p.Ts;

        if (k < n_window)        { early_ss += rmse2; ++early_n; }
        if (k >= N - n_window)   { late_ss  += rmse2; ++late_n;  }

        double f_raw = ctrl.compute(x, T_ref);
        // Apply absorber safety guard (mirrors simulation_runner.cpp)
        if (x(0) >= p.T_abs_max - 1.0) f_raw = 1.0;
        const double f_shade = std::clamp(f_raw, 0.0, 1.0);
        plant.step(f_shade, d);
    }

    if (!std::isfinite(plant.T_pot())) res.finite = false;

    res.early_rmse = early_n > 0 ? std::sqrt(early_ss / early_n) : 0.0;
    res.late_rmse  = late_n  > 0 ? std::sqrt(late_ss  / late_n)  : 0.0;
    return res;
}

// ---------------------------------------------------------------------------
// Shared test scenario - clear-sky steady, T_ref=95 ^\circC, T_init=30 ^\circC
// ---------------------------------------------------------------------------

static const double T_REF       = 95.0;
static const double T_ABS_INIT  = 30.0;
static const double T_POT_INIT  = 30.0;
static const cooker::Disturbance D_CLEAR{ 800.0, 25.0, 1.0 };  // G, T_amb, V_wind

// ---------------------------------------------------------------------------
// TEST 1 - PID: direct-acting on T_pot drives toward T_ref
// ---------------------------------------------------------------------------

TEST_CASE("SolarCooker s01: PID drives T_pot toward cooking setpoint",
          "[solar_cooker][regression][pid]")
{
    cooker::PlantParams p;
    cooker::PIDCookerCtrl ctrl(p);
    auto res = runCookerSim(ctrl, p, T_REF, D_CLEAR, T_ABS_INIT, T_POT_INIT,
                             p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.iae > 0.0);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 2 - ADRC: ESO rejects solar irradiance disturbances
// ---------------------------------------------------------------------------

TEST_CASE("SolarCooker s01: ADRC drives T_pot toward cooking setpoint",
          "[solar_cooker][regression][adrc]")
{
    cooker::PlantParams  p;
    cooker::ADRCCookerCtrl ctrl(p);
    auto res = runCookerSim(ctrl, p, T_REF, D_CLEAR, T_ABS_INIT, T_POT_INIT,
                             p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 3 - MPC: FOPDT model with negative B converges on clear-sky scenario
// ---------------------------------------------------------------------------

TEST_CASE("SolarCooker s01: MPC converges on clear-sky steady scenario",
          "[solar_cooker][regression][mpc]")
{
    cooker::PlantParams  p;
    cooker::MPCCookerCtrl ctrl(p);
    auto res = runCookerSim(ctrl, p, T_REF, D_CLEAR, T_ABS_INIT, T_POT_INIT,
                             p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 4 - SMC: sliding surface on T_pot - T_ref drives toward setpoint
// ---------------------------------------------------------------------------

TEST_CASE("SolarCooker s01: SMC drives T_pot toward cooking setpoint",
          "[solar_cooker][regression][smc]")
{
    cooker::PlantParams  p;
    cooker::SMCCookerCtrl ctrl(p);
    auto res = runCookerSim(ctrl, p, T_REF, D_CLEAR, T_ABS_INIT, T_POT_INIT,
                             p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 5 - MRAC: adaptive controller tracks cooking setpoint
// ---------------------------------------------------------------------------

TEST_CASE("SolarCooker s01: MRAC tracks cooking setpoint",
          "[solar_cooker][regression][mrac]")
{
    cooker::PlantParams  p;
    cooker::MRACCookerCtrl ctrl(p);
    auto res = runCookerSim(ctrl, p, T_REF, D_CLEAR, T_ABS_INIT, T_POT_INIT,
                             p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 6 - Smoke: all 12 controllers run 10 steps without exception or NaN
// ---------------------------------------------------------------------------

TEST_CASE("SolarCooker: all 12 controllers complete 10 steps without exception or NaN",
          "[solar_cooker][regression][smoke]")
{
    cooker::PlantParams p;

    using namespace cooker;
    std::vector<std::unique_ptr<ControllerBase>> ctrls;
    ctrls.push_back(std::make_unique<OpenLoopCtrl>(p));
    ctrls.push_back(std::make_unique<PIDCookerCtrl>(p));
    ctrls.push_back(std::make_unique<ADRCCookerCtrl>(p));
    ctrls.push_back(std::make_unique<MPCCookerCtrl>(p));
    ctrls.push_back(std::make_unique<FuzzyPIDCookerCtrl>(p));
    ctrls.push_back(std::make_unique<SMCCookerCtrl>(p));
    ctrls.push_back(std::make_unique<GainScheduledCookerCtrl>(p));
    ctrls.push_back(std::make_unique<MRACCookerCtrl>(p));
    ctrls.push_back(std::make_unique<L1AdaptiveCookerCtrl>(p));
    ctrls.push_back(std::make_unique<NeuralPIDCookerCtrl>(p));
    ctrls.push_back(std::make_unique<DynaCookerCtrl>(p));
    ctrls.push_back(std::make_unique<ScenarioMPCCookerCtrl>(p));

    REQUIRE(ctrls.size() == 12u);

    constexpr int N_SMOKE = 10;  // 300 s at Ts=30 s

    for (auto& c : ctrls) {
        INFO("Controller: " << c->name());
        CookerResult res;
        REQUIRE_NOTHROW(res = runCookerSim(*c, p, T_REF, D_CLEAR,
                                            T_ABS_INIT, T_POT_INIT,
                                            N_SMOKE * p.Ts));
        CHECK(res.finite);
    }
}
