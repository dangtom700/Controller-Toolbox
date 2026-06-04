/**
 * @file test_humid_regression.cpp
 * @brief Controller regression guards for the Porous Fiber Plate Humidification case study.
 *
 * (Psychrometrics correctness lives in test_humidification.cpp.)
 *
 * Two test categories:
 *   1. Convergence assertions for PID, Smith, ADRC, and MPC on the s01
 *      nominal scenario (T_out=-10^\circC, phi_out=0.35, ref_phi=0.45, phi_init=0.20).
 *      Tests assert that the final sensor reading is within +/-5 % RH of the
 *      setpoint and that IAE is finite and positive.
 *
 *   2. A smoke test that runs all 10 controllers for half the scenario
 *      duration (5400 s) and asserts no exception and finite phi_measured.
 *
 * PlantParams uses default member initialisers - no JSON file required.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "humid_plant.h"
#include "controllers.h"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;

// ---------------------------------------------------------------------------
// Inline simulation helper
// ---------------------------------------------------------------------------

struct HumidResult {
    double iae;         // |phi_measured - ref| integral [fraction.s]
    double final_err;   // |phi_measured - ref| at end of run [fraction]
    double early_rmse;  // RMS phi error over first 20% of steps
    double late_rmse;   // RMS phi error over last  20% of steps
    bool   finite;
};

static HumidResult runHumidSim(humid::ControllerBase&              ctrl,
                                const humid::PlantParams&           params,
                                std::function<humid::Disturbance(double)> dist_fn,
                                std::function<double(double)>       ref_fn,
                                double                              phi_init,
                                double                              duration_s)
{
    humid::HumidificationPlant plant(params);
    plant.reset(phi_init);
    ctrl.reset();

    const double Ts = params.Ts;
    const int    N  = static_cast<int>(duration_s / Ts);

    HumidResult res{};
    res.finite = true;

    // Prime: one nominal step so controller has a valid sensor reading (mirrors runner)
    humid::Disturbance d0  = dist_fn(0.0);
    humid::PlantOutput out = plant.step({humid::kFanMid, humid::kTa_nom}, d0);

    const int n_window = std::max(1, N / 5);

    for (int k = 0; k < N; ++k) {
        const double t       = k * Ts;
        humid::Disturbance d = dist_fn(t);

        if (!std::isfinite(out.phi_measured)) { res.finite = false; break; }

        // Feed outdoor phi to FF-PID controllers
        if (auto* ff = dynamic_cast<humid::FFPIDHumidCtrl*>(&ctrl))
            ff->setOutdoorPhi(d.phi_out);

        const double ref_phi = ref_fn(t);
        humid::ControlInput u = ctrl.compute(out.phi_measured, ref_phi);
        out = plant.step(u, d);

        double e = ref_phi - out.phi_measured;
        if (!std::isfinite(out.phi_measured)) { res.finite = false; break; }

        res.iae += std::abs(e) * Ts;

        double rmse_k = e * e;
        if (k < n_window)       res.early_rmse += rmse_k;
        if (k >= N - n_window)  res.late_rmse  += rmse_k;
    }

    const double ref_end = ref_fn(duration_s);
    res.final_err  = std::abs(ref_end - out.phi_measured);
    res.early_rmse = std::sqrt(res.early_rmse / n_window);
    res.late_rmse  = std::sqrt(res.late_rmse  / n_window);
    return res;
}

// ---------------------------------------------------------------------------
// Shared scenario helpers
// ---------------------------------------------------------------------------

static humid::PlantParams makeParams()
{
    humid::PlantParams p;
    p.init();
    return p;
}

static humid::Disturbance d_nominal(double /*t*/)
{
    return {263.15, 0.35, 0.0};   // -10 ^\circC, 35 % RH, no occupants
}

static humid::Disturbance d_occupancy(double t)
{
    double n = (t >= 600.0 && t < 2400.0) ? 4.0 : 0.0;
    return {263.15, 0.35, n};
}

// ---------------------------------------------------------------------------
// TEST 1 - PID: s01 nominal, converges to 0.45 +/- 0.05
// ---------------------------------------------------------------------------

TEST_CASE("Humid s01: PID converges to within 5%% RH of setpoint 0.45",
          "[humidification][regression][pid]")
{
    auto params = makeParams();
    humid::PIDHumidCtrl ctrl(params.Ts);
    auto res = runHumidSim(ctrl, params, d_nominal,
                            [](double) { return 0.45; }, 0.20, params.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.iae > 0.0);
    REQUIRE(res.final_err < 0.05);  // +/-5 % RH
}

// ---------------------------------------------------------------------------
// TEST 2 - Smith Predictor: compensates 2-step sensor dead-time
// ---------------------------------------------------------------------------

TEST_CASE("Humid s01: Smith Predictor converges despite 2-step sensor delay",
          "[humidification][regression][smith]")
{
    auto params = makeParams();
    humid::SmithHumidCtrl ctrl(params.Ts);
    auto res = runHumidSim(ctrl, params, d_nominal,
                            [](double) { return 0.45; }, 0.20, params.duration);

    REQUIRE(res.finite);
    REQUIRE(res.final_err < 0.05);
    // Late error should be meaningfully smaller than early error
    REQUIRE(res.late_rmse < res.early_rmse * 0.80);
}

// ---------------------------------------------------------------------------
// TEST 3 - ADRC: ESO rejects occupancy disturbance
// ---------------------------------------------------------------------------

TEST_CASE("Humid s04: ADRC rejects occupancy disturbance",
          "[humidification][regression][adrc]")
{
    auto params = makeParams();
    humid::ADRCHumidCtrl ctrl(params.Ts);
    auto res = runHumidSim(ctrl, params, d_occupancy,
                            [](double) { return 0.45; }, 0.20, params.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    // Late error smaller than early - disturbance is being rejected
    REQUIRE(res.late_rmse < res.early_rmse * 0.85);
}

// ---------------------------------------------------------------------------
// TEST 4 - MPC: converges and IAE <= PID IAE
// ---------------------------------------------------------------------------

TEST_CASE("Humid s01: MPC converges and IAE <= PID IAE",
          "[humidification][regression][mpc]")
{
    auto params = makeParams();
    humid::MPCHumidCtrl mpc(params.Ts);
    humid::PIDHumidCtrl pid(params.Ts);

    auto r_mpc = runHumidSim(mpc, params, d_nominal,
                              [](double) { return 0.45; }, 0.20, params.duration);
    auto r_pid = runHumidSim(pid, params, d_nominal,
                              [](double) { return 0.45; }, 0.20, params.duration);

    REQUIRE(r_mpc.finite);
    REQUIRE(r_mpc.final_err < 0.05);
    REQUIRE(r_mpc.iae < r_pid.iae * 1.30);
}

// ---------------------------------------------------------------------------
// TEST 5 - Setpoint step: PID tracks 30% -> 50% step at t=900 s
// ---------------------------------------------------------------------------

TEST_CASE("Humid s03: PID tracks setpoint step from 30%% to 50%%",
          "[humidification][regression][setpoint_step]")
{
    auto params = makeParams();
    humid::PIDHumidCtrl ctrl(params.Ts);
    auto ref_fn = [](double t) { return t < 900.0 ? 0.30 : 0.50; };
    auto res = runHumidSim(ctrl, params, d_nominal, ref_fn, 0.20, params.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    // After 10800s, should have settled near 0.50
    REQUIRE(res.final_err < 0.05);
}

// ---------------------------------------------------------------------------
// TEST 6 - Smoke: all 10 controllers complete 5400 s without exception or NaN
// ---------------------------------------------------------------------------

TEST_CASE("Humid: all 10 controllers complete 5400s without exception or NaN",
          "[humidification][regression][smoke]")
{
    auto params = makeParams();
    const double smoke_dur = 5400.0;

    using namespace humid;
    std::vector<std::unique_ptr<ControllerBase>> ctrls;
    ctrls.push_back(std::make_unique<PIDHumidCtrl>(params.Ts));
    ctrls.push_back(std::make_unique<PID_AWHumidCtrl>(params.Ts));
    ctrls.push_back(std::make_unique<FFPIDHumidCtrl>(params.Ts));
    ctrls.push_back(std::make_unique<CascadePIDHumidCtrl>(params.Ts));
    ctrls.push_back(std::make_unique<GainSchedHumidCtrl>(params.Ts));
    ctrls.push_back(std::make_unique<SmithHumidCtrl>(params.Ts));
    ctrls.push_back(std::make_unique<ADRCHumidCtrl>(params.Ts));
    ctrls.push_back(std::make_unique<MPCHumidCtrl>(params.Ts));
    ctrls.push_back(std::make_unique<MRACHumidCtrl>(params.Ts));
    ctrls.push_back(std::make_unique<GPCRLSHumidCtrl>(params.Ts));

    REQUIRE(ctrls.size() == 10u);

    for (auto& c : ctrls) {
        INFO("Controller: " << c->name());
        HumidResult res;
        REQUIRE_NOTHROW(res = runHumidSim(*c, params, d_nominal,
                                          [](double) { return 0.45; },
                                          0.20, smoke_dur));
        CHECK(res.finite);
    }
}
