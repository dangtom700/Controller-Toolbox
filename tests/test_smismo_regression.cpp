/**
 * @file test_smismo_regression.cpp
 * @brief Regression guards for the Separate Meter-In Separate Meter-Out
 *        (SMISMO) hydraulic actuator case study.
 *
 * Two test categories:
 *   1. Convergence assertions for PID, CascadePID, LQR, and ADRC on the
 *      s01_resistive_step scenario (x_ref step 0.05 -> 0.25 m, F_ext = +500 N).
 *      Each test asserts that |x_ref - x_L| in the last 20% of the run is at
 *      least 50% smaller than in the first 20%, that pressures stay inside
 *      [0, P_max], and that the piston stays inside its stroke.
 *
 *   2. A smoke test that runs all 12 controllers for 200 steps (0.2 s) and
 *      asserts no exception and a finite plant state.
 *
 * Controllers return the scalar working-side command u_ctrl; the shared
 * ValveAllocator (mode selection + off-side backpressure PI to P_bd = 20 bar)
 * maps it to the two PDCV commands - same loop as the simulation runner.
 *
 * PlantParams uses default member initialisers - no JSON required.
 */

#include <catch2/catch_test_macros.hpp>

#include "smismo_plant.h"
#include "controllers.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

// ---------------------------------------------------------------------------
// Inline simulation helper (mirrors simulation_runner.cpp)
// ---------------------------------------------------------------------------

struct SmismoResult {
    double iae;            // integral |x_ref - x_L| dt  [m.s]
    double early_rmse;     // RMS position error over first 20% of steps
    double late_rmse;      // RMS position error over last  20% of steps
    bool   finite;         // false if any plant state is NaN/Inf
    bool   bounds_ok;      // false if pressure or stroke limits are violated
};

static SmismoResult runSmismoSim(smismo::ControllerBase&    ctrl,
                                 const smismo::PlantParams& p,
                                 double x_ref_0, double x_ref_1, double t_step,
                                 double F_ext,
                                 double x_init, double P1_init, double P2_init,
                                 double duration_s)
{
    smismo::SmismoPlant plant(p);
    plant.reset(x_init, P1_init, P2_init);
    ctrl.reset();

    smismo::ValveAllocator alloc(p);
    alloc.reset();

    const int N        = static_cast<int>(duration_s / p.Ts);
    const int n_window = std::max(1, N / 5);  // 20% window

    SmismoResult res{};
    res.finite    = true;
    res.bounds_ok = true;

    double early_ss = 0.0;
    double late_ss  = 0.0;
    int    early_n  = 0;
    int    late_n   = 0;

    for (int k = 0; k < N; ++k) {
        const double t     = k * p.Ts;
        const double x_ref = (t >= t_step) ? x_ref_1 : x_ref_0;

        if (!plant.state().allFinite()) { res.finite = false; break; }

        const double e = x_ref - plant.x_L();
        res.iae += std::abs(e) * p.Ts;
        if (k < n_window)      { early_ss += e * e; ++early_n; }
        if (k >= N - n_window) { late_ss  += e * e; ++late_n;  }

        if (plant.P1() < p.P_min - 1.0 || plant.P1() > p.P_max + 1.0 ||
            plant.P2() < p.P_min - 1.0 || plant.P2() > p.P_max + 1.0 ||
            plant.x_L() < -1e-9 || plant.x_L() > p.stroke + 1e-9)
            res.bounds_ok = false;

        double u_ctrl = ctrl.compute(plant.state(), x_ref);
        if (!std::isfinite(u_ctrl)) u_ctrl = 0.0;

        ctrl.beforePlantStep(plant);  // DOBEnergyCtrl adjusts P_s; no-op for others

        const auto cmd = alloc.allocate(u_ctrl, plant.state());
        plant.step(cmd.u1, cmd.u2, F_ext);
    }

    if (!plant.state().allFinite()) res.finite = false;

    res.early_rmse = early_n > 0 ? std::sqrt(early_ss / early_n) : 0.0;
    res.late_rmse  = late_n  > 0 ? std::sqrt(late_ss  / late_n)  : 0.0;
    return res;
}

// ---------------------------------------------------------------------------
// Shared test scenario - s01_resistive_step
// ---------------------------------------------------------------------------

static const double X_REF_0  = 0.05;
static const double X_REF_1  = 0.25;
static const double T_STEP   = 0.0;     // step at t=0: early window sees full error
static const double F_EXT    = 500.0;
static const double X_INIT   = 0.05;
static const double P1_INIT  = 2.5e6;
static const double P2_INIT  = 2.0e6;
static const double DURATION = 4.0;

// ---------------------------------------------------------------------------
// TEST 1 - PID: position step converges under resistive load
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO s01: PID converges on resistive position step",
          "[smismo][regression][pid]")
{
    smismo::PlantParams p;
    smismo::PIDPosCtrl  ctrl(p);
    auto res = runSmismoSim(ctrl, p, X_REF_0, X_REF_1, T_STEP, F_EXT,
                            X_INIT, P1_INIT, P2_INIT, DURATION);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.iae > 0.0);
    REQUIRE(res.bounds_ok);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 2 - CascadePID: outer position / inner velocity loop converges
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO s01: CascadePID converges on resistive position step",
          "[smismo][regression][cascade_pid]")
{
    smismo::PlantParams    p;
    smismo::CascadePIDCtrl ctrl(p);
    auto res = runSmismoSim(ctrl, p, X_REF_0, X_REF_1, T_STEP, F_EXT,
                            X_INIT, P1_INIT, P2_INIT, DURATION);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.bounds_ok);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 3 - LQR: 2-state Bryson gain converges
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO s01: LQR converges on resistive position step",
          "[smismo][regression][lqr]")
{
    smismo::PlantParams p;
    smismo::LQRCtrl     ctrl(p);
    auto res = runSmismoSim(ctrl, p, X_REF_0, X_REF_1, T_STEP, F_EXT,
                            X_INIT, P1_INIT, P2_INIT, DURATION);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.bounds_ok);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 4 - ADRC: ESO rejects load force and friction as total disturbance
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO s01: ADRC converges on resistive position step",
          "[smismo][regression][adrc]")
{
    smismo::PlantParams p;
    smismo::ADRCCtrl    ctrl(p);
    auto res = runSmismoSim(ctrl, p, X_REF_0, X_REF_1, T_STEP, F_EXT,
                            X_INIT, P1_INIT, P2_INIT, DURATION);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.bounds_ok);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 5 - Backpressure regulation: off-side chamber held near P_bd
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO: off-side backpressure regulated near P_bd during extension",
          "[smismo][regression][backpressure]")
{
    smismo::PlantParams p;
    smismo::PIDPosCtrl  ctrl(p);

    smismo::SmismoPlant plant(p);
    plant.reset(X_INIT, P1_INIT, P2_INIT);
    ctrl.reset();
    smismo::ValveAllocator alloc(p);
    alloc.reset();

    const int N = static_cast<int>(DURATION / p.Ts);
    double p2_dev_sum = 0.0;
    int    n_settled  = 0;

    for (int k = 0; k < N; ++k) {
        const double t = k * p.Ts;
        double u_ctrl = ctrl.compute(plant.state(), X_REF_1);
        if (!std::isfinite(u_ctrl)) u_ctrl = 0.0;
        ctrl.beforePlantStep(plant);
        const auto cmd = alloc.allocate(u_ctrl, plant.state());
        plant.step(cmd.u1, cmd.u2, F_EXT);

        // After the transient, the rod chamber should sit near P_bd = 20 bar
        if (t > DURATION * 0.5) {
            p2_dev_sum += std::abs(plant.P2() - p.P_bd);
            ++n_settled;
        }
    }

    REQUIRE(plant.state().allFinite());
    REQUIRE(n_settled > 0);
    const double p2_dev_mean = p2_dev_sum / n_settled;
    REQUIRE(p2_dev_mean < 0.5e6);   // within 5 bar of the 20 bar setpoint on average
}

// ---------------------------------------------------------------------------
// TEST 6 - Smoke: all 13 controllers run 200 steps without exception or NaN
// ---------------------------------------------------------------------------

TEST_CASE("SMISMO: all 13 controllers complete 200 steps without exception or NaN",
          "[smismo][regression][smoke]")
{
    smismo::PlantParams p;

    using namespace smismo;
    std::vector<std::unique_ptr<ControllerBase>> ctrls;
    ctrls.push_back(std::make_unique<PIDPosCtrl>(p));
    ctrls.push_back(std::make_unique<CascadePIDCtrl>(p));
    ctrls.push_back(std::make_unique<LQRCtrl>(p));
    ctrls.push_back(std::make_unique<LQGCtrl>(p));
    ctrls.push_back(std::make_unique<MPCCtrl>(p));
    ctrls.push_back(std::make_unique<ADRCCtrl>(p));
    ctrls.push_back(std::make_unique<SMCCtrl>(p));
    ctrls.push_back(std::make_unique<FLCtrl>(p));
    ctrls.push_back(std::make_unique<TubeMPCCtrl>(p));
    ctrls.push_back(std::make_unique<L1Ctrl>(p));
    ctrls.push_back(std::make_unique<GainSchedCtrl>(p));
    ctrls.push_back(std::make_unique<NMPCCtrl>(p));
    ctrls.push_back(std::make_unique<DOBEnergyCtrl>(p));

    REQUIRE(ctrls.size() == 13u);

    constexpr double T_SMOKE = 0.2;  // 200 steps at Ts = 1 ms

    for (auto& c : ctrls) {
        INFO("Controller: " << c->name());
        SmismoResult res;
        REQUIRE_NOTHROW(res = runSmismoSim(*c, p, X_REF_0, X_REF_1, T_STEP,
                                           F_EXT, X_INIT, P1_INIT, P2_INIT,
                                           T_SMOKE));
        CHECK(res.finite);
        CHECK(res.bounds_ok);
    }
}
