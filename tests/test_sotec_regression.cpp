/**
 * @file test_sotec_regression.cpp
 * @brief Regression guards for the Solar Ocean Thermal Energy Conversion System
 *        case study.
 *
 * Two test categories:
 *   1. Convergence assertions for PID, ADRC, MPC, LQR, and GainScheduled on
 *      the s01_mppt_steady scenario (G=800 W/m^2, T_h_ref=63 ^\circC, T_c=6 ^\circC,
 *      T_h_init=T_coll_init=20 ^\circC).  Each test asserts that |T_h - T_h_ref|
 *      in the last 20% of the run is at least 50% smaller than in the first
 *      20%, and that the P_inlet hard constraint is never violated.
 *
 *   2. A smoke test that runs all 12 controllers for 10 steps (300 s) and
 *      asserts no exception, finite T_h, and P_inlet <= P_inlet_max.
 *
 * All controllers return CtrlOutput{m_dot_f, m_dot_wf}.  The secondary loop
 * (m_dot_wf) is handled internally by each controller via pressure-constrained
 * feedforward; the test only asserts P_inlet <= P_inlet_max on the ORC output.
 *
 * PlantParams uses default member initialisers - no JSON required.
 */

#include <catch2/catch_test_macros.hpp>

#include "sotec_plant.h"
#include "controllers.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

// ---------------------------------------------------------------------------
// Inline simulation helper
// ---------------------------------------------------------------------------

struct SotecResult {
    double iae;            // integral |T_h_ref - T_h| dt  [^\circC.s]
    double early_rmse;     // RMS T_h error over first 20% of steps
    double late_rmse;      // RMS T_h error over last  20% of steps
    bool   finite;         // false if T_h or T_coll is NaN/Inf
    bool   constraint_ok;  // false if any P_inlet > P_inlet_max + tolerance
};

static SotecResult runSotecSim(sotec::ControllerBase&       ctrl,
                                 const sotec::PlantParams&    p,
                                 double                       T_h_ref,
                                 double                       G,
                                 double                       T_c,
                                 double                       T_h_init,
                                 double                       T_coll_init,
                                 double                       duration_s)
{
    sotec::SotecPlant plant(p);
    plant.reset(T_h_init, T_coll_init);
    ctrl.reset();

    const int N        = static_cast<int>(duration_s / p.Ts);
    const int n_window = std::max(1, N / 5);  // 20% window

    SotecResult res{};
    res.finite        = true;
    res.constraint_ok = true;

    double early_ss = 0.0;
    double late_ss  = 0.0;
    int    early_n  = 0;
    int    late_n   = 0;

    sotec::Disturbance d;
    d.G     = G;
    d.T_amb = 20.0;
    d.T_c   = T_c;

    for (int k = 0; k < N; ++k) {
        const Eigen::Vector2d& x   = plant.state();
        const double           T_h = x(0);

        if (!std::isfinite(T_h)) { res.finite = false; break; }

        const double e     = T_h_ref - T_h;
        const double rmse2 = e * e;
        res.iae += std::abs(e) * p.Ts;

        if (k < n_window)        { early_ss += rmse2; ++early_n; }
        if (k >= N - n_window)   { late_ss  += rmse2; ++late_n;  }

        sotec::CtrlOutput cmd = ctrl.compute(x, T_h_ref, T_c, G);

        // Verify pressure constraint using ORC algebraic output at current state
        sotec::OrcState orc = plant.computeOrc(cmd.m_dot_f, cmd.m_dot_wf, d);
        if (orc.P_inlet > p.P_inlet_max + 0.01)
            res.constraint_ok = false;

        plant.step(cmd.m_dot_f, cmd.m_dot_wf, d);
    }

    if (!plant.state().allFinite()) res.finite = false;

    res.early_rmse = early_n > 0 ? std::sqrt(early_ss / early_n) : 0.0;
    res.late_rmse  = late_n  > 0 ? std::sqrt(late_ss  / late_n)  : 0.0;
    return res;
}

// ---------------------------------------------------------------------------
// Shared test scenario - s01_mppt_steady
// ---------------------------------------------------------------------------

static const double T_H_REF     = 63.0;
static const double G_STEADY    = 800.0;
static const double T_C         = 6.0;
static const double T_H_INIT    = 20.0;
static const double T_COLL_INIT = 20.0;

// ---------------------------------------------------------------------------
// TEST 1 - PID: regulates T_h to T_h_ref under steady irradiance
// ---------------------------------------------------------------------------

TEST_CASE("SOTEC s01: PID regulates T_h toward setpoint under steady irradiance",
          "[sotec][regression][pid]")
{
    sotec::PlantParams p;
    sotec::PIDThCtrl   ctrl(p);
    auto res = runSotecSim(ctrl, p, T_H_REF, G_STEADY, T_C,
                            T_H_INIT, T_COLL_INIT, p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.iae > 0.0);
    REQUIRE(res.constraint_ok);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 2 - ADRC: ESO rejects solar-irradiance disturbance
// ---------------------------------------------------------------------------

TEST_CASE("SOTEC s01: ADRC drives T_h toward setpoint",
          "[sotec][regression][adrc]")
{
    sotec::PlantParams p;
    sotec::ADRCCtrl    ctrl(p);
    auto res = runSotecSim(ctrl, p, T_H_REF, G_STEADY, T_C,
                            T_H_INIT, T_COLL_INIT, p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.constraint_ok);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 3 - MPC: FOPDT deviation model drives T_h toward setpoint
// ---------------------------------------------------------------------------

TEST_CASE("SOTEC s01: MPC converges on steady MPPT scenario",
          "[sotec][regression][mpc]")
{
    sotec::PlantParams p;
    sotec::MPCCtrl     ctrl(p);
    auto res = runSotecSim(ctrl, p, T_H_REF, G_STEADY, T_C,
                            T_H_INIT, T_COLL_INIT, p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.constraint_ok);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 4 - LQR: Bryson-optimal 2-state gain drives T_h toward setpoint
// ---------------------------------------------------------------------------

TEST_CASE("SOTEC s01: LQR converges on steady MPPT scenario",
          "[sotec][regression][lqr]")
{
    sotec::PlantParams p;
    sotec::LQRCtrl     ctrl(p);
    auto res = runSotecSim(ctrl, p, T_H_REF, G_STEADY, T_C,
                            T_H_INIT, T_COLL_INIT, p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.constraint_ok);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 5 - GainScheduled: 3-bracket PIDs span T_h operating range
// ---------------------------------------------------------------------------

TEST_CASE("SOTEC s01: GainScheduled converges across T_h operating range",
          "[sotec][regression][gain_scheduled]")
{
    sotec::PlantParams      p;
    sotec::GainScheduledCtrl ctrl(p);
    auto res = runSotecSim(ctrl, p, T_H_REF, G_STEADY, T_C,
                            T_H_INIT, T_COLL_INIT, p.duration);

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.constraint_ok);
    REQUIRE(res.late_rmse < res.early_rmse * 0.50);
}

// ---------------------------------------------------------------------------
// TEST 6 - Smoke: all 12 controllers run 10 steps without exception or NaN
// ---------------------------------------------------------------------------

TEST_CASE("SOTEC: all 12 controllers complete 10 steps without exception or NaN",
          "[sotec][regression][smoke]")
{
    sotec::PlantParams p;

    using namespace sotec;
    std::vector<std::unique_ptr<ControllerBase>> ctrls;
    ctrls.push_back(std::make_unique<OpenLoopCtrl>(p));
    ctrls.push_back(std::make_unique<PIDThCtrl>(p));
    ctrls.push_back(std::make_unique<ADRCCtrl>(p));
    ctrls.push_back(std::make_unique<MPCCtrl>(p));
    ctrls.push_back(std::make_unique<LQRCtrl>(p));
    ctrls.push_back(std::make_unique<FuzzyPIDCtrl>(p));
    ctrls.push_back(std::make_unique<MRACCtrl>(p));
    ctrls.push_back(std::make_unique<L1AdaptiveCtrl>(p));
    ctrls.push_back(std::make_unique<GainScheduledCtrl>(p));
    ctrls.push_back(std::make_unique<ScenarioMPCCtrl>(p));
    ctrls.push_back(std::make_unique<DynaCtrl>(p));
    ctrls.push_back(std::make_unique<NeuralPIDCtrl>(p));

    REQUIRE(ctrls.size() == 12u);

    constexpr int N_SMOKE = 10;  // 300 s at Ts=30 s

    sotec::Disturbance d_smoke;
    d_smoke.G     = G_STEADY;
    d_smoke.T_amb = 20.0;
    d_smoke.T_c   = T_C;

    for (auto& c : ctrls) {
        INFO("Controller: " << c->name());
        SotecResult res;
        REQUIRE_NOTHROW(res = runSotecSim(*c, p, T_H_REF, G_STEADY, T_C,
                                           T_H_INIT, T_COLL_INIT,
                                           N_SMOKE * p.Ts));
        CHECK(res.finite);
        CHECK(res.constraint_ok);
    }
}
