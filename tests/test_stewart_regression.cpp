/**
 * @file test_stewart_regression.cpp
 * @brief Regression guards for the 6-DOF Stewart Platform Vessel Motion Simulator.
 *
 * The plant tracks a continuously-oscillating 6-DOF sea-state reference (no
 * fixed setpoint to "settle" to), so these tests check:
 *   1. The simulation stays finite throughout (no NaN/Inf explosion).
 *   2. Steady-state (late-window) per-rod RMS tracking error stays within a
 *      sane bound relative to the commanded sea-state amplitude - i.e. the
 *      controller is actually tracking, not just saturating uselessly.
 *
 * PlantParams/SeaStateConfig use default member initialisers - no JSON
 * required, matching the project's "regression tests run without config
 * files" convention.
 */

#include <catch2/catch_test_macros.hpp>

#include "stewart_plant.h"
#include "cfd_input_model.h"
#include "controllers.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

using namespace stewart;

// ---------------------------------------------------------------------------
// Inline simulation helper
// ---------------------------------------------------------------------------

struct StewartResult {
    double iae;          // sum of |rod error|_1 dt over the run
    double early_rms_mm; // per-rod RMS tracking error over first 20% of steps [mm]
    double late_rms_mm;  // per-rod RMS tracking error over last 20% of steps [mm]
    bool   finite;
};

static StewartResult runStewartSim(ControllerBase& ctrl,
                                    const PlantParams& p,
                                    const SeaStateConfig& cfg,
                                    int n_steps_override = -1)
{
    StewartPlant plant(p);
    plant.reset();
    ctrl.reset();
    CFDInputModel cfd(cfg, p);

    const double Ts = p.Ts;
    const int N = (n_steps_override > 0)
                     ? n_steps_override
                     : std::max(1, static_cast<int>(std::lround(cfg.duration_s / Ts)));
    const int n_window = std::max(1, N / 5);

    StewartResult res{};
    res.finite = true;

    double early_ss = 0.0, late_ss = 0.0;
    int    early_n  = 0,   late_n  = 0;

    for (int k = 0; k < N; ++k) {
        const double t = k * Ts;
        const PoseRef ref = cfd.poseAt(t);

        Vec6 L_cmd; Mat6 J;
        plant.geometry().ikAndJacobian(ref, L_cmd, J);

        Vec6 F_load = Vec6::Zero();
        if (cfg.equipment_load) {
            Vec6 wrench;
            wrench << 0.0, 0.0, -(p.F_eq + p.m_platform * GRAVITY), 0.0, 0.0, 0.0;
            F_load = -(J.transpose().partialPivLu().solve(wrench));
        }

        const Vec6 L  = plant.length();
        const Vec6 dL = plant.velocity();

        if (!L.allFinite() || !dL.allFinite()) { res.finite = false; break; }

        const double z_ref_global = std::abs(ref.P(2) - p.z0_mid);
        Vec6 u = ctrl.compute(L_cmd, L, dL, t, z_ref_global);
        for (int i = 0; i < N_RODS; ++i)
            u(i) = std::clamp(u(i), -p.F_rod_max, p.F_rod_max);

        const Vec6 e_rod = L_cmd - L;
        const double rms2 = e_rod.squaredNorm() / N_RODS;
        res.iae += e_rod.lpNorm<1>() * Ts;

        if (k < n_window)      { early_ss += rms2; ++early_n; }
        if (k >= N - n_window) { late_ss  += rms2; ++late_n;  }

        plant.step(u, F_load);
    }

    if (!plant.length().allFinite()) res.finite = false;

    res.early_rms_mm = (early_n > 0) ? std::sqrt(early_ss / early_n) * 1000.0 : 0.0;
    res.late_rms_mm  = (late_n  > 0) ? std::sqrt(late_ss  / late_n)  * 1000.0 : 0.0;
    return res;
}

static SeaStateConfig testConfig(int douglas_state, double Hs, WaveDirection dir,
                                  bool swell, double duration_s)
{
    SeaStateConfig cfg;
    cfg.douglas_state  = douglas_state;
    cfg.Hs             = Hs;
    cfg.direction       = dir;
    cfg.swell           = swell;
    cfg.equipment_load  = true;
    cfg.duration_s      = duration_s;
    cfg.id              = "test_cfg";
    cfg.description     = "regression test configuration";
    return cfg;
}

// Douglas state 5 (Hs=3.25 m, T~6.2 s), Head seas, no swell - moderate-severe
// tracking test used by all per-controller convergence checks below.
static SeaStateConfig defaultTestConfig()
{
    return testConfig(5, 3.25, WaveDirection::Head, false, 30.0);
}

// ---------------------------------------------------------------------------
// Per-controller convergence tests
// ---------------------------------------------------------------------------

TEST_CASE("Stewart ss05_head: PID tracks the sea-state reference", "[stewart][regression][pid]")
{
    PlantParams p;
    PIDStewartCtrl ctrl(p);
    auto res = runStewartSim(ctrl, p, defaultTestConfig());

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.iae > 0.0);
    // Steady-state per-rod RMS error should stay well under 50 mm at sea state 5.
    REQUIRE(res.late_rms_mm < 50.0);
}

TEST_CASE("Stewart ss05_head: ADRC tracks the sea-state reference", "[stewart][regression][adrc]")
{
    PlantParams p;
    ADRCStewartCtrl ctrl(p);
    auto res = runStewartSim(ctrl, p, defaultTestConfig());

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rms_mm < 50.0);
}

TEST_CASE("Stewart ss05_head: SMC tracks the sea-state reference", "[stewart][regression][smc]")
{
    PlantParams p;
    SMCStewartCtrl ctrl(p);
    auto res = runStewartSim(ctrl, p, defaultTestConfig());

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rms_mm < 50.0);
}

TEST_CASE("Stewart ss05_head: LQR tracks the sea-state reference", "[stewart][regression][lqr]")
{
    PlantParams p;
    LQRStewartCtrl ctrl(p);
    auto res = runStewartSim(ctrl, p, defaultTestConfig());

    REQUIRE(res.finite);
    REQUIRE(std::isfinite(res.iae));
    REQUIRE(res.late_rms_mm < 50.0);
}

// ---------------------------------------------------------------------------
// Smoke test - all 12 controllers, short duration, multiple representative
// configurations (calm/moderate/extreme x one direction each).
// ---------------------------------------------------------------------------

TEST_CASE("Stewart: all 12 controllers complete short runs without exception or NaN",
          "[stewart][regression][smoke]")
{
    PlantParams p;

    std::vector<SeaStateConfig> configs{
        testConfig(0, 0.0,  WaveDirection::Head, false, 5.0),   // calm
        testConfig(5, 3.25, WaveDirection::Beam, true,  5.0),   // moderate, beam + swell
        testConfig(9, 16.0, WaveDirection::Following, true, 5.0), // extreme, following + swell
    };

    constexpr int N_SMOKE = 100; // 0.5 s at Ts=5ms

    for (const auto& cfg : configs) {
        std::vector<std::unique_ptr<ControllerBase>> ctrls;
        ctrls.push_back(std::make_unique<PIDStewartCtrl>(p));
        ctrls.push_back(std::make_unique<FuzzyPIDStewartCtrl>(p));
        ctrls.push_back(std::make_unique<ADRCStewartCtrl>(p));
        ctrls.push_back(std::make_unique<SMCStewartCtrl>(p));
        ctrls.push_back(std::make_unique<LQRStewartCtrl>(p));
        ctrls.push_back(std::make_unique<MPCStewartCtrl>(p));
        ctrls.push_back(std::make_unique<MRACStewartCtrl>(p));
        ctrls.push_back(std::make_unique<L1AdaptiveStewartCtrl>(p));
        ctrls.push_back(std::make_unique<GainScheduledStewartCtrl>(p));
        ctrls.push_back(std::make_unique<TubeMPCStewartCtrl>(p));
        ctrls.push_back(std::make_unique<NeuralPIDStewartCtrl>(p));
        ctrls.push_back(std::make_unique<ScenarioMPCStewartCtrl>(p));

        REQUIRE(ctrls.size() == 12u);

        for (auto& c : ctrls) {
            INFO("Config: " << cfg.id << "  Controller: " << c->name());
            StewartResult res;
            REQUIRE_NOTHROW(res = runStewartSim(*c, p, cfg, N_SMOKE));
            CHECK(res.finite);
        }
    }
}

// ---------------------------------------------------------------------------
// Workspace-scaling sanity check (decision #8)
// ---------------------------------------------------------------------------

TEST_CASE("Stewart: extreme Douglas states stay within the Table-1 workspace",
          "[stewart][regression][cfd_input]")
{
    PlantParams p;
    MatrixTuning tuning; // code defaults

    for (int state : {7, 8, 9}) {
        const double Hs = (state == 7) ? 7.5 : (state == 8 ? 11.5 : 16.0);
        auto cfg = testConfig(state, Hs, WaveDirection::Head, true, 10.0);
        CFDInputModel cfd(cfg, p, tuning);

        double max_pos = 0.0, max_att_deg = 0.0;
        const int N = 2000;
        for (int k = 0; k < N; ++k) {
            const double t = k * p.Ts;
            PoseRef ref = cfd.poseAt(t);
            max_pos = std::max(max_pos, std::abs(ref.P(2) - p.z0_mid));
            max_att_deg = std::max(max_att_deg, std::abs(ref.rpy(1)) * RAD2DEG);
        }
        INFO("Douglas state " << state << " scale_factor=" << cfd.scaleFactor());
        CHECK(max_pos <= p.workspace.heave_max * 1.0001); // small float-compare margin
        CHECK(max_att_deg <= p.workspace.pitch_max_deg * 1.0001);
    }
}
