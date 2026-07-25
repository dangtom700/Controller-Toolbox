// test_ddmr_regression.cpp - Catch2 regression suite for the Differential Drive Robot
// Tracking (FUHAC) case study. Tag: [ddmr]
//
// Run with:  build/tests/test_ddmr_regression.exe "[ddmr]"
// NOT with:  ctest -R test_ddmr_regression   (catch_discover_tests registers each TEST_CASE
//                                             under its full sentence name, so a filter on
//                                             the target name matches zero tests)
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "controllers.h"
#include "differential_drive_robot_tracking_plant.h"
#include "simulation_runner.h"
#include "trajectory.h"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#ifndef DDMR_SIM_SOURCE_DIR
#define DDMR_SIM_SOURCE_DIR "."
#endif

using namespace differentialdriverobottracking;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Nominal parameters, hard-coded so the tests do not depend on the JSON being present.
/// Kept in step with config/plant_params.json by the "config matches test defaults" case.
PlantParams nominalParams() {
    return PlantParams{};   // struct defaults ARE the nominal Pioneer 3-DX set
}

/// Short circle scenario used by the convergence cases: starts off-path so the controller
/// has real work to do, but only 12 s so the suite stays fast.
Scenario circleScenario(bool off_path = true) {
    Scenario s;
    s.id            = "test_circle";
    s.trajectory    = "circle";
    s.a             = 3.0;
    s.T_sim         = 12.0;
    s.time_scale    = 1.0;
    s.start_on_path = !off_path;
    s.x0            = 2.6;      // ~0.4 m inside the circle at t = 0
    s.y0            = 0.3;
    s.theta0        = 1.4;
    return s;
}

std::vector<ControllerPtr> roster() { return makeControllers(nominalParams()); }

ControllerPtr byName(const std::string& want) {
    for (auto& c : roster())
        if (c->name() == want) return std::move(c);
    return nullptr;
}

}  // namespace

// ===========================================================================
// Plant
// ===========================================================================
TEST_CASE("DDMR plant: inertia matrix is invertible for nominal parameters", "[ddmr][plant]") {
    Plant plant(nominalParams());
    REQUIRE(plant.isHealthy());
    REQUIRE(plant.stateSize() == 5);
}

TEST_CASE("DDMR plant: zero torque from rest is a fixed point", "[ddmr][plant]") {
    Plant plant(nominalParams());
    plant.resetPose(1.0, -2.0, 0.5);
    for (int k = 0; k < 200; ++k) plant.step(0.0, 0.0);

    CHECK(plant.X() == Catch::Approx(1.0).margin(1e-12));
    CHECK(plant.Y() == Catch::Approx(-2.0).margin(1e-12));
    CHECK(plant.theta() == Catch::Approx(0.5).margin(1e-12));
    CHECK(plant.v() == Catch::Approx(0.0).margin(1e-12));
    CHECK(plant.w() == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("DDMR plant: equal wheel torques drive straight", "[ddmr][plant]") {
    Plant plant(nominalParams());
    plant.resetPose(0.0, 0.0, 0.0);
    for (int k = 0; k < 400; ++k) plant.step(2.0, 2.0);   // 2 s at Ts_plant = 5 ms

    // theta must not change, y must not move, x must advance.
    CHECK(std::abs(plant.theta()) < 1e-9);
    CHECK(std::abs(plant.Y()) < 1e-9);
    CHECK(plant.X() > 0.1);
    CHECK(plant.v() > 0.0);
    CHECK(std::abs(plant.w()) < 1e-9);
}

TEST_CASE("DDMR plant: opposite wheel torques spin in place", "[ddmr][plant]") {
    Plant plant(nominalParams());
    plant.resetPose(0.0, 0.0, 0.0);
    for (int k = 0; k < 200; ++k) plant.step(1.0, -1.0);

    CHECK(std::abs(plant.v()) < 1e-9);
    CHECK(plant.w() > 0.0);            // tau_R > tau_L turns counter-clockwise
    CHECK(std::abs(plant.theta()) > 1e-3);
}

TEST_CASE("DDMR plant: velocity is bounded by friction at saturation", "[ddmr][plant]") {
    const PlantParams p = nominalParams();
    Plant plant(p);
    plant.resetPose(0.0, 0.0, 0.0);
    for (int k = 0; k < 4000; ++k) plant.step(p.tau_max, p.tau_max);

    // Steady state: tau = Kf*omega_wheel  =>  v_ss = r*tau_max/Kf.
    const double v_ss = p.r_wheel * p.tau_max / p.Kf;
    CHECK(plant.v() == Catch::Approx(v_ss).epsilon(0.02));
    CHECK(plant.state().allFinite());
}

TEST_CASE("DDMR plant: non-finite torque holds the state (NaN guard)", "[ddmr][plant][nan]") {
    Plant plant(nominalParams());
    plant.resetPose(0.5, 0.5, 0.25);
    for (int k = 0; k < 50; ++k) plant.step(3.0, 3.0);

    const Eigen::VectorXd before = plant.state();
    plant.step(std::numeric_limits<double>::quiet_NaN(), 3.0);
    plant.step(3.0, std::numeric_limits<double>::infinity());

    CHECK(plant.state().isApprox(before));
    CHECK(plant.state().allFinite());
}

TEST_CASE("DDMR plant: torque saturation is enforced inside step()", "[ddmr][plant]") {
    const PlantParams p = nominalParams();
    Plant a(p), b(p);
    a.resetPose(0, 0, 0);
    b.resetPose(0, 0, 0);
    for (int k = 0; k < 100; ++k) {
        a.step(1e6, 1e6);          // absurd command, must clip to tau_max
        b.step(p.tau_max, p.tau_max);
    }
    CHECK(a.state().isApprox(b.state()));
}

TEST_CASE("DDMR plant: wrapAngle maps into (-pi, pi]", "[ddmr][plant]") {
    CHECK(wrapAngle(0.0) == Catch::Approx(0.0));
    CHECK(wrapAngle(3.0 * kPi) == Catch::Approx(kPi).margin(1e-12));
    CHECK(wrapAngle(-3.0 * kPi) == Catch::Approx(kPi).margin(1e-12));
    CHECK(wrapAngle(2.0 * kPi + 0.5) == Catch::Approx(0.5).margin(1e-12));
    CHECK(std::abs(wrapAngle(std::numeric_limits<double>::quiet_NaN())) < 1e-12);
}

// ===========================================================================
// Multi-rate bookkeeping
// ===========================================================================
TEST_CASE("DDMR timing: the paper's 30/150 ms ratio yields eps = 0.2", "[ddmr][plant]") {
    const PlantParams p = nominalParams();
    CHECK(p.Tf == Catch::Approx(0.03));
    CHECK(p.Ts_slow == Catch::Approx(0.15));
    CHECK(p.Tf / p.Ts_slow == Catch::Approx(0.2));
    CHECK(p.slowDivider() == 5);        // one slow tick per five fast ticks
    CHECK(p.plantSubSteps() == 6);      // 30 ms / 5 ms
}

// ===========================================================================
// Trajectory
// ===========================================================================
TEST_CASE("DDMR trajectory: all three paths are closed over one 2*pi period", "[ddmr][traj]") {
    struct Case { PathType p; double a; };
    const Case cases[] = {{PathType::Lemniscate, 2.0},
                          {PathType::Circle,     3.0},
                          {PathType::Diamond,    1.0}};
    for (const auto& c : cases) {
        double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        refPosition(c.p, c.a, 0.0, x0, y0);
        refPosition(c.p, c.a, 2.0 * kPi, x1, y1);
        CHECK(x1 == Catch::Approx(x0).margin(1e-9));
        CHECK(y1 == Catch::Approx(y0).margin(1e-9));
    }
}

TEST_CASE("DDMR trajectory: circle radius is exactly a", "[ddmr][traj]") {
    for (double t = 0.0; t < 6.5; t += 0.31) {
        double x = 0, y = 0;
        refPosition(PathType::Circle, 3.0, t, x, y);
        CHECK(std::hypot(x, y) == Catch::Approx(3.0).margin(1e-12));
    }
}

TEST_CASE("DDMR trajectory: v_r and omega_r stay finite and clamped, diamond included",
          "[ddmr][traj]") {
    const PlantParams p = nominalParams();
    const PathType paths[] = {PathType::Lemniscate, PathType::Circle, PathType::Diamond};
    const double amps[]    = {2.0, 3.0, 1.0};

    for (int i = 0; i < 3; ++i) {
        Trajectory traj(paths[i], amps[i], 1.0, p.v_max, p.w_max);
        // Step finely enough to land near the diamond's corners at t = k*pi/2.
        for (double t = 0.0; t <= 30.0; t += 0.005) {
            const RefPoint r = traj.at(t);
            REQUIRE(std::isfinite(r.x));
            REQUIRE(std::isfinite(r.y));
            REQUIRE(std::isfinite(r.theta));
            REQUIRE(std::isfinite(r.v));
            REQUIRE(std::isfinite(r.w));
            CHECK(std::abs(r.v) <= p.v_max + 1e-9);
            CHECK(std::abs(r.w) <= p.w_max + 1e-9);
        }
    }
}

TEST_CASE("DDMR trajectory: circle feedforward matches the analytic v_r and omega_r",
          "[ddmr][traj]") {
    const PlantParams p = nominalParams();
    Trajectory traj(PathType::Circle, 3.0, 1.0, p.v_max, p.w_max);
    // For x = a cos t, y = a sin t: |v| = a = 3.0 m/s and omega = 1.0 rad/s, both under clamp.
    for (double t = 0.3; t < 6.0; t += 0.7) {
        const RefPoint r = traj.at(t);
        CHECK(r.v == Catch::Approx(3.0).epsilon(1e-4));
        CHECK(r.w == Catch::Approx(1.0).epsilon(1e-3));
    }
}

// ===========================================================================
// Roster
// ===========================================================================
TEST_CASE("DDMR roster: exactly 12 controllers with unique names", "[ddmr][roster]") {
    auto ctrls = roster();
    REQUIRE(ctrls.size() == 12);

    std::vector<std::string> names;
    for (auto& c : ctrls) names.push_back(c->name());
    for (size_t i = 0; i < names.size(); ++i)
        for (size_t j = i + 1; j < names.size(); ++j)
            CHECK(names[i] != names[j]);

    // The paper's proposed method and its three Table 3 comparators must all be present.
    auto has = [&](const std::string& n) {
        for (const auto& s : names) if (s == n) return true;
        return false;
    };
    CHECK(has("FUHAC"));
    CHECK(has("PID"));
    CHECK(has("Backstepping"));
    CHECK(has("SMC"));
}

TEST_CASE("DDMR roster: every controller runs a full circle with finite output",
          "[ddmr][roster][smoke]") {
    const PlantParams p = nominalParams();
    Scenario s = circleScenario(/*off_path=*/false);

    for (auto& c : roster()) {
        INFO("controller = " << c->name());
        const RunMetrics m = runSimulation(p, s, *c, "");   // empty log_dir = no file I/O
        CHECK(std::isfinite(m.ise));
        CHECK(std::isfinite(m.iae));
        CHECK(std::isfinite(m.itae));
        CHECK(std::isfinite(m.final_err));
        CHECK(std::isfinite(m.mean_torque));
        CHECK(m.iae >= 0.0);
        CHECK(m.max_torque <= p.tau_max + 1e-9);   // saturation must never be exceeded
    }
}

TEST_CASE("DDMR roster: non-finite error input holds the feedforward (NaN contract)",
          "[ddmr][roster][nan]") {
    const PlantParams p = nominalParams();
    BodyError bad;
    bad.e1 = std::numeric_limits<double>::quiet_NaN();
    bad.e2 = 0.1; bad.e3 = 0.05; bad.de1 = 0.0;

    // The composite / hand-written controllers guarantee the hold explicitly.
    for (const std::string& n : {"Backstepping", "FuzzyTSK", "LQR", "NMPC",
                                 "L1Adaptive", "GainScheduled", "FUHAC"}) {
        auto c = byName(n);
        REQUIRE(c != nullptr);
        INFO("controller = " << n);
        const Eigen::Vector2d u = c->compute(bad, 1.5, 0.5);
        CHECK(u(0) == Catch::Approx(1.5));
        CHECK(u(1) == Catch::Approx(0.5));
    }
}

TEST_CASE("DDMR roster: OpenLoop is pure feedforward", "[ddmr][roster]") {
    auto c = byName("OpenLoop");
    REQUIRE(c != nullptr);
    BodyError e;
    e.e1 = 0.7; e.e2 = -0.3; e.e3 = 0.2;
    const Eigen::Vector2d u = c->compute(e, 1.25, -0.75);
    CHECK(u(0) == Catch::Approx(1.25));
    CHECK(u(1) == Catch::Approx(-0.75));
}

// ===========================================================================
// Convergence
// ===========================================================================
TEST_CASE("DDMR convergence: feedback controllers recover from an off-path start",
          "[ddmr][convergence]") {
    const PlantParams p = nominalParams();
    const Scenario s = circleScenario(/*off_path=*/true);

    for (const std::string& n : {"Backstepping", "SMC", "AdaptiveSMC", "FUHAC"}) {
        auto c = byName(n);
        REQUIRE(c != nullptr);
        INFO("controller = " << n);
        const RunMetrics m = runSimulation(p, s, *c, "");
        CHECK(m.final_err < 0.10);       // back on the path within 12 s
        CHECK(std::isfinite(m.ise));
    }
}

TEST_CASE("DDMR convergence: closed loop beats open loop from an off-path start",
          "[ddmr][convergence]") {
    const PlantParams p = nominalParams();
    const Scenario s = circleScenario(/*off_path=*/true);

    auto ol = byName("OpenLoop");
    auto fu = byName("FUHAC");
    REQUIRE(ol != nullptr);
    REQUIRE(fu != nullptr);

    const RunMetrics m_ol = runSimulation(p, s, *ol, "");
    const RunMetrics m_fu = runSimulation(p, s, *fu, "");
    CHECK(m_fu.ise < m_ol.ise);
    CHECK(m_fu.final_err < m_ol.final_err);
}

TEST_CASE("DDMR convergence: FUHAC beats plain PID on the hard diamond case",
          "[ddmr][convergence]") {
    // Paper Table 3 reports PID ISE 14.7 vs FUHAC 3.7. The absolute numbers depend on the
    // assumed physical parameters, so only the ORDERING is asserted - and it is asserted on
    // the DIAMOND (sharp corners + offset start), not on a smooth path.
    //
    // On the noiseless lemniscate a well-tuned PID actually edges FUHAC out in this
    // reproduction (ISE 0.189 vs 0.233): with no uncertainty to reject, FUHAC's always-on
    // sliding term is pure overhead. That is the same trade the paper itself concedes when it
    // reports FUHAC (3.7) as worse than pure backstepping (3.3) on raw ISE and justifies it on
    // chattering and actuator health instead. FUHAC's advantage is real where the paper claims
    // it - nonlinearity and uncertainty - so that is what this guard pins.
    const PlantParams p = nominalParams();
    Scenario s;
    s.id            = "test_diamond";
    s.trajectory    = "diamond";
    s.a             = 1.0;
    s.T_sim         = 20.0;
    s.start_on_path = false;
    s.x0            = 0.6;
    s.y0            = -0.4;
    s.theta0        = 1.2;

    auto pid = byName("PID");
    auto fu  = byName("FUHAC");
    REQUIRE(pid != nullptr);
    REQUIRE(fu != nullptr);

    const RunMetrics m_pid = runSimulation(p, s, *pid, "");
    const RunMetrics m_fu  = runSimulation(p, s, *fu, "");
    INFO("PID ISE = " << m_pid.ise << ", FUHAC ISE = " << m_fu.ise);
    CHECK(m_fu.ise < m_pid.ise);
}

// ===========================================================================
// FUHAC internals
// ===========================================================================
TEST_CASE("DDMR FUHAC: alpha stays inside [alpha_min, alpha_max]", "[ddmr][fuhac]") {
    auto c = byName("FUHAC");
    REQUIRE(c != nullptr);

    BodyError e;
    for (int k = 0; k < 500; ++k) {
        e.e1  = 0.5 * std::sin(0.05 * k);
        e.e2  = 0.3 * std::cos(0.07 * k);
        e.e3  = 0.2 * std::sin(0.03 * k);
        e.de1 = 0.1 * std::cos(0.05 * k);
        c->compute(e, 1.5, 0.5);
        if (k % 5 == 0) c->slowTick();

        const CtrlTelemetry t = c->telemetry();
        REQUIRE(std::isfinite(t.alpha));
        CHECK(t.alpha >= 0.25 - 1e-9);
        CHECK(t.alpha <= 0.95 + 1e-9);
    }
}

TEST_CASE("DDMR FUHAC: adaptive sliding gain Ks stays within [3.0, 5.0]", "[ddmr][fuhac]") {
    // Paper Table 1 sets Ks in 3.0-4.6; the AdaptiveSMC clamp is [Kmin, Kmax] = [3.0, 5.0].
    auto c = byName("FUHAC");
    REQUIRE(c != nullptr);
    CHECK(c->telemetry().Ks == Catch::Approx(3.0));   // K0 after reset()

    BodyError e;
    for (int k = 0; k < 1000; ++k) {
        e.e1 = 0.4; e.e2 = 0.3; e.e3 = 0.25; e.de1 = 0.0;   // persistent |s| > epsilon
        c->compute(e, 1.5, 0.5);
        const double Ks = c->telemetry().Ks;
        REQUIRE(std::isfinite(Ks));
        CHECK(Ks >= 3.0 - 1e-9);
        CHECK(Ks <= 5.0 + 1e-9);
    }
    // Persistent off-surface error must have driven the gain up from K0.
    CHECK(c->telemetry().Ks > 3.0);
}

TEST_CASE("DDMR FUHAC: Ks is non-decreasing while the surface stays outside the dead-band",
          "[ddmr][fuhac]") {
    auto c = byName("FUHAC");
    REQUIRE(c != nullptr);

    BodyError e;
    e.e1 = 0.2; e.e2 = 0.5; e.e3 = 0.4; e.de1 = 0.0;   // |s| = |e2 + 0.8*e3| = 0.82 >> eps
    double prev = c->telemetry().Ks;
    for (int k = 0; k < 200; ++k) {
        c->compute(e, 1.0, 0.0);
        const double now = c->telemetry().Ks;
        CHECK(now >= prev - 1e-12);
        prev = now;
    }
}

TEST_CASE("DDMR FUHAC: composite Lyapunov value is finite and decays on the circle",
          "[ddmr][fuhac]") {
    auto c = byName("FUHAC");
    REQUIRE(c != nullptr);

    // Shrinking error should shrink V - it is a positive-definite function of the error,
    // weight, sliding and observer energies.
    BodyError e;
    double V_first = 0.0, V_last = 0.0;
    for (int k = 0; k < 400; ++k) {
        const double decay = std::exp(-0.01 * k);
        e.e1 = 0.8 * decay; e.e2 = 0.6 * decay; e.e3 = 0.4 * decay; e.de1 = 0.0;
        c->compute(e, 1.5, 0.5);
        const double V = c->telemetry().V;
        REQUIRE(std::isfinite(V));
        CHECK(V >= 0.0);
        if (k == 20)  V_first = V;
        if (k == 399) V_last = V;
    }
    CHECK(V_last < V_first);
}

TEST_CASE("DDMR FUHAC: reset() restores the initial adaptive state", "[ddmr][fuhac]") {
    auto c = byName("FUHAC");
    REQUIRE(c != nullptr);

    BodyError e;
    e.e1 = 0.5; e.e2 = 0.4; e.e3 = 0.3; e.de1 = 0.2;
    for (int k = 0; k < 300; ++k) c->compute(e, 1.5, 0.5);
    REQUIRE(c->telemetry().Ks > 3.0);

    c->reset();
    const CtrlTelemetry t = c->telemetry();
    CHECK(t.Ks == Catch::Approx(3.0));
    CHECK(t.d_hat == Catch::Approx(0.0));
    CHECK(t.alpha == Catch::Approx(0.95));    // alpha_max after reset
}

TEST_CASE("DDMR FUHAC: disturbance estimate stays bounded by d_max", "[ddmr][fuhac]") {
    auto c = byName("FUHAC");
    REQUIRE(c != nullptr);

    BodyError e;
    for (int k = 0; k < 2000; ++k) {
        e.e1 = 5.0;  e.e2 = 0.0; e.e3 = 0.0; e.de1 = 0.0;   // large persistent bias
        c->compute(e, 1.5, 0.5);
        const double d = c->telemetry().d_hat;
        REQUIRE(std::isfinite(d));
        CHECK(std::abs(d) <= 2.0 + 1e-9);
    }
}

TEST_CASE("DDMR AdaptiveSMC roster entry also publishes its adaptive gain", "[ddmr][fuhac]") {
    auto c = byName("AdaptiveSMC");
    REQUIRE(c != nullptr);
    CHECK(c->telemetry().Ks == Catch::Approx(3.0));

    BodyError e;
    e.e1 = 0.0; e.e2 = 0.6; e.e3 = 0.5; e.de1 = 0.0;
    for (int k = 0; k < 500; ++k) c->compute(e, 1.0, 0.0);
    CHECK(c->telemetry().Ks > 3.0);
    CHECK(c->telemetry().Ks <= 5.0 + 1e-9);
}

// ===========================================================================
// Scenario plumbing
// ===========================================================================
TEST_CASE("DDMR scenarios: all five ship and parse", "[ddmr][config]") {
    const std::string dir = std::string(DDMR_SIM_SOURCE_DIR) + "/config/scenarios/";
    const char* files[] = {"s01_lemniscate.json",
                           "s02_circle.json",
                           "s03_diamond_offset_start.json",
                           "s04_noise_disturbance.json",
                           "s05_saturation_mismatch.json"};
    for (const char* f : files) {
        INFO("scenario = " << f);
        Scenario s;
        REQUIRE_NOTHROW(s = Scenario::fromJson(dir + f));
        CHECK(!s.id.empty());
        CHECK(!s.description.empty());
        CHECK(s.T_sim > 0.0);
        CHECK(s.a > 0.0);
    }
}

TEST_CASE("DDMR config: plant_params.json matches the struct defaults", "[ddmr][config]") {
    const std::string path = std::string(DDMR_SIM_SOURCE_DIR) + "/config/plant_params.json";
    PlantParams j;
    REQUIRE_NOTHROW(j = PlantParams::fromJson(path));
    const PlantParams d = nominalParams();

    CHECK(j.M_total == Catch::Approx(d.M_total));
    CHECK(j.r_wheel == Catch::Approx(d.r_wheel));
    CHECK(j.R_half_axle == Catch::Approx(d.R_half_axle));
    CHECK(j.Kf == Catch::Approx(d.Kf));
    CHECK(j.d_com == Catch::Approx(d.d_com));
    CHECK(j.tau_max == Catch::Approx(d.tau_max));
    CHECK(j.Tf == Catch::Approx(d.Tf));
    CHECK(j.Ts_slow == Catch::Approx(d.Ts_slow));
}

TEST_CASE("DDMR scenarios: overrides reach the simulated plant only", "[ddmr][config]") {
    const PlantParams nominal = nominalParams();
    Scenario s;
    s.tau_max_override = 5.0;
    s.mass_mismatch    = 1.5;

    const PlantParams eff = effectivePlantParams(nominal, s);
    CHECK(eff.tau_max == Catch::Approx(5.0));
    CHECK(eff.M_total == Catch::Approx(nominal.M_total * 1.5));
    // Timing and design-side limits must be untouched.
    CHECK(eff.Tf == Catch::Approx(nominal.Tf));
    CHECK(eff.v_max == Catch::Approx(nominal.v_max));
}

TEST_CASE("DDMR scenarios: a tighter torque limit is actually respected", "[ddmr][config]") {
    const PlantParams p = nominalParams();
    Scenario s = circleScenario(/*off_path=*/true);
    s.tau_max_override = 4.0;

    auto c = byName("FUHAC");
    REQUIRE(c != nullptr);
    const RunMetrics m = runSimulation(p, s, *c, "");
    CHECK(m.max_torque <= 4.0 + 1e-9);
}

TEST_CASE("DDMR runner: results are deterministic for a fixed seed", "[ddmr][runner]") {
    const PlantParams p = nominalParams();
    Scenario s = circleScenario(/*off_path=*/true);
    s.encoder_noise_std = 0.01;
    s.seed = 12345;

    auto a = byName("FUHAC");
    auto b = byName("FUHAC");
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);

    const RunMetrics ma = runSimulation(p, s, *a, "");
    const RunMetrics mb = runSimulation(p, s, *b, "");
    CHECK(ma.ise == Catch::Approx(mb.ise));
    CHECK(ma.iae == Catch::Approx(mb.iae));
    CHECK(ma.final_err == Catch::Approx(mb.final_err));
}
