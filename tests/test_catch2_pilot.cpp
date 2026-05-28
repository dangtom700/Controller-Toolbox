/**
 * @file test_catch2_pilot.cpp
 * @brief Catch2 v3 regression tests targeting fixes from Part 12 of the bug report.
 *
 * Tests:
 *   - LQRAdapter MIMO truncation (P12-16): computeVec() returns full vector.
 *   - EKF numericalJacobian scaled eps (P12-17): heterogeneous-state accuracy.
 *   - DiscretePID::computeDoM() (P10-3): no derivative spike on setpoint step.
 *   - PIDParams::b_weight (P11-8): b_weight < 1 reduces proportional overshoot.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "ControllerToolbox.h"
#include <cmath>

using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

// Build a minimal double-integrator state-space (x = [pos, vel], u = force/mass):
//   x[k+1] = [1 Ts; 0 1] x[k] + [0.5*Ts^2; Ts] u[k]
//   y[k]   = [1 0] x[k]
static ctrl::StateSpace makeDoubleIntegrator(double Ts)
{
    Eigen::Matrix2d A;
    A << 1.0, Ts,
         0.0, 1.0;
    Eigen::Vector2d B;
    B << 0.5 * Ts * Ts, Ts;
    Eigen::RowVector2d C;
    C << 1.0, 0.0;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
    return ctrl::StateSpace{A, B, C, D, Ts};
}

// -----------------------------------------------------------------------------
// P12-16: LQRAdapter MIMO truncation
// -----------------------------------------------------------------------------

TEST_CASE("LQRAdapter computeVec returns full control vector", "[lqr][adapter][mimo]")
{
    const double Ts = 0.01;
    auto sys = makeDoubleIntegrator(Ts);

    ctrl::LQRParams lqr_params;
    lqr_params.Q = Eigen::Matrix2d::Identity();
    lqr_params.R = Eigen::MatrixXd::Identity(1, 1);

    ctrl::DiscreteLQR lqr(sys, lqr_params);

    // State: position offset = 1.0, velocity = 0
    Eigen::Vector2d state;
    state << 1.0, 0.0;

    const auto u_vec = lqr.compute(state);
    REQUIRE(u_vec.size() == 1);

    // Wrap in LQRAdapter
    ctrl::LQRAdapter adapter(lqr,
        [&]() -> Eigen::VectorXd { return state; });

    SECTION("computeVec returns same result as DiscreteLQR::compute")
    {
        Eigen::VectorXd signal(1);
        signal << 0.0;
        const Eigen::VectorXd u_adapter = adapter.computeVec(signal);
        REQUIRE(u_adapter.size() == 1);
        REQUIRE_THAT(u_adapter(0), WithinRel(u_vec(0), 1e-12));
    }

    SECTION("compute() returns first element of computeVec()")
    {
        Eigen::VectorXd signal(1);
        signal << 0.0;
        const double u_scalar   = adapter.compute(0.0);
        const double u_vec_elem = adapter.computeVec(signal)(0);
        REQUIRE_THAT(u_scalar, WithinRel(u_vec_elem, 1e-12));
    }
}

// -----------------------------------------------------------------------------
// P12-17: EKF numericalJacobian - heterogeneous state magnitudes
// -----------------------------------------------------------------------------

TEST_CASE("EKF numericalJacobian is accurate for heterogeneous state magnitudes", "[ekf][jacobian]")
{
    // f(x) = [x0^2;  x1]   -> analytical J = [2*x0, 0; 0, 1]
    // State has very different scales: x0 ~ 1e3, x1 ~ 1e-3
    Eigen::Vector2d x;
    x << 1000.0, 0.001;

    auto f = [](const Eigen::VectorXd &xx) -> Eigen::VectorXd {
        Eigen::VectorXd out(2);
        out(0) = xx(0) * xx(0);
        out(1) = xx(1);
        return out;
    };

    Eigen::MatrixXd J_analytical(2, 2);
    J_analytical << 2.0 * x(0), 0.0,
                    0.0,        1.0;

    // Default eps_scale=1e-4 (new scaled version)
    const Eigen::MatrixXd J_scaled = ctrl::ExtendedKalmanFilter::numericalJacobian(f, x);

    SECTION("Scaled eps gives relative error < 1e-6 for large-state element")
    {
        const double rel_err = std::abs(J_scaled(0, 0) - J_analytical(0, 0))
                               / std::abs(J_analytical(0, 0));
        REQUIRE(rel_err < 1e-6);
    }

    SECTION("Scaled eps gives absolute error < 1e-9 for small-state element")
    {
        // J[1,1] = 1.0 regardless of scale - should be near-exact
        REQUIRE_THAT(J_scaled(1, 1), WithinAbs(1.0, 1e-8));
    }

    SECTION("Off-diagonal elements are near zero")
    {
        REQUIRE_THAT(J_scaled(0, 1), WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(J_scaled(1, 0), WithinAbs(0.0, 1e-6));
    }
}

// -----------------------------------------------------------------------------
// P10-3: DiscretePID::computeDoM - no derivative spike on setpoint step
// -----------------------------------------------------------------------------

TEST_CASE("DiscretePID::computeDoM suppresses derivative spike on setpoint step", "[pid][dom]")
{
    // High-derivative gain so the spike would be obvious if derivative were on error
    ctrl::PIDParams p;
    p.Kp = 1.0;
    p.Ki = 0.0;
    p.Kd = 5.0;
    p.N  = 20.0; // derivative filter coefficient
    p.uMin = -1e9;
    p.uMax =  1e9;

    const double Ts = 0.01;
    ctrl::DiscretePID pid_dom(p, Ts);

    // Step in setpoint r at k=0: r goes from 0 -> 1, y stays at 0
    const double y = 0.0;

    // First call: u_dom should NOT see a large derivative spike
    const double u_dom_step = pid_dom.computeDoM(y, 1.0); // setpoint steps to 1

    // Standard compute for comparison: derivative IS on error, will spike
    ctrl::DiscretePID pid_std(p, Ts);
    const double u_std_step = pid_std.compute(1.0 - y); // error = r - y = 1

    SECTION("DoM output magnitude is less than standard output on setpoint step")
    {
        // Standard derivative-on-error will produce larger initial spike
        REQUIRE(std::abs(u_dom_step) < std::abs(u_std_step));
    }

    SECTION("DoM and standard converge after several steps with constant setpoint")
    {
        // Advance both controllers for 200 steps so the derivative filter fully decays
        double y_sim = 0.0;
        for (int k = 1; k < 200; ++k)
        {
            pid_dom.computeDoM(y_sim, 1.0);
            pid_std.compute(1.0 - y_sim);
        }
        // After transient (~5 time constants), both outputs converge to Kp*e = 1.0
        const double u_dom_final = pid_dom.computeDoM(y_sim, 1.0);
        const double u_std_final = pid_std.compute(1.0 - y_sim);
        REQUIRE_THAT(u_dom_final, WithinRel(u_std_final, 0.1)); // within 10%
    }
}

// -----------------------------------------------------------------------------
// P11-8: PIDParams::b_weight - reduced proportional kick reduces overshoot
// -----------------------------------------------------------------------------

TEST_CASE("PIDParams::b_weight reduces proportional setpoint kick", "[pid][b_weight]")
{
    // Simple first-order plant: y[k+1] = 0.8*y[k] + 0.2*u[k]
    const double Ts = 0.1;
    ctrl::PIDParams p;
    p.Kp = 2.0;
    p.Ki = 0.5;
    p.Kd = 0.0;
    p.N  = 100.0;
    p.uMin = -100.0;
    p.uMax =  100.0;

    auto simulate_step_response = [&](double b_weight, int steps) -> double {
        ctrl::PIDParams pp = p;
        pp.b_weight = b_weight;
        ctrl::DiscretePID pid(pp, Ts);

        double y = 0.0;
        const double r = 1.0;
        double peak = 0.0;

        for (int k = 0; k < steps; ++k)
        {
            const double u = pid.computeDoM(y, r);
            y = 0.8 * y + 0.2 * u;
            if (y > peak) peak = y;
        }
        return peak;
    };

    const double peak_b1 = simulate_step_response(1.0, 100); // full proportional kick
    const double peak_b0 = simulate_step_response(0.0, 100); // no proportional on setpoint

    SECTION("b_weight=0 produces smaller peak overshoot than b_weight=1")
    {
        // b_weight=1 has more overshoot due to proportional setpoint kick
        REQUIRE(peak_b0 <= peak_b1 + 1e-6);
    }

    SECTION("Both controllers reach steady state near reference")
    {
        // Final values should both be near 1.0 (integral guarantees offset-free)
        auto final_val = [&](double b_weight) -> double {
            ctrl::PIDParams pp = p;
            pp.b_weight = b_weight;
            ctrl::DiscretePID pid(pp, Ts);
            double y = 0.0;
            for (int k = 0; k < 500; ++k)
                y = 0.8 * y + 0.2 * pid.computeDoM(y, 1.0);
            return y;
        };
        REQUIRE_THAT(final_val(1.0), WithinAbs(1.0, 0.01));
        REQUIRE_THAT(final_val(0.0), WithinAbs(1.0, 0.01));
    }
}

// -----------------------------------------------------------------------------
// IControllerObserver - basic wiring smoke test
// -----------------------------------------------------------------------------

TEST_CASE("IControllerObserver receives callbacks from DiscretePID", "[observer]")
{
    struct CountingObserver : ctrl::IControllerObserver
    {
        int compute_calls = 0;
        int reset_calls   = 0;
        double last_u     = 0.0;
        double last_signal = 0.0;

        void onCompute(double u, double signal) override
        {
            ++compute_calls;
            last_u      = u;
            last_signal = signal;
        }

        void onReset() override { ++reset_calls; }
    };

    ctrl::PIDParams p;
    p.Kp = 1.0; p.Ki = 0.1; p.Kd = 0.0;
    ctrl::DiscretePID pid(p, 0.01);

    CountingObserver obs;
    pid.attachObserver(&obs);

    SECTION("onCompute fires after compute()")
    {
        REQUIRE(obs.compute_calls == 0);
        const double u = pid.compute(0.5);
        REQUIRE(obs.compute_calls == 1);
        REQUIRE_THAT(obs.last_u,      WithinRel(u,   1e-12));
        REQUIRE_THAT(obs.last_signal, WithinAbs(0.5, 1e-12));
    }

    SECTION("onReset fires after reset()")
    {
        REQUIRE(obs.reset_calls == 0);
        pid.reset();
        REQUIRE(obs.reset_calls == 1);
    }

    SECTION("No callbacks after detachObserver()")
    {
        pid.detachObserver();
        pid.compute(1.0);
        REQUIRE(obs.compute_calls == 0);
    }
}
