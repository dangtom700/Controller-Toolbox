/**
 * ex56_feedback_linearisation.cpp
 * Demonstrates FeedbackLinearisationController on two nonlinear plants.
 *
 * System 1 - Cubic drift:  xdot = -x^3 + u
 *   f(x, u) = -x(0)^3,   g(x, u) = 1.0
 *   Inner PD: Kp=5, Ki=2, Kd=0  (stabilises the virtual integrator xdot_v = v)
 *   Target: y = x(0) -> 1.0 from x(0)=0.
 *   PASS when |y - 1.0| < 0.05 after 500 steps (Ts = 0.01 s).
 *
 * System 2 - Nonlinear pendulum:  xdot1=x2, xdot2 = -(g/L)sin(x1) - b.x2 + u
 *   Output y = x1 (angle). f = -(g/L)sin(x[0]) - b.x[1], g = 1.0
 *   NOTE: the angle output has relative degree 2 (u appears in xddot, not theta.),
 *   so the inner controller effectively acts on a double integrator. Gains are
 *   chosen to place closed-loop poles at s approx = {-1, -2+/-j} (tau_dominant approx = 1 s).
 *   Inner PID: Kp=8, Ki=4, Kd=6 (pole placement for double integrator).
 *   Target: x1 -> pi/6 (30^\circ) from x=(0,0).
 *   PASS when |x1 - pi/6| < 0.05 rad after 2000 steps (Ts = 0.01 s, 20 s total).
 *
 * Both systems use explicit (forward) Euler integration of the true nonlinear
 * plant - FL provides algebraic inversion of the nonlinear terms and delegates
 * to the inner controller for the resulting linear error dynamics.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <numbers>

int main()
{
    bool all_pass = true;
    const double Ts = 0.01;

    // =========================================================================
    // System 1: xdot = -x^3 + u
    // =========================================================================
    {
        // f: drift evaluated at (x, u_prev) -> scalar
        ctrl::FeedbackLinearisationController::DriftFn f =
            [](const Eigen::VectorXd &x, double /*u_prev*/) {
                return -x(0) * x(0) * x(0);   // -x^3
            };

        // g: input gain -> constant 1.0
        ctrl::FeedbackLinearisationController::GainFn g =
            [](const Eigen::VectorXd &, double) { return 1.0; };

        // Inner PID (acts as an integrator controller on the linearised system)
        ctrl::PIDParams pp;
        pp.Kp = 5.0;
        pp.Ki = 2.0;
        pp.Kd = 0.0;
        pp.N  = 100.0;
        pp.uMin = -100.0;
        pp.uMax =  100.0;
        auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);

        ctrl::FLParams flp;
        flp.uMin = -50.0;
        flp.uMax =  50.0;
        flp.regularisationEps = 1e-6;

        ctrl::FeedbackLinearisationController fl(f, g, inner, flp, Ts);

        const double ref = 1.0;
        Eigen::VectorXd x(1);
        x(0) = 0.0;

        for (int k = 0; k < 500; ++k)
        {
            fl.setState(x);
            const double u = fl.compute(ref - x(0));
            // Euler integration of xdot = -x^3 + u
            x(0) += Ts * (-x(0) * x(0) * x(0) + u);
        }

        const double err1 = std::abs(x(0) - ref);
        std::cout << "System 1 (xdot=-x^3+u): y=" << x(0)
                  << "  |err|=" << err1 << "  (need < 0.05)\n";

        if (err1 > 0.05 || !std::isfinite(x(0)))
        {
            std::cerr << "FAIL: cubic drift system did not converge.\n";
            all_pass = false;
        }
    }

    // =========================================================================
    // System 2: Nonlinear pendulum  xdot1=x2, xdot2=-(g/L)sin(x1)-b.x2+u
    // =========================================================================
    {
        const double grav = 9.81, L = 1.0, b_damp = 0.5;

        // f(x, u_prev) = -(g/L)sin(x1) - b.x2   (the nonlinear terms in xddot = torque)
        ctrl::FeedbackLinearisationController::DriftFn f =
            [grav, L, b_damp](const Eigen::VectorXd &x, double) {
                return -(grav / L) * std::sin(x(0)) - b_damp * x(1);
            };

        // g(x, u_prev) = 1.0  (u enters directly as acceleration)
        ctrl::FeedbackLinearisationController::GainFn g =
            [](const Eigen::VectorXd &, double) { return 1.0; };

        // Inner PID tuned for a double-integrator virtual plant (relative degree 2):
        // Desired poles: s = {-1, -2+j, -2-j} ->
        //   (s+1)(s^2+4s+5) = s^3 + 5s^2 + 9s + 5
        //   -> Kd=5, Kp=9, Ki=5
        ctrl::PIDParams pp;
        pp.Kp =  9.0;
        pp.Ki =  5.0;
        pp.Kd =  5.0;
        pp.N  = 100.0;
        pp.uMin = -50.0;
        pp.uMax =  50.0;
        auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);

        ctrl::FLParams flp;
        flp.uMin = -30.0;
        flp.uMax =  30.0;
        flp.regularisationEps = 1e-6;

        ctrl::FeedbackLinearisationController fl(f, g, inner, flp, Ts);

        const double ref = std::numbers::pi / 6.0;  // 30 degrees
        Eigen::VectorXd x(2);
        x << 0.0, 0.0;

        // 2000 steps = 20 s; dominant pole tau = 1 s -> 20 time constants
        for (int k = 0; k < 2000; ++k)
        {
            fl.setState(x);
            const double u = fl.compute(ref - x(0));

            // Euler integration of the true pendulum
            const double xdot0 = x(1);
            const double xdot1 = -(grav / L) * std::sin(x(0)) - b_damp * x(1) + u;
            x(0) += Ts * xdot0;
            x(1) += Ts * xdot1;
        }

        const double err2 = std::abs(x(0) - ref);
        std::cout << "System 2 (pendulum): theta=" << x(0) * 180.0 / std::numbers::pi
                  << " deg  ref=" << ref * 180.0 / std::numbers::pi
                  << " deg  |err|=" << err2 << " rad  (need < 0.05)\n";

        if (err2 > 0.05 || !std::isfinite(x(0)))
        {
            std::cerr << "FAIL: pendulum did not reach reference angle.\n";
            all_pass = false;
        }
    }

    if (all_pass)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return all_pass ? 0 : 1;
}
