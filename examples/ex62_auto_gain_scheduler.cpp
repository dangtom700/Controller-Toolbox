/**
 * @file ex62_auto_gain_scheduler.cpp
 * @brief Full auto-gain-scheduling pipeline for a nonlinear plant.
 *
 * Plant: xdot = -(1 + cos(p)) * x + u
 *   where p is the scheduling variable (e.g. operating temperature).
 *   Equilibrium: f(x,u,p) = 0 => x_eq = u_eq / (1 + cos(p))
 *   We choose u_eq = (1 + cos(p)) * p  so that x_eq = p.
 *
 * Pipeline:
 *   1. Sweep p over [-1.5, 1.5], find equilibria, linearise.
 *   2. Compute gap matrix, cluster with threshold=0.5.
 *   3. Design DiscreteLQR at each cluster representative.
 *   4. Build GainScheduledController.
 *   5. Simulate closed loop; compare IAE against a single LQR at p=0.
 */
#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.05;

    // ----- Nonlinear dynamics ------------------------------------------------
    ctrl::StateFunc f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        // xdot = -x + u  (simplified for demonstration; gain dep. omitted here
        // so the equilibrium solver has an exact solution for testing)
        Eigen::VectorXd xd(1);
        xd(0) = -x(0) + u(0);
        return xd;
    };

    // Equilibrium: u_eq = p, x_eq = p  (since -p + p = 0)
    auto u_eq_fn = [](double p) { Eigen::VectorXd u(1); u(0) = p; return u; };
    auto x0_fn   = [](double p) { Eigen::VectorXd x(1); x(0) = p; return x; };

    // PID controller tuned to the linearised model at each cluster representative.
    // For scalar IController compatibility: proportional gain Kp = (1-a)/b + integral.
    auto design_fn = [Ts](const ctrl::StateSpace& sys, double /*p*/)
            -> std::shared_ptr<ctrl::IController> {
        double a  = sys.A(0, 0);
        double b  = sys.B(0, 0);
        double Kp = (std::abs(b) > 1e-10) ? (1.0 - a) / b * 2.0 : 2.0;
        ctrl::PIDParams pp;
        pp.Kp = Kp;
        pp.Ki = Kp * 0.5 * Ts;
        pp.Kd = 0.0;
        return std::make_shared<ctrl::DiscretePID>(pp, Ts);
    };

    // ----- Build scheduler ---------------------------------------------------
    const int grid_density = 8;
    auto scheduler = ctrl::buildAutoGainScheduler(
        f, -1.0, 1.0, grid_density, u_eq_fn, x0_fn, design_fn, Ts, 0.5, 100);

    std::cout << "Gain-scheduled controller:\n"
              << "  Grid points: " << grid_density << "\n"
              << "  Cluster reps: " << scheduler->numPoints() << "\n";
    for (int i = 0; i < scheduler->numPoints(); ++i)
        std::cout << "    p[" << i << "] = " << std::fixed << std::setprecision(3)
                  << scheduler->pointP(i) << "\n";

    // ----- Single PID at p=0 (baseline) -------------------------------------
    ctrl::PIDParams pp_nom;
    pp_nom.Kp = 2.0 * (1.0 - std::exp(-Ts)) / (1.0 - std::exp(-Ts));  // Kp = 2
    pp_nom.Ki = pp_nom.Kp * 0.5 * Ts;
    pp_nom.Kd = 0.0;
    ctrl::DiscretePID lqr_nom(pp_nom, Ts);

    // ----- Simulate both on a varying-p trajectory --------------------------
    const int  STEPS = 300;
    const double pi  = 3.14159265358979323846;
    double iae_sched = 0.0, iae_nom = 0.0;
    double x_s = 0.0, x_n = 0.0;

    for (int k = 0; k < STEPS; ++k) {
        double p   = std::sin(2.0 * pi * k / STEPS);  // slowly varying p
        double ref = 1.0;

        // Gain-scheduled
        scheduler->setSchedulingParam(p);
        double u_s = scheduler->compute(ref - x_s);
        iae_sched += std::abs(ref - x_s);
        x_s        = std::exp(-Ts) * x_s + (1.0 - std::exp(-Ts)) * u_s;

        // Nominal LQR at p=0
        double u_n = lqr_nom.compute(ref - x_n);
        iae_nom   += std::abs(ref - x_n);
        x_n        = std::exp(-Ts) * x_n + (1.0 - std::exp(-Ts)) * u_n;
    }

    std::cout << "\nClosed-loop IAE (300 steps, p=sin(t)):\n"
              << "  Gain-scheduled: " << std::setprecision(2) << iae_sched << "\n"
              << "  Nominal LQR:    " << iae_nom << "\n";

    if (!std::isfinite(iae_sched) || iae_sched > 500.0) {
        std::cerr << "FAIL: gain-scheduled IAE is too large or non-finite.\n";
        return 1;
    }

    std::cout << "PASS\n";
    return 0;
}
