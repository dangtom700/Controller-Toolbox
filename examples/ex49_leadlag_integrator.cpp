/**
 * ex49_leadlag_integrator.cpp
 * ADDITIVE: LeadLag (phase advance) + pure integral controller.
 *
 * A pure integral controller (I-only) provides zero steady-state error but
 * poor phase margin near crossover. A Lead-Lag compensator in additive mode
 * restores phase margin without modifying the integrator's steady-state properties.
 *
 * u_total = u_I + u_leadlag
 *   u_I      = Ki * integral(e)  (zero steady-state error, poor phase)
 *   u_leadlag = LeadLag(e)        (phase advance at crossover, improves stability)
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.02;

    // Pure integral controller: Ki * int(e)
    const double Ki = 0.8;
    double integral  = 0.0;

    // Lead-Lag compensator (additive phase advance near crossover)
    ctrl::LeadLagParams llp;
    // Lead: zero at 0.7 rad/s, pole at 3.5 rad/s (lead ratio ~5)
    llp.continuousZero = 0.7;   // zero frequency [rad/s]
    llp.continuousPole = 3.5;   // pole frequency [rad/s]
    llp.gain           = 1.2;   // DC gain
    ctrl::DiscreteLeadLag lead_lag(llp, Ts);

    // Plant: G(s) = 1/(s*(s+0.5)) - plant with inherent integrator (double integrator-like)
    // Simplified: use first-order G(s) = 1/(s+0.2) (slow stable plant)
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-0.2),
        Eigen::MatrixXd::Constant(1,1, 0.2),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
    double y = 0.0;
    const double ref = 1.0;
    // G(s)=1/(s+0.2) has tau=5s; underdamped closed-loop has settling ~50s.
    // 800 steps = 16s is only 3 time constants, insufficient. 3000 steps = 60s settles to <2%.
    const int N = 3000;

    for (int k = 0; k < N; ++k) {
        const double e = ref - y;

        // Pure integral output
        integral     += e * Ts;
        const double u_I  = Ki * integral;

        // Lead-lag additive correction
        const double u_ll = lead_lag.compute(e);

        // Additive combination
        const double u_total = u_I + 0.5 * u_ll;

        Eigen::VectorXd uv(1); uv(0) = u_total;
        y = ctrl::ssStep(sys, x, uv)(0);
    }

    std::cout << "ADDITIVE LeadLag+Integrator: final y = " << y
              << "  (ref = " << ref << ")\n";

    if (std::abs(y - ref) < 0.05)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
