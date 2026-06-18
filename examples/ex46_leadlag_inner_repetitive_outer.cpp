/**
 * ex46_leadlag_inner_repetitive_outer.cpp
 * CASCADE: LeadLag (inner phase corrector) + RepetitiveController (outer, periodic cancellation).
 *
 * Bandwidth rule:
 *   Lead-lag corrects phase lag at crossover, allowing the repetitive controller's
 *   learning to converge faster (stability condition: inner closed-loop BW > 5x RC period).
 *
 * Architecture:
 *   u_total = u_pid_baseline + RC_correction
 *   u_actual = LeadLag(u_total)   <- lead-lag sharpens the loop before the RC memory bank
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <numbers>

int main()
{
    const double Ts = 0.01;

    // Inner feedback PID (baseline)
    ctrl::PIDParams pp;
    pp.Kp = 2.0; pp.Ki = 0.5; pp.Kd = 0.0;
    pp.uMin = -10.0; pp.uMax = 10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    // Lead-Lag compensator (inner corrector, sharpens phase at periodic frequency)
    // Lead at the fundamental disturbance frequency 1 Hz = 2pi rad/s
    ctrl::LeadLagParams llp;
    // Lead compensator: zero at omega_z, pole at omega_p = alpha*omega_z (alpha>1 for lead)
    const double omega_z = 2.0 * std::numbers::pi;  // zero at 1 Hz = disturbance frequency
    llp.continuousZero = omega_z;
    llp.continuousPole = 4.0 * omega_z;  // alpha=4 lead ratio
    llp.gain  = 1.0;
    ctrl::DiscreteLeadLag lead_lag(llp, Ts);

    // Repetitive controller (outer, cancels periodic disturbance)
    const int period_steps = 100;  // 1 s period
    ctrl::RepetitiveParams rp;
    rp.periodSteps = period_steps;
    rp.Krc         = 0.6;
    rp.Q           = 0.97;
    ctrl::RepetitiveController rc(pid, rp, Ts);

    // Plant: G(s) = 1/(s+1), ZOH
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
    double y = 0.0;
    const double ref = 1.0;
    const double distAmp = 0.4;
    const double distFreq = 2.0 * std::numbers::pi;

    const int N = 500;
    double iae_first = 0.0, iae_last = 0.0;

    for (int k = 0; k < N; ++k) {
        const double d   = distAmp * std::sin(distFreq * k * Ts);
        const double e   = ref - (y + d);

        // RepetitiveController provides the error-corrected signal
        const double u_rc = rc.compute(e);
        // Lead-lag corrects phase for the output signal
        const double u_ll = lead_lag.compute(u_rc);

        Eigen::VectorXd uv(1); uv(0) = u_ll;
        y = ctrl::ssStep(sys, x, uv)(0);

        const double err = std::abs(e);
        if (k < 100) iae_first += err * Ts;
        if (k >= 400) iae_last  += err * Ts;
    }

    std::cout << "CASCADE LeadLag(inner)+RC(outer):\n";
    std::cout << "  IAE first 1s = " << iae_first << "\n";
    std::cout << "  IAE last  1s = " << iae_last  << "\n";

    if (iae_last < iae_first && std::isfinite(iae_last))
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
