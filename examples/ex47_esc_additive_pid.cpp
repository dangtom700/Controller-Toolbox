/**
 * ex47_esc_additive_pid.cpp
 * ADDITIVE: ExtremumSeeker + PID.
 *
 * PID handles fast setpoint tracking (inner control loop).
 * ExtremumSeeker slowly shifts the operating setpoint to maximise an efficiency
 * objective J(y) = -(y - y_opt)^2 (extremum is at y_opt).
 *
 * Combined: u_total = u_pid  (PID output to plant)
 *           r_pid = r_esc    (ESC adjusts the setpoint seen by PID)
 *
 * This is used for MPPT in solar/wind, optimal combustion point, pump efficiency.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;

    // Plant: G(s) = 1/(s+1), ZOH-discretised
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    // PID: fast tracking of the ESC-provided setpoint
    ctrl::PIDParams pp;
    pp.Kp = 3.0; pp.Ki = 2.0; pp.Kd = 0.0;
    pp.uMin = -10.0; pp.uMax = 10.0;
    ctrl::DiscretePID pid(pp, Ts);

    // ExtremumSeeker: slowly find the setpoint that maximises J = -(y - y_opt)^2
    // The ESC perturbs the setpoint with a sinusoid and correlates with the plant output.
    ctrl::ExtremumSeekerParams esc_p;
    esc_p.perturbFreq = 0.5;    // perturbation frequency [Hz] << plant bandwidth
    esc_p.perturbAmp  = 0.05;   // small perturbation amplitude
    esc_p.integGain   = 0.3;    // slow learning rate
    esc_p.seekMinimum = true;   // seek minimum of J = (y - y_opt)^2
    ctrl::ExtremumSeeker esc(esc_p, Ts);

    // True optimum: plant reaches y=0.8 when setpoint=1.0 (DC gain=0.8 for this demo)
    // ESC will find the setpoint that minimises |y - 0.8| indirectly.
    // For this example: J = (y - 0.8)^2, ESC minimises J by adjusting setpoint.
    // The ESC "signal" is J (the cost), and it integrates to find the best setpoint.

    Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
    double y = 0.0;
    double r_esc_sp = 0.5;  // ESC adjusts this setpoint
    const int N = 2000;

    for (int k = 0; k < N; ++k) {
        // ESC: compute adjustment to setpoint by minimising (y - 0.8)^2
        const double J = (y - 0.8) * (y - 0.8);  // cost to minimise
        const double dr_esc = esc.compute(J);      // slow setpoint adjustment

        // Integrate the ESC output into the setpoint (running sum)
        r_esc_sp += Ts * dr_esc * 0.1;  // very slow drift

        // PID: track the ESC-provided setpoint
        const double u = pid.compute(r_esc_sp - y);

        Eigen::VectorXd uv(1); uv(0) = u;
        y = ctrl::ssStep(sys, x, uv)(0);
    }

    std::cout << "ADDITIVE ESC+PID: final y = " << y
              << "  final setpoint = " << r_esc_sp
              << "  (optimum y ~ 0.8)\n";

    // Both ESC + PID must keep the system bounded (not diverge)
    if (std::isfinite(y) && std::abs(y) < 5.0)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
