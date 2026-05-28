// ============================================================
//  ex29_repetitive_controller.cpp
//  RepetitiveController learns to cancel a periodic disturbance.
//  Shows how the learning buffer converges over multiple periods.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <numbers>

int main()
{
    // First-order plant: G(s) = 2/(5s+1), ZOH @ Ts=0.1s
    const double Ts = 0.1;
    const double tau = 5.0, K_plant = 2.0;
    const double a = std::exp(-Ts / tau);
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A << a; B << K_plant*(1-a); C << 1.0; D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    // Base PID stabilising controller
    ctrl::PIDParams pp;
    pp.Kp = 1.5; pp.Ki = 0.3; pp.N = 10.0;
    pp.uMin = -10.0; pp.uMax = 10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    // Periodic disturbance: sine at 0.05 Hz, period T=20s -> N=200 steps
    const int period_steps = 200;
    const double omega_d = 2.0 * std::numbers::pi * 0.05;  // [rad/s]

    // RepetitiveController wraps PID and adds learned periodic correction
    ctrl::RepetitiveParams rp;
    rp.periodSteps = period_steps;
    rp.Krc = 0.4;   // learning gain - conservatively tuned for stability
    rp.Q   = 0.98;  // slight forgetting for robustness
    rp.uMin = -10.0; rp.uMax = 10.0;
    ctrl::RepetitiveController rc(pid, rp, Ts);

    const double r_sp = 1.0;
    Eigen::VectorXd x_plant(1); x_plant << 0.0;
    Eigen::VectorXd x_pid_only(1); x_pid_only << 0.0;
    ctrl::DiscretePID pid_only(pp, Ts);  // PID baseline for comparison

    std::cout << "=== RepetitiveController: Periodic Disturbance Rejection ===\n"
              << "  Disturbance: sin(2*pi*0.05*t), period=20s, " << period_steps << " steps/period\n\n"
              << std::setw(8) << "t[s]"
              << std::setw(12) << "dist"
              << std::setw(12) << "y_PID"
              << std::setw(12) << "y_RC"
              << std::setw(12) << "v_corr\n"
              << std::string(58, '-') << "\n";

    double IAE_pid = 0.0, IAE_rc = 0.0;
    const int N = period_steps * 5; // simulate 5 periods

    for (int k = 0; k < N; ++k) {
        const double t = k * Ts;
        const double dist = 1.5 * std::sin(omega_d * t);  // periodic disturbance

        // PID-only baseline
        Eigen::VectorXd u_pid_v(1); u_pid_v << pid_only.compute(r_sp - x_pid_only(0));
        Eigen::VectorXd y_pid_v = ctrl::ssStep(plant, x_pid_only, u_pid_v);
        x_pid_only(0) = x_pid_only(0); // ssStep already updated x_pid_only
        const double y_pid = y_pid_v(0) + dist * Ts;  // crude dist injection

        // RepetitiveController
        const double e_rc = r_sp - x_plant(0);
        const double u_rc = rc.compute(e_rc);
        Eigen::VectorXd u_rc_v(1); u_rc_v << u_rc;
        Eigen::VectorXd y_rc_v = ctrl::ssStep(plant, x_plant, u_rc_v);
        const double y_rc = y_rc_v(0);

        // Add disturbance directly to state for RC simulation
        x_plant(0) += dist * Ts * 0.5;  // approximate disturbance effect

        IAE_pid += std::abs(r_sp - y_pid) * Ts;
        IAE_rc  += std::abs(r_sp - y_rc)  * Ts;

        if (k % (period_steps / 4) == 0)
            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(8) << t
                      << std::setw(12) << dist
                      << std::setw(12) << y_pid
                      << std::setw(12) << y_rc
                      << std::setw(12) << rc.correction() << "\n";
    }

    std::cout << "\nIAE comparison over " << N * Ts << "s:\n"
              << "  PID only:           " << std::fixed << std::setprecision(3) << IAE_pid << "\n"
              << "  RepetitiveController: " << IAE_rc << "\n"
              << "  Reduction:          " << (1.0 - IAE_rc/IAE_pid)*100.0 << " %\n";
    return 0;
}
