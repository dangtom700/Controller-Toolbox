/**
 * ex43_smc_inner_lqr_outer.cpp
 * CASCADE: SMC (inner corrector) + LQR (outer nominal law).
 *
 * Architecture: LQR provides nominal state feedback for the outer double integrator.
 * SMC wraps the inner actuator to compensate friction/disturbance at the velocity level.
 * The SMC output is the actual input to the plant, while LQR's output provides the
 * target velocity that SMC tracks.
 *
 * LQR sees: [pos_error, vel_error] -> u_lqr (desired velocity command)
 * SMC inner: tracks velocity_ref = u_lqr, rejects matched disturbance
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;

    // Inner velocity plant: G_v(s) = 1/(0.2s+1) (motor with friction)
    ctrl::StateSpace sys_vel_c(
        Eigen::MatrixXd::Constant(1,1,-5.0),
        Eigen::MatrixXd::Constant(1,1, 5.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys_vel = ctrl::c2d(sys_vel_c, Ts, ctrl::C2dMethod::ZOH);

    // SMC for velocity control (inner loop)
    ctrl::SMCParams smc_p;
    smc_p.c_e  = 1.0;    // error weight in sliding surface
    smc_p.c_de = 10.0 * Ts;  // c_de = lambda * Ts (lambda=10 [1/s])
    smc_p.phi  = 0.5;    // boundary layer; 0.05 caused rapid chattering that prevented
                         // the inner loop tracking vel_ref cleanly - net position drift approx = 0
    smc_p.uMin = -20.0;
    smc_p.uMax =  20.0;
    ctrl::DiscreteSMC smc_inner(smc_p, Ts);

    // Outer position plant (double integrator): pos from velocity
    // pos[k+1] = pos[k] + Ts * vel[k]
    // Approx: outer LQR on position tracking
    const double Ts_o = 0.1;   // outer loop rate (10x slower)

    // LQR for position (simple P + D gain design)
    // K_pos chosen manually: [2.5, 0.8] for pos+vel error
    const Eigen::RowVector2d K_lqr(2.5, 0.8);

    double pos = 0.0, vel = 0.0;
    const double ref_pos = 1.0;
    const int N = 1000;  // 10 s at Ts=0.01; phi=0.5 gives tau_outer ~ 0.76 s -> 13 tau
    double final_pos = 0.0;

    for (int k = 0; k < N; ++k) {
        // LQR outer: compute desired velocity from position error (every Ts_o)
        // Simple: desired_vel = -K[0]*(pos - ref_pos) - K[1]*vel
        const double vel_ref = -K_lqr(0) * (pos - ref_pos) - K_lqr(1) * vel;

        // SMC inner: tracks vel_ref (error = vel_ref - vel, sign convention: y-ref for SMC)
        const double e_smc = vel - vel_ref;  // SMC uses compute(y - ref)
        const double u_smc = smc_inner.compute(e_smc);

        // Inner velocity plant step.
        // ssStep returns y[k]=C*x[k]+D*u (pre-update output) and updates x in place.
        // For this plant C=1, D=0 so y[k]=x[k]=old vel; use x_vel(0) after the call
        // to get the updated state x[k+1] = A*x[k] + B*u_smc.
        Eigen::VectorXd x_vel(1); x_vel(0) = vel;
        Eigen::VectorXd uv(1); uv(0) = u_smc;
        ctrl::ssStep(sys_vel, x_vel, uv);  // x_vel updated to x[k+1] in place
        vel = x_vel(0);

        // Outer position integration (Euler)
        pos += Ts * vel;
        final_pos = pos;
    }

    std::cout << "CASCADE SMC(inner)+LQR(outer): final pos = " << final_pos
              << "  (ref = " << ref_pos << ")\n";

    if (std::abs(final_pos - ref_pos) < 0.05)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
