/**
 * ex36_feedforward_mpc.cpp
 * MPC + FeedforwardController for measured disturbance rejection.
 *
 * A static feedforward gain cancels a known load disturbance while MPC
 * handles setpoint tracking. Demonstrates ControllerStack::Additive.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.1;

    // Plant: G(s) = 1/(s+1), discretised ZOH
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    // MPC for setpoint tracking
    ctrl::MPCParams mp;
    mp.Np    = 12;
    mp.Nc    = 4;
    mp.rho_y = 1.0;
    mp.rho_u = 0.1;
    auto mpc_ctrl = std::make_shared<ctrl::DiscreteMPC>(sys, mp);

    // Feedforward: static gain = 1 / K_plant to cancel step disturbance d
    // G_ff(z) = 1 (unity static gain), then scale inside
    Eigen::MatrixXd A_ff(1,1); A_ff(0,0) = 0.0;
    Eigen::MatrixXd B_ff(1,1); B_ff(0,0) = 0.0;
    Eigen::MatrixXd C_ff(1,1); C_ff(0,0) = 0.0;
    Eigen::MatrixXd D_ff(1,1); D_ff(0,0) = 1.0;   // K_ff = 1.0 (= 1/K_plant)
    ctrl::StateSpace Gff(A_ff, B_ff, C_ff, D_ff, Ts);
    auto ff_ctrl = std::make_shared<ctrl::FeedforwardController>(Gff);

    // Additive stack: u_total = u_mpc + u_ff
    ctrl::ControllerStack stack(ctrl::StackMode::Additive, Ts);
    stack.addController(mpc_ctrl, "MPC");
    stack.addController(ff_ctrl,  "FF");

    // Simulate: step ref=1 at k=0, disturbance d=0.5 applied at k=30
    Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
    double y = 0.0;
    const double ref = 1.0;
    double final_y = 0.0;

    for (int k = 0; k < 200; ++k) {
        const double d   = (k >= 30) ? 0.5 : 0.0;
        // MPC computes from error; FF computes from disturbance signal
        // error signal drives MPC, disturbance drives FF
        const double u_total = mpc_ctrl->compute(ref - y) + ff_ctrl->compute(d);

        Eigen::VectorXd uv(1); uv(0) = u_total;
        const Eigen::VectorXd yv = ctrl::ssStep(sys, x, uv);
        y = yv(0) + d;  // output disturbed by d

        final_y = y;
    }

    std::cout << "FF+MPC final y = " << final_y << "  (ref = " << ref << ")\n";

    if (std::abs(final_y - ref) < 0.1)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return 0;
}
