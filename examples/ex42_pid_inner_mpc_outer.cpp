/**
 * ex42_pid_inner_mpc_outer.cpp
 * CASCADE: PID (inner, fast) + MPC (outer, slow).
 *
 * Inner PID (Ts=0.01 s) corrects fast actuator dynamics and disturbances.
 * Outer MPC (Ts=0.1 s) provides a setpoint to the inner PID based on the
 * slow outer-plant output. This is the standard inner/outer cascade architecture.
 *
 * Bandwidth rule satisfied: inner bandwidth >> outer bandwidth.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    // Inner plant: fast actuator G_i(s) = 1/(0.1s+1), Ts_i = 0.01 s
    const double Ts_i = 0.01;
    ctrl::StateSpace sys_i_c(
        Eigen::MatrixXd::Constant(1,1,-10.0),  // pole at -10 rad/s
        Eigen::MatrixXd::Constant(1,1,10.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys_i = ctrl::c2d(sys_i_c, Ts_i, ctrl::C2dMethod::ZOH);

    // Inner PID: fast tuning for the fast plant
    ctrl::PIDParams pp_i;
    pp_i.Kp = 5.0; pp_i.Ki = 20.0; pp_i.Kd = 0.05;
    pp_i.uMin = -20.0; pp_i.uMax = 20.0;
    ctrl::DiscretePID pid_inner(pp_i, Ts_i);

    // Outer plant: slow process G_o(s) = 1/(s+0.5), Ts_o = 0.1 s
    const double Ts_o = 0.1;
    ctrl::StateSpace sys_o_c(
        Eigen::MatrixXd::Constant(1,1,-0.5),
        Eigen::MatrixXd::Constant(1,1, 0.5),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys_o = ctrl::c2d(sys_o_c, Ts_o, ctrl::C2dMethod::ZOH);

    // Outer MPC: slow horizon for the slow process
    ctrl::MPCParams mp;
    mp.Np    = 15; mp.Nc = 4;
    mp.rho_y = 1.0; mp.rho_u = 0.5;
    ctrl::DiscreteMPC mpc_outer(sys_o, mp);

    // Simulation: outer ref = 1.0, 500 outer steps (50 s), 10 inner steps per outer
    Eigen::VectorXd x_i = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd x_o = Eigen::VectorXd::Zero(1);
    double y_i = 0.0, y_o = 0.0;
    const double outer_ref = 1.0;
    const int N_outer = 400;
    const int N_inner_per_outer = 10;

    for (int ko = 0; ko < N_outer; ++ko) {
        // Outer MPC computes setpoint for inner loop
        const double inner_sp = mpc_outer.compute(outer_ref - y_o);

        // Inner loop: run N_inner_per_outer steps to track inner_sp
        for (int ki = 0; ki < N_inner_per_outer; ++ki) {
            const double e_i = inner_sp - y_i;
            const double u_i = pid_inner.compute(e_i);
            Eigen::VectorXd uv(1); uv(0) = u_i;
            y_i = ctrl::ssStep(sys_i, x_i, uv)(0);
        }

        // Inner output drives outer plant as input
        Eigen::VectorXd uv_o(1); uv_o(0) = y_i;
        y_o = ctrl::ssStep(sys_o, x_o, uv_o)(0);
    }

    std::cout << "CASCADE PID(inner)+MPC(outer): final y_outer = " << y_o
              << "  (ref = " << outer_ref << ")\n";

    if (std::abs(y_o - outer_ref) < 0.05)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
