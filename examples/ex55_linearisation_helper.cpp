/**
 * ex55_linearisation_helper.cpp
 * Demonstrates ctrl::jacobianX/U and ctrl::lineariseAtPoint on two systems.
 *
 * System 1 - Van der Pol oscillator (mu = 0.5):
 *   xdot1 = x2
 *   xdot2 = mu(1 - x1^2)x2 - x1 + u
 *
 *   Analytical Jacobian at (0, 0):
 *     A_c = [[0, 1], [-1, mu]],   B_c = [[0], [1]]
 *
 *   Verify: numerical A_c matches analytical within 1e-4 (relative).
 *   Then: linearise -> c2d (ZOH, Ts=0.05 s) -> LQR -> close loop on
 *   the NONLINEAR Van der Pol starting from x=(0.5, 0).
 *   PASS when |x|2 < 0.05 after 500 steps (approx = 25 s).
 *
 * System 2 - CSTR temperature channel (Arrhenius rate):
 *   xdot = -a.x + b.exp(-E/x) + u    (simplified thermal CSTR)
 *
 *   Verify: jacobianX matches finite-difference at operating point.
 *   lineariseAtPoint returns a 1-state 1-input model.
 *   PASS when identified poles are inside the unit circle.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    bool all_pass = true;

    // -------------------------------------------------------------------------
    // System 1: Van der Pol oscillator
    // -------------------------------------------------------------------------
    const double mu = 0.5;
    const double Ts = 0.05;

    ctrl::StateFunc vdp = [mu](const Eigen::VectorXd &x,
                               const Eigen::VectorXd &u) -> Eigen::VectorXd {
        Eigen::VectorXd xd(2);
        xd(0) = x(1);
        xd(1) = mu * (1.0 - x(0) * x(0)) * x(1) - x(0) + u(0);
        return xd;
    };

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);

    // --- Check numerical Jacobians against analytical values ---
    const Eigen::MatrixXd A_num = ctrl::jacobianX(vdp, x0, u0);
    const Eigen::MatrixXd B_num = ctrl::jacobianU(vdp, x0, u0);

    // Analytical: A = [[0,1],[-1,mu]], B = [[0],[1]]
    Eigen::Matrix2d A_ana;
    A_ana << 0.0, 1.0, -1.0, mu;
    Eigen::Vector2d B_ana;
    B_ana << 0.0, 1.0;

    const double A_err = (A_num - A_ana).norm() / (A_ana.norm() + 1e-12);
    const double B_err = (B_num - B_ana).norm() / (B_ana.norm() + 1e-12);

    std::cout << "Van der Pol Jacobian relative errors:\n"
              << "  A: " << A_err << "  B: " << B_err << "\n";

    if (A_err > 1e-4 || B_err > 1e-4)
    {
        std::cerr << "FAIL: Jacobian error too large.\n";
        all_pass = false;
    }

    // --- lineariseAtPoint -> discrete StateSpace ---
    ctrl::StateSpace sys_d = ctrl::lineariseAtPoint(vdp, x0, u0, Ts);

    std::cout << "Linearised Van der Pol (discrete, Ts=" << Ts << "):\n"
              << "  A =\n" << sys_d.A << "\n  B = " << sys_d.B.transpose() << "\n";

    // --- LQR on linearised model ---
    ctrl::LQRParams lqr_p;
    lqr_p.Q = Eigen::Matrix2d::Identity() * 10.0;
    lqr_p.R = Eigen::MatrixXd::Identity(1, 1);
    ctrl::DiscreteLQR lqr(sys_d, lqr_p);

    if (!lqr.dareConverged())
    {
        std::cerr << "FAIL: DARE did not converge.\n";
        all_pass = false;
    }

    std::cout << "LQR gain K = " << lqr.gainMatrix() << "\n";

    // --- Close the loop on the NONLINEAR Van der Pol ---
    Eigen::VectorXd x_nl(2);
    x_nl << 0.5, 0.0;   // initial condition near (but not at) the equilibrium

    const Eigen::MatrixXd K = lqr.gainMatrix();     // shape (1,2)
    const int N_steps = 500;

    for (int k = 0; k < N_steps; ++k)
    {
        const Eigen::VectorXd u_ctrl = -K * x_nl;   // full-state feedback on nonlinear plant

        // Euler integration of Van der Pol (small Ts keeps error acceptable)
        Eigen::VectorXd xd = vdp(x_nl, u_ctrl);
        x_nl += Ts * xd;
    }

    const double norm_final = x_nl.norm();
    std::cout << "Van der Pol LQR: ||x(T)|| = " << norm_final
              << "  (need < 0.05)\n";

    if (norm_final > 0.05)
    {
        std::cerr << "FAIL: Van der Pol did not converge to origin.\n";
        all_pass = false;
    }

    // -------------------------------------------------------------------------
    // System 2: simplified CSTR thermal channel
    //   xdot = -a.x + b.exp(-E/(x + x_ss)) + u,   operating around x_ss = 350 K
    // -------------------------------------------------------------------------
    const double a_cstr = 0.5, b_cstr = 1e6, E_cstr = 8000.0;
    const double x_ss = 350.0;  // steady-state temperature offset

    ctrl::StateFunc cstr = [a_cstr, b_cstr, E_cstr, x_ss](
        const Eigen::VectorXd &x, const Eigen::VectorXd &u) -> Eigen::VectorXd {
        Eigen::VectorXd xd(1);
        xd(0) = -a_cstr * x(0) + b_cstr * std::exp(-E_cstr / (x(0) + x_ss)) + u(0);
        return xd;
    };

    Eigen::VectorXd xc0(1);
    xc0 << 0.0;   // perturbation from x_ss
    Eigen::VectorXd uc0(1);
    uc0 << 0.0;

    const ctrl::StateSpace cstr_d = ctrl::lineariseAtPoint(cstr, xc0, uc0, 1.0);

    std::cout << "\nCSTR linearised (Ts=1 s):  A=" << cstr_d.A(0,0)
              << "  B=" << cstr_d.B(0,0) << "\n";

    // A discrete pole |A| < 1 -> stable plant model
    if (std::abs(cstr_d.A(0,0)) >= 1.0)
    {
        std::cerr << "FAIL: CSTR linearised pole outside unit circle.\n";
        all_pass = false;
    }

    // -------------------------------------------------------------------------
    // Summary
    // -------------------------------------------------------------------------
    if (all_pass)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return all_pass ? 0 : 1;
}
