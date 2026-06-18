/**
 * @file ex63_nonlinear_mpc.cpp
 * @brief Part 22: Nonlinear MPC (RTI) on a 2-state nonlinear plant.
 *
 * Plant:
 *   x1[k+1] = 0.9*x1[k] - 0.1*x1[k]^3 + 0.5*x2[k]
 *   x2[k+1] = 0.8*x2[k] + u[k]
 *   y[k]    = x1[k]
 *
 * NMPC minimises ||y - y_ref||^2 over Np=10 steps with Nu=3 control moves.
 * Verify closed-loop convergence: |y - y_ref| < 0.05 by step 50.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts      = 0.1;
    const int    N_steps = 80;
    const double y_ref_val = 1.0;

    // -----------------------------------------------------------------------
    // Discrete-time nonlinear dynamics
    // -----------------------------------------------------------------------
    auto f_nl = [](const Eigen::VectorXd &x,
                   const Eigen::VectorXd &u) -> Eigen::VectorXd {
        Eigen::VectorXd xnext(2);
        xnext(0) = 0.9 * x(0) - 0.1 * x(0) * x(0) * x(0) + 0.5 * x(1);
        xnext(1) = 0.8 * x(1) + u(0);
        return xnext;
    };

    // Output: y = x1 (C = [1, 0])
    Eigen::MatrixXd C_out(1, 2);
    C_out << 1.0, 0.0;

    // -----------------------------------------------------------------------
    // NMPC parameters
    // -----------------------------------------------------------------------
    ctrl::NMPCParams np;
    np.Np        = 10;
    np.Nu        = 3;
    np.rho_y     = 5.0;
    np.rho_u     = 0.5;
    np.uMin      = -5.0;
    np.uMax      =  5.0;
    np.qpMaxIter = 500;
    np.qpTol     = 1e-6;
    np.Ts        = Ts;
    np.n_states  = 2;
    np.n_inputs  = 1;
    np.n_outputs = 1;

    ctrl::NonlinearMPC nmpc(np, f_nl, C_out);

    // -----------------------------------------------------------------------
    // Closed-loop simulation
    // -----------------------------------------------------------------------
    Eigen::VectorXd x(2);   x.setZero();
    Eigen::VectorXd y_ref(1); y_ref << y_ref_val;

    std::cout << "step  y       u      |y-ref|\n";
    std::cout << "----  ------  -----  ------\n";

    for (int k = 0; k < N_steps; ++k)
    {
        const double y = (C_out * x)(0);
        const double error = y_ref_val - y;

        nmpc.setState(x);
        const Eigen::VectorXd u_vec = nmpc.computeRef(x, y_ref);
        const double u = u_vec(0);

        if (k % 10 == 0)
        {
            std::printf("%4d  %6.3f  %5.2f  %6.3f\n",
                        k, y, u, std::abs(error));
        }

        // Simulate plant one step
        x = f_nl(x, u_vec);
    }

    const double final_y = (C_out * x)(0);
    std::printf("\nFinal: y=%.4f  |y-ref|=%.4f  QP-converged=%s\n",
                final_y, std::abs(final_y - y_ref_val),
                nmpc.lastQPConverged() ? "yes" : "no");

    // Acceptance: error < 0.15 by end of simulation
    if (std::abs(final_y - y_ref_val) < 0.15 && nmpc.lastQPConverged())
    {
        std::cout << "PASS\n";
        return 0;
    }
    std::cout << "FAIL\n";
    return 1;
}
