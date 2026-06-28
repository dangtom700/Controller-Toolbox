/**
 * @file ex118_lp_solver.cpp
 * @brief Phase 4 (OC4): LPSolver direct usage - a textbook 2-variable LP.
 *
 * maximize x1 + x2  (== minimize -x1 - x2)
 * s.t.     x1 + 2*x2 <= 4
 *          3*x1 + x2 <= 6
 *          x1, x2 >= 0
 *
 * Known vertex optimum: (x1, x2) = (1.6, 1.2), objective = 2.8 (i.e. c'x = -2.8).
 *
 * @see docs/superpowers/specs/2026-06-27-lp-solver-lp-mpc-design.md
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    ctrl::LPProblem problem;
    problem.c.resize(2);
    problem.c << -1.0, -1.0;

    problem.A_ineq.resize(2, 2);
    problem.A_ineq << 1.0, 2.0,
                       3.0, 1.0;
    problem.b_ineq.resize(2);
    problem.b_ineq << 4.0, 6.0;

    problem.lb = Eigen::VectorXd::Zero(2);
    problem.ub = Eigen::VectorXd::Constant(2, 1e9);

    const ctrl::LPResult result = ctrl::LPSolver::solve(problem);

    std::printf("status=%d x=[%.4f, %.4f] cost=%.4f iters=%d\n",
                static_cast<int>(result.status), result.x(0), result.x(1), result.cost, result.iters);

    const bool ok = result.status == ctrl::LPStatus::Optimal &&
                    std::fabs(result.x(0) - 1.6) < 1e-4 &&
                    std::fabs(result.x(1) - 1.2) < 1e-4 &&
                    std::fabs(result.cost - (-2.8)) < 1e-4;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
