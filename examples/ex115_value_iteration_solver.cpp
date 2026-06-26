/**
 * @file ex115_value_iteration_solver.cpp
 * @brief Phase 4 (OC2): Grid-based dynamic programming / value iteration - pendulum swing-up.
 *
 * Solves the classical undamped-pendulum swing-up problem (start hanging down at theta=0,
 * reach upright theta=pi) via discretized-state-space value iteration rather than a locally
 * linearizing controller - demonstrating a globally optimal policy on a low-dimensional (n=2)
 * nonlinear system. Pendulum parameters (ml2=1.0, mgl=9.8) match examples/ex98_passivity_based.cpp.
 *
 * @see docs/superpowers/specs/2026-06-26-value-iteration-solver-design.md
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01, ml2 = 1.0, mgl = 9.8;

    auto f = [Ts, ml2, mgl](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(2);
        const double theta_ddot = (u(0) - mgl * std::sin(x(0))) / ml2;
        xn(1) = x(1) + Ts * theta_ddot;
        xn(0) = x(0) + Ts * xn(1);
        return xn;
    };
    auto cost = [](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return (1.0 + std::cos(x(0))) + 0.05 * x(1) * x(1) + 0.001 * u(0) * u(0);
    };

    // theta in [0, 2*pi] (not [-pi, pi]) so the upright target theta=pi sits in the grid's
    // interior, not at its edge -- with no periodic wraparound, a boundary-adjacent target
    // would be actively discouraged by out_of_grid_penalty, preventing the policy from ever
    // balancing precisely there. The start state theta=0 sits at the lower edge instead, which
    // is harmless: the optimal trajectory moves away from it immediately (swinging toward pi),
    // never needing to hover near that boundary.
    ctrl::DPGridParams gp;
    gp.x_min = Eigen::Vector2d(0.0,   -8.0);
    gp.x_max = Eigen::Vector2d(2.0 * M_PI, 8.0);
    gp.n_grid_per_dim = Eigen::Vector2i(41, 41);
    gp.u_min = Eigen::VectorXd::Constant(1, -15.0);
    gp.u_max = Eigen::VectorXd::Constant(1,  15.0);
    gp.n_grid_u = 15;
    gp.discount = 0.97;
    gp.max_iter = 400;
    gp.tol      = 1e-4;
    gp.out_of_grid_penalty = 1e5;

    ctrl::ValueIterationSolver vi(f, cost, gp);
    vi.solve();
    std::printf("solve(): converged=%d iterations=%d finalDelta=%.6g\n",
                vi.converged() ? 1 : 0, vi.iterations(), vi.finalDelta());

    Eigen::VectorXd state = Eigen::VectorXd::Zero(2); // hanging straight down
    for (int k = 0; k < 1500; ++k)
    {
        const Eigen::VectorXd u = vi.policy(state);
        state = f(state, u);
    }

    std::printf("Final state: theta=%.4f theta_dot=%.4f (target theta=pi=%.4f)\n",
                state(0), state(1), M_PI);

    const bool ok = vi.converged() && state.allFinite() &&
                    std::cos(state(0)) < -0.9 && std::fabs(state(1)) < 1.5;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
