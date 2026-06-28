/**
 * @file ex119_lp_mpc.cpp
 * @brief Phase 4 (OC4): LPMPC closed-loop step tracking on a SISO 2nd-order plant.
 *
 * Same plant as ex01_tf_pid.cpp (G(s) = 1/(s^2+1.5s+1), ZOH at Ts=0.01s) so the result is
 * directly comparable to that file's PID closed loop. LPMPC casts L1 tracking + L1 move-
 * suppression as an LP each step (LPSolver two-phase simplex) instead of DiscreteMPC's L2/QP.
 *
 * @see docs/superpowers/specs/2026-06-27-lp-solver-lp-mpc-design.md
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;
    ctrl::TransferFunction plant_tf(
        {0.0, 4.9625e-5, 4.9125e-5},
        {1.0, -1.98511, 0.98522},
        Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(plant_tf);

    ctrl::LPMPCParams params;
    params.Np = 15;
    params.Nc = 5;
    params.rho_y = 1.0;
    // L1-cost MPC has a "deadzone": a move is taken only if its aggregate marginal benefit
    // (rho_y * sum of this plant's per-step sensitivities across the horizon) clears the rho_u
    // penalty -- unlike QP/L2-cost MPC, which always takes an infinitesimal step for any nonzero
    // gradient. This plant's per-step sensitivity is tiny (Ts=0.01s, slow 2nd-order lag), so
    // rho_u must be kept well below that threshold or the optimizer correctly (not a bug) stays
    // at DeltaU=0 forever. See LPMPC.h's class docs and
    // docs/superpowers/specs/2026-06-27-lp-solver-lp-mpc-design.md.
    params.rho_u = 0.001;
    params.uMin = -5.0;
    params.uMax = 5.0;
    params.duMin = -1.0;
    params.duMax = 1.0;

    ctrl::LPMPC mpc(plant, params);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;
    const double ref = 1.0;

    double max_abs_u = 0.0;
    bool   all_converged = true;

    for (int k = 0; k < 1500; ++k)
    {
        const double e = ref - y;
        const double u = mpc.compute(e);
        all_converged   = all_converged && mpc.lastLPConverged();
        max_abs_u       = std::max(max_abs_u, std::fabs(u));

        Eigen::VectorXd uv(1);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);

        if (k % 150 == 0)
            std::printf("k=%4d  y=%.4f  e=%.4f  u=%.4f\n", k, y, e, u);
    }

    std::printf("Final y=%.4f, max|u|=%.4f, all LP solves converged=%d\n",
                y, max_abs_u, all_converged ? 1 : 0);

    const bool ok = all_converged && std::fabs(y - ref) < 0.02 && max_abs_u <= 5.0 + 1e-9;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
