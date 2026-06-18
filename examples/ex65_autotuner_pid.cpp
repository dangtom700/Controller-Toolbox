/**
 * @file ex65_autotuner_pid.cpp
 * @brief Part 22: AutoTuner (CMA-ES) finds PID gains minimising IAE.
 *
 * Plant: second-order G(s) = 1 / (s+1)^2, discretised at Ts=0.05s via ZOH.
 * Cost function: closed-loop IAE over 100 steps, step reference r=1.
 *
 * AutoTuner searches over [Kp, Ki, Kd] in [0.01, 5]^3.
 * Acceptance:
 *   - IAE with tuned gains < IAE with default (Kp=1, Ki=0.1, Kd=0) by >=10%.
 *   - Tuned gains remain within the specified bounds.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts      = 0.05;
    const int    N_sim   = 100;
    const double ref_val = 1.0;

    // -----------------------------------------------------------------------
    // Plant: (s+1)^2 -> ZOH discrete StateSpace
    // -----------------------------------------------------------------------
    Eigen::MatrixXd A_c(2,2), B_c(2,1), C_c(1,2), D_c(1,1);
    A_c << 0, 1, -1, -2;
    B_c << 0, 1;
    C_c << 1, 0;
    D_c << 0;
    ctrl::StateSpace sys_c(A_c, B_c, C_c, D_c, 0.0);
    ctrl::StateSpace plant = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    // -----------------------------------------------------------------------
    // Cost function: closed-loop IAE simulation
    // -----------------------------------------------------------------------
    auto simulate_iae = [&](const Eigen::VectorXd &params) -> double {
        ctrl::PIDParams pp;
        pp.Kp = params(0); pp.Ki = params(1); pp.Kd = params(2);
        pp.N  = 10.0;
        pp.uMin = -10.0; pp.uMax = 10.0;
        ctrl::DiscretePID pid(pp, Ts);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
        double iae = 0.0;
        for (int k = 0; k < N_sim; ++k)
        {
            const double y   = (plant.C * x)(0);
            const double e   = ref_val - y;
            const double u   = pid.compute(e);
            Eigen::VectorXd uv(1); uv << u;
            x = plant.A * x + plant.B * uv;
            iae += std::abs(e) * Ts;
        }
        return iae;
    };

    // -----------------------------------------------------------------------
    // Default gains baseline
    // -----------------------------------------------------------------------
    Eigen::Vector3d x_default(1.0, 0.1, 0.0);
    const double iae_default = simulate_iae(x_default);
    std::printf("Default gains  Kp=%.2f Ki=%.2f Kd=%.2f  IAE=%.4f\n",
                x_default(0), x_default(1), x_default(2), iae_default);

    // -----------------------------------------------------------------------
    // AutoTuner
    // -----------------------------------------------------------------------
    ctrl::AutoTunerParams atp;
    atp.n      = 3;
    atp.sigma0 = 0.5;
    atp.maxIter = 80;
    atp.tol    = 1e-8;
    atp.lower  = Eigen::Vector3d(0.01, 0.01, 0.0);
    atp.upper  = Eigen::Vector3d(5.0,  2.0,  1.0);

    ctrl::AutoTuner tuner(atp, /*seed=*/42);

    const ctrl::TunerResult result = tuner.tune(simulate_iae, x_default);

    const double iae_tuned = result.cost;
    std::printf("Tuned  gains   Kp=%.3f Ki=%.3f Kd=%.3f  IAE=%.4f\n",
                result.params(0), result.params(1), result.params(2), iae_tuned);
    std::printf("Evaluations=%d  Generations=%d  Converged=%s\n",
                result.nEvals, result.nGens, result.converged ? "yes" : "no");
    std::printf("IAE improvement: %.1f%%\n",
                (iae_default - iae_tuned) / iae_default * 100.0);

    // -----------------------------------------------------------------------
    // Acceptance checks
    // -----------------------------------------------------------------------
    const bool improved = (iae_tuned < iae_default * 0.9);
    const bool in_bounds = (result.params.minCoeff() >= 0.0) &&
                           (result.params(0) <= 5.0) &&
                           (result.params(1) <= 2.0) &&
                           (result.params(2) <= 1.0);

    if (improved && in_bounds)
    {
        std::cout << "PASS\n";
        return 0;
    }
    if (!improved)
        std::printf("FAIL: tuned IAE %.4f not < 0.9 * default %.4f\n",
                    iae_tuned, iae_default * 0.9);
    if (!in_bounds)
        std::cout << "FAIL: parameters out of bounds\n";
    return 1;
}
