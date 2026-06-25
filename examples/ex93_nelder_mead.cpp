/**
 * @file ex93_nelder_mead.cpp
 * @brief Phase 3 (MO2): NelderMead finds PID gains minimising IAE, no bounds needed.
 *
 * Same plant/cost shape as ex65_autotuner_pid.cpp, but tuned with NelderMead instead of
 * AutoTuner (CMA-ES) - demonstrating the "quick win" case: NelderMead needs only an initial
 * point (no lower/upper bounds, no population size to choose) for this small 3-parameter
 * retune, and converges in fewer cost evaluations than CMA-ES's population overhead requires.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts      = 0.05;
    const int    N_sim   = 100;
    const double ref_val = 1.0;

    Eigen::MatrixXd A_c(2, 2), B_c(2, 1), C_c(1, 2), D_c(1, 1);
    A_c << 0, 1, -1, -2;
    B_c << 0, 1;
    C_c << 1, 0;
    D_c << 0;
    ctrl::StateSpace sys_c(A_c, B_c, C_c, D_c, 0.0);
    ctrl::StateSpace plant = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

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
            const double y = (plant.C * x)(0);
            const double e = ref_val - y;
            const double u = pid.compute(e);
            Eigen::VectorXd uv(1); uv << u;
            x = plant.A * x + plant.B * uv;
            iae += std::abs(e) * Ts;
        }
        return iae;
    };

    Eigen::Vector3d x_default(1.0, 0.1, 0.0);
    const double iae_default = simulate_iae(x_default);
    std::printf("Default gains  Kp=%.2f Ki=%.2f Kd=%.2f  IAE=%.4f\n",
                x_default(0), x_default(1), x_default(2), iae_default);

    ctrl::NelderMeadParams nmp;
    nmp.n_dim = 3;
    ctrl::NelderMead nm(nmp);

    const ctrl::TunerResult result = nm.optimize(simulate_iae, x_default);

    const double iae_tuned = result.cost;
    std::printf("Tuned  gains   Kp=%.3f Ki=%.3f Kd=%.3f  IAE=%.4f\n",
                result.params(0), result.params(1), result.params(2), iae_tuned);
    std::printf("Evaluations=%d  Iterations=%d  Converged=%s\n",
                result.nEvals, result.nGens, result.converged ? "yes" : "no");
    std::printf("IAE improvement: %.1f%%\n",
                (iae_default - iae_tuned) / iae_default * 100.0);

    // Compare against AutoTuner's evaluation budget for the same problem (the "why use this"
    // case: a single-point start needs far fewer evaluations than a population-based search).
    ctrl::AutoTunerParams atp;
    atp.n = 3;
    atp.lower = Eigen::Vector3d(0.01, 0.0, 0.0);
    atp.upper = Eigen::Vector3d(5.0, 2.0, 1.0);
    ctrl::AutoTuner tuner(atp);
    const ctrl::TunerResult atResult = tuner.tune(simulate_iae, x_default);
    std::printf("AutoTuner (CMA-ES) for comparison: Evaluations=%d  IAE=%.4f\n",
                atResult.nEvals, atResult.cost);

    const bool improved = (iae_tuned < iae_default * 0.9);
    const bool finite   = std::isfinite(iae_tuned) && result.params.allFinite();
    const bool fewerEvals = result.nEvals < atResult.nEvals;

    const bool ok = improved && finite && fewerEvals;
    if (!improved)
        std::printf("FAIL: tuned IAE %.4f not < 0.9 * default %.4f\n", iae_tuned, iae_default * 0.9);
    if (!fewerEvals)
        std::printf("FAIL: NelderMead used %d evals, not fewer than AutoTuner's %d\n",
                    result.nEvals, atResult.nEvals);
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
