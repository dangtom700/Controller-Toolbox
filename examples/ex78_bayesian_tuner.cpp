/**
 * @file ex78_bayesian_tuner.cpp
 * @brief BayesianOptimizer: tune PID gains on a FOPDT plant using BO vs CMA-ES.
 *
 * Plant: FOPDT ZOH-discrete  G(s) = 1.2/(2.0s+1)  at Ts=0.05s
 *   y[k+1] = alpha*y[k] + beta*u[k]
 *
 * Objective: IAE over 80 simulation steps for a unit step response.
 * Parameters: [Kp, Ki]  in [0.01, 5] x [0.0, 2.0].
 *
 * Expected output:
 *   BayesianOptimizer uses 20 evals total (5 init + 15 BO).
 *   CMA-ES uses ~100 evals for the same n_gens.
 *   Both converge to a good [Kp, Ki] pair with IAE < 0.5.
 *   [PASS] BO found Kp, Ki within acceptable range.
 */

#include <ControllerToolbox.h>
#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
    constexpr double Ts    = 0.05;
    constexpr double K_dc  = 1.2;
    constexpr double tau   = 2.0;
    constexpr double alpha = std::exp(-Ts / tau);      // ~ 0.975
    const     double beta  = K_dc * (1.0 - alpha);    // ZOH

    // --- Cost function: IAE over 80 steps for unit step, from y=0 ----------
    auto simulate_iae = [&](const Eigen::VectorXd& p) -> double {
        const double Kp = p(0), Ki = p(1);
        ctrl::PIDParams pp; pp.Kp = Kp; pp.Ki = Ki; pp.Kd = 0.0;
        ctrl::DiscretePID pid(pp, Ts);
        double y = 0.0, iae = 0.0;
        for (int k = 0; k < 80; ++k) {
            double e = 1.0 - y;
            double u = pid.compute(e);
            u = std::clamp(u, -3.0, 3.0);
            y = alpha * y + beta * u;
            iae += std::abs(e) * Ts;
        }
        return iae;
    };

    Eigen::VectorXd x0(2); x0 << 1.0, 0.2;

    // --- BayesianOptimizer --------------------------------------------------
    ctrl::BayesOptParams bp;
    bp.n = 2; bp.n_init = 5; bp.maxIter = 15;
    bp.n_acq_restarts = 100; bp.seed = 42;
    bp.lower = Eigen::Vector2d(0.01, 0.0);
    bp.upper = Eigen::Vector2d(5.0,  2.0);
    ctrl::BayesianOptimizer bo(bp);

    auto r_bo = bo.tune(simulate_iae, x0);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "=== BayesianOptimizer ===\n";
    std::cout << "  Kp=" << r_bo.params(0) << "  Ki=" << r_bo.params(1)
              << "  IAE=" << r_bo.cost
              << "  evals=" << r_bo.nEvals << '\n';

    // --- CMA-ES (AutoTuner) for comparison ----------------------------------
    ctrl::AutoTunerParams ap;
    ap.n = 2; ap.sigma0 = 0.3; ap.maxIter = 15;
    ap.lower = bp.lower; ap.upper = bp.upper;
    ctrl::AutoTuner at(ap, 42);

    auto r_at = at.tune(simulate_iae, x0);

    std::cout << "=== AutoTuner (CMA-ES) ===\n";
    std::cout << "  Kp=" << r_at.params(0) << "  Ki=" << r_at.params(1)
              << "  IAE=" << r_at.cost
              << "  evals=" << r_at.nEvals << '\n';

    std::cout << "\nBO evals: " << r_bo.nEvals
              << "   CMA-ES evals: " << r_at.nEvals << '\n';

    // --- Verify BO converged ------------------------------------------------
    bool ok = (r_bo.cost < 0.5) &&
              (r_bo.params(0) > 0.01) && (r_bo.params(0) < 5.0) &&
              (r_bo.params(1) >= 0.0) && (r_bo.params(1) < 2.0);

    if (!ok)
        std::cerr << "[FAIL] BO did not find acceptable PID gains.\n";
    else
        std::cout << "[PASS] BO found Kp, Ki within acceptable range.\n";

    return ok ? 0 : 1;
}
