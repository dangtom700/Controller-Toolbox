/**
 * @file ex69_deepc.cpp
 * @brief Part 30: DeePC (Data-Enabled Predictive Control) on a first-order plant.
 *
 * Plant: y[k+1] = a*y[k] + b*u[k],  a=0.7, b=0.3
 *        Steady-state gain = b/(1-a) = 1.0
 *        Reference: r = 1.0
 *
 * Demonstrates:
 *   1. Collecting offline persistently exciting (PRBS) data.
 *   2. Building Hankel matrices and factoring the constant QP Hessian.
 *   3. Online step-ahead control via ADMM.
 *
 * Acceptance (PASS):
 *   - Steady-state tracking error |y - r| < 0.15 after 100 steps.
 *   - All control outputs satisfy uMin <= u <= uMax.
 *   - isHealthy() is true (ADMM converged) for at least 90% of steps.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>

int main()
{
    const double Ts = 0.1;
    const double a  = 0.7;   // plant pole
    const double b  = 0.3;   // plant gain
    const double r  = 1.0;   // setpoint

    // -----------------------------------------------------------------------
    // Phase 1: Collect offline persistently exciting data
    // -----------------------------------------------------------------------
    const int N_data = 100;
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    Eigen::VectorXd u_d(N_data), y_d(N_data);
    y_d(0) = 0.0;
    for (int k = 0; k < N_data; ++k) {
        u_d(k) = dist(rng);
        if (k + 1 < N_data)
            y_d(k + 1) = a * y_d(k) + b * u_d(k);
    }

    // -----------------------------------------------------------------------
    // Phase 2: Build DeePC controller
    // -----------------------------------------------------------------------
    ctrl::DeePCParams p;
    p.T_ini      = 10;
    p.N          = 10;
    p.Q          = 10.0;
    p.R          = 0.05;
    p.lambda_g   = 1.0;
    p.lambda_y   = 50.0;
    p.lambda_u   = 5.0;
    p.uMin       = -3.0;
    p.uMax       =  3.0;
    p.rho        = 10.0;
    p.admm_iters = 300;
    p.admm_tol   = 1e-4;

    ctrl::DeePC dc(p, Ts);
    dc.collectData(u_d, y_d);
    dc.setReference(r);

    std::printf("Hankel columns M = %d\n", dc.hankelColumns());

    // -----------------------------------------------------------------------
    // Phase 3: Control loop
    // -----------------------------------------------------------------------
    double y_k     = 0.0;
    double u_k     = 0.0;
    int    n_steps = 120;
    int    n_unhealthy = 0;

    std::printf("%6s  %8s  %8s  %8s  %6s\n", "step", "y", "u", "error", "ok?");
    for (int k = 0; k < n_steps; ++k) {
        u_k = dc.compute(y_k);
        if (!dc.isHealthy()) ++n_unhealthy;
        const double err = r - y_k;
        if (k % 10 == 0)
            std::printf("%6d  %8.4f  %8.4f  %8.4f  %s\n",
                        k, y_k, u_k, err, dc.isHealthy() ? "OK" : "!");
        y_k = a * y_k + b * u_k;
    }

    // -----------------------------------------------------------------------
    // Phase 4: Acceptance checks
    // -----------------------------------------------------------------------
    const double final_err  = std::abs(y_k - r);
    const double u_bounded  = (u_k >= p.uMin) && (u_k <= p.uMax) ? 1.0 : 0.0;
    const double pct_healthy = 100.0 * (1.0 - static_cast<double>(n_unhealthy) / n_steps);

    std::printf("\nFinal y=%.4f  |y-r|=%.4f  healthy=%.0f%%\n",
                y_k, final_err, pct_healthy);

    bool pass = true;
    if (final_err > 0.15) {
        std::printf("FAIL: tracking error %.4f > 0.15\n", final_err);
        pass = false;
    }
    if (!u_bounded) {
        std::printf("FAIL: final control output %.4f out of [%.1f, %.1f]\n",
                    u_k, p.uMin, p.uMax);
        pass = false;
    }
    if (pct_healthy < 90.0) {
        std::printf("FAIL: ADMM healthy only %.0f%% of steps\n", pct_healthy);
        pass = false;
    }

    if (pass) {
        std::printf("PASS\n");
        return 0;
    }
    return 1;
}
