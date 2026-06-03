/**
 * @file ex69_deepc.cpp
 * @brief Data-Enabled Predictive Control (DeePC) on a first-order SISO plant.
 *
 * Demonstrates:
 *   1. Generating persistently-exciting (PRBS) offline I/O data.
 *   2. Constructing a DeePC controller directly from that data.
 *   3. Closed-loop tracking of a unit step reference.
 *   4. Comparing DeePC to a simple PI controller.
 *
 * Plant:  y[k+1] = 0.8 y[k] + 0.2 u[k]   (DC gain = 1, tau ~ 4.5 steps)
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <cstdio>
#include <vector>

// ---------------------------------------------------------------------------
// First-order plant step
// ---------------------------------------------------------------------------
static double plantStep(double& x, double u, double a = 0.8, double b = 0.2)
{
    double y = x;
    x = a * x + b * u;
    return y;
}

// ---------------------------------------------------------------------------
// Generate PRBS offline data
// ---------------------------------------------------------------------------
static void collectOfflineData(Eigen::VectorXd& u_off,
                                Eigen::VectorXd& y_off,
                                int N)
{
    u_off.resize(N);
    y_off.resize(N);
    double x = 0.0;
    for (int k = 0; k < N; ++k) {
        // Alternating +-1 square wave (persistently exciting of any order)
        u_off(k) = (k % 2 == 0) ? 1.0 : -1.0;
        y_off(k) = plantStep(x, u_off(k));
    }
}

int main()
{
    std::printf("=== ex69: DeePC on first-order SISO plant ===\n\n");

    // ------------------------------------------------------------------
    // 1. Collect offline data  (N=400, alternating PRBS)
    // ------------------------------------------------------------------
    constexpr int    N_off = 400;
    constexpr double Ts    = 0.1;

    Eigen::VectorXd u_off, y_off;
    collectOfflineData(u_off, y_off, N_off);

    // ------------------------------------------------------------------
    // 2. Build DeePC controller
    // ------------------------------------------------------------------
    ctrl::DeePC::Params dp;
    dp.T_ini     = 5;
    dp.Np        = 20;
    dp.rho_y     = 1.0;
    dp.rho_u     = 0.05;
    dp.lambda_g  = 0.5;
    dp.lambda_eq = 1e5;
    dp.uMin      = -3.0;
    dp.uMax      =  3.0;
    dp.rho_admm  = 1.0;
    dp.admm_iter = 150;
    ctrl::DeePC deepc(u_off, y_off, dp, Ts);

    // ------------------------------------------------------------------
    // 3. Reference PI for comparison
    // ------------------------------------------------------------------
    ctrl::PIDParams pp;
    pp.Kp = 2.0;  pp.Ki = 0.5;  pp.Kd = 0.0;
    pp.N  = 10.0; pp.Kb = 1.0;
    pp.uMin = -3.0; pp.uMax = 3.0;
    ctrl::DiscretePID pid(pp, Ts);

    // ------------------------------------------------------------------
    // 4. Closed-loop simulation
    // ------------------------------------------------------------------
    constexpr int    N_sim = 200;
    constexpr double r     = 1.0;

    double x_deepc = 0.0, x_pid = 0.0;
    double iae_deepc = 0.0, iae_pid = 0.0;

    std::printf("  step  |   DeePC y   DeePC u   |    PI y     PI u\n");
    std::printf("--------|------------------------|------------------\n");

    for (int k = 0; k < N_sim; ++k) {
        // DeePC
        double y_deepc = x_deepc;
        double u_deepc = deepc.computeIO(y_deepc, r);
        x_deepc = 0.8 * x_deepc + 0.2 * u_deepc;

        // PI
        double y_pid = x_pid;
        double u_pid = pid.compute(r - y_pid);
        x_pid = 0.8 * x_pid + 0.2 * u_pid;

        // Accumulate IAE (exclude warm-up)
        if (k > dp.T_ini) {
            iae_deepc += std::abs(r - y_deepc) * Ts;
            iae_pid   += std::abs(r - y_pid)   * Ts;
        }

        if (k < 40 || k == N_sim - 1)
            std::printf("  %4d  | %9.4f  %7.4f | %9.4f  %7.4f\n",
                        k, y_deepc, u_deepc, y_pid, u_pid);
    }

    std::printf("\n  IAE (excl. warm-up): DeePC = %.4f,  PI = %.4f\n",
                iae_deepc, iae_pid);
    std::printf("  Final y: DeePC = %.4f,  PI = %.4f\n", x_deepc, x_pid);
    std::printf("  DeePC primal residual (last step): %.2e\n",
                deepc.lastPrimalResidual());

    return 0;
}
