/**
 * @file ex73_l1_adaptive.cpp
 * @brief L1 Adaptive Controller - bounded-transient step tracking.
 *
 * Demonstrates:
 *   - Step reference tracking on a first-order plant with unknown input gain.
 *   - L1 adapts sigma^ to reject the gain uncertainty.
 *   - Comparison to MRAC with the same reference model.
 */

#include "ControllerToolbox.h"
#include "L1AdaptiveController.h"
#include <cmath>
#include <cstdio>

int main()
{
    std::printf("=== ex73: L1 Adaptive Controller ===\n\n");
    constexpr double Ts   = 0.01;
    constexpr double r    = 1.0;
    constexpr int    N    = 800;

    // Plant: y[k+1] = 0.8*y[k] + true_gain * u[k]
    constexpr double a_plant = 0.8, true_gain = 0.4;  // gain mismatch vs b_m=0.2

    // L1 controller
    ctrl::L1AdaptiveController::Params p;
    p.a_m      = 0.8;    // reference model pole (matches plant a)
    p.b_m      = 0.2;    // reference model input gain (nominal)
    p.k_g      = 1.0 / (1.0 - p.a_m);  // DC feedforward: unity step response
    p.Gamma    = 500.0;
    p.omega_c  = 5.0;   // LP filter BW [rad/s]
    p.sigma_max= 10.0;
    p.uMin     = -3.0;
    p.uMax     =  3.0;
    ctrl::L1AdaptiveController l1(p, Ts);
    l1.setReference(r);

    // MRAC for comparison
    ctrl::MRACParams mp;
    mp.a_m = 0.8; mp.b_m = 0.2; mp.gamma_r = 20.0; mp.gamma_y = 20.0;
    mp.sigma = 0.01; mp.theta_max = 50.0;
    mp.uMin = -3.0; mp.uMax = 3.0;
    ctrl::MRACController mrac(mp, Ts);
    mrac.setReference(r);

    std::printf("  step |  L1 y    L1 sigma^     |  MRAC y\n");
    std::printf("-------|---------------------|--------\n");

    double y_l1 = 0.0, y_mrac = 0.0;
    double iae_l1 = 0.0, iae_mrac = 0.0;

    for (int k = 0; k < N; ++k) {
        double u_l1   = l1.compute(r - y_l1);
        double u_mrac = mrac.compute(y_mrac);

        y_l1   = a_plant * y_l1   + true_gain * u_l1;
        y_mrac = a_plant * y_mrac + true_gain * u_mrac;

        iae_l1   += std::abs(r - y_l1)   * Ts;
        iae_mrac += std::abs(r - y_mrac) * Ts;

        if (k < 25 || k == N-1)
            std::printf("  %4d | %6.4f  %8.4f  | %6.4f\n",
                        k, y_l1, l1.estimatedDisturbance(), y_mrac);
    }
    std::printf("\n  IAE: L1 = %.4f,  MRAC = %.4f\n", iae_l1, iae_mrac);
    std::printf("  L1 final sigma^ = %.4f  (adapts to cancel gain mismatch)\n",
                l1.estimatedDisturbance());
    return 0;
}
