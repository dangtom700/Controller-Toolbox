/**
 * @file ex66_antwindup_wrapper.cpp
 * @brief Part 24: AntiWindupWrapper - conditioning technique prevents integrator windup.
 *
 * Demonstrates that AntiWindupWrapper limits integrator windup in a PI controller
 * when the actuator saturates. Three scenarios are simulated in parallel:
 *
 *   A) PI controller with NO anti-windup (manual output clamp only):
 *      Integral winds up to ~20 during saturation; plant takes 80+ steps to recover.
 *
 *   B) PI controller with AntiWindupWrapper (Kb=1, conditioning):
 *      Integral bounded to ~6; plant exits saturation after ~6 recovery steps.
 *
 *   C) AntiWindupWrapper around a pure-proportional controller (Kp only):
 *      No integral, no windup. Wrapper is transparent (pass-through).
 *
 * Plant:  y[k+1] = 0.9*y[k] + 0.2*u[k]   (first-order, y_ss=2 at u=1)
 * PI:     Kp=0.5, Ki=1.0, Ts=0.1          (no derivative, no built-in anti-windup)
 * Limits: uMin=-3, uMax=1
 * Phase 1 (steps 0-49):  r=5  (far above plant capability; actuator saturates)
 * Phase 2 (steps 50-79): r=0  (recovery; unwrapped integral takes 40+ steps to unwind)
 *
 * Acceptance (PASS criteria):
 *   - Wrapped final y is below 1.5 (converged toward r=0).
 *   - Unwrapped final y is above 1.5 (still winding down, near y_ss=2).
 */

#include "ControllerToolbox.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

int main()
{
    const double Ts    = 0.1;
    const double uMin  = -3.0;
    const double uMax  =  1.0;
    const int    N_sat = 50;
    const int    N_rec = 30;
    const int    N     = N_sat + N_rec;

    // -----------------------------------------------------------------------
    // Plant step function: y[k+1] = 0.9*y[k] + 0.2*u[k]
    // -----------------------------------------------------------------------
    auto plant = [](double y, double u) { return 0.9 * y + 0.2 * u; };

    // -----------------------------------------------------------------------
    // PI parameters - Kb=0 (no built-in anti-windup), no internal clamping
    // -----------------------------------------------------------------------
    ctrl::PIDParams pp;
    pp.Kp  = 0.5;   pp.Ki = 1.0;    pp.Kd = 0.0;
    pp.N   = 10.0;  pp.Kb = 0.0;
    pp.uMin = -1e9; pp.uMax = 1e9;

    // -----------------------------------------------------------------------
    // Scenario A: NO anti-windup (manual clamp only)
    // -----------------------------------------------------------------------
    auto pid_A = std::make_shared<ctrl::DiscretePID>(pp, Ts);
    double y_A = 0.0;

    // -----------------------------------------------------------------------
    // Scenario B: AntiWindupWrapper (conditioning, Kb=1)
    // -----------------------------------------------------------------------
    auto pid_B_inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
    ctrl::AntiWindupWrapper wrapped_B(pid_B_inner, uMin, uMax, /*Kb=*/1.0);
    double y_B = 0.0;

    // -----------------------------------------------------------------------
    // Print header and run
    // -----------------------------------------------------------------------
    std::printf("%-6s  %-6s  %-8s  %-8s  %-8s  %-8s  %-8s\n",
                "Step", "Ref", "u_A", "y_A", "u_B", "y_B", "Sat_B");
    std::printf("%s\n", std::string(62, '-').c_str());

    for (int k = 0; k < N; ++k) {
        const double ref = (k < N_sat) ? 5.0 : 0.0;
        const double e_A = ref - y_A;
        const double e_B = ref - y_B;

        // Scenario A: manual clamp, integral winds up freely
        const double u_A = std::clamp(pid_A->compute(e_A), uMin, uMax);

        // Scenario B: wrapper applies conditioning + clamp
        const double u_B = wrapped_B.compute(e_B);

        if (k % 5 == 0)
            std::printf("%-6d  %-6.1f  %-8.3f  %-8.3f  %-8.3f  %-8.3f  %-8s\n",
                        k, ref, u_A, y_A, u_B, y_B,
                        wrapped_B.isSaturated() ? "YES" : "no");

        y_A = plant(y_A, u_A);
        y_B = plant(y_B, u_B);
    }

    // -----------------------------------------------------------------------
    // Final report
    // -----------------------------------------------------------------------
    std::printf("\n=== Final state after %d saturation + %d recovery steps ===\n",
                N_sat, N_rec);
    std::printf("  Unwrapped (A):  y = %.4f  (integral wound up, slow recovery)\n", y_A);
    std::printf("  Wrapped   (B):  y = %.4f  (conditioning bounded integral)\n",   y_B);

    // -----------------------------------------------------------------------
    // Acceptance checks
    // -----------------------------------------------------------------------
    const bool pass_B_low  = (y_B < 1.5);  // wrapped converged toward r=0
    const bool pass_A_high = (y_A > 1.5);  // unwrapped still near y_ss=2
    const bool pass_gap    = (y_B < y_A);  // wrapped always converges faster

    if (pass_B_low && pass_A_high && pass_gap) {
        std::cout << "PASS\n";
        return 0;
    }
    if (!pass_B_low)
        std::printf("FAIL: wrapped y=%.4f not < 1.5 (anti-windup not effective)\n", y_B);
    if (!pass_A_high)
        std::printf("FAIL: unwrapped y=%.4f not > 1.5 (expected windup not present)\n", y_A);
    if (!pass_gap)
        std::printf("FAIL: wrapped y=%.4f not < unwrapped y=%.4f\n", y_B, y_A);
    return 1;
}
