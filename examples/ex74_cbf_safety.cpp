/**
 * @file ex74_cbf_safety.cpp
 * @brief CBF Safety Filter - keeps a PID-controlled integrator inside a safe set.
 *
 * Plant: xdot = u  (single integrator),  discretised as x[k+1] = x[k] + Ts * u[k].
 * Safe set: h(x) = x_max - x >= 0  (state must stay below x_max = 1.5).
 * Nominal controller: PID tracking r = 2.0  (which would violate the safe set).
 * CBF filter: modifies u so x never exceeds x_max.
 */

#include "ControllerToolbox.h"
#include "CBFSafetyFilter.h"
#include <cmath>
#include <cstdio>

int main()
{
    std::printf("=== ex74: CBF Safety Filter ===\n\n");
    constexpr double Ts    = 0.01;
    constexpr double x_max = 1.5;
    constexpr double r     = 2.0;   // reference ABOVE the safety limit
    constexpr int    N     = 500;

    // Nominal PID (aggressive: would drive x toward r=2 without safety filter)
    ctrl::PIDParams pp;
    pp.Kp = 10.0; pp.Ki = 2.0; pp.Kd = 0.5;
    pp.N  = 20.0; pp.Kb = 1.0;
    pp.uMin = -5.0; pp.uMax = 5.0;
    auto nominal = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    // CBF functions for h(x) = x_max - x;  plant xdot = u  (no drift)
    auto h_fn    = [x_max](double x) { return x_max - x; };
    auto dh_fn   = [](double)        { return -1.0; };  // dh/dx = -1
    auto f0_fn   = [](double)        { return 0.0; };   // no drift
    auto g_fn    = [Ts](double)      { return Ts; };    // discrete: dx = Ts*u

    ctrl::CBFSafetyFilter::Params cbf_p;
    cbf_p.alpha = 5.0;
    cbf_p.uMin  = -5.0;
    cbf_p.uMax  =  5.0;

    ctrl::CBFSafetyFilter cbf(nominal, h_fn, dh_fn, f0_fn, g_fn, cbf_p, Ts);

    std::printf("  step |   x     u_nom   u_safe  cbf_on  h(x)\n");
    std::printf("-------|---------------------------------------\n");

    double x = 0.0;
    int cbf_activations = 0;

    for (int k = 0; k < N; ++k) {
        cbf.setState(x);
        double u_safe = cbf.compute(r - x);
        double u_nom  = cbf.nominalOutput();
        bool   active = cbf.cbfActive();

        if (active) ++cbf_activations;
        x += Ts * u_safe;   // single integrator

        if (k < 30 || k == N-1)
            std::printf("  %4d | %6.4f  %6.4f  %6.4f    %c    %.4f\n",
                        k, x, u_nom, u_safe, active ? 'Y' : ' ',
                        cbf.barrierValue());
    }

    std::printf("\n  Final x = %.4f (safety limit: %.4f)\n", x, x_max);
    std::printf("  CBF activations: %d / %d steps\n", cbf_activations, N);
    std::printf("  Safety constraint violated: %s\n", x > x_max + 1e-6 ? "YES (BUG)" : "NO");

    return 0;
}
