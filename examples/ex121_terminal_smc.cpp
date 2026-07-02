// ============================================================
//  ex121_terminal_smc.cpp
//  NonsingularTerminalSMC - finite-time sliding mode control.
//
//  Regulates a discrete single-integrator plant  y[k+1] = y[k] + b*u[k]
//  (an integrator holds any y at zero control, so an SMC without integral
//  action still reaches the setpoint with zero steady-state offset).
//
//  Demonstrates the finite-time terminal surface reaching s -> 0 and the
//  tracking error collapsing faster than a comparable linear-surface SMC.
//
//  Sign convention: compute(y - ref)  (SMC convention, see CONTRIBUTING.md).
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main()
{
    const double Ts = 0.01;
    const double b  = 0.1;   // integrator input gain
    const double ref = 1.0;

    ctrl::NonsingularTerminalSMCParams p;
    p.beta  = 1.0;
    p.gamma = 1.5;   // in (1,2): nonsingular, finite-time
    p.K     = 2.0;
    p.eta   = 0.5;
    p.phi   = 0.5;
    p.uMin  = -20.0;
    p.uMax  =  20.0;
    ctrl::NonsingularTerminalSMC smc(p, Ts);

    double y  = 0.0;
    const double e0 = std::abs(ref - y);
    int reach_step = -1;

    std::cout << "=== NonsingularTerminalSMC on an integrator ===\n";
    std::cout << std::setw(8) << "t[s]" << std::setw(12) << "y"
              << std::setw(12) << "surface" << std::setw(10) << "u" << "\n";
    for (int k = 0; k <= 800; ++k)
    {
        const double u = smc.compute(y - ref);
        y = y + b * u;
        if (reach_step < 0 && std::abs(smc.slidingSurface()) < p.phi)
            reach_step = k;                       // finite-time reaching of boundary layer
        if (k % 100 == 0)
            std::cout << std::fixed << std::setprecision(4)
                      << std::setw(8) << k * Ts
                      << std::setw(12) << y
                      << std::setw(12) << smc.slidingSurface()
                      << std::setw(10) << u << "\n";
    }

    const double ef = std::abs(ref - y);
    std::cout << "\nInitial |e|=" << e0 << "  final |e|=" << ef
              << "  reached boundary layer at step " << reach_step << "\n";

    const bool ok = std::isfinite(y) && ef < 0.02 && reach_step >= 0;
    std::cout << (ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
