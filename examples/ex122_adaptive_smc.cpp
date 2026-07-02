// ============================================================
//  ex122_adaptive_smc.cpp
//  AdaptiveSMC - sliding mode with online switching-gain adaptation.
//
//  The plant has a MATCHED DISTURBANCE of UNKNOWN bound:
//      y[k+1] = y[k] + b*(u[k] + d)
//  A classical SMC would need K > |d|/b known a-priori.  AdaptiveSMC instead
//  grows K online until the trajectory is confined to the sliding band, so the
//  designer never has to know the disturbance bound.
//
//  Demonstrates: gain K grows from K0 and then plateaus, and the error is
//  rejected despite the disturbance.
//
//  Sign convention: compute(y - ref)  (SMC convention).
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main()
{
    const double Ts  = 0.01;
    const double b   = 0.1;
    const double d   = 0.3;   // matched disturbance - bound NOT given to the controller
    const double ref = 1.0;

    ctrl::AdaptiveSMCParams p;
    p.c_e     = 1.0;
    p.c_de    = 0.05;
    p.gamma   = 8.0;    // adaptation rate
    p.epsilon = 0.02;   // dead-band on |s|
    p.K0      = 0.2;    // start with a deliberately too-small gain
    p.Kmin    = 0.0;
    p.Kmax    = 100.0;
    p.phi     = 0.3;
    p.uMin    = -20.0;
    p.uMax    =  20.0;
    ctrl::AdaptiveSMC smc(p, Ts);

    double y = 0.0;
    const double K_start = smc.adaptiveGain();

    std::cout << "=== AdaptiveSMC vs unknown-bound disturbance (d=" << d << ") ===\n";
    std::cout << std::setw(8) << "t[s]" << std::setw(12) << "y"
              << std::setw(12) << "K(adapt)" << std::setw(10) << "u" << "\n";
    for (int k = 0; k <= 1500; ++k)
    {
        const double u = smc.compute(y - ref);
        y = y + b * (u + d);
        if (k % 150 == 0)
            std::cout << std::fixed << std::setprecision(4)
                      << std::setw(8) << k * Ts
                      << std::setw(12) << y
                      << std::setw(12) << smc.adaptiveGain()
                      << std::setw(10) << u << "\n";
    }

    const double ef      = std::abs(ref - y);
    const double K_final = smc.adaptiveGain();
    std::cout << "\nGain adapted " << K_start << " -> " << K_final
              << "   final |e|=" << ef << "\n";

    const bool ok = std::isfinite(y) && ef < 0.05 && K_final > K_start;
    std::cout << (ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
