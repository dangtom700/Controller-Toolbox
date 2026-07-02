// ============================================================
//  ex124_super_twisting_smc.cpp
//  SuperTwistingSMC - 2nd-order (super-twisting) sliding mode control.
//
//  This controller already existed in lib/ (DiscreteSMC.h) and was covered by a
//  Catch2 test, but had no runnable example / Python binding until now.  This
//  demo closes that gap: chattering-free tracking with a continuous control
//  signal and finite-time convergence, on a first-order plant that requires
//  nonzero steady control (the super-twisting integrator removes the offset).
//
//      y[k+1] = 0.8*y[k] + 0.2*u[k]
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
    const double ref = 1.0;

    ctrl::SuperTwistingParams p;
    p.c_e  = 1.0;
    p.c_de = 0.01;   // lambda*Ts with lambda=1
    p.K1   = 2.0;    // power term |s|^{1/2}
    p.K2   = 3.0;    // integral term (K2 > K1^2/4 = 1)
    p.uMin = -20.0;
    p.uMax =  20.0;
    ctrl::SuperTwistingSMC smc(p, Ts);

    double y = 0.0;
    const double e0 = std::abs(ref - y);

    // Discrete super-twisting settles into a small period-2 ripple, so tracking is judged by
    // the MEAN output over a final window (which averages the ripple out) rather than one sample.
    const int N = 1000, WIN = 200;
    double y_sum = 0.0;
    int    y_cnt = 0;

    std::cout << "=== SuperTwistingSMC step tracking ===\n";
    std::cout << std::setw(8) << "t[s]" << std::setw(12) << "y"
              << std::setw(12) << "surface" << std::setw(10) << "u" << "\n";
    for (int k = 0; k <= N; ++k)
    {
        const double u = smc.compute(y - ref);
        y = 0.8 * y + 0.2 * u;
        if (k > N - WIN) { y_sum += y; ++y_cnt; }
        if (k % 100 == 0)
            std::cout << std::fixed << std::setprecision(4)
                      << std::setw(8) << k * Ts
                      << std::setw(12) << y
                      << std::setw(12) << smc.slidingSurface()
                      << std::setw(10) << u << "\n";
    }

    const double y_mean = y_sum / y_cnt;
    const double ef = std::abs(ref - y_mean);
    std::cout << "\nInitial |e|=" << e0 << "  mean-tracking |e|=" << ef
              << " (over last " << WIN << " steps)\n";

    const bool ok = std::isfinite(y) && ef < 0.03;
    std::cout << (ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
