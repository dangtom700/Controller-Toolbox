// ============================================================
//  ex15_esc_moving_minimum.cpp
//  Extremum Seeking Control tracking a continuously shifting optimum.
//  Cost function J(u, t) = (u - min(t))^2 + 2.0
//  min(t) = 3.0 + sin(t)
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main()
{
    const double Ts = 0.01;

    ctrl::ExtremumSeekerParams ep;
    ep.perturbAmp  = 0.3;   // dither
    ep.perturbFreq = 10.0;  // fast dither
    ep.lpfCutoff   = 0.5;
    ep.hpfCutoff   = 0.1;
    ep.integGain   = 5.0;   // fast tracking
    ep.seekMinimum = true;

    ctrl::ExtremumSeeker esc(ep, Ts);

    std::cout << "=== ESC Moving Minimum Tracking ===\n";
    std::cout << "  k    t[s]   True_Min   ESC_Estimate   J(cost)\n";
    std::cout << std::string(55, '-') << "\n";

    for (int k = 0; k <= 1000; ++k) {
        double t = k * Ts;
        double true_min = 3.0 + std::sin(t); // Moving target
        
        double u_prev = esc.currentEstimate();
        double J = (u_prev - true_min) * (u_prev - true_min) + 2.0;
        
        esc.compute(J); // update

        if (k % 100 == 0)
            std::cout << std::setw(4)  << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8)  << t
                      << std::setw(10) << true_min
                      << std::setw(14) << esc.currentEstimate()
                      << std::setw(12) << J << "\n";
    }
    return 0;
}
