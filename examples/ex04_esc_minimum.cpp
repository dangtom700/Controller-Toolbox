// ============================================================
//  ex04_esc_minimum.cpp
//  Extremum Seeking Control on a static quadratic cost surface.
//  Goal: find the unknown minimum of J(u) = (u - 3.5)^2 + 5
//  without knowing where 3.5 is.
//
//  MATLAB equivalent:
//    % Simulink "Extremum Seeking" block demo (R2020b+)
//    % or: krstic_esc_sim.m
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>

int main()
{
    const double Ts = 0.01;

    ctrl::ExtremumSeekerParams ep;
    ep.perturbAmp  = 0.2;   // dither amplitude
    ep.perturbFreq = 5.0;   // 5 Hz dither - well above gradient bandwidth
    ep.lpfCutoff   = 0.2;   // gradient LPF
    ep.hpfCutoff   = 0.1;   // DC removal
    ep.integGain   = 2.0;   // how fast the operating point moves
    ep.seekMinimum = true;

    ctrl::ExtremumSeeker esc(ep, Ts);

    // True cost surface (unknown to the ESC)
    const double true_min = 3.5;
    auto cost = [&](double u) { return (u - true_min) * (u - true_min) + 5.0; };

    std::cout << std::setw(7) << "k" << std::setw(8)  << "t[s]"
              << std::setw(10) << "u_total" << std::setw(10) << "theta"
              << std::setw(10) << "J(u)" << "\n";
    std::cout << std::string(45, '-') << "\n";

    for (int k = 0; k <= 1000; ++k) {
        // ESC receives cost J and returns next plant input u = theta + dither
        const double u_prev = esc.currentEstimate();  // theta before this step
        const double J      = cost(u_prev);            // evaluate cost at current op point
        const double u_next = esc.compute(J);          // update and return new input

        if (k % 100 == 0)
            std::cout << std::setw(7)  << k
                      << std::fixed << std::setprecision(4)
                      << std::setw(8)  << k * Ts
                      << std::setw(10) << u_next
                      << std::setw(10) << esc.currentEstimate()
                      << std::setw(10) << J << "\n";
    }

    std::cout << "\nFinal operating point estimate: "
              << std::fixed << std::setprecision(4)
              << esc.currentEstimate()
              << "  (true minimum: " << true_min << ")\n";
    return 0;
}
