/**
 * ex44_fuzzy_pd_inner_pid_outer.cpp
 * CASCADE: FuzzyPD (inner corrector) + PID (outer primary).
 *
 * Outer PID handles steady-state error (integral action).
 * Inner FuzzyPD provides adaptive gain correction, handling nonlinear gain changes
 * without modifying the outer PID's integrator.
 *
 * Use case: temperature control where heater gain varies with temperature.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.05;

    // Outer PID (slow, provides overall error correction with integral)
    ctrl::PIDParams pp_outer;
    pp_outer.Kp = 0.6; pp_outer.Ki = 0.15; pp_outer.Kd = 0.0;
    pp_outer.uMin = -5.0; pp_outer.uMax = 5.0;
    ctrl::DiscretePID pid_outer(pp_outer, Ts);

    // Inner FuzzyPD (fast, adaptive gain correction)
    ctrl::FuzzyPDParams fpdp;
    fpdp.e_scale  = 1.0;
    fpdp.de_scale = 0.1;
    fpdp.u_scale  = 3.0;
    ctrl::FuzzyPD fuzzy_inner(fpdp, Ts);

    // Plant: G(s) = K_plant / (tau*s + 1) where K_plant varies with output
    // K_plant(y) = 1 + 0.5*y  (gain increases with temperature - nonlinear)
    double y = 0.0;
    const double tau = 1.5, ref = 1.0;
    const double a = std::exp(-Ts / tau);
    double prev_e = 0.0;
    const int N = 600;

    for (int k = 0; k < N; ++k) {
        const double e        = ref - y;
        const double de       = (e - prev_e) / Ts;

        // Outer PID provides nominal command
        const double u_pid    = pid_outer.compute(e);
        // Inner FuzzyPD provides nonlinear correction on the same error
        const double u_fuzzy  = fuzzy_inner.compute(e);
        // Total: additive combination (PID nominal + FuzzyPD correction)
        const double u_total  = u_pid + 0.3 * u_fuzzy;

        // Nonlinear plant (gain depends on output)
        const double K_plant  = 1.0 + 0.5 * y;
        y = a * y + (1.0 - a) * K_plant * u_total;
        prev_e = e;
    }

    std::cout << "CASCADE FuzzyPD(inner)+PID(outer): final y = " << y
              << "  (ref = " << ref << ")\n";

    if (std::abs(y - ref) < 0.05)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
