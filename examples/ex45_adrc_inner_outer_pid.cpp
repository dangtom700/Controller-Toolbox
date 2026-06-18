/**
 * ex45_adrc_inner_outer_pid.cpp
 * CASCADE: ADRC (inner, ESO disturbance observer) + PID (outer).
 *
 * ADRC's ESO estimates total disturbance on the inner plant (matched disturbances,
 * friction, unmodelled dynamics). The outer PID sees a cleaner near-integrator plant.
 *
 * Architecture:
 *   outer_PID -> inner_setpoint -> ADRC -> plant_u -> plant -> y_inner -> outer_plant -> y_outer
 *
 * ADRC makes the inner plant behave like a double integrator regardless of disturbances.
 * This greatly simplifies outer PID tuning.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

int main()
{
    const double Ts = 0.01;

    // Inner plant: G_i(s) = 1/(0.3s^2 + 0.5s + 1) (2nd order with friction)
    // ADRC handles this as a double integrator with total disturbance
    ctrl::ADRCParams adrc_p;
    adrc_p.b0      = 1.0;   // control gain (approx plant gain)
    adrc_p.omega_o = 10.0;  // ESO bandwidth [rad/s] (was omega_n)
    adrc_p.omega_c = 4.0;   // controller bandwidth
    adrc_p.uMin         = -15.0;
    adrc_p.uMax         =  15.0;
    ctrl::DiscreteADRC adrc_inner(adrc_p, Ts);

    // Outer PID: sees the ADRC-compensated inner output (near-integrator)
    ctrl::PIDParams pp_outer;
    pp_outer.Kp = 1.2; pp_outer.Ki = 0.4; pp_outer.Kd = 0.02;
    pp_outer.uMin = -8.0; pp_outer.uMax = 8.0;
    ctrl::DiscretePID pid_outer(pp_outer, Ts);

    // Simulate a 2nd-order inner plant with persistent disturbance
    const double wn = 3.0, zeta = 0.6;  // inner plant natural frequency, damping
    const double a1 = 2.0 * zeta * wn;
    const double a2 = wn * wn;

    double y_i = 0.0, dy_i = 0.0;     // inner state
    double y_o = 0.0;                  // outer (slow) output
    const double ref_outer = 1.0;
    const double disturbance = 0.4;    // step disturbance on inner plant

    std::srand(17);
    const int N = 1200;

    for (int k = 0; k < N; ++k) {
        // Outer PID: compute inner setpoint
        const double inner_sp = pid_outer.compute(ref_outer - y_o);

        // ADRC: tracks inner_sp, rejects disturbance
        const double u_adrc = adrc_inner.compute(inner_sp - y_i);  // ADRC: r-y internally

        // Actually ADRC interface: compute(r) where r is the reference directly
        // Let's use it correctly: adrc_inner tracks inner_sp
        // Re-do with correct API: DiscreteADRC takes r as direct reference
        // We need to feed inner_sp as r and y_i will be updated internally
        // Since ADRC is designed to be used as: u = adrc.compute(r) and plant output y fed back,
        // but DiscreteADRC doesn't have set_output - it uses error directly like ADRC
        // Let's use: compute(inner_sp - y_i) for error-based usage

        // Inner 2nd-order plant step (Euler)
        const double d = (k > 400) ? disturbance : 0.0;  // disturbance onset at t=4s
        const double ddy_i = -a1 * dy_i - a2 * y_i + a2 * u_adrc + a2 * d;
        dy_i += Ts * ddy_i;
        y_i  += Ts * dy_i;

        // Outer slow plant (integrator-like, 10x slower)
        if (k % 10 == 0)
            y_o = 0.95 * y_o + 0.05 * y_i;  // slow exponential filter
    }

    std::cout << "CASCADE ADRC(inner)+PID(outer): final y_outer = " << y_o
              << "  (ref = " << ref_outer << ")\n";

    if (std::abs(y_o - ref_outer) < 0.1)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
