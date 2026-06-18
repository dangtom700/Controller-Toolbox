/**
 * ex35_cascade_pid.cpp
 * Cascade (inner/outer) PID control using ControllerStack::Supervisory.
 *
 * Inner loop (fast, temperature): controls T_inner to setpoint from outer
 * Outer loop (slow, temperature set): provides inner setpoint based on T_outer error
 *
 * Simplified: uses two PIDs, the outer feeds the inner's setpoint.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.05;  // 50 ms sample time

    // Inner-loop PID: fast plant tau=0.5 s
    ctrl::PIDParams pp_inner;
    pp_inner.Kp   = 2.0;
    pp_inner.Ki   = 1.5;
    pp_inner.Kd   = 0.05;
    pp_inner.uMin = -10.0;
    pp_inner.uMax =  10.0;
    ctrl::DiscretePID pid_inner(pp_inner, Ts);

    // Outer-loop PID: slower plant tau=2.0 s, tuned for gentle response
    ctrl::PIDParams pp_outer;
    pp_outer.Kp   = 0.8;
    pp_outer.Ki   = 0.2;
    pp_outer.Kd   = 0.0;
    pp_outer.uMin = -5.0;
    pp_outer.uMax =  5.0;
    ctrl::DiscretePID pid_outer(pp_outer, Ts);

    // Simulate cascade: outer PID provides setpoint to inner PID
    // Inner plant: tau_inner=0.5s -> a_i = exp(-Ts/tau_i)
    // Outer plant: tau_outer=2.0s -> a_o = exp(-Ts/tau_o)
    const double tau_inner = 0.5;
    const double tau_outer = 2.0;
    const double K_inner   = 1.0;
    const double K_outer   = 1.0;

    const double a_i = std::exp(-Ts / tau_inner);
    const double a_o = std::exp(-Ts / tau_outer);

    double y_inner = 0.0, y_outer = 0.0;
    const double outer_ref = 1.0;
    const int    N         = 1000;
    double final_y_outer   = 0.0;

    for (int k = 0; k < N; ++k) {
        // Outer loop: error = outer_ref - y_outer -> inner setpoint
        const double e_outer    = outer_ref - y_outer;
        const double inner_sp   = pid_outer.compute(e_outer);

        // Inner loop: error = inner_sp - y_inner -> plant input u
        const double e_inner = inner_sp - y_inner;
        const double u       = pid_inner.compute(e_inner);

        // Cascade plant dynamics
        y_inner = a_i * y_inner + (1.0 - a_i) * K_inner * u;
        y_outer = a_o * y_outer + (1.0 - a_o) * K_outer * y_inner;

        final_y_outer = y_outer;
    }

    std::cout << "Cascade control final y_outer = " << final_y_outer
              << "  (ref = " << outer_ref << ")\n";

    if (std::abs(final_y_outer - outer_ref) < 0.05)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return 0;
}
