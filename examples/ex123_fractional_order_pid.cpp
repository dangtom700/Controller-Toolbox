// ============================================================
//  ex123_fractional_order_pid.cpp
//  FractionalOrderPID (PI^lambda D^mu) with Oustaloup-approximated operators.
//
//  Part 1: validate the Oustaloup fractional operator s^{alpha}.  At the
//          geometric band centre omega = sqrt(wb*wh) the magnitude is
//          |s^{alpha}| = omega^{alpha}; centring the band at 1 rad/s makes the
//          expected gain 1.0 for any order.
//
//  Part 2: closed-loop step tracking on a first-order plant
//          y[k+1] = 0.8*y[k] + 0.2*u[k]  (DC gain 1) - the fractional-integral
//          branch supplies the steady control, driving the error toward zero.
//
//  Sign convention: compute(r - y)  (same as DiscretePID).
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main()
{
    // ---- Part 1: fractional operator magnitude at the band centre ----
    const double Tsf = 0.005;                 // fine step: Nyquist ~628 rad/s >> wh
    ctrl::FractionalDifferintegrator half(0.5, 0.01, 100.0, 5, Tsf); // s^{0.5}, band centre = 1

    const double w = 1.0;                      // rad/s (geometric centre of [0.01,100])
    double ymax = 0.0;
    const int Nsin = 40000;
    for (int k = 0; k < Nsin; ++k)
    {
        const double out = half.compute(std::sin(w * k * Tsf));
        if (k > Nsin - 4000)                   // steady-state window
            ymax = std::max(ymax, std::abs(out));
    }
    std::cout << "=== Oustaloup s^0.5 gain at band centre (expected 1.0) ===\n";
    std::cout << "  measured amplitude = " << std::fixed << std::setprecision(4) << ymax
              << "  (sections=" << half.sectionCount() << ")\n";
    const bool op_ok = std::isfinite(ymax) && ymax > 0.85 && ymax < 1.15;

    // ---- Part 2: closed-loop FOPID step tracking ----
    const double Ts = 0.01;
    ctrl::FOPIDParams p;
    p.Kp     = 0.5;
    p.Ki     = 0.3;
    p.Kd     = 0.02;
    p.lambda = 0.9;    // near-integer integral order
    p.mu     = 0.6;    // fractional derivative
    p.wb     = 0.01;
    p.wh     = 100.0;
    p.N      = 4;
    p.uMin   = -10.0;
    p.uMax   =  10.0;
    ctrl::FractionalOrderPID fopid(p, Ts);

    double y = 0.0;
    const double ref = 1.0;
    std::cout << "\n=== FractionalOrderPID step tracking (first-order plant) ===\n";
    std::cout << std::setw(8) << "t[s]" << std::setw(12) << "y"
              << std::setw(10) << "u" << "\n";
    for (int k = 0; k <= 2000; ++k)
    {
        const double u = fopid.compute(ref - y);
        y = 0.8 * y + 0.2 * u;
        if (k % 250 == 0)
            std::cout << std::fixed << std::setprecision(4)
                      << std::setw(8) << k * Ts
                      << std::setw(12) << y
                      << std::setw(10) << u << "\n";
    }

    const double ef = std::abs(ref - y);
    std::cout << "\n  s^0.5 gain OK = " << (op_ok ? "yes" : "no")
              << "   final |e| = " << ef << "\n";

    const bool track_ok = std::isfinite(y) && ef < 0.1;
    const bool ok = op_ok && track_ok;
    std::cout << (ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
