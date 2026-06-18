/**
 * ex57_mrac.cpp
 * Model Reference Adaptive Control (MRAC) on a first-order plant with gain jump.
 *
 * Plant (discrete, Ts=0.1):  y[k+1] = 0.7*y[k] + b_p*u[k]
 *   Phase 1 (k=0..499):  b_p = 0.5  (nominal)
 *   Phase 2 (k=500..999): b_p = 0.9  (gain jump +80%, unknown to controller)
 *
 * Reference model: a_m=0.5, b_m=0.5 -> DC gain 1, time constant faster than plant.
 * Step reference: r = 1.0 (constant).
 *
 * MRAC adapts theta_r (feed-forward) and theta_y (feedback) online.
 *
 * PASS criteria:
 *   Phase 1 end: |e_m| < 0.05  (adapted to nominal plant)
 *   Phase 2 end: |e_m| < 0.10  (re-adapted after gain jump)
 *   Parameters bounded: ||theta|| <= theta_max at all times
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    constexpr double Ts      = 0.1;
    constexpr double a_plant = 0.7;
    const double r           = 1.0;   // constant reference

    ctrl::MRACParams mp;
    mp.a_m      = 0.5;    // reference model pole
    mp.b_m      = 0.5;    // reference model gain (b_m = 1 - a_m -> DC gain 1)
    mp.gamma_r  = 3.0;    // fast adaptation for feed-forward
    mp.gamma_y  = 1.5;    // moderate feedback adaptation
    mp.sigma    = 0.02;   // sigma-modification prevents drift on constant reference
    mp.theta_max = 20.0;

    ctrl::MRACController mrac(mp, Ts);

    bool all_pass = true;
    double y       = 0.0;
    double b_plant = 0.5;   // Phase 1 gain

    for (int k = 0; k < 1000; ++k)
    {
        if (k == 500) b_plant = 0.9;  // gain jump at k=500

        mrac.setReference(r);
        const double u = mrac.compute(y);

        // True plant: y[k+1] = a_plant * y[k] + b_plant * u[k]
        y = a_plant * y + b_plant * u;

        const double theta_norm = std::sqrt(mrac.theta_r() * mrac.theta_r() +
                                            mrac.theta_y() * mrac.theta_y());
        if (theta_norm > mp.theta_max + 1e-6)
        {
            std::cerr << "FAIL: ||theta|| = " << theta_norm << " > theta_max at k=" << k << "\n";
            all_pass = false;
        }
    }

    // Phase 1 check: run a fresh simulation, sample e_m at k=499
    {
        ctrl::MRACController m2(mp, Ts);
        double yy = 0.0;
        double em_end = 0.0;
        for (int k = 0; k < 500; ++k)
        {
            m2.setReference(r);
            const double u = m2.compute(yy);
            yy = a_plant * yy + 0.5 * u;
            em_end = m2.modelError();
        }
        std::cout << "Phase 1 |e_m| at k=499: " << std::abs(em_end) << "  (need < 0.05)\n";
        if (std::abs(em_end) > 0.05)
        {
            std::cerr << "FAIL: Phase 1 did not adapt.\n";
            all_pass = false;
        }
    }

    // Phase 2 check: run full 1000 steps, sample e_m at k=999
    {
        ctrl::MRACController m3(mp, Ts);
        double yy = 0.0, em_end = 0.0;
        double bp = 0.5;
        for (int k = 0; k < 1000; ++k)
        {
            if (k == 500) bp = 0.9;
            m3.setReference(r);
            const double u = m3.compute(yy);
            yy = a_plant * yy + bp * u;
            em_end = m3.modelError();
        }
        std::cout << "Phase 2 |e_m| at k=999: " << std::abs(em_end) << "  (need < 0.10)\n";
        if (std::abs(em_end) > 0.10)
        {
            std::cerr << "FAIL: Phase 2 did not re-adapt to gain jump.\n";
            all_pass = false;
        }
    }

    // Print final adaptive parameters
    {
        ctrl::MRACController mf(mp, Ts);
        double yy = 0.0;
        double bp = 0.5;
        for (int k = 0; k < 1000; ++k)
        {
            if (k == 500) bp = 0.9;
            mf.setReference(r);
            const double u = mf.compute(yy);
            yy = a_plant * yy + bp * u;
        }
        std::cout << "Final: theta_r=" << mf.theta_r() << "  theta_y=" << mf.theta_y()
                  << "  y_m=" << mf.modelOutput() << "  y=" << y << "\n";
        // Ideal theta_r* = b_m / b_p_phase2 = 0.5/0.9 approx = 0.556
        // Ideal theta_y* = (a_m - a_plant) / b_p = (0.5-0.7)/0.9 approx = -0.222
    }

    if (all_pass) std::cout << "PASS\n";
    else          std::cout << "FAIL\n";

    return all_pass ? 0 : 1;
}
