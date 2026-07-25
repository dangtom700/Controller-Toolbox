// ============================================================
//  ex126_cascade_controller.cpp
//  CascadeController - series inner/outer composition.
//
//  Plant: two first-order lags in series (the textbook cascade plant).
//      inner:  x1[k+1] = a1.x1 + (1-a1).(u + d)      tau = 0.5 s  (fast, disturbed)
//      outer:  x2[k+1] = a2.x2 + (1-a2).x1           tau = 2.0 s  (slow, measured)
//
//  Part 1: a load disturbance enters at the INNER loop. A single outer PID only
//          sees it after the slow lag; the cascade's fast inner loop rejects it
//          before it propagates. Metric = IAE of the outer error after the step.
//
//  Part 2: the same cascade with a DiscreteSMC inner loop. CascadeController reads
//          inner->signConvention() and feeds (y_in - sp) instead of (sp - y_in),
//          so the SMC converges without the caller flipping anything by hand.
//
//  Sign convention: compute(r_outer - y_outer)  (same as DiscretePID).
// ============================================================
#include "ControllerToolbox.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>

namespace
{
constexpr double Ts = 0.05;
constexpr int N = 1200;
constexpr int K_DIST = 400;     // load disturbance step index
constexpr double D_MAG = 0.60;  // load disturbance magnitude [input units]
constexpr double REF = 1.0;

const double a1 = std::exp(-Ts / 0.5); // fast inner lag
const double a2 = std::exp(-Ts / 2.0); // slow outer lag

ctrl::PIDParams outerGains()
{
    ctrl::PIDParams p;
    p.Kp = 1.2;
    p.Ki = 0.35;
    p.Kd = 0.0;
    p.uMin = -5.0;
    p.uMax = 5.0;
    return p;
}

ctrl::PIDParams innerGains()
{
    ctrl::PIDParams p;
    p.Kp = 2.5;
    p.Ki = 5.0;
    p.Kd = 0.0;
    p.uMin = -10.0;
    p.uMax = 10.0;
    return p;
}
} // namespace

int main()
{
    // ---- Part 1a: single-loop PID (baseline) -------------------------------
    // Same outer gains as the cascade so the comparison isolates the structure,
    // not the tuning.
    ctrl::DiscretePID single(outerGains(), Ts);
    double x1 = 0.0, x2 = 0.0, iae_single = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double d = (k >= K_DIST) ? D_MAG : 0.0;
        const double u = single.compute(REF - x2);
        if (k >= K_DIST)
            iae_single += std::abs(REF - x2) * Ts;
        x2 = a2 * x2 + (1.0 - a2) * x1;
        x1 = a1 * x1 + (1.0 - a1) * (u + d);
    }

    // ---- Part 1b: cascade --------------------------------------------------
    auto outer = std::make_shared<ctrl::DiscretePID>(outerGains(), Ts);
    auto inner = std::make_shared<ctrl::DiscretePID>(innerGains(), Ts);
    ctrl::CascadeParams cp;
    cp.spMin = -5.0;
    cp.spMax = 5.0;
    ctrl::CascadeController casc(outer, inner, cp, Ts);

    x1 = 0.0;
    x2 = 0.0;
    double iae_casc = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double d = (k >= K_DIST) ? D_MAG : 0.0;
        casc.setInnerMeasurement(x1);
        const double u = casc.compute(REF - x2);
        if (k >= K_DIST)
            iae_casc += std::abs(REF - x2) * Ts;
        x2 = a2 * x2 + (1.0 - a2) * x1;
        x1 = a1 * x1 + (1.0 - a1) * (u + d);
    }

    std::cout << "=== Inner-loop disturbance rejection (IAE after step at k=" << K_DIST << ") ===\n"
              << std::fixed << std::setprecision(4)
              << "  single-loop PID : IAE = " << iae_single << "\n"
              << "  cascade PID/PID : IAE = " << iae_casc << "\n"
              << "  improvement     : " << (100.0 * (1.0 - iae_casc / iae_single)) << " %\n";

    const bool reject_ok = std::isfinite(iae_casc) && iae_casc < iae_single;

    // ---- Part 2: SMC inner loop, sign convention handled automatically -----
    auto outer2 = std::make_shared<ctrl::DiscretePID>(outerGains(), Ts);
    ctrl::SMCParams sp;
    sp.c_e = 1.0;
    sp.c_de = 5.0 * Ts; // c_de = lambda.Ts, lambda = 5 [1/s]
    sp.K = 6.0;
    sp.phi = 0.30;
    sp.uMin = -10.0;
    sp.uMax = 10.0;
    auto inner_smc = std::make_shared<ctrl::DiscreteSMC>(sp, Ts);
    ctrl::CascadeController casc_smc(outer2, inner_smc, cp, Ts);

    x1 = 0.0;
    x2 = 0.0;
    for (int k = 0; k < N; ++k)
    {
        casc_smc.setInnerMeasurement(x1);
        const double u = casc_smc.compute(REF - x2);
        x2 = a2 * x2 + (1.0 - a2) * x1;
        x1 = a1 * x1 + (1.0 - a1) * u;
    }

    const double e_smc = std::abs(REF - x2);
    std::cout << "\n=== PID outer + SMC inner (signConvention auto-flip) ===\n"
              << "  inner reports e = y - r : "
              << (inner_smc->signConvention() == ctrl::SignConvention::TrackingErrorYMinusR ? "yes" : "no")
              << "\n  final |r - y|          : " << e_smc << "\n";

    const bool smc_ok = std::isfinite(x2) && e_smc < 0.15;

    const bool ok = reject_ok && smc_ok;
    std::cout << "\n  disturbance rejection improved = " << (reject_ok ? "yes" : "no")
              << "   SMC inner converged = " << (smc_ok ? "yes" : "no") << "\n";
    std::cout << (ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
