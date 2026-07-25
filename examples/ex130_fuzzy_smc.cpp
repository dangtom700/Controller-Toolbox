// ============================================================
//  ex130_fuzzy_smc.cpp
//  FuzzySlidingModeController - fuzzy-scheduled switching gain + boundary layer.
//
//  Plant: y[k+1] = a.y[k] + (1-a).(u[k] + d[k]),  tau = 0.2 s
//         with a MATCHED sinusoidal disturbance d[k] = 0.3.sin(2.pi.t).
//
//  Comparison is matched on reaching authority: the fixed-gain DiscreteSMC runs at
//  K = 8, and the FSMC's nominal K = 8/(1+gainSpan) = 4.44 grows back to 8 when the
//  fuzzy block sees a large |s|. So both have the same gain far from the surface;
//  the FSMC additionally relaxes it once inside the boundary layer.
//
//  Expect: comparable tracking (IAE), materially lower total control variation
//  TV = sum |u[k] - u[k-1]| - i.e. the same robustness with less actuator wear.
//
//  Sign convention: compute(y - r)  (inherited from DiscreteSMC - REVERSED from PID).
// ============================================================
#include "ControllerToolbox.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace
{
constexpr double Ts = 0.01;
constexpr int N = 2000;
constexpr double REF = 1.0;
constexpr double K_REACH = 8.0;  // switching gain far from the surface
constexpr double GAIN_SPAN = 0.8;
constexpr double PHI_NOM = 0.05;

const double a = std::exp(-Ts / 0.2);

double disturbance(int k) { return 0.3 * std::sin(2.0 * M_PI * k * Ts); }

struct Result
{
    double iae = 0.0;
    double tv = 0.0; // total variation of u
    double y_final = 0.0;
};
} // namespace

int main()
{
    // ---- Baseline: fixed-gain DiscreteSMC at the full reaching gain --------
    ctrl::SMCParams sp;
    sp.c_e = 1.0;
    sp.c_de = 5.0 * Ts; // c_de = lambda.Ts, lambda = 5 [1/s]
    sp.K = K_REACH;
    sp.phi = PHI_NOM;
    sp.uMin = -20.0;
    sp.uMax = 20.0;
    ctrl::DiscreteSMC smc(sp, Ts);

    Result fixed;
    {
        double y = 0.0, u_prev = 0.0;
        for (int k = 0; k < N; ++k)
        {
            const double u = smc.compute(y - REF); // SMC convention: e = y - r
            fixed.tv += std::abs(u - u_prev);
            u_prev = u;
            if (k > N / 2)
                fixed.iae += std::abs(REF - y) * Ts;
            y = a * y + (1.0 - a) * (u + disturbance(k));
        }
        fixed.y_final = y;
    }

    // ---- FSMC: same reaching authority, relaxed inside the layer -----------
    ctrl::FuzzySMCParams fp;
    fp.smc = sp;
    fp.smc.K = K_REACH / (1.0 + GAIN_SPAN); // grows back to K_REACH at m = 1
    fp.fuzzy.e_scale = 0.5;   // normalises the SLIDING SURFACE s, not the error
    fp.fuzzy.de_scale = 20.0; // normalises s_dot = (s[k]-s[k-1])/Ts
    fp.fuzzy.u_scale = 1.0;   // modulation universe: m = |fuzzy| / u_scale
    fp.gainSpan = GAIN_SPAN;
    fp.phiSpan = 0.5;
    fp.Kmin = 0.5;
    fp.Kmax = 20.0;
    fp.phiMin = 0.01;
    fp.phiMax = 1.0;
    ctrl::FuzzySlidingModeController fsmc(fp, Ts);

    Result fuzzy;
    double k_min = 1e9, k_max = -1e9, phi_min = 1e9, phi_max = -1e9;
    {
        double y = 0.0, u_prev = 0.0;
        for (int k = 0; k < N; ++k)
        {
            const double u = fsmc.compute(y - REF);
            fuzzy.tv += std::abs(u - u_prev);
            u_prev = u;
            if (k > N / 2)
                fuzzy.iae += std::abs(REF - y) * Ts;

            k_min = std::min(k_min, fsmc.switchingGain());
            k_max = std::max(k_max, fsmc.switchingGain());
            phi_min = std::min(phi_min, fsmc.boundaryLayer());
            phi_max = std::max(phi_max, fsmc.boundaryLayer());

            y = a * y + (1.0 - a) * (u + disturbance(k));
        }
        fuzzy.y_final = y;
    }

    std::cout << "=== Fixed-gain SMC vs fuzzy-scheduled SMC (matched reaching gain) ===\n"
              << std::fixed << std::setprecision(4)
              << std::setw(22) << "" << std::setw(14) << "IAE(2nd half)"
              << std::setw(16) << "TV of u" << std::setw(12) << "final y" << "\n"
              << std::setw(22) << "DiscreteSMC  K=8" << std::setw(14) << fixed.iae
              << std::setw(16) << fixed.tv << std::setw(12) << fixed.y_final << "\n"
              << std::setw(22) << "FuzzySMC K=4.4->8" << std::setw(14) << fuzzy.iae
              << std::setw(16) << fuzzy.tv << std::setw(12) << fuzzy.y_final << "\n"
              << "\n  control-variation reduction : "
              << (100.0 * (1.0 - fuzzy.tv / fixed.tv)) << " %\n";

    std::cout << "\n=== Scheduled parameter ranges over the run ===\n"
              << "  K   in [" << k_min << ", " << k_max << "]   (bounds " << fp.Kmin
              << " .. " << fp.Kmax << ")\n"
              << "  phi in [" << phi_min << ", " << phi_max << "]   (bounds " << fp.phiMin
              << " .. " << fp.phiMax << ")\n";

    // The point of the class is less actuator activity at comparable tracking.
    const bool tv_ok = std::isfinite(fuzzy.tv) && fuzzy.tv < fixed.tv;
    const bool track_ok = std::isfinite(fuzzy.y_final) && fuzzy.iae < 2.0 * fixed.iae;
    // The gain must actually move, and never leave its declared bounds.
    const bool sched_ok = (k_max - k_min) > 1e-3 && k_min >= fp.Kmin - 1e-9 && k_max <= fp.Kmax + 1e-9;

    const bool ok = tv_ok && track_ok && sched_ok;
    std::cout << "\n  lower control variation = " << (tv_ok ? "yes" : "no")
              << "   tracking comparable = " << (track_ok ? "yes" : "no")
              << "   gain scheduled in bounds = " << (sched_ok ? "yes" : "no") << "\n";
    std::cout << (ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
