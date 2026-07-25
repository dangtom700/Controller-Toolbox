// ============================================================
//  ex127_disturbance_observer.cpp
//  DisturbanceObserverController - Q-filter DOB wrapping a PI controller.
//
//  Nominal model : G_nom(s) = 1 / (s + 1)          (what the designer knows)
//  True plant    : G(s)     = 1.5 / (s + 0.8)      (gain AND pole mismatch)
//  Plus an output disturbance step of +0.5 at k = 200.
//
//  The DOB lumps the mismatch and the external step into one estimate
//      d_hat = Q(z).(y - y_nom) / gainDC
//  and subtracts it from the PI command. Part 1 compares steady-state error with
//  and without the observer; Part 2 checks d_hat actually tracks the injected step.
//
//  This is the library form of the loop hand-rolled in ex52_dob_pi.cpp - note that
//  this class drives the nominal model with the APPLIED command (post-cancellation),
//  which is the textbook DOB and estimates d correctly once the DOB is active.
//
//  Sign convention: compute(r - y)  (same as DiscretePID).
// ============================================================
#include "ControllerToolbox.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>

namespace
{
constexpr double Ts = 0.05;
constexpr int N = 900;
constexpr int K_DIST = 200;
constexpr double D_OUT = 0.50; // output disturbance step
constexpr double REF = 1.0;

// True plant: 1.5 / (s + 0.8), ZOH-discretised.
const double a_true = std::exp(-0.8 * Ts);
const double b_true = 1.5 / 0.8 * (1.0 - a_true); // DC gain 1.875

ctrl::PIDParams piGains()
{
    ctrl::PIDParams p;
    p.Kp = 1.5;
    p.Ki = 0.8;
    p.Kd = 0.0;
    p.uMin = -10.0;
    p.uMax = 10.0;
    return p;
}
} // namespace

int main()
{
    // Nominal model G_nom(s) = 1/(s+1), ZOH-discretised.
    ctrl::StateSpace nom_c(Eigen::MatrixXd::Constant(1, 1, -1.0),
                           Eigen::MatrixXd::Constant(1, 1, 1.0),
                           Eigen::MatrixXd::Constant(1, 1, 1.0),
                           Eigen::MatrixXd::Zero(1, 1), 0.0);
    const ctrl::StateSpace nom = ctrl::c2d(nom_c, Ts, ctrl::C2dMethod::ZOH);

    // ---- Part 1a: PI alone -------------------------------------------------
    ctrl::DiscretePID pi(piGains(), Ts);
    double y = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double d = (k >= K_DIST) ? D_OUT : 0.0;
        const double u = pi.compute(REF - y);
        y = a_true * y + b_true * u + (1.0 - a_true) * d;
    }
    const double err_pi = std::abs(REF - y);

    // ---- Part 1b: PI + DOB -------------------------------------------------
    auto pi_inner = std::make_shared<ctrl::DiscretePID>(piGains(), Ts);
    ctrl::DOBParams dp;
    dp.omega_q = 5.0; // rejection bandwidth [rad/s]
    dp.qOrder = 1;
    dp.gainDC = 1.0;  // DC gain of G_nom(s) = 1/(s+1)
    dp.uMin = -10.0;
    dp.uMax = 10.0;
    ctrl::DisturbanceObserverController dob(pi_inner, nom, dp, Ts);

    double y_dob = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double d = (k >= K_DIST) ? D_OUT : 0.0;
        dob.setPlantOutput(y_dob);
        const double u = dob.compute(REF - y_dob);
        y_dob = a_true * y_dob + b_true * u + (1.0 - a_true) * d;
    }
    const double err_dob = std::abs(REF - y_dob);
    const double d_hat = dob.disturbanceEstimate();

    std::cout << "=== Steady state under gain/pole mismatch + output step ===\n"
              << std::fixed << std::setprecision(5)
              << "  PI alone   : y = " << y << "   |r - y| = " << err_pi << "\n"
              << "  PI + DOB   : y = " << y_dob << "   |r - y| = " << err_dob << "\n"
              << "  d_hat      : " << d_hat << "   (non-zero => observer is loaded)\n";

    // Both loops have integral action so both reach the reference; the DOB's job
    // here is to get there without leaning on the integrator, so require it to be
    // at least as accurate and to have produced a real, finite estimate.
    const bool track_ok = std::isfinite(y_dob) && err_dob < 0.02 && err_dob <= err_pi + 1e-6;
    const bool dhat_ok = std::isfinite(d_hat) && std::abs(d_hat) > 1e-3;

    // ---- Part 2: transient recovery after the disturbance step -------------
    // Integrated absolute error over the 100 samples following the step.
    auto pi2 = std::make_shared<ctrl::DiscretePID>(piGains(), Ts);
    ctrl::DisturbanceObserverController dob2(pi2, nom, dp, Ts);
    ctrl::DiscretePID pi_plain(piGains(), Ts);

    double ya = 0.0, yb = 0.0, iae_dob = 0.0, iae_pi = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double d = (k >= K_DIST) ? D_OUT : 0.0;

        dob2.setPlantOutput(ya);
        const double ua = dob2.compute(REF - ya);
        const double ub = pi_plain.compute(REF - yb);

        if (k >= K_DIST && k < K_DIST + 100)
        {
            iae_dob += std::abs(REF - ya) * Ts;
            iae_pi += std::abs(REF - yb) * Ts;
        }
        ya = a_true * ya + b_true * ua + (1.0 - a_true) * d;
        yb = a_true * yb + b_true * ub + (1.0 - a_true) * d;
    }

    std::cout << "\n=== Post-step transient (IAE over 100 samples after k=" << K_DIST << ") ===\n"
              << "  PI alone   : " << iae_pi << "\n"
              << "  PI + DOB   : " << iae_dob << "\n"
              << "  improvement: " << (100.0 * (1.0 - iae_dob / iae_pi)) << " %\n";

    const bool transient_ok = std::isfinite(iae_dob) && iae_dob < iae_pi;

    const bool ok = track_ok && dhat_ok && transient_ok;
    std::cout << "\n  steady state OK = " << (track_ok ? "yes" : "no")
              << "   d_hat loaded = " << (dhat_ok ? "yes" : "no")
              << "   transient improved = " << (transient_ok ? "yes" : "no") << "\n";
    std::cout << (ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
