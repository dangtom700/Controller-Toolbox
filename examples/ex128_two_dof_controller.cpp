// ============================================================
//  ex128_two_dof_controller.cpp
//  TwoDOFController - functional feedforward + feedback trim.
//
//  Plant: y[k+1] = a.y[k] + (1-a).K_p.(u[k] + d[k])   (DC gain K_p, load d)
//
//  Part 1: the feedforward is the analytic plant inverse u_ff = r / K_p, exactly the
//          "physics inversion + PID trim" shape used across the case studies
//          (converter steady-state duty, airship trim input, PCM load feedforward).
//          Expect: setpoint reached far faster than feedback alone, and the feedback
//          term decaying to ~0 because the feedforward already carries the load.
//
//  Part 2: a MEASURED disturbance is fed to the same callable, so the feedforward
//          cancels it before the feedback controller ever sees an error.
//
//  Contrast with the two existing near-neighbours:
//    - FeedforwardController : needs a designed StateSpace G_ff(z) driven by r
//    - PIDParams::b_weight   : setpoint weighting inside a single PID
//  This class takes any double(double r, double d) callable.
//
//  Sign convention: compute(r - y)  (same as DiscretePID).
// ============================================================
#include "ControllerToolbox.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

namespace
{
constexpr double Ts = 0.05;
constexpr int N = 400;
constexpr double K_PLANT = 2.5; // plant DC gain
constexpr double REF = 1.0;

const double a = std::exp(-Ts / 1.0); // tau = 1 s

ctrl::PIDParams trimGains()
{
    ctrl::PIDParams p;
    p.Kp = 0.6;
    p.Ki = 0.30;
    p.Kd = 0.0;
    p.uMin = -5.0;
    p.uMax = 5.0;
    return p;
}

// Settling index: first k after which |r - y| stays below 2 % of r.
int settleIndex(const std::vector<double> &y, double ref)
{
    int idx = static_cast<int>(y.size());
    for (int k = static_cast<int>(y.size()) - 1; k >= 0; --k)
    {
        if (std::abs(ref - y[k]) > 0.02 * std::abs(ref))
            break;
        idx = k;
    }
    return idx;
}
} // namespace

int main()
{
    // ---- Part 1a: feedback only (baseline) ---------------------------------
    ctrl::DiscretePID fb_only(trimGains(), Ts);
    std::vector<double> y_fb(N, 0.0);
    double y = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double u = fb_only.compute(REF - y);
        y = a * y + (1.0 - a) * K_PLANT * u;
        y_fb[k] = y;
    }

    // ---- Part 1b: 2-DOF with the analytic inverse as feedforward ----------
    auto trim = std::make_shared<ctrl::DiscretePID>(trimGains(), Ts);
    auto ff = [](double r, double d) { return (r / K_PLANT) - d; }; // inverse + load cancel
    ctrl::TwoDOFParams tp;
    tp.uMin = -5.0;
    tp.uMax = 5.0;
    ctrl::TwoDOFController c2(trim, ff, tp, Ts);

    std::vector<double> y_2dof(N, 0.0);
    y = 0.0;
    c2.setReference(REF);
    for (int k = 0; k < N; ++k)
    {
        const double u = c2.compute(REF - y);
        y = a * y + (1.0 - a) * K_PLANT * u;
        y_2dof[k] = y;
    }

    const int s_fb = settleIndex(y_fb, REF);
    const int s_2d = settleIndex(y_2dof, REF);
    const double fb_share = std::abs(c2.feedbackTerm());

    std::cout << "=== Step response, feedback-only vs 2-DOF (plant DC gain " << K_PLANT << ") ===\n"
              << std::fixed << std::setprecision(4)
              << "  feedback only : settles at k = " << s_fb << "  (t = " << s_fb * Ts << " s)\n"
              << "  2-DOF         : settles at k = " << s_2d << "  (t = " << s_2d * Ts << " s)\n"
              << "  final u_ff    : " << c2.feedforwardTerm() << "   (exact inverse = "
              << REF / K_PLANT << ")\n"
              << "  final u_fb    : " << c2.feedbackTerm() << "   (-> 0 when FF is exact)\n";

    const bool faster_ok = (s_2d < s_fb);
    const bool ff_carries_ok = (fb_share < 0.02) && std::isfinite(y_2dof[N - 1]);

    // ---- Part 2: measured-disturbance feedforward -------------------------
    // A load step enters the plant AND is measured, so the same callable cancels it.
    auto trim2 = std::make_shared<ctrl::DiscretePID>(trimGains(), Ts);
    ctrl::TwoDOFController c2d(trim2, ff, tp, Ts);
    c2d.setReference(REF);

    double y_d = 0.0, peak_err = 0.0;
    const double d_load = 0.20;
    for (int k = 0; k < N; ++k)
    {
        const double d = (k >= 200) ? d_load : 0.0;
        c2d.setMeasuredDisturbance(d);
        const double u = c2d.compute(REF - y_d);
        y_d = a * y_d + (1.0 - a) * K_PLANT * (u + d);
        if (k >= 200)
            peak_err = std::max(peak_err, std::abs(REF - y_d));
    }

    std::cout << "\n=== Measured load step of " << d_load << " at k = 200 ===\n"
              << "  peak |r - y| after the step : " << peak_err << "\n";

    const bool dff_ok = std::isfinite(y_d) && peak_err < 0.02;

    const bool ok = faster_ok && ff_carries_ok && dff_ok;
    std::cout << "\n  faster settling = " << (faster_ok ? "yes" : "no")
              << "   FF carries the load = " << (ff_carries_ok ? "yes" : "no")
              << "   measured-d cancelled = " << (dff_ok ? "yes" : "no") << "\n";
    std::cout << (ok ? "PASS\n" : "FAIL\n");
    return ok ? 0 : 1;
}
