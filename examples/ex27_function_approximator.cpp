// ============================================================
//  ex27_function_approximator.cpp
//
//  Demonstrates TaylorApproximator, PadeApproximator, and
//  the fractional dead-time SmithPredictor.
//
//  Three sections:
//    A) Taylor (polynomial) fit to  f(x) = sin(x)  on [0, 2]
//    B) Padé [2/2] fit to  f(x) = 1/(1+x)  on [0, 3]  (true pole at x=-1)
//    C) Smith Predictor with fractional dead-time theta=0.73 s, Ts=0.5 s
//       (d_int=1, theta_frac=0.23 s) — compare integer-rounded vs Padé.
// ============================================================
#include "ControllerToolbox.h"
#include "FunctionApproximator.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Utility: print a small table header
// ─────────────────────────────────────────────────────────────────────────────
static void header(const std::string &title)
{
    std::cout << "\n══════════════════════════════════════\n";
    std::cout << "  " << title << "\n";
    std::cout << "══════════════════════════════════════\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Simple plant with fractional dead-time for section C
// ─────────────────────────────────────────────────────────────────────────────
struct FractDelayPlant
{
    ctrl::StateSpace model;  // delay-free first-order plant
    double           theta;  // true dead time [s]
    double           Ts;

    // Ring buffer of past outputs (fractional delay approximated by holding)
    std::vector<std::pair<double,double>> history; // (time, y) pairs
    Eigen::VectorXd x;

    FractDelayPlant(ctrl::StateSpace m, double theta_, double Ts_)
        : model(m), theta(theta_), Ts(Ts_), x(Eigen::VectorXd::Zero(m.stateSize()))
    {}

    // Simulate one step: apply u now, observe y delayed by theta seconds.
    double step(double u, double t)
    {
        Eigen::VectorXd uv(1); uv << u;
        double y_now = ctrl::ssStep(model, x, uv)(0);

        // Store output with timestamp
        history.push_back({t, y_now});

        // Find most recent sample taken at (t - theta)
        const double t_delayed = t - theta;
        double y_out = 0.0;
        for (int i = static_cast<int>(history.size()) - 1; i >= 0; --i)
        {
            if (history[i].first <= t_delayed + 1e-9)
            {
                y_out = history[i].second;
                break;
            }
        }
        return y_out;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    // =========================================================================
    // A) Taylor approximation of sin(x) on [0, 2]
    // =========================================================================
    header("A) Taylor fit: f(x) = sin(x) on [0, 2]");

    {
        // Build dataset: 20 uniformly spaced points
        const int N = 20;
        std::vector<double> xs(N), ys(N);
        for (int i = 0; i < N; ++i)
        {
            xs[i] = 2.0 * i / (N - 1);
            ys[i] = std::sin(xs[i]);
        }

        // Fit degrees 3, 5, 7 and compare accuracy
        for (int deg : {3, 5, 7})
        {
            ctrl::TaylorApproximator ta(xs, ys, deg);
            std::cout << "Degree " << deg << "  fit RMSE = "
                      << std::scientific << std::setprecision(3) << ta.fitRMSE() << "\n";
            std::cout << "  Coefficients:";
            for (double c : ta.coefficients())
                std::cout << "  " << std::fixed << std::setprecision(6) << c;
            std::cout << "\n";

            // Spot-check at x = 0.5, 1.0, 1.5
            std::cout << "  Spot-check:";
            for (double xv : {0.5, 1.0, 1.5})
            {
                double approx = ta.evaluate(xv);
                double exact  = std::sin(xv);
                std::cout << "  x=" << xv
                          << " err=" << std::scientific << std::setprecision(2)
                          << std::abs(approx - exact);
            }
            std::cout << "\n\n";
        }
    }

    // =========================================================================
    // B) Padé [2/2] approximation of f(x) = 1/(1+x) on [0, 3]
    // =========================================================================
    header("B) Pade [2/2] fit: f(x) = 1/(1+x) on [0, 3]");

    {
        const int N = 30;
        std::vector<double> xs(N), ys(N);
        for (int i = 0; i < N; ++i)
        {
            xs[i] = 3.0 * i / (N - 1);
            ys[i] = 1.0 / (1.0 + xs[i]);
        }

        ctrl::PadeApproximator pa(xs, ys, 2, 2);
        std::cout << "Fit RMSE = " << std::scientific << std::setprecision(3)
                  << pa.fitRMSE() << "\n";

        std::cout << "Numerator:   ";
        for (double c : pa.numerator())   std::cout << std::fixed << std::setprecision(6) << c << "  ";
        std::cout << "\nDenominator: ";
        for (double c : pa.denominator()) std::cout << std::fixed << std::setprecision(6) << c << "  ";
        std::cout << "\n\n";

        // Compare with Taylor degree-4 on the same data
        ctrl::TaylorApproximator ta(xs, ys, 4);
        std::cout << std::setw(8) << "x"
                  << std::setw(12) << "exact"
                  << std::setw(12) << "Pade[2/2]"
                  << std::setw(12) << "Taylor-4"
                  << std::setw(12) << "Pade err"
                  << std::setw(12) << "Taylor err" << "\n";

        for (double xv : {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0})
        {
            double exact   = 1.0 / (1.0 + xv);
            double pade_v  = pa.evaluate(xv);
            double taylor_v= ta.evaluate(xv);
            std::cout << std::fixed << std::setprecision(4)
                      << std::setw(8)  << xv
                      << std::setw(12) << exact
                      << std::setw(12) << pade_v
                      << std::setw(12) << taylor_v
                      << std::scientific << std::setprecision(2)
                      << std::setw(12) << std::abs(pade_v  - exact)
                      << std::setw(12) << std::abs(taylor_v - exact)
                      << "\n";
        }

        // Pole check
        bool has_pole = pa.hasPoleInDomain(0.0, 3.0);
        std::cout << "\nPole in [0,3]: " << (has_pole ? "YES" : "no") << "\n";
    }

    // =========================================================================
    // C) Smith Predictor: integer-rounded vs fractional Pade
    //    Plant G(s) = 1/(s + 0.5),  theta = 0.73 s,  Ts = 0.5 s
    //    -> d_int = 1 step,  theta_frac = 0.23 s
    // =========================================================================
    header("C) Smith Predictor: integer vs Pade fractional delay");

    {
        const double Ts    = 0.5;
        const double theta = 0.73;  // true dead time [s]

        // Continuous plant G(s) = 1/(s + 0.5) -> ZOH at Ts=0.5:
        //   A = exp(-0.5*0.5) = 0.7788,  B = (1-0.7788)/0.5 = 0.4424
        ctrl::StateSpace G0(
            (Eigen::MatrixXd(1,1) << std::exp(-0.5 * Ts)).finished(),
            (Eigen::MatrixXd(1,1) << (1.0 - std::exp(-0.5 * Ts)) / 0.5).finished(),
            (Eigen::MatrixXd(1,1) << 1.0).finished(),
            (Eigen::MatrixXd(1,1) << 0.0).finished(),
            Ts);

        // Simple PI inner controller
        ctrl::PIDParams pp;
        pp.Kp = 1.5; pp.Ki = 0.5; pp.Kd = 0.0;
        pp.N  = 10.0; pp.uMin = -10.0; pp.uMax = 10.0;

        // ── Run 1: integer-rounded Smith (d=1, theta_rounded = 0.5 s) ──
        FractDelayPlant plant1(G0, theta, Ts);
        {
            auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
            ctrl::SmithPredictor sp(inner, G0, 1); // integer only: d=1
            double y = 0.0, ref = 1.0;

            std::cout << "\n--- Integer-rounded Smith (d=1, rounded theta=0.5 s) ---\n";
            std::cout << std::setw(6) << "k" << std::setw(8) << "t[s]"
                      << std::setw(10) << "y" << std::setw(10) << "u" << "\n";

            for (int k = 0; k <= 30; ++k)
            {
                double u = sp.compute(ref - y);
                y = plant1.step(u, k * Ts);
                if (k % 3 == 0)
                    std::cout << std::setw(6) << k
                              << std::fixed << std::setprecision(3)
                              << std::setw(8)  << k * Ts
                              << std::setw(10) << y
                              << std::setw(10) << u << "\n";
            }
        }

        // ── Run 2: fractional-delay Smith (d=1 + Pade for 0.23 s) ──
        FractDelayPlant plant2(G0, theta, Ts);
        {
            auto inner = std::make_shared<ctrl::DiscretePID>(pp, Ts);
            // Convenience constructor: pass full theta + Ts
            ctrl::SmithPredictor sp(inner, G0, theta, Ts);
            double y = 0.0, ref = 1.0;

            std::cout << "\n--- Fractional-delay Smith (d=1, Pade for theta_frac=0.23 s) ---\n";
            std::cout << std::setw(6) << "k" << std::setw(8) << "t[s]"
                      << std::setw(10) << "y" << std::setw(10) << "u" << "\n";

            for (int k = 0; k <= 30; ++k)
            {
                double u = sp.compute(ref - y);
                y = plant2.step(u, k * Ts);
                if (k % 3 == 0)
                    std::cout << std::setw(6) << k
                              << std::fixed << std::setprecision(3)
                              << std::setw(8)  << k * Ts
                              << std::setw(10) << y
                              << std::setw(10) << u << "\n";
            }
        }

        // ── Show the Pade filter coefficients ──
        {
            const double theta_frac = theta - 1 * Ts; // = 0.23 s
            ctrl::StateSpace Hfrac = ctrl::padeDelayFilter(theta_frac, Ts);
            std::cout << "\nPade filter H_frac (theta_frac=" << theta_frac << " s, Ts=" << Ts << " s):\n";
            std::cout << "  A = " << Hfrac.A(0,0) << "\n";
            std::cout << "  B = " << Hfrac.B(0,0) << "\n";
            std::cout << "  C = " << Hfrac.C(0,0) << "\n";
            std::cout << "  D = " << Hfrac.D(0,0) << "\n";
        }
    }

    return 0;
}
