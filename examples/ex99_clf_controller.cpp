/**
 * @file ex99_clf_controller.cpp
 * @brief Phase 3 (NC4): CLF synthesis via Sontag's universal formula.
 *
 * Stabilizes xdot = x^3 + u (an open-loop-unstable scalar drift) toward the origin using a
 * candidate CLF V(x) = x^2, comparing the synthesized law against the hand-derived Sontag
 * closed form.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;
    ctrl::CLFParams p;
    p.alpha = 2.0;
    ctrl::CLFController clf(
        [](const Eigen::VectorXd &x) { return x(0) * x(0); },                       // V
        [](const Eigen::VectorXd &x) { return 2.0 * std::pow(x(0), 4); },           // LfV = 2x*(x^3)
        [](const Eigen::VectorXd &x) { return 2.0 * x(0); },                        // LgV
        p, Ts);

    // Closed-form check at the initial point.
    const double x0 = 1.5;
    clf.setState(Eigen::VectorXd::Constant(1, x0));
    const double u0 = clf.compute(0.0);
    const double a0 = 2.0 * std::pow(x0, 4) + p.alpha * x0 * x0;
    const double b0 = 2.0 * x0;
    const double uExpected = -(a0 + std::sqrt(a0 * a0 + b0 * b0 * b0 * b0)) / b0;
    std::printf("Sontag formula check: u=%.6f  expected=%.6f\n", u0, uExpected);

    // Closed-loop simulation.
    double x = x0;
    for (int k = 0; k < 2000; ++k)
    {
        clf.setState(Eigen::VectorXd::Constant(1, x));
        const double u = clf.compute(0.0);
        x += Ts * (x * x * x + u);
    }
    std::printf("Final state after closed-loop simulation: x=%.6f\n", x);

    const bool ok = std::isfinite(u0) && std::fabs(u0 - uExpected) < 1e-9
                   && std::isfinite(x) && std::fabs(x) < 0.05;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
