/**
 * @file ex97_backstepping.cpp
 * @brief Phase 3 (NC1): Backstepping control of a 2-stage strict-feedback system.
 *
 * x1' = x2, x2' = u (the textbook double-integrator strict-feedback system) - a relative-
 * degree-2 structure FeedbackLinearisationController (relative-degree-1 only) cannot handle
 * directly, but backstepping's recursive virtual-control construction handles cleanly.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;
    std::vector<ctrl::BacksteppingController::DriftFn> f = {
        [](const Eigen::VectorXd &, int) { return 0.0; },
        [](const Eigen::VectorXd &, int) { return 0.0; },
    };
    std::vector<ctrl::BacksteppingController::GainFn> g = {
        [](const Eigen::VectorXd &, int) { return 1.0; },
        [](const Eigen::VectorXd &, int) { return 1.0; },
    };
    ctrl::BacksteppingParams p;
    p.k_gains = {2.0, 2.0};
    ctrl::BacksteppingController bc(f, g, p, Ts);

    const double ref = 1.0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    for (int k = 0; k < 2000; ++k)
    {
        bc.setState(x);
        const double u = bc.compute(ref - x(0));
        x(0) += Ts * x(1);
        x(1) += Ts * u;
    }

    std::printf("Final state: x1=%.4f (ref=%.1f)  x2=%.4f\n", x(0), ref, x(1));

    const bool ok = x.allFinite() && std::fabs(x(0) - ref) < 0.02 && std::fabs(x(1)) < 0.05;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
