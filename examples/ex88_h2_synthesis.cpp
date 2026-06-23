/**
 * ex88_h2_synthesis.cpp
 * Phase 4 Iteration 3: discrete H2/LQG output-feedback synthesis.
 *
 * Builds a small D11=0 generalised plant by hand (not via MixedSensitivity, which
 * produces D11 != 0 from its W1/W3 weight gains - see lib/DiscreteH2.h) and synthesises
 * an H2-optimal controller for it.
 */

#include "ControllerToolbox.h"
#include <iostream>

int main()
{
#if defined(CTRL_HAS_HINF)
    // Discretised first-order plant x+ = 0.9x + 1.0*u, with a 2-channel exogenous input
    // (nw=2 deliberately - a single noise channel (nw=1) is a degenerate case for the
    // filter Riccati: S2^2 == Q2*R2 always holds for a scalar B1/D21, forcing Y=0 and an
    // overly aggressive observer gain - see lib/DiscreteH2.cpp's solve() comment and the
    // [h2] Catch2 tests for the same plant shape, independently verified against
    // scipy.linalg.solve_discrete_are).
    ctrl::GeneralisedPlant P;
    P.Ts = 0.1;
    P.A   = Eigen::MatrixXd::Constant(1, 1, 0.9);
    P.B1  = (Eigen::MatrixXd(1, 2) << 0.3, 0.1).finished();  // process noise input
    P.B2  = Eigen::MatrixXd::Constant(1, 1, 1.0);            // control input
    P.C1  = (Eigen::MatrixXd(2, 1) << 1.0, 0.3).finished();  // state + cross-term cost
    P.C2  = Eigen::MatrixXd::Constant(1, 1, 1.0);            // measurement = state
    P.D11 = Eigen::MatrixXd::Zero(2, 2);
    P.D12 = (Eigen::MatrixXd(2, 1) << 0.2, 1.0).finished();  // control-cost row
    P.D21 = (Eigen::MatrixXd(1, 2) << 0.1, 0.4).finished();  // measurement noise
    P.D22 = Eigen::MatrixXd::Zero(1, 1);

    const ctrl::H2Result result = ctrl::DiscreteH2::solve(P);
    std::cout << "H2: feasible=" << result.feasible
              << "  achievedH2Norm=" << result.achievedH2Norm << "\n";

    bool ok = result.feasible && std::isfinite(result.achievedH2Norm) && result.achievedH2Norm > 0.0;

    if (result.feasible)
    {
        ctrl::DiscreteH2 ctrl_h2(result);
        const double u = ctrl_h2.compute(0.1);
        std::cout << "DiscreteH2 compute(0.1) = " << u << "\n";
        ok = ok && std::isfinite(u);
    }

    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
#else
    std::cout << "CTRL_HAS_HINF not defined - skipping\n";
    std::cout << "PASS\n";
    return 0;
#endif
}
