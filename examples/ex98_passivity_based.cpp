/**
 * @file ex98_passivity_based.cpp
 * @brief Phase 3 (NC2): Passivity-based (PD+) regulation of a single pendulum.
 *
 * Regulates a single-pendulum Euler-Lagrange system to a nonzero desired angle despite the
 * gravitational nonlinearity, monitoring the shaped storage-energy function's non-increase.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01, ml2 = 1.0, mgl = 9.8;
    ctrl::PBCParams p;
    p.Kp = Eigen::MatrixXd::Constant(1, 1, 10.0);
    p.Kd = Eigen::MatrixXd::Constant(1, 1, 4.0);

    auto M  = [ml2](const Eigen::VectorXd &) { return Eigen::MatrixXd::Constant(1, 1, ml2); };
    auto dV = [mgl](const Eigen::VectorXd &q) {
        Eigen::VectorXd g(1); g(0) = mgl * std::sin(q(0)); return g;
    };
    auto C  = [](const Eigen::VectorXd &, const Eigen::VectorXd &) {
        return Eigen::MatrixXd::Zero(1, 1);
    };

    ctrl::PassivityBasedController pbc(M, dV, C, p, Ts);
    pbc.setDesired(Eigen::VectorXd::Constant(1, 0.5));

    Eigen::VectorXd state = Eigen::VectorXd::Zero(2); // [q; qdot]
    double prevEnergy = std::numeric_limits<double>::infinity();
    bool everIncreased = false;
    for (int k = 0; k < 3000; ++k)
    {
        const Eigen::VectorXd u = pbc.computeVec(state);
        const double energy = pbc.storageEnergy();
        if (k > 50 && energy > prevEnergy + 1e-6) everIncreased = true;
        prevEnergy = energy;

        const double qddot = (u(0) - mgl * std::sin(state(0))) / ml2;
        state(1) += Ts * qddot;
        state(0) += Ts * state(1);
    }

    std::printf("Final angle: q=%.4f (desired=0.5)  storage energy=%.6f\n",
                state(0), pbc.storageEnergy());

    const bool ok = state.allFinite() && std::fabs(state(0) - 0.5) < 0.02 && !everIncreased;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
