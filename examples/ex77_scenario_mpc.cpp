/**
 * @file ex77_scenario_mpc.cpp
 * @brief ScenarioMPC: stochastic MPC compared to deterministic MPC on a noisy FOPDT.
 *
 * Plant: FOPDT ZOH-discrete at Ts=0.1s + additive Gaussian noise w~N(0, sigma_w^2).
 *   y[k+1] = 0.9*y[k] + 0.1*u[k] + w[k]
 *
 * Comparison over 200 steps at ref=1:
 *   - Scenario MPC  (N_samples=40, sigma_w=0.05): noise-averaged QP each step.
 *   - Deterministic MPC (N_samples=1, sigma_w=0): ignores noise.
 *
 * Expected output:
 *   Both controllers converge. ScenarioMPC shows lower variance in tracking error.
 *   [PASS] ScenarioMPC converged to reference.
 */

#include <ControllerToolbox.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

int main()
{
    constexpr double Ts      = 0.1;
    constexpr double sigma_w = 0.05;
    constexpr int    N       = 200;
    constexpr double ref     = 1.0;

    // --- Plant ---------------------------------------------------------------
    Eigen::Matrix<double,1,1> A; A << 0.9;
    Eigen::Matrix<double,1,1> B; B << 0.1;
    Eigen::Matrix<double,1,1> C; C << 1.0;
    Eigen::Matrix<double,1,1> D; D << 0.0;
    ctrl::StateSpace sys(A, B, C, D, Ts);

    // --- Shared QP parameters ------------------------------------------------
    auto makeParams = [&](int n_samples, double sw) {
        ctrl::ScenarioMPCParams p;
        p.Np = 10; p.Nu = 4; p.Ts = Ts;
        p.Q  = Eigen::MatrixXd::Identity(1, 1);
        p.R  = Eigen::MatrixXd::Identity(1, 1) * 0.1;
        p.Sigma_w  = Eigen::MatrixXd::Identity(1, 1) * (sw * sw);
        p.N_samples = n_samples; p.seed = 42;
        p.uMin = Eigen::VectorXd::Constant(1, -2.0);
        p.uMax = Eigen::VectorXd::Constant(1,  2.0);
        return p;
    };

    ctrl::ScenarioMPC smpc(sys, makeParams(40, sigma_w));   // stochastic
    ctrl::ScenarioMPC dmpc(sys, makeParams(1,  0.0));       // deterministic

    // --- Noise source --------------------------------------------------------
    std::mt19937 rng(1337);
    std::normal_distribution<double> noise(0.0, sigma_w);

    // --- Closed-loop simulation ----------------------------------------------
    double y_s = 0.0, y_d = 0.0;
    double iae_s = 0.0, iae_d = 0.0;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Step |  y_smpc  |  y_dmpc  | e_smpc  | e_dmpc\n";
    std::cout << "-----+----------+----------+---------+--------\n";

    for (int k = 0; k < N; ++k) {
        double w = noise(rng);

        // ScenarioMPC step
        Eigen::VectorXd xs(1); xs(0) = y_s;
        Eigen::VectorXd rv(1); rv(0) = ref;
        smpc.setState(xs); smpc.setReference(rv);
        double us = smpc.computeControl()(0);
        y_s = 0.9 * y_s + 0.1 * us + w;

        // Deterministic MPC step
        Eigen::VectorXd xd(1); xd(0) = y_d;
        dmpc.setState(xd); dmpc.setReference(rv);
        double ud = dmpc.computeControl()(0);
        y_d = 0.9 * y_d + 0.1 * ud + w;   // same noise realisation for fairness

        iae_s += std::abs(ref - y_s) * Ts;
        iae_d += std::abs(ref - y_d) * Ts;

        if (k % 40 == 0 || k == N - 1)
            std::cout << std::setw(4) << k
                      << " | " << std::setw(8) << y_s
                      << " | " << std::setw(8) << y_d
                      << " | " << std::setw(7) << (ref - y_s)
                      << " | " << std::setw(7) << (ref - y_d) << '\n';
    }

    std::cout << "\nIAE ScenarioMPC:  " << iae_s << '\n';
    std::cout << "IAE Deterministic: " << iae_d << '\n';

    // --- Convergence check ---------------------------------------------------
    bool ok = (std::abs(ref - y_s) < 0.10);
    if (!ok)
        std::cerr << "[FAIL] ScenarioMPC did not converge: final error = "
                  << std::abs(ref - y_s) << '\n';
    else
        std::cout << "[PASS] ScenarioMPC converged to reference.\n";

    return ok ? 0 : 1;
}
