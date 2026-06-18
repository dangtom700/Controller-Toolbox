/**
 * ex41_lpv_gain_scheduling.cpp
 * Linear Parameter-Varying (LPV) gain scheduling via blended LQR designs.
 *
 * Interpolates between two LQR designs tuned for slow and fast response regimes.
 * The scheduling parameter rho in [0,1] linearly blends control outputs:
 *   u = (1-rho)*u_slow + rho*u_fast
 *
 * Uses DiscreteLQR directly (not LQRAdapter) to avoid callback complexity.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;

    // Plant: double integrator (position/velocity)
    Eigen::Matrix2d A;
    A << 1.0, Ts, 0.0, 1.0;
    Eigen::Vector2d B;
    B << 0.5 * Ts * Ts, Ts;
    Eigen::RowVector2d C;
    C << 1.0, 0.0;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
    ctrl::StateSpace sys(A, B, C, D, Ts);

    // LQR "slow" design: standard weighting
    ctrl::LQRParams lp_slow;
    lp_slow.Q = (Eigen::Matrix2d() << 10.0, 0.0, 0.0, 1.0).finished();
    lp_slow.R = Eigen::MatrixXd::Constant(1, 1, 1.0);
    ctrl::DiscreteLQR lqr_slow(sys, lp_slow);

    // LQR "fast" design: aggressive effort
    ctrl::LQRParams lp_fast;
    lp_fast.Q = (Eigen::Matrix2d() << 10.0, 0.0, 0.0, 1.0).finished();
    lp_fast.R = Eigen::MatrixXd::Constant(1, 1, 0.01);
    ctrl::DiscreteLQR lqr_fast(sys, lp_fast);

    const Eigen::MatrixXd &K_slow = lqr_slow.gainMatrix();
    const Eigen::MatrixXd &K_fast = lqr_fast.gainMatrix();
    std::cout << "LQR slow K = [" << K_slow(0,0) << ", " << K_slow(0,1) << "]\n";
    std::cout << "LQR fast K = [" << K_fast(0,0) << ", " << K_fast(0,1) << "]\n";

    // Simulate with rho schedule
    Eigen::Vector2d x = Eigen::Vector2d::Zero();
    const double ref_pos = 1.0;
    const int N = 600;
    double iae = 0.0, final_y = 0.0;

    for (int k = 0; k < N; ++k) {
        // Scheduling: rho=0 for first 3s, ramps to 1 over next 3s
        const double rho = std::min(1.0, std::max(0.0, (k - 300.0) / 300.0));

        // State error vector (position error, velocity target = 0)
        Eigen::Vector2d x_err;
        x_err(0) = ref_pos - x(0);
        x_err(1) = 0.0 - x(1);

        // Blend LQR gains: K_blend = (1-rho)*K_slow + rho*K_fast
        const Eigen::MatrixXd K_blend = (1.0 - rho) * K_slow + rho * K_fast;
        const double u = (K_blend * x_err)(0);

        Eigen::VectorXd uv(1); uv(0) = u;
        ctrl::ssStep(sys, x, uv);

        iae += std::abs(ref_pos - x(0)) * Ts;
        final_y = x(0);
    }

    std::cout << "LPV gain-scheduled final position: " << final_y
              << "  (ref = " << ref_pos << ")\n";
    std::cout << "IAE = " << iae << "\n";

    if (std::abs(final_y - ref_pos) < 0.05)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return 0;
}
