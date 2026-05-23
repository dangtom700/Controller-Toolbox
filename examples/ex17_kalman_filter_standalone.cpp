// ============================================================
//  ex17_kalman_filter_standalone.cpp
//  Demonstrates Kalman Filter state estimation separately from LQG.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <random>

int main()
{
    const double Ts = 0.01;

    // Plant: 2nd order
    Eigen::Matrix2d A; A << 0.99, 0.01, -0.02, 0.98;
    Eigen::Vector2d B; B << 0.0, 0.01;
    Eigen::RowVector2d C; C << 1.0, 0.0; // Only position measured
    Eigen::MatrixXd D(1, 1); D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, Ts);

    // Noise parameters
    Eigen::Matrix2d Qn = 0.005 * Eigen::Matrix2d::Identity();
    Eigen::MatrixXd Rn(1, 1); Rn << 0.2; // High measurement noise

    ctrl::KalmanFilter kf(plant, Qn, Rn, Eigen::Matrix2d::Identity());

    std::mt19937 rng(1337);
    std::normal_distribution<double> v_dist(0.0, std::sqrt(Rn(0,0)));

    std::cout << "=== Standalone Kalman Filter ===\n";
    std::cout << "  k    t[s]  True_y   Noisy_y  Est_x0   True_v   Est_v(x1)\n";
    std::cout << std::string(66, '-') << "\n";

    Eigen::Vector2d x_true = Eigen::Vector2d::Zero();
    Eigen::VectorXd u(1); u << 1.0; // Constant input

    for (int k = 0; k <= 200; ++k) {
        // True system evolution
        x_true = A * x_true + B * u(0);
        double y_true = (C * x_true)(0);
        
        // Measurement
        double y_meas = y_true + v_dist(rng);

        // Kalman Filter
        kf.predict(u);
        Eigen::VectorXd yv(1); yv << y_meas;
        kf.update(yv, u);

        Eigen::Vector2d x_est = kf.state();

        if (k % 20 == 0)
            std::cout << std::setw(4) << k
                      << std::fixed << std::setprecision(3)
                      << std::setw(8) << k * Ts
                      << std::setw(8) << y_true
                      << std::setw(10) << y_meas
                      << std::setw(9) << x_est(0)
                      << std::setw(9) << x_true(1)
                      << std::setw(10) << x_est(1) << "\n";
    }

    return 0;
}
