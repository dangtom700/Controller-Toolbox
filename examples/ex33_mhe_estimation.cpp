/**
 * ex33_mhe_estimation.cpp
 * Moving Horizon Estimation on a 2-state thermal system.
 *
 * Simulates a thermal plant (two coupled temperatures) with process noise
 * and measurement noise. MHE estimates the states from noisy measurements.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

int main()
{
    // 2-state thermal system: T1, T2 (two heaters, coupled)
    //   T1[k+1] = a11*T1 + a12*T2 + b1*u + w1
    //   T2[k+1] = a21*T1 + a22*T2 + b2*u + w2
    //   y       = T1 + noise
    const double Ts = 1.0;  // 1 s sample time

    Eigen::Matrix2d A;
    A << 0.90, 0.05,
         0.08, 0.88;

    Eigen::Vector2d B;
    B << 0.1, 0.05;

    Eigen::RowVector2d C;
    C << 1.0, 0.0;

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);

    ctrl::StateSpace plant(A, B, C, D, Ts);

    // Noise covariances
    Eigen::Matrix2d Q_proc = 0.01 * Eigen::Matrix2d::Identity();
    Eigen::MatrixXd R_meas = Eigen::MatrixXd::Constant(1, 1, 0.1);

    ctrl::MHEParams mp;
    mp.N = 10;

    ctrl::MovingHorizonEstimator mhe(plant, Q_proc, R_meas, mp);
    mhe.initialize(Eigen::Vector2d::Zero(),
                   10.0 * Eigen::Matrix2d::Identity());

    // Reference Kalman filter for comparison
    ctrl::KalmanFilter kf(plant, Q_proc, R_meas);

    std::srand(42);
    auto randn = []() { return 0.1 * (static_cast<double>(std::rand()) / RAND_MAX - 0.5); };

    Eigen::Vector2d x_true = Eigen::Vector2d::Zero();
    const int N_steps = 80;
    double sse_mhe = 0.0, sse_kf = 0.0;

    std::cout << "Step  x1_true  x1_mhe  x1_kf\n";
    std::cout << "------------------------------\n";

    for (int k = 0; k < N_steps; ++k) {
        const double u_k = (k < 40) ? 1.0 : 0.0;  // step input, then off
        Eigen::VectorXd u_vec(1); u_vec(0) = u_k;

        // True plant step with process noise
        Eigen::Vector2d w; w << randn(), randn();
        x_true = A * x_true + B * u_k + 0.1 * w;

        // Noisy measurement
        const double v = randn();
        Eigen::VectorXd y_meas(1);
        y_meas(0) = C * x_true + v;

        // MHE estimate
        const Eigen::VectorXd x_mhe = mhe.estimate(y_meas, u_vec);

        // Kalman filter estimate
        kf.step(y_meas, u_vec);
        const Eigen::VectorXd x_kf = kf.state();

        const double e_mhe = x_mhe(0) - x_true(0);
        const double e_kf  = x_kf(0)  - x_true(0);
        sse_mhe += e_mhe * e_mhe;
        sse_kf  += e_kf  * e_kf;

        if (k % 10 == 0) {
            std::cout << k
                      << "  " << x_true(0)
                      << "  " << x_mhe(0)
                      << "  " << x_kf(0) << "\n";
        }
    }

    const double rmse_mhe = std::sqrt(sse_mhe / N_steps);
    const double rmse_kf  = std::sqrt(sse_kf  / N_steps);
    std::cout << "\nRMSE MHE = " << rmse_mhe << "\n";
    std::cout << "RMSE KF  = " << rmse_kf  << "\n";
    std::cout << "MHE converged last step: " << (mhe.lastConverged() ? "yes" : "no") << "\n";

    // Both estimators should have finite, bounded error
    if (std::isfinite(rmse_mhe) && rmse_mhe < 5.0 && mhe.lastConverged())
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return 0;
}
