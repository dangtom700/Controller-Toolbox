/**
 * ex40_ukf_quadrotor.cpp
 * UKF attitude estimation on a simplified nonlinear quadrotor roll model.
 *
 * State: [phi, phi_dot] (roll angle and roll rate)
 * Plant: phi_ddot = -g_damp * phi_dot + K_torque * u + w
 * Measurement: y = phi + v  (IMU roll angle)
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

int main()
{
    const double Ts      = 0.02;   // 50 Hz IMU rate
    const double g_damp  = 2.0;    // roll damping coefficient
    const double K_torq  = 5.0;    // torque/thrust coefficient

    // UKF process model f(x, u):
    // phi[k+1]     = phi[k] + Ts * phi_dot[k]
    // phi_dot[k+1] = (1 - Ts*g_damp) * phi_dot[k] + Ts*K_torq*u[k]
    auto f_ukf = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(2);
        xn(0) = x(0) + Ts * x(1);
        xn(1) = (1.0 - Ts * g_damp) * x(1) + Ts * K_torq * u(0);
        return xn;
    };
    // Measurement model h(x, u): observe phi only
    auto h_ukf = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        return Eigen::VectorXd(x.head(1));
    };

    Eigen::MatrixXd Q_ukf = Eigen::MatrixXd::Zero(2,2);
    Q_ukf(0,0) = 1e-4;   // phi process noise
    Q_ukf(1,1) = 1e-3;   // phi_dot process noise

    Eigen::MatrixXd R_ukf(1,1);
    R_ukf(0,0) = 0.01;   // IMU roll noise (sigma ~ 0.1 rad)

    // Larger alpha for 2-state system to avoid Wc0 numerical issues
    ctrl::UnscentedKalmanFilter ukf(2, 1, f_ukf, h_ukf, Q_ukf, R_ukf, Ts,
                                    Eigen::MatrixXd(),  // default P0 = I
                                    0.1,  // alpha
                                    2.0,  // beta
                                    0.0); // kappa

    std::srand(42);
    auto randn = []() { return 0.1 * (static_cast<double>(std::rand()) / RAND_MAX - 0.5); };

    Eigen::Vector2d x_true = Eigen::Vector2d::Zero();
    double sse_phi = 0.0;
    const int N = 200;

    for (int k = 0; k < N; ++k) {
        // Step input: u = 0.5 for first 2 s, then -0.5
        const double u_k = (k < 100) ? 0.5 : -0.5;
        Eigen::VectorXd uv(1); uv(0) = u_k;

        // True plant step (Euler) with small process noise
        const double w = 0.01 * randn();
        x_true(0) += Ts * x_true(1);
        x_true(1) = (1.0 - Ts * g_damp) * x_true(1) + Ts * K_torq * u_k + w;

        // Noisy measurement
        Eigen::VectorXd y_meas(1);
        y_meas(0) = x_true(0) + 0.1 * randn();

        ukf.predict(uv);
        ukf.update(y_meas, uv);

        const double e = ukf.state()(0) - x_true(0);
        sse_phi += e * e;
    }

    const double rmse_phi = std::sqrt(sse_phi / N);
    std::cout << "UKF roll estimation RMSE = " << rmse_phi << " rad\n";
    std::cout << "Final phi: true=" << x_true(0)
              << "  est=" << ukf.state()(0) << "\n";

    if (rmse_phi < 0.15 && std::isfinite(rmse_phi))
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return 0;
}
