/**
 * ex51_ukf_smc.cpp
 * OBSERVER + STATE FEEDBACK: UKF (state estimator) + SMC (sliding-mode feedback).
 *
 * UKF filters noisy position/velocity measurements from a nonlinear pendulum.
 * SMC uses the UKF state estimate to compute a robust sliding-mode control input.
 *
 * UKF strength: handles non-Gaussian noise and nonlinear dynamics without Jacobians.
 * SMC strength: robust to matched disturbances and model uncertainty.
 *
 * SMC sliding surface: s = e_dot + lambda*e  (uses UKF-estimated velocity)
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

int main()
{
    const double Ts   = 0.02;
    const double g    = 9.81;
    const double L    = 1.0;    // pendulum length [m]
    const double b    = 0.5;    // damping coefficient

    // UKF process model: nonlinear pendulum
    // x = [theta, theta_dot], u = torque
    auto f_ukf = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        Eigen::VectorXd xn(2);
        xn(0) = x(0) + Ts * x(1);
        xn(1) = x(1) + Ts * (-g/L * std::sin(x(0)) - b * x(1) + u(0));
        return xn;
    };
    // Measurement: angle only
    auto h_ukf = [](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        return Eigen::VectorXd(x.head(1));
    };

    Eigen::Matrix2d Q_ukf;
    Q_ukf << 1e-4, 0.0,
             0.0, 1e-3;
    Eigen::MatrixXd R_ukf(1,1); R_ukf(0,0) = 0.01;

    ctrl::UnscentedKalmanFilter ukf(2, 1, f_ukf, h_ukf, Q_ukf, R_ukf, Ts,
                                    Eigen::MatrixXd(), 0.1, 2.0, 0.0);

    // SMC: stabilise pendulum at theta=0 using UKF state
    // Sliding surface: s = theta_dot + 5*theta
    const double lambda_smc = 5.0;
    const double eta_smc    = 3.0;
    const double phi_smc    = 0.2;

    std::srand(42);
    auto randn = []() { return 0.1 * (static_cast<double>(std::rand()) / RAND_MAX - 0.5); };

    Eigen::Vector2d x_true;
    x_true << 0.5, 0.0;  // initial angle 0.5 rad

    const int N = 400;
    double sse_theta = 0.0;

    for (int k = 0; k < N; ++k) {
        // Noisy measurement
        Eigen::VectorXd ym(1); ym(0) = x_true(0) + 0.1 * randn();
        Eigen::VectorXd u_prev(1); u_prev(0) = 0.0;

        ukf.predict(u_prev);
        ukf.update(ym, u_prev);

        const Eigen::VectorXd &x_est = ukf.state();

        // SMC: s = theta_dot_est + lambda * theta_est
        const double s   = x_est(1) + lambda_smc * x_est(0);
        // Equivalent control + switching control
        const double phi = phi_smc;
        const double u_smc = -(eta_smc + std::abs(g/L * std::sin(x_est(0)))) * (std::abs(s) < phi ? s/phi : (s > 0 ? 1.0 : -1.0));

        Eigen::VectorXd uv(1); uv(0) = u_smc;

        // True plant step (Euler) with noise
        const double ddth = -g/L * std::sin(x_true(0)) - b * x_true(1) + u_smc + 0.1 * randn();
        x_true(0) += Ts * x_true(1);
        x_true(1) += Ts * ddth;

        sse_theta += x_true(0) * x_true(0);
    }

    const double rmse_theta = std::sqrt(sse_theta / N);
    std::cout << "UKF+SMC pendulum: RMSE theta = " << rmse_theta << " rad\n";
    std::cout << "Final theta = " << x_true(0) << "  theta_dot = " << x_true(1) << "\n";

    if (rmse_theta < 0.3 && std::isfinite(rmse_theta))
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
