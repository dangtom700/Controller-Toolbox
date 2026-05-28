// ============================================================
//  ex30_ekf_nonlinear.cpp
//  Extended Kalman Filter on a nonlinear pendulum.
//  Demonstrates numerical Jacobians and nonlinear state estimation.
// ============================================================
#include "ControllerToolbox.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <numbers>
#include <random>

#if !defined(CTRL_HAS_ADVANCED_KALMAN)
int main() { std::puts("Skipped: CTRL_HAS_ADVANCED_KALMAN not enabled."); return 0; }
#else

static constexpr double g   = 9.81;   // [m/s^2]
static constexpr double L   = 1.0;    // [m]
static constexpr double b   = 0.1;    // [N.m.s/rad] viscous damping
static constexpr double Ts  = 0.02;   // [s] 50 Hz

// True nonlinear pendulum dynamics (Euler integration)
static Eigen::VectorXd f_pend(const Eigen::VectorXd& x, const Eigen::VectorXd& u)
{
    const double th  = x(0), dth = x(1), tau = u(0);
    const double ddth = -(g/L)*std::sin(th) - b*dth + tau;
    return Eigen::Vector2d{th + Ts*dth, dth + Ts*ddth};
}

// Measurement: observe only angle theta
static Eigen::VectorXd h_pend(const Eigen::VectorXd& x, const Eigen::VectorXd&)
{
    return Eigen::VectorXd::Constant(1, x(0));
}

// Analytical Jacobian df/dx (2x2)
static Eigen::MatrixXd Fj_pend(const Eigen::VectorXd& x, const Eigen::VectorXd&)
{
    Eigen::Matrix2d J;
    J << 1.0, Ts,
         -(g/L)*std::cos(x(0))*Ts, 1.0 - b*Ts;
    return J;
}

// Jacobian dh/dx (1x2)
static Eigen::MatrixXd Hj_pend(const Eigen::VectorXd& x, const Eigen::VectorXd&)
{
    Eigen::MatrixXd H(1, 2);
    H << 1.0, 0.0;
    return H;
}

int main()
{
    std::mt19937 rng(42);
    const double sigma_meas = 5.0 * std::numbers::pi / 180.0; // 5 deg measurement noise
    std::normal_distribution<double> noise(0.0, sigma_meas);

    // EKF construction: n=2 states, p=1 measurement
    Eigen::Matrix2d Q_kf = (Eigen::Matrix2d() << 1e-5, 0, 0, 1e-4).finished();
    Eigen::MatrixXd R_kf(1, 1); R_kf << sigma_meas * sigma_meas;

    ctrl::ExtendedKalmanFilter ekf(2, 1, f_pend, h_pend, Fj_pend, Hj_pend, Q_kf, R_kf, Ts);

    // Simple PD control on EKF-estimated state
    const double Kp_pd = 10.0, Kd_pd = 2.0;

    Eigen::Vector2d x_true{0.5, 0.0}; // 0.5 rad initial displacement
    Eigen::VectorXd x_est0(2); x_est0 << 0.4, 0.0;
    ekf.setState(x_est0);

    Eigen::VectorXd u_prev(1); u_prev << 0.0;

    std::cout << "=== EKF on Nonlinear Pendulum ===\n"
              << "  Initial: theta=0.5 rad, EKF init: 0.4 rad, noise sigma=5 deg\n\n"
              << std::setw(7) << "t[s]"
              << std::setw(12) << "th_true[deg]"
              << std::setw(12) << "th_meas[deg]"
              << std::setw(12) << "th_ekf[deg]"
              << std::setw(12) << "dth_ekf\n"
              << std::string(60, '-') << "\n";

    double rmse_raw = 0.0, rmse_ekf = 0.0;
    const int N = static_cast<int>(5.0 / Ts);

    for (int k = 0; k < N; ++k) {
        const double t  = k * Ts;
        const double y_noisy = x_true(0) + noise(rng);

        // EKF: predict + update
        ekf.step(Eigen::VectorXd::Constant(1, y_noisy), u_prev);
        const Eigen::VectorXd x_hat = ekf.state();

        // PD control on estimated state
        Eigen::VectorXd u_ctrl(1);
        u_ctrl(0) = std::clamp(Kp_pd * (0.0 - x_hat(0)) - Kd_pd * x_hat(1), -10.0, 10.0);

        // True plant step
        x_true = f_pend(x_true, u_ctrl);
        u_prev = u_ctrl;

        rmse_raw += (y_noisy - x_true(0)) * (y_noisy - x_true(0));
        rmse_ekf += (x_hat(0) - x_true(0)) * (x_hat(0) - x_true(0));

        if (k % static_cast<int>(0.5 / Ts) == 0)
            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(7) << t
                      << std::setw(12) << x_true(0)  * 180.0 / std::numbers::pi
                      << std::setw(12) << y_noisy     * 180.0 / std::numbers::pi
                      << std::setw(12) << x_hat(0)   * 180.0 / std::numbers::pi
                      << std::setw(12) << std::setprecision(3) << x_hat(1) << "\n";
    }

    rmse_raw = std::sqrt(rmse_raw / N) * 180.0 / std::numbers::pi;
    rmse_ekf = std::sqrt(rmse_ekf / N) * 180.0 / std::numbers::pi;

    std::cout << "\nRMSE (raw meas): " << std::fixed << std::setprecision(3) << rmse_raw << " deg\n"
              << "RMSE (EKF est):  " << rmse_ekf << " deg  (noise reduction: "
              << std::setprecision(1) << (1.0 - rmse_ekf/rmse_raw)*100.0 << "%)\n";
    return 0;
}
#endif
