/**
 * ex50_ekf_mpc.cpp
 * OBSERVER + STATE FEEDBACK: EKF (state estimator) + MPC (state-feedback controller).
 *
 * EKF provides state estimates from noisy measurements.
 * MPC uses those estimates as if they were exact states (certainty-equivalence principle).
 * This is the output-feedback MPC / LQG-MPC pattern commonly used in process control.
 *
 * Reference: dynopt-master/EstimatorTypes.ipynb (EKF in MPC loop)
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

int main()
{
    const double Ts = 0.1;

    // Plant: 2-state thermal system
    Eigen::Matrix2d A;
    A << 0.90, 0.05,
         0.08, 0.88;
    Eigen::Vector2d B;
    B << 0.1, 0.05;
    Eigen::RowVector2d C;
    C << 1.0, 0.0;
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
    ctrl::StateSpace plant(A, B, C, D, Ts);

    // MPC: tracks reference on y = C*x
    ctrl::MPCParams mp;
    mp.Np = 10; mp.Nc = 4;
    mp.rho_y = 5.0; mp.rho_u = 0.1;
    ctrl::DiscreteMPC mpc(plant, mp);

    // EKF: linear plant -> EKF = KF
    const int n = 2;
    auto f_ekf = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return Eigen::VectorXd(A * x + B * u(0));
    };
    auto h_ekf = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        return Eigen::VectorXd(C * x);
    };
    auto Fj = [&](const Eigen::VectorXd &, const Eigen::VectorXd &) -> Eigen::MatrixXd {
        return A;
    };
    auto Hj = [&](const Eigen::VectorXd &, const Eigen::VectorXd &) -> Eigen::MatrixXd {
        return C;
    };

    Eigen::Matrix2d Q_ekf = 0.01 * Eigen::Matrix2d::Identity();
    Eigen::MatrixXd R_ekf = Eigen::MatrixXd::Constant(1, 1, 0.05);
    ctrl::ExtendedKalmanFilter ekf(n, 1, f_ekf, h_ekf, Fj, Hj, Q_ekf, R_ekf, Ts);

    std::srand(42);
    auto noisy = [](double sigma) {
        return sigma * (static_cast<double>(std::rand()) / RAND_MAX - 0.5);
    };

    Eigen::Vector2d x_true = Eigen::Vector2d::Zero();
    const double ref = 1.0;
    const int N = 200;
    double final_y = 0.0;

    for (int k = 0; k < N; ++k) {
        // Noisy measurement
        const double y_meas = (C * x_true)(0) + noisy(0.1);
        Eigen::VectorXd ym(1); ym(0) = y_meas;

        // EKF predict + update
        Eigen::VectorXd u_prev(1); u_prev(0) = (k > 0) ? mpc.params().rho_u : 0.0;
        ekf.predict(u_prev);
        ekf.update(ym, u_prev);

        // MPC uses EKF state estimate (certainty equivalence)
        mpc.setState(ekf.state());
        const double u_mpc = mpc.compute(ref - y_meas);

        // True plant step with process noise
        Eigen::Vector2d w; w << noisy(0.05), noisy(0.05);
        Eigen::VectorXd uv(1); uv(0) = u_mpc;
        x_true = A * x_true + B * u_mpc + 0.1 * w;
        final_y = (C * x_true)(0);
    }

    std::cout << "EKF+MPC: final y = " << final_y << "  (ref = " << ref << ")\n";

    if (std::isfinite(final_y) && std::abs(final_y - ref) < 0.15)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";
    return 0;
}
