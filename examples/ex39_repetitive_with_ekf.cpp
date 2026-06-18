/**
 * ex39_repetitive_with_ekf.cpp
 * RepetitiveController + EKF for periodic disturbance rejection.
 *
 * A sinusoidal disturbance with a known period is rejected by the
 * repetitive controller. EKF estimates the plant state concurrently.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <numbers>

int main()
{
    const double Ts = 0.01;

    // Plant: G(s) = 1/(s+1), ZOH discretised
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1,1,-1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Constant(1,1, 1.0),
        Eigen::MatrixXd::Zero(1,1), 0.0);
    const ctrl::StateSpace sys = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    // Feedback PI controller (inner)
    ctrl::PIDParams pp;
    pp.Kp = 2.0;
    pp.Ki = 1.0;
    pp.Kd = 0.0;
    pp.uMin = -10.0;
    pp.uMax =  10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    // Repetitive controller: period = 1.0 s = 100 steps
    const int period_steps = 100;
    ctrl::RepetitiveParams rp;
    rp.periodSteps = period_steps;
    rp.Krc         = 0.8;    // learning gain
    rp.Q           = 0.95;   // forgetting/stability factor
    ctrl::RepetitiveController rc(pid, rp, Ts);

    // EKF for state estimation: construct with (n, p, f, h, Fjac, Hjac, Q, R, Ts)
    const int n_ekf = 1, p_ekf = 1;

    auto f_ekf = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
        return Eigen::VectorXd(sys.A * x + sys.B * u);
    };
    auto h_ekf = [&](const Eigen::VectorXd &x, const Eigen::VectorXd &) {
        return Eigen::VectorXd(sys.C * x);
    };
    auto Fj = [&](const Eigen::VectorXd &, const Eigen::VectorXd &) -> Eigen::MatrixXd {
        return sys.A;
    };
    auto Hj = [&](const Eigen::VectorXd &, const Eigen::VectorXd &) -> Eigen::MatrixXd {
        return sys.C;
    };

    Eigen::MatrixXd Q_ekf = 1e-4 * Eigen::MatrixXd::Identity(1,1);
    Eigen::MatrixXd R_ekf = 0.01 * Eigen::MatrixXd::Identity(1,1);

    ctrl::ExtendedKalmanFilter ekf(n_ekf, p_ekf,
                                   f_ekf, h_ekf, Fj, Hj,
                                   Q_ekf, R_ekf, Ts);

    // Simulate: step ref=1 + sinusoidal disturbance at period=1 s
    Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
    double y = 0.0;
    const double ref = 1.0;
    const double distAmp = 0.3;
    const double distFreq = 2.0 * std::numbers::pi;  // 1 Hz = 2pi rad/s

    const int N = 400;  // 4 s
    double iae_first = 0.0, iae_last = 0.0;

    for (int k = 0; k < N; ++k) {
        const double d  = distAmp * std::sin(distFreq * k * Ts);
        const double e  = ref - (y + d);
        const double u  = rc.compute(e);

        Eigen::VectorXd uv(1); uv(0) = u;
        const Eigen::VectorXd yv = ctrl::ssStep(sys, x, uv);
        y = yv(0);

        // EKF state update (predict then update with current measurement)
        ekf.predict(uv);
        Eigen::VectorXd ymeas(1); ymeas(0) = y + d;
        ekf.update(ymeas, uv);

        const double err = std::abs(e);
        if (k < 100) iae_first += err * Ts;
        if (k >= 300) iae_last  += err * Ts;
    }

    std::cout << "IAE (first 1s):  " << iae_first << "\n";
    std::cout << "IAE (last 1s):   " << iae_last  << "\n";
    std::cout << "EKF state after sim: " << ekf.state()(0) << "\n";

    // Repetitive controller should significantly reduce IAE by the 4th second
    if (iae_last < iae_first && std::isfinite(iae_last))
        std::cout << "PASS\n";
    else
        std::cout << "FAIL\n";

    return 0;
}
