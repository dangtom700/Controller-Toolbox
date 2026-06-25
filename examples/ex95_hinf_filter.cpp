/**
 * @file ex95_hinf_filter.cpp
 * @brief Phase 3 (EF1): HinfFilter vs. KalmanFilter under bounded impulsive disturbances.
 *
 * A vibration-sensor-style scenario: a stable scalar plant subject to bounded but
 * non-Gaussian (occasional sharp impulse) process disturbances. KalmanFilter's Gaussian
 * assumption has no guarantee against worst-case impulses; HinfFilter bounds the worst-case
 * estimation-error energy regardless of the disturbance's distribution.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <random>

int main()
{
#if defined(CTRL_HAS_HINF)
    Eigen::MatrixXd A(1, 1); A << 0.9;
    Eigen::MatrixXd B(1, 1); B << 0.0;
    Eigen::MatrixXd C(1, 1); C << 1.0;
    Eigen::MatrixXd D(1, 1); D << 0.0;
    ctrl::StateSpace plant(A, B, C, D, 0.1);

    const Eigen::MatrixXd Qw = Eigen::MatrixXd::Constant(1, 1, 0.01);
    const Eigen::MatrixXd Rv = Eigen::MatrixXd::Constant(1, 1, 0.05);

    const auto hfResult = ctrl::HinfFilter::solve(plant, Qw, Rv);
    if (!hfResult.feasible)
    {
        std::cout << "FAIL: HinfFilter synthesis infeasible\n";
        return 1;
    }
    std::printf("HinfFilter achieved gamma = %.4f\n", hfResult.achievedGamma);

    ctrl::HinfFilter hf(hfResult);
    ctrl::KalmanFilter kf(plant, Qw, Rv);

    std::mt19937 rng(42);
    std::normal_distribution<double> wDist(0.0, std::sqrt(Qw(0, 0)));
    std::normal_distribution<double> vDist(0.0, std::sqrt(Rv(0, 0)));

    double xTrue = 0.0;
    double sseHf = 0.0, sseKf = 0.0;
    const int N = 500;
    for (int k = 0; k < N; ++k)
    {
        // Bounded but non-Gaussian: occasional sharp impulse disturbance, otherwise small
        // Gaussian noise - the scenario where KalmanFilter's Gaussian assumption breaks down.
        double w = wDist(rng);
        if (k % 50 == 0) w += (k % 100 == 0 ? 1.0 : -1.0) * 0.5;
        const double v = vDist(rng);
        const double y = plant.C(0, 0) * xTrue + v;

        hf.predict(Eigen::VectorXd::Constant(1, 0.0));
        hf.update(Eigen::VectorXd::Constant(1, y));
        kf.step(Eigen::VectorXd::Constant(1, y), Eigen::VectorXd::Constant(1, 0.0));

        sseHf += std::pow(xTrue - hf.state()(0), 2);
        sseKf += std::pow(xTrue - kf.state()(0), 2);

        xTrue = plant.A(0, 0) * xTrue + w;
    }

    const double rmsHf = std::sqrt(sseHf / N);
    const double rmsKf = std::sqrt(sseKf / N);
    std::printf("RMS error under impulsive disturbance:  HinfFilter=%.4f  KalmanFilter=%.4f\n",
                rmsHf, rmsKf);

    const bool ok = std::isfinite(rmsHf) && std::isfinite(rmsKf) && rmsHf < 5.0 * rmsKf;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
#else
    std::cout << "SKIP: built with CTRL_ENABLE_HINF=OFF\n";
    return 0;
#endif
}
