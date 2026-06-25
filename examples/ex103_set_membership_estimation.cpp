/**
 * @file ex103_set_membership_estimation.cpp
 * @brief Phase 3 Roadmap Phase 2 (EF2): bounded-error ellipsoidal state estimation.
 *
 * A sensor with a hard calibration spec (bounded error, not Gaussian) - a guaranteed feasible
 * ellipsoid is more meaningful to a safety case than a Kalman filter's probabilistic confidence
 * interval, and never excludes the true state under non-Gaussian (uniform-bounded) noise.
 */

#include "ControllerToolbox.h"
#include <iostream>
#include <random>

int main()
{
    const ctrl::StateSpace plant(Eigen::MatrixXd::Constant(1, 1, 0.9),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Constant(1, 1, 1.0),
                                  Eigen::MatrixXd::Zero(1, 1), 0.1);

    ctrl::SetMembershipParams smp;
    smp.w_bound = 0.05;
    smp.v_bound = 0.3;
    ctrl::SetMembershipEstimator est(plant, smp, Eigen::VectorXd::Zero(1),
                                      Eigen::MatrixXd::Identity(1, 1));
    ctrl::KalmanFilter kf(plant, Eigen::MatrixXd::Constant(1, 1, smp.w_bound * smp.w_bound),
                          Eigen::MatrixXd::Constant(1, 1, smp.v_bound * smp.v_bound / 9.0));

    std::mt19937 rng(9);
    std::uniform_real_distribution<double> wDist(-smp.w_bound, smp.w_bound);
    std::uniform_real_distribution<double> vDist(-smp.v_bound, smp.v_bound);

    double xTrue = 0.0;
    const Eigen::VectorXd u = Eigen::VectorXd::Constant(1, 0.2);
    int smContains = 0, kfContains = 0;
    const int N = 200;

    for (int k = 0; k < N; ++k)
    {
        xTrue = 0.9 * xTrue + u(0) + wDist(rng);
        est.predict(u);
        kf.predict(u);
        Eigen::VectorXd y(1); y(0) = xTrue + vDist(rng);
        est.update(y);
        kf.update(y, u);

        const Eigen::VectorXd d = Eigen::VectorXd::Constant(1, xTrue) - est.centerEstimate();
        const double quad = d.dot(est.ellipsoidShape().inverse() * d);
        if (quad <= 1.0 + 1e-6) ++smContains;

        const double kfSigma = std::sqrt(kf.covariance()(0, 0));
        if (std::fabs(xTrue - kf.state()(0)) <= 3.0 * kfSigma) ++kfContains;
    }

    std::cout << "SetMembershipEstimator: ellipsoid contained the true state in "
              << smContains << "/" << N << " steps\n";
    std::cout << "KalmanFilter: 3-sigma interval contained the true state in "
              << kfContains << "/" << N << " steps\n";

    const bool ok = (smContains == N);
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
