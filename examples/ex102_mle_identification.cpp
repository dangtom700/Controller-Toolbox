/**
 * @file ex102_mle_identification.cpp
 * @brief Phase 3 Roadmap Phase 2 (SI1): MLE/MAP identification on outlier-heavy data.
 *
 * A quantizing sensor occasionally produces a large outlier reading - non-Gaussian measurement
 * noise that biases a plain least-squares (Gaussian-MLE) fit. MLEIdentifier with a Laplace
 * noise model is far less sensitive to the outliers, recovering the true plant more accurately.
 */

#include "ControllerToolbox.h"
#include <iostream>
#include <random>

int main()
{
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> uDist(-1.0, 1.0);
    std::uniform_real_distribution<double> noiseDist(-0.01, 0.01);
    std::uniform_real_distribution<double> outlierDist(0.0, 1.0);

    const int N = 300;
    const double trueA1 = -0.6, trueB1 = 0.4; // y[k] = 0.6*y[k-1] + 0.4*u[k-1] + noise
    Eigen::VectorXd u(N), y = Eigen::VectorXd::Zero(N);
    for (int k = 0; k < N; ++k) u(k) = uDist(rng);
    for (int k = 1; k < N; ++k)
    {
        double noise = noiseDist(rng);
        if (outlierDist(rng) < 0.05) noise += (outlierDist(rng) < 0.5 ? -5.0 : 5.0); // quantizer glitch
        y(k) = -trueA1 * y(k - 1) + trueB1 * u(k - 1) + noise;
    }

    ctrl::MLEParams gaussParams; gaussParams.na = 1; gaussParams.nb = 1;
    gaussParams.noise = ctrl::NoiseModel::Gaussian;
    const auto gaussResult = ctrl::MLEIdentifier::fit(u, y, 0.1, gaussParams);

    ctrl::MLEParams laplaceParams = gaussParams; laplaceParams.noise = ctrl::NoiseModel::Laplace;
    const auto laplaceResult = ctrl::MLEIdentifier::fit(u, y, 0.1, laplaceParams);

    std::cout << "True theta:    [" << trueA1 << ", " << trueB1 << "]\n";
    std::cout << "Gaussian MLE:  " << gaussResult.theta.transpose() << "\n";
    std::cout << "Laplace MLE:   " << laplaceResult.theta.transpose() << "\n";

    const Eigen::Vector2d trueTheta(trueA1, trueB1);
    const double gaussErr = (gaussResult.theta - trueTheta).norm();
    const double laplaceErr = (laplaceResult.theta - trueTheta).norm();
    std::cout << "Gaussian error: " << gaussErr << "  Laplace error: " << laplaceErr << "\n";

    const bool ok = laplaceErr < gaussErr;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
