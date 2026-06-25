/**
 * @file ex100_hammerstein_wiener.cpp
 * @brief Phase 3 (SI5): Hammerstein identification of a valve-with-cubic-nonlinearity system.
 *
 * A common industrial pattern: a valve with a static nonlinear characteristic (here a cubic
 * softening term, standing in for deadzone/saturation) followed by linear actuator dynamics.
 * Recovers both the static nonlinearity and the linear dynamics from input/output data alone.
 */

#include "ControllerToolbox.h"
#include <iostream>
#include <random>

int main()
{
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    const int N = 400;
    Eigen::VectorXd u(N), v(N), y(N);
    for (int k = 0; k < N; ++k) u(k) = dist(rng);
    for (int k = 0; k < N; ++k) v(k) = u(k) + 0.3 * u(k) * u(k) * u(k); // valve nonlinearity
    y(0) = 0.0;
    for (int k = 1; k < N; ++k) y(k) = 0.8 * y(k - 1) + 0.5 * v(k - 1); // linear actuator

    ctrl::HammersteinWienerParams params;
    params.na = 1; params.nb = 1; params.nl_degree = 3;
    const auto result = ctrl::HammersteinWienerIdentifier::fitHammerstein(u, y, 0.1, params);

    std::cout << "Recovered static nonlinearity coefficients [c0..c3]: "
              << result.nl_input_coeffs.transpose() << " (true: [0, 1, 0, 0.3])\n";
    std::cout << "Recovered linear part: num=[" << result.linear_part.num[0] << ", "
              << result.linear_part.num[1] << "]  den=[1, " << result.linear_part.den[1]
              << "] (true: num=[0, 0.5], den=[1, -0.8])\n";
    std::printf("Converged=%s after %d iterations\n",
                result.converged ? "yes" : "no", result.iters);

    const bool ok = result.nl_input_coeffs.allFinite()
        && std::fabs(result.nl_input_coeffs(1) - 1.0) < 1e-9
        && std::fabs(result.nl_input_coeffs(3) - 0.3) < 0.05
        && std::fabs(result.linear_part.den[1] - (-0.8)) < 0.05;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
