/**
 * ex92_correlation_id.cpp
 * Phase 3 (SI2): cross-correlation impulse-response identification.
 *
 * Drives a known first-order plant with a PRBS test signal, estimates its impulse response
 * via CorrelationID::identify(), and compares against the plant's true impulse response
 * (obtained directly by stepping a unit impulse through the same StateSpace).
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;
    ctrl::TransferFunction tf_true({0.0, 0.2}, {1.0, -0.8}, Ts);
    ctrl::StateSpace plant = ctrl::tf2ss(tf_true);

    const int N = 4000;
    Eigen::VectorXd u = ctrl::CorrelationID::generatePRBS(N, 10, 7);

    // Simulate the plant under PRBS excitation.
    Eigen::VectorXd y(N);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.A.rows());
    for (int k = 0; k < N; ++k)
    {
        Eigen::VectorXd uv(1);
        uv << u(k);
        y(k) = ctrl::ssStep(plant, x, uv)(0);
    }

    ctrl::CorrelationIDParams params;
    params.max_lag = 15;
    const auto result = ctrl::CorrelationID::identify(u, y, Ts, params);

    // True impulse response: step a unit impulse through the same (fresh) plant.
    Eigen::VectorXd g_true(params.max_lag + 1);
    Eigen::VectorXd x_imp = Eigen::VectorXd::Zero(plant.A.rows());
    for (int k = 0; k <= params.max_lag; ++k)
    {
        Eigen::VectorXd uv(1);
        uv << (k == 0 ? 1.0 : 0.0);
        g_true(k) = ctrl::ssStep(plant, x_imp, uv)(0);
    }

    std::cout << "lag   g_hat        g_true\n";
    double maxErr = 0.0;
    for (int k = 0; k <= params.max_lag; ++k)
    {
        std::cout << "  " << k << "   " << result.impulse_response(k)
                   << "   " << g_true(k) << "\n";
        maxErr = std::max(maxErr, std::fabs(result.impulse_response(k) - g_true(k)));
    }
    std::cout << "Max abs error vs. true impulse response: " << maxErr << "\n";

    const bool ok = std::isfinite(maxErr) && maxErr < 0.05;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
