/**
 * ex32_sopdt_identification.cpp
 * SOPDT step-response identification and IMC-PID tuning.
 *
 * Generates a synthetic SOPDT step response, identifies parameters using
 * both graphical and optimization methods, prints IMC-PID gains, and
 * validates closed-loop convergence.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <vector>

int main()
{
    // True model: G(s) = 1.5 * exp(-2s) / ((4s+1)(1.5s+1))
    const double K_true    = 1.5;
    const double tau1_true = 4.0;
    const double tau2_true = 1.5;
    const double th_true   = 2.0;
    const double step_mag  = 1.0;
    const double Ts_data   = 0.25;

    const int    N         = 160;  // 40 s of data
    std::vector<double> t(N), y(N);

    for (int i = 0; i < N; ++i) {
        t[i] = i * Ts_data;
        const double dt = t[i] - th_true;
        y[i] = 0.0;
        if (dt > 0.0) {
            y[i] = K_true * step_mag
                 * (1.0 - (tau1_true * std::exp(-dt / tau1_true)
                          - tau2_true * std::exp(-dt / tau2_true))
                          / (tau1_true - tau2_true));
        }
    }

    ctrl::SOPDTIdentifier id(t, y, step_mag, 0.0);

    // Graphical identification
    const ctrl::SOPDTModel mg = id.identify(ctrl::SOPDTMethod::Graphical);
    std::cout << "=== Graphical identification ===\n";
    std::cout << "  K     = " << mg.K     << "  (true: " << K_true    << ")\n";
    std::cout << "  tau1  = " << mg.tau1  << "  (true: " << tau1_true << ")\n";
    std::cout << "  tau2  = " << mg.tau2  << "  (true: " << tau2_true << ")\n";
    std::cout << "  theta = " << mg.theta << "  (true: " << th_true   << ")\n";
    std::cout << "  RMSE  = " << mg.fitRMSE << "\n\n";

    // Optimization identification
    const ctrl::SOPDTModel mo = id.identify(ctrl::SOPDTMethod::Optimization);
    std::cout << "=== Optimization identification ===\n";
    std::cout << "  K     = " << mo.K     << "  (true: " << K_true    << ")\n";
    std::cout << "  tau1  = " << mo.tau1  << "  (true: " << tau1_true << ")\n";
    std::cout << "  tau2  = " << mo.tau2  << "  (true: " << tau2_true << ")\n";
    std::cout << "  theta = " << mo.theta << "  (true: " << th_true   << ")\n";
    std::cout << "  RMSE  = " << mo.fitRMSE << "\n\n";

    // IMC-PID tuning: lambda_c = 2 * theta
    const double lambda_c = 2.0 * mo.theta;
    const double Ts_ctrl  = Ts_data;
    const auto   pp = ctrl::SOPDTIdentifier::imcTuning(mo, lambda_c, Ts_ctrl);

    std::cout << "=== IMC-PID (lambda_c = " << lambda_c << " s) ===\n";
    std::cout << "  Kp = " << pp.Kp << "\n";
    std::cout << "  Ki = " << pp.Ki << "\n";
    std::cout << "  Kd = " << pp.Kd << "\n\n";

    // Closed-loop validation with PI on approximate FOPDT (tauEq = tau1+tau2)
    const double tauEq = mo.tau1 + mo.tau2;
    double y_cl = 0.0, integ = 0.0;
    const double ref = 1.0;
    for (int k = 0; k < 3000; ++k) {
        const double e = ref - y_cl;
        integ += e * Ts_ctrl;
        const double u = pp.Kp * e + pp.Ki * integ;
        const double a = std::exp(-Ts_ctrl / tauEq);
        y_cl = a * y_cl + (1.0 - a) * mo.K * u;
    }

    std::cout << "Closed-loop final output (ref=1.0): " << y_cl << "\n";
    if (std::abs(y_cl - ref) < 0.05)
        std::cout << "PASS\n";
    else
        std::cout << "FAIL (y_cl=" << y_cl << " not within 0.05 of ref)\n";

    return 0;
}
