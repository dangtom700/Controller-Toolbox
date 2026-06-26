/**
 * @file ex110_nonlinear_imc.cpp
 * @brief Phase 3 (NC3): Nonlinear Internal Model Control.
 *
 * A first-order process is regulated by NonlinearIMC using a one-step model and its inverse.
 * Two scenarios: (1) the model matches the plant exactly -> offset-free tracking (IMC's
 * defining property); (2) the plant gain is perturbed away from the model -> the mismatch
 * feedback path still drives the steady-state offset to zero.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

// Model: y_hat = 0.7*x + 0.3*u ; inverse: u = (y_target - 0.7*x)/0.3
static double model_fn(const Eigen::VectorXd &x, double u) { return 0.7 * x(0) + 0.3 * u; }
static double inverse_fn(const Eigen::VectorXd &x, double y_t) { return (y_t - 0.7 * x(0)) / 0.3; }

static double run(double plant_a, double plant_b)
{
    const double Ts = 0.1;
    ctrl::NonlinearIMCParams p;
    p.filter_lambda = 0.5;
    p.uMin = -100.0;
    p.uMax = 100.0;
    ctrl::NonlinearIMC imc(model_fn, inverse_fn, p, Ts);

    const double r = 1.0;
    double y = 0.0;
    Eigen::VectorXd x(1);
    for (int k = 0; k < 500; ++k)
    {
        x(0) = y;
        imc.setState(x);
        const double u = imc.compute(r - y);
        y = plant_a * y + plant_b * u;
    }
    return y;
}

int main()
{
    const double y_exact = run(0.7, 0.3);    // model == plant
    const double y_mis   = run(0.75, 0.28);  // plant differs from the model
    std::printf("Exact-match y=%.5f  Mismatch y=%.5f (ref=1.0)\n", y_exact, y_mis);

    const bool ok = std::isfinite(y_exact) && std::isfinite(y_mis) &&
                    std::fabs(y_exact - 1.0) < 1e-3 && std::fabs(y_mis - 1.0) < 1e-2;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
