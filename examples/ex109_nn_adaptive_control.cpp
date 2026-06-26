/**
 * @file ex109_nn_adaptive_control.cpp
 * @brief Phase 3 (ML2): NN-adaptive control with online output-weight adaptation.
 *
 * A first-order plant with an unknown static input nonlinearity is regulated to a first-order
 * reference model. The hidden tanh layer is a fixed nonlinear basis; only the output-layer
 * weights adapt online via the Lyapunov + sigma-modification law. We check that the plant
 * output tracks the reference model and that the adapting weights stay bounded.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;

    // Fixed hidden layer (6 tanh units over [y_m - y, r]) + adaptive linear output (init 0).
    ctrl::NNLayerSpec hidden;
    hidden.W = Eigen::MatrixXd(6, 2);
    hidden.W << 1.0, 0.5, -0.8, 0.3, 0.6, -0.4, -0.5, 0.7, 0.9, -0.2, 0.2, 0.8;
    hidden.b = Eigen::VectorXd::Zero(6);
    hidden.activation = ctrl::NNLayerSpec::Activation::Tanh;

    ctrl::NNLayerSpec out;
    out.W = Eigen::MatrixXd::Zero(1, 6);
    out.b = Eigen::VectorXd::Zero(1);
    out.activation = ctrl::NNLayerSpec::Activation::Linear;

    ctrl::NNAdaptiveParams p;
    p.nn.layers = {hidden, out};
    p.nn.n_input_features = 2;
    p.gamma_adapt = 3.0;
    p.sigma_mod = 0.01;
    p.a_m = 0.6;
    p.b_m = 0.4; // unity DC gain
    p.uMin = -50.0;
    p.uMax = 50.0;
    ctrl::NNAdaptiveController c(p, Ts);

    const double r = 1.0;
    double y = 0.0;
    double y_m = 0.0;
    double max_err_late = 0.0;
    for (int k = 0; k < 12000; ++k)
    {
        c.setReference(r);
        const double u = c.compute(y);
        // Plant: y[k+1] = 0.9 y + 0.1 (u + 0.3 sin(y))  (unknown input nonlinearity)
        y = 0.9 * y + 0.1 * (u + 0.3 * std::sin(y));
        y_m = p.a_m * y_m + p.b_m * r;
        if (k > 10000)
            max_err_late = std::max(max_err_late, std::fabs(y - y_m));
    }

    const double wnorm = c.outputWeightNorm();
    std::printf("Final y=%.4f y_m=%.4f late |y-y_m|max=%.4f weightNorm=%.3f\n",
                y, y_m, max_err_late, wnorm);

    const bool ok = std::isfinite(y) && std::isfinite(wnorm) &&
                    max_err_late < 0.15 && wnorm < 1e3;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
