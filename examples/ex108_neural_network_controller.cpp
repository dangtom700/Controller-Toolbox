/**
 * @file ex108_neural_network_controller.cpp
 * @brief Phase 3 (ML1): generic feedforward NeuralNetworkController forward pass.
 *
 * Demonstrates importing fixed weights and driving a plant. A single Linear layer with
 * W = [-1, -2] realises the linear state-feedback law u = -x1 - 2*x2 over the feature
 * vector [x1, x2], regulating a double integrator to the origin - the simplest check that
 * the forward pass + multi-feature computeVec() path is wired correctly. A second network
 * with a Tanh hidden layer is verified against a hand-computed forward pass.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.01;

    // ---- Network 1: linear state-feedback as a single Linear layer --------------------
    ctrl::NNLayerSpec layer;
    layer.W = Eigen::MatrixXd(1, 2);
    layer.W << -1.0, -2.0;
    layer.b = Eigen::VectorXd::Zero(1);
    layer.activation = ctrl::NNLayerSpec::Activation::Linear;

    ctrl::NeuralControllerParams p;
    p.layers = {layer};
    p.n_input_features = 2;
    p.uMin = -100.0;
    p.uMax = 100.0;
    ctrl::NeuralNetworkController nn(p, Ts);

    Eigen::VectorXd x(2);
    x << 1.0, 0.0; // displaced from origin
    for (int k = 0; k < 4000; ++k)
    {
        const double u = nn.computeVec(x)(0);
        x(0) += Ts * x(1);
        x(1) += Ts * u;
    }
    std::printf("Regulated state: x1=%.5f x2=%.5f\n", x(0), x(1));
    const bool regulated = x.allFinite() && std::fabs(x(0)) < 1e-2 && std::fabs(x(1)) < 1e-2;

    // ---- Network 2: 2-layer tanh, verified against a hand-computed forward pass --------
    ctrl::NNLayerSpec h;
    h.W = Eigen::MatrixXd(2, 1);
    h.W << 0.5, -0.5;
    h.b = Eigen::VectorXd::Zero(2);
    h.activation = ctrl::NNLayerSpec::Activation::Tanh;
    ctrl::NNLayerSpec o;
    o.W = Eigen::MatrixXd(1, 2);
    o.W << 1.0, 1.0;
    o.b = Eigen::VectorXd::Zero(1);
    o.activation = ctrl::NNLayerSpec::Activation::Linear;

    ctrl::NeuralControllerParams p2;
    p2.layers = {h, o};
    p2.n_input_features = 1;
    ctrl::NeuralNetworkController nn2(p2, Ts);

    const double in = 0.7;
    const double expected = std::tanh(0.5 * in) + std::tanh(-0.5 * in); // = 0 (odd symmetry)
    const double got = nn2.compute(in);
    std::printf("Tanh net: expected=%.6f got=%.6f\n", expected, got);
    const bool fwd_ok = std::fabs(got - expected) < 1e-9;

    const bool ok = regulated && fwd_ok;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
