#include "NeuralNetworkController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

void NeuralNetworkController::validateLayers(const std::vector<NNLayerSpec> &layers, int n_input)
{
    if (layers.empty())
        throw std::invalid_argument("NeuralNetworkController: layer stack must be non-empty.");
    int prev_out = n_input;
    for (std::size_t i = 0; i < layers.size(); ++i)
    {
        const auto &L = layers[i];
        if (L.W.cols() != prev_out)
            throw std::invalid_argument(
                "NeuralNetworkController: layer " + std::to_string(i) +
                " input width does not match the previous layer's output width.");
        if (L.b.size() != L.W.rows())
            throw std::invalid_argument(
                "NeuralNetworkController: layer " + std::to_string(i) +
                " bias size does not match its output width.");
        prev_out = static_cast<int>(L.W.rows());
    }
    if (prev_out != 1)
        throw std::invalid_argument(
            "NeuralNetworkController: final layer must output exactly one value (the control u).");
}

NeuralNetworkController::NeuralNetworkController(const NeuralControllerParams &params, double Ts)
    : Ts_(Ts), uMin_(params.uMin), uMax_(params.uMax), n_input_(params.n_input_features)
{
    if (Ts_ <= 0.0)
        throw std::invalid_argument("NeuralNetworkController: Ts must be > 0.");
    if (uMin_ >= uMax_)
        throw std::invalid_argument("NeuralNetworkController: uMin must be < uMax.");
    if (n_input_ < 1)
        throw std::invalid_argument("NeuralNetworkController: n_input_features must be >= 1.");
    validateLayers(params.layers, n_input_);
    layers_ = params.layers;
}

Eigen::VectorXd NeuralNetworkController::applyActivation(const Eigen::VectorXd &z,
                                                         NNLayerSpec::Activation a)
{
    switch (a)
    {
    case NNLayerSpec::Activation::Tanh:
        return z.array().tanh().matrix();
    case NNLayerSpec::Activation::ReLU:
        return z.cwiseMax(0.0);
    case NNLayerSpec::Activation::Sigmoid:
        return (1.0 / (1.0 + (-z.array()).exp())).matrix();
    case NNLayerSpec::Activation::Softplus:
        // numerically stable log(1+exp(x)): x>0 -> x + log1p(exp(-x)); else log1p(exp(x))
        return z.unaryExpr([](double x) {
                    return x > 0.0 ? x + std::log1p(std::exp(-x)) : std::log1p(std::exp(x));
                }).matrix();
    case NNLayerSpec::Activation::Linear:
    default:
        return z;
    }
}

double NeuralNetworkController::forward(const Eigen::VectorXd &features) const
{
    Eigen::VectorXd a = features;
    for (const auto &L : layers_)
        a = applyActivation(L.W * a + L.b, L.activation);
    return a(0); // validateLayers guarantees a single output
}

Eigen::VectorXd NeuralNetworkController::hiddenFeatures(const Eigen::VectorXd &features) const
{
    Eigen::VectorXd a = features;
    for (std::size_t i = 0; i + 1 < layers_.size(); ++i)
        a = applyActivation(layers_[i].W * a + layers_[i].b, layers_[i].activation);
    return a; // input to the (linear) output layer
}

Eigen::VectorXd NeuralNetworkController::computeVec(const Eigen::VectorXd &features)
{
    if (features.size() != n_input_)
        throw std::invalid_argument(
            "NeuralNetworkController::computeVec: feature vector size does not match n_input_features.");
    if (!features.allFinite())
        return Eigen::VectorXd::Constant(1, u_prev_); // hold-last NaN contract

    double u = forward(features);
    if (!std::isfinite(u))
        return Eigen::VectorXd::Constant(1, u_prev_);
    u = std::min(std::max(u, uMin_), uMax_);
    u_prev_ = u;
    Eigen::VectorXd out = Eigen::VectorXd::Constant(1, u);
    notifyObserverVec(out, features);
    return out;
}

double NeuralNetworkController::compute(double signal)
{
    if (n_input_ != 1)
        throw std::logic_error(
            "NeuralNetworkController::compute: scalar interface requires n_input_features == 1; "
            "use computeVec() for multi-feature networks.");
    if (!std::isfinite(signal))
        return u_prev_; // hold-last NaN contract

    Eigen::VectorXd feat = Eigen::VectorXd::Constant(1, signal);
    double u = forward(feat);
    if (!std::isfinite(u))
        return u_prev_;
    u = std::min(std::max(u, uMin_), uMax_);
    u_prev_ = u;
    notifyObserver(u, signal);
    return u;
}

void NeuralNetworkController::loadWeights(const std::vector<NNLayerSpec> &layers)
{
    validateLayers(layers, n_input_);
    layers_ = layers;
}

void NeuralNetworkController::reset()
{
    u_prev_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
