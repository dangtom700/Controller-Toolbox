#include "NNAdaptiveController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

NNAdaptiveController::NNAdaptiveController(const NNAdaptiveParams &params, double Ts)
    : NeuralNetworkController(params.nn, Ts),
      params_(params),
      a_m_(params.a_m), b_m_(params.b_m),
      gamma_(params.gamma_adapt), sigma_(params.sigma_mod),
      uMin_(params.uMin), uMax_(params.uMax)
{
    if (std::fabs(a_m_) >= 1.0)
        throw std::invalid_argument("NNAdaptiveController: |a_m| must be < 1.");
    if (gamma_ <= 0.0)
        throw std::invalid_argument("NNAdaptiveController: gamma_adapt must be > 0.");
    if (sigma_ < 0.0)
        throw std::invalid_argument("NNAdaptiveController: sigma_mod must be >= 0.");
    if (uMin_ >= uMax_)
        throw std::invalid_argument("NNAdaptiveController: uMin must be < uMax.");
    if (inputFeatures() != 2)
        throw std::invalid_argument(
            "NNAdaptiveController: network input width must be 2 ([y_m - y, r] feature convention).");
    if (layers_.back().activation != NNLayerSpec::Activation::Linear)
        throw std::invalid_argument(
            "NNAdaptiveController: the output layer must use Linear activation (it is the adaptive layer).");

    theta0_ = layers_.back().W;
    b0_     = layers_.back().b;
}

double NNAdaptiveController::compute(double y_plant)
{
    if (!std::isfinite(y_plant))
        return u_prev_; // hold-last NaN contract

    // Fixed feature map phi from the hidden layers: input = [y_m - y, r].
    Eigen::VectorXd in(2);
    in << (y_m_ - y_plant), r_;
    const Eigen::VectorXd phi = hiddenFeatures(in);

    // Control law over the adapting output layer (Linear): u = theta . phi + b.
    Eigen::MatrixXd &theta = layers_.back().W; // (1 x m)
    Eigen::VectorXd &b     = layers_.back().b; // (1)
    double u = theta.row(0).dot(phi) + b(0);
    if (!std::isfinite(u))
        return u_prev_;
    u = std::min(std::max(u, uMin_), uMax_);

    // Lyapunov gradient + sigma-modification adaptation (MRAC convention, e_m = y - y_m).
    const double e_m = y_plant - y_m_;
    const double Ts  = sampleTime();
    theta.row(0) -= Ts * (gamma_ * e_m * phi.transpose() + sigma_ * theta.row(0));
    b(0)         -= Ts * (gamma_ * e_m + sigma_ * b(0));

    // Advance the reference model.
    y_m_ = a_m_ * y_m_ + b_m_ * r_;

    u_prev_ = u;
    notifyObserver(u, y_plant);
    return u;
}

double NNAdaptiveController::outputWeightNorm() const
{
    return layers_.back().W.row(0).norm();
}

void NNAdaptiveController::reset()
{
    layers_.back().W = theta0_;
    layers_.back().b = b0_;
    y_m_    = 0.0;
    r_      = 0.0;
    u_prev_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
