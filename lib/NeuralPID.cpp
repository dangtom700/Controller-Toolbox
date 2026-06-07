#include "NeuralPID.h"
#include <cmath>
#include <random>
#include <stdexcept>

namespace ctrl {

NeuralPID::NeuralPID(const Params& p) : p_(p)
{
    if (p_.n_hidden <= 0)
        throw std::invalid_argument("NeuralPID: n_hidden must be positive");
    if (p_.plant_gain == 0.0)
        throw std::invalid_argument("NeuralPID: plant_gain must be non-zero");

    // He initialisation: W ~ N(0, 2/fan_in)
    std::mt19937 rng(42);
    std::normal_distribution<double> nd(0.0, 1.0);

    W1_.resize(p_.n_hidden, 3);
    b1_.setZero(p_.n_hidden);
    W2_.resize(3, p_.n_hidden);
    b2_.setZero(3);

    const double scale1 = std::sqrt(2.0 / 3.0);
    const double scale2 = std::sqrt(2.0 / p_.n_hidden);

    for (int i = 0; i < p_.n_hidden; ++i) {
        for (int j = 0; j < 3; ++j)
            W1_(i, j) = scale1 * nd(rng);
        for (int j = 0; j < 3; ++j)
            W2_(j, i) = scale2 * nd(rng);
    }

    // Bias initialisation: set b2 so initial gains approx = [Kp0, Ki0, Kd0]
    // softplus^{-1}(y) = log(exp(y) - 1); for y > 20, softplus(y) approx = y so use y directly
    // to avoid exp() overflow (e.g. Kp0=2000 would overflow to inf otherwise)
    auto sp_inv = [](double y) {
        return (y > 20.0) ? y : std::log(std::expm1(y));
    };
    b2_(0) = sp_inv(p_.Kp0);
    b2_(1) = sp_inv(p_.Ki0 + 1e-6);
    b2_(2) = sp_inv(p_.Kd0 + 1e-6);

    Kp_ = p_.Kp0;  Ki_ = p_.Ki0;  Kd_ = p_.Kd0;
}

// ---------------------------------------------------------------------------
// Forward pass: z -> gains [Kp, Ki, Kd], stores hidden activations
// ---------------------------------------------------------------------------

Eigen::Vector3d NeuralPID::forward(const Eigen::Vector3d& z,
                                     Eigen::VectorXd& h_out) const
{
    // Hidden layer: h = tanh(W1*z + b1)
    h_out = (W1_ * z + b1_).array().tanh();

    // Output layer: gains = softplus(W2*h + b2)
    Eigen::Vector3d pre = W2_ * h_out + b2_;
    return pre.unaryExpr([](double x) { return std::log1p(std::exp(x)); });
}

// ---------------------------------------------------------------------------
// compute()
// ---------------------------------------------------------------------------

double NeuralPID::compute(double error)
{
    if (!std::isfinite(error)) return u_prev_;
    const double e     = error;
    const double e_dot = (e - e_prev_) / p_.Ts;
    e_int_ += e * p_.Ts;

    Eigen::Vector3d z(e, e_dot, e_int_);

    // Forward pass
    Eigen::VectorXd h;
    Eigen::Vector3d gains = forward(z, h);
    Kp_ = gains(0);
    Ki_ = gains(1);
    Kd_ = gains(2);

    // PID output
    double u = std::clamp(Kp_ * e + Ki_ * e_int_ + Kd_ * e_dot,
                          p_.uMin, p_.uMax);

    // Online gradient update
    // Loss J = e^2,  dJ/du = 2e * de/du approx = 2e * (-plant_gain)  (negative feedback)
    // dJ/dgains: du/dKp = e, du/dKi = e_int, du/dKd = e_dot
    const double dJ_du  = 2.0 * e * (-p_.plant_gain);
    Eigen::Vector3d du_dgains(e, e_int_, e_dot);
    Eigen::Vector3d dJ_dgains = dJ_du * du_dgains;

    // dgains_i/dpre_i = softplus'(pre_i)
    Eigen::Vector3d pre = W2_ * h + b2_;
    Eigen::Vector3d sp_d = pre.unaryExpr([](double x) { return 1.0 / (1.0 + std::exp(-x)); });

    // dJ/dpre = dJ_dgains .* sp_d
    Eigen::Vector3d dJ_dpre = dJ_dgains.array() * sp_d.array();

    // Gradients for W2, b2
    const Eigen::MatrixXd dW2 = dJ_dpre * h.transpose();
    const Eigen::VectorXd db2 = dJ_dpre;

    // Backprop through hidden: dJ/dh = W2' * dJ_dpre,  dJ/dpre1 = dJ/dh .* (1-h^2)
    Eigen::VectorXd dJ_dh    = W2_.transpose() * dJ_dpre;
    Eigen::VectorXd dJ_dpre1 = dJ_dh.array() * (1.0 - h.array().square());

    const Eigen::MatrixXd dW1 = dJ_dpre1 * z.transpose();
    const Eigen::VectorXd db1 = dJ_dpre1;

    // Gradient step
    W2_ -= p_.lr * dW2;
    b2_ -= p_.lr * db2;
    W1_ -= p_.lr * dW1;
    b1_ -= p_.lr * db1;

    clipWeights();

    e_prev_ = e;
    u_prev_ = u;
    notifyObserver(u, e);
    return u;
}

void NeuralPID::clipWeights()
{
    const double mn = p_.max_weight_norm;
    auto clip = [&](Eigen::MatrixXd& W) {
        double n = W.norm();
        if (n > mn) W *= mn / n;
    };
    clip(W1_); clip(W2_);
}

void NeuralPID::reset()
{
    e_prev_ = 0.0;
    e_int_  = 0.0;
    u_prev_ = 0.0;
    // Do NOT reset network weights - learning should persist across resets
}

} // namespace ctrl
