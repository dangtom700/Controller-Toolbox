#include "SimPlant.h"

namespace ctrl {

SimPlant::SimPlant(const StateSpace& model, const Eigen::VectorXd& x0)
    : model_(model)
{
    const int n = model_.stateSize();
    x0_ = (x0.size() == n) ? x0 : Eigen::VectorXd::Zero(n);
    x_  = x0_;
    updateOutput(0.0);
}

void SimPlant::step(double u)
{
    std::lock_guard<std::mutex> lk(mu_);
    Eigen::VectorXd uv(model_.inputSize());
    uv.fill(u);

    const Eigen::VectorXd x_next = model_.A * x_ + model_.B * uv;
    updateOutput(u); // y[k] = C.x[k] + D.u[k]  (output at current x, before advance)
    x_ = x_next;    // advance state for next step
}

double SimPlant::output() const
{
    std::lock_guard<std::mutex> lk(mu_);
    return y_cached_;
}

void SimPlant::setState(const Eigen::VectorXd& x)
{
    std::lock_guard<std::mutex> lk(mu_);
    if (x.size() == model_.stateSize())
        x_ = x;
    updateOutput(0.0);
}

void SimPlant::reset()
{
    std::lock_guard<std::mutex> lk(mu_);
    x_ = x0_;
    updateOutput(0.0);
}

void SimPlant::updateOutput(double u)
{
    // Caller must hold mu_.
    Eigen::VectorXd uv(model_.inputSize());
    uv.fill(u);
    y_cached_ = (model_.C * x_ + model_.D * uv)(0);
}

} // namespace ctrl
