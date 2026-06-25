#include "CLFController.h"
#include <algorithm>
#include <cmath>

namespace ctrl {

CLFController::CLFController(VFn V, LfVFn LfV, LgVFn LgV, const CLFParams &params, double Ts)
    : V_(std::move(V)), LfV_(std::move(LfV)), LgV_(std::move(LgV)), params_(params), Ts_(Ts)
{
}

double CLFController::compute(double /*error*/)
{
    if (!x_.allFinite() || x_.size() == 0)
        return u_prev_;

    const double Vx  = V_(x_);
    const double a   = LfV_(x_) + params_.alpha * Vx;
    const double b   = LgV_(x_);

    if (!std::isfinite(a) || !std::isfinite(b))
        return u_prev_;

    constexpr double kEpsB = 1e-12;
    double u;
    if (std::fabs(b) > kEpsB)
    {
        u = -(a + std::sqrt(a * a + b * b * b * b)) / b;
        healthy_ = true;
    }
    else if (a <= 0.0)
    {
        u = 0.0; // drift already decaying; no control needed in this (uncontrollable) direction
        healthy_ = true;
    }
    else
    {
        // Uncontrollable direction (LgV == 0) with non-decaying drift: infeasible.
        healthy_ = false;
        return u_prev_;
    }

    u = std::clamp(u, params_.uMin, params_.uMax);
    u_prev_ = u;
    notifyObserver(u, 0.0);
    return u;
}

void CLFController::reset()
{
    u_prev_ = 0.0;
    healthy_ = true;
    notifyObserverReset();
}

} // namespace ctrl
