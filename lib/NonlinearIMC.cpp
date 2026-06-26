#include "NonlinearIMC.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

NonlinearIMC::NonlinearIMC(ModelFn model, InverseModelFn inverse,
                           const NonlinearIMCParams &params, double Ts)
    : model_(std::move(model)), inverse_(std::move(inverse)),
      lambda_(params.filter_lambda), uMin_(params.uMin), uMax_(params.uMax), Ts_(Ts)
{
    if (!model_ || !inverse_)
        throw std::invalid_argument("NonlinearIMC: model and inverse callbacks must be set.");
    if (Ts_ <= 0.0)
        throw std::invalid_argument("NonlinearIMC: Ts must be > 0.");
    if (lambda_ < 0.0 || lambda_ >= 1.0)
        throw std::invalid_argument("NonlinearIMC: filter_lambda must be in [0, 1).");
    if (uMin_ >= uMax_)
        throw std::invalid_argument("NonlinearIMC: uMin must be < uMax.");
}

double NonlinearIMC::compute(double error)
{
    if (!std::isfinite(error))
        return u_prev_; // hold-last NaN contract

    // Corrected setpoint: s = r - d = e + y_model  (d = y - y_model is the model mismatch).
    const double s = error + y_model_;

    // First-order IMC filter on the corrected setpoint.
    sf_ = lambda_ * sf_ + (1.0 - lambda_) * s;

    // Invert the model to find u that drives the model output to the filtered target.
    double u = inverse_(x_, sf_);
    if (!std::isfinite(u))
        return u_prev_; // graceful fallback on a singular/ill-posed inverse
    u = std::min(std::max(u, uMin_), uMax_);

    // Advance the internal model with the chosen control.
    const double y_model_new = model_(x_, u);
    if (std::isfinite(y_model_new))
        y_model_ = y_model_new;

    u_prev_ = u;
    notifyObserver(u, error);
    return u;
}

void NonlinearIMC::reset()
{
    y_model_ = 0.0;
    sf_      = 0.0;
    u_prev_  = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
