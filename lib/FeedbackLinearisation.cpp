#include "FeedbackLinearisation.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

FeedbackLinearisationController::FeedbackLinearisationController(
    DriftFn f,
    GainFn  g,
    std::shared_ptr<IController> inner,
    const FLParams &params,
    double Ts)
    : f_(std::move(f))
    , g_(std::move(g))
    , inner_(std::move(inner))
    , params_(params)
    , Ts_(Ts)
{
    if (!inner_)
        throw std::invalid_argument("FeedbackLinearisationController: inner controller is null.");
    if (params_.uMin >= params_.uMax)
        throw std::invalid_argument("FeedbackLinearisationController: uMin must be < uMax.");
    if (params_.regularisationEps <= 0.0)
        throw std::invalid_argument("FeedbackLinearisationController: regularisationEps must be > 0.");
    // Initialise x_ as empty until setState() is called the first time
    x_ = Eigen::VectorXd::Zero(1);
}

double FeedbackLinearisationController::compute(double error)
{
    if (!std::isfinite(error)) return u_last_;
    // Step 1: inner controller computes the virtual control v
    const double v = inner_->compute(error);

    // Step 2: evaluate the drift and gain at the current state
    const double f_val = f_(x_, u_last_);
    const double g_val = g_(x_, u_last_);

    // Step 3: regularise to prevent division by zero
    // Preserve sign of g so the inversion direction is correct
    const double g_sign = (g_val >= 0.0) ? 1.0 : -1.0;
    const double g_eff  = g_sign * std::max(std::abs(g_val), params_.regularisationEps);

    // Step 4: feedback linearisation inversion
    double u = (v - f_val) / g_eff;

    // Step 5: saturate and store
    u = std::clamp(u, params_.uMin, params_.uMax);
    u_last_ = u;
    return u;
}

void FeedbackLinearisationController::reset()
{
    u_last_ = 0.0;
    if (inner_)
        inner_->reset();
}

} // namespace ctrl
