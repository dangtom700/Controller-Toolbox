#include "TwoDOFController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

TwoDOFController::TwoDOFController(std::shared_ptr<IController> feedback,
                                   FeedforwardFn ff,
                                   const TwoDOFParams &params,
                                   double Ts)
    : feedback_(std::move(feedback)), ff_(std::move(ff)), p_(params), Ts_(Ts)
{
    if (!feedback_)
        throw std::invalid_argument("TwoDOFController: feedback controller must not be null");
    if (!ff_)
        throw std::invalid_argument("TwoDOFController: feedforward callable must not be empty");
    if (Ts_ <= 0.0)
        throw std::invalid_argument("TwoDOFController: Ts must be > 0");
    if (p_.uMin >= p_.uMax)
        throw std::invalid_argument("TwoDOFController: uMin must be < uMax");
}

double TwoDOFController::compute(double error)
{
    if (!std::isfinite(error))
        return u_prev_; // hold last command; the feedback loop does not advance

    // A user feedforward that returns a non-finite value would poison the command
    // path, so it is treated the same way as a bad measurement: drop the term.
    const double u_ff_raw = ff_(r_, d_);
    u_ff_ = std::isfinite(u_ff_raw) ? u_ff_raw : 0.0;

    u_fb_ = feedback_->compute(error);

    const double u_raw = u_ff_ + u_fb_;
    double u = std::clamp(u_raw, p_.uMin, p_.uMax);
    if (!std::isfinite(u))
        u = u_prev_;

    saturated_ = (u != u_raw);

    // Back-calculate the feedback path against the achievable trim (u_sat - u_ff),
    // which is the conditional-integration behaviour case studies hand-roll.
    if (p_.antiWindup && saturated_)
        feedback_->bumplessInit(u - u_ff_, error);

    u_prev_ = u;

    notify_buf_(0) = u_ff_;
    notifyObserverState("u_ff", notify_buf_);
    notifyObserver(u, error);
    return u;
}

void TwoDOFController::bumplessInit(double u_target, double error)
{
    // Only the feedback path is adjustable - the feedforward is a pure function of
    // (r, d), so the target it must hit is u_target minus the current feedforward.
    const double u_ff_raw = ff_(r_, d_);
    const double u_ff = std::isfinite(u_ff_raw) ? u_ff_raw : 0.0;
    feedback_->bumplessInit(u_target - u_ff, error);
    u_prev_ = u_target;
}

void TwoDOFController::reset()
{
    feedback_->reset();
    r_ = 0.0;
    d_ = 0.0;
    u_ff_ = 0.0;
    u_fb_ = 0.0;
    saturated_ = false;
    u_prev_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
