#include "LearningFeedforwardController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

LearningFeedforwardController::LearningFeedforwardController(std::shared_ptr<IController> nominal,
                                                             const ILC::Params &ilc_p,
                                                             const LearningFFParams &params,
                                                             double Ts)
    : nominal_(std::move(nominal)), ilc_(ilc_p), p_(params), Ts_(Ts)
{
    if (!nominal_)
        throw std::invalid_argument("LearningFeedforwardController: nominal controller must not be null");
    if (Ts_ <= 0.0)
        throw std::invalid_argument("LearningFeedforwardController: Ts must be > 0");
    if (p_.uMin >= p_.uMax)
        throw std::invalid_argument("LearningFeedforwardController: uMin must be < uMax");
    if (p_.learnTrials < 0)
        throw std::invalid_argument("LearningFeedforwardController: learnTrials must be >= 0");
    // A mismatch here silently truncates learning (ILC would only ever see the first
    // trialLength samples of its buffer, or feedforward() would throw), so reject it.
    if (p_.trialLength != ilc_p.N)
        throw std::invalid_argument("LearningFeedforwardController: trialLength must equal ILC::Params::N");

    flip_record_ = (nominal_->signConvention() == SignConvention::TrackingErrorYMinusR);
}

double LearningFeedforwardController::compute(double error)
{
    if (!std::isfinite(error))
        return u_prev_; // hold last command; neither the loop nor the trial advances

    const double u_fb = nominal_->compute(error);

    u_ff_ = learning() ? 0.0 : ilc_.feedforward(k_);

    // ILC's update law is written for e = r - y; undo the nominal's convention if needed.
    ilc_.recordError(k_, flip_record_ ? -error : error);

    const double u_raw = u_ff_ + u_fb;
    double u = std::clamp(u_raw, p_.uMin, p_.uMax);
    if (!std::isfinite(u))
        u = u_prev_;
    u_prev_ = u;

    if (++k_ >= p_.trialLength)
    {
        k_ = 0;
        if (p_.autoAdvance)
            ilc_.updateFeedforward();
    }

    notify_buf_(0) = u_ff_;
    notifyObserverState("u_ilc", notify_buf_);
    notifyObserver(u, error);
    return u;
}

void LearningFeedforwardController::endTrial()
{
    ilc_.updateFeedforward();
    k_ = 0;
}

void LearningFeedforwardController::bumplessInit(double u_target, double error)
{
    // Only the nominal loop is adjustable; the learned term is fixed for this step.
    const double u_ff = learning() ? 0.0 : ilc_.feedforward(k_);
    nominal_->bumplessInit(u_target - u_ff, error);
    u_prev_ = u_target;
}

void LearningFeedforwardController::reset()
{
    nominal_->reset();
    ilc_.reset();
    k_ = 0;
    u_ff_ = 0.0;
    u_prev_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
