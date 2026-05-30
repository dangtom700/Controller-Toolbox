#include "AdaptiveSmithPredictor.h"
#include <algorithm>
#include <cmath>

namespace ctrl
{

AdaptiveSmithPredictor::AdaptiveSmithPredictor(
    std::shared_ptr<IController>  inner,
    const StateSpace              &delayModel,
    int                            initialDelaySteps,
    double                         Ts,
    const AdaptiveSPParams        &params)
    : inner_(std::move(inner)),
      delay_model_(delayModel),
      params_(params),
      Ts_(Ts),
      current_delay_(std::max(0, initialDelaySteps))
{
    rebuildSP();
}

void AdaptiveSmithPredictor::rebuildSP()
{
    // Build new SmithPredictor with the current delay estimate.
    // The inner controller is shared so its accumulated state is preserved.
    sp_ = std::make_shared<SmithPredictor>(inner_, delay_model_, current_delay_);
}

// ---------------------------------------------------------------------------
// estimateDelay - cross-correlation argmax over tau = 0..maxDelaySteps
// ---------------------------------------------------------------------------

void AdaptiveSmithPredictor::estimateDelay()
{
    const int N       = static_cast<int>(u_hist_.size());
    const int tau_max = std::min(params_.maxDelaySteps, N - 1);
    if (tau_max < 1) return; // not enough data

    // R_uy(tau) = sum_{k=tau}^{N-1} u_hist[k-tau] * y_hist[k]
    double best_R   = -1e300;
    int    best_tau = current_delay_;

    for (int tau = 0; tau <= tau_max; ++tau)
    {
        double R = 0.0;
        for (int k = tau; k < N; ++k)
            R += u_hist_[k - tau] * y_hist_[k];

        if (R > best_R)
        {
            best_R   = R;
            best_tau = tau;
        }
    }

    if (best_tau != current_delay_)
    {
        current_delay_ = best_tau;
        rebuildSP();
    }
}

// ---------------------------------------------------------------------------
// compute
// ---------------------------------------------------------------------------

double AdaptiveSmithPredictor::compute(double error)
{
    // Delegate to the inner Smith Predictor.
    const double u = sp_->compute(error);
    last_output_ = u;

    // Determine y for the correlation buffer.
    // If setPlantOutput() was called this step, use it; otherwise approximate y approx = -error.
    const double y_buf_val = has_y_ ? last_y_ : -error;
    has_y_ = false;

    // Append u[k-1] (the control applied at the previous step) and current y[k].
    u_hist_.push_back(last_u_);
    y_hist_.push_back(y_buf_val);

    if (static_cast<int>(u_hist_.size()) > params_.bufferLen)
    {
        u_hist_.pop_front();
        y_hist_.pop_front();
    }

    last_u_ = u; // store for next step
    ++step_count_;

    // Trigger delay estimation when buffer is sufficiently full.
    const int min_buf = params_.maxDelaySteps + 10;
    if (step_count_ % params_.estimateInterval == 0 &&
        static_cast<int>(u_hist_.size()) >= min_buf)
    {
        estimateDelay();
    }

    notifyObserver(u, error);
    return u;
}

// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------

void AdaptiveSmithPredictor::reset()
{
    sp_->reset();
    u_hist_.clear();
    y_hist_.clear();
    last_y_      = 0.0;
    has_y_       = false;
    last_u_      = 0.0;
    last_output_ = 0.0;
    step_count_  = 0;
    notifyObserverReset();
}

} // namespace ctrl
