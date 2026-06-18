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

    // Demean both buffers before cross-correlation.
    // Without demeaning, R_uy(tau) is dominated by u_mean*y_mean*(N-tau),
    // which always peaks at tau=0 regardless of the true delay.
    double u_mean = 0.0, y_mean = 0.0;
    for (int i = 0; i < N; ++i) { u_mean += u_hist_[i]; y_mean += y_hist_[i]; }
    u_mean /= N;
    y_mean /= N;

    // Normalized cross-covariance: divide by (N-tau) so longer windows at small tau
    // do not artificially inflate R at tau=0.
    // rho(tau) = [1/(N-tau)] * sum_{k=tau}^{N-1} (u[k-tau]-u_mean) * (y[k]-y_mean)
    double best_R   = -1e300;
    int    best_tau = current_delay_;

    for (int tau = 0; tau <= tau_max; ++tau)
    {
        const int n_terms = N - tau;
        if (n_terms < 1) continue;
        double R = 0.0;
        for (int k = tau; k < N; ++k)
            R += (u_hist_[k - tau] - u_mean) * (y_hist_[k] - y_mean);
        R /= static_cast<double>(n_terms);

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
    if (params_.bufferLen > 0) {
        u_hist_.push_back(last_u_);
        y_hist_.push_back(y_buf_val);

        if (static_cast<int>(u_hist_.size()) > params_.bufferLen)
        {
            u_hist_.pop_front();
            y_hist_.pop_front();
        }
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
