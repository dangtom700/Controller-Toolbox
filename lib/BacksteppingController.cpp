#include "BacksteppingController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

BacksteppingController::BacksteppingController(std::vector<DriftFn> f, std::vector<GainFn> g,
                                                const BacksteppingParams &params, double Ts)
    : f_(std::move(f)), g_(std::move(g)), params_(params), Ts_(Ts)
{
    N_ = static_cast<int>(f_.size());
    if (N_ == 0 || static_cast<int>(g_.size()) != N_ ||
        static_cast<int>(params_.k_gains.size()) != N_)
        throw std::invalid_argument(
            "BacksteppingController: f, g, and params.k_gains must all have the same "
            "non-zero size (one entry per recursion stage).");

    x_ = Eigen::VectorXd::Zero(N_);
    alphaPrevStore_.assign(N_, 0.0);
}

double BacksteppingController::compute(double error)
{
    if (!std::isfinite(error) || !x_.allFinite())
        return u_prev_;

    const double r = x_(0) + error; // error = r - x1 => r = x1 + error

    const double rDot = initialized_ ? (r - rPrev_) / Ts_ : 0.0;

    std::vector<double> z(N_), alpha(N_), alphaDot(N_);
    double alphaPrevVal = r;
    double alphaPrevDot = rDot;
    double gPrevVal      = 0.0;

    for (int s = 0; s < N_; ++s)
    {
        z[s] = x_(s) - alphaPrevVal;

        const double fs = f_[s](x_, s);
        const double gs = g_[s](x_, s);
        constexpr double kEpsG = 1e-9;
        const double gEff = (std::fabs(gs) > kEpsG) ? gs : (gs >= 0.0 ? kEpsG : -kEpsG);

        const double crossTerm = (s > 0) ? gPrevVal * z[s - 1] : 0.0;
        alpha[s] = (-fs - params_.k_gains[s] * z[s] + alphaPrevDot - crossTerm) / gEff;

        alphaDot[s] = initialized_ ? (alpha[s] - alphaPrevStore_[s]) / Ts_ : 0.0;

        gPrevVal      = gs;
        alphaPrevVal  = alpha[s];
        alphaPrevDot  = alphaDot[s];
    }

    rPrev_ = r;
    alphaPrevStore_ = alpha; // store the *unclamped* virtual controls for the next finite difference
    initialized_ = true;

    const double u = std::clamp(alpha[N_ - 1], params_.uMin, params_.uMax);
    u_prev_ = u;
    notifyObserver(u, error);
    return u;
}

void BacksteppingController::reset()
{
    initialized_ = false;
    rPrev_ = 0.0;
    std::fill(alphaPrevStore_.begin(), alphaPrevStore_.end(), 0.0);
    u_prev_ = 0.0;
    notifyObserverReset();
}

} // namespace ctrl
