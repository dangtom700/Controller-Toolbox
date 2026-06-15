#include "CEMController.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace ctrl {

CEMController::CEMController(const Params&          p,
                               StateFunc              f,
                               const Eigen::MatrixXd& C,
                               double                 Ts)
    : p_(p)
    , f_(std::move(f))
    , C_(C)
    , Ts_(Ts)
    , rng_(p.seed)
{
    if (p_.Np <= 0 || p_.N_samples <= 0 || p_.n_iter <= 0)
        throw std::invalid_argument("CEMController: Np, N_samples, n_iter must be positive");
    if (p_.elite_frac <= 0.0 || p_.elite_frac >= 1.0)
        throw std::invalid_argument("CEMController: elite_frac must be in (0, 1)");

    mu_.setZero(p_.Np);
    samples_.assign(p_.N_samples, Eigen::VectorXd(p_.Np));
    costs_.resize(p_.N_samples);
    new_mu_.resize(p_.Np);
    u_k_.resize(1);
    u_k_roll_.resize(1);
    // x_roll_ and e_roll_ sized lazily on first computeRef() (state size not known at ctor)
}

// ---------------------------------------------------------------------------
// Cost function for a single action sequence
// ---------------------------------------------------------------------------

double CEMController::rolloutCost(const Eigen::VectorXd& x0,
                                    const Eigen::VectorXd& u_seq,
                                    const Eigen::VectorXd& y_ref) const
{
    // Use pre-allocated mutable workspaces: no per-rollout or per-iteration heap allocs.
    x_roll_ = x0;
    double cost = 0.0;
    for (int k = 0; k < p_.Np; ++k) {
        u_k_roll_(0) = u_seq(k);
        x_roll_ = f_(x_roll_, u_k_roll_);
        e_roll_.noalias() = C_ * x_roll_ - y_ref;
        cost += p_.Q * e_roll_.squaredNorm() + p_.R * u_k_roll_.squaredNorm();
    }
    return cost;
}

// ---------------------------------------------------------------------------
// CEM optimisation
// ---------------------------------------------------------------------------

Eigen::VectorXd CEMController::computeRef(const Eigen::VectorXd& x_cur,
                                             const Eigen::VectorXd& y_ref)
{
    // Lazy-init rollout workspaces on first call (state/output sizes known here)
    if (x_roll_.size() != x_cur.size()) x_roll_.resize(x_cur.size());
    if (e_roll_.size() != C_.rows())    e_roll_.resize(C_.rows());

    const int N = p_.N_samples;
    const int K = std::max(1, static_cast<int>(p_.elite_frac * N));

    Eigen::VectorXd mu = mu_;   // warm-start from previous solution
    double sigma = p_.sigma_init;

    std::normal_distribution<double> nd(0.0, 1.0);

    // Reuse pre-allocated workspace (samples_, costs_, new_mu_)
    for (int iter = 0; iter < p_.n_iter; ++iter) {
        // Sample action sequences
        for (int i = 0; i < N; ++i) {
            for (int k = 0; k < p_.Np; ++k)
                samples_[i](k) = std::clamp(mu(k) + sigma * nd(rng_),
                                             p_.uMin, p_.uMax);
            costs_[i] = rolloutCost(x_cur, samples_[i], y_ref);
        }

        // Sort by cost and keep elite set
        std::vector<int> idx(N);
        std::iota(idx.begin(), idx.end(), 0);
        std::partial_sort(idx.begin(), idx.begin() + K, idx.end(),
                          [&](int a, int b) { return costs_[a] < costs_[b]; });

        // Update distribution from elite set
        new_mu_.setZero();
        double new_var = 0.0;
        for (int j = 0; j < K; ++j) {
            new_mu_ += samples_[idx[j]];
            new_var += (samples_[idx[j]] - mu).squaredNorm() / p_.Np;
        }
        mu      = new_mu_ / K;
        sigma   = std::max(p_.sigma_min, std::sqrt(new_var / K));
    }

    last_cost_ = rolloutCost(x_cur, mu, y_ref);
    mu_        = mu;   // warm-start next call

    u_k_(0) = std::clamp(mu(0), p_.uMin, p_.uMax);
    notifyObserver(u_k_(0), x_cur.norm());
    return u_k_;
}

double CEMController::compute(double error)
{
    if (!std::isfinite(error) || x_.size() == 0 || r_.size() == 0) {
#ifndef NDEBUG
        std::clog << "[CEMController] WARNING: compute() called before setState/setReference - returning 0\n";
#endif
        return 0.0;
    }

    auto u = computeRef(x_, r_);
    return u(0);
}

void CEMController::reset()
{
    mu_.setZero(p_.Np);
    last_cost_ = 0.0;
}

} // namespace ctrl
