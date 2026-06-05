#include "CEMController.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace ctrl {

CEMController::CEMController(const Params& p,
                               StateFunc f,
                               Eigen::MatrixXd C,
                               double Ts)
    : p_(p)
    , f_(std::move(f))
    , C_(std::move(C))
    , Ts_(Ts)
    , rng_(p.seed)
{
    if (p_.Np <= 0 || p_.N_samples <= 0 || p_.n_iter <= 0)
        throw std::invalid_argument("CEMController: Np, N_samples, n_iter must be positive");
    if (p_.elite_frac <= 0.0 || p_.elite_frac >= 1.0)
        throw std::invalid_argument("CEMController: elite_frac must be in (0, 1)");

    mu_.setZero(p_.Np);   // initial mean = zeros (no initial warm-start)
}

// ---------------------------------------------------------------------------
// Cost function for a single action sequence
// ---------------------------------------------------------------------------

double CEMController::rolloutCost(const Eigen::VectorXd& x0,
                                    const Eigen::VectorXd& u_seq,
                                    const Eigen::VectorXd& y_ref) const
{
    Eigen::VectorXd x = x0;
    double cost = 0.0;
    for (int k = 0; k < p_.Np; ++k) {
        Eigen::VectorXd u_k(1); u_k(0) = u_seq(k);
        x = f_(x, u_k);
        Eigen::VectorXd e = C_ * x - y_ref;
        cost += p_.Q * e.squaredNorm() + p_.R * u_k.squaredNorm();
    }
    return cost;
}

// ---------------------------------------------------------------------------
// CEM optimisation
// ---------------------------------------------------------------------------

Eigen::VectorXd CEMController::computeRef(const Eigen::VectorXd& x_cur,
                                             const Eigen::VectorXd& y_ref)
{
    const int N = p_.N_samples;
    const int K = std::max(1, static_cast<int>(p_.elite_frac * N));

    Eigen::VectorXd mu = mu_;   // warm-start from previous solution
    double sigma = p_.sigma_init;

    std::normal_distribution<double> nd(0.0, 1.0);

    std::vector<Eigen::VectorXd> samples(N, Eigen::VectorXd(p_.Np));
    std::vector<double>          costs(N);

    for (int iter = 0; iter < p_.n_iter; ++iter) {
        // Sample action sequences
        for (int i = 0; i < N; ++i) {
            for (int k = 0; k < p_.Np; ++k)
                samples[i](k) = std::clamp(mu(k) + sigma * nd(rng_),
                                            p_.uMin, p_.uMax);
            costs[i] = rolloutCost(x_cur, samples[i], y_ref);
        }

        // Sort by cost and keep elite set
        std::vector<int> idx(N);
        std::iota(idx.begin(), idx.end(), 0);
        std::partial_sort(idx.begin(), idx.begin() + K, idx.end(),
                          [&](int a, int b) { return costs[a] < costs[b]; });

        // Update distribution from elite set
        Eigen::VectorXd new_mu = Eigen::VectorXd::Zero(p_.Np);
        double new_var = 0.0;
        for (int j = 0; j < K; ++j) {
            new_mu += samples[idx[j]];
            new_var += (samples[idx[j]] - mu).squaredNorm() / p_.Np;
        }
        mu      = new_mu / K;
        sigma   = std::max(p_.sigma_min, std::sqrt(new_var / K));
    }

    last_cost_ = rolloutCost(x_cur, mu, y_ref);
    mu_        = mu;   // warm-start next call

    Eigen::VectorXd u_out(1);
    u_out(0) = std::clamp(mu(0), p_.uMin, p_.uMax);
    notifyObserver(u_out(0), x_cur.norm());
    return u_out;
}

double CEMController::compute(double error)
{
    if (!std::isfinite(error) || x_.size() == 0 || r_.size() == 0)
        return 0.0;

    auto u = computeRef(x_, r_);
    return u(0);
}

void CEMController::reset()
{
    mu_.setZero(p_.Np);
    last_cost_ = 0.0;
}

} // namespace ctrl
