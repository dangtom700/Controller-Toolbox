#include "GaussianProcess.h"
#include <cmath>
#include <stdexcept>

namespace ctrl {

GaussianProcess::GaussianProcess(int x_dim, const Params& p)
    : x_dim_(x_dim), p_(p)
{
    if (x_dim_ <= 0)
        throw std::invalid_argument("GaussianProcess: x_dim must be positive");
}

double GaussianProcess::kernel(const Eigen::VectorXd& a,
                                 const Eigen::VectorXd& b) const
{
    double d2 = (a - b).squaredNorm();
    return p_.signal_var * std::exp(-d2 / (2.0 * p_.length_scale * p_.length_scale));
}

void GaussianProcess::addPoint(const Eigen::VectorXd& x, double y)
{
    if (p_.n_max > 0 && static_cast<int>(X_.size()) >= p_.n_max) {
        X_.erase(X_.begin());
        Y_.erase(Y_.begin());
    }
    X_.push_back(x);
    Y_.push_back(y);
    fitted_ = false;   // must re-fit after adding data
}

void GaussianProcess::fit()
{
    const int N = size();
    if (N == 0) {
        fitted_ = true;
        return;
    }

    Eigen::MatrixXd K(N, N);
    for (int i = 0; i < N; ++i) {
        K(i, i) = kernel(X_[i], X_[i]) + p_.noise_var;
        for (int j = i + 1; j < N; ++j) {
            double kij = kernel(X_[i], X_[j]);
            K(i, j) = K(j, i) = kij;
        }
    }

    L_chol_.compute(K);
    if (L_chol_.info() != Eigen::Success)
        throw std::runtime_error("GaussianProcess::fit(): covariance not positive-definite");

    Eigen::VectorXd y_vec = Eigen::Map<const Eigen::VectorXd>(Y_.data(), N);
    alpha_ = L_chol_.solve(y_vec);
    fitted_ = true;
}

GaussianProcess::Prediction
GaussianProcess::predict(const Eigen::VectorXd& x_test) const
{
    if (!fitted_)
        throw std::runtime_error("GaussianProcess::predict(): call fit() first");

    const int N = size();
    if (N == 0)
        return {0.0, p_.signal_var};

    // k_star: covariance between test point and training data
    Eigen::VectorXd k_star(N);
    for (int i = 0; i < N; ++i)
        k_star(i) = kernel(x_test, X_[i]);

    double mean = k_star.dot(alpha_);

    // Posterior variance: k** - k_*' K^{-1} k_*
    Eigen::VectorXd v = L_chol_.solve(k_star);
    double var = kernel(x_test, x_test) - k_star.dot(v);
    var = std::max(var, 0.0);   // numerical safety

    return {mean, var};
}

void GaussianProcess::reset()
{
    X_.clear();
    Y_.clear();
    alpha_.resize(0);
    fitted_ = false;
}

} // namespace ctrl
