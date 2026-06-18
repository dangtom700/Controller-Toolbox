#include "ParticleFilter.h"
#include <stdexcept>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace ctrl {

// ---------------------------------------------------------------------------
ParticleFilter::ParticleFilter(const ParticleFilterParams &p,
                               int n_states, int n_meas,
                               ParticleFn  f,
                               ParticleMeasFn h)
    : p_(p), n_states_(n_states), n_meas_(n_meas),
      f_(std::move(f)), h_(std::move(h))
{
    if (p_.Q.rows() != n_states_ || p_.Q.cols() != n_states_)
        throw std::invalid_argument("ParticleFilter: Q must be n_states x n_states.");
    if (p_.R.rows() != n_meas_ || p_.R.cols() != n_meas_)
        throw std::invalid_argument("ParticleFilter: R must be n_meas x n_meas.");
    if (p_.n_particles <= 0)
        throw std::invalid_argument("ParticleFilter: n_particles must be > 0.");

    // Cholesky of Q for sampling
    Eigen::LLT<Eigen::MatrixXd> llt_Q(p_.Q);
    if (llt_Q.info() != Eigen::Success)
        throw std::invalid_argument("ParticleFilter: Q is not positive definite.");
    L_Q_ = llt_Q.matrixL();

    // Inverse of R for likelihood
    R_inv_ = p_.R.inverse();

    // RNG
    if (p_.seed == 0u) {
        std::random_device rd;
        rng_.seed(rd());
    } else {
        rng_.seed(p_.seed);
    }

    resample_thresh_ = (p_.resample_threshold < 0.0)
                       ? static_cast<double>(p_.n_particles) / 2.0
                       : p_.resample_threshold;

    particles_.resize(p_.n_particles, Eigen::VectorXd::Zero(n_states_));
    w_.resize(p_.n_particles);
    w_.setConstant(1.0 / static_cast<double>(p_.n_particles));

    resample_buf_.resize(p_.n_particles, Eigen::VectorXd::Zero(n_states_));
    cdf_.resize(p_.n_particles);
}

// ---------------------------------------------------------------------------
void ParticleFilter::initialise(const Eigen::VectorXd &x0,
                                 const Eigen::MatrixXd &P0)
{
    Eigen::MatrixXd L_P;
    if (P0.size() == 0) {
        L_P = Eigen::MatrixXd::Identity(n_states_, n_states_);
    } else {
        Eigen::LLT<Eigen::MatrixXd> llt(P0);
        if (llt.info() != Eigen::Success)
            throw std::invalid_argument("ParticleFilter::initialise: P0 is not positive definite.");
        L_P = llt.matrixL();
    }

    Eigen::VectorXd noise(n_states_);
    const double w0 = 1.0 / static_cast<double>(p_.n_particles);
    for (int i = 0; i < p_.n_particles; ++i) {
        sampleNoise(noise, L_P);
        particles_[i] = x0 + noise;
        w_(i) = w0;
    }
    initialised_ = true;
}

// ---------------------------------------------------------------------------
void ParticleFilter::sampleNoise(Eigen::VectorXd &out,
                                  const Eigen::MatrixXd &L)
{
    out.resize(L.cols());
    for (int j = 0; j < L.cols(); ++j)
        out(j) = normal_(rng_);
    out = L * out;
}

// ---------------------------------------------------------------------------
void ParticleFilter::predict(const Eigen::VectorXd &u)
{
    Eigen::VectorXd noise(n_states_);
    for (int i = 0; i < p_.n_particles; ++i) {
        sampleNoise(noise, L_Q_);
        particles_[i] = f_(particles_[i], u) + noise;
    }
}

// ---------------------------------------------------------------------------
// Log-sum-exp normalisation for numerical stability:
//   log w_i_unnorm = log w_i + log p(y|x_i)
//   Subtract max before exponentiating to avoid underflow.
// ---------------------------------------------------------------------------
void ParticleFilter::update(const Eigen::VectorXd &y,
                             const Eigen::VectorXd &u)
{
    Eigen::VectorXd log_w = w_.array().log(); // log of current weights

    for (int i = 0; i < p_.n_particles; ++i) {
        const Eigen::VectorXd innov = y - h_(particles_[i], u);
        // log p(y|x_i) = -0.5 * innov' * R_inv * innov   (Gaussian, dropping constant)
        const double log_lik = -0.5 * innov.dot(R_inv_ * innov);
        log_w(i) += log_lik;
    }

    // Normalise (log-sum-exp trick)
    const double log_max = log_w.maxCoeff();
    w_ = (log_w.array() - log_max).exp();
    const double sum_w = w_.sum();
    if (sum_w > 0.0)
        w_ /= sum_w;
    else
        w_.setConstant(1.0 / p_.n_particles); // degenerate fallback

    // Resample if effective sample size falls below threshold
    if (resample_thresh_ > 0.0 && effectiveSampleSize() < resample_thresh_)
        resample();
}

// ---------------------------------------------------------------------------
void ParticleFilter::step(const Eigen::VectorXd &y,
                           const Eigen::VectorXd &u_prev)
{
    predict(u_prev);
    update(y, u_prev);
}

// ---------------------------------------------------------------------------
// Systematic resampling (Kitagawa 1996) - O(N), lower variance than multinomial.
// Generates N equally-spaced points on [0,1) offset by a single uniform draw.
// ---------------------------------------------------------------------------
void ParticleFilter::resample()
{
    const int N = p_.n_particles;

    // Build CDF into pre-allocated member
    cdf_(0) = w_(0);
    for (int i = 1; i < N; ++i)
        cdf_(i) = cdf_(i - 1) + w_(i);

    // Single uniform draw U ~ Uniform[0, 1/N)
    const double inv_N = 1.0 / static_cast<double>(N);
    double u0 = uniform_(rng_) * inv_N;

    int j = 0;
    for (int i = 0; i < N; ++i) {
        const double ui = u0 + static_cast<double>(i) * inv_N;
        while (j < N - 1 && cdf_(j) < ui)
            ++j;
        resample_buf_[i] = particles_[j];
    }

    particles_.swap(resample_buf_);
    w_.setConstant(inv_N);
    ++resample_count_;
}

// ---------------------------------------------------------------------------
Eigen::VectorXd ParticleFilter::state() const
{
    Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(n_states_);
    for (int i = 0; i < p_.n_particles; ++i)
        x_hat += w_(i) * particles_[i];
    return x_hat;
}

// ---------------------------------------------------------------------------
Eigen::MatrixXd ParticleFilter::covariance() const
{
    const Eigen::VectorXd x_hat = state();
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_states_, n_states_);
    for (int i = 0; i < p_.n_particles; ++i) {
        const Eigen::VectorXd d = particles_[i] - x_hat;
        P += w_(i) * d * d.transpose();
    }
    return P;
}

// ---------------------------------------------------------------------------
double ParticleFilter::effectiveSampleSize() const
{
    return 1.0 / w_.squaredNorm();
}

// ---------------------------------------------------------------------------
void ParticleFilter::reset()
{
    for (auto &p : particles_)
        p.setZero();
    w_.setConstant(1.0 / static_cast<double>(p_.n_particles));
    resample_count_ = 0;
    initialised_    = false;
}

} // namespace ctrl
