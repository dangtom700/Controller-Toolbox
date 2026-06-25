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

// ===========================================================================
// ParticleFilterV2 (Phase 3 Roadmap Phase 2 EF3)
// ===========================================================================

ParticleFilterV2::ParticleFilterV2(const ParticleFilterParamsV2 &p, int n_states, int n_meas,
                                    ParticleFn f, ParticleMeasFn h,
                                    const Eigen::MatrixXd &A_lin, const Eigen::MatrixXd &B_lin,
                                    const Eigen::MatrixXd &C_lin,
                                    const Eigen::MatrixXd &Q_lin, const Eigen::MatrixXd &R_lin)
    : ParticleFilter(p, n_states, n_meas, std::move(f), std::move(h)),
      p2_(p), A_lin_(A_lin), B_lin_(B_lin), C_lin_(C_lin), Q_lin_(Q_lin), R_lin_(R_lin)
{
    if (p2_.variant == PFVariant::RaoBlackwellized)
    {
        const int nLin = static_cast<int>(p2_.linear_state_indices.size());
        if (nLin == 0 || A_lin_.rows() != nLin || A_lin_.cols() != nLin ||
            C_lin_.cols() != nLin || Q_lin_.rows() != nLin || Q_lin_.cols() != nLin)
            throw std::invalid_argument(
                "ParticleFilterV2: RaoBlackwellized requires A_lin/Q_lin (nLin x nLin), "
                "C_lin (n_meas x nLin), and a non-empty linear_state_indices");
        for (int idx : p2_.linear_state_indices)
            if (idx < 0 || idx >= n_states)
                throw std::invalid_argument("ParticleFilterV2: linear_state_indices out of range");

        const StateSpace linPlant(A_lin_, B_lin_, C_lin_,
                                   Eigen::MatrixXd::Zero(C_lin_.rows(), B_lin_.cols()), p.Ts);
        kf_.reserve(p.n_particles);
        kfResampleBuf_.reserve(p.n_particles);
        for (int i = 0; i < p.n_particles; ++i)
        {
            kf_.emplace_back(linPlant, Q_lin_, R_lin_);
            kfResampleBuf_.emplace_back(linPlant, Q_lin_, R_lin_);
        }
    }
}

std::vector<int> ParticleFilterV2::systematicIndices(const Eigen::VectorXd &weights)
{
    const int N = p_.n_particles;
    cdf_(0) = weights(0);
    for (int i = 1; i < N; ++i) cdf_(i) = cdf_(i - 1) + weights(i);

    const double inv_N = 1.0 / static_cast<double>(N);
    const double u0 = uniform_(rng_) * inv_N;

    std::vector<int> idx(N);
    int j = 0;
    for (int i = 0; i < N; ++i)
    {
        const double ui = u0 + static_cast<double>(i) * inv_N;
        while (j < N - 1 && cdf_(j) < ui) ++j;
        idx[i] = j;
    }
    return idx;
}

void ParticleFilterV2::predict(const Eigen::VectorXd &u)
{
    if (p2_.variant != PFVariant::RaoBlackwellized)
    {
        ParticleFilter::predict(u); // Bootstrap: pure inheritance. Auxiliary: see step().
        return;
    }

    Eigen::VectorXd noise(n_states_);
    for (int i = 0; i < p_.n_particles; ++i)
    {
        sampleNoise(noise, L_Q_);
        particles_[i] = f_(particles_[i], u) + noise; // linear-indexed entries overwritten below
        kf_[i].predict(u);
        const Eigen::VectorXd &xLin = kf_[i].state();
        for (size_t j = 0; j < p2_.linear_state_indices.size(); ++j)
            particles_[i](p2_.linear_state_indices[j]) = xLin(static_cast<int>(j));
    }
}

void ParticleFilterV2::update(const Eigen::VectorXd &y, const Eigen::VectorXd &u)
{
    if (p2_.variant != PFVariant::RaoBlackwellized)
    {
        ParticleFilter::update(y, u); // Bootstrap: pure inheritance. Auxiliary: see step().
        return;
    }

    // A_lin/C_lin/Q_lin/R_lin are LTI and shared across particles, so every kf_[i]'s covariance
    // is identical after predict() (the covariance recursion doesn't depend on the data) -
    // compute the shared innovation covariance once.
    const Eigen::MatrixXd S = C_lin_ * kf_[0].covariance() * C_lin_.transpose() + R_lin_;
    const Eigen::MatrixXd Sinv = S.inverse();

    Eigen::VectorXd log_w = w_.array().log();
    for (int i = 0; i < p_.n_particles; ++i)
    {
        Eigen::VectorXd xZeroLin = particles_[i];
        for (int idx : p2_.linear_state_indices) xZeroLin(idx) = 0.0;
        const Eigen::VectorXd yOffset = h_(xZeroLin, u); // exploits additive separability
        const Eigen::VectorXd yPred = yOffset + C_lin_ * kf_[i].state();
        const Eigen::VectorXd innov = y - yPred;
        log_w(i) += -0.5 * innov.dot(Sinv * innov);

        kf_[i].update(y - yOffset, u);
        const Eigen::VectorXd &xLin = kf_[i].state();
        for (size_t j = 0; j < p2_.linear_state_indices.size(); ++j)
            particles_[i](p2_.linear_state_indices[j]) = xLin(static_cast<int>(j));
    }

    const double log_max = log_w.maxCoeff();
    w_ = (log_w.array() - log_max).exp();
    const double sum_w = w_.sum();
    if (sum_w > 0.0) w_ /= sum_w; else w_.setConstant(1.0 / p_.n_particles);

    if (resample_thresh_ > 0.0 && effectiveSampleSize() < resample_thresh_) resample();
}

void ParticleFilterV2::step(const Eigen::VectorXd &y, const Eigen::VectorXd &u_prev)
{
    if (p2_.variant != PFVariant::Auxiliary)
    {
        predict(u_prev); // dispatches virtually: Bootstrap or RaoBlackwellized
        update(y, u_prev);
        return;
    }

    // Auxiliary particle filter (Pitt & Shephard 1999): look-ahead resample using a
    // deterministic (noise-free) propagation proxy, before the real noisy propagation.
    const int N = p_.n_particles;
    std::vector<Eigen::VectorXd> mu(N);
    Eigen::VectorXd auxLogW(N);
    for (int i = 0; i < N; ++i)
    {
        mu[i] = f_(particles_[i], u_prev);
        const Eigen::VectorXd innov = y - h_(mu[i], u_prev);
        auxLogW(i) = std::log(w_(i)) - 0.5 * innov.dot(R_inv_ * innov);
    }
    const double auxLogMax = auxLogW.maxCoeff();
    Eigen::VectorXd auxW = (auxLogW.array() - auxLogMax).exp();
    const double auxSum = auxW.sum();
    if (auxSum > 0.0) auxW /= auxSum; else auxW.setConstant(1.0 / N);

    const std::vector<int> ancestors = systematicIndices(auxW);

    std::vector<Eigen::VectorXd> newParticles(N);
    Eigen::VectorXd newLogW(N);
    Eigen::VectorXd noise(n_states_);
    for (int i = 0; i < N; ++i)
    {
        const int a = ancestors[i];
        sampleNoise(noise, L_Q_);
        newParticles[i] = f_(particles_[a], u_prev) + noise;

        const Eigen::VectorXd innovReal = y - h_(newParticles[i], u_prev);
        const double logLikReal = -0.5 * innovReal.dot(R_inv_ * innovReal);
        const Eigen::VectorXd innovProxy = y - h_(mu[a], u_prev);
        const double logLikProxy = -0.5 * innovProxy.dot(R_inv_ * innovProxy);
        newLogW(i) = logLikReal - logLikProxy; // importance correction dividing out the proxy
    }
    particles_.swap(newParticles);

    const double newLogMax = newLogW.maxCoeff();
    w_ = (newLogW.array() - newLogMax).exp();
    const double newSum = w_.sum();
    if (newSum > 0.0) w_ /= newSum; else w_.setConstant(1.0 / N);

    if (resample_thresh_ > 0.0 && effectiveSampleSize() < resample_thresh_) resample();
}

void ParticleFilterV2::resample()
{
    if (p2_.variant != PFVariant::RaoBlackwellized)
    {
        ParticleFilter::resample();
        return;
    }

    const std::vector<int> ancestors = systematicIndices(w_);
    const int N = p_.n_particles;
    for (int i = 0; i < N; ++i)
    {
        resample_buf_[i] = particles_[ancestors[i]];
        kfResampleBuf_[i] = kf_[ancestors[i]];
    }
    particles_.swap(resample_buf_);
    kf_.swap(kfResampleBuf_);
    w_.setConstant(1.0 / static_cast<double>(N));
    ++resample_count_;
}

} // namespace ctrl
