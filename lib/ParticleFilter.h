#pragma once
#include <Eigen/Dense>
#include <functional>
#include <random>
#include <vector>

/**
 * @file ParticleFilter.h
 * @brief Sequential Importance Resampling (SIR) particle filter.
 *
 * Estimates the state of an arbitrary discrete-time nonlinear system
 * from noisy measurements, without any Gaussianity or unimodality assumption.
 *
 * **Algorithm (SIR / Bootstrap filter, Gordon, Salmond & Smith 1993):**
 *
 *   **Initialise:**  sample N particles x_i ~ N(x0, P0), weights w_i = 1/N.
 *
 *   **Predict:**     x_i = f(x_i, u) + noise, noise ~ N(0, Q).
 *
 *   **Update:**      w_i *= p(y | x_i, u) = N(y - h(x_i,u); 0, R).
 *                    Normalise weights.
 *
 *   **Resample:**    If N_eff = 1/Sigmaw_i^2 < resample_threshold,
 *                    apply systematic resampling (lower variance than multinomial).
 *
 *   **Estimate:**    x_hat = Sigma w_i * x_i
 *
 * **When to use this over EKF / UKF:**
 *   - Multi-modal posterior (e.g. bearings-only tracking, ABS slip estimation).
 *   - Heavy-tailed or non-Gaussian noise.
 *   - Strongly nonlinear dynamics where linearisation (EKF) diverges.
 *   - Discontinuous measurement functions (e.g., quantised sensors).
 *
 * **Computational cost:** O(N.(n_state + n_meas)) per step.
 *   Typical N: 200-2000. For real-time use, profile against your compute budget.
 *
 * **Degeneracy:** Weight degeneracy (N_eff -> 1) occurs when the process noise
 * prior is a poor proposal for the likelihood. In this case, use an improved
 * proposal distribution (e.g. the locally optimal proposal) or increase N.
 *
 * @see ExtendedKalmanFilter  - Gaussian linearised estimator.
 * @see UnscentedKalmanFilter - Sigma-point Gaussian estimator.
 * @see Gordon, Salmond & Smith (1993). "Novel approach to nonlinear/non-Gaussian
 *      Bayesian state estimation." IEE Proc. F, 140(2):107-113.
 * @see Arulampalam et al. (2002). "A tutorial on particle filters for online
 *      nonlinear/non-Gaussian Bayesian tracking." IEEE Trans. Signal Proc. 50(2).
 */

namespace ctrl {

/** @brief Process and measurement model functions for the particle filter. */
using ParticleFn  = std::function<Eigen::VectorXd(const Eigen::VectorXd &x,
                                                    const Eigen::VectorXd &u)>;
using ParticleMeasFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &x,
                                                      const Eigen::VectorXd &u)>;

/**
 * @brief Configuration parameters for the SIR particle filter.
 */
struct ParticleFilterParams
{
    int n_particles = 500;   ///< Number of particles N. More -> accuracy, cost.

    /**
     * @brief Process noise covariance Q (n_state * n_state, PSD).
     *
     * Samples are drawn as epsilon ~ N(0, Q) and added to the propagated particle:
     *   x_i_predicted = f(x_i, u) + epsilon_i.
     * Too small Q -> weight degeneracy (particles collapse to one).
     * Too large Q -> high variance, slow convergence.
     */
    Eigen::MatrixXd Q;

    /**
     * @brief Measurement noise covariance R (n_meas * n_meas, PD).
     *
     * Used to evaluate the likelihood: p(y|x_i) = N(y - h(x_i,u); 0, R).
     * Set to match the actual sensor noise statistics.
     */
    Eigen::MatrixXd R;

    /**
     * @brief Resampling threshold on the effective sample size.
     *
     * Resampling is triggered when N_eff = 1/Sigmaw_i^2 < resample_threshold.
     * Default: n_particles/2 (standard rule of thumb).
     * Setting to 0 disables automatic resampling.
     */
    double resample_threshold = -1.0; // -1 = auto (N/2)

    double Ts = 0.1;     ///< Sample time [s] (informational, not used in algorithm).
    unsigned seed = 42u; ///< RNG seed (0 = random device).
};

/**
 * @brief SIR particle filter for nonlinear/non-Gaussian state estimation.
 *
 * Mirrors the EKF/UKF API: predict(u) + update(y,u) + step(y,u_prev).
 */
class ParticleFilter
{
public:
    /**
     * @brief Construct a particle filter with the given models and parameters.
     *
     * @param p         Filter parameters (Q, R, n_particles, seed).
     * @param n_states  State dimension (n).
     * @param n_meas    Measurement dimension (p).
     * @param f         Process function x[k+1] = f(x[k], u[k]).
     * @param h         Measurement function y[k] = h(x[k], u[k]).
     * @throws std::invalid_argument If Q/R dimensions do not match n_states/n_meas.
     */
    ParticleFilter(const ParticleFilterParams &p,
                   int n_states, int n_meas,
                   ParticleFn  f,
                   ParticleMeasFn h);

    /**
     * @brief Initialise particle cloud from a Gaussian prior N(x0, P0).
     *
     * Must be called before predict() / update() / step().
     *
     * @param x0 Initial state mean (n * 1).
     * @param P0 Initial covariance (n * n, positive definite). Defaults to I.
     */
    void initialise(const Eigen::VectorXd &x0,
                    const Eigen::MatrixXd &P0 = Eigen::MatrixXd());

    /**
     * @brief Prediction step: propagate all particles through f and add process noise.
     *
     * @param u Control input u[k] (m * 1).
     */
    void predict(const Eigen::VectorXd &u);

    /**
     * @brief Update step: compute likelihoods and update weights.
     *
     * Calls resample() if N_eff < resample_threshold after normalisation.
     *
     * @param y Measurement y[k] (p * 1).
     * @param u Control input u[k] (m * 1). Passed to h(x, u).
     */
    void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u);

    /**
     * @brief Combined predict(u_prev) + update(y, u_prev).
     *
     * Most common usage - call once per sample step.
     *
     * @param y      Measurement y[k].
     * @param u_prev Control applied at step k-1 (used in predict step).
     */
    void step(const Eigen::VectorXd &y, const Eigen::VectorXd &u_prev);

    /**
     * @brief Trigger a manual resample regardless of N_eff.
     *
     * Systematic resampling (Kitagawa 1996) - lower variance than multinomial.
     */
    void resample();

    // -------------------------------------------------------------------------
    // State accessors
    // -------------------------------------------------------------------------

    /**
     * @brief Weighted mean estimate x_hat = Sigma w_i * x_i.
     * @return State estimate (n * 1).
     */
    Eigen::VectorXd state() const;

    /**
     * @brief Weighted sample covariance P = Sigma w_i * (x_i - x_hat)(x_i - x_hat)'.
     * @return Covariance matrix (n * n).
     */
    Eigen::MatrixXd covariance() const;

    /**
     * @brief Effective sample size N_eff = 1 / Sigma w_i^2.
     *
     * N_eff approx = N: all particles contribute equally (good diversity).
     * N_eff approx = 1: weight degeneracy (increase Q or n_particles).
     *
     * @return Effective sample size \in [1, N].
     */
    double effectiveSampleSize() const;

    /** @return Read-only access to the particle cloud (n_particles vectors, each n * 1). */
    const std::vector<Eigen::VectorXd> &particles() const { return particles_; }

    /** @return Read-only access to the normalised weight vector (n_particles * 1). */
    const Eigen::VectorXd &weights() const { return w_; }

    /** @return Number of times resampling has been triggered since construction. */
    int resampleCount() const { return resample_count_; }

    /** @return True if initialise() has been called. */
    bool isInitialised() const { return initialised_; }

    /** @return Sample time Ts [s] from parameters. */
    double sampleTime() const { return p_.Ts; }

    /**
     * @brief Reset the filter (particles and weights) to the initial distribution.
     *
     * Requires a subsequent initialise() call before predict/update.
     */
    void reset();

private:
    void sampleNoise(Eigen::VectorXd &out, const Eigen::MatrixXd &L);

    ParticleFilterParams   p_;
    int n_states_, n_meas_;
    ParticleFn             f_;
    ParticleMeasFn         h_;

    std::vector<Eigen::VectorXd> particles_; ///< Current particle set (N * n).
    Eigen::VectorXd              w_;          ///< Normalised probability weights (N), each in [0, 1], sum = 1.

    Eigen::MatrixXd L_Q_; ///< Cholesky of Q for noise sampling.
    Eigen::MatrixXd R_inv_; ///< R^{-1} for likelihood evaluation.

    std::mt19937                  rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};
    std::uniform_real_distribution<double> uniform_{0.0, 1.0};

    double  resample_thresh_;
    bool    initialised_{false};
    int     resample_count_{0};

    // Pre-allocated systematic resampling workspaces
    std::vector<Eigen::VectorXd> resample_buf_; ///< N particle scratch buffer for resample()
    Eigen::VectorXd              cdf_;           ///< N-length CDF for systematic resampling
};

} // namespace ctrl
