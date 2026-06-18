#pragma once
#include <Eigen/Dense>
#include <functional>

/**
 * @file UnscentedKalmanFilter.h
 * @brief Discrete-time Unscented Kalman Filter (UKF) for nonlinear systems.
 *
 * Handles nonlinear process and measurement models **without Jacobians**:
 * @code
 *   x[k+1] = f(x[k], u[k]) + w[k],   w ~ N(0, Q_noise)
 *   y[k]   = h(x[k], u[k]) + v[k],   v ~ N(0, R_noise)
 * @endcode
 *
 * @par Noise model assumption
 * Process and measurement noise are **additive** as shown above. For multiplicative or
 * state-dependent noise, augment the state vector with the noise channels and propagate
 * augmented sigma points - that form is **not** implemented here.
 *
 * The Unscented Transform propagates 2n+1 deterministically chosen sigma points through
 * the exact nonlinear functions, capturing mean and covariance to **third order** (vs.
 * EKF's first-order linearisation). No Jacobians are required.
 *
 * **Sigma points (scaled UT, Wan & Van der Merwe 2000):**
 * @code
 *   X_0     = x^
 *   X_i     = x^ + (\sqrt((n+lambda).P))_i       i = 1...n
 *   X_{n+i} = x^ - (\sqrt((n+lambda).P))_i       i = 1...n
 *   lambda = alpha^2.(n + κ) - n
 * @endcode
 *
 * **Weights:**
 * @code
 *   Wm0 = lambda/(n+lambda),           Wc0 = lambda/(n+lambda) + (1 - alpha^2 + beta)
 *   Wm_i = Wc_i = 1/(2(n+lambda))   i = 1...2n
 * @endcode
 *
 * **Recommended tuning:** alpha = 1e-3, beta = 2 (Gaussian prior), κ = 0.
 *
 * @par Alpha sensitivity for large state dimensions
 * For n >= 4, alpha = 1e-3 causes Wc0 to become large and negative, amplifying numerical noise.
 * Increase alpha toward 0.1-1.0 for larger state spaces. Setting alpha = \sqrt((κ + n) / n) gives Wc0 = 0.
 * The eigenvalue floor in update() is a backstop, not a substitute for a well-tuned alpha.
 *
 * @see Wan & Van der Merwe, "The UKF for Nonlinear Estimation", Proc. IEEE ASSPCC (2000).
 * @see Julier & Uhlmann, "A New Extension of the Kalman Filter" (1997).
 */

namespace ctrl
{

/**
 * @brief Discrete-time Unscented Kalman Filter.
 */
class UnscentedKalmanFilter
{
public:
    /**
     * @brief Construct the UKF with nonlinear models, noise statistics, and UT parameters.
     * @param n       State dimension.
     * @param p       Measurement dimension.
     * @param f       Process function x[k+1] = f(x[k], u[k]).
     * @param h       Measurement function y[k] = h(x[k], u[k]).
     * @param Q_noise Process noise covariance (n * n, positive semi-definite).
     * @param R_noise Measurement noise covariance (p * p, positive definite).
     * @param Ts      Sample time [s].
     * @param P0      Initial error covariance (n * n). Defaults to identity when empty.
     * @param alpha   Spread of sigma points around the mean (default 1e-3; increase to 0.1-1.0 for n >= 4).
     * @param beta    Prior distribution shape parameter (default 2.0 for Gaussian priors - encodes
     *                fourth-order kurtosis via Wc0 += 1 - alpha^2 + beta; set beta = 0 for unknown distributions).
     * @param kappa   Tertiary scaling parameter (default 0.0; use 3 - n to recover the minimum-variance
     *                unbiased estimator for Gaussian priors when n <= 3; negative lambda = alpha^2(n+κ)-n is
     *                permitted but can push sigma points outside the covariance ellipsoid).
     */
    UnscentedKalmanFilter(int n, int p,
                          std::function<Eigen::VectorXd(const Eigen::VectorXd &,
                                                        const Eigen::VectorXd &)> f,
                          std::function<Eigen::VectorXd(const Eigen::VectorXd &,
                                                        const Eigen::VectorXd &)> h,
                          const Eigen::MatrixXd &Q_noise,
                          const Eigen::MatrixXd &R_noise,
                          double Ts,
                          const Eigen::MatrixXd &P0 = Eigen::MatrixXd(),
                          double alpha = 1e-3,
                          double beta  = 2.0,
                          double kappa = 0.0);

    /**
     * @brief Prediction step - propagate sigma points through f and compute predicted mean/covariance.
     * @param u Control input u[k].
     */
    void predict(const Eigen::VectorXd &u);

    /**
     * @brief Update step - propagate predicted sigma points through h and compute Kalman gain.
     * @param y Measurement y[k].
     * @param u Control input u[k] (used in measurement function h).
     */
    void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u);

    /**
     * @brief Combined predict + update - most common usage.
     * @param y      Measurement y[k].
     * @param u_prev Control input u[k-1] (predict step).
     */
    void step(const Eigen::VectorXd &y, const Eigen::VectorXd &u_prev);

    /** @brief Reset state estimate to zero and covariance to P0 (or identity). */
    void reset();

    /**
     * @brief Inject an initial state estimate.
     * @param x0 Initial state vector (n * 1).
     */
    void setState(const Eigen::VectorXd &x0) { x_hat_ = x0; }

    /** @brief Current state estimate x^[k|k] (n * 1). */
    const Eigen::VectorXd &state()      const { return x_hat_; }

    /** @brief Current error covariance P[k|k] (n * n). */
    const Eigen::MatrixXd &covariance() const { return P_; }

    /** @brief Sample time Ts [s]. */
    double sampleTime()                 const { return Ts_; }

private:
    /**
     * @brief Compute 2n+1 sigma points from the current state estimate and covariance.
     *
     * Writes into the pre-allocated mutable workspace @c sigma_pts_ws_ and returns a
     * const reference to it - no per-call heap allocation in steady state.
     *
     * @return Const reference to an n * (2n+1) matrix; each column is a sigma point.
     *         The reference is valid until the next call to sigmaPoints().
     */
    const Eigen::MatrixXd& sigmaPoints() const;

    int n_states_, n_outputs_;
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> f_, h_;
    Eigen::MatrixXd Q_, R_;
    Eigen::VectorXd x_hat_;
    Eigen::MatrixXd P_;
    double Ts_;

    mutable Eigen::MatrixXd sigma_pts_ws_; ///< Pre-allocated n * (2n+1) sigma-point workspace.

    double lambda_;          ///< UT scaling parameter lambda = alpha^2.(n+κ) - n.
    Eigen::VectorXd Wm_, Wc_; ///< Mean and covariance weights (2n+1), precomputed at construction.
};

} // namespace ctrl
