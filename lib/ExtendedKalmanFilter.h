#pragma once
#include <Eigen/Dense>
#include <functional>

/**
 * @file ExtendedKalmanFilter.h
 * @brief Discrete-time Extended Kalman Filter (EKF) for nonlinear systems.
 *
 * Handles nonlinear process and measurement models with **additive** Gaussian noise:
 * @code
 *   x[k+1] = f(x[k], u[k]) + w[k],   w ~ N(0, Q_noise)
 *   y[k]   = h(x[k], u[k]) + v[k],   v ~ N(0, R_noise)
 * @endcode
 *
 * @par Noise model assumption
 * Process and measurement noise are **additive** as shown above. For multiplicative or
 * state-dependent noise (e.g., gyroscope scale-factor noise, Poisson photon counts),
 * the EKF must be reformulated with augmented state and noise channels - that form is
 * **not** implemented here.
 *
 * The EKF linearises around the current estimate at each step:
 * @code
 *   F[k] = df/dx |_(x^[k|k], u[k])     - state Jacobian (n*n)
 *   H[k] = dh/dx |_(x^[k+1|k], u[k])   - observation Jacobian (p*n)
 * @endcode
 *
 * **Predict:**
 * @code
 *   x^[k+1|k]  = f(x^[k|k], u[k])
 *   P[k+1|k]   = F[k].P[k|k].F[k]ᵀ + Q
 * @endcode
 *
 * **Update (Joseph form):**
 * @code
 *   S            = H[k].P[k+1|k].H[k]ᵀ + R
 *   K            = P[k+1|k].H[k]ᵀ.S^-^1
 *   x^[k+1|k+1] = x^[k+1|k] + K.(y - h(x^[k+1|k], u[k]))
 *   P[k+1|k+1]  = (I - K.H).P.(I - K.H)ᵀ + K.R.Kᵀ
 * @endcode
 *
 * Jacobians may be analytical (preferred) or computed numerically via numericalJacobian().
 *
 * @see Jazwinski, "Stochastic Processes and Filtering Theory" (1970).
 * @see Bar-Shalom et al., "Estimation with Applications to Tracking" (2001).
 */

namespace ctrl
{

/** @brief Function type for the nonlinear process model: x[k+1] = f(x[k], u[k]). */
using StateFunc  = std::function<Eigen::VectorXd(const Eigen::VectorXd &x,
                                                  const Eigen::VectorXd &u)>;

/** @brief Function type for the nonlinear measurement model: y[k] = h(x[k], u[k]). */
using MeasFunc   = std::function<Eigen::VectorXd(const Eigen::VectorXd &x,
                                                  const Eigen::VectorXd &u)>;

/** @brief Function type for a Jacobian: returns an (n*n) or (p*n) matrix given (x, u). */
using JacobianFn = std::function<Eigen::MatrixXd(const Eigen::VectorXd &x,
                                                  const Eigen::VectorXd &u)>;

/**
 * @brief Discrete-time Extended Kalman Filter.
 */
class ExtendedKalmanFilter
{
public:
    /**
     * @brief Construct the EKF with nonlinear models and noise statistics.
     * @param n       State dimension.
     * @param p       Measurement dimension.
     * @param f       Process function x[k+1] = f(x[k], u[k]).
     * @param h       Measurement function y[k] = h(x[k], u[k]).
     * @param Fjac    Jacobian df/dx (n * n), evaluated at (x^, u).
     * @param Hjac    Jacobian dh/dx (p * n), evaluated at (x^_pred, u).
     * @param Q_noise Process noise covariance (n * n, positive semi-definite).
     * @param R_noise Measurement noise covariance (p * p, positive definite).
     * @param Ts      Sample time [s].
     * @param P0      Initial error covariance (n * n). Defaults to identity when empty.
     */
    ExtendedKalmanFilter(int n, int p,
                         StateFunc f, MeasFunc h,
                         JacobianFn Fjac, JacobianFn Hjac,
                         const Eigen::MatrixXd &Q_noise,
                         const Eigen::MatrixXd &R_noise,
                         double Ts,
                         const Eigen::MatrixXd &P0 = Eigen::MatrixXd());

    /**
     * @brief Prediction step - propagate state through f and inflate covariance with F.
     * @param u Control input u[k] applied at the current sample.
     */
    void predict(const Eigen::VectorXd &u);

    /**
     * @brief Update step - incorporate measurement y[k].
     * @param y Current measurement y[k].
     * @param u Control input u[k] (used in innovation h(x^, u)).
     */
    void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u);

    /**
     * @brief Combined predict + update - most common usage.
     * @param y      Measurement y[k].
     * @param u_prev Control input u[k-1] applied at the previous sample.
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

    /**
     * @brief Central-difference numerical Jacobian with state-scaled step size.
     *
     * Per-element step: h_i = eps_scale . max(|x_i|, 1). This avoids near-zero absolute
     * steps for small-magnitude states and cancellation errors for large-magnitude states.
     * For heterogeneous state vectors (e.g., [position_m, velocity_m/s, angle_rad]) the
     * fixed-epsilon approach degrades significantly; this formulation is robust across scales.
     *
     * Use when analytical Jacobians are unavailable or too complex to maintain.
     * Bind u as a constant to obtain df/dx or dh/dx:
     * @code
     *   auto Fjac = [&](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
     *       return ExtendedKalmanFilter::numericalJacobian(
     *           [&](const Eigen::VectorXd& xx){ return f(xx, u); }, x);
     *   };
     * @endcode
     *
     * @param func      Scalar-to-vector function to differentiate.
     * @param x         Point at which to evaluate the Jacobian.
     * @param eps_scale Relative step scale (default 1e-4; absolute floor is eps_scale itself
     *                  when |x_i| < 1). Equivalent to the old fixed-eps when all |x_i| approx = 1.
     * @return Jacobian matrix (output_dim * input_dim).
     */
    static Eigen::MatrixXd numericalJacobian(
        const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &func,
        const Eigen::VectorXd &x,
        double eps_scale = 1e-4);

private:
    int n_, p_;
    StateFunc  f_;
    MeasFunc   h_;
    JacobianFn Fjac_, Hjac_;
    Eigen::MatrixXd Q_, R_;
    Eigen::VectorXd x_hat_; ///< State estimate x^[k|k].
    Eigen::MatrixXd P_;     ///< Error covariance P[k|k].
    double Ts_;
};

} // namespace ctrl
