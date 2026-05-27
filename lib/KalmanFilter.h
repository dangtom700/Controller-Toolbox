#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>
#include <optional>

/**
 * @file KalmanFilter.h
 * @brief Discrete-time Linear Kalman Filter (LKF).
 *
 * Optimal state estimator for linear, time-invariant plants with Gaussian noise.
 *
 * **Plant model:**
 * @code
 *   x[k+1] = A·x[k] + B·u[k] + w[k],   w ~ N(0, Q_noise)
 *   y[k]   = C·x[k] + D·u[k] + v[k],   v ~ N(0, R_noise)
 * @endcode
 *
 * **Predict step** (given control input u[k−1]):
 * @code
 *   x̂[k|k−1] = A·x̂[k−1|k−1] + B·u[k−1]
 *   P[k|k−1]  = A·P[k−1|k−1]·Aᵀ + Q
 * @endcode
 *
 * **Update step** (given measurement y[k]):
 * @code
 *   S[k]    = C·P[k|k−1]·Cᵀ + R
 *   K[k]    = P[k|k−1]·Cᵀ·S⁻¹
 *   x̂[k|k]  = x̂[k|k−1] + K·(y[k] − C·x̂[k|k−1] − D·u[k])
 *   P[k|k]  = (I − K·C)·P[k|k−1]           (Joseph form for numerical stability)
 * @endcode
 *
 * @par Tuning Q and R — NEES consistency test
 * Run the filter in simulation alongside the true state and compute the Normalised Estimation
 * Error Squared (NEES) at each step:
 * @code
 *   ε[k] = (x_true[k] − x̂[k|k])ᵀ · P[k|k]⁻¹ · (x_true[k] − x̂[k|k])
 * @endcode
 * A consistent filter has E[ε[k]] ≈ n (state dimension). Diagnostic rules:
 * - E[ε] >> n → filter overconfident: Q too small or R too large (increase Q or R).
 * - E[ε] << n → filter underconfident: Q too large or R too small (decrease Q or R).
 * - E[ε] ≈ n but P diverges → plant–model mismatch; verify A, B, C matrices.
 *
 * @see Bar-Shalom, Li & Kirubarajan, "Estimation with Applications" §5.4 (2001).
 * @see Kalman, "A New Approach to Linear Filtering" (1960).
 * @see MATLAB kalman(), kalmd(); Simulink Kalman Filter block.
 */

namespace ctrl
{

/**
 * @brief Discrete-time Linear Kalman Filter.
 */
class KalmanFilter
{
public:
    /**
     * @brief Construct the Kalman filter.
     * @param plant   Linear discrete-time model (A, B, C, D, Ts).
     * @param Q_noise Process noise covariance (n × n, positive semi-definite).
     * @param R_noise Measurement noise covariance (p × p, positive definite).
     * @param P0      Initial error covariance (n × n). Defaults to identity when empty.
     */
    KalmanFilter(const StateSpace &plant,
                 const Eigen::MatrixXd &Q_noise,
                 const Eigen::MatrixXd &R_noise,
                 const Eigen::MatrixXd &P0 = Eigen::MatrixXd());

    /**
     * @brief Prediction step — advance state estimate with the previous control input.
     * @param u Control input u[k−1] applied at the last sample.
     */
    void predict(const Eigen::VectorXd &u);

    /**
     * @brief Update step — incorporate a new measurement.
     * @param y         Measurement vector y[k].
     * @param u_current Control input u[k] applied at the current sample (used in D·u feedthrough).
     */
    void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u_current);

    /**
     * @brief Combined predict + update — most common usage pattern.
     *
     * Equivalent to calling predict(u_prev) followed by update(y, u_current).
     *
     * @param y         Measurement vector y[k].
     * @param u_prev    Control input u[k−1] (predict step).
     * @param u_current Control input u[k] (update step, D·u feedthrough).
     *                  Defaults to nullopt, which falls back to u_prev — correct for D = 0 plants.
     *                  For D ≠ 0, pass the actual current input to avoid a one-step feedthrough bias.
     */
    void step(const Eigen::VectorXd &y,
              const Eigen::VectorXd &u_prev,
              std::optional<std::reference_wrapper<const Eigen::VectorXd>> u_current = std::nullopt);

    /** @brief Reset state estimate to zero and covariance to P0 (or identity). */
    void reset();

    /** @brief Current state estimate x̂[k|k] (n × 1). */
    const Eigen::VectorXd &state() const { return x_hat_; }

    /** @brief Current error covariance P[k|k] (n × n). */
    const Eigen::MatrixXd &covariance() const { return P_; }

    /** @brief Sample time Ts [s]. */
    double sampleTime() const { return Ts_; }

private:
    StateSpace plant_;
    Eigen::MatrixXd Q_, R_;
    Eigen::VectorXd x_hat_; ///< State estimate x̂[k|k].
    Eigen::MatrixXd P_;     ///< Error covariance P[k|k].
    double Ts_;
};

} // namespace ctrl
