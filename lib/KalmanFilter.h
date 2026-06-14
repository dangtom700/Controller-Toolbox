#pragma once
#include "PlantModel.h"
#include "MismatchDetector.h"
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
 *   x[k+1] = A.x[k] + B.u[k] + w[k],   w ~ N(0, Q_noise)
 *   y[k]   = C.x[k] + D.u[k] + v[k],   v ~ N(0, R_noise)
 * @endcode
 *
 * **Predict step** (given control input u[k-1]):
 * @code
 *   x^[k|k-1] = A.x^[k-1|k-1] + B.u[k-1]
 *   P[k|k-1]  = A.P[k-1|k-1].Aᵀ + Q
 * @endcode
 *
 * **Update step** (given measurement y[k]):
 * @code
 *   S[k]    = C.P[k|k-1].Cᵀ + R
 *   K[k]    = P[k|k-1].Cᵀ.S^-^1
 *   x^[k|k]  = x^[k|k-1] + K.(y[k] - C.x^[k|k-1] - D.u[k])
 *   P[k|k]  = (I - K.C).P[k|k-1]           (Joseph form for numerical stability)
 * @endcode
 *
 * @par Tuning Q and R - NEES consistency test
 * Run the filter in simulation alongside the true state and compute the Normalised Estimation
 * Error Squared (NEES) at each step:
 * @code
 *   epsilon[k] = (x_true[k] - x^[k|k])ᵀ . P[k|k]^-^1 . (x_true[k] - x^[k|k])
 * @endcode
 * A consistent filter has E[epsilon[k]] approx = n (state dimension). Diagnostic rules:
 * - E[epsilon] >> n -> filter overconfident: Q too small or R too large (increase Q or R).
 * - E[epsilon] << n -> filter underconfident: Q too large or R too small (decrease Q or R).
 * - E[epsilon] approx = n but P diverges -> plant-model mismatch; verify A, B, C matrices.
 *
 * @see Bar-Shalom, Li & Kirubarajan, "Estimation with Applications" Section 5.4 (2001).
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
     * @param Q_noise Process noise covariance (n * n, positive semi-definite).
     * @param R_noise Measurement noise covariance (p * p, positive definite).
     * @param P0      Initial error covariance (n * n). Defaults to identity when empty.
     */
    KalmanFilter(const StateSpace &plant,
                 const Eigen::MatrixXd &Q_noise,
                 const Eigen::MatrixXd &R_noise,
                 const Eigen::MatrixXd &P0 = Eigen::MatrixXd());

    /**
     * @brief Prediction step - advance state estimate with the previous control input.
     * @param u Control input u[k-1] applied at the last sample.
     */
    void predict(const Eigen::VectorXd &u);

    /**
     * @brief Update step - incorporate a new measurement.
     * @param y         Measurement vector y[k].
     * @param u_current Control input u[k] applied at the current sample (used in D.u feedthrough).
     */
    void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u_current);

    /**
     * @brief Combined predict + update - most common usage pattern.
     *
     * Equivalent to calling predict(u_prev) followed by update(y, u_current).
     *
     * @param y         Measurement vector y[k].
     * @param u_prev    Control input u[k-1] (predict step).
     * @param u_current Control input u[k] (update step, D.u feedthrough).
     *                  Defaults to nullopt, which falls back to u_prev - correct for D = 0 plants.
     *                  For D != 0, pass the actual current input to avoid a one-step feedthrough bias.
     */
    void step(const Eigen::VectorXd &y,
              const Eigen::VectorXd &u_prev,
              std::optional<std::reference_wrapper<const Eigen::VectorXd>> u_current = std::nullopt);

    /**
     * @brief Plain-reference overload of step() - preferred from Python bindings.
     *
     * Equivalent to step(y, u_prev, std::cref(u_current)).  Exists because
     * std::optional<std::reference_wrapper<...>> is not directly bindable by pybind11.
     *
     * @param y         Measurement vector y[k].
     * @param u_prev    Control input u[k-1] (predict step).
     * @param u_current Control input u[k]   (update step, D.u feedthrough).
     */
    void step(const Eigen::VectorXd &y,
              const Eigen::VectorXd &u_prev,
              const Eigen::VectorXd &u_current);

    /** @brief Reset state estimate to zero and covariance to P0 (or identity). */
    void reset();

    /** @brief Current state estimate x^[k|k] (n * 1). */
    const Eigen::VectorXd &state() const { return x_hat_; }

    /** @brief Current error covariance P[k|k] (n * n). */
    const Eigen::MatrixXd &covariance() const { return P_; }

    /** @brief Sample time Ts [s]. */
    double sampleTime() const { return Ts_; }

    /**
     * @brief Enable real-time model-plant mismatch detection on the innovation sequence (D1).
     *
     * After enabling, every @c update() / @c step() call feeds the current innovation
     * `y - C*x_pred - D*u` into a CUSUM chart.  Tune @p params.sigma to the expected
     * innovation RMS when the model is correct (see MismatchDetector.h for guidance).
     *
     * @param params  CUSUM parameters (sigma, k_cusum, h_threshold).
     */
    void enableMismatchDetection(const MismatchDetectorParams& params = {})
    {
        mismatch_det_.emplace(params);
    }

    /**
     * @brief @c true if the CUSUM statistic has exceeded the detection threshold.
     *
     * Always @c false when @c enableMismatchDetection() has not been called.
     * Remains @c true until the detector is reset or re-enabled.
     */
    bool mismatchDetected() const
    {
        return mismatch_det_.has_value() && mismatch_det_->detected();
    }

    /**
     * @brief Current CUSUM score: max(C+, C-) from the innovation CUSUM chart.
     *
     * Returns 0.0 when detection is not enabled.  Compare against
     * `params.h_threshold * params.sigma` for a threshold-relative reading.
     */
    double mismatchScore() const
    {
        return mismatch_det_.has_value() ? mismatch_det_->score() : 0.0;
    }

    /**
     * @brief Reset mismatch detector accumulators (does not disable detection).
     *
     * Useful after a parameter re-estimation event to clear false-alarm history.
     */
    void resetMismatchDetector()
    {
        if (mismatch_det_) mismatch_det_->reset();
    }

private:
    StateSpace plant_;
    Eigen::MatrixXd Q_, R_;
    Eigen::VectorXd x_hat_; ///< State estimate x^[k|k].
    Eigen::MatrixXd P_;     ///< Error covariance P[k|k].
    double Ts_;

    // Pre-allocated update() workspaces - eliminates four heap allocs per update call.
    Eigen::MatrixXd R_safe_; ///< R with minimum noise floor applied.
    Eigen::MatrixXd S_;      ///< Innovation covariance (p x p).
    Eigen::MatrixXd Kf_;     ///< Kalman gain (n x p).
    Eigen::MatrixXd IKC_;    ///< (I - Kf*C) (n x n).
    Eigen::MatrixXd P_new_;  ///< Scratch for Joseph-form P update (avoids P_=f(P_) aliasing).

    std::optional<MismatchDetector> mismatch_det_;  ///< D1: optional innovation CUSUM.
};

} // namespace ctrl
