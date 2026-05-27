#pragma once
#include "DiscreteLQR.h"
#include "KalmanFilter.h"
#include <Eigen/Dense>
#include <memory>

/**
 * @file DiscreteLQG.h
 * @brief Discrete-time Linear Quadratic Gaussian (LQG) Controller.
 *
 * Combines DiscreteLQR (optimal state-feedback gain) with KalmanFilter (optimal state estimator)
 * via the **separation principle**: the LQR and Kalman filter can be designed independently
 * and combined without loss of optimality (Wonham 1968).
 *
 * **Control law per step:**
 * 1. Kalman predict: x^[k|k-1] from u[k-1]
 * 2. Kalman update: x^[k|k] from y[k]
 * 3. LQR control: u[k] = -K.(x^[k|k] - x_ref)
 *
 * @par Scope limitation
 * The separation principle holds exactly only for linear systems with additive Gaussian noise.
 * For nonlinear plants, this class linearises the plant model at construction time; stability
 * and performance guarantees apply only near the linearisation point. For wider operating
 * regions, consider periodic re-linearisation or using ExtendedKalmanFilter + DiscreteLQR
 * as separate objects to recompute Jacobians at each step.
 *
 * @warning compute(y) takes the raw plant output **not** the error.
 *          Call setReference() before compute() for IController-compatible usage,
 *          or use the full step() method directly.
 *
 * @see Athans (1971), "Role of Decision Theory in Systems Engineering".
 * @see MATLAB lqg(), kalman(); Astrom, "Introduction to Stochastic Control".
 */

namespace ctrl
{

/**
 * @brief Discrete-time LQG controller (LQR + Kalman filter).
 */
class DiscreteLQG
{
public:
    /**
     * @brief Construct the LQG controller.
     *
     * @param plant   Discrete-time model (A, B, C, D, Ts). If D != 0, a warning is printed:
     *                the Kalman innovation uses u[k-1] for the D.u feedthrough term (one step stale).
     *                Use D = 0 for noise-free feedthrough accuracy.
     * @param lqr_p   LQR cost weights Q (state) and R (control).
     * @param Q_noise Process noise covariance (n * n, positive semi-definite).
     * @param R_noise Measurement noise covariance (p * p, positive definite).
     * @param P0      Initial Kalman error covariance (n * n). Defaults to identity when empty.
     */
    DiscreteLQG(const StateSpace &plant,
                const LQRParams &lqr_p,
                const Eigen::MatrixXd &Q_noise,
                const Eigen::MatrixXd &R_noise,
                const Eigen::MatrixXd &P0 = Eigen::MatrixXd());

    /**
     * @brief Full step: update Kalman filter with (y, u_prev), then apply LQR toward x_ref.
     *
     * @param y     Plant output measurement y[k] (p * 1).
     * @param u_prev Control input applied at the previous step u[k-1] (m * 1).
     * @param x_ref  Reference state x_ref[k] (n * 1). Pass empty for regulation to origin.
     * @return Control action u[k] (m * 1).
     */
    Eigen::VectorXd step(const Eigen::VectorXd &y,
                         const Eigen::VectorXd &u_prev,
                         const Eigen::VectorXd &x_ref = Eigen::VectorXd());

    /**
     * @brief SISO convenience: returns the first control component.
     *
     * Requires setReference() and setUPrev() to be called before each invocation,
     * or use the full step() method instead.
     *
     * @param y_scalar Scalar plant output y[k].
     * @return First element of the control vector u[k].
     */
    double compute(double y_scalar);

    /**
     * @brief Set the reference state for the next compute() call (IController compat).
     * @param x_ref Reference state vector (n * 1).
     */
    void setReference(const Eigen::VectorXd &x_ref) { x_ref_ = x_ref; }

    /**
     * @brief Set the previous control input for the next compute() call (IController compat).
     * @param u Previous control input u[k-1] (m * 1).
     */
    void setUPrev(const Eigen::VectorXd &u) { u_prev_ = u; }

    /** @brief Reset Kalman filter state to zero and covariance to P0. */
    void reset();

    /** @brief Sample time Ts [s]. */
    double sampleTime() const { return lqr_->sampleTime(); }

    /** @brief Current Kalman state estimate x^[k|k]. */
    const Eigen::VectorXd &stateEstimate() const { return kf_->state(); }

    /** @brief Optimal LQR feedback gain matrix K* (m * n). */
    const Eigen::MatrixXd &gainMatrix()    const { return lqr_->gainMatrix(); }

private:
    std::unique_ptr<DiscreteLQR>    lqr_;
    std::unique_ptr<KalmanFilter>   kf_;
    Eigen::VectorXd x_ref_;
    Eigen::VectorXd u_prev_;
    StateSpace plant_;
};

} // namespace ctrl
