#pragma once
#include "GreyBoxEstimator.h"
#include "UnscentedKalmanFilter.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <memory>

namespace ctrl {

/**
 * @brief Online parameter tracking via augmented-state UKF (E2).
 *
 * Augments the plant state x (n_x) with the parameter vector p (n_p):
 *   z = [x (n_x); p (n_p)]
 *
 * Augmented dynamics (used as UKF process model):
 *   z_next = [RK4(f(x, u, p), Ts);  p]   -- p held constant + Q_param diffusion
 *
 * Tuning guide:
 * - Q_state: process noise for the physical states (typical sensor/actuator noise level).
 * - Q_param: how fast parameters can change.  Small = slow, stable tracking; large = fast
 *   but noisy.  Start at 1e-6 * I and increase if the plant is drifting.
 * - R_meas: measurement noise covariance.
 * - P0_param: initial parameter uncertainty.  Empty defaults to 100 * Q_param.
 *
 * @note Requires CTRL_HAS_ADVANCED_KALMAN (compiled in by default).
 *       For one-shot batch fitting, see GreyBoxEstimator.
 *
 * @par Typical usage
 * @code
 *   ctrl::RecursiveGreyBoxEstimator::Params par;
 *   par.p0 = Eigen::Vector2d(1.0, 0.3);  // initial [k, c] guess
 *   par.Q_state = 1e-4 * Eigen::MatrixXd::Identity(2, 2);
 *   par.Q_param = 1e-6 * Eigen::MatrixXd::Identity(2, 2);
 *   par.R_meas  = 0.01 * Eigen::MatrixXd::Identity(1, 1);
 *   par.Ts      = 0.01;
 *   ctrl::RecursiveGreyBoxEstimator rge(ode, meas, 2, 1, par);
 *   rge.initialize(x0, 0.1 * Eigen::MatrixXd::Identity(2, 2));
 *   for (int k = 0; k < N; ++k) {
 *       Eigen::VectorXd x_hat = rge.step(y[k], u[k]);
 *       Eigen::VectorXd p_hat = rge.paramEstimate();
 *   }
 * @endcode
 */
class RecursiveGreyBoxEstimator {
public:
    using OdeFn  = GreyBoxEstimator::OdeFn;
    using MeasFn = GreyBoxEstimator::MeasFn;

    struct Params {
        Eigen::VectorXd p0;          ///< Initial parameter estimate (n_p x 1)
        Eigen::MatrixXd Q_state;     ///< Process noise for state (n_x x n_x)
        Eigen::MatrixXd Q_param;     ///< Process noise for params (n_p x n_p)
        Eigen::MatrixXd R_meas;      ///< Measurement noise (n_y x n_y)
        Eigen::MatrixXd P0_param;    ///< Initial param covariance (n_p x n_p); empty -> 100*Q_param
        double Ts         = 0.01;    ///< Sample time [s]
        int    rk4_steps  = 1;       ///< RK4 substeps per Ts
        double alpha      = 0.1;     ///< UKF sigma-point spread (0.1 good for n_aug >= 4)
        double beta       = 2.0;     ///< UKF prior-shape parameter
        double kappa      = 0.0;     ///< UKF tertiary scaling
    };

    /**
     * @brief Construct estimator. Call initialize() before step().
     * @param ode     Continuous-time ODE f(x, u, p) -> xdot.
     * @param meas    Measurement function h(x, p) -> y.
     * @param n_state Plant state dimension n_x.
     * @param n_meas  Measurement dimension n_y.
     * @param par     Estimator parameters including p0.
     */
    RecursiveGreyBoxEstimator(OdeFn ode, MeasFn meas,
                               int n_state, int n_meas,
                               const Params& par);

    /**
     * @brief Set initial state and (re-)build the augmented UKF.
     * @param x0       Initial state estimate (n_x x 1).
     * @param P0_state Initial state error covariance (n_x x n_x).
     */
    void initialize(const Eigen::VectorXd& x0,
                    const Eigen::MatrixXd& P0_state);

    /**
     * @brief Run one UKF predict + update step.
     * @param y  Measurement y[k] (n_y x 1).
     * @param u  Control input u[k] (n_u x 1).
     * @return   Updated state estimate x^[k|k] (n_x x 1).
     */
    Eigen::VectorXd step(const Eigen::VectorXd& y,
                         const Eigen::VectorXd& u);

    /** @brief Current state estimate x^[k|k] (n_x x 1). */
    Eigen::VectorXd stateEstimate() const;

    /** @brief Current parameter estimate p^[k|k] (n_p x 1). */
    Eigen::VectorXd paramEstimate() const;

    /** @brief Full augmented covariance P_aug[k|k] (n_aug x n_aug). */
    const Eigen::MatrixXd& covariance() const;

    /** @brief True after initialize() has been called. */
    bool isInitialized() const { return initialized_; }

private:
    OdeFn   f_;
    MeasFn  h_;
    Params  par_;
    int     n_state_, n_meas_, n_param_;
    bool    initialized_ = false;

    std::unique_ptr<UnscentedKalmanFilter> ukf_;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(recursive_grey_box)
