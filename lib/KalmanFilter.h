#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>

// Discrete-time Linear Kalman Filter (LKF).
//
// Plant model (linear, discrete):
//   x[k+1] = A.x[k] + B.u[k] + w[k],   w ~ N(0, Q_noise)
//   y[k]   = C.x[k] + D.u[k] + v[k],   v ~ N(0, R_noise)
//
// Predict step (called with control input u[k-1]):
//   x^[k|k-1]  = A.x^[k-1|k-1] + B.u[k-1]
//   P[k|k-1]   = A.P[k-1|k-1].A' + Q
//
// Update step (called with measurement y[k]):
//   S[k]        = C.P[k|k-1].C' + R
//   K_kf[k]     = P[k|k-1].C'.S^-¹
//   x^[k|k]     = x^[k|k-1] + K_kf.(y[k] - C.x^[k|k-1] - D.u[k])
//   P[k|k]      = (I - K_kf.C).P[k|k-1]   (Joseph form for numerical stability)
//
// Combined one-call step: step(y, u_prev) = predict(u_prev) + update(y)
//
// Ref: Kalman "A New Approach to Linear Filtering" (1960);
//      MATLAB kalman(), kalmd(); Simulink Kalman Filter block.
namespace ctrl
{

    class KalmanFilter
    {
    public:
        // plant:   linear discrete-time model (A,B,C,D)
        // Q_noise: process noise covariance  (n*n, positive semi-definite)
        // R_noise: measurement noise covariance (p*p, positive definite)
        // P0:      initial error covariance (n*n, default = I)
        KalmanFilter(const StateSpace &plant,
                     const Eigen::MatrixXd &Q_noise,
                     const Eigen::MatrixXd &R_noise,
                     const Eigen::MatrixXd &P0 = Eigen::MatrixXd());

        // Predict: advance state estimate with control input u[k-1].
        void predict(const Eigen::VectorXd &u);

        // Update: incorporate measurement y[k] and current input u[k].
        void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u_current);

        // Combined predict + update (most common usage pattern).
        void step(const Eigen::VectorXd &y,
                  const Eigen::VectorXd &u_prev);

        void reset();

        const Eigen::VectorXd &state() const { return x_hat_; }
        const Eigen::MatrixXd &covariance() const { return P_; }
        double sampleTime() const { return Ts_; }

    private:
        StateSpace plant_;
        Eigen::MatrixXd Q_, R_;
        Eigen::VectorXd x_hat_; // x^[k|k]
        Eigen::MatrixXd P_;     // P[k|k]
        double Ts_;
    };

} // namespace ctrl
