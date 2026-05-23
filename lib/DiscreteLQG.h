#pragma once
#include "DiscreteLQR.h"
#include "KalmanFilter.h"
#include <Eigen/Dense>
#include <memory>

// Discrete-time Linear Quadratic Gaussian (LQG) Controller.
//
// Combines:
//   - DiscreteLQR  - optimal state-feedback gain K (solved offline via DARE)
//   - KalmanFilter - optimal state estimator (handles noisy / partial observations)
//
// Separation principle: the LQR and Kalman filter can be designed independently
// and combined without loss of optimality (Wonham 1968).
//
// Control law:
//   1. Kalman predict:  x^[k|k-1] from u[k-1]
//   2. Kalman update:   x^[k|k]   from y[k]
//   3. LQR control:     u[k]      = -K.(x^[k|k] - x_ref)
//
// step(y, u_prev, x_ref) performs all three in the correct order.
//
// IMPORTANT: compute(y) takes the raw plant output (NOT the error).
//            Use setReference(x_ref) before calling compute(y) for IController compat.
//
// Ref: Athans (1971) "Role of Decision Theory in Systems Engineering";
//      MATLAB lqg(), kalman(); Åström "Introduction to Stochastic Control".
namespace ctrl
{

    class DiscreteLQG
    {
    public:
        // plant:   discrete-time model (A,B,C,D,Ts).
        //          If D != 0, a warning is printed: the Kalman innovation uses
        //          u[k-1] for the D*u term (one step stale). Use D = 0 for accuracy.
        // lqr_p:   LQR cost weights (Q,R)
        // Q_noise: process noise covariance (n*n)
        // R_noise: measurement noise covariance (p*p)
        // P0:      initial Kalman error covariance (defaults to I)
        DiscreteLQG(const StateSpace &plant,
                    const LQRParams &lqr_p,
                    const Eigen::MatrixXd &Q_noise,
                    const Eigen::MatrixXd &R_noise,
                    const Eigen::MatrixXd &P0 = Eigen::MatrixXd());

        // Full step: update KF with (y, u_prev), return LQR control toward x_ref.
        // x_ref: reference state (empty = zero reference).
        Eigen::VectorXd step(const Eigen::VectorXd &y,
                             const Eigen::VectorXd &u_prev,
                             const Eigen::VectorXd &x_ref = Eigen::VectorXd());

        // SISO convenience: returns first control component.
        // Requires setReference() + setUPrev() before calling, or use step().
        double compute(double y_scalar);
        void setReference(const Eigen::VectorXd &x_ref) { x_ref_ = x_ref; }
        void setUPrev(const Eigen::VectorXd &u) { u_prev_ = u; }

        void reset();
        double sampleTime() const { return lqr_->sampleTime(); }

        const Eigen::VectorXd &stateEstimate() const { return kf_->state(); }
        const Eigen::MatrixXd &gainMatrix() const { return lqr_->gainMatrix(); }

    private:
        std::unique_ptr<DiscreteLQR> lqr_;
        std::unique_ptr<KalmanFilter> kf_;
        Eigen::VectorXd x_ref_;
        Eigen::VectorXd u_prev_;
        StateSpace plant_;
    };

} // namespace ctrl
