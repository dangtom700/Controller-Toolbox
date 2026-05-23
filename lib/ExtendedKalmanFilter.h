#pragma once
#include <Eigen/Dense>
#include <functional>

// Discrete-time Extended Kalman Filter (EKF).
//
// Handles nonlinear systems:
//   x[k+1] = f(x[k], u[k]) + w[k],   w ~ N(0, Q_noise)
//   y[k]   = h(x[k], u[k]) + v[k],   v ~ N(0, R_noise)
//
// EKF linearises around the current estimate at each step:
//   F[k] = ∂f/∂x |_(x^[k|k], u[k])    - state Jacobian  (n*n)
//   H[k] = ∂h/∂x |_(x^[k+1|k], u[k])  - observation Jacobian (p*n)
//
// Predict:
//   x^[k+1|k]   = f(x^[k|k], u[k])
//   P[k+1|k]    = F[k].P[k|k].F[k]' + Q
//
// Update (Joseph form for numerical stability):
//   S            = H[k].P[k+1|k].H[k]' + R
//   K            = P[k+1|k].H[k]'.S^-1
//   x^[k+1|k+1]  = x^[k+1|k] + K.(y - h(x^[k+1|k], u[k]))
//   P[k+1|k+1]  = (I - K.H).P.(I - K.H)' + K.R.K'
//
// Jacobians can be analytical (preferred) or numerical via numericalJacobian().
//
// Ref: Jazwinski "Stochastic Processes and Filtering Theory" (1970);
//      Bar-Shalom et al. "Estimation with Applications to Tracking" (2001).
namespace ctrl
{

using StateFunc  = std::function<Eigen::VectorXd(const Eigen::VectorXd &x,
                                                  const Eigen::VectorXd &u)>;
using MeasFunc   = std::function<Eigen::VectorXd(const Eigen::VectorXd &x,
                                                  const Eigen::VectorXd &u)>;
using JacobianFn = std::function<Eigen::MatrixXd(const Eigen::VectorXd &x,
                                                  const Eigen::VectorXd &u)>;

class ExtendedKalmanFilter
{
public:
    // n:       state dimension
    // p:       measurement dimension
    // f:       process function  x[k+1] = f(x[k], u[k])
    // h:       measurement function  y[k] = h(x[k], u[k])
    // Fjac:    Jacobian ∂f/∂x (n*n) evaluated at (x^, u)
    // Hjac:    Jacobian ∂h/∂x (p*n) evaluated at (x^_pred, u)
    // Q_noise: process noise covariance (n*n, PSD)
    // R_noise: measurement noise covariance (p*p, PD)
    // Ts:      sample time [s]
    // P0:      initial error covariance (n*n, default = I)
    ExtendedKalmanFilter(int n, int p,
                         StateFunc f, MeasFunc h,
                         JacobianFn Fjac, JacobianFn Hjac,
                         const Eigen::MatrixXd &Q_noise,
                         const Eigen::MatrixXd &R_noise,
                         double Ts,
                         const Eigen::MatrixXd &P0 = Eigen::MatrixXd());

    // Predict: propagate state with f, inflate P using linearised F.
    void predict(const Eigen::VectorXd &u);

    // Update: incorporate measurement y[k] and current input u[k].
    void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u);

    // Combined predict + update (most common usage).
    void step(const Eigen::VectorXd &y, const Eigen::VectorXd &u_prev);

    void reset();

    void setState(const Eigen::VectorXd &x0) { x_hat_ = x0; }

    const Eigen::VectorXd &state()      const { return x_hat_; }
    const Eigen::MatrixXd &covariance() const { return P_; }
    double sampleTime()                 const { return Ts_; }

    // Central-difference numerical Jacobian (eps = 1e-5).
    // Bind u as a constant to get the ∂f/∂x or ∂h/∂x you need:
    //   auto Fjac = [&](auto& x, auto& u) {
    //       return EKF::numericalJacobian([&](auto& xx){ return f(xx,u); }, x);
    //   };
    static Eigen::MatrixXd numericalJacobian(
        const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &func,
        const Eigen::VectorXd &x,
        double eps = 1e-5);

private:
    int n_, p_;
    StateFunc  f_;
    MeasFunc   h_;
    JacobianFn Fjac_, Hjac_;
    Eigen::MatrixXd Q_, R_;
    Eigen::VectorXd x_hat_; // x^[k|k]
    Eigen::MatrixXd P_;     // P[k|k]
    double Ts_;
};

} // namespace ctrl
