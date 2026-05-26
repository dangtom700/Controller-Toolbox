#pragma once
#include <Eigen/Dense>
#include <functional>

// Discrete-time Unscented Kalman Filter (UKF).
//
// Handles nonlinear systems WITHOUT requiring Jacobians:
//   x[k+1] = f(x[k], u[k]) + w[k],   w ~ N(0, Q_noise)
//   y[k]   = h(x[k], u[k]) + v[k],   v ~ N(0, R_noise)
//
// NOISE MODEL ASSUMPTION: process and measurement noise are ADDITIVE (as above).
// For multiplicative or state-dependent noise (e.g. gyroscope scale-factor noise,
// Poisson photon counts), the correct formulation augments the state vector with the
// noise channels and propagates augmented sigma points.  That form is NOT implemented
// here.  If your system requires it, augment the state manually before constructing
// the filter.
//
// Uses the Unscented Transform: propagates 2n+1 deterministically chosen
// sigma points through the exact nonlinear functions to capture mean and
// covariance to third order (vs. EKF's first-order linearisation).
//
// Sigma points (scaled UT, Wan & Van der Merwe 2000):
//   X_0     = x^
//   X_i     = x^ + (sqrt((n+lambda).P))_i      i = 1..n
//   X_{n+i} = x^ - (sqrt((n+lambda).P))_i      i = 1..n
//   lambda = alpha^2.(n + kappa) - n
//
// Weights:
//   Wm_0 = lambda/(n+lambda),           Wc_0 = lambda/(n+lambda) + (1 - alpha^2 + beta)
//   Wm_i = Wc_i = 1/(2(n+lambda))  i = 1..2n
//
// Recommended tuning: alpha=1e-3, beta=2 (Gaussian prior), kappa=0.
//
// Alpha sensitivity for large state dimensions:
//   alpha = 1e-3 is the Wan & Van der Merwe default, appropriate for n <= 3.
//   For larger n, the zeroth covariance weight Wc_0 = lambda/(n+lambda) + (1-alpha^2+beta)
//   becomes large and negative (e.g., Wc_0 ~ -6e6 for n=6 with alpha=1e-3).
//   This is mathematically valid but amplifies numerical noise in the covariance update.
//   For n >= 4, increase alpha toward 0.1..1.0 to keep |Wc_0| within [-n, 0]:
//     alpha = sqrt((kappa + n) / n)  gives Wc_0 = 0 (minimum noise amplification).
//   The eigenvalue floor (1e-10*trace(P)) in update() is a backstop, not a substitute
//   for a well-tuned alpha. Verify Wc_0 is reasonable after construction.
//
// Ref: Wan & Van der Merwe, "The Unscented Kalman Filter for Nonlinear
//      Estimation", Proc. IEEE ASSPCC 2000;
//      Julier & Uhlmann, "A New Extension of the Kalman Filter" (1997).
namespace ctrl
{

class UnscentedKalmanFilter
{
public:
    // n:       state dimension
    // p:       measurement dimension
    // f:       process function  x[k+1] = f(x[k], u[k])
    // h:       measurement function  y[k] = h(x[k], u[k])
    // Q_noise: process noise covariance (n*n, PSD)
    // R_noise: measurement noise covariance (p*p, PD)
    // Ts:      sample time [s]
    // P0:      initial error covariance (n*n, default = I)
    // alpha:   spread of sigma points around mean (typically 1e-3)
    // beta:    prior distribution parameter (2 for Gaussian)
    // kappa:   secondary scaling (0 or 3-n)
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

    // Predict: propagate sigma points through f, compute predicted mean/cov.
    void predict(const Eigen::VectorXd &u);

    // Update: propagate predicted sigmas through h, compute Kalman gain.
    void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u);

    // Combined predict + update.
    void step(const Eigen::VectorXd &y, const Eigen::VectorXd &u_prev);

    void reset();

    void setState(const Eigen::VectorXd &x0) { x_hat_ = x0; }

    const Eigen::VectorXd &state()      const { return x_hat_; }
    const Eigen::MatrixXd &covariance() const { return P_; }
    double sampleTime()                 const { return Ts_; }

private:
    // Compute 2n+1 sigma points from current x^ and P.
    // Returns matrix of size n x (2n+1); each column is a sigma point.
    Eigen::MatrixXd sigmaPoints() const;

    int n_, p_;
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> f_, h_;
    Eigen::MatrixXd Q_, R_;
    Eigen::VectorXd x_hat_;
    Eigen::MatrixXd P_;
    double Ts_;

    // UT weights (pre-computed at construction)
    double lambda_;
    Eigen::VectorXd Wm_, Wc_; // (2n+1) weight vectors
};

} // namespace ctrl
