#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>

// Discrete-time Recursive Least Squares (RLS) - SISO ARX system identification.
//
// Identifies a discrete-time ARX model online from I/O data:
//   A(q)y[k] = B(q)u[k] + e[k]
//   y[k] = -a1.y[k-1] - ... - ana.y[k-na]
//          + b1.u[k-1] + ... + bnb.u[k-nb]  + e[k]
//
// Regressor:  phi[k] = [-y[k-1],...,-y[k-na], u[k-1],...,u[k-nb]]'  (na+nb * 1)
// Parameters: theta     = [a1,...,ana, b1,...,bnb]'
//
// Recursive update (directional forgetting via scalar lambda):
//   K[k] = P[k-1].phi[k] / (lambda + phi[k]'.P[k-1].phi[k])
//   e[k] = y[k] - phi[k]'.theta[k-1]       (a-posteriori prediction error)
//   theta[k] = theta[k-1] + K[k].e[k]
//   P[k] = (P[k-1] - K[k].phi[k]'.P[k-1]) / lambda
//
// Forgetting factor lambda \in (0,1]:
//   lambda = 1    -> standard least squares (weights all data equally)
//   lambda < 1    -> exponential forgetting; effective window ~ 1/(1-lambda) samples
//   Typical: lambda = 0.95..0.99 for slowly time-varying plants.
//
// After sufficient excitation, call toTransferFunction() or toStateSpace()
// to extract a model for use with DiscreteMPC, DiscreteLQG, etc.
//
// Ref: Astrom & Wittenmark "Adaptive Control" (1995) Ch.2;
//      Ljung "System Identification" (1999) Section 11.2.
namespace ctrl
{

class RecursiveLeastSquares
{
public:
    // na:     number of output (A-polynomial) lags
    // nb:     number of input  (B-polynomial) lags (delay starts at k-1)
    // Ts:     sample time [s]
    // lambda: forgetting factor \in (0,1] (default = 0.98)
    // P0_scale: initial covariance = P0_scale * I  (large -> high initial uncertainty)
    RecursiveLeastSquares(int na, int nb, double Ts,
                          double lambda  = 0.98,
                          double P0_scale = 1e4);

    // Feed one new sample (y[k], u[k]) and update the estimate.
    // Returns the current prediction error e[k] = y[k] - phi'.theta_prev.
    double update(double y, double u);

    // Reset buffers and covariance; restart identification.
    void reset();

    // Current parameter estimate theta = [a1,...,ana, b1,...,bnb]'.
    const Eigen::VectorXd &params() const { return theta_; }

    // Denominator polynomial [1, a1,...,ana] (monic, length na+1).
    Eigen::VectorXd denominator() const;

    // Numerator polynomial [b1,...,bnb] (length nb).
    Eigen::VectorXd numerator() const;

    // Build a TransferFunction from the current estimate.
    // Valid only after the regressor buffer has been filled (k >= max(na,nb)).
    TransferFunction toTransferFunction() const;

    // Build a minimal StateSpace model from the current estimate.
    StateSpace toStateSpace() const;

    // Covariance matrix P (na+nb * na+nb); trace -> parameter uncertainty.
    const Eigen::MatrixXd &covariance() const { return P_; }

    // Number of samples processed since construction / last reset.
    int sampleCount() const { return k_; }

    double sampleTime() const { return Ts_; }
    double forgettingFactor() const { return lambda_; }

private:
    int na_, nb_, ntheta_;
    double Ts_, lambda_, P0_scale_; // P0_scale_ cached so reset() is idempotent
    Eigen::VectorXd theta_;     // parameter estimate
    Eigen::MatrixXd P_;         // covariance
    Eigen::VectorXd y_buf_;     // circular buffer for y[k-1..k-na]
    Eigen::VectorXd u_buf_;     // circular buffer for u[k-1..k-nb]
    int k_;                     // sample counter (also tracks buffer fill)
};

} // namespace ctrl
