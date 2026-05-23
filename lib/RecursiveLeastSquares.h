#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>

// Discrete-time Recursive Least Squares (RLS) — SISO ARX system identification.
//
// Identifies a discrete-time ARX model online from I/O data:
//   A(q)y[k] = B(q)u[k] + e[k]
//   y[k] = -a1.y[k-1] - ... - ana.y[k-na]
//          + b1.u[k-1] + ... + bnb.u[k-nb]  + e[k]
//
// Regressor:  φ[k] = [-y[k-1],...,-y[k-na], u[k-1],...,u[k-nb]]'  (na+nb × 1)
// Parameters: θ     = [a1,...,ana, b1,...,bnb]'
//
// Recursive update (directional forgetting via scalar λ):
//   K[k] = P[k-1].φ[k] / (λ + φ[k]'.P[k-1].φ[k])
//   e[k] = y[k] - φ[k]'.θ[k-1]       (a-posteriori prediction error)
//   θ[k] = θ[k-1] + K[k].e[k]
//   P[k] = (P[k-1] - K[k].φ[k]'.P[k-1]) / λ
//
// Forgetting factor λ ∈ (0,1]:
//   λ = 1    → standard least squares (weights all data equally)
//   λ < 1    → exponential forgetting; effective window ~ 1/(1-λ) samples
//   Typical: λ = 0.95..0.99 for slowly time-varying plants.
//
// After sufficient excitation, call toTransferFunction() or toStateSpace()
// to extract a model for use with DiscreteMPC, DiscreteLQG, etc.
//
// Ref: Åström & Wittenmark "Adaptive Control" (1995) Ch.2;
//      Ljung "System Identification" (1999) §11.2.
namespace ctrl
{

class RecursiveLeastSquares
{
public:
    // na:     number of output (A-polynomial) lags
    // nb:     number of input  (B-polynomial) lags (delay starts at k-1)
    // Ts:     sample time [s]
    // lambda: forgetting factor ∈ (0,1] (default = 0.98)
    // P0_scale: initial covariance = P0_scale * I  (large → high initial uncertainty)
    RecursiveLeastSquares(int na, int nb, double Ts,
                          double lambda  = 0.98,
                          double P0_scale = 1e4);

    // Feed one new sample (y[k], u[k]) and update the estimate.
    // Returns the current prediction error e[k] = y[k] - φ'.θ_prev.
    double update(double y, double u);

    // Reset buffers and covariance; restart identification.
    void reset();

    // Current parameter estimate θ = [a1,...,ana, b1,...,bnb]'.
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

    // Covariance matrix P (na+nb × na+nb); trace → parameter uncertainty.
    const Eigen::MatrixXd &covariance() const { return P_; }

    // Number of samples processed since construction / last reset.
    int sampleCount() const { return k_; }

    double sampleTime() const { return Ts_; }
    double forgettingFactor() const { return lambda_; }

private:
    int na_, nb_, ntheta_;
    double Ts_, lambda_;
    Eigen::VectorXd theta_;     // parameter estimate
    Eigen::MatrixXd P_;         // covariance
    Eigen::VectorXd y_buf_;     // circular buffer for y[k-1..k-na]
    Eigen::VectorXd u_buf_;     // circular buffer for u[k-1..k-nb]
    int k_;                     // sample counter (also tracks buffer fill)
};

} // namespace ctrl
