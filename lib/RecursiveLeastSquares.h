#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>

/**
 * @file RecursiveLeastSquares.h
 * @brief Online SISO ARX system identification via Recursive Least Squares (RLS).
 *
 * Identifies a discrete-time ARX model from input-output data:
 * @code
 *   A(q)y[k] = B(q)u[k] + e[k]
 *   y[k] = −a₁·y[k−1] − … − aₙₐ·y[k−nₐ]
 *          + b₁·u[k−1] + … + bₙᵦ·u[k−nᵦ] + e[k]
 * @endcode
 *
 * **Regressor:** φ[k] = [−y[k−1], …, −y[k−nₐ], u[k−1], …, u[k−nᵦ]]ᵀ ∈ ℝ^(nₐ+nᵦ)
 *
 * **Recursive update (exponential forgetting via scalar λ):**
 * @code
 *   K[k]     = P[k−1]·φ[k] / (λ + φ[k]ᵀ·P[k−1]·φ[k])
 *   e[k]     = y[k] − φ[k]ᵀ·θ[k−1]           (a-posteriori prediction error)
 *   θ[k]     = θ[k−1] + K[k]·e[k]
 *   P[k]     = (P[k−1] − K[k]·φ[k]ᵀ·P[k−1]) / λ
 * @endcode
 *
 * After sufficient excitation, call toTransferFunction() or toStateSpace() to extract
 * a model for use with DiscreteMPC, DiscreteLQG, etc.
 *
 * @see Åström & Wittenmark, "Adaptive Control" (1995) Ch. 2.
 * @see Ljung, "System Identification" (1999) §11.2.
 */

namespace ctrl
{

/**
 * @brief Online SISO ARX system identifier using Recursive Least Squares.
 */
class RecursiveLeastSquares
{
public:
    /**
     * @brief Construct the RLS estimator.
     * @param na       Number of output (A-polynomial) lags.
     * @param nb       Number of input  (B-polynomial) lags (delay starts at k−1).
     * @param Ts       Sample time [s].
     * @param lambda   Forgetting factor λ ∈ (0, 1].
     *                 - λ = 1: standard least squares (weights all data equally).
     *                 - λ < 1: exponential forgetting; effective window ≈ 1/(1−λ) samples.
     *                 - Typical range: 0.95–0.99 for slowly time-varying plants.
     * @param P0_scale Initial covariance = P0_scale·I. Large value = high initial uncertainty.
     */
    RecursiveLeastSquares(int na, int nb, double Ts,
                          double lambda   = 0.98,
                          double P0_scale = 1e4);

    /**
     * @brief Feed one new I/O sample and update the parameter estimate.
     *
     * Fills internal circular buffers for y and u, then runs the RLS recursion.
     * Updates are reliable once the buffer has been filled: after max(nₐ, nᵦ) calls.
     *
     * @param y Plant output sample y[k].
     * @param u Control input sample u[k].
     * @return A-posteriori prediction error e[k] = y[k] − φ[k]ᵀ·θ[k−1].
     */
    double update(double y, double u);

    /** @brief Reset circular buffers, parameter estimate, and covariance. Restarts identification. */
    void reset();

    /** @brief Current parameter estimate θ = [a₁,…,aₙₐ, b₁,…,bₙᵦ]ᵀ. */
    const Eigen::VectorXd &params() const { return theta_; }

    /** @brief Monic denominator polynomial [1, a₁, …, aₙₐ] (length nₐ + 1). */
    Eigen::VectorXd denominator() const;

    /** @brief Numerator polynomial [b₁, …, bₙᵦ] (length nᵦ). */
    Eigen::VectorXd numerator() const;

    /**
     * @brief Build a TransferFunction from the current estimate.
     *
     * Valid only after the regressor buffer has been filled (k ≥ max(nₐ, nᵦ)).
     * @return Discrete-time TransferFunction in z⁻¹ form.
     */
    TransferFunction toTransferFunction() const;

    /**
     * @brief Build a minimal StateSpace model from the current estimate.
     * @return Discrete-time StateSpace in controllable canonical form.
     */
    StateSpace toStateSpace() const;

    /**
     * @brief Covariance matrix P ((nₐ+nᵦ) × (nₐ+nᵦ)).
     *
     * trace(P) is a scalar measure of parameter uncertainty; decreasing trace indicates
     * converging estimates. Large trace after many samples suggests insufficient excitation.
     */
    const Eigen::MatrixXd &covariance() const { return P_; }

    /** @brief Number of samples processed since construction or the last reset(). */
    int sampleCount() const { return k_; }

    /** @brief Sample time Ts [s]. */
    double sampleTime() const { return Ts_; }

    /** @brief Forgetting factor λ. */
    double forgettingFactor() const { return lambda_; }

private:
    int na_, nb_, ntheta_;
    double Ts_, lambda_, P0_scale_;
    Eigen::VectorXd theta_; ///< Parameter estimate θ.
    Eigen::MatrixXd P_;     ///< Covariance matrix P.
    Eigen::VectorXd y_buf_; ///< Circular buffer for y[k−1…k−nₐ].
    Eigen::VectorXd u_buf_; ///< Circular buffer for u[k−1…k−nᵦ].
    int k_;                 ///< Sample counter (also tracks buffer fill).
};

} // namespace ctrl
