#pragma once
#include "ControllerRegistry.h"
#include <Eigen/Dense>

/**
 * @file MLEIdentifier.h
 * @brief Maximum Likelihood / MAP batch ARX identification (Phase 3 Roadmap Phase 2 SI1).
 *
 * Statistical alternative to RecursiveLeastSquares's pure least-squares cost: maximizes
 * log-likelihood under an assumed noise model (Gaussian by default, optionally MAP with a
 * Gaussian prior on theta). Reduces exactly to batch least squares in the Gaussian/no-prior
 * case, but generalizes to heavier-tailed (Laplace) noise where plain LS is biased by outliers.
 *
 * Uses the same `phi[k] = [-y[k-1..k-na], u[k-1..k-nb]]` regressor convention as
 * `RecursiveLeastSquares`, but as a **batch** (offline) fit - not a streaming estimator.
 *
 * @see Astrom & Wittenmark, "Adaptive Control", 2nd ed. (1995), Ch. 2 (the LS/MLE equivalence
 *      under Gaussian noise this class generalizes away from).
 * @see docs/superpowers/specs/2026-06-25-adaptive-identification-design.md
 */

namespace ctrl
{

/** @brief Assumed measurement-noise distribution for MLEIdentifier. */
enum class NoiseModel
{
    Gaussian, ///< Reduces exactly to least squares (no prior).
    Laplace   ///< Robust to outliers (heavier-tailed than Gaussian).
};

/** @brief Parameters for MLEIdentifier::fit. */
struct MLEParams
{
    int na = 2; ///< Output (A-polynomial) lag order.
    int nb = 1; ///< Input (B-polynomial) lag order.

    NoiseModel noise = NoiseModel::Gaussian;

    Eigen::VectorXd prior_mean; ///< MAP prior mean (empty = pure MLE, no prior).
    Eigen::MatrixXd prior_cov;  ///< MAP prior covariance (empty = pure MLE, no prior).

    int max_iter = 300;  ///< Forwarded to AutoTuner::maxIter (CMA-ES generations).
    double tol = 1e-10;  ///< Forwarded to AutoTuner::tol.
};

/** @brief Result of MLEIdentifier::fit. */
struct MLEResult
{
    Eigen::VectorXd theta;      ///< [a1..a_na, b1..b_nb], RecursiveLeastSquares-identical layout.
    Eigen::MatrixXd covariance; ///< Asymptotic parameter covariance (inverse-Hessian) at theta.
    double logLikelihood = 0.0;
    bool converged = false;
};

/**
 * @brief Batch Maximum Likelihood / MAP ARX identifier.
 */
class MLEIdentifier
{
public:
    /**
     * @brief Fit an ARX model to (u, y) data by maximizing the assumed-noise-model likelihood.
     * @param u Input sequence.
     * @param y Output sequence (same length as u).
     * @param Ts Sample time [s] (informational; not used in the fit itself).
     * @param params Fit configuration.
     * @throws std::invalid_argument if u.size() != y.size(), or too few samples for na/nb.
     */
    static MLEResult fit(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                          double Ts, const MLEParams &params = {});
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(mle_identifier)
