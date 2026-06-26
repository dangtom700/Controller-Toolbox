#pragma once
#include "Features.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

/**
 * @file NARMAXIdentifier.h
 * @brief Polynomial NARMAX identification via Orthogonal Forward Regression (Phase 3 SI4).
 *
 * Fits `y[k] = f(y[k-1..k-na], u[k-1..k-nb], e[k-1..k-nc]) + e[k]` where `f` is a polynomial
 * expansion (up to `poly_degree`) over the *lagged* input/output/noise terms, with model
 * structure selected by Orthogonal Forward Regression using the Error Reduction Ratio (ERR)
 * - the standard NARMAX approach (Billings & Korenberg).
 *
 * Distinct from `SINDy`, which expands a library over state *derivatives*; NARMAX expands over
 * lagged I/O terms and selects a parsimonious subset by ERR.
 *
 * **Noise (MA) terms:** when `nc > 0`, lagged-residual regressors `e[k-i]` are handled by
 * Extended Least Squares - an initial NARX fit produces a residual series, which then enters
 * the candidate library for a second OFR pass (one Billings-style refinement iteration).
 *
 * @see Billings, "Nonlinear System Identification: NARMAX Methods" (2013), Ch. 3-4.
 * @see SINDy.h (sparse-regression alternative, derivative library).
 * @see docs/ALGORITHM_ROADMAP_PHASE3.md (SI4)
 */

namespace ctrl {

/** @brief Parameters for NARMAXIdentifier. */
struct NARMAXParams
{
    int    na = 1;                  ///< Output (autoregressive) lag order.
    int    nb = 1;                  ///< Input lag order.
    int    nc = 0;                  ///< Noise (MA) lag order (0 = pure NARX).
    int    poly_degree = 2;         ///< Maximum monomial total degree over the lagged variables.
    double significance_tol = 0.01; ///< Stop when cumulative ERR >= 1 - tol.
    int    max_terms = 20;          ///< Hard cap on selected terms (parsimony / safety).
};

/** @brief Result of a NARMAX fit. */
struct NARMAXResult
{
    std::vector<std::string> selected_terms; ///< Human-readable selected regressors, e.g. "y(k-1)*u(k-2)".
    Eigen::VectorXd          coefficients;    ///< Least-squares coefficient per selected term.
    double                   final_err_sum = 0.0; ///< Cumulative ERR of the selected terms in [0,1].

    // --- Encoding used by predict() (each selected term is a multiset of base-regressor indices) ---
    std::vector<std::vector<int>> term_factors; ///< Base-regressor indices per selected term ({} = constant).
    int na = 0, nb = 0, nc = 0;                  ///< Lag orders this model was fitted with.
};

/**
 * @brief Polynomial NARMAX identifier (static fit + one-step prediction).
 */
class NARMAXIdentifier
{
public:
    /**
     * @brief Fit a NARMAX model to an input/output record.
     * @param u      Input series (length N).
     * @param y      Output series (length N, same length as u).
     * @param params Lag orders, polynomial degree, and OFR stopping rule.
     * @return Selected term set, coefficients, and cumulative ERR.
     * @throws std::invalid_argument If u and y differ in length, the record is too short for
     *         the requested lags, any order is negative, poly_degree < 1, or the candidate
     *         library would exceed the internal safety cap (lower the degree/orders).
     */
    static NARMAXResult fit(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                            const NARMAXParams &params);

    /**
     * @brief One-step-ahead prediction y_hat[k] from past I/O.
     *
     * Noise (e-lag) terms are evaluated as zero (their future residuals are unknown), the
     * standard one-step-ahead convention.
     *
     * @param model  A fitted NARMAXResult.
     * @param u_hist Past inputs, most recent last (size >= nb); u_hist.back() = u[k-1].
     * @param y_hist Past outputs, most recent last (size >= na); y_hist.back() = y[k-1].
     * @return Predicted y[k].
     * @throws std::invalid_argument If history is shorter than the model's lag orders.
     */
    static double predict(const NARMAXResult &model, const Eigen::VectorXd &u_hist,
                          const Eigen::VectorXd &y_hist);
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(narmax)
