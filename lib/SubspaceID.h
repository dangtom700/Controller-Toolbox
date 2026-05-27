#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>
#include <optional>

/**
 * @file SubspaceID.h
 * @brief Batch subspace state-space system identification (N4SID / MOESP).
 *
 * Identifies a linear discrete-time state-space model from batch I/O data:
 * @code
 *   x[k+1] = A.x[k] + B.u[k] + w[k]
 *   y[k]   = C.x[k] + D.u[k] + v[k]
 * @endcode
 *
 * **Algorithm - MOESP oblique-projection variant (Verhaegen & Dewilde 1992):**
 * 1. Build 2i-block Hankel matrices from (Y, U): Z = [Uf; Wp; Yf], Wp = [Up; Yp].
 * 2. Thin LQ decomposition isolates the (3,2) block L32 (oblique projection of Yf onto Wp \perp Uf).
 * 3. SVD(L32): singular values reveal system order; left singular vectors form the
 *    extended observability matrix Gamma = U[:, :n].\sqrt(S[:n]).
 * 4. Extract C = Gamma[:p, :], A from shift-invariance of Gamma (least squares).
 * 5. Least-squares regression for B and D from the state sequence.
 *
 * @par Data requirements
 * - Persistent excitation: input must excite at least n_order frequencies.
 * - N ≫ 2.i.(m+p): more samples -> better estimates.
 * - i >= n_order/p (i_horizon must be large enough to capture all modes).
 *
 * @par Similarity transform note
 * The returned realisation (A, B, C, D) is determined only up to a similarity transform T.
 * **Similarity-invariant** (trustworthy): eigenvalues of A, I/O transfer function G(z),
 * DC gain, stability. **Not invariant**: individual entries of B, C, D, state norms.
 * For Kalman filter use, prefer the returned kalmanGain (already in the identified basis)
 * over manual Q/R tuning.
 *
 * @see Van Overschee & De Moor, "Subspace Identification for Linear Systems" (Kluwer 1996).
 * @see Verhaegen, "Identification of the Deterministic Part of MIMO State Space Models",
 *      Automatica (1994).
 */

namespace ctrl
{

/**
 * @brief Output of the n4sid() identification algorithm.
 */
struct SubspaceIDResult
{
    std::optional<StateSpace> model;        ///< Identified A, B, C, D, Ts; empty on failure.
    Eigen::VectorXd  singularValues;        ///< All singular values of L32 (for order selection).

    /**
     * @brief Estimated Kalman gain (n * p).
     *
     * Kalman gain K such that x[k+1] = A.x[k] + B.u[k] + K.epsilon[k].
     * Pass directly to KalmanFilter or DiscreteLQG to avoid manual Q/R tuning.
     * Empty on failure.
     */
    Eigen::MatrixXd  kalmanGain;

    /**
     * @brief Innovation covariance Lambda = E[epsilon.epsilonᵀ] (p * p).
     *
     * Use as R_noise in KalmanFilter for noise-matched estimation. Empty on failure.
     */
    Eigen::MatrixXd  innovCov;

    bool        success = false; ///< @c true when identification succeeded.
    std::string message;         ///< Error description on failure.
};

/**
 * @brief Batch subspace state-space identification (N4SID / MOESP).
 *
 * Plot result.singularValues to choose n_order: look for an elbow where values drop sharply
 * (signal vs. noise subspace boundary). Alternatively, use suggestOrder().
 *
 * @param Y         Output data matrix (p * N): rows = outputs, columns = time samples.
 * @param U         Input data matrix  (m * N): rows = inputs,  columns = time samples.
 * @param n_order   Desired model order n. Choose by inspecting singularValues or suggestOrder().
 * @param i_horizon Block-row count in Hankel matrices. Recommend i >= 2.n_order/p, minimum n_order+1.
 * @param Ts        Sample time [s].
 * @param svd_tol   SVD truncation tolerance; if > 0, n_order is capped at the number of
 *                  singular values exceeding svd_tol. Pass -1.0 to disable (default).
 * @return SubspaceIDResult containing the model, singular values, and diagnostics.
 *         Check result.success before using result.model.
 */
SubspaceIDResult n4sid(const Eigen::MatrixXd &Y,
                       const Eigen::MatrixXd &U,
                       int n_order,
                       int i_horizon,
                       double Ts,
                       double svd_tol = -1.0);

/**
 * @brief Automated system-order selection from the singular-value spectrum.
 *
 * Uses the maximum-consecutive-ratio heuristic: finds the index i* where sv(i*)/sv(i*+1)
 * is largest (sharpest drop) and returns i*+1 as the suggested order. A secondary threshold
 * guard caps the result at the first index where sv(i)/sv(0) falls below @p threshold.
 *
 * Use after n4sid() to eliminate the manual elbow-inspection step in adaptive loops
 * (e.g., RLS -> n4sid -> GPC).
 *
 * @param sv        SubspaceIDResult::singularValues from a prior n4sid() call.
 * @param threshold Relative floor; singular values below threshold.sv(0) are treated as noise.
 *                  Set to 0 to disable the threshold guard.
 * @param maxOrder  Hard cap on the returned order (-1 = no cap).
 * @return Suggested model order >= 1.
 */
int suggestOrder(const Eigen::VectorXd &sv,
                 double threshold = 0.01,
                 int maxOrder     = -1);

} // namespace ctrl
