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
 * **Algorithm - MOESP oblique-projection (Verhaegen & Dewilde 1992), 6 steps:**
 * @code
 *   INPUT  : Y (p*N), U (m*N), n (order), i (block rows), Ts
 *   OUTPUT : A (n*n), B (n*m), C (p*n), D (p*m), K_kalman (n*p), Lambda (p*p)
 *   s = N - 2.i    (Hankel column count)
 *
 *   Step 1 - Build Hankel matrices and partition past / future
 *     Yp = Y_hankel[:i.p, :]        (past outputs,   i.p * s)
 *     Yf = Y_hankel[i.p:, :]        (future outputs, i.p * s)
 *     Up, Uf  similarly from U.
 *     Wp = [Up; Yp]                  (past I/O,   i.(m+p) * s)
 *     Z  = [Uf; Wp; Yf]             (stacked,    (r_Uf+r_Wp+r_Yf) * s)
 *
 *   Step 2 - LQ decomposition  ->  oblique projection L32
 *     Z^T = Q . R^T   (Q column-orthonormal, R lower-triangular)
 *     L32 = R[r_Uf+r_Wp:, r_Uf : r_Uf+r_Wp]      (i.p * i.(m+p))
 *     Interpretation: L32 = oblique projection  Yf /_{Uf} Wp
 *                     (projects Yf onto row(Wp) along row(Uf))
 *                     Satisfies O_i . X_i = L32 . Q_Wp^T   (MOESP Eq. 4.3)
 *
 *   Step 3 - SVD of L32  ->  extended observability matrix Gamma
 *     [U, S, V^T] = svd(L32,  "thin")
 *     Gamma = U[:, :n] . diag(\sqrtS[:n])                 (i.p * n)
 *     Plot S to choose n: elbow = signal / noise boundary
 *     (suggestOrder() automates this via max-consecutive-ratio heuristic)
 *
 *   Step 4 - Extract C and A  (shift-invariance of Gamma)
 *     C  = Gamma[:p, :]                                (p * n)
 *     Gamma_up   = Gamma[:(i-1).p, :]
 *     Gamma_down = Gamma[p:,       :]
 *     A  = Gamma_up^+ . Gamma_down   (least squares)       (n * n)
 *
 *   Step 5 - Least-squares regression for B and D
 *     X^ = Gamma^+ . Yf                                 (state sequence, n * s)
 *     for k = 0 ... T-1  (T = s-1):
 *       D: solve  y[k] - C.x^[k]    = D.u[k]        (p * m)
 *       B: solve  x^[k+1] - A.x^[k] = B.u[k]        (n * m)
 *     (separate colPivHouseholderQr solves, batched over all T columns)
 *
 *   Step 6 - Stochastic realisation  (Kalman gain K, innovation cov Lambda)
 *     epsilon[k] = y[k]   - C.x^[k]   - D.u[k]   (innovation,    p * T)
 *     eta[k] = x^[k+1] - A.x^[k]  - B.u[k]   (state residual, n * T)
 *     Lambda = (1/T) . epsilon . epsilon^T                   (innovation cov, p * p)
 *     K = (eta . epsilon^T) . (epsilon . epsilon^T)^{-1}       (Kalman gain,    n * p)
 *     -> pass K to KalmanFilter::KalmanFilter(sys, Q, R) or
 *       DiscreteLQG to skip manual Q/R tuning.
 * @endcode
 *
 * @par Data requirements
 * - Persistent excitation: input must excite at least n_order frequencies.
 * - N >> 2.i.(m+p): more samples -> better estimates.
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
 * @brief Weighting variant for subspaceID().
 *
 * All three share the same Hankel/LQ/extraction pipeline n4sid() already uses; they differ
 * only in how the oblique projection L32 is weighted before its SVD (Step 3).
 */
enum class SubspaceMethod
{
    MOESP,  ///< Unweighted oblique projection (Verhaegen & Dewilde 1992). Identical to n4sid().
            ///< The strongest performer across every scenario tried in prototyping; default.
    N4SID,  ///< Right-weights the past block by its Uf-conditioned covariance (Cholesky-clean).
    CVA     ///< Additionally weights each output channel by an estimated noise scale.
};

/**
 * @brief Batch subspace identification with a choice of weighting (MOESP / N4SID / CVA).
 *
 * MOESP reproduces n4sid() exactly. N4SID right-weights the past data block by its
 * Uf-conditioned covariance. CVA additionally left-weights each *output channel* by a
 * noise-scale estimate (derived from the LQ factor's residual block). This is a regularized,
 * per-channel-scale variant, not full canonical-variate (cross-covariance) whitening: that
 * requires inverting an (i*p)x(i*p) matrix whose true rank is only n_order (<< i*p whenever
 * i_horizon > n_order, the normal operating regime), which is numerically ill-conditioned
 * even with ridge regularization (verified in prototyping -- see
 * docs/superpowers/specs/2026-06-25-subspace-id-variants-design.md).
 *
 * Verified via a 40-trial Monte Carlo on a synthetic mismatched-output-noise system: CVA
 * reliably outperforms N4SID (~85% win rate) when output channels have very different noise
 * levels, confirming the per-channel weighting fix over N4SID's right-weighting-only
 * shortcoming. However, neither N4SID nor CVA reliably outperforms unweighted MOESP on that
 * same system -- no method here is a universal winner (consistent with the subspace-ID
 * literature, where none of MOESP/N4SID/CVA dominates the others across all problem
 * instances either); MOESP is offered as the default for that reason.
 *
 * kalmanGain/innovCov are computed for all three methods, including MOESP -- the same
 * "free" diagnostic n4sid() already always populates, regardless of whether the chosen
 * method has a textbook stochastic step.
 *
 * @param Y         Output data matrix (p * N): rows = outputs, columns = time samples.
 * @param U         Input data matrix  (m * N): rows = inputs,  columns = time samples.
 * @param n_order   Desired model order n.
 * @param i_horizon Block-row count in Hankel matrices. Recommend i >= 2.n_order/p, minimum n_order+1.
 * @param Ts        Sample time [s].
 * @param method    Weighting variant. Defaults to MOESP (n4sid()'s existing behavior).
 * @param svd_tol   SVD truncation tolerance; if > 0, n_order is capped at the number of
 *                  singular values exceeding svd_tol. Pass -1.0 to disable (default).
 * @return SubspaceIDResult containing the model, singular values, and diagnostics.
 *         Check result.success before using result.model. success=false (with a
 *         descriptive message) on a near-singular weighting matrix (N4SID/CVA only,
 *         indicating non-persistent input excitation), in addition to n4sid()'s existing
 *         failure modes.
 */
SubspaceIDResult subspaceID(const Eigen::MatrixXd &Y,
                            const Eigen::MatrixXd &U,
                            int n_order,
                            int i_horizon,
                            double Ts,
                            SubspaceMethod method = SubspaceMethod::MOESP,
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
