#pragma once
#include "SystemAnalysis.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

/**
 * @file LyapunovRobustness.h
 * @brief Common quadratic Lyapunov function search for polytopic uncertainty.
 *
 * For a polytopic uncertain system `A(t) in conv{A_1, ..., A_L}` (the state matrix
 * switches or varies within the convex hull of vertex matrices), searches for a single
 * `P > 0` such that `V(x) = x^T P x` is a Lyapunov function for every vertex
 * simultaneously: `A_i^T P A_i - P < 0` for all i. If found, the system is
 * quadratically stable under arbitrary switching/variation among the vertices - a
 * stronger guarantee than each vertex being individually stable.
 *
 * Algorithm (heuristic averaging + projection, since the project has no SDP solver):
 *   1. For each vertex A_i, solve the *exact* discrete Lyapunov equation
 *      `A_i^T X_i A_i - X_i + Q = 0` via @ref SystemAnalysis::solveDiscreteLyapunov
 *      (this is the single-system Lyapunov matrix for A_i alone).
 *   2. Sum the X_i across vertices and project the result onto the positive-definite
 *      cone (eigenvalue floor).
 *   3. Verify the summed P against the actual multi-vertex inequality; report
 *      `found = true` only if every vertex's decrease condition holds with margin Q.
 *
 * Step 2 sums rather than averages: for a candidate `P = sum_i X_i`, checking vertex
 * `k` reduces to `A_k^T P A_k - P + Q = sum_{i != k} (A_k^T X_i A_k - X_i)` (the `i = k`
 * term cancels exactly against `Q` by construction). Averaging instead divides this
 * candidate down by `L`, which can - and in simple worked examples, provably does -
 * leave some vertex with a smaller margin than its own exact solution required, even
 * though every vertex individually admits a Lyapunov function. Summing keeps each
 * vertex's own exact margin intact and adds the (typically small, same-sign) cross
 * terms from the other nearby vertices on top.
 *
 * This is NOT a true SDP/LMI solver: summing individual Lyapunov solutions does not
 * guarantee a common Lyapunov function exists even when every vertex is individually
 * stable (simultaneous stability does not imply joint quadratic stability in general).
 * It works well for vertices clustered around a common nominal (the typical case for
 * small parametric uncertainty), which @ref buildBoxVertices is designed to produce.
 *
 * Phase 5 of the robustness-analysis roadmap (docs/robust_implementation_plan.md).
 *
 * @see SystemAnalysis.h
 */

namespace ctrl
{

/** @brief Parameters controlling the common-Lyapunov search. */
struct LyapunovSearchParams
{
    int    max_iter   = 200;  ///< Maximum summation/projection iterations.
    double tol        = 1e-6; ///< Fixed-point convergence: ||P_new - P_old||_F / ||P_old||_F < tol.
    double regularise = 1e-8; ///< Added to Q before each per-vertex solve and floors the PD projection.
};

/** @brief Result of a common-Lyapunov search. */
struct LyapunovResult
{
    bool            found      = false; ///< True if a common P satisfying every vertex was found.
    Eigen::MatrixXd P;                  ///< Candidate common Lyapunov matrix (n x n, symmetric).
    /**
     * @brief Worst-case (max over vertices) largest eigenvalue of `A_i^T P A_i - P + Q`.
     *
     * Negative (and more negative is better) means the decrease condition holds with
     * margin; non-negative means at least one vertex violates it (`found` is false).
     */
    double          residual   = 0.0;
    int             iterations = 0;     ///< Number of summation/projection iterations actually run.
};

/**
 * @brief Search for a common quadratic Lyapunov function over a polytope of vertices.
 *
 * @param vertices Vertex state matrices `A_1..A_L`, each n x n.
 * @param Q        Positive-definite decrease-margin matrix (defaults to identity).
 * @param p        Search parameters.
 * @return LyapunovResult; `found` is the authoritative pass/fail (the fixed-point
 *         iteration itself typically converges in 1-2 steps since the per-vertex
 *         solves do not depend on the running P - see the file header).
 * @throws std::invalid_argument If vertices is empty, vertices are not all square and
 *         the same size, or Q has the wrong dimensions.
 */
inline LyapunovResult findCommonLyapunov(const std::vector<Eigen::MatrixXd>& vertices,
                                          Eigen::MatrixXd Q = {},
                                          const LyapunovSearchParams& p = {})
{
    if (vertices.empty())
        throw std::invalid_argument("findCommonLyapunov: vertices must be non-empty.");

    const int n = static_cast<int>(vertices.front().rows());
    for (const auto& A : vertices)
        if (A.rows() != n || A.cols() != n)
            throw std::invalid_argument("findCommonLyapunov: all vertices must be square and the same size.");

    if (Q.size() == 0)
        Q = Eigen::MatrixXd::Identity(n, n);
    else if (Q.rows() != n || Q.cols() != n)
        throw std::invalid_argument("findCommonLyapunov: Q must be n x n (matching the vertices).");

    const Eigen::MatrixXd Qreg = Q + p.regularise * Eigen::MatrixXd::Identity(n, n);

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n, n);
    int iterations = 0;

    for (int iter = 0; iter < p.max_iter; ++iter)
    {
        ++iterations;

        // Sum (not average) the per-vertex exact solutions - see the file header for
        // why summing, not averaging, is what actually keeps every vertex's own margin.
        Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(n, n);
        for (const auto& A : vertices)
            // Solve A^T X A - X + Qreg = 0  <=>  A^T X A - X = -Qreg.
            P_new += SystemAnalysis::solveDiscreteLyapunov(A.transpose(), Qreg);

        // Project onto the symmetric positive-definite cone (eigenvalue floor).
        P_new = 0.5 * (P_new + P_new.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(P_new);
        P_new = eig.eigenvectors()
              * eig.eigenvalues().cwiseMax(p.regularise).asDiagonal()
              * eig.eigenvectors().transpose();

        const double denom = std::max(P.norm(), 1e-300);
        const double rel_change = (P_new - P).norm() / denom;
        P = P_new;
        if (rel_change < p.tol)
            break;
    }

    // Verify the actual multi-vertex decrease condition at the found P: every vertex
    // must satisfy A_i^T P A_i - P + Q negative semi-definite (margin Q > 0 makes this
    // strict). residual is the worst-case (largest) eigenvalue of that matrix across
    // vertices - negative means satisfied, non-negative means violated.
    double residual = -std::numeric_limits<double>::infinity();
    for (const auto& A : vertices)
    {
        Eigen::MatrixXd M = A.transpose() * P * A - P + Q;
        M = 0.5 * (M + M.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigM(M, Eigen::EigenvaluesOnly);
        residual = std::max(residual, eigM.eigenvalues().maxCoeff());
    }

    LyapunovResult result;
    result.P          = P;
    result.iterations = iterations;
    result.residual   = residual;
    result.found      = residual < std::sqrt(p.tol);
    return result;
}

/**
 * @brief Quadratic stability certificate for a polytope of vertices.
 *
 * Returns @c true iff every vertex is individually Schur-stable (necessary but not
 * sufficient) AND @ref findCommonLyapunov reports @c found for the same vertex set.
 */
inline bool isQuadraticallyStable(const std::vector<Eigen::MatrixXd>& vertices,
                                   const LyapunovSearchParams& p = {})
{
    for (const auto& A : vertices)
    {
        if (A.rows() != A.cols())
            throw std::invalid_argument("isQuadraticallyStable: vertices must be square.");
        Eigen::EigenSolver<Eigen::MatrixXd> es(A, /*computeEigenvectors=*/false);
        if (es.eigenvalues().cwiseAbs().maxCoeff() >= 1.0)
            return false;
    }
    return findCommonLyapunov(vertices, {}, p).found;
}

/**
 * @brief Build the 2^m vertex matrices of a box uncertainty around a nominal A.
 *
 * `vertices = {A_nominal + sum_j (+-1)*delta_j : j = 1..m}`, i.e. every combination of
 * adding/subtracting each perturbation direction. Each direction `delta_j` is supplied
 * as the j-th column of @p uncertainty_cols, flattened column-major (so
 * `uncertainty_cols` is `n*n` rows by `m` columns - reshape column j back to n x n to
 * recover `delta_j`).
 *
 * @param A_nominal       Nominal n x n state matrix.
 * @param uncertainty_cols n*n x m matrix; column j is a flattened n x n perturbation
 *                         direction.
 * @return Vector of `2^m` vertex matrices.
 * @throws std::invalid_argument If A_nominal is not square, uncertainty_cols does not
 *         have n*n rows, or m > 12 (2^m vertices would be impractical).
 */
inline std::vector<Eigen::MatrixXd> buildBoxVertices(const Eigen::MatrixXd& A_nominal,
                                                      const Eigen::MatrixXd& uncertainty_cols)
{
    const int n = static_cast<int>(A_nominal.rows());
    if (A_nominal.cols() != n)
        throw std::invalid_argument("buildBoxVertices: A_nominal must be square.");

    const int m = static_cast<int>(uncertainty_cols.cols());
    if (uncertainty_cols.rows() != n * n)
        throw std::invalid_argument(
            "buildBoxVertices: uncertainty_cols must have n*n rows (one flattened n x n direction per column).");
    if (m > 12)
        throw std::invalid_argument("buildBoxVertices: m > 12 perturbation directions is impractical (2^m vertices).");

    std::vector<Eigen::MatrixXd> deltas(m);
    for (int j = 0; j < m; ++j)
    {
        const Eigen::VectorXd flat = uncertainty_cols.col(j);
        deltas[j] = Eigen::Map<const Eigen::MatrixXd>(flat.data(), n, n);
    }

    const std::size_t num_vertices = std::size_t(1) << m;
    std::vector<Eigen::MatrixXd> vertices;
    vertices.reserve(num_vertices);
    for (std::size_t code = 0; code < num_vertices; ++code)
    {
        Eigen::MatrixXd A = A_nominal;
        for (int j = 0; j < m; ++j)
            A += ((code >> j) & 1u) ? deltas[j] : -deltas[j];
        vertices.push_back(std::move(A));
    }
    return vertices;
}

} // namespace ctrl

// Self-registration - included by ControllerToolbox.h umbrella.
#include "ControllerRegistry.h"
CTRL_REGISTER_FEATURE(lyapunov_robustness)
