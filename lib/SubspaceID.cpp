#include "SubspaceID.h"
#include <Eigen/QR>
#include <Eigen/SVD>

namespace ctrl
{

namespace
{

// Build block Hankel matrix: data is (q * N), output is (i*q * (N-i)).
// Column c of the result stacks data[:, c], data[:, c+1], ..., data[:, c+i-1].
Eigen::MatrixXd buildHankel(const Eigen::MatrixXd &data, int i)
{
    const int q = data.rows();
    const int N = data.cols();
    const int s = N - i;
    Eigen::MatrixXd H(i * q, s);
    for (int r = 0; r < i; ++r)
        H.middleRows(r * q, q) = data.middleCols(r, s);
    return H;
}

} // namespace

// ---------------------------------------------------------------------------
// n4sid - MOESP oblique-projection subspace identification
// ---------------------------------------------------------------------------
SubspaceIDResult n4sid(const Eigen::MatrixXd &Y,
                       const Eigen::MatrixXd &U,
                       int n_order,
                       int i,
                       double Ts)
{
    SubspaceIDResult res;
    const int p = Y.rows(); // output dimension
    const int m = U.rows(); // input dimension
    const int N = Y.cols(); // number of samples
    const int s = N - 2 * i; // number of Hankel columns

    // Input checks
    if (U.cols() != N)
    {
        res.message = "U and Y must have the same number of columns (time samples).";
        return res;
    }
    if (s <= n_order)
    {
        res.message = "Not enough data: need N > 2*i_horizon + n_order. "
                      "Increase N or reduce i_horizon / n_order.";
        return res;
    }
    if (i <= n_order / p)
    {
        res.message = "i_horizon too small: need i_horizon > n_order / p. "
                      "Increase i_horizon.";
        return res;
    }
    if (n_order < 1)
    {
        res.message = "n_order must be >= 1.";
        return res;
    }

    // ------------------------------------------------------------------
    // Step 1: build 2i-block Hankel matrices and partition past/future
    // ------------------------------------------------------------------
    const Eigen::MatrixXd Yh = buildHankel(Y, 2 * i); // 2i*p * s
    const Eigen::MatrixXd Uh = buildHankel(U, 2 * i); // 2i*m * s

    const Eigen::MatrixXd Yp = Yh.topRows(i * p);      // i*p * s  (past)
    const Eigen::MatrixXd Yf = Yh.bottomRows(i * p);   // i*p * s  (future)
    const Eigen::MatrixXd Up = Uh.topRows(i * m);      // i*m * s  (past)
    const Eigen::MatrixXd Uf = Uh.bottomRows(i * m);   // i*m * s  (future)

    // Wp = [Up; Yp]  (i*(m+p) * s)
    const int r_uf = i * m;
    const int r_wp = i * (m + p);
    const int r_yf = i * p;
    const int r_tot = r_uf + r_wp + r_yf;

    Eigen::MatrixXd Z(r_tot, s);
    Z.topRows(r_uf)              = Uf;
    Z.middleRows(r_uf, i * m)    = Up;
    Z.middleRows(r_uf + i * m, i * p) = Yp;
    Z.bottomRows(r_yf)           = Yf;

    // ------------------------------------------------------------------
    // Step 2: thin LQ decomposition of Z via QR of Z'.
    //   Z' = Q_thin * R_thin  (s * r_tot and r_tot * r_tot)
    //   Z  = R_thin' * Q_thin'  ->  L = R_thin', Q_rows = Q_thin'
    //
    // The (3,2) block L32 of the lower-triangular L is the key quantity:
    //   L32 = L[r_uf+r_wp : r_tot, r_uf : r_uf+r_wp]
    // Its column space equals the oblique projection O_i = Y_f || U_f onto W_p.
    // ------------------------------------------------------------------
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(Z.transpose()); // QR of (s * r_tot)
    // R is upper triangular (r_tot * r_tot in the thin form produced by the
    // full QR of a fat matrix); L = R' is lower triangular.
    const Eigen::MatrixXd L =
        qr.matrixQR()
          .topRows(r_tot)
          .triangularView<Eigen::Upper>()
          .transpose(); // r_tot * r_tot, lower triangular

    // Extract L32: (r_yf * r_wp)
    const Eigen::MatrixXd L32 = L.block(r_uf + r_wp, r_uf, r_yf, r_wp);

    // ------------------------------------------------------------------
    // Step 3: SVD of L32 to get extended observability matrix Gamma
    // ------------------------------------------------------------------
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(L32, Eigen::ComputeThinU | Eigen::ComputeThinV);
    res.singularValues = svd.singularValues();

    if (n_order > svd.singularValues().size())
    {
        res.message = "n_order exceeds rank of L32. Reduce n_order.";
        return res;
    }

    // Gamma: (i*p * n)
    const Eigen::VectorXd S_sqrt = svd.singularValues().head(n_order).cwiseSqrt();
    const Eigen::MatrixXd Gamma  = svd.matrixU().leftCols(n_order) * S_sqrt.asDiagonal();

    // ------------------------------------------------------------------
    // Step 4: extract C and A from shift invariance of Gamma
    //   C     = Gamma[:p, :]            (p * n)
    //   A approx = Gamma_up† * Gamma_down      (n * n, least squares)
    //   Gamma_up   = Gamma[:(i-1)*p, :]
    //   Gamma_down = Gamma[p:, :]
    // ------------------------------------------------------------------
    const Eigen::MatrixXd C = Gamma.topRows(p);

    const Eigen::MatrixXd Gamma_up   = Gamma.topRows((i - 1) * p);
    const Eigen::MatrixXd Gamma_down = Gamma.bottomRows((i - 1) * p);

    // A = Gamma_up† * Gamma_down  (n * n)
    const Eigen::MatrixXd A =
        Gamma_up.colPivHouseholderQr().solve(Gamma_down);

    // ------------------------------------------------------------------
    // Step 5: least-squares regression for B and D.
    //
    // Reconstruct the state sequence from the data and the observability
    // matrix: X_hat[:, k] = Gamma† * Yf_col[:, k] (truncated future output
    // stacked in a column).
    //
    // From x[k+1] = A*x[k] + B*u[k]  and  y[k] = C*x[k] + D*u[k]:
    //
    //   [y[k] - C.x[k]] = D . u[k]
    //   [x[k+1] - A.x[k]] = B . u[k]
    //
    // Stack these over k = 0..s-1 and solve in least squares.
    // ------------------------------------------------------------------
    const Eigen::MatrixXd Gamma_pinv =
        Gamma.colPivHouseholderQr().solve(Eigen::MatrixXd::Identity(i * p, i * p));
    // X_hat: n * s  - state estimates
    const Eigen::MatrixXd X_hat = Gamma_pinv * Yf;

    // Build regression: stack y[k] = C*x[k] + D*u[k] over k=0..s-2
    // (use s-1 columns so x[k+1] exists)
    const int T = s - 1;
    Eigen::MatrixXd Phi_BD(T, m);      // input regressors (T * m)
    Eigen::MatrixXd Lhs_D(p, T);      // y[k] - C*x[k]   (p * T)
    Eigen::MatrixXd Lhs_B(n_order, T); // x[k+1] - A*x[k] (n * T)

    for (int k = 0; k < T; ++k)
    {
        // Use the column of U aligned with the future block start
        const Eigen::VectorXd u_k = U.col(i + k);
        const Eigen::VectorXd x_k   = X_hat.col(k);
        const Eigen::VectorXd x_k1  = X_hat.col(k + 1);
        const Eigen::VectorXd y_k   = Y.col(i + k);

        Phi_BD.row(k)  = u_k.transpose();
        Lhs_D.col(k)   = y_k   - C * x_k;
        Lhs_B.col(k)   = x_k1  - A * x_k;
    }

    // Solve D: (p * m) = Lhs_D * Phi_BD†
    const Eigen::MatrixXd D =
        (Phi_BD.colPivHouseholderQr().solve(Lhs_D.transpose())).transpose();

    // Solve B: (n * m) = Lhs_B * Phi_BD†
    const Eigen::MatrixXd B =
        (Phi_BD.colPivHouseholderQr().solve(Lhs_B.transpose())).transpose();

    // ------------------------------------------------------------------
    // Pack result
    // ------------------------------------------------------------------
    res.model   = std::make_optional<StateSpace>(A, B, C, D, Ts);
    res.success = true;
    return res;
}

// ---------------------------------------------------------------------------
// suggestOrder
// ---------------------------------------------------------------------------
int suggestOrder(const Eigen::VectorXd &sv, double threshold, int maxOrder)
{
    const int n = (maxOrder > 0)
                  ? std::min(static_cast<int>(sv.size()), maxOrder + 1)
                  : static_cast<int>(sv.size());

    if (n <= 1)
        return 1;

    // Elbow: find i* = argmax  sv(i) / sv(i+1)
    int    bestI    = 0;
    double bestRatio = 0.0;
    for (int i = 0; i < n - 1; ++i)
    {
        if (sv(i + 1) < 1e-14)
        {
            bestI = i; // everything beyond is numerical zero
            break;
        }
        const double ratio = sv(i) / sv(i + 1);
        if (ratio > bestRatio)
        {
            bestRatio = ratio;
            bestI     = i;
        }
    }
    int order = bestI + 1; // keep sv[0..bestI]

    // Threshold guard: cap at first index where sv(i)/sv(0) < threshold
    if (threshold > 0.0 && sv(0) > 1e-14)
    {
        for (int i = 0; i < n; ++i)
        {
            if (sv(i) / sv(0) < threshold)
            {
                order = std::min(order, i);
                break;
            }
        }
    }

    return std::max(1, order);
}

} // namespace ctrl
