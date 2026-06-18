#include "BalancedTruncation.h"
#include "SystemAnalysis.h"
#include <cmath>
#include <stdexcept>

namespace ctrl
{

TruncationResult balancedTruncate(const StateSpace &sys, int r)
{
    const int n = sys.stateSize();

    if (r < 1 || r >= n)
        throw std::invalid_argument(
            "balancedTruncate: r must satisfy 1 <= r < n (n=" + std::to_string(n) + ").");

    // --- Step 0: Check stability ---
    if (!SystemAnalysis::isDiscreteStable(sys))
        throw std::invalid_argument(
            "balancedTruncate: system must be strictly stable (all |eigenvalues(A)| < 1). "
            "Use a stabilising pre-compensator if needed.");

    const Eigen::MatrixXd &A = sys.A;
    const Eigen::MatrixXd &B = sys.B;
    const Eigen::MatrixXd &C = sys.C;
    const Eigen::MatrixXd &D = sys.D;

    // --- Step 1: Gramians ---
    // Controllability:  A*Pc*A' - Pc + B*B' = 0
    const Eigen::MatrixXd Pc = SystemAnalysis::solveDiscreteLyapunov(A, B * B.transpose());

    // Observability:    A'*Po*A - Po + C'*C = 0
    const Eigen::MatrixXd Po = SystemAnalysis::solveDiscreteLyapunov(A.transpose(),
                                                                       C.transpose() * C);

    // --- Step 2: Cholesky of Pc -> Pc = L_c * L_c' ---
    Eigen::LLT<Eigen::MatrixXd> llt(Pc);
    if (llt.info() != Eigen::Success)
        throw std::invalid_argument(
            "balancedTruncate: controllability gramian Pc is not positive definite. "
            "The system may have uncontrollable modes. Add a small regularisation: "
            "Pc += eps*I before calling.");

    const Eigen::MatrixXd L_c = llt.matrixL(); // lower triangular

    // --- Step 3: Symmetric matrix M = L_c' * Po * L_c ---
    const Eigen::MatrixXd M = L_c.transpose() * Po * L_c;

    // --- Step 4: Symmetric eigendecomposition of M ---
    // Eigenvalues lambda_i = sigma_i^2 where sigma_i are the Hankel singular values.
    // SelfAdjointEigenSolver gives ascending order - we want descending.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(M);

    // Collect HSVs in descending order
    const int n_eig = eig.eigenvalues().size();
    Eigen::VectorXd hsv(n_eig);
    Eigen::MatrixXd U_desc(n_eig, n_eig);
    for (int i = 0; i < n_eig; ++i)
    {
        // Clamp negative eigenvalues (rounding errors) to 0
        const double ev = std::max(eig.eigenvalues()(n_eig - 1 - i), 0.0);
        hsv(i)         = std::sqrt(ev);
        U_desc.col(i)  = eig.eigenvectors().col(n_eig - 1 - i);
    }

    // --- Step 5: Balancing transformation ---
    // T_r = L_c * U * diag(hsv^{-1/2})   [right transformation]
    // T_l = T_r^{-1}                      [left transformation]
    const Eigen::VectorXd sig_inv_sqrt =
        hsv.cwiseInverse().cwiseSqrt();   // sigma_i^{-1/2}, all sigma_i > 0 if gramians are PD

    const Eigen::MatrixXd T_r = L_c * U_desc * sig_inv_sqrt.asDiagonal();
    const Eigen::MatrixXd T_l = T_r.inverse(); // affordable for small n

    // --- Step 6: Balanced realisation ---
    const Eigen::MatrixXd A_bal = T_l * A * T_r;
    const Eigen::MatrixXd B_bal = T_l * B;
    const Eigen::MatrixXd C_bal = C * T_r;
    // D is unchanged

    // --- Step 7: Truncate to order r ---
    StateSpace reduced(
        A_bal.topLeftCorner(r, r),
        B_bal.topRows(r),
        C_bal.leftCols(r),
        D,
        sys.Ts);

    // --- Step 8: Error bound ---
    // ||G - Gᵣ||inf <= 2 . Sigma_i=ᵣ₊1^n sigma_i
    const double error_bound = 2.0 * hsv.tail(n - r).sum();

    const bool is_stable = SystemAnalysis::isDiscreteStable(reduced);

    return {reduced, hsv, error_bound, is_stable};
}

int suggestOrder(const TruncationResult &res, double tol)
{
    const Eigen::VectorXd &hsv = res.hankelSingularValues;
    const int n = static_cast<int>(hsv.size());
    const double total_norm = 2.0 * hsv.sum(); // approx = 2.||G||inf (loose upper bound)
    const double threshold  = tol * total_norm;

    // Return smallest r such that 2.Sigma_i=ᵣ₊1^n sigma_i < threshold
    double tail_sum = 0.0;
    for (int i = n - 1; i >= 1; --i)
    {
        tail_sum += hsv(i);
        if (2.0 * tail_sum >= threshold)
            return std::min(i + 1, n - 1); // order r = i: tail from i+1 is within tol
    }
    return 1; // single state is sufficient
}

} // namespace ctrl
