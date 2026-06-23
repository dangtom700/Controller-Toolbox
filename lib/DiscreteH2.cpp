#include "DiscreteH2.h"

#if defined(CTRL_HAS_HINF)

#include "DiscreteLQR.h"
#include "SystemAnalysis.h"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

DiscreteH2::DiscreteH2(const H2Result &result)
{
    if (!result.feasible)
        throw std::invalid_argument(
            "DiscreteH2: cannot construct from an infeasible H2Result.");

    Ak_     = result.Ak;
    Bk_     = result.Bk;
    Ck_     = result.Ck;
    Dk_     = result.Dk;
    Ts_     = result.Ts;
    h2norm_ = result.achievedH2Norm;

    const int nk = static_cast<int>(Ak_.rows());
    const int nu = static_cast<int>(Ck_.rows());
    xk_     = Eigen::VectorXd::Zero(nk);
    u_work_.resize(nu);
}

double DiscreteH2::compute(double signal)
{
    if (!std::isfinite(signal)) return u_prev_;
    // signal = y[k], the measurement (NOT tracking error - same convention as DiscreteHinf)
    Eigen::VectorXd y(1);
    y(0) = signal;
    u_prev_ = computeVec(y)(0);
    return u_prev_;
}

Eigen::VectorXd DiscreteH2::computeVec(const Eigen::VectorXd &y)
{
    u_work_.noalias() = Ck_ * xk_;
    u_work_.noalias() += Dk_ * y;
    xk_ = Ak_ * xk_ + Bk_ * y;
    return u_work_;
}

void DiscreteH2::reset()
{
    xk_.setZero();
}

// =============================================================================
// buildClosedLoop - [x; xk] -> [x+; xk+], w -> z  (D11 = D22 = Dk = 0 always here)
//
//   x+  = A x + B1 w + B2 u,   u  = Ck xk           (Dk = 0)
//   xk+ = Ak xk + Bk y,        y  = C2 x + D21 w     (D22 = 0)
//   z   = C1 x + D12 u                               (D11 = 0)
//
// Substituting u and y directly (no E = (I - Dk*D22)^-1 term needed, unlike
// DiscreteHinf::buildClosedLoop, since Dk*D22 = 0 identically):
//   x+  = A x + B2*Ck*xk + B1 w
//   xk+ = Bk*C2 x + Ak xk + Bk*D21 w
//   z   = C1 x + D12*Ck*xk
// =============================================================================
void DiscreteH2::buildClosedLoop(
    const GeneralisedPlant &P,
    const Eigen::MatrixXd  &Ak, const Eigen::MatrixXd &Bk,
    const Eigen::MatrixXd  &Ck,
    Eigen::MatrixXd &A_cl, Eigen::MatrixXd &B_cl, Eigen::MatrixXd &C_cl)
{
    const int n  = P.stateSize();
    const int nk = static_cast<int>(Ak.rows());
    const int nc = n + nk;

    A_cl.resize(nc, nc);
    B_cl.resize(nc, P.nw());
    C_cl.resize(P.nz(), nc);

    A_cl.topLeftCorner(n, n)       = P.A;
    A_cl.topRightCorner(n, nk)     = P.B2 * Ck;
    A_cl.bottomLeftCorner(nk, n)   = Bk * P.C2;
    A_cl.bottomRightCorner(nk, nk) = Ak;

    B_cl.topRows(n)     = P.B1;
    B_cl.bottomRows(nk) = Bk * P.D21;

    C_cl.leftCols(n)  = P.C1;
    C_cl.rightCols(nk) = P.D12 * Ck;
}

// =============================================================================
// solve - discrete H2/LQG synthesis via cross-term elimination + dual Riccati solve
//
// Control side: minimise sum z'z = x'C1'C1 x + 2x'(C1'D12)u + u'(D12'D12)u subject to
// x+ = Ax + B2u (D11=0 means w does not enter the cost cross term). Standard LQR-with-
// cross-term reduction (Bryson & Ho; Anderson & Moore Ch. 3):
//   R1 = D12'D12,  S1 = C1'D12,  Ahat = A - B2*R1^-1*S1',  Qhat = C1'C1 - S1*R1^-1*S1'
//   X solves the standard (no-cross-term) DARE(Ahat, B2, Qhat, R1)
//   F  = -(R1 + B2'XB2)^-1 * (B2'XA + S1')      [feedback gain, u = Ck*xk = F*xk]
//
// Filter side: the exact dual problem on (A', C2', B1, D21) with cross term S2 = B1*D21'.
// Solving DARE(Ahat_f, C2', Qhat_f, R2) gives Y; define
//   L = -(A Y C2' + S2) * (R2 + C2 Y C2')^-1
// so that the *standard* (textbook, "+" convention) Kalman observer gain is Lobs = -L:
//   xk[k+1] = A xk + B2 u + Lobs (y - C2 xk),  u = F xk
//           = (A + B2 F - Lobs C2) xk + Lobs y
// Substituting Lobs = -L gives the assembly below. Verified independently against
// scipy.linalg.solve_discrete_are + solve_discrete_lyapunov for a plant with nonzero
// cross terms S1, S2 (the [h2] Catch2 test) - the closed-loop [x;xk] system is stable with
// eigenvalues matching the separation principle exactly: {eig(A+B2F)} union {eig(A-Lobs*C2)}.
//   Ak = A + B2 F + L C2,  Bk = -L,  Ck = F,  Dk = 0
// =============================================================================
H2Result DiscreteH2::solve(const GeneralisedPlant &P, const H2Params & /*params*/)
{
    H2Result result;
    result.feasible = false;

    if (P.stateSize() == 0)
        throw std::invalid_argument("DiscreteH2::solve: empty generalised plant.");
    if (P.nu() == 0 || P.ny() == 0)
        throw std::invalid_argument(
            "DiscreteH2::solve: plant must have at least one control and one measurement channel.");
    if (P.nw() == 0 || P.nz() == 0)
        throw std::invalid_argument(
            "DiscreteH2::solve: plant must have exogenous and performance channels.");

    // D11 = 0 and D22 = 0 are required: the cross-term-elimination DARE reduction below has
    // no closed form for D11 != 0 (see class docs for why this is a real, not hypothetical,
    // limitation - most MixedSensitivity-built plants have D11 != 0). Throwing here, rather
    // than silently ignoring the feedthrough, avoids producing a wrong/suboptimal controller.
    if (!P.D11.isZero(1e-12))
        throw std::invalid_argument(
            "DiscreteH2::solve: generalised plant has nonzero D11 (exogenous-to-performance "
            "feedthrough). DiscreteH2 requires D11 = 0; build a hand-written GeneralisedPlant "
            "without direct feedthrough from w to z (most MixedSensitivity-built plants have "
            "D11 != 0 from the W1/W3 weight gains and are not usable here).");
    if (!P.D22.isZero(1e-12))
        throw std::invalid_argument(
            "DiscreteH2::solve: generalised plant has nonzero D22 (control-to-measurement "
            "feedthrough). DiscreteH2 requires D22 = 0.");

    // D12 full column rank, D21 full row rank (same DGKF-style regularity assumption
    // DiscreteHinf::solve() already requires).
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdD12(P.D12);
        const int rankD12 = static_cast<int>((svdD12.singularValues().array() > 1e-10).count());
        if (rankD12 < P.nu())
            throw std::invalid_argument(
                "DiscreteH2::solve: D12 does not have full column rank.");

        Eigen::JacobiSVD<Eigen::MatrixXd> svdD21(P.D21);
        const int rankD21 = static_cast<int>((svdD21.singularValues().array() > 1e-10).count());
        if (rankD21 < P.ny())
            throw std::invalid_argument(
                "DiscreteH2::solve: D21 does not have full row rank.");
    }

    const Eigen::MatrixXd &A   = P.A;
    const Eigen::MatrixXd &B1  = P.B1;
    const Eigen::MatrixXd &B2  = P.B2;
    const Eigen::MatrixXd &C1  = P.C1;
    const Eigen::MatrixXd &C2  = P.C2;
    const Eigen::MatrixXd &D12 = P.D12;
    const Eigen::MatrixXd &D21 = P.D21;

    // -------------------------------------------------------------------------
    // Control Riccati (cross-term eliminated)
    // -------------------------------------------------------------------------
    const Eigen::MatrixXd R1 = D12.transpose() * D12;
    const Eigen::MatrixXd S1 = C1.transpose() * D12;

    Eigen::LDLT<Eigen::MatrixXd> ldltR1(R1);
    if (ldltR1.info() != Eigen::Success) return result;

    const Eigen::MatrixXd Ahat = A - B2 * ldltR1.solve(S1.transpose());
    Eigen::MatrixXd Qhat = C1.transpose() * C1 - S1 * ldltR1.solve(S1.transpose());
    Qhat = 0.5 * (Qhat + Qhat.transpose());

    const DareResult dx = DiscreteLQR::solveDARE(Ahat, B2, Qhat, R1);
    result.dareConvX = dx.converged;
    if (!dx.converged) return result;

    const Eigen::MatrixXd X = 0.5 * (dx.P + dx.P.transpose());

    const Eigen::MatrixXd RF = R1 + B2.transpose() * X * B2;
    Eigen::LDLT<Eigen::MatrixXd> ldltRF(RF);
    if (ldltRF.info() != Eigen::Success) return result;
    const Eigen::MatrixXd F = -ldltRF.solve(B2.transpose() * X * A + S1.transpose());

    // -------------------------------------------------------------------------
    // Filter Riccati (dual problem on A', C2', B1, D21)
    // -------------------------------------------------------------------------
    const Eigen::MatrixXd R2 = D21 * D21.transpose();
    const Eigen::MatrixXd S2 = B1 * D21.transpose();

    Eigen::LDLT<Eigen::MatrixXd> ldltR2(R2);
    if (ldltR2.info() != Eigen::Success) return result;

    const Eigen::MatrixXd Ahat_f = A.transpose() - C2.transpose() * ldltR2.solve(S2.transpose());
    Eigen::MatrixXd Qhat_f = B1 * B1.transpose() - S2 * ldltR2.solve(S2.transpose());
    Qhat_f = 0.5 * (Qhat_f + Qhat_f.transpose());

    const DareResult dy = DiscreteLQR::solveDARE(Ahat_f, C2.transpose(), Qhat_f, R2);
    result.dareConvY = dy.converged;
    if (!dy.converged) return result;

    const Eigen::MatrixXd Y = 0.5 * (dy.P + dy.P.transpose());

    const Eigen::MatrixXd RL = R2 + C2 * Y * C2.transpose();
    Eigen::LDLT<Eigen::MatrixXd> ldltRL(RL);
    if (ldltRL.info() != Eigen::Success) return result;
    const Eigen::MatrixXd L = -(A * Y * C2.transpose() + S2) * ldltRL.solve(
        Eigen::MatrixXd::Identity(P.ny(), P.ny()));

    // -------------------------------------------------------------------------
    // Controller assembly + stability check
    // -------------------------------------------------------------------------
    result.Ak = A + B2 * F + L * C2;
    result.Bk = -L;
    result.Ck = F;
    result.Dk = Eigen::MatrixXd::Zero(P.nu(), P.ny());
    result.X  = X;
    result.Y  = Y;

    Eigen::EigenSolver<Eigen::MatrixXd> esAk(result.Ak, /*computeEigenvectors=*/false);
    for (int i = 0; i < esAk.eigenvalues().size(); ++i)
        if (std::abs(esAk.eigenvalues()(i)) >= 1.0 + 1e-6) return result;

    // -------------------------------------------------------------------------
    // Achieved H2 norm: sqrt(trace(C_cl * Wc * C_cl')), Wc solves the discrete
    // controllability Lyapunov equation A_cl*Wc*A_cl' - Wc + B_cl*B_cl' = 0.
    // -------------------------------------------------------------------------
    Eigen::MatrixXd A_cl, B_cl, C_cl;
    buildClosedLoop(P, result.Ak, result.Bk, result.Ck, A_cl, B_cl, C_cl);
    const Eigen::MatrixXd Wc = SystemAnalysis::solveDiscreteLyapunov(A_cl, B_cl * B_cl.transpose());
    const double h2sq = (C_cl * Wc * C_cl.transpose()).trace();
    result.achievedH2Norm = std::sqrt(std::max(h2sq, 0.0));
    result.Ts = P.Ts;
    result.feasible = true;
    return result;
}

} // namespace ctrl

#endif // CTRL_HAS_HINF
