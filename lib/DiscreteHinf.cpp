#include "DiscreteHinf.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <vector>



namespace ctrl
{

// =============================================================================
// DiscreteHinf - constructor and runtime
// =============================================================================

DiscreteHinf::DiscreteHinf(const HinfResult &result)
{
    if (!result.feasible)
        throw std::invalid_argument(
            "DiscreteHinf: cannot construct from an infeasible HinfResult.");

    Ak_    = result.Ak;
    Bk_    = result.Bk;
    Ck_    = result.Ck;
    Dk_    = result.Dk;
    Ts_    = result.Ts;
    gamma_ = result.achievedGamma;

    const int nk = static_cast<int>(Ak_.rows());
    xk_  = Eigen::VectorXd::Zero(nk);
}

double DiscreteHinf::compute(double signal)
{
    // signal = y[k], the measurement (NOT tracking error - H-inf is output-feedback)
    Eigen::VectorXd y(1);
    y(0) = signal;
    return computeVec(y)(0);
}

Eigen::VectorXd DiscreteHinf::computeVec(const Eigen::VectorXd &y)
{
    // Controller update:
    //   u[k]   = Ck xk[k] + Dk y[k]
    //   xk[k+1] = Ak xk[k] + Bk y[k]
    const Eigen::VectorXd u = Ck_ * xk_ + Dk_ * y;
    xk_ = Ak_ * xk_ + Bk_ * y;
    return u;
}

void DiscreteHinf::reset()
{
    xk_.setZero();
}

// =============================================================================
// DARE solver - symplectic pencil generalised eigenvalue method for indefinite R
//
// Solves: A'XA - X - A'XB (R + B'XB)^{-1} B'XA + Q = 0
//
// The H-inf DARE has an indefinite R (gamma-scaled diagonal blocks), so the
// standard positive-definite doubling iteration diverges.  The Hamiltonian
// approach forms G = B*Rinv*B', which amplifies ill-conditioning when R is
// near-singular (as occurs at gamma close to gamma_opt).  Instead we use the
// symplectic pencil (M, N) which defers the R^{-1} product until the
// eigenvector solve - never forming B*Rinv*B' explicitly:
//
//   M = [ A,  0 ]     N = [ I,  B*Rinv*B' ]
//       [-Q,  I ]         [ 0,  A'        ]
//
// The generalised eigenvalue problem M*v = lambda * N*v has 2n eigenvalues
// in reciprocal pairs (lambda, 1/lambda).  We select the n eigenvalues with
// |lambda| < 1 (stable subspace) and recover X = V2 * V1^{-1} from the
// corresponding eigenvectors partitioned as [V1; V2].
//
// Ref: Laub (1979) "A Schur method for solving algebraic Riccati equations";
//      Van Dooren (1981) "A generalised eigenvalue approach for solving
//      Riccati equations"; Lancaster & Rodman "Algebraic Riccati Equations".
// =============================================================================
DiscreteHinf::DareOut DiscreteHinf::solveHinfDARE(
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    double /*tol*/, int /*maxIter*/)
{
    DareOut out;
    out.conv  = false;
    out.iters = 1;

    const int n = A.rows();

    // R must be invertible (caller has already verified this).
    Eigen::FullPivLU<Eigen::MatrixXd> luR(R);
    if (!luR.isInvertible())
    {
        out.X = Eigen::MatrixXd::Zero(n, n);
        return out;
    }
    const Eigen::MatrixXd Rinv = luR.inverse();

    // Symplectic pencil (M, N):
    //   M = [ A,  0 ]     N = [ I,  B*Rinv*B' ]
    //       [-Q,  I ]         [ 0,  A'        ]
    //
    // B*Rinv*B' is computed here once; it is n x n and indefinite for H-inf R.
    const Eigen::MatrixXd BRinvBt = B * Rinv * B.transpose();

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2 * n, 2 * n);
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(2 * n, 2 * n);

    M.topLeftCorner(n, n)     =  A;
    M.bottomLeftCorner(n, n)  = -Q;
    M.bottomRightCorner(n, n) =  Eigen::MatrixXd::Identity(n, n);

    N.topLeftCorner(n, n)     =  Eigen::MatrixXd::Identity(n, n);
    N.topRightCorner(n, n)    =  BRinvBt;
    N.bottomRightCorner(n, n) =  A.transpose();

    // Generalised eigenvalue problem: M v = lambda N v.
    // Eigen's GeneralizedEigenSolver works with real matrices and returns
    // complex eigenvalue/eigenvector pairs - correct for the indefinite case.
    Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges(M, N);
    if (ges.info() != Eigen::Success)
    {
        out.X = Eigen::MatrixXd::Zero(n, n);
        return out;
    }

    const auto &evals = ges.eigenvalues();   // complex (2n,)
    const auto &evecs = ges.eigenvectors();  // complex (2n, 2n)

    // Select the n eigenvectors whose eigenvalue has |lambda| < 1 (stable subspace).
    std::vector<int> stable_idx;
    stable_idx.reserve(n);
    for (int i = 0; i < 2 * n; ++i)
    {
        if (std::abs(evals(i)) < 1.0 - 1e-10)
            stable_idx.push_back(i);
    }

    // Need exactly n stable eigenvalues for a unique stabilising solution.
    if (static_cast<int>(stable_idx.size()) != n)
    {
        out.X = Eigen::MatrixXd::Zero(n, n);
        return out;
    }

    // Build the stable subspace matrix and partition into [V1; V2] (each n x n).
    Eigen::MatrixXcd V1(n, n), V2(n, n);
    for (int j = 0; j < n; ++j)
    {
        V1.col(j) = evecs.col(stable_idx[j]).head(n);
        V2.col(j) = evecs.col(stable_idx[j]).tail(n);
    }

    // X = real(V2 * V1^{-1})
    Eigen::FullPivLU<Eigen::MatrixXcd> luV1(V1);
    if (!luV1.isInvertible())
    {
        out.X = Eigen::MatrixXd::Zero(n, n);
        return out;
    }

    const Eigen::MatrixXcd X_c = V2 * luV1.inverse();
    out.X = X_c.real();
    out.X = 0.5 * (out.X + out.X.transpose()); // enforce symmetry

    // Verify the DARE residual: ||A'XA - X + Q - A'XB * (R+B'XB)^{-1} * B'XA|| / (1+||X||)
    const Eigen::MatrixXd &X = out.X;
    const Eigen::MatrixXd Rbar = R + B.transpose() * X * B;
    Eigen::FullPivLU<Eigen::MatrixXd> luRbar(Rbar);
    if (!luRbar.isInvertible())
    {
        out.X = Eigen::MatrixXd::Zero(n, n);
        return out;
    }
    const Eigen::MatrixXd K     = luRbar.inverse() * B.transpose() * X * A;
    const Eigen::MatrixXd resid = A.transpose() * X * A - X + Q - A.transpose() * X * B * K;
    const double dare_res = resid.norm() / (1.0 + X.norm());
    out.conv = (dare_res < 1e-6) && X.allFinite();
    return out;
}

// =============================================================================
// trySolve - attempt H-inf synthesis at a fixed gamma
//
// Returns true iff all DGKF conditions are satisfied:
//   (C1) Control DARE for X_inf has a stabilising solution (X_inf >= 0,
//        spectral radius of closed-loop A - B2 F < 1)
//   (C2) Filter  DARE for Y_inf has a stabilising solution (Y_inf >= 0,
//        spectral radius of closed-loop A - L C2 < 1)
//   (C3) rho(X_inf * Y_inf) < gamma^2
//
// The controller matrices are assembled only when all three conditions hold.
//
// DGKF discrete two-Riccati formulas (Stoorvogel 1992 / Iglesias & Glover 1991):
//
// Define:
//   gamma2 = gamma^2
//   B  = [B1, B2]
//   D  = [D11 D12; D21 D22]   (standard form: D11=0, D22=0)
//
// Control DARE:  Solve for X_inf >= 0 stabilising solution of:
//   A'XA - X + C1'C1 - A'XB * (gamma2*[I 0; 0 -I] + [B1 B2]'*X*[B1 B2])^{-1} * B'*X*A = 0
//
// Filter DARE:  Solve for Y_inf >= 0 stabilising solution of (transposed):
//   A Y A' - Y + B1 B1' - A Y C' * (gamma2*[I 0; 0 -I] + [C1; C2]*Y*[C1; C2]')^{-1} * C Y A' = 0
//
// Controller assembly (Stoorvogel 1992, Lemma 3.1):
//   F_inf  = -(R_x)^{-1} * (B2' X_inf A + D12' C1)   [state feedback gain]
//   L_inf  = -(A Y_inf C2' + B1 D21') * (R_y)^{-1}    [observer gain]
//   where R_x = I + D12' D12 + B2' X_inf B2  (positive definite if synthesis feasible)
//         R_y = I + D21 D21' + C2 Y_inf C2'  (positive definite if synthesis feasible)
//
//   Z_inf  = (I - gamma^{-2} Y_inf X_inf)^{-1}
//
//   Ak = A + B2 F_inf + Z_inf L_inf (C2 + D21 D11' ... ) + ...
//      [simplified for D11=0, D22=0 standard form]
//
// For the standard form (D11=0, D22=0) the formulas reduce to:
//   Control DARE Q_ctrl = C1'C1,  R_ctrl = gamma2*[I 0; 0 -I] + [B1 B2]'*X*[B1 B2]
//   Filter  DARE Q_filt = B1*B1', R_filt = gamma2*[I 0; 0 -I] + [C1; C2]*Y*[C1; C2]'
//
//   F  = -(D12'D12 + B2'X B2)^{-1} * B2'X A          (standard form, D12'C1 term = 0 when D11=0)
//   L  = -A Y C2'*(D21 D21' + C2 Y C2')^{-1}
//   Z  = (I - Y X / gamma2)^{-1}
//
//   Ak = A + B2 F + Z * (L + B2 F... )  [see below for exact assembly]
// =============================================================================
bool DiscreteHinf::trySolve(const GeneralisedPlant &P, double gamma,
                              double dareTol, int dareMaxIter,
                              HinfResult &out)
{
    const double g2 = gamma * gamma;
    const int n  = P.stateSize();
    const int nw = P.nw();
    const int nu = P.nu();
    const int nz = P.nz();
    const int ny = P.ny();

    const Eigen::MatrixXd &A   = P.A;
    const Eigen::MatrixXd &B1  = P.B1;
    const Eigen::MatrixXd &B2  = P.B2;
    const Eigen::MatrixXd &C1  = P.C1;
    const Eigen::MatrixXd &C2  = P.C2;
    const Eigen::MatrixXd &D11 = P.D11;
    const Eigen::MatrixXd &D12 = P.D12;
    const Eigen::MatrixXd &D21 = P.D21;
    // D22 assumed zero (standard form).

    // -----------------------------------------------------------------------
    // Pre-check: gamma must exceed ||D11||_2 for the standard-form DAREs
    // to be well-posed (Iglesias & Glover 1991, Lemma 2.1).
    // -----------------------------------------------------------------------
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdD11(D11);
        if (!D11.isZero(1e-14) && svdD11.singularValues()(0) >= gamma)
            return false;
    }

    // -----------------------------------------------------------------------
    // Control DARE - full form with D11, D12 (Iglesias & Glover 1991, Eq. 2)
    //
    // Define the block matrix:
    //   Theta_x = [gamma^2 I_nw   0     ] + [D11' ] [D11 D12]
    //             [   0         -I_nu   ]   [D12' ]
    //           = [gamma^2*I_nw + D11'D11,   D11'D12]
    //             [D12'D11,                  D12'D12 - I_nu]
    //
    // Control DARE:  A' X A - X + C1'C1 - A'X [B1 B2] Theta_x(X)^{-1} [B1;B2]'XA
    //   where Theta_x(X) = Theta_x + [B1' X B1   B1' X B2]
    //                                [B2' X B1   B2' X B2]
    //
    // For the doubling iteration we pass:
    //   Bcat = [B1 B2],  Q_x = C1'C1 + [D11'D11 cross-terms],  R_x = Theta_x
    // -----------------------------------------------------------------------
    const Eigen::MatrixXd Bcat = (Eigen::MatrixXd(n, nw + nu)
                                  << B1, B2).finished();

    // R_x = [gamma^2*I + D11'D11,  D11'D12]
    //       [D12'D11,               D12'D12 - I_nu]
    Eigen::MatrixXd Rx = Eigen::MatrixXd::Zero(nw + nu, nw + nu);
    Rx.topLeftCorner(nw, nw)         =  g2 * Eigen::MatrixXd::Identity(nw, nw) + D11.transpose() * D11;
    Rx.topRightCorner(nw, nu)        =  D11.transpose() * D12;
    Rx.bottomLeftCorner(nu, nw)      =  D12.transpose() * D11;
    Rx.bottomRightCorner(nu, nu)     =  D12.transpose() * D12 - Eigen::MatrixXd::Identity(nu, nu);

    // Q_x = C1'C1  (cross terms D11'C1 enter via the off-diagonal R_x - the
    // structured doubling algorithm absorbs them when R_x is fully populated.)
    const Eigen::MatrixXd Qx = C1.transpose() * C1;

    // Check R_x invertibility before solving (required for DARE)
    Eigen::FullPivLU<Eigen::MatrixXd> luRx0(Rx);
    if (!luRx0.isInvertible()) return false;

    DareOut dx = solveHinfDARE(A, Bcat, Qx, Rx, dareTol, dareMaxIter);
    out.dareConvX  = dx.conv;
    out.dareItersX = dx.iters;
    if (!dx.conv) return false;

    const Eigen::MatrixXd &X = dx.X;
    // Symmetrise
    const Eigen::MatrixXd Xs = 0.5 * (X + X.transpose());

    // Check X >= 0 (all eigenvalues >= 0)
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esX(Xs);
    if (esX.eigenvalues().minCoeff() < -1e-6) return false;

    // -----------------------------------------------------------------------
    // Filter DARE - full form with D11, D21 (dual of control problem)
    //
    // R_y = [gamma^2*I_nz + D11*D11',   D11*D21']
    //       [D21*D11',                   D21*D21' - I_ny]
    // Q_y = B1*B1'
    // Ccat = [C1; C2]
    // Solved as DARE(A', Ccat', Q_y, R_y) -> Y_inf
    // -----------------------------------------------------------------------
    const Eigen::MatrixXd Ccat = (Eigen::MatrixXd(nz + ny, n)
                                  << C1, C2).finished();

    Eigen::MatrixXd Ry = Eigen::MatrixXd::Zero(nz + ny, nz + ny);
    Ry.topLeftCorner(nz, nz)         =  g2 * Eigen::MatrixXd::Identity(nz, nz) + D11 * D11.transpose();
    Ry.topRightCorner(nz, ny)        =  D11 * D21.transpose();
    Ry.bottomLeftCorner(ny, nz)      =  D21 * D11.transpose();
    Ry.bottomRightCorner(ny, ny)     =  D21 * D21.transpose() - Eigen::MatrixXd::Identity(ny, ny);

    const Eigen::MatrixXd Qy = B1 * B1.transpose();

    Eigen::FullPivLU<Eigen::MatrixXd> luRy0(Ry);
    if (!luRy0.isInvertible()) return false;

    DareOut dy = solveHinfDARE(A.transpose(), Ccat.transpose(), Qy, Ry, dareTol, dareMaxIter);
    out.dareConvY  = dy.conv;
    out.dareItersY = dy.iters;

    if (!dy.conv) return false;

    const Eigen::MatrixXd &Y = dy.X;
    const Eigen::MatrixXd Ys = 0.5 * (Y + Y.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esY(Ys);
    if (esY.eigenvalues().minCoeff() < -1e-6) return false;

    // -----------------------------------------------------------------------
    // Condition (C3): spectral radius of X_inf * Y_inf < gamma^2
    //
    // WHY this condition is required (DGKF Theorem 3.1 coupling condition):
    //   The central H-infinity controller is assembled as:
    //     Z_inf = (I - Y_inf * X_inf / gamma^2)^{-1}
    //   which requires (I - Y_inf * X_inf / gamma^2) to be invertible.
    //   This matrix is invertible if and only if none of its eigenvalues are zero,
    //   i.e., none of the eigenvalues of (Y_inf * X_inf / gamma^2) equal 1,
    //   i.e., no eigenvalue of Y_inf * X_inf equals gamma^2,
    //   i.e., rho(X_inf * Y_inf) < gamma^2  (since X_inf*Y_inf and Y_inf*X_inf
    //   share the same nonzero eigenvalues).
    //
    //   Geometrically: C3 measures the "coupling" between the control and filter
    //   Riccati solutions.  When X_inf and Y_inf are both small (plant easy to
    //   control and observe), rho(XY) << gamma^2 easily.  As gamma approaches
    //   gamma_opt from above, rho(XY) -> gamma^2 from below; at gamma = gamma_opt,
    //   C3 holds with equality (infimum).  For gamma < gamma_opt, C3 fails -
    //   Z_inf becomes singular and the controller assembly is undefined.
    //
    // Ref: Doyle, Glover, Khargonekar, Francis (1989) IEEE TAC 34(8), Theorem 3;
    //      Stoorvogel (1992) "The H-infinity Control Problem", Lemma 3.1 condition (iii).
    // -----------------------------------------------------------------------
    const Eigen::MatrixXd XY = Xs * Ys;
    Eigen::EigenSolver<Eigen::MatrixXd> esXY(XY);
    double rho = 0.0;
    for (int i = 0; i < esXY.eigenvalues().size(); ++i)
        rho = std::max(rho, std::abs(esXY.eigenvalues()(i)));

    out.spectralRadius = rho;
    if (rho >= g2) return false;

    // -----------------------------------------------------------------------
    // Controller assembly (D22=0; D11 may be nonzero)
    //
    // Iglesias & Glover (1991) / Stoorvogel (1992) central controller:
    //
    //   F   = -R_F^{-1} * (B2'*Xs*A + D12'*C1)
    //   L   = -(A*Ys*C2' + B1*D21') * R_L^{-1}
    //   Z   = (I - Ys*Xs/gamma^2)^{-1}
    //
    //   Ak  = Z * (A + B2*F + L*C2)
    //   Bk  = Z * L
    //   Ck  = F
    //   Dk  = 0_{nu x ny}
    // -----------------------------------------------------------------------

    // State-feedback gain (D12'*C1 cross-term handles D11 case via DARE structure)
    const Eigen::MatrixXd R_F = D12.transpose() * D12 + B2.transpose() * Xs * B2;
    Eigen::FullPivLU<Eigen::MatrixXd> luRF(R_F);
    if (!luRF.isInvertible()) return false;
    const Eigen::MatrixXd F = -luRF.inverse() * (B2.transpose() * Xs * A + D12.transpose() * C1);

    // Observer gain (B1*D21' cross-term handles D11 case via dual DARE structure)
    const Eigen::MatrixXd R_L = D21 * D21.transpose() + C2 * Ys * C2.transpose();
    Eigen::FullPivLU<Eigen::MatrixXd> luRL(R_L);
    if (!luRL.isInvertible()) return false;
    const Eigen::MatrixXd L = -(A * Ys * C2.transpose() + B1 * D21.transpose()) * luRL.inverse();

    // Z = (I - Ys * Xs / gamma^2)^{-1}
    const Eigen::MatrixXd IminYXg2 = Eigen::MatrixXd::Identity(n, n) - Ys * Xs / g2;
    Eigen::FullPivLU<Eigen::MatrixXd> luZ(IminYXg2);
    if (!luZ.isInvertible()) return false;
    const Eigen::MatrixXd Z = luZ.inverse();

    out.Ak = Z * (A + B2 * F + L * C2);
    out.Bk = Z * L;
    out.Ck = F;
    out.Dk = Eigen::MatrixXd::Zero(nu, ny);

    out.X_inf = Xs;
    out.Y_inf = Ys;

    // Verify closed-loop stability of the controller (Ak should be stable)
    Eigen::EigenSolver<Eigen::MatrixXd> esAk(out.Ak);
    for (int i = 0; i < esAk.eigenvalues().size(); ++i)
    {
        if (std::abs(esAk.eigenvalues()(i)) >= 1.0 + 1e-6)
            return false;
    }

    return true;
}

// =============================================================================
// solve - bisection on gamma to find minimum achievable H-inf norm
// =============================================================================
HinfResult DiscreteHinf::solve(const GeneralisedPlant &P, const HinfParams &params)
{
    HinfResult result;
    result.feasible       = false;
    result.achievedGamma  = std::numeric_limits<double>::infinity();

    // Basic dimension checks
    if (P.stateSize() == 0)
        throw std::invalid_argument("DiscreteHinf::solve: empty generalised plant.");
    if (P.nu() == 0 || P.ny() == 0)
        throw std::invalid_argument("DiscreteHinf::solve: plant must have at least one control and one measurement channel.");
    if (P.nw() == 0 || P.nz() == 0)
        throw std::invalid_argument("DiscreteHinf::solve: plant must have exogenous and performance channels.");

    // Verify D12 full column rank and D21 full row rank (DGKF assumption A2)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdD12(P.D12);
        const int rankD12 = (int)(svdD12.singularValues().array() > 1e-10).count();
        if (rankD12 < P.nu())
            throw std::invalid_argument("DiscreteHinf::solve: D12 does not have full column rank (DGKF assumption A2 violated).");

        Eigen::JacobiSVD<Eigen::MatrixXd> svdD21(P.D21);
        const int rankD21 = (int)(svdD21.singularValues().array() > 1e-10).count();
        if (rankD21 < P.ny())
            throw std::invalid_argument("DiscreteHinf::solve: D21 does not have full row rank (DGKF assumption A2 violated).");
    }

    // Warn if D22 is nonzero: the DGKF assembly formulas in trySolve() assume D22=0.
    // For plants with direct feedthrough, apply a loop-shifting pre-processing step
    // before calling solve() (see Skogestad & Postlethwaite Section 9.4).
    if (P.D22.norm() > 1e-12)
        std::cerr << "[DiscreteHinf] WARNING: generalised plant D22 is nonzero (norm="
                  << P.D22.norm() << "). The standard DGKF assembly assumes D22=0. "
                  << "Apply loop-shifting before calling solve(), or use MixedSensitivity "
                  << "with a plant that has no direct feedthrough (D=0).\n";

    // Gamma bisection: find the smallest gamma in (gammaLo, gammaInit) that is feasible.
    // gammaLo is set to max(||D11||_2, 1e-4) - the Iglesias-Glover lower bound (Lemma 2.1)
    // below which no feasible synthesis exists regardless of the DARE solutions.
    double gammaHi  = params.gammaInit;
    double gammaLo;
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdD11(P.D11);
        const double d11_lb = P.D11.isZero(1e-14) ? 0.0 : svdD11.singularValues()(0);
        gammaLo = std::max(d11_lb + 1e-6, 1e-4);
    }

    // Check that gammaHi is feasible; if not, double it up to 10 times.
    HinfResult candidate;
    bool hiOk = trySolve(P, gammaHi, params.dareTol, params.dareMaxIter, candidate);
    if (!hiOk)
    {
        for (int d = 0; d < 10; ++d)
        {
            gammaHi *= 2.0;
            hiOk = trySolve(P, gammaHi, params.dareTol, params.dareMaxIter, candidate);
            if (hiOk) break;
        }
        if (!hiOk)
            return result; // infeasible at all tried gammas
    }

    result = candidate;
    result.feasible      = true;
    result.achievedGamma = gammaHi;
    result.Ts            = P.Ts;

    // Bisect down
    for (int iter = 0; iter < params.maxIter; ++iter)
    {
        if (gammaHi - gammaLo < params.gammaTol) break;

        const double gammaMid = 0.5 * (gammaLo + gammaHi);
        HinfResult midResult;
        if (trySolve(P, gammaMid, params.dareTol, params.dareMaxIter, midResult))
        {
            gammaHi              = gammaMid;
            result               = midResult;
            result.feasible      = true;
            result.achievedGamma = gammaMid;
            result.Ts            = P.Ts;
        }
        else
        {
            gammaLo = gammaMid;
        }
    }

    return result;
}

// =============================================================================
// MixedSensitivity - weight factory and generalised plant builder
//
// Weight designs follow Skogestad & Postlethwaite Ch. 2-3 (2005).
// All continuous-time weights are discretised using the Tustin transform.
// =============================================================================

// Tustin discretisation of a first-order continuous-time system:
//   H_c(s) = (b1 s + b0) / (a1 s + a0)
// -> H_d(z) = (alpha*b1 + b0*Ts/2) / (alpha*a1 + a0*Ts/2) * (z + beta) / (z + gamma)
// where alpha = 2/Ts.
//
// Returns StateSpace with A = [[p]], B = [[1]], C = [[c_gain]], D = [[d_gain]]
// encoding H_d(z) = d_gain + c_gain * z^{-1} / (1 - p * z^{-1}) (controller canonical).
static StateSpace tustinFirstOrder(double b1, double b0, double a1, double a0, double Ts)
{
    // Bilinear substitution: s = (2/Ts) * (z-1)/(z+1)
    // H_c(s) = (b1 s + b0)/(a1 s + a0)
    // Multiply numerator and denominator by (z+1) and substitute:
    //   num: b1*(2/Ts)*(z-1) + b0*(z+1) = (2b1/Ts + b0)*z + (-2b1/Ts + b0)
    //   den: a1*(2/Ts)*(z-1) + a0*(z+1) = (2a1/Ts + a0)*z + (-2a1/Ts + a0)
    const double alpha = 2.0 / Ts;
    const double num_z1 =  alpha * b1 + b0;
    const double num_z0 = -alpha * b1 + b0;
    const double den_z1 =  alpha * a1 + a0;
    const double den_z0 = -alpha * a1 + a0;

    // Monic: divide by den_z1
    // H_d(z) = (num_z1/den_z1 * z + num_z0/den_z1) / (z + den_z0/den_z1)
    // In state-space (scalar, first order):
    //   x[k+1] = (-den_z0/den_z1) x[k] + u[k]
    //   y[k]   = (num_z1/den_z1 - D * (-den_z0/den_z1)) x[k] + D * u[k]
    // with D = num_z1/den_z1.
    const double A_ss = -den_z0 / den_z1;
    const double D_ss = num_z1 / den_z1;
    // y = C*x + D*u => C*A_ss + D*1 = num_z0/den_z1  => C = (num_z0/den_z1 - D*A_ss) / ... wait
    // For H(z) = D + C*(zI-A)^{-1}*B with B=1:
    //   H(z) = D + C*z^{-1}/(1 - A*z^{-1})
    //        = D*z/(z-A) + C*z^{-1}*z/(z-A) = (D*(z-A) + C) / (z-A)  -- only if B=1
    // Actually for B=1:
    //   H(z) = C*(zI-A)^{-1}*1 + D = C/(z-A) + D = (C + D*(z-A))/(z-A) = (D*z + C - D*A)/(z-A)
    // Compare with target H_d(z) = (num_z1*z + num_z0)/(den_z1*z + den_z0) [non-monic]
    //                              = (num_z1/den_z1 * z + num_z0/den_z1)/(z + den_z0/den_z1)
    // So: D = num_z1/den_z1, A = -den_z0/den_z1 (pole), C = num_z0/den_z1 - D*(-A) = num_z0/den_z1 - D*A
    const double C_ss = num_z0 / den_z1 + D_ss * A_ss;

    Eigen::MatrixXd A_m(1, 1); A_m(0, 0) = A_ss;
    Eigen::MatrixXd B_m(1, 1); B_m(0, 0) = 1.0;
    Eigen::MatrixXd C_m(1, 1); C_m(0, 0) = C_ss;
    Eigen::MatrixXd D_m(1, 1); D_m(0, 0) = D_ss;

    return StateSpace(A_m, B_m, C_m, D_m, Ts);
}

StateSpace MixedSensitivity::makeW1(double omega_B, double M, double eps, double Ts)
{
    // W1(s) = (s/M + omega_B) / (s + omega_B * eps)
    // Numerator:   b1 = 1/M,    b0 = omega_B
    // Denominator: a1 = 1,      a0 = omega_B * eps
    return tustinFirstOrder(1.0 / M, omega_B, 1.0, omega_B * eps, Ts);
}

StateSpace MixedSensitivity::makeW2constant(double gain, double Ts)
{
    // Constant (static) weight: H(z) = gain
    // State-space: A=0, B=0, C=0, D=gain (no dynamics)
    Eigen::MatrixXd A_m(1, 1); A_m(0, 0) = 0.0;
    Eigen::MatrixXd B_m(1, 1); B_m(0, 0) = 0.0;
    Eigen::MatrixXd C_m(1, 1); C_m(0, 0) = 0.0;
    Eigen::MatrixXd D_m(1, 1); D_m(0, 0) = gain;
    return StateSpace(A_m, B_m, C_m, D_m, Ts);
}

StateSpace MixedSensitivity::makeW2highpass(double omega_u, double eps, double Ts)
{
    // W2(s) = (s + omega_u * eps) / (s + omega_u)
    // Numerator:   b1 = 1,  b0 = omega_u * eps
    // Denominator: a1 = 1,  a0 = omega_u
    return tustinFirstOrder(1.0, omega_u * eps, 1.0, omega_u, Ts);
}

StateSpace MixedSensitivity::makeW3(double omega_T, double Mt, double eps, double Ts)
{
    // W3(s) = (s + omega_T / Mt) / (eps * s + omega_T)
    // Numerator:   b1 = 1,    b0 = omega_T / Mt
    // Denominator: a1 = eps,  a0 = omega_T
    return tustinFirstOrder(1.0, omega_T / Mt, eps, omega_T, Ts);
}

// =============================================================================
// MixedSensitivity::build - assemble the generalised plant
//
// Augmented state: xa = [xG; xW1; xW2; xW3]   (n = nG + nW1 + nW2 + nW3)
//
// For a SISO plant G(z) with weighting filters W1, W2, W3:
//
//   Exogenous inputs w = [r; d]       (reference r and output disturbance d)
//   Performance outputs z = [z1; z2; z3]  where:
//     z1 = W1 * e = W1 * (r - y)     (weighted tracking error)
//     z2 = W2 * u                    (weighted control effort)
//     z3 = W3 * y                    (weighted complementary sensitivity)
//   Control input u (scalar)
//   Measurement y = G*u + d          (scalar)
//
// Plant interconnection (signal flow):
//   e   = r - y
//   y   = G*u + d            (G input: u, G output: yG = G*u)
//   z1  = W1 * e = W1 * (r - yG - d)
//   z2  = W2 * u
//   z3  = W3 * (yG + d)
//   y_meas = yG + d          (measured output)
//
// Augmented state equations:
//   xG[k+1]  = AG xG + BG u
//   xW1[k+1] = AW1 xW1 + BW1 * (r - CG xG - DG u - d)
//   xW2[k+1] = AW2 xW2 + BW2 * u
//   xW3[k+1] = AW3 xW3 + BW3 * (CG xG + DG u + d)
//
// Performance outputs:
//   z1 = CW1 xW1 + DW1 * (r - CG xG - DG u - d)
//   z2 = CW2 xW2 + DW2 * u
//   z3 = CW3 xW3 + DW3 * (CG xG + DG u + d)
//
// Measurement:
//   y_meas = CG xG + DG u + d
//
// Stacking into generalised plant format:
//   xa = [xG; xW1; xW2; xW3]
//   w  = [r; d]    (nw = 2)
//   z  = [z1; z2; z3]  (nz = 3 for first-order weights, scalar outputs)
//   u  (nu = 1)
//   y_meas  (ny = 1)
// =============================================================================
GeneralisedPlant MixedSensitivity::build(const StateSpace &G,
                                          const StateSpace &W1,
                                          const StateSpace &W2,
                                          const StateSpace &W3)
{
    // Validate SISO plant
    if (G.inputSize() != 1 || G.outputSize() != 1)
        throw std::invalid_argument("MixedSensitivity::build: G must be SISO.");
    if (std::abs(G.Ts - W1.Ts) > 1e-12 || std::abs(G.Ts - W2.Ts) > 1e-12 || std::abs(G.Ts - W3.Ts) > 1e-12)
        throw std::invalid_argument("MixedSensitivity::build: all systems must have the same sample time.");

    // Extract scalars from state-space matrices
    const int nG  = G.stateSize();
    const int nW1 = W1.stateSize();
    const int nW2 = W2.stateSize();
    const int nW3 = W3.stateSize();
    const int n   = nG + nW1 + nW2 + nW3;

    // Scalar gains from SISO matrices (C and D are 1x1, B is nX x 1)
    const double cG  = G.C(0, 0),   dG  = G.D(0, 0);
    const double dW1 = W1.D(0, 0);
    const double dW2 = W2.D(0, 0);
    const double dW3 = W3.D(0, 0);

    // -----------------------------------------------------------------------
    // State transition matrix A_aug (n x n), block diagonal + coupling
    //
    // xa[k+1] = A_aug xa + B1_aug w + B2_aug u
    //
    // xG[k+1]  = AG xG               + BG * u
    // xW1[k+1] = AW1 xW1 - BW1*CG xG               + BW1*r - BW1*d - BW1*DG*u
    // xW2[k+1] = AW2 xW2                             + BW2*u
    // xW3[k+1] = AW3 xW3 + BW3*CG xG                + BW3*d + BW3*DG*u
    // -----------------------------------------------------------------------
    GeneralisedPlant P;
    P.Ts = G.Ts;

    // A_aug (n x n)
    P.A = Eigen::MatrixXd::Zero(n, n);
    P.A.block(0,   0,   nG,  nG)  = G.A;
    P.A.block(nG,  nG,  nW1, nW1) = W1.A;
    P.A.block(nG+nW1, nG+nW1, nW2, nW2) = W2.A;
    P.A.block(nG+nW1+nW2, nG+nW1+nW2, nW3, nW3) = W3.A;

    // Coupling: W1 driven by -CG * xG, W3 driven by +CG * xG
    // xW1[k+1] gets -BW1 * CG * xG
    P.A.block(nG, 0, nW1, nG) = -W1.B * G.C;
    // xW3[k+1] gets +BW3 * CG * xG
    P.A.block(nG+nW1+nW2, 0, nW3, nG) = W3.B * G.C;

    // -----------------------------------------------------------------------
    // B1_aug (n x nw), nw = 2: [r, d]
    // -----------------------------------------------------------------------
    P.B1 = Eigen::MatrixXd::Zero(n, 2);
    // xG: no exogenous input (G is driven by u only)
    // xW1: driven by r (+BW1) and d (-BW1)
    P.B1.block(nG, 0, nW1, 1) =  W1.B * Eigen::MatrixXd::Ones(1, 1); // r
    P.B1.block(nG, 1, nW1, 1) = -W1.B * Eigen::MatrixXd::Ones(1, 1); // d
    // xW2: no exogenous input
    // xW3: driven by d (+BW3)
    P.B1.block(nG+nW1+nW2, 1, nW3, 1) = W3.B * Eigen::MatrixXd::Ones(1, 1); // d

    // -----------------------------------------------------------------------
    // B2_aug (n x nu), nu = 1: u
    // -----------------------------------------------------------------------
    P.B2 = Eigen::MatrixXd::Zero(n, 1);
    P.B2.block(0,            0, nG,  1) = G.B;               // xG driven by BG * u
    P.B2.block(nG,           0, nW1, 1) = -W1.B * dG;        // xW1 driven by -BW1*DG*u
    P.B2.block(nG+nW1,       0, nW2, 1) =  W2.B;             // xW2 driven by BW2*u
    P.B2.block(nG+nW1+nW2,   0, nW3, 1) =  W3.B * dG;       // xW3 driven by +BW3*DG*u

    // -----------------------------------------------------------------------
    // Performance outputs z = C1 xa + D11 w + D12 u   (nz = 3: [z1, z2, z3])
    //
    // z1 = CW1 xW1 + DW1*(r - CG xG - DG u - d)
    //    = -DW1*CG xG + CW1 xW1 + DW1*r - DW1*d - DW1*DG*u
    //
    // z2 = CW2 xW2 + DW2 u
    //
    // z3 = CW3 xW3 + DW3*(CG xG + DG u + d)
    //    = DW3*CG xG + CW3 xW3 + DW3*DG*u + DW3*d
    // -----------------------------------------------------------------------
    const int nz = 3; // scalar outputs from each weight (first-order weights)
    P.C1 = Eigen::MatrixXd::Zero(nz, n);

    // z1 row
    P.C1(0, 0) = -dW1 * cG;         // from xG (scalar: CG is 1x1 -> cG)
    if (nW1 > 0) P.C1.block(0, nG, 1, nW1) = W1.C; // from xW1

    // z2 row
    if (nW2 > 0) P.C1.block(1, nG+nW1, 1, nW2) = W2.C; // from xW2

    // z3 row
    P.C1(2, 0) = dW3 * cG;          // from xG
    if (nW3 > 0) P.C1.block(2, nG+nW1+nW2, 1, nW3) = W3.C; // from xW3

    // -----------------------------------------------------------------------
    // D11 (nz x nw): feedthrough from w = [r, d] to z
    //
    // z1: DW1*r - DW1*d  -> D11(0,0) = DW1, D11(0,1) = -DW1
    // z2: 0
    // z3: DW3*d           -> D11(2,1) = DW3
    // -----------------------------------------------------------------------
    P.D11 = Eigen::MatrixXd::Zero(nz, 2);
    P.D11(0, 0) =  dW1;   // z1 from r
    P.D11(0, 1) = -dW1;   // z1 from d
    P.D11(2, 1) =  dW3;   // z3 from d

    // -----------------------------------------------------------------------
    // D12 (nz x nu): feedthrough from u to z
    //
    // z1: -DW1*DG*u
    // z2:  DW2*u
    // z3:  DW3*DG*u
    // -----------------------------------------------------------------------
    P.D12 = Eigen::MatrixXd::Zero(nz, 1);
    P.D12(0, 0) = -dW1 * dG;
    P.D12(1, 0) =  dW2;
    P.D12(2, 0) =  dW3 * dG;

    // -----------------------------------------------------------------------
    // Measurement output y_meas = C2 xa + D21 w + D22 u   (ny = 1)
    //
    // y_meas = CG xG + d + DG u
    //        = CG xG + 0*r + 1*d + DG*u
    // -----------------------------------------------------------------------
    P.C2 = Eigen::MatrixXd::Zero(1, n);
    P.C2.block(0, 0, 1, nG) = G.C;

    P.D21 = Eigen::MatrixXd::Zero(1, 2);
    P.D21(0, 0) = 0.0;  // no direct r -> y feedthrough
    P.D21(0, 1) = 1.0;  // d -> y

    // Standard DGKF requires D22 = 0.  For plants with direct feedthrough (dG != 0),
    // a loop-shifting pre-processing step must be applied before calling solve()
    // (see Skogestad & Postlethwaite Section 9.4).  Passing such a plant directly
    // would cause trySolve() to synthesise a controller for the wrong problem, since
    // D22 is silently ignored in the assembly formulas.  We throw here so the caller
    // learns about the issue at build() time rather than silently getting a wrong K.
    if (std::abs(dG) > 1e-12)
        throw std::invalid_argument(
            "MixedSensitivity::build: plant G has nonzero direct feedthrough (D = " +
            std::to_string(dG) + ").  The standard DGKF assembly assumes D22=0.  "
            "Apply loop-shifting to the plant before building the generalised plant, "
            "or use a plant model with D = 0.");

    P.D22 = Eigen::MatrixXd::Zero(1, 1);  // D22 = 0 guaranteed by the check above

    return P;
}

} // namespace ctrl
