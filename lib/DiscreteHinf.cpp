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
DareResult DiscreteHinf::solveHinfDARE(
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    double /*tol*/, int /*maxIter*/)
{
    DareResult out;
    out.converged  = false;
    out.iterations = 1;

    const int n = A.rows();

    // R must be invertible (caller has already verified this).
    Eigen::FullPivLU<Eigen::MatrixXd> luR(R);
    if (!luR.isInvertible())
    {
        out.P = Eigen::MatrixXd::Zero(n, n);
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
        out.P = Eigen::MatrixXd::Zero(n, n);
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
        out.P = Eigen::MatrixXd::Zero(n, n);
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
        out.P = Eigen::MatrixXd::Zero(n, n);
        return out;
    }

    const Eigen::MatrixXcd X_c = V2 * luV1.inverse();
    out.P = X_c.real();
    out.P = 0.5 * (out.P + out.P.transpose()); // enforce symmetry

    // Verify the DARE residual: ||A'PA - P + Q - A'PB * (R+B'PB)^{-1} * B'PA|| / (1+||P||)
    const Eigen::MatrixXd &X = out.P;
    const Eigen::MatrixXd Rbar = R + B.transpose() * X * B;
    Eigen::FullPivLU<Eigen::MatrixXd> luRbar(Rbar);
    if (!luRbar.isInvertible())
    {
        out.P = Eigen::MatrixXd::Zero(n, n);
        return out;
    }
    const Eigen::MatrixXd K     = luRbar.inverse() * B.transpose() * X * A;
    const Eigen::MatrixXd resid = A.transpose() * X * A - X + Q - A.transpose() * X * B * K;
    const double dare_res = resid.norm() / (1.0 + X.norm());
    out.converged = (dare_res < 1e-6) && X.allFinite();
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

    DareResult dx = solveHinfDARE(A, Bcat, Qx, Rx, dareTol, dareMaxIter);
    out.dareConvX  = dx.converged;
    out.dareItersX = dx.iterations;
    if (!dx.converged) return false;

    const Eigen::MatrixXd &X = dx.P;
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

    DareResult dy = solveHinfDARE(A.transpose(), Ccat.transpose(), Qy, Ry, dareTol, dareMaxIter);
    out.dareConvY  = dy.converged;
    out.dareItersY = dy.iterations;

    if (!dy.converged) return false;

    const Eigen::MatrixXd &Y = dy.P;
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

// =============================================================================
// mu-synthesis: DK-iteration with constant amplitude-balance D-scaling
// =============================================================================

// -----------------------------------------------------------------------------
// buildClosedLoop - lower LFT F_l(P, K): w -> z state-space
//
// Given P and K (xk+ = Ak*xk + Bk*y, u = Ck*xk + Dk*y), compute:
//   [x;  xk]+ = A_cl [x; xk] + B_cl w
//   z         = C_cl [x; xk] + D_cl w
//
// Derivation: eliminate y and u from the plant/controller equations.
//   u = Dk*(C2 x + D21 w + D22 u) + Ck*xk  =>  u = E*(Dk*C2*x + Dk*D21*w + Ck*xk)
//   where E = (I - Dk*D22)^{-1}
// -----------------------------------------------------------------------------
void DiscreteHinf::buildClosedLoop(
    const GeneralisedPlant &P,
    const Eigen::MatrixXd  &Ak, const Eigen::MatrixXd &Bk,
    const Eigen::MatrixXd  &Ck, const Eigen::MatrixXd &Dk,
    Eigen::MatrixXd &A_cl, Eigen::MatrixXd &B_cl,
    Eigen::MatrixXd &C_cl, Eigen::MatrixXd &D_cl)
{
    const int n  = P.stateSize();
    const int nk = static_cast<int>(Ak.rows());
    const int nu = P.nu();
    const int nc = n + nk;

    // E = (I - Dk*D22)^{-1}
    const Eigen::MatrixXd I_nu = Eigen::MatrixXd::Identity(nu, nu);
    const Eigen::MatrixXd E    = (I_nu - Dk * P.D22).inverse();

    // Pre-compute common products
    const Eigen::MatrixXd EDkC2  = E * Dk * P.C2;   // nu x n
    const Eigen::MatrixXd EDkD21 = E * Dk * P.D21;  // nu x nw
    const Eigen::MatrixXd ECk    = E * Ck;           // nu x nk

    A_cl.resize(nc, nc);
    B_cl.resize(nc, P.nw());
    C_cl.resize(P.nz(), nc);
    D_cl.resize(P.nz(), P.nw());

    A_cl.topLeftCorner(n, n)       = P.A   + P.B2 * EDkC2;
    A_cl.topRightCorner(n, nk)     = P.B2  * ECk;
    A_cl.bottomLeftCorner(nk, n)   = Bk    * P.C2 + Bk * P.D22 * EDkC2;
    A_cl.bottomRightCorner(nk, nk) = Ak    + Bk * P.D22 * ECk;

    B_cl.topRows(n)    = P.B1  + P.B2 * EDkD21;
    B_cl.bottomRows(nk) = Bk   * P.D21 + Bk * P.D22 * EDkD21;

    C_cl.leftCols(n)   = P.C1  + P.D12 * EDkC2;
    C_cl.rightCols(nk) = P.D12 * ECk;

    D_cl = P.D11 + P.D12 * EDkD21;
}

// -----------------------------------------------------------------------------
// evalFreqResponse - M(z) = C(zI-A)^{-1}B + D at z = e^{j*w*Ts}
// -----------------------------------------------------------------------------
Eigen::MatrixXcd DiscreteHinf::evalFreqResponse(
    const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &C, const Eigen::MatrixXd &D,
    double w, double Ts)
{
    const int n = static_cast<int>(A.rows());
    const std::complex<double> z(std::cos(w * Ts), std::sin(w * Ts));
    const Eigen::MatrixXcd zImA =
        z * Eigen::MatrixXcd::Identity(n, n) - A.cast<std::complex<double>>();
    return C.cast<std::complex<double>>()
           * zImA.colPivHouseholderQr().solve(B.cast<std::complex<double>>())
           + D.cast<std::complex<double>>();
}

// -----------------------------------------------------------------------------
// amplitudeBalance - Sinkhorn-Knopp iteration to equalize row/col norms of
//                   diag(d_L) * Mabs * diag(1/d_R).
//
// Initialises d_L = 1_{nz}, d_R = 1_{nw} then alternates:
//   Row step: d_L[i] set so row i norm = geometric-mean row norm.
//   Col step: d_R[j] set so col j norm = geometric-mean col norm.
// Convergence: norms are equalised, minimising an upper bound on sigma_max.
// -----------------------------------------------------------------------------
void DiscreteHinf::amplitudeBalance(const Eigen::MatrixXd &Mabs,
                                     int max_iter,
                                     Eigen::VectorXd &d_L,
                                     Eigen::VectorXd &d_R)
{
    const int nz = static_cast<int>(Mabs.rows());
    const int nw = static_cast<int>(Mabs.cols());
    d_L = Eigen::VectorXd::Ones(nz);
    d_R = Eigen::VectorXd::Ones(nw);

    for (int iter = 0; iter < max_iter; ++iter) {
        // Row step: compute unnormalised row norms (without d_L, with 1/d_R)
        Eigen::VectorXd raw_row(nz);
        for (int i = 0; i < nz; ++i) {
            double s = 0.0;
            for (int j = 0; j < nw; ++j) { double v = Mabs(i,j)/d_R(j); s += v*v; }
            raw_row(i) = std::sqrt(std::max(s, 1e-32));
        }
        const double gm_row = std::exp(raw_row.array().log().mean());
        for (int i = 0; i < nz; ++i)
            d_L(i) = (raw_row(i) > 1e-16) ? std::sqrt(gm_row) / raw_row(i) : 1.0;

        // Col step: compute col norms with updated d_L
        Eigen::VectorXd raw_col(nw);
        for (int j = 0; j < nw; ++j) {
            double s = 0.0;
            for (int i = 0; i < nz; ++i) { double v = d_L(i)*Mabs(i,j); s += v*v; }
            raw_col(j) = std::sqrt(std::max(s, 1e-32));
        }
        const double gm_col = std::exp(raw_col.array().log().mean());
        for (int j = 0; j < nw; ++j)
            d_R(j) = (gm_col > 1e-16) ? raw_col(j) / std::sqrt(gm_col) : 1.0;
    }
}

// -----------------------------------------------------------------------------
// applyDScaling - scale P's performance/exogenous channels by constant D_L, D_R.
//
// The resulting plant P_scaled satisfies:
//   F_l(P_scaled, K) = D_L * F_l(P, K) * D_R^{-1}
//
// Only the channels connected to uncertainty (z, w) are touched; the
// measurement y and control u channels (C2, B2, D22) are unchanged.
// -----------------------------------------------------------------------------
GeneralisedPlant DiscreteHinf::applyDScaling(const GeneralisedPlant &P,
                                              const Eigen::VectorXd  &d_L,
                                              const Eigen::VectorXd  &d_R)
{
    GeneralisedPlant Ps = P;
    Ps.B1  = P.B1 * d_R.asDiagonal();          // B1 * D_R
    Ps.C1  = d_L.asDiagonal() * P.C1;          // D_L * C1
    Ps.D11 = d_L.asDiagonal() * P.D11 * d_R.asDiagonal(); // D_L * D11 * D_R
    Ps.D12 = d_L.asDiagonal() * P.D12;         // D_L * D12
    Ps.D21 = P.D21 * d_R.asDiagonal();         // D21 * D_R
    return Ps;
}

// =============================================================================
// fitFirstOrderDFilter - fit D_j(z) = K*(z-z0)/(z-p) from DC and Nyquist gains
//
// Given two magnitude values d_lo (approx at w~0) and d_hi (approx at w~pi/Ts),
// choose pole p=0.85 (moderate bandwidth) and solve for K and z0 analytically:
//   |D_j(1)|  = K * |1-z0| / |1-p|   = d_lo
//   |D_j(-1)| = K * |1+z0| / |1+p|   = d_hi
//
// Defining r = (d_lo/d_hi) * (1+p)/(1-p):
//   |1-z0| / |1+z0| = r
//   For z0 in (-1,1): z0 = (1-r)/(1+r)
//   K = d_lo * (1-p) / |1-z0|
//
// The resulting D_j(z) is a minimum-phase lead-lag with real coefficients.
// Returns StateSpace in controllable canonical form: A=p, B=1, C=K*(p-z0), D=K.
// =============================================================================
StateSpace DiscreteHinf::fitFirstOrderDFilter(double d_lo, double d_hi, double Ts)
{
    // Guard against zero or equal gains -> fall back to constant filter (pure gain)
    const double eps = 1e-12;
    if (d_lo < eps || d_hi < eps || std::abs(d_lo - d_hi) < eps * (d_lo + d_hi + 1.0)) {
        const double d_avg = 0.5 * (d_lo + d_hi);
        Eigen::MatrixXd A1(1,1); A1(0,0) = 0.0;
        Eigen::MatrixXd B1(1,1); B1(0,0) = 0.0;
        Eigen::MatrixXd C1(1,1); C1(0,0) = 0.0;
        Eigen::MatrixXd D1(1,1); D1(0,0) = d_avg;
        return StateSpace(A1, B1, C1, D1, Ts);
    }

    const double p  = 0.85;    // stable pole inside unit disk
    const double r  = (d_lo / d_hi) * (1.0 + p) / (1.0 - p);
    // z0 = (1-r)/(1+r), clamped to (-0.99, 0.99) for numerical stability
    double z0 = std::max(-0.99, std::min(0.99, (1.0 - r) / (1.0 + r)));
    const double K  = d_lo * (1.0 - p) / std::max(std::abs(1.0 - z0), eps);

    // Controllable canonical form for D_j(z) = K*(z-z0)/(z-p)
    // x[k+1] = p*x[k] + u[k],  y[k] = K*(p-z0)*x[k] + K*u[k]
    Eigen::MatrixXd A1(1,1); A1(0,0) = p;
    Eigen::MatrixXd B1(1,1); B1(0,0) = 1.0;
    Eigen::MatrixXd C1(1,1); C1(0,0) = K * (p - z0);
    Eigen::MatrixXd D1(1,1); D1(0,0) = K;
    return StateSpace(A1, B1, C1, D1, Ts);
}

// =============================================================================
// applyRationalDScaling - augment plant with first-order D_L(z) and D_R(z)
//
// New augmented state: x_aug = [x; xDL_1; ...; xDL_nz; xDR_1; ...; xDR_nw]
// where xDL_j is the state of D_L_j(z) (performance output scaling) and
// xDR_j is the state of D_R_j(z) (exogenous input scaling).
//
// Augmented dynamics (for each D filter in controllable canonical form
// A_Dj = p_j, B_Dj = 1, C_Dj = K_j*(p_j-z0_j), D_Dj = K_j):
//
//   x[k+1]      = A*x      + B1 * (D_DR*xDR + D_DRgain*w_d) + B2*u
//   xDL_j[k+1]  = p_Lj*xDL_j + C1_j*x + D11_j*(D_DR*xDR + D_DRgain*w_d) + D12_j*u
//   xDR_j[k+1]  = p_Rj*xDR_j + w_d_j
//
// Performance output: z_aug_j = C_DLj*xDL_j + K_Lj * (C1_j*x + D11_j*w + D12_j*u)
// Measurement output: y = C2*x + D21*(D_DR*xDR + D_DRgain*w_d) + D22*u
// =============================================================================
GeneralisedPlant DiscreteHinf::applyRationalDScaling(
    const GeneralisedPlant        &P,
    const std::vector<StateSpace> &dL,
    const std::vector<StateSpace> &dR)
{
    const int n  = P.stateSize();
    const int nz = P.nz(), nw = P.nw(), nu = P.nu(), ny = P.ny();

    if ((int)dL.size() != nz || (int)dR.size() != nw)
        throw std::invalid_argument("applyRationalDScaling: filter count mismatch.");

    const int n_aug = n + nz + nw;

    // Extract D_L scalar parameters: p_Lj, K_Lj, CL_j = K_Lj*(p_Lj-z0_Lj)
    Eigen::VectorXd p_L(nz), K_L(nz), CL_scalar(nz);
    for (int j = 0; j < nz; ++j) {
        p_L(j)       = dL[j].A(0,0);
        K_L(j)       = dL[j].D(0,0);
        CL_scalar(j) = dL[j].C(0,0);  // = K_Lj*(p_Lj - z0_Lj)
    }

    // Extract D_R scalar parameters
    Eigen::VectorXd p_R(nw), K_R(nw), CR_scalar(nw);
    for (int j = 0; j < nw; ++j) {
        p_R(j)       = dR[j].A(0,0);
        K_R(j)       = dR[j].D(0,0);
        CR_scalar(j) = dR[j].C(0,0);  // = K_Rj*(p_Rj - z0_Rj)
    }

    // Diagonal gain matrices
    const Eigen::MatrixXd D_DRgain = K_R.asDiagonal();  // nw x nw: w = D_DRgain*w_d + ...
    const Eigen::MatrixXd D_DR     = CR_scalar.asDiagonal(); // nw x nw: coupling from xDR
    const Eigen::MatrixXd D_DLgain = K_L.asDiagonal();   // nz x nz

    // Augmented A matrix (n_aug x n_aug)
    // Rows: [x block, xDL block, xDR block]
    // Cols: [x,       xDL,       xDR      ]
    Eigen::MatrixXd A_aug = Eigen::MatrixXd::Zero(n_aug, n_aug);
    // x row: x[k+1] = A*x + B1*(D_DR*xDR + D_DRgain*w_d) + B2*u
    A_aug.block(0, 0, n, n)         = P.A;
    A_aug.block(0, n + nz, n, nw)   = P.B1 * D_DR;

    // xDL row: xDL_j[k+1] = p_Lj*xDL_j + C1_j*x + D11_j*(D_DR*xDR + D_DRgain*w_d) + D12_j*u
    A_aug.block(n, 0, nz, n)         = P.C1;
    A_aug.block(n, n, nz, nz)        = p_L.asDiagonal();
    A_aug.block(n, n + nz, nz, nw)   = P.D11 * D_DR;

    // xDR row: xDR_j[k+1] = p_Rj*xDR_j + w_d_j
    A_aug.block(n + nz, n + nz, nw, nw) = p_R.asDiagonal();

    // Augmented B1 (for w_d), size n_aug x nw
    Eigen::MatrixXd B1_aug = Eigen::MatrixXd::Zero(n_aug, nw);
    B1_aug.block(0, 0, n, nw)    = P.B1 * D_DRgain;
    B1_aug.block(n, 0, nz, nw)   = P.D11 * D_DRgain;
    B1_aug.block(n + nz, 0, nw, nw) = Eigen::MatrixXd::Identity(nw, nw);

    // Augmented B2 (for u), size n_aug x nu
    Eigen::MatrixXd B2_aug = Eigen::MatrixXd::Zero(n_aug, nu);
    B2_aug.block(0, 0, n, nu)  = P.B2;
    B2_aug.block(n, 0, nz, nu) = P.D12;

    // Augmented C1 (performance output z_aug_j = CL_j*xDL_j + K_Lj*(C1_j*x + ...))
    // z_aug = D_DLgain*C1*x + CL_scalar*xDL + D_DLgain*D11*(D_DR*xDR + D_DRgain*w_d) + D_DLgain*D12*u
    Eigen::MatrixXd C1_aug = Eigen::MatrixXd::Zero(nz, n_aug);
    C1_aug.block(0, 0, nz, n)       = D_DLgain * P.C1;
    C1_aug.block(0, n, nz, nz)      = CL_scalar.asDiagonal();
    C1_aug.block(0, n + nz, nz, nw) = D_DLgain * P.D11 * D_DR;

    // Augmented C2 (measurement output y = C2*x + D21*(D_DR*xDR + ...) + D22*u)
    Eigen::MatrixXd C2_aug = Eigen::MatrixXd::Zero(ny, n_aug);
    C2_aug.block(0, 0, ny, n)       = P.C2;
    C2_aug.block(0, n + nz, ny, nw) = P.D21 * D_DR;

    // Augmented D11 (performance from w_d): nz x nw
    const Eigen::MatrixXd D11_aug = D_DLgain * P.D11 * D_DRgain;

    // Augmented D12 (performance from u): nz x nu
    const Eigen::MatrixXd D12_aug = D_DLgain * P.D12;

    // Augmented D21 (measurement from w_d): ny x nw
    const Eigen::MatrixXd D21_aug = P.D21 * D_DRgain;

    GeneralisedPlant Paug;
    Paug.Ts  = P.Ts;
    Paug.A   = A_aug;
    Paug.B1  = B1_aug;
    Paug.B2  = B2_aug;
    Paug.C1  = C1_aug;
    Paug.C2  = C2_aug;
    Paug.D11 = D11_aug;
    Paug.D12 = D12_aug;
    Paug.D21 = D21_aug;
    Paug.D22 = P.D22;
    return Paug;
}

// -----------------------------------------------------------------------------
// solveMuSyn - outer DK-iteration loop
// Constant D-scaling (useRationalD=false) or rational D-scaling (useRationalD=true).
// -----------------------------------------------------------------------------
MuSynResult DiscreteHinf::solveMuSyn(const GeneralisedPlant &P,
                                       const MuSynParams &params)
{
    MuSynResult best;
    best.achievedMuUpper = std::numeric_limits<double>::infinity();

    const int nz = P.nz(), nw = P.nw();
    const double Ts = P.Ts;
    const double pi = std::acos(-1.0);

    // Initial D-scaling: identity (constant)
    Eigen::VectorXd d_L = Eigen::VectorXd::Ones(nz);
    Eigen::VectorXd d_R = Eigen::VectorXd::Ones(nw);

    // For rational D-scaling: track per-channel DC and Nyquist values
    // d_lo_L[j] = D_L_j at low frequencies, d_hi_L[j] at Nyquist
    Eigen::VectorXd d_lo_L = Eigen::VectorXd::Ones(nz);
    Eigen::VectorXd d_hi_L = Eigen::VectorXd::Ones(nz);
    Eigen::VectorXd d_lo_R = Eigen::VectorXd::Ones(nw);
    Eigen::VectorXd d_hi_R = Eigen::VectorXd::Ones(nw);

    // Current rational D filters (initially pure-gain = constant D)
    std::vector<StateSpace> cur_dL, cur_dR;
    if (params.useRationalD) {
        for (int j = 0; j < nz; ++j)
            cur_dL.push_back(fitFirstOrderDFilter(1.0, 1.0, Ts));
        for (int j = 0; j < nw; ++j)
            cur_dR.push_back(fitFirstOrderDFilter(1.0, 1.0, Ts));
    }

    double mu_prev = std::numeric_limits<double>::infinity();

    for (int dk = 0; dk < params.maxDKIter; ++dk) {

        // ----------------------------------------------------------------
        // K-step: H-inf synthesis on D-scaled plant
        // ----------------------------------------------------------------
        GeneralisedPlant Ps;
        if (params.useRationalD && !cur_dL.empty())
            Ps = applyRationalDScaling(P, cur_dL, cur_dR);
        else
            Ps = applyDScaling(P, d_L, d_R);

        const HinfResult hr = DiscreteHinf::solve(Ps, params.hinfParams);
        if (!hr.feasible) break;

        // ----------------------------------------------------------------
        // Build closed-loop F_l(P, K) using the UNSCALED original plant
        // ----------------------------------------------------------------
        Eigen::MatrixXd A_cl, B_cl, C_cl, D_cl;
        buildClosedLoop(P, hr.Ak, hr.Bk, hr.Ck, hr.Dk,
                        A_cl, B_cl, C_cl, D_cl);

        // ----------------------------------------------------------------
        // Evaluate mu upper bound over log-spaced frequency grid.
        // Accumulate: M_peak (element-wise max), M_lo (low-half geo mean),
        // M_hi (high-half geo mean) for rational D fitting.
        // ----------------------------------------------------------------
        const int nf      = params.nFreqPoints;
        const int half_nf = nf / 2;

        Eigen::MatrixXd M_peak  = Eigen::MatrixXd::Zero(nz, nw);
        Eigen::MatrixXd M_lo    = Eigen::MatrixXd::Zero(nz, nw);  // geometric accumulator (log sum)
        Eigen::MatrixXd M_hi    = Eigen::MatrixXd::Zero(nz, nw);
        double mu_upper         = 0.0;

        for (int fi = 0; fi < nf; ++fi) {
            const double w = (pi / Ts)
                           * std::pow(0.01, 1.0 - static_cast<double>(fi) / (nf - 1));

            const Eigen::MatrixXcd Mw = evalFreqResponse(A_cl, B_cl, C_cl, D_cl, w, Ts);

            for (int i = 0; i < nz; ++i) {
                for (int j = 0; j < nw; ++j) {
                    const double abs_ij = std::abs(Mw(i, j));
                    M_peak(i, j) = std::max(M_peak(i, j), abs_ij);
                    // accumulate log for geometric mean in each half
                    if (fi < half_nf)
                        M_lo(i, j) += std::log(std::max(abs_ij, 1e-30));
                    else
                        M_hi(i, j) += std::log(std::max(abs_ij, 1e-30));
                }
            }

            // sigma_max(D_L * Mw * D_R^{-1}) with current constant d_L, d_R
            Eigen::MatrixXcd Mw_sc = Mw;
            for (int i = 0; i < nz; ++i)
                for (int j = 0; j < nw; ++j)
                    Mw_sc(i, j) *= d_L(i) / d_R(j);
            Eigen::JacobiSVD<Eigen::MatrixXcd> svd(Mw_sc, Eigen::ComputeThinU | Eigen::ComputeThinV);
            mu_upper = std::max(mu_upper, svd.singularValues()(0));
        }

        // ----------------------------------------------------------------
        // Track best result
        // ----------------------------------------------------------------
        if (mu_upper < best.achievedMuUpper) {
            best.achievedMuUpper = mu_upper;
            best.hinfResult      = hr;
            if (params.useRationalD) {
                best.dFilters_L = cur_dL;
                best.dFilters_R = cur_dR;
            }
        }
        best.iterations = dk + 1;
        best.muHistory.push_back(mu_upper);

        // ----------------------------------------------------------------
        // Convergence check
        // ----------------------------------------------------------------
        if (mu_prev < std::numeric_limits<double>::infinity()) {
            const double rel_change = std::abs(mu_prev - mu_upper)
                                    / std::max(mu_prev, 1e-10);
            if (rel_change < params.muTol) { best.converged = true; break; }
        }
        mu_prev = mu_upper;

        // ----------------------------------------------------------------
        // D-step: amplitude balance (constant D for mu evaluation)
        // ----------------------------------------------------------------
        amplitudeBalance(M_peak, params.dScaleMaxIter, d_L, d_R);

        // ----------------------------------------------------------------
        // Rational D-step: fit first-order D_j(z) per channel
        // ----------------------------------------------------------------
        if (params.useRationalD) {
            const int nlo = std::max(1, half_nf);
            const int nhi = std::max(1, nf - half_nf);

            // Left D-scaling filters (for performance outputs)
            // Each filter j is driven by the j-th performance output z_j.
            // Use row-wise geometric mean of M_lo / M_hi for each output j.
            cur_dL.clear();
            for (int j = 0; j < nz; ++j) {
                // DC and Nyquist magnitudes for row j (max over nw input channels)
                double lo_j = 0.0, hi_j = 0.0;
                for (int k = 0; k < nw; ++k) {
                    lo_j += M_lo(j, k);
                    hi_j += M_hi(j, k);
                }
                const double d_lo_j = std::exp(lo_j / (nlo * nw));
                const double d_hi_j = std::exp(hi_j / (nhi * nw));
                cur_dL.push_back(fitFirstOrderDFilter(d_lo_j, d_hi_j, Ts));
                d_lo_L(j) = d_lo_j;
                d_hi_L(j) = d_hi_j;
            }

            // Right D-scaling filters (for exogenous inputs)
            cur_dR.clear();
            for (int j = 0; j < nw; ++j) {
                double lo_j = 0.0, hi_j = 0.0;
                for (int k = 0; k < nz; ++k) {
                    lo_j += M_lo(k, j);
                    hi_j += M_hi(k, j);
                }
                const double d_lo_j = std::exp(lo_j / (nlo * nz));
                const double d_hi_j = std::exp(hi_j / (nhi * nz));
                cur_dR.push_back(fitFirstOrderDFilter(d_lo_j, d_hi_j, Ts));
                d_lo_R(j) = d_lo_j;
                d_hi_R(j) = d_hi_j;
            }
        }
    }

    return best;
}

} // namespace ctrl
