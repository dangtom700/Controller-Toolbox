#include "HinfFilter.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ctrl {

namespace {

struct FilterCandidate
{
    Eigen::MatrixXd L;
    Eigen::MatrixXd P;
};

// Attempt the bordered-Riccati filter solve at a fixed gamma. Returns false if the DARE
// doesn't converge, Y isn't PSD, or Y's spectral radius isn't strictly below gamma^2.
bool tryFilterSolve(const StateSpace &plant, const Eigen::MatrixXd &Qw, const Eigen::MatrixXd &Rv,
                     double gamma, double dareTol, int dareMaxIter, FilterCandidate &out)
{
    const int n  = static_cast<int>(plant.A.rows());
    const int ny = static_cast<int>(plant.C.rows());

    Eigen::MatrixXd Cbar(ny + n, n);
    Cbar.topRows(ny)    = plant.C;
    Cbar.bottomRows(n)  = Eigen::MatrixXd::Identity(n, n);

    Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(ny + n, ny + n);
    Rbar.topLeftCorner(ny, ny)     = Rv;
    Rbar.bottomRightCorner(n, n)   = -(gamma * gamma) * Eigen::MatrixXd::Identity(n, n);

    const DareResult dr = DiscreteHinf::solveHinfDARE(
        plant.A.transpose(), Cbar.transpose(), Qw, Rbar, dareTol, dareMaxIter);
    if (!dr.converged)
        return false;

    const Eigen::MatrixXd &Y = dr.P;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(Y);
    if (eig.info() != Eigen::Success)
        return false;

    const double scale  = std::max(1.0, Y.cwiseAbs().maxCoeff());
    const double minEig = eig.eigenvalues().minCoeff();
    const double maxEig = eig.eigenvalues().maxCoeff();
    if (minEig < -1e-8 * scale)        return false; // not PSD
    if (maxEig >= gamma * gamma)       return false; // feasibility bound violated

    const Eigen::MatrixXd Rinnov = Rv + plant.C * Y * plant.C.transpose();
    Eigen::LDLT<Eigen::MatrixXd> ldlt(Rinnov);
    if (ldlt.info() != Eigen::Success)
        return false;

    const Eigen::MatrixXd M = plant.A * Y * plant.C.transpose(); // n x ny
    out.L = ldlt.solve(M.transpose()).transpose();                // n x ny
    out.P = Y;
    return true;
}

} // namespace

HinfFilter::HinfFilter(const HinfFilterResult &result)
{
    if (!result.feasible)
        throw std::invalid_argument(
            "HinfFilter: cannot construct from an infeasible HinfFilterResult.");

    A_     = result.plant.A;
    B_     = result.plant.B;
    C_     = result.plant.C;
    L_     = result.L;
    P_     = result.P;
    gamma_ = result.achievedGamma;
    x_     = Eigen::VectorXd::Zero(A_.rows());
}

void HinfFilter::predict(const Eigen::VectorXd &u)
{
    if (!u.allFinite()) return; // hold last state
    x_ = A_ * x_ + B_ * u;
}

void HinfFilter::update(const Eigen::VectorXd &y)
{
    if (!y.allFinite()) return; // hold last state
    x_ = x_ + L_ * (y - C_ * x_);
}

void HinfFilter::reset()
{
    x_.setZero();
}

HinfFilterResult HinfFilter::solve(const StateSpace &plant,
                                    const Eigen::MatrixXd &Qw, const Eigen::MatrixXd &Rv,
                                    const HinfFilterParams &params)
{
    // HinfFilterResult holds a StateSpace (no default constructor), so it cannot be
    // default-constructed and incrementally filled in - accumulate into plain locals
    // instead and build the result via aggregate-init only at each return point.
    if (plant.A.rows() == 0)
        throw std::invalid_argument("HinfFilter::solve: empty plant.");
    if (plant.C.rows() == 0)
        throw std::invalid_argument(
            "HinfFilter::solve: plant must have at least one measurement channel.");

    double gammaHi = params.gammaInit;
    const double gammaLoFixed = 1e-4;
    double gammaLo = gammaLoFixed;

    FilterCandidate candidate;
    bool hiOk = tryFilterSolve(plant, Qw, Rv, gammaHi, params.dareTol, params.dareMaxIter, candidate);
    if (!hiOk)
    {
        for (int d = 0; d < 10; ++d)
        {
            gammaHi *= 2.0;
            hiOk = tryFilterSolve(plant, Qw, Rv, gammaHi, params.dareTol, params.dareMaxIter, candidate);
            if (hiOk) break;
        }
        if (!hiOk)
            return HinfFilterResult{false, std::numeric_limits<double>::infinity(),
                                     Eigen::MatrixXd(), Eigen::MatrixXd(), plant};
    }

    double achievedGamma = gammaHi;
    Eigen::MatrixXd L = candidate.L;
    Eigen::MatrixXd P = candidate.P;

    for (int iter = 0; iter < params.maxIter; ++iter)
    {
        if (gammaHi - gammaLo < params.gammaTol) break;

        const double gammaMid = 0.5 * (gammaLo + gammaHi);
        FilterCandidate midCandidate;
        if (tryFilterSolve(plant, Qw, Rv, gammaMid, params.dareTol, params.dareMaxIter, midCandidate))
        {
            gammaHi       = gammaMid;
            achievedGamma = gammaMid;
            L             = midCandidate.L;
            P             = midCandidate.P;
        }
        else
        {
            gammaLo = gammaMid;
        }
    }

    return HinfFilterResult{true, achievedGamma, L, P, plant};
}

} // namespace ctrl
