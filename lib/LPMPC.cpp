#include "LPMPC.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace ctrl
{

LPMPC::LPMPC(const StateSpace &plant, const LPMPCParams &params)
    : plant_(plant), p_(params), Ts_(plant.Ts)
{
    if (plant.inputSize() != 1 || plant.outputSize() != 1)
        throw std::invalid_argument("LPMPC: plant must be SISO (1 input, 1 output)");
    if (p_.Nc < 1 || p_.Np < p_.Nc)
        throw std::invalid_argument("LPMPC: require Nc >= 1 and Np >= Nc");

#ifndef NDEBUG
    if (plant.D.norm() > 1e-12)
        std::cerr << "[LPMPC] WARNING: plant.D != 0. The compute(error) SISO wrapper uses "
                     "u[k-1] for the D*u term (one step stale). Use computeRef(x, r) directly "
                     "for D != 0 plants.\n";
#endif

    x_hat_ = Eigen::VectorXd::Zero(plant_.stateSize());
    buildCondensedMatrices();
}

// ---------------------------------------------------------------------------
// buildCondensedMatrices - rebuilds the SISO prediction matrices (F, Phi, Gu) identically to
// DiscreteMPC::buildPredictionMatrices (specialized to m=p=1), then the LP's structural pieces:
// A_ineq_ (Phi_/identity blocks, constant until setPlant/setParams) and c_ (cost weights) and
// the static (t_y, t_u) portions of lb_/ub_. The DeltaU portion of lb_/ub_ and b_ineq_'s
// rhs1-dependent rows are left as-is here; computeRef() overwrites them every call.
// ---------------------------------------------------------------------------
void LPMPC::buildCondensedMatrices()
{
    const int n  = plant_.stateSize();
    const int Np = p_.Np;
    const int Nc = p_.Nc;

    // Powers of A: Apow[k] = A^k
    std::vector<Eigen::MatrixXd> Apow(static_cast<size_t>(Np) + 1);
    Apow[0] = Eigen::MatrixXd::Identity(n, n);
    for (int k = 1; k <= Np; ++k)
        Apow[static_cast<size_t>(k)] = plant_.A * Apow[static_cast<size_t>(k - 1)];

    F_.resize(Np, n);
    for (int i = 0; i < Np; ++i)
        F_.row(i) = plant_.C * Apow[static_cast<size_t>(i + 1)];

    Phi_.resize(Np, Nc);
    Phi_.setZero();
    for (int i = 0; i < Np; ++i)
        for (int j = 0; j <= std::min(i, Nc - 1); ++j)
            Phi_(i, j) = (plant_.C * Apow[static_cast<size_t>(i - j)] * plant_.B)(0, 0);

    Gu_.resize(Np);
    Gu_(0) = (plant_.C * plant_.B)(0, 0);
    for (int i = 1; i < Np; ++i)
        Gu_(i) = Gu_(i - 1) + (plant_.C * Apow[static_cast<size_t>(i)] * plant_.B)(0, 0);

    // LP variable layout: [DeltaU (Nc); t_y (Np); t_u (Nc)].
    const int colDU = 0, colTy = Nc, colTu = Nc + Np;
    const int n_lp  = 2 * Nc + Np;
    const int m_lp  = 2 * Np + 2 * Nc;

    A_ineq_ = Eigen::MatrixXd::Zero(m_lp, n_lp);
    A_ineq_.block(0, colDU, Np, Nc)      = Phi_;
    A_ineq_.block(0, colTy, Np, Np)      = -Eigen::MatrixXd::Identity(Np, Np);
    A_ineq_.block(Np, colDU, Np, Nc)     = -Phi_;
    A_ineq_.block(Np, colTy, Np, Np)     = -Eigen::MatrixXd::Identity(Np, Np);
    A_ineq_.block(2 * Np, colDU, Nc, Nc)          = Eigen::MatrixXd::Identity(Nc, Nc);
    A_ineq_.block(2 * Np, colTu, Nc, Nc)          = -Eigen::MatrixXd::Identity(Nc, Nc);
    A_ineq_.block(2 * Np + Nc, colDU, Nc, Nc)     = -Eigen::MatrixXd::Identity(Nc, Nc);
    A_ineq_.block(2 * Np + Nc, colTu, Nc, Nc)     = -Eigen::MatrixXd::Identity(Nc, Nc);

    c_ = Eigen::VectorXd::Zero(n_lp);
    c_.segment(colTy, Np) = Eigen::VectorXd::Constant(Np, p_.rho_y);
    c_.segment(colTu, Nc) = Eigen::VectorXd::Constant(Nc, p_.rho_u);

    b_ineq_ = Eigen::VectorXd::Zero(m_lp); // tail (groups 3/4) stays 0; head refreshed per step

    lb_ = Eigen::VectorXd::Zero(n_lp);
    ub_ = Eigen::VectorXd::Zero(n_lp);
    lb_.segment(colTy, Np) = Eigen::VectorXd::Zero(Np);
    ub_.segment(colTy, Np) = Eigen::VectorXd::Constant(Np, 1e9);
    lb_.segment(colTu, Nc) = Eigen::VectorXd::Zero(Nc);
    ub_.segment(colTu, Nc) = Eigen::VectorXd::Constant(Nc, 1e9);
    // lb_/ub_.head(Nc) (DeltaU) is overwritten every computeRef() call.
}

double LPMPC::compute(double error)
{
    if (!std::isfinite(error)) return u_prev_;
    const double y_hat = (plant_.C * x_hat_)(0, 0) + plant_.D(0, 0) * u_prev_;
    const double r_ref = y_hat + error;
    return computeRef(x_hat_, r_ref);
}

double LPMPC::computeRef(const Eigen::VectorXd &x, double r_ref)
{
    const int Np = p_.Np;
    const int Nc = p_.Nc;

    const Eigen::VectorXd rhs1 = Eigen::VectorXd::Constant(Np, r_ref) - F_ * x - Gu_ * u_prev_;
    b_ineq_.head(Np)        = rhs1;
    b_ineq_.segment(Np, Np) = -rhs1;

    // Rolling worst-case tightened box bounds on DeltaU - scalar (m=1) port of
    // DiscreteMPC::computeRef's cumMin_/cumMax_ loop; QP/LP-agnostic.
    double cumMin = 0.0, cumMax = 0.0;
    for (int j = 0; j < Nc; ++j)
    {
        const double lo = std::max(p_.duMin, p_.uMin - u_prev_ - cumMax);
        const double hi = std::min(p_.duMax, p_.uMax - u_prev_ - cumMin);
        lb_(j) = lo;
        ub_(j) = (hi >= lo) ? hi : lo; // collapse infeasible interval to lo
        cumMin += lo;
        cumMax += ub_(j);
    }

    LPProblem problem;
    problem.c       = c_;
    problem.A_ineq  = A_ineq_;
    problem.b_ineq  = b_ineq_;
    problem.lb      = lb_;
    problem.ub      = ub_;
    // A_eq/b_eq left default-constructed (0 rows): LPMPC's cumulative-u bound is folded into
    // DeltaU's box above, exactly as DiscreteMPC does for its QP - no equality rows needed.

    const LPResult result = LPSolver::solve(problem, p_.lpMaxIter, p_.lpTol);
    last_lp_converged_ = (result.status == LPStatus::Optimal);
    last_lp_iters_      = result.iters;

    notify_buf_(0) = static_cast<double>(last_lp_iters_);
    notifyObserverState("lp_iters", notify_buf_);
    if (!last_lp_converged_)
    {
        notify_buf_(0) = 0.0;
        notifyObserverState("health", notify_buf_);
    }

#ifndef NDEBUG
    if (!last_lp_converged_)
        std::clog << "[LPMPC] WARNING: LP solver did not reach Optimal (status="
                  << static_cast<int>(result.status)
                  << "). Use lastLPConverged() to check per step.\n";
#endif

    // Degenerate LP (non-Optimal): hold u_prev_ rather than trust a meaningless DeltaU.
    const double du = last_lp_converged_ ? result.x(0) : 0.0;
    const double u  = std::clamp(u_prev_ + du, p_.uMin, p_.uMax);

    x_hat_  = plant_.A * x + plant_.B * u;
    u_prev_ = u;

    return u;
}

void LPMPC::setParams(const LPMPCParams &p)
{
    if (p.Nc < 1 || p.Np < p.Nc)
        throw std::invalid_argument("LPMPC: require Nc >= 1 and Np >= Nc");
    p_ = p;
    buildCondensedMatrices();
}

void LPMPC::setPlant(const StateSpace &plant)
{
    if (plant.inputSize() != 1 || plant.outputSize() != 1)
        throw std::invalid_argument("LPMPC: plant must be SISO (1 input, 1 output)");
    plant_ = plant;
    Ts_    = plant.Ts;
    buildCondensedMatrices();
}

void LPMPC::reset()
{
    x_hat_.setZero();
    u_prev_ = 0.0;
}

} // namespace ctrl
