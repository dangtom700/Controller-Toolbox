#include "NonlinearMPC.h"
#include <algorithm>
#include <cmath>

namespace ctrl
{

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static Eigen::MatrixXd jacX_discrete(
    const NonlinearMPC::DiscreteDynamics &f,
    const Eigen::VectorXd &x,
    const Eigen::VectorXd &u,
    double eps = 1e-5)
{
    // Wrap the discrete dynamics into the StateFunc signature (same as LinearisationHelper)
    // and apply central-difference Jacobian w.r.t. x.
    const int n = static_cast<int>(x.size());
    const int out_dim = static_cast<int>(f(x, u).size());
    Eigen::MatrixXd J(out_dim, n);

    for (int i = 0; i < n; ++i)
    {
        const double h = eps * std::max(std::abs(x(i)), 1.0);
        Eigen::VectorXd xp = x, xm = x;
        xp(i) += h;
        xm(i) -= h;
        J.col(i) = (f(xp, u) - f(xm, u)) / (2.0 * h);
    }
    return J;
}

static Eigen::MatrixXd jacU_discrete(
    const NonlinearMPC::DiscreteDynamics &f,
    const Eigen::VectorXd &x,
    const Eigen::VectorXd &u,
    double eps = 1e-5)
{
    const int m = static_cast<int>(u.size());
    const int out_dim = static_cast<int>(f(x, u).size());
    Eigen::MatrixXd J(out_dim, m);

    for (int i = 0; i < m; ++i)
    {
        const double h = eps * std::max(std::abs(u(i)), 1.0);
        Eigen::VectorXd up = u, um = u;
        up(i) += h;
        um(i) -= h;
        J.col(i) = (f(x, up) - f(x, um)) / (2.0 * h);
    }
    return J;
}

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------

void NonlinearMPC::init()
{
    const int n  = p_.n_states;
    const int m  = p_.n_inputs;
    const int p  = p_.n_outputs;
    const int Np = p_.Np;
    const int Nu = p_.Nu;
    const int Nu_total = m * Nu;

    x_current_ = Eigen::VectorXd::Zero(n);
    y_ref_     = Eigen::VectorXd::Zero(p);
    U_warm_    = Eigen::MatrixXd::Zero(m, Nu);

    Theta_.resize(p * Np, Nu_total);
    xi_.resize(p * Np);

    H_qp_.resize(Nu_total, Nu_total);
    g_qp_.resize(Nu_total);
    lb_qp_.resize(Nu_total);
    ub_qp_.resize(Nu_total);
    dU_sol_.resize(Nu_total);
    tmp1_.resize(Nu_total);
    tmp2_.resize(Nu_total);
    u_opt_.resize(m);
}

NonlinearMPC::NonlinearMPC(const NMPCParams &params, DiscreteDynamics f_d)
    : p_(params), f_(std::move(f_d))
{
    const int n = p_.n_states;
    const int p = p_.n_outputs;
    C_ = Eigen::MatrixXd::Identity(p, n);
    init();
}

NonlinearMPC::NonlinearMPC(const NMPCParams &params, DiscreteDynamics f_d,
                           const Eigen::MatrixXd &C_out)
    : p_(params), f_(std::move(f_d)), C_(C_out)
{
    init();
}

// -----------------------------------------------------------------------------
// buildAndSolve - RTI step
// -----------------------------------------------------------------------------

void NonlinearMPC::buildAndSolve()
{
    const int n  = p_.n_states;
    const int m  = p_.n_inputs;
    const int p  = C_.rows();
    const int Np = p_.Np;
    const int Nu = p_.Nu;

    // Step 1 - simulate nominal trajectory from x_current_ under U_warm_
    std::vector<Eigen::VectorXd> x_traj(Np + 1);
    x_traj[0] = x_current_;
    for (int j = 0; j < Np; ++j)
    {
        const int k = std::min(j, Nu - 1);
        x_traj[j + 1] = f_(x_traj[j], U_warm_.col(k));
    }

    // Step 2 - linearise at each step: A_j = df/dx, B_j = df/du
    std::vector<Eigen::MatrixXd> A_list(Np), B_list(Np);
    for (int j = 0; j < Np; ++j)
    {
        const int k = std::min(j, Nu - 1);
        A_list[j] = jacX_discrete(f_, x_traj[j], U_warm_.col(k));
        B_list[j] = jacU_discrete(f_, x_traj[j], U_warm_.col(k));
    }

    // Step 3 - build xi: nominal output trajectory minus reference (stacked Np times)
    for (int j = 0; j < Np; ++j)
        xi_.segment(j * p, p) = C_ * x_traj[j + 1] - y_ref_;

    // Step 4 - build Theta (p*Np x m*Nu): time-varying step-response matrix.
    //
    // Row block j (output Deltay_{j+1}), col block k (from Deltau_k):
    //   Theta[j, k] = C * A_j * A_{j-1} * ... * A_{k+1} * B_k   for k <= j
    //
    // For k >= Nu the input is held at Deltau_{Nu-1}, so contributions are
    // accumulated into column block Nu-1.
    //
    // Incremental product: for fixed j iterate k from j down to 0.
    //   Phi starts at I, then after each k: Phi = Phi * A_k.
    //   This gives Phi[k-1] = A_j * ... * A_k correctly.
    Theta_.setZero();
    for (int j = 0; j < Np; ++j)
    {
        Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(n, n);
        for (int k = j; k >= 0; --k)
        {
            const Eigen::MatrixXd contrib = C_ * Phi * B_list[k];
            const int col_k = std::min(k, Nu - 1);
            Theta_.block(j * p, col_k * m, p, m) += contrib;
            // Update Phi for k-1: Phi_new = Phi_old * A_k
            Phi = Phi * A_list[k];
        }
    }

    // Step 5 - build QP in terms of DeltaU (deviation from warm start)
    //   H = rho_y * Theta^T Theta + rho_u * I
    //   g = rho_y * Theta^T * xi
    const int Nu_total = m * Nu;
    H_qp_.noalias() = p_.rho_y * (Theta_.transpose() * Theta_);
    H_qp_ += p_.rho_u * Eigen::MatrixXd::Identity(Nu_total, Nu_total);
    g_qp_.noalias() = p_.rho_y * (Theta_.transpose() * xi_);

    // Box constraints on DeltaU: uMin - u_bar_k <= DeltaU_k <= uMax - u_bar_k
    for (int k = 0; k < Nu; ++k)
    {
        lb_qp_.segment(k * m, m).setConstant(p_.uMin);
        ub_qp_.segment(k * m, m).setConstant(p_.uMax);
        lb_qp_.segment(k * m, m) -= U_warm_.col(k);
        ub_qp_.segment(k * m, m) -= U_warm_.col(k);
    }

    // Lipschitz constant and LDLT (must be recomputed each RTI step)
    L_qp_ = H_qp_.selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff();
    if (L_qp_ < 1e-12) L_qp_ = 1.0; // guard against degenerate H
    ldlt_  = H_qp_.ldlt();

    // Step 6 - solve QP via FISTA for DeltaU
    auto res = solveGradientProjectionQP(H_qp_, g_qp_, lb_qp_, ub_qp_,
                                         ldlt_, L_qp_,
                                         p_.qpMaxIter, p_.qpTol,
                                         dU_sol_, tmp1_, tmp2_);
    last_qp_converged_ = res.converged;
    last_qp_iters_     = res.iters;

    // Step 7 - reconstruct U* = U_warm + DeltaU; extract first control
    Eigen::MatrixXd U_opt(m, Nu);
    for (int k = 0; k < Nu; ++k)
        U_opt.col(k) = U_warm_.col(k) + dU_sol_.segment(k * m, m);

    u_opt_      = U_opt.col(0);
    last_output_ = u_opt_(0);

    // Shift warm start: [u*_1, u*_2, ..., u*_{Nu-1}, u*_{Nu-1}]
    for (int k = 0; k < Nu - 1; ++k)
        U_warm_.col(k) = U_opt.col(k + 1);
    U_warm_.col(Nu - 1) = U_opt.col(Nu - 1);
}

// -----------------------------------------------------------------------------
// Public interface
// -----------------------------------------------------------------------------

Eigen::VectorXd NonlinearMPC::computeRef(const Eigen::VectorXd &x_current,
                                          const Eigen::VectorXd &y_ref)
{
    x_current_ = x_current;
    y_ref_     = y_ref;
    buildAndSolve();
    notifyObserver(last_output_, (C_ * x_current_ - y_ref_)(0));
    return u_opt_;
}

double NonlinearMPC::compute(double error)
{
    if (!std::isfinite(error)) return last_output_;
    // Reconstruct scalar output reference from error: y_ref = y + error
    const Eigen::VectorXd y_current = C_ * x_current_;
    y_ref_ = y_current.array() + error;
    buildAndSolve();
    notifyObserver(last_output_, error);
    return last_output_;
}

void NonlinearMPC::reset()
{
    x_current_.setZero();
    y_ref_.setZero();
    U_warm_.setZero();
    u_opt_.setZero();
    last_output_        = 0.0;
    last_qp_converged_  = true;
    last_qp_iters_      = 0;
    notifyObserverReset();
}

} // namespace ctrl
