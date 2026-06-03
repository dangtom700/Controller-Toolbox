#include "DeePC.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

// ---------------------------------------------------------------------------
// Static helper: build an L x (N-L+1) Hankel matrix
// ---------------------------------------------------------------------------

Eigen::MatrixXd DeePC::buildHankel(const Eigen::VectorXd& data, int L)
{
    const int N     = static_cast<int>(data.size());
    const int n_col = N - L + 1;
    Eigen::MatrixXd H(L, n_col);
    for (int j = 0; j < n_col; ++j)
        H.col(j) = data.segment(j, L);
    return H;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

DeePC::DeePC(const Eigen::VectorXd& u_data,
             const Eigen::VectorXd& y_data,
             const Params&          params,
             double                 Ts)
    : p_(params), Ts_(Ts)
{
    if (p_.T_ini <= 0 || p_.Np <= 0)
        throw std::invalid_argument("DeePC: T_ini and Np must be positive");
    if (p_.admm_iter <= 0)
        throw std::invalid_argument("DeePC: admm_iter must be positive");

    N_ = static_cast<int>(u_data.size());
    if (N_ != static_cast<int>(y_data.size()))
        throw std::invalid_argument("DeePC: u_data and y_data must have the same length");

    L_     = p_.T_ini + p_.Np;
    n_col_ = N_ - L_ + 1;

    if (n_col_ < 1)
        throw std::invalid_argument(
            "DeePC: offline data too short; need N >= T_ini + Np + 1 "
            "(got N=" + std::to_string(N_) +
            ", T_ini+Np=" + std::to_string(L_) + ")");

    // Build full Hankel then split into past/future blocks
    const Eigen::MatrixXd H_u = buildHankel(u_data, L_);  // L x n_col
    const Eigen::MatrixXd H_y = buildHankel(y_data, L_);

    H_up_ = H_u.topRows(p_.T_ini);
    H_uf_ = H_u.bottomRows(p_.Np);
    H_yp_ = H_y.topRows(p_.T_ini);
    H_yf_ = H_y.bottomRows(p_.Np);

    // Pre-allocate ADMM variables (zero warm-start)
    g_.setZero(n_col_);
    u_opt_.setZero(p_.Np);
    lam_.setZero(p_.Np);

    // Rolling buffers
    u_buf_.setZero(p_.T_ini);
    y_buf_.setZero(p_.T_ini);

    factoriseHessian();
}

// ---------------------------------------------------------------------------
// Hessian factorisation (called once in constructor)
//
// M = rho_y . H_yf'.H_yf
//   + lambda_g . I
//   + lambda_eq. H_up'.H_up
//   + rho   . H_uf'.H_uf
//
// M is constant because all four Hankel matrices are fixed offline data.
// Subsequent per-step solves become O(n_col^2) triangular back-substitutions.
// ---------------------------------------------------------------------------

void DeePC::factoriseHessian()
{
    Eigen::MatrixXd M =
          p_.rho_y    * (H_yf_.transpose() * H_yf_)
        + p_.lambda_g * Eigen::MatrixXd::Identity(n_col_, n_col_)
        + p_.lambda_eq * (H_up_.transpose() * H_up_)
        + p_.rho_admm * (H_uf_.transpose() * H_uf_);

    M_ldlt_.compute(M);
    if (M_ldlt_.info() != Eigen::Success)
        throw std::runtime_error(
            "DeePC: Hessian factorisation failed - matrix is not positive-definite. "
            "Check that the offline data is persistently exciting (increase lambda_g if needed).");
}

// ---------------------------------------------------------------------------
// Core control computation
// ---------------------------------------------------------------------------

double DeePC::computeIO(double y_meas, double r)
{
    // Shift I/O buffer left (drop oldest) and push (u_prev_, y_meas) at the end
    u_buf_.head(p_.T_ini - 1) = u_buf_.tail(p_.T_ini - 1).eval();
    y_buf_.head(p_.T_ini - 1) = y_buf_.tail(p_.T_ini - 1).eval();
    u_buf_(p_.T_ini - 1) = u_prev_;
    y_buf_(p_.T_ini - 1) = y_meas;

    ++step_cnt_;

    // Return zero during warm-up (buffer not yet representative)
    if (step_cnt_ <= p_.T_ini) {
        notifyObserver(0.0, y_meas);
        return 0.0;
    }

    // Constant reference trajectory over the horizon
    const Eigen::VectorXd y_ref = Eigen::VectorXd::Constant(p_.Np, r);

    // Pre-compute constant parts of the g-update RHS that don't change within the loop
    const Eigen::VectorXd rhs_fixed =
          p_.rho_y    * (H_yf_.transpose() * y_ref)
        + p_.lambda_eq * (H_up_.transpose() * u_buf_);

    // Scalar used in u-update (element-wise for diagonal R = rho_u.I)
    const double inv_Ru_rho = 1.0 / (p_.rho_u + p_.rho_admm);

    // ADMM iterations
    // -----------------------------------------------------------------------
    // g-update:  M . g = rhs_fixed + H_uf' . (rho.u - lambda)
    // u-update:  u_free = (rho.H_uf.g + lambda) / (rho_u + rho);  u = clip(u_free, uMin, uMax)
    // lambda-update:  lambda += rho . (H_uf.g - u)
    // -----------------------------------------------------------------------
    for (int k = 0; k < p_.admm_iter; ++k) {
        // g-update
        const Eigen::VectorXd rhs =
            rhs_fixed + H_uf_.transpose() * (p_.rho_admm * u_opt_ - lam_);
        g_ = M_ldlt_.solve(rhs);

        // u-update: unconstrained optimum then project
        const Eigen::VectorXd huf_g = H_uf_ * g_;
        const Eigen::VectorXd u_free = inv_Ru_rho * (p_.rho_admm * huf_g + lam_);
        u_opt_ = u_free.cwiseMin(p_.uMax).cwiseMax(p_.uMin);

        // dual update
        lam_ += p_.rho_admm * (huf_g - u_opt_);
    }

    primal_res_ = (H_uf_ * g_ - u_opt_).norm();

    const double u_apply = std::clamp(u_opt_(0), p_.uMin, p_.uMax);
    u_prev_ = u_apply;
    notifyObserver(u_apply, y_meas);
    return u_apply;
}

// ---------------------------------------------------------------------------
// IController interface
// ---------------------------------------------------------------------------

double DeePC::compute(double error)
{
    // error = r - y  =>  y = r - error
    return computeIO(r_ - error, r_);
}

void DeePC::reset()
{
    g_.setZero();
    u_opt_.setZero();
    lam_.setZero();
    u_buf_.setZero();
    y_buf_.setZero();
    u_prev_     = 0.0;
    step_cnt_   = 0;
    primal_res_ = 0.0;
}

} // namespace ctrl
