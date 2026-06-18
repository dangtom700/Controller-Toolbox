#include "DeePC.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace ctrl {

DeePC::DeePC(const DeePCParams& params, double Ts)
    : p_(params), Ts_(Ts)
{
    if (p_.T_ini < 1)
        throw std::invalid_argument("[DeePC] T_ini must be >= 1.");
    if (p_.N < 1)
        throw std::invalid_argument("[DeePC] N must be >= 1.");
    if (p_.uMin >= p_.uMax)
        throw std::invalid_argument("[DeePC] uMin must be < uMax.");
    if (Ts_ <= 0.0)
        throw std::invalid_argument("[DeePC] Ts must be positive.");
    r_stacked_ = Eigen::VectorXd::Constant(p_.N, r_);
}

// ---------------------------------------------------------------------------
// Data loading and offline pre-computation
// ---------------------------------------------------------------------------

void DeePC::collectData(const Eigen::VectorXd& u_data, const Eigen::VectorXd& y_data) {
    const int N_data = static_cast<int>(u_data.size());
    if (static_cast<int>(y_data.size()) != N_data)
        throw std::invalid_argument("[DeePC] u_data and y_data must have equal length.");
    const int L = p_.T_ini + p_.N;
    if (N_data < L + 1)
        throw std::invalid_argument(
            "[DeePC] Data too short: need at least T_ini + N + 1 = " +
            std::to_string(L + 1) + " samples.");

    buildHankel(u_data, y_data);
    buildQPMatrices();

    // Reset runtime state
    u_buf_.assign(p_.T_ini, 0.0);
    y_buf_.assign(p_.T_ini, 0.0);
    g_.setZero(M_);
    uf_.setZero(p_.N);
    dual_.setZero(p_.N);
    u_last_        = 0.0;
    healthy_       = true;
    data_collected_ = true;
}

void DeePC::buildHankel(const Eigen::VectorXd& u_data, const Eigen::VectorXd& y_data) {
    const int T      = p_.T_ini;
    const int N      = p_.N;
    const int N_data = static_cast<int>(u_data.size());
    M_ = N_data - T - N + 1;

    H_up_.resize(T, M_);
    H_yp_.resize(T, M_);
    H_uf_.resize(N, M_);
    H_yf_.resize(N, M_);

    for (int j = 0; j < M_; ++j) {
        for (int i = 0; i < T; ++i) {
            H_up_(i, j) = u_data(i + j);
            H_yp_(i, j) = y_data(i + j);
        }
        for (int i = 0; i < N; ++i) {
            H_uf_(i, j) = u_data(T + i + j);
            H_yf_(i, j) = y_data(T + i + j);
        }
    }

    H_up_T_ = H_up_.transpose();
    H_yp_T_ = H_yp_.transpose();
    H_uf_T_ = H_uf_.transpose();
    H_yf_T_ = H_yf_.transpose();
}

void DeePC::buildQPMatrices() {
    const double Q   = p_.Q;
    const double ly  = p_.lambda_y;
    const double lu  = p_.lambda_u;
    const double lg  = p_.lambda_g;
    const double rho = p_.rho;

    // A_g = 2*(Q*H_yf'*H_yf + ly*H_yp'*H_yp + lu*H_up'*H_up + lg*I) + rho*H_uf'*H_uf
    // Note: R is not in A_g because R penalises u_f (handled in the u_f ADMM step).
    A_g_.resize(M_, M_);
    A_g_.noalias()  = (2.0 * Q)  * (H_yf_T_ * H_yf_);
    A_g_.noalias() += (2.0 * ly) * (H_yp_T_ * H_yp_);
    A_g_.noalias() += (2.0 * lu) * (H_up_T_ * H_up_);
    A_g_.noalias() += rho        * (H_uf_T_ * H_uf_);
    for (int i = 0; i < M_; ++i)
        A_g_(i, i) += 2.0 * lg;

    // Enforce exact symmetry to avoid LDLT numerical issues
    A_g_ = 0.5 * (A_g_ + A_g_.transpose()).eval();

    A_g_ldlt_.compute(A_g_);

    // Pre-allocate workspace vectors
    b_g_.setZero(M_);
    b_g_ini_.setZero(M_);
    b_g_ref_.setZero(M_);
    H_uf_g_.setZero(p_.N);
    r_stacked_ = Eigen::VectorXd::Constant(p_.N, r_);
    b_g_ref_.noalias() = (2.0 * Q) * (H_yf_T_ * r_stacked_);
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------

void DeePC::setReference(double r) {
    r_ = r;
    r_stacked_.setConstant(p_.N, r_);
    if (data_collected_) {
        b_g_ref_.noalias() = (2.0 * p_.Q) * (H_yf_T_ * r_stacked_);
    }
}

double DeePC::compute(double y_meas) {
    if (!std::isfinite(y_meas) || !data_collected_) {
        notifyObserver(u_last_, y_meas);
        return u_last_;
    }

    // Append current measurement and trim buffer to T_ini length
    y_buf_.push_back(y_meas);
    if (static_cast<int>(y_buf_.size()) > p_.T_ini)
        y_buf_.pop_front();

    const double u = std::clamp(solveADMM(), p_.uMin, p_.uMax);
    u_last_ = u;

    // Append applied input to past buffer (recorded AFTER solving so the
    // current y_meas is matched to the previous u, consistent with ZOH)
    u_buf_.push_back(u);
    if (static_cast<int>(u_buf_.size()) > p_.T_ini)
        u_buf_.pop_front();

    notifyObserver(u, y_meas);
    return u;
}

double DeePC::solveADMM() {
    // Build y_ini and u_ini from deque buffers
    Eigen::VectorXd y_ini(p_.T_ini), u_ini(p_.T_ini);
    {
        int idx = 0;
        for (double v : y_buf_) y_ini(idx++) = v;
    }
    {
        int idx = 0;
        for (double v : u_buf_) u_ini(idx++) = v;
    }

    // Past-trajectory contribution (changes each step, precomputed here)
    b_g_ini_.noalias() = (2.0 * p_.lambda_y) * (H_yp_T_ * y_ini)
                       + (2.0 * p_.lambda_u) * (H_up_T_ * u_ini);

    // Cold-start u_f and dual (g_ is warm-started from previous step)
    uf_.setZero();
    dual_.setZero();

    const double denom_uf = 2.0 * p_.R + p_.rho;
    const double rho      = p_.rho;

    // Boyd et al. (2011) Section 3.3.1 stopping criterion: absolute + relative tolerances.
    // Scale factors avoid dependence on problem units or matrix norms.
    const double eps_abs = p_.admm_tol;
    const double eps_rel = 1e-3;
    const double sq_N    = std::sqrt(static_cast<double>(p_.N));
    const double sq_M    = std::sqrt(static_cast<double>(M_));

    bool converged = false;
    for (int iter = 0; iter < p_.admm_iters; ++iter) {
        // ---------------------------------------------------------------
        // g-update: A_g * g = b_g_ref + b_g_ini + H_uf' * (rho*u_f - nu)
        // ---------------------------------------------------------------
        b_g_.noalias() = b_g_ref_ + b_g_ini_ + H_uf_T_ * (rho * uf_ - dual_);
        const Eigen::VectorXd g_new = A_g_ldlt_.solve(b_g_);

        // ---------------------------------------------------------------
        // u_f-update: box-project (rho * H_uf * g + nu) / (2R + rho)
        // ---------------------------------------------------------------
        H_uf_g_.noalias() = H_uf_ * g_new;
        const Eigen::VectorXd uf_new = ((rho * H_uf_g_ + dual_) / denom_uf)
                                           .cwiseMax(p_.uMin)
                                           .cwiseMin(p_.uMax);

        // ---------------------------------------------------------------
        // Dual update: nu += rho * (H_uf * g - u_f)
        // ---------------------------------------------------------------
        const Eigen::VectorXd prim_res = H_uf_g_ - uf_new;
        const Eigen::VectorXd dual_res = rho * (H_uf_T_ * (uf_new - uf_));

        dual_.noalias() += rho * prim_res;
        g_  = g_new;
        uf_ = uf_new;

        // Scaled stopping: eps_prim = eps_abs*sqrt(N) + eps_rel*max(||H_uf g||, ||u_f||)
        //                  eps_dual = eps_abs*sqrt(M) + eps_rel*||H_uf^T nu||
        const double eps_prim = eps_abs * sq_N
                              + eps_rel * std::max(H_uf_g_.norm(), uf_.norm());
        const double eps_dual = eps_abs * sq_M
                              + eps_rel * (H_uf_T_ * dual_).norm();
        if (prim_res.norm() < eps_prim && dual_res.norm() < eps_dual) {
            converged = true;
            break;
        }
    }
    healthy_ = converged;
    return uf_(0);
}

void DeePC::reset() {
    u_buf_.assign(p_.T_ini, 0.0);
    y_buf_.assign(p_.T_ini, 0.0);
    g_.setZero();
    uf_.setZero();
    dual_.setZero();
    u_last_  = 0.0;
    healthy_ = true;
    notifyObserverReset();
}

} // namespace ctrl
