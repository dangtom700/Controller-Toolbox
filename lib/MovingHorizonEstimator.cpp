#include "MovingHorizonEstimator.h"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace ctrl {

// =============================================================================
// Constructor
// =============================================================================

MovingHorizonEstimator::MovingHorizonEstimator(const StateSpace      &plant,
                                               const Eigen::MatrixXd &Q_proc,
                                               const Eigen::MatrixXd &R_meas,
                                               const MHEParams       &params)
    : plant_(plant)
    , params_(params)
    , Q_proc_(Q_proc)
    , R_meas_(R_meas)
    , Ts_(plant.Ts)
    , n_(plant.stateSize())
    , m_(plant.inputSize())
    , p_(plant.outputSize())
    , step_count_(0)
{
    if (Q_proc.rows() != n_ || Q_proc.cols() != n_)
        throw std::invalid_argument("MHE: Q_proc must be n x n.");
    if (R_meas.rows() != p_ || R_meas.cols() != p_)
        throw std::invalid_argument("MHE: R_meas must be p x p.");
    if (params_.N < 1)
        throw std::invalid_argument("MHE: horizon N must be >= 1.");

    P0inv_ = Eigen::MatrixXd::Identity(n_, n_);
    x_bar_ = Eigen::VectorXd::Zero(n_);
    x_est_ = Eigen::VectorXd::Zero(n_);

    buildCondensedMatrices();

    // Pre-allocate history buffers
    const int Np1 = params_.N + 1;
    y_hist_.resize(Np1, Eigen::VectorXd::Zero(p_));
    u_hist_.resize(params_.N, Eigen::VectorXd::Zero(m_));
}

// =============================================================================
// initialize
// =============================================================================

void MovingHorizonEstimator::initialize(const Eigen::VectorXd &x0,
                                        const Eigen::MatrixXd &P0)
{
    if (x0.size() != n_)
        throw std::invalid_argument("MHE::initialize: x0 must have size n.");
    if (P0.rows() != n_ || P0.cols() != n_)
        throw std::invalid_argument("MHE::initialize: P0 must be n x n.");

    // P0inv via LDLT
    Eigen::LDLT<Eigen::MatrixXd> ldlt_p0(P0);
    P0inv_ = ldlt_p0.solve(Eigen::MatrixXd::Identity(n_, n_));

    x_bar_  = x0;
    x_est_  = x0;
    step_count_ = 0;

    for (auto &v : y_hist_) v.setZero();
    for (auto &v : u_hist_) v.setZero();

    // Rebuild H because P0inv changes the arrival-cost block
    buildCondensedMatrices();
}

// =============================================================================
// estimate
// =============================================================================

Eigen::VectorXd MovingHorizonEstimator::estimate(const Eigen::VectorXd &y,
                                                 const Eigen::VectorXd &u)
{
    if (y.size() != p_)
        throw std::invalid_argument("MHE::estimate: y must have size p.");
    if (u.size() != m_)
        throw std::invalid_argument("MHE::estimate: u must have size m.");

    const int N   = params_.N;

    // Shift history: oldest entry falls off, new y and u go at the back.
    // y_hist_ has N+1 slots: indices [0..N], where 0 is the oldest.
    // u_hist_ has N slots: u_hist_[i] drives the transition x[i] -> x[i+1].
    for (int i = 0; i < N; ++i) y_hist_[i] = y_hist_[i + 1];
    y_hist_[N] = y;
    for (int i = 0; i < N - 1; ++i) u_hist_[i] = u_hist_[i + 1];
    if (N >= 1) u_hist_[N - 1] = u;

    ++step_count_;

    // Effective horizon: ramp up from 1 to N during the first N steps
    const int eff_N = std::min(step_count_, N);
    const int eff_Np1 = eff_N + 1;
    const int dz = n_ * (eff_N + 1); // decision variable size

    // --- Build QP linear cost g = H_mhe*prior_z - Psi'*C_bar'*Rinv*y_hist ---
    // Decision variable z = [x_bar_; 0; ...; 0] (prior on the process-noise window)

    // Stack measurements from window tail (most recent eff_N steps + 1 measurement)
    Eigen::VectorXd Y_hist(p_ * eff_Np1);
    for (int i = 0; i < eff_Np1; ++i) {
        // y_hist_[N - eff_N + i] maps to window position i
        const int idx = N - eff_N + i;
        Y_hist.segment(i * p_, p_) = y_hist_[idx];
    }

    // Prior z_prior = [x_bar_; 0; ...; 0]
    Eigen::VectorXd z_prior = Eigen::VectorXd::Zero(dz);
    z_prior.head(n_) = x_bar_;

    // Stack inputs for the effective window
    Eigen::VectorXd U_hist_eff((eff_N) * m_);
    for (int i = 0; i < eff_N; ++i) {
        const int idx = N - eff_N + i;
        U_hist_eff.segment(i * m_, m_) = u_hist_[idx];
    }

    // For the effective horizon, build Psi, Gamma_u, C_bar sub-matrices
    // Psi_eff: (n*eff_Np1) x (n*eff_Np1)
    Eigen::MatrixXd Psi_eff = Eigen::MatrixXd::Zero(n_ * eff_Np1, n_ * eff_Np1);
    Eigen::MatrixXd Gamma_u_eff = Eigen::MatrixXd::Zero(n_ * eff_Np1, m_ * eff_N);
    Eigen::MatrixXd C_bar_eff = Eigen::MatrixXd::Zero(p_ * eff_Np1, n_ * eff_Np1);

    // x_0 row
    Psi_eff.block(0, 0, n_, n_) = Eigen::MatrixXd::Identity(n_, n_);
    C_bar_eff.block(0, 0, p_, n_) = plant_.C;

    // Precompute powers of A
    std::vector<Eigen::MatrixXd> A_pow(eff_Np1);
    A_pow[0] = Eigen::MatrixXd::Identity(n_, n_);
    for (int i = 1; i < eff_Np1; ++i) A_pow[i] = plant_.A * A_pow[i - 1];

    for (int i = 1; i < eff_Np1; ++i) {
        // x_i: Psi block [i] col 0 = A^i (x_0 contribution)
        Psi_eff.block(i * n_, 0, n_, n_) = A_pow[i];
        // w_j contributes A^{i-1-j} to x_i for j = 0..i-1
        for (int j = 0; j < i; ++j) {
            // w_j is decision variable index j+1 (after x_0)
            Psi_eff.block(i * n_, (j + 1) * n_, n_, n_) = A_pow[i - 1 - j];
        }
        // u_j contributes A^{i-1-j}*B*u_j (known, goes into Gamma_u)
        for (int j = 0; j < i; ++j) {
            Gamma_u_eff.block(i * n_, j * m_, n_, m_) = A_pow[i - 1 - j] * plant_.B;
        }
        // C_bar block
        C_bar_eff.block(i * p_, i * n_, p_, n_) = plant_.C;
    }

    // Build Hessian for effective horizon
    // H_eff = Hprior + Psi'*C_bar'*Rinv*C_bar*Psi
    // Hprior = block_diag(P0inv, Qinv_block)  (only for effective size)
    Eigen::MatrixXd Qinv = Q_proc_.ldlt().solve(Eigen::MatrixXd::Identity(n_, n_));
    Eigen::MatrixXd Rinv = R_meas_.ldlt().solve(Eigen::MatrixXd::Identity(p_, p_));

    Eigen::MatrixXd H_prior = Eigen::MatrixXd::Zero(dz, dz);
    H_prior.block(0, 0, n_, n_) = P0inv_;
    for (int i = 1; i < eff_Np1; ++i)
        H_prior.block(i * n_, i * n_, n_, n_) = Qinv;

    // R_bar = block_diag(Rinv, ..., Rinv), size p*eff_Np1
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(p_ * eff_Np1, p_ * eff_Np1);
    for (int i = 0; i < eff_Np1; ++i)
        R_bar.block(i * p_, i * p_, p_, p_) = Rinv;

    Eigen::MatrixXd CTPsi = C_bar_eff.transpose() * R_bar * C_bar_eff;
    Eigen::MatrixXd H_eff = H_prior + Psi_eff.transpose() * CTPsi * Psi_eff;

    // Linear cost: f = -H_prior*z_prior - Psi'*C_bar'*Rinv*(Y - C_bar*Gamma_u*U)
    // (Y - C_bar*Gamma_u*U) is the residual from the known input contributions
    Eigen::VectorXd Y_pred_u = C_bar_eff * Gamma_u_eff * U_hist_eff;
    Eigen::VectorXd Y_res = Y_hist - Y_pred_u;

    Eigen::VectorXd f_eff = -H_prior * z_prior
                             - Psi_eff.transpose() * C_bar_eff.transpose() * R_bar * Y_res;

    // Box constraints on z: x_0 state bounds + per-element w noise bounds.
    // Decision vector layout: z = [x_0 (n); w_0 (n); w_1 (n); ...; w_{eff_N-1} (n)]
    Eigen::VectorXd lb_eff = Eigen::VectorXd::Constant(dz, -1e30);
    Eigen::VectorXd ub_eff = Eigen::VectorXd::Constant(dz,  1e30);

    // x_0 state bounds (MHEParams::xMin / xMax), if provided.
    if (params_.xMin.size() == n_) lb_eff.head(n_) = params_.xMin;
    if (params_.xMax.size() == n_) ub_eff.head(n_) = params_.xMax;

    // Process-noise bounds for w_1..w_{eff_N}.
    for (int i = 1; i < eff_Np1; ++i) {
        lb_eff.segment(i * n_, n_).fill(params_.wMin);
        ub_eff.segment(i * n_, n_).fill(params_.wMax);
    }

    // Solve QP
    Eigen::LDLT<Eigen::MatrixXd> ldlt_eff(H_eff);
    if (ldlt_eff.info() != Eigen::Success) {
        // Fallback: return prior estimate
        x_est_ = x_bar_;
        last_converged_ = false;
        last_qp_iters_  = 0;
        return x_est_;
    }

    const double L_eff = H_eff.eigenvalues().real().maxCoeff();
    if (L_eff <= 0.0) {
        x_est_ = x_bar_;
        last_converged_ = false;
        return x_est_;
    }

    Eigen::VectorXd z_sol_eff(dz), t1(dz), t2(dz);
    z_sol_eff.setZero();
    auto result = solveGradientProjectionQP(
        H_eff, f_eff, lb_eff, ub_eff,
        ldlt_eff, L_eff,
        params_.qpMaxIter, params_.qpTol,
        z_sol_eff, t1, t2);

    last_converged_ = result.converged;
    last_qp_iters_  = result.iters;

    // Extract x_0 from solution, propagate to get current x estimate
    const Eigen::VectorXd x0_opt = z_sol_eff.head(n_);

    // Propagate x_0_opt through the horizon using stored inputs and estimated w
    Eigen::VectorXd x_cur = x0_opt;
    for (int i = 0; i < eff_N; ++i) {
        const int idx = N - eff_N + i;
        const Eigen::VectorXd w_i = z_sol_eff.segment((i + 1) * n_, n_);
        x_cur = plant_.A * x_cur + plant_.B * u_hist_[idx] + w_i;
    }

    x_est_ = x_cur;

    // Update arrival state: x_bar_ for the next step is x_0 shifted by one
    // (the oldest state in the window advances by one step)
    if (step_count_ >= N) {
        // x_bar_ = A*x0_opt + B*u_hist_[N-eff_N] + w_0
        const Eigen::VectorXd w0 = z_sol_eff.segment(n_, n_);
        const int u0_idx = N - eff_N;
        x_bar_ = plant_.A * x0_opt + plant_.B * u_hist_[u0_idx] + w0;
    } else {
        x_bar_ = x0_opt;
    }

    return x_est_;
}

// =============================================================================
// setHorizon
// =============================================================================

void MovingHorizonEstimator::setHorizon(int N)
{
    if (N < 1)
        throw std::invalid_argument("MHE::setHorizon: N must be >= 1.");
    params_.N = N;
    y_hist_.assign(N + 1, Eigen::VectorXd::Zero(p_));
    u_hist_.assign(N,     Eigen::VectorXd::Zero(m_));
    step_count_ = 0;
    x_bar_  = x_est_;
    buildCondensedMatrices();
}

// =============================================================================
// setWeightMatrices
// =============================================================================

void MovingHorizonEstimator::setWeightMatrices(const Eigen::MatrixXd &Q_proc,
                                               const Eigen::MatrixXd &R_meas)
{
    Q_proc_ = Q_proc;
    R_meas_ = R_meas;
    buildCondensedMatrices();
}

// =============================================================================
// reset
// =============================================================================

void MovingHorizonEstimator::reset()
{
    step_count_ = 0;
    x_bar_  = Eigen::VectorXd::Zero(n_);
    x_est_  = Eigen::VectorXd::Zero(n_);
    for (auto &v : y_hist_) v.setZero();
    for (auto &v : u_hist_) v.setZero();
}

// =============================================================================
// buildCondensedMatrices
// Precomputes the full-horizon matrices for the QP.
// estimate() uses sub-matrices for the effective (ramped-up) horizon.
// =============================================================================

void MovingHorizonEstimator::buildCondensedMatrices()
{
    const int N   = params_.N;
    const int Np1 = N + 1;
    const int dz  = n_ * Np1;

    // Psi_: (n*Np1) x (n*Np1)
    // Row block i: x_i = sum_j Psi_[i,j]*z_j + Gamma_u terms
    Psi_    = Eigen::MatrixXd::Zero(n_ * Np1, n_ * Np1);
    Gamma_u_= Eigen::MatrixXd::Zero(n_ * Np1, m_ * N);
    C_bar_  = Eigen::MatrixXd::Zero(p_ * Np1, n_ * Np1);

    // x_0 row
    Psi_.block(0, 0, n_, n_) = Eigen::MatrixXd::Identity(n_, n_);
    C_bar_.block(0, 0, p_, n_) = plant_.C;

    std::vector<Eigen::MatrixXd> A_pow(Np1);
    A_pow[0] = Eigen::MatrixXd::Identity(n_, n_);
    for (int i = 1; i < Np1; ++i) A_pow[i] = plant_.A * A_pow[i - 1];

    for (int i = 1; i < Np1; ++i) {
        Psi_.block(i * n_, 0, n_, n_) = A_pow[i];
        for (int j = 0; j < i; ++j) {
            Psi_.block(i * n_, (j + 1) * n_, n_, n_) = A_pow[i - 1 - j];
            Gamma_u_.block(i * n_, j * m_, n_, m_)   = A_pow[i - 1 - j] * plant_.B;
        }
        C_bar_.block(i * p_, i * n_, p_, n_) = plant_.C;
    }

    // Inverses
    Eigen::MatrixXd Qinv = Q_proc_.ldlt().solve(Eigen::MatrixXd::Identity(n_, n_));
    Eigen::MatrixXd Rinv = R_meas_.ldlt().solve(Eigen::MatrixXd::Identity(p_, p_));

    // H_prior = block_diag(P0inv, Qinv, ..., Qinv) size dz x dz
    Eigen::MatrixXd H_prior = Eigen::MatrixXd::Zero(dz, dz);
    H_prior.block(0, 0, n_, n_) = P0inv_;
    for (int i = 1; i < Np1; ++i)
        H_prior.block(i * n_, i * n_, n_, n_) = Qinv;

    // R_bar = block_diag(Rinv, ...) size p*Np1
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(p_ * Np1, p_ * Np1);
    for (int i = 0; i < Np1; ++i)
        R_bar.block(i * p_, i * p_, p_, p_) = Rinv;

    H_mhe_ = H_prior + Psi_.transpose() * C_bar_.transpose() * R_bar * C_bar_ * Psi_;

    ldlt_.compute(H_mhe_);
    L_ = H_mhe_.eigenvalues().real().maxCoeff();
    if (L_ <= 0.0) L_ = 1.0;

    // Pre-allocate work vectors
    z_sol_.resize(dz); z_sol_.setZero();
    g_qp_ .resize(dz); g_qp_ .setZero();
    lb_z_ .resize(dz); lb_z_ .fill(-1e30);
    ub_z_ .resize(dz); ub_z_ .fill( 1e30);
    tmp1_  .resize(dz); tmp1_ .setZero();
    tmp2_  .resize(dz); tmp2_ .setZero();
}

} // namespace ctrl
