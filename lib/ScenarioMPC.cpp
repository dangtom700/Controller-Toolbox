#include "ScenarioMPC.h"
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace ctrl {

// ---------------------------------------------------------------------------
ScenarioMPC::ScenarioMPC(const StateSpace &sys, const ScenarioMPCParams &p)
    : sys_(sys), p_(p),
      n_(static_cast<int>(sys.A.rows())),
      m_(static_cast<int>(sys.B.cols())),
      pp_(static_cast<int>(sys.C.rows())),
      rng_(p.seed)
{
    // --- dimension checks ----------------------------------------------------
    if (p_.Nu > p_.Np)
        throw std::invalid_argument("ScenarioMPC: Nu must be <= Np.");
    if (p_.Q.rows() != pp_ || p_.Q.cols() != pp_)
        throw std::invalid_argument("ScenarioMPC: Q must be pp x pp.");
    if (p_.R.rows() != m_ || p_.R.cols() != m_)
        throw std::invalid_argument("ScenarioMPC: R must be m x m.");
    if (p_.Sigma_w.rows() != n_ || p_.Sigma_w.cols() != n_)
        throw std::invalid_argument("ScenarioMPC: Sigma_w must be n x n.");
    if (p_.uMin.size() != m_ || p_.uMax.size() != m_)
        throw std::invalid_argument("ScenarioMPC: uMin/uMax must have m elements.");
    if (p_.N_samples < 1)
        throw std::invalid_argument("ScenarioMPC: N_samples must be >= 1.");

    // --- Cholesky of Sigma_w for sampling w = L * z, z ~ N(0, I) -----------
    // For Sigma_w = 0 (deterministic limit) L = 0.
    Eigen::LLT<Eigen::MatrixXd> llt(p_.Sigma_w);
    if (llt.info() == Eigen::Success) {
        Sigma_w_chol_ = llt.matrixL();
    } else {
        // Fall back: treat as zero noise (pure deterministic MPC)
        Sigma_w_chol_ = Eigen::MatrixXd::Zero(n_, n_);
    }

    // --- build condensed matrices and QP ------------------------------------
    buildCondensedMatrices();

    // --- initialise online state --------------------------------------------
    x_nom_    = Eigen::VectorXd::Zero(n_);
    y_ref_    = Eigen::VectorXd::Zero(pp_);
    V_warm_   = Eigen::VectorXd::Zero(p_.Nu * m_);
    avg_W_    = Eigen::VectorXd::Zero(p_.Np * pp_);
    g_        = Eigen::VectorXd::Zero(p_.Nu * m_);
    tmp1_      = Eigen::VectorXd::Zero(p_.Nu * m_);
    tmp2_      = Eigen::VectorXd::Zero(p_.Nu * m_);
    y_fista_   = Eigen::VectorXd::Zero(p_.Nu * m_);
    x_noise_   = Eigen::VectorXd::Zero(n_);
    z_noise_   = Eigen::VectorXd::Zero(n_);
    R_stacked_ = Eigen::VectorXd::Zero(p_.Np * pp_);
}

// ---------------------------------------------------------------------------
void ScenarioMPC::buildCondensedMatrices()
{
    const int Np = p_.Np;
    const int Nu = p_.Nu;

    F_.resize(Np * pp_, n_);
    Phi_.resize(Np * pp_, Nu * m_);
    Phi_.setZero();

    // Precompute C * A^k * B for k = 0..Np-1
    Eigen::MatrixXd Ak(n_, n_);
    Ak.setIdentity();
    std::vector<Eigen::MatrixXd> CAkB(Np);
    for (int k = 0; k < Np; ++k) {
        CAkB[k] = sys_.C * Ak * sys_.B;
        Ak      = sys_.A * Ak;
    }

    // F: row block i = C * A^(i+1) * x
    Eigen::MatrixXd Aipow = sys_.A;
    for (int i = 0; i < Np; ++i) {
        F_.block(i * pp_, 0, pp_, n_) = sys_.C * Aipow;
        Aipow = sys_.A * Aipow;
    }

    // Phi: standard hold-last condensed input matrix
    for (int j = 0; j < Nu; ++j) {
        const bool is_last = (j == Nu - 1);
        for (int i = j; i < Np; ++i)
            Phi_.block(i * pp_, j * m_, pp_, m_) += CAkB[i - j];

        if (is_last) {
            for (int t = Nu; t < Np; ++t)
                for (int i = t; i < Np; ++i)
                    Phi_.block(i * pp_, j * m_, pp_, m_) += CAkB[i - t];
        }
    }

    // Build Q_blk and R_blk for cost
    Eigen::MatrixXd Qy = Eigen::MatrixXd::Zero(Np * pp_, Np * pp_);
    for (int k = 0; k < Np; ++k)
        Qy.block(k * pp_, k * pp_, pp_, pp_) = p_.Q;

    Eigen::MatrixXd Ru = Eigen::MatrixXd::Zero(Nu * m_, Nu * m_);
    for (int k = 0; k < Nu; ++k)
        Ru.block(k * m_, k * m_, m_, m_) = p_.R;

    PhiTQy_ = Phi_.transpose() * Qy;   // precompute for online use

    H_ = 2.0 * (PhiTQy_ * Phi_ + Ru);
    H_ = 0.5 * (H_ + H_.transpose()); // symmetrise

    ldlt_.compute(H_);

    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(H_);
    const double L = eig.eigenvalues().maxCoeff();
    L_inv_ = (L > 1e-12) ? (1.0 / L) : 1.0;

    lb_.resize(Nu * m_);
    ub_.resize(Nu * m_);
    for (int k = 0; k < Nu; ++k) {
        lb_.segment(k * m_, m_) = p_.uMin;
        ub_.segment(k * m_, m_) = p_.uMax;
    }
}

// ---------------------------------------------------------------------------
void ScenarioMPC::setState(const Eigen::VectorXd &x)
{
    x_nom_      = x;
    first_step_ = false;
}

void ScenarioMPC::setReference(const Eigen::VectorXd &y_ref)
{
    y_ref_ = y_ref;
}

// ---------------------------------------------------------------------------
Eigen::VectorXd ScenarioMPC::computeRef(const Eigen::VectorXd &x,
                                          const Eigen::VectorXd &y_ref)
{
    setState(x);
    setReference(y_ref);
    return computeControl();
}

// ---------------------------------------------------------------------------
Eigen::VectorXd ScenarioMPC::computeControl()
{
    const int Np = p_.Np;
    const int Nu = p_.Nu;

    // --- sample N_s noise trajectories and compute avg_W -------------------
    // avg_W[j] = C * (mean over s of: sum_{l=0}^{j-1} A^(j-1-l) * w^(s)[l])
    avg_W_.setZero();

    for (int s = 0; s < p_.N_samples; ++s) {
        x_noise_.setZero();
        for (int j = 0; j < Np; ++j) {
            // Accumulate C * x_noise for this scenario's step j
            avg_W_.segment(j * pp_, pp_) += sys_.C * x_noise_;

            // Sample w_j ~ N(0, Sigma_w) = Sigma_w_chol * z, z ~ N(0,I) (reuse z_noise_)
            for (int i = 0; i < n_; ++i)
                z_noise_(i) = normal_(rng_);
            x_noise_ = sys_.A * x_noise_ + Sigma_w_chol_ * z_noise_;
        }
    }
    avg_W_ /= static_cast<double>(p_.N_samples);

    // --- stacked reference (reuse pre-allocated R_stacked_) -----------------
    for (int k = 0; k < Np; ++k)
        R_stacked_.segment(k * pp_, pp_) = y_ref_;

    // --- linear cost: g = 2 * Phi'*Q_blk * (F*x_nom + avg_W - R) -----------
    const Eigen::VectorXd err = F_ * x_nom_ + avg_W_ - R_stacked_;

    // g = 2 * Phi' * Q_blk * err  (PhiTQy_ = Phi' * Q_blk precomputed at construction)
    g_ = 2.0 * (PhiTQy_ * err);

    // --- solve FISTA QP -----------------------------------------------------
    QPSolveResult res = solveGradientProjectionQP(
        H_, g_, lb_, ub_, ldlt_, 1.0 / L_inv_, p_.qpMaxIter, p_.qpTol,
        V_warm_, tmp1_, tmp2_, y_fista_);
    qp_converged_ = res.converged;

    // --- extract first nominal move -----------------------------------------
    Eigen::VectorXd u = V_warm_.head(m_);
    for (int i = 0; i < m_; ++i)
        u(i) = std::clamp(u(i), p_.uMin(i), p_.uMax(i));

    // --- advance nominal model ----------------------------------------------
    x_nom_ = sys_.A * x_nom_ + sys_.B * u;

    // --- shift warm start ---------------------------------------------------
    if (Nu > 1) {
        V_warm_.head((Nu - 1) * m_) = V_warm_.segment(m_, (Nu - 1) * m_);
        V_warm_.tail(m_) = u;
    }

    notifyObserver(u(0), y_ref_(0));
    return u;
}

// ---------------------------------------------------------------------------
double ScenarioMPC::compute(double error)
{
    if (!std::isfinite(error)) return u_prev_;
    // SISO shortcut: reconstruct reference from nominal output + error
    const double y_est = (sys_.C * x_nom_)(0);
    Eigen::VectorXd yref(1);
    yref(0) = y_est + error;
    setReference(yref);
    u_prev_ = computeControl()(0);
    return u_prev_;
}

// ---------------------------------------------------------------------------
void ScenarioMPC::reset()
{
    x_nom_.setZero();
    y_ref_.setZero();
    V_warm_.setZero();
    avg_W_.setZero();
    first_step_   = true;
    qp_converged_ = true;
    notifyObserverReset();
}

} // namespace ctrl
