#include "TubeMPC.h"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace ctrl {

// ---------------------------------------------------------------------------
TubeMPC::TubeMPC(const StateSpace &sys, const TubeMPCParams &p)
    : sys_(sys), p_(p),
      n_(static_cast<int>(sys.A.rows())),
      m_(static_cast<int>(sys.B.cols())),
      pp_(static_cast<int>(sys.C.rows()))
{
    // --- auto-fill optional parameters ---
    if (p_.K.size() == 0)
        p_.K = Eigen::MatrixXd::Zero(m_, n_);
    if (p_.wMax.size() == 0)
        p_.wMax = Eigen::VectorXd::Zero(n_);
    if (p_.uMin.size() == 0)
        p_.uMin = Eigen::VectorXd::Constant(m_, -1e9);
    if (p_.uMax.size() == 0)
        p_.uMax = Eigen::VectorXd::Constant(m_,  1e9);

    // --- dimension checks ---
    if (p_.Nu > p_.Np)
        throw std::invalid_argument("TubeMPC: Nu must be <= Np.");
    if (p_.Q.rows() != pp_ || p_.Q.cols() != pp_)
        throw std::invalid_argument("TubeMPC: Q must be pp x pp.");
    if (p_.R.rows() != m_ || p_.R.cols() != m_)
        throw std::invalid_argument("TubeMPC: R must be m x m.");
    if (p_.K.rows() != m_ || p_.K.cols() != n_)
        throw std::invalid_argument("TubeMPC: K must be m x n.");
    if (p_.wMax.size() != n_)
        throw std::invalid_argument("TubeMPC: wMax must have n elements.");
    if (p_.uMin.size() != m_ || p_.uMax.size() != m_)
        throw std::invalid_argument("TubeMPC: uMin/uMax must have m elements.");
    if ((p_.wMax.array() < 0.0).any())
        throw std::invalid_argument("TubeMPC: wMax elements must be >= 0.");

    // --- spectral radius check: rho(A_cl) < 1 ---
    const Eigen::MatrixXd A_cl = sys_.A + sys_.B * p_.K;
    const double rho = A_cl.eigenvalues().cwiseAbs().maxCoeff();
    if (rho >= 1.0)
        throw std::invalid_argument(
            "TubeMPC: K does not stabilise A+B*K (spectral radius = " +
            std::to_string(rho) + " >= 1). Choose a stabilising gain K.");

    // --- compute mRPI and build condensed matrices ---
    computeMRPI();
    buildCondensedMatrices();

    // --- initialise online state ---
    x_actual_ = Eigen::VectorXd::Zero(n_);
    x_nom_    = Eigen::VectorXd::Zero(n_);
    y_ref_    = Eigen::VectorXd::Zero(pp_);
    V_warm_   = Eigen::VectorXd::Zero(p_.Nu * m_);
    g_        = Eigen::VectorXd::Zero(p_.Nu * m_);
    tmp1_     = Eigen::VectorXd::Zero(p_.Nu * m_);
    tmp2_     = Eigen::VectorXd::Zero(p_.Nu * m_);
}

// ---------------------------------------------------------------------------
// Minimal Robust Positively Invariant set via iterated box-summation.
//
// For the closed-loop system  e[k+1] = A_cl * e[k] + w[k],
// the mRPI set is the limit of the Minkowski sum:
//   Z_inf = ⊕_{k=0}^{inf} A_cl^k * W
//
// For box W = {w : |w_i| <= wMax_i} the element-wise bound is:
//   z_max_new = |A_cl| * z_max + wMax
// (where |A_cl| is the matrix of absolute values of A_cl elements).
// This converges because rho(A_cl) < 1 (verified in constructor).
// ---------------------------------------------------------------------------
void TubeMPC::computeMRPI()
{
    const Eigen::MatrixXd A_cl     = sys_.A + sys_.B * p_.K;
    const Eigen::MatrixXd A_cl_abs = A_cl.cwiseAbs();

    z_max_ = Eigen::VectorXd::Zero(n_);
    for (int iter = 0; iter < p_.mRPI_maxIter; ++iter) {
        const Eigen::VectorXd z_new = A_cl_abs * z_max_ + p_.wMax;
        if ((z_new - z_max_).cwiseAbs().maxCoeff() < p_.mRPI_tol)
        {
            z_max_ = z_new;
            break;
        }
        z_max_ = z_new;
    }

    // Tighten input constraints: v \in [uMin + |K|*z_max, uMax - |K|*z_max]
    const Eigen::MatrixXd K_abs = p_.K.cwiseAbs();
    const Eigen::VectorXd ku    = K_abs * z_max_;
    v_min_ = p_.uMin + ku;
    v_max_ = p_.uMax - ku;

    // Guard against infeasible tightening (would mean wMax or K is too large)
    for (int i = 0; i < m_; ++i) {
        if (v_min_(i) > v_max_(i)) {
            // Fall back to zero-width interval at midpoint (safest fallback)
            const double mid = 0.5 * (p_.uMin(i) + p_.uMax(i));
            v_min_(i) = mid;
            v_max_(i) = mid;
        }
    }
}

// ---------------------------------------------------------------------------
// Condensed prediction matrices.
//
//   Y_nom = F * x_nom_0 + Phi * V
//
// F   : (Np*pp) x n     -- free-response (no input)
// Phi : (Np*pp) x (Nu*m)-- Markov response with hold-last at j = Nu-1
//
// Phi[i, j] = C * A^(i-1-j) * B           for j < Nu-1 and j <= i-1
// Phi[i, Nu-1] = C * sum_{t=Nu-1}^{i-1} A^(i-1-t) * B  (accumulated hold-last)
//
// Hessian: H = 2*(Phi' * Qy * Phi + Ru)  where Qy = block-diag(Q,..Q), Ru = block-diag(R,..R)
// ---------------------------------------------------------------------------
void TubeMPC::buildCondensedMatrices()
{
    const int Np = p_.Np;
    const int Nu = p_.Nu;

    F_.resize(Np * pp_, n_);
    Phi_.resize(Np * pp_, Nu * m_);
    Phi_.setZero();

    // Precompute A^k * B for k=0..Np-1
    Eigen::MatrixXd Ak(n_, n_);
    Ak.setIdentity();
    std::vector<Eigen::MatrixXd> CB_list(Np); // CB_list[k] = C * A^k * B

    for (int k = 0; k < Np; ++k) {
        CB_list[k] = sys_.C * Ak * sys_.B;  // pp x m
        Ak = sys_.A * Ak;                    // A^(k+1)
    }

    // F: row block i (0-based) = C * A^(i+1) * x0  (i=0..Np-1)
    Eigen::MatrixXd Aipow(n_, n_);
    Aipow = sys_.A;
    for (int i = 0; i < Np; ++i) {
        F_.block(i * pp_, 0, pp_, n_) = sys_.C * Aipow;
        Aipow = sys_.A * Aipow; // A^(i+2) for next iteration
    }

    // Phi: column block j (0-based) covers input at time j (or hold-last at j=Nu-1)
    // Phi[i, j] for i >= j: CB_list[i-j-1] (since C*A^(i-j-1)*B)
    for (int j = 0; j < Nu; ++j) {
        const bool is_last = (j == Nu - 1);
        for (int i = j; i < Np; ++i) {
            // At step i, input at step j contributes C*A^(i-j-1)*B
            // For hold-last at j=Nu-1, accumulate from j onward
            // CB_list[k] = C * A^k * B
            // Phi[row i, col j] = C * A^(i-j) * B  for j <= i  (0-based block indices)
            Phi_.block(i * pp_, j * m_, pp_, m_) += CB_list[i - j];

            if (is_last && i < Np - 1) {
                // For hold-last: input v[Nu-1] is repeated at steps Nu, Nu+1, ..., Np-1.
                // The accumulated contribution is already handled via the loop below.
                // Nothing extra here - the hold-last accumulation is done in the inner loop.
            }
        }

        // For hold-last at j = Nu-1: add contributions for t = Nu, ..., Np-1
        // where v[t] = v[Nu-1]  (held), and these affect rows i >= t.
        if (is_last) {
            for (int t = Nu; t < Np; ++t) { // extra held steps
                for (int i = t; i < Np; ++i) {
                    Phi_.block(i * pp_, j * m_, pp_, m_) += CB_list[i - t];
                }
            }
        }
    }

    // Build cost matrices
    // Qy = block-diag(Q, Q, ..., Q)  size (Np*pp) x (Np*pp)
    // Ru = block-diag(R, R, ..., R)  size (Nu*m)  x (Nu*m)
    // H  = 2*(Phi'*Qy*Phi + Ru)
    Eigen::MatrixXd Qy = Eigen::MatrixXd::Zero(Np * pp_, Np * pp_);
    for (int k = 0; k < Np; ++k)
        Qy.block(k * pp_, k * pp_, pp_, pp_) = p_.Q;

    Eigen::MatrixXd Ru = Eigen::MatrixXd::Zero(Nu * m_, Nu * m_);
    for (int k = 0; k < Nu; ++k)
        Ru.block(k * m_, k * m_, m_, m_) = p_.R;

    const Eigen::MatrixXd PhiTQy = Phi_.transpose() * Qy;
    H_ = 2.0 * (PhiTQy * Phi_ + Ru);
    H_ = 0.5 * (H_ + H_.transpose()); // symmetrise

    ldlt_.compute(H_);

    // Lipschitz constant = lambda_max(H)
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(H_);
    const double L = eig.eigenvalues().maxCoeff();
    L_inv_ = (L > 1e-12) ? (1.0 / L) : 1.0;

    // Stacked box bounds for FISTA
    lb_.resize(Nu * m_);
    ub_.resize(Nu * m_);
    for (int k = 0; k < Nu; ++k) {
        lb_.segment(k * m_, m_) = v_min_;
        ub_.segment(k * m_, m_) = v_max_;
    }
}

// ---------------------------------------------------------------------------
void TubeMPC::setState(const Eigen::VectorXd &x)
{
    x_actual_ = x;
    if (first_step_) {
        x_nom_     = x;
        first_step_ = false;
    }
    state_set_ = true;
}

void TubeMPC::setReference(const Eigen::VectorXd &y_ref)
{
    y_ref_ = y_ref;
}

// ---------------------------------------------------------------------------
Eigen::VectorXd TubeMPC::computeRef(const Eigen::VectorXd &x,
                                     const Eigen::VectorXd &y_ref)
{
    setState(x);
    setReference(y_ref);
    return computeControl();
}

// ---------------------------------------------------------------------------
Eigen::VectorXd TubeMPC::computeControl()
{
    const int Nu = p_.Nu;

    // --- build stacked reference ---
    Eigen::VectorXd R_stacked(p_.Np * pp_);
    for (int k = 0; k < p_.Np; ++k)
        R_stacked.segment(k * pp_, pp_) = y_ref_;

    // --- linear cost: g = 2*Phi'*Qy*(F*x_nom - R_stacked) ---
    // (Qy factored into Phi'*Q block products for efficiency)
    const Eigen::VectorXd Fx = F_ * x_nom_;
    const Eigen::VectorXd err = Fx - R_stacked; // (Np*pp) x 1

    // g = 2 * Phi' * Qy * err
    // Qy = block-diag(Q) - apply block-wise to avoid explicit Qy allocation
    Eigen::VectorXd Qy_err(p_.Np * pp_);
    for (int k = 0; k < p_.Np; ++k)
        Qy_err.segment(k * pp_, pp_) = p_.Q * err.segment(k * pp_, pp_);
    g_ = 2.0 * (Phi_.transpose() * Qy_err);

    // --- solve QP ---
    QPSolveResult res = solveGradientProjectionQP(
        H_, g_, lb_, ub_, ldlt_, 1.0 / L_inv_, p_.qpMaxIter, p_.qpTol,
        V_warm_, tmp1_, tmp2_);
    qp_converged_ = res.converged;

    // --- extract nominal first move ---
    const Eigen::VectorXd v_nom = V_warm_.head(m_);

    // --- composite tube control: u = K*(x - x_nom) + v_nom ---
    Eigen::VectorXd u = p_.K * (x_actual_ - x_nom_) + v_nom;
    // Hard clamp to physical limits (tube feedback may slightly exceed tightened limits)
    for (int i = 0; i < m_; ++i)
        u(i) = std::clamp(u(i), p_.uMin(i), p_.uMax(i));

    // --- advance nominal model ---
    x_nom_ = sys_.A * x_nom_ + sys_.B * v_nom;

    state_set_ = false;

    // Shift warm start (move-blocking)
    if (Nu > 1) {
        V_warm_.head((Nu - 1) * m_) = V_warm_.segment(m_, (Nu - 1) * m_);
        V_warm_.tail(m_) = v_nom; // hold last
    }

    const double u_scalar = u(0);
    notifyObserver(u_scalar, y_ref_(0));
    return u;
}

// ---------------------------------------------------------------------------
double TubeMPC::compute(double error)
{
    if (!std::isfinite(error)) return 0.0;
    // Reconstruct scalar reference from current y (= C*x_nom approx) + error
    const double y_approx = (sys_.C * x_nom_)(0);
    const double r = y_approx + error;
    Eigen::VectorXd yref(1);
    yref(0) = r;
    setReference(yref);
    return computeControl()(0);
}

// ---------------------------------------------------------------------------
void TubeMPC::reset()
{
    x_nom_     = Eigen::VectorXd::Zero(n_);
    x_actual_  = Eigen::VectorXd::Zero(n_);
    y_ref_     = Eigen::VectorXd::Zero(pp_);
    V_warm_.setZero();
    first_step_ = true;
    state_set_  = false;
    qp_converged_ = true;
    notifyObserverReset();
}

} // namespace ctrl
