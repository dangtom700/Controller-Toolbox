#include "KoopmanEDMD.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace ctrl {

// ---------------------------------------------------------------------------
// Lifted dimension computation
// ---------------------------------------------------------------------------

int KoopmanEDMD::computeNLifted() const
{
    const int n = p_.n_state;
    const int m = p_.n_input;
    const int d = n + m;

    switch (p_.dict) {
    case Dict::PolyDeg1:
        return 1 + n + m;

    case Dict::PolyDeg2:
        // 1 + n + m + all degree-2 terms of [x;u]: d*(d+1)/2
        return 1 + n + m + d * (d + 1) / 2;

    case Dict::RBF:
        // n (original state) + n_centres (RBF outputs) + m (inputs passed through)
        return n + p_.rbf_n_cent + m;

    default:
        return 1 + n + m;
    }
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

KoopmanEDMD::KoopmanEDMD(const Params& p) : p_(p)
{
    if (p_.n_state <= 0)
        throw std::invalid_argument("KoopmanEDMD: n_state must be positive");
    n_lifted_ = computeNLifted();
    lift_z_.resize(p_.n_state + p_.n_input);
    lift_psi_.resize(n_lifted_);
}

// ---------------------------------------------------------------------------
// Lifting maps
// ---------------------------------------------------------------------------

Eigen::VectorXd KoopmanEDMD::liftPoly(const Eigen::VectorXd& x,
                                        const Eigen::VectorXd& u) const
{
    const int n = p_.n_state, m = p_.n_input;
    const int d = n + m;
    lift_z_.head(n) = x;
    if (m > 0) lift_z_.tail(m) = u;

    int idx = 0;
    lift_psi_(idx++) = 1.0;
    lift_psi_.segment(idx, n) = x; idx += n;
    if (m > 0) { lift_psi_.segment(idx, m) = u; idx += m; }

    if (p_.dict == Dict::PolyDeg2) {
        for (int i = 0; i < d; ++i)
            for (int j = i; j < d; ++j)
                lift_psi_(idx++) = lift_z_(i) * lift_z_(j);
    }
    return lift_psi_;
}

Eigen::VectorXd KoopmanEDMD::liftRBF(const Eigen::VectorXd& x,
                                       const Eigen::VectorXd& u) const
{
    const int n = p_.n_state, m = p_.n_input;
    Eigen::VectorXd psi(n_lifted_);
    int idx = 0;

    psi.segment(idx, n) = x; idx += n;

    const double inv2sig2 = 0.5 / (p_.rbf_width * p_.rbf_width);
    const int nc = std::min(p_.rbf_n_cent, static_cast<int>(rbf_centres_.cols()));
    for (int c = 0; c < nc; ++c) {
        double d2 = (x - rbf_centres_.col(c)).squaredNorm();
        psi(idx++) = std::exp(-d2 * inv2sig2);
    }
    // Fill remaining centres slots with 0 if not yet set
    while (idx < n + p_.rbf_n_cent)
        psi(idx++) = 0.0;

    if (m > 0) { psi.segment(idx, m) = u; idx += m; }
    return psi;
}

Eigen::VectorXd KoopmanEDMD::lift(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& u) const
{
    if (p_.dict == Dict::RBF) return liftRBF(x, u);
    return liftPoly(x, u);
}

// ---------------------------------------------------------------------------
// RBF centre initialisation (called on first few snapshots)
// ---------------------------------------------------------------------------

void KoopmanEDMD::maybeSetCentres(const Eigen::VectorXd& x)
{
    if (p_.dict != Dict::RBF || centres_set_) return;
    if (static_cast<int>(psi_k_.size()) < p_.rbf_n_cent) return;

    // Pick n_centres random x-snapshots as centres
    rbf_centres_.resize(p_.n_state, p_.rbf_n_cent);
    for (int c = 0; c < p_.rbf_n_cent; ++c) {
        rbf_centres_.col(c) = x_k1_[c].head(p_.n_state);
    }
    centres_set_ = true;
    (void)x;
}

// ---------------------------------------------------------------------------
// Snapshot collection
// ---------------------------------------------------------------------------

void KoopmanEDMD::addSnapshot(const Eigen::VectorXd& x_k,
                                const Eigen::VectorXd& u_k,
                                const Eigen::VectorXd& x_k1)
{
    Eigen::VectorXd zero_u = Eigen::VectorXd::Zero(p_.n_input);
    psi_k_ .push_back(lift(x_k,  u_k));
    psi_k1_.push_back(lift(x_k1, zero_u));
    x_k1_  .push_back(x_k1);

    maybeSetCentres(x_k1);
}

int KoopmanEDMD::snapshotCount() const noexcept
{
    return static_cast<int>(psi_k_.size());
}

void KoopmanEDMD::reserveSnapshots(int N)
{
    psi_k_.reserve(N);
    x_k1_.reserve(N);
    psi_k1_.reserve(N);
}

// ---------------------------------------------------------------------------
// Fit: solve min ||Ψ_{k+1} - K.Ψ_k||_F  (Tikhonov regularised)
//
// K = Ψ_{k+1} . Ψ_k' . (Ψ_k . Ψ_k' + alpha.I)^{-1}
// ---------------------------------------------------------------------------

StateSpace KoopmanEDMD::fit()
{
    const int K = snapshotCount();
    if (K == 0)
        throw std::runtime_error("KoopmanEDMD::fit(): no snapshots added");

    const int nz = n_lifted_;

    // Assemble Ψ_k (nz x K) and Ψ_{k+1} (nz x K)
    Eigen::MatrixXd Psi_k (nz, K), Psi_k1(nz, K);
    for (int j = 0; j < K; ++j) {
        Psi_k .col(j) = psi_k_ [j];
        Psi_k1.col(j) = psi_k1_[j];
    }

    // G = Ψ_k . Ψ_k' + alpha.I  (nz x nz)
    Eigen::MatrixXd G = Psi_k * Psi_k.transpose()
                      + p_.tikhonov * Eigen::MatrixXd::Identity(nz, nz);

    // K_op = Ψ_{k+1} . Ψ_k' . G^{-1}
    Eigen::LDLT<Eigen::MatrixXd> ldlt(G);
    if (ldlt.info() != Eigen::Success)
        throw std::runtime_error("KoopmanEDMD: EDMD factorisation failed");

    // Solve G' . K_op' = (Ψ_{k+1} . Ψ_k')' -> K_op = solution'
    const Eigen::MatrixXd A_lift = ldlt.solve((Psi_k * Psi_k1.transpose())).transpose();
    // A_lift : nz * nz  (lifted state transition)

    // Split A_lift into [A_zz | B_zu] parts:
    // The last p_.n_input columns of ψ carry the input (for poly dict)
    const int n_x = nz - p_.n_input;  // lifted state part
    Eigen::MatrixXd A = A_lift.topLeftCorner(n_x, n_x);
    Eigen::MatrixXd B = A_lift.topRightCorner(n_x, p_.n_input);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(n_x, n_x);  // full lifted output
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(n_x, p_.n_input);

    return StateSpace(A, B, C, D, 1.0);   // Ts=1 (data-driven, discrete)
}

StateSpace KoopmanEDMD::fitProjected()
{
    // Fit full lifted model
    StateSpace ss_lifted = fit();

    // Projection matrix: maps lifted state -> original state
    // P = X_{k+1} . Ψ_k' . (Ψ_k . Ψ_k' + alpha.I)^{-1}  (first n_state rows of K_op)
    const int K   = snapshotCount();
    const int nz  = n_lifted_ - p_.n_input;  // lifted state size (excl. input columns)

    Eigen::MatrixXd Psi_k(nz, K), X_k1(p_.n_state, K);
    for (int j = 0; j < K; ++j) {
        Psi_k .col(j) = psi_k_[j].head(nz);
        X_k1  .col(j) = x_k1_[j];
    }

    Eigen::MatrixXd G = Psi_k * Psi_k.transpose()
                      + p_.tikhonov * Eigen::MatrixXd::Identity(nz, nz);
    Eigen::LDLT<Eigen::MatrixXd> ldlt(G);

    // Output matrix C_proj: n_state * nz
    Eigen::MatrixXd C_proj = ldlt.solve(Psi_k * X_k1.transpose()).transpose();

    // Build projected state-space: n_state states, n_input inputs
    const Eigen::MatrixXd& A_l = ss_lifted.A;
    const Eigen::MatrixXd& B_l = ss_lifted.B;
    Eigen::MatrixXd A_proj = C_proj * A_l;  // n_state * nz (approximate)
    // For full projected model we need A: n_state * n_state
    // Use A_proj * pseudo-inverse(C_proj) as approximation
    Eigen::MatrixXd A_out = A_proj * C_proj.completeOrthogonalDecomposition()
                                             .pseudoInverse();
    Eigen::MatrixXd B_out = C_proj * B_l;
    Eigen::MatrixXd C_out = Eigen::MatrixXd::Identity(p_.n_state, p_.n_state);
    Eigen::MatrixXd D_out = Eigen::MatrixXd::Zero(p_.n_state, p_.n_input);

    return StateSpace(A_out, B_out, C_out, D_out, 1.0);
}

} // namespace ctrl
