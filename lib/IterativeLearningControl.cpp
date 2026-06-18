#include "IterativeLearningControl.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

ILC::ILC(const Params& p) : p_(p)
{
    if (p_.N <= 0)
        throw std::invalid_argument("ILC: N must be positive");
    if (p_.Lp < 0.0 || p_.Lp > 1.0)
        throw std::invalid_argument("ILC: Lp must be in [0, 1]");
    if (p_.Q_filter <= 0.0 || p_.Q_filter > 1.0)
        throw std::invalid_argument("ILC: Q_filter must be in (0, 1]");

    u_ff_.setZero(p_.N);
    e_curr_.setZero(p_.N);
    e_prev_.setZero(p_.N);
}

ILC::ILC(const Params& p, const Eigen::MatrixXd& G) : ILC(p)
{
    if (G.rows() != p_.N || G.cols() != p_.N)
        throw std::invalid_argument(
            "ILC: Markov matrix G must be N x N (N=" + std::to_string(p_.N) + ")");
    buildNormOptimal(G);
    has_G_ = true;
}

// ---------------------------------------------------------------------------
// Norm-optimal pre-computation
// L_opt = (G'*rho_e*G + rho_u*I)^{-1} * G' * rho_e
// Q_e   = I - L_opt * G
// ---------------------------------------------------------------------------

void ILC::buildNormOptimal(const Eigen::MatrixXd& G)
{
    const int N = p_.N;
    const Eigen::MatrixXd GtG = G.transpose() * G;
    const Eigen::MatrixXd M   = p_.rho_e * GtG
                               + p_.rho_u * Eigen::MatrixXd::Identity(N, N);

    Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
    if (ldlt.info() != Eigen::Success)
        throw std::runtime_error("ILC (norm-optimal): failed to factor (G'G + rho_u.I)");

    // L_opt = M^{-1} * G' * rho_e  ->  solve M * L_opt' = G * rho_e
    const Eigen::MatrixXd rhs = p_.rho_e * G.transpose();
    // ldlt.solve(rhs) solves M * X = rhs  for X = L_opt'  ->  transpose
    L_opt_ = ldlt.solve(rhs).transpose();   // N x N

    Q_e_   = Eigen::MatrixXd::Identity(N, N) - L_opt_ * G;
}

// ---------------------------------------------------------------------------
// Trial interface
// ---------------------------------------------------------------------------

double ILC::feedforward(int k) const
{
    if (k < 0 || k >= p_.N)
        return 0.0;
    return u_ff_(k);
}

void ILC::recordError(int k, double error)
{
    if (k >= 0 && k < p_.N)
        e_curr_(k) = error;
}

void ILC::updateFeedforward()
{
    rms_last_ = std::sqrt(e_curr_.squaredNorm() / p_.N);

    Eigen::VectorXd u_new(p_.N);

    if (p_.mode == Mode::NormOptimal && has_G_) {
        // Norm-optimal: u_{j+1} = Q_e * u_j + L_opt * e_j
        u_new = Q_e_ * u_ff_ + L_opt_ * e_curr_;

    } else if (p_.mode == Mode::DType) {
        // D-type: u_{j+1}[k] = Q * u_j[k] + Lp * e_j[k] + Ld * (e_j[k] - e_j[k-1]) / Ts
        for (int k = 0; k < p_.N; ++k) {
            double de = (k > 0) ? (e_curr_(k) - e_curr_(k - 1)) / p_.Ts
                                : (e_curr_(k) - e_prev_(p_.N - 1)) / p_.Ts;
            u_new(k) = p_.Q_filter * u_ff_(k)
                     + p_.Lp * e_curr_(k)
                     + p_.Ld * de;
        }

    } else {
        // P-type (default): u_{j+1}[k] = Q * u_j[k] + Lp * e_j[k]
        for (int k = 0; k < p_.N; ++k)
            u_new(k) = p_.Q_filter * u_ff_(k) + p_.Lp * e_curr_(k);
    }

    // Saturate feedforward
    u_new = u_new.cwiseMin(p_.uMax).cwiseMax(p_.uMin);

    // Shift: previous error becomes e_prev for D-type
    e_prev_ = e_curr_;
    e_curr_.setZero();

    u_ff_ = u_new;
    ++trial_;
}

void ILC::reset()
{
    u_ff_.setZero();
    e_curr_.setZero();
    e_prev_.setZero();
    trial_    = 0;
    rms_last_ = 0.0;
}

} // namespace ctrl
