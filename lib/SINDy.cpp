#include "SINDy.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

// ---------------------------------------------------------------------------
// Library term count
// ---------------------------------------------------------------------------

int SINDy::computeNTerms() const
{
    const int n = p_.n_state;
    const int m = p_.n_input;
    const int d = n + m;   // combined dimension for polynomial terms

    switch (p_.library) {
    case SINDyLibrary::PolyDeg1:
        // 1 (constant) + d (linear)
        return 1 + d;

    case SINDyLibrary::PolyDeg2:
        // 1 + d + d*(d+1)/2  (all monomials up to degree 2)
        return 1 + d + d * (d + 1) / 2;

    case SINDyLibrary::PolyDeg3:
        // degree 0 + degree 1 + degree 2 + degree 3 terms
        // C(d+3,3) via stars-and-bars
        return (d + 1) * (d + 2) * (d + 3) / 6;

    case SINDyLibrary::PolyTrig:
        // PolyDeg2 terms + sin(x_i) + cos(x_i) for each state
        return 1 + d + d * (d + 1) / 2 + 2 * n;

    default:
        return 1 + d + d * (d + 1) / 2;
    }
}

// ---------------------------------------------------------------------------
// SINDy constructor
// ---------------------------------------------------------------------------

SINDy::SINDy(const Params& p) : p_(p)
{
    if (p_.n_state <= 0 || p_.n_input < 0)
        throw std::invalid_argument("SINDy: n_state must be > 0, n_input >= 0");
    n_terms_ = computeNTerms();
}

// ---------------------------------------------------------------------------
// Library row builder  Theta(x, u)
// ---------------------------------------------------------------------------

Eigen::VectorXd SINDy::libraryRow(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& u) const
{
    const int n = p_.n_state;
    const int m = p_.n_input;
    Eigen::VectorXd row(n_terms_);
    int idx = 0;

    // Combine [x; u] into a single vector z for polynomial generation
    Eigen::VectorXd z(n + m);
    z.head(n) = x;
    if (m > 0) z.tail(m) = u;
    const int d = n + m;

    // --- Degree 0: constant 1 ---
    row(idx++) = 1.0;

    // --- Degree 1: z_i ---
    for (int i = 0; i < d; ++i)
        row(idx++) = z(i);

    if (p_.library == SINDyLibrary::PolyDeg1) {
        return row;
    }

    // --- Degree 2: z_i * z_j  (i <= j) ---
    for (int i = 0; i < d; ++i)
        for (int j = i; j < d; ++j)
            row(idx++) = z(i) * z(j);

    if (p_.library == SINDyLibrary::PolyDeg2) {
        return row;
    }

    if (p_.library == SINDyLibrary::PolyTrig) {
        // Trig: sin(x_i), cos(x_i) for each state component
        for (int i = 0; i < n; ++i)
            row(idx++) = std::sin(x(i));
        for (int i = 0; i < n; ++i)
            row(idx++) = std::cos(x(i));
        return row;
    }

    // --- Degree 3: z_i * z_j * z_k  (i <= j <= k) ---
    for (int i = 0; i < d; ++i)
        for (int j = i; j < d; ++j)
            for (int k = j; k < d; ++k)
                row(idx++) = z(i) * z(j) * z(k);

    return row;
}

// ---------------------------------------------------------------------------
// Snapshot accumulation
// ---------------------------------------------------------------------------

void SINDy::addSnapshot(const Eigen::VectorXd& x_dev,
                          const Eigen::VectorXd& u_dev,
                          const Eigen::VectorXd& xdot_dev)
{
    theta_rows_.push_back(libraryRow(x_dev, u_dev));
    xdot_rows_ .push_back(xdot_dev);
}

void SINDy::addSnapshotFD(const Eigen::VectorXd& x_k,
                            const Eigen::VectorXd& x_k1,
                            const Eigen::VectorXd& u_k,
                            double Ts)
{
    addSnapshot(x_k, u_k, (x_k1 - x_k) / Ts);
}

int SINDy::snapshotCount() const noexcept
{
    return static_cast<int>(theta_rows_.size());
}

void SINDy::reserveSnapshots(int N)
{
    theta_rows_.reserve(N);
    xdot_rows_.reserve(N);
}

// ---------------------------------------------------------------------------
// Sequentially-Thresholded Least Squares (STLS)
// ---------------------------------------------------------------------------

Eigen::MatrixXd SINDy::stls(const Eigen::MatrixXd& Theta,
                               const Eigen::MatrixXd& Xdot) const
{
    const int K = static_cast<int>(Theta.rows());  // snapshots
    const int L = static_cast<int>(Theta.cols());  // library terms
    const int S = static_cast<int>(Xdot.cols());   // states

    // Initial least-squares estimate (possibly dense)
    Eigen::MatrixXd Xi = Theta.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                               .solve(Xdot);

    if (p_.use_ols)
        return Xi;

    // STLS: iterate threshold + refit on active terms
    for (int iter = 0; iter < p_.stls_iter; ++iter) {
        // Threshold small coefficients
        for (int s = 0; s < S; ++s)
            for (int l = 0; l < L; ++l)
                if (std::abs(Xi(l, s)) < p_.threshold)
                    Xi(l, s) = 0.0;

        // Refit each state using only the active (non-zero) terms
        for (int s = 0; s < S; ++s) {
            // Find active indices for column s
            stls_active_.clear();
            for (int l = 0; l < L; ++l)
                if (Xi(l, s) != 0.0)
                    stls_active_.push_back(l);

            if (stls_active_.empty())
                continue;

            // Sub-matrix of active columns
            const int na = static_cast<int>(stls_active_.size());
            stls_Theta_a_.resize(K, na);
            for (int j = 0; j < na; ++j)
                stls_Theta_a_.col(j) = Theta.col(stls_active_[j]);

            // Least-squares on reduced system
            Eigen::VectorXd xi_a =
                stls_Theta_a_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                             .solve(Xdot.col(s));

            // Write back, zeroing inactive
            Xi.col(s).setZero();
            for (int j = 0; j < static_cast<int>(stls_active_.size()); ++j)
                Xi(stls_active_[j], s) = xi_a(j);
        }
    }
    return Xi;
}

// ---------------------------------------------------------------------------
// fit()
// ---------------------------------------------------------------------------

SINDyModel SINDy::fit() const
{
    if (theta_rows_.empty())
        throw std::runtime_error("SINDy::fit(): no snapshots have been added");

    const int K = static_cast<int>(theta_rows_.size());
    const int L = n_terms_;
    const int S = p_.n_state;

    // Assemble matrices
    Eigen::MatrixXd Theta(K, L);
    Eigen::MatrixXd Xdot(K, S);
    for (int k = 0; k < K; ++k) {
        Theta.row(k) = theta_rows_[k].transpose();
        Xdot .row(k) = xdot_rows_ [k].transpose();
    }

    // Solve
    Eigen::MatrixXd Xi = stls(Theta, Xdot);  // L * S

    // Package result
    SINDyModel model;
    model.n_state_  = p_.n_state;
    model.n_input_  = p_.n_input;
    model.lib_type_ = p_.library;
    model.xi_       = Xi;
    return model;
}

// ---------------------------------------------------------------------------
// SINDyModel
// ---------------------------------------------------------------------------

Eigen::VectorXd SINDyModel::predict(const Eigen::VectorXd& x_dev,
                                      const Eigen::VectorXd& u_dev) const
{
    // Lazy-initialise the cached helper so the constructor runs once, not once per call
    if (!helper_cache_) {
        SINDy::Params p_tmp;
        p_tmp.n_state = n_state_;
        p_tmp.n_input = n_input_;
        p_tmp.library = lib_type_;
        helper_cache_ = std::make_unique<SINDy>(p_tmp);
        row_work_.resize(helper_cache_->nTerms());
    }
    row_work_ = helper_cache_->libraryRow(x_dev, u_dev);
    return xi_.transpose() * row_work_;
}

StateFunc SINDyModel::stateFunc() const
{
    // Capture by value so the lambda owns a copy of the model
    SINDyModel self = *this;
    return [self](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        return self.predict(x, u);
    };
}

double SINDyModel::sparsity() const
{
    if (xi_.size() == 0) return 0.0;
    int nz = 0;
    for (int i = 0; i < xi_.rows(); ++i)
        for (int j = 0; j < xi_.cols(); ++j)
            if (xi_(i, j) != 0.0) ++nz;
    return 1.0 - static_cast<double>(nz) / xi_.size();
}

} // namespace ctrl
