#include "UnscentedKalmanFilter.h"
#include <iostream>
#include <stdexcept>

namespace ctrl
{

    UnscentedKalmanFilter::UnscentedKalmanFilter(
        int n, int p,
        std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> f,
        std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> h,
        const Eigen::MatrixXd &Q_noise,
        const Eigen::MatrixXd &R_noise,
        double Ts,
        const Eigen::MatrixXd &P0,
        double alpha, double beta, double kappa)
        : n_states_(n), n_outputs_(p), f_(std::move(f)), h_(std::move(h)),
          Q_(Q_noise), R_(R_noise), Ts_(Ts)
    {
        x_hat_ = Eigen::VectorXd::Zero(n);
        P_ = P0.rows() == n ? P0 : Eigen::MatrixXd::Identity(n, n);

        // Scaled UT parameters
        lambda_ = alpha * alpha * (n + kappa) - n;

        // Guard: (n + lambda) must be positive for the Cholesky sqrt in sigmaPoints()
        // to be well-defined. Equivalent condition: alpha^2*(n+kappa) > 0.
        if (n + lambda_ <= 0.0)
            throw std::invalid_argument(
                "[UnscentedKalmanFilter] n + lambda <= 0: sigma point spread is invalid. "
                "Increase alpha or set kappa >= 0. "
                "Typical values: alpha=1e-3, kappa=0 or kappa=3-n.");

        const int L = 2 * n + 1;
        Wm_.resize(L);
        Wc_.resize(L);
        Wm_(0) = lambda_ / (n + lambda_);
        Wc_(0) = lambda_ / (n + lambda_) + (1.0 - alpha * alpha + beta);

        // Warn when Wc(0) < 0: the zeroth covariance weight is negative, which is
        // mathematically valid in the sigma-point formulation but can cause P to lose
        // positive-semidefiniteness numerically. Consider increasing beta or alpha.
#ifndef NDEBUG
        if (Wc_(0) < 0.0)
            std::cerr << "[UnscentedKalmanFilter] WARNING: Wc(0) = " << Wc_(0)
                      << " < 0. Predicted covariance may lose PSD. "
                      << "Consider increasing beta (default 2) or alpha.\n";
#endif

        const double w_rest = 0.5 / (n + lambda_);
        for (int i = 1; i < L; ++i)
            Wm_(i) = Wc_(i) = w_rest;
    }

    // Cholesky of (n+lambda)P, columns are the perturbation vectors.
    Eigen::MatrixXd UnscentedKalmanFilter::sigmaPoints() const
    {
        const Eigen::MatrixXd S =
            ((n_states_ + lambda_) * P_).llt().matrixL(); // lower triangular sqrt

        Eigen::MatrixXd X(n_states_, 2 * n_states_ + 1);
        X.col(0) = x_hat_;
        for (int i = 0; i < n_states_; ++i)
        {
            X.col(i + 1)             = x_hat_ + S.col(i);
            X.col(i + 1 + n_states_) = x_hat_ - S.col(i);
        }
        return X;
    }

    void UnscentedKalmanFilter::predict(const Eigen::VectorXd &u)
    {
        const Eigen::MatrixXd X = sigmaPoints();
        const int L = 2 * n_states_ + 1;

        // Propagate sigma points through f
        Eigen::MatrixXd Xp(n_states_, L);
        for (int i = 0; i < L; ++i)
            Xp.col(i) = f_(X.col(i), u);

        // Predicted mean
        x_hat_.setZero();
        for (int i = 0; i < L; ++i)
            x_hat_.noalias() += Wm_(i) * Xp.col(i);

        // Predicted covariance - outer-product accumulation; noalias() avoids per-iteration temps.
        P_ = Q_;
        for (int i = 0; i < L; ++i)
        {
            const Eigen::VectorXd d = Xp.col(i) - x_hat_;
            P_.noalias() += Wc_(i) * d * d.transpose();
        }
    }

    void UnscentedKalmanFilter::update(const Eigen::VectorXd &y,
                                       const Eigen::VectorXd &u)
    {
        const Eigen::MatrixXd X = sigmaPoints();
        const int L = 2 * n_states_ + 1;

        // Propagate sigma points through h
        Eigen::MatrixXd Y(n_outputs_, L);
        for (int i = 0; i < L; ++i)
            Y.col(i) = h_(X.col(i), u);

        // Predicted measurement mean
        Eigen::VectorXd y_hat = Eigen::VectorXd::Zero(n_outputs_);
        for (int i = 0; i < L; ++i)
            y_hat.noalias() += Wm_(i) * Y.col(i);

        Eigen::MatrixXd R_safe = R_;
        R_safe.diagonal() = R_safe.diagonal().cwiseMax(1e-12);

        // Innovation covariance Syy and cross-covariance Pxy
        Eigen::MatrixXd Syy = R_safe;
        Eigen::MatrixXd Pxy = Eigen::MatrixXd::Zero(n_states_, n_outputs_);
        for (int i = 0; i < L; ++i)
        {
            const Eigen::VectorXd dx = X.col(i) - x_hat_;
            const Eigen::VectorXd dy = Y.col(i) - y_hat;
            Syy.noalias() += Wc_(i) * dy * dy.transpose();
            Pxy.noalias() += Wc_(i) * dx * dy.transpose();
        }

        const auto ldlt = Syy.ldlt();
        if (ldlt.info() != Eigen::Success)
            return;

        const Eigen::MatrixXd K = Pxy * ldlt.solve(Eigen::MatrixXd::Identity(n_outputs_, n_outputs_));

        x_hat_.noalias() += K * (y - y_hat);

        // Covariance update: P = P - K*Syy*K'
        // Unlike the linear KF (which uses the numerically superior Joseph form),
        // the UKF Joseph form requires an explicit H matrix that does not exist here -
        // H is replaced by the sigma-point cross-covariance Pxy/Syy.  The raw
        // subtraction is the standard UKF formula (Wan & Van der Merwe 2000, eq. 26).
        // Round-off in this subtraction can cause P to lose PSD on large measurement
        // updates; the eigenvalue floor below catches this case.
        P_.noalias() -= K * Syy * K.transpose();

        // Enforce PSD: symmetrise first, then clamp any negative eigenvalue to zero.
        // Symmetrisation alone is insufficient - a negative diagonal entry survives it.
        P_ = 0.5 * (P_ + P_.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(P_);
        if (eig.eigenvalues().minCoeff() < 0.0)
            P_ += (-eig.eigenvalues().minCoeff() + 1e-10) *
                  Eigen::MatrixXd::Identity(n_states_, n_states_);
    }

    void UnscentedKalmanFilter::step(const Eigen::VectorXd &y,
                                     const Eigen::VectorXd &u_prev)
    {
        predict(u_prev);
        update(y, u_prev);
    }

    void UnscentedKalmanFilter::reset()
    {
        x_hat_.setZero();
        P_ = Eigen::MatrixXd::Identity(n_states_, n_states_);
    }

} // namespace ctrl
