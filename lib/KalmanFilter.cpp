#include "KalmanFilter.h"
#include <optional>

namespace ctrl
{

    KalmanFilter::KalmanFilter(const StateSpace &plant,
                               const Eigen::MatrixXd &Q_noise,
                               const Eigen::MatrixXd &R_noise,
                               const Eigen::MatrixXd &P0)
        : plant_(plant), Q_(Q_noise), R_(R_noise), Ts_(plant.Ts)
    {
        const int n = plant.stateSize();
        const int p = plant.outputSize();
        x_hat_ = Eigen::VectorXd::Zero(n);
        P_ = P0.rows() == n ? P0 : Eigen::MatrixXd::Identity(n, n);
        R_safe_.resize(p, p);
        S_    .resize(p, p);
        Kf_   .resize(n, p);
        IKC_  .resize(n, n);
        P_new_.resize(n, n);
    }

    // Predict: advance state estimate and inflate covariance with process noise.
    void KalmanFilter::predict(const Eigen::VectorXd &u)
    {
        x_hat_ = plant_.A * x_hat_ + plant_.B * u;
        P_ = plant_.A * P_ * plant_.A.transpose() + Q_;
    }

    // Update: incorporate measurement y[k], correct estimate, deflate covariance.
    // Joseph form for P update maintains positive semi-definiteness numerically.
    void KalmanFilter::update(const Eigen::VectorXd &y,
                              const Eigen::VectorXd &u_current)
    {
        const Eigen::MatrixXd &C = plant_.C;
        const Eigen::MatrixXd &D = plant_.D;
        const int p = plant_.outputSize();
        const int n = plant_.stateSize();

        // Enforce minimum noise floor on R_ into pre-allocated R_safe_
        R_safe_ = R_;
        R_safe_.diagonal() = R_safe_.diagonal().cwiseMax(1e-12);

        // Innovation covariance (pre-allocated S_)
        S_.noalias() = C * P_ * C.transpose();
        S_ += R_safe_;

        // Kalman gain (pre-allocated Kf_) - skip update entirely if S is numerically singular
        const auto ldlt = S_.ldlt();
        if (ldlt.info() != Eigen::Success)
            return;
        Kf_.noalias() = P_ * C.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(p, p));

        // State update
        const Eigen::VectorXd innov = y - C * x_hat_ - D * u_current;
        if (mismatch_det_) mismatch_det_->update(innov);
        x_hat_.noalias() += Kf_ * innov;

        // Covariance update - Joseph form: P = (I-KC).P.(I-KC)' + K.R.K'
        // Accumulate into P_new_ to avoid aliasing (P_ appears on both sides)
        IKC_.noalias() = Eigen::MatrixXd::Identity(n, n) - Kf_ * C;
        P_new_.noalias() = IKC_ * P_ * IKC_.transpose();
        P_new_.noalias() += Kf_ * R_safe_ * Kf_.transpose();
        P_ = P_new_;
    }

    void KalmanFilter::step(const Eigen::VectorXd &y,
                            const Eigen::VectorXd &u_prev,
                            std::optional<std::reference_wrapper<const Eigen::VectorXd>> u_current)
    {
        predict(u_prev);
        // Use u_current for the D.u innovation term when provided; fall back to u_prev for D=0 plants.
        update(y, u_current.has_value() ? u_current->get() : u_prev);
    }

    // Plain-reference overload - avoids std::optional<std::reference_wrapper<...>> for pybind11.
    void KalmanFilter::step(const Eigen::VectorXd &y,
                            const Eigen::VectorXd &u_prev,
                            const Eigen::VectorXd &u_current)
    {
        predict(u_prev);
        update(y, u_current);
    }

    void KalmanFilter::reset()
    {
        x_hat_.setZero();
        P_ = Eigen::MatrixXd::Identity(plant_.stateSize(), plant_.stateSize());
    }

} // namespace ctrl
