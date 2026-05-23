#include "KalmanFilter.h"

namespace ctrl
{

    KalmanFilter::KalmanFilter(const StateSpace &plant,
                               const Eigen::MatrixXd &Q_noise,
                               const Eigen::MatrixXd &R_noise,
                               const Eigen::MatrixXd &P0)
        : plant_(plant), Q_(Q_noise), R_(R_noise), Ts_(plant.Ts)
    {
        const int n = plant.stateSize();
        x_hat_ = Eigen::VectorXd::Zero(n);
        P_ = P0.rows() == n ? P0 : Eigen::MatrixXd::Identity(n, n);
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

        // Enforce minimum noise floor on R to prevent near-singular innovation covariance
        Eigen::MatrixXd R_safe = R_;
        R_safe.diagonal() = R_safe.diagonal().cwiseMax(1e-12);

        // Innovation covariance
        const Eigen::MatrixXd S = C * P_ * C.transpose() + R_safe;

        // Kalman gain - skip update entirely if S is numerically singular
        const auto ldlt = S.ldlt();
        if (ldlt.info() != Eigen::Success)
            return;
        const Eigen::MatrixXd Kf = P_ * C.transpose() *
                                   ldlt.solve(Eigen::MatrixXd::Identity(p, p));

        // State update
        const Eigen::VectorXd innov = y - C * x_hat_ - D * u_current;
        x_hat_ += Kf * innov;

        // Covariance update - Joseph form: P = (I-KC).P.(I-KC)' + K.R.K'
        const Eigen::MatrixXd IKC = Eigen::MatrixXd::Identity(n, n) - Kf * C;
        P_ = IKC * P_ * IKC.transpose() + Kf * R_safe * Kf.transpose();
    }

    void KalmanFilter::step(const Eigen::VectorXd &y,
                            const Eigen::VectorXd &u_prev)
    {
        predict(u_prev);
        update(y, u_prev);
    }

    void KalmanFilter::reset()
    {
        x_hat_.setZero();
        P_ = Eigen::MatrixXd::Identity(plant_.stateSize(), plant_.stateSize());
    }

} // namespace ctrl
