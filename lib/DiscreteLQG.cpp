#include "DiscreteLQG.h"
#include <iostream>

namespace ctrl
{

    DiscreteLQG::DiscreteLQG(const StateSpace &plant,
                             const LQRParams &lqr_p,
                             const Eigen::MatrixXd &Q_noise,
                             const Eigen::MatrixXd &R_noise,
                             const Eigen::MatrixXd &P0)
        : plant_(plant)
    {
        // D != 0: the Kalman innovation y - C*x^ - D*u uses u_prev (one step stale)
        // instead of the true u[k]. Accuracy degrades proportionally to D's magnitude.
        if (!plant.D.isZero(1e-12))
            std::cerr << "[DiscreteLQG] WARNING: plant.D != 0. "
                         "Kalman innovation uses u[k-1] for the D*u term (one step stale). "
                         "For accurate filtering, set D = 0 in the model.\n";

        lqr_ = std::make_unique<DiscreteLQR>(plant, lqr_p);
        kf_ = std::make_unique<KalmanFilter>(plant, Q_noise, R_noise, P0);
        x_ref_ = Eigen::VectorXd::Zero(plant.stateSize());
        u_prev_ = Eigen::VectorXd::Zero(plant.inputSize());
    }

    // Full step: KF predict -> KF update -> LQR control.
    //
    // Causal ordering note: the Kalman update uses u_prev as its u_current argument.
    // KalmanFilter::update() uses u_current only in the D*u feedthrough term of the
    // innovation (y - C*x^ - D*u). Since u[k] has not been computed yet at update
    // time, u[k-1] is the only available approximation. For D=0 plants (the standard
    // case for ZOH-discretised models) the D*u term is zero and this has no effect.
    // The constructor warns when D!=0; avoid D!=0 plants with LQG if accuracy matters.
    Eigen::VectorXd DiscreteLQG::step(const Eigen::VectorXd &y,
                                      const Eigen::VectorXd &u_prev,
                                      const Eigen::VectorXd &x_ref)
    {
        // 1. Predict: x^[k|k-1] = A*x^[k-1|k-1] + B*u[k-1]
        kf_->predict(u_prev);

        // 2. Update: x^[k|k] from y[k]. u_prev is used for the D*u term - see note above.
        kf_->update(y, u_prev);

        // 3. LQR feedback on corrected state estimate
        const Eigen::VectorXd &xhat = kf_->state();
        Eigen::VectorXd ref = x_ref.size() == plant_.stateSize() ? x_ref : x_ref_;

        Eigen::VectorXd u = lqr_->compute(xhat, ref);

        // Store for internal convenience
        u_prev_ = u;
        return u;
    }

    // IController-compatible SISO wrapper.
    // Requires setReference() and setUPrev() to be called first if needed.
    double DiscreteLQG::compute(double y_scalar)
    {
        Eigen::VectorXd y(plant_.outputSize());
        y.fill(y_scalar);
        Eigen::VectorXd u = step(y, u_prev_, x_ref_);
        return u(0);
    }

    void DiscreteLQG::reset()
    {
        kf_->reset();
        u_prev_.setZero();
        x_ref_.setZero();
    }

} // namespace ctrl
