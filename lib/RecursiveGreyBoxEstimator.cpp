#include "RecursiveGreyBoxEstimator.h"
#include <stdexcept>

namespace ctrl {

RecursiveGreyBoxEstimator::RecursiveGreyBoxEstimator(
    OdeFn ode, MeasFn meas,
    int n_state, int n_meas,
    const Params& par)
    : f_(std::move(ode)), h_(std::move(meas)),
      par_(par), n_state_(n_state), n_meas_(n_meas),
      n_param_(static_cast<int>(par.p0.size()))
{
    if (n_param_ == 0)
        throw std::invalid_argument("RecursiveGreyBoxEstimator: p0 must be non-empty");
    if (par_.Q_state.rows() != n_state_ || par_.Q_param.rows() != n_param_)
        throw std::invalid_argument(
            "RecursiveGreyBoxEstimator: Q_state / Q_param size mismatch with n_state / p0");
}

void RecursiveGreyBoxEstimator::initialize(const Eigen::VectorXd& x0,
                                            const Eigen::MatrixXd& P0_state)
{
    const int n_aug = n_state_ + n_param_;

    // ---- Build augmented noise matrices ------------------------------------
    Eigen::MatrixXd Q_aug = Eigen::MatrixXd::Zero(n_aug, n_aug);
    Q_aug.topLeftCorner(n_state_, n_state_)       = par_.Q_state;
    Q_aug.bottomRightCorner(n_param_, n_param_)   = par_.Q_param;

    Eigen::MatrixXd P0_p = (par_.P0_param.rows() == n_param_)
                           ? par_.P0_param
                           : (100.0 * par_.Q_param);
    Eigen::MatrixXd P0_aug = Eigen::MatrixXd::Zero(n_aug, n_aug);
    P0_aug.topLeftCorner(n_state_, n_state_)     = P0_state;
    P0_aug.bottomRightCorner(n_param_, n_param_) = P0_p;

    // ---- Augmented process function ----------------------------------------
    // z = [x; p].  Dynamics: integrate f(x,u,p) via RK4; p stays constant.
    const int n_x   = n_state_;
    const int n_p   = n_param_;
    const double Ts = par_.Ts;
    const int rk4s  = par_.rk4_steps;
    OdeFn f_copy    = f_;

    // Pre-allocate RK4 scratch inside the closure to avoid per-sigma-point heap alloc
    auto f_aug = [f_copy, n_x, n_p, Ts, rk4s,
                  x_scr   = Eigen::VectorXd(n_x),
                  p_scr   = Eigen::VectorXd(n_p),
                  k1      = Eigen::VectorXd(n_x),
                  k2      = Eigen::VectorXd(n_x),
                  k3      = Eigen::VectorXd(n_x),
                  k4      = Eigen::VectorXd(n_x),
                  z_next  = Eigen::VectorXd(n_x + n_p)](
        const Eigen::VectorXd& z,
        const Eigen::VectorXd& u) mutable -> Eigen::VectorXd
    {
        x_scr = z.head(n_x);
        p_scr = z.tail(n_p);
        const double h = Ts / rk4s;
        for (int s = 0; s < rk4s; ++s) {
            k1 = f_copy(x_scr,             u, p_scr);
            k2 = f_copy(x_scr + 0.5*h*k1, u, p_scr);
            k3 = f_copy(x_scr + 0.5*h*k2, u, p_scr);
            k4 = f_copy(x_scr + h*k3,     u, p_scr);
            x_scr += (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
        }
        z_next.head(n_x) = x_scr;
        z_next.tail(n_p) = p_scr;
        return z_next;
    };

    // ---- Augmented measurement function ------------------------------------
    MeasFn h_copy = h_;
    auto h_aug = [h_copy, n_x, n_p](
        const Eigen::VectorXd& z,
        const Eigen::VectorXd& /*u*/) -> Eigen::VectorXd
    {
        return h_copy(z.head(n_x), z.tail(n_p));
    };

    // ---- Build UKF ---------------------------------------------------------
    ukf_ = std::make_unique<UnscentedKalmanFilter>(
        n_aug, n_meas_,
        std::move(f_aug), std::move(h_aug),
        Q_aug, par_.R_meas,
        Ts, P0_aug,
        par_.alpha, par_.beta, par_.kappa);

    // Set initial augmented state [x0; p0]
    Eigen::VectorXd z0(n_aug);
    z0.head(n_state_) = x0;
    z0.tail(n_param_) = par_.p0;
    ukf_->setState(z0);

    initialized_ = true;
}

Eigen::VectorXd RecursiveGreyBoxEstimator::step(const Eigen::VectorXd& y,
                                                  const Eigen::VectorXd& u)
{
    if (!initialized_)
        throw std::logic_error("RecursiveGreyBoxEstimator: call initialize() first");
    ukf_->step(y, u);
    return ukf_->state().head(n_state_);
}

Eigen::VectorXd RecursiveGreyBoxEstimator::stateEstimate() const
{
    if (!initialized_) return Eigen::VectorXd::Zero(n_state_);
    return ukf_->state().head(n_state_);
}

Eigen::VectorXd RecursiveGreyBoxEstimator::paramEstimate() const
{
    if (!initialized_) return par_.p0;
    return ukf_->state().tail(n_param_);
}

const Eigen::MatrixXd& RecursiveGreyBoxEstimator::covariance() const
{
    if (!initialized_)
        throw std::logic_error("RecursiveGreyBoxEstimator: not initialized");
    return ukf_->covariance();
}

} // namespace ctrl
