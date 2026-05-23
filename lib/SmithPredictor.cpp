#include "SmithPredictor.h"
#include <algorithm>

namespace ctrl
{

    SmithPredictor::SmithPredictor(std::shared_ptr<IController> inner,
                                   const StateSpace &delayModel,
                                   int delaySteps)
        : inner_(std::move(inner)), model_(delayModel), d_(delaySteps), Ts_(delayModel.Ts),
          buf_head_(0)
    {
        x_model_ = Eigen::VectorXd::Zero(model_.stateSize());
        u_prev_  = Eigen::VectorXd::Zero(model_.inputSize());
        // Pre-allocate fixed buffer at construction - no heap allocation in compute()
        if (d_ > 0) y_buf_.assign(d_, 0.0);
    }

    // Modified error:  e_sp = (r-y) + (ŷ_now - ŷ_delayed)
    //                        = error + ŷ_now - ŷ_now[k-d]
    //
    // The correction term removes the delay from the error signal presented to
    // the inner controller, giving it an effectively delay-free closed loop.
    double SmithPredictor::compute(double error)
    {
        // Current model output ŷ[k] from the delay-free plant model (uses u[k-1] for D term)
        const double y_now = (model_.C * x_model_ + model_.D * u_prev_)(0);

        // d-step delayed model output - read oldest slot from the circular buffer
        double y_delayed = y_now; // no delay when d_ == 0
        if (d_ > 0)
        {
            y_delayed = y_buf_[buf_head_]; // oldest entry
            y_buf_[buf_head_] = y_now;     // overwrite with current (no heap alloc)
            buf_head_ = (buf_head_ + 1) % d_;
        }

        // Modified error: routes inner controller as if there were no dead-time
        const double e_sp = error + (y_now - y_delayed);

        // Compute control action via the inner controller
        const double u = inner_->compute(e_sp);

        // Advance internal model state: x^[k+1] = A.x^[k] + B.u[k]
        Eigen::VectorXd uv(model_.inputSize());
        uv.fill(u);
        x_model_ = model_.A * x_model_ + model_.B * uv;
        u_prev_ = uv;

        return u;
    }

    void SmithPredictor::reset()
    {
        inner_->reset();
        x_model_.setZero();
        u_prev_.setZero();
        if (d_ > 0)
        {
            std::fill(y_buf_.begin(), y_buf_.end(), 0.0);
            buf_head_ = 0;
        }
    }

} // namespace ctrl
