#include "SmithPredictor.h"
#include "FunctionApproximator.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

// -- Helpers ------------------------------------------------------------------

// Build a trivial 1-state unity-gain filter (no fractional delay).
static StateSpace makeIdentityFilter(double Ts)
{
    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A(0,0)=0.0; B(0,0)=0.0; C(0,0)=0.0; D(0,0)=1.0;
    return StateSpace(A, B, C, D, Ts);
}

void SmithPredictor::initBuffers()
{
    x_model_ = Eigen::VectorXd::Zero(model_.stateSize());
    u_prev_  = Eigen::VectorXd::Zero(model_.inputSize());
    x_frac_  = Eigen::VectorXd::Zero(frac_filter_.stateSize());
    y_buf_.assign(d_ > 0 ? d_ : 0, 0.0);
    buf_head_ = 0;
}

// -- Constructors -------------------------------------------------------------

SmithPredictor::SmithPredictor(std::shared_ptr<IController> inner,
                               const StateSpace &delayModel,
                               int delaySteps)
    : inner_(std::move(inner)), model_(delayModel), d_(delaySteps), Ts_(delayModel.Ts),
      has_frac_(false), frac_filter_(makeIdentityFilter(delayModel.Ts))
{
    initBuffers();
}

SmithPredictor::SmithPredictor(std::shared_ptr<IController> inner,
                               const StateSpace &delayModel,
                               int delaySteps,
                               const StateSpace &fracDelayFilter)
    : inner_(std::move(inner)), model_(delayModel), d_(delaySteps), Ts_(delayModel.Ts),
      has_frac_(true), frac_filter_(fracDelayFilter)
{
    if (std::abs(fracDelayFilter.Ts - delayModel.Ts) > 1e-12)
        throw std::invalid_argument(
            "SmithPredictor: fracDelayFilter.Ts must match delayModel.Ts.");
    initBuffers();
}

SmithPredictor::SmithPredictor(std::shared_ptr<IController> inner,
                               const StateSpace &delayModel,
                               double theta,
                               double Ts)
    : SmithPredictor(
          std::move(inner),
          delayModel,
          [&]() -> int {
              if (theta < 0.0)
                  throw std::invalid_argument("SmithPredictor: theta must be >= 0.");
              if (std::abs(Ts - delayModel.Ts) > 1e-12)
                  throw std::invalid_argument(
                      "SmithPredictor: Ts must match delayModel.Ts.");
              return static_cast<int>(std::floor(theta / Ts));
          }(),
          [&]() -> StateSpace {
              const int d = static_cast<int>(std::floor(theta / Ts));
              const double tf = theta - d * Ts;
              return tf > 1e-12 ? padeDelayFilter(tf, Ts) : makeIdentityFilter(Ts);
          }())
{}

// -- compute ------------------------------------------------------------------

// Signal flow:
//   1. Compute delay-free model output y_model[k] = C*x_model + D*u[k-1]
//   2. If a Pade fractional filter is active, pass y_model through H_frac:
//        y_now = H_frac(y_model)    (adds the sub-sample delay to the model)
//   3. Buffer y_now; read d_-step-old y_now as y_delayed.
//   4. Modified error: e_sp = error + (y_now - y_delayed)
//   5. Compute u = inner->compute(e_sp)
//   6. Advance model and filter states.

double SmithPredictor::compute(double error)
{
    // Step 1: delay-free model output (uses u[k-1] for D term)
    const double y_model = (model_.C * x_model_ + model_.D * u_prev_)(0);

    // Step 2: fractional filter (H_frac)
    double y_now;
    if (has_frac_)
    {
        Eigen::VectorXd ym_vec(1); ym_vec(0) = y_model;
        y_now = (frac_filter_.C * x_frac_ + frac_filter_.D * ym_vec)(0);
    }
    else
    {
        y_now = y_model;
    }

    // Step 3: d-step delayed buffer
    double y_delayed = y_now;
    if (d_ > 0)
    {
        y_delayed          = y_buf_[buf_head_];
        y_buf_[buf_head_]  = y_now;
        buf_head_          = (buf_head_ + 1) % d_;
    }

    // Step 4: modified Smith error
    const double e_sp = error + (y_now - y_delayed);

    // Step 5: inner controller
    const double u = inner_->compute(e_sp);

    // Step 6: advance states
    Eigen::VectorXd uv(model_.inputSize()); uv.fill(u);
    x_model_ = model_.A * x_model_ + model_.B * uv;
    u_prev_  = uv;

    if (has_frac_)
    {
        Eigen::VectorXd ym_vec(1); ym_vec(0) = y_model;
        x_frac_ = frac_filter_.A * x_frac_ + frac_filter_.B * ym_vec;
    }

    return u;
}

// -- reset --------------------------------------------------------------------

void SmithPredictor::reset()
{
    inner_->reset();
    x_model_.setZero();
    u_prev_.setZero();
    x_frac_.setZero();
    if (d_ > 0)
    {
        std::fill(y_buf_.begin(), y_buf_.end(), 0.0);
        buf_head_ = 0;
    }
}

// -- setModel -----------------------------------------------------------------

void SmithPredictor::setModel(const StateSpace &delayModel, int delaySteps)
{
    model_       = delayModel;
    d_           = delaySteps;
    has_frac_    = false;
    frac_filter_ = makeIdentityFilter(delayModel.Ts);
    Ts_          = delayModel.Ts;
    initBuffers();
}

void SmithPredictor::setModel(const StateSpace &delayModel, int delaySteps,
                               const StateSpace &fracDelayFilter)
{
    if (std::abs(fracDelayFilter.Ts - delayModel.Ts) > 1e-12)
        throw std::invalid_argument(
            "SmithPredictor::setModel: fracDelayFilter.Ts must match delayModel.Ts.");
    model_       = delayModel;
    d_           = delaySteps;
    has_frac_    = true;
    frac_filter_ = fracDelayFilter;
    Ts_          = delayModel.Ts;
    initBuffers();
}

} // namespace ctrl
