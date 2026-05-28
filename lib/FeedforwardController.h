#pragma once
#include "IController.h"
#include "PlantModel.h"

/**
 * @file FeedforwardController.h
 * @brief Discrete-time feedforward controller driven by the reference signal.
 *
 * Applies a pre-designed StateSpace filter G_ff(z) to the **reference** r[k]
 * (not the error), producing a feedforward command:
 * @code
 *   u_ff[k] = G_ff(z) * r[k]
 * @endcode
 *
 * Combine with a feedback controller via ControllerStack::Additive to form a
 * two-degree-of-freedom (2DOF) structure:
 * @code
 *   ctrl::ControllerStack stack(ctrl::StackMode::Additive, Ts);
 *   stack.addController(std::make_shared<ctrl::FeedforwardController>(Gff), "FF");
 *   stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts),        "PID");
 *   // stack.compute(r) returns u_ff + u_pid (with r serving as both signal and error)
 * @endcode
 *
 * **Design guide (pdc-master reference - feedforward_widget.ipynb):**
 *
 * For a SISO plant G(z) with disturbance model Gd(z), the perfect
 * feedforward cancels disturbances exactly:
 * @code
 *   G_ff_ideal(z) = -Gd(z) / G(z)    (may be improper; need to add a filter)
 * @endcode
 *
 * For setpoint tracking, the lead-filter approximation:
 * @code
 *   G_ff(z) = (tau_p * z - 1) / (tau_ff * z - 1)   (discrete lead)
 * @endcode
 * where tau_p is the plant time constant and tau_ff is the desired closed-loop
 * time constant, is a practical 1st-order approximant (see Cohen & Coon, 1953).
 *
 * **Usage:**
 * @code
 *   // Inverse-plant feedforward for G(s)=1/(s+1) at Ts=0.1
 *   ctrl::StateSpace Gff = ctrl::c2d(G_model_inv_filtered, 0.1, ctrl::C2dMethod::ZOH);
 *   ctrl::FeedforwardController ff(Gff);
 *   double u_ff = ff.compute(ref);   // pass reference, not error
 * @endcode
 *
 * @note The compute() argument is the **reference signal r**, not the tracking
 * error. When used inside ControllerStack::Additive the stack passes its input
 * unchanged to each layer, so wire the reference (not error) into the stack.
 *
 * @see pdc-master/feedforward_widget.ipynb
 * @see ControllerStack - for combining FF with a feedback controller.
 * @see PlantModel.h    - for building discrete-time StateSpace filters.
 */

namespace ctrl {

/**
 * @brief Feedforward controller: u_ff[k] = G_ff(z) * r[k].
 */
class FeedforwardController : public IController
{
public:
    /**
     * @brief Construct from a pre-designed discrete-time filter.
     *
     * @param ff_model Discrete-time StateSpace G_ff(z) (single-input, single-output
     *                 or MIMO if the plant is MIMO).
     *
     * @note G_ff must have Ts > 0 (already discretised). Use c2d() to convert
     *       a continuous-time inverse-plant model before passing it here.
     */
    explicit FeedforwardController(const StateSpace &ff_model)
        : model_(ff_model),
          x_(Eigen::VectorXd::Zero(ff_model.stateSize())),
          Ts_(ff_model.Ts)
    {}

    /**
     * @brief Compute u_ff[k] = G_ff(z) * r[k].
     *
     * @param r Reference signal r[k] (not tracking error).
     * @return  Feedforward command u_ff[k].
     */
    double compute(double r) override
    {
        Eigen::VectorXd rv(model_.inputSize());
        rv.fill(r);
        const double u_ff = (model_.C * x_ + model_.D * rv)(0);
        x_ = model_.A * x_ + model_.B * rv;
        notifyObserver(u_ff, r);   // (u, signal) convention: u_ff is output, r is input
        return u_ff;
    }

    /** @brief Reset internal filter state. */
    void reset() override
    {
        x_.setZero();
        notifyObserverReset();
    }

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief MIMO feedforward: u_ff[k] = G_ff(z) * r[k].
     * @param r Reference vector (n_inputs * 1).
     * @return  Feedforward command vector (n_outputs * 1).
     */
    Eigen::VectorXd computeVec(const Eigen::VectorXd &r) override
    {
        const Eigen::VectorXd u_ff = model_.C * x_ + model_.D * r;
        x_ = model_.A * x_ + model_.B * r;
        return u_ff;
    }

    /** @brief Read-only access to the feedforward filter model. */
    const StateSpace &model() const { return model_; }

    /** @brief Current filter state x [n * 1]. */
    const Eigen::VectorXd &state() const { return x_; }

private:
    StateSpace       model_;
    Eigen::VectorXd  x_;
    double           Ts_;
};

} // namespace ctrl
