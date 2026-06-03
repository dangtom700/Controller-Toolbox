#pragma once
#include "IControllerObserver.h"
#include <Eigen/Dense>
#include <memory>
#include <stdexcept>

namespace ctrl
{

/**
 * @brief Abstract interface for all discrete-time controllers.
 *
 * All implementations operate at a fixed sample time Ts and are called once per step.
 * The interface covers both SISO (compute()) and MIMO (computeVec()) controllers,
 * bumpless initialisation for supervisory switching, and a runtime health query
 * used by ControllerStack to implement automatic fallback chains.
 *
 * @see MATLAB Control System Toolbox discrete controller pattern.
 */
class IController
{
public:
    virtual ~IController() = default;

    /**
     * @brief Advance one sample step - scalar SISO interface.
     *
     * - Tracking controllers (PID, MPC, LQR-adapter): @p signal = e[k] = r[k] - y[k]
     * - Optimisation-based controllers (ESC): @p signal = plant output y[k] (cost to extremize)
     *
     * @param signal Tracking error e[k] or plant output y[k], depending on controller type.
     * @return Control action u[k].
     */
    virtual double compute(double signal) = 0;

    /**
     * @brief Advance one sample step - vector MIMO interface.
     *
     * The default implementation delegates to compute() when @p signal has exactly one element,
     * and throws std::logic_error for multi-element signals to prevent silent truncation.
     * Override this method in MIMO controllers (DiscreteMPC, DiscreteLQR, etc.).
     *
     * @param signal Reference or error vector (n x 1).
     * @return Control action vector u[k] (m x 1).
     * @throws std::logic_error If a multi-element signal is passed to a SISO controller.
     */
    virtual Eigen::VectorXd computeVec(const Eigen::VectorXd &signal)
    {
        if (signal.size() == 1)
            return Eigen::VectorXd::Constant(1, compute(signal(0)));
        throw std::logic_error(
            "IController::computeVec: MIMO signal passed to a SISO controller. "
            "Override computeVec() in MIMO subclasses.");
    }

    /**
     * @brief Reset all internal states (integrators, delay buffers, estimators).
     */
    virtual void reset() = 0;

    /**
     * @brief Sample time in seconds.
     * @return Ts [s].
     */
    virtual double sampleTime() const = 0;

    /**
     * @brief Bumpless initialisation - prepares the controller for smooth activation.
     *
     * Called by ControllerStack (Supervisory mode) when this controller is newly selected,
     * so its first compute() call returns @p u_target at the current @p error without a bump.
     *
     * The default is a no-op; override in controllers that have integral or memory state
     * (DiscretePID, DiscreteMPC, DiscreteSMC).
     *
     * @param u_target The composite output the loop is currently delivering.
     * @param error    The current tracking error that will be passed to compute().
     *
     * @note MIMO limitation: this interface is scalar. For MIMO controllers that need
     *       multi-channel bumpless transfer, implement it outside IController (e.g., call
     *       DiscreteMPC::setState() / setLastApplied() directly before handing over).
     */
    virtual void bumplessInit(double /*u_target*/, double /*error*/) {}

    /**
     * @brief Health query for supervisory fault-tolerant control.
     *
     * Returns @c false when the controller's last compute() call produced a result known to be
     * suboptimal or potentially unsafe:
     * - DiscreteLQR: DARE did not converge at construction.
     * - DiscreteMPC: QP solver hit qpMaxIter (last step did not converge).
     * - GeneralizedPredictiveControl: same as MPC.
     *
     * ControllerStack (Supervisory mode) checks isHealthy() before selecting an entry;
     * an unhealthy controller is skipped in favour of the next eligible entry, enabling
     * automatic fallback: MPC -> GPC -> PID when the QP consistently fails.
     *
     * The default returns @c true. Override only in controllers with meaningful runtime
     * health state.
     *
     * @return @c true if the last computation was nominal; @c false if suboptimal.
     */
    virtual bool isHealthy() const { return true; }

    /** @brief Human-readable controller identifier. Override in each subclass. */
    virtual std::string name() const { return ""; }

    // -- Observer (telemetry) ------------------------------------------------

    /**
     * @brief Attach a telemetry observer.
     *
     * The observer receives onCompute() / onComputeVec() / onReset() callbacks
     * after the corresponding methods fire. Only one observer is supported at a
     * time; attaching a second replaces the first.
     *
     * The observer is a non-owning raw pointer and must outlive the controller.
     * Call detachObserver() before the observer is destroyed.
     *
     * @param obs Pointer to the observer; may be @c nullptr to detach.
     */
    void attachObserver(IControllerObserver *obs) { observer_owned_.reset(); observer_ = obs; }

    /**
     * @brief Lifetime-safe overload - controller co-owns the observer via shared_ptr.
     *
     * Prevents dangling-pointer UB when the observer is held only from Python (where
     * the C++ raw pointer has no reference from the caller's side).
     * Replaces any previously attached raw-pointer observer.
     */
    void attachObserver(std::shared_ptr<IControllerObserver> obs)
    {
        observer_owned_ = obs;
        observer_       = obs.get();
    }

    /** @brief Detach the current observer (if any). */
    void detachObserver() { observer_ = nullptr; observer_owned_.reset(); }

    /** @brief @c true if an observer is currently attached. */
    bool hasObserver() const { return observer_ != nullptr; }

protected:
    /**
     * @brief Fire the observer's onCompute() callback (if attached).
     *
     * Call this at the end of every compute() override to deliver telemetry.
     * ControllerStack calls it automatically for controllers dispatched through the stack;
     * controllers used directly should call it themselves.
     *
     * @param u      Computed control action.
     * @param signal Signal that was passed to compute().
     */
    void notifyObserver(double u, double signal) const
    {
        if (observer_) observer_->onCompute(u, signal);
    }

    /**
     * @brief Fire the observer's onComputeVec() callback (if attached).
     * @param u      Computed control vector.
     * @param signal Signal vector that was passed to computeVec().
     */
    void notifyObserverVec(const Eigen::VectorXd &u, const Eigen::VectorXd &signal) const
    {
        if (observer_) observer_->onComputeVec(u, signal);
    }

    /**
     * @brief Fire the observer's onReset() callback (if attached).
     *
     * Call this at the end of every reset() override.
     */
    void notifyObserverReset() const
    {
        if (observer_) observer_->onReset();
    }

private:
    IControllerObserver *observer_ = nullptr;
    std::shared_ptr<IControllerObserver> observer_owned_; ///< Keeps shared_ptr observers alive.
};

} // namespace ctrl
