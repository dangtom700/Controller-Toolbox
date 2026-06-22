#pragma once
#include "IControllerObserver.h"
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string_view>

namespace ctrl
{

/**
 * @brief Abstract interface for all discrete-time controllers.
 *
 * **NaN-guard fleet contract (hold-last):** Every compute() override that receives a
 * non-finite input *must* return the last finite output (u_prev_) and leave all internal
 * states (integrators, ESO, estimators) unchanged.  Returning 0.0 or an unguarded NaN on
 * bad sensor data is a safety violation - a sudden zero can be more dangerous than holding
 * the previous command.  The one exception is MRAC/L1Adaptive which take y_plant directly;
 * they also hold the last output on a non-finite measurement.
 *
 * All implementations operate at a fixed sample time Ts and are called once per step.
 * The interface covers both SISO (compute()) and MIMO (computeVec()) controllers,
 * bumpless initialisation for supervisory switching, and a runtime health query
 * used by ControllerStack to implement automatic fallback chains.
 *
 * @see MATLAB Control System Toolbox discrete controller pattern.
 */
/**
 * @brief compute()'s `signal` parameter convention - see CONTRIBUTING.md#sign-conventions.
 *
 * lib/ deliberately does not enforce one convention across controllers (a tracking
 * error needs a sign convention; a cost-extremising controller doesn't have one at all).
 * This lets a caller query the convention at runtime instead of only finding out from
 * documentation. Additive, optional: the default is Unspecified, not a guess - override
 * it only once a class's convention has actually been audited.
 */
enum class SignConvention
{
    Unspecified,          ///< Not yet audited via this API - check CONTRIBUTING.md/class docs.
    TrackingErrorRMinusY, ///< signal = e = r - y (most common: PID, MPC, SmithPredictor, ...).
    TrackingErrorYMinusR, ///< signal = e = y - r (reversed - e.g. DiscreteSMC).
    PlantOutput,          ///< signal = raw plant output y, not an error (e.g. MRACController).
    CostSignal,           ///< signal = cost/objective value to extremize, not an error (ExtremumSeeker).
    Other                 ///< Doesn't fit the above (e.g. signal ignored, reference set out-of-band)
                           ///< - see the class's own header docs.
};

class IController
{
public:
    virtual ~IController() = default;

    /**
     * @brief Query this controller's compute()/computeVec() signal convention.
     * @return Unspecified by default; overridden in classes audited per
     *         CONTRIBUTING.md#sign-conventions.
     */
    virtual SignConvention signConvention() const { return SignConvention::Unspecified; }

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

    /**
     * @brief Query whether the controller has built-in anti-windup.
     *
     * Returns @c true when the controller manages its own integrator saturation
     * internally (e.g., DiscretePID with Kb != 0).  AntiWindupWrapper checks this
     * at construction and refuses to wrap a controller that already has anti-windup,
     * preventing double-correction which would destabilise the loop.
     *
     * The default returns @c false. Override only in controllers with internal AW.
     *
     * @return @c true if the controller handles anti-windup internally.
     */
    virtual bool hasInternalAntiWindup() const { return false; }

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

    /**
     * @brief Fire the observer's onState() callback (M3 internal-state telemetry).
     *
     * Call from compute() to emit richer diagnostics (ESO states, sliding surface, etc.).
     * Zero cost when no observer is attached.
     *
     * @param key    Short identifier (e.g. "eso", "surface", "qp_iters").
     * @param value  Quantity as VectorXd (use Eigen::VectorXd::Constant(1, x) for scalars).
     */
    void notifyObserverState(std::string_view key, const Eigen::VectorXd& value) const
    {
        if (observer_) observer_->onState(key, value);
    }

private:
    IControllerObserver *observer_ = nullptr;
    std::shared_ptr<IControllerObserver> observer_owned_; ///< Keeps shared_ptr observers alive.
};

} // namespace ctrl
