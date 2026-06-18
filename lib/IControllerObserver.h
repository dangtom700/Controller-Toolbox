#pragma once
#include <Eigen/Dense>
#include <string_view>

/**
 * @file IControllerObserver.h
 * @brief Observer hook for non-intrusive controller telemetry (Observer pattern).
 *
 * Attach an IControllerObserver to any IController-derived object via
 * IController::attachObserver(). The observer is notified on every compute() call with
 * the controller's input signal and the resulting control action.
 *
 * **Usage - logging every output of a PID:**
 * @code
 *   struct PIDLogger : ctrl::IControllerObserver {
 *       void onCompute(double u, double signal) override {
 *           std::printf("u=%.4f  e=%.4f\n", u, signal);
 *       }
 *   };
 *
 *   ctrl::DiscretePID pid(params, Ts);
 *   PIDLogger logger;
 *   pid.attachObserver(&logger);
 *   // Every pid.compute(e) call now also fires logger.onCompute(u, e).
 * @endcode
 *
 * **ControllerStack integration:**
 * ControllerStack calls notifyObserver() after every dispatched compute() so observers
 * registered on the stack itself receive the composite output automatically.
 *
 * @note Observers are non-owning raw pointers. The observer must outlive the controller.
 *       Use detachObserver() before the observer is destroyed to prevent dangling calls.
 *
 * @see Gang of Four, "Design Patterns", Observer (1994).
 */

namespace ctrl
{

/**
 * @brief Interface for controller telemetry observers.
 */
class IControllerObserver
{
public:
    virtual ~IControllerObserver() = default;

    /**
     * @brief Called after every SISO compute() invocation.
     *
     * @param u      Control action produced by the controller.
     * @param signal Input signal passed to compute() (tracking error or plant output).
     */
    virtual void onCompute(double /*u*/, double /*signal*/) {}

    /**
     * @brief Called after every MIMO computeVec() invocation.
     *
     * The default implementation delegates to onCompute() for single-input signals,
     * enabling SISO observers to work transparently with MIMO controllers.
     *
     * @param u      Control vector produced by the controller (m * 1).
     * @param signal Input vector passed to computeVec() (n * 1).
     */
    virtual void onComputeVec(const Eigen::VectorXd &u, const Eigen::VectorXd &signal)
    {
        if (u.size() == 1 && signal.size() == 1)
            onCompute(u(0), signal(0));
    }

    /**
     * @brief Called when the controller's reset() is invoked.
     *
     * Useful for synchronising external state (e.g., flushing a log buffer or
     * resetting a running average) whenever the controller is reinitialised.
     */
    virtual void onReset() {}

    /**
     * @brief Called by controllers that expose internal state for diagnostics (M3).
     *
     * Fired by `notifyObserverState()` after compute() for controllers that
     * emit richer telemetry:
     *   - DiscreteADRC: key="eso",  value=[z1,z2,z3] (ESO state)
     *   - DiscreteSMC:  key="surface", value=[s]      (sliding surface)
     *   - DynaController: key="model_fitted", value=[1 or 0]
     *
     * The default is a no-op so existing observers are unaffected.
     *
     * @param key    Short identifier for the emitted quantity (e.g. "eso").
     * @param value  Quantity as a column vector (may be 1-D for scalars).
     */
    virtual void onState(std::string_view /*key*/,
                         const Eigen::VectorXd& /*value*/) {}
};

} // namespace ctrl
