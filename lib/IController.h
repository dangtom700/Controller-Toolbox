#pragma once
#include <Eigen/Dense>
#include <stdexcept>

// Abstract interface for all discrete-time controllers.
// All implementations operate at a fixed sample time Ts and are called once per step.
// Ref: MATLAB Control System Toolbox discrete controller pattern.
namespace ctrl
{

    class IController
    {
    public:
        virtual ~IController() = default;

        // Advance one sample step (scalar SISO interface).
        // For tracking controllers (PID, MPC, LQR-adapter): signal = e[k] = r[k] - y[k]
        // For optimisation-based controllers (ESC): signal = plant output y[k] (cost to extremize)
        virtual double compute(double signal) = 0;

        // MIMO interface - override in MIMO controllers (DiscreteMPC, DiscreteLQR, etc.).
        // The default throws std::logic_error rather than silently truncating a multi-element
        // signal to its first element, which would produce wrong results without any warning.
        // SISO controllers that are never called through this interface are unaffected.
        virtual Eigen::VectorXd computeVec(const Eigen::VectorXd &signal)
        {
            if (signal.size() == 1)
                return Eigen::VectorXd::Constant(1, compute(signal(0)));
            throw std::logic_error(
                "IController::computeVec: MIMO signal passed to a SISO controller. "
                "Override computeVec() in MIMO subclasses.");
        }

        // Reset all internal states (integrators, delay buffers, estimators).
        virtual void reset() = 0;

        // Sample time in seconds.
        virtual double sampleTime() const = 0;

        // Bumpless initialisation - called by ControllerStack (Supervisory mode) when
        // this controller is newly selected, so it produces u_target at the current error
        // without an output bump.
        //
        // u_target: the output the composite loop is currently delivering.
        // error:    the current tracking error passed to compute().
        //
        // The default is a no-op (e.g., for stateless or non-integrating controllers).
        // Override in controllers that have integral or memory state (PID, MPC, SMC).
        //
        // MIMO limitation: this interface is scalar.  For MIMO controllers (e.g.,
        // DiscreteMPC operating on a vector plant), the bumpless handover is a vector
        // operation (initialise each input channel to match the corresponding u_target
        // component).  ControllerStack currently calls bumplessInit with a scalar
        // (the single composite output from the previous step), which is only meaningful
        // for the first output channel.  MIMO controllers that need multi-channel
        // bumpless transfer must implement it outside the IController interface, e.g.,
        // by calling DiscreteMPC::setState() and DiscreteMPC::setLastApplied() directly
        // from the switching logic before handing control to the new controller.
        virtual void bumplessInit(double /*u_target*/, double /*error*/) {}

        // Health query for supervisory fault-tolerant control.
        //
        // Returns false when the controller's last compute() call produced a result
        // that is known to be suboptimal or potentially unsafe:
        //   - DiscreteLQR:   DARE did not converge at construction.
        //   - DiscreteMPC:   QP solver exited at qpMaxIter (last step did not converge).
        //   - GeneralizedPredictiveController: same as MPC.
        //
        // ControllerStack (Supervisory mode) checks isHealthy() before selecting an entry:
        // an unhealthy controller is skipped in favour of the next eligible entry.  This
        // enables automatic fallback from MPC -> PID when the QP consistently fails.
        //
        // The default returns true (healthy) - override only in controllers that have
        // meaningful runtime health state.
        virtual bool isHealthy() const { return true; }
    };

} // namespace ctrl
