#pragma once
#include <Eigen/Dense>

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

        // MIMO interface: default wraps compute(signal(0)) and returns a 1-vector.
        // Override in MIMO controllers (DiscreteMPC, DiscreteLQR, etc.) for full vectorisation.
        virtual Eigen::VectorXd computeVec(const Eigen::VectorXd &signal)
        {
            return Eigen::VectorXd::Constant(1, compute(signal(0)));
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
        virtual void bumplessInit(double /*u_target*/, double /*error*/) {}
    };

} // namespace ctrl
