#pragma once

// Abstract interface for all discrete-time controllers.
// All implementations operate at a fixed sample time Ts and are called once per step.
// Ref: MATLAB Control System Toolbox discrete controller pattern.
namespace ctrl
{

    class IController
    {
    public:
        virtual ~IController() = default;

        // Advance one sample step.
        // For tracking controllers (PID, MPC, LQR-adapter): signal = e[k] = r[k] - y[k]
        // For optimisation-based controllers (ESC): signal = plant output y[k] (cost to extremize)
        virtual double compute(double signal) = 0;

        // Reset all internal states (integrators, delay buffers, estimators).
        virtual void reset() = 0;

        // Sample time in seconds.
        virtual double sampleTime() const = 0;
    };

} // namespace ctrl
