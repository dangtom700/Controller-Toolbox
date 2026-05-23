#pragma once
#include "IController.h"

// Discrete-time PID controller with derivative filter and back-calculation anti-windup.
// Discretisation: backward Euler for integral; backward Euler for derivative filter.
// ISA standard form: u[k] = Kp.e[k] + I[k] + D[k]
//
// Derivative filter (backward Euler, ref MATLAB 'Filter coefficient' N):
//   d[k] = (1/(1+N.Ts)).d[k-1] + (Kd.N/(1+N.Ts)).(e[k] - e[k-1])
//
// Anti-windup - back-calculation:
//   I[k]   = I[k-1] + Ki.Ts.e[k] + Kb.(u_sat[k] - u_unsat[k])
//   u[k]   = Kp.e[k] + I[k] + D[k]   (I[k] includes current e[k])
//
// Ref: Åström & Wittenmark "Computer Controlled Systems" Ch 3;
//      MATLAB pid(), pidtune(), Simulink Discrete PID Controller block.
namespace ctrl
{

    // Tuning parameters.  All gains follow MATLAB's parallel PID form.
    struct PIDParams
    {
        double Kp = 1.0;    // Proportional gain
        double Ki = 0.0;    // Integral gain  (Ki = Kp / Ti)
        double Kd = 0.0;    // Derivative gain (Kd = Kp . Td)
        double N = 100.0;   // Derivative filter coefficient [rad/s] - higher = less filtering
        double uMin = -1e9; // Lower output saturation limit
        double uMax = 1e9;  // Upper output saturation limit
        double Kb = 1.0;    // Anti-windup back-calculation gain (0 = disabled)
    };

    class DiscretePID : public IController
    {
    public:
        // Construct with tuning params and fixed sample time Ts [s].
        explicit DiscretePID(const PIDParams &params, double sampleTime);

        // Compute u[k] from tracking error e[k] = r[k] - y[k].
        double compute(double error) override;

        void reset() override;
        double sampleTime() const override { return Ts_; }

        // Hot-update tuning parameters at runtime (bumpless if changes are gradual).
        void setParams(const PIDParams &params) { p_ = params; }
        const PIDParams &params() const { return p_; }

        double lastOutput() const { return u_prev_; }

    private:
        PIDParams p_;
        double Ts_;
        double integral_; // accumulated integral I[k]
        double deriv_;    // filtered derivative state d[k-1]
        double e_prev_;   // e[k-1]
        double u_prev_;   // u[k-1] - for anti-windup back-calculation
    };

} // namespace ctrl
