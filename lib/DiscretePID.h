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
// Ref: Astrom & Wittenmark "Computer Controlled Systems" Ch 3;
//      MATLAB pid(), pidtune(), Simulink Discrete PID Controller block.
namespace ctrl
{

    // Tuning parameters.  All gains follow MATLAB's parallel PID form.
    struct PIDParams
    {
        double Kp = 1.0;    // Proportional gain
        double Ki = 0.0;    // Integral gain  (Ki = Kp / Ti)
        double Kd = 0.0;    // Derivative gain (Kd = Kp . Td)
        // Derivative filter coefficient N [rad/s] - higher = less filtering (sharper derivative).
        // The derivative pole is placed at z = 1/(1 + N*Ts) in the backward-Euler discretisation,
        // corresponding to a continuous-time pole at s = -N. This attenuates derivative action
        // above N rad/s. Setting N too large causes the derivative to amplify high-frequency noise:
        //   - Nyquist limit: N < pi / Ts  (e.g., N < 314 rad/s for Ts=0.01s)
        //   - Practical limit: N < pi / (2 * Ts * omega_c) where omega_c is the closed-loop BW
        //     (ensures the derivative pole stays below Nyquist with a 2x margin)
        // The default N=100 is appropriate for many processes; reduce if derivative action is noisy.
        double N = 100.0;
        double uMin = -1e9; // Lower output saturation limit
        double uMax = 1e9;  // Upper output saturation limit
        // Anti-windup back-calculation gain (0 = disabled, 1 = default).
        // Recommended tuning range (Astrom & Wittenmark §3.5): 1/Ti <= Kb <= 1/Td
        //   where Ti = Kp/Ki and Td = Kd/Kp (i.e., Kb in [Ki/Kp, Kp/Kd]).
        //   - Kb = Ki/Kp  (slow reset): gentle recovery; good for dead-time-dominated plants.
        //   - Kb = Kp/Kd  (fast reset): fastest recovery; may cause actuator spike on re-entry.
        //   - Kb = sqrt(Ki*Kp/Kd) is a geometric-mean rule of thumb for balanced reset.
        //   - Kb = 0: disables anti-windup entirely; use only when saturation cannot occur.
        //   - Kb = 1.0 (default): reasonable for many processes; may be slow for high-Ki loops.
        double Kb = 1.0;
        // Two-degree-of-freedom proportional setpoint weight b in [0, 1].
        //   Applied only by computeDoM(y, r): proportional term acts on (b*r - y) while
        //   the integral term acts on (r - y).  This decouples setpoint-tracking bandwidth
        //   from disturbance-rejection bandwidth without changing any gains.
        //
        // Tuning guide (Skogestad 2003 / Astrom & Hagglund 2006 §8.4):
        //   b = 1.0 (default): standard 1DOF — maximum setpoint gain.
        //   b = 0.5:           reduces overshoot ~10-15 % vs b=1 for ZN-tuned loops.
        //   b = 0.0:           setpoint feed-forward entirely through integrator;
        //                      no proportional kick on step — slowest but smoothest.
        //
        // Has no effect on compute(error) because that overload receives only the
        // pre-computed error (r - y) and cannot separate r from y.
        double b_weight = 1.0;
    };

    class DiscretePID : public IController
    {
    public:
        // Construct with tuning params and fixed sample time Ts [s].
        explicit DiscretePID(const PIDParams &params, double sampleTime);

        // Compute u[k] from tracking error e[k] = r[k] - y[k].
        double compute(double error) override;

        // Derivative-on-Measurement variant: avoids derivative kick when the setpoint steps.
        // Computes u[k] with the derivative filter applied to -y (plant output) rather than
        // to the error e = r - y. The proportional and integral terms still use e = r - y.
        //
        // Use this overload instead of compute(error) when:
        //   - The setpoint changes frequently (batch reactors, motion controllers)
        //   - Large setpoint steps cause unacceptable derivative spikes
        //
        // Equivalent MATLAB: DiscretePID block with 'Derivative on measurement' selected.
        double computeDoM(double y, double r);

        void reset() override;
        double sampleTime() const override { return Ts_; }

        // Hot-update tuning parameters at runtime (bumpless if changes are gradual).
        void setParams(const PIDParams &params) { p_ = params; }
        const PIDParams &params() const { return p_; }

        double lastOutput() const { return u_prev_; }

        // Bumpless initialisation: set integral so that compute(error) would return u_target.
        // Called by ControllerStack on supervisory activation to avoid output bumps.
        void bumplessInit(double u_target, double error) override;

    private:
        PIDParams p_;
        double Ts_;
        double integral_; // accumulated integral I[k]
        double deriv_;    // filtered derivative state d[k-1]
        double e_prev_;   // e[k-1]
        double y_prev_;   // y[k-1] - for derivative-on-measurement
        double u_prev_;   // u[k-1] - for anti-windup back-calculation
    };

} // namespace ctrl
