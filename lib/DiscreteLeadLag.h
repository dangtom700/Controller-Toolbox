#pragma once
#include "IController.h"

// Discrete Lead-Lag / Lead / Lag Compensator.
//
// Continuous form: C(s) = K . (s + z_c) / (s + p_c)
//   Lead:  p_c > z_c > 0  -> adds positive phase, improves phase margin / speed
//   Lag:   0 < p_c < z_c  -> adds gain at low freq, improves steady-state accuracy
//
// Discretised via Tustin (bilinear) substitution s = 2(z-1)/(Ts(z+1)):
//   C(z) = b0 + b1.z^-^1
//          ------------   (first-order IIR)
//          1  + a1.z^-^1
//
//   b0 = K.(2/Ts + z_c) / (2/Ts + p_c)
//   b1 = K.(z_c - 2/Ts) / (2/Ts + p_c)
//   a1 = (p_c - 2/Ts)   / (2/Ts + p_c)
//
// Difference equation: y[k] = b0.u[k] + b1.u[k-1] - a1.y[k-1]
//
// Typical use: placed in series with a PID or as a standalone compensator
// in the forward or feedback path.
//
// Ref: Franklin, Powell & Emami-Naeini "Feedback Control of Dynamic Systems";
//      MATLAB c2d() with 'tustin' method.
namespace ctrl
{

    struct LeadLagParams
    {
        double continuousZero = 1.0;  // z_c [rad/s] - zero of C(s)
        double continuousPole = 10.0; // p_c [rad/s] - pole of C(s)
        double gain = 1.0;            // DC gain K
    };

    class DiscreteLeadLag : public IController
    {
    public:
        DiscreteLeadLag(const LeadLagParams &params, double sampleTime);

        // Filter input signal u (typically the error or plant output).
        double compute(double u) override;

        void reset() override;
        double sampleTime() const override { return Ts_; }

        void setParams(const LeadLagParams &p);
        const LeadLagParams &params() const { return p_; }

        // Phase lead/lag at frequency omega [rad/s] (continuous-domain approximation).
        double phaseAt(double omega_rad_s) const;

    private:
        LeadLagParams p_;
        double Ts_;
        double b0_, b1_, a1_; // difference-equation coefficients
        double u_prev_, y_prev_;

        void computeCoeffs();
    };

} // namespace ctrl
