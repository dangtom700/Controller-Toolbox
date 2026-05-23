#pragma once
#include "IController.h"
#include <Eigen/Dense>

// Active Disturbance Rejection Controller - 2nd-order Linear ADRC (LADRC).
//
// Models the controlled plant as a double integrator with unknown total disturbance:
//   ÿ = f(y, ẏ, d, t) + b₀.u    (b₀ is an approximate input gain)
//
// A 3rd-order Extended State Observer (ESO) estimates:
//   z₁ approx = y          (output)
//   z₂ approx = ẏ          (rate)
//   z₃ approx = f(...)       (total disturbance - lumped unknown dynamics + external)
//
// ESO (bandwidth-parameterised, forward Euler, bandwidth omega₀):
//   z₁[k+1] = z₁[k] + Ts.(z₂[k] + beta₁.epsilon[k])
//   z₂[k+1] = z₂[k] + Ts.(z₃[k] + beta₂.epsilon[k] + b₀.u[k])
//   z₃[k+1] = z₃[k] + Ts.beta₃.epsilon[k]
//   epsilon[k]    = y[k] - z₁[k]   (observer error)
//
//   beta₁ = 3omega₀,  beta₂ = 3omega₀^2,  beta₃ = omega₀^3   (characteristic poly (s+omega₀)^3)
//
// PD control + disturbance cancellation (bandwidth omega_c):
//   u₀[k]  = omega_c^2.(r[k] - z₁[k]) - 2omega_c.z₂[k]
//   u[k]   = (u₀[k] - z₃[k]) / b₀
//
// IMPORTANT: compute(y) takes the raw plant output y (NOT the error).
//            Call setReference(r) before each step or use computeTracking(y,r).
//
// Ref: Han "From PID to Active Disturbance Rejection Control" (2009);
//      Gao "Scaling and bandwidth-parameterization based controller tuning" (2003).
namespace ctrl
{

    struct ADRCParams
    {
        double omega_o = 20.0; // ESO bandwidth     [rad/s] - typically 3-10* omega_c
        double omega_c = 5.0;  // Controller BW     [rad/s]
        double b0 = 1.0;       // Approximate plant input gain (b₀ = Km/J for motors)
        double uMin = -1e9;
        double uMax = 1e9;
    };

    class DiscreteADRC : public IController
    {
    public:
        explicit DiscreteADRC(const ADRCParams &params, double sampleTime);

        // Full interface: y = plant output (measurement), r = reference.
        double computeTracking(double y, double r);

        // IController wrapper: signal = plant output y (NOT error).
        // Call setReference(r) once per cycle BEFORE compute(y).
        double compute(double y) override;

        void setReference(double r) { r_ = r; }
        void reset() override;
        double sampleTime() const override { return Ts_; }

        void setParams(const ADRCParams &p);
        const ADRCParams &params() const { return p_; }

        // ESO state estimates: [z₁, z₂, z₃]
        const Eigen::Vector3d &esoState() const { return z_; }

    private:
        ADRCParams p_;
        double Ts_;
        double r_;                     // stored reference
        Eigen::Vector3d z_;            // ESO state [z1, z2, z3]
        double u_prev_;                // u[k-1] for ESO update
        double beta1_, beta2_, beta3_; // observer gains

        void updateGains();
    };

} // namespace ctrl
