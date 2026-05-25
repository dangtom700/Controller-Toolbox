#pragma once
#include "IController.h"
#include <Eigen/Dense>

// Active Disturbance Rejection Controller - 2nd-order Linear ADRC (LADRC).
//
// Models the controlled plant as a double integrator with unknown total disturbance:
//   y = f(y, ydot, d, t) + b0.u    (b0 is an approximate input gain)
//
// A 3rd-order Extended State Observer (ESO) estimates:
//   z1 approx = y          (output)
//   z2 approx = ydot          (rate)
//   z3 approx = f(...)       (total disturbance - lumped unknown dynamics + external)
//
// ESO (bandwidth-parameterised, backward Euler, bandwidth omega0):
// Discrete update solves (I - Ts.Ae) z_new = z + Ts.(Be_eps.eps + Be_u.u)
// where Ae = [[0,1,0],[0,0,1],[0,0,0]] (nilpotent).  The analytical inverse of
// (I - Ts.Ae) is [[1,Ts,Ts^2],[0,1,Ts],[0,0,1]], yielding:
//   z3[k+1] = z3[k] + Ts.beta3.epsilon[k]
//   z2[k+1] = z2[k] + Ts.(beta2.epsilon[k] + b0.u[k]) + Ts.z3[k+1]
//   z1[k+1] = z1[k] + Ts.beta1.epsilon[k]              + Ts.z2[k+1]
//   epsilon[k] = y[k] - z1[k]  (observer error, using old z1 - semi-implicit)
//
// A-stable: stable for all omega0.Ts > 0 (unlike the prior forward-Euler scheme).
//   beta1 = 3omega0,  beta2 = 3omega0^2,  beta3 = omega0^3   (characteristic poly (s+omega0)^3)
//
// PD control + disturbance cancellation (bandwidth omega_c):
//   u0[k]  = omega_c^2.(r[k] - z1[k]) - 2omega_c.z2[k]
//   u[k]   = (u0[k] - z3[k]) / b0
//
// IController::compute(error) follows the standard contract (error = r - y).
// For the full interface use computeTracking(y, r) directly.
//
// Ref: Han "From PID to Active Disturbance Rejection Control" (2009);
//      Gao "Scaling and bandwidth-parameterization based controller tuning" (2003).
namespace ctrl
{

    struct ADRCParams
    {
        double omega_o = 20.0; // ESO bandwidth     [rad/s] - typically 3-10* omega_c
        double omega_c = 5.0;  // Controller BW     [rad/s]
        double b0 = 1.0;       // Approximate plant input gain (b0 = Km/J for motors)
        double uMin = -1e9;
        double uMax = 1e9;
    };

    class DiscreteADRC : public IController
    {
    public:
        explicit DiscreteADRC(const ADRCParams &params, double sampleTime);

        // Full interface: y = plant output (measurement), r = reference.
        double computeTracking(double y, double r);

        // IController wrapper: signal = error = r - y  (standard IController contract).
        //
        // IMPORTANT: call setReference(r) once per cycle BEFORE calling compute(error).
        // If setReference() is not called, r_ defaults to 0 and this method will drive
        // the plant output to zero regardless of the intended setpoint - a silent wrong
        // answer that is hard to diagnose inside a ControllerStack.
        //
        // Internally recovers y = r_ - error and delegates to computeTracking(y, r_).
        // Prefer computeTracking(y, r) directly whenever y and r are independently available.
        double compute(double error) override;

        void setReference(double r) { r_ = r; r_was_set_ = true; }
        void reset() override;
        double sampleTime() const override { return Ts_; }

        void setParams(const ADRCParams &p);
        const ADRCParams &params() const { return p_; }

        // ESO state estimates: [z1, z2, z3]
        const Eigen::Vector3d &esoState() const { return z_; }

    private:
        ADRCParams p_;
        double Ts_;
        double r_;                     // stored reference
        Eigen::Vector3d z_;            // ESO state [z1, z2, z3]
        double u_prev_;                // u[k-1] for ESO update
        bool   r_was_set_;             // guards against compute() without setReference()
        double beta1_, beta2_, beta3_; // observer gains

        void updateGains();
    };

} // namespace ctrl
