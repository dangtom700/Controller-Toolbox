#pragma once
#include "IController.h"

// Discrete First-Order Sliding Mode Controller (SMC).
//
// Sliding surface (SISO, error-based, 1st order):
//   s[k] = c_e . e[k] + c_de . (e[k] - e[k-1])
//         = proportional error + rate-of-error term
//
// Control law (boundary-layer saturation to reduce chattering):
//   u[k] = -K . sat(s[k] / phi)
//   sat(x) = x        if |x| <= 1   (linear PD inside boundary layer)
//   sat(x) = sign(x)  if |x| > 1   (relay switching outside)
//
// Inside boundary layer (|s| <= phi): equivalent to a PD controller.
// Outside (|s| > phi): full switching control - robust to matched disturbances.
//
// Setting phi -> 0 recovers ideal SMC with chattering.
// Setting phi large gives a soft PD approximation.
//
// Ref: Utkin "Sliding Modes in Control and Optimization" (1992);
//      Edwards & Spurgeon "Sliding Mode Control: Theory and Applications" (1998);
//      Simulink Variable-Structure Control block.
namespace ctrl {

struct SMCParams {
    double c_e  = 1.0;   // Error weight in sliding surface
    // Error-rate weight in sliding surface.
    //
    // IMPORTANT - sample-time dependence:
    //   In the discrete implementation de/dt is approximated as (e[k] - e[k-1]) / Ts.
    //   c_de is therefore implicitly Ts-dependent: if you tune c_de at Ts=0.01s and then
    //   halve the sample rate to Ts=0.005s, the sliding surface sensitivity doubles without
    //   any parameter change.  This is a calibration trap.
    //
    //   Convention: c_de stores the DISCRETE coefficient (i.e., c_de already includes Ts).
    //   To convert from a continuous-time slope lambda [1/s] (used in control theory):
    //     c_de = lambda * Ts
    //   If you change Ts, recalculate c_de accordingly.
    //
    // Larger c_de = faster convergence to sliding surface, more chattering.
    double c_de = 0.1;
    double K    = 5.0;   // Switching gain     (larger = more robust, more chattering)
    double phi  = 0.5;   // Boundary layer thickness (larger = smoother, slower)
    double uMin = -1e9;  // Output saturation lower limit
    double uMax =  1e9;  // Output saturation upper limit
};

class DiscreteSMC : public IController {
public:
    explicit DiscreteSMC(const SMCParams& params, double sampleTime);

    // Compute u[k] from error e[k] = r[k] - y[k].
    double compute(double error) override;

    void   reset()             override;
    double sampleTime()  const override { return Ts_; }

    void             setParams(const SMCParams& p) { p_ = p; }
    const SMCParams& params()  const               { return p_; }

    // Current value of the sliding surface s[k] (useful for monitoring).
    double slidingSurface() const { return s_prev_; }

private:
    SMCParams p_;
    double    Ts_;
    double    e_prev_;
    double    s_prev_;
    double    u_prev_;
};

// ---------------------------------------------------------------------------
// SuperTwistingSMC - Discrete 2nd-order sliding mode (super-twisting algorithm).
//
// Eliminates chattering without a boundary layer while retaining finite-time
// convergence and robustness to matched disturbances.
//
// Continuous-time super-twisting law (Levant 1993):
//   u[k]  = v[k] - K1 . |s|^{1/2} . sign(s)
//   v.[k]  = -K2 . sign(s)
//
// Discrete approximation (Euler integration of v):
//   s[k]  = c_e . e[k] + c_de . (e[k] - e[k-1])
//   u[k]  = v[k] - K1 . |s[k]|^{1/2} . sign(s[k])
//   v[k+1]= v[k] - K2 . Ts . sign(s[k])
//
// Gain selection guidance (Moreno & Osorio 2008):
//   K1 > 0,  K2 > K1^2 / 4  (sufficient for finite-time convergence in continuous time).
//   In discrete time keep K2 * Ts small ( << K1 ) to limit discretisation error.
//
// Ref: Levant "Sliding order and sliding accuracy in SMC" IJRNLC (1993);
//      Moreno & Osorio "A Lyapunov approach to super-twisting" IEEE TAC (2012).
// ---------------------------------------------------------------------------

struct SuperTwistingParams
{
    double c_e  = 1.0;   // Error weight in sliding surface (same as SMCParams)
    double c_de = 0.1;   // Error-rate weight (discrete; stores lambda*Ts; see SMCParams c_de note)
    double K1   = 3.0;   // Power term gain  ( |s|^{1/2} )
    double K2   = 5.0;   // Integral gain    ( sign(s) )
    double uMin = -1e9;  // Output saturation lower limit
    double uMax =  1e9;  // Output saturation upper limit
};

class SuperTwistingSMC : public IController
{
public:
    explicit SuperTwistingSMC(const SuperTwistingParams &params, double sampleTime);

    // Compute u[k] from error e[k] = r[k] - y[k].
    double compute(double error) override;

    void   reset()             override;
    double sampleTime()  const override { return Ts_; }

    void                     setParams(const SuperTwistingParams &p) { p_ = p; }
    const SuperTwistingParams &params() const                         { return p_; }

    // Current sliding surface s[k] (for diagnostics).
    double slidingSurface() const { return s_prev_; }

private:
    SuperTwistingParams p_;
    double Ts_;
    double e_prev_;
    double s_prev_;
    double v_;        // integrator state of the super-twisting algorithm
};

} // namespace ctrl
