#pragma once
#include "IController.h"

// Discrete First-Order Sliding Mode Controller (SMC).
//
// Sliding surface (SISO, error-based, 1st order):
//   s[k] = c_e . e[k] + c_de . (e[k] - e[k-1])
//         = proportional error + rate-of-error term
//
// Control law (boundary-layer saturation to reduce chattering):
//   u[k] = -K . sat(s[k] / φ)
//   sat(x) = x        if |x| <= 1   (linear PD inside boundary layer)
//   sat(x) = sign(x)  if |x| > 1   (relay switching outside)
//
// Inside boundary layer (|s| <= φ): equivalent to a PD controller.
// Outside (|s| > φ): full switching control - robust to matched disturbances.
//
// Setting φ -> 0 recovers ideal SMC with chattering.
// Setting φ large gives a soft PD approximation.
//
// Ref: Utkin "Sliding Modes in Control and Optimization" (1992);
//      Edwards & Spurgeon "Sliding Mode Control: Theory and Applications" (1998);
//      Simulink Variable-Structure Control block.
namespace ctrl {

struct SMCParams {
    double c_e  = 1.0;   // Error weight in sliding surface
    // Error-rate weight in sliding surface.
    // Note: c_de absorbs the sample time Ts. To match a continuous-time
    // slope lambda [1/s], set c_de = lambda * Ts.
    // Larger c_de = faster convergence but more chattering.
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

} // namespace ctrl
