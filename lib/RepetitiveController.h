#pragma once
#include "IController.h"
#include <Eigen/Dense>
#include <memory>
#include <vector>

// Discrete-time Plug-in Repetitive Controller.
//
// Cancels periodic references and disturbances with known period T = N.Ts.
// Wraps any existing IController (base) and adds a learned periodic correction:
//
//   v[k]  = Q . v[k-N] + Krc . e[k]          (repetitive learning law)
//   u[k]  = u_base[k] + v[k]                  (plug-in addition)
//
// v is a circular buffer of length N; each element accumulates over periods.
// After M periods the stored correction cancels the periodic disturbance exactly
// (Q = 1) or with exponential forgetting (Q < 1, better robustness to model error).
//
// Parameters:
//   periodSteps (N): disturbance/reference period in samples = T/Ts (integer).
//   Krc:            learning gain.  Small -> slow but stable. Typical: 0.3..0.8.
//   Q:              forgetting/robustness factor \in (0,1].
//                   Q = 1  -> perfect cancellation, zero robustness margin.
//                   Q < 1  -> exponential decay; suppresses model-mismatch instability.
//                   Typical: 0.95..1.0.
//
// Stability condition (sufficient, SISO, linear base):
//   The closed-loop with the base controller must be stable, AND
//   Q / |1 + L(e^jomega).P(e^jomega)| < 1 for all omega (internal model robustness test).
//
// Usage:
//   auto pid   = std::make_shared<ctrl::DiscretePID>(pp, Ts);
//   auto rc    = ctrl::RepetitiveController(pid, N, 0.5, 0.98, Ts);
//   double u   = rc.compute(error);   // replaces pid.compute(error)
//
// Ref: Hara et al. "Repetitive Control System: A New Type Servo System" (1988);
//      Longman "Iterative Learning and Repetitive Control" (2000).
namespace ctrl
{

struct RepetitiveParams
{
    int    periodSteps = 100;  // N - period in samples
    double Krc         = 0.5;  // learning gain
    double Q           = 0.98; // forgetting / robustness factor
    double uMin        = -1e9;
    double uMax        =  1e9;
};

class RepetitiveController : public IController
{
public:
    // inner:  base stabilising controller (must already close the loop adequately)
    // params: repetitive learning parameters
    // Ts:     sample time [s]
    RepetitiveController(std::shared_ptr<IController> inner,
                         const RepetitiveParams &params,
                         double Ts);

    // compute(error): base control + repetitive correction, clamped to [uMin, uMax].
    double compute(double error) override;

    void   reset()             override;
    double sampleTime()  const override { return Ts_; }

    void             setParams(const RepetitiveParams &p);
    const RepetitiveParams &params() const { return p_; }

    // Current value of the repetitive correction term v[k].
    double correction() const { return v_now_; }

    IController &innerController() { return *inner_; }

private:
    std::shared_ptr<IController> inner_;
    RepetitiveParams p_;
    double Ts_;
    std::vector<double> v_buf_; // circular buffer of length N
    int buf_idx_;               // write pointer (wraps mod N)
    double v_now_;              // most recent correction (for diagnostics)
};

} // namespace ctrl
