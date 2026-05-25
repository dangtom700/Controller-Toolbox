#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <memory>
#include <vector>

// Smith Predictor - compensates for pure integer dead-time in discrete plants.
//
// Replaces the dead-time delay in the feedback path with a prediction from
// an internal model, so the inner controller C(z) sees a delay-free loop.
//
// Modified error delivered to the inner controller:
//   e_sp[k] = (r[k] - y[k]) + (yhat_model[k] - yhat_model[k-d])
//            = error + (current model output - d-step-delayed model output)
//
// Signal-flow equivalent (ref: Smith 1957):
//   Inner loop: C(z) -> P0(z) . z^-^d   (plant with delay)
//   Model:      yhat    = P0(z) . u       (model without delay)
//   Correction: c    = yhat - z^-^dyhat       (delay-induced error cancelled)
//
// Requirements: plant model P0 must represent the delay-FREE dynamics.
//
// Limitation - fractional dead times: delaySteps must be a positive integer.
// If the true dead time theta is not a multiple of Ts, users must round:
//   delaySteps = std::lround(theta / Ts)
// For plants where theta falls between 0.5 and 1.5 sample times the rounding
// error is significant. The standard fix is a first-order Pade approximant for
// the fractional part (theta_frac = theta - floor(theta/Ts)*Ts):
//   H_pade(z) approx (1 - 0.5*theta_frac/Ts * z^{-1}) / (1 + 0.5*theta_frac/Ts * z^{-1})
// Absorb H_pade into P0 before passing to the constructor when sub-sample accuracy matters.
//
// Ref: Smith (1957); Astrom & Wittenmark "Computer Controlled Systems" Section 6.4;
//      MATLAB Smith Predictor example (smithpredict documentation).
namespace ctrl
{

    class SmithPredictor : public IController
    {
    public:
        // inner:       any discrete controller (e.g., DiscretePID)
        // delayModel:  state-space model of the plant WITHOUT the dead-time delay
        // delaySteps:  integer dead-time length in samples d
        SmithPredictor(std::shared_ptr<IController> inner,
                       const StateSpace &delayModel,
                       int delaySteps);

        // Compute u[k] from closed-loop error e[k] = r[k] - y[k] (actual plant output).
        double compute(double error) override;

        void reset() override;
        double sampleTime() const override { return Ts_; }

        // Access the wrapped inner controller for runtime tuning.
        IController &innerController() { return *inner_; }

        // Replace the internal delay-free plant model and dead-time length at runtime.
        // Resets model state and output buffer (equivalent to calling reset()).
        // Use when the plant dynamics change (e.g., adaptive Smith Predictor).
        void setModel(const StateSpace &delayModel, int delaySteps);

    private:
        std::shared_ptr<IController> inner_;
        StateSpace model_;
        int d_;
        double Ts_;
        Eigen::VectorXd      x_model_;  // internal model state x^
        Eigen::VectorXd      u_prev_;   // u[k-1] for D.u feedthrough in y_now
        std::vector<double>  y_buf_;   // fixed circular buffer of yhat (length d_, pre-allocated)
        int                  buf_head_; // index of the oldest slot
    };

} // namespace ctrl
