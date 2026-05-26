#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <memory>
#include <optional>
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
//   Inner loop: C(z) -> P0(z) . z^{-d}   (plant with delay)
//   Model:      yhat    = P0(z) . u        (model without delay)
//   Correction: c    = yhat - z^{-d}.yhat  (delay-induced error cancelled)
//
// Requirements: plant model P0 must represent the delay-FREE dynamics.
//
// Fractional dead-time support:
//   If the true dead time theta is not a multiple of Ts, pass the optional
//   fracDelayFilter argument (a 1-state StateSpace from padeDelayFilter()).
//   The filter H_frac is connected in series with delayModel so the effective
//   model is  P0_eff(z) = H_frac(z) . P0(z), matching e^{-theta_frac * s}
//   to first order without rounding error.
//
//   Convenience construction via the whole dead time:
//     ctrl::SmithPredictor sp(inner, G0, theta, Ts);
//   This overload splits theta automatically into floor(theta/Ts) integer steps
//   plus a first-order Pade filter for the remainder.
//
// Ref: Smith (1957); Astrom & Wittenmark "Computer Controlled Systems" Sec 6.4.
namespace ctrl
{

    class SmithPredictor : public IController
    {
    public:
        // -- Overload 1 (original): integer delay only -------------------------
        // inner:       any discrete controller (e.g., DiscretePID)
        // delayModel:  state-space model of the plant WITHOUT the dead-time delay
        // delaySteps:  integer dead-time length in samples d
        SmithPredictor(std::shared_ptr<IController> inner,
                       const StateSpace &delayModel,
                       int delaySteps);

        // -- Overload 2: fractional delay support via Pade filter --------------
        // inner:            any discrete controller
        // delayModel:       state-space model of the plant WITHOUT dead-time
        // delaySteps:       integer part of dead-time in samples
        // fracDelayFilter:  1-state StateSpace from padeDelayFilter(theta_frac, Ts)
        //                   representing the sub-sample fractional delay.
        SmithPredictor(std::shared_ptr<IController> inner,
                       const StateSpace &delayModel,
                       int delaySteps,
                       const StateSpace &fracDelayFilter);

        // -- Overload 3: convenient whole dead-time constructor ----------------
        // inner:      any discrete controller
        // delayModel: delay-free plant model
        // theta:      total dead time [s]
        // Ts:         sample time [s] (must match delayModel.Ts)
        // Automatically computes delaySteps = floor(theta/Ts) and builds the
        // first-order Pade filter for the fractional remainder.
        SmithPredictor(std::shared_ptr<IController> inner,
                       const StateSpace &delayModel,
                       double theta,
                       double Ts);

        // Compute u[k] from closed-loop error e[k] = r[k] - y[k].
        double compute(double error) override;

        void reset() override;
        double sampleTime() const override { return Ts_; }

        // Access the wrapped inner controller for runtime tuning.
        IController &innerController() { return *inner_; }

        // Replace the internal delay-free plant model and dead-time length at runtime.
        // Resets model state and output buffer (equivalent to calling reset()).
        void setModel(const StateSpace &delayModel, int delaySteps);

        // Fractional-delay variant of setModel.
        void setModel(const StateSpace &delayModel, int delaySteps,
                      const StateSpace &fracDelayFilter);

    private:
        void initBuffers();

        std::shared_ptr<IController>  inner_;
        StateSpace                    model_;       // delay-free plant model P0
        int                           d_;           // integer delay steps
        double                        Ts_;
        bool                          has_frac_;    // true when a Pade filter is active
        StateSpace                    frac_filter_; // H_frac state-space (1 state)
        Eigen::VectorXd               x_frac_;      // state of the fractional filter

        Eigen::VectorXd               x_model_;     // internal model state x^
        Eigen::VectorXd               u_prev_;      // u[k-1] for D.u feedthrough
        std::vector<double>           y_buf_;       // circular buffer for yhat delay
        int                           buf_head_;
    };

} // namespace ctrl
