#pragma once
#include "IController.h"
#include <cmath>

// Perturbation-based Extremum Seeking Controller (ESC).
//
// Injects a sinusoidal dither into the plant operating point, demodulates the
// plant output to extract a gradient estimate, then integrates the gradient to
// converge to the extremum of an unknown static (or slowly varying) cost surface.
//
// Signal chain per sample k:
//   1. Dither:        d[k]     = a . sin(2pi.f_p.k.Ts)
//   2. Plant input:   u[k]     = θ[k] + d[k]
//   3. HPF on y:      y_h[k]   (removes DC/slow bias)
//   4. Demodulate:    ξ[k]     = y_h[k] . sin(2pi.f_p.k.Ts)
//   5. LPF:           ĝ[k]     (gradient estimate - low-frequency content only)
//   6. Integrate:     θ[k+1]   = θ[k] - sign . k_int . Ts . ĝ[k]
//        sign = -1 for minimum seeking, +1 for maximum seeking
//
// Dither frequency must satisfy: plant bandwidth << f_p << 1/(Ts . N_filter)
// so the plant behaves quasi-statically at the dither frequency.
//
// Ref: Ariyur & Krstic "Real-Time Optimisation by Extremum-Seeking Control" (2003);
//      Tan et al. "On non-local stability of ESC" (2006);
//      Simulink Extremum Seeking block (MATLAB R2020b+).
namespace ctrl
{

    struct ExtremumSeekerParams
    {
        double perturbAmp = 0.1;  // Dither amplitude  a      - larger = faster, more perturbation
        double perturbFreq = 1.0; // Dither frequency  f_p [Hz] - well above closed-loop BW
        double lpfCutoff = 0.1;   // Low-pass filter   f_lpf [Hz] - gradient smoothing
        double hpfCutoff = 0.05;  // High-pass filter  f_hpf [Hz] - DC removal
        double integGain = 1.0;   // Gradient integrator gain k_int
        bool seekMinimum = true;  // true -> seek minimum;  false -> seek maximum
    };

    class ExtremumSeeker : public IController
    {
    public:
        ExtremumSeeker(const ExtremumSeekerParams &params, double sampleTime);

        // signal = plant output y[k] (cost/performance metric, NOT tracking error).
        // Returns plant input u[k] = operating-point estimate + dither.
        double compute(double signal) override;

        void reset() override;
        double sampleTime() const override { return Ts_; }

        void setParams(const ExtremumSeekerParams &p) { p_ = p; }
        const ExtremumSeekerParams &params() const { return p_; }

        // Current operating-point estimate θ (without dither).
        double currentEstimate() const { return theta_; }

    private:
        ExtremumSeekerParams p_;
        double Ts_;
        long step_;
        double theta_;     // operating-point integrator state
        double hpf_state_; // HPF IIR state (backward-Euler first-order)
        double lpf_state_; // LPF IIR state (backward-Euler first-order)
        double y_prev_;    // y[k-1] for HPF difference term
    };

} // namespace ctrl
