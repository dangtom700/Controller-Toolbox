#pragma once
#include "IController.h"
#include <cmath>

// Perturbation-based Extremum Seeking Controller (ESC).
//
// Injects a sinusoidal dither into the plant operating point, demodulates the
// plant output to extract a gradient estimate, then integrates the gradient to
// converge to the extremum of an unknown static (or slowly varying) cost surface.
//
// Algorithm pseudocode (per sample k):
//
//   phase[k]    = phase[k-1] + 2*pi*f_p*Ts          // phase accumulator (mod 2*pi)
//   dither[k]   = a * sin(phase[k])                  // perturbation signal
//   u[k]        = theta[k] + dither[k]               // total plant input
//
//   // High-pass filter (backward-Euler first-order, cutoff f_hpf):
//   alpha_h     = 1 / (1 + 2*pi*f_hpf*Ts)
//   y_h[k]      = alpha_h * (y_h[k-1] + y[k] - y[k-1])   // removes DC/slow drift
//
//   // Demodulate: multiply HPF output by reference dither to extract gradient phase
//   xi[k]       = y_h[k] * sin(phase[k])
//
//   // Low-pass filter (backward-Euler first-order, cutoff f_lpf):
//   alpha_l     = 2*pi*f_lpf*Ts / (1 + 2*pi*f_lpf*Ts)
//   ghat[k]     = ghat[k-1] + alpha_l * (xi[k] - ghat[k-1])   // gradient estimate
//
//   // Gradient-descent (ascent) integration:
//   sign_dir    = seekMinimum ? -1.0 : +1.0
//   theta[k+1]  = theta[k] + sign_dir * k_int * Ts * ghat[k]
//
// Separation of timescales requirement:
//   plant settling time << 1/f_p << 1/(f_lpf)
// so the plant responds quasi-statically to the dither and the LPF cleanly extracts
// the gradient. Violating this produces a biased gradient estimate and slow/no convergence.
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

        // Current operating-point estimate theta (without dither).
        double currentEstimate() const { return theta_; }

    private:
        ExtremumSeekerParams p_;
        double Ts_;
        double phase_;     // dither phase accumulator [rad], wrapped to [0, 2pi)
        double theta_;     // operating-point integrator state
        double hpf_state_; // HPF IIR state (backward-Euler first-order)
        double lpf_state_; // LPF IIR state (backward-Euler first-order)
        double y_prev_;    // y[k-1] for HPF difference term
    };

} // namespace ctrl
