#include "ExtremumSeeker.h"

namespace ctrl
{

    ExtremumSeeker::ExtremumSeeker(const ExtremumSeekerParams &params, double sampleTime)
        : p_(params), Ts_(sampleTime)
    {
        reset();
    }

    // ---------------------------------------------------------------------------
    // ESC step - perturbation + demodulation + gradient integration.
    //
    // HPF (backward Euler first-order, removes DC offset from y):
    //   alpha_h     = 1 / (1 + omega_h.Ts)
    //   y_h[k]  = alpha_h.(y_h[k-1] + y[k] - y[k-1])
    //
    // Demodulate by multiplying with reference dither sin(omega_p.k.Ts):
    //   xi[k] = y_h[k] . sin(omega_p.k.Ts)
    //   After LPF: ghat approx = J'(theta).a/2   (gradient of cost w.r.t. operating point)
    //
    // LPF (backward Euler first-order):
    //   alpha_l     = omega_l.Ts / (1 + omega_l.Ts)
    //   ghat[k]   = (1-alpha_l).ghat[k-1] + alpha_l.xi[k]
    //
    // Operating-point update (gradient descent / ascent):
    //   theta[k+1] = theta[k] - sign . k_int . Ts . ghat[k]
    // ---------------------------------------------------------------------------
    double ExtremumSeeker::compute(double y)
    {
        // Advance phase accumulator - stays bounded in [0, 2pi) for arbitrarily long runs,
        // avoiding the floating-point precision loss of step_ * Ts_ at large step counts
        // and the 32-bit overflow of a long counter on embedded targets.
        phase_ = std::fmod(phase_ + 2.0 * M_PI * p_.perturbFreq * Ts_, 2.0 * M_PI);

        // HPF
        const double wh = 2.0 * M_PI * p_.hpfCutoff;
        const double alpha_h = 1.0 / (1.0 + wh * Ts_);
        const double y_h = alpha_h * (hpf_state_ + y - y_prev_);
        hpf_state_ = y_h;
        y_prev_ = y;

        // Demodulate using the same phase_ as the dither - coherent demodulation
        const double demod = y_h * std::sin(phase_);

        // LPF
        const double wl = 2.0 * M_PI * p_.lpfCutoff;
        const double alpha_l = wl * Ts_ / (1.0 + wl * Ts_);
        lpf_state_ = (1.0 - alpha_l) * lpf_state_ + alpha_l * demod;

        // Gradient integration
        const double sign = p_.seekMinimum ? -1.0 : 1.0;
        theta_ += sign * p_.integGain * lpf_state_ * Ts_;

        // Return operating point plus dither signal
        return theta_ + p_.perturbAmp * std::sin(phase_);
    }

    void ExtremumSeeker::reset()
    {
        phase_ = 0.0;
        theta_ = 0.0;
        hpf_state_ = 0.0;
        lpf_state_ = 0.0;
        y_prev_ = 0.0;
    }

} // namespace ctrl
