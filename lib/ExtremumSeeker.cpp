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
    //   ξ[k] = y_h[k] . sin(omega_p.k.Ts)
    //   After LPF: ĝ approx = J'(θ).a/2   (gradient of cost w.r.t. operating point)
    //
    // LPF (backward Euler first-order):
    //   alpha_l     = omega_l.Ts / (1 + omega_l.Ts)
    //   ĝ[k]   = (1-alpha_l).ĝ[k-1] + alpha_l.ξ[k]
    //
    // Operating-point update (gradient descent / ascent):
    //   θ[k+1] = θ[k] - sign . k_int . Ts . ĝ[k]
    // ---------------------------------------------------------------------------
    double ExtremumSeeker::compute(double y)
    {
        step_++;
        const double t = static_cast<double>(step_) * Ts_;
        const double omega = 2.0 * M_PI * p_.perturbFreq;

        // HPF
        const double wh = 2.0 * M_PI * p_.hpfCutoff;
        const double alpha_h = 1.0 / (1.0 + wh * Ts_);
        const double y_h = alpha_h * (hpf_state_ + y - y_prev_);
        hpf_state_ = y_h;
        y_prev_ = y;

        // Demodulate
        const double demod = y_h * std::sin(omega * t);

        // LPF
        const double wl = 2.0 * M_PI * p_.lpfCutoff;
        const double alpha_l = wl * Ts_ / (1.0 + wl * Ts_);
        lpf_state_ = (1.0 - alpha_l) * lpf_state_ + alpha_l * demod;

        // Gradient integration
        const double sign = p_.seekMinimum ? -1.0 : 1.0;
        theta_ += sign * p_.integGain * lpf_state_ * Ts_;

        // Return operating point plus dither signal
        return theta_ + p_.perturbAmp * std::sin(omega * t);
    }

    void ExtremumSeeker::reset()
    {
        step_ = 0;
        theta_ = 0.0;
        hpf_state_ = 0.0;
        lpf_state_ = 0.0;
        y_prev_ = 0.0;
    }

} // namespace ctrl
