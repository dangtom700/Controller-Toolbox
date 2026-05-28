#include "FOPDTIdentifier.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace ctrl {

// =============================================================================
// Constructor
// =============================================================================

FOPDTIdentifier::FOPDTIdentifier(const std::vector<double> &times,
                                 const std::vector<double> &outputs,
                                 double stepMagnitude,
                                 double stepTime)
    : step_(stepMagnitude)
{
    if (times.size() != outputs.size())
        throw std::invalid_argument("FOPDTIdentifier: times and outputs must be the same length.");
    if (times.size() < 4)
        throw std::invalid_argument("FOPDTIdentifier: need at least 4 data points.");
    if (std::abs(stepMagnitude) < 1e-14)
        throw std::invalid_argument("FOPDTIdentifier: stepMagnitude must be non-zero.");

    // Find first index at or after stepTime
    std::size_t start = 0;
    while (start < times.size() && times[start] < stepTime - 1e-12)
        ++start;
    if (start == times.size())
        throw std::invalid_argument("FOPDTIdentifier: stepTime is beyond the data range.");

    // Initial output level = value just before step (or at start if no pre-step data)
    y0_ = (start > 0) ? outputs[start - 1] : outputs[start];

    // Build time-shifted, zero-referenced data
    t_.reserve(times.size() - start);
    y_.reserve(times.size() - start);
    for (std::size_t i = start; i < times.size(); ++i) {
        t_.push_back(times[i] - stepTime);
        y_.push_back(outputs[i] - y0_);
    }

    if (t_.size() < 3)
        throw std::invalid_argument(
            "FOPDTIdentifier: fewer than 3 data points remain after stepTime.");

    Ts_data_ = (t_.back() - t_.front()) / static_cast<double>(t_.size() - 1);
}

// =============================================================================
// Public: identify
// =============================================================================

FOPDTModel FOPDTIdentifier::identify(FOPDTMethod method) const
{
    return (method == FOPDTMethod::Optimization) ? identifyOptimization()
                                                 : identifyGraphical();
}

// =============================================================================
// Public: evaluate fitted model at absolute time t
// =============================================================================

double FOPDTIdentifier::evaluate(const FOPDTModel &m, double t) const
{
    const double dt = t - m.theta;   // t relative to step application
    if (dt <= 0.0) return y0_;
    return y0_ + m.K * step_ * (1.0 - std::exp(-dt / m.tau));
}

// =============================================================================
// Public: IMC-PID tuning (Rivera, Morari & Skogestad 1986)
// =============================================================================

PIDParams FOPDTIdentifier::imcTuning(const FOPDTModel &m, double lambdaC,
                                     double Ts, bool piOnly)
{
    if (lambdaC <= 0.0)
        throw std::invalid_argument("FOPDTIdentifier::imcTuning: lambdaC must be > 0.");

    const double tau = std::max(m.tau,   1e-9);
    const double th  = std::max(m.theta, 0.0);
    const double lam = lambdaC;

    // IMC-PID ideal parameters (parallel form)
    const double Ti = tau + 0.5 * th;
    const double Td = piOnly ? 0.0 : (tau * th) / (2.0 * tau + th);
    const double Kp = Ti / (m.K * (lam + 0.5 * th));

    PIDParams pp;
    pp.Kp   = Kp;
    pp.Ki   = (Ti > 1e-12) ? Kp / Ti : 0.0;
    pp.Kd   = Kp * Td;
    pp.N    = 10.0;  // derivative filter coefficient (10 = moderate filtering)
    pp.uMin = -std::numeric_limits<double>::infinity();
    pp.uMax =  std::numeric_limits<double>::infinity();
    return pp;
}

// =============================================================================
// Private: graphical method (Ziegler-Nichols tangent + 63.2% intercept)
// =============================================================================

FOPDTModel FOPDTIdentifier::identifyGraphical() const
{
    const int    N       = static_cast<int>(y_.size());
    const double y_final = y_.back();
    const double K       = y_final / step_;

    // Find steepest tangent using centred finite differences
    double max_slope = 0.0;
    int    slope_idx  = 1;
    for (int i = 1; i < N - 1; ++i) {
        const double dt = t_[i + 1] - t_[i - 1];
        if (dt < 1e-14) continue;
        const double slope = (y_[i + 1] - y_[i - 1]) / dt;
        if (slope > max_slope) { max_slope = slope; slope_idx = i; }
    }

    // Tangent intercept with y=0: t_theta = t[slope_idx] - y[slope_idx] / slope
    double theta = 0.0;
    if (max_slope > 1e-14)
        theta = std::max(0.0, t_[slope_idx] - y_[slope_idx] / max_slope);

    // Time to 63.2% of final value
    const double y_632 = 0.632 * y_final;
    double t_632 = t_.back();
    for (int i = 0; i < N - 1; ++i) {
        if (y_[i] <= y_632 && y_[i + 1] >= y_632) {
            const double frac = (y_632 - y_[i]) / (y_[i + 1] - y_[i] + 1e-30);
            t_632 = t_[i] + frac * (t_[i + 1] - t_[i]);
            break;
        }
    }

    const double tau = std::max(t_632 - theta, Ts_data_);

    // RMSE
    double sse = 0.0;
    for (int i = 0; i < N; ++i) {
        const double dt    = t_[i] - theta;
        const double y_hat = (dt > 0.0) ? K * step_ * (1.0 - std::exp(-dt / tau)) : 0.0;
        const double e     = y_[i] - y_hat;
        sse += e * e;
    }
    return {K, tau, theta, std::sqrt(sse / N)};
}

// =============================================================================
// Private: optimization via golden-section search on theta, then tau
// =============================================================================

FOPDTModel FOPDTIdentifier::identifyOptimization() const
{
    const int    N       = static_cast<int>(y_.size());
    const double y_final = y_.back();
    const double K       = y_final / step_;
    const double T       = t_.back();

    // Search bounds
    const double theta_lo = 0.0;
    const double theta_hi = 0.5 * T;

    // For a fixed theta, minimise SSE over tau via golden-section
    auto best_tau_sse = [&](double theta) -> std::pair<double, double> {
        const double tau_lo  = Ts_data_;
        const double tau_hi  = std::max(T - theta, 2.0 * Ts_data_);
        const double opt_tau = goldenSection(
            [&](double tau) { return modelSSE(K, tau, theta); },
            tau_lo, tau_hi);
        return {opt_tau, modelSSE(K, opt_tau, theta)};
    };

    // Outer golden-section on theta
    const double opt_theta = goldenSection(
        [&](double theta) { return best_tau_sse(theta).second; },
        theta_lo, theta_hi);

    auto [opt_tau, sse] = best_tau_sse(opt_theta);
    const double rmse = std::sqrt(sse / N);
    return {K, opt_tau, opt_theta, rmse};
}

// =============================================================================
// Private: sum of squared errors for FOPDT model over data
// =============================================================================

double FOPDTIdentifier::modelSSE(double K_norm, double tau, double theta) const
{
    double sse = 0.0;
    for (std::size_t i = 0; i < y_.size(); ++i) {
        const double dt    = t_[i] - theta;
        const double y_hat = (dt > 0.0) ? K_norm * step_ * (1.0 - std::exp(-dt / tau)) : 0.0;
        const double e     = y_[i] - y_hat;
        sse += e * e;
    }
    return sse;
}

// =============================================================================
// Private: golden-section minimum search on [a, b]
// =============================================================================

double FOPDTIdentifier::goldenSection(std::function<double(double)> f,
                                      double a, double b, double tol)
{
    // Golden ratio conjugate: (sqrt(5)-1)/2
    constexpr double gr = 0.6180339887498949;
    double c = b - gr * (b - a);
    double d = a + gr * (b - a);
    while (std::abs(b - a) > tol) {
        if (f(c) < f(d)) { b = d; }
        else              { a = c; }
        c = b - gr * (b - a);
        d = a + gr * (b - a);
    }
    return 0.5 * (a + b);
}

} // namespace ctrl
