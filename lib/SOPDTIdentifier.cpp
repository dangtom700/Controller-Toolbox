#include "SOPDTIdentifier.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace ctrl {

// =============================================================================
// Constructor
// =============================================================================

SOPDTIdentifier::SOPDTIdentifier(const std::vector<double> &times,
                                 const std::vector<double> &outputs,
                                 double stepMagnitude,
                                 double stepTime)
    : step_(stepMagnitude)
{
    if (times.size() != outputs.size())
        throw std::invalid_argument(
            "SOPDTIdentifier: times and outputs must have the same length.");
    if (times.size() < 5)
        throw std::invalid_argument(
            "SOPDTIdentifier: need at least 5 data points.");
    if (std::abs(stepMagnitude) < 1e-14)
        throw std::invalid_argument(
            "SOPDTIdentifier: stepMagnitude must be non-zero.");

    std::size_t start = 0;
    while (start < times.size() && times[start] < stepTime - 1e-12)
        ++start;
    if (start == times.size())
        throw std::invalid_argument(
            "SOPDTIdentifier: stepTime is beyond the data range.");

    y0_ = (start > 0) ? outputs[start - 1] : outputs[start];

    t_.reserve(times.size() - start);
    y_.reserve(times.size() - start);
    for (std::size_t i = start; i < times.size(); ++i) {
        t_.push_back(times[i] - stepTime);
        y_.push_back(outputs[i] - y0_);
    }

    if (t_.size() < 4)
        throw std::invalid_argument(
            "SOPDTIdentifier: fewer than 4 data points remain after stepTime.");

    Ts_data_ = (t_.back() - t_.front()) / static_cast<double>(t_.size() - 1);
}

// =============================================================================
// Public: identify
// =============================================================================

SOPDTModel SOPDTIdentifier::identify(SOPDTMethod method) const
{
    return (method == SOPDTMethod::Optimization) ? identifyOptimization()
                                                 : identifyGraphical();
}

// =============================================================================
// Public: evaluate
// =============================================================================

double SOPDTIdentifier::evaluate(const SOPDTModel &m, double t) const
{
    return y0_ + sopdtResponse(t - m.theta, m.K, m.tau1, m.tau2, 0.0);
}

// =============================================================================
// Public: IMC-PID tuning (Rivera 1986 SOPDT extension)
// =============================================================================

PIDParams SOPDTIdentifier::imcTuning(const SOPDTModel &m, double lambdaC,
                                     double Ts, bool piOnly)
{
    if (lambdaC <= 0.0)
        throw std::invalid_argument(
            "SOPDTIdentifier::imcTuning: lambdaC must be > 0.");

    const double tau1  = std::max(m.tau1,  1e-9);
    const double tau2  = std::max(m.tau2,  0.0);
    const double theta = std::max(m.theta, 0.0);
    const double tauEq = tau1 + tau2;  // equivalent first-order time constant

    // IMC-PID parameters (parallel form, first-order Pade for dead time)
    const double Ti = tauEq;
    const double Td = piOnly ? 0.0 : (tau1 * tau2) / tauEq;
    const double Kp = tauEq / (m.K * (lambdaC + 0.5 * theta));

    PIDParams pp;
    pp.Kp   = Kp;
    pp.Ki   = (Ti > 1e-12) ? Kp / Ti : 0.0;
    pp.Kd   = Kp * Td;
    pp.N    = 10.0;
    pp.uMin = -std::numeric_limits<double>::infinity();
    pp.uMax =  std::numeric_limits<double>::infinity();
    return pp;
}

// =============================================================================
// Private: SOPDT step response (zero-referenced, relative to t=theta)
// =============================================================================

double SOPDTIdentifier::sopdtResponse(double dt, double K_norm,
                                      double tau1, double tau2,
                                      double /*theta*/) const
{
    // dt = t - theta (time past dead-time horizon)
    if (dt <= 0.0) return 0.0;

    const double tol = 1e-9 * (tau1 + tau2 + 1e-30);
    double unit_step;
    if (std::abs(tau1 - tau2) < tol) {
        // Critically damped: 1 - (1 + dt/tau)*exp(-dt/tau)
        const double tau = 0.5 * (tau1 + tau2);
        unit_step = 1.0 - (1.0 + dt / tau) * std::exp(-dt / tau);
    } else {
        // Overdamped: 1 - (tau1*exp(-dt/tau1) - tau2*exp(-dt/tau2)) / (tau1-tau2)
        unit_step = 1.0 - (tau1 * std::exp(-dt / tau1)
                         - tau2 * std::exp(-dt / tau2)) / (tau1 - tau2);
    }
    return K_norm * step_ * unit_step;
}

// =============================================================================
// Private: graphical method
//   theta  : ZN tangent intercept (same as FOPDT)
//   tau_sum: 63.2%-crossing minus theta
//   tau1/tau2 split: estimated from 28.3%-crossing relative to tau_sum.
//     For FOPDT:  t_28.3 / t_63.2 = -ln(1-0.283) / 1.0 = 0.332
//     For SOPDT (equal poles): approximately 0.530
//   Interpolate linearly between these two extremes.
// =============================================================================

SOPDTModel SOPDTIdentifier::identifyGraphical() const
{
    const int    N       = static_cast<int>(y_.size());
    const double y_final = y_.back();
    const double K       = y_final / step_;

    // --- Step 1: ZN tangent -> theta ---
    double max_slope = 0.0;
    int    slope_idx  = 1;
    for (int i = 1; i < N - 1; ++i) {
        const double dt = t_[i + 1] - t_[i - 1];
        if (dt < 1e-14) continue;
        const double slope = (y_[i + 1] - y_[i - 1]) / dt;
        if (slope > max_slope) { max_slope = slope; slope_idx = i; }
    }

    double theta = 0.0;
    if (max_slope > 1e-14)
        theta = std::max(0.0, t_[slope_idx] - y_[slope_idx] / max_slope);

    // --- Step 2: 63.2% crossing -> tau_sum ---
    const double y_632 = 0.632 * y_final;
    double t_632 = t_.back();
    for (int i = 0; i < N - 1; ++i) {
        if (y_[i] <= y_632 && y_[i + 1] >= y_632) {
            const double frac = (y_632 - y_[i]) / (y_[i + 1] - y_[i] + 1e-30);
            t_632 = t_[i] + frac * (t_[i + 1] - t_[i]);
            break;
        }
    }
    const double tau_sum = std::max(t_632 - theta, 2.0 * Ts_data_);

    // --- Step 3: 28.3% crossing -> estimate tau1/tau2 ratio ---
    // 28.3% = 1 - exp(-0.332) for FOPDT -> ratio r = (t28-theta)/tau_sum ~ 0.332
    // 28.3% for critically-damped SOPDT -> ratio r ~ 0.530
    // Interpolate: if r is close to 0.332 => tau2 very small (nearly FOPDT),
    //              if r is close to 0.530 => tau1 = tau2 (equal poles, critical).
    const double y_283 = 0.283 * y_final;
    double t_283 = t_632 * 0.5;  // fallback: midpoint
    for (int i = 0; i < N - 1; ++i) {
        if (y_[i] <= y_283 && y_[i + 1] >= y_283) {
            const double frac = (y_283 - y_[i]) / (y_[i + 1] - y_[i] + 1e-30);
            t_283 = t_[i] + frac * (t_[i + 1] - t_[i]);
            break;
        }
    }

    const double ratio_28 = std::max(0.0, (t_283 - theta) / (tau_sum + 1e-30));
    // Map ratio from [0.332, 0.530] to split alpha in [0, 0.5]
    // alpha=0 -> equal poles (tau1=tau2=tau_sum/2), alpha=0.5 -> FOPDT-like (tau2~0)
    constexpr double r_fopdt = 0.332;
    constexpr double r_crit  = 0.530;
    double alpha = 0.5 * (ratio_28 - r_crit) / (r_fopdt - r_crit + 1e-30);
    alpha = std::max(0.0, std::min(0.45, alpha));

    // tau1 is the dominant (larger) pole
    const double tau1 = tau_sum * (0.5 + alpha);
    const double tau2 = std::max(tau_sum * (0.5 - alpha), Ts_data_);

    // --- RMSE ---
    double sse = 0.0;
    for (int i = 0; i < N; ++i) {
        const double y_hat = sopdtResponse(t_[i] - theta, K, tau1, tau2, 0.0);
        const double e     = y_[i] - y_hat;
        sse += e * e;
    }
    return {K, tau1, tau2, theta, std::sqrt(sse / N)};
}

// =============================================================================
// Private: optimization via nested golden-section search on (theta, tau1, tau2)
//   Outer: theta in [0, 0.5*T]
//   Middle: tau1 in [Ts_data, T - theta]
//   Inner: tau2 in [Ts_data, tau1]   (enforce tau1 >= tau2)
// =============================================================================

SOPDTModel SOPDTIdentifier::identifyOptimization() const
{
    const int    N       = static_cast<int>(y_.size());
    const double y_final = y_.back();
    const double K       = y_final / step_;
    const double T       = t_.back();

    // For fixed (theta, tau1), minimise over tau2 in [Ts_data, tau1]
    auto best_tau2_sse = [&](double theta, double tau1) -> std::pair<double, double> {
        const double tau2_lo = Ts_data_;
        const double tau2_hi = std::max(tau1, tau2_lo + 1e-9);
        const double opt_tau2 = goldenSection(
            [&](double tau2) { return modelSSE(K, tau1, tau2, theta); },
            tau2_lo, tau2_hi);
        return {opt_tau2, modelSSE(K, tau1, opt_tau2, theta)};
    };

    // For fixed theta, minimise over tau1 in [Ts_data, T-theta], then over tau2
    auto best_tau1_sse = [&](double theta) -> std::tuple<double, double, double> {
        const double tau1_lo = Ts_data_;
        const double tau1_hi = std::max(T - theta, 2.0 * Ts_data_);
        const double opt_tau1 = goldenSection(
            [&](double tau1) { return best_tau2_sse(theta, tau1).second; },
            tau1_lo, tau1_hi);
        auto [opt_tau2, sse] = best_tau2_sse(theta, opt_tau1);
        return {opt_tau1, opt_tau2, sse};
    };

    // Outer: optimise theta
    const double opt_theta = goldenSection(
        [&](double theta) { return std::get<2>(best_tau1_sse(theta)); },
        0.0, 0.5 * T);

    auto [opt_tau1, opt_tau2, sse] = best_tau1_sse(opt_theta);

    // Ensure tau1 >= tau2 (ordering convention)
    if (opt_tau1 < opt_tau2) std::swap(opt_tau1, opt_tau2);

    const double rmse = std::sqrt(sse / N);
    return {K, opt_tau1, opt_tau2, opt_theta, rmse};
}

// =============================================================================
// Private: SSE of SOPDT model over data
// =============================================================================

double SOPDTIdentifier::modelSSE(double K_norm, double tau1, double tau2,
                                 double theta) const
{
    double sse = 0.0;
    for (std::size_t i = 0; i < y_.size(); ++i) {
        const double y_hat = sopdtResponse(t_[i] - theta, K_norm, tau1, tau2, 0.0);
        const double e     = y_[i] - y_hat;
        sse += e * e;
    }
    return sse;
}

// =============================================================================
// Private: golden-section search on [a, b]
// =============================================================================

double SOPDTIdentifier::goldenSection(std::function<double(double)> f,
                                      double a, double b, double tol)
{
    constexpr double gr = 0.6180339887498949; // (sqrt(5)-1)/2
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
