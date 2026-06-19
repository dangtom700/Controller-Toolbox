#pragma once
// Shared, header-only fault-injection + statistics utilities for case-study
// robustness analysis (fault sweep, Monte Carlo, WCET). Deliberately has no
// Eigen / controller_toolbox dependency so any case study's robustness_main.cpp
// can include it directly alongside its own plant/controller headers.
//
// FaultKind/FaultSpec mirror tools/fault_injector.py's FaultSpec/FAULT_KINDS.
// MetricStats mirrors lib/RobustnessAnalysis.h's MonteCarloResult::MetricStats
// shape (mean/std_dev/percentiles/worst), reimplemented here so callers don't
// need to construct a ctrl::StateSpace just to get ensemble statistics over a
// real nonlinear closed-loop run.

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace robust {

enum class FaultKind {
    None,
    SensorBias,
    SensorNoise,
    ActuatorLoss,
    ActuatorStuck,
    SetpointStep,
};

inline const char* faultKindName(FaultKind k) {
    switch (k) {
        case FaultKind::None:          return "none";
        case FaultKind::SensorBias:    return "sensor_bias";
        case FaultKind::SensorNoise:   return "sensor_noise";
        case FaultKind::ActuatorLoss:  return "actuator_loss";
        case FaultKind::ActuatorStuck: return "actuator_stuck";
        case FaultKind::SetpointStep:  return "setpoint_step";
    }
    return "unknown";
}

struct FaultSpec {
    FaultKind kind       = FaultKind::None;
    double    fault_time = 0.0;                                      // [s] activation time
    double    magnitude  = 0.0;                                      // bias / noise sigma / loss fraction / stuck value / step amount
    double    end_time   = std::numeric_limits<double>::infinity();  // [s] deactivation time

    bool active(double t) const { return kind != FaultKind::None && t >= fault_time && t < end_time; }
};

// Sensor fault: perturbs the measured value fed into controller.compute().
inline double applySensorFault(double y_true, const FaultSpec& f, double t, std::mt19937& rng) {
    if (!f.active(t)) return y_true;
    if (f.kind == FaultKind::SensorBias) return y_true + f.magnitude;
    if (f.kind == FaultKind::SensorNoise) {
        std::normal_distribution<double> n(0.0, f.magnitude);
        return y_true + n(rng);
    }
    return y_true;
}

// Actuator fault: perturbs the controller's commanded output before plant.step().
inline double applyActuatorFault(double u_cmd, const FaultSpec& f, double t) {
    if (!f.active(t)) return u_cmd;
    if (f.kind == FaultKind::ActuatorLoss)  return u_cmd * (1.0 - f.magnitude);
    if (f.kind == FaultKind::ActuatorStuck) return f.magnitude;
    return u_cmd;
}

// Setpoint fault: steps the reference signal fed into controller.compute().
inline double applySetpointFault(double ref_nominal, const FaultSpec& f, double t) {
    if (!f.active(t)) return ref_nominal;
    if (f.kind == FaultKind::SetpointStep) return ref_nominal + f.magnitude;
    return ref_nominal;
}

// ---------------------------------------------------------------------------
// Per-run summary (one Monte Carlo sample or one fault-sweep trial)
// ---------------------------------------------------------------------------
struct SimSummary {
    bool   stable       = true;  // false if NaN/Inf seen or plant left its plausible operating range
    double iae           = 0.0;
    double rms_error      = 0.0;
    double max_abs_error  = 0.0;
    double settle_time_s  = -1.0;  // -1 if it never settles within the run
    double overshoot_pct  = 0.0;
    double max_u          = 0.0;
    double energy_var     = 0.0;  // population variance of u(t) over the run (actuator wear proxy)
};

// Population variance (ddof=0, matching numpy's np.var default) from streaming sums.
inline double varianceFromSums(double sum_u, double sum_u2, double n) {
    if (n <= 0.0) return 0.0;
    const double mean_u = sum_u / n;
    return std::max(0.0, sum_u2 / n - mean_u * mean_u);
}

// ---------------------------------------------------------------------------
// Aggregated percentile statistics over an ensemble.
// ---------------------------------------------------------------------------
struct MetricStats {
    double mean = 0.0, std_dev = 0.0;
    double p5 = 0.0, p25 = 0.0, p50 = 0.0, p75 = 0.0, p95 = 0.0;
    double worst = 0.0;  // max, since iae/rms/settle/overshoot are all "smaller is better"
};

// Percentile via linear interpolation on a sorted copy of `sorted_values`.
inline double percentile(std::vector<double> sorted_values, double q) {
    if (sorted_values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(sorted_values.begin(), sorted_values.end());
    const double pos = q * static_cast<double>(sorted_values.size() - 1);
    const size_t lo  = static_cast<size_t>(std::floor(pos));
    const size_t hi  = static_cast<size_t>(std::ceil(pos));
    if (lo == hi) return sorted_values[lo];
    const double frac = pos - static_cast<double>(lo);
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac;
}

inline MetricStats computeStats(const std::vector<double>& values) {
    MetricStats s;
    if (values.empty()) return s;
    const double n = static_cast<double>(values.size());

    double sum = 0.0;
    for (double v : values) sum += v;
    s.mean = sum / n;

    double var = 0.0;
    for (double v : values) var += (v - s.mean) * (v - s.mean);
    s.std_dev = std::sqrt(var / n);

    std::vector<double> sorted_v = values;
    std::sort(sorted_v.begin(), sorted_v.end());
    s.p5    = percentile(sorted_v, 0.05);
    s.p25   = percentile(sorted_v, 0.25);
    s.p50   = percentile(sorted_v, 0.50);
    s.p75   = percentile(sorted_v, 0.75);
    s.p95   = percentile(sorted_v, 0.95);
    s.worst = sorted_v.back();
    return s;
}

// Relative multiplicative perturbation factor for Monte Carlo plant params: 1 + sigma*N(0,1).
inline double perturbFactor(double sigma, std::mt19937& rng) {
    std::normal_distribution<double> n(0.0, sigma);
    return 1.0 + n(rng);
}

} // namespace robust
