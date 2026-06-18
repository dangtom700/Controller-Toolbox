#pragma once
#include "IControllerObserver.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

/**
 * @file ControllerMonitor.h
 * @brief Statistical Process Control (SPC) charts for live controller monitoring (M3/SPC).
 *
 * Attaches to any `IController` as an observer and runs SPC charts on the
 * control output stream.  Fires a configurable alarm callback when a chart
 * signals a fault or performance degradation.
 *
 * **Charts included:**
 *
 * | Chart  | Detects                         | Parameter        |
 * |--------|---------------------------------|------------------|
 * | CUSUM  | Sustained mean shift            | k (slack), h (threshold) |
 * | EWMA   | Slow drift / gradual trend      | lambda (weight), L (sigma multiplier) |
 *
 * **Usage - monitor PID output for mean shift:**
 * @code
 *   ctrl::ControllerMonitor monitor;
 *   monitor.setTarget(0.0);          // expected steady-state u
 *   monitor.setSigma(0.05);          // process noise level
 *   monitor.setAlarmCallback([](const std::string& chart, double stat) {
 *       std::printf("[ALARM] %s statistic = %.3f\n", chart.c_str(), stat);
 *   });
 *
 *   pid.attachObserver(&monitor);    // monitor runs on every compute() call
 * @endcode
 *
 * **Usage - monitor ADRC ESO disturbance estimate (z3) via onState:**
 * @code
 *   ctrl::ControllerMonitor esomon;
 *   esomon.setWatchKey("eso");        // responds to key="eso" from ADRC
 *   esomon.setWatchIndex(2);          // watch z3 (index 2 of [z1,z2,z3])
 *   esomon.setTarget(0.0); esomon.setSigma(0.1);
 *   adrc.attachObserver(&esomon);
 * @endcode
 *
 * @see Shewhart, W.A. (1931). Economic Control of Quality of Manufactured Product.
 * @see Page, E.S. (1954). Continuous inspection schemes. Biometrika 41(1-2):100-115.
 * @see Roberts, S.W. (1959). Control chart tests based on geometric moving averages.
 *      Technometrics 1(3):239-250.
 */

namespace ctrl {

/**
 * @brief SPC alarm callback: (chart_name, current_statistic).
 */
using AlarmCallback = std::function<void(std::string_view, double)>;

/**
 * @brief CUSUM chart: detects sustained upward and downward mean shifts.
 *
 * Two-sided CUSUM:
 * @code
 *   C_plus[k]  = max(0, C_plus[k-1]  + (x - mu0 - k_slack))
 *   C_minus[k] = max(0, C_minus[k-1] - (x - mu0 + k_slack))
 *   Alarm if C_plus >= h  OR  C_minus >= h
 * @endcode
 */
struct CUSUMChart {
    double target    = 0.0;   ///< In-control mean mu0
    double sigma     = 1.0;   ///< In-control standard deviation
    double k         = 0.5;   ///< Slack parameter (multiples of sigma; typically 0.5)
    double h         = 5.0;   ///< Detection threshold (multiples of sigma; typically 4-5)

    double C_plus  = 0.0;
    double C_minus = 0.0;

    void reset() { C_plus = C_minus = 0.0; }

    /**
     * @brief Update chart with new observation.
     * @return true if an alarm condition is detected.
     */
    bool update(double x) {
        const double k_abs = k * sigma;
        const double h_abs = h * sigma;
        C_plus  = std::max(0.0, C_plus  + (x - target) - k_abs);
        C_minus = std::max(0.0, C_minus - (x - target) - k_abs);
        return (C_plus >= h_abs) || (C_minus >= h_abs);
    }

    double statistic() const { return std::max(C_plus, C_minus); }
};

/**
 * @brief EWMA chart: detects gradual trends and slow drifts.
 *
 * @code
 *   Z[k] = lambda * x[k] + (1 - lambda) * Z[k-1]
 *   UCL  = mu0 + L * sigma * sqrt(lambda / (2 - lambda))
 *   Alarm if |Z[k] - mu0| >= UCL_half_width
 * @endcode
 */
struct EWMAChart {
    double target = 0.0;   ///< In-control mean mu0
    double sigma  = 1.0;   ///< In-control standard deviation
    double lambda = 0.2;   ///< Smoothing weight in (0,1]; smaller = more smoothing
    double L      = 3.0;   ///< Control limit multiplier (typically 2.7-3.0)

    double Z = 0.0;

    void reset() { Z = target; }

    /**
     * @brief Update EWMA chart.
     * @return true if Z crosses the control limits.
     */
    bool update(double x) {
        Z = lambda * x + (1.0 - lambda) * Z;
        const double hw = L * sigma * std::sqrt(lambda / (2.0 - lambda));
        return std::abs(Z - target) >= hw;
    }

    double statistic() const { return Z; }
};

/**
 * @brief ControllerMonitor - observer that runs SPC charts on controller output.
 *
 * Listens to `onCompute()` for the scalar control output, and to `onState()` for
 * richer internal-state channels (e.g. ADRC ESO z3, SMC surface s).
 *
 * All charts share the same `target`, `sigma`, and callback settings for the
 * primary output.  Use `setWatchKey` / `setWatchIndex` to monitor an internal
 * state channel instead of the control output.
 */
class ControllerMonitor : public IControllerObserver {
public:
    // ---- Configuration -------------------------------------------------------

    /** @brief Expected mean of the monitored quantity. Default: 0. */
    void setTarget(double mu0) {
        cusum_.target = mu0;
        ewma_.target  = mu0;
        ewma_.Z       = mu0;   // re-initialise EWMA at new target
    }

    /** @brief Expected standard deviation of the monitored quantity. Default: 1. */
    void setSigma(double s) {
        cusum_.sigma = s;
        ewma_.sigma  = s;
    }

    /** @brief Directly set CUSUM parameters (k, h in sigma units). */
    void setCUSUMParams(double k, double h) { cusum_.k = k; cusum_.h = h; }

    /** @brief Directly set EWMA parameters (lambda, L). */
    void setEWMAParams(double lambda, double L) {
        ewma_.lambda = lambda;
        ewma_.L      = L;
        ewma_.reset();
    }

    /**
     * @brief Watch an `onState` channel instead of `onCompute`.
     * @param key    The key emitted by the controller (e.g. "eso", "surface").
     * @param index  Element index of the vector to monitor (default 0).
     */
    void setWatchKey(std::string key, int index = 0) {
        watch_key_   = std::move(key);
        watch_index_ = index;
    }

    /**
     * @brief Set alarm callback f(chart_name, statistic).
     * Called on the same thread as compute(), so keep it short.
     */
    void setAlarmCallback(AlarmCallback cb) { alarm_cb_ = std::move(cb); }

    // ---- IControllerObserver -------------------------------------------------

    void onCompute(double u, double /*signal*/) override {
        if (watch_key_.empty())   // monitor control output
            feed(u);
    }

    void onState(std::string_view key, const Eigen::VectorXd& value) override {
        if (!watch_key_.empty() && key == watch_key_) {
            if (watch_index_ < value.size())
                feed(value(watch_index_));
        }
    }

    void onReset() override {
        cusum_.reset();
        ewma_.reset();
        n_samples_    = 0;
        n_alarms_     = 0;
    }

    // ---- Diagnostics ---------------------------------------------------------

    /** @brief Number of observations processed since last reset. */
    int  nSamples() const noexcept { return n_samples_; }

    /** @brief Total alarm events fired since last reset. */
    int  nAlarms()  const noexcept { return n_alarms_; }

    /** @brief Current CUSUM statistic max(C+, C-). */
    double cusumStat() const noexcept { return cusum_.statistic(); }

    /** @brief Current EWMA statistic Z[k]. */
    double ewmaStat()  const noexcept { return ewma_.statistic(); }

    /** @brief Read-only access to the CUSUM chart. */
    const CUSUMChart& cusum() const noexcept { return cusum_; }
    /** @brief Read-only access to the EWMA chart. */
    const EWMAChart&  ewma()  const noexcept { return ewma_; }

private:
    void feed(double x) {
        ++n_samples_;
        bool alarm_cusum = cusum_.update(x);
        bool alarm_ewma  = ewma_.update(x);
        if (alarm_cusum && alarm_cb_) {
            ++n_alarms_;
            alarm_cb_("CUSUM", cusum_.statistic());
        }
        if (alarm_ewma && alarm_cb_) {
            ++n_alarms_;
            alarm_cb_("EWMA", ewma_.statistic());
        }
    }

    CUSUMChart    cusum_;
    EWMAChart     ewma_;
    AlarmCallback alarm_cb_;
    std::string   watch_key_;
    int           watch_index_ = 0;
    int           n_samples_   = 0;
    int           n_alarms_    = 0;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(controller_monitor)
