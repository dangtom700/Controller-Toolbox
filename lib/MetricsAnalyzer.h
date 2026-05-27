#pragma once
#include <vector>

/**
 * @file MetricsAnalyzer.h
 * @brief Time-domain step-response metric extraction.
 */

namespace ctrl
{

/**
 * @brief Standard step-response performance metrics.
 */
struct TimeDomainMetrics
{
    double riseTime;         ///< 10%–90% rise time [s]; −1 if not reached within the data window.
    double settlingTime;     ///< Time to enter and stay within ±2% of the final value [s]; −1 if not settled.
    double peakOvershoot;    ///< (y_max − y_final) / y_final × 100 [%]; 0 if there is no overshoot.
    double steadyStateError; ///< |reference − y_final| at the end of the data [same units as y].
};

/**
 * @brief Extracts standard step-response metrics from sampled time-series data.
 */
class MetricsAnalyzer
{
public:
    /**
     * @brief Calculate step-response metrics from time-series data.
     *
     * @param t_data           Time vector [s], strictly increasing.
     * @param y_data           Output vector, same length as t_data.
     * @param reference        Target steady-state value (used for 10/90% thresholds and SSE).
     * @param finalValueWindow Number of trailing samples averaged to estimate the final settled value.
     *                         Increase for noisy data; decrease for short records.
     * @return TimeDomainMetrics struct. riseTime and settlingTime are −1 when the response does not
     *         cross the threshold within the provided data window.
     */
    static TimeDomainMetrics calculate(const std::vector<double> &t_data,
                                       const std::vector<double> &y_data,
                                       double reference = 1.0,
                                       int finalValueWindow = 10);
};

} // namespace ctrl
