#pragma once
#include <vector>

namespace ctrl
{

    struct TimeDomainMetrics
    {
        double riseTime;         // Time to go from 10% to 90% of final value
        double settlingTime;     // Time after which response stays within 2% of final value
        double peakOvershoot;    // Maximum percentage over the final value
        double steadyStateError; // Absolute error at the final recorded time
    };

    class MetricsAnalyzer
    {
    public:
        // Automatically extracts time-domain metrics from step response data.
        // reference: The target step value (e.g., 1.0)
        // finalValueWindow: The number of trailing samples to average for the final "settled" value
        static TimeDomainMetrics calculate(const std::vector<double> &t_data,
                                           const std::vector<double> &y_data,
                                           double reference = 1.0,
                                           int finalValueWindow = 10);
    };

} // namespace ctrl
