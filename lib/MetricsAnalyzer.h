#pragma once
#include <vector>

namespace ctrl
{

    struct TimeDomainMetrics
    {
        double riseTime;         // 10%-to-90% rise time [s]; -1 if not reached
        double settlingTime;     // time to enter and stay within +/-2% of final value [s]; -1 if not settled
        double peakOvershoot;    // (y_max - y_final) / y_final * 100 [%]; 0 if no overshoot
        double steadyStateError; // |reference - y_final| at end of data [same units as y]
    };

    class MetricsAnalyzer
    {
    public:
        // Extract standard step-response metrics from time-series data.
        //
        // t_data:           time vector [s], strictly increasing
        // y_data:           output vector, same length as t_data
        // reference:        target steady-state value (used for 10/90% thresholds and SSE)
        // finalValueWindow: number of trailing samples averaged for the final settled value;
        //                   increase for noisy data, decrease for short records
        //
        // riseTime and settlingTime return -1.0 when the response does not cross the
        // threshold within the provided data window.
        static TimeDomainMetrics calculate(const std::vector<double> &t_data,
                                           const std::vector<double> &y_data,
                                           double reference = 1.0,
                                           int finalValueWindow = 10);
    };

} // namespace ctrl
