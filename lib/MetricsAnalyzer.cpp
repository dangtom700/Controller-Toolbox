#include "MetricsAnalyzer.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace ctrl
{

    TimeDomainMetrics MetricsAnalyzer::calculate(const std::vector<double> &t_data,
                                                 const std::vector<double> &y_data,
                                                 double reference,
                                                 int finalValueWindow)
    {
        if (t_data.size() != y_data.size() || t_data.empty())
        {
            throw std::invalid_argument("MetricsAnalyzer: time and output arrays must be equal and non-empty.");
        }

        TimeDomainMetrics m = {0.0, 0.0, 0.0, 0.0};
        int N = static_cast<int>(y_data.size());

        // Calculate final value
        double final_val = 0.0;
        int window = std::min(N, finalValueWindow);
        for (int i = N - window; i < N; ++i)
        {
            final_val += y_data[i];
        }
        final_val /= window;

        m.steadyStateError = std::abs(reference - final_val);

        // Peak Overshoot
        double max_val = *std::max_element(y_data.begin(), y_data.end());
        if (reference > 0 && max_val > reference)
        {
            m.peakOvershoot = ((max_val - reference) / reference) * 100.0;
        }
        else
        {
            m.peakOvershoot = 0.0;
        }

        // Settling Time (2% band) - scan backward to find last sample outside the band
        double lower_bound = reference * 0.98;
        double upper_bound = reference * 1.02;
        int settle_idx = -1; // -1 = signal never entered the 2% band
        for (int i = N - 1; i >= 0; --i)
        {
            if (y_data[i] < lower_bound || y_data[i] > upper_bound)
            {
                settle_idx = i + 1; // first sample that stays inside from here on
                break;
            }
        }
        if (settle_idx < 0 || settle_idx >= N)
            m.settlingTime = std::numeric_limits<double>::quiet_NaN(); // never settled
        else
            m.settlingTime = t_data[settle_idx];

        // Rise Time (10% to 90% of final_val)
        double val_10 = 0.1 * final_val;
        double val_90 = 0.9 * final_val;
        double t_10 = 0.0;
        double t_90 = 0.0;
        bool found_10 = false, found_90 = false;

        for (int i = 0; i < N; ++i)
        {
            if (!found_10 && y_data[i] >= val_10)
            {
                t_10 = t_data[i];
                found_10 = true;
            }
            if (!found_90 && y_data[i] >= val_90)
            {
                t_90 = t_data[i];
                found_90 = true;
                break;
            }
        }
        m.riseTime = (found_10 && found_90) ? (t_90 - t_10) : 0.0;

        return m;
    }

} // namespace ctrl
