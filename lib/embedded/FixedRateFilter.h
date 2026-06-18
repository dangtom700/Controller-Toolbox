#pragma once
#include <cmath>

namespace ctrl_embedded {

/**
 * @brief Compile-time order IIR low-pass filter (backward Euler, cascade of first-order sections).
 *
 * No heap allocation. State is stored in a stack-allocated array.
 * Use `Order = 1` for a single-pole filter; `Order = 2` for a second-order cascade (two poles).
 *
 * @tparam Scalar  Numeric type (float or double).
 * @tparam Order   Filter order (number of first-order stages cascaded). Must be >= 1.
 *
 * @code
 *   FixedRateFilter<float, 2> lpf(100.0f, 0.01f);  // 100 Hz cutoff, Ts = 10 ms
 *   float y = lpf.filter(x);
 * @endcode
 */
template <typename Scalar = float, int Order = 1>
class FixedRateFilter
{
    static_assert(Order >= 1, "Filter Order must be >= 1");

public:
    /**
     * @brief Construct with cutoff frequency and sample time.
     * @param cutoff_hz Cutoff frequency [Hz].
     * @param sample_time Sample period Ts [s].
     */
    FixedRateFilter(Scalar cutoff_hz, Scalar sample_time)
    {
        const Scalar tau = Scalar(1) / (Scalar(2) * Scalar(3.14159265358979323846) * cutoff_hz);
        alpha_ = Scalar(1) / (Scalar(1) + tau / sample_time);  // backward Euler coefficient
        for (int i = 0; i < Order; ++i) state_[i] = Scalar(0);
    }

    /**
     * @brief Filter one sample through all Order stages.
     * @param input Raw input sample.
     * @return Filtered output.
     */
    Scalar filter(Scalar input)
    {
        Scalar x = input;
        for (int i = 0; i < Order; ++i) {
            state_[i] = state_[i] + alpha_ * (x - state_[i]);
            x = state_[i];
        }
        return x;
    }

    /** @brief Reset all filter states to zero. */
    void reset()
    {
        for (int i = 0; i < Order; ++i) state_[i] = Scalar(0);
    }

    /** @brief Current output (last filtered value). */
    Scalar value() const { return state_[Order - 1]; }

private:
    Scalar alpha_;
    Scalar state_[Order];
};

} // namespace ctrl_embedded
