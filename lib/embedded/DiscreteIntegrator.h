#pragma once

namespace ctrl_embedded {

/**
 * @brief Discrete-time integrator using backward Euler.
 *
 * Suitable for embedded targets with no heap, no virtual dispatch, and no Eigen.
 * Works with float (32-bit MCU FPU) or double.
 *
 * @code
 *   DiscreteIntegrator<float> I(0.01f);   // Ts = 10 ms
 *   float out = I.integrate(error);        // out = sum(error * Ts)
 * @endcode
 */
template <typename Scalar = float>
class DiscreteIntegrator
{
public:
    explicit DiscreteIntegrator(Scalar sample_time) : Ts_(sample_time), acc_(Scalar(0)) {}

    /** @brief Accumulate one sample. Returns current integral value. */
    Scalar integrate(Scalar input)
    {
        acc_ += input * Ts_;
        return acc_;
    }

    /** @brief Current accumulator value (same as last integrate() return). */
    Scalar value() const { return acc_; }

    /** @brief Reset accumulator to zero. */
    void reset() { acc_ = Scalar(0); }

    /** @brief Set accumulator to a specific value (bumpless transfer). */
    void set(Scalar v) { acc_ = v; }

private:
    Scalar Ts_;
    Scalar acc_;
};

} // namespace ctrl_embedded
