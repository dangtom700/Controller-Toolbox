#pragma once
#include "IActuator.h"
#include "SimPlant.h"
#include <cmath>
#include <limits>

/**
 * @file SimActuator.h
 * @brief IActuator adapter backed by a SimPlant.
 *
 * `write(u)` applies the control input to the plant with two safety guards:
 * - **NaN/Inf rejection** — non-finite values are discarded and the last valid output
 *   is repeated. This mirrors the "hold-last" fail-safe behaviour of real hardware.
 * - **Saturation** — an optional [uMin, uMax] clamp (default: ±∞) is applied before
 *   forwarding to the plant, mirroring physical actuator limits.
 *
 * `lastOutput()` returns the most recent finite, saturated value actually applied.
 */

namespace ctrl {

/**
 * @brief IActuator implementation that drives a SimPlant.
 */
class SimActuator : public IActuator {
public:
    /**
     * @brief Construct the actuator adapter.
     * @param plant SimPlant instance (must outlive this object).
     * @param uMin  Lower saturation limit (default: −∞).
     * @param uMax  Upper saturation limit (default: +∞).
     */
    explicit SimActuator(SimPlant& plant,
                         double uMin = -std::numeric_limits<double>::infinity(),
                         double uMax =  std::numeric_limits<double>::infinity())
        : plant_(&plant), u_last_(0.0), u_min_(uMin), u_max_(uMax)
    {}

    /**
     * @brief Apply @p u to the plant after NaN/Inf rejection and saturation.
     *
     * Non-finite inputs are replaced with the last valid output (hold-last).
     * Saturated inputs are clamped to [uMin, uMax].
     *
     * @param u Desired control input.
     */
    void write(double u) override
    {
        if (!std::isfinite(u)) u = u_last_;
        if (u < u_min_) u = u_min_;
        if (u > u_max_) u = u_max_;
        u_last_ = u;
        plant_->step(u);
    }

    /**
     * @brief Return the most recent value successfully applied to the plant.
     * @return Last finite, saturated u.
     */
    double lastOutput() const override { return u_last_; }

    /**
     * @brief Reset last output to zero (does not reset the plant).
     */
    void reset() override { u_last_ = 0.0; }

private:
    SimPlant* plant_;   ///< Non-owning pointer; lifetime managed by the caller.
    double    u_last_;
    double    u_min_;
    double    u_max_;
};

} // namespace ctrl
