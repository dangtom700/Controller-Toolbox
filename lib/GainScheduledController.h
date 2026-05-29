#pragma once
#include "IController.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <stdexcept>

/**
 * @file GainScheduledController.h
 * @brief Gain-scheduled IController wrapper for nonlinear / LPV plants.
 *
 * Stores a sorted list of (p, IController) pairs. Before calling compute(),
 * update the current scheduling variable via setSchedulingParam(). Two
 * interpolation modes are available:
 *
 *  - NearestNeighbor: only the closest schedule point is evaluated. No
 *    spurious state evolution in idle controllers. Best for hard-switching
 *    designs (e.g. supervisor-style gain scheduling).
 *
 *  - LinearBlend: both controllers bracketing the current p are evaluated
 *    and their outputs are weighted by proximity. Produces smooth output
 *    trajectories, but advances the state of both adjacent controllers.
 *    Best suited for stateless or near-stateless controllers (LQR K*x,
 *    pure-proportional), or when integral windup is actively managed.
 *
 * @par Typical workflow
 * @code
 *   auto sched = std::make_shared<GainScheduledController>(Ts);
 *   sched->addSchedulePoint(0.0, make_shared<DiscreteLQR>(sys0, lqr_p));
 *   sched->addSchedulePoint(0.5, make_shared<DiscreteLQR>(sys1, lqr_p));
 *   sched->addSchedulePoint(1.0, make_shared<DiscreteLQR>(sys2, lqr_p));
 *
 *   // In the control loop
 *   sched->setSchedulingParam(p_measured);
 *   double u = sched->compute(error);
 * @endcode
 *
 * @note sign convention: error = reference - measurement (same as DiscretePID).
 *
 * @see buildAutoGainScheduler() in AutoGainScheduler.h for automated construction
 *      from a nonlinear plant model using gap-metric clustering.
 */

namespace ctrl {

/** @brief Interpolation strategy for GainScheduledController. */
enum class GainScheduleMode {
    NearestNeighbor, ///< Hard-switch: only one controller is active at a time.
    LinearBlend      ///< Weighted-average: both bracketing controllers compute.
};

/**
 * @brief IController subclass that interpolates a set of linear controllers
 *        parameterised by a scalar scheduling variable p.
 */
class GainScheduledController : public IController
{
public:
    /**
     * @brief Construct a gain-scheduled controller.
     * @param Ts    Sample time [s] (passed through to sampleTime()).
     * @param mode  Interpolation mode (default LinearBlend).
     */
    explicit GainScheduledController(double Ts,
                                     GainScheduleMode mode = GainScheduleMode::LinearBlend)
        : Ts_(Ts), mode_(mode), current_p_(0.0), last_output_(0.0), active_idx_(0)
    {}

    /**
     * @brief Register a controller at a scheduling parameter value.
     *
     * Points are kept sorted by p. Duplicate p values are allowed but
     * discouraged; the new point is inserted immediately after any existing
     * point with the same p.
     *
     * @param p     Scheduling parameter value (any physical unit).
     * @param ctrl  Pre-constructed controller (shared ownership).
     */
    void addSchedulePoint(double p, std::shared_ptr<IController> ctrl) {
        SchedulePoint sp{p, std::move(ctrl)};
        auto it = std::lower_bound(schedule_.begin(), schedule_.end(), sp,
            [](const SchedulePoint& a, const SchedulePoint& b){ return a.p < b.p; });
        schedule_.insert(it, std::move(sp));
    }

    /**
     * @brief Set the current scheduling parameter before compute().
     * @param p  Current value of the scheduling variable.
     */
    void setSchedulingParam(double p) noexcept { current_p_ = p; }

    /** @return Current scheduling parameter value. */
    double schedulingParam() const noexcept { return current_p_; }

    /** @return Number of registered schedule points. */
    int numPoints() const noexcept { return static_cast<int>(schedule_.size()); }

    /** @return Scheduling parameter value at index i (0-based). */
    double pointP(int i) const { return schedule_.at(i).p; }

    /** @return Shared pointer to the controller at schedule index i. */
    std::shared_ptr<IController> pointController(int i) const {
        return schedule_.at(i).ctrl;
    }

    // -----------------------------------------------------------------------
    // IController interface
    // -----------------------------------------------------------------------

    /**
     * @brief Compute control output by interpolating between schedule points.
     *
     * Finds the two schedule points bracketing current_p_ and either hard-switches
     * (NearestNeighbor) or blends outputs (LinearBlend).
     *
     * @param error  Tracking error (reference - measurement).
     * @return       Interpolated control output.
     */
    double compute(double error) override {
        if (schedule_.empty()) { last_output_ = 0.0; return 0.0; }
        if (schedule_.size() == 1) {
            last_output_ = schedule_[0].ctrl->compute(error);
            return last_output_;
        }

        int lo = lowerIndex(current_p_);
        int hi = std::min(lo + 1, static_cast<int>(schedule_.size()) - 1);

        if (mode_ == GainScheduleMode::NearestNeighbor) {
            // Pick whichever neighbour is closer in p-space
            int idx = (std::abs(current_p_ - schedule_[lo].p) <=
                       std::abs(current_p_ - schedule_[hi].p)) ? lo : hi;
            active_idx_ = idx;
            last_output_ = schedule_[idx].ctrl->compute(error);
            return last_output_;
        }

        // LinearBlend
        if (lo == hi) {
            last_output_ = schedule_[lo].ctrl->compute(error);
            return last_output_;
        }
        double p0 = schedule_[lo].p, p1 = schedule_[hi].p;
        double alpha = (p1 > p0) ? (current_p_ - p0) / (p1 - p0) : 0.0;
        alpha = std::max(0.0, std::min(1.0, alpha));
        double u0 = schedule_[lo].ctrl->compute(error);
        double u1 = schedule_[hi].ctrl->compute(error);
        last_output_ = (1.0 - alpha) * u0 + alpha * u1;
        return last_output_;
    }

    /** @brief Reset all registered controllers and internal state. */
    void reset() override {
        for (auto& sp : schedule_) sp.ctrl->reset();
        last_output_ = 0.0;
        active_idx_ = 0;
    }

    double sampleTime() const override { return Ts_; }
    /** @return Most recent control output (not part of IController interface). */
    double lastOutput()  const { return last_output_; }

private:
    struct SchedulePoint {
        double p;
        std::shared_ptr<IController> ctrl;
    };

    std::vector<SchedulePoint> schedule_;  // sorted by p
    double            Ts_;
    GainScheduleMode  mode_;
    double            current_p_;
    double            last_output_;
    int               active_idx_;

    // Returns index of the largest p_i <= current_p (clamped to valid range).
    int lowerIndex(double p) const noexcept {
        if (p <= schedule_.front().p) return 0;
        if (p >= schedule_.back().p)
            return static_cast<int>(schedule_.size()) - 1;
        int lo = 0, hi = static_cast<int>(schedule_.size()) - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (schedule_[mid].p <= p) lo = mid; else hi = mid;
        }
        return lo;
    }
};

} // namespace ctrl
