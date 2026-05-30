#pragma once
#include "IController.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

/**
 * @file AntiWindupWrapper.h
 * @brief Generic anti-windup decorator for any IController.
 *
 * Wraps an arbitrary IController with output saturation and the Hanus (1987)
 * conditioning technique, which prevents integrator windup in controllers that
 * lack built-in anti-windup (e.g., DiscreteLQG, DiscreteHinf, DiscreteMPC
 * without explicit u-constraints).
 *
 * **Conditioning technique (Astrom & Wittenmark, "Computer Controlled Systems"
 * 3rd ed., Section 8.5):**
 *
 *   1. Augment the incoming error with the previous saturation correction:
 *        e_in[k] = e[k] + c[k-1]
 *   2. Compute the raw (unsaturated) output from the inner controller:
 *        u_raw[k] = C(z) * e_in[k]
 *   3. Saturate to actuator limits:
 *        u[k] = clamp(u_raw[k], uMin, uMax)
 *   4. Compute the correction for the next step:
 *        c[k] = Kb * (u[k] - u_raw[k])
 *
 * When unsaturated, c[k] = 0 and the wrapper is transparent (identity).
 * When saturated, the negative feedback correction c[k] progressively reduces
 * the augmented error e_in, steering the controller's internal state away from
 * the saturating region without requiring direct access to its integrators.
 *
 * **Kb tuning (Astrom & Wittenmark Section 3.5 analogy):**
 *   - Kb = 0   : pure output clamp; no conditioning. Windup is not prevented.
 *   - Kb = 1.0 : standard conditioning; good starting point for most plants.
 *   - Kb > 1   : aggressive reset; may excite high-frequency dynamics on exit
 *                from saturation.
 *
 * **When to use this wrapper vs. DiscretePID built-in anti-windup:**
 *   - DiscretePID already implements back-calculation natively (PIDParams::Kb).
 *     Use that directly; wrapping a PID with AntiWindupWrapper doubles the
 *     anti-windup correction and may destabilise the loop.
 *   - Use AntiWindupWrapper for DiscreteLQG, DiscreteHinf, NonlinearMPC
 *     (without u-bounds), GPC, and custom IController subclasses that have
 *     internal integrating states but no built-in saturation handling.
 *
 * **External actuator feedback:**
 *   Call setActualOutput(u_applied) *after* compute() each step when the actual
 *   applied output differs from the internally saturated value (e.g., hardware
 *   DAC clipping, external supervisory override). This feeds the true saturation
 *   error back into the conditioning correction.
 *
 * @see IController, DiscretePID (for PID-specific anti-windup)
 * @see Hanus R. (1987): "A new approach to the limitation problem of multivariable
 *      controllers", Control Engineering Practice.
 */

namespace ctrl {

/**
 * @brief Generic anti-windup decorator for any IController.
 *
 * Thread safety: not thread-safe. All calls must come from the same thread.
 */
class AntiWindupWrapper : public IController {
public:
    /**
     * @brief Construct an anti-windup wrapper around an existing controller.
     *
     * @param inner  The controller to wrap. Must not be nullptr. The wrapper
     *               co-owns the inner controller via shared_ptr.
     * @param uMin   Lower actuator saturation limit.
     * @param uMax   Upper actuator saturation limit. Must be > uMin.
     * @param Kb     Conditioning gain. 0 = pure clamp; 1.0 = standard
     *               conditioning (default). Must be >= 0.
     * @throws std::invalid_argument If inner is nullptr, uMin >= uMax, or Kb < 0.
     */
    AntiWindupWrapper(std::shared_ptr<IController> inner,
                      double uMin, double uMax, double Kb = 1.0)
        : inner_(std::move(inner)), uMin_(uMin), uMax_(uMax), Kb_(Kb)
    {
        if (!inner_)
            throw std::invalid_argument("AntiWindupWrapper: inner controller is nullptr.");
        if (uMin_ >= uMax_)
            throw std::invalid_argument("AntiWindupWrapper: uMin must be < uMax.");
        if (Kb_ < 0.0)
            throw std::invalid_argument("AntiWindupWrapper: Kb must be >= 0.");
    }

    /**
     * @brief Advance one step with saturation and conditioning.
     *
     * Applies the conditioning correction from the previous step, delegates to
     * the inner controller, clamps the output, and stores the correction for the
     * next call.
     *
     * @param error Tracking error e[k] = r[k] - y[k] (or plant output y[k]
     *              for optimisation-based controllers like ESC).
     * @return Saturated control output u[k] in [uMin, uMax].
     */
    double compute(double error) override
    {
        // Step 1: augment error with conditioning correction from previous step
        const double e_in = error + correction_;

        // Step 2: raw (unsaturated) output from inner controller
        const double u_raw = inner_->compute(e_in);

        // Step 3: saturate
        const double u_sat = std::clamp(u_raw, uMin_, uMax_);

        // Step 4: conditioning correction for next step
        correction_ = Kb_ * (u_sat - u_raw);

        saturated_ = (std::abs(u_sat - u_raw) > 1e-12);
        sat_error_ = u_sat - u_raw;
        u_last_    = u_sat;
        has_actual_ = false;

        notifyObserver(u_sat, error);
        return u_sat;
    }

    /**
     * @brief Reset the wrapper and the inner controller.
     *
     * Clears the conditioning correction, saturation flag, and last output.
     */
    void reset() override
    {
        inner_->reset();
        correction_ = 0.0;
        u_last_     = 0.0;
        saturated_  = false;
        sat_error_  = 0.0;
        has_actual_ = false;
        notifyObserverReset();
    }

    /** @return Sample time [s] inherited from the inner controller. */
    double sampleTime() const override { return inner_->sampleTime(); }

    /**
     * @brief Delegates health check to the inner controller.
     *
     * Returns false when the inner controller reports failure (e.g., QP
     * did not converge in DiscreteMPC, DARE did not converge in DiscreteLQR).
     */
    bool isHealthy() const override { return inner_->isHealthy(); }

    /**
     * @brief Bumpless initialisation - prepares the wrapper for smooth activation.
     *
     * Delegates to the inner controller's bumplessInit() and resets the
     * conditioning correction so the wrapper does not inject spurious error
     * on the first compute() call after a supervisory switch.
     *
     * @param u_target The control output the loop is currently delivering.
     * @param error    The current tracking error.
     */
    void bumplessInit(double u_target, double error) override
    {
        inner_->bumplessInit(u_target, error);
        correction_ = 0.0;
        saturated_  = false;
        sat_error_  = 0.0;
        u_last_     = u_target;
    }

    // -----------------------------------------------------------------------
    // Wrapper-specific accessors
    // -----------------------------------------------------------------------

    /**
     * @brief Last saturated output u[k] returned by compute().
     * @return u_last [engineering units - same as inner controller output].
     */
    double lastOutput() const { return u_last_; }

    /**
     * @brief True when the previous compute() call hit the saturation limits.
     *
     * Useful for logging and supervisory logic. The flag is cleared by reset().
     *
     * @return true if |u_raw - u_sat| > 1e-12 on the last step.
     */
    bool isSaturated() const { return saturated_; }

    /**
     * @brief Saturation error from the last step: u_sat[k] - u_raw[k].
     *
     * Negative when the output was clamped below uMax; positive when clamped
     * above uMin. Zero when not saturated.
     *
     * @return Saturation error [engineering units].
     */
    double saturationError() const { return sat_error_; }

    /**
     * @brief Feed back the true applied actuator output for external clamping.
     *
     * Call this *after* compute() when the actual output applied to the plant
     * differs from the internally clamped value (hardware DAC clipping, external
     * override, etc.). The wrapper re-computes the conditioning correction using
     * (u_applied - u_raw) instead of the internal clamp result.
     *
     * Must be called before the next compute(); its effect lasts one step.
     *
     * @param u_applied Actual output applied to the plant this step.
     */
    void setActualOutput(double u_applied)
    {
        // u_raw for the last step is: u_raw = u_last_ - sat_error_
        // (u_last_ was the clamped output, sat_error_ = u_sat - u_raw)
        const double u_raw = u_last_ - sat_error_;
        correction_ = Kb_ * (u_applied - u_raw);
        sat_error_  = u_applied - u_raw;
        saturated_  = (std::abs(sat_error_) > 1e-12);
        u_last_     = u_applied;
        has_actual_ = true;
    }

    /** @return Pointer to the inner controller (non-owning). */
    IController *inner() const { return inner_.get(); }

private:
    std::shared_ptr<IController> inner_;
    double uMin_;
    double uMax_;
    double Kb_;

    double correction_{0.0}; ///< Conditioning correction c[k] injected into next error.
    double u_last_{0.0};     ///< Last saturated output.
    double sat_error_{0.0};  ///< u_sat[k] - u_raw[k] from last step.
    bool   saturated_{false};
    bool   has_actual_{false};
};

} // namespace ctrl
