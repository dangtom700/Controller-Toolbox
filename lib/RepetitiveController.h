#pragma once
#include "IController.h"
#include <Eigen/Dense>
#include <memory>
#include <vector>

/**
 * @file RepetitiveController.h
 * @brief Discrete-time Plug-in Repetitive Controller (RC).
 *
 * Cancels periodic references and disturbances with known period T = N·Ts by wrapping
 * any existing IController (base) and adding a learned periodic correction:
 *
 * @code
 *   v[k]  = Q·v[k−N] + Krc·e[k]   (repetitive learning law)
 *   u[k]  = u_base[k] + v[k]       (plug-in addition to base controller output)
 * @endcode
 *
 * @p v is a circular buffer of length N. Each element accumulates over periods:
 * - Q = 1: perfect cancellation after sufficient learning, zero robustness margin.
 * - Q < 1: exponential forgetting; suppresses model-mismatch instability.
 *
 * **Stability condition (sufficient, SISO linear base):**
 * The closed loop with the base controller must be stable, and
 * Q / |1 + L(e^jω)·P(e^jω)| < 1 for all ω (internal model robustness test).
 *
 * @code
 * // Typical usage
 * auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);
 * auto rc  = ctrl::RepetitiveController(pid, {100, 0.5, 0.98}, Ts);
 * double u = rc.compute(error);   // replaces pid.compute(error)
 * @endcode
 *
 * @see Hara et al., "Repetitive Control System: A New Type Servo System", IEEE TAC (1988).
 * @see Longman, "Iterative Learning and Repetitive Control" (2000).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for RepetitiveController.
 */
struct RepetitiveParams
{
    int    periodSteps = 100;  ///< Period N in samples (T = N·Ts).
    double Krc         = 0.5;  ///< Learning gain. Small = slow but stable. Typical: 0.3–0.8.
    double Q           = 0.98; ///< Forgetting/robustness factor ∈ (0, 1]. Typical: 0.95–1.0.
    double uMin        = -1e9; ///< Output saturation lower limit.
    double uMax        =  1e9; ///< Output saturation upper limit.
};

/**
 * @brief Discrete-time Plug-in Repetitive Controller.
 *
 * Inherits from IController so it can be used directly in ControllerStack.
 */
class RepetitiveController : public IController
{
public:
    /**
     * @brief Construct the repetitive controller.
     * @param inner  Base stabilising controller (must already close the loop adequately).
     * @param params Repetitive learning parameters (period, gain, forgetting factor, limits).
     * @param Ts     Sample time [s].
     */
    RepetitiveController(std::shared_ptr<IController> inner,
                         const RepetitiveParams &params,
                         double Ts);

    /**
     * @brief Compute base control + repetitive correction, clamped to [uMin, uMax].
     * @param error Current tracking error e[k].
     * @return Composite control output u[k].
     */
    double compute(double error) override;

    /** @brief Reset circular learning buffer and inner controller state. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Update parameters and resize the learning buffer if periodSteps changes.
     * @param p New repetitive parameters.
     */
    void setParams(const RepetitiveParams &p);

    /** @brief Read-only access to current parameters. */
    const RepetitiveParams &params() const { return p_; }

    /**
     * @brief Current repetitive correction term v[k].
     *
     * Useful for monitoring learning convergence. Should approach the negative of the
     * periodic disturbance after sufficient repetitions.
     */
    double correction() const { return v_now_; }

    /**
     * @brief Access the wrapped inner controller for runtime tuning.
     * @return Mutable reference to the base controller.
     */
    IController &innerController() { return *inner_; }

private:
    std::shared_ptr<IController> inner_;
    RepetitiveParams p_;
    double Ts_;
    std::vector<double> v_buf_; ///< Circular learning buffer (length N).
    int buf_idx_;               ///< Write pointer (wraps mod N).
    double v_now_;              ///< Most recent correction (for diagnostics).
};

} // namespace ctrl
