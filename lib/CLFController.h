#pragma once
#include "IController.h"
#include "Features.h"
#include <Eigen/Dense>
#include <functional>

/**
 * @file CLFController.h
 * @brief CLF synthesis via Sontag's universal formula (Phase 3 NC4).
 *
 * Given a candidate Control Lyapunov Function `V(x)` and its Lie derivatives, synthesizes a
 * stabilizing law toward `V`'s equilibrium via Sontag's universal formula - controller
 * *synthesis*, distinct from `LyapunovRobustness` (which only *analyzes* a fixed linear
 * system's robustness via polytopic vertex matrices, not a general nonlinear `V(x)`).
 *
 * **Constructor gap resolved:** the CLF decay condition `LfV + LgV*u <= -alpha*V` needs
 * `V(x)` itself, not just its Lie derivatives - a `VFn V` callback is therefore required
 * alongside `LfVFn`/`LgVFn`.
 *
 * **Control law (Sontag's universal formula):** with `a = LfV(x) + alpha*V(x)` and
 * `b = LgV(x)`,
 * @code
 *   u = -(a + sqrt(a^2 + b^4)) / b      if b != 0
 * @endcode
 * which is smooth and always satisfies `a + b*u <= 0` exactly (`a + b*u = -sqrt(a^2+b^4)`).
 * If `b == 0` (uncontrollable direction) and `a > 0` (drift not naturally decaying), the
 * problem is infeasible at this state - `isHealthy()` reports `false` and the last finite
 * output is held, rather than producing a divide-by-zero/nonsense result.
 *
 * **`useQP` note:** `GradientProjectionQP` solves box-constrained QPs only; the CLF-QP
 * problem's actual constraint is a half-space, not a box, so it cannot be expressed via that
 * solver directly. For the SISO scope here, `useQP=true` and `useQP=false` both run the
 * identical closed-form path above - `useQP` is kept for sketch-API compatibility and as a
 * hook for a future MIMO QP-based extension.
 *
 * @see docs/superpowers/specs/2026-06-24-nonlinear-control-trio-design.md
 */

namespace ctrl {

/** @brief Tuning parameters for CLFController. */
struct CLFParams
{
    double alpha = 1.0;        ///< Decay rate in LfV + LgV*u <= -alpha*V.
    double uMin = -1e9, uMax = 1e9;
    bool   useQP = true;       ///< No behavioral effect in v1 (SISO closed-form only); see header note.
};

/**
 * @brief CLF-synthesized controller via Sontag's universal formula (SISO).
 */
class CLFController : public IController
{
public:
    using VFn   = std::function<double(const Eigen::VectorXd &x)>;  ///< Candidate Lyapunov function V(x).
    using LfVFn = std::function<double(const Eigen::VectorXd &x)>;  ///< Drift Lie derivative.
    using LgVFn = std::function<double(const Eigen::VectorXd &x)>;  ///< Control Lie derivative.

    CLFController(VFn V, LfVFn LfV, LgVFn LgV, const CLFParams &params, double Ts);

    /**
     * @brief Compute one control step.
     * @param error Unused - CLFController regulates toward V's equilibrium using the state
     *              injected via setState(), not a tracking error (CLF synthesis is a
     *              regulation problem toward V's minimum, by construction).
     */
    double compute(double error) override;

    void setState(const Eigen::VectorXd &x) { x_ = x; }
    const Eigen::VectorXd &state() const { return x_; }

    void reset() override;
    double sampleTime() const override { return Ts_; }

    /** @brief False after a step where LgV(x) == 0 (uncontrollable) and the drift didn't decay. */
    bool isHealthy() const override { return healthy_; }

private:
    VFn    V_;
    LfVFn  LfV_;
    LgVFn  LgV_;
    CLFParams params_;
    double Ts_;

    Eigen::VectorXd x_;
    double u_prev_ = 0.0;
    bool   healthy_ = true;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(clf_controller)
