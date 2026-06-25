#pragma once
#include "IController.h"
#include "Features.h"
#include <Eigen/Dense>
#include <functional>
#include <vector>

/**
 * @file BacksteppingController.h
 * @brief Recursive Lyapunov design for strict-feedback nonlinear systems (Phase 3 NC1).
 *
 * Handles N-stage strict-feedback chains:
 * @code
 *   x1' = f_0(x) + g_0(x)*x2
 *   x2' = f_1(x) + g_1(x)*x3
 *   ...
 *   xN' = f_{N-1}(x) + g_{N-1}(x)*u
 * @endcode
 * (relative degree > 1 structures `FeedbackLinearisationController` - relative-degree-1
 * only - cannot handle directly).
 *
 * **Recursive control law** (1-indexed in the math, `z_i = x_i - alpha_{i-1}`,
 * `alpha_0 := r`):
 * @code
 *   alpha_i = (1/g_i) * ( -f_i - k_i*z_i + alpha_{i-1}' - [i>1] g_{i-1}*z_{i-1} )   (i < N)
 *   u = alpha_N
 * @endcode
 * `alpha_{i-1}'` (the previous stage's virtual-control derivative, needed for the Lyapunov
 * cross-term cancellation) is approximated via a backward finite difference over `Ts`
 * rather than computed analytically - this keeps the `DriftFn`/`GainFn` callback API as
 * simple as `FeedbackLinearisationController`'s (`(x, stage)`, no Jacobian callbacks needed)
 * at the cost of an O(Ts) lag in the cancellation term.
 *
 * @see docs/superpowers/specs/2026-06-24-nonlinear-control-trio-design.md
 */

namespace ctrl {

/** @brief Tuning parameters for BacksteppingController. */
struct BacksteppingParams
{
    std::vector<double> k_gains; ///< One stabilizing gain per recursion stage (size N).
    double uMin = -1e9, uMax = 1e9;
};

/**
 * @brief Recursive backstepping controller for an N-stage strict-feedback system.
 */
class BacksteppingController : public IController
{
public:
    /** @brief Drift term f_s(x), evaluated at the full state and the 0-indexed stage s. */
    using DriftFn = std::function<double(const Eigen::VectorXd &x, int stage)>;
    /** @brief Gain term g_s(x), evaluated at the full state and the 0-indexed stage s. */
    using GainFn  = std::function<double(const Eigen::VectorXd &x, int stage)>;

    /**
     * @brief Construct the backstepping controller.
     * @param f      Drift functors f_0..f_{N-1} (size N).
     * @param g      Gain functors g_0..g_{N-1} (size N), must be non-zero in the operating region.
     * @param params Per-stage gains (size N) and output saturation.
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument If `f`, `g`, and `params.k_gains` don't all have the
     *         same non-zero size.
     */
    BacksteppingController(std::vector<DriftFn> f, std::vector<GainFn> g,
                            const BacksteppingParams &params, double Ts);

    /**
     * @brief Compute one control step.
     * @param error Top-level tracking error e = r - x1.
     * @return Physical control signal u[k] (the final recursion stage's virtual control).
     * @note Call setState(x) before each compute() with the current plant state.
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }

    /** @brief Reset the finite-difference history and last output. Does not clear setState()'s x. */
    void reset() override;

    double sampleTime() const override { return Ts_; }

    /** @brief Inject the current plant state x[k] (size N). Must be called before compute(). */
    void setState(const Eigen::VectorXd &x) { x_ = x; }

    /** @brief Current state held by the controller (as last set by setState()). */
    const Eigen::VectorXd &state() const { return x_; }

private:
    std::vector<DriftFn> f_;
    std::vector<GainFn>  g_;
    BacksteppingParams   params_;
    double               Ts_;
    int                  N_;

    Eigen::VectorXd x_;
    double u_prev_ = 0.0;

    bool   initialized_ = false; ///< False until the first finite-difference history exists.
    double rPrev_ = 0.0;
    std::vector<double> alphaPrevStore_;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(backstepping_controller)
