#pragma once
#include "IController.h"
#include "RecursiveLeastSquares.h"
#include <Eigen/Dense>
#include <random>
#include <string>

/**
 * @file SelfTuningRegulator.h
 * @brief Self-tuning regulator (STR) - online RLS identification + minimum-variance or
 *        pole-placement control law, re-derived every step from the latest parameter estimate.
 *
 * Merges three `docs/algorithm_backlog.md` lines (minimum-variance control, adaptive pole
 * placement, self-tuning regulators) into one `IController`: a single `RecursiveLeastSquares`
 * identifier drives either of two selectable control laws (Phase 3 Roadmap OC1).
 *
 * **Plant convention (identical to RecursiveLeastSquares):**
 * @code
 *   A(q^-1) y[k] = B(q^-1) u[k] + e[k]
 *   A(q^-1) = 1 + a1.q^-1 + ... + a_na.q^-na      (monic)
 *   B(q^-1) = b1.q^-1 + ... + b_nb.q^-nb           (no b0 term - inherent one-step delay)
 * @endcode
 *
 * **Usage (ADRC/MRAC-style convention: compute() takes plant output, not error):**
 * @code
 *   ctrl::STRParams sp;
 *   sp.na = 2; sp.nb = 1; sp.mode = ctrl::STRMode::MinimumVariance;
 *   ctrl::SelfTuningRegulator str(sp, Ts);
 *   for (int k = 0; k < N; ++k) {
 *       str.setReference(r);
 *       double u = str.compute(y);   // updates RLS, re-derives the control law, returns u[k]
 *   }
 * @endcode
 *
 * @see Astrom & Wittenmark, "Adaptive Control", 2nd ed. (1995), Ch. 3 (minimum variance) and
 *      Ch. 7 (pole placement self-tuners).
 * @see RecursiveLeastSquares.h - the online ARX identifier this class wraps.
 * @see docs/superpowers/specs/2026-06-25-adaptive-identification-design.md
 *
 * @warning **Deadbeat (or near-deadbeat) `desired_poles`/minimum-variance targets are fragile
 * during the online-identification transient.** Both control laws divide by an identified
 * leading coefficient (`b1` or `r0`); while RLS is still converging, small parameter errors get
 * amplified enormously by an aggressive (deadbeat) design, which can saturate `u` at `uMin`/
 * `uMax` and feed a corrupted (clipped) data point back into RLS - a feedback loop that can
 * prevent convergence entirely. Always set realistic `uMin`/`uMax` (matching real actuator
 * limits, never the defaults' +-1e9) and prefer comfortably-damped `desired_poles` over the
 * origin unless the plant model is already known very precisely.
 *
 * @warning **Closed-loop operation does not generally guarantee persistent excitation, even
 * with a randomly-varying reference and `STRParams::probeAmplitude` dither.** Certainty-
 * equivalence direct adaptive control (both modes) can converge confidently (shrinking
 * `covariance()`) to a *stabilizing but numerically wrong* parameter estimate - a well-known,
 * textbook-documented property of this algorithm class (Astrom & Wittenmark Ch. 3), not a bug.
 * `probeAmplitude` helps in many practical cases but is not a guarantee. If guaranteed parameter
 * recovery matters (not just a stabilizing controller), identify the plant open-loop first, or
 * see the (separately scoped, not implemented here) dual-control approach that deliberately
 * trades exploration against exploitation.
 */

namespace ctrl
{

/** @brief Control-law mode for SelfTuningRegulator. */
enum class STRMode
{
    MinimumVariance, ///< One-step-ahead direct-cancellation STR (Astrom Ch. 3).
    PolePlacement     ///< Diophantine-equation pole-placement self-tuner (Astrom Ch. 7).
};

/** @brief Parameters for SelfTuningRegulator. */
struct STRParams
{
    int na = 2; ///< Plant A-polynomial order (RecursiveLeastSquares convention).
    int nb = 1; ///< Plant B-polynomial order (RecursiveLeastSquares convention).

    STRMode mode = STRMode::MinimumVariance;

    /**
     * @brief Desired closed-loop poles (PolePlacement mode only).
     * Size MUST equal na + nb - 1 (the Diophantine equation's degree-matching requirement);
     * the constructor throws std::invalid_argument otherwise. Unused in MinimumVariance mode.
     */
    Eigen::VectorXd desired_poles;

    double lambda = 0.98; ///< RLS forgetting factor, forwarded to RecursiveLeastSquares.

    /**
     * @brief Leading-coefficient floor (|b1| for MinimumVariance, |r0|/|B(1)| for PolePlacement).
     * Below this, the control law is ill-conditioned for direct cancellation; compute() holds
     * the last output instead of dividing by a near-zero value.
     */
    double bMin = 1e-6;

    double uMin = -1e9; ///< Output saturation lower bound.
    double uMax =  1e9; ///< Output saturation upper bound.

    /**
     * @brief Persistent probing/dither amplitude added to u before saturation (0 = disabled).
     *
     * Certainty-equivalence direct adaptive control (both modes) has no general guarantee of
     * persistent excitation from the reference alone (Astrom & Wittenmark Ch. 3/7): the closed
     * loop can converge confidently to a *stabilizing but incorrect* parameter estimate that
     * still drives the identification covariance toward zero, leaving a steady-state tracking
     * error. A small zero-mean uniform dither in [-probeAmplitude, +probeAmplitude], added
     * *before* RLS sees the applied u (so identification and control stay consistent), is the
     * standard textbook remedy. Set to a small fraction of the expected input range (e.g. 5-10%)
     * when persistent excitation from the reference isn't already guaranteed.
     */
    double probeAmplitude = 0.0;

    unsigned probeSeed = 1; ///< RNG seed for the probing dither.
};

/**
 * @brief Online self-tuning regulator: RLS identification + minimum-variance/pole-placement law.
 *
 * Implements `IController` with the `PlantOutput` sign convention (like `MRACController`):
 * call `setReference(r)` once per step, then `compute(y_plant)`.
 */
class SelfTuningRegulator : public IController
{
public:
    /**
     * @brief Construct the regulator.
     * @param params STR tuning parameters.
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument if na<1, nb<1, Ts<=0, uMin>=uMax, bMin<=0, or (PolePlacement
     *         mode) desired_poles.size() != na+nb-1.
     */
    SelfTuningRegulator(const STRParams &params, double Ts);

    /** @brief Set the current reference r[k]. Must be called before each compute(). */
    void setReference(double r) { r_ = r; }

    /**
     * @brief Advance one step: refresh the RLS estimate, re-derive u[k] from the selected
     *        control law, and update internal history.
     * @param y_plant Current plant output y[k].
     * @return Control action u[k].
     */
    double compute(double y_plant) override;

    void reset() override;

    double sampleTime() const override { return Ts_; }

    SignConvention signConvention() const override { return SignConvention::PlantOutput; }

    std::string name() const override { return "SelfTuningRegulator"; }

    /** @brief Current estimated B-polynomial [b1..b_nb] (mirrors RecursiveLeastSquares). */
    Eigen::VectorXd estimatedNumerator() const { return rls_.numerator(); }

    /** @brief Current estimated A-polynomial [1,a1..a_na] (mirrors RecursiveLeastSquares). */
    Eigen::VectorXd estimatedDenominator() const { return rls_.denominator(); }

    /** @brief RLS parameter covariance - decreasing trace indicates converging identification. */
    const Eigen::MatrixXd &covariance() const { return rls_.covariance(); }

    /** @brief Read-only access to the tuning parameters. */
    const STRParams &params() const { return p_; }

private:
    double computeMinimumVariance();
    double computePolePlacement();
    double fallbackProportional() const;

    RecursiveLeastSquares rls_;
    STRParams p_;
    double Ts_;
    double r_ = 0.0;
    double uPrev_ = 0.0;
    bool havePrevU_ = false;

    // Pre-sized circular history, construction-time allocation only (no push_back/resize in
    // compute() - mirrors RecursiveLeastSquares's own y_buf_/u_buf_ style).
    // yHist_[0] = y[k] (current, just observed) .. yHist_[na-1] = y[k-na+1].
    Eigen::VectorXd yHist_;
    // uHist_[0] = u[k-1] .. uHist_[nb-2] = u[k-nb+1]. Empty when nb == 1.
    Eigen::VectorXd uHist_;

    std::mt19937 probeRng_;
    std::uniform_real_distribution<double> probeDist_{-1.0, 1.0};
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(self_tuning_regulator)
