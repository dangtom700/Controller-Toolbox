#pragma once
#include "IController.h"
#include "Features.h"
#include <Eigen/Dense>
#include <functional>

/**
 * @file NonlinearIMC.h
 * @brief Nonlinear Internal Model Control (Phase 3 NC3).
 *
 * The nonlinear analogue of the IMC structure `SmithPredictor` / SOPDT-Rivera-IMC provide for
 * linear plants: a user-supplied nonlinear one-step process model runs in parallel with the
 * plant, the controller inverts that model for feedforward, and the model-mismatch residual
 * `d = y - y_model` is fed back so steady-state offset and unmodelled dynamics are rejected.
 *
 * **Per-step law** (error convention `e = r - y`, classic IMC interconnection):
 * @code
 *   s[k]   = e[k] + y_model[k-1]          (= r - d, with d = y - y_model the model mismatch)
 *   sf[k]  = lambda . sf[k-1] + (1-lambda) . s[k]   (first-order IMC filter, robustness)
 *   u[k]   = clamp( inverse(x, sf[k]), uMin, uMax )  (invert the model to hit the filtered target)
 *   y_model[k] = model(x, u[k])           (advance the internal model)
 * @endcode
 *
 * **Exact-match property:** when the model equals the plant, `d == 0` and the loop tracks the
 * reference through the filter with no offset - IMC's defining property.
 *
 * @see SmithPredictor.h (linear model-in-the-loop structure this generalises).
 * @see docs/ALGORITHM_ROADMAP_PHASE3.md (NC3)
 */

namespace ctrl {

/** @brief Parameters for NonlinearIMC. */
struct NonlinearIMCParams
{
    double filter_lambda = 0.5; ///< IMC filter pole in [0,1): larger = slower/more robust.
    double uMin = -1e9;         ///< Output lower saturation limit.
    double uMax =  1e9;         ///< Output upper saturation limit.
};

/**
 * @brief Nonlinear Internal Model Controller.
 */
class NonlinearIMC : public IController
{
public:
    /** @brief One-step process model: y_hat = model(x, u). */
    using ModelFn = std::function<double(const Eigen::VectorXd &x, double u)>;
    /** @brief Model inverse: u that drives model(x, u) to y_target. */
    using InverseModelFn = std::function<double(const Eigen::VectorXd &x, double y_target)>;

    /**
     * @brief Construct the nonlinear IMC controller.
     * @param model   One-step process model y_hat = f(x, u).
     * @param inverse Model inverse u = f^{-1}(x, y_target).
     * @param params  IMC filter pole + output saturation.
     * @param Ts      Sample time [s].
     * @throws std::invalid_argument If `model`/`inverse` are empty, Ts <= 0,
     *         filter_lambda not in [0,1), or uMin >= uMax.
     */
    NonlinearIMC(ModelFn model, InverseModelFn inverse,
                 const NonlinearIMCParams &params, double Ts);

    /**
     * @brief Compute u[k] from the tracking error e[k] = r[k] - y[k].
     * @param error Current tracking error.
     * @return Saturated control output; holds last output on a non-finite input or a
     *         non-finite inverse-model result (graceful fallback).
     * @note Call setState(x) before each compute() with the current plant state.
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }

    void reset() override;
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "NonlinearIMC"; }

    /** @brief Inject the current plant state x[k]. Call before compute(). */
    void setState(const Eigen::VectorXd &x) { x_ = x; }

    /** @brief Current internal model output y_model[k] (mismatch-feedback diagnostic). */
    double modelOutput() const { return y_model_; }

private:
    ModelFn        model_;
    InverseModelFn inverse_;
    double         lambda_;
    double         uMin_, uMax_;
    double         Ts_;

    Eigen::VectorXd x_;            ///< Current plant state (from setState).
    double          y_model_ = 0.0; ///< Internal model output y_model[k-1].
    double          sf_      = 0.0; ///< IMC filter state.
    double          u_prev_  = 0.0; ///< Last finite output (hold-last NaN contract).
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(nonlinear_imc)
