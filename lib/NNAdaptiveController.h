#pragma once
#include "NeuralNetworkController.h"
#include "Features.h"
#include <Eigen/Dense>

/**
 * @file NNAdaptiveController.h
 * @brief Lyapunov-stable online NN-adaptive control over the ML1 forward-pass core (Phase 3 ML2).
 *
 * A single-hidden-layer (or deeper) network whose hidden layers are *fixed* random/trained
 * features phi(.) and whose **output layer adapts online**:
 * @code
 *   phi[k] = hiddenFeatures([y_m - y, r])    (fixed feature map, from ML1)
 *   u[k]   = clamp(theta[k] . phi[k] + b[k], uMin, uMax)
 * @endcode
 *
 * **Adaptation law (Lyapunov gradient + sigma-modification, MRAC convention):**
 * @code
 *   e_m[k]      = y[k] - y_m[k]
 *   theta[k+1]  = theta[k] - Ts.( gamma . e_m . phi  +  sigma . theta )
 *   b[k+1]      = b[k]     - Ts.( gamma . e_m         +  sigma . b )
 *   y_m[k+1]    = a_m . y_m[k] + b_m . r[k]   (first-order reference model)
 * @endcode
 *
 * sigma-modification (same role as `MRACController::sigma`) bounds weight drift under
 * persistent disturbance / lack of persistent excitation. The hidden layers are the
 * fixed nonlinear basis; this is the classic RBF/NN functional-approximation adaptive
 * controller, generalised to ML1's arbitrary fixed feature map.
 *
 * @par Convention
 *  compute() takes the plant output y (not the error), matching MRACController. The last
 *  layer of the supplied network must be Linear with a single output.
 *
 * @see MRACController.h (sigma-modification convention), NeuralNetworkController.h (ML1 core).
 * @see docs/ALGORITHM_ROADMAP_PHASE3.md (ML2)
 */

namespace ctrl {

/** @brief Parameters for NNAdaptiveController. */
struct NNAdaptiveParams
{
    NeuralControllerParams nn;        ///< ML1 base architecture; hidden layers fixed, output adapts.
    double gamma_adapt = 1.0;         ///< Adaptation gain (> 0).
    double sigma_mod   = 0.01;        ///< sigma-modification leakage (>= 0; same role as MRAC sigma).
    double a_m         = 0.5;         ///< Reference-model pole (|a_m| < 1).
    double b_m         = 0.5;         ///< Reference-model gain (b_m = 1 - a_m for unity DC gain).
    double uMin        = -1e9;        ///< Output lower saturation limit.
    double uMax        =  1e9;        ///< Output upper saturation limit.
};

/**
 * @brief NN-adaptive controller: online output-weight adaptation over a fixed ML1 feature map.
 */
class NNAdaptiveController : public NeuralNetworkController
{
public:
    /**
     * @brief Construct from an ML1 architecture plus adaptation parameters.
     * @param params NN architecture (last layer must be Linear, single output) + adaptation gains.
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument If |a_m| >= 1, gamma_adapt <= 0, sigma_mod < 0, the network
     *         input width is not 2 ([y_m - y, r] feature convention), or the last layer is not
     *         Linear (validated on top of the base ML1 dimension checks).
     */
    NNAdaptiveController(const NNAdaptiveParams &params, double Ts);

    /**
     * @brief Compute one control step.
     * @param y_plant Current plant output y[k].
     * @return Saturated control action u[k]; holds last output on a non-finite input.
     */
    double compute(double y_plant) override;

    SignConvention signConvention() const override { return SignConvention::PlantOutput; }

    /** @brief Set the current reference r[k]. Call once per step before compute(). */
    void setReference(double r) { r_ = r; }

    void reset() override;
    std::string name() const override { return "NNAdaptiveController"; }

    /** @brief Reference-model output y_m[k] (updated inside compute()). */
    double modelOutput() const { return y_m_; }

    /** @brief Euclidean norm of the adapting output-layer weights (drift diagnostic). */
    double outputWeightNorm() const;

private:
    NNAdaptiveParams params_;
    double a_m_, b_m_, gamma_, sigma_;
    double uMin_, uMax_;
    double r_      = 0.0;   ///< Current reference.
    double y_m_    = 0.0;   ///< Reference-model state.
    double u_prev_ = 0.0;   ///< Last finite output (hold-last NaN contract).

    Eigen::MatrixXd theta0_; ///< Initial output-layer W (for reset()).
    Eigen::VectorXd b0_;     ///< Initial output-layer b (for reset()).
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(nn_adaptive_controller)
