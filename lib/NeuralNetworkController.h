#pragma once
#include "IController.h"
#include "Features.h"
#include <Eigen/Dense>
#include <vector>

/**
 * @file NeuralNetworkController.h
 * @brief Generic feedforward neural-network controller - forward-pass only (Phase 3 ML1).
 *
 * A depth-arbitrary fully-connected network mapping a feature vector to a single control
 * action `u`, usable as an `IController`. Unlike `NeuralPID` (which fixes a 3-layer
 * [e, e_dot, e_int] -> [Kp, Ki, Kd] architecture and adapts online), this class runs a
 * *fixed* forward pass with user-supplied weights - the intended workflow is training
 * offline (e.g. PyTorch on simulation data) and importing the weights for deployment.
 *
 * **Forward pass:**
 * @code
 *   a_0 = features                         (size n_input_features)
 *   a_i = activation_i(W_i . a_{i-1} + b_i) (per layer)
 *   u   = clamp(a_L(0), uMin, uMax)        (last layer must have a single output)
 * @endcode
 *
 * This is the forward-pass primitive `NNAdaptiveController` (ML2) layers online weight
 * adaptation on top of - hence "core".
 *
 * @note Research/deployment-import controller. The network is *not* trained here; supply
 *       pre-trained weights via the constructor or `loadWeights()`.
 * @see docs/ALGORITHM_ROADMAP_PHASE3.md (ML1)
 */

namespace ctrl {

/** @brief One fully-connected layer: activation(W . a + b). */
struct NNLayerSpec
{
    /** @brief Elementwise activation applied after the affine map. */
    enum class Activation { Tanh, ReLU, Sigmoid, Linear, Softplus };

    Eigen::MatrixXd W;                          ///< Weight matrix (out_dim x in_dim).
    Eigen::VectorXd b;                          ///< Bias vector (out_dim).
    Activation      activation = Activation::Tanh; ///< Activation for this layer.
};

/** @brief Construction parameters for NeuralNetworkController. */
struct NeuralControllerParams
{
    std::vector<NNLayerSpec> layers;       ///< Layers in forward order; last must output a scalar.
    double uMin = -1e9;                    ///< Output lower saturation limit.
    double uMax =  1e9;                    ///< Output upper saturation limit.
    int    n_input_features = 1;           ///< Input feature count (must equal layers[0].W.cols()).
};

/**
 * @brief Feedforward neural-network controller (fixed forward pass, scalar output).
 */
class NeuralNetworkController : public IController
{
public:
    /**
     * @brief Construct from a layer stack.
     * @param params Network architecture, saturation, and input width.
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument If Ts <= 0, uMin >= uMax, the layer stack is empty,
     *         layer dimensions are inconsistent, the first layer's input width does not
     *         match `n_input_features`, or the last layer does not output exactly 1 value.
     */
    NeuralNetworkController(const NeuralControllerParams &params, double Ts);

    /**
     * @brief Compute one control step from a scalar feature.
     * @param signal Single input feature (valid only when n_input_features == 1).
     * @return Saturated control action u[k]; holds last output on a non-finite input.
     * @throws std::logic_error If n_input_features != 1 (use computeVec() instead).
     */
    double compute(double signal) override;

    /**
     * @brief Compute one control step from the full feature vector.
     * @param features Input feature vector (size n_input_features).
     * @return 1-element control vector [u[k]]; holds last output on a non-finite input.
     * @throws std::invalid_argument If features.size() != n_input_features.
     */
    Eigen::VectorXd computeVec(const Eigen::VectorXd &features) override;

    void   reset() override;
    double sampleTime() const override { return Ts_; }
    std::string name() const override { return "NeuralNetworkController"; }

    /**
     * @brief Hot-swap the network weights at runtime (e.g. after re-training offline).
     * @param layers New layer stack; validated against the existing n_input_features.
     * @throws std::invalid_argument On the same dimension-consistency rules as the constructor.
     */
    void loadWeights(const std::vector<NNLayerSpec> &layers);

    /** @brief Number of input features the network expects. */
    int inputFeatures() const noexcept { return n_input_; }

    /** @brief Number of layers in the network. */
    int numLayers() const noexcept { return static_cast<int>(layers_.size()); }

protected:
    /** @brief Run the full forward pass over @p features (no saturation, no NaN guard). */
    double forward(const Eigen::VectorXd &features) const;

    /**
     * @brief Run all layers *except the last*, returning the last layer's input (the hidden
     *        feature vector phi). Used by NNAdaptiveController to adapt only the output layer.
     */
    Eigen::VectorXd hiddenFeatures(const Eigen::VectorXd &features) const;

    /** @brief Apply an elementwise activation. */
    static Eigen::VectorXd applyActivation(const Eigen::VectorXd &z, NNLayerSpec::Activation a);

    static void validateLayers(const std::vector<NNLayerSpec> &layers, int n_input);

    std::vector<NNLayerSpec> layers_;

private:
    double Ts_;
    double uMin_, uMax_;
    int    n_input_;
    double u_prev_ = 0.0; ///< Last finite output (hold-last NaN contract).
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(neural_network_controller)
