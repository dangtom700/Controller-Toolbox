#pragma once
#include "PlantModel.h"          // StateSpace
#include "LinearisationHelper.h" // StateFunc
#include <Eigen/Dense>
#include <functional>
#include <random>
#include <stdexcept>
#include <string>

namespace ctrl {

/**
 * @brief Echo State Network (ESN) / Reservoir Computing.
 *
 * Jaeger (2001). A randomly-initialized recurrent network (the "reservoir") where
 * only the readout layer W_out is trained via ridge regression.  Identifies
 * nonlinear dynamics without backpropagation.
 *
 * **Architecture:**
 * @code
 *   r[k+1] = tanh(W_res . r[k] + W_in . u[k])      (fixed, random reservoir)
 *   y_hat[k] = W_out . [r[k]; u[k]]                  (trained readout)
 * @endcode
 *
 * @note W_res is scaled so its spectral radius rho(W_res) = spectral_radius < 1.
 *       Typical values: spectral_radius = 0.9, sparsity = 0.9, alpha = 0.3.
 *
 * **Usage:**
 * @code
 *   ctrl::EchoStateNetwork::Params p;
 *   p.n_res = 100; p.n_in = 2; p.n_out = 1;
 *   p.spectral_radius = 0.9; p.sparsity = 0.9;
 *   ctrl::EchoStateNetwork esn(p);
 *
 *   // Collect training data
 *   for (int k = 0; k < N_train; ++k) {
 *       esn.stepReservoir(u_train[k]);
 *       esn.addTrainingTarget(y_train[k]);
 *   }
 *   esn.fitReadout();   // one-shot ridge regression for W_out
 *
 *   // Online prediction
 *   esn.stepReservoir(u_new);
 *   double y_hat = esn.predict()(0);
 * @endcode
 *
 * @see Jaeger, H. (2001). The echo state approach to analysing and training
 *      recurrent neural networks. Technical Report GMD 148, German National Research Center for IT.
 */
class EchoStateNetwork {
public:
    struct Params {
        int    n_res           = 100;   ///< Reservoir size (number of recurrent units)
        int    n_in            = 1;     ///< Input dimension
        int    n_out           = 1;     ///< Output dimension
        double spectral_radius = 0.9;   ///< Spectral radius of W_res (should be < 1 for echo state property)
        double sparsity        = 0.9;   ///< Fraction of W_res set to zero (improves memory)
        double input_scaling   = 1.0;   ///< W_in scaling factor
        double alpha           = 0.3;   ///< Leaking rate (0 = no leak, 1 = fast dynamics)
        double ridge           = 1e-4;  ///< Ridge regression regularisation
        int    washout         = 50;    ///< Initial timesteps to discard (reservoir warm-up)
        unsigned int seed      = 42;    ///< Random seed for reproducibility
    };

    explicit EchoStateNetwork(const Params& p);

    // ---- Training phase ---------------------------------------------------

    /** @brief Advance the reservoir one step with input u. */
    void stepReservoir(const Eigen::VectorXd& u);

    /** @brief Record a training target for the current reservoir state. */
    void addTrainingTarget(const Eigen::VectorXd& y_target);

    /** @brief Compute W_out via ridge regression on all accumulated data. */
    void fitReadout();

    // ---- Prediction phase -------------------------------------------------

    /**
     * @brief Advance reservoir and return output prediction.
     * Convenience: calls stepReservoir internally if you pass u here.
     */
    Eigen::VectorXd predict(const Eigen::VectorXd& u);

    /**
     * @brief Return output based on the CURRENT reservoir state (no step).
     * Requires stepReservoir to be called first.
     */
    Eigen::VectorXd predict() const;

    /**
     * @brief Return a StateFunc lambda for use in NonlinearMPC / EKF.
     *
     * The returned StateFunc:
     *   - state x = reservoir state r (n_res-dimensional)
     *   - input u = plant input (n_in-dimensional)
     *   - returns: next reservoir state (1-step forward)
     *
     * @note The actual plant output is W_out . [r; u]; use predict() for that.
     */
    StateFunc reservoirStateFunc() const;

    // ---- Utilities --------------------------------------------------------

    void   reset();
    bool   isFitted()   const noexcept { return fitted_; }
    int    reservoirSize() const noexcept { return p_.n_res; }

    const Eigen::VectorXd& reservoirState() const noexcept { return r_; }

private:
    Params p_;

    Eigen::MatrixXd W_res_;   ///< n_res * n_res  (fixed, random, sparse)
    Eigen::MatrixXd W_in_;    ///< n_res * n_in   (fixed, random)
    Eigen::MatrixXd W_out_;   ///< n_out * (n_res + n_in)  (trained)

    Eigen::VectorXd r_;       ///< Current reservoir state

    // Training data accumulation
    std::vector<Eigen::VectorXd> states_;   // [r; u] at each training step
    std::vector<Eigen::VectorXd> targets_;  // y_target at each training step
    int washout_cnt_ = 0;

    bool fitted_ = false;

    void initWeights();
    Eigen::VectorXd extendedState(const Eigen::VectorXd& u) const;
};

} // namespace ctrl
