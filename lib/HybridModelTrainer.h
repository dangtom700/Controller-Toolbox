#pragma once
#include "HybridModel.h"
#include "GaussianProcess.h"
#include "EchoStateNetwork.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

/**
 * @file HybridModelTrainer.h
 * @brief Off-line trainer for the data component of a HybridModel (H4).
 *
 * Given a batch of observed state transitions (x_k, u_k) -> x_next_k_true,
 * HybridModelTrainer computes the residuals against the physical model,
 * fits a data-driven function to those residuals, and installs it into the
 * HybridModel via setDataModel().
 *
 * Three training methods are provided:
 *
 * | Method | Class               | Notes                                       |
 * |--------|---------------------|---------------------------------------------|
 * | Ridge  | Eigen ridge regress | Linear, always converges, fastest           |
 * | GP     | n independent GPs   | Nonlinear + uncertainty, O(N^3) per state   |
 * | ESN    | EchoStateNetwork    | Nonlinear, reservoir in zero-state mode     |
 *
 * **Usage:**
 * @code
 *   ctrl::HybridModelTrainer::Params tp;
 *   tp.method = ctrl::HybridModelTrainer::Method::GP;
 *   tp.gp.length_scale = 0.5;
 *   ctrl::HybridModelTrainer trainer(tp);
 *
 *   // X_obs: (n_states x N), U_obs: (n_inputs x N), X_next_obs: (n_states x N)
 *   trainer.trainHybridModel(*model, X_obs, U_obs, X_next_obs);
 *
 *   double rmse = trainer.validate(*model, X_obs, U_obs, X_next_obs);
 *   std::cout << "validation RMSE: " << rmse << "\n";
 * @endcode
 *
 * @see HybridModel - H1 plant model.
 * @see HybridMPC   - H2 controller with built-in online ridge trainer.
 */

namespace ctrl {

/**
 * @brief Trains the f_data component of a HybridModel from state-transition data.
 */
class HybridModelTrainer
{
public:
    /** @brief Training method selector. */
    enum class Method { Ridge, GP, ESN };

    struct Params
    {
        Method method = Method::Ridge;  ///< Which training algorithm to use.

        // Ridge options
        double ridge_lambda = 1e-4;     ///< Ridge regularisation coefficient.

        // GP options (one GP per state dimension)
        GaussianProcess::Params gp;     ///< SE-kernel GP hyperparameters applied to all dimensions.

        // ESN options (single ESN, n_out = n_states)
        EchoStateNetwork::Params esn;   ///< ESN architecture parameters; n_in/n_out are overridden.
    };

    /** @brief Result returned by trainHybridModel(). */
    struct Result
    {
        bool   success;      ///< Training completed without errors.
        double train_rmse;   ///< Root-mean-square residual prediction error on training data.
        std::string method;  ///< Which method was used ("Ridge", "GP", "ESN").
        int    n_samples;    ///< Number of training samples used.
    };

    /** @brief Construct trainer with default parameters. */
    HybridModelTrainer();
    /** @brief Construct trainer with given parameters. */
    explicit HybridModelTrainer(const Params& params);

    /**
     * @brief Train the data component and install it in model.
     *
     * Computes residuals  delta_x_k = X_next_obs.col(k) - model.predictPhys(x_k, u_k)
     * then fits the chosen model type on feature [x; u] -> delta_x.
     *
     * @param model        HybridModel to update (in-place via setDataModel()).
     * @param X_obs        State observations at step k    (n_states x N, column-major).
     * @param U_obs        Input observations at step k    (n_inputs x N).
     * @param X_next_obs   True next states                (n_states x N).
     * @return Result struct with success flag and training RMSE.
     */
    Result trainHybridModel(HybridModel&           model,
                             const Eigen::MatrixXd& X_obs,
                             const Eigen::MatrixXd& U_obs,
                             const Eigen::MatrixXd& X_next_obs) const;

    /**
     * @brief Evaluate prediction RMSE of model against observed transitions.
     *
     * Returns the root mean square of ||x_next_true - model.predict(x, u)||_2
     * over all N samples.
     */
    double validate(const HybridModel&     model,
                    const Eigen::MatrixXd& X_obs,
                    const Eigen::MatrixXd& U_obs,
                    const Eigen::MatrixXd& X_next_obs) const;

private:
    Params p_;

    HybridModel::DataFunc trainRidge(const Eigen::MatrixXd& Feat,
                                      const Eigen::MatrixXd& Resid) const;

    HybridModel::DataFunc trainGP(const Eigen::MatrixXd& Feat,
                                   const Eigen::MatrixXd& Resid) const;

    HybridModel::DataFunc trainESN(const Eigen::MatrixXd& Feat,
                                    const Eigen::MatrixXd& Resid) const;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(hybrid_model_trainer)
