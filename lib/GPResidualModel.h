#pragma once
#include "GaussianProcess.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <functional>
#include <vector>

namespace ctrl {

/**
 * @brief GP Residual Model - learn model-plant mismatch for risk-aware MPC (E3).
 *
 * Fits a Gaussian Process to the residuals epsilon_k = y_true_k - y_model_k,
 * where y_model is the prediction from a known (but imperfect) physical model.
 *
 * At prediction time:
 *   total = model_pred + GP_mean(x_feat)   -- bias-corrected output
 *   var   = GP_variance(x_feat)            -- uncertainty for constraint tightening
 *
 * Risk-aware MPC usage: tighten state constraints by sigma_k = sqrt(var_k),
 * so the controller keeps a safety margin when far from training data.
 *
 * @par Typical workflow
 * @code
 *   ctrl::GPResidualModel::Params p;
 *   p.gp.length_scale = 0.5;
 *   p.gp.signal_var   = 0.1;
 *   p.gp.noise_var    = 0.01;
 *   p.gp.n_max        = 200;
 *   ctrl::GPResidualModel grm(1, p);           // 1-D feature
 *
 *   // Option A: accumulate points online, then fit once
 *   grm.addResidualPoint(x_feat, y_true, model_output);
 *   grm.fit();
 *
 *   // Option B: batch fit from trajectory + model function
 *   grm.residualFit(X_feat, Y_true_vec,
 *                   [&](const auto& xf) { return linear_model(xf); });
 *
 *   // Prediction with uncertainty
 *   auto pred = grm.predictWithUncertainty(x_feat, model_output);
 *   // pred.mean_total = corrected estimate
 *   // pred.variance   = posterior GP variance (use sqrt for 1-sigma bound)
 * @endcode
 *
 * @see GaussianProcess for the underlying SE-kernel GP.
 */
class GPResidualModel {
public:
    /** @brief Physical model function: y_model = f(x_feat). */
    using ModelFn = std::function<double(const Eigen::VectorXd&)>;

    struct Params {
        GaussianProcess::Params gp;    ///< SE-kernel GP hyperparameters
    };

    struct Prediction {
        double mean_total;   ///< model_pred + GP correction mean  (corrected estimate)
        double gp_mean;      ///< GP correction mean only           (learned residual)
        double variance;     ///< GP posterior variance (>= 0)      (uncertainty measure)
    };

    /**
     * @brief Construct residual model.
     * @param x_dim  Dimension of the feature vector passed to the GP.
     * @param par    Parameters (GP hyperparameters).
     */
    GPResidualModel(int x_dim, const Params& par);

    /**
     * @brief Add one residual sample online.
     * Computes residual = y_true - y_model and appends to GP dataset.
     * Call fit() after accumulating sufficient points.
     * @param x_feat   Feature vector (x_dim x 1).
     * @param y_true   True plant output at this feature.
     * @param y_model  Physical model prediction at this feature.
     */
    void addResidualPoint(const Eigen::VectorXd& x_feat,
                           double y_true,
                           double y_model);

    /**
     * @brief Batch: compute residuals via model_fn, reset GP, and fit in one call.
     * @param X_feat    Feature matrix (x_dim x N); column k = feature[k].
     * @param Y_true    True outputs (N x 1).
     * @param model_fn  Physical model: f(x_feat) -> scalar prediction.
     */
    void residualFit(const Eigen::MatrixXd& X_feat,
                     const Eigen::VectorXd& Y_true,
                     ModelFn                model_fn);

    /** @brief (Re-)fit the GP Cholesky factorisation on all accumulated data. */
    void fit();

    /**
     * @brief Predict total output and uncertainty at x_feat.
     *
     * If not fitted yet, returns {model_pred, 0.0, 0.0} (no correction, zero variance).
     *
     * @param x_feat     Feature vector (x_dim x 1).
     * @param model_pred Physical model prediction at x_feat.
     * @return           Prediction struct with mean_total, gp_mean, variance.
     */
    Prediction predictWithUncertainty(const Eigen::VectorXd& x_feat,
                                       double model_pred) const;

    /** @brief Access underlying GP (read-only). */
    const GaussianProcess& gp()     const { return gp_; }

    /** @brief Number of residual data points stored. */
    int size()                      const { return gp_.size(); }

    /** @brief True if fit() has been called with at least one data point. */
    bool isFitted()                 const { return gp_.isFitted(); }

    /** @brief Input feature dimension of the underlying GP. */
    int xDim()                      const { return gp_.xDim(); }

    /** @brief Clear all stored data and reset the GP. */
    void reset();

private:
    GaussianProcess gp_;
    Params          par_;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(gp_residual_model)
