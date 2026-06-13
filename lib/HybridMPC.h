#pragma once
#include "NonlinearMPC.h"
#include "HybridModel.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>

/**
 * @file HybridMPC.h
 * @brief Hybrid Model Predictive Controller (H2).
 *
 * Inherits NonlinearMPC and drives prediction through a shared HybridModel.
 * Supports online data accumulation and periodic refitting of the data component
 * via ridge regression on state-transition residuals.
 *
 * **Online learning loop:**
 * @code
 *   auto model = std::make_shared<ctrl::HybridModel>(f_phys, hmp, p_phys);
 *   ctrl::HybridMPCParams hp;
 *   hp.nmpc.Np = 10; hp.nmpc.Ts = 0.01;
 *   hp.nmpc.n_states = 2; hp.nmpc.n_inputs = 1;
 *   hp.data_update_interval = 30;
 *   ctrl::HybridMPC hmpc(hp, model);
 *
 *   for (int k = 0; k < N; ++k) {
 *       hmpc.setState(x_k);
 *       double u = hmpc.compute(e_k);
 *       // advance plant...
 *       hmpc.addStateObservation(x_k, u_vec, x_next_true);
 *   }
 * @endcode
 *
 * When data_update_interval observations have been accumulated and
 * min_observations is reached, refitDataModel() is called automatically.
 * This fits a ridge-regression linear model on the (feature=[x;u]) ->
 * (delta_x = x_next_true - predictPhys(x,u)) mapping and injects it into
 * the HybridModel's data component.  For advanced non-linear data models
 * use HybridModelTrainer (H4) and call model->setDataModel() directly.
 *
 * @see HybridModel        - H1 plant model combining physics + data.
 * @see HybridModelTrainer - H4 off-line trainer (GP / ESN alternatives).
 */

namespace ctrl {

/** @brief Parameters for HybridMPC. */
struct HybridMPCParams
{
    NMPCParams nmpc;                   ///< Base NonlinearMPC parameters (horizons, weights, bounds).
    int data_update_interval = 20;     ///< Auto-refit every N addStateObservation() calls (0 = manual only).
    int min_observations     = 10;     ///< Minimum observations required before any refit.
    double ridge_lambda      = 1e-4;   ///< Ridge regression regularisation for online linear data model.
};

/**
 * @brief HybridMPC: NonlinearMPC with a HybridModel and online residual learning (H2).
 *
 * Inherits all NonlinearMPC public methods (compute, computeRef, setState,
 * setReference, lastQPConverged, lastQPIters, lastControlVec).
 *
 * Extra online-learning interface:
 *   - addStateObservation(x, u, x_next_true)
 *   - refitDataModel()
 *   - observationCount()
 *   - isDataModelFitted()
 */
class HybridMPC : public NonlinearMPC
{
public:
    /**
     * @brief Construct HybridMPC.
     *
     * The NonlinearMPC base is initialised with a DiscreteDynamics lambda that
     * delegates to model->predict(x,u).  Because the lambda captures model by
     * shared_ptr, any later call to model->setDataModel() is immediately
     * reflected in future NonlinearMPC solves without re-constructing the controller.
     *
     * @param params  Hybrid MPC parameters (NMPCParams + update interval).
     * @param model   Shared HybridModel.  Must have dimensions consistent with params.nmpc.
     */
    HybridMPC(const HybridMPCParams& params, std::shared_ptr<HybridModel> model);

    std::string name() const override { return "HybridMPC"; }

    // -------------------------------------------------------------------------
    // Online data collection
    // -------------------------------------------------------------------------

    /**
     * @brief Record one discrete-time state transition for online residual learning.
     *
     * Computes residual = x_next_true - model->predictPhys(x, u), stores the
     * feature [x; u] and residual pair, then triggers refitDataModel() when
     * the auto-update interval is reached (if data_update_interval > 0).
     *
     * @param x           State at step k (n_states x 1).
     * @param u           Input applied at step k (n_inputs x 1).
     * @param x_next_true True next state measured from the plant (n_states x 1).
     */
    void addStateObservation(const Eigen::VectorXd& x,
                              const Eigen::VectorXd& u,
                              const Eigen::VectorXd& x_next_true);

    /**
     * @brief Fit a ridge-regression data model on all accumulated observations.
     *
     * Feature matrix: [x; u]  (n_feat x N).
     * Target matrix:  delta_x = x_next_true - predictPhys(x, u)  (n_states x N).
     * Regression:     W = (X'X + lambda*I)^{-1} X' Y   (n_feat x n_states).
     * Data model:     delta_x_next = W' * [x; u].
     *
     * Resets obs_since_refit_ counter.
     *
     * @return true  if the refit was performed (enough data present).
     * @return false if fewer than min_observations samples are stored.
     */
    bool refitDataModel();

    /** @brief Number of state observations accumulated so far. */
    int  observationCount()  const noexcept { return static_cast<int>(feat_data_.size()); }

    /** @brief True if the data model has been fitted at least once. */
    bool isDataModelFitted() const noexcept { return data_fitted_; }

    /** @brief Shared pointer to the underlying HybridModel. */
    std::shared_ptr<HybridModel> hybridModel() const { return model_; }

private:
    HybridMPCParams              hp_;
    std::shared_ptr<HybridModel> model_;
    int                          obs_since_refit_ = 0;
    bool                         data_fitted_     = false;

    // Accumulated training data
    std::vector<Eigen::VectorXd> feat_data_;   // [x; u] for each observation
    std::vector<Eigen::VectorXd> resid_data_;  // delta_x for each observation
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(hybrid_mpc)
