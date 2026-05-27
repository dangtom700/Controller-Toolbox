#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <Eigen/Dense>

/**
 * @file GeneralizedPredictiveControl.h
 * @brief Discrete-time Generalised Predictive Controller (GPC).
 *
 * Extends DiscreteMPC with two practical features:
 *
 * **1. Velocity-form (CARIMA) model** — offset-free tracking without a separate integral state.
 * The augmented state xₐ = [Δx; y] is propagated with incremental inputs ΔU, giving a
 * built-in integrating disturbance model:
 * @code
 *   xₐ[k+1] = Aₐ·xₐ[k] + Bₐ·Δu[k]     Aₐ = [A, 0; C·A, I]
 *   y[k]    = Cₐ·xₐ[k]                  Bₐ = [B; C·B],  Cₐ = [0, I]
 * @endcode
 *
 * **2. Reference trajectory** — the setpoint is approached along a first-order filter:
 * @code
 *   y*[k+j] = αʲ·y[k] + (1−αʲ)·r[k],   α ∈ [0, 1)
 * @endcode
 * - α = 0: step reference (same as DiscreteMPC).
 * - α → 1: very soft approach; reduces overshoot without derivative weights.
 *
 * The plant model can be hot-swapped via setPlant() for adaptive GPC (pair with
 * RecursiveLeastSquares::toStateSpace() for a self-tuning loop).
 *
 * @note Output constraints (y_min, y_max) are **not** supported. For hard output
 *       constraints, use DiscreteMPC with a slack-variable formulation instead.
 *
 * @see Clarke et al., "Generalised Predictive Control", Automatica (1987).
 * @see Maciejowski, "Predictive Control with Constraints" (2002) Ch. 3.
 */

namespace ctrl
{

/**
 * @brief Tuning and horizon parameters for GeneralizedPredictiveController.
 */
struct GPCParams
{
    int    Np      = 10;    ///< Prediction horizon [steps].
    int    Nu      = 3;     ///< Control horizon [steps], Nu ≤ Np.
    double rho_y   = 1.0;   ///< Output tracking weight (Qy = ρy·I).
    double rho_u   = 0.1;   ///< Move suppression weight (Ru = ρu·I).

    /**
     * @brief Reference trajectory softening factor α ∈ [0, 1).
     *
     * α = 0 recovers a step reference (same as DiscreteMPC).
     * α → 1 produces a very soft approach, reducing overshoot without tuning derivative weights.
     */
    double alpha   = 0.0;

    double uMin    = -1e9;  ///< Hard lower limit on u.
    double uMax    =  1e9;  ///< Hard upper limit on u.
    double duMin   = -1e9;  ///< Hard lower limit on Δu.
    double duMax   =  1e9;  ///< Hard upper limit on Δu.
    int    qpMaxIter = 200; ///< Gradient-projection iteration limit.
    double qpTol     = 1e-8;///< Convergence tolerance (‖Δx‖∞).
};

/**
 * @brief Discrete-time Generalised Predictive Controller.
 *
 * Internally uses the same gradient-projection QP solver as DiscreteMPC.
 * The health interface reflects QP convergence for ControllerStack fallback.
 */
class GeneralizedPredictiveController : public IController
{
public:
    /**
     * @brief Construct the GPC and precompute augmented condensed matrices.
     *
     * @param plant  Discrete-time state-space model (A, B, C, D, Ts).
     *               D should be zero; non-zero D is accepted but ignored in the
     *               velocity-form augmentation.
     * @param params GPC tuning parameters.
     */
    GeneralizedPredictiveController(const StateSpace &plant, const GPCParams &params);

    /**
     * @brief Compute u[k] from tracking error e[k] = r[k] − y[k].
     *
     * Uses the setpoint stored via setReference(). Defaults to r = 0.
     * Prefer computeRef(y, r) directly when y and r are independently available.
     *
     * @param error Tracking error e[k].
     * @return Control output u[k].
     */
    double compute(double error) override;

    /**
     * @brief Full interface — supply plant output y and setpoint r separately.
     *
     * Preferred over compute(error) because the reference trajectory requires y[k]
     * independently of the error.
     *
     * @param y Current plant output y[k].
     * @param r Current setpoint r[k].
     * @return Control output u[k].
     */
    double computeRef(double y, double r);

    /**
     * @brief Hot-swap the plant model (adaptive GPC). Rebuilds condensed matrices.
     * @param plant Updated discrete-time state-space model.
     */
    void setPlant(const StateSpace &plant);

    /**
     * @brief Update parameters and rebuild condensed matrices.
     * @param p New GPC parameters.
     */
    void setParams(const GPCParams &p);

    /** @brief Read-only access to current parameters. */
    const GPCParams &params() const { return p_; }

    /**
     * @brief Set the reference setpoint for the next compute(error) call.
     * @param r Setpoint r[k].
     */
    void setReference(double r) { r_ref_ = r; }

    /** @brief Reset augmented state and previous input to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Current augmented state xₐ = [Δx; y] (size n+p).
     *
     * Useful for monitoring the velocity-form integrator state.
     */
    const Eigen::VectorXd &augmentedState() const { return xa_; }

    /**
     * @brief @c true if the most recent QP converged within qpMaxIter iterations.
     *
     * Repeated @c false indicates qpMaxIter is too small or H is ill-conditioned.
     */
    bool lastQPConverged() const { return last_qp_converged_; }

    /** @brief Actual gradient-projection iterations used in the most recent computeRef(). */
    int  lastQPIters()     const { return last_qp_iters_; }

    /**
     * @brief Health reflects QP convergence for ControllerStack fallback.
     * @return @c false when the most recent QP exited at qpMaxIter.
     */
    bool isHealthy() const override { return last_qp_converged_; }

private:
    void buildCondensedMatrices();

    StateSpace plant_;
    GPCParams  p_;
    double     Ts_;

    Eigen::MatrixXd Aa_, Ba_;  ///< Augmented state-space matrices ((n+p)×(n+p), (n+p)×m).
    Eigen::MatrixXd Ca_;       ///< Augmented output matrix (p×(n+p)).

    Eigen::MatrixXd Fa_;       ///< (Np·p) × (n+p) prediction matrix.
    Eigen::MatrixXd Ga_;       ///< (Np·p) × (Nu·m) step-response matrix.
    Eigen::MatrixXd Qy_, Ru_;
    Eigen::MatrixXd H_;
    /// Pre-factored H_, refreshed exclusively by buildCondensedMatrices().
    /// NEVER call ldlt_.compute() from outside that function — doing so with a
    /// stale or incorrect matrix silently corrupts every subsequent QP solve.
    Eigen::LDLT<Eigen::MatrixXd> ldlt_;

    Eigen::VectorXd Rtraj_, err_, grad_, DeltaU_, grad_k_, DU_new_, lb_, ub_, cumMin_, cumMax_;

    double L_;

    bool last_qp_converged_ = true;
    int  last_qp_iters_     = 0;

    Eigen::VectorXd xa_;     ///< Augmented state estimate.
    Eigen::VectorXd u_prev_; ///< u[k−1] for Δu = u − u_prev.
    double          r_ref_;  ///< Stored setpoint for compute(error) wrapper.
};

} // namespace ctrl
