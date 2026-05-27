#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <Eigen/Dense>

/**
 * @file DiscreteMPC.h
 * @brief Discrete-time Model Predictive Controller — condensed incremental QP formulation.
 *
 * Minimises a receding-horizon cost over predicted outputs and control moves:
 * @code
 *   J = Σᵢ₌₁ᴺᵖ ρy·‖ŷ[k+i|k] − r‖² + Σⱼ₌₀ᴺᶜ⁻¹ ρu·‖Δu[k+j]‖²
 * @endcode
 *
 * **Condensed prediction:**
 * @code
 *   Ŷ = F·x[k] + Gu·u[k−1] + Φ·ΔU
 * @endcode
 * where Φ(i,j) = C·A^(i−j)·B (j ≤ i, else 0) and Gu(i) = Σⱼ₌₀ⁱ C·Aʲ·B.
 *
 * **Unconstrained optimum:**
 * @code
 *   ΔU* = −(Φᵀ·Qy·Φ + Ru)⁻¹·Φᵀ·Qy·(F·x[k] − R_stacked)
 *   u[k] = u[k−1] + ΔU*[0:m]
 * @endcode
 *
 * Box constraints on Δu and u are solved via gradient projection (step size 1/L,
 * L = λmax(H), precomputed). Convergence is controlled by MPCParams::qpMaxIter and qpTol.
 *
 * @see Camacho & Bordons, "Model Predictive Control" (2007).
 * @see Maciejowski, "Predictive Control with Constraints" (2002).
 * @see MATLAB mpc(), mpcDesigner.
 */

namespace ctrl
{

/**
 * @brief Tuning and horizon parameters for DiscreteMPC.
 */
struct MPCParams
{
    int    Np       = 10;    ///< Prediction horizon [steps]. Cover approx. settling time.
    int    Nc       = 3;     ///< Control horizon [steps], Nc ≤ Np. Fewer = smoother moves.
    double rho_y    = 1.0;   ///< Output tracking weight  (Qy = ρy·I_{Np·p}).
    double rho_u    = 0.1;   ///< Move suppression weight (Ru = ρu·I_{Nc·m}).
    double uMin     = -1e9;  ///< Hard lower limit on u.
    double uMax     =  1e9;  ///< Hard upper limit on u.
    double duMin    = -1e9;  ///< Hard lower limit on Δu.
    double duMax    =  1e9;  ///< Hard upper limit on Δu.
    int    qpMaxIter = 200;  ///< Gradient-projection iteration limit.
    double qpTol     = 1e-8; ///< Convergence tolerance (‖Δx‖∞).
};

/**
 * @brief Discrete-time Model Predictive Controller.
 *
 * Prediction matrices F, Φ, and Hessian H are precomputed at construction and whenever the
 * plant or parameters change. Per-step heap allocation is eliminated via pre-allocated work vectors.
 */
class DiscreteMPC : public IController
{
public:
    /**
     * @brief Construct the MPC and precompute condensed prediction matrices.
     * @param plant  Discrete-time state-space plant (A, B, C, D, Ts).
     * @param params Horizon and cost parameters.
     */
    explicit DiscreteMPC(const StateSpace &plant, const MPCParams &params);

    /**
     * @brief SISO convenience interface — compute u[k] from tracking error.
     *
     * Reconstructs the reference as r = ŷ + error where ŷ = C·x̂ + D·u_prev.
     * Valid for D = 0 plants. For D ≠ 0, the feedthrough term uses u[k−1] (one step stale);
     * use computeRef() directly and supply the current r explicitly.
     *
     * @param error Tracking error e[k] = r[k] − y[k].
     * @return Control output u[k].
     */
    double compute(double error) override;

    /**
     * @brief Full MIMO interface — optimise u[k] given state and reference vector.
     * @param x_current Current state vector x[k] (n × 1).
     * @param r_ref     Reference output vector r[k] (p × 1).
     * @return Control action u[k] (m × 1).
     */
    Eigen::VectorXd computeRef(const Eigen::VectorXd &x_current,
                               const Eigen::VectorXd &r_ref);

    /** @brief Reset internal state estimate and previous input to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /**
     * @brief Update horizon/weight parameters and recompute condensed matrices.
     * @param p New MPC parameters.
     */
    void setParams(const MPCParams &p);

    /** @brief Read-only access to current parameters. */
    const MPCParams &params() const { return p_; }

    /**
     * @brief Replace the internal plant model (successive linearisation).
     *
     * Triggers a full recomputation of condensed prediction matrices.
     * @param plant Updated discrete-time state-space model.
     */
    void setPlant(const StateSpace &plant);

    /**
     * @brief Inject a state estimate from an external observer (e.g., Kalman filter).
     * @param x State estimate x̂[k] (n × 1).
     */
    void setState(const Eigen::VectorXd &x) { x_hat_ = x; }

    /**
     * @brief Correct the previous applied input after external actuator saturation.
     *
     * When an actuator layer clips or redistributes the MPC output before it reaches the plant,
     * call this method after each step so that the next computeRef() uses the physically applied
     * input when advancing the internal state estimate. Without this, repeated saturation causes
     * x̂ to drift silently from the real plant state.
     *
     * Typical call pattern (each step, after actuator):
     * @code
     *   auto u_cmd = mpc.computeRef(x, r);   // MPC command
     *   auto u_act = allocator.apply(u_cmd);  // actuator may clip/redistribute
     *   mpc.setLastApplied(u_act);            // correct u_prev_ for next step
     * @endcode
     *
     * @param u_applied Actual input applied to the plant at this step.
     */
    void setLastApplied(const Eigen::VectorXd &u_applied) { u_prev_ = u_applied; }

    /**
     * @brief @c true if the most recent QP converged within qpMaxIter iterations.
     *
     * @c false means the returned u[k] is the best available suboptimal iterate.
     * Repeated non-convergence indicates qpMaxIter is too small, qpTol is too tight,
     * or H is ill-conditioned (large Np, small ρu).
     */
    bool lastQPConverged() const { return last_qp_converged_; }

    /** @brief Actual gradient-projection iterations used in the most recent computeRef(). */
    int  lastQPIters()     const { return last_qp_iters_; }

    /**
     * @brief Health reflects QP convergence.
     *
     * Returns @c false when the most recent QP exited at qpMaxIter without converging.
     * ControllerStack uses this to fall back to a backup controller (e.g., PID).
     */
    bool isHealthy() const override { return last_qp_converged_; }

private:
    StateSpace plant_;
    MPCParams  p_;
    double     Ts_;
    Eigen::VectorXd x_hat_;  ///< Open-loop state estimate.
    Eigen::VectorXd u_prev_; ///< u[k−1] for incremental form.

    Eigen::MatrixXd F_;   ///< (Np·p) × n prediction matrix.
    Eigen::MatrixXd Phi_; ///< (Np·p) × (Nc·m) step-response matrix.
    Eigen::MatrixXd Gu_;  ///< (Np·p) × m cumulative step-response offset for u_prev.
    Eigen::MatrixXd H_;   ///< Precomputed Hessian Φᵀ·Qy·Φ + Ru.
    Eigen::MatrixXd Qy_;  ///< (Np·p) × (Np·p) output cost matrix.
    Eigen::MatrixXd Ru_;  ///< (Nc·m) × (Nc·m) move suppression cost matrix.
    double          L_;   ///< λmax(H) — Lipschitz constant for QP step size.
    Eigen::LDLT<Eigen::MatrixXd> ldlt_; ///< Pre-factored H_, refreshed in buildCondensedMatrices().

    // Pre-allocated work vectors — eliminate per-step heap allocation in computeRef().
    Eigen::VectorXd R_stack_, pred_err_, grad_, DeltaU_, grad_k_, DU_new_, lb_, ub_, cumMin_, cumMax_;

    bool last_qp_converged_ = true;
    int  last_qp_iters_     = 0;

    void buildPredictionMatrices();
    void buildCostMatrix();
    void buildCondensedMatrices();
};

} // namespace ctrl
