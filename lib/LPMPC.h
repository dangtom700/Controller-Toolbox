#pragma once
#include "IController.h"
#include "LPSolver.h"
#include "PlantModel.h"
#include <Eigen/Dense>

/**
 * @file LPMPC.h
 * @brief SISO L1-cost linear MPC solved via LPSolver's two-phase simplex (Phase 3 OC4).
 *
 * Minimises an L1 (not L2) receding-horizon cost over the same condensed prediction
 * `DiscreteMPC` uses:
 * @code
 *   Yhat = F*x[k] + Gu*u[k-1] + Phi*DeltaU
 *   J = rho_y * Sigma_i |yhat[k+i|k] - r| + rho_u * Sigma_j |Deltau[k+j]|
 * @endcode
 * cast as an LP via epigraph slacks `t_y >= |e|`, `t_u >= |Deltau|` and solved each step by
 * `LPSolver::solve()`.
 *
 * **SISO only (m=1, p=1).** Going MIMO would need vector bookkeeping for both the epigraph
 * row count and the rolling cumulative-u-bound tightening; deferred (see the design doc).
 *
 * **NOT a zero-allocation hot path.** Unlike `DiscreteMPC` (whose FISTA workspace is fully
 * pre-allocated at construction), `LPSolver::solve()` builds its simplex tableau fresh every
 * call. This is a documented, deliberate scoping choice -- see
 * `docs/superpowers/specs/2026-06-27-lp-solver-lp-mpc-design.md` decision 7. Prefer
 * `DiscreteMPC` for hard-RT loops where an L2 cost is acceptable.
 *
 * **L1-cost "deadzone" (tune rho_u relative to rho_y * plant sensitivity, not in isolation).**
 * Unlike QP/L2-cost MPC -- which always takes an infinitesimal DeltaU for any nonzero gradient --
 * an L1 cost only takes a move when its aggregate marginal benefit
 * (~ rho_y * sum_i Phi(i,j), summed across the horizon for control channel j) exceeds the rho_u
 * penalty. Below that threshold the LP-optimal DeltaU is *exactly* zero, every step, forever --
 * this is the correct optimum, not a bug. For plants with a small per-step step-response
 * coefficient (e.g. a slow lag at a short Ts), DiscreteMPC-style rho_y/rho_u ratios (~10-100x)
 * can leave LPMPC stuck at u=0 even with a large persistent tracking error; rho_u typically needs
 * to be 1-2 orders of magnitude smaller than the QP analogue would suggest. Verified empirically
 * during this class's implementation (`docs/superpowers/specs/2026-06-27-lp-solver-lp-mpc-design.md`).
 *
 * @see Camacho & Bordons, "Model Predictive Control" (2007) -- L1/Linf-cost MPC as an LP.
 */

namespace ctrl
{

/** @brief Tuning and horizon parameters for LPMPC. */
struct LPMPCParams
{
    int    Np = 10;     ///< Prediction horizon [steps].
    int    Nc = 3;       ///< Control horizon [steps], Nc <= Np.
    double rho_y = 1.0;   ///< Output-tracking L1 weight.
    double rho_u = 0.1;   ///< Control-move L1 weight.
    double uMin  = -1e9;  ///< Hard lower limit on u.
    double uMax  =  1e9;  ///< Hard upper limit on u.
    double duMin = -1e9;  ///< Hard lower limit on Deltau.
    double duMax =  1e9;  ///< Hard upper limit on Deltau.
    /**
     * @brief LPSolver pivot budget per step.
     *
     * Higher than LPSolver's own 200-pivot default: Bland's-rule simplex (required for
     * cycle-freedom) converges more slowly near degenerate/tied vertices, and L1-cost epigraph
     * formulations are prone to exactly that near their optimum (multiple (t_y, t_u)
     * combinations sharing the same cost at a kink). Verified empirically -- 200 was
     * insufficient for a Np=15/Nc=5 problem to fully resolve once steady-state put it near a
     * degenerate optimum, even though the practically-useful answer was already correct.
     */
    int    lpMaxIter = 500;
    double lpTol     = 1e-8; ///< LPSolver feasibility/optimality tolerance.
};

/**
 * @brief SISO L1-cost linear MPC (condensed prediction + LPSolver simplex per step).
 */
class LPMPC : public IController
{
public:
    /**
     * @brief Construct the LP-MPC and precompute condensed prediction matrices.
     * @param plant  Discrete-time SISO state-space plant (A, B, C, D, Ts).
     * @param params Horizon and cost parameters.
     * @throws std::invalid_argument If plant.inputSize() != 1 or plant.outputSize() != 1.
     */
    LPMPC(const StateSpace &plant, const LPMPCParams &params);

    /**
     * @brief SISO convenience interface -- compute u[k] from tracking error.
     * @param error Tracking error e[k] = r[k] - y[k].
     * @return Control output u[k].
     */
    double compute(double error) override;

    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }

    /**
     * @brief Full interface -- optimise u[k] given state and scalar reference.
     * @param x_current Current state vector x[k] (n x 1).
     * @param r_ref     Reference output r[k].
     * @return Control action u[k].
     */
    double computeRef(const Eigen::VectorXd &x_current, double r_ref);

    /** @brief Reset internal state estimate and previous input to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Update horizon/weight parameters and recompute condensed matrices. */
    void setParams(const LPMPCParams &p);

    /** @brief Read-only access to current parameters. */
    const LPMPCParams &params() const { return p_; }

    /** @brief Replace the internal plant model (successive linearisation). */
    void setPlant(const StateSpace &plant);

    /** @brief Inject a state estimate from an external observer. */
    void setState(const Eigen::VectorXd &x) { x_hat_ = x; }

    /** @brief Correct the previous applied input after external actuator saturation. */
    void setLastApplied(double u_applied) { u_prev_ = u_applied; }

    /** @brief @c true if the most recent LP solve reached LPStatus::Optimal. */
    [[nodiscard]] bool lastLPConverged() const { return last_lp_converged_; }

    /** @brief Simplex pivots used in the most recent computeRef() call. */
    int lastLPIters() const { return last_lp_iters_; }

    /** @brief Health reflects LP convergence (mirrors DiscreteMPC::isHealthy()). */
    [[nodiscard]] bool isHealthy() const override { return last_lp_converged_; }

private:
    StateSpace   plant_;
    LPMPCParams  p_;
    double       Ts_;
    Eigen::VectorXd x_hat_; ///< Open-loop state estimate.
    double       u_prev_ = 0.0;

    Eigen::MatrixXd F_;   ///< Np x n prediction matrix.
    Eigen::MatrixXd Phi_; ///< Np x Nc step-response matrix.
    Eigen::VectorXd Gu_;  ///< Np x 1 cumulative step-response offset for u_prev.

    // LP structure: variables = [DeltaU (Nc); t_y (Np); t_u (Nc)]. A_ineq_/c_ and the static
    // (t_y, t_u) portions of lb_/ub_ are built once; only b_ineq_.head(2*Np) and lb_/ub_.head(Nc)
    // (the DeltaU rolling-tightened box) are refreshed per computeRef() call.
    Eigen::MatrixXd A_ineq_;
    Eigen::VectorXd c_, b_ineq_, lb_, ub_;

    bool last_lp_converged_ = true;
    int  last_lp_iters_     = 0;
    mutable Eigen::VectorXd notify_buf_{Eigen::VectorXd::Constant(1, 0.0)};

    void buildCondensedMatrices();
};

} // namespace ctrl
