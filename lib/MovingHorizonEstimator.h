#pragma once
#include "PlantModel.h"
#include "GradientProjectionQP.h"
#include "MismatchDetector.h"
#include <Eigen/Dense>

/**
 * @file MovingHorizonEstimator.h
 * @brief Moving Horizon Estimator (MHE) - batch state estimator via condensed QP.
 *
 * MHE minimises a windowed sum of measurement residuals and process noise penalties
 * over a receding horizon of N past steps.  Unlike the Kalman filter (single-step
 * recursive), MHE explicitly supports state and noise constraints, making it
 * preferable for nonlinear systems or when physical bounds on states are known.
 *
 * **Plant model:**
 * @code
 *   x[k+1] = A.x[k] + B.u[k] + w[k],   w ~ N(0, Q_proc)
 *   y[k]   = C.x[k] + v[k],             v ~ N(0, R_meas)
 * @endcode
 * (D = 0 assumed for simplicity; extend by modifying the measurement residual.)
 *
 * **MHE objective (horizon N, current step k):**
 * @code
 *   min_{x_0, w_0, ..., w_{N-1}}
 *       (x_0 - x_bar)' P0inv (x_0 - x_bar)             [arrival cost]
 *     + sum_{i=0}^{N-1} w_i' Qinv w_i                   [process noise]
 *     + sum_{i=0}^{N}   (y_i - C x_i)' Rinv (y_i - C x_i) [meas residuals]
 * @endcode
 * where x_{i+1} = A x_i + B u_i + w_i.
 *
 * **Condensed formulation:**
 * Decision vector z = [x_0; w_0; ...; w_{N-1}] (size n*(N+1)).
 * State sequence x_i = A^i x_0 + sum A^j*(B*u + w) terms -> linear in z.
 * After condensing:
 * @code
 *   H_mhe z = h_mhe  (where H = Q_bar + Psi'*C_bar'*R_bar*C_bar*Psi)
 * @endcode
 * Solved by the library's GradientProjectionQP solver (same back-end as DiscreteMPC).
 *
 * **Equivalence to Kalman filter:** For a linear, Gaussian system with no constraints
 * and N large enough to cover the initial transient, MHE converges to the Kalman
 * filter steady-state estimate.  Constrained MHE is strictly better than KF when
 * physical bounds on states or noise are known.
 *
 * **Process-noise box constraints:**
 * Set @p wMin / @p wMax in MHEParams to constrain each w_i element.
 * State constraints require linear constraint handling (future extension).
 *
 * @see dynopt-master/MovingHorizonEstimation.ipynb (algorithmic reference).
 * @see dynopt-master/EstimatorTypes.ipynb (KF vs MHE comparison).
 * @see Rawlings & Mayne, "Model Predictive Control" Ch. 4 (2009).
 */

namespace ctrl {

/**
 * @brief Parameters for MovingHorizonEstimator.
 */
struct MHEParams
{
    int    N        = 10;    ///< Estimation horizon [steps].
    double wMin     = -1e9;  ///< Per-element lower bound on process noise w.
    double wMax     =  1e9;  ///< Per-element upper bound on process noise w.
    int    qpMaxIter = 300;  ///< Gradient-projection iteration limit.
    double qpTol     = 1e-8; ///< QP convergence tolerance.

    /**
     * @brief Per-element lower bound on the arrival state x_0 (size n, or empty = unconstrained).
     *
     * x_0 is the oldest state in the estimation window and is the first n elements of the QP
     * decision variable.  Constraining it prevents physically impossible values (e.g. negative
     * concentrations) from entering the estimate.
     *
     * @note Only x_0 is directly box-constrained.  States x_1..x_N are determined by the
     *       dynamics x_{i+1} = A*x_i + B*u_i + w_i; to bound them, also tighten wMin/wMax
     *       so that the propagation cannot drift outside the intended region.
     */
    Eigen::VectorXd xMin; ///< Lower box bound on x_0 (size n), empty = no constraint.
    Eigen::VectorXd xMax; ///< Upper box bound on x_0 (size n), empty = no constraint.

    /**
     * @brief General linear inequality constraints on the arrival state x_0 (E4).
     *
     * Enforces  C_ineq * x_0 <= d_ineq  after the FISTA solve via Hildreth's cyclic
     * half-space projections.  Useful for:
     *  - Simplex constraints: mole fractions sum to 1, all non-negative.
     *  - Coupled bounds: e.g. x_1 + x_2 <= 1.
     *  - Any polytope expressible as a set of linear inequalities.
     *
     * Set @p C_ineq (m_c x n) and @p d_ineq (m_c) together; both must be non-empty
     * for the constraint to be active.  Box constraints @p xMin / @p xMax are
     * re-applied after the polytope projection so both are simultaneously satisfied.
     *
     * @param C_ineq Constraint matrix (m_c x n), empty = no polytopic constraint.
     * @param d_ineq Constraint RHS (m_c x 1).
     * @param ineq_proj_iters Hildreth iterations (default 20, usually sufficient).
     */
    Eigen::MatrixXd C_ineq;         ///< Polytope constraint LHS (m_c x n), empty = off.
    Eigen::VectorXd d_ineq;         ///< Polytope constraint RHS (m_c x 1).
    int ineq_proj_iters = 20;       ///< Hildreth cyclic-projection iterations for C_ineq.
};

/**
 * @brief Moving Horizon Estimator.
 *
 * Call @c initialize() once, then @c estimate(y, u) at each sample step.
 * The returned state estimate is x^[k|k] (post-measurement update).
 */
class MovingHorizonEstimator
{
public:
    /**
     * @brief Construct the estimator and precompute condensed QP matrices.
     *
     * @param plant   Discrete-time state-space model (A, B, C, D, Ts). D is ignored.
     * @param Q_proc  Process noise covariance (n x n, positive semi-definite).
     * @param R_meas  Measurement noise covariance (p x p, positive definite).
     * @param params  Horizon and QP parameters.
     */
    MovingHorizonEstimator(const StateSpace       &plant,
                           const Eigen::MatrixXd  &Q_proc,
                           const Eigen::MatrixXd  &R_meas,
                           const MHEParams        &params = {});

    /**
     * @brief Set initial state estimate and arrival-cost covariance.
     *
     * Must be called before the first @c estimate().  Resets the measurement
     * and input history buffers.
     *
     * @param x0 Initial state guess (n x 1).
     * @param P0 Initial error covariance (n x n). Scales the arrival cost.
     */
    void initialize(const Eigen::VectorXd &x0,
                    const Eigen::MatrixXd &P0);

    /**
     * @brief Run one MHE step: update history buffers and solve the QP.
     *
     * @param y  Current measurement y[k] (p x 1).
     * @param u  Current control input u[k] applied at this step (m x 1).
     * @return State estimate x^[k|k] (n x 1).
     */
    Eigen::VectorXd estimate(const Eigen::VectorXd &y,
                             const Eigen::VectorXd &u);

    /**
     * @brief Change the horizon N and rebuild condensed matrices.
     *
     * Clears history; call @c initialize() again after this.
     *
     * @param N New horizon length [steps].
     */
    void setHorizon(int N);

    /**
     * @brief Update process/measurement noise covariances and rebuild matrices.
     *
     * @param Q_proc New process noise covariance (n x n).
     * @param R_meas New measurement noise covariance (p x p).
     */
    void setWeightMatrices(const Eigen::MatrixXd &Q_proc,
                           const Eigen::MatrixXd &R_meas);

    /** @brief Reset history and state estimate to zero without changing Q, R, or P0. */
    void reset();

    /** @brief Current state estimate (available after first estimate() call). */
    const Eigen::VectorXd &state() const { return x_est_; }

    /** @brief @c true if the most recent QP converged. */
    bool lastConverged() const { return last_converged_; }

    /** @brief Iterations used in the most recent QP solve. */
    int  lastQPIters()   const { return last_qp_iters_; }

    /** @brief Sample time Ts [s]. */
    double sampleTime() const { return Ts_; }

    /**
     * @brief Enable real-time model-plant mismatch detection on the MHE residual (D1).
     *
     * After enabling, every @c estimate() call feeds the one-step-ahead prediction
     * residual `y[k] - C*x_est_[k]` into a CUSUM chart.  Tune @p params.sigma to
     * the expected residual RMS under a consistent model.
     *
     * @param params  CUSUM parameters (sigma, k_cusum, h_threshold).
     */
    void enableMismatchDetection(const MismatchDetectorParams& params = {})
    {
        mismatch_det_.emplace(params);
    }

    /**
     * @brief @c true if the CUSUM statistic has exceeded the detection threshold.
     *
     * Always @c false when @c enableMismatchDetection() has not been called.
     */
    bool mismatchDetected() const
    {
        return mismatch_det_.has_value() && mismatch_det_->detected();
    }

    /**
     * @brief Current CUSUM score from the MHE residual chart.
     *
     * Returns 0.0 when detection is not enabled.
     */
    double mismatchScore() const
    {
        return mismatch_det_.has_value() ? mismatch_det_->score() : 0.0;
    }

    /** @brief Reset mismatch detector accumulators without disabling. */
    void resetMismatchDetector()
    {
        if (mismatch_det_) mismatch_det_->reset();
    }

private:
    StateSpace       plant_;
    MHEParams        params_;
    Eigen::MatrixXd  Q_proc_;  ///< Process noise covariance.
    Eigen::MatrixXd  R_meas_;  ///< Measurement noise covariance.
    Eigen::MatrixXd  P0inv_;   ///< Inverse of arrival-cost covariance.
    double           Ts_;
    int              n_;       ///< State dimension.
    int              m_;       ///< Input dimension.
    int              p_;       ///< Output dimension.

    // History buffers (circular, length N+1 measurements, N inputs)
    std::vector<Eigen::VectorXd> y_hist_; ///< y_hist_[i] = y at window step i.
    std::vector<Eigen::VectorXd> u_hist_; ///< u_hist_[i] = u at window step i.
    Eigen::VectorXd              x_bar_;  ///< Arrival-state prior (x_{k-N} estimate).
    int                          step_count_; ///< Steps since initialize().

    // Condensed QP: H_mhe z + f_mhe = 0
    Eigen::MatrixXd Psi_;       ///< (n*(N+1)) x (n*(N+1)) state transition matrix.
    Eigen::MatrixXd Gamma_u_;   ///< (n*(N+1)) x (m*N) input contribution matrix.
    Eigen::MatrixXd C_bar_;     ///< ((p*(N+1)) x (n*(N+1))) stacked output matrix.
    Eigen::MatrixXd H_mhe_;     ///< QP Hessian (n*(N+1)) x (n*(N+1)).
    Eigen::LDLT<Eigen::MatrixXd> ldlt_;
    double          L_;         ///< Lipschitz constant = lambdamax(H_mhe).

    // Pre-allocated QP work vectors
    Eigen::VectorXd z_sol_, g_qp_, lb_z_, ub_z_, tmp1_, tmp2_;

    // Effective-horizon workspace (sized to full N; sliced via block views during ramp-up)
    Eigen::MatrixXd Psi_eff_ws_, Gamma_u_eff_ws_, C_bar_eff_ws_;
    Eigen::MatrixXd H_prior_ws_, R_bar_ws_;
    Eigen::VectorXd Y_hist_ws_, U_hist_eff_ws_, f_eff_ws_, lb_eff_ws_, ub_eff_ws_, z_prior_ws_, y_fista_ws_;

    Eigen::VectorXd x_est_;    ///< Last returned state estimate.
    bool last_converged_ = true;
    int  last_qp_iters_  = 0;

    std::optional<MismatchDetector> mismatch_det_;  ///< D1: optional residual CUSUM.

    void buildCondensedMatrices();

    /// Propagate x_0 through the horizon using stored u/w history.
    Eigen::VectorXd propagateState(const Eigen::VectorXd &x0,
                                   const Eigen::VectorXd &z) const;

    /// Project x0 onto {x : C_ineq * x <= d_ineq} ∩ [xMin, xMax] via Hildreth's algorithm.
    /// Returns x unchanged when C_ineq is empty.
    Eigen::VectorXd projectX0Polytope(const Eigen::VectorXd &x0) const;
};

} // namespace ctrl
