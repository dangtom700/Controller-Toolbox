#pragma once
#include "PlantModel.h"
#include "GradientProjectionQP.h"
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

    Eigen::VectorXd x_est_;    ///< Last returned state estimate.
    bool last_converged_ = true;
    int  last_qp_iters_  = 0;

    void buildCondensedMatrices();

    /// Propagate x_0 through the horizon using stored u/w history.
    Eigen::VectorXd propagateState(const Eigen::VectorXd &x0,
                                   const Eigen::VectorXd &z) const;
};

} // namespace ctrl
