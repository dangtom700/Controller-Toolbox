#pragma once
#include "IController.h"
#include "PlantModel.h"
#include "GradientProjectionQP.h"
#include <Eigen/Dense>
#include <functional>

/**
 * @file TubeMPC.h
 * @brief Tube Model Predictive Controller - robust MPC for bounded additive disturbances.
 *
 * Addresses the limitation of `DiscreteMPC` and `NonlinearMPC`, which assume perfect
 * predictions. Real systems experience additive disturbances w[k] \in W. Tube-MPC
 * guarantees that the actual state x[k] stays within a "tube" Z around the disturbance-
 * free nominal trajectory x_nom[k]:
 *
 *     x[k] - x_nom[k] \in Z  for all k >= 0,  whenever w[k] \in W.
 *
 * **Algorithm (Mayne, Seron, Rakovic 2005 - simplified box-constraint variant):**
 *
 *   **Offline (at construction):**
 *   1. Compute the minimal robust positively invariant (mRPI) set Z for the error
 *      dynamics  e[k+1] = (A+B*K)*e[k] + w[k]:
 *        z_max <- lim_{N->inf} Sigma |A_cl|^k * w_max  (element-wise, converges if rho(A_cl) < 1)
 *   2. Tighten the input constraints by K*Z:
 *        v_min = u_min + |K|*z_max,   v_max = u_max - |K|*z_max
 *   3. Build the condensed prediction matrices F, Phi (Np*n, Np*Nu) and Hessian H.
 *
 *   **Online (each sample step):**
 *   4. Build the linear cost g from the current nominal state x_nom and reference y_ref.
 *   5. Solve the tightened QP (FISTA) for the nominal input sequence V*.
 *   6. Apply the composite control law:
 *        u[k] = K*(x[k] - x_nom[k]) + V*[0]
 *   7. Update the nominal model:
 *        x_nom[k+1] = A*x_nom[k] + B*V*[0]
 *
 * **Tube property guarantee:**
 *   If x[0] = x_nom[0] (zero initial error) and A+B*K is strictly stable,
 *   then x[k] - x_nom[k] \in Z for all k under any w[k] \in W = {w: |w_i| <= w_max_i}.
 *
 * **Limitation (simplified variant):**
 *   This implementation does not enforce a terminal constraint set (required for
 *   formal recursive feasibility proofs). For most practical plants with a stabilising
 *   K and long horizon Np, this is not an issue in practice.
 *
 * **Usage:**
 * @code
 *   ctrl::TubeMPCParams p;
 *   p.Np = 10;  p.Nu = 3;
 *   p.Q  = Eigen::MatrixXd::Identity(n,n);
 *   p.R  = Eigen::MatrixXd::Identity(m,m) * 0.1;
 *   p.K  = lqr_feedback_gain;            // must make A+B*K stable
 *   p.wMax  = Eigen::VectorXd::Constant(n, 0.05);
 *   p.uMin  = Eigen::VectorXd::Constant(m, -1.0);
 *   p.uMax  = Eigen::VectorXd::Constant(m,  1.0);
 *   p.Ts = 0.1;
 *
 *   ctrl::TubeMPC tmpc(sys, p);
 *   // Each step:
 *   tmpc.setState(x_measured);
 *   tmpc.setReference(y_ref);
 *   Eigen::VectorXd u = tmpc.computeControl();
 * @endcode
 *
 * @see DiscreteMPC     - nominal (no robustness) MPC.
 * @see NonlinearMPC    - RTI-based NMPC (same setState/computeRef pattern).
 * @see AntiWindupWrapper - saturation handling for any IController.
 * @see Mayne, D.Q., Seron, M.M., Rakovic, S.V. (2005). "Robust model predictive
 *      control of constrained linear systems with bounded disturbances."
 *      Automatica 41(2):219-224.
 */

namespace ctrl {

/**
 * @brief Parameters for the Tube MPC controller.
 */
struct TubeMPCParams
{
    int Np = 10;     ///< Prediction horizon [steps]. Should cover approx. settling time.
    int Nu = 3;      ///< Control horizon [steps], Nu <= Np. Inputs held at V*[Nu-1] beyond.

    /**
     * @brief State/output tracking weight matrix (p*p for p outputs).
     *
     * For full-state output (C=I): this is n*n. For scalar plants: 1*1 scalar.
     * Larger Q relative to R: tighter tracking, larger tube corrections.
     */
    Eigen::MatrixXd Q;

    /**
     * @brief Input effort weight matrix (m*m).
     *
     * Larger R: smoother nominal inputs, smaller tightened input range.
     */
    Eigen::MatrixXd R;

    /**
     * @brief Tube feedback gain matrix (m*n).
     *
     * Must make A_cl = A + B*K strictly stable (all eigenvalues inside unit circle).
     * The LQR gain for the same Q,R is a natural choice - it minimises the tube
     * radius z_max_i * |K|_i contribution to the tightened input constraints.
     *
     * @note The feedback convention is: u_tube = K*(x - x_nom). For a stabilising
     * gain in MATLAB lqr() convention (u = -K*x), negate the matrix: K = -K_lqr.
     */
    Eigen::MatrixXd K;

    /**
     * @brief Element-wise disturbance bound (n-vector, each entry >= 0).
     *
     * Defines W = {w : |w_i| <= wMax_i}. The mRPI set radius z_max is computed
     * at construction; z_max_i = Sigma_k |(A+B*K)^k * wMax|_i (infinite series, truncated).
     * Larger wMax -> larger tube -> more tightening -> smaller feasible input range.
     */
    Eigen::VectorXd wMax;

    /** @brief Hard lower bound on u (m-vector). Tightened internally by K*z_max. */
    Eigen::VectorXd uMin;
    /** @brief Hard upper bound on u (m-vector). Tightened internally by K*z_max. */
    Eigen::VectorXd uMax;

    double Ts          = 0.1;   ///< Sample time [s].
    int    qpMaxIter   = 500;   ///< FISTA iteration limit per step.
    double qpTol       = 1e-6;  ///< FISTA convergence tolerance.
    double mRPI_tol    = 1e-8;  ///< mRPI iteration convergence threshold.
    int    mRPI_maxIter = 2000; ///< mRPI iteration limit (divergence guard).
};

/**
 * @brief Tube MPC - robust IController for plants with bounded additive disturbances.
 *
 * Requires setState() and setReference() before each compute() call,
 * mirroring the NonlinearMPC API.
 */
class TubeMPC : public IController
{
public:
    /**
     * @brief Construct Tube MPC for a linear discrete-time plant.
     *
     * Computes the mRPI set Z and the tightened constraints at construction.
     * Factored Hessian and condensed matrices are precomputed here (O(Np^2.n^2.m)).
     *
     * @param sys  Discrete-time state-space model. Must have Ts > 0.
     * @param p    Tube-MPC parameters.
     * @throws std::invalid_argument If dimensions are inconsistent or K is not
     *         stabilising (checked via spectral radius test: rho(A+BK) >= 1).
     */
    TubeMPC(const StateSpace &sys, const TubeMPCParams &p);

    // -------------------------------------------------------------------------
    // Primary API (mirroring NonlinearMPC)
    // -------------------------------------------------------------------------

    /**
     * @brief Set the current measured plant state.
     *
     * Must be called before compute() or computeControl() each step.
     * On the first call, the nominal state x_nom is also initialised to x,
     * ensuring zero initial tube error.
     *
     * @param x Measured state vector (n * 1).
     */
    void setState(const Eigen::VectorXd &x);

    /**
     * @brief Set the output reference for the next solve.
     * @param y_ref Reference vector (p * 1).
     */
    void setReference(const Eigen::VectorXd &y_ref);

    /**
     * @brief Solve the tube-MPC QP and return the composite control input.
     *
     * Equivalent to: setState(x); setReference(y_ref); return computeControl();
     *
     * @param x     Measured state (n * 1).
     * @param y_ref Reference (p * 1).
     * @return      Composite control u[k] = K*(x-x_nom) + v_nom (m * 1).
     */
    Eigen::VectorXd computeRef(const Eigen::VectorXd &x,
                                const Eigen::VectorXd &y_ref);

    /**
     * @brief Solve the tube-MPC QP and return the composite control input.
     *
     * setState() and setReference() must have been called this step.
     *
     * @return Composite control u[k] = K*(x-x_nom) + v_nom (m * 1).
     *         Clamped to [uMin, uMax].
     */
    Eigen::VectorXd computeControl();

    // -------------------------------------------------------------------------
    // IController interface
    // -------------------------------------------------------------------------

    /**
     * @brief Scalar IController wrapper (SISO plants only).
     *
     * Interprets @p error as the scalar tracking error e = r - y and calls
     * setState+setReference using internally stored x and r = y+error.
     * setState() MUST have been called this step for x to be known.
     *
     * @param error Tracking error e[k] = r[k] - y[k].
     * @return      Composite control u[k] (scalar).
     */
    double compute(double error) override;

    /** @brief Reset nominal state, warm-start, and health flag. */
    void reset() override;

    /** @return Sample time Ts [s]. */
    double sampleTime() const override { return p_.Ts; }

    /** @return false if the last QP solve did not converge within qpMaxIter. */
    bool isHealthy() const override { return qp_converged_; }

    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------

    /** @return true if the last QP solve converged. */
    bool lastQPConverged() const { return qp_converged_; }

    /**
     * @brief Radius of the mRPI tube (element-wise, n * 1).
     *
     * The actual state satisfies |x[k] - x_nom[k]|_i <= tubeRadius()[i]
     * for all k under any w[k] \in W.
     */
    const Eigen::VectorXd &tubeRadius() const { return z_max_; }

    /** @return Current nominal state x_nom (n * 1). */
    const Eigen::VectorXd &nominalState() const { return x_nom_; }

    /**
     * @brief Tightened lower input bound after mRPI constraint tightening (m * 1).
     */
    const Eigen::VectorXd &tightenedUMin() const { return v_min_; }
    /** @brief Tightened upper input bound after mRPI constraint tightening (m * 1). */
    const Eigen::VectorXd &tightenedUMax() const { return v_max_; }

private:
    void computeMRPI();
    void buildCondensedMatrices();

    const StateSpace sys_;
    TubeMPCParams    p_;

    int n_;   ///< State dimension.
    int m_;   ///< Input dimension.
    int pp_;  ///< Output dimension.

    Eigen::VectorXd z_max_;   ///< mRPI tube radius (n).
    Eigen::VectorXd v_min_;   ///< Tightened u lower bound (m).
    Eigen::VectorXd v_max_;   ///< Tightened u upper bound (m).

    // Condensed QP matrices (precomputed)
    Eigen::MatrixXd F_;    ///< (Np*pp) x n  - free evolution of outputs
    Eigen::MatrixXd Phi_;  ///< (Np*pp) x (Nu*m) - response to V (with hold-last)
    Eigen::MatrixXd H_;    ///< (Nu*m) x (Nu*m) - QP Hessian
    Eigen::LDLT<Eigen::MatrixXd> ldlt_; ///< Factored H for warm-start
    double L_inv_;         ///< 1 / lambda_max(H) for FISTA step

    // Stacked bounds for QP
    Eigen::VectorXd lb_;   ///< (Nu*m) lower bound
    Eigen::VectorXd ub_;   ///< (Nu*m) upper bound

    // Online state
    Eigen::VectorXd x_actual_;  ///< Current measured state (n).
    Eigen::VectorXd x_nom_;     ///< Current nominal state (n).
    Eigen::VectorXd y_ref_;     ///< Current reference (pp).
    Eigen::VectorXd V_warm_;    ///< QP warm-start (Nu*m).
    bool first_step_{true};     ///< True before first setState() call.
    bool state_set_{false};     ///< True after setState() in current step.
    bool qp_converged_{true};

    // QP scratch vectors (pre-allocated)
    Eigen::VectorXd g_;    ///< Linear cost vector (Nu*m).
    Eigen::VectorXd tmp1_, tmp2_; ///< FISTA scratch (Nu*m).
};

} // namespace ctrl
