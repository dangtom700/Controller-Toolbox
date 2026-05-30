#pragma once
#include "IController.h"
#include "PlantModel.h"
#include "GradientProjectionQP.h"
#include "LinearisationHelper.h"
#include <Eigen/Dense>
#include <functional>
#include <vector>

/**
 * @file NonlinearMPC.h
 * @brief Nonlinear MPC via Real-Time Iteration (RTI) with time-varying condensed QP.
 *
 * At each time step the discrete-time nonlinear dynamics
 *   x[k+1] = f(x[k], u[k])
 * are linearised along a warm-started nominal trajectory to produce a
 * time-varying LTV prediction model. A single condensed QP is then solved
 * per step (one RTI iteration), matching the acados/RTI spirit.
 *
 * **Cost:**
 * @code
 *   J = rho_y * sum_{j=1}^{Np} ||y_j - y_ref||^2
 *     + rho_u * sum_{k=0}^{Nu-1} ||Delta_u_k||^2
 * @endcode
 * where Delta_u_k = u_k - u_bar_k is the deviation from the warm-started
 * nominal input.  Box constraints uMin <= u_k <= uMax are enforced via
 * FISTA projection.
 *
 * **Usage pattern (similar to FeedbackLinearisationController):**
 * @code
 *   nmpc.setState(x_current);          // inject current full state
 *   nmpc.setReference(y_ref);          // set output reference (p-vector)
 *   double u0 = nmpc.compute(error);   // SISO wrapper (error = r - y)
 *   // or: VectorXd u = nmpc.computeRef(x, y_ref);   // MIMO
 * @endcode
 *
 * @see Diehl, M. et al. (2005). A real-time iteration scheme for nonlinear
 *      optimization in optimal feedback control. SIAM J. Control Optim. 43(5).
 * @see Camacho & Bordons, "Model Predictive Control" (2007), Ch. 11.
 */

namespace ctrl
{

/**
 * @brief Parameters for NonlinearMPC.
 */
struct NMPCParams
{
    int    Np        = 10;    ///< Prediction horizon [steps].
    int    Nu        = 3;     ///< Control horizon [steps] (<= Np). Last input held beyond Nu.
    double rho_y     = 1.0;   ///< Output tracking weight (scalar, applied to I_p).
    double rho_u     = 0.1;   ///< Input effort weight on Delta_u (scalar, applied to I_m).
    double uMin      = -1e9;  ///< Hard lower bound on each control input element.
    double uMax      =  1e9;  ///< Hard upper bound on each control input element.
    int    qpMaxIter = 500;   ///< FISTA iteration limit.
    double qpTol     = 1e-6;  ///< FISTA convergence tolerance ||Delta_u||_inf.
    double Ts        = 0.01;  ///< Sample time [s].
    int    n_states  = 1;     ///< State dimension n.
    int    n_inputs  = 1;     ///< Input dimension m.
    int    n_outputs = 1;     ///< Output dimension p (must match C rows).
};

/**
 * @brief Nonlinear MPC controller using Real-Time Iteration (RTI).
 *
 * Accepts a discrete-time dynamics function f: (n,m) -> n and an optional
 * output matrix C (p x n).  If C is omitted the identity is used (y = x).
 */
class NonlinearMPC : public IController
{
public:
    /** @brief Discrete-time dynamics: x[k+1] = f(x[k], u[k]). */
    using DiscreteDynamics = std::function<Eigen::VectorXd(const Eigen::VectorXd &x,
                                                           const Eigen::VectorXd &u)>;

    /**
     * @brief Construct NMPC with full-state output (C = I).
     * @param params  Horizon, cost, and constraint parameters.
     * @param f_d     Discrete-time dynamics x[k+1] = f(x[k], u[k]).
     */
    NonlinearMPC(const NMPCParams &params, DiscreteDynamics f_d);

    /**
     * @brief Construct NMPC with explicit output matrix C (p x n).
     * @param params  Horizon, cost, and constraint parameters.
     * @param f_d     Discrete-time dynamics.
     * @param C_out   Output matrix y = C_out * x  (p x n).
     */
    NonlinearMPC(const NMPCParams &params, DiscreteDynamics f_d,
                 const Eigen::MatrixXd &C_out);

    // -------------------------------------------------------------------------
    // IController interface
    // -------------------------------------------------------------------------

    /**
     * @brief SISO compute interface.
     *
     * Reconstructs the scalar output reference as y_ref = C*x_current + error,
     * then runs the RTI step.  setState() must have been called this step.
     *
     * @param error Tracking error e = r - y (scalar).
     * @return First element of the optimal control u*_0.
     */
    double compute(double error) override;

    /** @brief Reset warm-started input sequence, state, and reference to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return p_.Ts; }

    /** @brief Last scalar control output (first element of u*_0). */
    double lastOutput() const { return last_output_; }

    // -------------------------------------------------------------------------
    // MIMO / state-aware interface
    // -------------------------------------------------------------------------

    /**
     * @brief Full MIMO interface - inject state and reference, return u*_0.
     * @param x_current Current full state (n x 1).
     * @param y_ref     Output reference (p x 1).
     * @return Optimal first control input u*_0 (m x 1).
     */
    Eigen::VectorXd computeRef(const Eigen::VectorXd &x_current,
                               const Eigen::VectorXd &y_ref);

    /**
     * @brief Inject the current full state before calling compute() or computeRef().
     * @param x State vector (n x 1).
     */
    void setState(const Eigen::VectorXd &x) { x_current_ = x; }

    /**
     * @brief Set the output reference for the next compute() call.
     * @param y_ref Reference vector (p x 1).
     */
    void setReference(const Eigen::VectorXd &y_ref) { y_ref_ = y_ref; }

    /** @brief @c true if the most recent FISTA QP converged within qpMaxIter. */
    bool lastQPConverged() const { return last_qp_converged_; }

    /** @brief Number of FISTA iterations used in the most recent QP solve. */
    int  lastQPIters()     const { return last_qp_iters_; }

    /** @brief Full optimal first input u*_0 (m x 1). */
    const Eigen::VectorXd &lastControlVec() const { return u_opt_; }

private:
    void init();

    /**
     * @brief RTI step: linearise along warm-start, build Theta and xi, solve QP.
     *
     * On entry x_current_ and y_ref_ are current.
     * On exit U_warm_ is shifted by one step.
     */
    void buildAndSolve();

    NMPCParams      p_;
    DiscreteDynamics f_;
    Eigen::MatrixXd  C_;          ///< Output matrix (p x n).

    Eigen::VectorXd  x_current_;  ///< Current state (n x 1).
    Eigen::VectorXd  y_ref_;      ///< Output reference (p x 1).

    // Warm-started control sequence: U_warm_(m x Nu), column k = u_bar_k.
    Eigen::MatrixXd  U_warm_;

    // Condensed matrices rebuilt every RTI step.
    Eigen::MatrixXd  Theta_;   ///< (p*Np) x (m*Nu): time-varying step-response matrix.
    Eigen::VectorXd  xi_;      ///< (p*Np): nominal output deviation from reference.

    // QP buffers (pre-allocated, size m*Nu).
    Eigen::MatrixXd  H_qp_;
    Eigen::VectorXd  g_qp_;
    Eigen::VectorXd  lb_qp_, ub_qp_;
    Eigen::VectorXd  dU_sol_, tmp1_, tmp2_;
    Eigen::LDLT<Eigen::MatrixXd> ldlt_;
    double           L_qp_ = 1.0;

    Eigen::VectorXd  u_opt_;      ///< u*_0 (m x 1).
    double           last_output_ = 0.0;
    bool             last_qp_converged_ = true;
    int              last_qp_iters_     = 0;
};

} // namespace ctrl
