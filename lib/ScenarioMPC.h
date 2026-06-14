#pragma once
#include "IController.h"
#include "PlantModel.h"
#include "GradientProjectionQP.h"
#include "ControllerRegistry.h"
#include <Eigen/Dense>
#include <random>
#include <string>

/**
 * @file ScenarioMPC.h
 * @brief Scenario-based Stochastic Model Predictive Controller.
 *
 * Extends deterministic MPC to handle **Gaussian process noise** w[k] ~ N(0, Sigma_w).
 * Rather than ignoring noise (DiscreteMPC) or bounding it worst-case (TubeMPC),
 * ScenarioMPC averages the cost over N_s sampled noise trajectories:
 *
 * @code
 *   J = (1/N_s) * sum_i sum_j || y^(i)[j|k] - r ||^2_Q + || u[j] ||^2_R
 *   s.t.  u_min <= u[j] <= u_max  for all j
 * @endcode
 *
 * Because the same input sequence U is applied to all scenarios (open-loop),
 * the average cost collapses to a standard QP:
 *
 * @code
 *   H = 2*(Phi' * Q_blk * Phi + R_blk)           (constant -- precomputed)
 *   g = 2 * Phi' * Q_blk * (F*x + avgW - R)      (per step)
 * @endcode
 *
 * where avgW = (1/N_s) * sum_i W^(i) is the average noise-propagated output
 * bias, and W^(i)[j] = C * sum_{l=0}^{j-1} A^(j-1-l) * w^(i)[l].
 *
 * **Key differences from TubeMPC:**
 * - No stabilising gain K and no mRPI tube computation.
 * - Unbounded Gaussian noise (not a bounded set W).
 * - Less conservative than TubeMPC; provides a noise-averaged optimum.
 * - For N_s -> inf, avgW -> 0 and the solution matches deterministic MPC.
 *   For small N_s, there is variance in the solution (stochastic).
 *
 * **Usage:**
 * @code
 *   ctrl::ScenarioMPCParams p;
 *   p.Np = 10; p.Nu = 3;
 *   p.Q  = Eigen::MatrixXd::Identity(n, n);
 *   p.R  = Eigen::MatrixXd::Identity(m, m) * 0.1;
 *   p.Sigma_w = Eigen::MatrixXd::Identity(n, n) * 0.01;  // process noise covariance
 *   p.N_samples = 30;
 *   p.uMin = Eigen::VectorXd::Constant(m, -1.0);
 *   p.uMax = Eigen::VectorXd::Constant(m,  1.0);
 *
 *   ctrl::ScenarioMPC smpc(sys, p);
 *   smpc.setState(x_measured);
 *   smpc.setReference(y_ref);
 *   Eigen::VectorXd u = smpc.computeControl();
 * @endcode
 *
 * @see TubeMPC     -- bounded disturbance robust MPC.
 * @see DiscreteMPC -- deterministic MPC (noise-free).
 * @see Calafiore, G.C. & Campi, M.C. (2006). The scenario approach to robust control
 *      design. IEEE TAC 51(5):742-753.
 * @see Schildbach, G., Fagiano, L., Frei, C. & Morari, M. (2014). The scenario approach
 *      for stochastic model predictive control with bounds on closed-loop constraint
 *      violations. Automatica 50(12):3009-3018.
 */

namespace ctrl {

/**
 * @brief Parameters for ScenarioMPC.
 */
struct ScenarioMPCParams
{
    int Np = 10;   ///< Prediction horizon [steps].
    int Nu = 3;    ///< Control horizon [steps], Nu <= Np.

    /** @brief Output tracking weight (pp x pp). */
    Eigen::MatrixXd Q;
    /** @brief Input effort weight (m x m). */
    Eigen::MatrixXd R;

    /**
     * @brief Process noise covariance matrix (n x n).
     *
     * At each step and each horizon sample, w[k] ~ N(0, Sigma_w).
     * Zero Sigma_w reduces ScenarioMPC to deterministic MPC.
     */
    Eigen::MatrixXd Sigma_w;

    /**
     * @brief Number of noise scenarios sampled per control step.
     *
     * Larger N_samples reduces variance in the cost estimate at the expense
     * of computation. N_samples = 1 gives maximum variance; N_samples ~ 20-50
     * is a practical compromise for most applications.
     */
    int N_samples = 30;

    /** @brief Hard lower bound on u (m-vector). */
    Eigen::VectorXd uMin;
    /** @brief Hard upper bound on u (m-vector). */
    Eigen::VectorXd uMax;

    double Ts          = 0.1;   ///< Sample time [s].
    int    qpMaxIter   = 500;   ///< FISTA iteration limit per step.
    double qpTol       = 1e-6;  ///< FISTA convergence tolerance.
    unsigned int seed  = 42;    ///< RNG seed for reproducibility.
};

/**
 * @brief Scenario-based Stochastic MPC -- noise-averaged QP with FISTA.
 *
 * Mirrors the TubeMPC API: setState() + setReference() + computeControl(),
 * plus a compute(error) scalar IController shortcut.
 */
class ScenarioMPC : public IController
{
public:
    /**
     * @brief Construct ScenarioMPC for a linear discrete-time plant.
     *
     * Precomputes F, Phi, H, and the Cholesky factor of Sigma_w.
     *
     * @param sys  Discrete-time state-space model (Ts > 0 required).
     * @param p    ScenarioMPC parameters.
     * @throws std::invalid_argument if dimensions are inconsistent or Sigma_w
     *         is not positive semi-definite.
     */
    ScenarioMPC(const StateSpace &sys, const ScenarioMPCParams &p);

    // -------------------------------------------------------------------------
    // Primary MIMO API (mirrors TubeMPC)
    // -------------------------------------------------------------------------

    /** @brief Set the current measured plant state (n x 1). */
    void setState(const Eigen::VectorXd &x);

    /** @brief Set the output reference for the next solve (pp x 1). */
    void setReference(const Eigen::VectorXd &y_ref);

    /**
     * @brief Convenience: setState + setReference + computeControl in one call.
     * @param x      Measured state (n x 1).
     * @param y_ref  Reference (pp x 1).
     * @return       Optimal first control action u[0] (m x 1).
     */
    Eigen::VectorXd computeRef(const Eigen::VectorXd &x,
                                const Eigen::VectorXd &y_ref);

    /**
     * @brief Sample N_s noise trajectories, compute average cost, solve QP.
     *
     * setState() and setReference() must have been called this step.
     *
     * @return Optimal first control action u[0] (m x 1), clamped to [uMin, uMax].
     */
    Eigen::VectorXd computeControl();

    // -------------------------------------------------------------------------
    // IController interface (SISO)
    // -------------------------------------------------------------------------

    /**
     * @brief Scalar SISO wrapper: reference = C*x_nom + error.
     *
     * Equivalent to calling setState() + setReference(y_est + error) +
     * computeControl()(0), using the stored nominal state for y_est.
     */
    double compute(double error) override;
    void   reset()               override;
    double sampleTime() const    override { return p_.Ts; }
    bool   isHealthy()  const    override { return qp_converged_; }
    std::string name()  const    override { return "ScenarioMPC"; }

    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------

    /** @brief True if the last QP solve converged within qpMaxIter. */
    bool lastQPConverged() const noexcept { return qp_converged_; }

    /** @brief Current nominal state x_nom (n x 1). */
    const Eigen::VectorXd &nominalState() const noexcept { return x_nom_; }

    /** @brief Average noise-propagated output bias from the last solve (Np*pp x 1). */
    const Eigen::VectorXd &lastAvgNoiseBias() const noexcept { return avg_W_; }

private:
    void buildCondensedMatrices();

    StateSpace        sys_;
    ScenarioMPCParams p_;

    int n_;   ///< State dimension.
    int m_;   ///< Input dimension.
    int pp_;  ///< Output dimension.

    // Condensed QP matrices (precomputed at construction)
    Eigen::MatrixXd  F_;        ///< (Np*pp) x n
    Eigen::MatrixXd  Phi_;      ///< (Np*pp) x (Nu*m)
    Eigen::MatrixXd  PhiTQy_;   ///< (Nu*m)  x (Np*pp) -- Phi'*Q_blk, precomputed
    Eigen::MatrixXd  H_;        ///< (Nu*m)  x (Nu*m)
    Eigen::LDLT<Eigen::MatrixXd> ldlt_;
    double           L_inv_;

    // Input box constraints (stacked)
    Eigen::VectorXd lb_;
    Eigen::VectorXd ub_;

    // Cholesky factor of Sigma_w for noise sampling
    Eigen::MatrixXd Sigma_w_chol_;  ///< Lower-triangular L: Sigma_w = L*L'

    // Online state
    Eigen::VectorXd x_nom_;      ///< Nominal state (n).
    Eigen::VectorXd y_ref_;      ///< Current reference (pp).
    Eigen::VectorXd V_warm_;     ///< QP warm-start (Nu*m).
    Eigen::VectorXd avg_W_;      ///< Average noise bias (Np*pp) -- last solve.
    Eigen::VectorXd g_;          ///< Linear QP cost (Nu*m).
    Eigen::VectorXd tmp1_;       ///< FISTA scratch.
    Eigen::VectorXd tmp2_;       ///< FISTA scratch.
    Eigen::VectorXd y_fista_;    ///< FISTA momentum point (pre-allocated).
    Eigen::VectorXd x_noise_;    ///< Per-scenario noise accumulation (n_).
    Eigen::VectorXd z_noise_;    ///< Standard-normal sample vector (n_).
    Eigen::VectorXd R_stacked_;  ///< Stacked reference vector (Np*pp_).
    bool            first_step_  = true;
    bool            qp_converged_ = true;
    double          u_prev_       = 0.0; ///< Last finite compute(error) scalar (hold-last NaN contract).

    // RNG
    std::mt19937                     rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(scenario_mpc)
