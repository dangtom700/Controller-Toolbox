#pragma once
#include "IController.h"
#include "DiscreteHinf.h"   // GeneralisedPlant (shared with Hinf synthesis)
#include "PlantModel.h"     // StateSpace
#include <Eigen/Dense>

/**
 * @file DiscreteH2.h
 * @brief Discrete-time H2-optimal (LQG) dynamic output-feedback synthesis.
 *
 * Synthesises a discrete-time dynamic output-feedback controller K(z) that minimises the
 * closed-loop H2 norm ||F_l(P, K)||_2, where P is the same GeneralisedPlant format used by
 * DiscreteHinf. Solved via the standard discrete LQG separation principle: a control Riccati
 * equation (LQR-with-cross-term) and a dual filter Riccati equation (Kalman-filter-with-
 * cross-term), each reduced to a standard (no-cross-term) DARE via the textbook
 * cross-term-elimination substitution and solved with DiscreteLQR::solveDARE.
 *
 * **D11 (exogenous-to-performance feedthrough) is fully supported.** With Dk = 0 (always true
 * for the controller this class assembles), u[k] depends only on the controller state xk[k] -
 * a function of past y's, hence past w's, never w[k] itself - so for zero-mean white w the
 * cross term E[(C1 x[k] + D12 u[k])' D11 w[k]] is exactly zero regardless of the controller.
 * Consequence: D11 never enters the control or filter Riccati equations (the cross-term-
 * eliminated DARE reduction below was never a function of D11 to begin with), so the same F/L
 * derivation is exactly correct for any D11. The only place D11 matters is the achieved H2 norm,
 * which picks up an extra trace(D11*D11') term (a direct feedthrough contributes a constant,
 * controller-independent amount to the H2 norm) - see DiscreteH2::solve()'s implementation.
 * This means MixedSensitivity::build()'s S/KS/T-construction plants (which have D11 != 0 from
 * the W1/W3 weight gains) can be passed directly, unlike in earlier revisions of this class.
 *
 * **Regularity assumptions still enforced by throwing:**
 * - D22 = 0 (no direct control-to-measurement feedthrough) - the Dk = 0 assembly requires this.
 * - D12 full column rank, D21 full row rank (same DGKF-style regularity DiscreteHinf::solve()
 *   already requires).
 *
 * @code
 * // D11 may be nonzero - e.g. a MixedSensitivity::build()-produced plant works directly.
 * ctrl::H2Result result = ctrl::DiscreteH2::solve(P);
 * if (result.feasible) {
 *     ctrl::DiscreteH2 ctrl(result);
 *     double u = ctrl.compute(y);
 * }
 * @endcode
 *
 * @see de Souza & Xie, "On the Discrete-Time Bounded Real Lemma with Application in the
 *      Characterization of Static State Feedback H-infinity Controllers", Systems & Control
 *      Letters 18 (1992) - the discrete LQG/H2 separation result this mirrors.
 * @see Zhou, Doyle & Glover, "Robust and Optimal Control" (1996), Chapter 14.
 * @see Astrom & Wittenmark, "Computer-Controlled Systems" (1997), Chapter 11 (discrete LQG).
 */

#if defined(CTRL_HAS_HINF)

namespace ctrl
{

/**
 * @brief H2 synthesis parameters.
 *
 * Currently empty: the underlying DiscreteLQR::solveDARE doubling-iteration solver has no
 * tunable tolerance/iteration-limit parameters to forward. Kept as a struct (rather than
 * dropping the parameter) for API symmetry with DiscreteHinf::solve(P, HinfParams) and as an
 * extension point if solveDARE gains configurable tolerances in the future.
 */
struct H2Params
{
};

/**
 * @brief Result returned by DiscreteH2::solve().
 */
struct H2Result
{
    bool   feasible       = false; ///< @c true when synthesis succeeded.
    double achievedH2Norm  = 0.0;  ///< Closed-loop H2 norm ||F_l(P,K)||_2 at the solution.
    double Ts              = 0.0; ///< Plant sample time.

    /**
     * @name Controller matrices - K(z): xk[k+1] = Ak.xk + Bk.y,  u = Ck.xk + Dk.y
     * @{
     */
    Eigen::MatrixXd Ak; ///< Controller state transition (n * n, n = plant state size).
    Eigen::MatrixXd Bk; ///< Controller input gain (n * ny).
    Eigen::MatrixXd Ck; ///< Controller output gain (nu * n).
    Eigen::MatrixXd Dk; ///< Controller feedthrough (nu * ny); always zero for H2.
    /** @} */

    /**
     * @name Riccati solutions (available for diagnostics)
     * @{
     */
    Eigen::MatrixXd X; ///< Control Riccati solution.
    Eigen::MatrixXd Y; ///< Filter Riccati solution.
    /** @} */

    bool dareConvX = false; ///< Control DARE convergence flag.
    bool dareConvY = false; ///< Filter DARE convergence flag.
};

/**
 * @brief Discrete-time dynamic output-feedback H2-optimal (LQG) controller.
 *
 * Implements the synthesised controller K(z), identical in shape to DiscreteHinf:
 * @code
 *   xk[k+1] = Ak.xk[k] + Bk.y[k]
 *   u[k]    = Ck.xk[k] + Dk.y[k]
 * @endcode
 */
class DiscreteH2 : public IController
{
public:
    /**
     * @brief Construct from a completed synthesis result.
     * @param result A H2Result with feasible == true.
     * @throws std::invalid_argument If result.feasible is false.
     */
    explicit DiscreteH2(const H2Result &result);

    /**
     * @brief Solve the discrete-time H2-optimal output-feedback control problem.
     *
     * @param P      Generalised plant with D11 = 0, D22 = 0, D12 full column rank,
     *               D21 full row rank.
     * @param params Synthesis parameters (currently unused - see H2Params).
     * @return H2Result containing controller matrices and diagnostics.
     *         Check result.feasible before constructing a DiscreteH2.
     * @throws std::invalid_argument If P has empty dimensions, D12/D21 are rank-deficient,
     *         or D11/D22 are nonzero (see class docs for why D11/D22 = 0 is required here).
     */
    static H2Result solve(const GeneralisedPlant &P, const H2Params &params = {});

    /**
     * @brief Compute u[k] - SISO interface (ny = 1, nu = 1).
     *
     * @p signal is the plant measurement y[k], not the tracking error (same convention as
     * DiscreteHinf).
     * @param signal Measurement output y[k].
     * @return Control input u[k].
     */
    double compute(double signal) override;

    /**
     * @brief Compute u[k] - MIMO interface.
     * @param y Measurement vector y[k] (ny * 1).
     * @return Control vector u[k] (nu * 1).
     */
    Eigen::VectorXd computeVec(const Eigen::VectorXd &y) override;

    /** @brief Reset controller internal state xk to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Current controller internal state xk (n * 1). */
    const Eigen::VectorXd &controllerState() const { return xk_; }

    /** @brief Achieved H2 norm from synthesis. */
    double achievedH2Norm() const { return h2norm_; }

    /**
     * @name Read-only access to controller matrices
     * @{
     */
    const Eigen::MatrixXd &Ak() const { return Ak_; }
    const Eigen::MatrixXd &Bk() const { return Bk_; }
    const Eigen::MatrixXd &Ck() const { return Ck_; }
    const Eigen::MatrixXd &Dk() const { return Dk_; }
    /** @} */

private:
    Eigen::MatrixXd Ak_, Bk_, Ck_, Dk_;
    Eigen::VectorXd xk_;
    Eigen::VectorXd u_work_; ///< Pre-allocated control output vector for computeVec().
    double Ts_;
    double h2norm_;
    double u_prev_ = 0.0; ///< Last finite compute() scalar (hold-last NaN contract).

    // Builds the [x;xk] -> [x+;xk+], w -> z closed-loop state-space (A_cl, B_cl, C_cl) used
    // to evaluate achievedH2Norm. Simpler than DiscreteHinf::buildClosedLoop since D11 = D22 =
    // Dk = 0 always holds here (no E = (I-Dk*D22)^-1 term, no D_cl term needed for the norm).
    static void buildClosedLoop(const GeneralisedPlant &P,
                                const Eigen::MatrixXd  &Ak, const Eigen::MatrixXd &Bk,
                                const Eigen::MatrixXd  &Ck,
                                Eigen::MatrixXd &A_cl, Eigen::MatrixXd &B_cl,
                                Eigen::MatrixXd &C_cl);
};

} // namespace ctrl

#include "ControllerRegistry.h"
CTRL_REGISTER_FEATURE(h2_synthesis)

#endif // CTRL_HAS_HINF
