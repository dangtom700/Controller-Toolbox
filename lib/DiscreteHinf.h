#pragma once
#include "IController.h"
#include "PlantModel.h"
#include <Eigen/Dense>
#include <optional>

/**
 * @file DiscreteHinf.h
 * @brief Discrete-time H∞ controller synthesis (DGKF two-Riccati method).
 *
 * Synthesises a discrete-time dynamic output-feedback controller K(z) that minimises the
 * closed-loop H∞ norm: ‖F_l(P, K)‖∞ < γ, where P is the generalised plant (augmented with
 * weighting functions) and F_l denotes the lower Linear Fractional Transformation.
 *
 * **Generalised plant format:**
 * @code
 *   x[k+1] = A·x  + B1·w + B2·u
 *   z[k]   = C1·x + D11·w + D12·u   (performance output)
 *   y[k]   = C2·x + D21·w + D22·u   (measurement output)
 * @endcode
 *
 * **Mixed-sensitivity design (most common):** Use MixedSensitivity to build P automatically
 * from a SISO plant G and weighting functions W1 (tracking), W2 (control effort), W3 (robustness):
 * @code
 *   ‖[ W1·S  ]‖
 *   ‖[ W2·KS ]‖  < γ
 *   ‖[ W3·T  ]‖∞
 * @endcode
 *
 * **Algorithm:** Discrete DGKF (Doyle, Glover, Khargonekar & Francis 1989; discrete version
 * by Stoorvogel 1992 / Iglesias & Glover 1991). A γ-bisection drives the achieved norm
 * to within HinfParams::gammaTol of the optimum.
 *
 * @code
 * // Typical usage
 * auto G = ctrl::c2d(G_c, 0.01, ctrl::C2dMethod::ZOH);
 * auto W1 = ctrl::MixedSensitivity::makeW1(10.0, 1.0, 1e-3, 0.01);
 * auto W2 = ctrl::MixedSensitivity::makeW2constant(0.2, 0.01);
 * auto W3 = ctrl::MixedSensitivity::makeW3(50.0, 1.5, 1e-3, 0.01);
 * auto P  = ctrl::MixedSensitivity::build(G, W1, W2, W3);
 * ctrl::HinfParams hp;
 * auto result = ctrl::DiscreteHinf::solve(P, hp);
 * if (result.feasible) {
 *     ctrl::DiscreteHinf ctrl(result);
 *     double u = ctrl.compute(y);
 * }
 * @endcode
 *
 * @see Doyle, Glover, Khargonekar & Francis, IEEE TAC 34(8), 1989.
 * @see Stoorvogel, "The Discrete-Time H∞ Control Problem," SIAM 1992.
 * @see Iglesias & Glover, Int. J. Control 54(5), 1991.
 * @see Skogestad & Postlethwaite, "Multivariable Feedback Design" Ch. 9, 2005.
 * @see MATLAB hinfsyn() documentation.
 */

namespace ctrl
{

/**
 * @brief Generalised plant for H∞ synthesis.
 *
 * Dimensions: n = state, nw = exogenous inputs, nu = control inputs,
 * nz = performance outputs, ny = measurement outputs.
 */
struct GeneralisedPlant
{
    Eigen::MatrixXd A;   ///< State matrix (n × n).
    Eigen::MatrixXd B1;  ///< Exogenous input matrix (n × nw).
    Eigen::MatrixXd B2;  ///< Control input matrix (n × nu).
    Eigen::MatrixXd C1;  ///< Performance output matrix (nz × n).
    Eigen::MatrixXd C2;  ///< Measurement output matrix (ny × n).
    Eigen::MatrixXd D11; ///< Exogenous→performance feedthrough (nz × nw).
    Eigen::MatrixXd D12; ///< Control→performance feedthrough (nz × nu).
    Eigen::MatrixXd D21; ///< Exogenous→measurement feedthrough (ny × nw).
    Eigen::MatrixXd D22; ///< Control→measurement feedthrough (ny × nu).
    double Ts = 0.01;    ///< Sample time [s].

    int stateSize() const { return static_cast<int>(A.rows()); }  ///< State dimension n.
    int nw()        const { return static_cast<int>(B1.cols()); } ///< Exogenous input count.
    int nu()        const { return static_cast<int>(B2.cols()); } ///< Control input count.
    int nz()        const { return static_cast<int>(C1.rows()); } ///< Performance output count.
    int ny()        const { return static_cast<int>(C2.rows()); } ///< Measurement output count.
};

/**
 * @brief H∞ synthesis algorithm parameters.
 */
struct HinfParams
{
    double gammaInit   = 10.0;  ///< Initial upper bound for γ bisection.
    double gammaTol    = 1e-3;  ///< Stop bisection when γ bracket < this.
    int    maxIter     = 60;    ///< Maximum bisection iterations.
    double dareTol     = 1e-12; ///< DARE convergence tolerance.
    int    dareMaxIter = 200;   ///< DARE iteration limit.
};

/**
 * @brief Result returned by DiscreteHinf::solve().
 */
struct HinfResult
{
    bool   feasible       = false; ///< @c true when synthesis succeeded.
    double achievedGamma  = 0.0;   ///< Smallest feasible γ found by bisection.
    double Ts             = 0.0;   ///< Plant sample time.

    /**
     * @name Controller matrices — K(z): xk[k+1] = Ak·xk + Bk·y,  u = Ck·xk + Dk·y
     * @{
     */
    Eigen::MatrixXd Ak; ///< Controller state transition (nk × nk, nk = plant state size).
    Eigen::MatrixXd Bk; ///< Controller input gain (nk × ny).
    Eigen::MatrixXd Ck; ///< Controller output gain (nu × nk).
    Eigen::MatrixXd Dk; ///< Controller feedthrough (nu × ny).
    /** @} */

    /** @name DARE solutions (available for diagnostics) @{ */
    Eigen::MatrixXd X_inf; ///< Control Riccati solution.
    Eigen::MatrixXd Y_inf; ///< Filter Riccati solution.
    /** @} */

    /** @name Synthesis diagnostics @{ */
    int    dareItersX    = 0;
    int    dareItersY    = 0;
    bool   dareConvX     = false;
    bool   dareConvY     = false;
    double spectralRadius = 0.0; ///< ρ(X∞·Y∞); must be < γ² for feasibility.
    /** @} */
};

/**
 * @brief Discrete-time dynamic output-feedback H∞ controller.
 *
 * Implements the synthesised controller K(z):
 * @code
 *   xk[k+1] = Ak·xk[k] + Bk·y[k]
 *   u[k]    = Ck·xk[k] + Dk·y[k]
 * @endcode
 */
class DiscreteHinf : public IController
{
public:
    /**
     * @brief Construct from a completed synthesis result.
     * @param result A HinfResult with feasible == true.
     */
    explicit DiscreteHinf(const HinfResult &result);

    /**
     * @brief Solve the discrete-time H∞ optimal control problem.
     *
     * Bisects γ from [gammaLow, params.gammaInit] until the bracket is narrower than
     * params.gammaTol. gammaLow is set automatically.
     *
     * @param P      Generalised plant (use MixedSensitivity::build() for S/KS/T design).
     * @param params Synthesis algorithm parameters.
     * @return HinfResult containing controller matrices and diagnostics.
     *         Check result.feasible before constructing a DiscreteHinf.
     */
    static HinfResult solve(const GeneralisedPlant &P, const HinfParams &params = {});

    /**
     * @brief Compute u[k] — SISO interface (ny = 1, nu = 1).
     *
     * @p signal is the plant measurement y[k] **not** the tracking error.
     * @param signal Measurement output y[k].
     * @return Control input u[k].
     */
    double compute(double signal) override;

    /**
     * @brief Compute u[k] — MIMO interface.
     * @param y Measurement vector y[k] (ny × 1).
     * @return Control vector u[k] (nu × 1).
     */
    Eigen::VectorXd computeVec(const Eigen::VectorXd &y) override;

    /** @brief Reset controller internal state xk to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Current controller internal state xk (nk × 1). */
    const Eigen::VectorXd &controllerState() const { return xk_; }

    /** @brief Achieved H∞ bound γ from synthesis. */
    double achievedGamma() const { return gamma_; }

    /** @name Read-only access to controller matrices @{ */
    const Eigen::MatrixXd &Ak() const { return Ak_; }
    const Eigen::MatrixXd &Bk() const { return Bk_; }
    const Eigen::MatrixXd &Ck() const { return Ck_; }
    const Eigen::MatrixXd &Dk() const { return Dk_; }
    /** @} */

private:
    Eigen::MatrixXd Ak_, Bk_, Ck_, Dk_;
    Eigen::VectorXd xk_;
    double Ts_;
    double gamma_;

    struct DareOut { Eigen::MatrixXd X; bool conv; int iters; };
    static DareOut solveHinfDARE(const Eigen::MatrixXd &A,
                                 const Eigen::MatrixXd &B,
                                 const Eigen::MatrixXd &Q,
                                 const Eigen::MatrixXd &R,
                                 double tol, int maxIter);

    static bool trySolve(const GeneralisedPlant &P, double gamma,
                         double dareTol, int dareMaxIter,
                         HinfResult &out);
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Helper class that assembles a GeneralisedPlant for S/KS/T mixed-sensitivity design.
 *
 * Augments a SISO discrete-time plant G(z) with three weighting state-space models:
 * - **W1** — tracking / disturbance rejection (high gain at low frequencies).
 * - **W2** — control effort / actuator limits (limits high-frequency commands).
 * - **W3** — robustness / unmodelled dynamics (forces T rolloff at high frequencies).
 *
 * @par Infeasibility troubleshooting
 * If solve() returns feasible = @c false:
 * 1. Increase gammaInit (start at 10·‖W1‖∞·‖W3‖∞ as a rule of thumb).
 * 2. Verify W1 crossover is below the plant open-loop bandwidth.
 * 3. For non-minimum-phase plants, the achievable sensitivity peak is bounded below by
 *    the Poisson integral; reduce W1 gain or relax the crossover requirement
 *    (Skogestad & Postlethwaite §6.3).
 * 4. If W1 and W3 both have high gain in overlapping bands, the design may be
 *    fundamentally infeasible (S + T = 1). Reduce |W1(jω)|·|W3(jω)|.
 * 5. If gammaInit is below the achievable γ, bisection cannot start — increase it.
 */
class MixedSensitivity
{
public:
    /**
     * @brief Sensitivity weight W1 for tracking and disturbance rejection.
     *
     * Continuous: W1(s) = (s/M + ωB) / (s + ωB·ε).
     * Low-freq gain ≈ 1/ε, crossover at ωB, high-freq gain ≈ M.
     *
     * @param omega_B Crossover frequency [rad/s].
     * @param M       High-frequency gain bound.
     * @param eps     Low-frequency sensitivity tolerance (e.g., 0.001 for 1% steady-state error).
     * @param Ts      Sample time [s] (Tustin discretisation).
     */
    static StateSpace makeW1(double omega_B, double M, double eps, double Ts);

    /**
     * @brief Control effort weight W2 — flat gain at all frequencies.
     *
     * Simple actuator-level constraint: set gain = 1/u_max.
     *
     * @param gain Scalar gain value.
     * @param Ts   Sample time [s].
     */
    static StateSpace makeW2constant(double gain, double Ts);

    /**
     * @brief Control effort weight W2 — first-order high-pass.
     *
     * Continuous: W2(s) = (s + ωu·ε) / (s/1 + ωu).
     * Low-freq gain ≈ ε, high-freq gain ≈ 1, rolloff at ωu.
     *
     * @param omega_u Rolloff frequency [rad/s].
     * @param eps     Low-frequency gain floor.
     * @param Ts      Sample time [s].
     */
    static StateSpace makeW2highpass(double omega_u, double eps, double Ts);

    /**
     * @brief Robustness weight W3 for complementary sensitivity rolloff.
     *
     * Continuous: W3(s) = (s + ωT/Mt) / (ε·s + ωT).
     * Low-freq gain ≈ ε, high-freq gain ≈ 1/Mt, rollup above ωT.
     *
     * @param omega_T Rollup frequency [rad/s].
     * @param Mt      Maximum complementary sensitivity (e.g., 1.2–2.0).
     * @param eps     Low-frequency gain floor.
     * @param Ts      Sample time [s].
     */
    static StateSpace makeW3(double omega_T, double Mt, double eps, double Ts);

    /**
     * @brief Assemble the generalised plant from G, W1, W2, W3.
     *
     * All inputs must be discrete-time (Ts > 0) with the same sample time. G must be SISO.
     * W1, W2, W3 may be any order (first-order recommended for efficiency).
     *
     * Augmented state: xₐ = [xG; xW1; xW2; xW3].
     * Exogenous inputs: w = [r; d] (reference + output disturbance), nw = 2.
     * Performance outputs: z = [W1·(r−y); W2·u; W3·y], nz = 3.
     *
     * @param G  SISO discrete-time plant.
     * @param W1 Tracking weight.
     * @param W2 Control effort weight.
     * @param W3 Robustness weight.
     * @return Assembled GeneralisedPlant ready for DiscreteHinf::solve().
     */
    static GeneralisedPlant build(const StateSpace &G,
                                  const StateSpace &W1,
                                  const StateSpace &W2,
                                  const StateSpace &W3);
};

} // namespace ctrl
