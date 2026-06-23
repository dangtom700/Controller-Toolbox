#pragma once
#include "IController.h"
#include "PlantModel.h"
#include "AutoTuner.h"
#include "SystemAnalysis.h"
#include <Eigen/Dense>
#include <functional>
#include <optional>

/**
 * @file DiscreteHinf.h
 * @brief Discrete-time Hinf controller synthesis (DGKF two-Riccati method).
 *
 * Synthesises a discrete-time dynamic output-feedback controller K(z) that minimises the
 * closed-loop Hinf norm: ||F_l(P, K)||inf < gamma, where P is the generalised plant (augmented with
 * weighting functions) and F_l denotes the lower Linear Fractional Transformation.
 *
 * **Generalised plant format:**
 * @code
 *   x[k+1] = A.x  + B1.w + B2.u
 *   z[k]   = C1.x + D11.w + D12.u   (performance output)
 *   y[k]   = C2.x + D21.w + D22.u   (measurement output)
 * @endcode
 *
 * **Mixed-sensitivity design (most common):** Use MixedSensitivity to build P automatically
 * from a SISO plant G and weighting functions W1 (tracking), W2 (control effort), W3 (robustness):
 * @code
 *   ||[ W1.S  ]||
 *   ||[ W2.KS ]||  < gamma
 *   ||[ W3.T  ]||inf
 * @endcode
 *
 * **Algorithm:** Discrete DGKF (Doyle, Glover, Khargonekar & Francis 1989; discrete version
 * by Stoorvogel 1992 / Iglesias & Glover 1991). A gamma-bisection drives the achieved norm
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
 * @see Stoorvogel, "The Discrete-Time Hinf Control Problem," SIAM 1992.
 * @see Iglesias & Glover, Int. J. Control 54(5), 1991.
 * @see Skogestad & Postlethwaite, "Multivariable Feedback Design" Ch. 9, 2005.
 * @see MATLAB hinfsyn() documentation.
 */

namespace ctrl
{

/**
 * @brief Generalised plant for Hinf synthesis.
 *
 * Dimensions: n = state, nw = exogenous inputs, nu = control inputs,
 * nz = performance outputs, ny = measurement outputs.
 */
struct GeneralisedPlant
{
    Eigen::MatrixXd A;   ///< State matrix (n * n).
    Eigen::MatrixXd B1;  ///< Exogenous input matrix (n * nw).
    Eigen::MatrixXd B2;  ///< Control input matrix (n * nu).
    Eigen::MatrixXd C1;  ///< Performance output matrix (nz * n).
    Eigen::MatrixXd C2;  ///< Measurement output matrix (ny * n).
    Eigen::MatrixXd D11; ///< Exogenous->performance feedthrough (nz * nw).
    Eigen::MatrixXd D12; ///< Control->performance feedthrough (nz * nu).
    Eigen::MatrixXd D21; ///< Exogenous->measurement feedthrough (ny * nw).
    Eigen::MatrixXd D22; ///< Control->measurement feedthrough (ny * nu).
    double Ts = 0.01;    ///< Sample time [s].

    int stateSize() const { return static_cast<int>(A.rows()); }  ///< State dimension n.
    int nw()        const { return static_cast<int>(B1.cols()); } ///< Exogenous input count.
    int nu()        const { return static_cast<int>(B2.cols()); } ///< Control input count.
    int nz()        const { return static_cast<int>(C1.rows()); } ///< Performance output count.
    int ny()        const { return static_cast<int>(C2.rows()); } ///< Measurement output count.
};

/**
 * @brief Hinf synthesis algorithm parameters.
 */
struct HinfParams
{
    double gammaInit   = 10.0;  ///< Initial upper bound for gamma bisection.
    double gammaTol    = 1e-3;  ///< Stop bisection when gamma bracket < this.
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
    double achievedGamma  = 0.0;   ///< Smallest feasible gamma found by bisection.
    double Ts             = 0.0;   ///< Plant sample time.

    /**
     * @name Controller matrices - K(z): xk[k+1] = Ak.xk + Bk.y,  u = Ck.xk + Dk.y
     * @{
     */
    Eigen::MatrixXd Ak; ///< Controller state transition (nk * nk, nk = plant state size).
    Eigen::MatrixXd Bk; ///< Controller input gain (nk * ny).
    Eigen::MatrixXd Ck; ///< Controller output gain (nu * nk).
    Eigen::MatrixXd Dk; ///< Controller feedthrough (nu * ny).
    /** @} */

    /**
     * @name DARE solutions (available for diagnostics)
     * @{
     */
    Eigen::MatrixXd X_inf; ///< Control Riccati solution.
    Eigen::MatrixXd Y_inf; ///< Filter Riccati solution.
    /** @} */

    /**
     * @name Synthesis diagnostics
     * @{
     */
    int    dareItersX    = 0;
    int    dareItersY    = 0;
    bool   dareConvX     = false;
    bool   dareConvY     = false;
    double spectralRadius = 0.0; ///< rho(Xinf.Yinf); must be < gamma^2 for feasibility.
    /** @} */
};

// -----------------------------------------------------------------------------

/**
 * @brief Parameters for mu-synthesis via DK-iteration.
 *
 * DK-iteration alternates between:
 * - **K-step:** H-infinity synthesis on the D-scaled plant.
 * - **D-step:** Amplitude-balance D-scaling that minimises an upper bound on
 *               the structured singular value mu over a frequency grid.
 *
 * When @p useRationalD is @c false (default), D-scaling is held constant
 * (frequency-independent) via Sinkhorn-Knopp amplitude balance.
 *
 * When @p useRationalD is @c true, a first-order rational filter D_j(z) is
 * fitted per diagonal element from the per-frequency amplitude-balance data.
 * The plant is then augmented with these filters before the next K-step.
 * This provides a tighter mu bound at the cost of a larger augmented plant.
 *
 * @code
 *   mu_upper <= sigma_max(D(jw) * F_l(P, K)(jw) * D(jw)^{-1})   for all w
 * @endcode
 *
 * @see Skogestad & Postlethwaite, "Multivariable Feedback Design" Ch. 8.12.
 * @see MATLAB dksyn() for the reference algorithm with vector-fitting D(jw).
 */
struct MuSynParams
{
    int    maxDKIter     = 20;    ///< Maximum DK-iteration rounds.
    double muTol         = 0.02;  ///< Stop when relative change in mu_upper < this.
    int    nFreqPoints   = 80;    ///< Frequency grid size for D-step (log-spaced, Nyquist).
    int    dScaleMaxIter = 30;    ///< Inner Sinkhorn-Knopp iterations per D-step.
    bool   useRationalD  = false; ///< When true, fit rational D_j(z) per channel.
    /**
     * @brief Order of the rational D-scaling filters (number of poles per channel).
     *
     * - 1 (default): first-order heuristic (fitFirstOrderDFilter) - fast, two data points.
     * - >= 2: full vector-fitting (VectorFitting::fitMagnitude) - accurate multi-peak profile.
     *
     * Only used when useRationalD is true.
     */
    int    dFitOrder     = 1;
    HinfParams hinfParams;        ///< H-inf sub-problem parameters for each K-step.
};

/**
 * @brief Result of mu-synthesis via solveMuSyn().
 */
struct MuSynResult
{
    bool   converged       = false; ///< @c true if relative mu improvement < MuSynParams::muTol.
    int    iterations      = 0;     ///< DK-iteration rounds completed.
    double achievedMuUpper = 0.0;   ///< Best mu upper bound achieved.
    HinfResult hinfResult;          ///< H-infinity result for the best K found.

    /**
     * @name D-scaling information (available when MuSynParams::useRationalD is true)
     * @{
     */
    /// Per-channel left D-scaling filters D_L_j(z), size = nz.
    /// Each entry is a first-order discrete-time StateSpace representing D_L_j(z).
    /// Empty when useRationalD = false (constant D was used).
    std::vector<StateSpace> dFilters_L;
    /// Per-channel right D-scaling filters D_R_j(z), size = nw.
    std::vector<StateSpace> dFilters_R;
    /** @} */

    /// Per-iteration mu upper bound history (index 0 = first iteration).
    std::vector<double> muHistory;
};

// -----------------------------------------------------------------------------

/**
 * @brief Caller-supplied controller structure for solveStructured().
 *
 * Maps a free parameter vector @p theta to a fixed-order/fixed-structure controller
 * (Ak, Bk, Ck, Dk). The controller order @c nk (= Ak.rows()) is determined entirely by what
 * this function produces - e.g. a static-gain controller (nk = 0, Ak/Bk/Ck all 0x*) or a
 * PID-like/lead-lag-shaped fixed-order controller. solveStructured() validates the produced
 * matrices' dimensions against the plant's ny/nu once before starting the search.
 */
using StructureFn = std::function<void(const Eigen::VectorXd &theta,
                                        Eigen::MatrixXd &Ak, Eigen::MatrixXd &Bk,
                                        Eigen::MatrixXd &Ck, Eigen::MatrixXd &Dk)>;

/**
 * @brief Parameters for structured Hinf synthesis via solveStructured().
 */
struct StructuredHinfParams
{
    Eigen::VectorXd lower; ///< Optional box lower bound on theta (empty = unbounded).
    Eigen::VectorXd upper; ///< Optional box upper bound on theta (empty = unbounded).
    int      maxEvals               = 2000; ///< Approximate CMA-ES cost-function evaluation budget.
    double   sigma0                 = 0.3;  ///< Initial CMA-ES step size.
    unsigned seed                   = 42;   ///< RNG seed forwarded to AutoTuner.
    /**
     * @brief Smooth instability penalty: @c instabilityPenaltyBase +
     *        @c instabilityPenaltySlope * max(0, maxPoleMagnitude - 1).
     *
     * A hard +infinity penalty for unstable candidates gives CMA-ES no gradient back toward
     * stability once the population drifts into the unstable region (unlike WorstCaseSearch's
     * use of +inf, which is a *maximisation* search where any unstable point is equally
     * "good"). This smooth penalty still strongly dominates any feasible candidate's cost
     * (calculateHInfinityNorm is rarely above 1e3 for realistic stable designs) while
     * remaining differentiable-in-effect for the search.
     * @see Apkarian & Noll, "Nonsmooth H-infinity Synthesis", IEEE TAC 51(1), 2006.
     */
    double   instabilityPenaltyBase  = 1e6;
    double   instabilityPenaltySlope = 1e3; ///< @see instabilityPenaltyBase.
};

/**
 * @brief Result of structured Hinf synthesis via solveStructured().
 */
struct StructuredHinfResult
{
    /**
     * @brief Controller matrices/gamma at the best theta found.
     *
     * Only Ak/Bk/Ck/Dk/Ts/feasible/achievedGamma are populated; X_inf/Y_inf/dareConvX/
     * dareConvY/dareItersX/dareItersY/spectralRadius are left at their default values since
     * solveStructured() does not solve a Riccati equation (no DARE diagnostics exist).
     */
    HinfResult       hinfResult;
    Eigen::VectorXd  theta;          ///< Best parameter vector found.
    bool             converged = false; ///< CMA-ES internal convergence flag.
    int              nEvals    = 0;     ///< Total cost-function evaluations used.
};

/**
 * @brief Discrete-time dynamic output-feedback Hinf controller.
 *
 * Implements the synthesised controller K(z):
 * @code
 *   xk[k+1] = Ak.xk[k] + Bk.y[k]
 *   u[k]    = Ck.xk[k] + Dk.y[k]
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
     * @brief Solve the discrete-time Hinf optimal control problem.
     *
     * Bisects gamma from [gammaLow, params.gammaInit] until the bracket is narrower than
     * params.gammaTol. gammaLow is set automatically.
     *
     * @param P      Generalised plant (use MixedSensitivity::build() for S/KS/T design).
     * @param params Synthesis algorithm parameters.
     * @return HinfResult containing controller matrices and diagnostics.
     *         Check result.feasible before constructing a DiscreteHinf.
     */
    static HinfResult solve(const GeneralisedPlant &P, const HinfParams &params = {});

    /**
     * @brief Mu-synthesis via DK-iteration (constant D-scaling).
     *
     * Iterates between an H-infinity K-step (solving the scaled plant) and a
     * D-step (amplitude-balance Sinkhorn-Knopp scaling to tighten the mu upper
     * bound). Returns the best controller found across all iterations.
     *
     * **Algorithm:**
     * 1. Initialise D = I.
     * 2. **K-step:** solve H-inf for D-scaled plant P_d.
     * 3. Build closed-loop M = F_l(P, K).
     * 4. **D-step:** evaluate M(jw) at @p nFreqPoints log-spaced frequencies;
     *    apply Sinkhorn-Knopp balancing to minimise sigma_max(D*M*D^{-1}).
     * 5. Compute mu_upper = max_w sigma_max(D*M(jw)*D^{-1}).
     * 6. Check convergence; if |delta mu| / mu < muTol or K-step infeasible: stop.
     *
     * **Typical usage:**
     * @code
     *   ctrl::MuSynParams mp;
     *   mp.hinfParams.gammaInit = 20.0;
     *   auto r = ctrl::DiscreteHinf::solveMuSyn(P, mp);
     *   if (r.hinfResult.feasible) {
     *       ctrl::DiscreteHinf K(r.hinfResult);
     *       std::cout << "mu_upper = " << r.achievedMuUpper << "\n";
     *   }
     * @endcode
     *
     * @param P      Generalised plant (same format as for solve()).
     * @param params Mu-synthesis parameters.
     * @return MuSynResult with the best K and its mu upper bound.
     */
    static MuSynResult solveMuSyn(const GeneralisedPlant &P,
                                   const MuSynParams &params = {});

    /**
     * @brief Structured (fixed-order/fixed-structure) Hinf synthesis via direct search.
     *
     * A third synthesis mode alongside solve() (DGKF two-Riccati) and solveMuSyn()
     * (DK-iteration): rather than solving Riccati equations, parameterises the controller
     * via a caller-supplied @ref StructureFn and searches over the free parameter vector
     * theta with CMA-ES (AutoTuner) to minimise the closed-loop Hinf norm. This is the
     * non-commercial analogue of MATLAB's hinfstruct() - useful when a fixed controller
     * order/structure (e.g. PID, lead-lag, or a reduced-order observer) is required and the
     * full-order DGKF central controller from solve() is not acceptable.
     *
     * Unstable candidates are penalised smoothly (see StructuredHinfParams::
     * instabilityPenaltyBase), not with a hard +infinity wall - see that struct's docs.
     *
     * @param P           Generalised plant (same format as for solve()).
     * @param structureFn Caller-defined theta -> (Ak,Bk,Ck,Dk) controller parameterisation.
     * @param theta0      Initial parameter vector (also used to validate structureFn's
     *                    output dimensions against P.ny()/P.nu() before the search starts).
     * @param params      Search parameters.
     * @return StructuredHinfResult with the best controller found and its achieved gamma.
     * @throws std::invalid_argument If theta0 is empty, params.lower/upper have the wrong
     *         size, or structureFn(theta0, ...) produces matrices with the wrong dimensions.
     */
    static StructuredHinfResult solveStructured(const GeneralisedPlant &P,
                                                  StructureFn structureFn,
                                                  const Eigen::VectorXd &theta0,
                                                  const StructuredHinfParams &params = {});

    /**
     * @brief Compute u[k] - SISO interface (ny = 1, nu = 1).
     *
     * @p signal is the plant measurement y[k] **not** the tracking error.
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

    /** @brief Current controller internal state xk (nk * 1). */
    const Eigen::VectorXd &controllerState() const { return xk_; }

    /** @brief Achieved Hinf bound gamma from synthesis. */
    double achievedGamma() const { return gamma_; }

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
    double gamma_;
    double u_prev_ = 0.0; ///< Last finite compute() scalar (hold-last NaN contract).

    // DareResult is shared with DiscreteLQR (defined in PlantModel.h).
    static DareResult solveHinfDARE(const Eigen::MatrixXd &A,
                                    const Eigen::MatrixXd &B,
                                    const Eigen::MatrixXd &Q,
                                    const Eigen::MatrixXd &R,
                                    double tol, int maxIter);

    static bool trySolve(const GeneralisedPlant &P, double gamma,
                         double dareTol, int dareMaxIter,
                         HinfResult &out);

    // --- mu-synthesis helpers ------------------------------------------------

    // Build closed-loop state-space F_l(P, K): exogenous w -> performance z.
    static void buildClosedLoop(const GeneralisedPlant    &P,
                                const Eigen::MatrixXd     &Ak,
                                const Eigen::MatrixXd     &Bk,
                                const Eigen::MatrixXd     &Ck,
                                const Eigen::MatrixXd     &Dk,
                                Eigen::MatrixXd           &A_cl,
                                Eigen::MatrixXd           &B_cl,
                                Eigen::MatrixXd           &C_cl,
                                Eigen::MatrixXd           &D_cl);

    // Evaluate M(e^{j*w*Ts}) for the state-space (A,B,C,D).
    static Eigen::MatrixXcd evalFreqResponse(const Eigen::MatrixXd &A,
                                             const Eigen::MatrixXd &B,
                                             const Eigen::MatrixXd &C,
                                             const Eigen::MatrixXd &D,
                                             double w, double Ts);

    // Sinkhorn-Knopp amplitude balance: find d_L, d_R that equalise row/col
    // norms of diag(d_L)*Mabs*diag(1/d_R).
    static void amplitudeBalance(const Eigen::MatrixXd &Mabs,
                                 int max_iter,
                                 Eigen::VectorXd &d_L,
                                 Eigen::VectorXd &d_R);

    // Apply constant D-scaling to plant performance/exogenous channels only.
    static GeneralisedPlant applyDScaling(const GeneralisedPlant &P,
                                          const Eigen::VectorXd  &d_L,
                                          const Eigen::VectorXd  &d_R);

    // Fit a first-order rational D_j(z) = K*(z-z0)/(z-p) to frequency magnitude data.
    // d_lo and d_hi are the DC (w~0) and Nyquist (w~pi/Ts) magnitudes respectively.
    // Returns a StateSpace with A=p, B=1, C=K*(p-z0), D=K.
    static StateSpace fitFirstOrderDFilter(double d_lo, double d_hi, double Ts);

    // Augment the generalised plant with rational D_L and D_R filters.
    // New state: x_aug = [x; xDL; xDR],  n_aug = n + nz + nw.
    // Drives the K-step H-inf problem on the augmented plant.
    static GeneralisedPlant applyRationalDScaling(
        const GeneralisedPlant          &P,
        const std::vector<StateSpace>   &d_filters_L,
        const std::vector<StateSpace>   &d_filters_R);
};

// -----------------------------------------------------------------------------

/**
 * @brief Helper class that assembles a GeneralisedPlant for S/KS/T mixed-sensitivity design.
 *
 * Augments a SISO discrete-time plant G(z) with three weighting state-space models:
 * - **W1** - tracking / disturbance rejection (high gain at low frequencies).
 * - **W2** - control effort / actuator limits (limits high-frequency commands).
 * - **W3** - robustness / unmodelled dynamics (forces T rolloff at high frequencies).
 *
 * @par Infeasibility troubleshooting
 * If solve() returns feasible = @c false:
 * 1. Increase gammaInit (start at 10.||W1||inf.||W3||inf as a rule of thumb).
 * 2. Verify W1 crossover is below the plant open-loop bandwidth.
 * 3. For non-minimum-phase plants, the achievable sensitivity peak is bounded below by
 *    the Poisson integral; reduce W1 gain or relax the crossover requirement
 *    (Skogestad & Postlethwaite Section 6.3).
 * 4. If W1 and W3 both have high gain in overlapping bands, the design may be
 *    fundamentally infeasible (S + T = 1). Reduce |W1(jomega)|.|W3(jomega)|.
 * 5. If gammaInit is below the achievable gamma, bisection cannot start - increase it.
 */
class MixedSensitivity
{
public:
    /**
     * @brief Sensitivity weight W1 for tracking and disturbance rejection.
     *
     * Continuous: W1(s) = (s/M + omegaB) / (s + omegaB.epsilon).
     * Low-freq gain approx = 1/epsilon, crossover at omegaB, high-freq gain approx = M.
     *
     * @param omega_B Crossover frequency [rad/s].
     * @param M       High-frequency gain bound.
     * @param eps     Low-frequency sensitivity tolerance (e.g., 0.001 for 1% steady-state error).
     * @param Ts      Sample time [s] (Tustin discretisation).
     */
    static StateSpace makeW1(double omega_B, double M, double eps, double Ts);

    /**
     * @brief Control effort weight W2 - flat gain at all frequencies.
     *
     * Simple actuator-level constraint: set gain = 1/u_max.
     *
     * @param gain Scalar gain value.
     * @param Ts   Sample time [s].
     */
    static StateSpace makeW2constant(double gain, double Ts);

    /**
     * @brief Control effort weight W2 - first-order high-pass.
     *
     * Continuous: W2(s) = (s + omegau.epsilon) / (s/1 + omegau).
     * Low-freq gain approx = epsilon, high-freq gain approx = 1, rolloff at omegau.
     *
     * @param omega_u Rolloff frequency [rad/s].
     * @param eps     Low-frequency gain floor.
     * @param Ts      Sample time [s].
     */
    static StateSpace makeW2highpass(double omega_u, double eps, double Ts);

    /**
     * @brief Robustness weight W3 for complementary sensitivity rolloff.
     *
     * Continuous: W3(s) = (s + omegaT/Mt) / (epsilon.s + omegaT).
     * Low-freq gain approx = epsilon, high-freq gain approx = 1/Mt, rollup above omegaT.
     *
     * @param omega_T Rollup frequency [rad/s].
     * @param Mt      Maximum complementary sensitivity (e.g., 1.2-2.0).
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
     * Performance outputs: z = [W1.(r-y); W2.u; W3.y], nz = 3.
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
