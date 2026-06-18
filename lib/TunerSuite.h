#pragma once
#include "IController.h"
#include "PlantModel.h"
#include "DiscretePID.h"
#include "DiscreteLQR.h"
#include "DiscreteMPC.h"
#include "DiscreteLeadLag.h"
#include "ControllerTuner.h"
#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <iosfwd>

/**
 * @file TunerSuite.h
 * @brief Unified tuner library with runtime soft-warning dispatch across all seven tuning families.
 *
 * Covers all tuning families from the tuning cheatsheet:
 * 1. **Relay / Ziegler-Nichols** -> dedicated: PID.
 * 2. **IMC-PID** -> dedicated: PID.
 * 3. **Cohen-Coon** -> dedicated: PID (high dead-time).
 * 4. **LQR Bryson's Rule** -> dedicated: LQR, LQG.
 * 5. **LQG Kalman noise** -> dedicated: LQG.
 * 6. **MPC horizon/weight** -> dedicated: MPC.
 * 7. **Frequency-domain shaping** -> dedicated: LeadLag.
 * 8. **Optimisation-based (Nelder-Mead ISE)** -> any controller (generic).
 *
 * **Compatibility tiers** (applied at runtime, not compile time):
 * - `IDEAL`    - tuner was designed for this controller; no warning emitted.
 * - `SOFT`     - tuner can produce useful output but the match is imperfect;
 *                a diagnostic is written to `std::clog` and `result.warned == true`.
 * - `FALLBACK` - tuner is not meaningful for this controller; default/zero parameters
 *                are returned, `result.success == false`, strong warning emitted.
 *
 * @note The hard compile-time errors (ControllerTraits `static_assert`) in
 *       ControllerTuner.h remain unchanged; they fire when calling typed template
 *       wrappers such as `tuneFor<C>()`. TunerSuite takes a different path: it
 *       accepts a runtime CtrlKind tag and never blocks compilation.
 *
 * @see ControllerTuner.h for compile-time-safe typed wrappers.
 * @see Astrom & Hagglund, "Advanced PID Control" (2006).
 * @see Bryson & Ho, "Applied Optimal Control" (1975).
 * @see Gao, "Active Disturbance Rejection Control," ACC (2003).
 */

namespace ctrl
{

// -----------------------------------------------------------------------------
// Runtime controller-type tag
// -----------------------------------------------------------------------------

/**
 * @brief Runtime controller-type tag for TunerSuite dispatch.
 */
enum class CtrlKind
{
    PID,     ///< DiscretePID.
    LQR,     ///< DiscreteLQR.
    LQG,     ///< DiscreteLQG.
    MPC,     ///< DiscreteMPC.
    LeadLag, ///< DiscreteLeadLag.
    SMC,     ///< DiscreteSMC.
    ADRC,    ///< DiscreteADRC.
    ESC,     ///< ExtremumSeeker.
    Smith,   ///< SmithPredictor (tune the inner controller, not the wrapper).
    Generic, ///< Any IController with unknown or custom type.
};

/**
 * @brief Human-readable name for a CtrlKind tag - for diagnostic messages.
 * @param k Controller kind.
 * @return Null-terminated string (e.g., "DiscretePID").
 */
const char *ctrlKindName(CtrlKind k) noexcept;

// -----------------------------------------------------------------------------
// Typed result structs
// -----------------------------------------------------------------------------

/**
 * @brief Base fields shared by all TunerSuite result types.
 */
struct TuningResultBase
{
    bool        success = false;   ///< `false` -> parameters are default or invalid.
    bool        warned  = false;   ///< `true` -> a soft warning was emitted to std::clog.
    std::string warning;           ///< Diagnostic text (also sent to std::clog when non-empty).

    double predictedCrossoverFreq = -1.0; ///< Predicted gain crossover omegac [rad/s]; -1 if unknown.
    double predictedDampingRatio  = -1.0; ///< Predicted damping ratio zeta; -1 if unknown.
};

/** @brief Result of a relay/ZN, IMC, or Cohen-Coon tuning call. */
struct PIDTuneResult : TuningResultBase
{
    PIDParams params; ///< Tuned PID parameters.
};

/** @brief Result of a Bryson LQR tuning call. */
struct LQRTuneResult : TuningResultBase
{
    LQRParams params; ///< Tuned LQR weight matrices.
};

/** @brief Result of a Kalman noise tuning call. */
struct KalmanTuneResult : TuningResultBase
{
    KalmanNoiseParams params; ///< Tuned Kalman noise covariances.
};

/** @brief Result of an MPC horizon tuning call. */
struct MPCTuneResult : TuningResultBase
{
    MPCHorizonTuner::Recommendation params{}; ///< Recommended Np, Nc, rhoy, rhou.
};

/** @brief Result of a loop-shaping lead/lag tuning call. */
struct LeadLagTuneResult : TuningResultBase
{
    LeadLagParams params; ///< Tuned lead/lag compensator parameters.
};

/**
 * @brief Result of an optimisation-based tuning call (Nelder-Mead ISE/ITAE).
 */
struct OptimTuneResult : TuningResultBase
{
    std::vector<double> bestParams; ///< Optimised parameter vector.
    double              bestCost  = 1e30; ///< Minimum cost (ISE / IAE / ITAE) achieved.
    int                 evalCount = 0;    ///< Number of cost-function evaluations used.
};

// -----------------------------------------------------------------------------
// TunerSuite
// -----------------------------------------------------------------------------

/**
 * @brief Unified collection of all eight tuning strategies with runtime compatibility checking.
 *
 * Each static method:
 * 1. Checks the supplied CtrlKind against the tuner's compatibility tier.
 * 2. Runs the underlying tuner if IDEAL or SOFT.
 * 3. Returns default parameters and sets `success = false` for FALLBACK targets.
 * 4. Writes a diagnostic to `std::clog` and sets `warned = true` for SOFT targets.
 */
class TunerSuite
{
public:
    // -------------------------------------------------------------------------
    // 1 & 3. Relay Ziegler-Nichols
    // -------------------------------------------------------------------------

    /**
     * @brief Relay / ZN tuning (cheatsheet Section 1).
     *
     * Compatibility:
     * - IDEAL: PID.
     * - SOFT: Smith (tune inner PID).
     * - FALLBACK: LQR, LQG, MPC, LeadLag, SMC, ADRC, ESC.
     *
     * @param target  Runtime controller kind.
     * @param relay   A completed RelayAutoTuner (`isDone() == true`).
     * @param rule    ZN | TyreusLuyben | AMIGO | IMC.
     * @param lambda  IMC closed-loop time constant (-1 = auto).
     * @return PIDTuneResult with default params when success == false.
     */
    static PIDTuneResult relayZN(CtrlKind             target,
                                 const RelayAutoTuner &relay,
                                 PIDTuningRule         rule   = PIDTuningRule::TyreusLuyben,
                                 double                lambda = -1.0);

    // -------------------------------------------------------------------------
    // 2. IMC-PID
    // -------------------------------------------------------------------------

    /**
     * @brief IMC-PID tuning from a FOPDT model (cheatsheet Section 2).
     *
     * Compatibility:
     * - IDEAL: PID.
     * - SOFT: Smith (inner), LeadLag (Kp as bandwidth hint).
     * - FALLBACK: LQR, LQG, MPC, SMC, ADRC, ESC.
     *
     * @param target  Runtime controller kind.
     * @param fopdt   FOPDT model from StepResponseTuner::identify().
     * @param Ts      Sample time [s].
     * @param lambda  Closed-loop BW (-1 = 0.5.tau).
     * @return PIDTuneResult.
     */
    static PIDTuneResult imcPID(CtrlKind                           target,
                                const StepResponseTuner::FOPDTModel &fopdt,
                                double                              Ts,
                                double                              lambda = -1.0);

    // -------------------------------------------------------------------------
    // 3. Cohen-Coon
    // -------------------------------------------------------------------------

    /**
     * @brief Cohen-Coon PID tuning (cheatsheet Section 2 variant).
     *
     * Especially accurate when theta/tau \in [0.1, 1.0].
     *
     * Compatibility:
     * - IDEAL: PID.
     * - SOFT: Smith (inner).
     * - FALLBACK: all others.
     *
     * @note Emits a soft warning if `fopdt.theta` is near zero.
     *
     * @param target  Runtime controller kind.
     * @param fopdt   FOPDT model (must have theta > 0).
     * @param Ts      Sample time [s].
     * @return PIDTuneResult.
     */
    static PIDTuneResult cohenCoon(CtrlKind                           target,
                                   const StepResponseTuner::FOPDTModel &fopdt,
                                   double                              Ts);

    // -------------------------------------------------------------------------
    // 4. LQR Bryson's Rule
    // -------------------------------------------------------------------------

    /**
     * @brief LQR Bryson's rule (cheatsheet Section 3).
     *
     * Compatibility:
     * - IDEAL: LQR, LQG.
     * - SOFT: MPC (different Q/R convention), SMC (c_e ~ \sqrt(Q ratio) hint).
     * - FALLBACK: PID, LeadLag, ESC, Smith.
     *
     * @param target  Runtime controller kind.
     * @param xmax    Maximum acceptable state deviation per channel (n*1).
     * @param umax    Maximum acceptable control effort per channel (m*1).
     * @return LQRTuneResult.
     */
    static LQRTuneResult bryson(CtrlKind              target,
                                const Eigen::VectorXd &xmax,
                                const Eigen::VectorXd &umax);

    // -------------------------------------------------------------------------
    // 5. Kalman noise weights
    // -------------------------------------------------------------------------

    /**
     * @brief Kalman observer noise weights (cheatsheet Section 4, observer part).
     *
     * Uses an isotropic model: Qf = sigmap^2.I, Rf = sigmam^2.I, P0 = Qf.
     *
     * Compatibility:
     * - IDEAL: LQG.
     * - SOFT: Generic (standalone KalmanFilter).
     * - FALLBACK: LQR, PID, MPC, LeadLag, SMC, ADRC, ESC.
     *
     * @param target        Runtime controller kind.
     * @param nStates       Number of states n.
     * @param nOutputs      Number of outputs p.
     * @param sigmaProcess  Process noise standard deviation.
     * @param sigmaMeas     Measurement noise standard deviation.
     * @return KalmanTuneResult.
     */
    static KalmanTuneResult kalmanNoise(CtrlKind target,
                                        int      nStates,
                                        int      nOutputs,
                                        double   sigmaProcess,
                                        double   sigmaMeas);

    // -------------------------------------------------------------------------
    // 6. MPC Horizon / Weight Tuner
    // -------------------------------------------------------------------------

    /**
     * @brief MPC horizon and weight recommendation (cheatsheet Section 5).
     *
     * Compatibility:
     * - IDEAL: MPC.
     * - SOFT: LQR (settling-time estimate is useful as Np guidance).
     * - FALLBACK: all others.
     *
     * @param target  Runtime controller kind.
     * @param plant   Discrete-time state-space model.
     * @param Ts      Sample time [s].
     * @param rho_y   Output tracking weight (passed through).
     * @param rho_u   Move suppression weight (passed through).
     * @return MPCTuneResult with recommended Np, Nc, rhoy, rhou.
     */
    static MPCTuneResult mpcHorizon(CtrlKind         target,
                                    const StateSpace &plant,
                                    double           Ts,
                                    double           rho_y = 1.0,
                                    double           rho_u = 0.1);

    // -------------------------------------------------------------------------
    // 7. Frequency-domain Loop Shaping
    // -------------------------------------------------------------------------

    /**
     * @brief Frequency-domain loop shaping (cheatsheet Section 6).
     *
     * Compatibility:
     * - IDEAL: LeadLag.
     * - SOFT: PID (omegac used as a bandwidth / N-filter hint).
     * - FALLBACK: LQR, LQG, MPC, SMC, ADRC, ESC.
     *
     * @note @p in.phase_add_deg must be in (0, 90). LoopShapingTuner handles fallback
     *       internally for invalid angles.
     *
     * @param target  Runtime controller kind.
     * @param in      Loop-shaping inputs (omegac, phase lead, plant gain at omegac).
     * @return LeadLagTuneResult.
     */
    static LeadLagTuneResult loopShaping(CtrlKind                    target,
                                         const LoopShapingTuner::Input &in);

    // -------------------------------------------------------------------------
    // 8. Optimisation-based tuner - Nelder-Mead ISE / ITAE
    // -------------------------------------------------------------------------

    /**
     * @brief Black-box Nelder-Mead optimisation tuner (cheatsheet Section 7).
     *
     * Applies to **all** controller types. No warnings are emitted regardless of target
     * because this method treats the controller as a black box.
     *
     * The cost function must:
     * 1. Construct a controller from the parameter vector.
     * 2. Simulate it against the plant (or real hardware).
     * 3. Return a scalar cost (ISE, IAE, ITAE, ...).
     *
     * For multi-modal cost surfaces, run multiple restarts with different `x0`.
     *
     * @param target       Runtime controller kind (used for logging only).
     * @param paramBounds  [{lo, hi}, ...] for each scalar parameter.
     * @param costFn       Cost function `double(vector<double>)`.
     * @param x0           Initial guess; empty -> midpoint of each bound.
     * @param maxEvals     Cost-function evaluation budget.
     * @param tol          Convergence tolerance on simplex size.
     * @return OptimTuneResult with bestParams and bestCost.
     */
    static OptimTuneResult optimise(
        CtrlKind                                                      target,
        const std::vector<std::pair<double, double>>                 &paramBounds,
        std::function<double(const std::vector<double> &)>            costFn,
        std::vector<double>                                           x0       = {},
        int                                                           maxEvals = 300,
        double                                                        tol      = 1e-5);

    // -------------------------------------------------------------------------
    // Cost-function helpers
    // -------------------------------------------------------------------------

    /**
     * @brief Build an ISE cost function for a closed-loop simulation.
     *
     * @code
     *   auto costFn = TunerSuite::makeISECost(plant, ref, N,
     *       [](const std::vector<double>& p) -> std::unique_ptr<IController> {
     *           PIDParams pp; pp.Kp=p[0]; pp.Ki=p[1]; pp.Kd=p[2];
     *           return std::make_unique<DiscretePID>(pp, Ts);
     *       });
     *   auto result = TunerSuite::optimise(CtrlKind::PID, bounds, costFn);
     * @endcode
     *
     * @param plant    Discrete-time state-space model.
     * @param ref      Step reference value.
     * @param N        Simulation horizon [steps].
     * @param factory  Factory `unique_ptr<IController>(vector<double>)` -> controller.
     * @return Callable `double(vector<double>)` suitable for optimise().
     */
    static std::function<double(const std::vector<double> &)>
    makeISECost(const StateSpace &plant,
                double            ref,
                int               N,
                std::function<std::unique_ptr<IController>(const std::vector<double> &)> factory);

    /**
     * @brief Build an ITAE cost function - penalises late errors more than ISE.
     *
     * ITAE = \int t.|e(t)| dt. Otherwise identical interface to makeISECost().
     *
     * @param plant    Discrete-time state-space model.
     * @param ref      Step reference value.
     * @param N        Simulation horizon [steps].
     * @param factory  Factory `unique_ptr<IController>(vector<double>)` -> controller.
     * @return Callable `double(vector<double>)` suitable for optimise().
     */
    static std::function<double(const std::vector<double> &)>
    makeITAECost(const StateSpace &plant,
                 double            ref,
                 int               N,
                 std::function<std::unique_ptr<IController>(const std::vector<double> &)> factory);

private:
    static void softWarn(TuningResultBase &base, const std::string &msg);

    static std::vector<double> nelderMead(
        std::function<double(const std::vector<double> &)>  f,
        std::vector<double>                                  x0,
        const std::vector<std::pair<double, double>>        &bounds,
        int                                                  maxEvals,
        double                                               tol,
        int                                                 *evalCount_out);
};

} // namespace ctrl
