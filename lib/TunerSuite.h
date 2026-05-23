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

// ============================================================
//  TunerSuite.h  -  Unified tuner library with soft-warning dispatch
//
//  Covers all seven tuning families from the tuning_methods cheatsheet:
//    1. Relay / Ziegler-Nichols   -> dedicated: PID
//    2. IMC-PID                   -> dedicated: PID
//    3. Cohen-Coon                -> dedicated: PID  (high dead-time)
//    4. LQR Bryson's Rule         -> dedicated: LQR, LQG
//    5. LQG Kalman noise          -> dedicated: LQG
//    6. MPC horizon/weight        -> dedicated: MPC
//    7. Frequency-domain shaping  -> dedicated: LeadLag
//    8. Optimisation-based (Nelder-Mead ISE) -> any controller (generic)
//
//  Compatibility tiers (applied at RUNTIME, not compile time):
//    IDEAL    - tuner was designed for this controller; no warning emitted.
//    SOFT     - tuner can produce useful output but the match is imperfect;
//               a diagnostic is written to std::clog and result.warned == true.
//    FALLBACK - tuner is not meaningful for this controller; default/zero
//               parameters are returned, result.success == false, strong
//               warning emitted.  The caller still gets a valid (empty) object.
//
//  The HARD compile-time errors (ControllerTraits::static_assert) in
//  ControllerTuner.h remain unchanged; they fire when you call the typed
//  template wrappers (tuneFor<C>()).  TunerSuite takes a different path:
//  it accepts a runtime CtrlKind tag and NEVER blocks compilation.
//
//  Ref: cheatsheet/tuning_methods.md;
//       Åström & Hägglund (2006); Bryson & Ho (1975); Gao (2003).
// ============================================================
namespace ctrl
{

    // ============================================================
    //  Runtime controller-type tag
    // ============================================================
    enum class CtrlKind
    {
        PID,     // DiscretePID
        LQR,     // DiscreteLQR
        LQG,     // DiscreteLQG
        MPC,     // DiscreteMPC
        LeadLag, // DiscreteLeadLag
        SMC,     // DiscreteSMC
        ADRC,    // DiscreteADRC
        ESC,     // ExtremumSeeker
        Smith,   // SmithPredictor (tune the inner controller, not the wrapper)
        Generic, // Any IController with unknown / custom type
    };

    // Human-readable name for diagnostics.
    const char *ctrlKindName(CtrlKind k) noexcept;

    // ============================================================
    //  Typed result structs
    //  Every TunerSuite method returns one of these, carrying:
    //    success - whether a meaningful parameter set was produced
    //    warned  - whether a soft warning was emitted
    //    warning - the warning text (empty when warned == false)
    // ============================================================
    struct TuningResultBase
    {
        bool success = false; // false -> parameters are default/invalid
        bool warned = false;  // true  -> soft warning was emitted
        std::string warning;  // diagnostic text (also sent to std::clog)
    };

    struct PIDTuneResult : TuningResultBase
    {
        PIDParams params;
    };

    struct LQRTuneResult : TuningResultBase
    {
        LQRParams params;
    };

    struct KalmanTuneResult : TuningResultBase
    {
        KalmanNoiseParams params;
    };

    struct MPCTuneResult : TuningResultBase
    {
        MPCHorizonTuner::Recommendation params{};
    };

    struct LeadLagTuneResult : TuningResultBase
    {
        LeadLagParams params;
    };

    // ============================================================
    //  Optimisation-based tuner result
    //  The Nelder-Mead solver minimises a caller-supplied cost
    //  function over a bounded parameter space.
    // ============================================================
    struct OptimTuneResult : TuningResultBase
    {
        std::vector<double> bestParams; // optimised parameter vector
        double bestCost = 1e30;         // minimum ISE / cost achieved
        int evalCount = 0;              // cost-function evaluations used
    };

    // ============================================================
    //  TunerSuite
    // ============================================================
    class TunerSuite
    {
    public:
        // ----------------------------------------------------------
        //  1 & 3.  Relay Ziegler-Nichols  (cheatsheet Section 1)
        // ----------------------------------------------------------
        //  DEDICATED: PID
        //  SOFT WARN: Smith (tune the inner PID separately)
        //  FALLBACK:  LQR, LQG, MPC, LeadLag, SMC, ADRC, ESC - warns,
        //             returns default PIDParams so the caller can inspect the
        //             raw Ku/Tu data if they wish.
        //
        //  relay  - a completed RelayAutoTuner (isDone() == true)
        //  rule   - ZN | TyreusLuyben | AMIGO | IMC
        //  lambda - IMC closed-loop time constant (-1 -> auto)
        static PIDTuneResult relayZN(CtrlKind target,
                                     const RelayAutoTuner &relay,
                                     PIDTuningRule rule = PIDTuningRule::TyreusLuyben,
                                     double lambda = -1.0);

        // ----------------------------------------------------------
        //  2.  IMC-PID  (cheatsheet Section 2)
        // ----------------------------------------------------------
        //  DEDICATED: PID
        //  SOFT WARN: Smith (inner), LeadLag (use Kp as gain hint only)
        //  FALLBACK:  LQR, LQG, MPC, SMC, ADRC, ESC
        //
        //  fopdt - identified via StepResponseTuner::identify()
        //  lambda - closed-loop BW (-1 -> 0.5 . tau)
        static PIDTuneResult imcPID(CtrlKind target,
                                    const StepResponseTuner::FOPDTModel &fopdt,
                                    double Ts,
                                    double lambda = -1.0);

        // ----------------------------------------------------------
        //  3.  Cohen-Coon  (cheatsheet Section 2 variant)
        // ----------------------------------------------------------
        //  DEDICATED: PID  (especially when theta/tau \in [0.1, 1.0])
        //  SOFT WARN: Smith (inner)
        //  FALLBACK:  all others
        //
        //  Requires fopdt.theta > 0; emits warning if theta is near zero.
        static PIDTuneResult cohenCoon(CtrlKind target,
                                       const StepResponseTuner::FOPDTModel &fopdt,
                                       double Ts);

        // ----------------------------------------------------------
        //  4.  LQR Bryson's Rule  (cheatsheet Section 3)
        // ----------------------------------------------------------
        //  DEDICATED: LQR, LQG
        //  SOFT WARN: MPC (different Q/R convention - Np*p * Np*p vs n*n),
        //             SMC (c_e ~ sqrt(Q_ratio) as a design hint)
        //  FALLBACK:  PID, LeadLag, ESC, Smith
        //
        //  xmax - maximum acceptable state deviation per channel  (n*1)
        //  umax - maximum acceptable control effort per channel   (m*1)
        static LQRTuneResult bryson(CtrlKind target,
                                    const Eigen::VectorXd &xmax,
                                    const Eigen::VectorXd &umax);

        // ----------------------------------------------------------
        //  5.  Kalman noise weights  (cheatsheet Section 4 - LQG observer part)
        // ----------------------------------------------------------
        //  DEDICATED: LQG
        //  SOFT WARN: Generic (standalone KalmanFilter)
        //  FALLBACK:  LQR (no observer), PID, MPC, LeadLag, SMC, ADRC, ESC
        //
        //  isotropic model: Qf = sigmap^2.I,  Rf = sigmam^2.I,  P0 = Qf
        static KalmanTuneResult kalmanNoise(CtrlKind target,
                                            int nStates,
                                            int nOutputs,
                                            double sigmaProcess,
                                            double sigmaMeas);

        // ----------------------------------------------------------
        //  6.  MPC Horizon / Weight Tuner  (cheatsheet Section 5)
        // ----------------------------------------------------------
        //  DEDICATED: MPC
        //  SOFT WARN: LQR (settling-time estimate is still useful as Np guidance)
        //  FALLBACK:  all others
        //
        //  Returns MPCHorizonTuner::Recommendation.  The recommended Np, Nc,
        //  rho_y, rho_u can be used directly to construct MPCParams.
        static MPCTuneResult mpcHorizon(CtrlKind target,
                                        const StateSpace &plant,
                                        double Ts,
                                        double rho_y = 1.0,
                                        double rho_u = 0.1);

        // ----------------------------------------------------------
        //  7.  Frequency-domain Loop Shaping  (cheatsheet Section 6)
        // ----------------------------------------------------------
        //  DEDICATED: LeadLag
        //  SOFT WARN: PID (omega_c used as a bandwidth / N-filter hint)
        //  FALLBACK:  LQR, LQG, MPC, SMC, ADRC, ESC
        //
        //  phase_add_deg must be in (0, 90); LoopShapingTuner handles the
        //  fallback internally for invalid angles.
        static LeadLagTuneResult loopShaping(CtrlKind target,
                                             const LoopShapingTuner::Input &in);

        // ----------------------------------------------------------
        //  8.  Optimisation-based tuner - Nelder-Mead ISE  (cheatsheet Section 7)
        // ----------------------------------------------------------
        //  Applies to: ALL controller types  (Generic, PID, LQR, ...)
        //  No warnings emitted regardless of target - this method is truly
        //  universal because it treats the controller as a black box.
        //
        //  costFn(params) must:
        //    1. Construct a controller from params.
        //    2. Simulate it against the plant (or real hardware).
        //    3. Return a scalar cost (ISE, IAE, ITAE, ...).
        //
        //  paramBounds - [{lo, hi}, ...] for each scalar parameter
        //  x0          - initial guess; empty -> midpoint of each bound
        //  maxEvals    - cost-function evaluation budget
        //  tol         - convergence tolerance on simplex size
        //
        //  A lightweight Nelder-Mead simplex is used. For multi-modal surfaces
        //  use multiple restarts (call optimise() several times with different x0).
        static OptimTuneResult optimise(
            CtrlKind target,
            const std::vector<std::pair<double, double>> &paramBounds,
            std::function<double(const std::vector<double> &)> costFn,
            std::vector<double> x0 = {},
            int maxEvals = 300,
            double tol = 1e-5);

        // ----------------------------------------------------------
        //  Helpers
        // ----------------------------------------------------------

        // Build ISE cost function for a closed-loop simulation.
        // Usage:
        //   auto costFn = TunerSuite::makeISECost(plant, ref, N,
        //       [](const std::vector<double>& p) -> std::unique_ptr<IController> {
        //           PIDParams pp; pp.Kp=p[0]; pp.Ki=p[1]; pp.Kd=p[2];
        //           return std::make_unique<DiscretePID>(pp, Ts);
        //       });
        //   auto result = TunerSuite::optimise(CtrlKind::PID, bounds, costFn);
        static std::function<double(const std::vector<double> &)>
        makeISECost(const StateSpace &plant,
                    double ref,
                    int N,
                    std::function<std::unique_ptr<IController>(const std::vector<double> &)> factory);

        // Same but uses ITAE weighting (t . |error|) - penalises late errors more.
        static std::function<double(const std::vector<double> &)>
        makeITAECost(const StateSpace &plant,
                     double ref,
                     int N,
                     std::function<std::unique_ptr<IController>(const std::vector<double> &)> factory);

    private:
        // Emit soft warning to std::clog and store in base.
        static void softWarn(TuningResultBase &base, const std::string &msg);

        // Nelder-Mead simplex minimiser.
        static std::vector<double> nelderMead(
            std::function<double(const std::vector<double> &)> f,
            std::vector<double> x0,
            const std::vector<std::pair<double, double>> &bounds,
            int maxEvals,
            double tol,
            int *evalCount_out);
    };

} // namespace ctrl
