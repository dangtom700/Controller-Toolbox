#pragma once
#include "PlantModel.h"
#include "DiscretePID.h"
#include "DiscreteLQR.h"
#include "DiscreteMPC.h"
#include "DiscreteLeadLag.h"
#include "ControllerTraits.h"
#include <vector>
#include <complex>
#include <functional>

/**
 * @file ControllerTuner.h
 * @brief Offline and online tuning utilities with compile-time type safety.
 *
 * Every tuner exposes two interfaces:
 * - `tuneImpl(…)` / `computePIDParams(…)` — unchecked, for internal use or when the
 *   controller type is already known.
 * - `tuneFor<C>(…)` / `computePIDParamsFor<C>(…)` — template wrapper that triggers a
 *   `static_assert` (hard compiler error) when `C` is incompatible, with a diagnostic
 *   naming the correct tuner.
 *
 * **Tuners provided:**
 * - RelayAutoTuner     — Åström-Hägglund relay feedback test → (Ku, Tu) → PIDParams.
 * - StepResponseTuner  — Open-loop FOPDT identification → PIDParams.
 * - LQRWeightTuner     — Bryson's rule / pole-placement hint → LQRParams.
 * - MPCHorizonTuner    — Settling-time estimate → (Np, Nc, ρy, ρu) recommendation.
 * - ZieglerNicholsTuner — Standalone ZN from (Ku, Tu).
 * - CohenCoonTuner     — FOPDT-based PID (Cohen & Coon 1953).
 * - LoopShapingTuner   — Crossover frequency + phase margin → LeadLagParams.
 * - KalmanWeightTuner  — Noise levels → Qf / Rf for DiscreteLQG.
 *
 * **Strategies that are intentionally NOT included** (require external solvers):
 * - H∞ / μ-synthesis — SLICOT, Python-control, or MATLAB.
 * - QFT — QFT Toolbox, QDESIGN.
 * - GA / PSO / DE — Optuna, DEAP.
 * - Bayesian optimisation — BoTorch / GPyOpt / scikit-optimize.
 * - MRAC / STR / IFT / VRFT — adaptive control; separate library.
 *
 * @see Åström & Hägglund, "PID Controllers: Theory, Design, and Tuning" (1988, 2006).
 * @see Bryson & Ho, "Applied Optimal Control" (1975).
 * @see Cohen & Coon, "Theoretical Consideration of Retarded Control" (1953).
 * @see Rivera, Morari & Skogestad, "Internal Model Control" Ind. Eng. Chem. (1986).
 * @see Franklin, Powell & Emami-Naeini, "Feedback Control of Dynamic Systems".
 */

namespace ctrl
{

// ─────────────────────────────────────────────────────────────────────────────
// PID Tuning Rule Selector
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief PID tuning rule applied by heuristic tuners.
 */
enum class PIDTuningRule
{
    ZieglerNichols, ///< Classic ZN — fast, ~25% overshoot (Ziegler & Nichols 1942).
    TyreusLuyben,   ///< Conservative ZN variant — better load rejection (Tyreus & Luyben 1992).
    IMC,            ///< Internal Model Control (lambda-tuning) — requires FOPDT model + λ.
    AMIGO           ///< Approximate M-constrained Integral Gain Optimisation (Åström 2006).
};

// ─────────────────────────────────────────────────────────────────────────────
// RelayAutoTuner — Åström-Hägglund relay feedback test
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Configuration for RelayAutoTuner.
 */
struct RelayTunerConfig
{
    double relayAmplitude = 1.0; ///< Relay output amplitude ±d.
    double hysteresis     = 0.0; ///< Dead-band [same units as y] to suppress noise-driven chatter.
    int    cyclesRequired = 3;   ///< Stable oscillation cycles required before the tuner exits.
};

/**
 * @brief Åström-Hägglund relay feedback auto-tuner.
 *
 * Applies a relay ±d to the closed-loop plant, records the resulting limit cycle,
 * then derives the ultimate gain Ku and ultimate period Tu:
 * @code
 *   Ku = (4·d) / (π·a_y)   where a_y = peak-to-peak output amplitude / 2
 *   Tu = average oscillation period from zero-crossing times
 * @endcode
 *
 * **Usage pattern (simulation loop):**
 * @code
 *   while (!tuner.isDone()) {
 *       double u = tuner.step(plant_output);
 *       // apply u, advance plant simulation
 *   }
 *   PIDParams p = tuner.computePIDParams(PIDTuningRule::TyreusLuyben);
 * @endcode
 */
class RelayAutoTuner
{
public:
    /**
     * @brief Construct the relay tuner.
     * @param cfg        Relay amplitude, hysteresis, and cycle count.
     * @param sampleTime Ts [s].
     */
    RelayAutoTuner(const RelayTunerConfig &cfg, double sampleTime);

    /**
     * @brief Feed the current plant output and receive the relay control signal.
     * @param y Current plant output y[k].
     * @return Relay output u[k] ∈ {−d, +d}.
     */
    double step(double y);

    /** @brief Returns `true` when enough stable limit cycles have been collected. */
    bool isDone() const { return done_; }

    /**
     * @brief Compute PID parameters from the identified (Ku, Tu) — unchecked.
     * @param rule   PID tuning rule to apply.
     * @param lambda Closed-loop time constant for IMC (−1 = auto).
     * @return PIDParams for DiscretePID.
     */
    PIDParams computePIDParams(PIDTuningRule rule   = PIDTuningRule::TyreusLuyben,
                               double        lambda = -1.0) const;

    /**
     * @brief Type-checked wrapper — hard compiler error for non-PID controllers.
     *
     * @tparam C Target controller type. Must have `ControllerTraits<C>::supports_heuristic_pid == true`.
     * @param rule   PID tuning rule.
     * @param lambda IMC closed-loop time constant (−1 = auto).
     * @return PIDParams.
     */
    template <typename C>
    PIDParams computePIDParamsFor(PIDTuningRule rule   = PIDTuningRule::TyreusLuyben,
                                  double        lambda = -1.0) const
    {
        static_assert(ControllerTraits<C>::supports_heuristic_pid,
                      "\n[RelayAutoTuner::computePIDParamsFor<C>]"
                      " C does not support relay-based PID tuning.\n"
                      "  Relay tuning produces PIDParams from (Ku, Tu);"
                      " only valid for DiscretePID.\n"
                      "  --> DiscreteLQR / DiscreteLQG :"
                      " LQRWeightTuner::brysonMethodFor<C>(xmax, umax)\n"
                      "  --> DiscreteMPC               :"
                      " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                      "  --> DiscreteLeadLag           :"
                      " LoopShapingTuner::tuneFor<DiscreteLeadLag>(omega_c, phase_add, gain)\n"
                      "  --> DiscreteSMC / ADRC / ESC  :"
                      " set parameters directly (no standard auto-tuner)\n"
                      "  --> SmithPredictor            :"
                      " tune the inner IController, not the wrapper\n");
        return computePIDParams(rule, lambda);
    }

    /** @brief Ultimate gain Ku from the completed relay test. */
    double ultimateGain()   const { return Ku_; }
    /** @brief Ultimate period Tu [s] from the completed relay test. */
    double ultimatePeriod() const { return Tu_; }

private:
    RelayTunerConfig     cfg_;
    double               Ts_;
    bool                 done_       = false;
    bool                 relayHigh_  = false;
    bool                 collecting_ = false;
    double               Ku_         = 0.0;
    double               Tu_         = 0.0;
    double               y_prev_     = 0.0;
    double               peak_pos_   = -1e9;
    double               peak_neg_   =  1e9;
    int                  step_cnt_   = 0;
    std::vector<double>  crossTimes_;
};

// ─────────────────────────────────────────────────────────────────────────────
// StepResponseTuner — open-loop FOPDT identification
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Open-loop step-response FOPDT identifier and PID tuner.
 *
 * Fits a First-Order Plus Dead-Time (FOPDT) model to open-loop step-response data
 * using the process-reaction-curve tangent method:
 * @code
 *   G(s) ≈ K · e^{−θs} / (τs + 1)
 * @endcode
 */
class StepResponseTuner
{
public:
    /**
     * @brief FOPDT model parameters.
     */
    struct FOPDTModel
    {
        double K;     ///< Process static gain.
        double tau;   ///< First-order time constant [s].
        double theta; ///< Dead time [s].
    };

    /**
     * @brief Identify a FOPDT model from open-loop step-response data.
     * @param time          Time vector [s], strictly increasing.
     * @param output        Plant output vector (same length as time).
     * @param stepMagnitude Magnitude of the applied step input.
     * @return Identified FOPDTModel (K, τ, θ).
     */
    static FOPDTModel identify(const std::vector<double> &time,
                               const std::vector<double> &output,
                               double stepMagnitude);

    /**
     * @brief Compute PID parameters from a FOPDT model — unchecked.
     * @param model  Identified FOPDT model.
     * @param Ts     Sample time [s].
     * @param rule   PID tuning rule.
     * @param lambda IMC closed-loop time constant (−1 = 0.5·τ).
     * @return PIDParams for DiscretePID.
     */
    static PIDParams computePIDParams(const FOPDTModel &model,
                                      double            Ts,
                                      PIDTuningRule     rule   = PIDTuningRule::IMC,
                                      double            lambda = -1.0);

    /**
     * @brief Type-checked wrapper — hard compiler error for non-PID controllers.
     * @tparam C Target controller type.
     */
    template <typename C>
    static PIDParams computePIDParamsFor(const FOPDTModel &model,
                                         double            Ts,
                                         PIDTuningRule     rule   = PIDTuningRule::IMC,
                                         double            lambda = -1.0)
    {
        static_assert(ControllerTraits<C>::supports_heuristic_pid,
                      "\n[StepResponseTuner::computePIDParamsFor<C>]"
                      " C does not support FOPDT-based PID tuning.\n"
                      "  FOPDT tuning produces PIDParams; only valid for DiscretePID.\n"
                      "  --> DiscreteLQR / DiscreteLQG :"
                      " LQRWeightTuner::brysonMethodFor<C>(xmax, umax)\n"
                      "  --> DiscreteMPC               :"
                      " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                      "  --> DiscreteLeadLag           :"
                      " LoopShapingTuner::tuneFor<DiscreteLeadLag>(omega_c, phase_add, gain)\n"
                      "  --> DiscreteSMC / ADRC / ESC  :"
                      " set parameters directly (no standard auto-tuner)\n"
                      "  --> SmithPredictor            :"
                      " tune the inner IController, not the wrapper\n");
        return computePIDParams(model, Ts, rule, lambda);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// LQRWeightTuner — Bryson's rule and pole-placement hint
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief LQR weight selector for DiscreteLQR and DiscreteLQG.
 *
 * Provides two methods:
 * - **Bryson's rule** — diagonal Q and R from per-channel maximum acceptable deviations.
 * - **Pole-placement hint** — iterative Q/R search to steer closed-loop poles toward
 *   desired locations.
 */
class LQRWeightTuner
{
public:
    /**
     * @brief Bryson's rule: Q = diag(1/xmax²), R = diag(1/umax²) — unchecked.
     * @param maxStateDeviation   Maximum acceptable state deviation per channel (n×1).
     * @param maxControlEffort    Maximum acceptable control effort per channel (m×1).
     * @return LQRParams with diagonal Q and R.
     */
    static LQRParams brysonMethod(const Eigen::VectorXd &maxStateDeviation,
                                  const Eigen::VectorXd &maxControlEffort);

    /**
     * @brief Iterative Q/R search to steer closed-loop poles — unchecked.
     * @param plant         Discrete-time state-space model.
     * @param desiredPoles  Target closed-loop eigenvalues (inside the unit circle).
     * @param maxIter       Maximum search iterations.
     * @return LQRParams that approximate the desired pole locations.
     */
    static LQRParams polePlacementHint(const StateSpace                        &plant,
                                       const std::vector<std::complex<double>> &desiredPoles,
                                       int maxIter = 200);

    /**
     * @brief Type-checked Bryson's rule wrapper.
     * @tparam C Must have `supports_lqr_tuning == true` (DiscreteLQR or DiscreteLQG).
     */
    template <typename C>
    static LQRParams brysonMethodFor(const Eigen::VectorXd &maxStateDeviation,
                                     const Eigen::VectorXd &maxControlEffort)
    {
        static_assert(ControllerTraits<C>::supports_lqr_tuning,
                      "\n[LQRWeightTuner::brysonMethodFor<C>]"
                      " C does not support LQR weight tuning.\n"
                      "  Bryson's method returns LQRParams (Q, R diagonal matrices);\n"
                      "  only valid for DiscreteLQR and DiscreteLQG.\n"
                      "  --> DiscretePID     : ZieglerNicholsTuner or"
                      " StepResponseTuner::computePIDParamsFor<DiscretePID>()\n"
                      "  --> DiscreteMPC     : MPC has its own Q/R via MPCParams;"
                      " use MPCHorizonTuner::recommendFor<DiscreteMPC>()\n"
                      "  --> DiscreteLeadLag : LoopShapingTuner::tuneFor<DiscreteLeadLag>()\n"
                      "  --> DiscreteSMC / ADRC / ESC :"
                      " set parameters directly (no standard auto-tuner)\n");
        return brysonMethod(maxStateDeviation, maxControlEffort);
    }

    /**
     * @brief Type-checked pole-placement hint wrapper.
     *
     * Emits a `[[deprecated]]` warning (not error) when `C = DiscreteLQG`, because
     * pole placement only tunes the LQR part — the Kalman observer gains (Qf, Rf)
     * still need separate attention via KalmanWeightTuner.
     *
     * @tparam C Must have `supports_lqr_tuning == true`.
     */
    template <typename C>
    static LQRParams polePlacementHintFor(const StateSpace                        &plant,
                                          const std::vector<std::complex<double>> &desiredPoles,
                                          int maxIter = 200)
    {
        static_assert(ControllerTraits<C>::supports_lqr_tuning,
                      "\n[LQRWeightTuner::polePlacementHintFor<C>]"
                      " C does not support LQR weight tuning.\n"
                      "  Pole-placement hint adjusts LQR Q/R to steer closed-loop poles;\n"
                      "  only valid for DiscreteLQR and DiscreteLQG.\n"
                      "  --> DiscretePID     : ZieglerNicholsTuner or"
                      " StepResponseTuner::computePIDParamsFor<DiscretePID>()\n"
                      "  --> DiscreteMPC     : use MPCHorizonTuner::recommendFor<DiscreteMPC>()\n"
                      "  --> DiscreteLeadLag : LoopShapingTuner::tuneFor<DiscreteLeadLag>()\n");

        if constexpr (std::is_same_v<C, DiscreteLQG>)
        {
            detail::emit_PolePlacement_LQG_Warning<C>();
        }

        return polePlacementHint(plant, desiredPoles, maxIter);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// MPCHorizonTuner — settling-time-based horizon recommendation
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief MPC horizon and weight recommender for DiscreteMPC.
 */
class MPCHorizonTuner
{
public:
    /**
     * @brief Recommended horizon and weight parameters for DiscreteMPC.
     */
    struct Recommendation
    {
        int    Np;                     ///< Prediction horizon [steps].
        int    Nc;                     ///< Control horizon [steps].
        double rho_y;                  ///< Output tracking weight.
        double rho_u;                  ///< Move suppression weight.
        double estimatedSettlingTime;  ///< Estimated 2% settling time [s].
    };

    /**
     * @brief Estimate the 2% settling time of a plant by step simulation.
     * @param plant    Discrete-time state-space model.
     * @param maxSteps Maximum simulation steps before giving up.
     * @return Estimated settling time [s], or maxSteps·Ts if not settled.
     */
    static double estimateSettlingTime(const StateSpace &plant, int maxSteps = 5000);

    /**
     * @brief Recommend Np, Nc, ρy, ρu for a given plant — unchecked.
     * @param plant  Discrete-time state-space model.
     * @param Ts     Sample time [s].
     * @param rho_y  Output tracking weight (passed through).
     * @param rho_u  Move suppression weight (passed through).
     * @return Recommendation with Np and Nc scaled to the plant settling time.
     */
    static Recommendation recommend(const StateSpace &plant,
                                    double Ts,
                                    double rho_y = 1.0,
                                    double rho_u = 0.1);

    /**
     * @brief Type-checked wrapper — hard compiler error for non-MPC controllers.
     * @tparam C Must have `supports_mpc_tuning == true` (DiscreteMPC or GeneralizedPredictiveController).
     */
    template <typename C>
    static Recommendation recommendFor(const StateSpace &plant,
                                       double Ts,
                                       double rho_y = 1.0,
                                       double rho_u = 0.1)
    {
        static_assert(ControllerTraits<C>::supports_mpc_tuning,
                      "\n[MPCHorizonTuner::recommendFor<C>]"
                      " C does not support MPC horizon tuning.\n"
                      "  MPC horizon tuning (Np, Nc, rho) only applies to DiscreteMPC.\n"
                      "  --> DiscretePID     :"
                      " StepResponseTuner::computePIDParamsFor<DiscretePID>()\n"
                      "  --> DiscreteLQR     :"
                      " LQRWeightTuner::brysonMethodFor<DiscreteLQR>(xmax, umax)\n"
                      "  --> DiscreteLQG     :"
                      " LQRWeightTuner::brysonMethodFor<DiscreteLQG>()"
                      " + KalmanWeightTuner::fromNoiseFor<DiscreteLQG>()\n"
                      "  --> DiscreteLeadLag :"
                      " LoopShapingTuner::tuneFor<DiscreteLeadLag>()\n"
                      "  --> DiscreteSMC / ADRC / ESC :"
                      " set parameters directly (no standard auto-tuner)\n");
        return recommend(plant, Ts, rho_y, rho_u);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// KalmanNoiseParams — output of KalmanWeightTuner
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Noise covariance matrices for Kalman observer initialisation.
 */
struct KalmanNoiseParams
{
    Eigen::MatrixXd Qf; ///< Process noise covariance (n×n).
    Eigen::MatrixXd Rf; ///< Measurement noise covariance (p×p).
    Eigen::MatrixXd P0; ///< Initial state covariance estimate.
};

// ─────────────────────────────────────────────────────────────────────────────
// ZieglerNicholsTuner — standalone ZN from (Ku, Tu)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Standalone Ziegler-Nichols tuner from ultimate gain and period.
 *
 * Classic ZN rules derived from (Ku, Tu). Produces aggressive settings; consider
 * de-tuning Kp by 20–30% or switching to TyreusLuyben / AMIGO for better robustness.
 *
 * @see Ziegler & Nichols, "Optimum Settings for Automatic Controllers" (1942).
 */
class ZieglerNicholsTuner
{
public:
    /**
     * @brief Input to the ZN tuner.
     */
    struct Input
    {
        double Ku; ///< Ultimate gain from relay test or manual search.
        double Tu; ///< Ultimate period [s].
    };

    /**
     * @brief Compute PID parameters from (Ku, Tu) — unchecked.
     * @param in ZN input (Ku, Tu).
     * @return PIDParams.
     */
    static PIDParams tuneImpl(const Input &in);

    /**
     * @brief Type-checked wrapper — hard compiler error for non-PID controllers.
     * @tparam C Must have `supports_heuristic_pid == true`.
     */
    template <typename C>
    static PIDParams tuneFor(const Input &in)
    {
        static_assert(ControllerTraits<C>::supports_heuristic_pid,
                      "\n[ZieglerNicholsTuner::tuneFor<C>]"
                      " C does not support ZN tuning.\n"
                      "  ZN produces PIDParams from (Ku, Tu);"
                      " only valid for DiscretePID.\n"
                      "  --> DiscreteLQR / DiscreteLQG :"
                      " LQRWeightTuner::brysonMethodFor<C>(xmax, umax)\n"
                      "  --> DiscreteMPC               :"
                      " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                      "  --> DiscreteLeadLag           :"
                      " LoopShapingTuner::tuneFor<DiscreteLeadLag>(omega_c, phase_add, gain)\n"
                      "  --> DiscreteSMC / ADRC / ESC  :"
                      " set parameters directly (no standard auto-tuner)\n"
                      "  --> SmithPredictor            :"
                      " tune the inner IController, not the wrapper\n");
        return tuneImpl(in);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// CohenCoonTuner — FOPDT-based PID (Cohen & Coon 1953)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Cohen-Coon PID tuner from a FOPDT model.
 *
 * More accurate than ZN for processes where θ/τ ∈ [0.1, 1.0].
 *
 * Formulas (PID form, r = θ/τ):
 * @code
 *   Kp = (τ / (K·θ)) · (4/3 + r/4)
 *   Ti = θ · (32 + 6r) / (13 + 8r)
 *   Td = 4θ / (11 + 2r)
 * @endcode
 *
 * @see Cohen & Coon, "Theoretical Consideration of Retarded Control" (1953).
 */
class CohenCoonTuner
{
public:
    /**
     * @brief Compute PID parameters from a FOPDT model — unchecked.
     *
     * Obtain @p m via StepResponseTuner::identify().
     *
     * @param m  FOPDT model (K, τ, θ).
     * @param Ts Sample time [s].
     * @return PIDParams.
     */
    static PIDParams tuneImpl(const StepResponseTuner::FOPDTModel &m, double Ts);

    /**
     * @brief Type-checked wrapper — hard compiler error for non-PID controllers.
     * @tparam C Must have `supports_heuristic_pid == true`.
     */
    template <typename C>
    static PIDParams tuneFor(const StepResponseTuner::FOPDTModel &m, double Ts)
    {
        static_assert(ControllerTraits<C>::supports_heuristic_pid,
                      "\n[CohenCoonTuner::tuneFor<C>]"
                      " C does not support Cohen-Coon tuning.\n"
                      "  Cohen-Coon produces PIDParams from a FOPDT model;"
                      " only valid for DiscretePID.\n"
                      "  --> DiscreteLQR / DiscreteLQG :"
                      " LQRWeightTuner::brysonMethodFor<C>(xmax, umax)\n"
                      "  --> DiscreteMPC               :"
                      " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                      "  --> DiscreteLeadLag           :"
                      " LoopShapingTuner::tuneFor<DiscreteLeadLag>(omega_c, phase_add, gain)\n"
                      "  --> DiscreteSMC / ADRC / ESC  :"
                      " set parameters directly (no standard auto-tuner)\n"
                      "  --> SmithPredictor            :"
                      " tune the inner IController, not the wrapper\n");
        return tuneImpl(m, Ts);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// LoopShapingTuner — frequency-domain → LeadLagParams
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Lead compensator designer for DiscreteLeadLag via frequency-domain loop shaping.
 *
 * Designs C(s) = K·(s+z)/(s+p) so that:
 * - The gain crossover occurs at ωc (|L(jωc)| = 1).
 * - The compensator contributes @p phase_add_deg of phase lead at ωc.
 *
 * Derivation (lead, φ > 0):
 * @code
 *   β = sin(φ),  α = (1+β)/(1−β)
 *   z = ωc / √α,  p = ωc · √α,  K = √α / |G(jωc)|
 * @endcode
 *
 * @see Franklin, Powell & Emami-Naeini, "Feedback Control of Dynamic Systems" §9.3.
 */
class LoopShapingTuner
{
public:
    /**
     * @brief Input parameters for the loop-shaping design.
     */
    struct Input
    {
        double omega_c;       ///< Desired gain crossover frequency [rad/s].
        double phase_add_deg; ///< Phase lead to add at ωc [°] — must be in (0, 90).
        double gain_at_wc;    ///< Open-loop |G(jωc)| without the compensator.
    };

    /**
     * @brief Design the lead compensator — unchecked.
     * @param in Loop-shaping inputs (ωc, phase lead, plant gain at ωc).
     * @return LeadLagParams for DiscreteLeadLag.
     */
    static LeadLagParams tuneImpl(const Input &in);

    /**
     * @brief Type-checked wrapper — hard compiler error for non-frequency-domain controllers.
     * @tparam C Must have `supports_freq_tuning == true` (DiscreteLeadLag).
     */
    template <typename C>
    static LeadLagParams tuneFor(const Input &in)
    {
        static_assert(ControllerTraits<C>::supports_freq_tuning,
                      "\n[LoopShapingTuner::tuneFor<C>]"
                      " C does not support frequency-domain tuning.\n"
                      "  LoopShapingTuner returns LeadLagParams;"
                      " only valid for DiscreteLeadLag.\n"
                      "  --> DiscretePID     :"
                      " ZieglerNicholsTuner or"
                      " StepResponseTuner::computePIDParamsFor<DiscretePID>()\n"
                      "  --> DiscreteLQR     :"
                      " LQRWeightTuner::brysonMethodFor<DiscreteLQR>(xmax, umax)\n"
                      "  --> DiscreteLQG     :"
                      " LQRWeightTuner::brysonMethodFor<DiscreteLQG>()"
                      " + KalmanWeightTuner::fromNoiseFor<DiscreteLQG>()\n"
                      "  --> DiscreteMPC     :"
                      " MPCHorizonTuner::recommendFor<DiscreteMPC>(plant, Ts)\n"
                      "  --> DiscreteSMC / ADRC / ESC :"
                      " set parameters directly (no standard auto-tuner)\n");
        return tuneImpl(in);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// KalmanWeightTuner — noise covariance selection for DiscreteLQG
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Heuristic noise covariance selector for the Kalman observer in DiscreteLQG.
 *
 * Provides starting points for Qf (process noise covariance) and Rf (measurement noise
 * covariance) using Bryson-like scaling: Qf = diag(σ²_process), Rf = diag(σ²_meas).
 *
 * The separation principle guarantees the LQR and Kalman gains can be tuned
 * independently; use LQRWeightTuner for the LQR part.
 *
 * @see Bryson & Ho, "Applied Optimal Control" (1975) §14.3.
 */
class KalmanWeightTuner
{
public:
    /**
     * @brief Per-channel noise: Qf = diag(σ²_process), Rf = diag(σ²_meas) — unchecked.
     * @param maxProcessNoise Per-channel process noise standard deviation (n×1).
     * @param maxMeasNoise    Per-channel measurement noise standard deviation (p×1).
     * @return KalmanNoiseParams with diagonal Qf, Rf, and P0 = Qf.
     */
    static KalmanNoiseParams fromNoise(const Eigen::VectorXd &maxProcessNoise,
                                       const Eigen::VectorXd &maxMeasNoise);

    /**
     * @brief Isotropic (scalar) noise: Qf = σp²·I, Rf = σm²·I — unchecked.
     * @param nStates      Number of states n.
     * @param nOutputs     Number of outputs p.
     * @param sigmaProcess Process noise standard deviation.
     * @param sigmaMeas    Measurement noise standard deviation.
     * @return KalmanNoiseParams.
     */
    static KalmanNoiseParams isotropic(int    nStates,
                                       int    nOutputs,
                                       double sigmaProcess,
                                       double sigmaMeas);

    /**
     * @brief Type-checked per-channel noise wrapper.
     * @tparam C Must have `supports_kalman_tuning == true` (DiscreteLQG).
     */
    template <typename C>
    static KalmanNoiseParams fromNoiseFor(const Eigen::VectorXd &maxProcessNoise,
                                          const Eigen::VectorXd &maxMeasNoise)
    {
        static_assert(ControllerTraits<C>::supports_kalman_tuning,
                      "\n[KalmanWeightTuner::fromNoiseFor<C>]"
                      " C does not contain a Kalman observer.\n"
                      "  Kalman noise parameters (Qf, Rf) only apply to DiscreteLQG.\n"
                      "  --> DiscreteLQR     :"
                      " full state is assumed known; no observer is needed\n"
                      "  --> DiscretePID     :"
                      " ZieglerNicholsTuner or StepResponseTuner\n"
                      "  --> DiscreteMPC     :"
                      " MPCHorizonTuner::recommendFor<DiscreteMPC>()\n"
                      "  --> Standalone KalmanFilter :"
                      " pass Qf/Rf directly to KalmanFilter's constructor\n");
        return fromNoise(maxProcessNoise, maxMeasNoise);
    }

    /**
     * @brief Type-checked isotropic noise wrapper.
     * @tparam C Must have `supports_kalman_tuning == true` (DiscreteLQG).
     */
    template <typename C>
    static KalmanNoiseParams isotropicFor(int    nStates,
                                          int    nOutputs,
                                          double sigmaProcess,
                                          double sigmaMeas)
    {
        static_assert(ControllerTraits<C>::supports_kalman_tuning,
                      "\n[KalmanWeightTuner::isotropicFor<C>]"
                      " C does not contain a Kalman observer.\n"
                      "  Same constraint as KalmanWeightTuner::fromNoiseFor<C>.\n"
                      "  Only DiscreteLQG embeds a Kalman filter with tunable Qf/Rf.\n");
        return isotropic(nStates, nOutputs, sigmaProcess, sigmaMeas);
    }
};

} // namespace ctrl
