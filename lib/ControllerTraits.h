#pragma once
#include <type_traits>

/**
 * @file ControllerTraits.h
 * @brief Compile-time metadata mapping each controller type to supported tuning strategies.
 *
 * Two enforcement levels are provided:
 *
 * - **Hard error (`static_assert`)** — fires in ControllerTuner.h `tuneFor<C>()` when
 *   `ControllerTraits<C>::supports_<X>` is `false`. The message names compatible
 *   controllers and suggests the correct tuner.
 *
 * - **Soft warning (`[[deprecated]]` struct)** — instantiated inside an `if constexpr`
 *   branch in the tuner template. Warns without blocking compilation when a strategy is
 *   technically valid but leaves part of the controller un-tuned (e.g., pole placement
 *   on DiscreteLQG ignores Kalman Qf/Rf).
 *
 * **To register a new controller:**
 * 1. Forward-declare it below.
 * 2. Add a `ctrl::ControllerTraits<YourType>` specialisation with all five `supports_*` fields.
 */

namespace ctrl
{

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations (no headers pulled in)
// ─────────────────────────────────────────────────────────────────────────────

class DiscretePID;
class DiscreteLQR;
class DiscreteLQG;
class DiscreteMPC;
class ExtremumSeeker;
class DiscreteSMC;
class SuperTwistingSMC;
class DiscreteADRC;
class DiscreteLeadLag;
class SmithPredictor;
class DiscreteHinf;
class RepetitiveController;
class GeneralizedPredictiveController;

// ─────────────────────────────────────────────────────────────────────────────
// Category tags
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Controller-category tag types for template dispatch.
 */
namespace tag
{
    struct PID               {}; ///< DiscretePID.
    struct StateFeedback     {}; ///< DiscreteLQR (full-state feedback).
    struct OutputFeedback    {}; ///< DiscreteLQG (LQR + Kalman observer).
    struct ModelPredictive   {}; ///< DiscreteMPC.
    struct ExtremumSeeking   {}; ///< ExtremumSeeker.
    struct SlidingMode       {}; ///< DiscreteSMC.
    struct ActiveDisturbance {}; ///< DiscreteADRC / LADRC.
    struct FrequencyDomain   {}; ///< DiscreteLeadLag.
    struct DeadTimeComp      {}; ///< SmithPredictor.
    struct RobustControl     {}; ///< DiscreteHinf.
    struct Repetitive        {}; ///< RepetitiveController.
    struct GeneralizedPredictive {}; ///< GeneralizedPredictiveController.
} // namespace tag

// ─────────────────────────────────────────────────────────────────────────────
// Soft-warning stubs (compile-time conditional warnings)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Internal detail namespace for conditional `[[deprecated]]` warnings.
 *
 * Instantiating any of these functions emits a `[[deprecated]]` compiler warning.
 * They are used inside `if constexpr` branches so the warning fires only when the
 * template argument triggers the condition.
 */
namespace detail
{
    /**
     * @brief Warn when LQR pole-placement is used with DiscreteLQG without also tuning Qf/Rf.
     */
    template <typename T>
    [[deprecated(
        "\n[LQRWeightTuner::polePlacementHintFor<DiscreteLQG>]"
        " Partial tuning — observer gains not addressed.\n"
        "  Pole placement steers LQR closed-loop eigenvalues via Q/R.\n"
        "  The Kalman observer gains depend on Qf, Rf (noise covariances),\n"
        "  which are NOT set by this method.\n"
        "  --> Also call KalmanWeightTuner::fromNoiseFor<DiscreteLQG>()\n"
        "      to tune the observer part of LQG.\n")]]
    void emit_PolePlacement_LQG_Warning() {}

} // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// Primary template — undefined for unregistered types
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Compile-time tuning capability traits for a controller type @p C.
 *
 * The primary template is intentionally undefined; using it for an unregistered
 * type produces a clear `static_assert` diagnostic that names the missing
 * specialisation.
 *
 * Each specialisation exposes:
 * - `category`                — tag type (tag::PID, tag::StateFeedback, …).
 * - `name`                    — human-readable string for diagnostics.
 * - `supports_heuristic_pid`  — ZN, Cohen-Coon, Lambda/IMC, Relay, AMIGO.
 * - `supports_lqr_tuning`     — Bryson's rule, pole-placement hint.
 * - `supports_mpc_tuning`     — MPCHorizonTuner.
 * - `supports_freq_tuning`    — LoopShapingTuner (lead/lag).
 * - `supports_kalman_tuning`  — Qf/Rf observer noise selection.
 */
template <typename C>
struct ControllerTraits
{
    static_assert(sizeof(C) == 0,
                  "\n[ControllerTraits] No traits registered for this controller type.\n"
                  "  Add a ctrl::ControllerTraits<YourType> specialisation\n"
                  "  in lib/ControllerTraits.h before using it with any tuner.\n");
};

// ─────────────────────────────────────────────────────────────────────────────
// Specialisations
// ─────────────────────────────────────────────────────────────────────────────

/** @brief Traits for DiscretePID — supports all heuristic PID tuners. */
template <>
struct ControllerTraits<DiscretePID>
{
    using category = tag::PID;
    static constexpr const char *name = "DiscretePID";
    static constexpr bool supports_heuristic_pid = true;   ///< ZN, Cohen-Coon, Lambda/IMC, Relay, AMIGO.
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;
};

/** @brief Traits for DiscreteLQR — supports Bryson's rule and pole-placement hint. */
template <>
struct ControllerTraits<DiscreteLQR>
{
    using category = tag::StateFeedback;
    static constexpr const char *name = "DiscreteLQR";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = true;   ///< Bryson's rule, pole-placement hint.
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;  ///< No observer.
};

/** @brief Traits for DiscreteLQG — supports LQR tuning (controller part) and Kalman noise tuning (observer part). */
template <>
struct ControllerTraits<DiscreteLQG>
{
    using category = tag::OutputFeedback;
    static constexpr const char *name = "DiscreteLQG";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = true;   ///< Bryson's rule, pole-placement hint (LQR part).
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = true;   ///< Qf/Rf observer noise selection.
};

/** @brief Traits for DiscreteMPC — supports MPCHorizonTuner. */
template <>
struct ControllerTraits<DiscreteMPC>
{
    using category = tag::ModelPredictive;
    static constexpr const char *name = "DiscreteMPC";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = true;   ///< MPCHorizonTuner.
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;
};

/**
 * @brief Traits for ExtremumSeeker — no classical auto-tuner; set parameters directly.
 *
 * ESC is self-optimising once deployed; dither amplitude, frequency, and integrator
 * gain are set from plant bandwidth knowledge rather than a closed-form tuner.
 */
template <>
struct ControllerTraits<ExtremumSeeker>
{
    using category = tag::ExtremumSeeking;
    static constexpr const char *name = "ExtremumSeeker";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;
};

/**
 * @brief Traits for DiscreteSMC — no classical auto-tuner; parameters from Lyapunov design.
 *
 * SMC parameters (c_e, K, φ) are derived from sliding-surface theory and Lyapunov
 * stability conditions, not from auto-tuners.
 */
template <>
struct ControllerTraits<DiscreteSMC>
{
    using category = tag::SlidingMode;
    static constexpr const char *name = "DiscreteSMC";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;
};

/**
 * @brief Traits for SuperTwistingSMC — same design philosophy as DiscreteSMC.
 *
 * Gains K1/K2 are derived from Moreno-Osorio Lyapunov conditions (K2 > K1²/4).
 * No auto-tuner support; tune by Lyapunov analysis or empirical gain sweep.
 */
template <>
struct ControllerTraits<SuperTwistingSMC>
{
    using category = tag::SlidingMode;
    static constexpr const char *name = "SuperTwistingSMC";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;
};

/**
 * @brief Traits for DiscreteADRC — uses bandwidth parameterisation (Gao 2003), not auto-tuners.
 *
 * Parameters (ωc, ωo, b0) are set directly via the bandwidth-parameterised LADRC approach.
 */
template <>
struct ControllerTraits<DiscreteADRC>
{
    using category = tag::ActiveDisturbance;
    static constexpr const char *name = "DiscreteADRC";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;
};

/** @brief Traits for DiscreteLeadLag — supports LoopShapingTuner. */
template <>
struct ControllerTraits<DiscreteLeadLag>
{
    using category = tag::FrequencyDomain;
    static constexpr const char *name = "DiscreteLeadLag";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = true;   ///< LoopShapingTuner.
    static constexpr bool supports_kalman_tuning = false;
};

/**
 * @brief Traits for SmithPredictor — tune the inner controller, not the wrapper.
 *
 * Construct SmithPredictor with a pre-tuned inner IController (typically DiscretePID)
 * and the dead-time model; the wrapper itself has no tunable parameters.
 */
template <>
struct ControllerTraits<SmithPredictor>
{
    using category = tag::DeadTimeComp;
    static constexpr const char *name = "SmithPredictor";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;
};

/**
 * @brief Traits for DiscreteHinf — synthesis via DGKF bisection, not classical auto-tuners.
 *
 * Performance weights W1/W2/W3 encode the design objectives. Use
 * `MixedSensitivity::build()` and `DiscreteHinf::solve()` to synthesise the controller.
 */
template <>
struct ControllerTraits<DiscreteHinf>
{
    using category = tag::RobustControl;
    static constexpr const char *name = "DiscreteHinf";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;  ///< Design via weights, not loop-shaping tuner.
    static constexpr bool supports_kalman_tuning = false;
};

/**
 * @brief Traits for RepetitiveController — internal-model-principle design, not auto-tuners.
 *
 * Learning gain and Q-filter are set from IMP design rules. The inner baseline
 * controller (typically DiscretePID) should be tuned first.
 */
template <>
struct ControllerTraits<RepetitiveController>
{
    using category = tag::Repetitive;
    static constexpr const char *name = "RepetitiveController";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = false;
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;
};

/**
 * @brief Traits for GeneralizedPredictiveController — supports MPCHorizonTuner (same horizon semantics as DiscreteMPC).
 */
template <>
struct ControllerTraits<GeneralizedPredictiveController>
{
    using category = tag::GeneralizedPredictive;
    static constexpr const char *name = "GeneralizedPredictiveController";
    static constexpr bool supports_heuristic_pid = false;
    static constexpr bool supports_lqr_tuning    = false;
    static constexpr bool supports_mpc_tuning    = true;   ///< MPCHorizonTuner — same horizon/weight semantics.
    static constexpr bool supports_freq_tuning   = false;
    static constexpr bool supports_kalman_tuning = false;
};

} // namespace ctrl
