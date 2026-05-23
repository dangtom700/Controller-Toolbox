#pragma once
#include <type_traits>

// ============================================================
//  ControllerTraits.h
//
//  Compile-time metadata that maps each controller type to the
//  tuning strategies it supports.
//
//  Two enforcement levels:
//
//   Hard error  (static_assert) - tuner<C>() in ControllerTuner.h
//       fires when ControllerTraits<C>::supports_<X> == false.
//       The message names the compatible controllers and
//       suggests the correct tuner to use instead.
//
//   Soft warning ([[deprecated]] struct) - instantiated inside
//       an `if constexpr` branch in the tuner template method.
//       Warns without blocking compilation when a strategy is
//       technically valid but leaves part of the controller
//       un-tuned (e.g. pole placement on LQG ignores Kalman Qf/Rf).
//
//  To register a new controller:
//    1. Forward-declare it below.
//    2. Add a ctrl::ControllerTraits<YourType> specialisation.
// ============================================================
namespace ctrl
{

    // -- Forward declarations (no headers pulled in) --------------
    class DiscretePID;
    class DiscreteLQR;
    class DiscreteLQG;
    class DiscreteMPC;
    class ExtremumSeeker;
    class DiscreteSMC;
    class DiscreteADRC;
    class DiscreteLeadLag;
    class SmithPredictor;

    // -- Category tags ---------------------------------------------
    namespace tag
    {
        struct PID
        {
        }; // DiscretePID
        struct StateFeedback
        {
        }; // DiscreteLQR (full-state feedback)
        struct OutputFeedback
        {
        }; // DiscreteLQG (LQR + Kalman observer)
        struct ModelPredictive
        {
        }; // DiscreteMPC
        struct ExtremumSeeking
        {
        }; // ExtremumSeeker
        struct SlidingMode
        {
        }; // DiscreteSMC
        struct ActiveDisturbance
        {
        }; // DiscreteADRC / LADRC
        struct FrequencyDomain
        {
        }; // DiscreteLeadLag
        struct DeadTimeComp
        {
        }; // SmithPredictor
    } // namespace tag

    // -- Compile-time warning stubs --------------------------------
    // Instantiating any of these types emits a [[deprecated]] compiler
    // warning.  They are used inside `if constexpr` branches so the
    // warning is conditional on the template argument.
    namespace detail
    {

        template <typename T>
        [[deprecated(
            "\n[LQRWeightTuner::polePlacementHintFor<DiscreteLQG>]"
            " Partial tuning - observer gains not addressed.\n"
            "  Pole placement steers LQR closed-loop eigenvalues via Q/R.\n"
            "  The Kalman observer gains depend on Qf, Rf (noise covariances),\n"
            "  which are NOT set by this method.\n"
            "  --> Also call KalmanWeightTuner::fromNoiseFor<DiscreteLQG>()\n"
            "      to tune the observer part of LQG.\n")]]
        void emit_PolePlacement_LQG_Warning()
        {
        }

    } // namespace detail

    // -- Primary template - undefined for unknown types ------------
    // Any use of ControllerTraits<T> for an unregistered T triggers
    // a clear compile error that names the missing specialisation.
    template <typename C>
    struct ControllerTraits
    {
        static_assert(sizeof(C) == 0,
                      "\n[ControllerTraits] No traits registered for this controller type.\n"
                      "  Add a ctrl::ControllerTraits<YourType> specialisation\n"
                      "  in lib/ControllerTraits.h before using it with any tuner.\n");
    };

    // -- DiscretePID -----------------------------------------------
    template <>
    struct ControllerTraits<DiscretePID>
    {
        using category = tag::PID;
        static constexpr const char *name = "DiscretePID";
        // Heuristic PID tuners: ZN, Cohen-Coon, Lambda/IMC, Relay, AMIGO
        static constexpr bool supports_heuristic_pid = true;
        static constexpr bool supports_lqr_tuning = false;
        static constexpr bool supports_mpc_tuning = false;
        static constexpr bool supports_freq_tuning = false;
        static constexpr bool supports_kalman_tuning = false;
    };

    // -- DiscreteLQR -----------------------------------------------
    template <>
    struct ControllerTraits<DiscreteLQR>
    {
        using category = tag::StateFeedback;
        static constexpr const char *name = "DiscreteLQR";
        static constexpr bool supports_heuristic_pid = false;
        static constexpr bool supports_lqr_tuning = true; // Bryson, pole-placement hint
        static constexpr bool supports_mpc_tuning = false;
        static constexpr bool supports_freq_tuning = false;
        static constexpr bool supports_kalman_tuning = false; // no observer
    };

    // -- DiscreteLQG -----------------------------------------------
    template <>
    struct ControllerTraits<DiscreteLQG>
    {
        using category = tag::OutputFeedback;
        static constexpr const char *name = "DiscreteLQG";
        static constexpr bool supports_heuristic_pid = false;
        static constexpr bool supports_lqr_tuning = true; // LQR part: Bryson / pole hint
        static constexpr bool supports_mpc_tuning = false;
        static constexpr bool supports_freq_tuning = false;
        static constexpr bool supports_kalman_tuning = true; // observer part: Qf / Rf
    };

    // -- DiscreteMPC -----------------------------------------------
    template <>
    struct ControllerTraits<DiscreteMPC>
    {
        using category = tag::ModelPredictive;
        static constexpr const char *name = "DiscreteMPC";
        static constexpr bool supports_heuristic_pid = false;
        static constexpr bool supports_lqr_tuning = false;
        static constexpr bool supports_mpc_tuning = true; // MPCHorizonTuner
        static constexpr bool supports_freq_tuning = false;
        static constexpr bool supports_kalman_tuning = false;
    };

    // -- ExtremumSeeker --------------------------------------------
    // ESC is self-optimising once deployed; parameters (dither amplitude,
    // frequency, integrator gain) are set from plant bandwidth knowledge,
    // not from a closed-form tuner.
    template <>
    struct ControllerTraits<ExtremumSeeker>
    {
        using category = tag::ExtremumSeeking;
        static constexpr const char *name = "ExtremumSeeker";
        static constexpr bool supports_heuristic_pid = false;
        static constexpr bool supports_lqr_tuning = false;
        static constexpr bool supports_mpc_tuning = false;
        static constexpr bool supports_freq_tuning = false;
        static constexpr bool supports_kalman_tuning = false;
    };

    // -- DiscreteSMC -----------------------------------------------
    // SMC parameters (c_e, K, phi) are derived from sliding-surface
    // theory and Lyapunov stability conditions, not from auto-tuners.
    template <>
    struct ControllerTraits<DiscreteSMC>
    {
        using category = tag::SlidingMode;
        static constexpr const char *name = "DiscreteSMC";
        static constexpr bool supports_heuristic_pid = false;
        static constexpr bool supports_lqr_tuning = false;
        static constexpr bool supports_mpc_tuning = false;
        static constexpr bool supports_freq_tuning = false;
        static constexpr bool supports_kalman_tuning = false;
    };

    // -- DiscreteADRC ----------------------------------------------
    // ADRC uses bandwidth parameterisation (omega_c, omega_o, b0).
    // There is no classical auto-tuner; set parameters directly via
    // the "bandwidth-parameterised LADRC" approach (Gao 2003).
    template <>
    struct ControllerTraits<DiscreteADRC>
    {
        using category = tag::ActiveDisturbance;
        static constexpr const char *name = "DiscreteADRC";
        static constexpr bool supports_heuristic_pid = false;
        static constexpr bool supports_lqr_tuning = false;
        static constexpr bool supports_mpc_tuning = false;
        static constexpr bool supports_freq_tuning = false;
        static constexpr bool supports_kalman_tuning = false;
    };

    // -- DiscreteLeadLag -------------------------------------------
    template <>
    struct ControllerTraits<DiscreteLeadLag>
    {
        using category = tag::FrequencyDomain;
        static constexpr const char *name = "DiscreteLeadLag";
        static constexpr bool supports_heuristic_pid = false;
        static constexpr bool supports_lqr_tuning = false;
        static constexpr bool supports_mpc_tuning = false;
        static constexpr bool supports_freq_tuning = true; // LoopShapingTuner
        static constexpr bool supports_kalman_tuning = false;
    };

    // -- SmithPredictor --------------------------------------------
    // SmithPredictor is a wrapper around an inner IController.
    // Tune the inner controller (typically DiscretePID) using the
    // appropriate tuner, then construct SmithPredictor(inner, model, d).
    template <>
    struct ControllerTraits<SmithPredictor>
    {
        using category = tag::DeadTimeComp;
        static constexpr const char *name = "SmithPredictor";
        static constexpr bool supports_heuristic_pid = false;
        static constexpr bool supports_lqr_tuning = false;
        static constexpr bool supports_mpc_tuning = false;
        static constexpr bool supports_freq_tuning = false;
        static constexpr bool supports_kalman_tuning = false;
    };

} // namespace ctrl
