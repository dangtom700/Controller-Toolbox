#pragma once
#include <string>
#include <unordered_map>

namespace ctrl
{

/**
 * @brief Runtime query of optional modules compiled into the library.
 *
 * Returns a map of feature-name -> bool reflecting the CTRL_HAS_* preprocessor flags
 * that were active when the library was built.  Intended for Python bindings and
 * plugin-discovery code that cannot rely on compile-time conditionals.
 *
 * @code
 *   for (const auto &[name, on] : ctrl::features())
 *       std::printf("  %-20s %s\n", name.c_str(), on ? "yes" : "no");
 * @endcode
 */
inline std::unordered_map<std::string, bool> features()
{
    return {
        {"hinf",
#if defined(CTRL_HAS_HINF)
            true
#else
            false
#endif
        },
        {"subspace",
#if defined(CTRL_HAS_SUBSPACE)
            true
#else
            false
#endif
        },
        {"fuzzy",
#if defined(CTRL_HAS_FUZZY)
            true
#else
            false
#endif
        },
        {"function_approx",
#if defined(CTRL_HAS_FUNCTION_APPROX)
            true
#else
            false
#endif
        },
        {"advanced_kalman",
#if defined(CTRL_HAS_ADVANCED_KALMAN)
            true
#else
            false
#endif
        },
        {"sopdt_identifier",        true},  // always compiled (SOPDTIdentifier.cpp in core sources)
        {"mhe",                     true},  // always compiled (MovingHorizonEstimator.cpp in core sources)
        {"linearisation_helper",    true},  // always compiled (LinearisationHelper.cpp in core sources)
        {"feedback_linearisation",  true},  // always compiled (FeedbackLinearisation.cpp in core sources)
        {"mrac",                    true},  // always compiled (MRACController.cpp in core sources)
        {"balanced_truncation",     true},  // always compiled (BalancedTruncation.cpp in core sources)
        {"zpetc",                   true},  // always compiled (ZeroPhaseTrackingFilter.cpp in core sources)
        {"gap_metric",              true},  // always compiled (GapMetric.cpp in core sources)
        {"lpv_system_id",           true},  // always compiled (LPVSystemID.cpp in core sources)
        {"gain_scheduled_ctrl",     true},  // always compiled (header-only GainScheduledController.h)
        {"auto_gain_scheduler",     true},  // always compiled (header-only AutoGainScheduler.h)
        {"linear_model_cluster",    true},  // always compiled (header-only LinearModelCluster.h)
        {"nonlinear_mpc",           true},  // always compiled (NonlinearMPC.cpp in core sources)
        {"adaptive_smith",          true},  // always compiled (AdaptiveSmithPredictor.cpp in core sources)
        {"auto_tuner",              true},  // always compiled (header-only AutoTuner.h)
        {"anti_windup_wrapper",     true},  // always compiled (header-only AntiWindupWrapper.h)
        {"tube_mpc",                true},  // always compiled (TubeMPC.cpp in core sources)
        {"particle_filter",         true},  // always compiled (ParticleFilter.cpp in core sources)
        {"ilc",                     true},  // always compiled (IterativeLearningControl.cpp)
        {"sindy",                   true},  // always compiled (SINDy.cpp)
        {"koopman_edmd",            true},  // always compiled (KoopmanEDMD.cpp)
        {"l1_adaptive",             true},  // always compiled (L1AdaptiveController.cpp)
        {"cbf_safety_filter",       true},  // always compiled (CBFSafetyFilter.cpp)
        {"gaussian_process",        true},  // always compiled (GaussianProcess.cpp)
        {"echo_state_network",      true},  // always compiled (EchoStateNetwork.cpp)
        {"neural_pid",              true},  // always compiled (NeuralPID.cpp)
        {"cem_mpc",                 true},  // always compiled (CEMController.cpp)
    };
}

} // namespace ctrl
