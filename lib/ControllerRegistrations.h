#pragma once
#include "ControllerRegistry.h"

/**
 * @file ControllerRegistrations.h
 * @brief Centralized feature registrations for controllers added before M2.
 *
 * New controllers (added after Part 33) should self-register by placing
 * `CTRL_REGISTER_FEATURE(name)` at the bottom of their own header instead
 * of adding an entry here.  This file exists only to migrate the existing
 * hand-maintained `Features.h` map to the self-registration system.
 *
 * Included automatically by `ControllerToolbox.h`.  Do NOT include directly.
 */

// --- Optional modules (reflect build-time CTRL_HAS_* flags) ----------------
#if defined(CTRL_HAS_HINF)
CTRL_REGISTER_FEATURE(hinf)
#endif
#if defined(CTRL_HAS_SUBSPACE)
CTRL_REGISTER_FEATURE(subspace)
#endif
#if defined(CTRL_HAS_FUZZY)
CTRL_REGISTER_FEATURE(fuzzy)
#endif
#if defined(CTRL_HAS_FUNCTION_APPROX)
CTRL_REGISTER_FEATURE(function_approx)
#endif
#if defined(CTRL_HAS_ADVANCED_KALMAN)
CTRL_REGISTER_FEATURE(advanced_kalman)
#endif

// --- Core / always-compiled algorithms -------------------------------------
CTRL_REGISTER_FEATURE(pid)
CTRL_REGISTER_FEATURE(mpc)
CTRL_REGISTER_FEATURE(lqr)
CTRL_REGISTER_FEATURE(lqg)
CTRL_REGISTER_FEATURE(kalman)
CTRL_REGISTER_FEATURE(esc)
CTRL_REGISTER_FEATURE(smith_predictor)
CTRL_REGISTER_FEATURE(lead_lag)
CTRL_REGISTER_FEATURE(smc)
CTRL_REGISTER_FEATURE(adrc)
CTRL_REGISTER_FEATURE(controller_stack)
CTRL_REGISTER_FEATURE(sopdt_identifier)
CTRL_REGISTER_FEATURE(mhe)
CTRL_REGISTER_FEATURE(linearisation_helper)
CTRL_REGISTER_FEATURE(feedback_linearisation)
CTRL_REGISTER_FEATURE(mrac)
CTRL_REGISTER_FEATURE(balanced_truncation)
CTRL_REGISTER_FEATURE(zpetc)
CTRL_REGISTER_FEATURE(gap_metric)
CTRL_REGISTER_FEATURE(lpv_system_id)
CTRL_REGISTER_FEATURE(gain_scheduled_ctrl)
CTRL_REGISTER_FEATURE(auto_gain_scheduler)
CTRL_REGISTER_FEATURE(linear_model_cluster)
CTRL_REGISTER_FEATURE(nonlinear_mpc)
CTRL_REGISTER_FEATURE(adaptive_smith)
CTRL_REGISTER_FEATURE(auto_tuner)
CTRL_REGISTER_FEATURE(anti_windup_wrapper)
CTRL_REGISTER_FEATURE(tube_mpc)
CTRL_REGISTER_FEATURE(particle_filter)
CTRL_REGISTER_FEATURE(gpc)
CTRL_REGISTER_FEATURE(repetitive)

// --- Part 31 ML/DD algorithms ----------------------------------------------
CTRL_REGISTER_FEATURE(ilc)
CTRL_REGISTER_FEATURE(sindy)
CTRL_REGISTER_FEATURE(koopman_edmd)
CTRL_REGISTER_FEATURE(l1_adaptive)
CTRL_REGISTER_FEATURE(cbf_safety_filter)
CTRL_REGISTER_FEATURE(gaussian_process)
CTRL_REGISTER_FEATURE(echo_state_network)
CTRL_REGISTER_FEATURE(neural_pid)
CTRL_REGISTER_FEATURE(cem_mpc)
