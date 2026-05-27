#pragma once

/**
 * @file ControllerToolbox.h
 * @brief Discrete-Time Controller Toolbox — umbrella include.
 *
 * Include path: the target must have `lib/` as an include root.
 * Usage: `#include "ControllerToolbox.h"`
 *
 * All controllers run at a fixed sample time Ts (discrete-time mode).
 * Plant input: TransferFunction or StateSpace (z-domain).
 *
 * **Quick-start:**
 * @code
 *   // --- Define plant ---
 *   ctrl::TransferFunction G({b0,b1}, {1,a1,a2}, Ts);   // TF form
 *   ctrl::StateSpace sys = ctrl::tf2ss(G);                // or convert TF → SS
 *   // ctrl::StateSpace sys(A, B, C, D, Ts);             // or direct SS
 *
 *   // --- Tune (optional) ---
 *   ctrl::RelayAutoTuner tuner(cfg, Ts);
 *   while (!tuner.isDone()) u = tuner.step(y);
 *   ctrl::PIDParams pp = tuner.computePIDParams();
 *
 *   // --- Instantiate controllers ---
 *   ctrl::DiscretePID    pid(pp, Ts);
 *   ctrl::DiscreteMPC    mpc(sys, mpc_p);
 *   ctrl::DiscreteLQR    lqr(sys, lqr_p);
 *   ctrl::ExtremumSeeker esc(esc_p, Ts);
 *   ctrl::SmithPredictor sp(std::make_shared<ctrl::DiscretePID>(pp, Ts), sys, d);
 *   ctrl::DiscreteLeadLag ll(ll_p, Ts);
 *   ctrl::DiscreteSMC    smc(smc_p, Ts);
 *   ctrl::DiscreteADRC   adrc(adrc_p, Ts);
 *   ctrl::DiscreteLQG    lqg(sys, lqr_p, Q_noise, R_noise);
 *
 *   // --- Build a stack (optional) ---
 *   ctrl::ControllerStack stack(ctrl::StackMode::Supervisory, Ts);
 *   stack.addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "PID");
 *
 *   // --- Simulation loop ---
 *   Eigen::VectorXd x = Eigen::VectorXd::Zero(sys.stateSize());
 *   for (int k = 0; k < N; ++k) {
 *       double e = ref - y;
 *       double u = pid.compute(e);              // or stack.compute(e)
 *       Eigen::VectorXd uv(1); uv << u;
 *       y = ctrl::ssStep(sys, x, uv)(0);
 *   }
 * @endcode
 *
 * @note Requires Eigen 3.4+ and C++20.
 */

#include "IController.h"         ///< Abstract controller interface.
#include "IControllerObserver.h" ///< Observer hook for non-intrusive controller telemetry.
#include "PlantModel.h"       ///< TransferFunction, StateSpace, tf2ss, ssStep, c2d.
#include "DiscretePID.h"      ///< PID — backward-Euler, derivative filter, anti-windup.
#include "DiscreteMPC.h"      ///< MPC — condensed receding-horizon QP.
#include "DiscreteLQR.h"      ///< LQR — DARE optimal gain, LQRAdapter.
#include "ExtremumSeeker.h"   ///< ESC — perturbation-based extremum seeking.
#include "SmithPredictor.h"   ///< Smith predictor — dead-time compensation.
#include "DiscreteLeadLag.h"  ///< Lead / Lag / Lead-Lag — Tustin biquad compensator.
#include "DiscreteSMC.h"      ///< SMC — first-order sliding mode, boundary layer.
#include "DiscreteADRC.h"     ///< ADRC — ESO + PD, active disturbance rejection.
#include "KalmanFilter.h"     ///< Kalman filter — optimal linear state estimator.
#include "DiscreteLQG.h"      ///< LQG — LQR + Kalman (output-feedback optimal).
#include "ControllerTraits.h" ///< Compile-time controller ↔ tuner compatibility metadata.
#include "ControllerTuner.h"  ///< RelayAutoTuner, StepResponseTuner, LQRWeightTuner, MPCHorizonTuner,
                              ///<   ZieglerNicholsTuner, CohenCoonTuner, LoopShapingTuner, KalmanWeightTuner.
#include "ControllerStack.h"  ///< Supervisory / Additive / Weighted controller stacks.
#include "TunerSuite.h"       ///< All tuning methods (runtime soft-warning dispatch, Nelder-Mead).
#include "MetricsAnalyzer.h"  ///< Time-domain step-response metric extraction.
#include "SystemAnalysis.h"   ///< Frequency-domain and stability analysis.
// #include "hal/HAL.h"          ///< ISensor, IActuator, SimPlant, SimSensor, SimActuator, SafeSensor, StdTimer.
// #include "AtomicParamBuffer.h" ///< Lock-free double-buffer for RT parameter updates.

#include "RecursiveLeastSquares.h"        ///< RLS — online ARX system identification with forgetting factor.
#include "RepetitiveController.h"         ///< RC  — plug-in periodic disturbance/reference cancellation.
#include "GeneralizedPredictiveControl.h" ///< GPC — velocity-form MPC with reference trajectory (CARIMA).
#include "GradientProjectionQP.h"         ///< Shared gradient-projection solver used by MPC and GPC.

// Optional modules — controlled by CTRL_ENABLE_* cmake options (all ON by default).
// When building without CMake, define CTRL_HAS_* manually to enable the relevant headers,
// or define CTRL_DISABLE_* (legacy) to suppress them.

#if defined(CTRL_HAS_ADVANCED_KALMAN) || (!defined(CTRL_DISABLE_ADVANCED_KALMAN))
#include "ExtendedKalmanFilter.h"  ///< EKF — nonlinear state estimation (analytical/numerical Jacobians).
#include "UnscentedKalmanFilter.h" ///< UKF — sigma-point nonlinear estimation (no Jacobians).
#endif

#if defined(CTRL_HAS_SUBSPACE) || (!defined(CTRL_DISABLE_SUBSPACE))
#include "SubspaceID.h" ///< N4SID — batch subspace state-space identification (MOESP).
#endif

#if defined(CTRL_HAS_FUZZY) || (!defined(CTRL_DISABLE_FUZZY))
#include "FuzzyLogic.h" ///< Fuzzy — Mamdani/TS inference, FuzzyPD, FuzzyPID, FuzzySupervisor.
#endif

// H∞ — honour both legacy CTRL_DISABLE_HINF and new CTRL_HAS_HINF / CTRL_DISABLE_HINF2.
#if (defined(CTRL_HAS_HINF) || !defined(CTRL_DISABLE_HINF))
#include "DiscreteHinf.h" ///< H∞ — DGKF 2-Riccati synthesis, Mixed-Sensitivity S/KS/T design.
#endif

#if defined(CTRL_HAS_FUNCTION_APPROX) || (!defined(CTRL_DISABLE_FUNCTION_APPROX))
#include "FunctionApproximator.h" ///< Taylor (polynomial) + Padé (rational) data-driven approximation.
                                  ///< Includes padeDelayFilter() for fractional dead-time SmithPredictor.
#endif
