#pragma once

/**
 * @file ControllerToolbox.h
 * @brief Discrete-Time Controller Toolbox - umbrella include.
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
 *   ctrl::StateSpace sys = ctrl::tf2ss(G);                // or convert TF -> SS
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
#include "ControllerRegistry.h"  ///< Self-registration registry + CTRL_REGISTER_FEATURE macro (M2).
#include "Features.h"            ///< ctrl::features() - runtime optional-module discovery.
#include "PlantModel.h"       ///< TransferFunction, StateSpace, tf2ss, ssStep, c2d.
#include "DiscretePID.h"      ///< PID - backward-Euler, derivative filter, anti-windup.
#include "DiscreteMPC.h"      ///< MPC - condensed receding-horizon QP.
#include "DiscreteLQR.h"      ///< LQR - DARE optimal gain, LQRAdapter.
#include "ExtremumSeeker.h"   ///< ESC - perturbation-based extremum seeking.
#include "SmithPredictor.h"   ///< Smith predictor - dead-time compensation.
#include "DiscreteLeadLag.h"  ///< Lead / Lag / Lead-Lag - Tustin biquad compensator.
#include "DiscreteSMC.h"      ///< SMC - first-order sliding mode, boundary layer.
#include "DiscreteADRC.h"     ///< ADRC - ESO + PD, active disturbance rejection.
#include "KalmanFilter.h"     ///< Kalman filter - optimal linear state estimator.
#include "DiscreteLQG.h"      ///< LQG - LQR + Kalman (output-feedback optimal).
#include "ControllerTraits.h" ///< Compile-time controller \leftrightarrow tuner compatibility metadata.
#include "ControllerTuner.h"  ///< RelayAutoTuner, StepResponseTuner, LQRWeightTuner, MPCHorizonTuner,
                              ///<   ZieglerNicholsTuner, CohenCoonTuner, LoopShapingTuner, KalmanWeightTuner.
#include "ControllerStack.h"  ///< Supervisory / Additive / Weighted controller stacks.
#include "TunerSuite.h"       ///< All tuning methods (runtime soft-warning dispatch, Nelder-Mead).
#include "MetricsAnalyzer.h"  ///< Time-domain step-response metric extraction.
#include "SystemAnalysis.h"   ///< Frequency-domain and stability analysis.
// #include "hal/HAL.h"          ///< ISensor, IActuator, SimPlant, SimSensor, SimActuator, SafeSensor, StdTimer.
// #include "AtomicParamBuffer.h" ///< Lock-free double-buffer for RT parameter updates.

#include "RecursiveLeastSquares.h"        ///< RLS - online ARX system identification with forgetting factor.
#include "SelfTuningRegulator.h"          ///< SelfTuningRegulator - RLS-driven minimum-variance/pole-placement STR (Phase 3 OC1).
#include "MLEIdentifier.h"                ///< MLEIdentifier - batch MLE/MAP ARX identification, Gaussian/Laplace noise (Phase 3 SI1).
#include "SetMembershipEstimator.h"       ///< SetMembershipEstimator - bounded-error ellipsoidal state estimation (Phase 3 EF2).
#include "RepetitiveController.h"         ///< RC  - plug-in periodic disturbance/reference cancellation.
#include "GeneralizedPredictiveControl.h" ///< GPC - velocity-form MPC with reference trajectory (CARIMA).
#include "GradientProjectionQP.h"         ///< Shared gradient-projection solver used by MPC and GPC.
#include "FOPDTIdentifier.h"              ///< FOPDT - step-response identification (K, tau, theta) + IMC-PID tuning.
#include "SOPDTIdentifier.h"              ///< SOPDT - second-order plus dead-time identification (K, tau1, tau2, theta) + IMC-PID tuning.
#include "FeedforwardController.h"        ///< Feedforward - G_ff(z)*r, combinable with feedback via ControllerStack.
#include "MovingHorizonEstimator.h"       ///< MHE - moving horizon state estimator (condensed QP, box constraints on process noise).
#include "LinearisationHelper.h"         ///< jacobianX/U, lineariseAtPoint - numerical Jacobians and ZOH linearisation at operating point.
#include "FeedbackLinearisation.h"       ///< FeedbackLinearisationController - exact FL for affine-in-control SISO systems (relative degree 1).
#include "BacksteppingController.h"      ///< BacksteppingController - recursive Lyapunov design for N-stage strict-feedback systems (Phase 3 NC1).
#include "PassivityBasedController.h"    ///< PassivityBasedController - PD+ energy-shaping/damping-injection regulation for Euler-Lagrange systems (Phase 3 NC2).
#include "CLFController.h"               ///< CLFController - CLF synthesis via Sontag's universal formula (Phase 3 NC4).
#include "HammersteinWienerIdentifier.h" ///< HammersteinWienerIdentifier - Hammerstein/Wiener structured nonlinear ID (Phase 3 SI5).
#include "MRACController.h"              ///< MRAC - Lyapunov-based MRAC with sigma-modification and parameter projection.
#include "BalancedTruncation.h"          ///< balancedTruncate, suggestOrder - H-infinity-bounded model order reduction.
#include "ZeroPhaseTrackingFilter.h"     ///< designZPETC, transmissionZeros - causal zero-phase feedforward prefilter.
#include "GainScheduledController.h"    ///< GainScheduledController - p-scheduled IController wrapper (LinearBlend / NearestNeighbor).
#include "GapMetric.h"                  ///< nuGap, nuGapMatrix, chordalDist, freqResponseGrid - nu-gap metric for gain scheduling.
#include "LinearModelCluster.h"         ///< clusterByGap, suggestGapThreshold, ClusterResult - nu-gap agglomerative clustering.
#include "LPVSystemID.h"                ///< identifyLPV, identifyLPVFromIO, LPVModel - polynomial LPV system identification.
#include "AutoGainScheduler.h"          ///< buildAutoGainScheduler, findEquilibrium, OperatingPoint - automated gain-scheduling pipeline.
#include "NonlinearMPC.h"               ///< NonlinearMPC - RTI-based NMPC with time-varying condensed QP (Diehl 2005).
#include "AdaptiveSmithPredictor.h"     ///< AdaptiveSmithPredictor - SmithPredictor with online cross-correlation delay estimation.
#include "AutoTuner.h"                  ///< AutoTuner - CMA-ES black-box optimizer for controller parameter tuning.
#include "AntiWindupWrapper.h"          ///< AntiWindupWrapper - generic anti-windup decorator (conditioning technique, Hanus 1987).
#include "TubeMPC.h"                    ///< TubeMPC - robust MPC with mRPI tube for bounded additive disturbances (Mayne 2005).
#include "ParticleFilter.h"             ///< ParticleFilter - SIR particle filter for nonlinear/non-Gaussian state estimation.
#include "IterativeLearningControl.h"   ///< ILC  - P-type, D-type and norm-optimal iterative learning control (Bristow 2006).
#include "SINDy.h"                      ///< SINDy - sparse identification of nonlinear dynamics (Brunton 2016); STLS regression.
#include "KoopmanEDMD.h"                ///< Koopman/EDMD - Extended Dynamic Mode Decomposition; linear lifting for nonlinear MPC/LQR.
#include "L1AdaptiveController.h"       ///< L1 Adaptive - bounded-transient adaptive control with LP filter (Hovakimyan 2010).
#include "CBFSafetyFilter.h"            ///< CBF - Control Barrier Function safety filter; 1-QP wrapper for any IController (Ames 2017).
#include "GaussianProcess.h"            ///< GP - Gaussian Process Regression; SE kernel, Cholesky inference, fixed-budget online.
#include "EchoStateNetwork.h"           ///< ESN - Echo State Network; random reservoir, ridge-regression readout (Jaeger 2001).
#include "NeuralPID.h"                  ///< NeuralPID - online neural network adapts Kp/Ki/Kd via backprop through linearised plant.
#include "NeuralNetworkController.h"    ///< NeuralNetworkController - generic feedforward NN, fixed forward pass (Phase 3 ML1).
#include "NNAdaptiveController.h"       ///< NNAdaptiveController - Lyapunov-stable online output-weight adaptation over ML1 (Phase 3 ML2).
#include "NonlinearIMC.h"               ///< NonlinearIMC - nonlinear Internal Model Control, model-in-the-loop mismatch feedback (Phase 3 NC3).
#include "NARMAXIdentifier.h"           ///< NARMAXIdentifier - polynomial NARMAX ID via orthogonal forward regression (Phase 3 SI4).
#include "CEMController.h"              ///< CEM-MPC - Cross-Entropy Method MPC; derivative-free stochastic rollout optimisation.
#include "DeePC.h"                      ///< DeePC - Data-Enabled Predictive Control; Hankel + ADMM, no system ID needed (Coulson 2019).
#include "DynaController.h"             ///< Dyna - model-based RL; SINDy error-dynamics fit + synthetic rollout planning (Sutton 1991).
#include "ScenarioMPC.h"                ///< ScenarioMPC - stochastic MPC; N_s noise-trajectory average cost QP (Calafiore & Campi 2006).
#include "BayesianOptimizer.h"          ///< BayesianOptimizer - GP surrogate + UCB/EI acquisition for expensive controller tuning (Srinivas 2010).
#include "GeneticAlgorithm.h"           ///< GeneticAlgorithm - real-valued GA: BLX-alpha crossover, tournament selection, elitism (Storn & Price 1997).
#include "ParticleSwarmOptimizer.h"     ///< ParticleSwarmOptimizer - Clerc-Kennedy PSO with velocity clamping (Clerc & Kennedy 2002).
#include "DifferentialEvolution.h"      ///< DifferentialEvolution - DE/rand/1/bin with boundary reflection (Storn & Price 1997).
#include "NelderMead.h"                 ///< NelderMead - reflect/expand/contract/shrink simplex search, no bounds/population needed (Phase 3 MO2).
#include "NSGA2.h"                      ///< NSGA2 - multi-objective (Pareto) evolutionary optimizer (Phase 3 MO1).
#include "ConstrainedTuning.h"          ///< tuneConstrained - exterior-penalty nonlinear-constraint wrapper for any CostFn optimizer (Phase 3 MO3).
#include "FaultClassifier.h"            ///< FaultClassifier - heuristic fault-type classifier over rolling residual statistics (Phase 3 DT4).
#include "FTCSupervisor.h"              ///< FTCSupervisor - reconfigures a ControllerStack's active entry on classified fault (Phase 3 DT4).
#include "ControllerMonitor.h"          ///< ControllerMonitor - CUSUM + EWMA SPC charts on live controller output or onState channels (M3/SPC).
#include "ComputationalDelayWrapper.h"  ///< ComputationalDelayWrapper - one-sample actuator delay decorator for realistic digital loop simulation (G3).
#include "GreyBoxEstimator.h"           ///< GreyBoxEstimator - nonlinear param estimation via Levenberg-Marquardt for user-supplied ODE f(x,u,p) (E1).
#include "GPResidualModel.h"            ///< GPResidualModel - learn model-plant mismatch epsilon=y_true-y_model as GP; risk-aware MPC correction (E3).
#include "HybridModel.h"               ///< HybridModel - plant combining physical ODE (RK4) + data-driven state correction f_data(x,u) (H1).
#include "HybridMPC.h"                 ///< HybridMPC - NonlinearMPC variant using HybridModel; online ridge-regression data update every N steps (H2).
#include "GPMPC.h"                     ///< GPMPC - GP-uncertainty-aware input-bound tightening for NonlinearMPC (ML3).
#include "HybridModelTrainer.h"        ///< HybridModelTrainer - off-line trainer for f_data: Ridge / GP marginal / ESN cross-validation (H4).
#include "VectorFitting.h"             ///< VectorFitting - SK iterative rational magnitude fitting; used by DiscreteHinf::solveMuSyn (T3 full DK-iteration).
#include "BasicPID.h"                  ///< BasicPID<Scalar> - header-only template PID for embedded/float targets; no virtual dispatch, no Eigen (M4).
#include "BasicSMC.h"                  ///< BasicSMC<Scalar> - header-only template SMC for embedded/float targets; no virtual dispatch, no Eigen (M4).
#include "MismatchDetector.h"          ///< MismatchDetector - CUSUM on KF/MHE innovation for real-time model-plant mismatch detection (D1).
#include "RobustnessAnalysis.h"        ///< RobustnessAnalysis - Monte-Carlo closed-loop robustness: spawn perturbed plants, aggregate stability/margin/sensitivity stats (Robustness Phase 1).
#include "MuAnalysis.h"                ///< MuAnalysis - structured singular value (mu) D-scaling upper bound, peakMu, robustStabilityRadius (Robustness Phase 3).
#include "LFTSystem.h"                 ///< LFTSystem - general multi-block LFT/Delta channel-gather for mu-analysis (Phase 3 RC1).
#include "WorstCaseSearch.h"           ///< WorstCaseSearch - CMA-ES worst-case parameter search over plant uncertainty (Robustness Phase 4).
#include "LyapunovRobustness.h"        ///< LyapunovRobustness - common quadratic Lyapunov function for polytopic uncertainty (Robustness Phase 5).
#include "FreqDomainIdentifier.h"      ///< FreqDomainIdentifier - Levy's method frequency-domain system identification (Phase 4 Iteration 2).
#include "ResonantController.h"       ///< ResonantController - single-harmonic internal-model corrector; composes via ControllerStack(Additive).
#include "NotchFilter.h"              ///< NotchFilter - fixed-design biquad notch filter (Bristow-Johnson cookbook); no IController base.
#include "PhaseLockedLoop.h"          ///< PhaseLockedLoop - single-input SOGI-PLL phase/frequency estimator; no IController base.
#include "CorrelationID.h"            ///< CorrelationID - cross-correlation impulse-response identification (Phase 3 SI2).
#include "SKFit.h"                    ///< SKFit - Sanathanan-Koerner-reweighted complex-response fitting (Phase 3 FD1).
#include "ComplexVectorFit.h"        ///< ComplexVectorFit - complex-conjugate-pole Vector Fitting (Phase 3 FD2).
#include "ValueIterationSolver.h"    ///< ValueIterationSolver - grid-based dynamic programming / value iteration (Phase 4 OC2).

// Optional modules - controlled by CTRL_ENABLE_* cmake options (all ON by default).
// When building without CMake, define CTRL_HAS_* manually to enable the relevant headers,
// or define CTRL_DISABLE_* (legacy) to suppress them.

#if defined(CTRL_HAS_ADVANCED_KALMAN) || (!defined(CTRL_DISABLE_ADVANCED_KALMAN))
#include "ExtendedKalmanFilter.h"          ///< EKF - nonlinear state estimation (analytical/numerical Jacobians).
#include "UnscentedKalmanFilter.h"         ///< UKF - sigma-point nonlinear estimation (no Jacobians).
#include "RecursiveGreyBoxEstimator.h"     ///< RecursiveGreyBoxEstimator - online param tracking via augmented-state UKF (E2).
#endif

#if defined(CTRL_HAS_SUBSPACE) || (!defined(CTRL_DISABLE_SUBSPACE))
#include "SubspaceID.h" ///< N4SID - batch subspace state-space identification (MOESP).
#endif

#if defined(CTRL_HAS_FUZZY) || (!defined(CTRL_DISABLE_FUZZY))
#include "FuzzyLogic.h" ///< Fuzzy - Mamdani/TS inference, FuzzyPD, FuzzyPID, FuzzySupervisor.
#endif

// Hinf - honour both legacy CTRL_DISABLE_HINF and new CTRL_HAS_HINF / CTRL_DISABLE_HINF2.
#if (defined(CTRL_HAS_HINF) || !defined(CTRL_DISABLE_HINF))
#include "DiscreteHinf.h" ///< Hinf - DGKF 2-Riccati synthesis, Mixed-Sensitivity S/KS/T design.
#include "DiscreteH2.h"   ///< H2 - discrete LQG/H2 synthesis via cross-term elimination (Phase 4 Iteration 3).
#include "HinfFilter.h"   ///< HinfFilter - H-infinity-optimal state filter, the estimation dual of DiscreteHinf (Phase 3 EF1).
#endif

#if defined(CTRL_HAS_FUNCTION_APPROX) || (!defined(CTRL_DISABLE_FUNCTION_APPROX))
#include "FunctionApproximator.h" ///< Taylor (polynomial) + Pade (rational) data-driven approximation.
                                  ///< Includes padeDelayFilter() for fractional dead-time SmithPredictor.
#endif

// M2: Centralized registrations for all pre-M2 controllers.
// Must come AFTER all algorithm includes so the conditional CTRL_HAS_* flags are set.
#include "ControllerRegistrations.h"
