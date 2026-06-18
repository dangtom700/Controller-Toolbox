# Controller Categorisation

Re-organises the master controller list along four practical axes:
**Implementation Approach**, **Plant Knowledge**, **SISO / MIMO**, and **Toolbox Tier**.

---

## Axis 1 - By Implementation Approach

### A. Closed-Form / Recurrence-Relation (real-time, no solver)
These controllers compute u[k] with O(n) arithmetic per step.

| Controller | Notes |
|---|---|
| On/Off (Bang-Bang) | 1-bit comparator |
| P, PI, PD, PID (all variants) | ISA parallel/series/ideal |
| Adaptive PID | online gain update only |
| Fractional-Order PID (FOPID) | IIR approximation of s^alpha |
| 2-DOF PID | two separate transfer functions |
| Lead, Lag, Lead-Lag Compensators | biquad filter |
| Phase-Lead / Phase-Lag Networks | same as lead-lag |
| Feedforward Control | static gain or FIR |
| Cascade Control | nested PID loops |
| Ratio / Override / Selector / Split-Range | signal routing + PID |
| Smith Predictor + PID | augmented PID with delay model |
| IMC / IMC-PID | stable plant inverse filter |
| Deadbeat Control | finite-settling FIR |
| Dahlin's Algorithm | z-domain direct design |
| Delta-Operator Control | near-continuous discrete design |
| Ragazzini-Franklin Direct Digital Design | z-plane pole-zero placement |
| Pole Placement / Full State Feedback | gain vector applied per step |
| Observer-Based State Feedback (Luenberger) | state estimator + pole placement |
| LQR (discrete, infinite horizon) | precomputed gain K |
| LQI (LQR + integrator augmentation) | augmented state |
| LQT (tracking LQR) | reference feed-forward |
| Discrete Sliding Mode Control | sign function |
| Integral Sliding Mode | SMC + integral surface |
| Disturbance Observer Based (DOBC) | DOB filter + baseline PID/LQR |
| Active Disturbance Rejection (ADRC) | Extended State Observer + PD |
| Composite Nonlinear Feedback (CNF) | two-gain switching |
| Steady-State / Dynamic Decoupling + PI/PID | decoupling matrix pre-filter |

---

### B. Quadratic Programming / Optimisation (online solver required)
These controllers solve a finite-horizon optimisation problem at each sample.

| Controller | Solver Class |
|---|---|
| Linear MPC | convex QP |
| Dynamic Matrix Control (DMC) | unconstrained LS |
| Model Algorithmic Control (MAC) | unconstrained LS |
| Generalized Predictive Control (GPC) | unconstrained / constrained QP |
| Explicit MPC | offline multi-parametric QP -> PWA lookup |
| Nonlinear MPC (NMPC) | nonlinear NLP (e.g., IPOPT, CasADi) |
| Economic MPC | non-quadratic NLP |
| Robust MPC (min-max / tube) | robust QP / SOCP |
| Stochastic MPC | chance-constrained QP / SOCP |
| Distributed / Decentralized MPC | coupled QP / ADMM |
| Hybrid MPC (MLD) | mixed-integer QP (MIQP) |
| Adaptive MPC | QP + online parameter update |
| LQG / LQG-LTR | offline Riccati (DARE/CARE) + online KF |
| H2 Control | offline LMI / Riccati solve |
| Hinf Control (loop-shaping, mixed sensitivity) | offline gamma-iteration / LMI |
| mu-Synthesis | offline D-K iteration |
| QFT | offline frequency-domain loop shaping |
| LMI-Based Robust Control | offline SDP (e.g., CVXPY, MOSEK) |
| Minimum-Time / Bang-Bang Optimal | offline Pontryagin |

---

### C. Adaptive / Online Learning (recursive parameter update)
Controller structure is fixed, but parameters evolve online.

| Controller | Adaptation Mechanism |
|---|---|
| Model Reference Adaptive Control (MRAC) | MIT rule / Lyapunov gradient |
| Self-Tuning Regulator (STR) | RLS / EKF + indirect control |
| Gain Scheduling (classical) | lookup table vs. scheduling variable |
| L1 Adaptive Control | fast adaptation + low-pass filter |
| Iterative Learning Control (ILC) | episode-to-episode Q-filter update |
| Repetitive Control (RC) | internal model principle |
| Extremum Seeking Control (ESC) | gradient estimate via dither |
| Multiple-Model Adaptive (MMAC) | weighted bank of models |
| Adaptive Sliding Mode | SMC + parameter adaptation |
| Adaptive PID (online) | gradient / relay re-tuning |
| Model-Free Adaptive Control (MFAC) | pseudo-partial derivative |
| Iterative Feedback Tuning (IFT) | gradient on closed-loop experiment |
| Virtual Reference Feedback Tuning (VRFT) | one-shot open-loop data |
| Fictitious Reference Iterative Tuning (FRIT) | IFT variant |
| Unfalsified Control | set-based controller falsification |
| PID with auto-tuning (relay / Z-N) | relay test + rule table |

---

### D. Intelligent / Inference-Based (requires inference engine or trained model)

| Controller | Backend |
|---|---|
| Fuzzy Logic Control (Mamdani, T-S) | rule base + defuzzifier |
| Neuro-Fuzzy Control (ANFIS) | gradient-trained fuzzy net |
| Neural Network Control (offline trained) | forward pass only at runtime |
| Adaptive Neural Network Control | online backprop / EKF update |
| Reinforcement Learning (Q-learning, DDPG, SAC, PPO) | policy network inference |
| GA-Tuned Controllers | offline evolutionary tuning |
| PSO-Tuned Controllers | offline swarm optimisation |
| Ant Colony / Differential Evolution Tuning | offline meta-heuristic |
| Fuzzy-PID | fuzzy + recurrence relation |
| Neuro-PID | NN gain schedule + PID |
| Fuzzy Sliding Mode | fuzzy + SMC |
| Fuzzy-LQR | fuzzy weights + LQR |
| Reinforcement Learning (model-free) | policy network inference |

---

### E. Large-Scale / Multi-Agent (network communication required)

| Controller | Architecture |
|---|---|
| Decentralized PID | independent loops |
| Distributed MPC | coupled QP with neighbour exchange |
| Multi-Agent Consensus Control | graph Laplacian |
| Cooperative Control (formation, flocking) | virtual leader / potential field |
| Overlapping Decomposition | shared subsystem states |

---

## Axis 2 - By Plant Knowledge Requirement

| Knowledge Level | Controllers |
|---|---|
| **Full model (A,B,C,D)** | LQR, LQG, MPC, Hinf, H2, mu-Synthesis, Pole Placement, DARE-based |
| **FOPDT / reduced model** | IMC-PID, Smith Predictor, Z-N, Tyreus-Luyben, Cohen-Coon |
| **Frequency-response data** | QFT, Hinf loop-shaping, Lead-Lag design |
| **Step / impulse response data** | DMC, MAC, GPC (with FIR model) |
| **No model (model-free)** | ESC, MFAC, IFT, VRFT, RL, relay auto-tune, Bang-Bang |
| **Partial / online identified** | STR, MRAC, Adaptive MPC, MMAC |

---

## Axis 3 - SISO vs. MIMO Capability

| Scope | Controllers |
|---|---|
| **SISO only** | On/Off, P/PI/PD/PID (scalar), Lead-Lag, Smith Predictor, IMC-PID, Dahlin, Deadbeat, ESC |
| **SISO or MIMO (natural extension)** | LQR, LQG, MPC, Hinf, H2, SMC, MRAC, Pole Placement, Observer-based |
| **MIMO by design** | mu-Synthesis, LQG/LTR, Dynamic Decoupling, Multivariable PID, Distributed MPC, Consensus |

---

## Axis 4 - Discrete C++ Toolbox Implementation Tier

| Tier | Description | Controllers |
|---|---|---|
| **Tier 0 - Embedded** | Header-only templates, no Eigen, no virtual dispatch, `float`-safe | `BasicPID<Scalar>`, `BasicSMC<Scalar>` |
| **Tier 1 - Core** | Closed-form, real-time, no external libraries | `DiscretePID` (all variants), `DiscreteLQR`, LQI, Pole Placement, Observer, `DiscreteLeadLag`, `SmithPredictor`, `AdaptiveSmithPredictor`, Cascade, `FeedforwardController`, `DiscreteSMC`, `DiscreteADRC`, DOBC, `ExtremumSeeker`, `RepetitiveController`, `ComputationalDelayWrapper`, `CBFSafetyFilter` |
| **Tier 2 - Optimisation** | Requires embedded QP solver (built-in `GradientProjectionQP`) | `DiscreteMPC`, `GeneralizedPredictiveController` (GPC), `MovingHorizonEstimator` (MHE), `DiscreteLQG`, `TubeMPC`, `NonlinearMPC`, `ScenarioMPC`, `HybridMPC`, `DeePC` (ADMM) |
| **Tier 3 - Advanced Adaptive** | Requires RLS / EKF / gradient update or episode memory | `MRACController`, Adaptive GPC (RLS+setPlant), `IterativeLearningControl` (ILC), `L1AdaptiveController`, `GainScheduledController`, `AutoGainScheduler`, `RecursiveLeastSquares`, `RecursiveGreyBoxEstimator` |
| **Tier 4 - Intelligent / Probabilistic** | Requires inference engine, GP inference, or reservoir | `FuzzyPD`, `FuzzyPID`, `FuzzySupervisor` (Mamdani/TS), `GaussianProcess`, `EchoStateNetwork`, `NeuralPID`, `SIRParticleFilter`, `GPResidualModel`, `BayesianOptimizer` |
| **Tier 5 - Offline / External Tool** | Controller synthesised offline; gain matrix or model used online | `DiscreteHinf` (gamma-bisection), mu-Synthesis DK-iteration, `SINDy`, `KoopmanEDMD`, `GreyBoxEstimator`, `HybridModelTrainer`, `CEMController`, `DynaController`, Economic MPC |
| **Tier 6 - Runtime Diagnostics** | Observers / decorators that monitor or protect running controllers | `ControllerMonitor` (CUSUM+EWMA), `MismatchDetector` (on KF/MHE), `AntiWindupWrapper` |

---

## Toolbox Implementation Mapping

The files in this repository implement the highlighted **Tier 0-1** controllers plus
key **Tier 2**, **Tier 3**, **Tier 4**, **Tier 5**, and **Tier 6** controllers:

| File | Controllers / Components Covered | Tier |
|---|---|---|
| `BasicPID.h` | `BasicPID<Scalar>` - header-only embedded PID, no Eigen, `float`-safe | 0 |
| `BasicSMC.h` | `BasicSMC<Scalar>` - header-only embedded SMC, no Eigen, `float`-safe | 0 |
| `DiscretePID.h/.cpp` | P, PI, PD, PID, 2-DOF (via b_weight), DoM derivative, anti-windup | 1 |
| `DiscreteLQR.h/.cpp` | Discrete LQR (DARE), LQI (augment state externally), LQRAdapter, `makeLQRController()` | 1 |
| `DiscreteSMC.h/.cpp` | First-order SMC with saturation boundary layer | 1 |
| `DiscreteADRC.h/.cpp` | 2nd-order LADRC: ESO (3-state) + PD law; ADRC bandwidth parameterisation | 1 |
| `DiscreteLeadLag.h/.cpp` | Tustin-discretised lead / lag / lead-lag compensator | 1 |
| `SmithPredictor.h/.cpp` | Dead-time compensator (integer + fractional Pade delay) | 1 |
| `AdaptiveSmithPredictor.h/.cpp` | Online cross-correlation delay estimator + Smith Predictor | 1 |
| `RepetitiveController.h/.cpp` | Internal-model repetitive control with Q-filter | 1 |
| `FeedforwardController.h/.cpp` | Static and dynamic feedforward (reference model-based) | 1 |
| `FeedbackLinearisation.h/.cpp` | Exact FL - DriftFn+GainFn algebraic inversion; SISO relative degree 1 | 1 |
| `ExtremumSeeker.h/.cpp` | Perturbation-based ESC | 1 |
| `CBFSafetyFilter.h/.cpp` | Control Barrier Function 1-D analytical QP; wraps any IController | 1 |
| `ComputationalDelayWrapper.h` | One-sample actuator delay decorator; header-only | 1 |
| `DiscreteMPC.h/.cpp` | Linear MPC (condensed QP, box constraints on u and Deltau) | 2 |
| `DiscreteLQG.h/.cpp` | LQG = LQR + Kalman output-feedback | 2 |
| `GeneralizedPredictiveControl.h/.cpp` | GPC (CARIMA predictor + RLS adaptive update) | 2 |
| `TubeMPC.h/.cpp` | Tube MPC (nominal + tube; DARE-based K; constraint tightening) | 2 |
| `NonlinearMPC.h/.cpp` | Nonlinear MPC (RTI sequential QP; internal integrator model) | 2 |
| `ScenarioMPC.h/.cpp` | Scenario MPC (N_s Gaussian scenario rollouts; avg_W QP update) | 2 |
| `HybridMPC.h/.cpp` | Hybrid MPC (HybridModel prediction + online Ridge correction) | 2 |
| `DeePC.h/.cpp` | DeePC (Hankel matrix data-enabled MPC; ADMM solver) | 2 |
| `MovingHorizonEstimator.h/.cpp` | MHE via condensed QP; box + polytopic inequality constraints | 2 |
| `MRACController.h/.cpp` | MRAC - Lyapunov adaptation + sigma-modification + projection | 3 |
| `IterativeLearningControl.h/.cpp` | ILC - P-type, D-type, norm-optimal; episode Q-filter update | 3 |
| `L1AdaptiveController.h/.cpp` | L1 Adaptive - state predictor + LP-filtered adaptation law | 3 |
| `GainScheduledController.h/.cpp` | Gain-scheduled linear blend; bumpless transfer on bracket change | 3 |
| `AutoGainScheduler.h/.cpp` | Automated gain schedule design via `design_fn` callbacks | 3 |
| `RecursiveLeastSquares.h/.cpp` | Online ARX identification with forgetting factor | 3 |
| `RecursiveGreyBoxEstimator.h/.cpp` | Augmented-state UKF for online ODE parameter tracking (E2) | 3 |
| `FuzzyLogic.h/.cpp` | Mamdani/TS inference engine; `FuzzyPD`, `FuzzyPID`, `FuzzySupervisor` | 4 |
| `GaussianProcess.h/.cpp` | SE kernel GP; Cholesky inference; fixed-budget eviction | 4 |
| `EchoStateNetwork.h/.cpp` | Reservoir computing; spectral-radius-scaled W_res; ridge readout | 4 |
| `NeuralPID.h/.cpp` | 3->nh->3 network; softplus gains; online backprop | 4 |
| `SIRParticleFilter.h/.cpp` | Sequential Importance Resampling particle filter (Kitagawa 1996) | 4 |
| `GPResidualModel.h/.cpp` | GP model-plant mismatch correction; `predictWithUncertainty()` (E3) | 4 |
| `BayesianOptimizer.h` | GP surrogate + UCB/EI acquisition; header-only | 4 |
| `DiscreteHinf.h/.cpp` | H-infinity (gamma bisection), mixed sensitivity, mu-synthesis DK-iteration | 5 |
| `SINDy.h/.cpp` | Sparse Identification of Nonlinear Dynamics (STLS) | 5 |
| `KoopmanEDMD.h/.cpp` | Koopman EDMD; PolyDeg1/2+RBF dictionary -> `StateSpace` | 5 |
| `GreyBoxEstimator.h/.cpp` | Levenberg-Marquardt ODE parameter fitting (E1) | 5 |
| `HybridModel.h/.cpp` | Physical ODE + `DataFunc` correction; `predictPhys()` / `predict()` | 5 |
| `HybridModelTrainer.h/.cpp` | Offline Ridge/GP/ESN trainer for `HybridModel::DataFunc` (H4) | 5 |
| `CEMController.h/.cpp` | Cross-Entropy Method stochastic rollout MPC; warm-start mu | 5 |
| `DynaController.h/.cpp` | Dyna MBRL (Sutton 1991); SINDy error-dynamics fit; wraps any IController | 5 |
| `ControllerMonitor.h` | CUSUM + EWMA SPC charts; `IControllerObserver`; header-only | 6 |
| `MismatchDetector.h` | CUSUM on KF/MHE innovation norm; `enableMismatchDetection()`; header-only | 6 |
| `LinearisationHelper.h/.cpp` | Numerical Jacobians (jacobianX/U) + `lineariseAtPoint` (ZOH) | util |
| `BalancedTruncation.h/.cpp` | Model order reduction; Hinf error bound; `suggestOrder` | util |
| `ZeroPhaseTrackingFilter.h/.cpp` | ZPETC prefilter + `transmissionZeros` via generalised eigenvalue | util |
| `VectorFitting.h/.cpp` | Rational approximation of frequency-response data (used by `DiscreteHinf`) | util |
| `FOPDTIdentifier.h/.cpp` | FOPDT step-response identification (graphical + golden-section) | util |
| `SOPDTIdentifier.h/.cpp` | SOPDT identification + Rivera 1986 IMC-PID tuning | util |
| `SubspaceID.h/.cpp` | N4SID subspace identification; `suggestOrder` | util |
| `GradientProjectionQP.h` | Shared projected-gradient QP solver (MPC / GPC / MHE / TubeMPC backend) | util |
| `ControllerTuner.h/.cpp` | Relay auto-tune, FOPDT step-response, Bryson LQR weights, MPC horizon | util |
| `TunerSuite.h/.cpp` | Unified tuner dispatcher (8 strategies, soft warnings) | util |
| `ControllerStack.h/.cpp` | Supervisory, Additive, Weighted stacks - cascade, bumpless transfer | util |
| `PlantModel.h/.cpp` | `TransferFunction`, `StateSpace`, `DAESystem`, `tf2ss`, `c2d` (ZOH/Tustin), `dae_c2d`, `ssStep` | util |
