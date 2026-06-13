### 1. Classical Control (SISO & Frequency-Domain)
- **On/Off (Bang-Bang) Control**
- **P, PI, PD, PID** (and variants: Ideal, Parallel, Series/Interacting) - **implemented: `DiscretePID`** (DoM, 2-DOF, anti-windup)
- **Lead, Lag, Lead-Lag Compensators** - **implemented: `DiscreteLeadLag`**
- **Phase-Lead / Phase-Lag Networks**
- **Feedforward Control** (static or dynamic) - **implemented: `FeedforwardController`**
- **Cascade Control** - **implemented via `ControllerStack::Additive`** (see ex42-ex46)
- **Ratio Control**
- **Override / Selector Control**
- **Split-Range Control**
- **Smith Predictor** (for dominant time delays) - **implemented: `SmithPredictor`** (integer + fractional Pade)
- **Internal Model Control (IMC)**
- **IMC-PID** (PID tuning via IMC) - **implemented: `FOPDTIdentifier::imcTuning`, `SOPDTIdentifier::imcTuning`**

---

### 2. State-Space and Optimal Control
- **Linear Quadratic Regulator (LQR)** - infinite & finite horizon - **implemented: `DiscreteLQR`** (DARE, LQRAdapter)
- **Linear Quadratic Gaussian (LQG)** - LQR + Kalman filter - **implemented: `DiscreteLQG`**
- **LQG with Loop Transfer Recovery (LQG/LTR)**
- **Linear Quadratic Integral (LQI)** - LQR with integral action
- **Linear Quadratic Tracking (LQT)**
- **Minimum-Time Control** (Bang-Bang optimal)
- **Minimum-Energy Control**
- **Pole Placement / Full State Feedback**
- **Observer-Based State Feedback** (Luenberger observer)

---

### 3. Model Predictive Control (MPC) Family
- **Linear MPC** (Quadratic programming based) - **implemented: `DiscreteMPC`** (condensed QP, box constraints)
- **Dynamic Matrix Control (DMC)**
- **Model Algorithmic Control (MAC)**
- **Generalized Predictive Control (GPC)** - **implemented: `GeneralizedPredictiveController`** (CARIMA + RLS adaptive)
- **Explicit MPC** (pre-computed PWA control law)
- **Nonlinear MPC (NMPC)** - **implemented: `NonlinearMPC`** (RTI sequential quadratic programming, internal integrator)
- **Tube MPC** (robust MPC with constraint tightening) - **implemented: `TubeMPC`** (nominal + tube controller, DARE-based K)
- **Scenario MPC** (stochastic, sample-average approximation) - **implemented: `ScenarioMPC`** (N_s Gaussian scenario rollouts, Calafiore & Campi 2006)
- **CEM-MPC** (cross-entropy method, sampling-based NMPC) - **implemented: `CEMController`** (elite-sample stochastic rollout, warm-start mu)
- **Data-Enabled Predictive Control (DeePC)** - **implemented: `DeePC`** (Hankel matrix, ADMM solver, Coulson 2019)
- **Hybrid MPC with Data Correction** - **implemented: `HybridMPC`** (physical ODE + online Ridge correction, see `phase2_hybrid_modeling.md`)
- **Economic MPC** (non-quadratic cost)
- **Distributed / Decentralized MPC**
- **Hybrid MPC** (mixed logical dynamical systems)

---

### 4. Robust Control
- **Hinf (H-infinity) Control** (mixed sensitivity, loop-shaping) - **implemented: `DiscreteHinf`** (gamma-bisection, mixed sensitivity)
- **H2 Control**
- **mu-Synthesis** (structured singular value) - **implemented: `DiscreteHinf::solveMuSyn`** (DK-iteration with rational D-scaling)
- **Quantitative Feedback Theory (QFT)**
- **Sliding Mode Control (SMC)** - classical first-order - **implemented: `DiscreteSMC`** (saturation boundary layer); embedded: **`BasicSMC<Scalar>`** (header-only, no Eigen)
- **Integral Sliding Mode Control**
- **Higher-Order Sliding Mode** (Super-Twisting, Twisting, Prescribed-time)
- **Control Barrier Function Safety Filter** - **implemented: `CBFSafetyFilter`** (1-D analytical QP wrapping any IController, Ames 2017)
- **Kharitonov-Based Robust Design**
- **Lyapunov's Direct Method Redesign**
- **LMI-Based Robust Control** (Hinf, H2, pole clustering)

---

### 5. Adaptive Control
- **Model Reference Adaptive Control (MRAC)** - direct & indirect - **implemented: `MRACController`** (Lyapunov adaptation, sigma-modification, parameter projection)
- **Self-Tuning Regulator (STR)**
- **Gain Scheduling** (classical, parameter-dependent) - **implemented: `GainScheduledController`** (linear blend across brackets, bumpless transfer); automated design: **`AutoGainScheduler`**
- **Adaptive PID**
- **L1 Adaptive Control** - **implemented: `L1AdaptiveController`** (state predictor + LP-filtered adaptation, Hovakimyan 2010)
- **Iterative Learning Control (ILC)** - **implemented: `IterativeLearningControl`** (P-type, D-type, norm-optimal modes; episode Q-filter update)
- **Repetitive Control (RC)** - **implemented: `RepetitiveController`** (IMP Q-filter)
- **Extremum Seeking Control** (model-free adaptive) - **implemented: `ExtremumSeeker`**
- **Dyna Model-Based RL** - **implemented: `DynaController`** (Sutton 1991 Dyna MBRL; SINDy error-dynamics fit; wraps any IController)
- **Multiple-Model Adaptive Control (MMAC)**
- **Adaptive Sliding Mode Control**

---

### 6. Nonlinear Control
- **Feedback Linearization** (input-output, full-state) - **implemented: `FeedbackLinearisationController`** (SISO, relative degree 1, DriftFn+GainFn)
- **Backstepping** (integrator backstepping)
- **Passivity-Based Control (PBC)**
- **Interconnection and Damping Assignment (IDA-PBC)**
- **Lyapunov Redesign**
- **Variable Structure Control**
- **Gain-Scheduled Nonlinear Control**
- **Flatness-Based Control**
- **Trajectory Linearization Control**
- **Describing Function Based Design** (for limit cycles)

---

### 7. Intelligent and Soft-Computing Control
- **Fuzzy Logic Control** (Mamdani, Takagi-Sugeno) - **implemented: `FuzzyLogic.h`** (`FuzzySystem`, `FuzzyPD`, `FuzzyPID`, `FuzzySupervisor`)
- **Neuro-Fuzzy Control (ANFIS)**
- **Neural Network Control** (off-line trained, model-inverse)
- **NeuralPID** (online backprop-tuned gain network) - **implemented: `NeuralPID`** (3->nh->3 network, softplus gains, online backprop)
- **Echo State Network (Reservoir Computing)** - **implemented: `EchoStateNetwork`** (spectral-radius-scaled W_res, ridge-regression readout; used as `HybridModelTrainer` ESN backend)
- **Gaussian Process Regression** (probabilistic, calibrated uncertainty) - **implemented: `GaussianProcess`** (SE kernel, Cholesky inference, fixed-budget eviction)
- **SINDy** (Sparse Identification of Nonlinear Dynamics) - **implemented: `SINDy`** (STLS sparse regression, Poly/Trig library; `SINDyModel::stateFunc()`)
- **Koopman EDMD** (data-driven linearisation) - **implemented: `KoopmanEDMD`** (PolyDeg1/2+RBF dictionary; `fit()` -> `ctrl::StateSpace`)
- **Adaptive Neural Network Control** (online learning)
- **Reinforcement Learning Control** (Q-learning, DDPG, SAC, PPO)
- **Bayesian Optimization** (GP surrogate + UCB/EI acquisition) - **implemented: `BayesianOptimizer`** (header-only; shares `TunerResult`/`CostFn` with `AutoTuner`)
- **Genetic Algorithm (GA) Tuned Controllers**
- **Particle Swarm Optimization (PSO) Tuned Controllers**
- **Ant Colony / Differential Evolution Based Tuning**

---

### 8. Stochastic and Estimation-Centric Control
- **LQG** (already optimal + estimation) - **implemented: `DiscreteLQG`**
- **Kalman-Filter Based State Feedback** - **implemented: `KalmanFilter`, `ExtendedKalmanFilter`, `UnscentedKalmanFilter`**
- **Certainty-Equivalence Control** - **implemented pattern** (EKF/UKF/MHE state -> MPC/LQR, see ex50-ex53)
- **Moving Horizon Estimation (MHE)** - **implemented: `MovingHorizonEstimator`** (condensed QP dual of MPC; box constraints on x_0; inequality constraints via C_ineq/d_ineq)
- **Particle Filter / SIR Filter** - **implemented: `SIRParticleFilter`** (sequential importance resampling, Kitagawa 1996)
- **Risk-Sensitive Control (LEQG)**
- **Dual Control** (probing + regulating)
- **Stochastic Optimal Control** (HJB equation)

---

### 9. Decoupling and Multivariable Structures
- **Steady-State Decoupling + PI**
- **Dynamic Decoupling + PID**
- **Multivariable PID (Centralized)**
- **Relative Gain Array (RGA) Pairing** with decentralized PI
- **Decentralized PID with Detuning**
- **Multivariable IMC**
- **Multivariable LQR / LQG**
- **Hinf Loop-Shaping for MIMO**
- **Input-Output Linearization** (MIMO)

---

### 10. Hybrid / Mixed Control Architectures (mixtures)
- **Fuzzy-PID** (Fuzzy gain scheduling, Fuzzy-tuned PID) - **implemented: `FuzzyPID`** in `FuzzyLogic.h`
- **Neuro-PID**
- **Sliding Mode + PID** (SM-PID, reaching-law based)
- **Fuzzy Sliding Mode Control**
- **Fuzzy-LQR** (LQR weights tuned by fuzzy rules)
- **LQR + Integral Action** (LQI, LQG with integral augmentation)
- **MPC with Integral Action** (disturbance model / incremental formulation)
- **Cascade P-PI / PID-LQR** (inner PI, outer LQR)
- **Adaptive MPC** (online parameter estimation + MPC)
- **L1 Adaptive with LQR Baseline**
- **Gain-Scheduled LQR**
- **IMC-PID** (PID synthesized from IMC)
- **Smith Predictor + PID**
- **Fractional-Order PID (FOPID)**
- **Active Disturbance Rejection Control (ADRC)** - Extended State Observer + PD/PID - **implemented: `DiscreteADRC`**
- **Two-Degree-of-Freedom (2-DOF) PID** (separate servo/regulator tuning)
- **Composite Nonlinear Feedback (CNF) Control**
- **Disturbance Observer Based Control (DOBC) + PID/LQR**

---

### 11. Digital / Discrete-Time Specific
- **Deadbeat Control**
- **Dahlin's Algorithm**
- **Discrete-Time LQR / LQG**
- **Discrete Sliding Mode Control**
- **Delta-Operator Control**
- **Ragazzini-Franklin Design** (direct digital)
- **Computational Delay Simulation** - **implemented: `ComputationalDelayWrapper`** (one-sample actuator delay decorator wrapping any IController; see `embedded_and_realtime.md`)

---

### 12. Data-Driven and Model-Free Control
- **Iterative Feedback Tuning (IFT)**
- **Virtual Reference Feedback Tuning (VRFT)**
- **Fictitious Reference Iterative Tuning (FRIT)**
- **Model-Free Adaptive Control (MFAC)**
- **Extremum Seeking** (model-free)
- **Reinforcement Learning (model-free)**
- **Unfalsified Control**
- **PID with auto-tuning** (relay, Ziegler-Nichols) - **implemented: `ControllerTuner`** (relay auto-tune, FOPDT step-response, Bryson LQR, MPC horizon; unified: `TunerSuite`)
- **Online ARX Identification** - **implemented: `RecursiveLeastSquares`** (online ARX with forgetting factor; accessor `params()`, update order: `update(y, u)`)

---

### 13. Embedded and Real-Time Implementations
- **Embedded PID** (header-only, no Eigen, no virtual dispatch) - **implemented: `BasicPID<Scalar>`** (`float` or `double`; MCU-safe, anti-windup built-in; see `embedded_and_realtime.md`)
- **Embedded SMC** (header-only, no Eigen) - **implemented: `BasicSMC<Scalar>`** (saturation boundary layer; sign convention: `compute(r - y)`)

---

### 14. Runtime Monitoring and Diagnostics
- **Statistical Process Control** (CUSUM + EWMA on control output) - **implemented: `ControllerMonitor`** (attaches as `IControllerObserver`; DiscreteADRC emits ESO state, DiscreteSMC emits surface)
- **Model Mismatch Detection** (CUSUM on KF/MHE innovation) - **implemented: `MismatchDetector`** via `KalmanFilter::enableMismatchDetection()` and `MovingHorizonEstimator::enableMismatchDetection()`; see `mismatch_detection.md`

---

### 15. Plant Model Utilities
- **DAE System** (differential-algebraic equations) - **implemented: `DAESystem`** (`PlantModel.h`); consistent init via Newton-Raphson; `dae_c2d()` algebraic elimination -> `StateSpace`; EKF projection via `setAlgebraicConstraint()` (see `mismatch_detection.md`)
- **Hybrid Model** (physical ODE + data-driven correction) - **implemented: `HybridModel`** + **`HybridModelTrainer`** (Ridge/GP/ESN backends) + **`HybridMPC`** (see `phase2_hybrid_modeling.md`)

---

### 13. Large-Scale, Decentralized & Cooperative Control
- **Decentralized PID**
- **Distributed MPC**
- **Multi-Agent Consensus Control**
- **Cooperative Control (formation, flocking)**
- **Overlapping Decomposition Based Control**

---