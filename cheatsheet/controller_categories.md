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
| H₂ Control | offline LMI / Riccati solve |
| Hinf Control (loop-shaping, mixed sensitivity) | offline γ-iteration / LMI |
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
| **Full model (A,B,C,D)** | LQR, LQG, MPC, Hinf, H₂, mu-Synthesis, Pole Placement, DARE-based |
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
| **SISO or MIMO (natural extension)** | LQR, LQG, MPC, Hinf, H₂, SMC, MRAC, Pole Placement, Observer-based |
| **MIMO by design** | mu-Synthesis, LQG/LTR, Dynamic Decoupling, Multivariable PID, Distributed MPC, Consensus |

---

## Axis 4 - Discrete C++ Toolbox Implementation Tier

| Tier | Description | Controllers |
|---|---|---|
| **Tier 1 - Core** | Closed-form, real-time, no external libraries | PID (all variants), LQR, LQI, Pole Placement, Observer, Deadbeat, Lead-Lag, Smith Predictor, Cascade, Feedforward, SMC (basic), ADRC, DOBC, ESC |
| **Tier 2 - Optimisation** | Requires embedded QP solver (e.g., OSQP, qpOASES) | Linear MPC, GPC, DMC, Robust MPC, Explicit MPC (offline), LQG |
| **Tier 3 - Advanced Adaptive** | Requires RLS / EKF / gradient update | MRAC, STR, Adaptive MPC, ILC, L1 Adaptive, MFAC |
| **Tier 4 - Intelligent** | Requires inference engine (ONNX, TensorFlow Lite, fuzzylite) | Fuzzy PID, NN Control, ANFIS, RL Policy |
| **Tier 5 - Offline / External Tool** | Controller synthesised offline (MATLAB, CVXPY); only the gain matrix runs online | Hinf, H₂, mu-Synthesis, LMI-Based, NMPC, Economic MPC |

---

## Toolbox Implementation Mapping

The files in this repository implement the highlighted **Tier 1** controllers plus
key **Tier 2** controllers:

| File | Controllers Covered |
|---|---|
| `DiscretePID.h/.cpp` | P, PI, PD, PID, 2-DOF (via Kd=0/Ki=0), anti-windup |
| `DiscreteLQR.h/.cpp` | Discrete LQR (DARE), LQI (augment state externally), LQRAdapter |
| `DiscreteMPC.h/.cpp` | Linear MPC (condensed QP, unconstrained + box constraints) |
| `ExtremumSeeker.h/.cpp` | Perturbation-based ESC (Tier 1 adaptive) |
| `ControllerTuner.h/.cpp` | Relay auto-tune, Step-response FOPDT, Bryson LQR weights, MPC horizon |
| `ControllerStack.h/.cpp` | Supervisory, Additive, Weighted stacks (gain scheduling architecture) |
| `PlantModel.h/.cpp` | TransferFunction, StateSpace, tf2ss, ssStep |
