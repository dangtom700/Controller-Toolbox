### 1. Classical Control (SISO & Frequency-Domain)
- **On/Off (Bang-Bang) Control**
- **P, PI, PD, PID** (and variants: Ideal, Parallel, Series/Interacting)
- **Lead, Lag, Lead-Lag Compensators**
- **Phase-Lead / Phase-Lag Networks**
- **Feedforward Control** (static or dynamic)
- **Cascade Control**
- **Ratio Control**
- **Override / Selector Control**
- **Split-Range Control**
- **Smith Predictor** (for dominant time delays)
- **Internal Model Control (IMC)**
- **IMC-PID** (PID tuning via IMC)

---

### 2. State-Space and Optimal Control
- **Linear Quadratic Regulator (LQR)** - infinite & finite horizon
- **Linear Quadratic Gaussian (LQG)** - LQR + Kalman filter
- **LQG with Loop Transfer Recovery (LQG/LTR)**
- **Linear Quadratic Integral (LQI)** - LQR with integral action
- **Linear Quadratic Tracking (LQT)**
- **Minimum-Time Control** (Bang-Bang optimal)
- **Minimum-Energy Control**
- **Pole Placement / Full State Feedback**
- **Observer-Based State Feedback** (Luenberger observer)

---

### 3. Model Predictive Control (MPC) Family
- **Linear MPC** (Quadratic programming based)
- **Dynamic Matrix Control (DMC)**
- **Model Algorithmic Control (MAC)**
- **Generalized Predictive Control (GPC)**
- **Explicit MPC** (pre-computed PWA control law)
- **Nonlinear MPC (NMPC)**
- **Economic MPC** (non-quadratic cost)
- **Robust MPC** (min-max, tube-based, constraint tightening)
- **Stochastic MPC** (chance constraints)
- **Distributed / Decentralized MPC**
- **Hybrid MPC** (mixed logical dynamical systems)

---

### 4. Robust Control
- **Hinf (H-infinity) Control** (mixed sensitivity, loop-shaping)
- **H₂ Control**
- **mu-Synthesis** (structured singular value)
- **Quantitative Feedback Theory (QFT)**
- **Sliding Mode Control (SMC)** - classical first-order
- **Integral Sliding Mode Control**
- **Higher-Order Sliding Mode** (Super-Twisting, Twisting, Prescribed-time)
- **Kharitonov-Based Robust Design**
- **Lyapunov's Direct Method Redesign**
- **LMI-Based Robust Control** (Hinf, H₂, pole clustering)

---

### 5. Adaptive Control
- **Model Reference Adaptive Control (MRAC)** - direct & indirect
- **Self-Tuning Regulator (STR)**
- **Gain Scheduling** (classical, parameter-dependent)
- **Adaptive PID**
- **L1 Adaptive Control**
- **Iterative Learning Control (ILC)**
- **Repetitive Control (RC)**
- **Extremum Seeking Control** (model-free adaptive)
- **Multiple-Model Adaptive Control (MMAC)**
- **Adaptive Sliding Mode Control**

---

### 6. Nonlinear Control
- **Feedback Linearization** (input-output, full-state)
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
- **Fuzzy Logic Control** (Mamdani, Takagi-Sugeno)
- **Neuro-Fuzzy Control (ANFIS)**
- **Neural Network Control** (off-line trained, model-inverse)
- **Adaptive Neural Network Control** (online learning)
- **Reinforcement Learning Control** (Q-learning, DDPG, SAC, PPO)
- **Genetic Algorithm (GA) Tuned Controllers**
- **Particle Swarm Optimization (PSO) Tuned Controllers**
- **Ant Colony / Differential Evolution Based Tuning**

---

### 8. Stochastic and Estimation-Centric Control
- **LQG** (already optimal + estimation)
- **Kalman-Filter Based State Feedback**
- **Certainty-Equivalence Control**
- **Risk-Sensitive Control (LEQG)**
- **Dual Control** (probing + regulating)
- **Stochastic Optimal Control** (HJB equation)
- **Particle Filter-Based Control**

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
- **Fuzzy-PID** (Fuzzy gain scheduling, Fuzzy-tuned PID)
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
- **Active Disturbance Rejection Control (ADRC)** - Extended State Observer + PD/PID
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

---

### 12. Data-Driven and Model-Free Control
- **Iterative Feedback Tuning (IFT)**
- **Virtual Reference Feedback Tuning (VRFT)**
- **Fictitious Reference Iterative Tuning (FRIT)**
- **Model-Free Adaptive Control (MFAC)**
- **Extremum Seeking** (model-free)
- **Reinforcement Learning (model-free)**
- **Unfalsified Control**
- **PID with auto-tuning** (relay, Ziegler-Nichols)

---

### 13. Large-Scale, Decentralized & Cooperative Control
- **Decentralized PID**
- **Distributed MPC**
- **Multi-Agent Consensus Control**
- **Cooperative Control (formation, flocking)**
- **Overlapping Decomposition Based Control**

---