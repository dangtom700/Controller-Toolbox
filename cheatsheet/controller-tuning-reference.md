# Controller Tuning - Parameter Guide

> **Tip:** In practice, combine strategies - use analytical rules to initialise
> parameters, then refine with numerical optimisation or data-driven iteration.

---

## 1. Analytical / Model-Based

### Pole Placement
**When to use:** You have an accurate state-space model and can specify desired
dynamic behaviour via pole locations.

**What you tune:** Closed-loop pole locations . State feedback gain K

| Parameter | Meaning & Effect |
|---|---|
| Pole locations (s-plane) | Determines natural frequency (omegaₙ) and damping ratio (ζ). Poles further left -> faster decay. Complex pairs -> oscillatory response. |
| Ackermann's formula / `place()` | Computes the gain matrix K so that (A - BK) has exactly the desired eigenvalues. |

---

### IMC (Internal Model Control)
**When to use:** You want a single, intuitive knob to control speed of response
with guaranteed stability.

**What you tune:** Filter time constant lambda

| Parameter | Meaning & Effect |
|---|---|
| lambda (filter time constant) | The sole tuning knob. Small lambda -> fast, aggressive response. Large lambda -> slow, robust response. Acts as the desired closed-loop time constant. |

---

### LQR (Linear Quadratic Regulator)
**When to use:** You have a linear state-space model and want an optimal
trade-off between state error and control effort.

**What you tune:** Q matrix (state weights) . R matrix (control weights)

| Parameter | Meaning & Effect |
|---|---|
| Q (state weighting matrix) | Diagonal entries Qᵢᵢ penalise deviations of state xᵢ. Higher Qᵢᵢ -> controller works harder to regulate xᵢ -> faster but more control effort. |
| R (control weighting matrix) | Diagonal entries Rⱼⱼ penalise control input uⱼ magnitude. Higher R -> less aggressive actuation, slower response. |
| Bryson's Rule initialisation | Set Qᵢᵢ = 1/(max xᵢ)^2, Rⱼⱼ = 1/(max uⱼ)^2 as a physically motivated starting point. |

---

### LQG (LQR + Kalman Filter)
**When to use:** You have process and measurement noise in addition to the LQR
scenario; you need optimal state estimation.

**What you tune:** Q . R (LQR) . Qf (process noise covariance) . Rf (measurement noise covariance)

| Parameter | Meaning & Effect |
|---|---|
| Qf (process noise covariance) | Reflects how much uncertainty/disturbance acts on the states. Larger Qf -> Kalman filter trusts measurements more, reacts faster to changes. |
| Rf (measurement noise covariance) | Reflects sensor noise level. Larger Rf -> filter trusts the model more, smooths out measurements (less responsive but quieter). |
| LQR Q, R | Same as LQR above; tuned separately from the Kalman filter via the separation principle. |

---

### Hinf / Robust Control
**When to use:** Plant has uncertainty (parametric or unstructured) and you need
guaranteed robustness over a range of conditions.

**What you tune:** Weighting functions WS, WT, Wu

| Parameter | Meaning & Effect |
|---|---|
| WS (sensitivity weight) | Shapes disturbance rejection. High magnitude at low frequencies -> good setpoint tracking and disturbance rejection. |
| WT (complementary sensitivity weight) | Limits noise amplification at high frequencies and ensures robust stability against unmodelled dynamics. |
| Wu (control effort weight) | Penalises control signal magnitude/bandwidth. Prevents overly aggressive actuation. |

---

### MPC (Model Predictive Control)
**When to use:** You have a model, hard constraints on inputs/states, and need
anticipatory control over a horizon.

**What you tune:** Np (prediction horizon) . Nc (control horizon) . Q (output weights) . R (input move weights)

| Parameter | Meaning & Effect |
|---|---|
| Np (prediction horizon) | How many steps ahead MPC optimises. Should cover the open-loop settling time. Too short -> poor performance; too long -> computational burden. |
| Nc (control horizon) | Number of free control moves computed. Typically Np/5 to Np/3. Smaller Nc -> smoother but less flexible control. |
| Q (output/state weighting) | Penalises tracking errors. Higher Q -> tighter setpoint tracking, more aggressive moves. |
| R (input move weighting / ρ) | Penalises changes in control input. Higher R (or scalar ρ) -> smooth, conservative actuation. |
| Terminal cost / constraints | Ensures closed-loop stability at the end of the horizon. Often set to the LQR cost-to-go. |

---

## 2. Heuristic / Classical (PID-focused)

### Ziegler-Nichols
**When to use:** Quick field tuning of PID with minimal model information;
step or ultimate-gain test available.

**What you tune:** Kp . Ti (integral time) . Td (derivative time)

| Parameter | Meaning & Effect |
|---|---|
| Ku (ultimate gain) | Gain at which the closed loop oscillates at constant amplitude. Used as the baseline for ZN rules. |
| Pu (ultimate period) | Period of the sustained oscillation at Ku. Together with Ku, ZN formulas derive Kp, Ti, Td. |
| Kp, Ti, Td (ZN output) | Proportional gain (Kp), integral time (Ti = 1/Ki), derivative time (Td). ZN rules tend to give aggressive settings; often de-tuned by 20-30 %. |

---

### Cohen-Coon
**When to use:** Process resembles a first-order plus dead-time (FOPDT) model;
more accurate than ZN for high dead-time processes.

**What you tune:** Kp . Ti . Td

| Parameter | Meaning & Effect |
|---|---|
| Process gain K | Steady-state output change per unit input change from step test. |
| Time constant τ | Speed of the natural process response. |
| Dead time θ | Transportation lag before process responds. Ratio θ/τ drives aggressiveness of Cohen-Coon formulas. |

---

### Lambda / IMC-PID
**When to use:** You want one tuning knob controlling the entire PID; especially
good for integrating or high dead-time processes.

**What you tune:** lambda (closed-loop time constant)

| Parameter | Meaning & Effect |
|---|---|
| lambda (desired closed-loop TC) | Directly sets speed of response. Rule of thumb: lambda >= θ (dead time). Larger lambda -> more robust, slower. Smaller lambda -> faster, less robust. |

---

### Åström-Hägglund Relay Auto-Tuning
**When to use:** Automated PID tuning in a running process; relay experiment is
safe and non-invasive.

**What you tune:** Relay amplitude d . Hysteresis epsilon

| Parameter | Meaning & Effect |
|---|---|
| Relay amplitude d | Size of the switching signal injected. Larger d -> cleaner identification but bigger process disturbance. |
| Hysteresis epsilon | Dead-band around zero to prevent chattering. Should match noise level. |
| AMIGO / modified ZN rules (output) | Post-identification formulas that give a better balance of performance and robustness than classic ZN. |

---

## 3. Frequency-Domain Shaping

### Loop Shaping (Classical)
**When to use:** SISO system; you want to specify gain/phase margins and
bandwidth graphically on a Bode plot.

**What you tune:** Lead/lag compensator parameters . Gain K . Crossover frequency omegac

| Parameter | Meaning & Effect |
|---|---|
| Gain crossover frequency omegac | Frequency where \|L(jomega)\| = 1 (0 dB). Determines closed-loop bandwidth; higher omegac -> faster response. |
| Phase margin (PM) | How far the phase is from -180^\circ at omegac. PM > 45^\circ typically ensures adequate damping. |
| Gain margin (GM) | How much gain can increase before instability. GM > 6 dB is a common minimum. |
| Lead compensator (zero/pole pair) | Adds phase at the crossover frequency; improves PM. Zero < pole (in magnitude). |
| Lag compensator (zero/pole pair) | Increases low-frequency gain to reduce steady-state error without destabilising the loop. |

---

### QFT (Quantitative Feedback Theory)
**When to use:** Plant has parametric uncertainty and you need to guarantee
performance bounds across all plant variations.

**What you tune:** Nominal loop L₀(jomega) . Controller C(s) parameters . Pre-filter F(s)

| Parameter | Meaning & Effect |
|---|---|
| Uncertainty templates | Frequency-domain regions covering all possible plant behaviours. Controller must push nominal loop above QFT bounds. |
| Performance bounds | Constraints on sensitivity and complementary sensitivity. Defined per frequency. |
| Stability bound | Minimum distance from the -1 point on the Nichols chart for worst-case plant. |

---

## 4. Numerical Optimisation-Based

### Genetic Algorithm (GA)
**When to use:** Complex, non-smooth cost landscape; mixed-integer problems
(e.g., choosing PID vs FOPID structure); no gradient available.

**What you tune:** Population size . Crossover rate . Mutation rate . Controller parameters

| Parameter | Meaning & Effect |
|---|---|
| Population size | Number of candidate solutions per generation. Larger -> better global search but slower. |
| Crossover rate | Probability of combining two solutions. Higher -> more exploration of combinations. |
| Mutation rate | Probability of random perturbation. Prevents premature convergence; too high -> random walk. |
| Cost function (IAE, ISE, ITAE) | IAE = \int\|e\|. ISE = \inte^2. ITAE = \intt.\|e\| (penalises lingering errors more). |

---

### PSO / Differential Evolution
**When to use:** Continuous parameter space; faster convergence needed than GA;
many design parameters.

**What you tune:** Swarm size . Inertia weight . Cognitive/social coefficients

| Parameter | Meaning & Effect |
|---|---|
| Inertia weight omega | Controls momentum of particles. High omega -> broad exploration; low omega -> fine local search. Often annealed from ~0.9 to ~0.4. |
| Cognitive coefficient c₁ | Pulls particle toward its own best found position (self-confidence). |
| Social coefficient c₂ | Pulls particle toward global best position (swarm influence). Balance c₁ vs c₂ controls exploration vs exploitation. |

---

### Bayesian Optimisation
**When to use:** Each evaluation is expensive (hardware-in-the-loop, real plant
experiment); want fewest iterations.

**What you tune:** Acquisition function . Surrogate model (GP hyperparameters)

| Parameter | Meaning & Effect |
|---|---|
| Acquisition function (EI, UCB) | EI (Expected Improvement): balances exploitation of known good regions and exploration of uncertain areas. UCB: more explorative. |
| GP kernel & hyperparameters | GP length-scale controls how smooth/correlated the objective surface is assumed to be. Tuned by maximising marginal likelihood. |

---

### Reinforcement Learning (RL)
**When to use:** Neural controller or gain-scheduled policy; reward-based tuning;
highly nonlinear or unknown dynamics.

**What you tune:** Reward function . Policy network architecture . Learning rate

| Parameter | Meaning & Effect |
|---|---|
| Reward function r(s, a) | Encodes control goals numerically (e.g., -\|error\| - alpha.\|u\|). The most critical design choice; bad reward -> bad policy. |
| Discount factor γ | How much future rewards matter. γ -> 1: long-horizon planning. γ -> 0: myopic greedy policy. |
| Learning rate alpha | Step size for policy/value network updates. Too large -> instability; too small -> slow convergence. |

---

## 5. Auto-Tuning / Adaptive

### Gain Scheduling
**When to use:** Dynamics change predictably with a measurable operating
condition (speed, temperature, load).

**What you tune:** Scheduling variable . Local controller parameters at each operating point

| Parameter | Meaning & Effect |
|---|---|
| Scheduling variable v | Measured signal that indexes which set of gains to use (e.g., airspeed, shaft speed). Must correlate with plant dynamics change. |
| Interpolation method | Linear, polynomial, or lookup-table interpolation between locally tuned gain sets. Affects smoothness of transitions. |

---

### MRAC (Model Reference Adaptive Control)
**When to use:** Desired closed-loop behaviour is known; plant parameters are
uncertain or slowly varying.

**What you tune:** Reference model . Adaptation gain Γ

| Parameter | Meaning & Effect |
|---|---|
| Reference model Mₘ(s) | Defines the ideal response the real closed-loop should match. Must be stable and physically achievable. |
| Adaptation gain Γ | Rate at which controller parameters are adjusted. Large Γ -> fast adaptation but risk of instability/oscillation. |
| Adaptation law (MIT / Lyapunov) | MIT rule: gradient descent on error. Lyapunov: guarantees stability of adaptation. Lyapunov preferred for robustness. |

---

### STR (Self-Tuning Regulator)
**When to use:** Plant parameters change over time; recursive online
identification is feasible.

**What you tune:** Forgetting factor lambda . Recursive estimator initial conditions

| Parameter | Meaning & Effect |
|---|---|
| Forgetting factor lambda (0 < lambda <= 1) | In recursive least squares, weights recent data more when lambda < 1. lambda = 1 -> no forgetting. Smaller lambda -> faster tracking of parameter changes but higher noise sensitivity. |
| Covariance matrix P₀ | Initial estimation uncertainty. Large P₀ -> fast initial adaptation; small P₀ -> slow to correct initial estimates. |

---

## 6. Data-Driven / Iterative Batch

### IFT (Iterative Feedback Tuning)
**When to use:** No plant model available; closed-loop experiments can be
repeated; direct gradient descent on performance.

**What you tune:** Controller parameters θ . Step size γ (learning rate)

| Parameter | Meaning & Effect |
|---|---|
| Cost function J(θ) | Typically a weighted sum of squared tracking error and control effort over the experiment. Gradient estimated from two closed-loop runs. |
| Step size γ | Size of parameter update per iteration. Too large -> overshoot; too small -> slow convergence. |
| Number of iterations | Each iteration requires at least 2 experiments. Converges in 5-20 iterations typically. |

---

### VRFT (Virtual Reference Feedback Tuning)
**When to use:** Single historical or experimental dataset available; want a
model-free one-shot controller design.

**What you tune:** Reference model Mr(z) . Controller structure

| Parameter | Meaning & Effect |
|---|---|
| Reference model Mr(z) | Desired closed-loop transfer function. VRFT constructs a 'virtual reference' from plant data that, ideally, the chosen controller would produce. |
| Prefilter L(z) | Applied to data to reduce bias when the chosen controller class is not rich enough. Should approximate Mr(z). |
