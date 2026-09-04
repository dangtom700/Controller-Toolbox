# The Controller Toolbox Guidebook
## A guide to controller design, and a catalog of every algorithm in the library

> **What this is.** A single, cover-to-cover companion to the Controller Toolbox — a discrete-time C++20/Eigen control library (~125 controller/estimator/identification classes, full pybind11 bindings, 22 physics case studies). **Part I** is a *design guide*: how to go from a plant to a shipped, verified controller. **Part II** is a *catalog*: every family and every class, with what it is, when to reach for it, what it assumes, its key knobs, and its source reference.
>
> **Relationship to the existing docs.** This book ties together and gives a reading path through the repo's reference docs; it does not replace them. Each is the source of truth for its area: [`README.md`](../README.md) (inventory), [`docs/DOCUMENTATION.md`](DOCUMENTATION.md) (class-by-class API), [`docs/control_strategies_deep_dive.md`](control_strategies_deep_dive.md) (strategy taxonomy + decision framework), [`docs/controller_selection_matrix.md`](controller_selection_matrix.md) (plant→shortlist triage), [`docs/deployment.md`](deployment.md) (RT/RTOS), and the [`cheatsheet/`](../cheatsheet/) notes (tuning, pipeline, identification). When this book and the source disagree, trust the source (and [`docs/handoff.md`](handoff.md) over any status doc).
>
> **How to read it.** New to control, or to this library → read Part I in order. Have a plant in hand and want a shortlist → jump to Ch. 2 (characterize) then Ch. 5 (choose). Know the technique, need the class → go straight to the Part II catalog. Every catalog entry names the exact `lib/ClassName` and its reference.

---

## Contents

**Part I — The Design Guide**
1. [[#1. The Discrete-Time Mindset]]
2. [[#2. Characterizing the Plant]]
3. [[#3. The Control Design Pipeline]]
4. [[#4. Identification and Modeling]]
5. [[#5. Choosing a Controller]]
6. [[#6. Tuning]]
7. [[#7. Robustness and Verification]]
8. [[#8. Composition and Architecture]]
9. [[#9. Estimation and State Feedback]]
10. [[#10. Implementation and Deployment]]

**Part II — The Controller Catalog**
11. [[#11. Classical Linear SISO]]
12. [[#12. Optimal and Predictive Control]]
13. [[#13. Robust Analysis and Synthesis]]
14. [[#14. Adaptive and Scheduled Control]]
15. [[#15. Nonlinear Model-Based Control]]
16. [[#16. Intelligent, Fuzzy, and Learning-Based Control]]
17. [[#17. Dead-Time Compensation]]
18. [[#18. Composition Wrappers]]
19. [[#19. Estimators and Filters]]
20. [[#20. System Identification]]
21. [[#21. Optimizers and Tuners]]
22. [[#22. Model and Analysis Utilities]]
23. [[#23. Embedded, Code-Gen, and Deployment]]

**Appendices**
- [[#Appendix A — Tuning Quick Reference]]
- [[#Appendix B — Case Studies as Worked Examples]]
- [[#Appendix C — Source-of-Truth Doc Map]]

---
---

# Part I — The Design Guide

## 1. The Discrete-Time Mindset

Everything in the toolbox runs in **discrete time** at a fixed sample period `Ts`. There is no continuous-time controller object; you design in the z-domain (or design continuous and discretize), and every controller advances one step per call.

**Plant representation.** Two forms, freely convertible:
- `ctrl::TransferFunction G({b...}, {a...}, Ts)` — numerator/denominator in `z⁻¹`.
- `ctrl::StateSpace sys(A, B, C, D, Ts)` — or `ctrl::tf2ss(G)` to convert.
- `ctrl::c2d(sys_c, Ts, ZOH|Tustin)` discretizes a continuous model; `ctrl::ssStep(sys, x, u)` advances the plant one step. `ctrl::DAESystem` + `ctrl::dae_c2d` handle index-1 differential-algebraic plants (kinematic/equilibrium constraints) by eliminating the algebraic part into a `StateSpace`.

**The control loop.** Every controller implements `IController` and is driven the same way — compute error, get `u`, step the plant:
```cpp
for (int k = 0; k < N; ++k) {
    double e = r - y;               // or vector error for MIMO
    double u = ctrl.compute(e);     // one step; internal state advances
    y = ctrl::ssStep(sys, x, uv)(0);
}
```

**Sign conventions are not uniform — this bites people.** PID uses `e = r − y`; SMC uses `e = y − r`. Always check `signConvention()` on the class (or `CONTRIBUTING.md#sign-conventions`) before wiring a new controller into a loop. Getting this wrong turns negative feedback into positive feedback.

**Sample-time selection** is a design decision, not a formality. Rule of thumb: `Ts` ≈ (1/10 to 1/20) of the dominant closed-loop time constant, and fast enough that a significant plant change cannot occur within one step. Too slow loses phase margin; too fast wastes compute and amplifies quantization/derivative noise. (The mechatronics companion book covers the discretization theory — BEM vs. bilinear/Tustin, frequency warping, pre-warping.)

**Three implementation tiers you'll meet:** *closed-form* controllers (PID, LQR, SMC, lead-lag) compute `u[k]` in O(n) arithmetic with no solver — real-time-safe; *optimization* controllers (MPC, GPC, MHE) solve a QP each step via the built-in `GradientProjectionQP`; *offline-synthesis* controllers (H∞, SINDy, Koopman) do the heavy work once and run a fixed gain/model online.

## 2. Characterizing the Plant

Before looking at any controller, answer these questions about the plant. **Each axis is a hard filter** that eliminates whole families. This is the single highest-leverage step in the whole process.

| Axis | Values that change the answer | What it rules in/out |
|---|---|---|
| **Linearity** | LTI · LPV (linear about a moving point) · nonlinear (known model) · black-box/data-only | LTI → LQR/H∞/MPC; LPV → gain scheduling; nonlinear → FL/backstepping/NMPC/SMC; data-only → DeePC/SINDy/Koopman |
| **Channels** | SISO · MIMO with coupling | MIMO → LQR/H∞/MPC (native); SISO-only → PID/lead-lag/Smith/IMC |
| **Open-loop stability** | stable · integrating · unstable (RHP poles) | Unstable → must identify closed-loop; limits pure-inversion methods |
| **Phase** | minimum-phase · non-minimum-phase (RHP zeros) | NMP kills clean inversion → no exact FL / IMC / perfect FF; use ZPETC, MPC, H∞ |
| **Dead-time** | negligible · significant transport delay | Big delay → Smith Predictor or MPC-with-preview |
| **Constraints** | none · soft · **hard** actuator/state/safety limits | Hard limits → MPC family, or wrap in `CBFSafetyFilter`/`AntiWindupWrapper` |
| **Dominant disturbance** | broadband/stochastic · **periodic** (known freq) · **matched** · **previewable/measured** | Periodic → Repetitive/Resonant; matched → SMC/L1; measured → feedforward; stochastic → H2/Kalman |
| **Model confidence** | accurate · structured-but-uncertain · none | Uncertain-but-bounded → H∞/µ/Tube; none → data-driven/adaptive |
| **State access** | full state measured · output only | Output-only → needs an estimator (Ch. 9) |
| **Time-variation** | LTI · slowly varying · fast varying | Slow drift → STR/MRAC/adaptive-GPC; fast → gain scheduling |
| **Compute budget** | MCU/tiny · normal · can afford an online QP/NLP | Tiny → embedded `BasicPID`/`BasicSMC`; QP-budget → MPC |

Write the answers down before shortlisting. A plant that is *nonlinear, output-only, hard-constrained, with a measured disturbance* points to a very different roster than a *benign stable SISO loop* — and you know that before naming a single controller.

## 3. The Control Design Pipeline

Real design is a **loop, not a single pass**. The toolbox supports all ten stages; the arrows back are where most real work happens.

1. **Requirements & specs (first, always).** Define what "good" means: time-domain (overshoot ≤ 10%, settling ≤ 2 s, zero steady-state error), frequency-domain (bandwidth, phase margin ≥ 45°, gain margin ≥ 6 dB), constraints (actuator/rate/state limits), disturbance-rejection and noise bounds, and robustness (allowed degradation under, e.g., ±20% mass). Without this you can neither choose a structure nor judge success.
2. **Experiment design for identification.** The input signal *is* the model quality: PRBS / swept-sine / multisine / chirp to persistently excite the relevant dynamics, amplitude high enough for SNR but inside the linear regime, safety trips, and — for unstable/integrating plants — identify **closed-loop** under a stabilizing controller.
3. **Identification.** Fit a model (Ch. 4).
4. **Model validation (prove it's good enough).** Residual whiteness, cross-validation on fresh data, *simulation* (output-error) response not just one-step-ahead, and uncertainty quantification (confidence intervals / frequency-domain uncertainty discs) for robust design. Skipping this yields controllers that shine in simulation and fail on hardware.
5. **Control-oriented model transformation.** Rarely is the identified model in the form your method needs: `c2d` for sampling, `balancedTruncate`+`suggestOrder` to cut order (n ≤ 10), `lineariseAtPoint` for scheduling, `dae_c2d` for DAEs, `HybridModel`/`GreyBoxEstimator` when physics underfits, disturbance-model augmentation (integrators for offset-free tracking), `padeDelayFilter` for delay, and `MixedSensitivity::build` for H∞ weights.
6. **Controller structure selection.** The architectural decision (Ch. 5) — *not* tuning.
7. **Design & tuning** (Ch. 6).
8. **Robustness & worst-case analysis** (Ch. 7). Nominal step response alone is dangerously optimistic.
9. **Implementation: discretization, anti-windup, bumpless transfer, derivative filtering, real-time scheduling, finite word length** (Ch. 10). These change the *effective* dynamics — part of design, not an afterthought.
10. **Commissioning & fine-tuning on the real plant** — start conservative (low bandwidth/authority), raise while watching signals, run step/load/setpoint tests, apply data-driven retune (IFT/VRFT) if off.

And then **monitoring & maintenance**: detect degradation (`MismatchDetector` CUSUM on innovations, `ControllerMonitor` SPC on output) and trigger re-identification or adaptive retune. Every stage can send you back: validation fails → redesign the experiment; robustness fails → relax specs or change structure; chatter/limit-cycle on hardware → filter, resample, or return to design.

## 4. Identification and Modeling

Pick the identifier by the **data you have** and the **model form you need**.

- **Step/impulse response, SISO, want a PID quickly** → `FOPDTIdentifier` (K, τ, θ) or `SOPDTIdentifier` (two time constants + θ), both with built-in IMC-PID tuning (Rivera 1986 for SOPDT). The fastest path from a bump test to a controller.
- **Online ARX from streaming I/O** → `RecursiveLeastSquares` (forgetting factor) — the engine behind adaptive GPC and self-tuning regulators.
- **MIMO state-space from batch I/O** → `SubspaceID` (N4SID/MOESP) with `suggestOrder`; the standard way to get an (A,B,C,D) for LQR/MPC/LQG.
- **Frequency-response data → rational model** → `FreqDomainIdentifier` (Levy), `VectorFitting`/`ComplexVectorFit` (pole-residue), `SKFit` (Sanathanan–Koerner), `CorrelationID` (cross-correlation impulse response).
- **Structured nonlinear** → `HammersteinWienerIdentifier` (static NL ± linear dynamics), `NARMAXIdentifier` (polynomial NARMAX via orthogonal forward regression), `LPVSystemID` (polynomial LPV).
- **Discover the equations from data** → `SINDy` (sparse regression → interpretable ODEs), `KoopmanEDMD` (lift nonlinear dynamics into a linear model for MPC/LQR), `DeePC` (skip the model entirely — predict straight from a Hankel matrix of data).
- **Known ODE structure, uncertain parameters** → `GreyBoxEstimator` (batch Levenberg–Marquardt) offline, `RecursiveGreyBoxEstimator` (augmented-state UKF) online. When physics underfits, add a learned correction: `HybridModel` + `HybridModelTrainer` (Ridge/GP/ESN) or `GPResidualModel` (learn the mismatch as a GP).
- **Guaranteed bounded-error sets** → `SetMembershipEstimator`, `MLEIdentifier` (batch MLE/MAP, Gaussian/Laplace noise).

Then **validate** (Stage 4 above) and **transform** (Stage 5) before you design. See [`cheatsheet/system_identification.md`](../cheatsheet/system_identification.md), the worked `system_identification/` examples, and [`cheatsheet/advanced_model_estimation.md`](../cheatsheet/advanced_model_estimation.md).

## 5. Choosing a Controller

Two complementary tools: a **decision table** (start at the top, stop at the first row that matches) and a **depth-of-interference** idea (use the *least* invasive technique that meets specs — every step deeper cancels more of the plant's natural dynamics and adds fragility).

**Primary decision table** (condition → strategy → toolbox path):

| Condition | Strategy | Toolbox path |
|---|---|---|
| Well-identified stable SISO, little noise | PID (+ lead-lag for phase) | `DiscretePID` (+ `DiscreteLeadLag`) |
| Periodic disturbance/reference | Add internal-model corrector | `RepetitiveController` (period) / `ResonantController` (single freq), additive stack |
| Measurable disturbance | Add feedforward | `FeedforwardController` / `TwoDOFController` (additive) |
| Unknown bounded disturbance | ESO / DOB | `DiscreteADRC` or `DisturbanceObserverController` |
| Output-feedback LQR needed | Kalman + state feedback | `KalmanFilter` → `DiscreteLQR::compute(x̂)` |
| Nonlinear, smooth, well-known | EKF/UKF + observer+SF | `ExtendedKalmanFilter` + `DiscreteSMC`/`DiscreteLQR` |
| Constrained MIMO | MPC with box constraints | `DiscreteMPC` (→ `TubeMPC`/`ScenarioMPC` if disturbance bounded/stochastic) |
| Time-varying gain (slow drift ≤ ~30%) | Adaptive GPC | `RecursiveLeastSquares` → `GPC::setPlant()` |
| Structured, quantified uncertainty | H∞ / µ-synthesis | `DiscreteHinf::solve` / `solveMuSyn` |
| Qualitatively different operating modes | Hybrid supervisory | `ControllerStack::Supervisory` |
| Large/structural gain drift, model-reference goal | MRAC | `MRACController` |
| Unknown static optimum, no reference | Extremum seeking | `ExtremumSeeker` |
| Strong nonlinearity + matched disturbance | SMC | `DiscreteSMC` (→ `FuzzySlidingModeController` if chattering matters) |
| High-order FEM/CFD model | Reduce first | `BalancedTruncation` → any of the above |

**Quick archetype filters:**
- Benign stable SISO → `DiscretePID` → `DiscreteLeadLag`/`FractionalOrderPID`.
- MIMO + hard constraints → `DiscreteMPC` (+`TubeMPC`/`ScenarioMPC`); `DiscreteLQR`+observer as the smooth baseline.
- MIMO + robustness-critical LTI → `DiscreteHinf` (→ `DiscreteH2` for stochastic-optimal).
- Unstable/fast, operating-point-varying → `GainScheduledController` + LQR/H∞ per point.
- Nonlinear robotics / Euler-Lagrange → `PassivityBasedController`, `FeedbackLinearisation` (computed-torque), `BacksteppingController`, `DiscreteSMC`.
- Periodic → `RepetitiveController` (period) / `ResonantController` (single freq). Big dead-time → `SmithPredictor`.
- Black-box/data-only → `DeePC`, `CEMController`, `NeuralNetworkController` (+ SINDy/Koopman ID upstream).
- **Safety set must never be violated** → wrap any of the above in `CBFSafetyFilter`.

**Wise vs. unwise** (the traps): SMC is wise for bounded matched disturbances with a fast actuator, unwise on noisy sensors / soft actuators / NMP internal dynamics. Feedback linearization needs an accurate (≤5% error), minimum-phase, full-state model — otherwise the cancellation backfires. MRAC wants slowly-varying parameters and persistent excitation. Extremum seeking wants a smooth, unimodal cost and a plant faster than the dither. Hard MPC constraints are for real physical limits, not soft preferences. And **never ship a single controller** — case studies compare a *roster*: a PID baseline plus 2–4 shortlist candidates, so the comparison is meaningful. Full reasoning: [`docs/controller_selection_matrix.md`](controller_selection_matrix.md) and [`docs/control_strategies_deep_dive.md`](control_strategies_deep_dive.md).

## 6. Tuning

Tuning is as consequential as structure — a well-tuned LQR on a rough model beats a poorly-tuned H∞ on a good one. Three families, by the information they need (formulas collected in Appendix A):

- **Heuristic / frequency-domain (little model info).** *Relay auto-tune* (`RelayAutoTuner`): closed-loop relay oscillation → ultimate gain/period → Ziegler–Nichols / Tyreus–Luyben / AMIGO rules (AMIGO is the best-balanced). *IMC-PID* (`FOPDTIdentifier`/`SOPDTIdentifier` → tuner): invert the model, add a λ low-pass; one knob (λ) trades speed for robustness. *Loop shaping* (`LoopShapingTuner`, `DiscreteLeadLag`): place gain crossover at ω_c with target phase margin via a lead network.
- **Optimization-based (need a model/simulation).** *LQR + Bryson's rule* (`LQRWeightTuner`): Q/R from max-allowable state/input → single DARE solve. *LQG* adds a Kalman gain from noise covariances (separation principle — but watch the Doyle-1978 non-robustness; check margins or use LQG/LTR / H∞). *MPC weight/horizon* (`MPCHorizonTuner`): N_p ≥ t_s/Ts, N_c ≈ N_p/3, start ρ_u ≈ 0.1 and raise to calm the input. *Metaheuristics* (`GeneticAlgorithm`, `ParticleSwarmOptimizer`, `DifferentialEvolution`, `BayesianOptimizer`, `NelderMead`, `NSGA2`): minimize a simulation cost (IAE/ITAE/H2) for any structure — model-free but compute-heavy, no stability guarantee during search. `AutoTuner` (CMA-ES) and `TunerSuite` (8 strategies with soft-warning dispatch) wrap these.
- **Adaptive / online (during operation).** `MRACController` (Lyapunov + σ-modification; γ adaptation rates, σ anti-drift), adaptive GPC (`RLS` + `setPlant`), `L1AdaptiveController` (fast adaptation, bounded transient), `SelfTuningRegulator` (RLS-ID → recompute law), `NeuralPID` (backprop-adapted gains).

Sequence that works in practice: **identify first, shape second, optimize last.** Per-controller knob-by-knob guidance is in [`cheatsheet/controller-tuning-reference.md`](../cheatsheet/controller-tuning-reference.md); method notes in [`cheatsheet/tuning_methods.md`](../cheatsheet/tuning_methods.md).

## 7. Robustness and Verification

Nominal performance (MSE/ITAE on one step response) is not evidence the loop will survive the real plant. The toolbox ships an analysis pipeline for exactly this:

- **Monte Carlo robustness** (`RobustnessAnalysis`, `tools/monte_carlo.py`): spawn many perturbed plants within the uncertainty bounds, aggregate stability / margin / sensitivity statistics.
- **Worst-case search** (`WorstCaseSearch`): CMA-ES hunt for the parameter combination that maximizes the sensitivity peak or breaks stability — far more informative than random sampling once you suspect a bad corner.
- **Structured singular value (µ)** (`MuAnalysis`, `LFTSystem`): D-scaling upper bound, `peakMu`, `robustStabilityRadius` for a *rigorous* uncertainty model; `LyapunovRobustness` gives a common quadratic Lyapunov certificate for polytopic uncertainty.
- **Gap metric** (`GapMetric`, `nuGap`): how "far apart" two linearizations are — drives gain-scheduling grid density and tells you when one controller can cover two operating points.
- **Disc margins / classical margins** (`SystemAnalysis`): simultaneous gain/phase tolerance, Bode/Nyquist, stability.
- **Fault & mismatch** (`MismatchDetector`, `FaultClassifier`, `ControllerMonitor`): detect model drift and actuator degradation online.

Wire these as gate 8 of the pipeline; the analysis tools also emit HTML reports (`tools/generate_report.py`, ANOVA, WCET). Design plan: [`docs/robust_implementation_plan.md`](robust_implementation_plan.md).

## 8. Composition and Architecture

**Compose small controllers rather than building a monolith.** Every wrapper is itself an `IController`, so they nest freely — e.g. `AntiWindupWrapper(SmithPredictor(DiscretePID))`.

- **`ControllerStack`** — the core combinator: **Additive** (parallel sum — baseline + corrector), **Weighted** (normalized blend — gain scheduling), **Supervisory** (health-aware switching with bumpless transfer + dwell-time).
- **Series cascade** → `CascadeController` (outer output *is* the inner setpoint; handles inner sign, setpoint clamp/rate-limit, multi-rate decimation, outer anti-windup). Note the additive stack is a *parallel sum* and can't express a cascade.
- **Correctors added to a baseline loop:** `FeedforwardController`/`TwoDOFController` (measured/inverted feedforward), `DisturbanceObserverController` (Q-filter DOB — cancels lumped disturbance+model error), `ResonantController`/`RepetitiveController` (periodic internal models), `LearningFeedforwardController` (two-phase ILC on a nominal loop).
- **Guards & decorators:** `AntiWindupWrapper` (saturation conditioning, Hanus 1987), `CBFSafetyFilter` (a 1-QP that projects any command into a safe set — the go-to for hard safety), `EventTriggeredWrapper` (recompute only past a deadband — save compute/comms), `ComputationalDelayWrapper` (model the controller's own one-sample latency), `FTCSupervisor` (reconfigure the active stack entry on a classified fault), `ControllerMonitor` (telemetry/health only).

The **corrector pattern** — Cascade / Additive / Observer+SF / Supervisory — is the library's recommended way to grow a loop from a PID baseline to a robust architecture without rewriting it.

## 9. Estimation and State Feedback

Any state-feedback law (LQR, SMC, pole placement, MPC) needs the state. When you measure output only, add an **estimator** and wire observer + state feedback **by hand**: `estimator.step(u, y)` → `ctrl.setState(x̂)` → `ctrl.compute(e)`. (It's a wiring pattern, not a fused class — see `CLAUDE.md §4`.)

| Estimator | Use when |
|---|---|
| `KalmanFilter` | Linear-Gaussian |
| `ExtendedKalmanFilter` | Mildly nonlinear (Jacobian valid); supports DAE algebraic projection |
| `UnscentedKalmanFilter` | Strongly nonlinear (sigma points, no Jacobian) |
| `ParticleFilter` | Highly nonlinear / non-Gaussian / multimodal (SIR) |
| `MovingHorizonEstimator` | Estimation with **constraints/bounds** (condensed QP) |
| `HinfFilter` | Worst-case (non-stochastic) estimation — the dual of `DiscreteHinf` |
| `SetMembershipEstimator` | Guaranteed **bounded-error** set estimate |
| `GreyBoxEstimator` / `RecursiveGreyBoxEstimator` | Joint parameter+state ID from a structured ODE |

The estimator's transient must be faster than the loop it feeds (a common rule: observer poles 2–5× faster than the controlled dynamics). Pair `MismatchDetector` on the KF/MHE innovation to catch model drift.

## 10. Implementation and Deployment

Moving from math to hardware introduces effects that change the *effective* dynamics — treat them as design, not cleanup:

- **Discretization** — Tustin / ZOH / matched pole-zero, chosen to preserve stability margins (`c2d`).
- **Anti-windup** — mandatory whenever integral action meets a saturating actuator (`AntiWindupWrapper`, or built into `DiscretePID`).
- **Bumpless transfer** — between manual/auto and between scheduled controllers (`ControllerStack`, `GainScheduledController` handle it).
- **Derivative filtering / derivative-on-measurement** — to avoid noise amplification and setpoint-kick (`DiscretePID` N-filter).
- **Real-time scheduling** — computational delay, jitter, sample-rate mismatch; `ComputationalDelayWrapper` models the loop's own latency; `tools/wcet_report` estimates worst-case execution time.
- **Finite word length** — fixed vs. floating point.

**Deployment surfaces the library provides:**
- **Embedded subset** (`lib/embedded/`): `BasicPID<Scalar>`, `BasicSMC<Scalar>` — header-only templates, no Eigen, no virtual dispatch, `float`-safe for MCUs; plus `DiscreteIntegrator`, `FixedRateFilter`, `RingBuffer`.
- **Code generation** (`CodeGenC`): emit flat C99 for a single tuned, step-based controller.
- **Lock-free RT parameter updates** (`AtomicParamBuffer`): double-buffer so a non-RT thread can retune a controller a RT thread is running.
- **HAL** (`lib/hal/`): `ISensor`/`IActuator`/`SimPlant`/`SafeSensor` for simulation and hardware-in-the-loop.
- **ROS 2** (`ros2/ctrl_toolbox_ros2`): `ControllerNode<T>` lifecycle node wrapping any `ctrl::IController` — topics `~/setpoint`, `~/measurement`, `~/control_output`.
- **Networked/distributed** (`NetworkChannel`): simulate a master/slave link (latency, jitter, loss, reorder) for server/PLC controller-fusion designs.

Full parameter constraints, zero-allocation/RTOS guidance, and troubleshooting recipes: [`docs/deployment.md`](deployment.md).

---
---

# Part II — The Controller Catalog

Every family and class. Each entry: **what it is / when to reach for it**, its **key knobs or assumptions**, and its **reference**. Class names are the exact `lib/ClassName` stems; open the header before wiring (it carries the authoritative API and its `signConvention()`). Controllers implement `IController` unless noted.

## 11. Classical Linear SISO

Cheap, first-thing-to-try compensators for stable/mildly-unstable single loops.

| Class | What it is / when to use | Key knobs · assumes/avoid | Ref |
|---|---|---|---|
| `DiscretePID` | The baseline every study includes — backward-Euler PID with derivative filter and anti-windup; supports P/PI/PD/PID, 2-DOF (`b_weight`), derivative-on-measurement | `Kp,Ki,Kd,N` · weak on strong coupling, hard constraints, big dead-time · `e=r−y` | ISA |
| `FractionalOrderPID` | PIᵏDᵘ — extra tuning freedom / fractional or long-memory dynamics | Oustaloup-approximated fractional operators; more params — overkill for benign plants | Oustaloup |
| `DiscreteLeadLag` | Phase/gain loop-shaping when you know the frequency response | Tustin biquad; LTI SISO only | classical |
| `TwoDOFController` | Feedforward (physics inversion or measured signal) + feedback trim | Use `FeedforwardController` instead when you have a designed `G_ff(z)` | 2-DOF |
| `FeedforwardController` | Add model-based feedforward `G_ff(z)·r` to any feedback loop (additive stack) | needs a plant/inverse model | — |
| `ResonantController` | Reject/track a **single known frequency** (AC, vibration, rotating machinery); additive corrector | frequency must be known/estimated; single-harmonic internal model | IMP |
| `RepetitiveController` | Reject/track a **periodic** signal (all harmonics) with known period | needs exact period; adds delay-line dynamics + Q-filter | IMP |
| `NotchFilter` | Fixed biquad notch to kill a resonance (not an `IController`) | Bristow–Johnson cookbook design | — |
| `PhaseLockedLoop` | SOGI-PLL phase/frequency estimator (grid sync, rotating machinery; not an `IController`) | single-input; tune loop bandwidth | SOGI |

## 12. Optimal and Predictive Control

Model-based MIMO and constrained workhorses. Optimal controllers precompute a gain; predictive controllers solve an optimization each step (built-in `GradientProjectionQP`/`LPSolver` — no external solver).

| Class | What it is / when to use | Key knobs · assumes/avoid | Ref |
|---|---|---|---|
| `DiscreteLQR` (+`LQRAdapter`) | LTI MIMO regulation, full state, smooth (no hard limits) | Q/R (Bryson); needs stabilizable/detectable + state (or observer); **no** hard constraints; DARE-doubling solve | classical |
| `DiscreteLQG` | LQR + Kalman — output-feedback optimal for linear-Gaussian | process/measurement covariances; sensitive to model mismatch (Doyle '78) | classical |
| `DiscreteH2` | LTI MIMO output-feedback, LQG-style stochastic optimality | needs LTI model; no hard constraints; cross-term elimination synthesis | — |
| `DiscreteHinf` | LTI MIMO **robust** output-feedback (worst-case disturbance/uncertainty); mixed-sensitivity S/KS/T; `solveMuSyn` DK-iteration | weighting filters + LTI model; conservative (γ ≥ needed); DGKF 2-Riccati | DGKF |
| `DiscreteMPC` | MIMO with **hard** input/state constraints, receding horizon, preview | N_p,N_c,ρ_y,ρ_u; needs LTI model + per-step QP budget; box constraints on u,Δu | — |
| `GeneralizedPredictiveControl` | Predictive control that pairs with online RLS (adaptive CARIMA) | horizons; SISO-leaning; velocity-form | Clarke |
| `LPMPC` | MPC with 1-/∞-norm cost (LP instead of QP) + constraints | linear cost only; SISO; solved via `LPSolver` | — |
| `TubeMPC` | Constrained MPC that stays feasible under **bounded** disturbance | needs disturbance bound; mRPI tube + constraint tightening; more conservative | Mayne 2005 |
| `ScenarioMPC` | **Stochastic** constraints via N_s sampled noise trajectories | needs scenarios; heavier compute; average-cost QP | Calafiore & Campi 2006 |
| `NonlinearMPC` | Nonlinear plant + constraints, model available | needs nonlinear model + NLP; RTI sequential QP; costly | Diehl 2005 |
| `HybridMPC` | NMPC over a `HybridModel` (physics + data); online ridge update every N steps | data correction budget | — |
| `GPMPC` | GP-uncertainty-aware input-bound tightening for NMPC | GP inference cost | — |
| `CEMController` | Sampling-based (Cross-Entropy Method) MPC over a nonlinear/black-box model | derivative-free rollouts; compute-heavy; warm-start μ | — |
| `DeePC` | Predictive control from **data only** (no parametric model) + constraints | needs rich, persistently-exciting data; LTI-ish; Hankel + ADMM | Coulson 2019 |
| `NonlinearIMC` | Nonlinear plant with a usable model + stable inverse | non-minimum-phase kills the inverse; model-in-loop mismatch feedback | — |
| `GradientProjectionQP`, `LPSolver` | Shared QP / two-phase-simplex LP backends (used by the MPC family) | utility solvers, not controllers | — |

## 13. Robust Analysis and Synthesis

Quantify and guarantee performance across an uncertainty set (see Ch. 7 for the workflow).

| Class | What it is / when to use | Ref |
|---|---|---|
| `DiscreteHinf` | Worst-case robust synthesis (mixed-sensitivity, µ-synthesis) — see Ch. 12 | DGKF |
| `MuAnalysis` | Structured singular value (µ) D-scaling upper bound; `peakMu`, `robustStabilityRadius` | Robustness Phase 3 |
| `LFTSystem` | General multi-block LFT/Δ channel-gather feeding µ-analysis | — |
| `WorstCaseSearch` | CMA-ES search for the worst-case parameter combination over plant uncertainty | Robustness Phase 4 |
| `LyapunovRobustness` | Common quadratic Lyapunov certificate for polytopic uncertainty | Robustness Phase 5 |
| `RobustnessAnalysis` | Monte-Carlo closed-loop robustness: perturbed-plant spawn + stability/margin/sensitivity stats | Robustness Phase 1 |
| `GapMetric` | ν-gap between linearizations (`nuGap`, `nuGapMatrix`, `chordalDist`) — scheduling density | — |
| `LinearModelCluster` | ν-gap agglomerative clustering (`clusterByGap`, `suggestGapThreshold`) of operating points | — |
| `BalancedTruncation` | H∞-bounded model order reduction (`balancedTruncate`, `suggestOrder`) — reduce before LQR/MPC | — |

## 14. Adaptive and Scheduled Control

For uncertain or time-varying plants — the controller adapts its gains or is scheduled by an operating point.

| Class | What it is / when to use | Key knobs · assumes/avoid | Ref |
|---|---|---|---|
| `MRACController` | Force an uncertain LTI-ish plant to track a reference model | γ rates, σ-modification, projection; relative-degree/SPR assumptions; transients rough; minimum-phase | Lyapunov MRAC |
| `L1AdaptiveController` | Fast adaptation with **guaranteed transient** on uncertain plants | bandwidth-limited by its LP filter design | Hovakimyan 2010 |
| `SelfTuningRegulator` | **Slowly** time-varying plant: RLS-ID → recompute min-variance/pole-placement law | poor on fast variation; needs ID excitation | STR |
| `GainScheduledController` | Plant whose linearization varies with a **measurable** scheduling variable (LPV) | needs scheduling signal + gains per point; LinearBlend/NearestNeighbor; bumpless transfer | — |
| `AutoGainScheduler` | Automated gain-scheduling pipeline (`findEquilibrium`, `OperatingPoint`, `design_fn`) | needs an LTI design method per point | — |
| `LPVSystemID` | Polynomial LPV identification (`identifyLPV`, `LPVModel`) feeding scheduling | — | — |
| `AdaptiveSmithPredictor` | Smith Predictor with online cross-correlation **delay** estimation | dead-time + plant model that may drift | — |
| `ExtremumSeeker` | **No reference known** — find the input that optimizes a static map | slow; steady-state only; unimodal cost; plant faster than dither | ESC |
| `NeuralPID` | Online NN adapts Kp/Ki/Kd via backprop through the linearized plant | learning rate; verification harder | — |
| `NNAdaptiveController` | Lyapunov-stable online output-weight adaptation over a fixed NN | stability via projection | — |

## 15. Nonlinear Model-Based Control

Mechanical / Euler–Lagrange / robotics / strict-feedback plants where you cancel or dominate the nonlinearity.

| Class | What it is / when to use | Key knobs · assumes/avoid | Ref |
|---|---|---|---|
| `DiscreteSMC` | Robust tracking under **matched** uncertainty/disturbance; nonlinear/uncertain | boundary-layer φ vs. chattering; needs known control direction; `e=y−r` | classical SMC |
| `FuzzySlidingModeController` | SMC where chattering matters: fuzzy inference on (s, ṡ) schedules K and φ | needs `CTRL_HAS_FUZZY`; `e_scale/de_scale` normalize **s**, not `e` | Palm 1994 |
| `DiscreteADRC` | Unknown disturbance, plant ≈ chain of integrators: 2nd-order LADRC (ESO + PD) | ESO bandwidth vs. noise; single bandwidth knob | Han/Gao |
| `BacksteppingController` | **Strict-feedback** nonlinear systems (many vehicle/robot models) | needs strict-feedback form + model; recursive Lyapunov | — |
| `FeedbackLinearisation` | Cancel a **known** nonlinearity, then close a linear loop | affine-in-control SISO, relative degree 1; DriftFn+GainFn; fails on unstable zero dynamics; needs ≤5% model error + full state | — |
| `PassivityBasedController` | **Euler–Lagrange / robotic** plants — PD+ energy-shaping/damping-injection | needs M(q), C, ∇V callbacks | — |
| `CLFController` | You can supply a control-Lyapunov function | needs a valid CLF; Sontag's universal formula | Sontag |
| `CBFSafetyFilter` | Wrap **any** `IController` so a hard safety set is never violated | 1-QP projection; needs a valid control barrier function | Ames 2017 |
| `DisturbanceObserverController` | Lump external disturbance + model error into d̂ (Q-filter) and cancel it | the only standalone DOB in `lib/` (ADRC's ESO is internal) | Ohishi 1987 |

## 16. Intelligent, Fuzzy, and Learning-Based Control

Hard-to-model, black-box, or data-rich plants; inference engines and trained models.

| Class | What it is / when to use | Key knobs · assumes/avoid | Ref |
|---|---|---|---|
| `FuzzyLogic` (`FuzzyPD`, `FuzzyPID`, `FuzzySupervisor`) | Rule-based control / scheduling when you have expert knowledge, not a model | Mamdani & Takagi–Sugeno inference; needs `CTRL_HAS_FUZZY`; rule base + defuzzifier | — |
| `NeuralNetworkController` | Hard-to-model nonlinear plant with training data — fixed forward pass at runtime | data + offline training; verification harder | — |
| `EchoStateNetwork` | Reservoir computing — temporal/black-box dynamics via ridge-regression readout | spectral radius; random reservoir | Jaeger 2001 |
| `GaussianProcess` | Probabilistic regression (residual models, surrogate) with uncertainty | SE kernel, Cholesky, fixed budget | — |
| `DynaController` | Model-based RL: learns a model (SINDy error-dynamics) + plans synthetic rollouts; wraps any `IController` | learning budget; exploration | Sutton 1991 |
| `ValueIterationSolver` | Grid-based dynamic programming / value iteration for small state spaces | curse of dimensionality | — |
| `CEMController`, `DeePC`, `NeuralPID` | (See Ch. 12 / 14) sampling-MPC, data-driven predictive, adaptive-gain PID | — | — |

## 17. Dead-Time Compensation

| Class | What it is / when to use | Ref |
|---|---|---|
| `SmithPredictor` | Significant transport delay with a plant + delay model — wraps an inner controller (integer + fractional Padé delay) | Smith |
| `AdaptiveSmithPredictor` | Same, but the delay is estimated online (cross-correlation) — for drifting dead-time | — |
| `ComputationalDelayWrapper` | Compensate the controller's **own** one-sample compute latency in simulation | — |

## 18. Composition Wrappers

All are `IController`s — nest freely (Ch. 8). `AntiWindupWrapper(SmithPredictor(DiscretePID))` is a legal, sensible stack.

| Class | What it is / when to use | Ref |
|---|---|---|
| `ControllerStack` | Additive (parallel sum) · Weighted (normalized blend) · Supervisory (health-aware switching, bumpless, dwell-time) | — |
| `CascadeController` | Series inner/outer: outer output **is** the inner setpoint; handles inner sign, setpoint clamp/rate-limit, multi-rate decimation, outer anti-windup | — |
| `AntiWindupWrapper` | Generic anti-windup decorator (back-calculation/conditioning) for any saturating loop | Hanus 1987 |
| `CBFSafetyFilter` | Hard-safety projection (Ch. 15) | Ames 2017 |
| `LearningFeedforwardController` | Repeating task: two-phase ILC (record → apply) layered on a nominal loop, trial state machine built in | Bristow 2006 |
| `IterativeLearningControl` | P-type / D-type / norm-optimal ILC for episodic tasks | Bristow 2006 |
| `EventTriggeredWrapper` | Aperiodic sampling — recompute only past a deadband (save compute/comms) | — |
| `FTCSupervisor` | Fault-tolerant control — reconfigure a `ControllerStack`'s active entry on a classified fault | — |
| `ControllerMonitor` | Telemetry/health only — CUSUM + EWMA SPC charts on output/onState channels | — |

## 19. Estimators and Filters

State/parameter estimation. Observer + state feedback is wired by hand (Ch. 9).

| Class | What it is / when to use | Ref |
|---|---|---|
| `KalmanFilter` | Optimal linear-Gaussian state estimator | Kalman |
| `ExtendedKalmanFilter` | Mildly nonlinear (analytical/numerical Jacobians); DAE algebraic projection | — |
| `UnscentedKalmanFilter` | Strongly nonlinear via sigma points (no Jacobian) | — |
| `ParticleFilter` | Highly nonlinear / non-Gaussian / multimodal (SIR) | Kitagawa 1996 |
| `MovingHorizonEstimator` | Estimation with **constraints/bounds** (condensed QP, box + polytopic) | — |
| `HinfFilter` | Worst-case (non-stochastic) H∞ state filter — estimation dual of `DiscreteHinf` | — |
| `SetMembershipEstimator` | Guaranteed **bounded-error** ellipsoidal set estimate | — |
| `RecursiveLeastSquares` | Online ARX identification (forgetting factor) — engine for adaptive GPC/STR | — |

## 20. System Identification

Fit a model from data (Ch. 4 maps data type → method).

| Class | What it is / when to use | Ref |
|---|---|---|
| `FOPDTIdentifier` | Step-response K, τ, θ + IMC-PID tuning — fastest bump-test-to-PID path | — |
| `SOPDTIdentifier` | Second-order + dead-time (K, τ₁, τ₂, θ) + IMC-PID | Rivera 1986 |
| `SubspaceID` | Batch subspace state-space ID (N4SID/MOESP) + `suggestOrder` — for LQR/MPC/LQG | — |
| `NARMAXIdentifier` | Polynomial NARMAX via orthogonal forward regression | — |
| `HammersteinWienerIdentifier` | Structured nonlinear (static NL ± linear dynamics) | — |
| `LPVSystemID` | Polynomial LPV models for gain scheduling | — |
| `MLEIdentifier` | Batch MLE/MAP ARX, Gaussian/Laplace noise | — |
| `FreqDomainIdentifier` | Levy's method — frequency-response data → rational model | — |
| `CorrelationID` | Cross-correlation impulse-response identification | — |
| `VectorFitting` / `ComplexVectorFit` | Rational (pole-residue) fit of frequency-response data | Gustavsen |
| `SKFit` | Sanathanan–Koerner reweighted complex-response fitting | — |
| `SINDy` | Sparse identification of nonlinear dynamics (STLS) → interpretable ODEs | Brunton 2016 |
| `KoopmanEDMD` | Extended DMD — lift nonlinear dynamics to a linear model for MPC/LQR | — |
| `DeePC` | Model-free predictive control straight from data (also Ch. 12) | Coulson 2019 |
| `GreyBoxEstimator` | Batch Levenberg–Marquardt ODE parameter fit (RK4 sensitivity) | — |
| `RecursiveGreyBoxEstimator` | Online ODE parameter tracking (augmented-state UKF) | — |
| `GPResidualModel` | Learn model-plant mismatch ε = y_true − y_model as a GP for risk-aware MPC | — |

## 21. Optimizers and Tuners

Search controller parameters (Ch. 6) or solve the inner problems.

| Class | What it is / when to use | Ref |
|---|---|---|
| `BayesianOptimizer` | GP-surrogate + UCB/EI acquisition for **expensive** controller tuning | Srinivas 2010 |
| `GeneticAlgorithm` | Real-valued GA (BLX-α crossover, tournament, elitism) for any structure | — |
| `ParticleSwarmOptimizer` | PSO with velocity clamping | Clerc & Kennedy 2002 |
| `DifferentialEvolution` | DE/rand/1/bin with boundary reflection | Storn & Price 1997 |
| `NelderMead` | Derivative-free simplex search (no bounds/population) | Nelder–Mead |
| `NSGA2` | Multi-objective (Pareto) evolutionary optimizer | Deb |
| `ConstrainedTuning` | Exterior-penalty nonlinear-constraint wrapper around any cost/optimizer | — |
| `AutoTuner` | CMA-ES black-box controller-parameter tuner | — |
| `ControllerTuner` | Relay auto-tune, step-response, Bryson LQR weights, MPC horizon | — |
| `TunerSuite` | Unified dispatcher over 8 tuning strategies (soft-warning) | — |
| `ValueIterationSolver`, `GradientProjectionQP`, `LPSolver` | DP / QP / LP solvers used internally | — |

## 22. Model and Analysis Utilities

Transform models and analyze loops (Ch. 5, 7, pipeline stage 5).

| Class | What it is / when to use | Ref |
|---|---|---|
| `PlantModel` | `TransferFunction`, `StateSpace`, `DAESystem`, `tf2ss`, `c2d` (ZOH/Tustin), `dae_c2d`, `ssStep`, `consistentInit` | — |
| `LinearisationHelper` | `jacobianX/U`, `lineariseAtPoint` (ZOH) — numerical linearization at an operating point | — |
| `BalancedTruncation` | Model order reduction with H∞ error bound (`suggestOrder`) | — |
| `ZeroPhaseTrackingFilter` | ZPETC causal zero-phase feedforward prefilter + `transmissionZeros` — for non-minimum-phase tracking | — |
| `GapMetric` / `LinearModelCluster` | ν-gap distance + clustering of linearizations (scheduling) | — |
| `HybridModel` / `HybridModelTrainer` | Physics ODE (RK4) + data-driven state correction; offline Ridge/GP/ESN trainer | — |
| `FunctionApproximator` | Taylor (polynomial) + Padé (rational) data-driven approximation; `padeDelayFilter` | — |
| `SystemAnalysis` | Frequency-domain + stability analysis (Bode/Nyquist, margins, poles/zeros) | — |
| `MetricsAnalyzer` | Time-domain step-response metric extraction (overshoot, settling, ITAE, …) | — |
| `MismatchDetector` | CUSUM on KF/MHE innovation → real-time model-plant mismatch alarm | — |
| `FaultClassifier` / `FTCSupervisor` | Heuristic fault-type classification over residual stats → reconfigure the stack | — |

## 23. Embedded, Code-Gen, and Deployment

Ship the controller (Ch. 10).

| Class / component | What it is / when to use | Ref |
|---|---|---|
| `BasicPID<Scalar>`, `BasicSMC<Scalar>` | Header-only template PID/SMC — no Eigen, no virtual dispatch, `float`-safe for MCUs | — |
| `lib/embedded/` | `DiscreteIntegrator`, `FixedRateFilter`, `RingBuffer` — MCU-safe building blocks | — |
| `CodeGenC` | Emit flat C99 for a single tuned, step-based controller | — |
| `AtomicParamBuffer` | Lock-free double-buffer for RT parameter updates from a non-RT thread | — |
| `lib/hal/` (`HAL.h`) | `ISensor`/`IActuator`/`SimPlant`/`SafeSensor`/`StdTimer` — simulation + HIL | — |
| `ros2/ctrl_toolbox_ros2` | `ControllerNode<T>` lifecycle node wrapping any `ctrl::IController` | ROS 2 Humble |
| `NetworkChannel` | Simulated master/slave link (latency, jitter, loss, reorder) for server/PLC fusion designs | — |

---
---

# Appendices

## Appendix A — Tuning Quick Reference

**Relay auto-tune → PID.** Closed-loop relay of amplitude ±d self-oscillates at the ultimate frequency; measure peak-to-peak output *a* and period *T_u*:
$$K_u=\frac{4d}{\pi a}$$

| Rule | $K_p$ | $T_i$ | $T_d$ |
|---|---|---|---|
| Classic Ziegler–Nichols | $0.6K_u$ | $0.5T_u$ | $0.125T_u$ |
| Tyreus–Luyben (conservative) | $K_u/2.2$ | $2.2T_u$ | $T_u/6.3$ |
| AMIGO (best balance) | $0.4K_u$ | $T_u/0.8$ | $0.1K_pT_u$ |

**IMC-PID for FOPDT** $G(s)=Ke^{-\theta s}/(\tau s+1)$ — one knob λ (closed-loop time constant; larger λ = slower + more robust):
$$K_p=\frac{2\tau+\theta}{2K(\lambda+\theta)},\quad T_i=\tau+\frac{\theta}{2},\quad T_d=\frac{\tau\theta}{2\tau+\theta}$$

**IMC-PID for SOPDT** (Rivera 1986) $G(s)=\frac{K}{(\tau_1 s+1)(\tau_2 s+1)}e^{-\theta s}$, $\tau_1\ge\tau_2$, $\tau_{eq}=\tau_1+\tau_2$:
$$K_p=\frac{\tau_{eq}}{K(\lambda_c+\theta/2)},\quad T_i=\tau_{eq},\quad T_d=\frac{\tau_1\tau_2}{\tau_{eq}}$$
Good default $\lambda_c=\theta$.

**LQR / Bryson's rule** — weights from max-allowable magnitudes, then the discrete Riccati solve:
$$Q_{ii}=\frac{1}{(\max x_i)^2},\quad R_{jj}=\frac{1}{(\max u_j)^2}$$
$$P=A^{\top}PA-(A^{\top}PB)(R+B^{\top}PB)^{-1}(B^{\top}PA)+Q,\quad K=(R+B^{\top}PB)^{-1}B^{\top}PA$$

**LQG** adds the Kalman gain $L=P_eC^{\top}(CP_eC^{\top}+R_f)^{-1}$, control $u_k=-K\hat x_k$ (separation principle). Watch LQG's lack of guaranteed margins — check robustness or use LQG/LTR / H∞.

**MPC weights/horizons.** $N_p\ge t_s/T_s$; $N_c\approx N_p/3$; start $\rho_y=1,\ \rho_u=0.1$ and raise $\rho_u$ to calm the input:
$$J=\sum_{i=1}^{N_p}\rho_y\|y_{k+i}-r\|^2+\sum_{j=0}^{N_c-1}\rho_u\|\Delta u_{k+j}\|^2$$

**Loop-shaping lead** for phase boost φ at crossover ω_c:
$$\alpha=\frac{1+\sin\phi}{1-\sin\phi},\quad z_c=\frac{\omega_c}{\sqrt\alpha},\quad p_c=\omega_c\sqrt\alpha,\quad K=\frac{\sqrt\alpha}{|G(j\omega_c)|}$$

**MRAC** (discrete Lyapunov + σ-modification), $e_m=y-y_m$:
$$\theta_r[k{+}1]=\theta_r[k]-T_s(\gamma_r e_m[k]r[k]+\sigma\theta_r[k]),\qquad \theta_y[k{+}1]=\theta_y[k]-T_s(\gamma_y e_m[k]y[k]+\sigma\theta_y[k])$$
Start γ ≈ 0.1, σ ≈ 0.01–0.05 (anti-drift), reference model $a_m=e^{-T_s/\tau_{des}}$. Full method notes: [`cheatsheet/tuning_methods.md`](../cheatsheet/tuning_methods.md); per-controller knobs: [`cheatsheet/controller-tuning-reference.md`](../cheatsheet/controller-tuning-reference.md).

## Appendix B — Case Studies as Worked Examples

The 22 physics studies under [`case-study/`](../case-study/) are the library's real proving ground: each pairs a nonlinear plant with a **roster** of controllers, sweeps scenarios, and writes CSV telemetry. Use them as templates for matching a plant archetype to a shortlist. Live per-study status: [`docs/case_study_status.md`](case_study_status.md).

| Study | Plant archetype | Representative roster |
|---|---|---|
| Boiler Control (C++ & MATLAB twin) | 3×3 MIMO boiler-turbine (Bell–Åström) | PID, LQR, LQG, MPC, GPC, H∞, SMC, ADRC, Fuzzy-PID … (27) |
| Tug Boat | 3-DOF marine, 6-state MIMO + thrust allocation | MPC, LQR, PID, gain-scheduled (18) |
| Active Suspension (2-DOF; 40-state 6×6 EV) | quarter-car / full-vehicle vibration | LQR, MPC, H∞, GA/PSO/DE-tuned PID (18) |
| Buck-Boost Converter | averaged 2-state, 50 kHz, mode hysteresis | PID, MPC, SMC, gain-scheduled |
| Stewart Platform (6-DOF) | 12-state per-rod, closed-form IK+Jacobian, sea-state input | LQR, MPC, H∞, SMC (12 × 60 configs) |
| Differential-Drive Robot Tracking | 5-state nonholonomic, multi-rate | Backstepping, FeedbackLin, NonlinearMPC, SMC |
| Drill String | 2-DOF torsional, stick-slip (Stribeck) | SMC, ADRC, robust variants (17) |
| Electro-Hydraulic Force Servo | 5-state servo-valve + cylinder | H∞ (ODFC/cascade), MPC, SMC, DOB (14) |
| Surface Ship Manoeuvring | 3-DOF MMG, SRUKF-identified | NMPC, backstepping, SMC + UKF |
| Satellite Launch Vehicle | unstable, fast, flight-phase-varying | GainScheduled, LQR/H∞, MPC |
| PCM Thermal Storage / Battery Thermal / Aircraft Engine Thermal | slow thermal, latent-heat NL, delays | MPC, GainScheduled, PID, ADRC, MRAC + EKF |

Controllers exercised across the studies span the full stack: PID, LQR, LQG, MPC, GPC-RLS, SMC, ADRC, Fuzzy-PID, Smith Predictor, MRAC, H∞, TubeMPC, ScenarioMPC, NonlinearMPC, Feedback Linearisation, EKF-LQR, MHE-LQR, SubspaceID-LQG, L1Adaptive, ILC, NeuralPID, Dyna-MBRL, CEM-MPC, Koopman-MPC, ESN, CBF safety filtering, adaptive SMC, and gain-scheduled variants. To scaffold a new study: [`docs/case_study_copilot_reference.md`](case_study_copilot_reference.md) and the `add-case-study` workflow.

## Appendix C — Source-of-Truth Doc Map

This book is the reading path; these are the authoritative sources for each area.

| Area | Source of truth |
|---|---|
| Onboarding / current verified state | [`docs/handoff.md`](handoff.md) — read before trusting any status doc |
| Inventory, quick start, minimal example | [`README.md`](../README.md) |
| Full class-by-class API | [`docs/DOCUMENTATION.md`](DOCUMENTATION.md) |
| Strategy taxonomy, plant interference, decision framework | [`docs/control_strategies_deep_dive.md`](control_strategies_deep_dive.md) |
| Plant → shortlist triage | [`docs/controller_selection_matrix.md`](controller_selection_matrix.md) |
| Controllers grouped by category / tier | [`cheatsheet/controller_categories.md`](../cheatsheet/controller_categories.md), [`cheatsheet/controller_list.md`](../cheatsheet/controller_list.md) |
| End-to-end design workflow | [`cheatsheet/control_design_pipeline.md`](../cheatsheet/control_design_pipeline.md) |
| Tuning methods + per-controller knobs | [`cheatsheet/tuning_methods.md`](../cheatsheet/tuning_methods.md), [`cheatsheet/controller-tuning-reference.md`](../cheatsheet/controller-tuning-reference.md) |
| System identification | [`cheatsheet/system_identification.md`](../cheatsheet/system_identification.md), [`cheatsheet/advanced_model_estimation.md`](../cheatsheet/advanced_model_estimation.md) |
| Robustness analysis plan | [`docs/robust_implementation_plan.md`](robust_implementation_plan.md) |
| Deployment / RT / RTOS / embedded | [`docs/deployment.md`](deployment.md), [`cheatsheet/embedded_and_realtime.md`](../cheatsheet/embedded_and_realtime.md) |
| Case-study status (auto-generated) | [`docs/case_study_status.md`](case_study_status.md) |
| Adding a controller / study; sign conventions | [`CONTRIBUTING.md`](../CONTRIBUTING.md) |

> **Sign-convention reminder.** They are **not** uniform across the library (PID: `e=r−y`; SMC: `e=y−r`). Always confirm `signConvention()` in the class header before wiring — the single most common integration bug.

---

*Guidebook generated from the Controller Toolbox source tree (`lib/ControllerToolbox.h`, README, docs/, cheatsheet/). It reflects the repository at the time of writing; when a class's header and this book disagree, the header wins.*
