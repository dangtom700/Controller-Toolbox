# Controller Selection Matrix — plant nature → candidate shortlist

Purpose: given a plant model, **filter the ~42-controller roster down to a 3–5 candidate
shortlist** for a case-study roster. This is a *triage* aid, not a design manual — once you have a
shortlist, read the per-strategy behaviour and pitfalls in
[`control_strategies_deep_dive.md`](control_strategies_deep_dive.md), confirm the exact API and the
`compute()` sign in the class header + `CONTRIBUTING.md#sign-conventions`, then wire it per the
[`add-case-study`](../.claude/skills/add-case-study/SKILL.md) workflow.

> Ground truth: roster enumerated from `lib/*.h` (42 classes deriving `IController` + 8
> estimators), all exercised green by `run.py` (Phase 5 175/175 exe, Phase 6 149/149 py). Names
> below are the actual `lib/ClassName` stems.

---

## 1. Characterise the plant first (the axes that decide everything)

Answer these before looking at any controller. Each axis is a hard filter.

| Axis | Values that change the answer |
|---|---|
| **Linearity** | LTI · LPV (linear about a moving operating point) · nonlinear (known model) · black-box/data-only |
| **Channels** | SISO · MIMO with coupling |
| **Open-loop stability** | stable · integrating · unstable (RHP poles) |
| **Phase** | minimum-phase · non-minimum-phase (RHP zeros → no clean inverse) |
| **Dead-time** | negligible · significant transport delay |
| **Constraints** | none · soft · **hard** actuator/state/safety limits |
| **Dominant disturbance** | broadband/stochastic · **periodic** (known freq) · **matched** uncertainty · **previewable/measured** |
| **Model confidence** | accurate · structured-but-uncertain · none (data only) |
| **State access** | full state measured · output only (⇒ needs an estimator, §4) |
| **Time-variation** | LTI · slowly varying · fast varying |
| **Compute budget** | MCU/tiny · normal · can afford an online QP/NLP each step |

---

## 2. Roster at a glance (use when / assumes-or-avoid)

### Classical linear SISO — cheap, first thing to try
| Class | Use when | Assumes / avoid |
|---|---|---|
| `DiscretePID` | Any stable/mildly-unstable SISO loop; the baseline every study should include | Weak on strong coupling, hard constraints, big dead-time |
| `FractionalOrderPID` | SISO needing extra tuning freedom / fractional or long-memory dynamics | More params to tune; overkill for benign plants |
| `DiscreteLeadLag` | Phase/gain loop-shaping when you know the frequency response | LTI SISO only |
| `ResonantController` | Reject/track a **single known frequency** (AC, vibration, rotating machinery) | Frequency must be known/estimated |
| `RepetitiveController` | Reject/track a **periodic** signal with known period (all harmonics) | Needs exact period; adds delay-line dynamics |
| `FeedforwardController` | Add model-based FF to any feedback loop | Needs a plant/inverse model |

### Optimal / predictive (model-based, linear) — the MIMO & constrained workhorses
| Class | Use when | Assumes / avoid |
|---|---|---|
| `DiscreteLQR` (+`LQRAdapter`) | LTI MIMO regulation with full state; smooth, no hard limits | Needs stabilizable/detectable + state (or observer); **no** hard constraints |
| `DiscreteH2` | LTI MIMO output-feedback, LQG-style stochastic optimality | Needs LTI model; no hard constraints |
| `DiscreteHinf` | LTI MIMO **robust** output-feedback (worst-case disturbance/uncertainty) | Needs weighting filters + LTI model; conservative |
| `DiscreteMPC` | MIMO with **hard** input/state constraints, receding horizon, preview | Needs LTI model + per-step QP (compute budget) |
| `GeneralizedPredictiveControl` | Predictive control that pairs with online RLS (adaptive CARIMA) | SISO-leaning; tuning of horizons |
| `LPMPC` | MPC with 1-/∞-norm cost (LP instead of QP), constraints | Linear cost only |
| `TubeMPC` | Constrained MPC that must stay feasible under **bounded** disturbance/uncertainty | Needs disturbance bound; more conservative than nominal MPC |
| `ScenarioMPC` | **Stochastic** constraints under uncertainty via scenario sampling | Needs disturbance samples/scenarios; heavier compute |
| `NonlinearMPC` | Nonlinear plant + constraints, model available | Needs nonlinear model + NLP solver; costly |
| `NonlinearIMC` | Nonlinear plant with a usable model + stable inverse | Non-minimum-phase kills the inverse |

### Adaptive / scheduled / model-free-tuning — uncertain or varying plants
| Class | Use when | Assumes / avoid |
|---|---|---|
| `MRACController` | Force an uncertain LTI-ish plant to track a reference model | Relative-degree/SPR assumptions; transients can be rough |
| `L1AdaptiveController` | Fast adaptation with **guaranteed transient** on uncertain plants | Bandwidth-limited by its filter design |
| `SelfTuningRegulator` | **Slowly** time-varying plant; RLS-ID + recompute control law | Poor on fast variation; ID excitation needed |
| `GainScheduledController` | Plant whose linearisation varies with a **measurable** scheduling variable (LPV) | Needs the scheduling signal + gains per operating point |
| `ExtremumSeeker` | **No reference known** — find the input that optimises a static map | Slow; steady-state optimisation only |

### Nonlinear (model-based) — mechanical/EL, robotics, strict-feedback
| Class | Use when | Assumes / avoid |
|---|---|---|
| `DiscreteSMC` | Robust tracking under **matched** uncertainty/disturbance; nonlinear/uncertain | Chattering vs. boundary layer; needs known control direction |
| `BacksteppingController` | **Strict-feedback** nonlinear systems (many vehicle/robot models) | Needs the strict-feedback form + model |
| `FeedbackLinearisation` | Cancel a **known** nonlinearity, then close a linear loop | Needs accurate model + full state; fails on unstable zero dynamics |
| `PassivityBasedController` | **Euler-Lagrange / robotic** plants (needs `M(q)`, `C`, `∂V`) — manipulators | Needs the EL structure callbacks |
| `CLFController` | You can supply a control-Lyapunov function | Needs a valid CLF |

### Data-driven / learning — soft robots, black-box, data-rich
| Class | Use when | Assumes / avoid |
|---|---|---|
| `DeePC` | Predictive control from **data only** (no parametric model), constraints | Needs rich, persistently-exciting data; LTI-ish behaviour |
| `NeuralNetworkController` / `NeuralPID` | Hard-to-model nonlinear plant with training data | Data + training; verification harder |
| `CEMController` | Sampling-based MPC over a nonlinear/black-box model | Compute-heavy sampling |
| `DynaController` | Model-based RL (learns a model + plans) | Learning budget; exploration |

### Dead-time
| Class | Use when |
|---|---|
| `SmithPredictor` / `AdaptiveSmithPredictor` | Significant transport delay with a (possibly-adapting) delay + plant model |
| `ComputationalDelayWrapper` | Compensate the controller's own compute latency |

---

## 3. Quick filters — plant archetype → shortlist

- **Benign stable SISO, no constraints** → `DiscretePID` → `DiscreteLeadLag` / `FractionalOrderPID`.
- **MIMO, coupled, hard constraints** → `DiscreteMPC` (→ `TubeMPC`/`ScenarioMPC` if disturbance is bounded/stochastic); `DiscreteLQR`+observer as the smooth baseline.
- **MIMO, robustness-critical, LTI** → `DiscreteHinf` (→ `DiscreteH2` for stochastic-optimal).
- **Unstable / fast, operating-point-varying** → `GainScheduledController` + `DiscreteLQR`/`DiscreteHinf` per point.
- **Nonlinear robotics / EL dynamics** → `PassivityBasedController`, `FeedbackLinearisation` (computed-torque), `BacksteppingController`, `DiscreteSMC`.
- **Matched uncertainty / robustness on nonlinear** → `DiscreteSMC`, `L1AdaptiveController`, `MRACController`.
- **Periodic reference/disturbance** → `RepetitiveController` (period) / `ResonantController` (single freq).
- **Big dead-time** → `SmithPredictor` wrapping the inner controller.
- **Black-box / data-only** → `DeePC`, `CEMController`, `NeuralNetworkController` (+ SINDy/Koopman model-ID upstream).
- **Safety set must never be violated** → wrap any of the above in `CBFSafetyFilter`.

---

## 4. Estimators (needed whenever state isn't measured — the observer-SF partner)

Observer + state feedback is **wired by hand** (`estimator.step()` → `ctrl.setState()` →
`compute()`), not a class — see `CLAUDE.md §4`.

| Estimator | Use when |
|---|---|
| `KalmanFilter` | Linear Gaussian |
| `ExtendedKalmanFilter` | Mildly nonlinear (Jacobian valid) |
| `UnscentedKalmanFilter` | Strongly nonlinear (sigma points) |
| `ParticleFilter` | Highly nonlinear / non-Gaussian / multimodal |
| `MovingHorizonEstimator` | State estimation with **constraints/bounds** |
| `GreyBoxEstimator` / `RecursiveGreyBoxEstimator` | Joint parameter+state ID from a structured model |
| `SetMembershipEstimator` | Guaranteed **bounded-error** set estimation |

## 5. Composition wrappers (all are `IController`s — nest freely)

`ControllerStack` (Additive=cascade/additive, Supervisory=health-aware switching) ·
`AntiWindupWrapper` (saturation) · `CBFSafetyFilter` (safety set) · `GainScheduledController` ·
`SmithPredictor` (dead-time) · `EventTriggeredWrapper` (reduce compute/comms) · `FTCSupervisor`
(fault-tolerant switching) · `ControllerMonitor` (telemetry/health only). Compose rather than
build a monolith: e.g. `AntiWindupWrapper(SmithPredictor(DiscretePID))`.

---

## 6. Candidate rosters for the 12 open / not-started studies

Shortlists implied by each plant's physics (fill these when building the roster; always include a
`DiscretePID` baseline for comparison). Confirm against the paper before committing.

| Study (status) | Plant nature | Candidate shortlist | Estimator |
|---|---|---|---|
| **Building Energy Management System** (not started) | slow thermal, MIMO, hard comfort/energy limits, weather disturbance | `DiscreteMPC`, `ScenarioMPC`, `GainScheduledController`, PID | `KalmanFilter` |
| **Residential Building Comfort SMPC** (placeholder) | stochastic thermal + comfort constraints (name = SMPC) | `ScenarioMPC` (primary), `TubeMPC`, `DiscreteMPC` | `KalmanFilter` |
| **PCM Thermal Energy Storage Control** (placeholder) | phase-change thermal, slow, latent-heat nonlinearity, limits | `DiscreteMPC`, `GainScheduledController`, PID | `ExtendedKalmanFilter` |
| **Satellite Launch Vehicle Systems** (placeholder) | unstable, fast, dynamics vary with flight phase | `GainScheduledController`, `DiscreteLQR`/`DiscreteH2`, `DiscreteHinf`, `DiscreteMPC` | `KalmanFilter` |
| **Differential Drive Robot Tracking** (placeholder) | nonholonomic kinematics, nonlinear tracking | `BacksteppingController`, `FeedbackLinearisation`, `NonlinearMPC`, `DiscreteSMC` | `ExtendedKalmanFilter` |
| **Underwater Glider Trajectory Tracking** (placeholder) | buoyancy-driven, slow, underactuated nonlinear | `BacksteppingController`, `NonlinearMPC`, `DiscreteSMC`, LQR(linearised) | `EKF`/`UKF` |
| **Dual-Arm IAUV Motion Planning** (placeholder) | underwater vehicle-manipulator, coupled MIMO nonlinear | `PassivityBasedController`, `NonlinearMPC`, `BacksteppingController` | `UnscentedKalmanFilter` |
| **Underwater Robotic Manipulator Trajectory Tracking** (not started) | EL manipulator, coupled nonlinear | `PassivityBasedController`, `FeedbackLinearisation` (computed-torque), `DiscreteSMC`, `BacksteppingController` | `UnscentedKalmanFilter` |
| **Heavy-Duty Parallel-Serial Hydraulic Manipulator VDC** (not started) | hydraulic, nonlinear, virtual-decomposition control | `PassivityBasedController`, `BacksteppingController`, `DiscreteSMC` | `EKF`/`UKF` |
| **Hybrid-Driven Tendon-Pneumatic Soft Manipulator** (not started) | soft robot, highly nonlinear, hysteresis, uncertain | `NeuralNetworkController`, `DeePC`, `DiscreteSMC`, `L1AdaptiveController` | `ParticleFilter`/`UKF` |
| **Data-Driven Sliding Mode Control of Soft Robot 2024** (not started) | soft robot, data-rich, model-free intent | `DeePC`, `DiscreteSMC` (data-driven surface), `NeuralNetworkController` | `UKF`/`ParticleFilter` |
| **Unmanned Surface Vehicle Wave-Predictive Attitude Control** (not started) | wave disturbance with **preview**, marine attitude | `DiscreteMPC` (preview/FF), `TubeMPC`, `ResonantController` (periodic wave), PID+DOB | `KalmanFilter`/`UKF` |

---

### Caveats
- This matrix is **technique-level**; a specific `lib/` class may add its own assumptions (relative
  degree, control-direction sign, required callbacks). Always open the header before wiring.
- Sign conventions are **not uniform** — check `signConvention()` / `CONTRIBUTING.md#sign-conventions`
  (e.g. SMC uses `e = y - r`, PID uses `e = r - y`).
- Don't ship a single controller: case studies compare a **roster**. Include a PID baseline plus 2–4
  candidates from the shortlist so the comparison table is meaningful.
