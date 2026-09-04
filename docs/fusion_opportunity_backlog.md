# Controller Fusion - Opportunity Backlog

Authored 2026-07-26 against a codebase with 51 `IController` subclasses, 135 C++ examples
(`ex01`-`ex135`) and 22 complete case studies. Updated 2026-08-02: **A1, B1, B2, B3 and B4 are
built** (`ex135`-`ex139`), taking the C++ example count to 139. Tier B is complete.
Updated 2026-09-03: **A2 is built** (`ex140`), taking the count to 140. **A3 is the only item
left** - and see section 2d, which argues part of it is already spoken for by roadmap `DT3`.

Companion to [`docs/server_plc_fusion_plan.md`](server_plc_fusion_plan.md), which covers the
server/PLC master-slave series specifically. **This document is the wider backlog**: fusions of
already-built `lib/` components that are feasible today, ranked, each sized to be picked up
independently. Nothing here requires new algorithms - only new compositions.

---

## 1. The headline finding: there are no dark components

A coverage sweep of every class in `lib/` against `examples/`, `tests/`, `case-study/` and
`bindings/` found **no component without at least one example**. An initial scan suggested six
uncovered classes (`ZeroPhaseTrackingFilter`, `LinearModelCluster`, `AutoGainScheduler`,
`GapMetric`, `CodeGenC`, `LPVSystemID`) - that result was **wrong**. Each is reached through a
free function rather than a constructor, which a class-name grep misses:

| Component | Actually reached via | Example |
|---|---|---|
| `ZeroPhaseTrackingFilter` | `designZPETC()`, `transmissionZeros()` | `ex59_zpetc` |
| `LinearModelCluster` | `clusterByGap()` | `ex60_gap_clustering` |
| `LPVSystemID` | `identifyLPV()`, `identifyLPVFromIO()` | `ex61_lpv_identification` |
| `AutoGainScheduler` | `buildAutoGainScheduler()` | `ex62_auto_gain_scheduler` |
| `GapMetric` | `nuGap()` | `ex60`, plus 2 tests, 2 bindings |
| `CodeGenC` | `generateControllerC()` | `ex120_code_generation` |

The single genuine zero-usage symbol is `chordalDist()` in `GapMetric.h` - a low-level helper
behind `nuGap()`, not a fusion target.

**Consequence for planning:** "add coverage for uncovered class X" is not available as a
justification for new work. Every item below is justified by an uncovered *composition* instead.

---

## 2. Ranked backlog

Effort is rough: **S** = one example, a few hours. **M** = one example plus non-trivial glue.
**L** = new infrastructure before the demo is possible.

### Tier A - server/PLC series continuation

These were Tier 2 in the original fusion proposal and were blocked on a transport abstraction.
`lib/NetworkChannel.h` (Stage 1a) unblocked them, and `ex131`-`ex135` have proven the pattern.

| # | Demo | Components | Effort | Notes |
|---|---|---|---|---|
| A1 | **PLC-adaptive online retuning** | `AutoTuner` + `RecursiveLeastSquares` + `MismatchDetector` + `AtomicParamBuffer` + `NetworkChannel` | M | **Built and passing - `ex135`** (post-drift IAE -75.8%). See the server/PLC plan, Stage 5, for the two-model detection split it forced. |
| A2 | **Fault-tolerant sensor voting** | `UnscentedKalmanFilter` + `FTCSupervisor` + 3x `NetworkChannel` | M | **Built and passing - `ex140`.** The premise below was **wrong on two counts**: the UKF cannot drop a channel (p and R are fixed at construction) and `FTCSupervisor` switches controllers, not sensors. Rebuilt around two residuals instead of one. See section 2d. |
| A3 | **Distributed MPC coordination** | 2x `DiscreteMPC` or `ScenarioMPC` + `NetworkChannel` + `ControllerStack` (Supervisory) | L | Two subsystems exchanging predicted trajectories, degrading to decentralised control on link loss. Heaviest of the three - needs a coupling model and a prediction-exchange protocol. |

### Tier B - compositions with no PLC dependency

| # | Demo | Components | Effort | Notes |
|---|---|---|---|---|
| B1 | **Repetitive-motion feedforward stack** | `designZPETC` + `LearningFeedforwardController` (owns the `ILC`) + `RepetitiveController` | M | **Built and passing - `ex137`.** The premise below ("they compose naturally") was **wrong** and the demo says so: ZPETC and ILC are *substitutes*, both attacking trial-repeatable error. Rebuilt as a 2x2 error-class matrix. See section 2b. |
| B2 | **Safety-supervised adaptation** | `CBFSafetyFilter` + `CLFController` + `L1AdaptiveController` + `KalmanFilter` + `ControllerMonitor` | M | **Built and passing - `ex138`.** The obvious wiring (feed L1's own `sigma_hat` to the CBF) **does not work** - an input-consistency break. Needed a fifth component. See section 2c. |
| B3 | **End-to-end auto-scheduling pipeline** | `identifyLPV` -> `clusterByGap` -> (design) -> `generateControllerC`, with `buildAutoGainScheduler` as the parallel front end | S | **Built and passing - `ex136`.** The chain as written above is **not composable** - `buildAutoGainScheduler` cannot consume an `LPVModel`. It also surfaced a real defect in `clusterByGap`. See section 2b. |
| B4 | **Robustness-driven controller selection** | `nuGap` + `MuAnalysis` + `WorstCaseSearch` + `LyapunovRobustness` | M | **Built and passing - `ex139`.** Three axes gate; `LyapunovRobustness` could not certify a single integral-action candidate and is reported rather than gated. See section 2c. |

---

## 2b. Findings from the items already built (B1, B3)

Both were sized **S/M** on the assumption that the named components slot together. Neither did,
and in both cases the mismatch was the most useful thing the work produced. Recorded here so the
next person does not re-derive them, and because two are defects in `lib/` rather than in the
proposals.

### B3 / `ex136` - the chain in the table above cannot be built as written

**`buildAutoGainScheduler` cannot consume an `LPVModel`, and no overload would fix that.** It
takes a *continuous-time* `StateFunc f(x, u)` and derives its own operating points by solving
`f(x_eq, u_eq) = 0`: the scheduling variable is a property of the **equilibrium**. An `LPVModel`
is *discrete-time* and scheduled by an **exogenous** signal `p` that never appears in `f`. These
are two different front ends onto the same back end, not two stages of one pipeline. `ex136`
therefore runs both - data-driven and model-based - and joins them at `generateControllerC`.

**`clusterByGap` does not deliver the property its docstring claims.**
[LinearModelCluster.h](../lib/LinearModelCluster.h) states that "all models within a cluster are
within `threshold` nu-gap distance of each other" and rests Vinnicombe's robust-stability
argument on it. It is **single-linkage**: it merges `i` and `j` whenever `gap(i,j) < threshold`,
so membership propagates transitively and cluster *diameter* is unbounded. On a smoothly-varying
1-D family it chains by construction, and it gets worse as the grid gets denser. Measured in
`ex136`: 11 models, threshold 0.35, max intra-cluster gap **0.805**, worst model-to-representative
gap **0.694**. A controller designed for the representative is not guaranteed to stabilise the
cluster, which is the one guarantee the method is chosen for.

*Workaround used:* greedy gap-**covering** - walk the grid and open a new breakpoint as soon as
the gap from the current anchor reaches the threshold. Worst model-to-breakpoint gap drops from
0.694 to 0.195, and closed-loop IAE from 1.52 to 1.42. This lives in the example, **not** in
`lib/` - fixing or re-documenting `clusterByGap` is an owner decision, not something to smuggle
in through a demo.

**`generateControllerC` emits one controller per translation unit.** The include guard is always
`CONTROLLER_GEN_H` and the reset function is always `controller_reset()`; only the step function
name is configurable. An N-cluster schedule needs N separately-compiled units.

**A guess that measurement overturned.** `GainScheduledController.h` warns that `LinearBlend`
advances every bracketing controller's integrator each tick, which reads as an argument for
`NearestNeighbor` with PI controllers. Measured, `NearestNeighbor` is **worse** (IAE 2.76 vs
1.42): hard switching re-seeds an integrator at every crossing and each re-seed costs a
transient, while gap-covering already places breakpoints closely enough to apply the header's own
mitigation. `buildAutoGainScheduler`'s hardcoded `LinearBlend` is the right default.

### B1 / `ex137` - "they compose naturally" was the wrong model

**ZPETC and ILC are substitutes, not complements.** Both attack the tracking error that repeats
trial to trial - one from a model, one from data - so whatever the model already explains, ILC
does not get to learn. Measured marginal contributions:

| scenario | ZPETC vs PID | ILC vs ZPETC | RC vs ZPETC |
|---|---|---|---|
| accurate model, no periodic disturbance | **-86%** | -34% | -0% |
| poor model, no periodic disturbance | -1% | **-32%** | -1% |
| accurate model, periodic disturbance | -30% | +4% | **-70%** |
| poor model, periodic disturbance | -24% | +3% | **-33%** |

The original B1 wording ("ZPETC supplies the inverse, ILC learns the residual, the repetitive
controller mops up what is left") describes a ladder where each rung adds to the last. The
measurement says each mechanism is matched to an **error class**, and applying one outside its
class earns nothing - the first build of `ex137` asserted a strict ladder and failed on three of
four rungs.

**Applying a mechanism outside its class is not free.** `RepetitiveController` at a conservative
`Krc = 0.02` is merely inert when there is no periodic disturbance (0.00960 vs 0.00962 without
it). Raise `Krc` to 0.05 for faster learning and inert becomes actively harmful: **0.0431, a 4.5x
degradation**. "It did not help" and "it made things worse" call for different responses.

**Both learning gains are stability-bounded, and tightly.** P-type ILC diverged at `Lp >= 0.20`
on this plant (RMS 0.63 -> 2.2 -> 2.5) because the direct feedthrough (0.020) is far smaller than
the later Markov parameters - the classic P-type ill-conditioning. `RepetitiveController`
diverged outright at `Krc = 0.10` (RMS 1.88). Both were found by sweeping, not by analysis;
budget a sweep harness when picking up B-tier items with learning loops.

**`LearningFeedforwardController` already owns an `ILC`** - trial state machine, wrap,
`updateFeedforward()` at the boundary and all. Constructing a second bare `ILC` beside it
duplicates the update law. Same shape as `ex133`'s KalmanFilter-embeds-MismatchDetector finding;
the B1 row listing both as separate components was wrong.

---

## 2c. Findings from B2 and B4

### B2 / `ex138` - the intuitive fusion is an input-consistency break

A CBF is not a guarantee; it is a guarantee *conditional on the (f0, g) it was handed*. Tell
`CBFSafetyFilter` the input gain is 1.0 when the plant's is 1.6 and it authorises 60 % more
control than the plant can safely absorb, reporting success the whole way. Measured: barrier
violation 0.207 with the safe set at x <= 1.00.

The repair looks obvious and is algebraically exact. Matched uncertainty (gain error *or* load)
satisfies `xdot = -a x + b_true u + d == -a x + b_nom (u + sigma)`, so it is representable as a
drift offset `f0_eff = -a x + b_nom sigma`, and at the barrier the corrected constraint reduces
to exactly `u <= (a x_max - d)/b_true`. Estimating sigma is what an adaptive law does. Wire
`L1AdaptiveController::estimatedDisturbance()` into the CBF's `f0` and it should work.

**It does not.** L1 advances its internal state predictor with the command *it* computed; the
CBF rewrites that command downstream and L1 is never told. Its `sigma_hat` therefore silently
becomes "plant uncertainty **plus whatever the filter just did**", and feeding it back closes a
loop through the filter's own edits:

| CBF drift source | max x | violation | sigma used | sigma true |
|---|---|---|---|---|
| nominal `f0` (no sharing) | 1.207 | 0.207 | 0.000 | 0.410 |
| **L1's own `sigma_hat`** | 1.217 | **0.217** | **0.103** | 0.403 |
| **`KalmanFilter` on the applied input** | 1.017 | **0.017** | **0.375** | 0.375 |

This is not tuning - swept `Gamma` over 1..100 and no value helps, because the shipped API has
no way to tell L1 what was actually applied. The estimator must *observe* the applied command,
so the drift correction comes from a `ctrl::KalmanFilter` on the augmented state `[x; sigma]`,
whose `step(y, u_prev, u_current)` takes it explicitly. L1 keeps its real job (adaptive
tracking); the KF supplies the filter.

**Generalisable rule: an adaptive controller's internal uncertainty estimate is a control
signal, not a plant measurement.** Nesting it under anything that rewrites its output
invalidates it for every downstream consumer. This applies to `AntiWindupWrapper`,
`ControllerStack`, saturation - any wrapper that edits a command.

Two smaller notes. `CLFController` is a regulator toward V's minimum with no equilibrium
feedforward and no integral action, so it parks at an offset on any plant needing a nonzero
equilibrium input - supply `u_eq` yourself. And a discrete-time CBF enforced at sample instants
permits a bounded one-step excursion (residual violation 0.017, not 0); the claim to assert is
an order-of-magnitude reduction, not zero.

### B4 / `ex139` - one of the four axes cannot certify the problem class

The procedure works: score a roster on nu-gap set radius, mu-based stability radius, CMA-ES
worst-case IAE and common-Lyapunov certification, gate, then rank. It selects `PI balanced`
(worst-case IAE 3.333, stability radius 1.000) over the nominal-IAE winner `PI aggressive`
(nominal IAE 0.948 - best of the roster - but **unstable inside the box**, stability radius
0.669 against a set nu-gap radius of 0.690).

**`isQuadraticallyStable` certified none of the five candidates**, and this is a property of the
method rather than of the candidates. It is explicitly a heuristic, not an SDP: it *sums* the
per-vertex Lyapunov solutions, and its own header warns it suits "vertices clustered around a
common nominal". Every closed loop here contains an integrator, hence a pole near `z = 1`, hence
enormous per-vertex Lyapunov matrices whose cross terms swamp the decrease condition. Swept
`Ts` from 0.05 to 0.50 and box widths down to +-10 %: **no integral-action loop was ever
certified, even at a closed-loop spectral radius of 0.80**. The same call certifies a
well-damped no-integrator vertex set (residual -1.60), so the tool is not broken - the problem
class is outside it. `ex139` runs that control check inline and reports the axis as evidence
rather than gating on it; gating would have rejected every candidate.

**Scope limit worth stating once:** all four tools are LTI. `findWorstCaseIAE` and
`robustStabilityRadius` take the controller as a `StateSpace`, and `isQuadraticallyStable` needs
closed-loop vertex A-matrices. `DiscreteSMC`, `DiscreteMPC`, `FuzzyPID` and ADRC's nonlinear part
cannot be scored by any of them. This procedure ranks LTI candidates against parametric
uncertainty, and nothing else.

---

## 2d. Findings from A2

### A2 / `ex140` - the proposed wiring was wrong in both directions

The A2 row said "three sensors on independent links; `FTCSupervisor::feedResidual()` +
`registerFaultResponse()` drop a faulted channel". Neither half survives contact with the API.

**`UnscentedKalmanFilter` cannot drop a channel.** Measurement dimension `p` and covariance `R`
are constructor arguments with no setter ([UnscentedKalmanFilter.h](../lib/UnscentedKalmanFilter.h)),
so there is no runtime way to shrink `y` or de-weight a row. Voting has to sit *in front of* a
`p = 1` filter, not inside a `p = 3` one. Anyone reaching for per-sensor gating in the estimator
should expect to add the setter first, or to fuse before filtering as `ex140` does.

**`FTCSupervisor` switches controllers, not sensors.** `registerFaultResponse()` maps a
`FaultType` onto a `ControllerStack` entry *name*; the class has no notion of a channel. Sensor
exclusion is the voter's job. The supervisor's job is deciding what to do once the vote is no
longer trustworthy - which is a different question, and the more interesting one.

**The actual finding: one residual is never enough in a voted architecture.** Two are available
and each is blind to what the other catches:

| window | disagreement `max_i \|y_i - y_vote\|` | innovation `y_vote - h_model` |
|---|---|---|
| healthy | 0.063 | 0.078 |
| 1 bad sensor | **0.387** | 0.024 |
| 2 bad sensors (same sign) | **0.375** | **0.327** |

With one faulty sensor the vote is still right, so the innovation sees *nothing* - correctly,
since nothing needs reconfiguring. Only the sensor-vs-sensor disagreement fires. With two faulty
sensors the vote carries the fault and both fire. Disagreement says "someone is lying";
only the model-referenced innovation says "the majority is lying". Monitoring one and not the
other leaves a whole failure mode invisible. Measured end to end: mean tracking error in the
2-bad window is 0.251 m voting alone versus 0.008 m with the reconfiguration (alarm threshold
0.15, sized ~2x clear on both sides rather than guessed).

**Acting on an absorbed residual erases it - the supervisor then flip-flops.** Feeding
`FTCSupervisor` the raw UKF innovation does not work. The filter absorbs the bias over a few
ticks, the residual decays under threshold, the classification returns to `None`, the supervisor
switches *back*, the loop re-engages on the lying measurement, and the innovation now sits at
zero because the estimate agrees with the lie. `ex140` therefore scores the fault against a
reference twin that tracks the filter while healthy and **freezes into open-loop propagation the
moment a fault is flagged**. This is `ex135`'s frozen-reference-model rule (lesson 4b) arriving
from the opposite direction, and it is now general enough to state as lesson 14 below.

**No feedback reconfiguration repairs a corrupted measurement.** This kills the obvious response
of switching to a detuned or gain-limited controller: *any* law with integral action drives the
**measured** value to setpoint, so detuning changes the transient and leaves the steady-state
error exactly where it was. The only reconfiguration that helps is one that stops using the
measurement - here a model-inverse feedforward entry. That has a real cost, and `ex140` runs a
permanently-open-loop control arm to price it: 0.273 m mean error against the load disturbance
versus 0.059 m closed-loop. The fallback is right *in the degraded window* and wrong everywhere
else, which is exactly why it has to be switched rather than adopted.

**Two smaller notes.** `FaultDetectorParams`'s defaults are scaled for a signal that swings:
`stuck_du_threshold = 1e-3` exceeded this level loop's entire command excursion (every healthy
tick classified as `ActuatorStuck`), and `corr_threshold = 0.3` assumes the command reacts to the
*raw* measurement when it actually reacts to the filtered estimate. Both needed re-scaling
(`1e-6`, `0.05`) before the classifier said anything sensible. And `ControllerStack`'s Supervisory
bumpless transfer only helps an incoming entry that *has* state to re-seed - `bumplessInit()` is
called, but a stateless feedforward has no integrator to seed, so the handover steps
(measured 0.119 m^3/s). That is correct behaviour, not a defect, but do not expect bumplessness
from a stateless entry.

**Scope limit stated once:** the 2-out-of-3 vote tolerates one fault and *detects but cannot
resolve* two, which is the standard result and the reason the second residual is load-bearing
here. `ex140`'s staleness gate is also present but **unexercised** - 10 % per-packet loss on a
150 ms link never opens a 450 ms gap, so the 2-sensor and 1-sensor vote paths are reached by a
link *outage*, not by packet loss. The demo says so in its own output rather than implying
coverage it does not have.

**One planning consequence.** A3 (distributed MPC coordination) overlaps roadmap item `DT3`
(distributed/networked control) substantially. Scope them together, or A3 gets built twice.

---

## 3. Assessed as not worth attempting

Recorded so they are not re-proposed. Each was checked against the actual API surface.

| Idea | Verdict |
|---|---|
| **Differentiable MPC + Neural ODE** | **Blocked, not merely hard.** `GradientProjectionQP` is not differentiable and there is no autodiff anywhere in the repo. This needs a missing foundation, not a fusion. |
| **"Quantum-inspired superposition controller"** | **Already exists under a plainer name.** It is `ControllerStack` in `Weighted` mode with `ExtremumSeeker` adjusting the weights - `ex12_weighted_stack` covers the mechanism. The framing adds nothing mechanical. |
| **Meta-learning adaptive controller** | **Large lift, uncertain payoff.** `BayesianOptimizer` exists (`ex78`) but there is no episodic training harness, and meta-learning for adaptive control has no settled theory to validate against. |
| **`AtomicParamBuffer` Python binding** | **Not bindable as-is.** It is a template over an arbitrary trivially-copyable `Params`; there is no natural single pybind11 instantiation. It gets C++ examples (`ex134`, `ex135`), not a binding. |

---

## 4. Cross-cutting lessons for whoever picks this up

Recorded because each of these produced a demo that passed while proving nothing, and each cost
a rebuild cycle to find.

1. **Pair every comparison on a shared seed.** Both arms must see the identical disturbance and
   link trace. `ex131` asserts this explicitly (identical delivered/dropped counts across arms);
   without it a lower IAE only proves the second arm drew an easier random draw.
2. **Check the fault has a steady-state signature before charting it.** In `ex132` an
   actuator-gain collapse was invisible because the steady command was zero both before and
   after; a constant load was needed to make the signature exist at all.
3. **Check the monitored signal is stationary.** Also `ex132`: the steady command
   `u = (c*v + load)/k_act` tracks the velocity setpoint, so a healthy moving axis and a faulted
   stationary one produce the *same* command value. SPC on a non-stationary signal alarms on
   motion, not on faults.
4. **Match the detector to the fault class.** In `ex133` an innovation-CUSUM could not see a slow
   sensor drift, because a Kalman filter tracks drift and the innovation never leaves its null
   distribution. Innovation-based detection needs abrupt faults; slow parametric drift is
   `RecursiveGreyBoxEstimator`'s job.
4b. **An adaptive estimator cannot detect the drift it is absorbing.** The same failure hit
   `ex135` from the other direction: feeding `MismatchDetector` the residual of a *tracking* RLS
   (`lambda = 0.995`, a 4 s memory) produced zero alarms against a 6 s drift ramp. Drift
   detection needs a **frozen** reference model scored against live data; keep the adaptive
   model for supplying the tuner, and keep the two separate. Re-baseline the frozen model after
   a successful adaptation, or the detector alarms forever.
5. **Measure the metric over the window that isolates the question.** Also `ex133`: averaging
   estimate error across the fault let the post-fault period dominate at ~0.46 m, swamping the
   actual question of whether event-triggering degrades the estimate.
6. **Make sure there is something to switch.** In `ex134` the primary wedged at steady state,
   where the frozen command is still correct - so failover gained nothing (IAE 0.3233 vs 0.3234)
   and the measured bump was exactly 0.00000 because both controllers produced identical
   outputs. Assert the gap is non-trivial *before* asserting it was handled well.
7. **Assert against what the alternative would have done**, not an absolute constant. `ex134`
   measures bumplessness relative to the command gap a hard switch would have delivered; an
   absolute threshold passes trivially whenever the gap happens to be small.
8. **Never write a placeholder acceptance flag.** A literal `const bool x = true;` in `ex132`
   hid a real defect until it was replaced with a measurement, at which point the demo failed
   immediately and correctly.
9. **Check the scenario actually needs the thing being demonstrated.** `ex136`'s first revision
   varied only the plant's *bandwidth* (35x) and left the DC gain nearly flat at 1.0 -> 1.5. A
   single mid-envelope PI then came within 6% of the full gain schedule - because what detunes a
   PI loop is the gain it sees, not how fast the plant is. Every clustering assertion passed
   while the demo proved nothing about whether scheduling was worth building. Vary the parameter
   the controller is actually sensitive to.
10. **A two-sided claim needs a two-sided test.** "Mechanism X is matched to error class Y" is
   only tested if the demo also shows X earning nothing on class Z. `ex137` asserts both
   directions for all three feedforward mechanisms; the one-sided version passed for reasons
   that had nothing to do with the thesis.
11. **Budget a sweep harness for anything with a learning loop.** ILC and repetitive control
   both have narrow stability windows that depend on the plant's Markov parameters, and both
   diverge quietly-then-violently outside them. `ex137`'s gains came from a throwaway sweep
   program compiled against `libcontroller_toolbox.a`; guessing twice cost more than writing it
   would have. Note `-std=c++20` does not define `M_PI` - the CMake build uses `gnu++20`, so a
   hand-rolled `g++` line needs `-std=gnu++20` to match.
12. **Check whether a nested estimate is still measuring the plant.** Any estimator that
   predicts from *its own* output stops being a plant measurement the moment something
   downstream rewrites that output. `ex138` lost a whole revision to this. Before consuming an
   adaptive controller's internal estimate anywhere else, ask what signal its predictor was
   advanced with - and prefer an estimator that takes the applied input explicitly, as
   `KalmanFilter::step(y, u_prev, u_current)` does.
13. **A tool returning "no" is not the same as a tool being wrong.** `ex139`'s common-Lyapunov
   axis certified nothing, and the useful move was a control case proving the same call
   succeeds on a problem it *can* handle - which converts "this looks broken" into "this
   problem class is outside the method". Run that control case before reporting a negative
   result, and never gate on an axis that cannot say yes.
14. **Anything that masks a fault also hides it from the detector downstream.** A voter that
   successfully outvotes a bad sensor leaves the post-fusion innovation in its null
   distribution; an adaptive model that absorbs a drift leaves its own residual clean
   (lesson 4b); a filter that swallows a bias erases the evidence a supervisor was about to
   act on. Whenever a mechanism is *supposed* to make a fault invisible, the detector needs a
   reference that mechanism does not touch - a second, structurally different residual
   (`ex140`), or a frozen model (`ex135`). Corollary: if acting on an alarm makes the alarm
   go away, the supervisor will flip-flop, so freeze the reference at alarm time.
15. **Check a component's default parameters against your signal's scale before believing its
   output.** `FaultDetectorParams` ships thresholds tuned for a swinging command; on a level
   loop holding a near-constant flow, `stuck_du_threshold` alone classified every healthy tick
   as `ActuatorStuck`. The classifier was not wrong - it was being asked about a signal two
   orders of magnitude smaller than its defaults assume. Print the statistic next to the
   threshold before trusting either.

---

## 5. Build note

The canonical build is **Release** (`CLAUDE.md` section 2). `lib/DiscreteHinf.cpp` fails to
compile in Debug/`-O0` on MinGW with "too many sections", which takes down any target that links
`controller_toolbox`. The build type has been observed reverting to `Debug` after an unqualified
`cmake -S . -B build`; if a build fails on `DiscreteHinf.cpp.obj`, check
`grep CMAKE_BUILD_TYPE build/CMakeCache.txt` first and reconfigure with
`-DCMAKE_BUILD_TYPE=Release` before looking for a real compile error.
