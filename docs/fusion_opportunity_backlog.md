# Controller Fusion - Opportunity Backlog

Authored 2026-07-26 against a codebase with 51 `IController` subclasses, 135 C++ examples
(`ex01`-`ex135`) and 22 complete case studies. Updated 2026-08-02: **A1, B1 and B3 are built**
(`ex135`, `ex137`, `ex136`), taking the C++ example count to 137.

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
| A2 | **Fault-tolerant sensor voting** | `UnscentedKalmanFilter` + `FTCSupervisor` + 3x `NetworkChannel` | M | Three sensors on independent links with different loss/latency; `FTCSupervisor::feedResidual()` + `registerFaultResponse()` drop a faulted channel. `FTCSupervisor` is an `IController`, so it composes into a stack directly. |
| A3 | **Distributed MPC coordination** | 2x `DiscreteMPC` or `ScenarioMPC` + `NetworkChannel` + `ControllerStack` (Supervisory) | L | Two subsystems exchanging predicted trajectories, degrading to decentralised control on link loss. Heaviest of the three - needs a coupling model and a prediction-exchange protocol. |

### Tier B - compositions with no PLC dependency

| # | Demo | Components | Effort | Notes |
|---|---|---|---|---|
| B1 | **Repetitive-motion feedforward stack** | `designZPETC` + `LearningFeedforwardController` (owns the `ILC`) + `RepetitiveController` | M | **Built and passing - `ex137`.** The premise below ("they compose naturally") was **wrong** and the demo says so: ZPETC and ILC are *substitutes*, both attacking trial-repeatable error. Rebuilt as a 2x2 error-class matrix. See section 2b. |
| B2 | **Safety-supervised adaptation** | `CBFSafetyFilter` + `CLFController` + `L1AdaptiveController` + `ControllerMonitor` | M | The safety filter enforces constraints while adaptation runs underneath. This is the *feasible core* of the original proposal's Tier 3 "self-reflective safety controller", with the unimplementable parts removed. |
| B3 | **End-to-end auto-scheduling pipeline** | `identifyLPV` -> `clusterByGap` -> (design) -> `generateControllerC`, with `buildAutoGainScheduler` as the parallel front end | S | **Built and passing - `ex136`.** The chain as written above is **not composable** - `buildAutoGainScheduler` cannot consume an `LPVModel`. It also surfaced a real defect in `clusterByGap`. See section 2b. |
| B4 | **Robustness-driven controller selection** | `nuGap` + `MuAnalysis` + `WorstCaseSearch` + `LyapunovRobustness` | M | Analysis-side rather than control-side: score a candidate roster against an uncertainty set and pick a winner on evidence. All four have examples (`ex60`, `ex85`, `ex86`, `ex87`); the *selection procedure* built on them does not. |

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

---

## 5. Build note

The canonical build is **Release** (`CLAUDE.md` section 2). `lib/DiscreteHinf.cpp` fails to
compile in Debug/`-O0` on MinGW with "too many sections", which takes down any target that links
`controller_toolbox`. The build type has been observed reverting to `Debug` after an unqualified
`cmake -S . -B build`; if a build fails on `DiscreteHinf.cpp.obj`, check
`grep CMAKE_BUILD_TYPE build/CMakeCache.txt` first and reconfigure with
`-DCMAKE_BUILD_TYPE=Release` before looking for a real compile error.
