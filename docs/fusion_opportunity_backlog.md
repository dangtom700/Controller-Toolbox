# Controller Fusion - Opportunity Backlog

Authored 2026-07-26, against a codebase with 51 `IController` subclasses, 135 C++ examples
(`ex01`-`ex135`), 22 complete case studies.

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
| B1 | **Repetitive-motion feedforward stack** | `designZPETC` + `ctrl::ILC` + `LearningFeedforwardController` + `RepetitiveController` | M | Four feedforward/learning mechanisms, each demoed alone (`ex59`, `ex70`, `ex129`, `ex29`), never combined. They compose naturally: ZPETC supplies a stable inverse feedforward, ILC learns the residual across trials via `updateFeedforward()`, the repetitive controller mops up what is left periodically. Strong pick-and-place story. |
| B2 | **Safety-supervised adaptation** | `CBFSafetyFilter` + `CLFController` + `L1AdaptiveController` + `ControllerMonitor` | M | The safety filter enforces constraints while adaptation runs underneath. This is the *feasible core* of the original proposal's Tier 3 "self-reflective safety controller", with the unimplementable parts removed. |
| B3 | **End-to-end auto-scheduling pipeline** | `identifyLPV` -> `clusterByGap` -> `buildAutoGainScheduler` -> `generateControllerC` | S | **Partially covered:** the chain is exercised in `tests/test_autoscheduling.cpp`, but `ex60`/`ex61`/`ex62`/`ex120` each show only one stage in isolation. No runnable demo of the full data -> deployable-C workflow exists. Cheapest item in this document. |
| B4 | **Robustness-driven controller selection** | `nuGap` + `MuAnalysis` + `WorstCaseSearch` + `LyapunovRobustness` | M | Analysis-side rather than control-side: score a candidate roster against an uncertainty set and pick a winner on evidence. All four have examples (`ex60`, `ex85`, `ex86`, `ex87`); the *selection procedure* built on them does not. |

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

---

## 5. Build note

The canonical build is **Release** (`CLAUDE.md` section 2). `lib/DiscreteHinf.cpp` fails to
compile in Debug/`-O0` on MinGW with "too many sections", which takes down any target that links
`controller_toolbox`. The build type has been observed reverting to `Debug` after an unqualified
`cmake -S . -B build`; if a build fails on `DiscreteHinf.cpp.obj`, check
`grep CMAKE_BUILD_TYPE build/CMakeCache.txt` first and reconfigure with
`-DCMAKE_BUILD_TYPE=Release` before looking for a real compile error.
