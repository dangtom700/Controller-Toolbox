# Scope Triage & Feature Gap Analysis

Written 2026-06-26. Source: a full-repo sweep for out-of-scope/stubbed/deferred items (comment
archaeology across `lib/`, `bindings/`, `tools/`, `case-study/`, `tests/`; cross-check against
`docs/algorithm_backlog.md`, `docs/ALGORITHM_ROADMAP_PHASE2.md`/`PHASE3.md`,
`docs/control_strategies_deep_dive.md`, `docs/robust_implementation_plan.md`,
`docs/case_study_status.md`, `docs/PROJECT_MASTER_STATE.md`; `version_1` branch diff check).

## Headline finding

This repo already runs an actively-maintained out-of-scope tracking system:
`docs/algorithm_backlog.md` (dedup source of truth, cross-checked against `lib/` line-by-line)
and `docs/ALGORITHM_ROADMAP_PHASE3.md` (sequencing layer with reuse/effort/prerequisite/test-plan
analysis per item, including its own "Pre-Implementation Audit" that verifies sketch claims
against real code with file:line citations). 23 of 32 already-scoped designs are shipped; what's
left is already enumerated with reasoning there. This report compresses that existing triage into
one place and adds what that system doesn't cover: items scattered in other docs/code comments
that never made it into `algorithm_backlog.md`.

**Anomaly flagged, not acted on:** `docs/ALGORITHM_ROADMAP_PHASE3.md:65-67` contains a note -
*"Currently open features will be paused to complete implementing features that are labeled as
'Out of Scope' in the past iterations... pay off the technical debt before proceeding any further
milestones"* - that breaks the document's own citation style and reads as a workflow directive
embedded in data rather than analysis. Surfaced for awareness; not treated as an instruction.

## 1. Executive Summary

| Metric | Count |
|---|---|
| Items already triaged by the existing backlog system (35 backlog lines / 32 designs) | 35 |
| - of which already shipped | 27 lines (23 designs) |
| - of which still open in the existing roadmap | 8 lines (9 designs: OC4, DT1, DT2, DT3, RC2, NC5, OC3, ML4) |
| Deliberate permanent sub-scope cuts already on record | 5 |
| **New** out-of-scope items found outside the backlog system | 10 |
| Case studies sitting incomplete (stub controllers / not started) | 12 of 31 |
| Proposed to Implement now (Promoted) | 7 |
| Proposed to Reject permanently | 7 |
| Proposed to Defer (real missing prerequisite) | 11 |

No evidence of removed/deprecated `lib/` classes from the `version_1` -> `main` diff (additive-only,
0 files deleted, per `CLAUDE.md` Section 8). No `TODO`/`FIXME`/`DEPRECATED` markers exist inside
`lib/` core algorithm files at all - every real gap lives in doc-comments, `docs/`, or binding
files. No GitHub Issues tracker is in active use; planning is doc-based (`algorithm_backlog.md`).

## 2. The "Rejected" Registry (Permanent Out-of-Scope)

| Feature Name | Location Found | Primary Rejection Rationale | Alternative Approach |
|---|---|---|---|
| `RC2` LMI solver: general N-block SDP beyond feasibility/cost-min/GEVP | `docs/ALGORITHM_ROADMAP_PHASE3.md:1750` | Full general-purpose SDP is a research project; the 3 problem types already scoped cover every robust-control use case on this roadmap. | If `RC2` itself is ever built, stop at feasibility/cost-min/GEVP. |
| `DT1` code generation: MPC/Hinf/MHE targets | `docs/ALGORITHM_ROADMAP_PHASE3.md:1429-1430,1751` | Would need a bundled QP solver or full offline precompute in generated code; v1 is deliberately closed-form-controllers-only. | Hand-port via the existing `lib/embedded/` header-only subset. |
| `ML4` RL control: full Stable-Baselines3 / general RL framework integration | `docs/ALGORITHM_ROADMAP_PHASE3.md:1752` | `DynaController`/`CEMController` already cover lightweight MBRL in C++; a Python example validates the pattern per Phase 2's H3 precedent. | Use existing `DynaController`/`CEMController`, or the Python-only ML4 example once built. |
| `DT3` distributed control: real multi-machine networking (sockets) | `docs/ALGORITHM_ROADMAP_PHASE3.md:1753` | Real network I/O is a deployment concern, not a control-algorithm one; simulated stochastic delay/loss is sufficient for the case-study roster. | Use the in-scope `NetworkedControlWrapper` (simulated) once `DT3` ships. |
| `OC2` DP/value iteration: state dimension > ~4 | `docs/ALGORITHM_ROADMAP_PHASE3.md:1754` | Curse of dimensionality makes grid-based DP intractable; the MPC family already exists for higher dimensions. | `NonlinearMPC`/`TubeMPC`. |
| Stale `// TODO: bind RepetitiveController` / `TODO: bind HinfResult/DiscreteHinf` / `TODO: bind MixedSensitivity` | `bindings/analysis_bindings.cpp:397,405,407` | False positive - verified all three are already fully bound in `bindings/advanced_bindings.cpp:12-61,437-546,615+`. Dead comments from before the work moved files. | Delete the 3 leftover comment blocks (doc hygiene, not a feature). |
| `lib/hal/FreeRTOSScheduler.h`, `lib/hal/ZephyrScheduler.h` as generic, finished implementations | `lib/hal/FreeRTOSScheduler.h:5-23`, `lib/hal/ZephyrScheduler.h:5-25` | Explicitly documented "porting stubs" - only compile against real FreeRTOS/Zephyr headers; doc comments tell the integrator to adapt per-target. No generic "finished" version is possible without real hardware. | Leave as integration points (see Deferred registry). |

## 3. The "Promoted" Registry (Proposed for Implementation)

Ranked by immediate user/case-study impact.

| Feature Name | Location Found | Proposed Module/Path | Required Prerequisites | Estimated Complexity |
|---|---|---|---|---|
| Finish the 6 "Open placeholder" case studies | `docs/case_study_status.md:16,17,24,26,27,34` | `case-study/{Differential Drive Robot Tracking, Dual-Arm IAUV Motion Planning, PCM Thermal Energy Storage Control, Residential Building Comfort SMPC, Satellite Launch Vehicle Systems, Underwater Glider Trajectory Tracking}/sim/` | Reference paper's plant equations (each README cites one) | Low-Medium each - only plant dynamics + controller roster need filling in |
| Dynamic Inversion feedforward wrapper | `docs/control_strategies_deep_dive.md:443-456` | New class wrapping a minimum-phase inverse `TransferFunction`/`StateSpace` feedforward, alongside `FeedforwardController` | None - reuses `TransferFunction`, `tf2ss`, `ssStep` directly | Low |
| `monte_carlo.py` real parallel workers | `tools/monte_carlo.py:148` (`"parallel workers (stub)"`) | `tools/monte_carlo.py` - wrap per-run loop in `multiprocessing.Pool` | None | Low |
| `DiscreteH2` D11!=0 loop-shifting | `lib/DiscreteH2.h:22-23` ("deliberately deferred follow-up") | Extend `lib/DiscreteH2.{h,cpp}` | None beyond existing `DiscreteLQR::solveDARE`/loop-shifting algebra | Medium |
| `MuAnalysis` RealScalar block G-scaling | `lib/MuAnalysis.h:24-29,44-45`, scoped-in then not built per `docs/robust_implementation_plan.md:325-329` | Extend `lib/MuAnalysis.{h,cpp}` | None - Packard & Doyle 1993 G-scaling is well-documented | Medium |
| Event-Triggered Control wrapper | `docs/control_strategies_deep_dive.md:201-217` | New `IController`-decorator class, same pattern as `ComputationalDelayWrapper`/`AntiWindupWrapper` | Known gap: doc's sketch calls `inner_->lastOutput()`, which does not exist on `IController` (verified). Needs a small interface addition first. | Low-Medium once prerequisite accessor is added |
| Continue Phase 4 in existing sequence: `OC4` -> `DT1` -> `DT2` -> `DT3` -> `RC2` | `docs/ALGORITHM_ROADMAP_PHASE3.md:1252-1559` | Already fully scoped with class sketches, reuse plan, and test plans | None new | ~24-27 days total per existing estimate, `RC2` dead last |

## 4. The "Deferred" Registry (Hold for v3.0 / Major Release)

| Feature Name | Dependency Blocking It | Condition for Re-Evaluation |
|---|---|---|
| `NC5` Globally Linearizing Control | None technical - already fully scoped, sequenced last. `docs/ALGORITHM_ROADMAP_PHASE3.md:1563-1604` | If a case study needs a globally-linearizable plant (rare per the doc's own admission). |
| `OC3` Dual Control | Same - fully scoped, niche/research-grade by explicit prior decision. `docs/ALGORITHM_ROADMAP_PHASE3.md:1607-1646` | If an adaptive-control case study specifically needs probing/exploration over certainty-equivalence. |
| `ML4` RL-based control | Needs a runtime setter added to `SelfTuningRegulator` for its own cited example to work - not a bare setter, touches RLS covariance conditioning. `lib/SelfTuningRegulator.h:161` | Add that setter with numerical-safety review, or pick a different example controller. |
| `RC2` LMI solver | Sole remaining motivating use case (multi-objective Hinf/H2 mixed synthesis) has no roadmap line of its own. `docs/ALGORITHM_ROADMAP_PHASE3.md:1261-1275` | Scope that consumer first, or scope `RC2` down to exactly it. |
| Full mu structured-singular-value lower bound | NP-hard in general. `docs/robust_implementation_plan.md:329`, `lib/MuAnalysis.h:27-29` | Only if a case study's robustness margin needs to be provably tight, not just the existing spectral-radius bound. |
| DAE Index >= 2 (Pantelides/BLT) | Research-grade effort; no current case-study plant needs it. `lib/PlantModel.h:250`, `docs/ALGORITHM_ROADMAP_PHASE2.md:557` | If a future case study's plant is genuinely index-2+ (Index-1 covers P1-P3 today). |
| IDA-PBC / full port-Hamiltonian energy shaping for underactuated systems | Needs a port-Hamiltonian system-description interface and offline/symbolic PDE solver for matching conditions - distinct and harder than the already-shipped `PassivityBasedController` (NC2). `docs/control_strategies_deep_dive.md:373-377` | If an underactuated-system case study needs energy-shaping beyond NC2's regulation case. |
| `lib/hal/FreeRTOSScheduler.h` / `ZephyrScheduler.h` full porting | Needs a real FreeRTOS/Zephyr target board/SDK. | When an actual embedded port is undertaken. |
| 6 "Not started" case studies (no sim code at all yet) | Need the source paper's governing equations transcribed before scaffolding is meaningful. `docs/case_study_status.md:14,15,18,20,35,36` | After the 6 placeholder studies are cleared, or if a specific paper becomes a priority. |
| FMU import/export, CasADi autodiff, control co-design, sparse GP | Each needs a new heavy external dependency or separate research track. `docs/ALGORITHM_ROADMAP_PHASE2.md:558-562` | Only if a future case study's accuracy bottleneck traces to one of these. |
| `docs/control_strategies_deep_dive.md`'s remaining `[FUTURE]`/`[PLANNED]` tags | Doc predates Phases 1-3; most other `[FUTURE]` items (Backstepping, MRAC, Feedback Linearisation, Balanced Truncation, Linearisation Helper) are already shipped and the doc was never updated. | Low-effort doc-hygiene pass to re-tag against current `lib/` reality. |

## Smaller observations (don't fit the three tables)

- `lib/HybridModel.h:166`'s `[[deprecated]]` `dynamicsFunc()` is intentional internal API hygiene
  (a safer replacement already exists), not a deferred feature.
- `docs/PROJECT_MASTER_STATE.md:408-409` references "CLAUDE.md's stub tracking," which doesn't
  exist in the current `CLAUDE.md` - likely doc drift from an earlier revision.
