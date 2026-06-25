# Algorithm Backlog (Phase 4+ candidates)

Written 2026-06-22. Source: an external feature wishlist comparing this toolbox against
commercial offerings (MATLAB Control System / Robust Control Toolboxes), ~50 items across
11 categories. Every item below was cross-checked against `lib/` (80 headers as of this
writing) before being kept — items already implemented are marked **DONE** with the file
that covers them, not carried forward as backlog. This is a candidate list for scoping
*future* phases one at a time (the way `docs/ALGORITHM_ROADMAP_PHASE2.md` scoped DAE→grey-box→
hybrid models, and the way [docs/superpowers/specs/2026-06-22-frequency-domain-analysis-plots-design.md](superpowers/specs/2026-06-22-frequency-domain-analysis-plots-design.md)
scoped classical frequency-domain plots) — not a commitment to build all of it, and not full
designs. Each entry is a one-line pointer, not a spec; brainstorm it properly before building.

**Shipped:** classical frequency-domain plots (Bode, Nyquist, Nichols, root locus,
singular-value) — Phase 4 Iteration 1, see
[2026-06-22-frequency-domain-analysis-plots-handoff.md](superpowers/specs/2026-06-22-frequency-domain-analysis-plots-handoff.md).

**Shipped:** frequency-domain system identification (Levy's method,
`FreqDomainIdentifier::fitLevy`) — Phase 4 Iteration 2, see
[2026-06-22-frequency-domain-identification-handoff.md](superpowers/specs/2026-06-22-frequency-domain-identification-handoff.md).

**Shipped:** Resonant Controller, Notch Filter, Phase-Locked Loop (the entire former
"Additional Controller Types" section), see
[2026-06-24-resonant-notch-pll-controllers-design.md](superpowers/specs/2026-06-24-resonant-notch-pll-controllers-design.md).

**Shipped:** `ALGORITHM_ROADMAP_PHASE3.md` Phase 1 (9 designs: EF1, RC1, NC1, NC2, NC4, SI5,
SI2, FD1, MO2), see [2026-06-24-small-foundational-utilities-design.md](superpowers/specs/2026-06-24-small-foundational-utilities-design.md),
[2026-06-24-hinf-filter-design.md](superpowers/specs/2026-06-24-hinf-filter-design.md),
[2026-06-24-lft-system-design.md](superpowers/specs/2026-06-24-lft-system-design.md),
[2026-06-24-nonlinear-control-trio-design.md](superpowers/specs/2026-06-24-nonlinear-control-trio-design.md),
and [2026-06-24-hammerstein-wiener-design.md](superpowers/specs/2026-06-24-hammerstein-wiener-design.md).

**Roadmap:** every item below is sequenced into 5 value/ROI-ordered phases (32 designs covering
all 35 open lines, 9 shipped so far) in [docs/ALGORITHM_ROADMAP_PHASE3.md](ALGORITHM_ROADMAP_PHASE3.md)
— goal, class sketch, reused components, effort estimate, and Catch2 test plan per item. Treat
this file as the dedup source of truth and that document as the sequencing/scoping layer on top
of it.

---

## Already done (removed from backlog, kept here so this doesn't get re-proposed)

| Wishlist item | Covered by |
|---|---|
| Model Reference Adaptive Control (MRAC) | `lib/MRACController.h` |
| Feedback Linearization | `lib/FeedbackLinearisation.h` |
| SINDy (Sparse Identification of Nonlinear Dynamics) | `lib/SINDy.h` |
| Repetitive Control | `lib/RepetitiveController.h` |
| Gaussian Process residual models | `lib/GaussianProcess.h`, `lib/GPResidualModel.h` |
| Disk Margin Analysis | `lib/SystemAnalysis.h` (`calculateDiskMargin`, Seiler/Packard/Gahinet 2020) |
| Time-domain analysis (rise/overshoot/settle/IAE/RMS) | `tools/metrics.py` |
| Worst-case / uncertain-system analysis (partial) | `lib/WorstCaseSearch.h` (CMA-ES), `lib/LyapunovRobustness.h` (polytopic uncertainty) — covers worst-case search and common-Lyapunov robustness; a general parametric-uncertainty (LFT) *representation* is still open, see Robust Control below |
| MHE nonlinear/constrained variants (partial) | `lib/MovingHorizonEstimator.h` already has polytopic inequality constraints (`ALGORITHM_ROADMAP_PHASE2.md` E4) |
| Sanathanan-Koerner iteration / Vector Fitting (partial) | `lib/VectorFitting.h` implements SK iteration and real-pole Vector Fitting (Gustavsen & Semlyen 1999), but only for fitting a *real positive magnitude* profile (no phase) to a *real-pole* filter, built for `DiscreteHinf::solveMuSyn`'s D-scaling fits — not general complex-response identification. See "Frequency-Domain Identification Extensions" below for what's still open. |
| Frequency-domain identification | `lib/FreqDomainIdentifier.h` (`fitLevy`, Levy 1959 linearised least-squares fit of a full complex frequency response to a `TransferFunction`) — Phase 4 Iteration 2. SK-iteration and complex-pole-pair generalizations are deliberately deferred, see "Frequency-Domain Identification Extensions" below. |
| H2 synthesis | `lib/DiscreteH2.h`/`.cpp` — discrete LQG separation-principle solve (two DAREs via `DiscreteLQR::solveDARE`, the "Riccati-based shortcut" this entry once said H2 needed instead of an LMI solver). Requires D11=0/D22=0 generalised plants (full D11≠0 loop-shifting support still open, see Robust Control below). Bound in `bindings/advanced_bindings.cpp` (`ctrl.DiscreteH2`), exercised by `examples/ex88_h2_synthesis.cpp`. |
| Structured H-infinity (fixed order/structure) | `DiscreteHinf::solveStructured()` in `lib/DiscreteHinf.h`/`.cpp` — CMA-ES (`AutoTuner`) direct search over a caller-supplied fixed-order parameterisation, exactly the non-LMI approach this entry predicted. **Caveat:** implemented but has no test/binding/example coverage yet (unlike `solve()`/`solveMuSyn()`) — verify behaviour before relying on it. |
| Mu-synthesis / structured-uncertainty analysis (partial) | `lib/MuAnalysis.h` (`UncertaintyStructure`/`computeMu`/`peakMu`/`robustStabilityRadius`, D-scaling upper bound on the structured singular value) plus `DiscreteHinf::solveMuSyn()` (DK-iteration). Covers the canonical block-diagonal multiplicative-output-uncertainty M-Delta loop; a general arbitrary P-Delta LFT interconnection builder is still open, see "General parametric-uncertainty representation (LFT)" below. |
| Resonant controllers | `lib/ResonantController.h`/`.cpp` (`IController`, Tustin-prewarped biquad) — `examples/ex89_resonant_controller.cpp`. |
| Notch filters | `lib/NotchFilter.h`/`.cpp` — `examples/ex90_notch_filter.cpp`. |
| Phase-locked loop (PLL) | `lib/PhaseLockedLoop.h`/`.cpp` — `examples/ex91_phase_locked_loop.cpp`. |
| Correlation-based identification | `lib/CorrelationID.h`/`.cpp` (cross-correlation impulse-response estimation + PRBS generator) — Phase 3 SI2, `examples/ex92_correlation_id.cpp`. See [2026-06-24-small-foundational-utilities-design.md](superpowers/specs/2026-06-24-small-foundational-utilities-design.md). |
| Nelder-Mead simplex | `lib/NelderMead.h` (header-only, shares `AutoTuner::CostFn`/`TunerResult`) — Phase 3 MO2, `examples/ex93_nelder_mead.cpp`. See [2026-06-24-small-foundational-utilities-design.md](superpowers/specs/2026-06-24-small-foundational-utilities-design.md). |
| Generalize SK iteration to full complex-response fitting | `lib/SKFit.h`/`.cpp` (iteratively-reweighted Levy fit; also added `FreqDomainIdentifier::buildLevySystem`/`fitRMSE` as shared, weight-parameterized helpers) — Phase 3 FD1, `examples/ex94_sk_complex_fit.cpp`. See [2026-06-24-small-foundational-utilities-design.md](superpowers/specs/2026-06-24-small-foundational-utilities-design.md). |
| H-infinity filter | `lib/HinfFilter.h`/`.cpp` (bordered-Riccati H-infinity state filter; promoted `DiscreteHinf::solveHinfDARE` to public for reuse) — Phase 3 EF1, `examples/ex95_hinf_filter.cpp`. See [2026-06-24-hinf-filter-design.md](superpowers/specs/2026-06-24-hinf-filter-design.md). |
| General parametric-uncertainty representation (LFT) | `lib/LFTSystem.h`/`.cpp` (arbitrary multi-block `Delta` placement via `LFTChannelMap`, channel-gather onto an open-loop `StateSpace`) — Phase 3 RC1, `examples/ex96_lft_system.cpp`. See [2026-06-24-lft-system-design.md](superpowers/specs/2026-06-24-lft-system-design.md). |
| Backstepping | `lib/BacksteppingController.h`/`.cpp` (`IController`, N-stage recursive Lyapunov design) — Phase 3 NC1, `examples/ex97_backstepping.cpp`. See [2026-06-24-nonlinear-control-trio-design.md](superpowers/specs/2026-06-24-nonlinear-control-trio-design.md). |
| Passivity-based control | `lib/PassivityBasedController.h`/`.cpp` (`IController`, PD+ energy-shaping/damping-injection regulation) — Phase 3 NC2, `examples/ex98_passivity_based.cpp`. See [2026-06-24-nonlinear-control-trio-design.md](superpowers/specs/2026-06-24-nonlinear-control-trio-design.md). |
| Direct Lyapunov redesign / CLF synthesis | `lib/CLFController.h`/`.cpp` (`IController`, Sontag's universal formula) — Phase 3 NC4, `examples/ex99_clf_controller.cpp`. See [2026-06-24-nonlinear-control-trio-design.md](superpowers/specs/2026-06-24-nonlinear-control-trio-design.md). |
| Hammerstein-Wiener models | `lib/HammersteinWienerIdentifier.h`/`.cpp` (alternating linear/nonlinear least squares; batch-ARX helper + approximate-inverse refresh for Wiener) — Phase 3 SI5, `examples/ex100_hammerstein_wiener.cpp`. See [2026-06-24-hammerstein-wiener-design.md](superpowers/specs/2026-06-24-hammerstein-wiener-design.md). |

---

## Robust Control

H2 synthesis and structured Hinf are **done** — see the "Already done" table above
(`lib/DiscreteH2.h`, `DiscreteHinf::solveStructured()`); neither ended up needing an LMI solver,
matching this section's original prediction that a Riccati shortcut / CMA-ES search would do.
What's left:

| Item | Notes |
|---|---|
| LMI solver (feasibility / cost-min / generalized eigenvalue-min) | No SDP solver exists in `lib/` today (`GradientProjectionQP` is QP, not SDP). With H2 and structured-Hinf both solved without it, the remaining motivation is purely for problems that genuinely need a convex LMI formulation (e.g. multi-objective Hinf/H2 mixed synthesis) — lower priority than originally scoped. |

General parametric-uncertainty representation (LFT) is **done** — see the "Already done" table above (`lib/LFTSystem.h`).

## Nonlinear Control

Backstepping, passivity-based control, and direct Lyapunov redesign / CLF synthesis are **done**
— see the "Already done" table above (`lib/BacksteppingController.h`, `lib/PassivityBasedController.h`,
`lib/CLFController.h`). What's left:

| Item | Notes |
|---|---|
| Nonlinear Internal Model Control | `SmithPredictor` + SOPDT/Rivera IMC cover the linear case; nonlinear IMC is a separate, smaller class. |
| Globally Linearizing Control (GLC) | Niche/rare in practice — low priority. |

## System Identification

Correlation-based identification and Hammerstein-Wiener models are **done** — see the "Already
done" table above (`lib/CorrelationID.h`, `lib/HammersteinWienerIdentifier.h`). What's left:

| Item | Notes |
|---|---|
| Maximum Likelihood / MAP identification | Statistical alternative to existing least-squares-based `RecursiveLeastSquares`/`GreyBoxEstimator`. |
| MOESP / CVA (subspace ID variants) | `lib/SubspaceID.h` only implements N4SID-style identification today. |
| NARMAX | Nonlinear parametric ID; distinct from `SINDy`'s sparse-regression approach. |

## Frequency-Domain Identification Extensions (follow-ups to Phase 4 Iteration 2)

`FreqDomainIdentifier::fitLevy` (Phase 4 Iteration 2,
[2026-06-22-frequency-domain-identification-design.md](superpowers/specs/2026-06-22-frequency-domain-identification-design.md))
ships only Levy's method, fitting a full complex frequency response (magnitude + phase) to an
arbitrary `TransferFunction`. Two related algorithms were deliberately deferred rather than
shipped as stubbed/non-functional code, per this repo's "no half-finished implementations"
rule — **note both already exist in a narrower form** (see the "Already done" table above:
`lib/VectorFitting.h`, real-pole/magnitude-only, built for `DiscreteHinf::solveMuSyn`):

Generalizing SK iteration to full complex-response fitting is **done** — see the "Already
done" table above (`lib/SKFit.h`). What's left:

| Item | Notes |
|---|---|
| Complex-conjugate-pole Vector Fitting | `VectorFitting::fitMagnitude` only places real poles, so it can't represent resonant/lightly-damped systems. A general Vector Fitting with complex-conjugate pole-pair bookkeeping and relocation logic is more robust for those cases than Levy/SK, but a materially bigger lift — its own design pass, not a quick extension. |

## Optimal Control

| Item | Notes |
|---|---|
| Dynamic programming / value iteration | No DP solver exists; would need a discretized state-space grid. |
| Dual control | Actively explores to reduce uncertainty — research-grade, niche; low priority. |
| Minimum-variance control / self-tuning regulator | `GeneralizedPredictiveControl` (GPC) is the closest existing relative but isn't a minimum-variance STR. |
| Linear-programming-based control | No LP solver exists in `lib/` (only `GradientProjectionQP` for QP). |

## Adaptive Control

| Item | Notes |
|---|---|
| Adaptive pole placement | Distinct from `MRACController`'s Lyapunov-based approach. |
| Self-tuning regulators | Pairs with minimum-variance control above. |
| Full neural-network adaptive control | `NeuralPID` is a hybrid NN-PID, not a general adaptive NN control law. |
| Reinforcement-learning-based adaptive control | `DynaController` (Dyna-style MBRL) and `CEMController` are lightweight; a full RL-policy controller is a larger, separate effort (see Deep RL below — same gap, two wishlist entries). |

## Advanced Estimation & Filtering

H-infinity filter is **done** — see the "Already done" table above (`lib/HinfFilter.h`). What's
left:

| Item | Notes |
|---|---|
| Set-membership estimation | Bounded-error estimation, structurally different from the probabilistic filters already present. |
| Particle filter variants (SIR, auxiliary PF, Rao-Blackwellized PF) | `lib/ParticleFilter.h` covers the base case only. |

## Machine Learning Integration

| Item | Notes |
|---|---|
| GP-MPC (combined) | `GaussianProcess`/`GPResidualModel` and `NonlinearMPC`/`TubeMPC` exist separately; a controller that consumes GP uncertainty directly in the MPC cost/constraints is still open (`ALGORITHM_ROADMAP_PHASE2.md` flagged this as the motivation for hybrid models — partially addressed by `HybridMPC`, but not GP-uncertainty-aware MPC specifically). |
| Deep reinforcement learning | Explicitly deferred already in `ALGORITHM_ROADMAP_PHASE2.md` ("Full RL framework... no C++ RL core needed" — Python example only, same reasoning likely applies here). |
| Direct neural-network controller architectures | Beyond `NeuralPID`'s NN-PID hybrid — a general NN control law. |

## Deployment & Real-Time Tools

*Kept here only as a marker — these are about embedded/deployment targets specifically, which
the frequency-domain-plots phase intentionally stepped outside of. Re-evaluate scope before
picking these up.*

| Item | Notes |
|---|---|
| Code generation (C/C++ from controller design) | Heavy lift; highest production value of this category per the original wishlist's own priority table. |
| Real-time profiling beyond WCET | `tools/wcet_report.py` covers worst-case execution time; finer-grained profiling is open. |
| Distributed / networked control | `ComputationalDelayWrapper` handles a single fixed delay; multi-node/networked control is a different problem. |
| Fault-tolerant control reconfiguration | `tools/fault_injector.py`/`fault_sweep.py` test fault *response*; an actively reconfiguring FTC *controller* is open. |

## Multi-Objective & Constrained Optimization

Nelder-Mead simplex is **done** — see the "Already done" table above (`lib/NelderMead.h`).
What's left:

| Item | Notes |
|---|---|
| Multi-objective (Pareto) optimization | `GeneticAlgorithm`/`ParticleSwarmOptimizer`/`DifferentialEvolution` are single-objective today. |
| General nonlinear constrained tuning | `AutoTuner`/`TunerSuite` constraint handling is currently limited to box bounds. |
