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

---

## Robust Control

| Item | Notes |
|---|---|
| LMI solver (feasibility / cost-min / generalized eigenvalue-min) | Foundational — unlocks H2, structured-Hinf, and a proper uncertainty representation below. No SDP solver exists in `lib/` today (`GradientProjectionQP` is QP, not SDP). Biggest single lift in this category. |
| H2 synthesis | Natural pairing with `DiscreteHinf`; needs the LMI solver or a Riccati-based shortcut. |
| Structured H-infinity (fixed order/structure) | Needs nonlinear/non-convex optimization over controller structure; likely reuses `AutoTuner`/`GeneticAlgorithm` cost-function pattern rather than the LMI solver. |
| General parametric-uncertainty representation (LFT) | `WorstCaseSearch`/`LyapunovRobustness` consume a perturbation already; a structured LFT *model* (vs. ad hoc sampling) is the open piece. |

## Nonlinear Control

| Item | Notes |
|---|---|
| Backstepping | Recursive Lyapunov design for strict-feedback systems; complements `DiscreteSMC`/`DiscreteADRC`. |
| Passivity-based control | Energy-shaping + damping injection; no overlap with existing classes. |
| Nonlinear Internal Model Control | `SmithPredictor` + SOPDT/Rivera IMC cover the linear case; nonlinear IMC is a separate, smaller class. |
| Direct Lyapunov redesign / CLF synthesis | Distinct from `lib/LyapunovRobustness.h` (that's robustness *analysis* of a fixed controller, not Lyapunov-based controller *synthesis*). |
| Globally Linearizing Control (GLC) | Niche/rare in practice — low priority. |

## System Identification

| Item | Notes |
|---|---|
| Maximum Likelihood / MAP identification | Statistical alternative to existing least-squares-based `RecursiveLeastSquares`/`GreyBoxEstimator`. |
| Correlation-based identification | Classical impulse-response estimation via correlation. |
| MOESP / CVA (subspace ID variants) | `lib/SubspaceID.h` only implements N4SID-style identification today. |
| NARMAX | Nonlinear parametric ID; distinct from `SINDy`'s sparse-regression approach. |
| Hammerstein-Wiener models | Structured nonlinear ID (static nonlinearity + linear dynamics); no current equivalent. |

## Frequency-Domain Identification Extensions (follow-ups to Phase 4 Iteration 2)

`FreqDomainIdentifier::fitLevy` (Phase 4 Iteration 2,
[2026-06-22-frequency-domain-identification-design.md](superpowers/specs/2026-06-22-frequency-domain-identification-design.md))
ships only Levy's method, fitting a full complex frequency response (magnitude + phase) to an
arbitrary `TransferFunction`. Two related algorithms were deliberately deferred rather than
shipped as stubbed/non-functional code, per this repo's "no half-finished implementations"
rule — **note both already exist in a narrower form** (see the "Already done" table above:
`lib/VectorFitting.h`, real-pole/magnitude-only, built for `DiscreteHinf::solveMuSyn`):

| Item | Notes |
|---|---|
| Generalize SK iteration to full complex-response fitting | `VectorFitting.h`'s SK machinery only fits a real magnitude profile to real poles. Extending it (or `fitLevy`) to iteratively reweight by `1/\|D_prev\|^2` against the *complex* response — recovering phase, not just magnitude — removes most of Levy's high-frequency bias. Reuses `fitLevy`'s linear-system-build step with iteration added around it; a natural small follow-up, not a from-scratch effort. |
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

| Item | Notes |
|---|---|
| H-infinity filter | Robust alternative to `KalmanFilter`/`EKF`/`UKF`; `DiscreteHinf` today is a controller, not a filter. |
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

| Item | Notes |
|---|---|
| Multi-objective (Pareto) optimization | `GeneticAlgorithm`/`ParticleSwarmOptimizer`/`DifferentialEvolution` are single-objective today. |
| Nelder-Mead simplex | Derivative-free, no population — different tool than the existing metaheuristics. |
| General nonlinear constrained tuning | `AutoTuner`/`TunerSuite` constraint handling is currently limited to box bounds. |

## Additional Controller Types

| Item | Notes |
|---|---|
| Resonant controllers | For AC/periodic reference tracking — distinct from `RepetitiveController`. |
| Notch filters | Vibration suppression; `lib/embedded/FixedRateFilter` is generic, not a notch-filter design tool. |
| Phase-locked loop (PLL) | Synchronization applications — no equivalent exists. |
