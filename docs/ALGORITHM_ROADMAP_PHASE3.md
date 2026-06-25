# Controller Toolbox — Algorithm Roadmap: Phase 3

**Created:** 2026-06-24.
**Status:** Planning — 9 of 32 items shipped (Phase 1 complete).
**Source:** Every item below is one of the 35 open lines in `docs/algorithm_backlog.md` (the
9 categories left after Phase 4's frequency-domain work and the Resonant/Notch/PLL controllers
shipped). 3 backlog items (`Minimum-variance control/STR`, `Adaptive pole placement`,
`Self-tuning regulators`) are merged into one design (**OC1**) since they share an identical
RLS-driven online-identification core and differ only in the control-law step. 2 backlog items
(`Reinforcement-learning-based adaptive control`, `Deep reinforcement learning`) are merged into
one design (**ML4**) for the same reason — `algorithm_backlog.md` itself flags them as "the same
gap, two wishlist entries." That leaves **32 distinct designs** below.
**Scope:** Phase 1 (foundational/quick-win) → Phase 2 (strong value, moderate effort) →
Phase 3 (bigger/specialized lifts) → Phase 4 (heavy infrastructure) → Phase 5 (niche/research-grade,
kept rather than cut, per explicit instruction).

This document follows the same per-item format as `docs/ALGORITHM_ROADMAP_PHASE2.md`: goal,
class/function sketch, reused components, effort estimate, example use case, Catch2 test plan.
Like that document, **this is a planning reference, not 32 approved specs** — each item still
gets brainstormed into its own design doc under `docs/superpowers/specs/` before being built,
the way Phase 4's frequency-domain work and the Resonant/Notch/PLL controllers were. The class
sketches here are directional (to scope effort and reuse), not committed APIs.

| ID | Name | Phase | Status |
|----|------|-------|--------|
| EF1 | H-infinity Filter | 1 | Done |
| RC1 | General LFT Uncertainty Representation | 1 | Done |
| NC1 | Backstepping | 1 | Done |
| NC2 | Passivity-Based Control | 1 | Done |
| NC4 | CLF Synthesis / Direct Lyapunov Redesign | 1 | Done |
| SI5 | Hammerstein-Wiener Identification | 1 | Done |
| SI2 | Correlation-Based Identification | 1 | Done |
| FD1 | Generalized SK Iteration (complex-response fitting) | 1 | Done |
| MO2 | Nelder-Mead Simplex | 1 | Done |
| OC1 | Self-Tuning Regulator (merged) | 2 | Open |
| SI1 | Maximum Likelihood / MAP Identification | 2 | Open |
| EF2 | Set-Membership Estimation | 2 | Open |
| EF3 | Particle Filter Variants | 2 | Open |
| MO1 | Multi-Objective (Pareto) Optimization | 2 | Open |
| MO3 | General Nonlinear Constrained Tuning | 2 | Open |
| DT4 | Fault-Tolerant Control Reconfiguration | 2 | Open |
| ML1 | NN Controller Core (direct NN architectures) | 3 | Open |
| ML2 | NN-Adaptive Control (depends on ML1) | 3 | Open |
| SI3 | MOESP / CVA Subspace ID Variants | 3 | Open |
| SI4 | NARMAX | 3 | Open |
| FD2 | Complex-Conjugate-Pole Vector Fitting | 3 | Open |
| NC3 | Nonlinear Internal Model Control | 3 | Open |
| ML3 | GP-MPC | 3 | Open |
| RC2 | LMI Solver | 4 | Open |
| OC2 | Dynamic Programming / Value Iteration | 4 | Open |
| OC4 | Linear-Programming-Based Control | 4 | Open |
| DT1 | Code Generation | 4 | Open |
| DT2 | Real-Time Profiling Beyond WCET | 4 | Open |
| DT3 | Distributed / Networked Control | 4 | Open |
| NC5 | Globally Linearizing Control | 5 | Open |
| OC3 | Dual Control | 5 | Open |
| ML4 | RL-Based Control (merged) | 5 | Open |

---

## Motivation

Phase 4 (frequency-domain plots, frequency-domain identification, Resonant/Notch/PLL
controllers) closed out the backlog items with no real architectural risk — each slotted
cleanly into an existing pattern (`SystemAnalysis`, `FreqDomainIdentifier`, `IController`).
What's left in `docs/algorithm_backlog.md` is a longer tail: 9 categories that don't share one
dependency chain the way Phase 2's DAE→grey-box→hybrid-model work did. Phase 3 (this document)
is the first attempt to sequence that tail by **value/ROI** rather than by category, on the
premise that the highest-leverage next steps are the ones that either (a) reuse a large chunk
of an existing class almost for free (`EF1` reusing `DiscreteHinf`'s Riccati machinery; `MO2`
reusing `AutoTuner`'s `CostFn`/`TunerResult` contract) or (b) fill a real practitioner-facing gap
that recurs across the case-study roster (`SI5` Hammerstein-Wiener for valve/actuator
nonlinearities; `NC1`/`NC2`/`NC4` for strict-feedback and energy-shaping nonlinear plants this
toolbox can't yet handle without full feedback linearization).

The lower-value tail (Phase 4/5 here) isn't cut, per explicit instruction — but it's sequenced
last because its main historical justification has weakened: the LMI solver (`RC2`) was
originally scoped as a prerequisite for H2 synthesis and structured Hinf, and both shipped via
a Riccati shortcut and a CMA-ES search instead (see `algorithm_backlog.md`'s Robust Control
section). What's left needing `RC2` is narrower — multi-objective Hinf/H2 mixed synthesis,
not the two items that originally justified the line item.

---

## Dependency Graph

```
EF1 (Hinf Filter)              — independent, reuses DiscreteHinf's two-Riccati pattern
RC1 (LFT Representation)       — independent, reuses MuAnalysis's UncertaintyStructure
NC1/NC2/NC4 (Backstepping /    — independent of each other, share the DriftFn/GainFn callback
  Passivity / CLF)               pattern from FeedbackLinearisation
SI5 (Hammerstein-Wiener)       — independent, reuses RecursiveLeastSquares for the linear sub-step
SI2 (Correlation ID)           — independent, smallest item in the roadmap
FD1 (Generalized SK)           — independent, wraps fitLevy's existing linear-system-build step
MO2 (Nelder-Mead)              — independent, drops into AutoTuner's CostFn/TunerResult contract
  │
  └─► MO3 (Constrained Tuning) — wraps ANY CostFn-based optimizer (AutoTuner, GA, PSO, DE,
                                  Nelder-Mead) in a penalty loop; benefits from MO2 existing but
                                  doesn't require it
MO1 (NSGA-II)                  — extends GeneticAlgorithm's operators with non-dominated sorting

OC1 (Self-Tuning Regulator)    — reuses RecursiveLeastSquares wholesale; independent
SI1 (MLE/MAP ID)               — reuses AutoTuner as its optimizer; independent of OC1
EF2 (Set-Membership Est.)      — independent, optionally borrows MovingHorizonEstimator's
                                  Hildreth-projection machinery as a v2 backend
EF3 (Particle Filter Variants) — extends ParticleFilter by inheritance; independent

DT4 (FTC Reconfiguration)      — reuses ControllerStack (Supervisory mode) + KalmanFilter's
                                  existing mismatchDetected()/mismatchScore() (Phase 2's D1)

ML1 (NN Controller Core)
  │
  └─► ML2 (NN-Adaptive Control) — needs ML1's forward-pass primitive before adding the
                                   Lyapunov-stable online weight-adaptation law
ML3 (GP-MPC)                   — reuses GaussianProcess/GPResidualModel + NonlinearMPC's
                                  DiscreteDynamics rollout hook; independent of ML1/ML2

SI3 (MOESP/CVA)                — extends SubspaceID's existing Hankel/LQ/SVD pipeline
SI4 (NARMAX)                   — independent, distinct from SINDy's sparse-regression approach
FD2 (Complex-pole VectorFit)   — extends VectorFitting; bigger lift than FD1, no dependency on it
NC3 (Nonlinear IMC)            — independent, smaller class alongside SmithPredictor

RC2 (LMI Solver)               — independent; nothing in Phase 1-3 requires it
OC2 (DP/Value Iteration)       — independent
OC4 (LP-Based Control)         — independent; could reuse GradientProjectionQP's box-constraint
                                  pattern as a starting point for a simplex/active-set LP solver
DT1 (Code Generation)          — independent, heaviest lift in the document
DT2 (RT Profiling)             — independent, extends tools/wcet_report.py
DT3 (Distributed Control)      — independent, extends ComputationalDelayWrapper's single-delay
                                  model to multi-node

NC5 / OC3 / ML4 (Phase 5)      — independent of everything above; kept for completeness
```

**Recommended order within Phase 1** (all independent, so this is about quick wins surfacing
first): SI2 → MO2 → FD1 → EF1 → RC1 → NC1 → NC2 → NC4 → SI5.

---

## Phase 1: Foundational / High-Value, Low-to-Moderate Effort

### EF1 — H-Infinity Filter

**Goal:** Discrete-time Hinf filter — the *estimation* dual of `DiscreteHinf`'s two-Riccati
controller synthesis. Bounds the worst-case ratio of estimation-error energy to disturbance/
noise energy, instead of assuming Gaussian noise the way `KalmanFilter` does. Today `DiscreteHinf`
is a controller only; there is no Hinf-optimal filter anywhere in `lib/`.

```cpp
struct HinfFilterParams {
    double gammaInit = 10.0;
    double gammaTol   = 1e-3;
    int    maxIter    = 30;
};

struct HinfFilterResult {
    bool   feasible      = false;
    double achievedGamma = 0.0;
    Eigen::MatrixXd L;   // filter gain
    Eigen::MatrixXd P;   // Riccati solution
    double Ts = 0.0;
};

class HinfFilter {
public:
    explicit HinfFilter(const HinfFilterResult& result);
    static HinfFilterResult solve(const StateSpace& plant,
                                   const Eigen::MatrixXd& Qw, const Eigen::MatrixXd& Rv,
                                   const HinfFilterParams& params = {});
    void predict(const Eigen::VectorXd& u);
    void update(const Eigen::VectorXd& y);
    const Eigen::VectorXd& state() const;
};
```

**Reused components:** `DiscreteHinf::solve()`'s gamma-bisection-over-DARE driver loop (the
filtering Riccati replaces the control Riccati, but the bisection structure is a near-direct
port); `DiscreteLQR::solveDARE` for the per-gamma Riccati solve; `KalmanFilter`'s
`predict()`/`update()` method shape for a familiar runtime API.

**Effort estimate:** ~350 lines (struct + bisection + DARE wiring + binding + tests) — most of
the control-flow plumbing already exists in `DiscreteHinf::solve()`.

**Example use case:** A vibration sensor subject to bounded but non-Gaussian noise (impact
disturbances). `KalmanFilter`'s Gaussian assumption breaks down; `HinfFilter` gives a guaranteed
worst-case error bound regardless of the noise distribution — valuable for a safety case.

**Catch2 test plan (`[hinf_filter]`):**
1. Known plant + bounded adversarial disturbance — achieved gamma correctly bounds the
   worst-case error-energy ratio under a simulated worst-case disturbance sequence.
2. Comparison against `KalmanFilter` on Gaussian noise — `HinfFilter` is conservative but
   stable, RMS error within an expected factor of the KF's.
3. Infeasible gamma (too tight) — `solve()` returns `feasible=false`, not a garbage result.

---

### RC1 — General LFT Uncertainty Representation

**Goal:** Generalize `MuAnalysis.h`'s `peakMu()` — which hardcodes the single canonical
"`M = sigma_rel * T`" multiplicative-output-uncertainty loop — into an `LFTSystem` that accepts
*arbitrary* placement of one or more `Delta` blocks against a `GeneralisedPlant`'s `w`/`z`
channels. This is the "real LFT model, not ad hoc sampling" gap `algorithm_backlog.md` flags
under Robust Control.

```cpp
struct LFTChannelMap {
    std::vector<int> deltaOutputToWIndex;   // Delta output i feeds into plant input w[index]
    std::vector<int> deltaInputFromZIndex;  // Delta input i comes from plant output z[index]
};

class LFTSystem {
public:
    LFTSystem(const GeneralisedPlant& P, const UncertaintyStructure& struc,
              const LFTChannelMap& map);
    std::vector<Eigen::MatrixXcd> closedLoopFreqResponse(const std::vector<double>& omegas) const;
    PeakMuResult peakMu(double sigma_rel = 1.0, int freq_points = 200,
                         double omega_min = 1e-2) const;
};
```

**Reused components:** `GeneralisedPlant` (`DiscreteHinf.h`) for `P`; `UncertaintyStructure`/
`computeMu()` (`MuAnalysis.h`) for `Delta` itself — no new uncertainty-block machinery needed,
only the general interconnection wiring around it.

**Effort estimate:** ~300 lines (channel-mapping bookkeeping + general `M`-builder + 3 tests) —
moderate; the heavy lifting (`computeMu`, frequency-response grids) already exists.

**Example use case:** A plant with simultaneous multiplicative input uncertainty *and* additive
output uncertainty (two `Delta` blocks at two different loop locations) — today's `peakMu()`
can only represent the single canonical case.

**Catch2 test plan (`[lft_system]`):**
1. Degenerate single-block case — `LFTSystem` reproduces existing `peakMu()`'s result exactly.
2. Two simultaneous blocks at different loop locations — closed-loop frequency response matches
   a hand-derived formula for a simple 2x2 case.
3. Mis-sized channel map — throws `std::invalid_argument`.

---

### NC1 — Backstepping

**Goal:** Recursive Lyapunov design for strict-feedback nonlinear systems
(`x1' = f1(x1) + g1(x1)*x2`, `x2' = f2(x1,x2) + g2(x1,x2)*u`, extendable via recursive virtual-
control construction). Complements `DiscreteSMC`/`DiscreteADRC` and handles relative-degree > 1
structures that `FeedbackLinearisationController` (relative-degree-1 only) cannot.

```cpp
struct BacksteppingParams {
    std::vector<double> k_gains;   // one stabilizing gain per recursion stage
    double uMin = -1e9, uMax = 1e9;
};

class BacksteppingController : public IController {
public:
    using DriftFn = std::function<double(const Eigen::VectorXd& x, int stage)>;  // f_i(x)
    using GainFn  = std::function<double(const Eigen::VectorXd& x, int stage)>;  // g_i(x)
    BacksteppingController(std::vector<DriftFn> f, std::vector<GainFn> g,
                            const BacksteppingParams& params, double Ts);
    double compute(double error) override;     // error = r - x1
    void setState(const Eigen::VectorXd& x);
    void reset() override;
};
```

**Reused components:** `FeedbackLinearisationController`'s `DriftFn`/`GainFn` callback pattern,
generalized to per-stage; `LyapunovRobustness` for optional post-hoc verification that the
resulting closed loop's Lyapunov function decreases.

**Effort estimate:** ~300 lines (recursive virtual-control loop + 3 tests).

**Example use case:** A two-link robotic arm with non-affine joint coupling — backstepping
handles the strict-feedback structure (relative degree > 1 with intermediate states) that flat
feedback linearization can't.

**Catch2 test plan (`[backstepping]`):**
1. 2nd-order strict-feedback system with a known analytic backstepping law — tracking error
   converges to zero, matching the hand-derived control law's output.
2. Lyapunov function verified numerically non-increasing along a simulated trajectory.
3. Actuator saturation (`uMin`/`uMax`) — output is hard-clamped, no internal windup.

---

### NC2 — Passivity-Based Control

**Goal:** Energy-shaping + damping-injection control for port-Hamiltonian-representable systems
(known mass/inertia + potential energy) — the canonical alternative to backstepping when a
natural storage function exists. `algorithm_backlog.md` correctly notes "no overlap with
existing classes"; this is built from scratch but follows the same callback-pattern convention
as `FeedbackLinearisationController`/`BacksteppingController` for consistency.

```cpp
struct PBCParams {
    Eigen::MatrixXd Kd;       // damping injection gain
    double uMin = -1e9, uMax = 1e9;
};

class PassivityBasedController : public IController {
public:
    using MassMatrixFn    = std::function<Eigen::MatrixXd(const Eigen::VectorXd& q)>;
    using PotentialGradFn = std::function<Eigen::VectorXd(const Eigen::VectorXd& q)>;
    using CoriolisFn      = std::function<Eigen::MatrixXd(const Eigen::VectorXd& q,
                                                            const Eigen::VectorXd& qdot)>;
    PassivityBasedController(MassMatrixFn M, PotentialGradFn dV, CoriolisFn C,
                              const PBCParams& params, double Ts);
    Eigen::VectorXd computeVec(const Eigen::VectorXd& q_qdot_error) override;
    void setDesired(const Eigen::VectorXd& q_d);
    void reset() override;
};
```

**Reused components:** `IController::computeVec`'s MIMO convention; no internal class reuse
(this is genuinely new), but the `MassMatrixFn`-style callback shape mirrors `NC1`/`NC4` for a
consistent "physics-callback" pattern across the new nonlinear-control trio.

**Effort estimate:** ~280 lines (energy-shaping law + damping injection + 3 tests).

**Example use case:** A 2-DOF manipulator regulated to a desired joint configuration despite
unmodeled friction — PBC's energy argument guarantees stability without linearizing, valuable
when the Lagrangian structure (`M`, `C`, `g`) is known but exact linearization is brittle.

**Catch2 test plan (`[passivity_based]`):**
1. Single-pendulum regulation — converges to the desired angle; the shaped total energy
   (kinetic + potential) is non-increasing.
2. Closed-loop passivity verified via a storage-function check across a trajectory.
3. Mass matrix singular at a boundary configuration — graceful handling (NaN guard, hold-last).

---

### NC4 — CLF Synthesis / Direct Lyapunov Redesign

**Goal:** Given a candidate Control Lyapunov Function `V(x)` and its Lie derivatives, synthesize
a stabilizing law via Sontag's universal formula or a CLF-QP
(`min ||u|| s.t. LfV + LgV*u <= -alpha*V`) — controller *synthesis*, distinct from
`LyapunovRobustness` which only *analyzes* a fixed controller's robustness.

```cpp
struct CLFParams {
    double alpha = 1.0;        // decay rate
    double uMin = -1e9, uMax = 1e9;
    bool   useQP = true;       // false = closed-form Sontag formula
};

class CLFController : public IController {
public:
    using LfVFn = std::function<double(const Eigen::VectorXd& x)>;  // drift Lie derivative
    using LgVFn = std::function<double(const Eigen::VectorXd& x)>;  // control Lie derivative
    CLFController(LfVFn LfV, LgVFn LgV, const CLFParams& params, double Ts);
    double compute(double error) override;
    void setState(const Eigen::VectorXd& x);
};
```

**Reused components:** `GradientProjectionQP` for the CLF-QP mode's box-constrained min-norm
solve (the MIMO generalization of the path; SISO collapses to closed form); kept consistent
with `LyapunovRobustness`'s `V`-function conventions so a synthesized controller can be handed
straight to `LyapunovRobustness` afterward for independent verification (synthesis → analysis
handoff).

**Effort estimate:** ~250 lines (Sontag formula + QP path + 3 tests).

**Example use case:** Synthesizing a stabilizing law directly from a candidate quadratic
`V = x'Px` (e.g. from a linearization's Lyapunov equation) for a mildly nonlinear plant where
full backstepping structure isn't available but a CLF candidate is known.

**Catch2 test plan (`[clf_controller]`):**
1. Known CLF for a scalar nonlinear system — Sontag-formula output matches the hand-derived
   closed form.
2. QP mode and Sontag-formula mode agree on unconstrained cases.
3. `LfV` positive and `LgV = 0` (uncontrollable direction) — flags infeasible rather than
   producing a nonsense `u`.

---

### SI5 — Hammerstein-Wiener Model Identification

**Goal:** Structured nonlinear identification for Hammerstein (static input nonlinearity →
linear dynamics) and Wiener (linear dynamics → static output nonlinearity) model classes, fit
via alternating linear/nonlinear least squares. Fills a gap the backlog calls out directly: "no
current equivalent," despite this structure being extremely common (valves, actuator
deadzone/saturation, sensor saturation).

```cpp
struct HammersteinWienerParams {
    int    na, nb;             // linear ARX orders
    int    nl_degree = 3;      // polynomial degree for the static nonlinearity
    int    max_iter  = 20;
    double tol        = 1e-6;
};

struct HammersteinWienerResult {
    Eigen::VectorXd nl_input_coeffs;    // Hammerstein static map
    Eigen::VectorXd nl_output_coeffs;   // Wiener static map (empty if Hammerstein-only)
    TransferFunction linear_part;
    bool converged;
    int  iters;
};

class HammersteinWienerIdentifier {
public:
    static HammersteinWienerResult fitHammerstein(const Eigen::VectorXd& u, const Eigen::VectorXd& y,
                                                    double Ts, const HammersteinWienerParams& params = {});
    static HammersteinWienerResult fitWiener(const Eigen::VectorXd& u, const Eigen::VectorXd& y,
                                               double Ts, const HammersteinWienerParams& params = {});
};
```

**Reused components:** `RecursiveLeastSquares`'s ARX-fitting machinery (run in batch mode each
outer iteration) for the linear sub-step; the polynomial-basis pattern from `SINDy`'s library
functions for the static-nonlinearity basis.

**Effort estimate:** ~350 lines (alternating-LS outer loop + polynomial basis + 3 tests) — most
of the heavy lifting (ARX fit) already exists.

**Example use case:** A valve with input saturation/deadzone (Hammerstein) followed by linear
actuator dynamics, or a sensor with linear dynamics followed by a saturating output nonlinearity
(Wiener) — both extremely common in real industrial loops.

**Catch2 test plan (`[hammerstein_wiener]`):**
1. Synthetic Hammerstein system (known cubic input nonlinearity + known 2nd-order linear part)
   — both recovered within tolerance.
2. Synthetic Wiener system, symmetric test.
3. Pure-linear system (nonlinearity = identity) — alternating fit converges to a near-identity
   nonlinearity, doesn't overfit.

---

### SI2 — Correlation-Based Identification

**Goal:** Classical non-parametric impulse-response estimation via cross-correlation, driven by
a PRBS input: `g_hat(k) = R_uy(k) / R_uu(0)` for near-white input. The simplest item in this
document — a standard first step in classical system-ID workflow that currently has no home in
this toolbox.

```cpp
struct CorrelationIDParams {
    int  max_lag;
    bool whiten_input = false;
};

struct CorrelationIDResult {
    Eigen::VectorXd impulse_response;
    Eigen::VectorXd autocorr_u;
    Eigen::VectorXd crosscorr_uy;
};

CorrelationIDResult correlationID(const Eigen::VectorXd& u, const Eigen::VectorXd& y,
                                   double Ts, const CorrelationIDParams& params);
Eigen::VectorXd generatePRBS(int length, int n_bits, unsigned seed = 42);
```

**Reused components:** None required — intentionally the simplest classical method in the set
(direct time-domain correlation sums). Could optionally reuse `FreqDomainIdentifier`'s FFT
utilities for a frequency-domain variant, but that's a v2 extension, not required for v1.

**Effort estimate:** ~150 lines (correlation sums + PRBS generator + 2 tests) — the smallest
item in the entire roadmap.

**Example use case:** A quick non-parametric "sanity check" impulse response before committing
to a parametric structure (ARX/state-space) — standard first step in a classical ID workflow.

**Catch2 test plan (`[correlation_id]`):**
1. Known linear system driven by PRBS — recovered impulse response matches the analytic one
   within the noise floor.
2. PRBS generator — verified near-white autocorrelation (single peak at lag 0).
3. Colored input without whitening — result is visibly biased (a regression test documenting
   the known limitation, not a bug).

---

### FD1 — Generalize SK Iteration to Full Complex-Response Fitting

**Goal:** `algorithm_backlog.md`'s own assessment: extend `VectorFitting`'s SK machinery (or
`FreqDomainIdentifier::fitLevy`) to iteratively reweight against the *complex* frequency response
(magnitude + phase), removing most of Levy's high-frequency bias — "a natural small follow-up,
not a from-scratch effort."

```cpp
// Final home (FreqDomainIdentifier.h vs VectorFitting.h) is a brainstorming-time decision,
// not fixed here.
struct SKFitResult {
    TransferFunction model;
    std::vector<double> iterCost;   // cost per SK iteration, for convergence diagnostics
    bool converged;
};

SKFitResult fitSK(const std::vector<double>& omega,
                   const std::vector<std::complex<double>>& response,
                   int n_poles, double Ts, int max_iter = 20, double tol = 1e-4);
```

**Reused components:** `fitLevy`'s linear-system-build step (`FreqDomainIdentifier.h`), wrapped
in an outer SK reweighting loop; `VectorFitting`'s existing SK convergence-check pattern.

**Effort estimate:** ~150 lines (outer iteration loop wrapping existing `fitLevy` machinery +
2 tests) — the cheapest item in Phase 1 besides `SI2`.

**Example use case:** A lightly-damped resonance where Levy's one-shot fit shows visible
high-frequency bias; SK reweighting tightens the fit without changing the user-facing API shape.

**Catch2 test plan (`[sk_complex_fit]`):**
1. Synthetic complex response with known poles — SK-reweighted fit has lower error than
   one-shot `fitLevy` on the same data.
2. Convergence — `iterCost` is monotonically non-increasing.
3. Already-good `fitLevy` result (low-order, low-damping) — SK iteration doesn't make it worse.

---

### MO2 — Nelder-Mead Simplex

**Goal:** Derivative-free, non-population direct-search optimizer (reflect/expand/contract/
shrink) — a lighter-weight alternative to CMA-ES/GA/PSO/DE for low-dimensional (`n < 10`) quick
tuning where population overhead isn't justified.

```cpp
struct NelderMeadParams {
    int    n_dim;
    int    max_iter = 500;
    double tol       = 1e-8;
    double alpha = 1.0, gamma = 2.0, rho = 0.5, sigma = 0.5;
};

class NelderMead {
public:
    explicit NelderMead(const NelderMeadParams& p);
    TunerResult optimize(const CostFn& cost, const Eigen::VectorXd& x0);
};
```

**Reused components:** `AutoTuner`'s `CostFn`/`TunerResult` types directly (zero new types) —
drops into the exact same call site as `GeneticAlgorithm`/`ParticleSwarmOptimizer`/
`DifferentialEvolution`/`AutoTuner`.

**Effort estimate:** ~200 lines (simplex operations + 2 tests).

**Example use case:** A quick 2-3 parameter PID/lead-lag retune where CMA-ES's population
overhead (and its `sigma0`/seed tuning) is unnecessary ceremony — Nelder-Mead needs only an
initial point.

**Catch2 test plan (`[nelder_mead]`):**
1. Rosenbrock function (2D) — converges to the known minimum within tolerance.
2. Quadratic bowl — converges in fewer evaluations than CMA-ES on the same problem (documents
   the "why use this" case).
3. Degenerate simplex collapse — detected and restarted, doesn't silently return a bad point.

---

## Phase 2: Strong Value, Moderate Effort

### OC1 — Self-Tuning Regulator (merged)

**Goal:** Merges 3 backlog line items — Optimal Control's "Minimum-variance control/STR" and
Adaptive Control's "Adaptive pole placement" + "Self-tuning regulators" — into one class. All
three share an identical RLS-driven online-identification core and differ only in the
control-law step, so this is one online identifier with two selectable control-law modes:
minimum-variance (classic Astrom direct-cancellation STR) or pole-placement (Diophantine-equation
solve for desired closed-loop poles).

```cpp
enum class STRMode { MinimumVariance, PolePlacement };

struct STRParams {
    int    na, nb, d;                // plant orders + dead time
    STRMode mode = STRMode::MinimumVariance;
    Eigen::VectorXd desired_poles;    // PolePlacement mode only
    double lambda = 0.98;             // RLS forgetting factor
    double uMin = -1e9, uMax = 1e9;
};

class SelfTuningRegulator : public IController {
public:
    explicit SelfTuningRegulator(const STRParams& params, double Ts);
    double compute(double error) override;   // drives RLS update + selected control law
    void reset() override;
    const Eigen::VectorXd& estimatedNumerator() const;
    const Eigen::VectorXd& estimatedDenominator() const;
};
```

**Reused components:** `RecursiveLeastSquares` wholesale for identification (`update()`,
`numerator()`/`denominator()`); the pole-placement mode's Diophantine solve is new but small
(a linear solve over a Sylvester-like matrix via `Eigen::MatrixXd::solve`, no new dependency).

**Effort estimate:** ~400 lines total (RLS wiring + 2 control-law modes + Diophantine solve +
4 tests) — larger than a single-mode item, but smaller than building 3 separate near-duplicate
classes, which is the entire point of the merge.

**Example use case:** A plant with slowly-changing dynamics (e.g. seasonal HVAC load) where a
fixed-gain controller degrades over time; the STR re-identifies online and updates the control
law every step, with no manual re-tuning required.

**Catch2 test plan (`[self_tuning_regulator]`):**
1. Known ARX plant, `MinimumVariance` mode — converges to the analytic Astrom minimum-variance
   law.
2. Known ARX plant, `PolePlacement` mode — closed-loop poles converge to `desired_poles` within
   tolerance.
3. Plant parameter step-change mid-run — STR re-converges within N steps (true adaptation, not
   just initial identification).
4. Non-identifiable input (constant `r`) — RLS covariance doesn't blow up, matching
   `RecursiveLeastSquares`'s existing numerical-safety contract.

---

### SI1 — Maximum Likelihood / MAP Identification

**Goal:** Statistical alternative to `RecursiveLeastSquares`/`GreyBoxEstimator`'s pure
least-squares cost — maximize log-likelihood under an assumed noise model (Gaussian by default),
optionally with a Gaussian prior on parameters (MAP). Reduces to ridge-regularized LS in the
Gaussian-Gaussian case but generalizes to non-Gaussian noise or heavier-tailed priors.

```cpp
struct MLEParams {
    int na, nb;
    Eigen::VectorXd prior_mean;   // empty = no prior (pure MLE)
    Eigen::MatrixXd prior_cov;    // empty = no prior
    int    max_iter = 100;
    double tol       = 1e-8;
};

struct MLEResult {
    Eigen::VectorXd theta;
    Eigen::MatrixXd covariance;    // asymptotic parameter covariance (inverse Fisher info)
    double logLikelihood;
    bool   converged;
};

class MLEIdentifier {
public:
    static MLEResult fit(const Eigen::VectorXd& u, const Eigen::VectorXd& y,
                          double Ts, const MLEParams& params);
};
```

**Reused components:** `AutoTuner`'s CMA-ES (`CostFn`/`tune()`) as the optimizer for the
(potentially non-convex, non-Gaussian-noise) log-likelihood — no new optimizer needed, just
`cost = -logLikelihood`; `RecursiveLeastSquares`'s ARX residual computation as the likelihood's
inner data term.

**Effort estimate:** ~250 lines (likelihood function + `AutoTuner` wiring + Fisher-information
covariance + 3 tests).

**Example use case:** Identification under known non-Gaussian measurement noise (e.g. a
quantizing sensor modeled as uniform rather than Gaussian noise) — MLE with the correct noise
model gives a less biased estimate than plain least squares.

**Catch2 test plan (`[mle_identification]`):**
1. Gaussian noise, no prior — matches `RecursiveLeastSquares`'s batch LS result (MLE under
   Gaussian noise = LS).
2. With an informative prior (MAP) — result is pulled toward `prior_mean` by the expected ridge
   amount relative to pure MLE.
3. Non-Gaussian (e.g. Laplace) noise — MLE outperforms LS on a synthetic outlier-heavy dataset.

---

### EF2 — Set-Membership Estimation

**Goal:** Bounded-error state estimation: given known noise *bounds* (not a probability
distribution), maintain a guaranteed feasible set (ellipsoidal, for tractable propagation)
containing the true state, recursively intersected with each measurement's consistency set.
Structurally distinct from every probabilistic filter already in `lib/`.

```cpp
struct SetMembershipParams {
    double w_bound;    // ||process noise||_inf <= w_bound
    double v_bound;    // ||measurement noise||_inf <= v_bound
};

class SetMembershipEstimator {
public:
    SetMembershipEstimator(const StateSpace& plant, const SetMembershipParams& params,
                            const Eigen::VectorXd& x0_center, const Eigen::MatrixXd& E0_shape);
    void predict(const Eigen::VectorXd& u);
    void update(const Eigen::VectorXd& y);
    const Eigen::VectorXd& centerEstimate() const;
    const Eigen::MatrixXd& ellipsoidShape() const;   // {x : (x-c)'E^-1(x-c) <= 1}
    bool isConsistent() const;                        // false if intersection became empty
};
```

**Reused components:** `KalmanFilter`'s `predict()`/`update()` naming convention for API
familiarity; `MovingHorizonEstimator`'s existing polytopic-constraint Hildreth-projection
machinery as a possible v2 backend if ellipsoidal propagation proves too conservative for a
given case study (not required for v1).

**Effort estimate:** ~300 lines (ellipsoid propagation + intersection via Fogel-Huang/Schweppe
bounding + 3 tests).

**Example use case:** A sensor with a hard calibration spec (±0.5% bounded error, not Gaussian)
where a guaranteed feasible set is more meaningful to a safety case than a Kalman filter's
probabilistic confidence interval.

**Catch2 test plan (`[set_membership]`):**
1. Known bounded noise — the true state stays inside the returned ellipsoid at every step (the
   core guarantee).
2. Comparison against `KalmanFilter` — the set-membership ellipsoid is conservative but never
   excludes the true state, while the KF's confidence interval occasionally does under
   non-Gaussian noise.
3. Inconsistent measurement (outside all bounds) — `isConsistent()` correctly flags it instead
   of silently producing a wrong estimate.

---

### EF3 — Particle Filter Variants

**Goal:** Extend `ParticleFilter.h` beyond the existing bootstrap/SIR baseline with (a) an
auxiliary particle filter (look-ahead resampling using a cheap proxy for next-step likelihood)
and (b) a Rao-Blackwellized PF (analytically marginalizing any linear-Gaussian substate via an
embedded `KalmanFilter` per particle).

```cpp
enum class PFVariant { Bootstrap, Auxiliary, RaoBlackwellized };

struct ParticleFilterParamsV2 : public ParticleFilterParams {
    PFVariant variant = PFVariant::Bootstrap;
    std::vector<int> linear_state_indices;   // RaoBlackwellized only
};

class ParticleFilterV2 : public ParticleFilter {
public:
    ParticleFilterV2(const ParticleFilterParamsV2& p, int n_states, int n_meas,
                      ParticleFn f, ParticleMeasFn h);
    // predict()/update()/step() inherited; variant dispatch happens internally
};
```

**Reused components:** `ParticleFilter` as the literal base class (Bootstrap mode falls
straight through, zero duplication); `KalmanFilter` embedded per-particle for the
Rao-Blackwellized linear substate.

**Effort estimate:** ~350 lines (auxiliary look-ahead weighting + per-particle embedded KF for
RB-PF + 3 tests) — moderate, since the bootstrap baseline is inherited, not rewritten.

**Example use case:** Target tracking with nonlinear bearing-only measurements (needs particles)
but linear-Gaussian velocity dynamics — RB-PF marginalizes velocity analytically, needing far
fewer particles than full bootstrap PF for equal accuracy.

**Catch2 test plan (`[particle_filter_variants]`):**
1. Bootstrap mode — numerically identical to the existing `ParticleFilter` (regression,
   confirms zero-duplication inheritance).
2. Auxiliary PF on a problem with informative look-ahead — lower effective-sample-size variance
   than bootstrap at equal particle count.
3. Rao-Blackwellized PF on a mixed linear/nonlinear system — matches a hand-coded analytic
   marginal-likelihood baseline, outperforms bootstrap at low particle counts.

---

### MO1 — Multi-Objective (Pareto) Optimization

**Goal:** NSGA-II-style multi-objective extension of `GeneticAlgorithm` (the population-based
metaheuristic most naturally extended via non-dominated sorting), returning a Pareto front
rather than a single best point. `GeneticAlgorithm`/`ParticleSwarmOptimizer`/
`DifferentialEvolution` are single-objective today.

```cpp
struct ParetoResult {
    Eigen::MatrixXd front_params;       // one row per non-dominated solution
    Eigen::MatrixXd front_objectives;   // corresponding objective vectors
    int nGens;
};

struct NSGA2Params {
    int n_dim, n_objectives;
    int population = 100, max_gen = 200;
    double crossover = 0.9, mutation = 0.1;
    unsigned seed = 42;
};

using MultiCostFn = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

class NSGA2 {
public:
    explicit NSGA2(const NSGA2Params& p);
    ParetoResult optimize(const MultiCostFn& cost);
};
```

**Reused components:** `GeneticAlgorithm`'s tournament-selection/BLX-alpha-crossover/elitism
operators, unchanged — only the selection *criterion* changes (non-dominated rank + crowding
distance replacing single-objective fitness comparison, the standard NSGA-II modification).

**Effort estimate:** ~400 lines (non-dominated sorting + crowding distance + operator reuse +
3 tests).

**Example use case:** Tuning a PID for both settling time *and* control effort simultaneously —
`TunerSuite::makeISECost`/`makeITAECost` give single-objective costs today; `NSGA2` returns the
actual tradeoff curve instead of forcing a weighted-sum compromise upfront.

**Catch2 test plan (`[nsga2]`):**
1. Classic 2-objective benchmark (e.g. ZDT1) — recovered front matches the known analytic shape.
2. Front diversity — crowding distance keeps solutions spread, not clustered.
3. Degenerate single-objective case (`n_objectives=1`) — equivalent-quality result to
   `GeneticAlgorithm::optimize` on the same problem.

---

### MO3 — General Nonlinear Constrained Tuning

**Goal:** Extend `TunerSuite`/`AutoTuner`'s box-bounds-only constraint handling to general
nonlinear inequality constraints `g(theta) <= 0` (e.g. "closed-loop must remain stable",
"overshoot <= 10%") via an augmented-Lagrangian/exterior-penalty wrapper around the existing
`CostFn` contract — no change needed inside any optimizer, since penalty methods only transform
the cost function passed in.

```cpp
struct ConstrainedTuneParams {
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> constraints;  // feasible iff all <= 0
    double penalty_init = 10.0, penalty_growth = 10.0;
    int    outer_iters   = 5;
};

TunerResult tuneConstrained(std::function<TunerResult(const CostFn&, const Eigen::VectorXd&)> optimizerRun,
                             const CostFn& objective,
                             const ConstrainedTuneParams& params,
                             const Eigen::VectorXd& x0);
```

**Reused components:** Every existing metaheuristic's `CostFn`/`TunerResult` contract directly
— this is a wrapper, not a new optimizer, so it composes with `AutoTuner`, `GeneticAlgorithm`,
`ParticleSwarmOptimizer`, `DifferentialEvolution`, and (once built) `NelderMead`/`NSGA2` with
zero changes to any of them.

**Effort estimate:** ~200 lines (penalty-growth outer loop + 2 tests) — small, pure composition.

**Example use case:** Tuning an MPC's `rho_y`/`rho_u` weights to minimize tracking error subject
to a hard constraint that the closed-loop spectral radius stay below 1 (via existing
`SystemAnalysis` utilities) — `TunerSuite::optimise` only supports box bounds on the parameters
themselves today, not derived closed-loop properties.

**Catch2 test plan (`[constrained_tuning]`):**
1. Constrained quadratic with a known analytic optimum — penalty method converges to the
   constrained optimum, not the unconstrained one.
2. Infeasible initial point — penalty growth still drives the search into the feasible region.
3. Wraps `AutoTuner` and `GeneticAlgorithm` interchangeably — same constrained problem gives
   consistent results from both backends.

---

### DT4 — Fault-Tolerant Control Reconfiguration

**Goal:** An actively-reconfiguring FTC controller closing the loop from fault detection to
controller reconfiguration, built directly on `ControllerStack`'s existing Supervisory mode
(health-aware fallback + bumpless transfer already built in) but driven by a fault classifier
instead of a static activation condition — wiring `fault_injector.py`'s fault taxonomy
(`sensor_bias`, `sensor_noise`, `actuator_loss`, `actuator_stuck`) into `ControllerStack`'s
per-entry activation automatically.

```cpp
enum class FaultType { None, SensorBias, SensorNoise, ActuatorLoss, ActuatorStuck };

struct FaultDetectorParams {
    double residual_threshold = 3.0;   // sigma, reuses KalmanFilter::mismatchScore() convention
    int    confirm_window      = 5;
};

class FaultClassifier {
public:
    explicit FaultClassifier(const FaultDetectorParams& p);
    FaultType classify(double innovation, double u_cmd, double y_meas);
};

class FTCSupervisor : public IController {
public:
    FTCSupervisor(std::shared_ptr<ControllerStack> stack, double Ts);
    void registerFaultResponse(FaultType fault, const std::string& controllerName);
    double compute(double error) override;   // delegates to stack; switches active entry on fault
};
```

**Reused components:** `ControllerStack` wholesale (Supervisory mode, `addController`/
`setActive`, bumpless transfer already built-in); `KalmanFilter::mismatchDetected()`/
`mismatchScore()` (Phase 2's `D1` CUSUM-on-innovation machinery) as the residual signal driving
`FaultClassifier`; `tools/fault_injector.py`'s fault taxonomy as `FaultType`'s real-world
grounding.

**Effort estimate:** ~300 lines (classifier + supervisor wiring over existing `ControllerStack`
+ 3 tests) — most of the orchestration machinery is pure reuse.

**Example use case:** A loop with a redundant sensor pair; `FTCSupervisor` detects a
`sensor_bias` fault via `mismatchScore()` crossing threshold and automatically switches
`ControllerStack`'s active entry to a controller relying on the healthy sensor, with bumpless
transfer already guaranteed by the existing Supervisory mode.

**Catch2 test plan (`[ftc_supervisor]`):**
1. Injected `actuator_loss` fault (via the `fault_injector` taxonomy) — switches to the
   registered fallback controller within `confirm_window` steps.
2. No fault — behaves identically to a plain `ControllerStack` in Supervisory mode (regression).
3. Fault clears — supervisor switches back (or stays, per registered policy) without a bump in
   `u` (verified via `ControllerStack`'s existing bumpless-transfer guarantee).

---

## Phase 3: Bigger / Specialized Lifts

### ML1 — NN Controller Core (direct NN architectures)

**Goal:** A generic feedforward neural-network controller primitive — forward-pass-only, usable
as `compute()` with arbitrary (e.g. offline-trained) weights — beyond `NeuralPID`'s specific
3-layer Kp/Ki/Kd-gain-output architecture. This is the dependency `ML2` needs before adding
online adaptation.

```cpp
struct NNLayerSpec {
    Eigen::MatrixXd W;
    Eigen::VectorXd b;
    enum class Activation { Tanh, ReLU, Sigmoid, Linear, Softplus } activation;
};

struct NeuralControllerParams {
    std::vector<NNLayerSpec> layers;   // arbitrary depth, set at construction
    double uMin = -1e9, uMax = 1e9;
    int n_input_features = 1;           // e.g. [e, e_dot, e_int] or full state vector
};

class NeuralNetworkController : public IController {
public:
    explicit NeuralNetworkController(const NeuralControllerParams& params, double Ts);
    double compute(double error) override;
    Eigen::VectorXd computeVec(const Eigen::VectorXd& features) override;
    void loadWeights(const std::vector<NNLayerSpec>& layers);   // hot-swap trained weights
    void reset() override;
};
```

**Reused components:** `NeuralPID`'s existing tanh/softplus activation-function implementations,
lifted into a small shared activation-dispatch helper rather than duplicated; `EchoStateNetwork`'s
forward-pass convention for feature-vector shape consistency.

**Effort estimate:** ~300 lines (layer forward pass + activation dispatch + 3 tests).

**Example use case:** Importing weights trained offline (e.g. via PyTorch on simulation data)
for a learned control law, deployed in C++ without re-implementing the network architecture at
runtime.

**Catch2 test plan (`[neural_network_controller]`):**
1. Known small network (manually specified weights) — output matches the hand-computed forward
   pass.
2. `loadWeights()` hot-swap mid-run — output changes immediately on the next `compute()`, no
   stale state.
3. Saturation (`uMin`/`uMax`) enforced regardless of network output magnitude.

---

### ML2 — NN-Adaptive Control (depends on ML1)

**Goal:** Online Lyapunov-stable weight adaptation layered on top of `ML1`'s forward-pass
primitive — the classic RBF-NN/Lyapunov-adaptive-control pattern (NN approximates an unknown
nonlinearity; weights update via a Lyapunov-derived gradient law with sigma-modification for
robustness, mirroring `MRACController`'s sigma-modification convention). Distinct from
`NeuralPID`'s fixed hybrid architecture.

```cpp
struct NNAdaptiveParams {
    NeuralControllerParams nn;   // ML1 base architecture (hidden layers fixed, output adapts)
    double gamma_adapt = 1.0;     // adaptation gain
    double sigma_mod    = 0.01;   // same role as MRACController::sigma
    double uMin = -1e9, uMax = 1e9;
};

class NNAdaptiveController : public IController {
public:
    explicit NNAdaptiveController(const NNAdaptiveParams& params, double Ts);
    double compute(double y_plant) override;   // plant output, like MRACController's convention
    void setReference(double r);
    const Eigen::MatrixXd& currentOutputWeights() const;
};
```

**Reused components:** `ML1`'s `NeuralNetworkController` as the forward-pass engine (only the
output layer adapts online — the reason `ML1` must exist first); `MRACController`'s
sigma-modification adaptation-law convention for bounding weight drift.

**Effort estimate:** ~300 lines (Lyapunov weight-update law wrapping `ML1` + 3 tests) — kept
smaller than a from-scratch NN forward pass because `ML1` already provides that.

**Example use case:** Adaptive cancellation of an unknown, NN-approximable nonlinearity (e.g.
unmodeled friction with no analytic form) where `MRACController`'s linear-in-parameters
structure is too restrictive but full RL is unnecessary.

**Catch2 test plan (`[nn_adaptive_control]`):**
1. Unmodeled nonlinearity approximable by the NN's hidden layer — tracking error converges,
   output weights converge near best-fit values.
2. Sigma-modification bounds weight drift under persistent disturbance (no unbounded growth).
3. `ML1` substrate hot-swap (different hidden-layer size) — adaptive layer re-initializes
   correctly.

---

### SI3 — MOESP / CVA Subspace ID Variants

**Goal:** Extend `SubspaceID.h`'s existing N4SID pipeline (Hankel matrix → LQ decomposition/
oblique projection → SVD → A/C extraction → B/D regression) with MOESP (no stochastic/
Kalman-gain step, simpler oblique projection) and CVA (canonical-variate weighting before the
SVD), sharing the same Hankel/SVD scaffolding.

```cpp
enum class SubspaceMethod { N4SID, MOESP, CVA };

SubspaceIDResult subspaceID(const Eigen::MatrixXd& Y, const Eigen::MatrixXd& U,
                             int n_order, int i_horizon, double Ts,
                             SubspaceMethod method = SubspaceMethod::N4SID,
                             double svd_tol = -1.0);
```

**Reused components:** `SubspaceID.h`'s existing Hankel-matrix construction, LQ decomposition,
and SVD steps directly — MOESP and CVA differ only in the projection/weighting applied before
the SVD, not in the surrounding pipeline.

**Effort estimate:** ~250 lines (2 new weighting variants inserted into the existing pipeline +
3 tests) — moderate, since most of the machinery is shared.

**Example use case:** A dataset where N4SID's stochastic balancing is unnecessary overhead
(pure deterministic excitation) — MOESP's simpler projection is just as good and faster; CVA is
preferred when output channels have very different noise scales.

**Catch2 test plan (`[subspace_id_variants]`):**
1. Known state-space system, all 3 methods — recover equivalent (up to similarity transform)
   realizations.
2. CVA on outputs with deliberately mismatched noise scales — outperforms N4SID/MOESP on the
   high-noise channel.
3. `suggestOrder()` works unchanged across all 3 methods (shared singular-value heuristic).

---

### SI4 — NARMAX

**Goal:** Nonlinear ARMAX identification — fit
`y[k] = f(y[k-1..k-na], u[k-1..k-nb], e[k-1..k-nc])` where `f` is a polynomial expansion over
*lagged* terms, structure-selected via orthogonal forward regression (the standard NARMAX
approach). Distinct from `SINDy`'s sparse-regression-over-a-library approach, which expands
over state *derivatives*, not lagged input/output terms.

```cpp
struct NARMAXParams {
    int    na, nb, nc;             // output, input, noise lag orders
    int    poly_degree = 2;
    double significance_tol = 0.01;   // ERR (error reduction ratio) threshold for term selection
};

struct NARMAXResult {
    std::vector<std::string> selected_terms;   // e.g. "y(k-1)*u(k-2)"
    Eigen::VectorXd coefficients;
    double final_err_sum;
};

class NARMAXIdentifier {
public:
    static NARMAXResult fit(const Eigen::VectorXd& u, const Eigen::VectorXd& y,
                             const NARMAXParams& params);
    static double predict(const NARMAXResult& model, const Eigen::VectorXd& u_hist,
                           const Eigen::VectorXd& y_hist);
};
```

**Reused components:** `SINDy`'s polynomial-library-term-generation pattern, re-targeted from
state-derivative regressors to lagged input/output regressors; a `RecursiveLeastSquares`-style
normal-equation solve for the coefficient fit once terms are selected.

**Effort estimate:** ~400 lines (term-library generation over lagged variables + orthogonal
forward regression + 3 tests) — one of the bigger Phase 3 items.

**Example use case:** A nonlinear process where the nonlinearity is naturally expressed in
input/output lag terms (e.g. a heat exchanger with bilinear flow×temperature coupling) rather
than as a sparse ODE right-hand side, which is `SINDy`'s domain.

**Catch2 test plan (`[narmax]`):**
1. Known NARMAX-generating synthetic system — term selection recovers the correct term set.
2. Prediction accuracy on held-out data — one-step-ahead out-of-sample error within tolerance.
3. Over-complete term library (more candidates than data supports) — `significance_tol` prunes
   to a parsimonious model, doesn't overfit.

---

### FD2 — Complex-Conjugate-Pole Vector Fitting

**Goal:** Per `algorithm_backlog.md`'s own assessment, a materially bigger lift than `FD1`:
general Vector Fitting with complex-conjugate pole-pair bookkeeping and relocation logic (the
full Gustavsen & Semlyen algorithm), needed to represent resonant/lightly-damped systems that
`VectorFitting::fitMagnitude`'s real-pole-only restriction cannot.

```cpp
struct VectorFitComplexParams {
    int    n_real_poles    = 0;
    int    n_complex_pairs = 4;
    int    max_iter         = 20;
    double tol               = 1e-6;
    std::vector<std::complex<double>> initial_poles;   // empty = auto-init on log-spaced grid
};

struct VectorFitComplexResult {
    std::vector<std::complex<double>> poles;     // includes conjugate pairs
    std::vector<std::complex<double>> residues;
    TransferFunction model;
    bool converged;
    std::vector<double> iterError;
};

VectorFitComplexResult fitComplex(const std::vector<double>& omega,
                                   const std::vector<std::complex<double>>& response,
                                   double Ts, const VectorFitComplexParams& params = {});
```

**Reused components:** `VectorFitting.h`'s existing SK-iteration convergence-check pattern and
pole-relocation outer-loop structure; the conjugate-pair bookkeeping itself is new (poles/
residues must come in conjugate pairs for a real-valued time-domain model) — the "own design
pass" the backlog calls for.

**Effort estimate:** ~450 lines (complex-pole relocation + conjugate-pair constraint enforcement
+ 3 tests) — the biggest System-ID-family item in this document.

**Example use case:** Fitting a frequency response with multiple lightly-damped resonances (e.g.
a flexible-structure plant) where `fitMagnitude`'s real-pole restriction cannot represent the
resonant peaks.

**Catch2 test plan (`[vector_fit_complex]`):**
1. Synthetic response with 2 known complex-conjugate pole pairs — recovered poles match within
   tolerance.
2. Conjugate-pair constraint — every returned pole has its conjugate also present (no orphaned
   complex pole, which would produce a non-real-valued time response).
3. Mixed real + complex pole system — correctly identifies which poles should be real vs.
   complex-paired.

---

### NC3 — Nonlinear Internal Model Control

**Goal:** Nonlinear extension of the IMC structure `SmithPredictor`/SOPDT-Rivera-IMC already
cover for linear plants — uses a nonlinear process model directly inside the IMC feedback
structure (model-based feedforward + model-mismatch feedback correction).

```cpp
struct NonlinearIMCParams {
    double filter_lambda = 1.0;   // IMC filter time constant
    double uMin = -1e9, uMax = 1e9;
};

class NonlinearIMC : public IController {
public:
    using ModelFn        = std::function<double(const Eigen::VectorXd& x, double u)>;
    using InverseModelFn = std::function<double(const Eigen::VectorXd& x, double y_target)>;
    NonlinearIMC(ModelFn model, InverseModelFn inverse, const NonlinearIMCParams& params,
                 double Ts);
    double compute(double error) override;
    void setState(const Eigen::VectorXd& x);
};
```

**Reused components:** `SmithPredictor`'s model-in-the-loop structure (parallel internal model,
feedback on the model-mismatch residual), generalized from a linear SOPDT model to an arbitrary
nonlinear `ModelFn`; the IMC filter follows the same first-order-filter convention as the
existing Rivera-IMC tuning rule.

**Effort estimate:** ~250 lines (parallel nonlinear model + mismatch feedback + 3 tests) —
`algorithm_backlog.md` correctly calls this "a separate, smaller class" relative to the linear
case.

**Example use case:** A chemical reactor with known nonlinear kinetics where a linear SOPDT
approximation loses accuracy away from the linearization point — `NonlinearIMC` uses the full
nonlinear model for feedforward prediction while still rejecting model mismatch via the IMC
feedback path.

**Catch2 test plan (`[nonlinear_imc]`):**
1. Exact model match (no plant-model mismatch) — perfect tracking (IMC's classic property).
2. Model mismatch (perturbed model parameters) — feedback path corrects the steady-state offset.
3. Inverse model unavailable/singular at an operating point — graceful fallback (hold-last, no
   NaN propagation).

---

### ML3 — GP-MPC

**Goal:** A controller that consumes GP uncertainty directly in the MPC cost/constraints.
`GaussianProcess`/`GPResidualModel` and `NonlinearMPC`/`TubeMPC` exist separately today;
`HybridMPC` partially addresses this (ridge-regression data model) but not GP-uncertainty-aware
tightening specifically. GP-MPC inflates/tightens constraints in the prediction horizon
proportional to the GP's predicted variance at each step, rather than using a fixed point
estimate.

```cpp
struct GPMPCParams {
    NMPCParams nmpc;                       // base NonlinearMPC params
    double uncertainty_inflation = 2.0;    // tightening = inflation * sqrt(variance)
};

class GPMPC : public NonlinearMPC {
public:
    GPMPC(const GPMPCParams& params, DiscreteDynamics f_d, std::shared_ptr<GPResidualModel> gp);
    // compute() inherited; rollout queries gp->predictWithUncertainty() at each predicted step
    // and tightens uMin/uMax (or output constraints) by uncertainty_inflation * sqrt(variance)
};
```

**Reused components:** `GPResidualModel::predictWithUncertainty()` directly for the per-step
variance; `NonlinearMPC`'s existing rollout structure (the same `DiscreteDynamics`-callback
pattern `HybridMPC` already overrides) as the integration point — `GPMPC` is architecturally a
sibling of `HybridMPC`, not a from-scratch MPC.

**Effort estimate:** ~300 lines (variance-aware constraint tightening inserted into the existing
`NonlinearMPC` rollout + 3 tests).

**Example use case:** A CSTR reactor where the GP residual model's predicted variance grows in
under-explored operating regions — GP-MPC automatically backs off the constraint bounds there,
which a fixed-point-estimate `HybridMPC` cannot do.

**Catch2 test plan (`[gp_mpc]`):**
1. GP confident (low variance, well-explored region) — behaves like the underlying
   `NonlinearMPC` (regression, confirms tightening vanishes at zero variance).
2. GP uncertain (high variance, extrapolation region) — constraints visibly tighten, control
   becomes more conservative.
3. Comparison against `HybridMPC` on the same scenario — GP-MPC's variance-aware tightening
   avoids a constraint violation that `HybridMPC`'s fixed-point estimate misses.

---

## Phase 4: Heavy Infrastructure, Lower Near-Term Priority

### RC2 — LMI Solver

**Goal:** A general-purpose LMI (Linear Matrix Inequality) solver — feasibility, cost
minimization, generalized-eigenvalue minimization — the convex-optimization primitive this
toolbox still lacks (`GradientProjectionQP` is QP, not SDP). Its original motivating use cases
(H2 synthesis, structured Hinf) shipped via other routes (see `algorithm_backlog.md`'s Robust
Control section), so this is now scoped narrower: multi-objective Hinf/H2 mixed synthesis and
any future LMI-native algorithm.

```cpp
struct LMIConstraint {
    std::function<Eigen::MatrixXd(const Eigen::VectorXd& x)> F;   // F(x) must be PSD (or NSD)
    bool requirePSD = true;
};

struct LMIProblem {
    std::vector<LMIConstraint> constraints;
    Eigen::VectorXd c;     // cost vector for minimize c'x (empty = feasibility only)
    int n_vars;
};

struct LMIResult {
    bool feasible;
    Eigen::VectorXd x;
    double cost;
    int    iters;
};

class LMISolver {
public:
    static LMIResult solveFeasibility(const LMIProblem& problem, int maxIter = 100, double tol = 1e-6);
    static LMIResult solveCostMin(const LMIProblem& problem, int maxIter = 100, double tol = 1e-6);
    static LMIResult solveGEVP(const LMIProblem& problemA, const LMIProblem& problemB,
                                int maxIter = 100, double tol = 1e-6);
};
```

**Reused components:** Nothing existing covers this directly — `GradientProjectionQP`'s
projected-update iteration shape is a loose inspiration for the primal-dual structure, but the
projection-onto-the-PSD-cone step (via eigenvalue decomposition) is new.

**Effort estimate:** ~600 lines (interior-point/projected-subgradient SDP solver + 3
problem-type wrappers + 5 tests) — the single biggest implementation effort in this roadmap.

**Example use case:** Multi-objective Hinf/H2 mixed-sensitivity synthesis (minimize H2 cost
subject to an Hinf constraint) — the textbook case requiring genuine LMI machinery rather than
a Riccati shortcut.

**Catch2 test plan (`[lmi_solver]`):**
1. Simple Lyapunov-stability LMI (`A'P + PA < 0`) — recovers a known feasible `P` for a stable
   `A`, correctly reports infeasible for an unstable `A`.
2. Cost-minimization LMI (e.g. minimize `trace(P)` subject to a Lyapunov LMI) — matches a
   hand-solved small case.
3. GEVP — matches a known generalized-eigenvalue benchmark.
4. Near-degenerate constraint — doesn't diverge; returns an iters-exhausted/reduced-confidence
   flag rather than a silently wrong answer.

---

### OC2 — Dynamic Programming / Value Iteration

**Goal:** Classical DP/value-iteration solver over a discretized state-space grid for
finite-horizon or discounted-infinite-horizon optimal control — valid for low-dimensional
(`n <= 3-4`, curse-of-dimensionality-limited) problems where a globally optimal (not just
locally optimal) policy is wanted and MPC's continuous optimization isn't required.

```cpp
struct DPGridParams {
    Eigen::VectorXd x_min, x_max;
    Eigen::VectorXi n_grid_per_dim;
    Eigen::VectorXd u_min, u_max;
    int    n_grid_u;
    double discount = 0.99;
    int    max_iter  = 500;
    double tol        = 1e-6;
};

class ValueIterationSolver {
public:
    using StageCost  = std::function<double(const Eigen::VectorXd& x, double u)>;
    using DynamicsFn = std::function<Eigen::VectorXd(const Eigen::VectorXd& x, double u)>;
    ValueIterationSolver(DynamicsFn f, StageCost cost, const DPGridParams& params);
    void   solve();
    double policy(const Eigen::VectorXd& x) const;   // nearest-grid-point + interpolation
    double value(const Eigen::VectorXd& x) const;
};
```

**Reused components:** None — no grid-based DP exists in `lib/` today; a from-scratch effort.
The resulting `policy()` lookup can be wrapped in an `IController`-derived class using the same
`compute()`-delegation pattern other wrappers use, which would be the first table-driven
controller in the library.

**Effort estimate:** ~350 lines (grid construction + value-iteration sweep + interpolated policy
lookup + 3 tests).

**Example use case:** A small 2-state mechanical system (e.g. pendulum swing-up) where the
globally optimal policy outperforms a locally-optimal `NonlinearMPC` initialized from a poor
guess, and the state dimension is low enough for grid discretization to be tractable.

**Catch2 test plan (`[value_iteration]`):**
1. LQR-equivalent problem (linear dynamics + quadratic cost on a fine grid) — converges to a
   policy matching `DiscreteLQR`'s gain within grid-resolution error.
2. Convergence — value-function update norm decreases monotonically toward `tol`.
3. Grid-resolution sensitivity — a documented accuracy-vs-grid-size tradeoff test, confirming
   expected curse-of-dimensionality behavior (not a bug).

---

### OC4 — Linear-Programming-Based Control

**Goal:** An LP solver (active-set) extending the QP-only optimization layer
(`GradientProjectionQP`) to linear cost/linear constraint problems, then an LP-based MPC variant
— min-time control, L1/Linf-cost MPC are naturally LPs, not QPs.

```cpp
struct LPProblem {
    Eigen::VectorXd c;            // minimize c'x
    Eigen::MatrixXd A_ineq;
    Eigen::VectorXd b_ineq;        // A_ineq * x <= b_ineq
    Eigen::VectorXd lb, ub;
};

struct LPResult {
    bool   feasible;
    Eigen::VectorXd x;
    double cost;
    int    iters;
};

class LPSolver {
public:
    static LPResult solve(const LPProblem& problem, int maxIter = 200, double tol = 1e-8);
};
```

**Reused components:** `GradientProjectionQP`'s box-constraint-projection pattern as a starting
point for the active-set working-set updates; the resulting `LPSolver` fills the same
"inner-loop solver feeding an MPC's per-step optimization" role `GradientProjectionQP` already
fills for `NonlinearMPC`/`TubeMPC`, so an LP-MPC variant reuses their existing rollout
scaffolding, swapping only the inner solver.

**Effort estimate:** ~400 lines (active-set LP solver + LP-MPC wiring + 3 tests).

**Example use case:** Minimum-time control of an actuator-limited system where the natural cost
is "time to reach target" (an LP after time-discretization), not a quadratic tracking cost.

**Catch2 test plan (`[lp_solver]`):**
1. Known LP with a textbook solution — matches the known optimum.
2. Infeasible LP — correctly reports `feasible=false`, doesn't loop forever.
3. LP-MPC on a min-time problem — converges to the bang-bang-like solution expected for
   minimum-time problems.

---

### DT1 — Code Generation

**Goal:** Per `algorithm_backlog.md`: "highest production value of this category" but also the
heaviest lift. **Scoped down from the full 90-class library**: emit dependency-free,
allocation-free C code only for controller types simple enough to have a clean closed-form
update equation (the ones already mirrored in `lib/embedded/`'s header-only subset, e.g.
`BasicPID`/`BasicSMC`, plus `LeadLagController`) — MPC/Hinf/MHE code-gen is explicitly out of
scope for v1 (see "Out of Scope" below).

```cpp
struct CodeGenParams {
    std::string function_name = "controller_step";
    std::string target_lang    = "c99";   // v1: C99 only
};

class ControllerCodeGenerator {
public:
    // Emits a self-contained .c/.h pair implementing compute() as a stateless-struct + step
    // function, matching lib/embedded/'s conventions. One overload per supported class —
    // adding coverage is additive.
    static std::string generateC(const DiscretePID& controller, const CodeGenParams& params);
    static std::string generateC(const DiscreteSMC& controller, const CodeGenParams& params);
    static std::string generateC(const LeadLagController& controller, const CodeGenParams& params);
};
```

**Reused components:** `lib/embedded/`'s existing header-only no-Eigen subset as both the
*target* code style and the proof that these specific controllers admit a clean allocation-free
C representation — the generator automates producing that style from a tuned `lib/` controller
instance's parameters, instead of requiring a hand port.

**Effort estimate:** ~500 lines (template-based emission for 3 initial controller types +
golden-file tests + 4 tests) — deliberately scoped down to stay achievable; each additional
controller type is a small additive follow-up once the emitter framework exists.

**Example use case:** A user who tuned a `DiscretePID` using the full C++ toolbox's analysis/
tuning tooling wants to deploy just the resulting fixed-gain controller on a bare-metal MCU
without linking Eigen or the rest of `lib/` — `generateC()` emits one dependency-free `.c` file
with the tuned gains baked in.

**Catch2 test plan (`[code_generation]`):**
1. Generated C for a `DiscretePID`, compiled standalone (no Eigen, no `lib/` link) — bit-identical
   output to the original `DiscretePID::compute()` across a reference input sequence
   (golden-file regression).
2. Same golden-file check for `DiscreteSMC` and `LeadLagController`.
3. Generated code has zero dynamic allocation (grep-able check, matching `deployment.md`'s
   zero-allocation checklist) and compiles with `-fno-exceptions -fno-rtti`.

---

### DT2 — Real-Time Profiling Beyond WCET

**Goal:** Extend `tools/wcet_report.py`'s worst-case-execution-time coverage with finer-grained
per-call-site profiling — timing *distribution* (not just the worst case), jitter analysis, and
identification of hot paths sensitive to cache/branch prediction.

```python
# tools/rt_profiler.py
class RTProfiler:
    def __init__(self, binary_path: str, iterations: int = 10000): ...
    def profile_compute_call(self, controller_name: str) -> ProfileResult:
        """Runs compute() in a tight loop, records per-call timing distribution via the same
        instrumentation hooks wcet_report.py already uses."""
    def jitter_report(self) -> JitterReport:
        """std-dev / percentile breakdown (p50/p95/p99/max) per controller."""
```

**Reused components:** `tools/wcet_report.py`'s existing instrumentation/timing-harness
approach directly — an additive sibling tool reusing the same measurement mechanism, aggregating
distributionally instead of max-only.

**Effort estimate:** ~250 lines Python (wraps existing WCET instrumentation with distributional
statistics + report generation) — a `tools/` extension, no C++ changes, mirroring how Phase 2's
D2 (Digital Twin Lite) was scoped Python-only.

**Example use case:** A controller comfortably within its WCET budget but exhibiting high
jitter (e.g. occasional cache misses on a branch-heavy NaN-guard path) — useful for diagnosing
intermittent control-loop timing issues a single worst-case number hides.

**Test plan (Python-level, not Catch2):** Verify against a synthetic controller with
deliberately injected variable-latency branches that `jitter_report()` correctly identifies the
bimodal timing distribution; verify percentile calculations against a known reference
distribution.

---

### DT3 — Distributed / Networked Control

**Goal:** Extend `ComputationalDelayWrapper`'s single-fixed-delay model to a multi-node
networked-control scenario: variable/stochastic network delay, packet loss, and a simple
consensus/synchronization primitive across multiple controller nodes.

```cpp
struct NetworkedDelayParams {
    double mean_delay        = 0.0;
    double delay_jitter      = 0.0;   // stddev of delay, sampled per packet
    double packet_loss_prob  = 0.0;   // probability a packet is dropped, not just delayed
    unsigned seed             = 42;
};

class NetworkedControlWrapper : public IController {
public:
    NetworkedControlWrapper(std::shared_ptr<IController> inner, const NetworkedDelayParams& params);
    double compute(double signal) override;   // applies stochastic delay/loss to inner's output
    void   reset() override;
    int    droppedPacketCount() const;
};

class ConsensusCoordinator {
public:
    ConsensusCoordinator(int n_nodes, double coupling_gain);
    void   receiveNeighborState(int node_id, double value);
    double consensusUpdate(double local_value);
};
```

**Reused components:** `ComputationalDelayWrapper`'s `IController`-decorator pattern directly,
generalized from a fixed 1-sample delay to a stochastic delay/loss model; the same decorator
composability the existing wrapper already supports (stacks with `AntiWindupWrapper`,
`GainScheduledController`, etc.).

**Effort estimate:** ~350 lines (stochastic delay/loss decorator + basic consensus primitive +
3 tests).

**Example use case:** A multi-actuator system where each actuator's local controller
communicates over a lossy network (e.g. wireless sensor/actuator network) — lets a case study
simulate realistic network conditions instead of assuming a perfect fixed-delay link.

**Catch2 test plan (`[networked_control]`):**
1. Zero jitter/loss — behaves identically to `ComputationalDelayWrapper` (regression, confirms
   the generalization preserves the deterministic-delay special case).
2. Packet loss — `droppedPacketCount()` matches the configured probability within statistical
   tolerance over many steps.
3. Consensus primitive on a known small network topology — converges to the analytically
   expected consensus value.

---

## Phase 5: Niche / Research-Grade (Kept, Not Cut)

### NC5 — Globally Linearizing Control

**Goal:** Already flagged "niche/rare in practice — low priority" in `algorithm_backlog.md`.
GLC achieves *global* (not just local) linearization via a coordinate transformation valid over
the entire state space, vs. `FeedbackLinearisationController`'s local relative-degree-1
approach. Kept for completeness as a real, named technique; sequenced last because the backlog's
assessment of low real-world demand stands.

```cpp
struct GLCParams {
    double uMin = -1e9, uMax = 1e9;
};

class GloballyLinearizingController : public IController {
public:
    using TransformFn        = std::function<Eigen::VectorXd(const Eigen::VectorXd& x)>;
    using InverseTransformFn = std::function<Eigen::VectorXd(const Eigen::VectorXd& z)>;
    GloballyLinearizingController(TransformFn phi, InverseTransformFn phi_inv,
                                   std::shared_ptr<IController> linear_inner,
                                   const GLCParams& params, double Ts);
    double compute(double error) override;
    void   setState(const Eigen::VectorXd& x);
};
```

**Reused components:** `FeedbackLinearisationController`'s compute()-delegation-to-inner-
controller pattern (transform → apply a linear controller in transformed coordinates →
transform back).

**Effort estimate:** ~250 lines (global coordinate-transform wrapper + 2 tests) — small because
it reuses `FeedbackLinearisation`'s delegation pattern; placed in Phase 5 purely for low expected
usage, not implementation difficulty.

**Example use case:** A system with a known global diffeomorphism to linear coordinates over
its entire operating envelope — rare in practice, which is exactly why this is niche.

**Catch2 test plan (`[glc]`):**
1. Known globally-linearizable textbook system — closed loop matches the underlying linear
   controller's response exactly in transformed coordinates.
2. Round-trip consistency: `Phi_inv(Phi(x)) == x` within tolerance.
3. Operating point outside the transform's valid domain — flagged/clamped, not silently wrong.

---

### OC3 — Dual Control

**Goal:** Already flagged "research-grade, niche; low priority" in `algorithm_backlog.md`. Dual
control actively balances exploration (reducing parameter uncertainty) against exploitation
(minimizing tracking cost) — distinct from `CEMController`/`DynaController`'s MBRL approach.
Kept for completeness; sequenced last given its niche, research-grade status.

```cpp
struct DualControlParams {
    double exploration_weight = 0.1;   // weight on the exploration/information term
    int    n_dim_params;
};

class DualController : public IController {
public:
    using CostFn = std::function<double(const Eigen::VectorXd& x, double u,
                                         const Eigen::MatrixXd& param_cov)>;
    DualController(CostFn cost, const DualControlParams& params, double Ts);
    double compute(double error) override;
    void   updateParameterEstimate(const Eigen::VectorXd& theta_hat, const Eigen::MatrixXd& cov);
};
```

**Reused components:** `RecursiveLeastSquares` (or `RecursiveGreyBoxEstimator` from Phase 2) for
the parameter-uncertainty input the dual cost needs; `AutoTuner`'s CMA-ES as the per-step
optimizer for the combined exploration+exploitation cost (generally non-convex).

**Effort estimate:** ~350 lines (combined cost formulation + per-step CMA-ES optimization +
3 tests) — placed in Phase 5 for niche applicability, not implementation difficulty.

**Example use case:** An adaptive controller for a system with persistent parameter uncertainty
where pure certainty-equivalence control (e.g. plain MRAC) risks poor excitation — dual control
explicitly trades off a probing action against tracking performance.

**Catch2 test plan (`[dual_control]`):**
1. Known uncertain parameter, dual control vs. certainty-equivalence — dual control's parameter
   estimate converges faster due to deliberate exploration.
2. `exploration_weight = 0` — reduces to certainty-equivalence behavior (regression case).
3. Already-converged estimate (near-zero covariance) — exploration term vanishes, matches pure
   exploitation.

---

### ML4 — RL-Based Control (merged)

**Goal:** Merges 2 backlog line items — Adaptive Control's "Reinforcement-learning-based
adaptive control" and Machine Learning Integration's "Deep reinforcement learning" — since
`algorithm_backlog.md` itself flags them as "the same gap, two wishlist entries." Following the
precedent `ALGORITHM_ROADMAP_PHASE2.md` set for its H3 item ("Full RL framework... no C++ RL
core needed"), this is scoped as a Python-only example wiring a small policy to adjust an
existing controller's parameters online — not a from-scratch C++ RL core. `DynaController`/
`CEMController` already cover the lightweight MBRL end of this spectrum in C++.

```python
# examples/python/exNN_rl_adaptive_control.py
# A small policy (2-layer MLP, <10k params, PyTorch) observes [e(t), |u(t)|, e_rms_window]
# and outputs an adjustment to an existing ctrl_toolbox controller's parameter each step
# (e.g. nudging DiscretePID's Kp, or OC1's SelfTuningRegulator forgetting factor lambda),
# trained via a standard RL library against a ctrl_toolbox-simulated plant.
```

**Reused components:** `ctrl_toolbox` Python bindings (any existing controller as the thing
being tuned); the same "Python policy adjusts a C++ controller's parameters" pattern Phase 2's
H3 (RL-MPC stitching) already established and validated.

**Effort estimate:** ~250 lines Python (policy + training loop + plant-simulation glue),
following H3's precedent almost exactly — no new C++ required.

**Example use case:** A policy that learns to adjust `OC1`'s forgetting factor `lambda` based
on observed tracking performance, rather than a fixed value — closing the gap between "online
RLS identification" and "RL-tuned identification," without a general-purpose C++ RL framework.

**Test plan (Python-level):** Verify the training loop converges (reward increases over
training episodes) on a simple benchmark plant; verify the trained policy outperforms a
fixed-parameter baseline controller on a held-out test trajectory.

---

## What's Explicitly Out of Scope (within items above)

Mirroring `ALGORITHM_ROADMAP_PHASE2.md`'s deferral table — these are sub-scope cuts *within* an
item above, not whole items cut from the roadmap (every category-level item is covered by Phase
1-5 per the "include all, flag low-priority" decision for this document):

| Sub-scope cut | Reason |
|---|---|
| `RC2` LMI solver: general N-block SDP beyond feasibility/cost-min/GEVP | Full general-purpose SDP (arbitrary cone combinations) is a research project on its own; the 3 problem types listed cover every robust-control use case currently on this roadmap |
| `DT1` code generation: MPC/Hinf/MHE targets | Would need either a bundled QP solver in generated code or full offline precomputation; v1 is scoped to closed-form controllers only |
| `ML4` RL-based control: full Stable-Baselines3 / general RL framework integration | Same reasoning as Phase 2's H3 — a Python example validates the pattern; no C++ RL core needed |
| `DT3` distributed control: real multi-machine networking (sockets/network stack) | Simulated stochastic delay/loss is sufficient for the case-study roster; real network I/O is a deployment concern, not an algorithm one |
| `OC2` DP/value iteration: state dimension > ~4 | Curse of dimensionality makes grid-based DP intractable beyond this; higher-dimensional problems should use the MPC family, which already exists |

---

## Implementation Checklist (per algorithm)

Each new `lib/` algorithm follows the same 8-step checklist `ALGORITHM_ROADMAP_PHASE2.md`
established, from `CLAUDE.md`/`CONTRIBUTING.md`:

```
1. lib/ClassName.{h,cpp} — implement; call notifyObserver() at end of compute()
2. lib/CMakeLists.txt — add ClassName.cpp to CTRL_CORE_SOURCES
3. lib/ControllerToolbox.h — add #include "ClassName.h"
4. lib/Features.h — add {"feature_name", true} entry
5. bindings/*_bindings.cpp — add pybind11 class with std::shared_ptr<T> 3rd arg
6. bindings/smoke_test.py — add assertion
7. tests/test_catch2_advanced.cpp — add 2+ Catch2 tests with [tag]
8. examples/exNN.cpp + examples/python/exNN.py + update CMakeLists.txt + compile.bat
```

For Python-only items (`DT2`, `ML4`): steps 1-4 and 7-8 (C++ side) are skipped.
For extensions to existing classes (`SI3`, `EF3`, `FD1`, `FD2`, `MO3`, `DT4`): only the modified
files need updating, not the full 8-step checklist — but Catch2 tests are always required.
Every item still gets its own design spec under `docs/superpowers/specs/` before being built —
this document scopes effort and sequencing, it does not replace that step.

---

## Estimated Timeline

| Phase | Items | Approx. lines | Effort | Notes |
|-------|-------|---------------|--------|-------|
| Phase 1 | EF1, RC1, NC1, NC2, NC4, SI5, SI2, FD1, MO2 (9 items) | ~2,330 | ~22-26 days | Highest value-to-effort ratio; no cross-item dependencies, can be done in any order |
| Phase 2 | OC1, SI1, EF2, EF3, MO1, MO3, DT4 (7 items, covers 9 backlog lines) | ~2,200 | ~21-24 days | `MO3` benefits from `MO2`/`MO1` existing but doesn't require them |
| Phase 3 | ML1, ML2, SI3, SI4, FD2, NC3, ML3 (7 items) | ~2,250 | ~21-25 days | `ML2` strictly requires `ML1` first; everything else independent |
| Phase 4 | RC2, OC2, OC4, DT1, DT2, DT3 (6 items) | ~2,450 | ~24-27 days | `RC2` is the single largest item in the roadmap |
| Phase 5 | NC5, OC3, ML4 (3 items, covers 4 backlog lines) | ~850 | ~8-10 days | Niche/research-grade; smallest phase |
| **Total** | **32 designs (35 backlog lines)** | **~10,080** | **~96-112 days** | Focused part-time development, roughly 19-22 weeks at the cadence `ALGORITHM_ROADMAP_PHASE2.md` used (~85-100 lines/day) |

Unlike `ALGORITHM_ROADMAP_PHASE2.md`'s single dependency chain, these phases are mostly
independent — the ordering above is a value/ROI recommendation, not a hard requirement. A phase
can be reprioritized or skipped without blocking the others (the two real dependency edges are
`ML1 → ML2` and `MO2/MO1 → MO3` as a soft preference).

---

*This document is the Phase 3 planning reference, parallel to `ALGORITHM_ROADMAP_PHASE2.md`.
As each item ships, move its corresponding line from `docs/algorithm_backlog.md`'s open
categories into its "Already done" table (the same way Phase 4's frequency-domain and
Resonant/Notch/PLL work was tracked) and mark the status table at the top of this document.*
