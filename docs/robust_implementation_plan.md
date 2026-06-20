# Robustness Analysis — C++ Implementation Plan

Generated 2026-06-17. Authored against codebase state at Part 60.

**Status update (Part 66, 2026-06-20): Phases 1-3 are done** (built shortly after this plan
was authored — confirmed complete and documented in `CLAUDE.md`'s ROB-1 entry by Part 63).

**Status update (Part 67, 2026-06-20): Phases 4 and 5 are also done.** Built as
`lib/WorstCaseSearch.h` (CMA-ES worst-case parameter search, wraps `AutoTuner`) and
`lib/LyapunovRobustness.h` (common quadratic Lyapunov function for polytopic uncertainty,
wraps `SystemAnalysis::solveDiscreteLyapunov`) — both header-only, both bound in
`bindings/analysis_bindings.cpp`, both exercised by `bindings/smoke_test.py` and dedicated
`[worst_case_search]`/`[lyapunov_robustness]` Catch2 tests, both with a runnable example
(`ex86_worst_case.cpp`, `ex87_lyapunov_robust.cpp`). All five phases of this plan are now
complete — see `CLAUDE.md`'s ROB-1 entry for the consolidated status and the "Part 67"
non-obvious-facts note for two real deviations from the literal pseudocode below (Phase 5's
sum-vs-average fix, and Phase 4's normalised-search-space construction). The sections below
are kept as the original design rationale, not a "to do" list. **Important:** the
"Integration with Existing Case Studies" section near the end of this document describes a
path that was **not** the one actually taken for case-study integration — see the
correction inserted there for what was actually built instead (Part 64 + 66) and why.

---

## Existing Foundation

| File | What it provides |
|------|-----------------|
| `lib/RobustnessAnalysis.h` / `.cpp` | **Built (Phase 1, done by Part 63).** `spawn_SS_samples`/`spawn_TF_samples`, `monteCarloAnalysis()` -> `MonteCarloResult` (stability probability, gain/phase margins, peak S/T, IAE, nu-gap). Bound as `ctrl.monte_carlo_analysis`/`ctrl.MonteCarloResult`/`ctrl.spawn_SS_samples`. Was a stub with broken syntax when this plan was authored (Part 60) — that description below is the *original* motivation for Phase 1, not the current state. |
| `lib/SystemAnalysis.h` | `getPoles`, `isDiscreteStable`, `solveDiscreteLyapunov`, `getFrequencyResponse`, `calculateMargins`, `calculateHInfinityNorm`. **Gang-of-Four + Disk Margin extensions (Phase 2) also done** (`gang_of_four`, `calculate_disk_margin`). |
| `lib/GapMetric.h` | `nuGap`, `nuGapMatrix`, `freqResponseGrid`, `chordalDist`, `subspaceDist` |
| `lib/DiscreteHinf.h` | Full DGKF Hinfty synthesis, `GeneralisedPlant`, `MixedSensitivity`, gamma-bisection |
| `lib/PlantModel.h` | `TransferFunction`, `StateSpace`, `tf2ss`, `ss2tf`, `c2d`, `ssStep`, `minreal`, `DAESystem`, `consistentInit` |
| `lib/AutoTuner.h` | CMA-ES optimiser — `AutoTuner`, `TunerResult`, `CostFn` |
| `lib/ControllerMonitor.h` | `CUSUMChart`, `EWMAChart` — reuse for run-chart analysis |

The implementation plan is structured in five phases ordered by value-to-effort ratio.
Each phase is independent and can be merged separately.

---

## Phase 1 — Fix and Complete `RobustnessAnalysis.h` (Model Spawning + Monte Carlo)

**Status: Done.** Built as `lib/RobustnessAnalysis.{h,cpp}`; bound, tested (`[robustness_mc]`),
and exampled (`ex83_robustness_mc.cpp`). The write-up below is the original design plan kept
for reference, not an outstanding task.

**Priority: HIGH — unblocks everything else.**

### 1.1 What to build

Rewrite `lib/RobustnessAnalysis.h` and add `lib/RobustnessAnalysis.cpp`.

#### Structs

```cpp
namespace ctrl {

// Result row from one closed-loop simulation on one sample plant.
struct RobustnessSample {
    int    sample_id;
    bool   is_stable;          // all closed-loop poles inside unit disk
    double gain_margin_db;     // NaN if MIMO or not found
    double phase_margin_deg;   // NaN if MIMO or not found
    double hinf_sensitivity;   // ||S||_inf
    double hinf_comp_sens;     // ||T||_inf
    double iae;                // integral absolute error (step response)
    double settling_time_s;    // 5% band, NaN if not settled
    double overshoot_pct;      // %, NaN if no overshoot
    double nu_gap_from_nominal; // nuGap(nominal, this sample)
};

// Aggregated result over the full ensemble.
struct MonteCarloResult {
    int    n_samples;
    int    n_unstable;
    double instability_probability;

    // Per-metric: mean, std, p5, p25, p50, p75, p95, worst
    struct MetricStats {
        double mean, std_dev, p5, p25, p50, p75, p95, worst;
    };
    MetricStats gm_stats;
    MetricStats pm_stats;
    MetricStats sensitivity_peak_stats;
    MetricStats comp_sensitivity_peak_stats;
    MetricStats iae_stats;
    MetricStats settling_stats;
    MetricStats nu_gap_stats;

    std::vector<RobustnessSample> samples; // full per-sample data
};

// Uncertainty description for one parameter (state-space coefficient).
struct PerturbationSpec {
    enum class Distribution { Uniform, Normal };
    Distribution dist    = Distribution::Normal;
    double sigma         = 0.05;  // relative sigma (Normal) or half-width (Uniform)
    double lower_bound   = 0.0;   // absolute lower clamp (0 = no clamp)
    double upper_bound   = 0.0;   // absolute upper clamp (0 = no clamp)
};

} // namespace ctrl
```

#### Functions

```cpp
namespace ctrl {

// -----------------------------------------------------------------------
// Model spawning
// -----------------------------------------------------------------------

// Perturb TF numerator and denominator coefficients independently.
// numerator_sigma / denominator_sigma: relative standard deviations per
// coefficient (vector must match num/den length, or be length-1 for uniform).
std::vector<TransferFunction>
spawn_TF_samples(const TransferFunction& nominal,
                 uint32_t num_samples,
                 const std::vector<double>& numerator_sigma,
                 const std::vector<double>& denominator_sigma,
                 uint32_t seed = 42);

// Perturb A, B, C, D matrices element-wise with relative Gaussian noise.
// sigma_A / sigma_B / sigma_C / sigma_D: relative standard deviations.
// Pass 0.0 to hold a matrix fixed (e.g., sigma_C=0 keeps the output map exact).
std::vector<StateSpace>
spawn_SS_samples(const StateSpace& nominal,
                 uint32_t num_samples,
                 double sigma_A,
                 double sigma_B  = 0.0,
                 double sigma_C  = 0.0,
                 double sigma_D  = 0.0,
                 uint32_t seed   = 42);

// -----------------------------------------------------------------------
// Monte Carlo analysis
// -----------------------------------------------------------------------

// Run closed-loop step simulation on a single (plant, controller_ss) pair.
// controller_ss: pre-computed state-space of the controller (from DiscreteHinf,
// DiscreteLQG, or any dynamic controller converted to SS form).
// Returns a single RobustnessSample.
RobustnessSample
evaluateSample(int sample_id,
               const StateSpace& plant,
               const StateSpace& controller_ss,
               const StateSpace& nominal_plant,
               double step_amplitude  = 1.0,
               double sim_duration_s  = 50.0,
               double settling_band   = 0.05);

// Run the full Monte Carlo over an ensemble of plants.
// Calls evaluateSample() for each element; computes aggregate statistics.
MonteCarloResult
runMonteCarlo(const std::vector<StateSpace>& ensemble,
              const StateSpace& controller_ss,
              const StateSpace& nominal_plant,
              double step_amplitude = 1.0,
              double sim_duration_s = 50.0);

// Convenience: spawn + run in one call.
MonteCarloResult
monteCarloAnalysis(const StateSpace& nominal_plant,
                   const StateSpace& controller_ss,
                   uint32_t num_samples,
                   double sigma_A,
                   double sigma_B  = 0.0,
                   double sigma_C  = 0.0,
                   double sigma_D  = 0.0,
                   uint32_t seed   = 42);

} // namespace ctrl
```

### 1.2 Key implementation details

- **`evaluateSample` closed-loop construction:** Form the closed-loop SS directly rather
  than simulating in a loop. For a unity-feedback loop with plant `G = (A_g,B_g,C_g,D_g)`
  and controller `K = (A_k,B_k,C_k,D_k)`, the closed-loop A matrix is:
  ```
  A_cl = [ A_g - B_g*(I+D_k*D_g)^-1*D_k*C_g,   -B_g*(I+D_k*D_g)^-1*C_k ]
         [ B_k*(I+D_g*D_k)^-1*C_g,               A_k - B_k*(I+D_g*D_k)^-1*D_g*C_k ]
  ```
  Use the closed-loop SS to compute poles (`SystemAnalysis::getPoles`), then
  simulate using `ssStep` for time-domain metrics (IAE, settling, overshoot).

- **`calculateHInfinityNorm` already exists** in `SystemAnalysis`. Use it directly on
  the sensitivity and complementary sensitivity SS models.

- **ν-gap:** call `nuGap(nominal, sample)` from `GapMetric.h` for each sample.
  This is MIMO-safe.

- **Seed reproducibility:** Use `std::mt19937` seeded from the `seed` parameter.
  Each `spawn_*` call allocates its own RNG so results are deterministic.

### 1.3 Checklist

1. `lib/RobustnessAnalysis.h` — structs + declarations (≈120 lines)
2. `lib/RobustnessAnalysis.cpp` — `spawn_*`, `evaluateSample`, `runMonteCarlo` (≈300 lines)
3. `lib/CMakeLists.txt` — add `RobustnessAnalysis.cpp` to `CTRL_CORE_SOURCES`
4. `lib/ControllerToolbox.h` — add `#include "RobustnessAnalysis.h"`
5. `lib/Features.h` — `{"robustness_analysis", true}`
6. `bindings/analysis_bindings.cpp` (or new file) — bind `MonteCarloResult`, `spawn_SS_samples`,
   `runMonteCarlo`, `monteCarloAnalysis`
7. `bindings/smoke_test.py` — `assert hasattr(ctrl, 'MonteCarloResult')`
8. `tests/test_catch2_advanced.cpp` — 4+ `[robustness_mc]` tests:
   - Spawned samples have perturbed A within expected range
   - All samples of a stable nominal plant return `is_stable=true` for small sigma
   - Unstable nominal → `instability_probability > 0`
   - `monteCarloAnalysis` end-to-end on a 2nd-order plant
9. `examples/ex83_robustness_mc.cpp` + `examples/python/ex103_robustness_mc.py`

**Estimated effort:** ~420 lines C++, ~50 lines bindings.

---

## Phase 2 — `SystemAnalysis` Extensions (Gang of Four + Disk Margin)

**Status: Done.** Built directly in `lib/SystemAnalysis.h`/`.cpp` (`gang_of_four`,
`calculate_disk_margin`). The write-up below is the original design plan kept for
reference, not an outstanding task.

**Priority: HIGH — frequency-domain robustness visualisation.**

### 2.1 Add to `lib/SystemAnalysis.h` / `lib/SystemAnalysis.cpp`

#### Disk margin (MIMO)

```cpp
struct DiskMargin {
    double alpha;          // disk half-angle: relates to simultaneous GM/PM tolerance
    double gain_margin;    // corresponding simultaneous gain margin (linear)
    double phase_margin_deg; // corresponding simultaneous phase margin
};

// Disk margin for a SISO or MIMO open-loop transfer L = G*K.
// Uses the maximum over frequency of sigma_max((I+L)^{-1}) (== ||S||_inf for SISO).
// DM = 1 / ||S||_inf.  Converts to equivalent simultaneous GM and PM.
// Reference: Blight et al. (1994); Seiler et al. (2020) "An Introduction to Disk Margins".
static DiskMargin calculateDiskMargin(const StateSpace& open_loop_L);
```

#### Gang of Four

```cpp
struct GangOfFour {
    StateSpace S;   // sensitivity      (I + G*K)^{-1}
    StateSpace T;   // comp. sensitivity G*K*(I + G*K)^{-1}  (= I - S)
    StateSpace GS;  // process sens.    G * S
    StateSpace KS;  // control sens.    K * S
};

// Build the Gang of Four from open-loop plant G and controller K (both SS, same Ts).
// Throws if dimensions are incompatible.
static GangOfFour gangOfFour(const StateSpace& G, const StateSpace& K);

// Convenience: peak Hinf norms of all four.  Uses calculateHInfinityNorm().
struct GangOfFourNorms {
    double norm_S, norm_T, norm_GS, norm_KS;
};
static GangOfFourNorms gangOfFourNorms(const GangOfFour& g4);
```

#### SS arithmetic helpers (needed by gangOfFour)

```cpp
// Series connection: sys_out = G2 * G1  (output of G1 feeds input of G2).
// Dimensions must be compatible: G1.p == G2.m.
static StateSpace series(const StateSpace& G1, const StateSpace& G2);

// Parallel connection: sys_out = G1 + G2 (same dimensions).
static StateSpace parallel(const StateSpace& G1, const StateSpace& G2);

// Feedback connection: closed-loop from r to y for unity negative feedback.
// Returns (I + G*K)^{-1} * G*K given forward-path G*K.
static StateSpace feedback(const StateSpace& GK);
```

### 2.2 Implementation notes

- `series` and `parallel` are standard state-space augmentation formulas.
  For `series(G1, G2)`:
  ```
  A = diag(A1, A2) + [0; B2*C1 | 0; 0]  (corrected standard form)
  B = [B1; B2*D1]
  C = [D2*C1, C2]
  D = D2*D1
  ```
- `feedback(GK)` computes `T = GK*(I+GK)^{-1}` and `S = I - T` using matrix
  inversion; the closed-loop A is `A_GK - B_GK*(I+D_GK)^{-1}*C_GK`.
- `DiskMargin::alpha` satisfies `GM = (1+alpha)/(1-alpha)`, `PM = 2*arcsin(alpha/2)`.
  Compute `alpha = 1/||S||_inf` directly.

### 2.3 Checklist

1. Extend `lib/SystemAnalysis.h` with `DiskMargin`, `GangOfFour`, `GangOfFourNorms` structs
   and `calculateDiskMargin`, `gangOfFour`, `gangOfFourNorms`, `series`, `parallel`, `feedback`
   declarations (≈80 lines header)
2. Extend `lib/SystemAnalysis.cpp` with implementations (≈200 lines)
3. Bindings: add `DiskMargin`, `GangOfFour`, `GangOfFourNorms`, all new methods (≈60 lines)
4. Smoke test assertions for `ctrl.DiskMargin`, `ctrl.GangOfFour`
5. Tests (4+ `[system_analysis_ext]`): series composition, Gang of Four norms on known plant,
   disk margin of well-tuned PID, `norm_S + norm_T` identity check
6. Add to `ex83_robustness_mc.cpp` or create `ex84_gang_of_four.cpp`

**Estimated effort:** ~280 lines C++, ~60 lines bindings.

---

## Phase 3 — `lib/MuAnalysis.h` (Structured Singular Value)

**Status: Done.** Built as `lib/MuAnalysis.{h,cpp}` (`compute_mu`/`peak_mu`/
`robust_stability_radius`). **Note the naming collision documented in `CLAUDE.md`'s
"Non-obvious API facts":** this is unrelated to `tools/mu_analysis.py`, which is an
ARMA(2,2)-identification heuristic over logged nonlinear closed-loop CSVs, not the real
structured singular value computed here. The write-up below is the original design plan
kept for reference, not an outstanding task.

**Priority: MEDIUM — most powerful deterministic robustness tool; highest effort.**

### 3.1 Scope

Implement μ upper-bound computation for **repeated real scalar** and **complex full-block**
uncertainty structures. This is the most commonly needed subset and matches what
`DiscreteHinf::solveMuSyn` already uses internally (D-scaling).

Full μ lower-bound (NP-hard in general) is deferred.

### 3.2 API

```cpp
// lib/MuAnalysis.h

namespace ctrl {

// One uncertainty block in the structured Delta matrix.
struct UncertaintyBlock {
    enum class Type {
        RealScalar,     // repeated real scalar: delta * I_{r x r}
        ComplexScalar,  // repeated complex scalar: delta * I_{r x r}
        ComplexFull     // full complex block: r_out x r_in
    };
    Type   type;
    int    r_out; // output dimension of the block
    int    r_in;  // input dimension (= r_out for scalar blocks)
};

// Description of the full Delta structure.
struct UncertaintyStructure {
    std::vector<UncertaintyBlock> blocks;
    int totalInputs()  const; // sum of r_in over blocks
    int totalOutputs() const; // sum of r_out over blocks
};

// Result at one frequency point.
struct MuBound {
    double upper;  // D-scaling upper bound on mu
    double lower;  // power-iteration lower bound (0 if not computed)
};

// Compute mu upper bound at each frequency in omega_grid using D-scaling.
// M:     frequency response of the interconnection matrix (same size at each omega)
// struc: block structure of Delta
// Returns one MuBound per frequency point.
std::vector<MuBound>
computeMu(const std::vector<Eigen::MatrixXcd>& M_freq,
          const UncertaintyStructure& struc,
          bool compute_lower_bound = false);

// Compute the peak mu (supremum over frequency) for a plant+controller pair.
// G:         nominal plant, K: controller
// struc:     uncertainty structure (must be compatible with G dimensions)
// sigma_rel: relative size of the uncertainty (scales M before computing mu)
// Returns peak MuBound and the frequency at which the peak occurs.
struct PeakMuResult {
    MuBound peak;
    double  peak_omega_rad_s;
    std::vector<MuBound> mu_curve; // full frequency curve
};

PeakMuResult
peakMu(const StateSpace& G, const StateSpace& K,
       const UncertaintyStructure& struc,
       double sigma_rel   = 0.1,
       int    freq_points = 200,
       double omega_min   = 1e-2);

// Robust stability radius: largest sigma_rel such that peak mu < 1.
// Equivalently, sup_omega mu(M(jw)) < 1 means robust stability is guaranteed.
// Uses bisection on sigma_rel in [0, sigma_max].
double
robustStabilityRadius(const StateSpace& G, const StateSpace& K,
                      const UncertaintyStructure& struc,
                      double sigma_max = 2.0,
                      int    bisect_iters = 30);

} // namespace ctrl
```

### 3.3 Algorithm (D-scaling upper bound)

For one frequency `omega`:

1. Evaluate `M = C*(z*I - A)^{-1}*B + D` at `z = exp(j*omega*Ts)` using
   `freqResponseGrid` from `GapMetric.h` (already available).
2. Initialise scaling matrices `D_L = I`, `D_R = I` (size = block structure).
3. Iterate:
   a. Compute `N = D_L * M * inv(D_R)`.
   b. Upper bound at this D: `sigma_max(N)` (largest singular value).
   c. Update D: for each complex full block, scale to equalise singular values
      of the corresponding sub-block of `M` (standard D-iteration step from
      Packard & Doyle 1993, equation 22).
4. Return `upper = min_{over iterations} sigma_max(D_L * M * inv(D_R))`.

For real scalar blocks, the D-scaling iteration must interleave with a G-scaling
step (Packard & Doyle 1993). For Phase 3, implement complex-block-only first;
add real-block support in a follow-up.

The infrastructure from `DiscreteHinf` (gamma bisection, DARE) already exists
and can be referenced but need not be duplicated.

### 3.4 Checklist

1. `lib/MuAnalysis.h` — structs + declarations (≈100 lines)
2. `lib/MuAnalysis.cpp` — `computeMu`, `peakMu`, `robustStabilityRadius` (≈350 lines)
3. `lib/CMakeLists.txt` — add `MuAnalysis.cpp`
4. `lib/ControllerToolbox.h` — `#include "MuAnalysis.h"`
5. `lib/Features.h` — `{"mu_analysis", true}`
6. Bindings (≈80 lines): `UncertaintyBlock`, `UncertaintyStructure`, `MuBound`,
   `PeakMuResult`, `computeMu`, `peakMu`, `robustStabilityRadius`
7. Smoke test: `assert hasattr(ctrl, 'UncertaintyStructure')`
8. Tests (5+ `[mu_analysis]`):
   - `UncertaintyStructure::totalInputs` for a known block list
   - `computeMu` returns `upper >= 0` for identity M
   - `computeMu` returns `upper <= 1` for small M
   - `peakMu` on a well-tuned second-order SISO plant ≈ known value
   - `robustStabilityRadius` > 0 for a stabilising controller
9. `examples/ex85_mu_analysis.cpp` + `examples/python/ex104_mu_analysis.py`

**Estimated effort:** ~450 lines C++, ~80 lines bindings.

---

## Phase 4 — `lib/WorstCaseSearch.h` (CMA-ES Worst-Case Parameter Search)

**Status: Done (Part 67).** Built as `lib/WorstCaseSearch.h`, header-only, wrapping
`AutoTuner`. One real deviation from the API sketch below: the search runs internally in
*normalised* coordinates (`z`), where physical parameter `i` is
`param_nominal(i) + z(i) * search_width(i)` and `search_width(i) = param_sigma(i) *
max(|param_nominal(i)|, 1)`. This lets `AutoTuner`'s single scalar `sigma0` (no
per-parameter step size) still explore every parameter at its own relative scale, and lets
hard bounds be supplied in physical units while CMA-ES itself only ever sees `z`-space
bounds. `WorstCaseSearchParams::population` is kept for API-shape compatibility with the
original sketch below but has no effect — `AutoTuner` derives its CMA-ES population size
internally from the dimension (`4 + floor(3*ln(n))`, Hansen 2006) and does not expose an
override; `max_evals` is honoured by dividing it by that same derived population size to
get `AutoTunerParams::maxIter`. The write-up below is otherwise unchanged from the original
design.

**Priority: MEDIUM — uses existing `AutoTuner`, low incremental code.**

### 4.1 Concept

Treat the uncertain plant parameters as the optimisation variables. Define a cost
function that returns the *negated* stability margin (or IAE). CMA-ES finds the
parameter vector that minimises the cost, i.e., maximises degradation.

### 4.2 API

```cpp
// lib/WorstCaseSearch.h  (header-only, ~150 lines)

namespace ctrl {

struct WorstCaseSearchParams {
    int    max_evals   = 500;
    double sigma_init  = 0.1;   // initial CMA-ES sigma (relative units)
    int    population  = 0;     // 0 = CMA-ES default (4 + floor(3*log(n)))
    uint32_t seed      = 42;
};

// Result of a worst-case search.
struct WorstCaseResult {
    Eigen::VectorXd worst_params; // parameter vector that achieved worst cost
    double          worst_cost;   // value at worst_params (the actual metric, not negated)
    bool            converged;
    int             n_evals;
};

// Find the uncertain plant (parameterised as a flat real vector) that maximises
// the closed-loop peak sensitivity ||S||_inf.
//
// plant_factory:  maps parameter vector -> StateSpace plant
// controller_ss:  fixed controller (pre-designed)
// param_nominal:  nominal parameter vector
// param_sigma:    per-parameter relative search width
// lower/upper:    hard bounds on each parameter (pass empty vectors for no bounds)
WorstCaseResult
findWorstCaseSensitivity(
    std::function<StateSpace(const Eigen::VectorXd&)> plant_factory,
    const StateSpace& controller_ss,
    const Eigen::VectorXd& param_nominal,
    const Eigen::VectorXd& param_sigma,
    const Eigen::VectorXd& lower_bounds = {},
    const Eigen::VectorXd& upper_bounds = {},
    const WorstCaseSearchParams& p = {});

// Same but maximises IAE (step response integral absolute error).
WorstCaseResult
findWorstCaseIAE(
    std::function<StateSpace(const Eigen::VectorXd&)> plant_factory,
    const StateSpace& controller_ss,
    const Eigen::VectorXd& param_nominal,
    const Eigen::VectorXd& param_sigma,
    const Eigen::VectorXd& lower_bounds = {},
    const Eigen::VectorXd& upper_bounds = {},
    double sim_duration_s = 50.0,
    const WorstCaseSearchParams& p = {});

// Generic worst-case: maximises user-supplied metric function.
// metric_fn(plant) -> double; higher = worse.
WorstCaseResult
findWorstCase(
    std::function<StateSpace(const Eigen::VectorXd&)> plant_factory,
    std::function<double(const StateSpace&)> metric_fn,
    const Eigen::VectorXd& param_nominal,
    const Eigen::VectorXd& param_sigma,
    const Eigen::VectorXd& lower_bounds = {},
    const Eigen::VectorXd& upper_bounds = {},
    const WorstCaseSearchParams& p = {});

} // namespace ctrl
```

### 4.3 Implementation notes

- Internally wraps `AutoTuner` (CMA-ES). The cost function passed to `AutoTuner`
  builds the plant from the parameter vector, computes the closed-loop SS,
  and returns the negated metric (so CMA-ES minimises → metric maximised).
- `AutoTuner`'s `CostFn` signature is `std::function<double(const Eigen::VectorXd&)>`,
  which matches exactly.
- Relative parameter bounds: multiply `param_sigma` by `param_nominal` to get
  absolute search widths before passing to `AutoTuner`.
- The `plant_factory` lambda is the user's responsibility; this keeps the API
  independent of how the plant parameterisation is defined.

### 4.4 Checklist

1. `lib/WorstCaseSearch.h` — header-only (~150 lines), includes `AutoTuner.h` and
   `RobustnessAnalysis.h`
2. `lib/ControllerToolbox.h` — `#include "WorstCaseSearch.h"`
3. `lib/Features.h` — `{"worst_case_search", true}`
4. Bindings (≈60 lines): `WorstCaseSearchParams`, `WorstCaseResult`, all three
   `findWorstCase*` variants with `py::function` for `plant_factory` and `metric_fn`
5. Smoke test: `assert hasattr(ctrl, 'WorstCaseResult')`
6. Tests (3+ `[worst_case_search]`):
   - Worst-case GM search finds lower GM than nominal for known perturbed plant
   - `findWorstCase` with identity metric returns `worst_cost ≈ metric(nominal)`
     when search space is epsilon-small
   - `findWorstCaseIAE` returns IAE worse than the nominal for a detuned controller
7. Add to `ex83_robustness_mc.cpp` or create `ex86_worst_case.cpp`

**Estimated effort:** ~150 lines C++ (header-only), ~60 lines bindings.

---

## Phase 5 — `lib/LyapunovRobustness.h` (Common Lyapunov Function for Polytopic Uncertainty)

**Status: Done (Part 67).** Built as `lib/LyapunovRobustness.h`, header-only, wrapping
`SystemAnalysis::solveDiscreteLyapunov`. **One real deviation from the algorithm in section
5.3 below: the aggregation step sums the per-vertex exact solutions `X_i`, it does not
average them** (the pseudocode's `P_new = P_new / L` line was dropped). This is not a
style choice — averaging is provably insufficient even for simple, clustered vertex sets.
Worked counterexample (`A_1=0.45, A_2=0.55, Q=1`, both individually stable): the exact
per-vertex solutions are `X_1 = 1.2539`, `X_2 = 1.4337`; their *average* `P=1.3438` fails
vertex 2's own decrease condition (`A_2^2*P - P + Q = +0.0627 > 0`, a violation), while
their *sum* `P=2.6876` satisfies both vertices comfortably (`-1.1434` and `-0.8746`). The
reason: for a candidate `P = sum_i X_i`, checking vertex `k` reduces algebraically to
`A_k^T P A_k - P + Q = sum_{i != k} (A_k^T X_i A_k - X_i)` — the `i = k` term cancels
exactly against `Q` by construction, so summing preserves every vertex's own exact margin
and only adds the (for nearby vertices, typically small and same-signed) cross terms on
top; dividing by `L` instead shrinks the very term that vertex `k` needs to stay negative.
This is still a heuristic, not a true SDP solver (per the scope note below), but the sum
form is the one that actually works for the intended use case (vertices clustered around a
common nominal, e.g. from `buildBoxVertices`) and is confirmed by both a dedicated unit
test and the worked example above.

**Priority: LOW — requires internal iterative solver; no external SDP dependency.**

### 5.1 Scope

For a **polytopic** uncertain system `A(t) ∈ conv{A_1, ..., A_L}` (convex hull of
vertex matrices), find a common quadratic Lyapunov function `V(x) = xᵀPx` such
that `A_i^T P A_i - P < 0` for all vertices `i`. If such `P` exists, the switching
system is quadratically stable.

Since the project uses only Eigen (no OSQP/SDPA), the solver uses a **projected
power iteration** (Lim & How 2002): alternately update `P` via the Lyapunov
equation at each vertex and project back to the positive-definite cone.
This converges for quadratically stable polytopes but may diverge if no common
Lyapunov function exists.

### 5.2 API

```cpp
// lib/LyapunovRobustness.h  (header-only, ~200 lines)

namespace ctrl {

struct LyapunovSearchParams {
    int    max_iter     = 200;
    double tol          = 1e-6;  // convergence: ||P_new - P_old||_F / ||P||_F
    double regularise   = 1e-8;  // added to diagonal to maintain PD
};

struct LyapunovResult {
    bool           found;     // true if a common P was found to tol
    Eigen::MatrixXd P;        // common Lyapunov matrix (n x n, symmetric PD)
    double         residual;  // max over vertices of ||A_i^T P A_i - P + Q||_F
    int            iterations;
};

// Find a common quadratic Lyapunov function for a polytopic system.
// vertices:  set of vertex A matrices A_1..A_L (each n x n).
// Q:         positive-definite Q matrix (defaults to I if empty).
// Returns LyapunovResult with found=true if a common P exists.
LyapunovResult
findCommonLyapunov(const std::vector<Eigen::MatrixXd>& vertices,
                   Eigen::MatrixXd Q = {},
                   const LyapunovSearchParams& p = {});

// Quadratic stability certificate: true iff all vertices are Schur-stable
// (necessary but not sufficient) AND a common Lyapunov function is found.
bool
isQuadraticallyStable(const std::vector<Eigen::MatrixXd>& vertices,
                      const LyapunovSearchParams& p = {});

// Build vertex matrices from a nominal A and a list of perturbation directions.
// vertices = {A + delta_i : delta_i ∈ Delta} where Delta is a box uncertainty.
// uncertainty_cols: each column of this matrix is added/subtracted from A,
//                   generating 2^m vertex matrices for m perturbation directions.
// Warning: 2^m grows fast; keep m <= 10.
std::vector<Eigen::MatrixXd>
buildBoxVertices(const Eigen::MatrixXd& A_nominal,
                 const Eigen::MatrixXd& uncertainty_cols);

} // namespace ctrl
```

### 5.3 Algorithm detail (projected power iteration)

```
Initialise P = I_n
For iter = 1..max_iter:
    P_new = 0
    For each vertex A_i:
        Solve discrete Lyapunov: A_i^T * X * A_i - X = -Q - regularise*I
        (use SystemAnalysis::solveDiscreteLyapunov which already exists)
        P_new = P_new + X
    P_new = P_new / L         // average across vertices
    Project to PD cone:       // eigendecomposition, clamp eigenvalues >= regularise
    If ||P_new - P||_F / ||P||_F < tol: found = true; break
    P = P_new
Check residual: for each vertex compute ||A_i^T P A_i - P + Q||_F
Return found = (max residual < sqrt(tol))
```

The existing `SystemAnalysis::solveDiscreteLyapunov` handles each vertex step.

### 5.4 Checklist

1. `lib/LyapunovRobustness.h` — header-only (~200 lines), includes `SystemAnalysis.h`
2. `lib/ControllerToolbox.h` — `#include "LyapunovRobustness.h"`
3. `lib/Features.h` — `{"lyapunov_robustness", true}`
4. Bindings (≈50 lines): `LyapunovSearchParams`, `LyapunovResult`,
   `findCommonLyapunov`, `isQuadraticallyStable`, `buildBoxVertices`
5. Tests (4+ `[lyapunov_robustness]`):
   - `findCommonLyapunov({A})` recovers standard Lyapunov P for single stable A
   - Polytope of stable vertices → `found = true`
   - Polytope including an unstable vertex → `found = false` (not guaranteed but
     typical for highly unstable vertices)
   - `buildBoxVertices` produces `2^m` matrices
6. Add to or create `ex87_lyapunov_robust.cpp`

**Estimated effort:** ~200 lines C++ (header-only), ~50 lines bindings.

---

## Summary Table

| Phase | File(s) | New API | Est. C++ lines | Priority | Depends on | Status |
|-------|---------|---------|----------------|----------|------------|--------|
| 1 | `lib/RobustnessAnalysis.{h,cpp}` | `spawn_*`, `MonteCarloResult`, `runMonteCarlo` | ~420 | HIGH | `GapMetric`, `SystemAnalysis` | Done (Part 63) |
| 2 | `lib/SystemAnalysis.{h,cpp}` | `gangOfFour`, `diskMargin`, `series`, `parallel`, `feedback` | ~280 | HIGH | `SystemAnalysis` (extension) | Done (Part 63) |
| 3 | `lib/MuAnalysis.{h,cpp}` | `computeMu`, `peakMu`, `robustStabilityRadius` | ~450 | MEDIUM | Phase 1, `GapMetric` | Done (Part 63) |
| 4 | `lib/WorstCaseSearch.h` | `findWorstCase*` | ~150 | MEDIUM | Phase 1, `AutoTuner` | Done (Part 67) |
| 5 | `lib/LyapunovRobustness.h` | `findCommonLyapunov`, `buildBoxVertices` | ~200 | LOW | `SystemAnalysis` | Done (Part 67) |

**Total new code:** ~1500 lines C++ + ~300 lines bindings. All five phases are complete —
the table above is kept for traceability, not as an outstanding work plan.
All five phases follow the standard 8-step checklist from CLAUDE.md.

---

## Recommended Build Order

```
Phase 1 (MC)  →  Phase 2 (Gang of Four)  →  Phase 4 (Worst-Case)
                                          →  Phase 3 (Mu)
                                          →  Phase 5 (Lyapunov)
```

Phases 3, 4, and 5 can proceed in parallel once Phase 1 is complete.
Phase 2 is self-contained and can be done before or after Phase 1.

---

## Non-obvious Caveats

```
SystemAnalysis::solveDiscreteLyapunov  ->  O(n^6) via Kronecker; use n <= 10 for Phase 5.
                                           For n > 10 a Bartels-Stewart solver is needed.
Gang of Four 'feedback' formula        ->  requires (I + D_GK) to be invertible; singular
                                           when plant has pure integrator and K has no
                                           rolloff. Add regularisation: (I + D_GK + eps*I).
MuAnalysis D-scaling                   ->  converges for complex blocks; real scalar blocks
                                           need G-scaling (defer to follow-up).
CMA-ES worst-case search               ->  non-convex; may find local minima. Run with
                                           multiple seeds if critical.
spawn_SS_samples sigma_B = sigma_C = 0  -> perturbs only A (most common use case for
                                            state-matrix uncertainty in robust control).
LyapunovRobustness buildBoxVertices     -> 2^m vertices; m > 12 is impractical.
                                           For high-dimensional uncertainty use MC instead.
WorstCaseSearch search_width            -> param_sigma(i) * max(|param_nominal(i)|, 1), NOT
                                           param_sigma(i) * |param_nominal(i)| alone - the
                                           floor at 1 keeps small-magnitude nominal parameters
                                           (e.g. nominal = 0.01) from collapsing to a useless
                                           near-zero search width.
WorstCaseSearchParams::population       -> reserved field, currently has NO effect; AutoTuner
                                           derives CMA-ES population internally from n and does
                                           not expose an override (Part 67).
findCommonLyapunov aggregation          -> SUMS per-vertex exact solutions, does NOT average
                                           (Part 67 deviation from section 5.3's pseudocode -
                                           averaging fails simple clustered-vertex cases; see
                                           the Phase 5 status note above for the worked example).
findCommonLyapunov residual sign        -> negative = vertex satisfied with margin, non-negative
                                           = violated; `found = (residual < sqrt(tol))`, where
                                           residual is the worst-case (max over vertices) largest
                                           eigenvalue of `A_i^T P A_i - P + Q` (an eigenvalue
                                           check, not the Frobenius-norm reading a literal pass
                                           over section 5.3 might suggest - the Frobenius norm of
                                           a comfortably-satisfied, very negative-definite matrix
                                           would itself be large, which is backwards from what
                                           "small residual = good" should mean).
```

---

## Integration with Existing Case Studies

**Correction (Part 66): this is not the path that was actually taken.** Phase 1 has been
complete since Part 63, but case-study integration did **not** go through
`ctrl.monteCarloAnalysis`/`grey_box_model()`/`run_robustness()` as originally proposed
below. Part 64 deliberately built a separate, independent mechanism instead —
`case-study/common/RobustnessStats.h` + a per-study `robustness_main.cpp`/`*_robustness`
CMake target that perturbs real physical plant parameters and reruns the actual nonlinear
closed-loop C++ simulation — because `ctrl.monteCarloAnalysis`'s linearized-`StateSpace`
Monte Carlo isn't meaningful for the SMC/ADRC/Fuzzy/GA-tuned nonlinear controllers most
case studies actually use. This pattern now covers all 10 C++ case studies (3 in Part 64,
the remaining 7 in Part 66; see `CLAUDE.md`'s ROB-1 entry). It remains unused (and the
paragraph below remains a live, unexecuted proposal) for the ~21 case studies that are
Python-only or not yet implemented — those would need either an equivalent Python-side
nonlinear-perturbation helper, or the original `ctrl.monteCarloAnalysis`-based approach
below if a future session judges a linearized-model check sufficient for a particular
study's controller roster.

Original proposal (not executed as written — see correction above):

Once Phase 1 is complete, any Python-only case study can add a
`grey_box_model() -> (ode, h, x0, names, lb, ub)` hook (already used by
`tools/model_validation.py`) and a `run_robustness()` hook that calls
`ctrl.monteCarloAnalysis`. The `tools/run_analysis.py` framework from Part 60 is
the natural host for scheduling robustness runs alongside MC and fault sweeps.
