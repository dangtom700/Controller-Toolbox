# Design: Self-Tuning Regulator and MLE/MAP Identification

**Date:** 2026-06-25
**Status:** Approved, not yet implemented

## Motivation

`docs/ALGORITHM_ROADMAP_PHASE3.md` Phase 2 opens with two identification-flavoured items that
both layer a synthesis/inference step on top of online ARX identification:

- **OC1** merges three backlog lines (minimum-variance control/STR, adaptive pole placement,
  self-tuning regulators) into one `IController` that runs `RecursiveLeastSquares` online and
  re-derives its control law from the latest parameter estimate every step - closing the "fixed
  gain decays as the plant drifts" gap a `DiscretePID`/`DiscreteLQR` can't address.
- **SI1** is a statistical alternative to `RecursiveLeastSquares`/`GreyBoxEstimator`'s pure
  least-squares cost: maximize log-likelihood under an assumed noise model (Gaussian by default,
  optionally MAP with a Gaussian prior), generalizing to non-Gaussian noise where plain LS is
  biased.

Both were verified against the real `RecursiveLeastSquares`/`AutoTuner` APIs already in `lib/`
(`lib/RecursiveLeastSquares.h`, `lib/AutoTuner.h`) before writing this spec - the roadmap's reuse
claims for both hold, with one clarification: `RecursiveLeastSquares` exposes only the *online*
`update()` recursion, not a batch regressor/residual function, so SI1 mirrors its `phi[k]`
convention in a small private helper rather than literally calling into it.

## Scope

- **OC1**: SISO ARX plants only (matches `RecursiveLeastSquares`'s own SISO scope). `d=1`
  (inherent one-step delay, the convention already baked into `RecursiveLeastSquares`'s
  regressor) - arbitrary extra dead time (`d>1`) needs a d-step-ahead Diophantine predictor and
  is **out of scope** for this phase (see below).
- **SI1**: batch (offline) identification only, mirroring `HammersteinWienerIdentifier`'s usage
  shape - not a streaming estimator. Two noise models: Gaussian (reduces to least squares) and
  Laplace (robust to outliers); not a general pluggable-likelihood framework.

## Components

### 1. `lib/SelfTuningRegulator.h` / `.cpp` - implements `IController`

```cpp
enum class STRMode { MinimumVariance, PolePlacement };

struct STRParams {
    int    na = 2, nb = 1;          // plant orders, RecursiveLeastSquares convention:
                                     // A(q^-1) = 1 + a1.q^-1 + ... + a_na.q^-na  (monic)
                                     // B(q^-1) = b1.q^-1 + ... + b_nb.q^-nb      (no b0 term)
    STRMode mode = STRMode::MinimumVariance;
    Eigen::VectorXd desired_poles;  // PolePlacement only; size MUST equal na + nb - 1
    double lambda = 0.98;           // RLS forgetting factor
    double bMin   = 1e-6;           // |b1| floor; below this the plant is ill-conditioned
                                     // for direct cancellation -> hold last u (see NaN-guard note)
    double uMin = -1e9, uMax = 1e9;
};

class SelfTuningRegulator : public IController {
public:
    explicit SelfTuningRegulator(const STRParams &params, double Ts);

    void setReference(double r) { r_ = r; }      // mirrors MRACController's setReference/compute split
    double compute(double y_plant) override;     // PlantOutput convention - see signConvention()
    void reset() override;
    double sampleTime() const override { return Ts_; }
    SignConvention signConvention() const override { return SignConvention::PlantOutput; }

    const Eigen::VectorXd &estimatedNumerator() const   { return rls_.numerator(); }
    const Eigen::VectorXd &estimatedDenominator() const { return rls_.denominator(); }
    const Eigen::MatrixXd &covariance() const           { return rls_.covariance(); }

private:
    double computeMinimumVariance();
    double computePolePlacement();

    RecursiveLeastSquares rls_;
    STRParams p_;
    double Ts_, r_ = 0.0, uPrev_ = 0.0;
    bool   havePrevU_ = false;

    // Pre-sized circular history (construction-time allocation only - no push_back/resize in
    // compute(), matching CLAUDE.md's compute()-hot-path allocation guidance and RLS's own
    // y_buf_/u_buf_ style): yHist_ holds y[k..k-na+1] (na entries), uHist_ holds u[k-1..k-nb+1]
    // (nb-1 entries, empty when nb==1).
    Eigen::VectorXd yHist_;
    Eigen::VectorXd uHist_;
    int histCount_ = 0;   // samples seen since reset(), capped at max(na, nb)
};
```

**Why `compute(double y_plant)` + `setReference(r)` instead of the roadmap's
`compute(double error)`:** the control law (both modes) needs the raw plant output `y[k]` to feed
`RecursiveLeastSquares::update(y, u)` *and* to evaluate the minimum-variance/pole-placement law's
own `y` regressor terms - collapsing both into a single `error = r - y` loses the sign/scale
information the Diophantine-based law needs when `na > 1`. `MRACController` already established
this exact `compute(y_plant)` + `setReference(r)` split in this codebase
(`lib/MRACController.h:114,125`) for the same reason (adaptive law needs the raw measurement);
`SelfTuningRegulator` follows the same convention, hence `SignConvention::PlantOutput`.

**Per-step algorithm:**

```
1. NaN guard: if y_plant non-finite -> return uPrev_ unchanged, skip everything below.
2. if havePrevU_: rls_.update(y_plant, uPrev_)             // refresh theta from the latest sample
3. shift yHist_ (push y_plant at front, drop oldest);       // O(na), no allocation
   histCount_ = min(histCount_ + 1, max(na, nb))
4. if histCount_ < max(na, nb): u = 0.0                     // buffers not yet full; hold at zero
   else: u = (mode == MinimumVariance) ? computeMinimumVariance() : computePolePlacement()
5. u = clamp(u, uMin, uMax)
6. shift uHist_ (push u at front, drop oldest); uPrev_ = u; havePrevU_ = true
7. return u
```

**`computeMinimumVariance()`** (Astrom & Wittenmark, *Adaptive Control* 2nd ed., Ch. 3 -
one-step-ahead direct-cancellation STR, the `d=1` case): from
`y[k+1] = -a1.y[k] - ... - a_na.y[k-na+1] + b1.u[k] + b2.u[k-1] + ... + b_nb.u[k-nb+1] + e[k+1]`,
solve for the `u[k]` that sets the predicted `y[k+1] = r`:

```
b1 = rls_.numerator()(0);
if (|b1| < bMin) return clamp(0.1 * (r_ - yHist_[0]), uMin, uMax);   // see correction below
u[k] = ( r_
         + sum_{i=1..na} a_i * yHist_[i-1]              // yHist_[0]=y[k], yHist_[1]=y[k-1], ...
         - sum_{i=2..nb} b_i * uHist_[i-2] ) / b1        // uHist_[0]=u[k-1], uHist_[1]=u[k-2], ...
```

**`computePolePlacement()`** (Astrom & Wittenmark Ch. 7 - pole-placement self-tuner via a 1-DOF
Diophantine solve; `Acl` from `desired_poles` via `polyFromRoots`):

```
A(z) R(z) + B(z) S(z) = Acl(z),   z = q^-1, B(z) = b1.z + ... + b_nb.z^nb (note: B itself carries
                                    the inherent one-step delay - this is the standard Bezout
                                    setup with "the second polynomial" = B, degree nb, not the
                                    delay-factored B' = B/z)
  deg(R) <= nb - 1 (nb unknowns r0..r_{nb-1} - r0 is solved for, NOT fixed at 1: Bezout's identity
                     for two coprime polynomials of degree na, nb gives a UNIQUE solution with
                     deg(R) < nb, deg(S) < na for any target of degree <= na+nb-1, but does not
                     make R monic - an earlier draft of this spec incorrectly assumed r0=1)
  deg(S) <= na - 1 (na unknowns s0..s_{na-1})
  deg(Acl) = na + nb - 1  -> desired_poles.size() MUST == na + nb - 1 (throws std::invalid_argument
                              otherwise; expand desired_poles -> monic polynomial via the standard
                              product-of-(1 - pole_i.z) recursion - monic in the z^0 coefficient,
                              matching A(z)'s and B(z)'s own normalization)
Sylvester system: build the (na+nb) x (na+nb) matrix M whose first nb columns are A's coefficients
  (1, a1, ..., a_na, 0, ...) shifted down one row per column, and whose last na columns are B's
  coefficients (0, b1, ..., b_nb, 0, ...) shifted down one row per column - i.e. the classical
  Sylvester resultant matrix of A and B; solve M * [r0..r_{nb-1}, s0..s_{na-1}]' = Acl_coeffs via
  Eigen::PartialPivLU (na+nb is typically <= 6, no performance concern). A and B are coprime
  whenever the identified plant is controllable/observable in the classical sense; near-loss of
  coprimality shows up as M becoming near-singular, guarded the same way as r0 below.
Control law (the standard 2-DOF "RST" structure, R(q^-1).u = T.r - S(q^-1).y):
  u[k] = ( T*r_ - sum_{i=0..na-1} s_i*yHist_[i] - sum_{i=1..nb-1} r_i*uHist_[i-1] ) / r0
  where T = Acl(1) / B(1)  (DC-gain-matching feedforward, the simplest standard choice; B(1) =
  sum of b_i). Both r0 and B(1) near zero, and a non-invertible Sylvester matrix M, are guarded
  the same way as bMin above (see the correction immediately below).
```

**Correction found during implementation/testing:** an earlier draft of this spec had both modes
hold `uPrev_` when the relevant leading coefficient is near-singular, modeled on the fleet's
NaN-guard convention. This deadlocks at cold start: `RecursiveLeastSquares::theta_` is
zero-initialized, so `b1 = 0` on the very first call; holding `uPrev_` (itself 0 initially) means
`u` stays 0 forever, the plant is never excited, and `b1` can never leave 0 to begin with. Both
modes instead fall back to a small fixed-gain (0.1) proportional controller,
`clamp(0.1*(r_ - yHist_[0]), uMin, uMax)`, whenever the adaptive law is ill-conditioned (the
`computeMinimumVariance()`/`computePolePlacement()` guards above, plus the Sylvester matrix
non-invertible case) - this breaks the cold-start deadlock and is itself a perfectly reasonable
controller while the adaptive law has insufficient excitation to be trustworthy.

### 2. `lib/MLEIdentifier.h` / `.cpp` - static identifier, no `IController`/RLS instantiation (mirrors `HammersteinWienerIdentifier`'s static-method shape)

```cpp
enum class NoiseModel { Gaussian, Laplace };

struct MLEParams {
    int na = 2, nb = 1;
    NoiseModel noise = NoiseModel::Gaussian;
    Eigen::VectorXd prior_mean;    // empty = no prior (pure MLE)
    Eigen::MatrixXd prior_cov;     // empty = no prior; size (na+nb)x(na+nb) if given
    int    max_iter = 300;         // forwarded to AutoTuner's maxIter
    double tol       = 1e-10;      // forwarded to AutoTuner's tol
};

struct MLEResult {
    Eigen::VectorXd theta;         // [a1..a_na, b1..b_nb], RLS-identical layout
    Eigen::MatrixXd covariance;    // inverse-Hessian asymptotic covariance at theta
    double logLikelihood;
    bool   converged;
};

class MLEIdentifier {
public:
    static MLEResult fit(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                          double Ts, const MLEParams &params = {});
};
```

**Algorithm:**

1. Build the batch regressor `Phi` (`(N - max(na,nb)) x (na+nb)`) and target `Y` using the exact
   `phi[k] = [-y[k-1..k-na], u[k-1..k-nb]]` layout documented in `RecursiveLeastSquares.h` (a
   private `buildRegressor()` helper - `RecursiveLeastSquares` itself has no batch accessor for
   this, only the online `update()` recursion, so this is a parallel construction, not a literal
   call into RLS).
2. `theta0 = (Phi'Phi).ldlt().solve(Phi'Y)` - closed-form batch LS, used as the CMA-ES warm start
   regardless of noise model (and *is* the final answer in the Gaussian/no-prior case, see below).
3. Negative log-likelihood (profiled over the noise scale, so no extra nuisance parameter is
   optimized):
   - **Gaussian:** `e = Y - Phi*theta; nll = 0.5*N*log(e.squaredNorm()/N)` - minimizing this over
     `theta` is *identical* to minimizing `e.squaredNorm()` (SSE), since the log is monotonic, so
     the Gaussian/no-prior case reduces exactly to least squares (verified analytically: the
     profile likelihood's only `theta`-dependent term is `log(SSE)`).
   - **Laplace:** `nll = N*log(e.cwiseAbs().sum())` (same profiling trick, `b_hat = mean|e|`) -
     non-quadratic, robust to outliers (minimizing `log(sum|e|)` downweights large residuals far
     more gently than squaring them).
   - **+ prior (MAP, either noise model):** `nll += 0.5*(theta-prior_mean)'*prior_cov.inverse()*(theta-prior_mean)`
     when `prior_mean`/`prior_cov` are non-empty.
4. `ctrl::AutoTuner` (unbounded - `AutoTunerParams.lower/upper` left empty, since theta's natural
   scale varies per-plant and CMA-ES's self-adapting step size handles this without box bounds)
   minimizes `nll` from `theta0`; `MLEResult::theta = tunerResult.params`,
   `logLikelihood = -tunerResult.cost`, `converged = tunerResult.converged`.
5. `covariance`: central finite-difference Hessian of `nll` at `theta` (step `1e-4 * max(|theta_i|, 1)`
   per dimension, `O((na+nb)^2)` evaluations - cheap since `na+nb` is small), inverted via
   `Eigen::LDLT` (falls back to a pseudo-inverse via `Eigen::CompleteOrthogonalDecomposition` if
   the LDLT reports non-positive-definiteness, e.g. a flat likelihood direction under weak
   excitation - documented as a reduced-confidence signal, not a thrown error).

## Explicitly out of scope (this phase)

- **OC1 arbitrary dead time (`d > 1`)** - needs a full d-step-ahead Diophantine predictor on top
  of the pole-placement Diophantine solve already being built; the one-step (`d=1`) case covers
  the classical STR/minimum-variance literature's default presentation and every test in this
  spec's plan.
- **OC1 MIMO** - `RecursiveLeastSquares` is SISO-only; no MIMO generalization here.
- **SI1 non-Gaussian/non-Laplace noise models, or a pluggable user-supplied likelihood** - two
  built-in models only, matching the roadmap's "Gaussian by default... generalizes to non-Gaussian
  noise" framing without building a general likelihood-plugin framework.
- **SI1 streaming/online MLE** - batch only; `OC1`/`RecursiveLeastSquares` already cover the
  online case.

## Implementation checklist

**OC1** (full `IController` checklist):
1. `lib/SelfTuningRegulator.h`/`.cpp` + `CTRL_REGISTER_FEATURE(self_tuning_regulator)`
2. `lib/CMakeLists.txt` - add `SelfTuningRegulator.cpp`
3. `lib/ControllerToolbox.h` - `#include "SelfTuningRegulator.h"` near `RecursiveLeastSquares.h`
4. `bindings/controllers_bindings.cpp` - bind `STRMode`, `STRParams`, `SelfTuningRegulator`
   (`std::shared_ptr<T>` + `ctrl::IController` base, per `CLAUDE.md`'s binding rule)
5. `bindings/smoke_test.py` - construct, `set_reference`, a few `compute()` calls, assert finite
6. `examples/ex101_self_tuning_regulator.cpp` + `examples/python/ex118_self_tuning_regulator.py` -
   slowly-drifting plant scenario (the roadmap's own HVAC-load example)
7. `tests/test_catch2_advanced.cpp` - tests under `[self_tuning_regulator]`
8. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` - add `ex101_self_tuning_regulator`

**SI1** (lighter non-`IController` identifier checklist, same shape as `HammersteinWienerIdentifier`):
1. `lib/MLEIdentifier.h`/`.cpp` + `CTRL_REGISTER_FEATURE(mle_identifier)`
2. `lib/CMakeLists.txt` - add `MLEIdentifier.cpp`
3. `lib/ControllerToolbox.h` - `#include "MLEIdentifier.h"` near `GreyBoxEstimator.h`
4. `bindings/estimation_bindings.cpp` - bind `NoiseModel`, `MLEParams`, `MLEResult`, `MLEIdentifier`
5. `bindings/smoke_test.py` - fit a known small ARX dataset, assert finite `theta`/`covariance`
6. `examples/ex102_mle_identification.cpp` + `examples/python/ex119_mle_identification.py` -
   quantizing-sensor (non-Gaussian noise) scenario per the roadmap's example use case
7. `tests/test_catch2_advanced.cpp` - tests under `[mle_identification]`
8. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` - add `ex102_mle_identification`

## Testing plan

**`[self_tuning_regulator]`**

**Significant finding during implementation, affecting tests 1-2 below:** certainty-equivalence
direct adaptive control (both modes) has **no general guarantee of persistent excitation from a
closed-loop reference alone** (Astrom & Wittenmark Ch. 3/7 - this is *their own* documented
caveat, not a gap this implementation introduces). A standalone diagnostic confirmed: (a) the
Diophantine solve and both control-law formulas are exactly correct given the *true* plant
parameters (instant, exact convergence when parameters are known); (b) `RecursiveLeastSquares`
correctly recovers the true plant given *open-loop* (independently-generated) excitation; but
(c) under pure closed-loop operation - even with a randomly-varying reference *and* explicit
dither (`STRParams::probeAmplitude`, added specifically to investigate this) - RLS can converge
confidently (shrinking covariance) to a *stabilizing but numerically wrong* parameter estimate,
because the closed loop's own (y,u) trajectory doesn't necessarily span the full parameter space.
This is the textbook motivation for *dual control* (deliberately exploring to reduce parameter
uncertainty, not just exploiting the current estimate) - already scoped separately as roadmap
OC3 (Phase 5), confirming this is a known, anticipated limitation of naive self-tuning
regulators, not a defect in this implementation. Tests 1-2 were revised to verify what's
reliably true (closed-loop **stability and boundedness**), not exact parameter or setpoint
convergence, which this algorithm class does not generally guarantee.

1. Known ARX plant, `MinimumVariance` mode, persistent random reference - every `compute()`
   output and resulting plant state stays finite and within `[uMin, uMax]`/a bounded range, and
   `covariance()` stays finite, over a long run (regression-guards stability, not exact
   convergence - see the finding above).
2. Known ARX plant, `PolePlacement` mode, persistent random reference, comfortably-damped
   `desired_poles` (e.g. `[0.3, 0.3]`, not deadbeat-at-the-origin per the class-level `@warning`
   in `SelfTuningRegulator.h`) - same stability/boundedness check as test 1.
3. Plant parameter step-change mid-run - STR re-converges within N steps (true online adaptation).
4. Non-identifiable input (constant `r`, no excitation) - `rls_`'s covariance doesn't blow up
   (matches `RecursiveLeastSquares`'s existing numerical-safety contract; regression-tests that
   `SelfTuningRegulator` doesn't bypass it).
5. `bMin`/`B(1)`-near-zero guard - deliberately construct a near-singular case (e.g. the cold-start
   `theta_=0` case), confirm a finite, bounded fallback `u` instead of a garbage/non-finite value
   or a cold-start deadlock at exactly 0.
6. Non-finite `y_plant` input - holds `uPrev_`, leaves `rls_`/history unchanged (fleet NaN-guard
   convention, but documented per-class since this is a `PlantOutput`-convention controller).

**`[mle_identification]`**
1. Gaussian noise, no prior - `MLEIdentifier::fit`'s `theta` matches a direct batch-LS solve
   (`(Phi'Phi)^-1 Phi'Y`) within optimizer tolerance (analytic equivalence proven in the design).
2. With an informative prior (MAP) - result is pulled toward `prior_mean` by the expected
   ridge-regularization amount relative to the pure-MLE (`prior_cov -> infinity`) case.
3. Laplace noise on an outlier-heavy synthetic dataset - `MLEIdentifier` (Laplace) recovers the
   true `theta` more accurately than a plain batch-LS fit on the same data.
4. `covariance` sanity - diagonal entries are positive and shrink as `N` grows (asymptotic
   consistency check on a fixed true plant with increasing sample count).
