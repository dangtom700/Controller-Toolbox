# Design: Set-Membership Estimation and Particle Filter Variants

**Date:** 2026-06-25
**Status:** Approved, not yet implemented

## Motivation

Phase 2's two estimation items both extend `lib/`'s state-estimation family in a direction no
existing class covers:

- **EF2** is bounded-error (not probabilistic) state estimation: given known noise *bounds*, not
  a distribution, maintain a guaranteed feasible ellipsoid containing the true state. Structurally
  distinct from every probabilistic filter already in `lib/` (`KalmanFilter`, EKF/UKF,
  `ParticleFilter`).
- **EF3** extends `ParticleFilter.h` beyond its existing bootstrap/SIR baseline with an auxiliary
  particle filter (look-ahead resampling) and a Rao-Blackwellized particle filter (analytic
  marginalization of a linear-Gaussian substate via an embedded `KalmanFilter` per particle).

`lib/ParticleFilter.h` was read in full before writing this spec: `predict()`/`update()`/`step()`
are **not virtual** and every data member is `private`. The roadmap's
`class ParticleFilterV2 : public ParticleFilter` sketch, as written, would compile but couldn't
actually override the algorithm - calling `predict()` on a `ParticleFilterV2&` (or even a
`ParticleFilterV2*`) would always run the *base* bootstrap logic. This spec includes the (small,
additive, behavior-preserving) base-class change needed to make the roadmap's inheritance claim
true.

## Scope

- **EF2**: linear plants (`StateSpace`) only; isotropic (componentwise-bound-derived) ellipsoidal
  process/measurement noise sets, since `SetMembershipParams` only exposes scalar `w_bound`/
  `v_bound` (matching the roadmap's own struct - a general anisotropic ellipsoid bound is a
  straightforward future extension, not built here).
- **EF3 Auxiliary mode**: the look-ahead reweighting only changes behavior through `step()`
  (predict+update called together, the common usage pattern per `ParticleFilter`'s own docs).
  Calling `predict()`/`update()` separately on an `Auxiliary`-mode filter is well-defined but
  degrades to plain bootstrap behavior for that call (documented, not an error) - splitting the
  look-ahead resample across two independently-callable methods would require either buffering
  `y` across calls or accepting a measurement in `predict()`, both worse than this restriction.
- **EF3 Rao-Blackwellized mode**: the linear substate's dynamics (`A_lin`, `B_lin`, `Q_lin`) are
  **fixed (LTI)**, and the nonlinear/linear coupling in the measurement is **additively
  separable** - `h(x, u) = h_nonlinear(x_nl, u) + C_lin * x_lin` exactly. This is the standard
  "mixed linear/nonlinear measurement coupling" special case in the marginalized-PF literature
  (Schon, Gustafsson & Nordlund 2005), not the fully general conditionally-linear-dynamics case
  (where `A_lin`/`C_lin` themselves depend on the nonlinear substate) - that generalization is
  future work, flagged below.

## Components

### 0. `lib/ParticleFilter.h` - base-class change enabling real inheritance (additive, zero behavior change for direct `ParticleFilter` use)

```cpp
class ParticleFilter {
public:
    ParticleFilter(...);                 // unchanged
    virtual ~ParticleFilter() = default;  // NEW - required once the class is a polymorphic base
    void initialise(...);                 // unchanged (construction-time only, not overridden)
    virtual void predict(const Eigen::VectorXd &u);                                  // was non-virtual
    virtual void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u);          // was non-virtual
    virtual void step(const Eigen::VectorXd &y, const Eigen::VectorXd &u_prev);       // was non-virtual
    virtual void resample();                                                          // was non-virtual
    // ... all existing accessors unchanged (state(), covariance(), effectiveSampleSize(), ...)
protected:                              // CHANGED from private - same members, same layout
    void sampleNoise(Eigen::VectorXd &out, const Eigen::MatrixXd &L);
    ParticleFilterParams p_;
    int n_states_, n_meas_;
    ParticleFn f_;
    ParticleMeasFn h_;
    std::vector<Eigen::VectorXd> particles_;
    Eigen::VectorXd w_;
    Eigen::MatrixXd L_Q_, R_inv_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};
    std::uniform_real_distribution<double> uniform_{0.0, 1.0};
    double resample_thresh_;
    bool initialised_{false};
    int resample_count_{0};
    std::vector<Eigen::VectorXd> resample_buf_;
    Eigen::VectorXd cdf_;
};
```

No method body changes in `ParticleFilter.cpp` - only the `private:` -> `protected:` access
specifier and four `virtual` keywords. Existing callers that hold a `ParticleFilter` (not
`ParticleFilterV2`) value or reference see byte-identical behavior (virtual dispatch on a concrete
base instance calls the base's own override, i.e. itself). This mirrors the precedent already set
twice in this codebase for exactly this kind of additive access/virtuality change
(`DiscreteLQR::solveDARE` and `DiscreteHinf::solveHinfDARE` both moved `private`->`public` so a new
class could reuse them with zero behavior change to existing callers).

### 1. `lib/SetMembershipEstimator.h` / `.cpp` - standalone estimator, no shared base (mirrors `KalmanFilter`'s `predict()`/`update()` shape)

```cpp
struct SetMembershipParams {
    double w_bound;    // ||process noise||_inf <= w_bound  (isotropic: Qw = w_bound^2 * I)
    double v_bound;    // ||measurement noise||_inf <= v_bound (isotropic: Rv = v_bound^2 * I)
};

class SetMembershipEstimator {
public:
    SetMembershipEstimator(const StateSpace &plant, const SetMembershipParams &params,
                            const Eigen::VectorXd &x0_center, const Eigen::MatrixXd &E0_shape);

    void predict(const Eigen::VectorXd &u);
    void update(const Eigen::VectorXd &y);

    const Eigen::VectorXd &centerEstimate() const { return c_; }
    const Eigen::MatrixXd &ellipsoidShape() const { return P_; }   // {x : (x-c)'P^-1(x-c) <= 1}
    bool isConsistent() const { return consistent_; }
    void reset();   // c_ = x0_center, P_ = E0_shape, consistent_ = true
};
```

**`predict()` - outer-bounding ellipsoid Minkowski sum** (Kurzhanski & Valyi, *Ellipsoidal
Calculus for Estimation and Control*, 1997 - the standard ellipsoidal-bound result for
`x[k+1] = A.x[k] + B.u[k] + w[k]`, `x[k] in E(c,P)`, `w[k] in E(0,Qw)`, independent):
for any scalar `p > 0`,
```
E(A.c+B.u, A.P.A') (+) E(0, Qw)  subset-of  E(A.c+B.u, (1+1/p).A.P.A' + (1+p).Qw)
```
`(+)` = Minkowski sum. The scalar minimizing `trace` of the bound is closed-form (elementary
calculus on `f(p) = (1+1/p).a + (1+p).b`, `a = trace(A.P.A')`, `b = trace(Qw)`: `f'(p) = -a/p^2 + b
= 0`):
```
p* = sqrt( trace(A.P.A') / trace(Qw) )
c_ = A*c_ + B*u
P_ = (1 + 1/p*) * A*P_*A' + (1 + p*) * Qw,   Qw = w_bound^2 * I
```

**`update()` - outer-bounding ellipsoid intersection** (self-derived via the S-procedure, the
same technique underlying the classical Schweppe 1968 / Fogel & Huang 1982 bounding-ellipsoid
update, so this reproduces their result without depending on recalling their exact published
closed form): for any `lambda in [0,1]`, since `E(c,P)` and the measurement consistency set `M =
{x : (y-Cx)'Rv^-1(y-Cx) <= 1}` are each a sublevel set of a quadratic `<= 0`, any non-negative
combination `(1-lambda).f_E(x) + lambda.f_M(x)` is also `<= 0` on `E ∩ M`, so its own sublevel set
is an outer bound on the intersection for *every* `lambda` - giving a one-parameter family of
valid bounds to optimize over:
```
M(lambda) = (1-lambda)*P^-1 + lambda*C'*Rv^-1*C
b(lambda) = (1-lambda)*P^-1*c + lambda*C'*Rv^-1*y
k(lambda) = (1-lambda)*(c'*P^-1*c - 1) + lambda*(y'*Rv^-1*y - 1)

c'(lambda)     = M(lambda)^-1 * b(lambda)
scale(lambda)  = b(lambda)' * M(lambda)^-1 * b(lambda) - k(lambda)
P'(lambda)     = scale(lambda) * M(lambda)^-1            (valid ellipsoid iff scale(lambda) > 0)
```
`update()` evaluates this on a fixed grid of `lambda in (0,1)` (e.g. 99 points - cheap, `na` is
small; a closed-form/golden-section optimum is not assumed since unimodality of `trace(P'(lambda))`
in `lambda` is not proven here, only empirically expected) and keeps the grid point minimizing
`trace(P'(lambda))` among those with `scale(lambda) > 0`. **`isConsistent()`**: if `scale(lambda)
<= 0` at *every* grid point, `E ∩ M` is empty (the measurement is inconsistent with the current
ellipsoid + noise bound) - set `consistent_ = false` and **do not update** `c_`/`P_` (keep the
pre-update, i.e. predicted, ellipsoid rather than corrupt state with an invalid result).

### 2. `lib/ParticleFilter.h` additions - `ParticleFilterV2` (depends on component 0)

```cpp
enum class PFVariant { Bootstrap, Auxiliary, RaoBlackwellized };

struct ParticleFilterParamsV2 : public ParticleFilterParams {
    PFVariant variant = PFVariant::Bootstrap;
    std::vector<int> linear_state_indices;   // RaoBlackwellized only: indices of x that are
                                              // linear-Gaussian given the rest (size = n_lin)
};

class ParticleFilterV2 : public ParticleFilter {
public:
    // A_lin/B_lin/C_lin/Q_lin/R_lin only required (non-empty) for RaoBlackwellized; ignored for
    // Bootstrap/Auxiliary.
    ParticleFilterV2(const ParticleFilterParamsV2 &p, int n_states, int n_meas,
                      ParticleFn f, ParticleMeasFn h,
                      const Eigen::MatrixXd &A_lin = {}, const Eigen::MatrixXd &B_lin = {},
                      const Eigen::MatrixXd &C_lin = {},
                      const Eigen::MatrixXd &Q_lin = {}, const Eigen::MatrixXd &R_lin = {});

    void predict(const Eigen::VectorXd &u) override;
    void update(const Eigen::VectorXd &y, const Eigen::VectorXd &u) override;
    void step(const Eigen::VectorXd &y, const Eigen::VectorXd &u_prev) override;

private:
    ParticleFilterParamsV2 p2_;
    Eigen::VectorXd          auxWeights_;   // Auxiliary only, size n_particles (first-stage weights)
    std::vector<KalmanFilter> kf_;          // RaoBlackwellized only, one embedded KF per particle
    Eigen::MatrixXd A_lin_, B_lin_, C_lin_, Q_lin_, R_lin_;
};
```

**`Bootstrap`:** `predict()`/`update()`/`step()` are not overridden - `ParticleFilterV2` in this
mode *is* `ParticleFilter`'s own implementation via inherited virtuals, exactly the "zero
duplication" property the roadmap wants (and now actually true, since component 0 made the
methods virtual).

**`Auxiliary` (Pitt & Shephard 1999):** only `step()` is overridden (per the Scope section's
restriction); `predict()`/`update()` called independently fall back to the inherited bootstrap
behavior.
```
step(y, u_prev):
  1. for each particle i: mu_i = f_(particles_[i], u_prev)          // deterministic propagation, no noise
  2. auxWeights_[i] = w_[i] * gaussianLikelihood(y; h_(mu_i, u_prev), R)   // look-ahead proxy weight
  3. normalize auxWeights_; draw N ancestor indices a_1..a_N ~ Categorical(auxWeights_)
     (systematic resampling on auxWeights_, reusing the existing resample_buf_/cdf_ workspace)
  4. for each j: particles_[j] = f_(particles_[a_j], u_prev) + processNoise()   // real (noisy) propagation
  5. w_[j] = gaussianLikelihood(y; h_(particles_[j], u_prev), R)
            / gaussianLikelihood(y; h_(mu_{a_j}, u_prev), R)        // importance correction
  6. normalize w_; resample again if N_eff < threshold (standard bootstrap criterion, inherited)
```

**`RaoBlackwellized` (Schon, Gustafsson & Nordlund 2005, additive-coupling special case per
Scope):** `predict()` and `update()` are both overridden; `kf_[i]` is constructed once (in the
constructor) from `StateSpace(A_lin, B_lin, C_lin, D=0, Ts)` + `Q_lin`/`R_lin`, sharing the same
matrices across particles (only each `kf_[i]`'s running mean/covariance differs).
```
predict(u):
  1. for each particle i: x_next = f_(particles_[i], u);
     write x_next's NON-linear-indexed entries into particles_[i] (+ process noise on those
     entries only - the linear-indexed entries are about to be overwritten by step 2, matching
     the additive-separable scope: f_'s own returned values at linear_state_indices are unused)
  2. kf_[i].predict(u)   for each i   // advances the embedded KF's mean/cov via A_lin/B_lin
  3. write kf_[i].state() into particles_[i] at linear_state_indices

update(y, u):
  1. Because A_lin/C_lin/Q_lin/R_lin are LTI and shared, every kf_[i].covariance() is identical
     after predict() (the covariance recursion doesn't depend on data) - compute the shared
     S = C_lin * kf_[0].covariance() * C_lin' + R_lin once.
  2. for each particle i:
       y_offset_i = h_(particles_[i] with linear_state_indices zeroed, u)   // h_nonlinear(x_nl,u),
                                                                              // extracted by
                                                                              // exploiting additive
                                                                              // separability
       y_pred_i   = y_offset_i + C_lin * kf_[i].state()
       w_[i]     *= gaussianDensity(y; y_pred_i, S)             // marginal likelihood weight
       kf_[i].update(y - y_offset_i, u)                          // refine the linear mean/cov
       write kf_[i].state() into particles_[i] at linear_state_indices
  3. normalize w_; resample (inherited criterion) if N_eff < threshold
```
(The "shared `S`/covariance across particles" step is a deliberate optimization, not an
approximation - it follows exactly from the LTI assumption stated in Scope, since a Kalman
filter's covariance recursion is independent of the actual measurement values.)

## Explicitly out of scope (this phase)

- **EF2 anisotropic/non-isotropic noise ellipsoids** - `SetMembershipParams` exposes only scalar
  bounds (matching the roadmap struct); a `Qw`/`Rv`-matrix-accepting overload is a natural future
  extension.
- **EF2 polytopic constraint sets** - `MovingHorizonEstimator`'s existing Hildreth-projection
  machinery remains a possible v2 backend per the roadmap's own note, not built here.
- **EF3 fully general conditionally-linear-dynamics RBPF** (`A_lin`/`C_lin` depending on the
  nonlinear substate) - the LTI + additive-measurement-coupling special case covers a real and
  common use case (the roadmap's own bearings-only-tracking example: nonlinear position, linear
  velocity) without the added complexity of state-dependent linear dynamics.
- **EF3 Auxiliary mode's `predict()`/`update()` called independently** - documented fallback to
  bootstrap behavior, not full look-ahead (see Scope).

## Implementation checklist

**Component 0** (`lib/ParticleFilter.h` base-class change):
1. Apply the `private:`->`protected:` and four `virtual` changes; add `virtual ~ParticleFilter()`.
2. Re-run all existing `[particle_filter]`-tagged Catch2 tests to confirm zero behavior change.

**EF2** (lighter non-`IController` estimator checklist, same shape as `KalmanFilter`):
1. `lib/SetMembershipEstimator.h`/`.cpp` + `CTRL_REGISTER_FEATURE(set_membership_estimator)`
2. `lib/CMakeLists.txt` - add `SetMembershipEstimator.cpp`
3. `lib/ControllerToolbox.h` - `#include "SetMembershipEstimator.h"` near `KalmanFilter.h`
4. `bindings/estimation_bindings.cpp` - bind `SetMembershipParams`, `SetMembershipEstimator`
5. `bindings/smoke_test.py` - construct, `predict`/`update`, assert `ellipsoid_shape()` PD
6. `examples/ex103_set_membership_estimation.cpp` + `examples/python/ex120_set_membership_estimation.py`
   - calibrated-sensor scenario per the roadmap's example use case, plus a side-by-side
   `KalmanFilter` comparison under non-Gaussian noise
7. `tests/test_catch2_advanced.cpp` - tests under `[set_membership]`
8. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` - add `ex103_set_membership_estimation`

**EF3** (extension to an existing class - only modified/added files, per `CLAUDE.md`'s "extension"
checklist row):
1. `lib/ParticleFilter.h`/`.cpp` - component 0's virtual/access changes, plus `ParticleFilterV2`,
   `PFVariant`, `ParticleFilterParamsV2` + `CTRL_REGISTER_FEATURE(particle_filter_v2)`
2. `lib/ControllerToolbox.h` - no new include (same header)
3. `bindings/estimation_bindings.cpp` - bind `PFVariant`, `ParticleFilterParamsV2`,
   `ParticleFilterV2` (base `ctrl::ParticleFilter` in the `py::class_` bases list)
4. `bindings/smoke_test.py` - one assertion per variant (construct + a few `step()` calls, assert
   `state()` finite)
5. `examples/ex104_particle_filter_variants.cpp` + `examples/python/ex121_particle_filter_variants.py`
   - bearings-only tracking scenario (nonlinear position/bearing, linear-Gaussian velocity),
   comparing all three variants' RMSE at equal particle count
6. `tests/test_catch2_advanced.cpp` - tests under `[particle_filter_variants]`
7. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` - add `ex104_particle_filter_variants`

## Testing plan

**`[set_membership]`**
1. Known bounded noise, simulated worst-case sequence - the true state stays inside the returned
   ellipsoid at every step (the core guarantee), checked via `(x_true-c)'P^-1(x_true-c) <= 1 +
   eps`.
2. Comparison against `KalmanFilter` under non-Gaussian (e.g. uniform-bounded) noise - the
   set-membership ellipsoid never excludes the true state while the KF's 99%-confidence interval
   occasionally does over many trials.
3. Deliberately inconsistent measurement (`y` placed outside every bound) - `isConsistent()`
   returns `false` and `centerEstimate()`/`ellipsoidShape()` are unchanged from the pre-update
   (predicted) values.
4. `predict()`'s `p*` reduces `trace(P_)` versus both `p=1` (naive equal-weight sum) and the
   un-optimized endpoints - regression-tests that the closed-form optimum is actually being used.

**`[particle_filter_variants]`**
1. `Bootstrap` mode - numerically identical to a plain `ParticleFilter` constructed with the same
   seed/params over the same input sequence (confirms zero-duplication inheritance from component
   0).
2. `Auxiliary` mode on a problem with an informative look-ahead (e.g. a sharply peaked likelihood)
   - lower weight-variance (or higher `effectiveSampleSize()`) than `Bootstrap` at equal particle
   count over repeated trials.
3. `RaoBlackwellized` mode on the bearings-only-tracking example - RMSE on the linear (velocity)
   substate matches a hand-coded analytic marginal-likelihood baseline, and outperforms
   `Bootstrap` at low particle counts (`N <= 50`).
4. `RaoBlackwellized`'s shared-covariance optimization - regression-test that `kf_[i].covariance()`
   is identical (within floating-point tolerance) across all particles after `predict()`, proving
   the LTI-shared-covariance claim holds in the implementation.
5. Mis-sized `linear_state_indices` (length mismatch with `A_lin`'s dimension, or indices out of
   `[0, n_states)`) - throws `std::invalid_argument` at construction.
