# Design: `GPMPC` (Phase 3, ML3 - GP-Uncertainty-Aware MPC)

**Date:** 2026-06-25
**Roadmap source:** `docs/ALGORITHM_ROADMAP_PHASE3.md` ML3 section; `docs/algorithm_backlog.md`
Machine Learning Integration table (`GP-MPC (combined)`).
**Status:** Approved design, ready for implementation plan.

## Goal

A controller that consumes `GaussianProcess`/`GPResidualModel` uncertainty directly inside
`NonlinearMPC`'s prediction horizon, tightening the control authority where the GP is least
confident about the dynamics - the gap `algorithm_backlog.md` flags as "partially addressed by
`HybridMPC`, but not GP-uncertainty-aware MPC specifically."

## Why this needed scoping down before a design was possible

The roadmap's own sketch framed `GPMPC` as tightening *output/state* constraints by GP variance,
"architecturally a sibling of `HybridMPC`." Neither holds up against the real code:

- `NonlinearMPC` has **no output/state constraints at all** - only input box bounds (`uMin`/
  `uMax`, `lib/NonlinearMPC.h:54-55`), applied via `solveGradientProjectionQP`'s FISTA box
  projection. There is nothing to "tighten" on the output side without first adding general
  linear-inequality support to the QP layer - a materially larger, separate lift, not a `GPMPC`
  detail.
- `HybridMPC`'s "override" precedent only swaps the dynamics function via a constructor-injected
  lambda (`lib/HybridMPC.h:69,75-78`) - it never touches constraints, so it is not actually
  evidence that per-step bound tightening is pluggable today.
- `TubeMPC` (`lib/TubeMPC.h`) is this codebase's real precedent for "constraint tightening": it
  shrinks **input** bounds (`uMin`/`uMax` -> `v_min`/`v_max`) by a disturbance-derived tube radius,
  computed once at construction and held constant for the whole horizon.

**Decision (made with the user before this design was written):** scope `GPMPC` to tighten input
bounds, like `TubeMPC`, but make the tightening **time-varying across the horizon** (driven by the
GP's variance at each predicted step) rather than `TubeMPC`'s static radius. This requires one
small, additive prerequisite change to `NonlinearMPC` (below) rather than a QP-solver rewrite.

## Prerequisite change to `NonlinearMPC` (small, additive, behavior-preserving)

`buildAndSolve()` (`lib/NonlinearMPC.cpp:111-206`) is private and non-virtual, and the members a
subclass would need are private. Required changes, all backward-compatible (widening access never
breaks an existing caller):

1. Change `x_traj_`, `U_warm_`, `lb_qp_`, `ub_qp_` from `private` to `protected`.
   `p_` (`NMPCParams`) stays private - `GPMPC` keeps its own `NMPCParams` copy via
   `GPMPCParams::nmpc`, mirroring `HybridMPCParams`'s exact existing pattern, so it never needs
   the base class's private params.
2. Add one new protected virtual hook, called from `buildAndSolve()` immediately after the
   existing uniform box-fill loop (`lib/NonlinearMPC.cpp:172-179`) and before the Lipschitz/LDLT
   factorization step:
   ```cpp
   protected:
       /// Called once per RTI step after lb_qp_/ub_qp_ are filled with the uniform
       /// uMin/uMax-relative bounds, before the QP is solved. Override to tighten them
       /// per-step. No-op by default -- preserves NonlinearMPC's exact existing behavior.
       virtual void tightenStepBounds() {}
   ```
   `buildAndSolve()` gains exactly one new line (`tightenStepBounds();`) at that point; everything
   else is untouched.

## Small additive change to `GaussianProcess` / `GPResidualModel`

Add a public accessor for the feature dimension the GP was constructed with (currently only
stored privately as `GaussianProcess::x_dim_`, `lib/GaussianProcess.h:86`):

```cpp
// GaussianProcess.h
int xDim() const { return x_dim_; }

// GPResidualModel.h
int xDim() const { return gp_.xDim(); }
```

Needed so `GPMPC`'s constructor can validate the GP matches the feature it will actually be
queried with (`n_states + n_inputs`, see below) and throw `std::invalid_argument` on a mismatch,
instead of risking an Eigen dimension assertion (compiled out in Release) at the first `compute()`
call.

## `GPMPC` design

**Key finding that simplifies the tightening formula:** `GPResidualModel::predictWithUncertainty`'s
`variance` comes purely from `gp_.predict(x_feat)` (`lib/GPResidualModel.cpp:39-47`) - it does not
depend on the `model_pred` argument at all (`model_pred` only affects `mean_total`, which `GPMPC`
never uses). So `GPMPC` never needs to compute an actual physical-model prediction at each step -
just the variance.

```cpp
// lib/GPMPC.h
struct GPMPCParams
{
    NMPCParams nmpc;
    double uncertainty_inflation = 2.0;   // tightening = inflation * sqrt(variance)
};

class GPMPC : public NonlinearMPC
{
public:
    GPMPC(const GPMPCParams &params, DiscreteDynamics f_d,
          std::shared_ptr<GPResidualModel> gp);

    std::string name() const override { return "GPMPC"; }

    /// Per-step shrink amount applied to lb_qp_/ub_qp_ on the most recent compute()
    /// (length Nu*m, one block per held control step). Mirrors TubeMPC's
    /// tightenedUMin()/tightenedUMax() diagnostic-accessor convention.
    const Eigen::VectorXd &lastTightening() const { return shrink_; }

protected:
    void tightenStepBounds() override;

private:
    GPMPCParams gp_params_;
    std::shared_ptr<GPResidualModel> gp_;
    Eigen::VectorXd shrink_;  // sized Nu*m in the constructor
};
```

**Constructor:** delegates `params.nmpc`/`f_d` to `NonlinearMPC`'s base constructor (full-state
output, `C = I` - only one constructor is provided, not `NonlinearMPC`'s second `C_out` overload;
adding that overload is a small additive follow-up if a real use case needs it, not scoped here).
Validates `gp->xDim() == params.nmpc.n_states + params.nmpc.n_inputs`, throwing
`std::invalid_argument` otherwise. Sizes `shrink_` to `params.nmpc.Nu * params.nmpc.n_inputs`,
zero-initialized.

**`tightenStepBounds()`** (called once per RTI step, `lb_qp_`/`ub_qp_` already hold the uniform
`uMin`/`uMax`-relative bounds at this point):

```cpp
void GPMPC::tightenStepBounds()
{
    const int m = static_cast<int>(U_warm_.rows());
    const int Nu = static_cast<int>(U_warm_.cols());

    for (int k = 0; k < Nu; ++k)
    {
        // Feature convention mirrors HybridMPC::addStateObservation/refitDataModel exactly
        // (lib/HybridMPC.cpp:26-27,71-72): feat << x, u.
        Eigen::VectorXd feat(x_traj_[k].size() + m);
        feat << x_traj_[k], U_warm_.col(k);

        const auto pred = gp_->predictWithUncertainty(feat, 0.0); // model_pred unused for variance
        const double shrink_k = gp_params_.uncertainty_inflation
                               * std::sqrt(std::max(pred.variance, 0.0));

        auto lb_seg = lb_qp_.segment(k * m, m);
        auto ub_seg = ub_qp_.segment(k * m, m);
        for (int j = 0; j < m; ++j)
        {
            const double mid = 0.5 * (lb_seg(j) + ub_seg(j));
            lb_seg(j) = std::min(lb_seg(j) + shrink_k, mid); // never crosses ub
            ub_seg(j) = std::max(ub_seg(j) - shrink_k, mid); // never crosses lb
        }
        shrink_.segment(k * m, m).setConstant(shrink_k);
    }
}
```

One scalar `GPResidualModel` models the dominant uncertainty source (matches the roadmap's own
singular `gp` parameter), and the same `shrink_k` applies uniformly across all `m` input
dimensions at step `k` - deliberately simpler than a per-input-dimension GP ensemble. The
per-element `mid`-clamp guarantees `lb_seg(j) <= ub_seg(j)` always (degenerates to a single point
at most, never an infeasible crossed box).

**Per-step feature is `x_traj_[k]` (the predicted state at the *start* of held control step `k`,
for `k = 0..Nu-1`), not an aggregate over the (possibly longer) tail of prediction steps `j` that
hold at `Nu-1`** (mirroring how `Theta_`'s own construction maps `col_k = min(j, Nu-1)`, but
`GPMPC` only evaluates the GP once per *held* step, not once per raw prediction step) - a
deliberate v1 simplification, noted as such rather than silently approximated.

## Edge cases / numerical safety

- GP not fitted (`gp_->isFitted() == false`): `predictWithUncertainty` returns `variance = 0.0`
  unconditionally (`lib/GPResidualModel.cpp:42-43`), so `shrink_k = 0` for every step -
  `tightenStepBounds()` becomes a true no-op and `GPMPC` is bit-for-bit `NonlinearMPC` on the same
  scenario. This is the "GP confident" regression case, and needs no trained GP to test.
- Constructor: `gp->xDim() != n_states + n_inputs` -> `std::invalid_argument`.
- `pred.variance` is clamped to `>= 0` before the `sqrt` (defends against any future floating-point
  underflow producing a tiny negative variance from the GP kernel computation).
- This still runs inside `buildAndSolve()`'s per-RTI-step hot path, but `NonlinearMPC` itself
  already performs per-step heap-allocating work there (`Eigen::MatrixXd Theta_` rebuild, etc.) and
  is documented as **not** subject to the `compute()`/`step()` zero-allocation rule the way scalar
  controllers are (`docs/deployment.md`'s RT constraints target the embedded subset, not the
  RTI-MPC family) - `GPMPC` inherits that same exemption, consistent with `HybridMPC`.

## Wiring

This is a **new class inheriting an existing one** (like `HybridMPC` inheriting `NonlinearMPC`) -
the full checklist applies, plus the two small prerequisite edits to already-shipped classes:

1. `lib/NonlinearMPC.h`/`.cpp` - 4 members `private` -> `protected`, add `tightenStepBounds()` hook
   + its one-line call site.
2. `lib/GaussianProcess.h` / `lib/GPResidualModel.h` - add `xDim()` accessors.
3. `lib/GPMPC.h`, `lib/GPMPC.cpp` (new).
4. `lib/CMakeLists.txt` - add `GPMPC.cpp` to `CTRL_CORE_SOURCES`.
5. `lib/ControllerToolbox.h` - `#include "GPMPC.h"`.
6. `bindings/controllers_bindings.cpp` (where `HybridMPC`/`NonlinearMPC` are already bound) - bind
   `GPMPCParams`/`GPMPC` with `std::shared_ptr<GPMPC>` as the 3rd `py::class_` template arg +
   `ctrl::IController` base (`CLAUDE.md`'s binding rule).
7. `bindings/smoke_test.py` - exercise construction + a `compute()`/`computeRef()` call with an
   unfitted GP (the no-op regression case, needing no training data).
8. `tests/test_catch2_advanced.cpp` - Catch2 tests tagged `[gp_mpc]`.
9. `examples/exNN_gp_mpc.cpp` + `examples/python/exNN_gp_mpc.py` (next free numbers at
   implementation time), plus `examples/CMakeLists.txt`, `compile.bat`, `compile.sh` updates.
10. `docs/algorithm_backlog.md` / `docs/ALGORITHM_ROADMAP_PHASE3.md` - mark ML3 done once shipped.

## Test plan (`[gp_mpc]`)

Each property below is either mathematically guaranteed by construction (not just empirically
observed) or a direct, deterministic check - after SI3's lesson about not locking in unverified
empirical claims, nothing here depends on a statistical comparison across random trials:

1. **GP not fitted -> regression-identical to `NonlinearMPC`.** Same dynamics/params fed to both a
   plain `NonlinearMPC` and a `GPMPC` wrapping an unfitted `GPResidualModel` - identical `u_opt_`
   sequence over N steps (bit-for-bit; guaranteed by `shrink_k == 0` always, not just "close").
2. **GP fitted with deliberately high variance at the predicted trajectory -> bounds visibly
   tighten.** `lastTightening()` reports a nonzero shrink at the relevant step(s), and the solved
   `|u_opt_ - u_bar|` is bounded by the tightened (smaller) box - guaranteed by FISTA's box
   projection (`solveGradientProjectionQP`), not just observed in one run.
3. **Constructor validation.** A `GPResidualModel` constructed with the wrong `xDim()` (e.g.
   `n_states + n_inputs - 1`) throws `std::invalid_argument`.
4. **Clamp correctness.** An artificially huge `uncertainty_inflation` (e.g. `1e9`) still produces
   `lb_seg(j) <= ub_seg(j)` for every step/element (the box collapses to a point, never crosses).

## Out of scope

- General output/state constraints in `NonlinearMPC` - would need a non-box QP solver; a
  prerequisite for textbook GP-MPC, not part of this design (see "Why this needed scoping down").
- A second `GPMPC` constructor mirroring `NonlinearMPC`'s explicit-`C_out` overload - only the
  full-state (`C = I`) constructor is provided; additive follow-up if needed later.
- Per-input-dimension GP ensembles (one GP per actuator) - a single scalar `GPResidualModel`
  drives a uniform shrink across all `m` inputs at each step.
- Evaluating the GP at every raw prediction step `j = 0..Np-1` - only once per *held* control step
  `k = 0..Nu-1`, at `x_traj_[k]` specifically (see "Per-step feature" above).
- A comparison test against `HybridMPC` demonstrating "avoids a violation `HybridMPC` misses" - the
  original roadmap sketch's claim; dropped because constructing a scenario where that's actually
  true needs a contrived setup, and the lesson from SI3 is not to lock in an unverified comparative
  claim into a Catch2 test.
