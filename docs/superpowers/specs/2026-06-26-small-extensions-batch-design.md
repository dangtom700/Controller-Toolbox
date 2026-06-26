# Small Extensions Batch — Design Spec

**Created:** 2026-06-26. **Source:** `docs/scope_triage_report.md`'s Promoted registry, minus the
case-study items and the (Deferred, NP-hard) full mu lower bound. Five independent, small items,
bundled into one spec per the Phase-1 "small-foundational-utilities" precedent
(`docs/superpowers/specs/2026-06-24-small-foundational-utilities-design.md`).

## 1. `invertTransferFunction()` (Dynamic Inversion helper)

**Problem:** `docs/control_strategies_deep_dive.md:452-456` claims dynamic inversion "is not yet
wrapped as a dedicated class." False — `FeedforwardController` (`lib/FeedforwardController.h`)
already applies any discrete `StateSpace` to the reference, including an inverse-plant model (its
own header demonstrates this at lines 39-45). The real, narrow gap: inverting a `TransferFunction`
`B(z)/A(z)` to get `A(z)/B(z)` isn't a pure swap, because `TransferFunction` requires a monic
denominator (`lib/PlantModel.h:35`, `den[0] == 1`) — so the new denominator (the old numerator `B`)
must be re-normalized by its own leading coefficient `b0`.

**API** (added to `lib/PlantModel.h`/`.cpp`, alongside `TransferFunction`/`tf2ss`):
```cpp
// Returns A(z)/B(z) for G(z) = B(z)/A(z), re-normalized so the result is a valid
// (monic-denominator) TransferFunction. Throws std::invalid_argument if |b0| < eps
// (G is not invertible this way - non-minimum-phase / non-causal inverse at DC).
TransferFunction invertTransferFunction(const TransferFunction& G, double eps = 1e-9);
```
Implementation: `b0 = G.num[0]`; if `|b0| < eps` throw; new `num = G.den` (already monic, stays
as-is since dividing by 1 changes nothing... actually new num = G.den / 1, i.e. unchanged since
G.den is already monic), new `den = G.num / b0` (so `den[0] == 1`). Result: `TransferFunction(G.den, G.num/b0, G.Ts)`.

**No new class.** Pair with one example (`exNN_dynamic_inversion_feedforward`) wiring
`invertTransferFunction` -> `tf2ss` -> `FeedforwardController`. Update
`control_strategies_deep_dive.md:456`'s status tag from `[EXISTS partial]`/`[PLANNED]` to `[EXISTS]`.

**Tests:** invert a known 1st-order TF, verify round-trip (`invert(invert(G))` recovers `G` within
tolerance); verify the inverted system driven by `G`'s own output reference recovers the original
input (perfect inversion property); verify throw when `b0 ~ 0`.

## 2. `tools/monte_carlo.py` real parallel workers

**Problem:** `--workers` is accepted but ignored (`tools/monte_carlo.py:148`, docstring says
"(stub)") — runs are always sequential.

**Design:** Extract the per-sample body of the existing `for ctrl_name: for i in range(args.n):`
loop into a module-level function `_mc_job(study_dir, ctrl_name, sample_id, perturbed, nominal_params)`
that re-imports the sim module itself (`_load_sim_module` is cheap relative to a sim run, and the
loaded module object isn't picklable across a process boundary, so each worker must import its own
copy rather than receive one from the parent). When `args.workers <= 1`, keep today's exact
sequential path (zero behavior change, zero `Pool` overhead). When `args.workers > 1`, build the
full list of `(ctrl_name, sample_id, perturbed_params)` jobs up front (so progress/ordering stays
deterministic per `rng`), submit via `multiprocessing.Pool(args.workers).starmap`, and reassemble
`all_rows` in the original order. The skip-counting/print-progress logic (currently inline per
sample) moves to a post-loop pass over the returned results so it works the same in both modes.

**Tests:** no existing pytest suite for `tools/`; verify manually by running
`monte_carlo.py --study "<a fast case study>" --n 8 --workers 1` and `--workers 4` and diffing the
resulting CSVs (same rows, same values, since `rng` is seeded before job generation in both paths).

## 3. `DiscreteH2` D11 != 0 support

**Problem:** `lib/DiscreteH2.h:22-23` calls full `D11 != 0` support "a deliberately deferred
follow-up... via loop-shifting." That's the wrong technique for this gap. Worked the math: with
`Dk = 0` (always true for `DiscreteH2`'s controller assembly, confirmed at
`lib/DiscreteH2.cpp:225`) and `w[k]` independent of `(x[k], u[k])` (both are functions of *past* w
only — `Dk=0` means `u[k]` depends only on controller state `xk[k]`, which depends on past `y`'s,
hence past `w`'s, never `w[k]` itself), the cross term
`E[(C1 x[k] + D12 u[k])' D11 w[k]]` is **exactly zero** for zero-mean white `w`, regardless of the
controller. Consequence: **`D11` never enters the control or filter Riccati equations** — the
existing `F`/`L` derivation in `lib/DiscreteH2.cpp:170-217` (cross-term-eliminated DARE on
`(A,B2,C1,D12)` and the dual on `(A,C2,B1,D21)`) is already exactly correct for any `D11`, because
it was never a function of `D11` to begin with. The only place `D11` matters is the achieved-H2-norm
bookkeeping: with `z = C1 x + D12 u + D11 w`, `||z||_2^2 = trace(D11 D11') + trace(C_cl Wc C_cl')`
(a direct feedthrough term adds a constant, controller-independent contribution to the H2 norm —
the standard discrete-time result for a system with direct disturbance feedthrough).

**Changes:**
- `lib/DiscreteH2.cpp::solve()`: delete the `D11` validation throw (lines 135-140 of the current
  file). `D22 != 0` is still rejected (genuinely required: `Dk=0` assembly assumes `D22=0`).
- `lib/DiscreteH2.cpp::buildClosedLoop()`: no change needed to `A_cl`/`B_cl`/`C_cl` (the existing
  derivation already doesn't reference `D11`).
- `lib/DiscreteH2.cpp::solve()`'s norm computation: change
  `h2sq = trace(C_cl Wc C_cl')` to `h2sq = trace(P.D11 * P.D11.transpose()) + trace(C_cl Wc C_cl')`.
- Update `lib/DiscreteH2.h`'s doc comment (lines 18-23) to drop the "loop-shifting... deferred"
  language and state the result above directly, with the independence argument.

**Tests:** `tests/test_catch2_advanced.cpp:1265`'s existing
`"DiscreteH2::solve throws on nonzero D11 (MixedSensitivity-built plant)"` test asserts the *old*
behavior and must be replaced with a test that `DiscreteH2::solve()` on that exact
MixedSensitivity-built plant now succeeds (`result.feasible == true`), and that `F`/`L`
(`result.Ck`/`result.Bk`) are numerically identical to a hand-built `D11=0` plant with everything
else equal (proving `D11` truly doesn't affect the gains), while `achievedH2Norm` differs by
exactly `sqrt(trace(D11*D11'))` in quadrature from the `D11=0` case's norm on otherwise-identical
plants. Keep the `D22 != 0` throw test (`:1294`) unchanged.

## 4. `MuAnalysis` RealScalar block G-scaling

**Problem:** `lib/MuAnalysis.h:24-29` and `MuAnalysis.cpp:104-107` reject any `RealScalar` block —
G-scaling (Packard & Doyle 1993) isn't implemented.

**Design:** Extend the existing coordinate-descent search in `MuAnalysis.cpp::computeMuOneFreq()`
(currently: one positive scalar `d_i` per block, golden-section line search per coordinate, several
sweeps) with a second scalar `g_i` per `RealScalar` block (zero/unused for `Complex*` blocks),
implementing:
```
mu(M) <= inf_{D,G} sigma_max( (I + G^2)^{-1/2} * (D_L M D_R^{-1} - jG) )
```
where `G` is block-diagonal real, `g_i` placed on every diagonal entry belonging to `RealScalar`
block `i` (zero elsewhere) — same one-scalar-per-block structure as `D`, so it reuses
`buildScaling`'s block-iteration pattern. Scope (matching the existing `ComplexScalar` caveat
already in the header docs): exact for `RealScalar` blocks of size 1, a valid-but-possibly-loose
upper bound for size > 1 (same scalar `g_i` repeated on the block's diagonal, not a full per-entry
Hermitian G). Each sweep now does, per block: golden-section search over `d_i` (all blocks, as
today), then for `RealScalar` blocks only, golden-section search over `g_i` holding everything else
fixed. Remove the `RealScalar` throw in `computeMuOneFreq` (`MuAnalysis.cpp:104-107`); keep the
`r_out != r_in` throw for all scalar block types.

**Validation target:** a known real-parametric-uncertainty example with an analytically-known mu
(Skogestad & Postlethwaite-style real-mu example, mirroring the existing complex 2x2 example at
`test_catch2_advanced.cpp:7769` but with a `RealScalar` structure) — not just a regression check,
since this is new math.

**Doc updates:** `lib/MuAnalysis.h:24-29` scope comment, `bindings/analysis_bindings.cpp:796`'s
"RealScalar blocks raise ValueError (G-scaling not implemented)" docstring line.

**Tests:** replace the `RealScalar`-throws half of
`test_catch2_advanced.cpp:7754`'s combined test (split into two tests: dimension-mismatch-throws
stays; RealScalar-throws becomes RealScalar-succeeds-and-matches-known-mu); add a 2-block mixed
real+complex case to exercise the alternating `d`/`g` sweep.

## 5. `EventTriggeredWrapper`

**Problem:** `docs/control_strategies_deep_dive.md:201-217` sketches an `EventTriggeredController`
wrapper but calls a nonexistent `IController::lastOutput()`, and its trigger condition
(`e - (y_last_ - ref)`) doesn't typecheck against any real signal convention.

**Design:** New header-only class `lib/EventTriggeredWrapper.h` (no `.cpp` — matches
`ComputationalDelayWrapper.h`'s fully-inline pattern, so **no `lib/CMakeLists.txt` change is
needed**, only a new `#include` in `lib/ControllerToolbox.h`):

```cpp
struct EventTriggeredParams {
    double sigma = 0.1; // deadband threshold on the incoming signal, engineering units
};

class EventTriggeredWrapper : public IController {
public:
    explicit EventTriggeredWrapper(std::shared_ptr<IController> inner,
                                    const EventTriggeredParams& params = {});
    double compute(double signal) override;   // NaN passthrough: hold-last, no trigger
    void   reset() override;                  // resets inner_, clears held output + last-triggered signal
    double sampleTime() const override { return inner_->sampleTime(); }

    double lastOutput() const { return u_held_; }      // own accessor, mirrors ComputationalDelayWrapper
    int    triggerCount() const { return n_triggered_; }
    int    holdCount() const { return n_held_; }
    const IController& inner() const { return *inner_; }

private:
    std::shared_ptr<IController> inner_;
    EventTriggeredParams         params_;
    double u_held_ = 0.0;
    double signal_last_triggered_ = 0.0;
    bool   triggered_once_ = false;
    int    n_triggered_ = 0, n_held_ = 0;
};
```

Trigger rule, convention-agnostic (works whether `signal` is `r-y`, `y-r`, or raw `y` — unlike the
deep-dive sketch): on the first call, always trigger (no prior reference to compare against). After
that, if `|signal - signal_last_triggered_| > params_.sigma`, call `inner_->compute(signal)`, store
the new output in `u_held_` and `signal` in `signal_last_triggered_`, increment `n_triggered_`;
otherwise return `u_held_` unchanged (zero-order hold) and increment `n_held_`. NaN input: hold-last
per the fleet NaN contract, no trigger, no counter change to `n_triggered_`/`n_held_` (matches
`ComputationalDelayWrapper`'s NaN handling, which also skips its own bookkeeping on bad input).

**Full 8-step checklist** (new `IController` subclass, per `CLAUDE.md`'s scope-discipline note):
1. `lib/EventTriggeredWrapper.h` (header-only, as above) + `CTRL_REGISTER_FEATURE(event_triggered)`.
2. No `lib/CMakeLists.txt` change (header-only).
3. `lib/ControllerToolbox.h`: add `#include "EventTriggeredWrapper.h"` near
   `ComputationalDelayWrapper.h` (`:135`).
4. Feature self-registers via the macro; no manual `Features.h` edit.
5. `bindings/controllers_bindings.cpp`: bind alongside `ComputationalDelayWrapper` (`:1620-1643`),
   mirroring its `py::class_<..., IController, shared_ptr<...>>` pattern.
6. `bindings/smoke_test.py`: add `assert hasattr(ctrl, 'EventTriggeredWrapper')`.
7. `tests/test_catch2_advanced.cpp`: tests tagged `[event_triggered]` — first-call always triggers;
   small signal change within deadband holds (counts via `holdCount()`); change exceeding `sigma`
   re-triggers; NaN input holds without affecting counters; `reset()` clears held state and forces
   the next call to trigger again (mirrors the first-call case).
8. New example `ex116_event_triggered_wrapper.cpp` / `examples/python/ex133_event_triggered_wrapper.py`
   (next free numbers, re-verified at `examples/ex115_value_iteration_solver.cpp` /
   `examples/python/ex132_value_iteration_solver.py` being the latest as of this writing) +
   `examples/CMakeLists.txt`, `compile.bat`, `compile.sh` registration.

## Cross-cutting notes

- None of the five touch `IController`'s base interface.
- Items 3 and 4 are extensions to existing classes — per this repo's own checklist note
  ("extensions to existing classes... only the modified files need updating, not the full 8-step
  checklist — but Catch2 tests are always required"), no new bindings/examples are needed for
  those two; existing bindings for `DiscreteH2`/`MuAnalysis` already expose everything touched.
- Per established project workflow (confirmed in today's `docs/superpowers/plans/2026-06-26-value-iteration-solver.md`):
  write every file's content first, then one full build + Catch2 + example + smoke-test pass,
  rather than a compile cycle after every item.
- No `git commit` unless explicitly asked.
