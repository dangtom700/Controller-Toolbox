# Design: ControllerCodeGen (Code Generation)

**Date:** 2026-06-30
**Status:** Approved, implementing
**Roadmap item:** DT1 (`docs/ALGORITHM_ROADMAP_PHASE3.md`, Phase 4).
**Revision (2026-06-30, same day):** Scope narrowed after initial approval. `FuzzyPD`/`FuzzyPID`
and `DiscreteMPC` are dropped from this phase - many real MCU targets have tight memory and want
CPU cycles spent predictably, and both of those controllers cost more of both per step than the
other three: `FuzzyPD`'s Mamdani inference re-runs a 101-point CoG grid search *every call*, and
`DiscreteMPC` re-runs an iterative FISTA QP solve *every call* plus needs the largest static
memory footprint (`Np`/`Nc`-sized baked matrices). `DiscretePID`, `DiscreteSMC`, and
`DiscreteLeadLag` all share one "step-based" pattern instead: a fixed, single-pass, O(1)
arithmetic update with no internal loop or iteration - the same shape the roadmap's own original
v1 sketch scoped to. Everything below reflects the narrowed scope; the dropped decisions (MPC
accessors, Fuzzy topology hardcoding) are removed rather than kept as dead entries.

## Motivation

`docs/algorithm_backlog.md` calls code generation the "highest production value" open item: a
user who tuned a controller with the full C++ toolbox wants to deploy just the resulting
fixed-gain controller on a bare-metal MCU without linking Eigen or the rest of `lib/`. The
roadmap's own sketch proposed a `ControllerCodeGenerator` *class* with static `generateC()`
overloads. Per explicit direction for this item: **no class, no inheritance, anywhere** - on an
MCU exactly one controller (and optionally one corrector wrapped around it) ever exists at a
time, so the runtime polymorphism `IController` exists for (interchangeable controllers, observer
hooks, `ControllerStack` composition) has no reason to appear in the *emitted* code, and the
*generator* itself doesn't need a class to organize what are really just three independent
"params struct in, C string out" functions.

## Scope

**In scope - three "step-based" controller types**, chosen because each has a closed-form,
single-pass, O(1) update equation with no internal iteration and minimal static state - the
pattern best suited to memory-constrained, CPU-cycle-predictable MCU targets:
- `DiscretePID` (`PIDParams`)
- `DiscreteSMC` (`SMCParams`) - first-order boundary-layer variant only.
- `DiscreteLeadLag` (`LeadLagParams`)

**One optional corrector:** `AntiWindupWrapper`, fused inline into the same emitted function
(pre-stage augmented error, post-stage conditioning correction) rather than emitted as a second
wrapping function - there is only ever one controller and at most one corrector on the target
MCU, so there is nothing to wrap at runtime.

**Out of scope (this phase):**
- `FuzzyPD` / `FuzzyPID` - even the fixed 5-term/25-rule diagonal Mamdani topology re-runs a
  101-point CoG grid-search evaluation *every* `controller_step()` call - unpredictable-enough
  extra CPU work per step, on top of being a bigger emitted-code surface, that it doesn't fit the
  "step-based" pattern this phase targets. A future phase can add it back once there's a concrete
  MCU target that can afford it.
- `DiscreteMPC` (any variant) - an iterative FISTA QP solve every step plus `Np`/`Nc`-sized baked
  matrices is both the most CPU-variable (iteration count depends on convergence) and the largest
  static-memory option of everything the roadmap considered. Deliberately deferred past this
  phase.
- `SuperTwistingSMC`, general `FuzzySystem` (arbitrary rule/MF sets), MIMO controllers of any
  kind.
- Any corrector other than `AntiWindupWrapper` (`ComputationalDelayWrapper`,
  `GainScheduledController`, `EventTriggeredWrapper` - future additive follow-ups, same pattern).
- Python bindings - this is a host-side dev tool exercised from C++ examples/tests; trivial to add
  a `pybind11::def()` later since every function returns a plain `std::string`-bearing struct.
- `target_lang` selection - C99 only, matching the roadmap's own v1 scoping.
- `float`-precision emission - same pattern as `BasicPID<float>` already existing alongside
  `BasicPID<double>`, deferred to keep this diff reviewable.

**Candidates for the next code-generation iteration** (identified during this phase's scoping
discussion, not yet designed/committed to): the following all share the same "step-based"
shape - fixed, single-pass, O(1) update, small fixed `static` state, no internal loop or
iteration - so each is a near-zero-cost extension of this phase's emitter framework once there is
a concrete need:
- `DiscreteADRC` - its entire state is one `Eigen::Vector3d` (extended-state-observer `z1`/`z2`/
  `z3`) plus a couple of scalars (`u_prev_`, the derived `beta1_`/`beta2_`/`beta3_` observer
  gains); single-pass ESO update, no iteration.
- `SuperTwistingSMC` - identical shape to `DiscreteSMC`, plus exactly one extra integrator state
  (`v_`). Excluded from this phase only because scope was narrowed to the first-order `DiscreteSMC`
  variant for simplicity, not for any memory/CPU reason.
- `NotchFilter` / `ResonantController` - plain biquad IIR filters (5-6 `static double`s each,
  `b0_`/`b1_`/`b2_`/`a1_`/`a2_` plus two-sample input/output history); neither has an `IController`
  base today, so their generation would be the simplest of the batch.
- `ComputationalDelayWrapper` / `EventTriggeredWrapper` - trivial correctors (one buffered scalar;
  one held value + a threshold/trigger-count), natural second and third corrector types alongside
  this phase's `AntiWindupWrapper`.

Everything else in `lib/` either needs a lookup table sized by a runtime parameter
(`RepetitiveController`'s period buffer), arbitrary user-supplied callback functions
(`FeedbackLinearisationController`'s drift/gain functions aren't flat numeric constants), or an
iterative solve (the MPC/QP family, GP, NN controllers) - a materially bigger lift than this
phase's three, and not assumed to be in scope for "the next iteration" without further scoping.

## Decision log (resolved before implementation)

1. **Free functions in a new `lib/CodeGenC.h`/`.cpp`, not a class.** `GeneratedCode
   generateControllerC(const PIDParams&, double Ts, const CodeGenParams&)`, one overload per
   controller type. This mirrors how `DiscreteLQR`'s stateless math already lives outside
   `IController` in this codebase (the one existing precedent for "pure function over a params
   struct, no base class").
2. **`GeneratedCode` is a plain aggregate (`{std::string header; std::string source;}`), not an
   object with methods.** Same category as `PIDParams` itself - data, not behavior.
3. **Emitted C has zero structs.** Gains -> `static const double KP = ...;`. Controller state ->
   file-scope `static double s_integral = 0.0;` (etc). One function
   `double controller_step(double error)` plus `void controller_reset(void)`. This is flatter
   than even `lib/embedded/`'s `BasicPID<Scalar>` (which is still a class, just non-virtual) -
   appropriate here because the embedded subset is a *reusable library* for multiple instances,
   whereas generated code is a *single baked instance*.
4. **Double precision by default, not float.** Enables bit-identical golden-file testing against
   the real `DiscretePID`/`DiscreteSMC`/`DiscreteLeadLag` `compute()` in Catch2. A `float`
   emission mode is a straightforward follow-up (same pattern as `BasicPID<float>` already
   existing alongside `BasicPID<double>`) but is not needed to satisfy the "deploy without Eigen"
   goal and is left out to keep the diff reviewable.
5. **Corrector fusion, not string-wrapping.** `CodeGenParams::corrector` (an
   `std::optional<AntiWindupConfig>{uMin, uMax, Kb}`) is consumed *inside* each
   `generateControllerC()` overload, which emits the pre-stage (`e_in = error + s_correction`) and
   post-stage (clamp + `s_correction = Kb*(u_sat - u_raw)`) directly around the inner controller's
   own math in the same function - not a second function that takes another function's source as
   a string. Generation throws `std::invalid_argument` if a corrector is requested together with
   an inner controller that already has built-in anti-windup (`PIDParams::Kb != 0`) - mirrors
   `AntiWindupWrapper`'s own constructor guard. `DiscreteSMC` and `DiscreteLeadLag` have no native
   anti-windup concept, so no such guard applies to those two overloads.

## Components

### `lib/CodeGenC.h` (declarations) / `lib/CodeGenC.cpp` (implementation)

```cpp
namespace ctrl {

struct AntiWindupConfig {
    double uMin, uMax;
    double Kb = 1.0;
};

struct CodeGenParams {
    std::string function_name = "controller_step";
    std::optional<AntiWindupConfig> corrector;   // nullopt = no corrector
};

struct GeneratedCode {
    std::string header;   // .h: include guard + prototypes only
    std::string source;    // .c: static state, gains, controller_step()/controller_reset()
};

GeneratedCode generateControllerC(const PIDParams& p, double Ts, const CodeGenParams& cfg = {});
GeneratedCode generateControllerC(const SMCParams& p, double Ts, const CodeGenParams& cfg = {});
GeneratedCode generateControllerC(const LeadLagParams& p, double Ts, const CodeGenParams& cfg = {});

} // namespace ctrl
```

Each overload:
1. Validates inputs (throws `std::invalid_argument` on the `PIDParams::Kb != 0` +
   corrector conflict).
2. Formats the `.h` (include guard, function prototypes, no other dependencies).
3. Formats the `.c` (gains as `static const double`, state as file-scope `static double`,
   the `controller_step`/`controller_reset` functions).

Example emitted `.c` body shape for `generateControllerC(PIDParams{Kp=2,Ki=0.5,...}, 0.01, {})`:

```c
#include "controller_gen.h"
#include <math.h>

static const double KP = 2.0;
static const double KI = 0.5;
static const double KD = 0.0;
static const double ND = 100.0;
static const double KB = 1.0;
static const double U_MIN = -1e9;
static const double U_MAX = 1e9;
static const double TS = 0.01;

static double s_integral = 0.0;
static double s_deriv = 0.0;
static double s_e_prev = 0.0;
static double s_u_prev = 0.0;

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

double controller_step(double error) {
    if (!isfinite(error)) return s_u_prev;
    const double alpha = 1.0 / (1.0 + ND * TS);
    const double d_new = alpha * s_deriv + KD * ND * alpha * (error - s_e_prev);
    const double ki_update = KI * TS * error;
    const double u_unsat = KP * error + (s_integral + ki_update) + d_new;
    const double u_raw = clampd(u_unsat, U_MIN, U_MAX);
    s_integral += ki_update + KB * (u_raw - u_unsat);
    s_deriv = d_new;
    s_e_prev = error;
    s_u_prev = u_raw;
    return u_raw;
}

void controller_reset(void) {
    s_integral = 0.0;
    s_deriv = 0.0;
    s_e_prev = 0.0;
    s_u_prev = 0.0;
}
```

(Directly ported from `DiscretePID::compute()`'s exact equations, including its `isfinite`
NaN-hold-last-output guard - guarantees bit-identical golden-file output.)

`DiscreteSMC` and `DiscreteLeadLag` follow the identical shape: consts for their few gains, 1-2
`static double` state variables, one straight-line `controller_step()` body ported verbatim from
`DiscreteSMC.cpp`/`DiscreteLeadLag.cpp`, and `controller_reset()`. `DiscreteLeadLag` has no
internal saturation (`LeadLagParams` has no `uMin`/`uMax` at all - it's a pure IIR filter), so its
own emitted body has no clamp; a requested corrector still applies its own outer clamp exactly as
`AntiWindupWrapper` would.

## Implementation checklist

1. `lib/CodeGenC.h` + `lib/CodeGenC.cpp` - the three `generateControllerC()` overloads.
   `CTRL_REGISTER_FEATURE(code_gen_c)` at the bottom of the header.
2. `lib/CMakeLists.txt` - append `CodeGenC.cpp` to `CTRL_CORE_SOURCES` (unconditional - unlike the
   dropped Fuzzy-dependent scope, `CodeGenC.h` now only needs `DiscretePID.h`/`DiscreteSMC.h`/
   `DiscreteLeadLag.h`, all always-on, so no `CTRL_ENABLE_*` gating is needed).
3. `lib/ControllerToolbox.h` - umbrella include for `CodeGenC.h`, inserted in the always-on
   include block.
4. `tests/test_catch2_advanced.cpp` - new `TEST_CASE`s tagged `[code_generation]` (see Testing
   plan). `tests/CMakeLists.txt` - `find_program(CTRL_C_COMPILER NAMES gcc cc clang)`; if found,
   `target_compile_definitions(test_catch2_advanced PRIVATE CTRL_C_COMPILER_PATH="${CTRL_C_COMPILER}")`
   so the golden-file tests can `std::system()` the discovered compiler; tests `SKIP` (not fail)
   when no C compiler is found in the build environment.
5. `examples/ex120_code_generation.cpp` - generates all three controller types (+ one
   corrector-fused example) and writes the resulting `.c`/`.h` pairs to disk, printing PASS/FAIL
   per generation call. `examples/CMakeLists.txt`, `compile.bat`, `compile.sh` - register `ex120`.
6. `docs/cumulative_bug_report.md` - new Part 71 section.
7. `docs/ALGORITHM_ROADMAP_PHASE3.md` status table - DT1 `Open` -> `Done`.
8. `docs/algorithm_backlog.md` - move the "Code generation" line to "Already done".

## Testing plan (`[code_generation]`, in `tests/test_catch2_advanced.cpp`)

1. **PID golden-file:** generate C for a `PIDParams` instance, compile standalone with the
   discovered C compiler (no Eigen/`lib/` link), run it over a fixed reference error sequence
   (including a saturating segment), compare output step-by-step against a live `DiscretePID`
   instance constructed with the same params - exact match (both are the same double-precision
   arithmetic).
2. Same golden-file pattern for `DiscreteSMC` and `DiscreteLeadLag`.
3. **Corrector golden-file:** `SMCParams` (no native anti-windup) + `AntiWindupConfig` corrector,
   compared against a live `AntiWindupWrapper(std::make_shared<DiscreteSMC>(...), uMin, uMax, Kb)`.
4. **Rejected-configuration guard:** `PIDParams` with `Kb != 0` + a corrector requested -> throws.
5. **Zero-allocation / freestanding check:** grep each generated `.c`/`.h` string for forbidden
   tokens (`malloc`, `calloc`, `new`, `std::`, `#include <vector>`, `#include <Eigen`) - none
   present.
6. **No C compiler available:** tests `SKIP` cleanly rather than failing the whole suite (matches
   how other environment-dependent Catch2 tests in this repo already degrade).
