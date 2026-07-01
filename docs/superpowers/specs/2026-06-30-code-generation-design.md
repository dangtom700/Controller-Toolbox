# Design: ControllerCodeGen (Code Generation)

**Date:** 2026-06-30
**Status:** Approved, implementing
**Roadmap item:** DT1 (`docs/ALGORITHM_ROADMAP_PHASE3.md`, Phase 4).

## Motivation

`docs/algorithm_backlog.md` calls code generation the "highest production value" open item: a
user who tuned a controller with the full C++ toolbox wants to deploy just the resulting
fixed-gain controller on a bare-metal MCU without linking Eigen or the rest of `lib/`. The
roadmap's own sketch proposed a `ControllerCodeGenerator` *class* with static `generateC()`
overloads. Per explicit direction for this item: **no class, no inheritance, anywhere** - on an
MCU exactly one controller (and optionally one corrector wrapped around it) ever exists at a
time, so the runtime polymorphism `IController` exists for (interchangeable controllers, observer
hooks, `ControllerStack` composition) has no reason to appear in the *emitted* code, and the
*generator* itself doesn't need a class to organize what are really just five independent
"params struct in, C string out" functions.

## Scope

**In scope - five controller types**, chosen because each already has a closed-form or
fixed-size-at-construction update equation:
- `DiscretePID` (`PIDParams`)
- `DiscreteSMC` (`SMCParams`) - first-order boundary-layer variant only.
- `DiscreteLeadLag` (`LeadLagParams`)
- `FuzzyPD` / `FuzzyPID` (`FuzzyPDParams` / `FuzzyPIDParams`) - the fixed 5-term/25-rule diagonal
  Mamdani controller, not the general arbitrary-rule `FuzzySystem`.
- `DiscreteMPC` - SISO plants only, box-constrained via its actual FISTA gradient-projection
  algorithm (the algorithm it already runs - not a generic active-set QP substitute).

**One optional corrector:** `AntiWindupWrapper`, fused inline into the same emitted function
(pre-stage augmented error, post-stage conditioning correction) rather than emitted as a second
wrapping function - there is only ever one controller and at most one corrector on the target
MCU, so there is nothing to wrap at runtime.

**Out of scope (this phase):**
- `SuperTwistingSMC`, general `FuzzySystem` (arbitrary rule/MF sets), MIMO `DiscreteMPC`.
- Any corrector other than `AntiWindupWrapper` (`ComputationalDelayWrapper`,
  `GainScheduledController`, `EventTriggeredWrapper` - future additive follow-ups, same pattern).
- Python bindings - this is a host-side dev tool exercised from C++ examples/tests; trivial to add
  a `pybind11::def()` later since every function returns a plain `std::string`-bearing struct.
- `target_lang` selection - C99 only, matching the roadmap's own v1 scoping.

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
   the real `DiscretePID`/`DiscreteSMC`/etc. `compute()` in Catch2. A `float` emission mode is a
   straightforward follow-up (same pattern as `BasicPID<float>` already existing alongside
   `BasicPID<double>`) but is not needed to satisfy the "deploy without Eigen" goal and is left out
   to keep the diff reviewable.
5. **`DiscreteMPC` codegen requires 5 new read-only accessors on the class**
   (`plant()`, `F()`, `Phi()`, `Gu()`, `H()`, `L()`), all trivial `const&`/`const` returns of
   existing private members. Alternative considered: have the codegen function re-derive
   `F`/`Phi`/`Gu`/`H`/`L` itself from `(plant, params)` using the same formulas as
   `DiscreteMPC::buildPredictionMatrices`/`buildCostMatrix`. Rejected - duplicating ~50 lines of
   matrix-assembly arithmetic in a second place risks silent drift (a future change to
   `DiscreteMPC`'s formulation would not be reflected in codegen, and golden-file tests would only
   catch it if someone remembered to re-run them). Reading the accessors guarantees the emitted C
   always matches whatever `DiscreteMPC` actually computed for that exact instance.
6. **`DiscreteMPC` codegen is SISO-only** (`plant.inputSize()==1 && plant.outputSize()==1`, throws
   `std::invalid_argument` otherwise) - matches `DiscreteMPC::compute(error)`'s own existing SISO
   convenience-wrapper scope (the MIMO `computeRef()` path is not emitted). `D != 0` plants are
   accepted (the emitted code follows the same "use `u_prev` for the feedthrough term" convention
   `DiscreteMPC::compute()` already documents and warns about).
7. **`H^-1` is precomputed host-side (via Eigen, at generation time) and baked as a `static const`
   array**, not recomputed via an emitted LDLT solve. `H` never changes after generation (the
   controller is a fixed, tuned instance), so its inverse is a compile-time constant - this
   replaces `DiscreteMPC`'s runtime `ldlt_.solve(g)` warm-start with a plain baked-matrix multiply
   in the emitted C, which is both simpler to emit correctly and avoids porting a
   Cholesky-like decomposition into flat C at all.
8. **Fuzzy codegen re-derives `FuzzyPD`'s fixed topology directly, not via `FuzzySystem`
   introspection.** `FuzzyPD::buildSystem()` always constructs the same 5-term
   (NL/NS/ZE/PS/PL) triangular+shoulder partition and the same 25-rule diagonal table, parameterized
   only by `e_scale`/`de_scale`/`u_scale` (all public via `FuzzyPDParams`). Codegen hardcodes that
   known-fixed structure (membership evaluation + rule strengths + CoG grid search over
   `cog_resolution` points) directly in the emitted C, rather than adding generic
   `FuzzySystem` accessors for arbitrary rule/MF introspection - there is no such general codegen
   need in scope, and hardcoding the one fixed topology keeps `generateControllerC(FuzzyPDParams)`
   self-contained. Breakpoints/rule table are ported verbatim from `FuzzyLogic.cpp`'s
   `FuzzyPD::buildSystem()` during implementation to guarantee golden-file parity.
9. **Corrector fusion, not string-wrapping.** `CodeGenParams::corrector` (an
   `std::optional<AntiWindupConfig>{uMin, uMax, Kb}`) is consumed *inside* each
   `generateControllerC()` overload, which emits the pre-stage (`e_in = error + s_correction`) and
   post-stage (clamp + `s_correction = Kb*(u_sat - u_raw)`) directly around the inner controller's
   own math in the same function - not a second function that takes another function's source as
   a string. Generation throws `std::invalid_argument` if a corrector is requested together with
   an inner controller that already has built-in anti-windup (`PIDParams::Kb != 0`,
   `FuzzyPIDParams::Ki != 0 && Kb != 0`) or with the `DiscreteMPC` overload (already
   box-constrained natively) - mirrors `AntiWindupWrapper`'s own constructor guard.

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
GeneratedCode generateControllerC(const FuzzyPDParams& p, double Ts, const CodeGenParams& cfg = {});
GeneratedCode generateControllerC(const FuzzyPIDParams& p, double Ts, const CodeGenParams& cfg = {});
GeneratedCode generateControllerC(const DiscreteMPC& mpc, const CodeGenParams& cfg = {});

} // namespace ctrl
```

Each overload:
1. Validates inputs (throws `std::invalid_argument` on corrector/native-anti-windup conflicts, on
   non-SISO `DiscreteMPC`, on a corrector requested with `DiscreteMPC`).
2. Formats the `.h` (include guard, `#include <stdbool.h>` only if needed, function prototypes,
   no other dependencies).
3. Formats the `.c` (gains/matrices as `static const double`, state as file-scope `static double`,
   the `controller_step`/`controller_reset` functions).

Example emitted `.c` body shape for `generateControllerC(PIDParams{Kp=2,Ki=0.5,...}, 0.01, {})`:

```c
#include "controller_step.h"

static const double KP = 2.0;
static const double KI = 0.5;
static const double KD = 0.0;
static const double N  = 100.0;
static const double KB = 1.0;
static const double U_MIN = -1e9;
static const double U_MAX = 1e9;
static const double TS = 0.01;

static double s_integral = 0.0;
static double s_deriv = 0.0;
static double s_e_prev = 0.0;

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

double controller_step(double error) {
    const double alpha = 1.0 / (1.0 + N * TS);
    const double d = alpha * s_deriv + KD * N * alpha * (error - s_e_prev);
    const double u_raw = KP * error + s_integral + d;
    const double u = clampd(u_raw, U_MIN, U_MAX);
    s_integral += KI * TS * error + KB * (u - u_raw);
    s_e_prev = error;
    s_deriv = d;
    return u;
}

void controller_reset(void) {
    s_integral = 0.0;
    s_deriv = 0.0;
    s_e_prev = 0.0;
}
```

(Directly ported from `BasicPID<double>::compute()`'s equations, the exact same closed-form
`DiscretePID` implements - guarantees bit-identical golden-file output.)

### `DiscreteMPC` accessor additions (`lib/DiscreteMPC.h`)

```cpp
const StateSpace& plant() const { return plant_; }
const Eigen::MatrixXd& F() const { return F_; }
const Eigen::MatrixXd& Phi() const { return Phi_; }
const Eigen::MatrixXd& Gu() const { return Gu_; }
const Eigen::MatrixXd& H() const { return H_; }
double L() const { return L_; }
```

Added under the existing "Read-only access" accessor group (alongside `params()`); no behavior
change to `DiscreteMPC` itself.

### MPC-emitted C shape (sketch, `Np`/`Nc`/`n` baked as literal loop bounds)

```c
double controller_step(double error) {
    /* y_hat = C.x_hat + D.u_prev; r_ref = y_hat + error */
    /* pred_err = F*x_hat + Gu*u_prev - r_stack (r_stack is r_ref repeated Np times) */
    /* grad = Phi' * Qy * pred_err   (Qy = rho_y * I, folds into a scalar multiply) */
    /* rolling box-bound tightening loop over Nc steps (cumMin/cumMax), m=1 */
    /* warm start: DeltaU = clamp(-Hinv * grad, lb, ub); y_fista = DeltaU */
    /* FISTA loop, up to QP_MAX_ITER, tol QP_TOL: grad = H*y_fista + grad_g; ... */
    /* u = clamp(u_prev + DeltaU[0], U_MIN, U_MAX) */
    /* x_hat = A*x_hat + B*u; u_prev = u; return u; */
}
```

All loop bounds (`NP`, `NC`, `N_STATES`, `QP_MAX_ITER`) are `#define`d constants baked from the
specific `DiscreteMPC` instance passed to `generateControllerC()` - no dynamic sizing.

## Explicitly out of scope (this phase)

- `SuperTwistingSMC`, general `FuzzySystem`, MIMO `DiscreteMPC` (see Scope).
- Correctors other than `AntiWindupWrapper` (see Scope).
- `float`-precision emission (decision log item 4) - same pattern as `BasicPID<float>`, deferred.
- Python bindings for the generator functions themselves.

## Implementation checklist

1. `lib/DiscreteMPC.h` - add the 5 read-only accessors (no `.cpp` change needed, all trivial
   inline returns).
2. `lib/CodeGenC.h` + `lib/CodeGenC.cpp` - the six `generateControllerC()` overloads.
   `CTRL_REGISTER_FEATURE(code_gen_c)` at the bottom of the header.
3. `lib/CMakeLists.txt` - append `CodeGenC.cpp` to `CTRL_CORE_SOURCES`.
4. `lib/ControllerToolbox.h` - umbrella include for `CodeGenC.h`, inserted in the always-on
   include block.
5. `tests/test_catch2_advanced.cpp` - new `TEST_CASE`s tagged `[code_generation]` (see Testing
   plan). `tests/CMakeLists.txt` - `find_program(CTRL_C_COMPILER NAMES gcc cc clang)`; if found,
   `target_compile_definitions(test_catch2_advanced PRIVATE CTRL_C_COMPILER_PATH="${CTRL_C_COMPILER}")`
   so the golden-file tests can `std::system()` the discovered compiler; tests `SKIP` (not fail)
   when no C compiler is found in the build environment.
6. `examples/ex120_code_generation.cpp` - generates all six controller types (+ one
   corrector-fused example) and writes the resulting `.c`/`.h` pairs to disk, printing PASS/FAIL
   per generation call. `examples/CMakeLists.txt`, `compile.bat`, `compile.sh` - register `ex120`.
7. `docs/cumulative_bug_report.md` - new Part 71 section.
8. `docs/ALGORITHM_ROADMAP_PHASE3.md` status table - DT1 `Open` -> `Done`.
9. `docs/algorithm_backlog.md` - move the "Code generation" line to "Already done".

## Testing plan (`[code_generation]`, in `tests/test_catch2_advanced.cpp`)

1. **PID golden-file:** generate C for a `PIDParams` instance, compile standalone with the
   discovered C compiler (no Eigen/`lib/` link), run it over a fixed reference error sequence
   (including a saturating segment), compare output step-by-step against a live `DiscretePID`
   instance constructed with the same params - exact match (both are the same double-precision
   arithmetic).
2. Same golden-file pattern for `DiscreteSMC`, `DiscreteLeadLag`, `FuzzyPD`, `FuzzyPID`.
3. **MPC golden-file:** small SISO plant, `Np=3`/`Nc=2`, run a reference sequence that includes at
   least one actuator-saturating step; compare emitted-C output against live `DiscreteMPC::compute()`
   within `1e-9` (FISTA has no closed form, but both sides run the identical algorithm from
   identical baked matrices, so convergence should match to solver tolerance).
4. **Corrector golden-file:** `SMCParams` (no native anti-windup) + `AntiWindupConfig` corrector,
   compared against a live `AntiWindupWrapper(std::make_shared<DiscreteSMC>(...), uMin, uMax, Kb)`.
5. **Rejected-configuration guards:** `PIDParams` with `Kb != 0` + a corrector requested -> throws;
   `DiscreteMPC` + a corrector requested -> throws; non-SISO plant passed to the `DiscreteMPC`
   overload -> throws.
6. **Zero-allocation / freestanding check:** grep each generated `.c`/`.h` string for forbidden
   tokens (`malloc`, `calloc`, `new`, `std::`, `#include <vector>`, `#include <Eigen`) - none
   present.
7. **No C compiler available:** tests `SKIP` cleanly rather than failing the whole suite (matches
   how other environment-dependent Catch2 tests in this repo already degrade).
