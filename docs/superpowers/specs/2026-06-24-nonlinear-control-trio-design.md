# Design: Backstepping, Passivity-Based Control, and CLF Synthesis

**Date:** 2026-06-24
**Status:** Approved, not yet implemented

## Motivation

`docs/algorithm_backlog.md`'s Nonlinear Control section lists three gaps this toolbox can't
handle today without full feedback linearization: strict-feedback systems with relative degree
> 1 (Backstepping), energy-shaping regulation of Euler-Lagrange systems (Passivity-Based
Control), and direct Lyapunov-based controller *synthesis* from a candidate `V(x)` (CLF
synthesis - distinct from `LyapunovRobustness`, which only *analyzes* a fixed linear system's
robustness, not nonlinear synthesis). Bundled into one spec because the roadmap explicitly frames
all three as sharing a "physics-callback" convention with `FeedbackLinearisationController`, and
all three are `IController`-derived nonlinear synthesis controllers shipping in the same batch -
mirroring how Resonant/Notch/PLL were bundled by category and chronology despite no shared base
class.

**Shared caveat resolved once here (applies to NC1 and NC4 below):** `LyapunovRobustness.h`
(confirmed: `findCommonLyapunov`/`isQuadraticallyStable`) operates on a **list of vertex
matrices** for discrete-time linear polytopic uncertainty (`A_i'PA_i - P < 0` per vertex) - it
has no notion of a nonlinear `V(x)` or Lie derivatives. Wherever the roadmap mentions "verified
via `LyapunovRobustness`" or "consistent with `LyapunovRobustness`'s conventions," that's a
conceptual analogy (both use a quadratic `V = x'Px` *in the test cases*), not a function call -
each controller's Lyapunov-monotonicity test is a self-contained, test-local simulate-and-check
helper.

## Scope

- All three are `IController` subclasses, so they get the full checklist (bindings, `IController`
  base, `CONTRIBUTING.md` sign-convention table row) - not the lighter utility-class checklist
  SI2/MO2/FD1/EF1/RC1 used.
- **NC1 (Backstepping):** strict-feedback chains of arbitrary stage count `N` (state dimension
  `N`), not just the textbook 2-stage example - the per-stage callback vectors generalize
  cleanly to any `N`.
- **NC2 (Passivity-Based):** single-equilibrium **regulation** (`setDesired(q_d)`, a constant
  configuration), not trajectory tracking - matching the roadmap's own single `setDesired()`
  method (no `qdot_d`/`qddot_d` setters).
- **NC4 (CLF Synthesis):** SISO only (matching the roadmap's `compute(double)`-based sketch and
  scalar `LfVFn`/`LgVFn`) - a MIMO CLF-QP extension is a natural v2, not built now.

## Components

### 1. `lib/BacksteppingController.h` / `.cpp` - implements `IController`

Recursive Lyapunov design for `N`-stage strict-feedback systems (`x1' = f1(x) + g1(x)*x2`, ...,
`xN' = fN(x) + gN(x)*u`).

```cpp
struct BacksteppingParams {
    std::vector<double> k_gains;   // one stabilizing gain per stage, size N
    double uMin = -1e9, uMax = 1e9;
};

class BacksteppingController : public IController {
public:
    // Deviates from FeedbackLinearisationController::DriftFn/GainFn ((x, u_prev), single-stage):
    // backstepping's recursive virtual-control construction needs a stage index, since each
    // stage's drift/gain functions are evaluated at a different point in the recursion.
    using DriftFn = std::function<double(const Eigen::VectorXd &x, int stage)>;  // f_i(x)
    using GainFn  = std::function<double(const Eigen::VectorXd &x, int stage)>;  // g_i(x)

    BacksteppingController(std::vector<DriftFn> f, std::vector<GainFn> g,
                            const BacksteppingParams &params, double Ts);
    double compute(double error) override;     // error = r - x1 (top-level tracking error)
    void setState(const Eigen::VectorXd &x);
    void reset() override;
    double sampleTime() const override { return Ts_; }
};
```

**Recursive control law (resolves the roadmap's directional sketch into an implementable
algorithm):** define `z1 = x1 - r`, `alpha_0 = r` (the reference itself plays the role of stage
0's "virtual control"). For each stage `i = 1..N`:
```
z_i      = x_i - alpha_{i-1}
alpha_i  = (1/g_i(x,i)) * [ -f_i(x,i) - k_i*z_i + alpha_{i-1}' - g_{i-1}(x,i-1)*z_{i-1} ]   (i < N)
u        = clamp(alpha_N, uMin, uMax)                                                       (i = N)
```
where `alpha_{i-1}'` (the time-derivative of the previous stage's virtual control, needed for
the Lyapunov cross-term cancellation) is **approximated via a backward finite difference over
`Ts`** rather than computed analytically: `alpha_{i-1}'_k ~= (alpha_{i-1,k} - alpha_{i-1,k-1}) /
Ts`, using the value stored from the previous `compute()` call (and `r'` similarly, from the
reconstructed `r_k = x1_k + error_k`). This keeps the `DriftFn`/`GainFn` callback API as simple
as `FeedbackLinearisationController`'s (just `(x, stage)` - no Jacobian/partial-derivative
callbacks required from the caller) at the cost of an `O(Ts)` lag in the cancellation term,
acceptable at the sample rates this toolbox targets; a future v2 could accept optional analytic
derivative callbacks for stiffer systems. Only the final stage's `u` is clamped - intermediate
`alpha_i` virtual controls are unclamped mathematical setpoints, not physical signals, and the
stored `alpha_prev_` values used for the next step's finite difference are the **unclamped**
values (clamping the physical `u` must not corrupt the virtual-control derivative chain).

Hold-last NaN guard on non-finite `error`/`x`, consistent with the rest of the `compute()` fleet.

### 2. `lib/PassivityBasedController.h` / `.cpp` - implements `IController` (MIMO, `computeVec`)

Energy-shaping + damping-injection regulation for Euler-Lagrange systems
`M(q)*qddot + C(q,qdot)*qdot + dV(q) = u`.

```cpp
struct PBCParams {
    Eigen::MatrixXd Kp;       // energy-shaping (stiffness) injection gain, PSD - ADDED, not in
                               // the roadmap's sketch (see "Kp addition" below)
    Eigen::MatrixXd Kd;       // damping injection gain, PSD
    double uMin = -1e9, uMax = 1e9;
};

class PassivityBasedController : public IController {
public:
    using MassMatrixFn    = std::function<Eigen::MatrixXd(const Eigen::VectorXd &q)>;
    using PotentialGradFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &q)>;
    using CoriolisFn      = std::function<Eigen::MatrixXd(const Eigen::VectorXd &q,
                                                            const Eigen::VectorXd &qdot)>;
    PassivityBasedController(MassMatrixFn M, PotentialGradFn dV, CoriolisFn C,
                              const PBCParams &params, double Ts);

    // Deviates from the roadmap's q_qdot_error naming: takes the raw stacked state [q; qdot]
    // (2n x 1), SignConvention::PlantOutput (matching DiscreteLQG/MRACController's convention
    // for controllers needing the plant state directly) - there's no meaningful "qdot error"
    // for constant-setpoint regulation (qdot_d = 0 implicitly), so the controller computes
    // q - q_d internally from the configuration set via setDesired().
    Eigen::VectorXd computeVec(const Eigen::VectorXd &state) override;
    double compute(double signal) override;   // throws std::logic_error - see below
    SignConvention signConvention() const override { return SignConvention::PlantOutput; }
    void setDesired(const Eigen::VectorXd &q_d);
    void reset() override;
    double sampleTime() const override { return Ts_; }

    // New accessor, not in the roadmap's sketch: exposes the shaped total-energy storage
    // function for passivity monitoring/testing (see "Kp addition" and storage-function note).
    double storageEnergy() const;
};
```

**Control law (PD+ regulation, Takegaki & Arimoto 1981):**
```
u = dV(q) - Kp*(q - q_d) - Kd*qdot
```
**`Kp` addition (resolves a gap in the roadmap's sketch):** the roadmap's `PBCParams` lists only
`Kd`, but a damping-only law has no restoring force toward `q_d` (nothing shapes the *potential*
side of the energy, only dissipates kinetic energy) - "energy shaping" specifically refers to the
`Kp` term reshaping the total potential so its minimum sits at `q_d`. `Kp` is added as a required
parameter; without it the controller cannot regulate to a nonzero `q_d` at all.

**Why `M`/`C` are accepted but not used in `u` (resolves the roadmap's apparent-but-unstated
assumption):** the classical PD+ stability proof (Lyapunov/storage function `V = 0.5*qdot'*M(q)
*qdot + 0.5*(q-q_d)'*Kp*(q-q_d)`) shows `V' = -qdot'*Kd*qdot <= 0` **without** `u` needing to
explicitly cancel `C` - the cross term vanishes via the standard Lagrangian skew-symmetry
property `qdot'*(Mdot(q) - 2*C(q,qdot))*qdot = 0`, which holds when `C` is derived from `M` via
the conventional Christoffel-symbol factorization (the standard choice in robotics/Lagrangian
modeling; a non-standard caller-supplied `C` may not satisfy it exactly, in which case the proof's
exact decay rate doesn't hold even though the law typically still stabilizes in practice). `M`/`C`
*are* still evaluated every step - they feed `storageEnergy()` (the new accessor above) for
passivity monitoring, which is also what the roadmap's own test plan item 2 needs to verify
(numerically checking the storage function is non-increasing along a trajectory).

**Mass-matrix-singular test (resolves an apparent contradiction with "M isn't used in `u`"):**
since `M(q)` is still evaluated every step for `storageEnergy()`, a caller-supplied `MassMatrixFn`
that returns a non-finite matrix (e.g. a physics model with a coordinate singularity at a
boundary configuration) must not corrupt `u` even though `u`'s formula never inverts `M`. Guard:
if `M(q)` or `C(q,qdot)` evaluates to a non-finite matrix, hold the last finite `u` (same NaN
contract as the rest of the fleet) rather than letting it propagate through `storageEnergy()`'s
internal state.

**`compute(double)` throws `std::logic_error`:** unlike `LQRAdapter` (which can return `u[0]`
because its state comes from a *separate* callback, not the scalar argument), `PassivityBasedController`
has no way to recover both `q` and `qdot` from a single scalar - even the SISO case (`n=1`,
single pendulum) needs 2 numbers. `compute(double)` therefore throws
`std::logic_error("PassivityBasedController is MIMO-only ([q;qdot] together) - call computeVec().")`,
mirroring `IController::computeVec`'s own default behavior of throwing rather than silently
truncating a multi-element signal.

### 3. `lib/CLFController.h` / `.cpp` - implements `IController` (SISO)

```cpp
struct CLFParams {
    double alpha = 1.0;        // decay rate
    double uMin = -1e9, uMax = 1e9;
    bool   useQP = true;       // see "useQP note" below
};

class CLFController : public IController {
public:
    // VFn ADDED - not in the roadmap's sketch. The decay condition LfV + LgV*u <= -alpha*V
    // needs V(x) itself to evaluate the right-hand side; LfV/LgV alone (the roadmap's two
    // callbacks) cannot express it. Resolved by adding a third required callback.
    using VFn   = std::function<double(const Eigen::VectorXd &x)>;  // candidate Lyapunov V(x)
    using LfVFn = std::function<double(const Eigen::VectorXd &x)>;  // drift Lie derivative
    using LgVFn = std::function<double(const Eigen::VectorXd &x)>;  // control Lie derivative
    CLFController(VFn V, LfVFn LfV, LgVFn LgV, const CLFParams &params, double Ts);
    double compute(double error) override;  // error is unused - see "compute(error)" note below
    void setState(const Eigen::VectorXd &x);
    bool isHealthy() const override;   // false after an infeasible step (see below)
};
```

**`compute(error)` note:** the roadmap sketch names this parameter `error`, but CLF synthesis
stabilizes toward `V`'s equilibrium (a regulation problem, not reference tracking) - the
parameter is unused, mirroring `LQRAdapter::compute(double /*signal*/)`'s pattern (state comes
entirely from `setState()`, not the scalar argument).

**Control law (Sontag's universal formula):** with `a = LfV(x) + alpha*V(x)` and `b = LgV(x)`,
```
u = -(a + sqrt(a^2 + b^4)) / b     if b != 0
```
(smooth and always satisfies `a + b*u <= 0`, since substituting gives `a + b*u = -sqrt(a^2+b^4)
<= 0` exactly). Then `u = clamp(u, uMin, uMax)`.

**Infeasibility (`LgV = 0`, uncontrollable direction, with `a > 0` - drift not naturally
decaying):** the roadmap's test plan item 3 ("flags infeasible rather than producing a nonsense
`u`") is satisfied via `isHealthy()` returning `false` after such a step (reusing
`IController::isHealthy()`'s existing supervisory-fallback contract - no new bespoke API) and
holding the last finite `u`, consistent with the rest of the fleet's hold-last NaN/fault
convention generalized to "instantaneously infeasible" as well as "non-finite input."

**`useQP` note (resolves the `GradientProjectionQP` mismatch):** `GradientProjectionQP` solves
box-constrained QPs only; the CLF-QP problem's actual constraint (`LfV + LgV*u <= -alpha*V`) is a
half-space constraint, not a box, so it cannot be expressed via that solver directly. For the
SISO scope this phase covers, `useQP=true` and `useQP=false` both run the identical closed-form
Sontag-formula path above - `useQP` is kept as a parameter for sketch-API compatibility and as a
hook for a genuine future MIMO QP-based extension (where a half-space constraint folded into a
box via a slack variable would actually call `GradientProjectionQP`), but v1 does not call that
solver at all. This satisfies the roadmap's test plan item 2 ("QP mode and Sontag-formula mode
agree on unconstrained cases") trivially and honestly, rather than building an unused QP code
path.

## Explicitly out of scope (this phase)

- **NC1: analytic virtual-control derivatives** - only the finite-difference approximation of
  `alpha_i'` is built; an optional analytic-Jacobian callback variant is a natural, separable v2.
- **NC2: trajectory tracking** (`qdot_d`/`qddot_d`, explicit Coriolis cancellation in `u`) - only
  constant-setpoint regulation; `setDesired()` deliberately takes one constant `q_d`.
- **NC2: IDA-PBC / full inertia shaping** - only potential-side energy shaping (`Kp`) plus
  damping injection (`Kd`); modifying the apparent inertia is a materially bigger lift.
- **NC4: MIMO CLF-QP** - SISO only; a real half-space-via-box-slack QP formulation for MIMO is
  deferred to a future extension once a concrete MIMO use case exists.
- **NC4: control-Lyapunov-function *search*** (finding a valid `V(x)` automatically) - `LfVFn`/
  `LgVFn` are caller-supplied; `CLFController` synthesizes the control law from a *given*
  candidate `V`, it does not search for one.

## Implementation checklist

(Full `IController` checklist per `CONTRIBUTING.md`'s "adding a new controller" workflow, for
all three.)

**Per controller** (`BacksteppingController`, `PassivityBasedController`, `CLFController`):
1. `lib/<Name>.h`/`.cpp` + `CTRL_REGISTER_FEATURE(<name>)`
2. `lib/CMakeLists.txt` - add to `CTRL_CORE_SOURCES`
3. `lib/ControllerToolbox.h` - add `#include "<Name>.h"` near `FeedbackLinearisation.h`
4. `bindings/controllers_bindings.cpp` - bind as `shared_ptr<T>` + `ctrl::IController` base
   (required for `ControllerStack.add_controller()`), alongside `FeedbackLinearisationController`
5. `bindings/smoke_test.py` - construct, call `compute()`/`computeVec()`, confirm callable
6. `tests/test_catch2_advanced.cpp` - tests under `[backstepping]` / `[passivity_based]` /
   `[clf_controller]` (see Testing plan)
7. `CONTRIBUTING.md` sign-convention table - add `BacksteppingController` -> `Other` (or audit
   to `TrackingErrorRMinusY` per the `error = r - x1` convention), `PassivityBasedController` ->
   `PlantOutput`, `CLFController` -> `Other` (state-dependent, not a fixed error sign)

**Examples** (next available numbers, in roadmap order NC1 -> NC2 -> NC4):
- `examples/ex97_backstepping.cpp` - 2-link-arm-style strict-feedback system, tracking a
  reference that flat feedback linearization (relative degree > 1) can't handle directly
  + `examples/python/ex114_backstepping.py`
- `examples/ex98_passivity_based.cpp` - single-pendulum regulation to a nonzero `q_d` despite
  unmodeled friction, monitoring `storageEnergy()` non-increase + `examples/python/ex115_passivity_based.py`
- `examples/ex99_clf_controller.cpp` - scalar nonlinear system with a known quadratic CLF,
  comparing Sontag-formula output to the hand-derived closed form + `examples/python/ex116_clf_controller.py`
- `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` - add all three example targets

## Testing plan

**`[backstepping]`**
1. 2-stage strict-feedback system with a known analytic backstepping law (textbook example,
   `Khalil Ch. 14`) - tracking error converges to zero, matching the hand-derived control law's
   output within the finite-difference approximation's expected error bound.
2. Lyapunov function `V = 0.5*sum(z_i^2)` verified numerically non-increasing along a simulated
   trajectory (test-local helper - not a call into `LyapunovRobustness`, see Motivation).
3. Actuator saturation (`uMin`/`uMax`) - output is hard-clamped; confirm the stored `alpha_prev_`
   chain used for the next step's finite difference is unaffected by the clamp (no windup
   artifact in the virtual-control derivative).

**`[passivity_based]`**
1. Single-pendulum regulation to a nonzero `q_d` - converges to the desired angle;
   `storageEnergy()` is non-increasing across the trajectory.
2. Closed-loop passivity verified via the storage-function check across a trajectory, using the
   caller-supplied `M`/`C` (confirming the skew-symmetry property holds for the test system's
   Christoffel-factorized `C`).
3. `MassMatrixFn` returns a non-finite matrix at a boundary configuration - `compute()`/
   `computeVec()` hold the last finite `u`, `storageEnergy()` does not propagate NaN into
   internal state.
4. `compute(double)` throws `std::logic_error`.

**`[clf_controller]`**
1. Known CLF for a scalar nonlinear system - Sontag-formula output matches the hand-derived
   closed form.
2. `useQP=true` and `useQP=false` produce numerically identical output (both run the same
   closed-form path in v1 - see "useQP note").
3. `LfV` positive, `LgV = 0` (uncontrollable direction) - `isHealthy()` becomes `false`, output
   holds the last finite value rather than a nonsense (or divide-by-zero) result.
