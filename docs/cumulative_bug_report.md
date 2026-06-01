# Cumulative Bug Report & Engineering Review -- Part 26+

> Active-issues tracker for Part 26 onward. Parts 1-25 are archived in
> `docs/compact_bug_report_parts_1-25.md`. This file opens with a senior-review
> pass (Part 26) and then becomes the running issue log.

---

## Part 26 -- Senior Code Review (controls + OSS lens)

**Reviewer hat:** someone who has shipped a discrete-time control library, broken
a few plants in the field, and read more `python-control` / Drake / Eigen source
than is healthy. Tone is deliberately blunt -- we're peers, you can take it.

**Baseline at review time:** C++ 90 / 0, Python 88 / 0, 4 case studies green
(Boiler 216/216, SMISMO 42/42, Solar 45/45, Tug 64/64). So none of what follows
is "it's broken" -- it's "it's good, here's where the next 10% of quality and the
scalability story actually live."

### TL;DR verdict

This is a genuinely strong codebase. ~52 headers, ~39 TUs, ~60 algorithms from
PID through TubeMPC/ParticleFilter, **zero `TODO`/`FIXME`/`HACK` in `lib/`**, and
**every `lib/` header carries a Doxygen `@brief`**. The DARE doubling and ADRC
ESO derivations are the kind of thing you wish every control library shipped. The
test suite is 83 Catch2 cases across ~63 tags with scipy/control cross-validation.

The problems are not in the algorithms. They're in the **scaling story**: the
"how do I add the 61st controller without touching 8 files" problem, the
"3 of my 4 flagship case studies have no unit tests" problem, and the slow
documentation drift that always starts the moment a `README` hard-codes a number.

---

### A. Algorithmic robustness

**R1 -- NaN/Inf guarding is inconsistent across the controller fleet.**
`DiscreteADRC::computeTracking()` does the right thing
([DiscreteADRC.cpp:54-55](../lib/DiscreteADRC.cpp#L54-L55)): non-finite input ->
return last good `u`, fail safe. That pattern is *not* uniform. Most controllers
trust their inputs. For a library whose whole pitch is "deploy this on real
digital hardware," a single `NaN` from a flaky sensor silently poisons every
integrator and observer state downstream. **Ask:** lift the ADRC guard into a
shared helper (`ctrl::sanitize(double, fallback)`) and apply it at every
`compute()` boundary, or document explicitly that input sanitisation is the
caller's job. Right now it's neither-here-nor-there.

**R2 -- Saturation/anti-windup handling is per-controller tribal knowledge.**
The CLAUDE.md caveat "do NOT wrap `DiscretePID` with `AntiWindupWrapper` (it has
built-in `Kb`)" is exactly the kind of footgun that should be impossible, not
documented. Two anti-windup mechanisms that conflict when composed is an
*interface* gap, not a docs gap. A capability query
(`bool hasInternalAntiWindup()`) would let `AntiWindupWrapper` refuse or no-op
instead of double-integrating.

**R3 -- DARE/QP non-convergence is reported but easy to ignore.**
`isHealthy()` ([IController.h:103](../lib/IController.h#L103)) is the right hook,
and `ControllerStack` honours it. But standalone users (every case study) call
`lqr.compute()` / `mpc.computeRef()` and never check health. The Tug's new
`TubeMPCTugCtrl`/AutoGS controllers emit 4 `[DiscreteLQR] WARNING: PBH
stabilizability` lines per run -- benign here, but the only signal is stderr spew.
**Ask:** make non-convergence observable through the telemetry channel (see M3),
not just stderr, so an application can react instead of grep.

**R4 -- Euler-discretised plant models inside controllers.**
The new NMPC controllers (`NMPCBoilerCtrl`, `NMPCSmismoCtrl`, `NMPCTugCtrl`) and
EKF predict steps integrate nonlinear dynamics with **forward Euler at the
control Ts**. Fine for the well-damped boiler; risky for the SMISMO hydraulic
plant whose own simulator runs 5 internal RK4 sub-steps because `beta_e=300 MPa`
gives ~60 Hz resonance. A controller's internal model integrated at 200 Hz Euler
will mispredict the fast pressure modes. Not wrong today (the regression passes),
but it's a latent accuracy cliff. At minimum, comment the stability assumption;
better, expose an internal sub-step count mirroring the plant.

### B. Maintainability & architecture

**M1 -- Core/adapter separation exists for exactly two controllers.**
`DiscreteLQR` (pure algorithm) + `LQRAdapter` (IController shim) is the clean
pattern the user's element #1 is asking for -- and it's only done for LQR/LQG.
Everywhere else the algorithm *is* the `IController` (`DiscretePID` is both the
math and the interface). That's fine pragmatically, but it means the "modular
architecture separating core logic from interface adapters" goal is **already
half-built and then abandoned.** Pick one: either commit to the
algorithm/adapter split library-wide, or document that `DiscreteLQR` is the
deliberate exception (stateless math that several adapters wrap) and stop
implying a pattern that isn't there.

**M2 -- Adding a controller touches ~8 files. This fights element #4 directly.**
The 8-step checklist (CLAUDE.md -> CONTRIBUTING.md Steps 1-5) requires edits to:
`lib/*.{h,cpp}`, `lib/CMakeLists.txt`, `lib/ControllerToolbox.h`,
`lib/Features.h`, `bindings/*_bindings.cpp`, `bindings/smoke_test.py`,
`tests/*.cpp`, `examples/* + compile.bat`. The user wants "a plugin system
allowing extension **without modifying existing codebase**." Today the *opposite*
is true -- and [Features.h:20-77](../lib/Features.h#L20-L77) is the smoking gun: a
**hand-maintained `unordered_map`** with `{"name", true}` literals and comments
like `// always compiled`. That map will rot the first time someone forgets to
update it (it already encodes build facts that CMake also encodes -- two sources
of truth). See element #4 below for the concrete fix.

**M3 -- Telemetry only sees the boundary, never the guts.**
[IControllerObserver.h:53](../lib/IControllerObserver.h#L53) gives you
`onCompute(u, signal)` and nothing else. For *linear* controllers that's enough.
For the nonlinear/observer-based half of the library -- ADRC's ESO states
`z1,z2,z3`, SMC's sliding surface `s`, MPC/NMPC QP iteration count, EKF
covariance trace -- the one signal you actually want when debugging a divergence
is invisible. This is element #5's whole point. The Observer pattern is in place;
it just needs a typed, optional `onState(string key, VectorXd value)` channel.

**M4 -- `double` is welded in everywhere.**
No scalar templating, no fixed-point path. That's a defensible choice (Eigen
`double` is the sane default), but element #3 ("precision levels typical in
digital control") has zero hooks today. A `float` build for an MCU target would
be a find-and-replace archaeology dig. If embedded precision is a real
requirement, the time to `template<typename Scalar>` the leaf algorithms is
*before* there are 60 of them, not after.

### C. Algorithm gap analysis

**G1 -- Estimation/ID is rich; *robust* and *constrained* corners are thin.**
You have KF/EKF/UKF/MHE/ParticleFilter and N4SID/RLS/LPV -- excellent spread. Gaps
a controls reviewer notices: (a) **MHE has no state constraints** (open item T4,
and the header admits it) -- that's *the* reason to pick MHE over a KF, so it's
the missing 50%; (b) no **explicit reference governor / constraint-admissible
set** for the non-MPC controllers; (c) **nu-gap is SISO-only** (T2) so the
auto-gain-scheduler's clustering can't reason about MIMO plants -- which is
exactly what the Boiler (3x3) and Tug (6-state MIMO) need.

**G2 -- `AutoGainScheduler` design callback can't return the obvious controller.**
Wiring up `AutoGSBoilerCtrl`/`AutoGSTugCtrl` this part exposed it: the
`design_fn` must return `shared_ptr<IController>`, but `DiscreteLQR` **isn't** an
`IController` (M1!). So the scheduler -- whose textbook use case is *gain-scheduled
LQR* -- can't host an LQR without hand-rolling a PID that apes the LQR gain. That's
a real capability hole created by the M1 inconsistency. Either ship a first-class
`LQRAdapter`-returning convenience, or make the scheduler accept a
state-feedback functor.

**G3 -- No discrete-time *delay/jitter* model in the loop.**
The HAL has a `SimScheduler`, but controllers assume `u[k]` lands instantly. Real
digital loops have one-sample computational delay and release jitter. A
first-class "one-step input delay" wrapper (and a test that proves PID/MPC
tolerate it) would be a credible, in-spirit addition for a *discrete-time* library
making deployment claims.

### D. Testing

**T1 -- The library is well-tested; the case studies essentially aren't.**
83 Catch2 cases, ~63 tags, scipy cross-val -- `lib/` coverage is genuinely strong.
But: a grep for `boiler::`, `smismo::`, `solar::` across `tests/` returns
**nothing**. Only the Tug has a regression harness
(`test_tugsim_regression.cpp`). So of **65 case-study controller classes, ~50
have no automated correctness check at all** -- they're validated by "the `.exe`
returned 0," which a controller that drives the plant to a wall also does.
**This is the single highest-value gap in the repo.** A controller that silently
regresses IAE by 40% is green today.

**T2 -- Edge-case/stability tests are present but not systematic.**
There are `[dare]`, `[stability_margins]`, `[numerical]` tags -- good. What's
missing is a *matrix*: for each controller, a test that (a) injects a `NaN` and
asserts graceful handling (ties to R1), (b) drives it into sustained saturation
and asserts anti-windup actually bounds the integral, (c) feeds a marginally
stable / non-stabilizable plant and asserts `isHealthy()==false`. Right now those
behaviours are asserted for a couple of controllers, not as a contract every
controller must pass.

### E. Documentation -- the good, the drifting, the absent

The header-level discipline is **excellent** and should be said plainly:

- **Gold standard:**
  [DiscreteADRC.cpp:31-78](../lib/DiscreteADRC.cpp#L31-L78) -- derives the
  backward-Euler ESO as `(I - Ts*Ae)^-1` in closed form, explains *why*
  semi-implicit (`eps` uses old `z1`) buys A-stability, and ties it to the PD +
  disturbance-cancellation law. You can re-derive the code from the comment. This
  is `python-control`/Drake quality.
- **Gold standard:** [IController.h](../lib/IController.h) and
  [IControllerObserver.h](../lib/IControllerObserver.h) -- every method has
  `@param`/`@return`, the `computeVec()` default *throws* on silent MIMO
  truncation ([IController.h:47-54](../lib/IController.h#L47-L54)) and says so.
  Defensive-by-documentation done right.

Now the **drift and the crutches** -- concrete, because the user asked for them:

- **Poor (drift) [FIXED in the Part 26 doc pass]:** [CONTRIBUTING.md:38](../CONTRIBUTING.md#L38)
  claimed *"Expected baseline (Part 22): C++ 85/85 | Python 84/84"* when reality was
  Part 26, 90/88. Now updated to the Part 26 numbers **plus** an explicit "treat the
  latest run_*.log as source of truth" disclaimer. A hard-coded number in prose is a
  guaranteed lie with a time delay; the moment a doc embeds a count, it needs a single
  source of truth or an "as of commit X" stamp. (This very file will rot the same way if
  we're not careful - every doc count touched in Part 26 carries an "as of Part 26" date.)
- **Poor (crutch):** the `compute(double signal)` contract is *polymorphic by
  prose*. [IController.h:25-34](../lib/IController.h#L25-L34) admits `signal` is
  "tracking error **or** plant output, depending on controller type," and the
  real spec lives in a 11-row table in
  [CONTRIBUTING.md:99-109](../CONTRIBUTING.md#L99-L109) plus the CLAUDE.md
  sign-convention block. When the *call site* can't tell you whether to pass
  `r-y` or `y`, that's an API leaking into documentation. The docs are excellent
  *because they're compensating for a sign convention the type system doesn't
  enforce.* (`DiscreteSMC` flips the sign; `MRAC`/`ESC` want raw `y`; everyone
  else wants error.) A `struct TrackingError{double}` / `struct PlantOutput{double}`
  newtype would move this from "tribal knowledge" to "compiler error."
- **Poor (rot-by-design):** [Features.h:58-75](../lib/Features.h#L58-L75) -- every
  entry is `{"name", true}, // always compiled`. The comment asserts a build fact
  the file can't verify. It's documentation *and* data *and* a manual sync chore,
  all in one place that no test checks.
- **Absent:** this very file (`docs/cumulative_bug_report.md`) **did not exist**
  before this review, despite CLAUDE.md citing it as the active issue tracker and
  CONTRIBUTING.md:132 instructing contributors to log every change here. The
  process doc referenced an artifact the repo didn't have. (Fixed: you're reading
  it.)

**Exemplary OSS practices worth stealing (the user asked for references):**

- **Eigen** -- the "Quick reference guide" + one focused page per module, and the
  rule that every public method's doc is *runnable*. Your ADRC comment already
  meets this bar; make it the enforced minimum.
- **numpydoc / NumPy docstring standard** -- the *Parameters / Returns / Raises /
  Notes / Examples* skeleton. You're 90% there in Doxygen; the missing slot is a
  consistent **`@throws`/Raises** line (you throw `std::logic_error`,
  `std::invalid_argument`, `std::runtime_error` in places that don't always
  document it).
- **python-control** -- your closest peer. Worth mirroring their convention of
  documenting the *discrete-time convention* (sample-and-hold, sign) once,
  canonically, and linking every controller to it instead of re-stating per file.
- **GNU GSL / Drake** -- error-handling-as-contract: a documented, typed error
  surface (`isHealthy()`, return codes) rather than stderr. You have the hook;
  finish wiring it (R3/M3).
- **Doxygen `\xrefitem`** (custom "Convention:" / "Stability limit:" tags) --
  would let the omega_o*Ts<0.5, ZOH-default, and sign rules render as a single
  auto-collated page instead of being scattered.

---

### F. The six requested additions -- honest scorecard

Keeping the discrete-time-controller spirit, here's where each stands and the
in-spirit way to land it:

| # | Request | Status today | In-spirit recommendation |
|---|---------|--------------|--------------------------|
| 1 | Core logic vs interface adapters | **Half-built** -- only `DiscreteLQR`/`LQRAdapter` (M1) | Either split library-wide, or canonise LQR as the deliberate "stateless math + thin adapters" exception and stop implying the pattern elsewhere. Don't leave it ambiguous. |
| 2 | Automated unit tests for stability edge cases | **Strong for `lib/`, absent for 3/4 case studies** (T1) | Add a `tests/test_casestudy_*.cpp` per plant asserting IAE/settling thresholds; add the R1/R2/R3 edge-case *contract matrix* (T2). This is the highest-ROI item. |
| 3 | Config for sampling rates / precision | **Ts: yes. Precision: no** (M4) | Ts is already a ctor arg -- good. For precision, `template<typename Scalar=double>` the leaf algorithms *now*, or explicitly scope it out. Don't pretend the hook exists. |
| 4 | Plugin system, no edits to existing code | **Actively contradicted** -- 8-file checklist + manual `Features.h` (M2) | Replace the hand-written `features()` map with a **self-registration registry**: a static `ControllerRegistry::add("name", factory)` invoked by a `CTRL_REGISTER(Type)` macro in each `.cpp`. New controller = new TU + one macro line; the umbrella header and feature map stop being edit points. That's an actual plugin system, and it's pure C++ (no framework). |
| 5 | Logging of state transitions for nonlinear debug | **Partial** -- Observer sees only `(u, signal)` (M3) | Extend `IControllerObserver` with an optional `onState(std::string_view, const Eigen::VectorXd&)`; have ADRC emit `z`, SMC emit `s`, MPC/NMPC emit QP iters/cost, EKF emit `trace(P)`. Default no-op so it's zero-cost when unused. |
| 6 | Doc templates mirroring Eigen/etc. | **Header docs already at that bar; process docs drifting** (E) | Lock a `@throws` line into the template, add a single canonical "Discrete-time conventions" page and `@see` it everywhere, and put a CI check that greps for stale hard-coded baselines like CONTRIBUTING.md:38. |

---

### G. Prioritised punch list (what I'd actually do next, in order)

1. **Case-study regression tests** (T1) -- biggest correctness blind spot. Boiler/
   SMISMO/Solar each get a `tests/test_*_regression.cpp` asserting per-controller
   IAE/settling bounds. Mirror `test_tugsim_regression.cpp`.
2. **Self-registration registry** (M2/element #4) -- kills the `Features.h` manual
   map and most of the 8-file checklist. Unblocks the "plugin" goal honestly.
3. **`onState()` telemetry** (M3/element #5) -- small, additive, high debugging
   value for the nonlinear half of the library.
4. **NaN-guard helper + edge-case contract matrix** (R1/T2) -- make fail-safe a
   tested contract, not an ADRC-only courtesy.
5. **Fix the doc drift** (E) -- CONTRIBUTING.md baseline stamp, a canonical
   conventions page, `@throws` in the template. Cheap, stops the rot.
6. **Newtype the `compute()` signal** (E) -- `TrackingError` / `PlantOutput`
   wrappers turn the sign-convention table into compile-time safety. Bigger
   change; do it when you next touch the binding layer.

None of this is "the library is broken." It's the difference between a very good
research-grade toolbox and one you'd hand to a junior engineer and trust to ship.

---

## Open issues log (Part 26+)

*(Append dated entries below as work proceeds. Keep `docs/compact_bug_report_parts_1-25.md`
as the frozen archive.)*

- **[Part 26 review]** All findings above are tracked as R1-R4, M1-M4, G1-G3,
  T1-T2, plus the element scorecard. None are regressions; all are
  forward-looking quality/scalability items. Existing backlog T2-T7 (see
  `CLAUDE.md`) folds into G1/G2 (MIMO nu-gap, DK-iteration, MHE constraints).
