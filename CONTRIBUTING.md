# Contributing to Controller Toolbox

This document covers the conventions, workflows, and checklists for adding to or modifying the Controller Toolbox. Read it fully before opening a pull request.

---

## Table of Contents

1. [Build and Test Workflow](#build-and-test-workflow)
2. [Adding a New Controller](#adding-a-new-controller)
3. [Adding a New Case Study](#adding-a-new-case-study)
4. [Sign Conventions](#sign-conventions)
5. [Numerical Safety Rules](#numerical-safety-rules)
6. [Documentation Standard](#documentation-standard)
7. [Python Binding Conventions](#python-binding-conventions)
8. [NumPy 2.x Compatibility](#numpy-2x-compatibility)
9. [PR Checklist](#pr-checklist)

---

## Build and Test Workflow

The project uses a seven-phase runner managed by `run.py`:

| Phase | What happens |
|---|---|
| 1 - Non-ASCII scan | Auto-replaces known non-standard Unicode characters in all `.cpp`, `.h`, `.py`, `.txt` files. Write ASCII-only source. |
| 2 - Compile | Runs `compile.bat` (Windows) or `compile.sh` (Linux/macOS) which call `cmake --build` **sequentially** (no `--parallel`). Targets are listed explicitly in dependency order. |
| 3 - Bindings | Builds `ctrl_toolbox` Python bindings and runs `bindings/smoke_test.py`. |
| 4 - Run C++ | Runs every `.exe` under `build/` (examples, tests, and case-study `*_sim` targets), prints pass/fail. |
| 5 - Run Python | Runs every `exNN_*.py` under `examples/python/` via the `soft_robotics` conda environment. |
| 6 - Python case studies | Discovers and runs `case-study/*/sim/main.py` (Python-only studies). |
| 7 - Case-study status + report | Runs `tools/case_study_tracker.py` (refreshes `docs/case_study_status.md`) and `tools/generate_report.py` (writes `docs/report.html`). |

**Always invoke via:**
```
conda run -n soft_robotics -- python run.py
```

**Avoid `cmake --build build --parallel` on local/dev machines** - `compile.bat`/`compile.sh` build
~140 targets sequentially specifically to avoid pegging every core on a dev machine for the whole
run. This is a local resource-stress mitigation, not a build-correctness requirement - CI
(`.github/workflows/cross-platform-cicd.yml`) intentionally builds with `--parallel $(nproc)`,
and that's fine there.

**Expected baseline (as of Part 60, 2026-06-16, UNVERIFIED):** `C++ ~174 passed | Python ~100 passed`,
nine C++ case studies (Boiler 216, Tug 72, Solar 70, Humid 75, ActiveSuspension **90**, BuckBoost 60,
SolarCooker 60, SOTEC 60, SMISMO **65**) and seven Python-only studies
(DrillString 85, WindWave 80, EHFS **70**, Firefighting 60, BTMS 60, SurfaceShip 60, EV6x6 **90**).
Treat the latest `run_*.log` as the source of truth, not this line.

A log file `run_YYYYMMDD_HHMMSS.log` is written to the project root after every run.

---

## Adding a New Controller

Follow these steps in order. Check each off before declaring the work done.
`tools/new_controller.py <ClassName>` scaffolds `lib/ClassName.{h,cpp}` and the C++/Python
example stubs for Steps 1 and 3 (TODOs left for the real control law); it prints the
remaining manual steps below since they touch shared files the script doesn't own.

### Step 1 - Implement the class

- Place in `lib/ClassName.h` and (if needed) `lib/ClassName.cpp`.
- Subclass `IController` (public inheritance).
- Override all three pure virtuals: `compute(double)`, `reset()`, `sampleTime()`.
- Call `notifyObserver(u, signal)` at the end of every `compute()` override.
- Call `notifyObserverReset()` at the end of every `reset()` override.
- All matrix inverses must be guarded (use `ldlt_.info() != Eigen::Success` or eigenvalue floor - see `DiscreteMPC::buildCondensedMatrices()` and `UnscentedKalmanFilter::update()` for patterns).
- No `std::cerr`/`std::clog` in production paths. Wrap in `#ifndef NDEBUG` if needed.

### Step 2 - Wire into the build

- Add the `.cpp` file to `CTRL_CORE_SOURCES` in `lib/CMakeLists.txt`.
- Add `#include "ClassName.h"` to `lib/ControllerToolbox.h` with a one-line `///< ...` docstring.
- Add a feature flag entry to `lib/Features.h` (`{"feature_name", true}`).
- Add the example target name to the `for %%T in (...)` loop in `compile.bat`.

### Step 3 - Write examples

- **C++ example:** `examples/exNN_classname.cpp` using `#include "ControllerToolbox.h"`.
  Print `PASS` on success, `FAIL` and exit code 1 on failure.
  Add `add_example(exNN_classname)` to `examples/CMakeLists.txt`.
- **Python example:** `examples/python/exNN_classname.py`.
  Start with `import _setup_bindings`, end with `print("PASS")`.
  run.py discovers it automatically.

### Step 4 - Add Catch2 tests

- Add at least one test to `tests/test_catch2_advanced.cpp`.
- Every test must have a `[tag]` matching the controller family (e.g., `[nmpc]`, `[smc]`).
- Tests must check an actual numeric value, not just "no crash".
- Tolerance choices must be commented with justification.
- scipy/control cross-validation (via Python reference values embedded in comments) is the standard for any mathematical result.
- **Never put an unbalanced `[` or `]` in a `TEST_CASE` name.** Catch2 v3.5.4's
  `CatchAddTests.cmake` parses `--list-tests` with `foreach(line ${output})`; the unquoted
  expansion makes CMake treat a lone bracket as a grouping delimiter and stop splitting on `;`,
  so the offending test **and every test declared after it** collapse into one bogus `ctest`
  entry whose filter matches nothing. The symptom is a single FAILED test with a
  hundreds-of-characters-long name and "No tests ran" in its output - it looks like an assertion
  failure and is not one. `"DDMR plant: wrapAngle maps into (-pi, pi]"` cost 25 tests this way.
  Balanced pairs (`"bounded [0, 1]"`) are safe; write half-open intervals in words instead.
  Enforced mechanically - `run.py` **Phase 2b** and the CI gate both run:
  ```bash
  python tools/check_test_names.py
  ```

### Step 5 - Python bindings

- Add binding code to the appropriate `bindings/*_bindings.cpp` file.
- All `IController` subclasses **must** use `std::shared_ptr<T>` as the third template argument to `py::class_`:
  ```cpp
  py::class_<ctrl::MyCtrl, ctrl::IController, std::shared_ptr<ctrl::MyCtrl>>(m, "MyCtrl")
  ```
  Missing this causes a "custom holder" `RuntimeError` at `ControllerStack.add_controller()`.
- Use `py::object` capture in `std::function` lambdas, **not** `py::cpp_function`.
- Add an assertion to `bindings/smoke_test.py`.

---

## Adding a New Case Study

Per-study status and links are auto-tracked in `docs/case_study_status.md` (regenerate via
`tools/case_study_tracker.py` - do not edit it by hand); rosters and tribal knowledge live in
`CLAUDE.md`'s Case Studies section and in each study's own `README.md`. Use the newest C++
studies (S-OTEC, Solar Cooker, SMISMO) as templates.

Note: the tracker's "On-going" status only checks for `sim/` + `logs/` + `config/` presence,
not real content. A few directories scaffolded by `tools/new_case_study.py` show as "On-going"
while still containing only placeholder dynamics and a single `OpenLoop` controller - check the
study's own `README.md` before treating it as a finished template.

**C++ study** (runs in Phase 5 as a `*_sim` executable):

1. `case-study/<StudyName>/sim/include/{<study>_plant.h, controllers.h, simulation_runner.h}`
   and `sim/src/{<study>_plant.cpp, controllers.cpp, simulation_runner.cpp, main.cpp}`.
2. `case-study/<StudyName>/config/plant_params.json` + `config/scenarios/sNN_*.json`
   (loaded with `nlohmann::json`; every field must have a default so tests run JSON-free).
3. Per-study `CMakeLists.txt` defining the `<study>_sim` target (link `controller_toolbox`
   + `nlohmann_json::nlohmann_json`, `cxx_std_20`, `<STUDY>_SIM_SOURCE_DIR` compile definition).
4. `add_subdirectory("<StudyName>")` in `case-study/CMakeLists.txt`. **The folder name must
   be ASCII-only** (CMake on Windows fails on Unicode dashes in `add_subdirectory` paths).
5. Add `<study>_sim` to the explicit target list in `compile.bat` - a missing target
   silently runs a stale `.exe`.
6. `tests/test_<study>_regression.cpp` (convergence tests for 3-5 controllers + an
   all-controller smoke test) registered in `tests/CMakeLists.txt` with `catch_discover_tests`.
7. `README.md` in the study folder: reference, plant equations, parameter table,
   controller roster, scenarios, CSV columns. Keep it reconciled with the actual sim source.
8. `main.cpp` hard-codes its controller count (`N_CONTROLLERS` + `static_assert`) - bump it
   when adding a controller. Update the case-study tables in `CLAUDE.md` / root `README.md`
   (re-run `tools/case_study_tracker.py` to refresh `docs/case_study_status.md`).

**Python-only study** (runs in Phase 7): add `sim/main.py` following the Drill String
pattern; no CMake/compile.bat registration. `sim/` locates the bindings 4 levels up
(`build/bindings`).

---

## Sign Conventions

These conventions are tribal knowledge. They **must** be respected when implementing controllers and when writing test cases.

| Controller | compute() signal | Notes |
|---|---|---|
| `DiscretePID` | `e = r - y` (tracking error) | Positive error -> positive output |
| `DiscreteMPC` | `e = r - y` (scalar SISO) | Internal state estimate updated each call |
| `DiscreteLQR` | state vector `x` directly | Not an IController; use `LQRAdapter` for IController interface |
| `DiscreteSMC` | `e = y - r` (sign flipped!) | Sliding surface `s = c_e*(y-r) + ...`; see `DiscreteSMC.h` |
| `DiscreteADRC` | reference `r` handled internally | Call `computeTracking(y, r)` not `compute(e)` |
| `SmithPredictor` | `e = r - y` | Subtracts model correction internally (negative feedback) |
| `MRACController` | plant output `y_plant` | Call `setReference(r)` before each `compute(y_plant)` |
| `FeedbackLinearisationController` | `e = r - y` | Call `setState(x)` before each compute() |
| `NonlinearMPC` | `e = r - y` or set via `setReference()` | Call `setState(x)` before each compute() |
| `ResonantController` | `e = r - y` (tracking error) | Composes via `ControllerStack(Additive)`; one instance per target harmonic |
| `BacksteppingController` | `e = r - x1` (tracking error) | Call `setState(x)` before each compute(); `error` reconstructs the reference internally |
| `PassivityBasedController` | raw stacked state `[q; qdot]` (MIMO only) | `compute(double)` always throws - call `computeVec()`; call `setDesired(q_d)` once before use |
| `CLFController` | unused (`Other`) | Regulates toward V's equilibrium using `setState(x)`, not the scalar argument |
| `CascadeController` | `e_out = r_out - y_out` (outer error) | Call `setInnerMeasurement(y_in)` first. The **inner** error is `sp - y_in`, auto-flipped to `y_in - sp` when `inner->signConvention()` is `TrackingErrorYMinusR` |
| `DisturbanceObserverController` | `e = r - y` | Call `setPlantOutput(y)` before each compute(); without it `y ~= -error` is assumed |
| `TwoDOFController` | `e = r - y` | Call `setReference(r)` (and `setMeasuredDisturbance(d)`) before compute() - the feedforward callable reads them, not `error` |
| `LearningFeedforwardController` | mirrors the **nominal** controller | `signConvention()` delegates; the error recorded into ILC is negated when the nominal is `TrackingErrorYMinusR` (ILC assumes `r - y`) |
| `FuzzySlidingModeController` | `e = y - r` (sign flipped!) | Inherits `DiscreteSMC`'s convention; `fuzzy.e_scale`/`de_scale` normalise the **sliding surface** `s` and `s_dot`, not `e` |

---

## Architecture Pattern

**DiscreteLQR / LQRAdapter is the deliberate "stateless math + thin adapter" exception.**

`DiscreteLQR` is a pure algorithm class: it holds no `IController` state and can be shared across multiple adapters. `LQRAdapter` is a thin `IController` shim that wires a `DiscreteLQR` to state/reference callbacks.

Every other algorithm in `lib/` embeds both the math and the `IController` interface in a single class (`DiscretePID`, `DiscreteMPC`, etc.). This is intentional: for controllers with significant internal state (integrators, observers, covariance matrices), the overhead of separating "algorithm" from "interface" produces little benefit and adds indirection. The `DiscreteLQR` split exists specifically because the gain matrix `K*` is computed once at construction and shared across use sites-a genuinely stateless operation.

**When adding a new controller:**
- Follow the single-class pattern (implement `IController` directly).
- If the core computation is truly stateless (no internal memory), consider the `DiscreteLQR` split. Document the choice in the header.
- Do **not** add an `Adapter` class "for consistency"-it adds complexity without benefit.

---

## Numerical Safety Rules

1. **Matrix inverses:** Never call `.inverse()` directly. Use `.ldlt().solve()` and check `.info() != Eigen::Success`.
2. **Eigenvalue floor:** When forming a covariance or Gramian, floor eigenvalues to `1e-20` before taking square roots (pattern in `UnscentedKalmanFilter::update()`).
3. **DARE convergence:** Check `DareResult::converged` before using `DareResult::P`. The doubling algorithm in `DiscreteLQR::solveDARE()` is the reference pattern.
4. **Discretisation:** Default to ZOH (`ctrl::c2d(..., C2dMethod::ZOH)`). Document the choice in a comment when using Tustin or TustinPrewarped.
5. **Stability limit for backward-Euler ADRC:** `omega_o * Ts < 0.5` for stability. At `Ts = 1s`, `omega_o <= 0.5`.
6. **No warnings in Release:** All `std::cerr`/`std::clog` calls that fire during normal simulation must be wrapped in `#ifndef NDEBUG`. Use `lastQPConverged()` / `isHealthy()` in application code to detect non-convergence.
7. **NaN guard at every compute() boundary:** The first statement of every `compute()` override must be a non-finite check: `if (!std::isfinite(signal)) return u_prev_;` (or `return 0.0;` if no last-output state). Use `ctrl::sanitize(v, fallback)` (defined in `IController.h`) for inline substitution rather than early return. This is the library's contract - callers must not be required to pre-filter sensor readings.
8. **Euler integration in discrete dynamics callbacks:** If a `DiscreteDynamics` or `StateFunc` callback uses forward Euler at the control `Ts`, add a comment stating the dominant plant time constant and the `Ts` for which Euler accuracy holds. For stiff systems or `Ts > tau_min / 5`, use RK4 sub-steps internally.

---

## Documentation Standard

The gold standard is the DARE doubling derivation in `lib/DiscreteLQR.cpp:40-57` and the ESO backward-Euler derivation in `lib/DiscreteADRC.cpp:31-50`.

- **Return values in physical units** must state the unit in `@return`.  
  The canonical failure: `DiscreteLeadLag::phaseAt()` said "in degrees" when it returned radians (fixed in P12-21).
- **Tolerance choices in tests** must be justified in a comment; never "loose tolerance" without explanation.
- **No multi-paragraph docstrings.** One `@brief` line + `@code` example + key `@param`/`@return`/`@see` entries.
- **`@throws` is mandatory** for any constructor or method that throws. State the exception type and the exact condition:  
  ```cpp
  /// @throws std::invalid_argument If Ts <= 0 or Q is not positive-semidefinite.
  ```  
  If a method is `noexcept`, mark it explicitly (`noexcept` keyword + no `@throws`).
- **Discrete-time convention:** Always `@see` the canonical conventions table in `CONTRIBUTING.md#sign-conventions` rather than re-stating the sign rule per-class.
- Document in `docs/cumulative_bug_report.md` following the Part numbering convention. Next part: check the current highest part number and increment.

---

## Python Binding Conventions

Hard-won lessons from Parts 14-16 that must not be re-learned:

| Rule | Reason |
|---|---|
| `std::shared_ptr<T>` as 3rd `py::class_` arg for all `IController` subclasses | Without it, `ControllerStack.add_controller()` throws a "custom holder" `RuntimeError` |
| Capture `py::object` in lambdas, not `py::cpp_function` | `py::cpp_function` triggers `function_signature_t` overload-deduction errors in pybind11 v2.13 |
| `py::return_value_policy::copy` on all Eigen matrix accessors | Prevents dangling references to temporaries |
| `py::keep_alive<0, N>()` when C++ holds a raw reference to a Python object | E.g., `LQRAdapter` keeps `DiscreteLQR` alive |
| `py::object.cast<Eigen::VectorXd>()` for functor return values | Required for correct NumPy -> Eigen conversion |

**Known naming mismatches (Python vs C++):**

| Python name | C++ name |
|---|---|
| `ExtremumSeekerParams` | `ESCParams` |
| `GPCParams.Nu` | control horizon (not `Nc`) |
| `suggest_order` | **two overloads**: SubspaceID (`VectorXd`) in `advanced_bindings`, BalancedTruncation (`TruncationResult`) in `analysis_bindings` |
| `ControllerStack.add_controller` condition | `lambda(error, last_output) -> bool` (two args) |

---

## NumPy 2.x Compatibility

The conda environment uses NumPy 2.4.4. `float()` on a 1-D array raises `TypeError` in NumPy >= 2.0.

**Always use** `float(np.squeeze(arr))` or `float(arr[0])` for 1-D results, especially after:
- Matrix-vector products: `-K @ x` gives shape `(m,)` not scalar.
- `ctrl.*` functions returning Eigen vectors as NumPy arrays.

---

## PR Checklist

Before marking a PR ready for review, confirm all of the following:

- [ ] `conda run -n soft_robotics -- python run.py` passes with **0 failures** in Phases 3-6
- [ ] New controller has at least one Catch2 test that checks a numeric value (not just "no crash")
- [ ] Sign convention is correct and documented in a test comment
- [ ] `lastOutput()` is **not** marked `override` (not virtual in `IController`)
- [ ] No `std::cerr`/`std::clog` calls outside `#ifndef NDEBUG` guards in library code
- [ ] `lib/CMakeLists.txt` has the new `.cpp` in `CTRL_CORE_SOURCES`
- [ ] `lib/ControllerToolbox.h` has the new `#include`
- [ ] `lib/Features.h` has the new feature flag
- [ ] `examples/CMakeLists.txt` has `add_example(exNN_name)`
- [ ] `compile.bat` **and** `compile.sh` have the example target in the sequential build list
- [ ] Python binding uses `std::shared_ptr<T>` as third `py::class_` template argument
- [ ] `bindings/smoke_test.py` has an assertion for the new class
- [ ] All `float(numpy_array)` calls use `.squeeze()` or `[0]` indexing
- [ ] `docs/cumulative_bug_report.md` has a new Part section documenting the change
- [ ] If adding a new case study C++ target: registered in `case-study/CMakeLists.txt`, `compile.bat`, and `compile.sh`
