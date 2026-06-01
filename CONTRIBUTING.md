# Contributing to Controller Toolbox

This document covers the conventions, workflows, and checklists for adding to or modifying the Controller Toolbox. Read it fully before opening a pull request.

---

## Table of Contents

1. [Build and Test Workflow](#build-and-test-workflow)
2. [Adding a New Controller](#adding-a-new-controller)
3. [Sign Conventions](#sign-conventions)
4. [Numerical Safety Rules](#numerical-safety-rules)
5. [Documentation Standard](#documentation-standard)
6. [Python Binding Conventions](#python-binding-conventions)
7. [NumPy 2.x Compatibility](#numpy-2x-compatibility)
8. [PR Checklist](#pr-checklist)

---

## Build and Test Workflow

The project uses a four-phase runner managed by `run.py`:

| Phase | What happens |
|---|---|
| 1 - Non-ASCII scan | Auto-replaces known non-standard Unicode characters in all `.cpp`, `.h`, `.py`, `.txt` files. Write ASCII-only source. |
| 2 - Compile | Runs `compile.bat` which calls `cmake --build` **sequentially** (no `--parallel`). Targets are listed explicitly in dependency order. |
| 3 - Run C++ | Runs every `.exe` under `build/`, prints pass/fail. |
| 4 - Run Python | Runs every `exNN_*.py` under `examples/python/` via the `soft_robotics` conda environment. |

**Always invoke via:**
```
conda run -n soft_robotics -- python run.py
```

**Never use** `cmake --build build --parallel` - sequential compilation per `compile.bat` is required.

**Expected baseline (as of Part 26, 2026-05-31):** `C++ 90/90 passed | Python 88/88 passed`,
plus the four case studies reporting full run counts (Boiler 216, SMISMO 42, Solar 45, Tug 64).
This number drifts every part - treat the latest `run_*.log` as the source of truth, not this line.

A log file `run_YYYYMMDD_HHMMSS.log` is written to the project root after every run.

---

## Adding a New Controller

Follow these steps in order. Check each off before declaring the work done.

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

---

## Numerical Safety Rules

1. **Matrix inverses:** Never call `.inverse()` directly. Use `.ldlt().solve()` and check `.info() != Eigen::Success`.
2. **Eigenvalue floor:** When forming a covariance or Gramian, floor eigenvalues to `1e-20` before taking square roots (pattern in `UnscentedKalmanFilter::update()`).
3. **DARE convergence:** Check `DareResult::converged` before using `DareResult::P`. The doubling algorithm in `DiscreteLQR::solveDARE()` is the reference pattern.
4. **Discretisation:** Default to ZOH (`ctrl::c2d(..., C2dMethod::ZOH)`). Document the choice in a comment when using Tustin or TustinPrewarped.
5. **Stability limit for backward-Euler ADRC:** `omega_o * Ts < 0.5` for stability. At `Ts = 1s`, `omega_o <= 0.5`.
6. **No warnings in Release:** All `std::cerr`/`std::clog` calls that fire during normal simulation must be wrapped in `#ifndef NDEBUG`. Use `lastQPConverged()` / `isHealthy()` in application code to detect non-convergence.

---

## Documentation Standard

The gold standard is the DARE doubling derivation in `lib/DiscreteLQR.cpp:40-57` and the ESO backward-Euler derivation in `lib/DiscreteADRC.cpp:31-50`.

- **Return values in physical units** must state the unit in `@return`.  
  The canonical failure: `DiscreteLeadLag::phaseAt()` said "in degrees" when it returned radians (fixed in P12-21).
- **Tolerance choices in tests** must be justified in a comment; never "loose tolerance" without explanation.
- **No multi-paragraph docstrings.** One `@brief` line + `@code` example + key `@param`/`@return`/`@see` entries.
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

- [ ] `conda run -n soft_robotics -- python run.py` passes with **0 failures** in Phase 3 and Phase 4
- [ ] New controller has at least one Catch2 test that checks a numeric value (not just "no crash")
- [ ] Sign convention is correct and documented in a test comment
- [ ] `lastOutput()` is **not** marked `override` (not virtual in `IController`)
- [ ] No `std::cerr`/`std::clog` calls outside `#ifndef NDEBUG` guards in library code
- [ ] `lib/CMakeLists.txt` has the new `.cpp` in `CTRL_CORE_SOURCES`
- [ ] `lib/ControllerToolbox.h` has the new `#include`
- [ ] `lib/Features.h` has the new feature flag
- [ ] `examples/CMakeLists.txt` has `add_example(exNN_name)`
- [ ] `compile.bat` has the example target in the sequential build list
- [ ] Python binding uses `std::shared_ptr<T>` as third `py::class_` template argument
- [ ] `bindings/smoke_test.py` has an assertion for the new class
- [ ] All `float(numpy_array)` calls use `.squeeze()` or `[0]` indexing
- [ ] `docs/cumulative_bug_report.md` has a new Part section documenting the change
