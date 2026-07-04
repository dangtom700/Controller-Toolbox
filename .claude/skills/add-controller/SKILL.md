---
name: add-controller
description: Add a new controller/estimator class to lib/ following this repo's full workflow (implementation, build wiring, examples, Catch2 tests, Python bindings, smoke test, PR checklist). Use when asked to add, implement, or wire in a new IController-derived class.
---

# Add Controller

Use this skill whenever asked to add a new controller, estimator, or identification class to
`lib/`. It mirrors the exact workflow in `CONTRIBUTING.md` ("Adding a New Controller") plus its
PR checklist - both are the durable source of truth; this skill is a guided checklist, not a
replacement. Track each step with TodoWrite: this is a multi-step task and steps are easy to
silently skip (the project's own PR checklist exists because of that).

## Before starting

- Confirm: class name, the controller family/tag it belongs to (for the Catch2 `[tag]`), and
  whether it's a SISO `IController`-style class or a MIMO/stateless case like `DiscreteLQR`.
  Read `CONTRIBUTING.md#architecture-pattern` before deciding - don't add an `Adapter` class
  "for consistency"; that split only exists because `DiscreteLQR`'s core computation is
  genuinely stateless.
- Check the sign-convention table in `CONTRIBUTING.md#sign-conventions` for the controller
  family and decide upfront which convention applies (`e = r - y` vs `e = y - r` vs raw
  state/plant-output) - getting this wrong is the most common review-cycle cause here.

## Step 1 - Implement the class

- `lib/ClassName.h` (+ `lib/ClassName.cpp` if needed). Subclass `IController`, override
  `compute(double)`, `reset()`, `sampleTime()`.
- Call `notifyObserver(u, signal)` at the end of every `compute()`; `notifyObserverReset()` at
  the end of every `reset()`.
- Guard every matrix inverse: `.ldlt().solve()` + check `.info() != Eigen::Success`. Never call
  `.inverse()` directly.
- First statement of every `compute()` must be a non-finite guard
  (`if (!std::isfinite(signal)) return ...;` or `ctrl::sanitize(v, fallback)`).
- If the class overrides `computeVec(const VectorXd&)` (any MIMO controller/estimator), that
  path needs its **own** guard - `if (!v.allFinite()) return <last output>;` **before** it
  advances any internal state - or one non-finite sample permanently poisons the state vector.
  The scalar guard does not cover it, and `tools/check_nan_guard.py` now scans `computeVec`
  bodies too (the `allFinite()` guard must precede the first `.noalias()`).
- No `std::cerr`/`std::clog` outside `#ifndef NDEBUG`.
- Document per `CONTRIBUTING.md#documentation-standard`: one `@brief` line, `@param`/`@return`
  with physical units stated, mandatory `@throws` for anything that throws (or explicit
  `noexcept`).

## Step 2 - Wire into the build

- `lib/CMakeLists.txt`: add the `.cpp` to `CTRL_CORE_SOURCES`.
- `lib/ControllerToolbox.h`: add `#include "ClassName.h"` with a one-line `///<` docstring.
- `lib/Features.h`: add `{"feature_name", true}`.
- `compile.bat`: add the example target name to the sequential `for %%T in (...)` loop. Check
  whether `compile.sh` needs the equivalent addition.

## Step 3 - Examples

- C++: `examples/exNN_classname.cpp` (`#include "ControllerToolbox.h"`, prints `PASS`/`FAIL`,
  exit code 1 on failure). Add `add_example(exNN_classname)` to `examples/CMakeLists.txt`.
- Python: `examples/python/exNN_classname.py` (starts with `import _setup_bindings`, ends with
  `print("PASS")`) - auto-discovered by `run.py`, no registration needed.

## Step 4 - Catch2 tests

- Add to `tests/test_catch2_advanced.cpp` with a `[tag]` matching the controller family.
- Assert actual numeric values, not "no crash". Justify any tolerance choice in a comment.
- Cross-validate against scipy/python-control reference values embedded in a comment when
  checking a mathematical result.

## Step 5 - Python bindings

- Add to the matching `bindings/*_bindings.cpp` file.
- `IController` subclasses **must** use `std::shared_ptr<T>` as the 3rd `py::class_` template
  argument - otherwise `ControllerStack.add_controller()` throws a "custom holder"
  `RuntimeError` at runtime, not at compile time.
- Capture `py::object` in `std::function` lambdas, not `py::cpp_function`.
- Add an assertion for the new class to `bindings/smoke_test.py`.

## Verify

- Build: `cmake --build build` (no `--parallel`), then the `ctrl_toolbox` binding target, then
  run `conda run -n soft_robotics -- python bindings/smoke_test.py`.
- Run the new Catch2 test by filtering the built executable directly:
  `build/tests/test_catch2_advanced.exe "[tag]"`. **Do not** use `ctest -R test_catch2_advanced` -
  it matches zero tests (verified). `catch_discover_tests` registers each `TEST_CASE` under its
  full sentence *name*, not the executable/target name, so `ctest -R "<name substring>"` works but
  the target name never does.
- Walk the 17-item PR checklist in `CONTRIBUTING.md#pr-checklist` before calling it done - most
  items are exactly Steps 2-5 above restated as checkboxes; use it as the final pass, not a
  substitute for actually doing the steps.
- Add a new Part section to `docs/cumulative_bug_report.md` (check the current highest
  `## Part N` and increment).
