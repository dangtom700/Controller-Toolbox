# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **Note on this file:** `CLAUDE.md` is gitignored (local-only, never committed). A prior
> session maintained an extensive version of this file with accumulated "tribal knowledge"
> (a session-start checklist, sign-convention table, per-controller gotchas, Part-by-part
> history) that several committed docs (`README.md`, `CONTRIBUTING.md`,
> `docs/case_study_copilot_reference.md`) still reference as "see CLAUDE.md" - but that
> content was never committed to git and is not recoverable on a fresh clone. This rewrite
> reconstructs the durable, verifiable parts from committed sources. The durable long-form
> history lives in `docs/cumulative_bug_report.md` (Part-numbered, currently up to Part 66);
> a synthesized, committed set of cross-cutting caveats (verified against current source, not
> just copied from historical prose) lives in `docs/handoff.md` - **read that file too**, since
> it captures exactly the kind of tribal knowledge that keeps getting lost when this file
> doesn't survive a clone.

## Commands

### One-time environment setup

- Windows: `.\setup.ps1` (requires PowerShell 7). Validates the MSYS2 **UCRT64** toolchain
  (`C:\msys64\ucrt64\bin` - not `mingw64`; the project switched from MinGW64 to UCRT64),
  creates the `soft_robotics` conda env from `environment.yml`, builds the Python bindings,
  and runs the smoke test. Add `-FullBuild` to also build every C++ target. Add
  `-SkipCondaCreate` to skip env creation on re-runs.
- Linux/macOS: `./setup.sh`.
- MinGW/UCRT64 builds **statically link** `libgcc`/`libstdc++`/`libwinpthread`
  (`if(MINGW) add_link_options(-static-libgcc -static-libstdc++ -static)` in root
  `CMakeLists.txt`) so every executable and `ctrl_toolbox*.pyd` runs without MSYS2's
  `bin/` on `PATH`. Python's loader for `.pyd` extensions ignores `PATH` for dependent DLLs
  since 3.8 - without static linking, `import ctrl_toolbox` fails outside an MSYS2 shell
  even when the build itself succeeds.

### Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build          # avoid --parallel locally (see note below); fine in CI
ctest --test-dir build --output-on-failure
```

With Python bindings: add `-DCTRL_BUILD_PYTHON_BINDINGS=ON` to the configure step, then
`cmake --build build --target ctrl_toolbox`, then `conda run -n soft_robotics -- python bindings/smoke_test.py`.

On Windows, `compile.bat` (or `compile.sh` on Linux/macOS) builds every C++ target
(examples, tests, case studies) **sequentially in an explicit, hand-maintained list** - a
target not listed there silently never builds, and a stale `.exe` from a previous build can
mask that. The two files must be kept in sync by hand (verified in sync as of this writing,
but nothing automated catches future drift between them). Sequential building here is to
avoid pegging every core on a dev machine for ~140 targets, not a build-correctness
requirement - CI builds the same targets with `--parallel` deliberately, and that's fine.

### Full verification (the canonical "is everything still passing" command)

```bash
conda run -n soft_robotics -- python run.py
```

This is the only command that exercises all 7 phases: ASCII-only source scan, sequential
C++ compile, Python binding build + smoke test, run every built `.exe`, run every
`examples/python/exNN_*.py`, discover and run every Python-only case study's `sim/main.py`,
then regenerate `docs/case_study_status.md` and `docs/report.html`. It writes
`run_YYYYMMDD_HHMMSS.log` to the repo root and, if any failures are detected, `bug_report.txt`
(absence of `bug_report.txt` after a run means a clean pass). **Don't claim something is
"verified" or "passing" without having just run this** - stale pass counts in README/CONTRIBUTING
are explicitly marked UNVERIFIED until the next clean run.

### Running a single test / example

- Single Catch2 suite: `ctest --test-dir build -R test_catch2_advanced --output-on-failure`
  (each `add_executable` in `tests/CMakeLists.txt` is registered via `catch_discover_tests`,
  so `ctest -R <name>` or `-R [tag]` filters by executable or Catch2 tag).
- Or run the built test executable directly with a Catch2 tag filter, e.g.
  `build/tests/test_catch2_advanced.exe [smc]`.
- Single C++ example: build then run `build/examples/exNN_name.exe` directly - it prints
  `PASS`/`FAIL` and sets exit code accordingly.
- Single Python example: `conda run -n soft_robotics -- python examples/python/exNN_name.py`.
- Single Python-only case study: `conda run -n soft_robotics -- python "case-study/<Study>/sim/main.py"`.

## Architecture

**`lib/` is flat** - no `lib/control/`/`lib/estimation/` subdirectories; every class is
`lib/ClassName.{h,cpp}` (145 files, ~90 controller/estimator/identification implementations).
Two real subdirectories: `lib/embedded/` (header-only, no-Eigen, MCU-safe subset -
`BasicPID`, `BasicSMC`, `DiscreteIntegrator`, `FixedRateFilter`, `RingBuffer`) and `lib/hal/`
(scheduler/sensor/actuator interfaces for deployment). `lib/Features.h` is a build-time
feature-flag registry consumed by `ctrl_toolbox.features()` in Python.

**`IController` (`lib/IController.h`) is the base interface**: `compute(double)`, `reset()`,
`sampleTime()`. Almost every controller implements algorithm + `IController` interface in one
class. The one deliberate exception is `DiscreteLQR`/`LQRAdapter`: `DiscreteLQR` is pure
stateless math (a gain matrix computed once, shareable across adapters); `LQRAdapter` is a
thin `IController` shim wiring it to state/reference callbacks. This split exists *because*
`DiscreteLQR` is genuinely stateless - don't introduce an `Adapter` class for other
controllers "for consistency" (see `CONTRIBUTING.md#architecture-pattern`).
Composition wrappers (`ControllerStack`, `AntiWindupWrapper`, `ComputationalDelayWrapper`,
`GainScheduledController`, `SmithPredictor`/`AdaptiveSmithPredictor`) wrap an `IController`
rather than subclassing it.

Sign conventions are **not uniform across controllers** (e.g. `DiscreteSMC.compute()` takes
`e = y - r`, the reverse of `DiscretePID`'s `e = r - y`; `MRACController.compute()` takes the
plant output directly, not an error) - see the full table in `CONTRIBUTING.md#sign-conventions`
before wiring up a new controller in a case study.

**`bindings/`** exposes one flat pybind11 module, `ctrl_toolbox` (no submodules).
`bindings/module.cpp` is the `PYBIND11_MODULE` entry point dispatching to
`plantmodel_bindings.cpp`, `controllers_bindings.cpp`, `estimation_bindings.cpp`,
`advanced_bindings.cpp`, `analysis_bindings.cpp`. Binding new `IController` subclasses
requires `std::shared_ptr<T>` as the third `py::class_` template argument or
`ControllerStack.add_controller()` throws at runtime - see `CONTRIBUTING.md#python-binding-conventions`
for this and other hard-won pybind11 v2.13 + NumPy 2.x rules.

**`case-study/<Study>/`** studies all share one shape regardless of language: instantiate
plant -> instantiate controller roster -> for each scenario, reset, step (error -> `compute()`
-> `plant.step(u)` -> accumulate IAE -> write a CSV row) -> next controller -> next scenario.
C++ studies have their own `CMakeLists.txt` (registered in `case-study/CMakeLists.txt` *and*
listed explicitly in `compile.bat`/`compile.sh`, or the target silently doesn't build) and are
run by `run.py` Phase 4 as a built `*_sim` executable. Python-only studies have just
`sim/main.py` (no CMake registration), are auto-discovered by `run.py` Phase 6, and locate the
bindings build 4 directories up. Scaffold new studies with `tools/new_case_study.py` rather
than hand-rolling boilerplate - its placeholder plant actually runs and produces real-looking
CSVs before any real implementation exists, so use `tools/case_study_tracker.py`'s "Open
placeholder" status (not the presence of log files) to tell whether a study has real content.
Per-study optional analysis hooks (`run_single`, `run_with_fault`, `grey_box_model`) consumed
by `tools/monte_carlo.py`/`tools/fault_sweep.py`/`tools/wcet_report.py` are documented in
`tools/study_protocol.py`.

**`tools/`** is a post-hoc analysis pipeline (not simulation-time code): `metrics.py`
(shared IAE/RMS/settle-time/overshoot computation), `compare_controllers.py`, `monte_carlo.py`,
`fault_injector.py`/`fault_sweep.py`, `anova.py`, `wcet_report.py`, `mu_analysis.py`,
`generate_report.py` (writes `docs/report.html`, degrades gracefully without Plotly), and
`case_study_tracker.py` (regenerates `docs/case_study_status.md` - never hand-edit that file).

For the full API surface and per-class constructor/sign-convention quick reference when
writing new case-study code, see `docs/case_study_copilot_reference.md` (condensed,
verified-only API map) and `docs/DOCUMENTATION.md` (full reference). For conventions,
checklists, and the numerical-safety/documentation rules, see `CONTRIBUTING.md` - it is the
durable source of record for everything this file would otherwise duplicate.

**Scope discipline:** treat `lib/*.{h,cpp}` as a stable public API to *consume* from
case-study work, not edit. A genuinely new controller class goes through the full
checklist in `CONTRIBUTING.md#adding-a-new-controller` (implementation + build wiring +
examples + Catch2 tests + Python bindings + smoke test) - that's a different, heavier review
bar than case-study work, not a quick edit.
