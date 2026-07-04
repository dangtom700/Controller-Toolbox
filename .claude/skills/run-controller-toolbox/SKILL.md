---
name: run-controller-toolbox
description: Build the C++20/Eigen core + ctrl_toolbox Python bindings and drive Controller-Toolbox by running the bindings smoke test, a compiled example .exe, a Python example, or a Catch2 test tag. Use when asked to build, run, smoke-test, or verify this control library works after a change to lib/, bindings/, or examples/.
---

# Run Controller-Toolbox

There is no server or GUI here - this is a C++20/Eigen control library exposed through one flat
`pybind11` module (`ctrl_toolbox`), plus ~130 compiled C++ example/test executables and ~100
Python examples. "Running" it means: build the piece you touched, then drive it directly.

**Primary harness (the one to reach for first):** `bindings/smoke_test.py` - a ~1900-line,
already-committed script that imports `ctrl_toolbox` and exercises essentially every controller,
estimator, and identification class end-to-end (construct -> `compute()`/`step()` -> assert
finite/shape/converged). It is this repo's real interaction harness for the Python surface; don't
write a new one; run it. For a C++-only change, the equivalent is running the specific example
`.exe` or Catch2 test tag that covers it (all `examples/exNN_*.cpp` print a `PASS`/`FAIL`-style
summary; Catch2 suites accept `[tag]` filters).

All paths below are relative to the repo root.

## Prerequisites (verified present in this environment)

- MSYS2 **UCRT64** toolchain at `C:\msys64\ucrt64\bin` (g++, cmake). Confirmed:
  ```bash
  cmake --version   # -> cmake version 4.3.4, found at /c/msys64/ucrt64/bin/cmake
  ```
- Conda env `soft_robotics` (Python 3.12, numpy/pandas via conda, rest via `requirements.txt` -
  see `environment.yml`). Confirmed:
  ```bash
  conda run -n soft_robotics python -c "import sys; print(sys.version)"
  # -> 3.12.13 | packaged by Anaconda, Inc. ...
  ```
- A fresh machine bootstraps both via `.\setup.ps1` (PowerShell 7) - not run this session since
  this environment already had them; see the script if starting from scratch.

## Build

This repo already had a `build/` directory configured. Full canonical (re)configure:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTRL_BUILD_PYTHON_BINDINGS=ON
```

Then build only the target you need - **do not** run the full `compile.bat`/`compile.sh` (~140
targets, intentionally sequential/single-threaded so it doesn't peg every core on a dev box) unless
you actually need everything rebuilt:

```bash
# Python bindings (core lib + pybind11 module) - this is what most lib/ changes need:
cmake --build build --target ctrl_toolbox

# A single C++ example, after editing lib/ or the example itself:
cmake --build build --target ex01_tf_pid

# A single Catch2 test binary:
cmake --build build --target test_catch2_advanced
```

Building `ctrl_toolbox`/`controller_toolbox` from a state where the core lib needs recompiling
took **~15-20 minutes** single-threaded in this environment (it recompiles all ~145 `lib/*.cpp`
translation units). Run it in the background and check back rather than blocking on it.

## Run (agent path)

**1. Python bindings smoke test** - the main "does the library still work" check:

```bash
conda run -n soft_robotics -- python bindings/smoke_test.py
```
Verified this session - runs every bound class through a real construct/compute/assert cycle and
ends with:
```
All smoke tests passed.
```
A failure here means either the bindings didn't rebuild after a `lib/` change, or a real
regression - the assert message names the exact class/behavior that broke.

**2. A single compiled C++ example** (after `cmake --build build --target <exNN_name>`):

```bash
./build/examples/ex01_tf_pid.exe
```
Prints a numeric trace and a verification block ending `[PASS]`/`[FAIL]` per check.

**3. A single Python example** (no build step - pulls the already-built `.pyd` from `build/`):

```bash
conda run -n soft_robotics -- python examples/python/ex01_plant_construction.py
```
Ends with `Verification: N/N checks passed` and a per-check `[PASS]`/`[FAIL]` list.

**4. A Catch2 suite, filtered to one tag** (after building that test target):

```bash
./build/tests/test_catch2_advanced.exe --list-tags        # discover available [tag]s
./build/tests/test_catch2_advanced.exe "[smc]"             # run only that tag
```
Ends with `All tests passed (N assertions in M test cases)`.

## Test (full gate)

The canonical "everything passing" run is `run.py` (8 phases: 1 Non-ASCII scan -> 2 NaN-guard
scan -> 3 `compile.bat`/`.sh` -> 4 bindings build+smoke -> 5 run every `.exe` -> 6 every Python
binding example -> 7 Python case studies -> 8 case-study status + static-report regen):

```bash
conda run -n soft_robotics -- python run.py
```

This is long-running (compiles ~140 targets sequentially, then runs ~200 executables + ~100
Python scripts + case studies) - do not block on it interactively; start it in the background and
let it finish, then read the generated `run_YYYYMMDD_HHMMSS.log` / `bug_report.txt` (only written
on failure). Don't claim "passing" without having actually seen this complete clean.

The fast, cheap sub-check worth running on its own after touching a `compute()` **or**
`computeVec()` body is the NaN-guard static scan (Phase 2, seconds not minutes). It enforces two
rules: every scalar `compute(double)` must *lead* with a non-finite guard, and every out-of-line
`computeVec(const VectorXd&)` must guard (`allFinite()`) **before** it commits to an output (its
first `.noalias()`) - the scalar guard does not cover the MIMO path:

```bash
conda run -n soft_robotics python tools/check_nan_guard.py
# -> OK: every IController::compute(double) leads with a non-finite guard, and
#       every computeVec() guards before committing to an output.
```

---

## Gotchas

- **`ctest -R <exe-target-name>` finds nothing.** CTest here discovers individual Catch2
  `TEST_CASE`s via `catch_discover_tests()`, not one test per executable - `ctest -N` lists
  entries like `LQRAdapter computeVec returns full control vector`, not `test_catch2_advanced`.
  `ctest --test-dir build -R test_catch2_advanced` (as CLAUDE.md's quick-reference shows) matches
  zero tests. To run a whole suite or one tag, invoke the `.exe` directly with a Catch2 filter:
  `build/tests/test_catch2_advanced.exe "[smc]"`.
- **An inherited `build/` directory may be configured `Debug`, not `Release`.** Check with
  `grep CMAKE_BUILD_TYPE build/CMakeCache.txt`. This repo's canonical config is
  `-DCMAKE_BUILD_TYPE=Release` (`-O3 -DNDEBUG`, no `-g`). A `Debug` config compiles with `-g` and
  no optimization, and rebuilding a heavy Eigen-template translation unit (e.g. `lib/DiscreteHinf.cpp`)
  under `-g` overflows MinGW's assembler COFF section table:
  ```
  as.exe: DiscreteHinf.cpp.obj: too many sections (80842)
  Fatal error: can't write 30 bytes to section .text of DiscreteHinf.cpp.obj: 'file too big'
  ```
  with **no other diagnostic printed** - it looks like a silent/mysterious build failure. The fix
  is not a source change: reconfigure with `-DCMAKE_BUILD_TYPE=Release` (verified in this session -
  the identical file compiles clean under `-O3 -DNDEBUG`, no `-g`).
- **`bug_report.txt` can be a false positive on a fully-green run.** `run.py`'s report generator
  keyword-scans the log for failure words - and one of them is `nan`, which legitimately appears
  in the *passing* Phase 2 banner (`Phase 2 - NaN-guard scan` / `NaN-guard scan PASSED`). So a
  clean run can still emit `bug_report.txt` with `FAILURE BLOCK #1 (keywords: nan)` pointing at
  the NaN-guard phase. Before treating it as real, open the block: if the only keyword is `nan`
  and the surrounding lines say `PASSED`, it is spurious. The authoritative pass signal is the
  log itself - every program prints `EXIT 0 - PASSED` and there are no `[FAIL`, `Traceback`, or
  `ERROR` markers:
  ```bash
  LOG=$(ls -t run_*.log | head -1)
  grep -cE "EXIT 0 . PASSED" "$LOG"          # count of passing programs
  grep -nE "\[FAIL|Traceback|\bERROR\b" "$LOG" | grep -viE "nan-guard|NaN-guard"
  ```
  (A single `[FAIL >NN%]` line from an example's internal metric annotation - e.g. relay-tuning
  `Ku` fidelity on an over-damped plant - is a soft grade, not a run failure, if that program
  still ends `EXIT 0 - PASSED`.)
