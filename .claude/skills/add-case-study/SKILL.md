---
name: add-case-study
description: Scaffold and register a new physics case study (C++ or Python-only) following this repo's workflow -- tools/new_case_study.py, CMake/compile.bat registration, regression tests, README, and status-tracker refresh. Use when asked to add a new case study.
---

# Add Case Study

Use this skill whenever asked to add a new case study under `case-study/`. Mirrors
`CONTRIBUTING.md`'s "Adding a New Case Study" section plus the case-study pattern in
`docs/case_study_copilot_reference.md`. Track steps with TodoWrite.

## Before starting

- Decide C++ (built target, runs in `run.py` Phase 4) or Python-only (auto-discovered, runs in
  Phase 6). Python-only is far less registration work - default to it unless there's a
  specific reason (e.g. real-time/WCET profiling) to need a compiled C++ target.
- Have the source paper/spec ready; `tools/new_case_study.py` takes a PDF path and scaffolds
  from it.
- Use the newest C++ studies (S-OTEC, Solar Cooker, SMISMO) or the Drill String study
  (Python-only) as structural reference if unsure what "done" looks like.

## Step 1 - Scaffold

```
python tools/new_case_study.py case-study/MyPaper.pdf --lang cpp --name "My Study"
python tools/new_case_study.py case-study/MyPaper.pdf --lang python
```

This generates `CMakeLists.txt`/`README.md`/`config/plant_params.json`/
`config/scenarios/s0N_*.json` and the `sim/` skeleton. **Don't hand-roll this boilerplate.**
The scaffold's placeholder plant (`x' = -a*x + b*u`) and `OpenLoop` controller actually run and
produce real-looking CSVs before anything real is implemented - `tools/case_study_tracker.py`
reports this as "Open placeholder" until the body is replaced. Don't mistake CSV output for
real progress.

## Step 2 - Fill in the real content

- Plant ODE in `*_plant.{h,cpp}` / `*_plant.py`.
- Controller roster in `controllers.{h,cpp}` / `controllers.py`, picked from `lib/` per the
  sign-convention table in `CONTRIBUTING.md#sign-conventions`. Do not edit `lib/*` itself -
  that's a different, heavier workflow (use the `add-controller` skill if a genuinely new
  algorithm is needed).
- Tune gains. Hand-tuned controllers tune inline in the controllers file; GA/PSO/DE-optimized
  ones run their cost function once in the constructor instead of reading gains from JSON.

## Step 3 - Registration (C++ studies only - Python-only studies skip to Step 4)

- `case-study/<StudyName>/CMakeLists.txt` defining the `<study>_sim` target (link
  `controller_toolbox` + `nlohmann_json::nlohmann_json`, `cxx_std_20`).
- `add_subdirectory("<StudyName>")` in `case-study/CMakeLists.txt`. **Folder name must be
  ASCII-only** - CMake on Windows fails on Unicode dashes in `add_subdirectory` paths.
- Add `<study>_sim` to the explicit target list in **both** `compile.bat` and `compile.sh` - a
  missing target means `run.py` silently runs a stale `.exe` instead of failing loudly.
- `tests/test_<study>_regression.cpp` (convergence tests for 3-5 controllers + an
  all-controller smoke test), registered in `tests/CMakeLists.txt` via `catch_discover_tests`.
- `main.cpp` hard-codes `N_CONTROLLERS` + a `static_assert` - keep it in sync with the roster.

## Step 4 - Documentation and tracking

- `README.md` in the study folder: reference/citation, plant equations, parameter table,
  controller roster, scenarios, CSV column documentation. Keep it reconciled with the actual
  sim source, not aspirational.
- Update the case-study tables in root `README.md` (and `CLAUDE.md` if it has a Case Studies
  section).
- Run `python tools/case_study_tracker.py` to refresh `docs/case_study_status.md` - never
  hand-edit that file.

## Step 5 - Optional analysis hooks

If the study should support Monte Carlo / fault-sweep / WCET / mu-analysis
(`tools/monte_carlo.py` etc.), implement the hook contract from `tools/study_protocol.py`:
`run_single(ctrl_name, params=None, scenario_id=None) -> dict` and
`run_with_fault(ctrl_name, fault, scenario_id=None) -> dict`, both returning a metrics dict
(at least `iae`, `name`, `scenario_id`). CSV output must use the `run_` filename prefix for
auto-discovery.

## Verify

- C++: build via `compile.bat`/`compile.sh`, then run the `*_sim` executable and the new
  regression test (`ctest --test-dir build -R test_<study>_regression`).
- Python-only: `conda run -n soft_robotics -- python "case-study/<Study>/sim/main.py"`.
- Full check: `conda run -n soft_robotics -- python run.py` - confirms Phase 4/6 discovery
  actually picks up the new study and `docs/case_study_status.md` reflects it correctly (not
  "Open placeholder").
