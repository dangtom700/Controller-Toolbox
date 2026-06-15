# Controller Toolbox - Phase 3: Deployment & Frontend Roadmap

**Created:** 2026-06-14 (Post-Part 56 strategic planning)
**Status:** DIST-1..5 complete (Parts 57E, 60). ANA-1..7, RPT-1 complete (Part 58). PLT-1/TRK-1 complete (Part 59).
**Scope:** Library distribution (DIST-1..5) . Analysis pipeline (ANA-1..7) . Static report (RPT-1)
**Not in scope:** Digital Twin / Streamlit (D2 remains open at LOW priority), TCLab HIL.

---

## Context

The algorithm set (A1-A11, E1-E4, H1-H4, D1, GA/PSO/DE) and all 16 case studies are mature. The
next investment cycle targets two distinct areas:

1. **Library distribution** - making the toolbox easier to consume (vcpkg, embedded subset, ROS2).
2. **Analysis depth + static reporting** - converting raw IAE/ISE CSVs into a credible,
   multi-dimensional controller comparison with statistical backing and structured uncertainty bounds.

These two tracks are **fully independent** and can be executed in parallel or in any order.

---

## Dependency Graph

```
DIST-1 (vcpkg)         --- independent
DIST-2 (embedded)      --- independent (builds on lib/BasicPID.h, lib/BasicSMC.h from Part 54)
DIST-3 (ROS2)          --- independent
DIST-4 (PyPI wheels)   --- requires DIST-1 (CMake install targets needed for wheel build)
DIST-5 (release.yml)   --- requires DIST-1 (install targets needed for artifact collection)

ANA-1 (metrics)        --- prerequisite for ANA-2, ANA-3, ANA-5
ANA-2 (Monte Carlo)    --- requires ANA-1; ANA-4 requires ANA-2 data
ANA-3 (fault sweep)    --- requires ANA-1; independent of ANA-2
ANA-4 (ANOVA)          --- requires ANA-2 mc_summary.csv
ANA-5 (WCET)           --- requires ANA-1 (timing hooks in runners); independent of ANA-2/3
ANA-6 (model val.)     --- independent; requires E1/E2 (done Part 52)
ANA-7 (mu-analysis)     --- independent; uses existing StateSpace models from lib/

RPT-1 (static report)  --- requires ANA-1..7 (or stubs for unfinished sections)
```

**Recommended order within ANA track:** ANA-1 -> (ANA-2 ∥ ANA-3 ∥ ANA-5 ∥ ANA-6 ∥ ANA-7) -> ANA-4 -> RPT-1

---

## Track 1: Library Distribution

### DIST-1 - vcpkg port + CMake install targets ✓ Done (Part 57E)

**Effort:** 2-3 person-days  
**Rationale:** Largest friction point for new C++ users is the build. vcpkg + `find_package` eliminates it.

**Step-by-step:**

1. Add `install()` targets to root `CMakeLists.txt` (after the existing `add_library` block):
   ```cmake
   include(GNUInstallDirs)
   install(TARGETS ctrl_toolbox
           EXPORT ctrl_toolboxTargets
           ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
           LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
   install(FILES lib/ControllerToolbox.h
           DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ctrl_toolbox)
   install(DIRECTORY lib/
           DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ctrl_toolbox
           FILES_MATCHING PATTERN "*.h")
   install(EXPORT ctrl_toolboxTargets
           FILE ctrl_toolboxConfig.cmake
           NAMESPACE ctrl::
           DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ctrl_toolbox)
   ```

2. Create `cmake/ctrl_toolboxConfig.cmake.in`:
   ```cmake
   @PACKAGE_INIT@
   include("${CMAKE_CURRENT_LIST_DIR}/ctrl_toolboxTargets.cmake")
   check_required_components(ctrl_toolbox)
   ```

3. Create `cmake/ports/ctrl_toolbox/vcpkg.json`:
   ```json
   {
     "name": "controller-toolbox",
     "version": "1.0.0",
     "description": "Discrete-time control library (C++20, Eigen, pybind11 optional)",
     "dependencies": ["eigen3"]
   }
   ```

4. Create `cmake/ports/ctrl_toolbox/portfile.cmake` (standard `vcpkg_from_git` + `vcpkg_cmake_configure` + `vcpkg_cmake_install` pattern).

5. Test locally: `vcpkg install controller-toolbox --overlay-ports=cmake/ports`

**Files to create/modify:**
- `CMakeLists.txt` (add install block)
- `cmake/ctrl_toolboxConfig.cmake.in` (new)
- `cmake/ports/ctrl_toolbox/vcpkg.json` (new)
- `cmake/ports/ctrl_toolbox/portfile.cmake` (new)

**Acceptance criteria:** A blank CMake project with `find_package(ctrl_toolbox REQUIRED)` and
`target_link_libraries(app ctrl::ctrl_toolbox)` compiles and links after `vcpkg install`.

---

### DIST-2 - Curated embedded header-only subset ✓ Done (Part 57E)

**Effort:** 1 person-week  
**Rationale:** `BasicPID<Scalar>` and `BasicSMC<Scalar>` (Part 54) are the seed. Extend to a
curated `lib/embedded/` with no Eigen, no heap, no virtual dispatch. Enables firmware targets
(STM32, ESP32, Cortex-M with `-fno-rtti -fno-exceptions`).

**Components to implement:**

| File | Class | Notes |
|------|-------|-------|
| `lib/embedded/BasicPID.h` | Copy from `lib/BasicPID.h` | Already done; move/symlink |
| `lib/embedded/BasicSMC.h` | Copy from `lib/BasicSMC.h` | Already done; move/symlink |
| `lib/embedded/DiscreteIntegrator.h` | `template<typename Scalar>` Euler integrator | `integrate(error)`, `reset()`, `value()` |
| `lib/embedded/FixedRateFilter.h` | `template<typename Scalar, int Order>` IIR | Compile-time order; no heap |
| `lib/embedded/RingBuffer.h` | `template<typename T, int N>` fixed FIFO | `push()`, `pop()`, `peek(int)` |
| `lib/embedded/EmbeddedControllers.h` | Umbrella `#include` | Includes all of the above |

**CMake option:**
```cmake
option(CTRL_BUILD_EMBEDDED_ONLY "Build only the header-only embedded subset (no Eigen)" OFF)
if(CTRL_BUILD_EMBEDDED_ONLY)
    add_library(ctrl_embedded INTERFACE)
    target_include_directories(ctrl_embedded INTERFACE lib/embedded)
    install(DIRECTORY lib/embedded/ DESTINATION include/ctrl_toolbox/embedded)
    return()  # skip rest of CMakeLists.txt
endif()
```

**Tests:** `tests/test_embedded_subset.cpp` - Catch2 tests with `float` and `double` instantiations:
- `BasicPID<float>` step response converges (anti-windup clamps correctly)
- `BasicSMC<float>` reaches zero in sliding phase
- `DiscreteIntegrator<double>` matches analytic integral of ramp
- `FixedRateFilter<double, 2>` attenuates above cutoff
- `RingBuffer<float, 8>` FIFO ordering, wrap-around

**Demo:** `examples/embedded/main.cpp` - compiles without Eigen headers in the TU. Verify with:
```bash
grep -r "Eigen" examples/embedded/main.cpp  # must return nothing
```

**Acceptance criteria:** All `test_embedded_subset` tests pass. `examples/embedded/main.cpp`
compiles with `-DCTRL_BUILD_EMBEDDED_ONLY=ON`. Zero Eigen includes in the embedded demo TU.

---

### DIST-3 - ROS2 thin wrapper package

**Effort:** 1-2 person-weeks  
**Rationale:** The HAL already has `IScheduler`/`SimScheduler`/FreeRTOS stubs. A ROS2 lifecycle
node wrapper is a thin bridge. Targets academic robotics labs and autonomous systems teams.

**Package layout:**
```
ros2/
  ctrl_toolbox_ros2/
    package.xml
    CMakeLists.txt
    include/ctrl_toolbox_ros2/
      controller_node.hpp          # ControllerNode<T> lifecycle node template
      multi_axis_node.hpp          # MultiAxisControllerNode<T, N> MIMO variant
    src/
      controller_node.cpp
    launch/
      demo_pid.launch.py           # PID on a simulated first-order plant
    README.md
```

**`ControllerNode<T>` interface:**
- Subscribes: `~/setpoint` (`std_msgs/Float64`), `~/measurement` (`std_msgs/Float64`)
- Publishes: `~/control_output` (`std_msgs/Float64`), `~/metrics` (`diagnostic_msgs/DiagnosticArray`)
- Services: `~/reset` (`std_srvs/Empty`), `~/set_params` (custom srv with JSON string)
- Parameters: `sample_time_ms` (double), `controller_name` (string, from registry)
- Lifecycle states: Unconfigured -> Inactive -> Active -> Finalized
- Timer uses `rclcpp::create_wall_timer` at `sample_time_ms` rate

**Key implementation note:** `T` must satisfy `IController`. The node holds a
`std::shared_ptr<T>` and calls `ctrl->compute(measurement - setpoint)` each tick.
For MRAC/L1Adaptive (set_reference convention), subclass and override `tick()`.

**Build test:**
```bash
cd ros2
colcon build --packages-select ctrl_toolbox_ros2
source install/setup.bash
ros2 launch ctrl_toolbox_ros2 demo_pid.launch.py
```

**Acceptance criteria:** `colcon build` succeeds on ROS2 Humble. A `DiscretePID`-backed node
tracks a step reference published to `~/setpoint` within 5% in < 10 s on a simulated first-order
plant. CI note: add a new GitHub Actions workflow `ros2.yml` that runs on Ubuntu 22.04 with
`ros-humble-ros-base` (separate from the main library CI).

---

### DIST-4 - PyPI wheel distribution via cibuildwheel ✓ Done (Part 57E)

**Effort:** 1 person-week  
**Rationale:** Python users currently must build from source (CMake + Eigen + pybind11). A PyPI
wheel enables `pip install controller-toolbox` and eliminates the build entirely for the
majority of Python users. This is the single highest-value accessibility improvement for the
Python binding audience.  
**Prerequisite:** DIST-1 (CMake `install()` targets must exist for the wheel build to collect headers).

**Files to create:**

`pyproject.toml` (root level):
```toml
[build-system]
requires = ["scikit-build-core>=0.9", "pybind11>=2.13"]
build-backend = "scikit_build_core.build"

[project]
name = "controller-toolbox"
version = "1.0.0"
description = "Discrete-time control library — Python bindings"
requires-python = ">=3.9"
dependencies = []

[tool.scikit-build]
cmake.build-type = "Release"
cmake.args = ["-DCTRL_BUILD_PYTHON_BINDINGS=ON", "-DCTRL_BUILD_TESTS=OFF"]
wheel.packages = ["ctrl_toolbox"]
```

`.github/workflows/publish.yml` (triggered on `v*.*.*` tag push):
```yaml
name: Publish to PyPI
on:
  push:
    tags: ["v*.*.*"]

jobs:
  build_wheels:
    name: Build wheels (${{ matrix.os }})
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [ubuntu-latest, windows-latest, macos-latest]
    steps:
      - uses: actions/checkout@v4
        with: {submodules: recursive}
      - uses: pypa/cibuildwheel@v2.19
        env:
          CIBW_BUILD: "cp39-* cp310-* cp311-* cp312-*"
          CIBW_SKIP: "*-musllinux*"
          CIBW_TEST_COMMAND: "python -c \"import ctrl_toolbox; print('OK')\""
      - uses: actions/upload-artifact@v4
        with:
          name: wheels-${{ matrix.os }}
          path: wheelhouse/

  upload_pypi:
    needs: build_wheels
    runs-on: ubuntu-latest
    environment: pypi
    permissions: {id-token: write}
    steps:
      - uses: actions/download-artifact@v4
        with: {pattern: wheels-*, merge-multiple: true, path: dist/}
      - uses: pypa/gh-action-pypi-publish@release/v1
```

**Implementation notes:**
- `scikit-build-core` replaces the hand-rolled `setup.py`; it drives CMake directly.
- Eigen must be vendored or fetched via `FetchContent` inside the wheel build (no system
  Eigen guaranteed on CI). Add a `FetchContent_Declare(Eigen3 ...)` fallback in root
  `CMakeLists.txt` guarded by `if(NOT Eigen3_FOUND)`.
- Test the wheel build locally first: `pip install cibuildwheel && cibuildwheel --platform linux`
  (requires Docker on Linux).
- Register the package name on PyPI (test.pypi.org first) before the first tag push.

**Acceptance criteria:**
1. `pip install controller-toolbox` succeeds on Python 3.9-3.12, Ubuntu/Windows/macOS.
2. `python -c "import ctrl_toolbox; print(ctrl_toolbox.registry_list())"` prints the full
   algorithm registry without error.
3. Wheel file size < 15 MB per platform (Eigen header inflation is normal).
4. `publish.yml` workflow completes green on a `v0.0.1-test` tag against test.pypi.org.

---

### DIST-5 - Automated GitHub release workflow ✓ Done (Part 57E)

**Effort:** 1-2 person-days  
**Rationale:** Currently there is no automated release process. Every release requires manual
artifact collection. A `release.yml` triggered on version tags produces a GitHub Release with
platform-specific static library bundles attached — giving C++ users a pre-built download
without needing vcpkg or a source build.  
**Prerequisite:** DIST-1 (CMake `install()` targets needed for `cmake --install` to collect
the right files into a staging directory).

**`.github/workflows/release.yml`:**
```yaml
name: Release
on:
  push:
    tags: ["v*.*.*"]

jobs:
  build:
    name: Build (${{ matrix.os }})
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [ubuntu-latest, windows-latest, macos-latest]
        include:
          - os: ubuntu-latest;  artifact: ctrl_toolbox-linux
          - os: windows-latest; artifact: ctrl_toolbox-windows
          - os: macos-latest;   artifact: ctrl_toolbox-macos
    steps:
      - uses: actions/checkout@v4
      - name: Configure
        run: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTRL_BUILD_TESTS=OFF
      - name: Build
        run: cmake --build build --config Release
      - name: Install to staging
        run: cmake --install build --prefix staging
      - name: Zip
        shell: bash
        run: cd staging && zip -r ../${{ matrix.artifact }}-${{ github.ref_name }}.zip .
      - uses: actions/upload-artifact@v4
        with:
          name: ${{ matrix.artifact }}
          path: ${{ matrix.artifact }}-${{ github.ref_name }}.zip

  release:
    needs: build
    runs-on: ubuntu-latest
    permissions: {contents: write}
    steps:
      - uses: actions/download-artifact@v4
        with: {merge-multiple: true, path: dist/}
      - uses: softprops/action-gh-release@v2
        with:
          files: dist/*.zip
          generate_release_notes: true
```

**Versioning convention:**
- Tags: `vMAJOR.MINOR.PATCH` (e.g., `v1.0.0`).
- `CMakeLists.txt`: add `project(ctrl_toolbox VERSION 1.0.0)` and propagate via
  `configure_file(cmake/version.h.in lib/version.h)`.
- The release notes are auto-generated from commit messages between tags (GitHub feature).

**Acceptance criteria:**
1. Pushing tag `v1.0.0` triggers the workflow and produces a GitHub Release with three `.zip`
   attachments (linux, windows, macos), each containing the static library, headers, and
   `ctrl_toolboxConfig.cmake`.
2. A downstream C++ project can unzip, point `CMAKE_PREFIX_PATH` at the staging dir, and
   `find_package(ctrl_toolbox REQUIRED)` succeeds without a source build.
3. The workflow completes in < 15 minutes wall-clock.

---

## Track 2: Analysis Pipeline

> **All ANA scripts live in `tools/`.** Each is a standalone CLI Python script.
> **ANA-1 is a prerequisite** - run it before any other ANA step. All scripts read from
> `case-study/*/logs/` and write enriched outputs back there.

### ANA-1 - Richer metric set + leaderboard (prerequisite) ✓ Done (Part 58)

**Effort:** 2-3 person-days  
**Prerequisite:** None

**New metric columns** (appended to every existing CSV schema or written to a sidecar
`*_metrics.csv` per run):

| Column | Definition | Notes |
|--------|-----------|-------|
| `settle_time_s` | Time when \|e(t)\| < 0.02.\|e(initial)\| and stays there (10-sample hysteresis) | -1.0 if never settles |
| `overshoot_pct` | (max(y) - y_ref_final) / \|y_ref_final\| * 100 | 0.0 if y never exceeds reference |
| `max_u` | max \|u(t)\| over the run | Actuator stress proxy |
| `energy_var` | variance of u(t) over the run | Actuator wear proxy |
| `rms_error` | sqrt(mean(e(t)^2)) | Complements IAE |

**`tools/metrics.py`** - shared Python module providing:
```python
def compute_metrics(t: np.ndarray, y: np.ndarray, u: np.ndarray,
                    ref: np.ndarray) -> dict[str, float]:
    """Returns dict with keys: settle_time_s, overshoot_pct, max_u, energy_var, rms_error"""
```

**Changes to case study runners:**
- Python-only (`sim/main.py`): import `tools/metrics.py`, call `compute_metrics()` at end of
  each (controller, scenario) run, append result dict as an extra row to CSV.
- C++ studies: add metric accumulators directly to `simulation_runner.cpp` using the same
  definitions. Write a final `METRICS` line to the CSV (or a sidecar file `<run>_metrics.csv`).

**`tools/compare_controllers.py` changes:**
- Add `--metric` flag: `settle_time_s | overshoot_pct | max_u | energy_var | rms_error | iae`
- Default sort: `iae`. Header row lists all 6 metrics. Width auto-adjusts.
- Add `--study` filter and `--scenario` filter (already partially present).

**Acceptance criteria:**
```bash
python tools/compare_controllers.py --study "Active Suspension" --metric settle_time_s
```
Prints a 6-column ranked table with no NaN values for all 18 controllers * 5 scenarios.

---

### ANA-2 - Monte Carlo sensitivity analysis ✓ Done (Part 58)

**Effort:** 4-5 person-days (1 day framework + 1 day per study batch)  
**Prerequisite:** ANA-1

**`tools/monte_carlo.py`** - CLI:
```
python tools/monte_carlo.py --study "Active Suspension" --n_trials 200 --param_spread 0.20 --seed 42
```

**Per-study parameter config** - `case-study/<Study>/config/param_ranges.json`:
```json
{
  "m_s": [350, 650],
  "k_s": [12000, 18000],
  "c_s": [900, 1500],
  "__comment": "Ranges are absolute [min, max]. Use __spread for relative +/-fraction."
}
```
If `"__spread": 0.20` is present instead of explicit ranges, derive +/-20% from the nominal
values found in the study's `config/scenarios/s01.json` (or equivalent).

**Algorithm:**
1. For each trial: sample param values uniformly from `param_ranges.json`.
2. Write a temp `_mc_params.json`, pass as env var `CTRL_MC_PARAMS` to the study's `main.py`.
3. Each study's runner reads `CTRL_MC_PARAMS` at startup and overrides plant params.
4. Collect all metric columns from the run's output CSV.
5. Aggregate to `case-study/<Study>/logs/mc_summary.csv`:
   columns: `trial, controller, scenario, iae, settle_time_s, overshoot_pct, max_u, energy_var, rms_error`

**`tools/mc_plots.py`** - reads `mc_summary.csv`, produces:
- Box plots: IAE distribution per controller, one subplot per scenario
- Failure-rate bar chart: fraction of trials where IAE > `fail_threshold` (set per study)
- Output: `case-study/<Study>/logs/mc_boxplots.png`, `mc_failure_rates.png`

**Acceptance criteria:** `monte_carlo.py --study "Vertical Drill String" --n_trials 50` completes
in < 10 min and produces `mc_summary.csv` with 50 * 17 * 5 = 4250 rows, no NaN in `iae` column.

---

### ANA-3 - Fault injection sweep ✓ Done (Part 58)

**Effort:** 1 person-week  
**Prerequisite:** ANA-1

**`tools/fault_injector.py`** - wrapper class:
```python
class FaultInjector:
    def __init__(self, ctrl, fault_type: str, severity: float, seed: int = 0):
        """
        fault_type in: 'sensor_noise', 'sensor_delay', 'actuator_deadband',
                        'sensor_bias', 'step_disturbance'
        severity: sigma for noise, n_steps for delay, width for deadband,
                  offset for bias, magnitude for disturbance
        """
    def compute(self, measurement: float) -> float:
        """Applies fault transform to measurement, delegates to wrapped ctrl."""
    def inject_disturbance(self, u: float, t: float) -> float:
        """For step_disturbance: adds magnitude to u at t_inject."""
```

**`tools/fault_sweep.py`** - CLI:
```
python tools/fault_sweep.py --study "Tug Boat" --fault sensor_delay \
    --levels "0,10,20,30,40,50" --scenario s01 --n_trials 1
```
- `levels`: comma-separated severity values (units depend on `fault_type`: ms for delay, sigma for noise)
- For each (controller, fault_level): run simulation with `FaultInjector` wrapping each controller,
  record all metric columns + `unstable` flag (IAE > threshold or NaN in output)
- Output: `case-study/<Study>/logs/fault_<type>_sweep.csv`

**`tools/fault_plots.py`** - reads fault sweep CSV, produces:
- Heatmap: rows = controllers, columns = severity levels, color = IAE (white->green = stable,
  yellow->red = degraded, black = unstable/NaN)
- One heatmap per fault type; all saved to `case-study/<Study>/logs/`

**Acceptance criteria:** `fault_sweep.py --study "Tug Boat" --fault sensor_delay --levels "0,10,20,30,40,50"`
produces a heatmap where PID goes black (unstable) before MPC and ADRC. SMC should be the most
delay-robust (characteristic of sliding mode).

---

### ANA-4 - ANOVA + Tukey HSD significance testing ✓ Done (Part 58)

**Effort:** 1-2 person-days  
**Prerequisite:** ANA-2 (`mc_summary.csv` must exist)

**`tools/anova.py`** - CLI:
```
python tools/anova.py --study "Active Suspension" --metric iae --scenario s02_sine_resonance
```

**Algorithm:**
1. Load `mc_summary.csv`. Group by `controller` for the specified `scenario` + `metric`.
2. One-way ANOVA (`scipy.stats.f_oneway`). If p < 0.05, proceed to post-hoc.
3. Tukey HSD (`statsmodels.stats.multicomp.pairwise_tukeyhsd`). Extract pairwise p-values.
4. Output pairwise significance matrix to console and CSV:
   `case-study/<Study>/logs/anova_<metric>_<scenario>.csv`

**Example console output:**
```
One-way ANOVA: IAE, scenario s02 (N=200 per controller)
F=22.1  p=3.4e-9  [SIGNIFICANT]

Tukey HSD (reject H0 if p < 0.05):
              MPC    ADRC    LQR    PID   SMC
MPC            -    0.001   0.18  <.001  0.002
ADRC        0.001     -     0.31   0.04  0.21
LQR          0.18   0.31     -     0.002 0.44
PID         <.001   0.04   0.002    -    0.03
SMC          0.002  0.21   0.44   0.03    -
```

**Dependencies (Python):** `scipy`, `statsmodels` - both already installed in `soft_robotics` env.

**Acceptance criteria:** Running on `mc_summary.csv` from ANA-2 with >= 50 trials per group
produces a p-value matrix with no NaN, and the result is interpretable (e.g., PID vs MPC
should be significant if the case study shows real performance differences).

---

### ANA-5 - Real-time WCET profiling ✓ Done (Part 58)

**Effort:** 1-2 person-days  
**Prerequisite:** ANA-1 (timing hooks go in the same runners)

**Python-only studies - instrumentation in `sim/main.py`:**
```python
import time, os
_profile = os.environ.get("CTRL_PROFILE") == "1"

# In the control loop:
if _profile:
    t0 = time.perf_counter_ns()
u = ctrl.compute(meas)
if _profile:
    elapsed_ns = time.perf_counter_ns() - t0
    timing_samples.append(elapsed_ns)
```
At run end, write `case-study/<Study>/logs/wcet_<ctrl_name>.csv` with columns:
`mean_us, max_us, p99_us, n_samples`

**C++ studies - add `--profile` flag to each `main.cpp`:**
Use `std::chrono::high_resolution_clock` around `ctrl->compute(...)`. Write timing CSV in
the same format.

**`tools/wcet_report.py`** - aggregates all `wcet_*.csv` files for a study, produces:
- Ranked bar chart: WCET (max_us) per controller
- Overlay: p99_us as error bar
- Horizontal reference lines at 0.5 ms, 1 ms, 5 ms, 10 ms (common RTOS budget thresholds)
- Output: `case-study/<Study>/logs/wcet_report.png` + `wcet_summary.csv`

**Acceptance criteria:** Running with `CTRL_PROFILE=1 python main.py` produces timing CSVs.
`wcet_report.py` correctly shows MPC WCET > LQR WCET > PID WCET for every study (this is
a sanity check, not a hard requirement on exact values).

---

### ANA-6 - Model validation / cross-validation ✓ Done (Part 58)

**Effort:** 1-2 person-days  
**Prerequisite:** E1/E2 (GreyBoxEstimator / RecursiveGreyBoxEstimator - done Part 52)

**Per-study plant spec** - create `case-study/<Study>/sim/plant_model_spec.py`:
```python
# Defines the ODE in GreyBoxEstimator-compatible form
def f_ode(x, u, p, t):
    """x: state, u: input, p: param vector, t: time -> xdot"""
    ...

def h_meas(x, p):
    """x: state, p: param vector -> y (measurement)"""
    ...

NOMINAL_PARAMS = np.array([...])   # nominal parameter values
PARAM_NAMES = ["m_s", "k_s", "c_s", ...]
PARAM_BOUNDS = (np.array([...]), np.array([...]))  # (lower, upper)
```

**`tools/model_validation.py`** - CLI:
```
python tools/model_validation.py --study "Vertical Drill String" --split 0.80
```

**Algorithm:**
1. Run study with OpenLoop controller, collect `(t, U, Y)` time series.
2. Split 80/20 (train/test) at the time axis.
3. Fit `ctrl.GreyBoxEstimator` on train data using `plant_model_spec.f_ode`, `h_meas`.
4. Call `estimator.predict(x0_test, U_test)` on test data.
5. Compute `NRMSE = ||Y_pred - Y_test|| / ||Y_test - mean(Y_test)||`.
6. Output table: param name, nominal, estimated, %error; + NRMSE_train, NRMSE_test.

**Output CSV:** `case-study/<Study>/logs/model_validation.csv`

**Acceptance criteria:** NRMSE_train < 0.05 on any study with a well-identified plant (sanity
check). A warning is printed if NRMSE_test > 0.10. Studies without `plant_model_spec.py` are
skipped with a clear message.

---

### ANA-7 - mu-analysis / structured uncertainty ✓ Done (Part 58)

**Effort:** 2 person-weeks  
**Prerequisite:** None (uses existing `StateSpace` models already in `lib/`)

**Scope:** Linear controllers only (PID, LQR, MPC in linear mode, ADRC as a linear approximation).
For each such controller in each C++ study:

1. Form closed-loop transfer matrix `T_cl(z)` from the linearised plant SS + controller gain.
2. Define structured uncertainty block `Delta = diag(delta1I_n1, ..., deltaₖI_nₖ)` where each `delta_i` is a
   +/-20% scalar uncertainty on a key plant parameter. Block structure mirrors `param_ranges.json`.
3. Compute mu upper bound via the skewed-mu power iteration:
   - Build `M = F_l(P, K)` (lower LFT of augmented plant with uncertainty ports)
   - Iterate: `mu_ub = min_D ||D.M.D^-^1||_inf` over diagonal real scalings D
   - In practice: start with D = I (gives `||M||_inf`), then refine via log-spaced grid search on D
4. Report: `mu_peak`, `mu_margin = 1 / mu_peak`, `omega_critical` (frequency of peak mu).

**`tools/mu_analysis.py`** - CLI:
```
python tools/mu_analysis.py --study "Active Suspension" --controllers "LQR,MPC,PID"
```

**`tools/mu_plots.py`** - produces:
- mu(omega) Bode-like plot (magnitude vs. frequency, one curve per controller)
- Summary table: controller, mu_peak, mu_margin, omega_critical_rad_s

**Implementation notes:**
- Use `python-control` for `ss()`, `feedback()`, `freqresp()`. Check availability:
  `conda run -n soft_robotics -- python -c "import control; print(control.__version__)"`.
  If absent: `conda install -n soft_robotics -c conda-forge python-control`.
- Full D-K iteration (alternating D-scale + Hinf synthesis) is high effort. Start with the
  simpler upper bound: `mu_ub(omega) = sigma_max(M(jomega))` (standard Hinf norm - valid when Delta is
  unstructured). Label plots clearly as "unstructured upper bound" until D-K is implemented.
- For structured Delta, the tighter D-scale bound can be computed offline; it does not need
  to be real-time.

**Files to create:**
- `tools/mu_analysis.py`
- `tools/mu_plots.py`
- `case-study/<Study>/sim/plant_model_spec.py` (shared with ANA-6; add uncertainty_structure dict)

**Acceptance criteria:** `mu_analysis.py` runs without error on Active Suspension. LQR mu-margin
> 0.5 (stable under +/-50% unstructured perturbation - expected for a well-tuned LQR). MPC
mu-margin is lower than LQR at high frequencies (expected: MPC is more aggressive).

---

## Track 3: Static HTML Report

### RPT-1 - Static controller performance report ✓ Done (Part 58)

**Effort:** 1 person-week (after ANA-1..7 data is available)  
**Prerequisite:** ANA-1..7 (sections for missing data are rendered as placeholder cards)  
**Technology:** Plotly (all charts embedded as JSON in HTML), Jinja2 (templating). Single
self-contained `.html` file per study. No server, no external HTTP requests, opens in any browser.

**Report structure - one HTML per case study:**

```
Section 1 - Study Overview
  Plant description (parsed from case-study/<Study>/README.md first ## block)
  Controller roster table (name, type, tuning summary if available)
  Scenario descriptions

Section 2 - Leaderboard  [requires ANA-1]
  Sortable Plotly table: IAE, settle_time_s, overshoot_pct, max_u, energy_var, rms_error
  Color scale per column (green = best quartile, red = worst)
  One tab per scenario

Section 3 - Monte Carlo Robustness  [requires ANA-2]
  Box plots: metric distribution per controller (Plotly box, one figure per metric)
  Failure rate bar chart (sorted descending)

Section 4 - Statistical Significance  [requires ANA-4]
  Pairwise Tukey HSD heatmap (Plotly heatmap)
  Color: green = not significantly different (p > 0.05), red = significantly different (p < 0.001)
  One dropdown per scenario

Section 5 - Fault Tolerance  [requires ANA-3]
  Heatmap: rows=controllers, columns=fault severity, color=IAE (Plotly heatmap)
  Tab per fault type (sensor_delay, sensor_noise, actuator_deadband)
  Cells with NaN (unstable) shown in black

Section 6 - Real-Time Feasibility  [requires ANA-5]
  Horizontal bar chart: WCET per controller (sorted)
  Vertical lines at 0.5 ms, 1 ms, 5 ms, 10 ms RTOS budget thresholds
  Secondary bars: p99_us

Section 7 - Model Validation  [requires ANA-6]
  Table: param name, nominal, estimated, %error
  Prediction overlay plot: Y_true vs Y_pred for test split
  NRMSE train/test displayed as subtitle

Section 8 - Structured Uncertainty  [requires ANA-7]
  mu(omega) Bode-like plot (one trace per linear controller)
  Summary table: controller, mu_peak, mu_margin, omega_critical
  Note: "unstructured upper bound" if D-K not implemented
```

**File layout:**
```
tools/
  generate_report.py          # CLI: --study, --output-dir
  generate_all_reports.py     # runs generate_report.py for all 16 studies
  templates/
    report_template.html      # Jinja2 master template
  report_sections/
    overview.py               # Section 1
    leaderboard.py            # Section 2
    monte_carlo.py            # Section 3
    anova.py                  # Section 4
    fault_tolerance.py        # Section 5
    wcet.py                   # Section 6
    model_validation.py       # Section 7
    mu_analysis.py            # Section 8
reports/                      # output directory (add to .gitignore)
  Active_Suspension_report.html
  Boiler_Control_report.html
  ...
```

**Data loading convention:** Each section module calls a `load(study_path) -> dict | None` function.
If the expected CSV does not exist (analysis not yet run), `load()` returns `None` and the section
renders a placeholder card: _"Run ANA-N to generate this section."_

**Build command:**
```bash
python tools/generate_all_reports.py --output-dir reports/
```

**Acceptance criteria:**
1. `generate_all_reports.py` completes without error for all 16 studies.
2. Each `.html` file is self-contained (no external URLs in the source).
3. All 8 sections render with real data (or placeholder cards) when opened in Firefox/Chrome.
4. File size < 10 MB per report (Plotly JSON compression enabled).
5. Leaderboard table is sortable by clicking column headers.

---

## Execution Schedule

```
Week 1      DIST-1 (vcpkg + CMake install, 2-3 days)   ← unblocks DIST-4 and DIST-5
            ANA-1  (metrics leaderboard, 2-3 days)

Week 2      DIST-5 (release.yml, 1-2 days)             ← quick win once DIST-1 done
            DIST-2 (embedded subset, 5 days)
            ANA-2  (Monte Carlo framework, 3 days)

Week 3      DIST-4 (pyproject.toml + publish.yml, 5 days)
            ANA-2  (per-study param_ranges.json + validation, 2 days)
            ANA-3  (fault injection, 3 days)

Week 4      DIST-3 (ROS2, starts here, 5-10 days)
            ANA-4  (ANOVA, 1-2 days)
            ANA-5  (WCET, 1-2 days)

Week 5      DIST-3 (cont.)
            ANA-6  (model validation, 2 days)
            ANA-7  (mu-analysis, starts here, 10 days)

Week 6      ANA-7  (cont.)

Week 7      RPT-1  (static report, 5 days - ANA-1..6 done, ANA-7 can be in progress)
```

DIST track and ANA track are fully parallel. If only one developer is available, do:
DIST-1 -> DIST-5 -> ANA-1 -> ANA-2 -> DIST-4 -> ANA-3 -> ANA-4 -> ANA-5 -> ANA-6 -> DIST-2 -> ANA-7 -> RPT-1 -> DIST-3

---

## Quick Acceptance Checklist

```
DIST-1  [x] CMake install targets + EXPORT added to lib/CMakeLists.txt (Part 57E)
        [x] cmake/ControllerToolboxConfig.cmake.in created (Part 57E)
        [x] cmake/ports/ctrl_toolbox/vcpkg.json + portfile.cmake created (Part 57E)
        [ ] vcpkg install succeeds on a clean machine (needs first tag + SHA512 update in portfile)
        [ ] find_package(ControllerToolbox) works in a blank CMake project

DIST-2  [x] lib/embedded/: DiscreteIntegrator.h, FixedRateFilter.h, RingBuffer.h, EmbeddedControllers.h (Part 57E)
        [x] tests/test_embedded_subset.cpp: 13 Catch2 tests, links only Catch2 (Part 57E)
        [x] examples/embedded/main.cpp: zero Eigen includes in TU (Part 57E)
        [x] CTRL_BUILD_EMBEDDED_ONLY=ON option added to root CMakeLists.txt (Part 57E)
        [ ] Verify test_embedded_subset passes after next cmake build

DIST-3  [x] ros2/ctrl_toolbox_ros2/ package created with ControllerNode<T> template (Part 60)
        [x] package.xml + CMakeLists.txt (ament_cmake, finds ctrl::controller_toolbox)
        [x] example/pid_temperature_node.cpp (DiscretePID, all gains as ROS 2 params)
        [x] README with colcon build instructions and topic/parameter table
        [ ] colcon build verified on ROS 2 Humble (Ubuntu 22.04) - needs ROS 2 environment
        [ ] PID lifecycle node tracks step reference in demo launch
        [ ] Consider ros2.yml CI workflow (separate job, ros-humble-ros-base on ubuntu-22.04)

DIST-4  [x] pyproject.toml created with scikit-build-core backend (Part 57E)
        [x] .github/workflows/publish.yml created, cibuildwheel v2.21.3 (Part 57E)
        [ ] pip install controller-toolbox succeeds on Python 3.9-3.12 (all 3 platforms)
        [ ] PyPI Trusted Publisher configured before first tag push
        [ ] publish.yml workflow green on test.pypi.org before production push

DIST-5  [x] .github/workflows/release.yml created, softprops pinned SHA (Part 57E)
        [ ] Pushing v1.0.0 tag creates GitHub Release with 3 platform zips attached
        [ ] Each zip contains static lib + headers + ControllerToolboxConfig.cmake
        [ ] Downstream find_package(ControllerToolbox) succeeds from unzipped staging dir

ANA-1   [ ] compare_controllers.py --metric settle_time_s produces 6-column table
        [ ] No NaN in any metric column after a full run.py

ANA-2   [ ] mc_summary.csv exists for at least 3 studies after --n_trials 50
        [ ] Box plots and failure rate charts render correctly

ANA-3   [ ] Fault heatmap correctly identifies unstable cells (black)
        [ ] SMC is more delay-robust than PID in at least one study

ANA-4   [ ] ANOVA p-value matrix has no NaN for studies with >= 50 MC trials
        [ ] At least one controller pair is significantly different (p < 0.05)

ANA-5   [ ] WCET_MPC > WCET_LQR > WCET_PID (sanity order holds)
        [ ] wcet_summary.csv exists for all profiled studies

ANA-6   [ ] NRMSE_train < 0.05 for at least one study (sanity)
        [ ] model_validation.csv lists all param estimates

ANA-7   [ ] mu_analysis.py runs without error
        [ ] mu_margin > 0.5 for LQR on Active Suspension (sanity)
        [ ] mu(omega) plot clearly distinguishes LQR from PID

RPT-1   [ ] generate_all_reports.py completes for all 16 studies
        [ ] No external URLs in any .html file
        [ ] Leaderboard table is sortable
        [ ] Placeholder cards shown where analysis data is absent
        [ ] File size < 10 MB per report
```

---

## Cross-References

- Existing handoff: `prompt/handoff_part57.md` - covers Iterations A-F (correctness sweep,
  performance pre-allocation, test coverage, CI hygiene, HAL thread safety, D2/C2 stubs).
  The items in this roadmap are **in addition to**, not a replacement for, those iterations.

## Deliberately Excluded (and why)

| Item | Reason not included |
|------|-------------------|
| CodeQL / ASan / UBSan | Already tracked as Iteration D finding #38 in `handoff_part57.md` |
| Doxygen → GitHub Pages automation | `doc.yml` already exists; minor tweak, not a distribution item |
| Conan package (`conanfile.py`) | User selected vcpkg specifically; redundant for same goal |
| Debian/RPM packages | Not in selected scope; niche compared to vcpkg/PyPI reach |
| CPack installers | DIST-1 + DIST-5 cover the C++ distribution need without CPack complexity |
| Digital Twin / Streamlit (D2) | Postponed; open at LOW priority in `handoff_part57.md` §3 |
| TCLab HIL | Postponed per user decision |
- Algorithm history: `docs/compact_bug_report_parts_1-25.md`, `docs/compact_bug_report_parts_26-50.md`
- Active issues: `docs/cumulative_bug_report.md`
- Audit findings: `docs/audit_report.md` (84 items, 80 open after Part 56)
- Case study patterns: `prompt/make_case_study_python.md`, `prompt/make_case_study_cpp.md`
