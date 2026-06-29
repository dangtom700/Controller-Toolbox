# Controller Toolbox - Cumulative Bug Report (Part 51+)

**Active issues start at Part 51.** Earlier history is archived in two compact references:
- [`docs/compact_bug_report_parts_1-25.md`](compact_bug_report_parts_1-25.md) - Parts 1-25 (2026-05-19 through 2026-05-30)
- [`docs/compact_bug_report_parts_26-50.md`](compact_bug_report_parts_26-50.md) - Parts 26-50 (2026-05-31 through 2026-06-11)

Read both compact files for tribal knowledge before making any changes to controllers or case studies.

---

## Open Issues Log (Part 51+)

*(Append dated entries below as work proceeds.)*

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| **P1** | `DAESystem` struct + `dae2ode()` - Index-1 semi-explicit DAE; Newton solve on `g` | HIGH | **Done (Part 51)** |
| **P2** | `c2d()` overload for DAE - linearise + algebraic elimination + ZOH/Tustin | MED | **Done (Part 51)** |
| **P3** | DAE-aware EKF - post-update algebraic projection via `consistentInit()` | MED | **Done (Part 51)** |
| **E1** | `GreyBoxEstimator` - non-linear param estimation via Levenberg-Marquardt | HIGH | **Done (Part 52)** |
| **E2** | `RecursiveGreyBoxEstimator` - augmented-state UKF for online param tracking | HIGH | **Done (Part 52)** |
| **E3** | GP Residual Model - extend `GaussianProcess` with uncertainty output | MED | **Done (Part 52)** |
| **E4** | MHE Polytopic Constraints - extend MHE with `C_ineq`/`d_ineq` | MED | **Done (Part 53)** |
| **H1** | `HybridModel` base class - `IPlantModel` with `f_phys + f_data` | MED | **Done (Part 53)** |
| **H2** | `HybridMPC` - `NonlinearMPC` variant using `HybridModel` | MED | **Done (Part 53)** |
| **H3** | RL-MPC stitching Python example | LOW | **Done (Part 53)** |
| **H4** | `HybridModelTrainer` - hyperopt for `f_data` component | LOW | **Done (Part 53)** |
| **D1** | Mismatch Detector - CUSUM on KF/MHE innovation | LOW | **Done (Part 54)** |
| **D2** | Digital Twin Lite Python app | LOW | Open |
| **C2** | 8 spec-only stubs remain (BEMS + MEMS no blocker; DustControl/ModularEvap/SoftRobot/ControlTheory need plant-model design; Bioreactor/Nuclear thin specs). **DeePC closed (Part 57)** - `lib/DeePC.{h,cpp}` confirmed present and fully smoke-tested. | MED | Open |
| **C3** | Active Suspension 2-DOF: add `GAOptPIDCtrl` / `PSOOptPIDCtrl` / `DEOptPIDCtrl` using new lib/ GA/PSO/DE optimisers; 15->18 controllers, 75->90 runs | MED | **Done (Part 55)** |
| **C4** | SMISMO: modify plant for variable P_s; add `DOBEnergyCtrl` (Chen 2018 Eq. 29-30 DOB + adaptive supply pressure); 12->13 controllers, 60->65 runs | MED | **Done (Part 55)** |
| **C5** | EHFS: add `HinfODFCCtrl` + `HinfCascadeCtrl` (DiscreteHinf ODFC + local nLMS); 12->14 controllers, 60->70 runs | MED | **Done (Part 55)** |
| **C6** | Active Suspension 6*6 EV Full Model: NEW Python-only case study; 40-state plant (15-DOF vehicle + 5-DOF human biodynamic model), road time delays, 18 controllers, 90 runs | MED | **Done (Part 55)** |
| **B36-3** | Unify NaN-guard across controller fleet | MED | **Done (Part 53)** |
| R1 | Edge-case contract matrix tests for every controller family | MED | **Done (Part 53)** |
| T3 | Full DK-iteration with vector-fitting rational D(jomega) | LOW | **Done (Part 53)** |
| B36-2 | `ex79_registry_monitor` monitors nothing (M3 telemetry mis-wired) | LOW | **Done (Part 39, confirmed Part 53)** |
| REL | Rebuild `ctrl_toolbox.pyd` in Release | LOW | Open |
| M4 | `template<typename Scalar>` leaf algorithms for embedded float target | Backlog | **Done (Part 54)** |
| **Iter E** | Audit Iteration E - final batch (13 code changes + 29 verified/deferred); audit complete (84/84 findings addressed across Iter A-E) | MIXED | **Done (Part 57E)** |
| **DIST-1/2/4/5** | CMake install target + vcpkg port / embedded header-only subset / PyPI wheels (cibuildwheel) / GitHub Release workflow | MED | **Done (Part 57E)** |
| **ANA-1..7** | `tools/metrics.py`, `compare_controllers.py`, `monte_carlo.py`/`mc_plots.py`, `fault_injector.py`/`fault_sweep.py`/`fault_plots.py`, `anova.py`, `wcet_report.py`, `model_validation.py`, `mu_analysis.py`/`mu_plots.py` | MED | **Done (Part 58)** |
| **RPT-1** | `tools/generate_report.py` - self-contained per-study HTML report, 8 sections, Plotly CDN charts | MED | **Done (Part 58)** |
| **PLT-1** | `setup.sh` + `compile.sh` - Linux/macOS bootstrap + full-build scripts mirroring `setup.ps1`/`compile.bat` | MED | **Done (Part 59)** |
| **TRK-1** | `tools/case_study_tracker.py` + `docs/case_study_status.md` - auto-generated case-study status table | MED | **Done (Part 59)** |
| **DIST-3** | ROS2 thin wrapper package (`ros2/ctrl_toolbox_ros2/`, `ControllerNode<T>` lifecycle template) | MED | **Done (Part 60)** |
| **C2-Stewart** | 6-DOF Stewart Platform Vessel Motion Simulator - new C++ case study (12 ctrl x 60 sea-state configs = 720 runs) | HIGH | **Done (Part 61)** |
| **TRK-2** | `case_study_tracker.py` rewritten to 4-tier status (Complete/On-going/Open placeholder/Not started); old 3-tier scheme mis-classified untouched `new_case_study.py` scaffolds as "On-going" | MED | **Done (Part 62)** |
| **DOC-1** | Reconciled `docs/PROJECT_MASTER_STATE.md` (was 12 Parts behind) + fixed stale repo-wide references | MED | **Done (Part 62)** |
| **C2-NEW** | `Aircraft Engine Thermal Management` promoted into the official 18-study roster (documentation only - already fully implemented) | - | **Done (Part 63)** |
| **ROB-1** | Robustness analysis (fault sweep + Monte Carlo + WCET via `case-study/common/RobustnessStats.h`) for C++ case studies - 3 studies Part 64 (ActiveSuspension/Humidification/SolarCooling), remaining 7 Part 66 (Boiler/Tug/BuckBoost/SolarCooker/SOTEC/SMISMO/Stewart) | MED | **Done for all 10 C++ studies (Part 64 + 66)** - open for ~21 Python-only/not-yet-implemented studies |
| **GR-1** | `tools/generate_report.py` `_section_comparison()` raised `KeyError` on any study with zero parseable run rows (every Open-placeholder/Not-started scaffold) | LOW | **Done (Part 66)** |
| **AE-WCET** | Aircraft Engine Thermal Management `run_wcet_profile()` + `config/analysis.json` hook | LOW | **Done (Part 66)** |
| **ACT-1** | Added `ResonantController`/`NotchFilter`/`PhaseLockedLoop` ("Additional Controller Types" backlog category) | MED | **Done (Part 67)** |
| **DBG-1** | `-DCMAKE_BUILD_TYPE=Debug` (`-g`) fails on pre-existing Eigen-heavy TUs (`DiscreteHinf.cpp`, `test_catch2_advanced.cpp`); root cause not fixed, workaround documented | LOW | Open |
| **P3-PH2** | `docs/ALGORITHM_ROADMAP_PHASE3.md` Phase 2 - OC1 `SelfTuningRegulator`, SI1 `MLEIdentifier`, EF2 `SetMembershipEstimator`, EF3 `ParticleFilterV2`, MO1 `NSGA2`, MO3 `tuneConstrained`, DT4 `FaultClassifier`/`FTCSupervisor` | HIGH | **Done (Part 68)** |

---

*(New parts appended below as work proceeds.)*

---

## Part 57E - Audit Iter E + DIST-1/2/4/5 - 2026-06-14

### Audit Iteration E (13 code changes, 29 verified/deferred - all 84 findings now closed)

All remaining audit findings from Iterations A-D were swept. See `docs/audit_report.md` Part 57E
section for full detail. Key code changes:
- **#46** `EchoStateNetwork.h` `W_out_` comment corrected (`n_out*n_res`, not `n_out*(n_res+n_in)`)
- **#54** `DiscreteSMC.h` `slidingSurface()` docstring clarified (returns s[k-1], not current s[k])
- **#55** `ParticleFilter.h` `w_` comment corrected (normalised probabilities, not log-sum-exp)
- **#59** `FeedforwardController.h` `rv_` promoted to pre-allocated member (no per-step heap alloc)
- **#64/#66** `test_catch2_advanced.cpp`: added `[pid]` tests (Kb anti-windup, N-filter decay, computeDoM comparative)
- **#65** `test_catch2_advanced.cpp`: added `[repetitive]` edge-case tests (invalid period, NaN hold-last, setParams reset)
- **#67** `test_catch2_advanced.cpp`: added `[extremum_seeker]` convergence test (J=(theta-2)^2)
- **#68** `test_catch2_advanced.cpp`: renamed DynaController test + added bounded-range assertions
- **#69** `controllers_bindings.cpp`: `DynaController::inner_controller()` changed to `return_value_policy::copy`
- **#71** `smoke_test.py`: GreyBoxEstimator predict() shape assertion now checks both dimensions
- **#72** Created `_setup_bindings.py` (repo root); removed inline DLL block from 19 `examples/python/` + 9 `case-study/sim/` files
- **#74/#75** `doc.yml`: peaceiris action pinned to SHA `4f9cc6ed`; added `permissions: contents: write`

### DIST-1 - CMake install targets + vcpkg port (complete)

- **`lib/CMakeLists.txt`** - Added `EXPORT ControllerToolboxTargets` + `configure_package_config_file` +
  `write_basic_package_version_file` + install rules for cmake config files.
  Consumers use: `find_package(ControllerToolbox REQUIRED)` then `target_link_libraries(app ctrl::controller_toolbox)`
- **`cmake/ControllerToolboxConfig.cmake.in`** - Package config template; propagates Eigen3 dependency.
- **`cmake/ports/ctrl_toolbox/vcpkg.json`** - vcpkg port manifest (eigen3 dependency, python-bindings feature).
- **`cmake/ports/ctrl_toolbox/portfile.cmake`** - Standard `vcpkg_from_git` + configure + install pattern.
  *Note: SHA512 must be updated with actual hash after first v*.*.* tag push.*

### DIST-2 - Embedded header-only subset (complete)

- **`lib/embedded/DiscreteIntegrator.h`** - `template<Scalar>` backward-Euler integrator; `integrate()`, `value()`, `reset()`, `set()`.
- **`lib/embedded/FixedRateFilter.h`** - `template<Scalar, Order>` compile-time-order IIR LPF; backward Euler, stack state.
- **`lib/embedded/RingBuffer.h`** - `template<T, N>` fixed-capacity FIFO ring buffer; `push()`, `pop()`, `peek()`, `clear()`.
- **`lib/embedded/EmbeddedControllers.h`** - Umbrella include: re-exports `BasicPID.h`, `BasicSMC.h` + all 3 new files.
- **`examples/embedded/main.cpp`** - Demo: zero Eigen includes; verify with `grep -r "Eigen" examples/embedded/main.cpp`.
- **`tests/test_embedded_subset.cpp`** - 13 Catch2 `[basic_pid_embedded]`/`[basic_smc_embedded]`/`[discrete_integrator]`/`[fixed_rate_filter]`/`[ring_buffer]` tests; links only Catch2 (no controller_toolbox Eigen dep).
- **`CMakeLists.txt`** - Added `CTRL_BUILD_EMBEDDED_ONLY` option (early `return()` skips all Eigen targets) and
  `CTRL_FETCH_EIGEN_IF_MISSING` option (FetchContent fallback for CI wheel builds).
- **`tests/CMakeLists.txt`** - Added `test_embedded_subset` target (no Eigen, C++17, Catch2 only).

### DIST-4 - PyPI wheel distribution (complete)

- **`pyproject.toml`** - `scikit-build-core` backend; `cmake.args` enable Python bindings, disable tests;
  `CTRL_FETCH_EIGEN_IF_MISSING=ON` for CI containers. Triggered on `v*.*.*` tag.
- **`.github/workflows/publish.yml`** - cibuildwheel v2.21.3 (pinned SHA); builds cp39-cp312 on
  Linux/Windows/macOS; skips musl and 32-bit; publishes via PyPI trusted publishing (OIDC).
  *Requires "Trusted Publisher" configured in PyPI settings before first push.*

### DIST-5 - GitHub Release workflow (complete)

- **`.github/workflows/release.yml`** - Triggered on `v*.*.*` tag; builds Release on 3 platforms;
  `cmake --install` collects lib + headers + cmake config; zips and attaches to GitHub Release.
  Uses softprops/action-gh-release pinned to SHA `c062e08b` (v2.0.8).

---

## Part 59 - Cross-Platform Scripts + Case Study Tracker - 2026-06-15

### PLT-1 - `setup.sh` (Linux/macOS bootstrap)

Mirrors `setup.ps1` exactly. Five steps:

1. **Toolchain check** - accepts `gcc` or `clang`; cmake, ninja via `_need()`; eigen3 by
   header-path scan (`/usr/include/eigen3`, `/opt/homebrew/...`). Non-fatal eigen3 miss:
   emits `CTRL_FETCH_EIGEN_IF_MISSING=ON` to cmake (FetchContent fallback).
   Per-distro install hints printed for apt/dnf/pacman/brew.
2. **Conda check** - fails clearly with Miniconda install URL if not on PATH.
3. **Env create/update** - `conda env create -f environment.yml` (first time) or
   `conda env update --prune` (existing). Skipped by `--skip-conda-create`.
4. **Bindings build** - `conda run -n soft_robotics -- cmake -G Ninja ... --target ctrl_toolbox`.
   Locates built `.so`/`.dylib` under `build/bindings/`; fails clearly if absent.
5. **Smoke test** - `conda run -n soft_robotics -- python bindings/smoke_test.py`.
6. **Optional full build** - calls `compile.sh` when `--full-build` passed.

Staged with `git update-index --chmod=+x` (100755 mode) -> lands executable on Linux clone.

### PLT-1 - `compile.sh` (Linux/macOS full build)

Mirrors `compile.bat`. Bash array of all 120 targets in dependency order; `cmake --build`
called once per target (sequential, no `--parallel`). Exits on first failure with clear error.
Flag: `--no-config` to skip cmake re-configure.

Staged with `git update-index --chmod=+x` (100755 mode).

### TRK-1 - `tools/case_study_tracker.py` + `docs/case_study_status.md`

Completed the tracker stub. Key fixes/additions:

- **`detect_language()`** - fixed extension comparison (was comparing `'cpp'` vs `'.cpp'`);
  fixed division-by-zero when no source files found; removed unreachable `return "mixed"`.
  Now follows 3-step spec: (1) check `sim/main.py` or `sim/src/main.cpp`; (2) depth heuristic
  on `sim/src/` existence; (3) extension count across whole tree.
- **`detect_status()`** - new: Complete (sim+logs+config+HTML), On-going (sim+logs+config),
  Incomplete (PDF or README on disk), Not started (default).
- **`find_pdf_link()`** / **`find_readme_link()`** - return `docs/`-relative markdown links.
- **`main()`** - walks `case-study/*/`; writes `docs/case_study_status.md` (Markdown table).

**Output (2026-06-15):** 23 studies detected.

| Status | Count | Examples |
|--------|-------|---------|
| On-going | 16 | All 9 C++ studies + 7 Python-only studies |
| Incomplete | 7 | BEMS, SoftRobot + 5 newly discovered (see below) |

Language breakdown: 9 C++, 7 Python, 7 undetermined (spec-only stubs with no source files).

### Reconciliation finding: 5 previously undocumented stubs

The tracker scan revealed 5 case-study directories on disk that were **not in CLAUDE.md**:

| Directory | Has PDF | Has README | Notes |
|-----------|---------|-----------|-------|
| `6-DOF Stewart Platform Vessel Motion Simulator/` | (check) | (check) | Stewart platform 6-DOF kinematics |
| `Heavy-Duty Parallel-Serial Hydraulic Manipulator VDC/` | (check) | (check) | VDC control; hydraulic parallel-serial arm |
| `Hybrid-Driven Tendon-Pneumatic Soft Manipulator/` | (check) | (check) | Adaptive kinematic + stiffness control |
| `Underwater Robotic Manipulator Trajectory Tracking/` | (check) | (check) | Implicit rigid TubeMPC + ASMC |
| `Unmanned Surface Vehicle Wave-Predictive Attitude Control/` | (check) | (check) | Short-time wave prediction MPC |

All added to the CLAUDE.md spec-only stubs table. Total stubs updated from 8 -> 12
(7 on-disk with PDF+README, 5 plan-only with no directory yet).

### Non-obvious caveats (Part 59)
```
setup.sh eigen3 check      -> non-fatal; emits CTRL_FETCH_EIGEN_IF_MISSING=ON when headers absent
setup.sh BINDING detection -> find build/bindings -name 'ctrl_toolbox*.so' -o -name '*.dylib'
compile.sh --no-config     -> skips cmake configure; build/ must already exist
case_study_tracker.py ROOT -> "case-study" relative path; must run from repo root
detect_status "Complete"   -> requires .html report file in study dir (generate_report.py output)
detect_language step 3     -> extension count only reaches here when no sim/ dir exists
```

---

## Part 58 - ANA-1..7 + RPT-1 Analysis Pipeline + Test Bug Fixes - 2026-06-15

### Analysis pipeline (`tools/`) - ANA-1 through ANA-7 + RPT-1

All analysis and reporting tools are now implemented as standalone CLI scripts under `tools/`.

**ANA-1 - `tools/metrics.py`** (prerequisite module)
- `compute_metrics(t, y, u, ref)` -> dict with `iae`, `rms_error`, `settle_time_s`, `overshoot_pct`,
  `max_u`, `energy_var`. Settling uses 2% band + 10-sample hysteresis.
- `compute_metrics_from_df(df)` - heuristic column detection; works on any case-study CSV.
- `extract_final_iae(df)` - reads last-row IAE from `iae_cumulative`, `IAE_y1..y3`, or computes from `error`.

**ANA-1 - `tools/compare_controllers.py`** (T7 re-implementation + ANA-1 extension)
- Auto-discovers `case-study/*/logs/run_*.csv`.
- Parses `run_{scenario}_{controller}.csv` naming convention (last `_`-token = controller).
- Flags: `--study`, `--scenario`, `--controller`, `--metric` (iae/rms_error/...), `--sort`, `--wide`, `--csv`.

**ANA-2 - `tools/monte_carlo.py` + `tools/mc_plots.py`**
- Imports study `sim/main.py` and calls `run_single(ctrl_name, perturbed_params)` if present.
- Perturbs `plant_params.json` numeric keys by `N(0, sigma)` per sample.
- Writes `mc_summary_*.csv`; `mc_plots.py` produces violin and scatter PNGs.

**ANA-3 - `tools/fault_injector.py` + `tools/fault_sweep.py` + `tools/fault_plots.py`**
- `FaultInjector`: composable sensor/actuator fault injection (bias, noise, loss, stuck, setpoint step).
- `fault_sweep.py` calls `sim.run_with_fault(ctrl_name, FaultSpec)` if present; writes `fault_sweep_*.csv`.
- `fault_plots.py` produces heatmaps and degradation curves per fault kind.

**ANA-4 - `tools/anova.py`**
- One-way ANOVA (scipy `f_oneway`) + Tukey HSD post-hoc (statsmodels `pairwise_tukeyhsd`).
- Reads any CSV with `controller` + metric column. Reports F, p, significance, and pairwise table.

**ANA-5 - `tools/wcet_report.py`**
- Discovers `wcet_*.csv` files (produced by optional timing instrumentation in `sim/main.py`).
- Aggregates mean, median, p99, WCET (q=0.999) per controller; writes `wcet_summary.csv` + optional bar chart.
- Includes instrumentation howto printed when no files found.

**ANA-6 - `tools/model_validation.py`**
- Uses `ctrl.GreyBoxEstimator` to fit ODE parameters to logged data; reports NRMSE.
- Studies must expose `grey_box_model()` -> `(ode_fn, h_fn, x0, param_names, bounds)` in `sim/main.py`.
- Graceful fallback (IAE proxy) when hook is missing.

**ANA-7 - `tools/mu_analysis.py` + `tools/mu_plots.py`**
- Identifies discrete ARMA(2,2) model from each CSV; evaluates peak singular value of S(z) and T(z).
- Peak |T(z)| is an unstructured mu upper bound. Writes `mu_summary.csv`; plots bar + S vs T scatter.

**RPT-1 - `tools/generate_report.py`**
- Single self-contained HTML with 8 sections: Summary, Comparison, Heatmap, MC, Fault, ANOVA, WCET, Mu.
- Uses Plotly (inline CDN) for interactive charts. Gracefully degrades to plain tables if plotly absent.
- `--out report.html --open` writes and opens in browser.

### Test bug fixes

**`lib/RepetitiveController.cpp` - `setParams()` did not reset `v_now_`**
- When `periodSteps` changes, `v_buf_.assign(...)` cleared the buffer to zeros but `v_now_` was not
  updated. `correction()` returned the stale pre-change value. Fixed: added `v_now_ = 0.0` inside the
  `if (p.periodSteps != p_.periodSteps)` block.

**`tests/test_catch2_advanced.cpp` - `computeDoM` test measured wrong signal**
- Test "DiscretePID computeDoM gives strictly lower peak output" measured plant output `y`, not
  control signal `u`. For a first-order plant, DoE gives higher `u[0]` (derivative kick) but that
  does not reliably translate to lower `max(y)` across all plants/gains. Fixed: test now measures
  `peak_u` (peak control signal). DoM provably eliminates the derivative kick in `u`, so
  `peak_dom_u < peak_standard_u` holds unconditionally for any `Kd > 0`.
- Test renamed to "DiscretePID computeDoM gives strictly lower peak control signal than compute on step".

### Non-obvious API facts (Part 58)
```
tools/compare_controllers.py  -> auto-discovers CSVs; controller = last _-token in filename
tools/monte_carlo.py          -> requires run_single(ctrl_name, params) hook in sim/main.py
tools/fault_sweep.py          -> requires run_with_fault(ctrl_name, FaultSpec) hook
tools/model_validation.py     -> requires grey_box_model() in sim/main.py + ctrl_toolbox binding
tools/wcet_report.py          -> discovers wcet_*.csv; prints instrumentation guide if none found
tools/generate_report.py      -> requires plotly (pip install plotly) for interactive charts
RepetitiveController.setParams -> now also resets v_now_ when periodSteps changes
```

---

## Part 54 - D1 (MismatchDetector), M4 (BasicPID/BasicSMC) - 2026-06-12

**D1 - `MismatchDetector`** (`lib/MismatchDetector.h`, header-only)

- Wraps `CUSUMChart` (from `ControllerMonitor.h`) to run real-time CUSUM on the
  normalised innovation of a `KalmanFilter` or `MovingHorizonEstimator`.
- `MismatchDetectorParams`: `sigma` (in-control innovation RMS), `k_cusum` (slack, default 0.5),
  `h_threshold` (alarm level, default 5.0).
- `update(double)` feeds a scalar innovation (absolute value used internally to detect
  both upward and downward shifts in magnitude).
- `update(VectorXd&)` computes `||innov||/sqrt(p)` and feeds to scalar CUSUM.
- `detected()`: sticky bool - stays `true` until `reset()` is called.
- `score()`: current CUSUM statistic `max(C+, C-)`.
- `CTRL_REGISTER_FEATURE(mismatch_detector)`.

**D1 - Extensions to `KalmanFilter`** (`lib/KalmanFilter.{h,cpp}`)

- `enableMismatchDetection(params)`: attaches a `MismatchDetector` member; feeds
  `innov = y - C*x_pred - D*u` into CUSUM after every `update()` call.
- `mismatchDetected()`, `mismatchScore()`, `resetMismatchDetector()` accessors.
- Private: `std::optional<MismatchDetector> mismatch_det_`.

**D1 - Extensions to `MovingHorizonEstimator`** (`lib/MovingHorizonEstimator.{h,cpp}`)

- Same API: `enableMismatchDetection()`, `mismatchDetected()`, `mismatchScore()`,
  `resetMismatchDetector()`.
- Feeds `y_hist_[N] - C*x_est_` (one-step-ahead residual) into CUSUM after each
  `estimate()` call.

**D1 - Bindings / tests**

- `estimation_bindings.cpp`: `enable_mismatch_detection(sigma, k_cusum, h_threshold)`,
  `mismatch_detected()`, `mismatch_score()`, `reset_mismatch_detector()` on both
  `KalmanFilter` and `MovingHorizonEstimator`.
- `smoke_test.py`: D1 block - creates KF, enables detection, asserts no alarm on zero
  steps and `mismatch_score()` is float, checks registry.
- `tests/test_catch2_advanced.cpp`: 7 new `[mismatch_detector]` tests:
  (1) no alarm on white-noise innovation, (2) sustained shift triggers detection,
  (3) reset clears alarm, (4) vector innovation fires alarm, (5) KF disabled by default,
  (6) KF fires on wrong model, (7) MHE fires on mismatched model.

**M4 - `BasicPID<Scalar>` and `BasicSMC<Scalar>`** (`lib/BasicPID.h`, `lib/BasicSMC.h`,
header-only)

- `BasicPID<Scalar>`: backward-Euler ISA PID with derivative filter (alpha = 1/(1+N*Ts)),
  back-calculation anti-windup (Kb), and saturation. No IController base, no Eigen, no
  virtual dispatch. Suitable for `float` on MCU targets. Methods: `compute(error)`,
  `reset()`, `setParams()`, `lastOutput()`, `integrator()`.
- `BasicSMC<Scalar>`: first-order SMC with boundary-layer saturation
  `u = -K * clamp(s/phi, -1, 1)` where `s = c_e*e + c_de*(e - e_prev)`. Same embedded-target
  constraints. Methods: `compute(error)`, `reset()`, `setParams()`, `lastOutput()`,
  `slidingSurface(error)`.
- Both added to `lib/ControllerToolbox.h` umbrella include.
- `CTRL_REGISTER_FEATURE(basic_pid)` / `CTRL_REGISTER_FEATURE(basic_smc)` for feature checks.
- No Python bindings (template types; not useful via pybind11 `shared_ptr<IController>` path).
- `smoke_test.py`: checks `registry_has('basic_pid')` and `registry_has('basic_smc')`.
- `tests/test_catch2_advanced.cpp`: 7 new tests - 4 `[basic_pid]` (step response, float
  saturation, reset, anti-windup bounded integrator) + 3 `[basic_smc]` (convergence,
  float saturation, reset reproducibility).

---

## Part 53 - Hybrid Models (H1-H4), E4, T3, B36-2/B36-3, R1 - 2026-06-12

**E4 - MHE Polytopic Inequality Constraints** (`lib/MovingHorizonEstimator.{h,cpp}`)

- `MHEParams::C_ineq` (m_c * n) and `d_ineq` (m_c): enforce `C_ineq * x_0 <= d_ineq` on
  arrival state after the FISTA solve via Hildreth's cyclic half-space projections.
- `ineq_proj_iters` (default 20): number of Hildreth sweeps. Box constraints (`xMin`/`xMax`) are
  re-applied after polytope projection so both are simultaneously satisfied.
- `projectX0Polytope()` private helper: no-op when `C_ineq` is empty; otherwise iterates
  half-space projections then re-clips to box.
- 3 `[mhe_polytopic]` Catch2 tests: half-space upper bound, simplex-coupled constraint,
  equivalence with xMax for pure box case.
- `estimation_bindings.cpp`: `C_ineq`, `d_ineq`, `ineq_proj_iters` exposed on `MHEParams`.
- `smoke_test.py`: E4 block asserts `C_ineq`/`d_ineq` shape round-trips correctly.

**H1 - `HybridModel`** (`lib/HybridModel.h`, header-only)

- `IPlantModel` abstract interface: `dynamics(x, u)`, `stateSize()`, `outputSize()`.
- `HybridModel` concrete class: `f_phys(x, u)` (physics, required) + optional `f_data(x, u)`
  (data-driven correction). `setDataModel()` / `clearDataModel()` swappable at runtime.
- RK4 `predict(x0, U, Ts)` helper on `IPlantModel`.

**H2 - `HybridMPC`** (`lib/HybridMPC.{h,cpp}`)

- Inherits `NonlinearMPC`; overrides the prediction rollout to call `HybridModel::dynamics`.
- `updateDataModel(DataFunc)`: hot-swaps the data correction without rebuilding the QP.
- `N_update` parameter: data model refreshed from new observations every N steps.

**H3 - RL-MPC Stitching** (`examples/python/ex101_rl_mpc_stitching.py`, Python-only)

- Lightweight DQN-style policy (<10 k params, numpy-only) that adjusts `rho_y` of `HybridMPC`
  in real time on a spring-mass-damper plant.
- Policy state: `[error, error_dot]`; actions: 4 discrete `rho_y` multipliers.
- Demonstrates H2 + H3 integration without PyTorch dependency.

**H4 - `HybridModelTrainer`** (`lib/HybridModelTrainer.{h,cpp}`)

- `trainGP(XU, residuals, gp)` -> `DataFunc`: fits `GaussianProcess` residual and returns a
  lambda capturing the trained GP's `predict()` method.
- `trainESN(XU, residuals, esn)` -> `DataFunc`: offline ridge-regression on `EchoStateNetwork`
  and returns the ESN forward-pass lambda.
- `trainRidge(XU, residuals, lambda_reg)` -> `DataFunc`: lightweight fallback using Eigen
  ridge (no external dependency).

**T3 - VectorFitting + full DK-iteration** (`lib/VectorFitting.{h,cpp}`)

- Gustavsen SK iterative rational fitting of complex frequency response data -> poles + residues.
- `solveMuSyn` in `DiscreteHinf`: `dFitOrder > 1` switches from first-order D-scaling to
  vector-fit rational D(jomega), enabling full DK-iteration for structured-uncertainty mu-synthesis.

**B36-2 - `ex79_registry_monitor` fix** (Part 39, confirmed Part 53)

- `shared_ptr` monitor was copy-constructed before attachment; fixed to create `mon_ptr` first
  so callback and observer share a single instance. Observer now actually fires.

**B36-3 - NaN-guard hold-last fleet contract** (`lib/IController.h` + 7 controllers)

- `sanitize()` removed from contract. Hold-last NaN behaviour added to:
  ExtremumSeeker, MRAC, TubeMPC (scalar path), ScenarioMPC (scalar path),
  RepetitiveController, SmithPredictor, DiscreteHinf.
- `IController.h` documents the hold-last contract.
- `[nan_guard]` Catch2 tags added across all affected controllers.

**R1 - Contract matrix tests** (`tests/test_catch2_advanced.cpp`)

- `[nan_guard]` + `[health_contract]` extended to all controller families.
- Saturation-bounded-integral and non-stabilizable `isHealthy()` coverage added.

**M4 - BasicPID / BasicSMC (CLAIMED done in CLAUDE.md - NOT VERIFIED)**

- CLAUDE.md states `lib/BasicPID.h` and `lib/BasicSMC.h` (header-only `BasicPID<Scalar>` /
  `BasicSMC<Scalar>` for embedded float usage) were created in Part 53.
- **Files do NOT exist** in the repository as of the Part 53 audit (2026-06-12).
- Marked Open in the issues table. Must be implemented before closing M4.

---

## Part 51 - DAE Architecture (P1/P2/P3) - 2026-06-12

**P1 - `DAESystem` + `consistentInit` + `dae2ode`** (`lib/PlantModel.h/.cpp`)

- `DAESystem` struct: `f` (differential), `g` (algebraic), `h` (output), `n_diff`, `n_alg`, `Ts`.
- `consistentInit(dae, x1_init, u0, x2_guess)`: Newton-Raphson (LDLT) solving `g=0` for `x2`; up to 20 iters, tol=1e-9.
- `dae2ode(dae)`: returns discrete step function `x_aug_next = F(x_aug, u)`. Forward Euler for `x1`, Newton projection for `x2` at both current and next `x1`. `Ts` must be set on `DAESystem`.
- Three static central-difference Jacobian helpers (`algJacX1`, `algJacX2`, `algJacU`) in `PlantModel.cpp`.
- `CTRL_REGISTER_FEATURE(dae_system)` added after `namespace ctrl`.

**P2 - `c2d(DAESystem, x1_op, x2_op, u_op, Ts, method)`** (`lib/PlantModel.h/.cpp`)

- Index-1 algebraic elimination: `A_red = A11 - A12*G2^-^1*G1`, `B_red = B1 - A12*G2^-^1*B2` where all Jacobians are computed numerically via `algJac*` helpers.
- Checks `rcond(G2) > 1e-12`; throws `std::runtime_error("c2d(DAESystem): G2 is singular - DAE is not Index-1 at operating point.")` otherwise.
- Output matrix built from `h` Jacobians (or identity w.r.t. `x1` if `h` not set).
- Dispatches to existing `c2d(StateSpace, Ts, method)` for ZOH/Tustin.
- Python binding registered as `dae_c2d` (avoids `py::overload_cast` ambiguity with existing `c2d`).

**P3 - DAE-aware EKF projection** (`lib/ExtendedKalmanFilter.h/.cpp`)

- `setAlgebraicConstraint(g_alg, n_diff, n_alg, tol=1e-9)`: attaches algebraic constraint function; validates `n_diff + n_alg == n_states_`.
- `hasAlgebraicConstraint()`: bool accessor.
- `projectAlgebraicStates(u)`: called at end of `update()` when constraint is set. Newton-Raphson on `x2` block using `numericalJacobian`; then covariance projection `P = J_proj * P * J_proj'` where `J_proj = [[I, 0]; [-G2^-^1G1, 0]]`.
- SISO assumption: `u_scalar = u(0)` (consistent with `DAESystem::AlgFunc` signature).
- Independent `AlgConstraintFn` type alias in EKF (does not depend on `PlantModel.h`).

**Bindings / tests**

- `plantmodel_bindings.cpp`: `DAESystem` class with `set_f/set_g/set_h`, `consistent_init`, `dae2ode`, `dae_c2d`.
- `estimation_bindings.cpp`: `set_algebraic_constraint`, `has_algebraic_constraint` on `ExtendedKalmanFilter`.
- `smoke_test.py`: 4 DAE assertions (`consistent_init`, `dae2ode`, `dae_c2d`, `registry_has('dae_system')`).
- `tests/test_catch2_advanced.cpp`: 7 Catch2 tests - `[dae_system]` *3, `[dae_c2d]` *2, `[dae_ekf]` *2.

---

## Part 52 - Model Estimation E1/E2/E3 - 2026-06-12

**E1 - `GreyBoxEstimator`** (`lib/GreyBoxEstimator.{h,cpp}`)

- Batch Levenberg-Marquardt for user ODE `f(x,u,p)` and measurement `h(x,p)`.
- RK4 integration with `rk4_steps` substeps per `Ts`; central finite-difference Jacobian
  (step `eps_i = eps_jac * max(|p_i|, 1)` per parameter); box-constrained params via projection.
- LM normal equations: `(J'J + lambda*diag(J'J)) dp = -J'r`; accept/reject; `lambda *= nu` on
  reject, `lambda /= nu` on accept. Convergence: `max|J'r| < tol_grad`.
- `fit(x0, U, Y)` returns `Result{params, cost, iterations, converged}`.
- `predict(x0, U)` returns `Y_hat (n_y x N)` using current `p_est_`.

**E2 - `RecursiveGreyBoxEstimator`** (`lib/RecursiveGreyBoxEstimator.{h,cpp}`)

- Augmented state `z = [x; p]`; `f_aug` integrates ODE (RK4) for `x` and holds `p` constant
  with diffusion `Q_param`. `h_aug(z, u) = h(z.head(n_x), z.tail(n_p))`.
- UKF created at `initialize()` with `P0_aug = blkdiag(P0_state, P0_param)`.
- Compiled only under `CTRL_ENABLE_ADVANCED_KALMAN` (same guard as UKF/EKF).
- Default `alpha=0.1` (not 1e-3) - augmented `n_aug = n_state + n_param >= 3`.

**E3 - `GPResidualModel`** (`lib/GPResidualModel.{h,cpp}`)

- Composition over existing `GaussianProcess`; stores residuals `epsilon = y_true - y_model`.
- `addResidualPoint(xf, y_true, y_model)`: appends `(xf, epsilon)` to GP dataset.
- `residualFit(X_feat, Y_true, model_fn)`: batch version - resets GP, adds all points, calls `fit()`.
- `predictWithUncertainty(xf, model_pred)`: returns `{model_pred + gp_mean, gp_mean, gp_variance}`.
  Returns `{model_pred, 0.0, 0.0}` before first `fit()` call.

**Bindings / tests**

- `estimation_bindings.cpp`: `GreyBoxParams`, `GreyBoxResult`, `GreyBoxEstimator`; `RecursiveGreyBoxParams`,
  `RecursiveGreyBoxEstimator` (inside `CTRL_HAS_ADVANCED_KALMAN` guard).
- `controllers_bindings.cpp`: `GPResidualParams`, `GPResidualPrediction`, `GPResidualModel`.
  `residual_fit` wraps `model_fn` via `py::object` lambda capture.
- `smoke_test.py`: 3 assertion blocks (E1 fit, E2 step, E3 residual + batch).
- `tests/test_catch2_advanced.cpp`: 8 new Catch2 tests - `[grey_box]` *3, `[recursive_grey_box]` *2,
  `[gp_residual]` *3. Bug fix: `Eigen::Vector1d` does not exist - replaced with explicit `VectorXd(1)`.
- `examples/ex80_grey_box_estimator.cpp`; `ex97_grey_box_estimator.py`, `ex98_recursive_grey_box.py`,
  `ex99_gp_residual_model.py`.

---

## Part 55 - Controller Gap Audit + C3-C6 (2026-06-13)

**Audit methodology:** All 15 case studies were cross-checked: each study's README was read to
identify the paper's proposed controller, then the actual `sim/` source was inspected to verify
presence. Studies that are pure plant-characterisation papers or literature reviews (no novel
controller proposed) were marked N/A. Three genuine gaps were identified (C3-C5) plus one new
study requested (C6).

---

**C3 - Active Suspension 2-DOF: metaheuristic-tuned PID controllers**

- Paper: Aydogan & Yildiz 2025 (*Alexandria Eng. J.* 127, 989-1003).
  Core contribution: GA/PSO/DE optimisation of suspension controller gains on a 15-DOF full vehicle.
- Gap: The 2-DOF sim (`susp_sim`, 15 controllers) has no parameter optimisation loop at all.
- Fix: Add `lib/GeneticAlgorithm.h`, `lib/ParticleSwarmOptimizer.h`, `lib/DifferentialEvolution.h`
  (header-only; reuse `TunerResult`/`CostFn` from `AutoTuner.h`); add `GAOptPIDCtrl`,
  `PSOOptPIDCtrl`, `DEOptPIDCtrl` to Active Suspension 2-DOF sim (15->18 controllers, 75->90 runs).
- New lib/ files require full 8-step checklist: bindings in `analysis_bindings.cpp`, smoke tests,
  Catch2 `[genetic_algorithm]`/`[pso]`/`[de]` tests (3 each), combined Rosenbrock example.
- Files: `lib/GeneticAlgorithm.h`, `lib/ParticleSwarmOptimizer.h`, `lib/DifferentialEvolution.h`,
  `lib/ControllerToolbox.h`, `bindings/analysis_bindings.cpp`, `bindings/smoke_test.py`,
  `tests/test_catch2_advanced.cpp`, `examples/exNN_metaheuristics.{cpp,py}`,
  `case-study/Active Suspension .../sim/{include/controllers.h, src/controllers.cpp, src/main.cpp}`,
  `tests/test_susp_regression.cpp`.

---

**C4 - SMISMO: disturbance-observer supply-pressure adaptation**

- Paper: Chen & Wang 2018. Core contribution: 2nd-order DOB (Eq. 29-30) estimates load force
  `F_hat`; adaptive supply pressure `P_s = k_f*|F_hat| + k_v*|v_L| + P_margin` (Eq. 37) achieves
  ~5/6 energy reduction vs. fixed supply. Grey predictor (GM(1,1), Eq. 52) drives pump speed.
- Gap: `smismo_sim` holds P_s constant at 60 bar; no disturbance observer; no energy comparison.
- Fix: (1) `smismo_plant.h/cpp` - add `setSupplyPressure(bar)`; replace hardcoded `P_s` with
  dynamic `P_s_dyn_` in valve flow equations; existing 12 controllers unaffected (never call setter).
  (2) Add `DOBEnergyCtrl` (#13): inner PID for position + DOB for load force estimation +
  adaptive P_s before each plant step. Observer gain L=30 (Chen Table 1). P_s clamped [22, 65] bar.
  (3) Grey predictor omitted (requires pump dynamics not in current sim; noted in comment).
- Files: `sim/include/smismo_plant.h`, `sim/src/smismo_plant.cpp`, `sim/include/controllers.h`,
  `sim/src/controllers.cpp`, `sim/src/main.cpp` (N_CONTROLLERS 12->13), `tests/test_smismo_regression.cpp`.
- Key invariant: `P_min = 22 bar > P_bd = 20 bar` - backpressure guard must be preserved.

---

**C5 - EHFS: PI + Hinf ODFC + nLMS 3-layer cascade**

- Paper: Shen et al. 2017. Core contribution: 3-layer cascade - base PI + Hinf offline-designed
  feedback controller (ODFC) + normalised LMS adaptive compensator (CSIA) for force ripple at
  velocity reversal and load stiffness variation.
- Gap: All 12 current EHFS controllers are generic toolbox algorithms; no Hinf ODFC or nLMS class.
- Fix: Python-only. Linearise 5-state EHFS plant around operating point (F0=3000N, x_v0=0);
  use `ctrl.DiscreteHinf.solve()` + `ctrl.MixedSensitivity.build()` offline at construction.
  `HinfODFCCtrl` (#13): Hinf controller as standalone feedback. `HinfCascadeCtrl` (#14): PI +
  Hinf (additive) + nLMS adaptive correction (additive). Fall back to PID if Hinf synthesis infeasible.
  Local `NormalisedLMS` class (~30 lines): regressor shift-register + normalised update rule.
- Files: `case-study/Tracking Control of Electro-Hydraulic Force Servo Systems/sim/main.py`
  (add 2 controllers; 12->14; update docstring 60->70 runs).
- Non-obvious: `DiscreteHinf` takes `HinfResult` in its constructor - use `ctrl.DiscreteHinf(result)`.
  EHFS Ts=0.5ms -> linearised state-space and Hinf design must use the same Ts.

---

**C6 - Active Suspension 6*6 EV Full Model (new Python-only case study)**

- Paper: Aydogan & Yildiz 2025 (same as C3). Full 20-DOF (40-state) system: 15-DOF body
  (Z, theta, phi + 6 wheels + 6 in-wheel motors) + 5-DOF human biodynamic model (seat/pelvis/lower
  torso/upper torso/head). Paper proposes GA/PSO/DE-optimised PD control for all 6 wheel actuators.
- Directory: `case-study/Active Suspension 6x6 EV Full Model/` (Python-only; not in CMake or compile.bat).
- Plant: `EV6x6Plant` in `sim/ev6x6_plant.py` - assembles M (20*20 diagonal), K and C (20*20,
  includes pitch/roll-to-wheel coupling via geometric offsets a_i, e_i), B_road (20*6), B_act (20*6);
  state-space form `A=[0, I; -M^{-1}K, -M^{-1}C]` (40*40); discretised at Ts=0.005s via ZOH.
- Road model: `sim/road_model.py` - 2 independent channels with time-delay ring buffer for middle
  (delay=L1/v) and rear (delay=(L1+L2)/v) axles. L1=L2=2.0m, v=22.2m/s (80 km/h).
- Parameters: per-corner values inherit from 2-DOF study (k_s=16000, c_s=980, k_t=160000,
  M_w=36kg); human model from ISO 2631 Guide E / Griffin 1990; full-vehicle mass sourced from
  paper Table 2. Note in README if any parameters are estimated.
- Controllers (18): PassiveCtrl, PDCtrl, GAOptPDCtrl (paper's method), PSOOptPDCtrl, DEOptPDCtrl,
  PIDCtrl, LQRCtrl (full 40-state DARE), LQGCtrl (partial obs), MPCCtrl (12-state reduced),
  ADRCCtrl, SMCCtrl, MRACCtrl, FuzzyPIDCtrl, TubeMPCCtrl, ILCCtrl, CBFCtrl, L1AdaptiveCtrl,
  ScenarioMPCCtrl. Per-wheel strategies = 6 independent SISO instances; centralised = full model.
- 5 scenarios matching 2-DOF study (step bump, sine resonance, rough road, speed bump, comfort).
  Total: 18 * 5 = 90 runs.
- CSV columns: `t, Z_body, Z_head, theta, phi, Z_wr1-3, Z_wl1-3, F_act_1-6, z_r_1-6, seat_accel,
  head_accel, body_accel, iae_cumulative`.
- Key tribal knowledge: MPC/TubeMPC/ScenarioMPC use a 12-state reduced model for prediction;
  the full 40-state plant always runs for simulation (they diverge). LQR uses the full system
  (40 states, 6 inputs) - DARE is feasible since B_act has rank 6 (independent wheel actuators).
  ADRC omega_o*Ts < 0.5 constraint at Ts=0.005s -> omega_o < 100.

---

## Part 56 - Test/Binding Bug Fixes - 2026-06-14

Six compile/binding/test errors found in `run_20260614_080012.log` and `run_20260614_084939.log`
and fixed in this session. No new algorithms added; no case-study logic changed.

---

**Fix 1 - `GradientProjectionQP` y_fista workspace (tests)**
- `solveGradientProjectionQP` signature has 12 parameters; the 12th (`y_fista` workspace
  `Eigen::Ref<VectorXd>`) was added in a prior session when FISTA momentum was introduced.
- Two tests in `tests/test_catch2_advanced.cpp` (lines ~109-112 and ~132-133) still called
  the 11-argument version -> compile error "too few arguments".
- Fix: added `Eigen::VectorXd y_fista(n);` workspace and passed as 12th argument in both tests.
- `Eigen::Ref<VectorXd>` (not raw `VectorXd&`) is required for the workspace parameters so that
  block expressions (e.g., `.head(n)`) can be passed without a copy.

---

**Fix 2 - `ConsistentInitResult` binding (`plantmodel_bindings.cpp`)**
- `ctrl::consistentInit()` returns `ConsistentInitResult{VectorXd x, bool converged}`.
- The `consistent_init` Python binding at `plantmodel_bindings.cpp:197` returned the full
  struct, which pybind11 cannot auto-convert -> `TypeError: Unregistered type: ctrl::ConsistentInitResult`.
- Fix: added `.x` to the return: `return ctrl::consistentInit(...).x;` - binding now returns
  a plain `np.ndarray`.
- Matching fix in `tests/test_catch2_advanced.cpp:4253`: test extracted `ci_result.x` manually.
- **Non-obvious:** The binding strips the struct; Python callers receive only the VectorXd.
  `converged` flag is not exposed in Python. C++ callers should always check `.converged`.

---

**Fix 3 - `TaylorApproximator::evaluate()` method name (test)**
- Test at `tests/test_catch2_advanced.cpp:5494` called `approx.eval(x)`.
- Actual method name is `evaluate(x)` -> compile error "no member named eval".
- Fix: renamed call to `approx.evaluate(x)`.

---

**Fix 4 - ZPETC amplitude-flatness test (`test_catch2_advanced.cpp`)**
- The `[zpetc]` test checked `|arg(G * Gff)| < 1^\circ` (near-zero phase). For a strictly proper
  2nd-order plant (relative degree 2), ZPETC yields `G * Gff = z^{-1}` (one-sample delay).
  At test frequency omega = 0.1pi/T, the phase is `-0.1pi rad approx = -18^\circ` - this is **correct ZPETC
  behaviour**, not a bug. The invariant is amplitude flatness, not zero phase.
- Fix: replaced phase assertion with amplitude assertion:
  `REQUIRE_THAT(std::abs(G * Gff), WithinAbs(1.0, 0.05))`
- **Tribal knowledge:** ZPETC (Tomizuka 1987) guarantees `|G(e^{jomegaT}) * G_ff(e^{jomegaT})| approx = 1`
  for all frequencies below Nyquist (amplitude flat), at the cost of a group delay equal to
  the plant relative degree d. Absolute phase equals `-d.omega.T` - non-zero and unavoidable.
  The correct performance metric for ZPETC is tracking amplitude fidelity, not phase lead.

---

**Fix 5 - `ComputationalDelayWrapper` Python binding (`controllers_bindings.cpp`)**
- `ComputationalDelayWrapper` (header-only, `lib/ComputationalDelayWrapper.h`) was included
  in the umbrella header and registered as a feature, but never added to pybind11 bindings.
- `smoke_test.py:1015` assertion `assert hasattr(ctrl, 'ComputationalDelayWrapper')` failed.
- Fix: added full `py::class_<ComputationalDelayWrapper, IController, shared_ptr<...>>` binding
  in `controllers_bindings.cpp` with `compute`, `reset`, `sample_time`, `last_output` methods.
  Constructor: `ComputationalDelayWrapper(inner: IController, initial_output: float = 0.0)`.
- **Python usage:**
  ```python
  pid     = ctrl.DiscretePID(Kp, Ki, Kd, Ts)
  delayed = ctrl.ComputationalDelayWrapper(pid)
  u = delayed.compute(error)  # returns u from previous step; first call returns 0.0
  ```

---

**Fix 6 - `make_lqr_controller` Python binding (`controllers_bindings.cpp`)**
- `ctrl::makeLQRController` (free function, `lib/DiscreteLQR.h`) creates a
  `shared_ptr<LQRAdapter>` (an `IController`) from a `StateSpace` plant + `LQRParams` +
  two `std::function<VectorXd()>` providers. Never exposed to Python.
- `smoke_test.py:1020` assertion `assert hasattr(ctrl, 'make_lqr_controller')` failed.
- Fix: added `m.def("make_lqr_controller", ...)` binding. Python callables are wrapped via
  `py::object` lambda capture (not `py::cpp_function`) so that pybind11 type-erasure works
  correctly for `std::function<Eigen::VectorXd()>` parameters.
- **Python usage:**
  ```python
  state_fn = lambda: np.array([x1, x2])   # returns current state
  ref_fn   = lambda: np.array([r1, r2])   # returns reference state (None = zero)
  lqr_ctrl = ctrl.make_lqr_controller(plant_ss, lqr_params, state_fn, ref_fn)
  u = lqr_ctrl.compute(0.0)  # state + reference fetched via closures each call
  ```
- **Important:** `state_fn` and `ref_fn` must return `np.ndarray`, not lists. The closure
  captures state by reference - update the captured variables before each `compute()` call.

---

**Fix 7 - `HybridModel::makeDynamicsFunc` safe factory (`examples/ex81_hybrid_model_mpc.cpp`)**
- `model_plain->dynamicsFunc()` (deprecated method capturing `this` via raw pointer) was used
  to construct a `NonlinearMPC`. This is undefined behaviour if `model_plain` is destroyed
  before the NMPC finishes.
- Fix: replaced with `ctrl::HybridModel::makeDynamicsFunc(model_plain)` which captures the
  `shared_ptr` by value, extending model lifetime to match the NMPC.

---

**Non-obvious API facts added / clarified (Part 56)**

```
consistentInit()            -> returns ConsistentInitResult{x, converged}; Python binding
                               strips struct and returns np.ndarray (only .x exposed)
solveGradientProjectionQP() -> 12 args total; 9th-12th are workspace Eigen::Ref<VectorXd>
                               (y_fista is the 12th); must declare before calling
ZeroPhaseTrackingFilter     -> ZPETC gives |G*Gff| approx = 1 (amplitude flat, NOT zero phase);
                               phase = -d*omega*T; test amplitude, not phase
ComputationalDelayWrapper   -> Python: ctrl.ComputationalDelayWrapper(inner, initial_output=0.0)
make_lqr_controller         -> Python: ctrl.make_lqr_controller(plant_ss, lqr_params,
                               state_fn, ref_fn=None); callables must return np.ndarray
HybridModel.dynamicsFunc()  -> DEPRECATED; use ctrl.HybridModel.makeDynamicsFunc(model_sptr)
```

---

## Part 57 - Audit Iteration A Verification - 2026-06-14

No new algorithms or case-study logic added. All 17 Iteration A findings from `docs/audit_report.md`
(2026-06-13) were verified as **already resolved** in the codebase before this session.

**DeePC confirmed implemented.** The CRITICAL audit finding (#1) - `lib/DeePC.{h,cpp}` absent -
was invalidated: both files are present. The smoke test at `bindings/smoke_test.py:1029-1050`
asserts `ctrl.DeePC`, `ctrl.DeePCParams`, and `registry_has('deepc')`. C2 DeePC note closed.

**Iteration A findings verified already resolved (no code changes needed):**

| Audit # | Sev | Finding | Confirmed state |
|---------|-----|---------|----------------|
| 1 | CRIT | DeePC files absent | `lib/DeePC.{h,cpp}` present |
| 2 | HIGH | DE reflection infinite loop | Cap at 20 iters + `std::clamp` already in `DifferentialEvolution.h:135` |
| 3 | HIGH | LQR MIMO warning in NDEBUG only | No `#ifndef NDEBUG` guard in current `DiscreteLQR.h:186` |
| 4 | HIGH | EKF LDLT unchecked | `.info() != Eigen::Success` guard + flag already at `ExtendedKalmanFilter.cpp:127` |
| 5 | HIGH | L1 dead `use_compute_y_` field | Field absent from `L1AdaptiveController.h` |
| 6 | HIGH | HybridModel raw-this capture | `[[deprecated(...)]]` already on `dynamicsFunc()` at `HybridModel.h:170` |
| 13 | HIGH | DiscreteLQG missing `shared_ptr` holder | `shared_ptr<ctrl::DiscreteLQG>` already at `controllers_bindings.cpp:410` |
| 14 | HIGH | 8 ML/optimizer classes missing `shared_ptr` holder | All 10 classes (AutoTuner, BayesianOptimizer, GA, PSO, DE, SINDy, KoopmanEDMD, GP, ESN, ILC) already have `shared_ptr<T>` |
| 19 | MED | HybridMPC training buffer unbounded | FIFO eviction (`max_buffer_` cap) already at `HybridMPC.cpp:34` |
| 20 | MED | GainScheduled `lowerIndex()` unchecked empty | `assert(!schedule_.empty())` already at `GainScheduledController.h:234` |
| 21 | MED | CEM silent zero return | `#ifndef NDEBUG` clog warning already at `CEMController.cpp:106` |
| 34 | MED | Stale Fuzzy TODO dead block | Block absent from `controllers_bindings.cpp` |
| 36 | MED | SubspaceID TODO stub dead block | Replaced with comment at `estimation_bindings.cpp:433` |
| 70 | LOW | 4 classes not smoke-tested | `CUSUMChart`, `EWMAChart`, `SmithPredictor`, `ExtremumSeeker` asserted at `smoke_test.py:1022-1025` |
| 73 | LOW | `cxx_std_17` in examples CMake | `cxx_std_20` already in `examples/CMakeLists.txt:6` |
| 76 | LOW | `.gitignore` missing patterns | `*.pyd`, `*.exe`, `CMakeCache.txt`, `CMakeFiles/`, `.idea/`, `case-study/**/logs/` all present |
| 77 | LOW | `/tools` gitignored | `/tools` is not gitignored; `!/tools/*.py` negation is superfluous but harmless |

**Docs updated this session:**
- `docs/audit_report.md`: resolution status table appended (Part 57 section)
- `docs/cumulative_bug_report.md`: C2 DeePC note closed; Part 57 section added
- `CLAUDE.md`: Open Items updated to Part 57; Part 57 Done block added

**Remaining open:** 67 audit findings - #7-12 (HIGH Performance), #15-18, #22-33, #35, #37-69,
#71-72, #74-75, #78-84. See Iterations B-E in `prompt/prompt_enhanced.txt` for the work order.

---

## Part 57B - Audit Iteration B (Performance Pre-allocation) - 2026-06-14

**Iteration B complete.** 13 per-step heap allocation findings checked; 9 already resolved,
4 required code changes. All 4 are now fixed:

| Finding | Controller | Code change |
|---------|-----------|-------------|
| #8 MHE `estimate()` | `MovingHorizonEstimator` | Added `A_pow_ws_` (pre-computed A-power table, eliminates N matrix mults/allocs per step), `Qinv_`/`Rinv_` (cached LDLT inverses, eliminates 2 LDLT solves per step), `CTPsi_ws_`/`H_eff_ws_` (scratch matrices, eliminate 2 large allocs per step). Zero heap allocs from these 5 sources in steady-state. |
| #24 CEM `rolloutCost()` | `CEMController` | Added `mutable x_roll_`, `e_roll_`, `u_k_roll_`; eliminates `VectorXd(n)` + `VectorXd(p)` inside the Np-step rollout loop (hottest allocation path in CEM). `computeRef()` uses pre-allocated `u_k_` for return value. |
| #26 NeuralPID `compute()` | `NeuralPID` | Added `Eigen::VectorXd h_` member; eliminates per-control-step `VectorXd(n_hidden)` hidden activation alloc. |
| #48 KoopmanEDMD `liftRBF()` | `KoopmanEDMD` | `liftRBF()` uses pre-allocated `mutable lift_psi_` instead of local `VectorXd psi(n_lifted_)`. |

**Non-obvious caveats from Part 57B:**
```
CEMController mutable workspaces  -> x_roll_/e_roll_ lazy-sized on first computeRef() call
                                      (state/output dims not known at construction time)
MHE CTPsi_ws_/H_eff_ws_ resize()  -> resize(dz,dz) is no-op in steady-state (dz == dz_max);
                                      ramp-up (first N steps) still reallocates as dz grows -
                                      this is correct and identical to original behaviour
KoopmanEDMD liftRBF vs liftPoly   -> both now share mutable lift_psi_; only one path is
                                      active per call (Dict::RBF dispatches to liftRBF,
                                      Dict::PolyDeg1/2 dispatches to liftPoly) - no aliasing
```

**Audit progress:** 17 (Iter A) + 13 (Iter B) = 30 findings closed. 63 remain (Iter C-E, mostly LOW/MED).
**Docs updated:** `docs/audit_report.md` Part 57B table appended; this section added.

---

## Part 57C - Audit Iteration C (Correctness Sweep) - 2026-06-14

**Iteration C complete.** 12 correctness findings checked (#9, #15-18, #22, #27-28, #30-33); 10 already resolved, 2 required code changes:

| Finding | Controller / File | Code change |
|---------|------------------|-------------|
| #22 MHE general `EigenSolver` on symmetric PD Hessian | `MovingHorizonEstimator` | `Eigen::SelfAdjointEigenSolver` at both `estimate()` call sites; avoids complex arithmetic on guaranteed-real eigenvalues |
| #33 `MismatchDetector::update(VectorXd)` zero-size undocumented | `MismatchDetector.h` | `@note` added explaining zero-size -> 0 fed to CUSUM (no alarm update) and `sqrt(max(p,1))` guards division-by-zero |

**Docs updated:** `docs/audit_report.md` Part 57C table appended; this section added.

---

## Part 57D - Audit Iteration D (LOW-severity batch) - 2026-06-14

**Iteration D complete.** 9 LOW-severity findings checked (#44, #45, #47, #50, #51, #53, #56, #58, #60); 5 already resolved/deferred, 4 required code changes:

| Finding | Controller / File | Code change |
|---------|------------------|-------------|
| #44 `ComputationalDelayWrapper::lastOutput()` naming ambiguity | `lib/ComputationalDelayWrapper.h` | Expanded docstring to clarify "last returned" == "next to be returned" equivalence under one-step delay semantics |
| #45 `EchoStateNetwork::extendedState()` dead private method | `lib/EchoStateNetwork.{h,cpp}` | Removed declaration + definition; method was never called after Part 31 |
| #51 `UnscentedKalmanFilter::sigmaPoints()` allocs n*(2n+1) per call | `lib/UnscentedKalmanFilter.{h,cpp}` | Added `mutable Eigen::MatrixXd sigma_pts_ws_` (sized in constructor); `sigmaPoints()` writes into it and returns `const Eigen::MatrixXd&`; callers in `predict()`/`update()` bind by const ref - eliminates two matrix allocs per `step()` |
| #56 `findEquilibrium()` silent 1000* tolerance fallback | `lib/AutoGainScheduler.h` | Added `#ifndef NDEBUG` `std::clog` warning when the `tol * 1e3` fallback fires; added `#include <iostream>` |

**Non-obvious caveats from Part 57D:**
```
UKF sigma_pts_ws_          -> mutable workspace; sigmaPoints() returns const&;
                               callers must bind const Eigen::MatrixXd& (not by value)
DiscreteADRC #58 deferred  -> runtime assert for "setReference() never called"
                               would fire spuriously in ControllerStack (r_=0 intentional);
                               @warning in header docstring is the correct guard
```

**Audit progress:** 51 total findings closed across Iterations A-D. 42 remain (Iter E).
**Docs updated:** `docs/audit_report.md` Part 57D table appended; this section added.

---

## Part 57E - Audit Iteration E (Final Batch) + Distribution Track - 2026-06-14

**Audit Iterations A-E complete: all 84 findings addressed.** Iteration E closed the
remaining 42 findings: 13 required code changes, 29 verified already-resolved or
deliberately deferred (documented, not silently dropped).

**Distribution track (DIST-1/2/4/5) shipped in the same part:**
- **DIST-1** - CMake install target: `controller_toolbox` exports as
  `ctrl::controller_toolbox` after `find_package(ControllerToolbox)`; vcpkg portfile added
  (SHA512 placeholder until first tagged release).
- **DIST-2** - Embedded header-only subset: `lib/embedded/` (`BasicPID.h`, `BasicSMC.h` +
  the Part 54 templates) builds with zero Eigen dependency via
  `CTRL_BUILD_EMBEDDED_ONLY`; `test_embedded_subset` links only Catch2.
- **DIST-4** - PyPI wheels via `cibuildwheel`; `CTRL_FETCH_EIGEN_IF_MISSING` gates Eigen
  auto-fetch (OFF by default, ON only in CI wheel builds).
- **DIST-5** - GitHub Release workflow (`release.yml`, softprops action pinned to a SHA).
- **publish.yml** requires PyPI "Trusted Publisher" configured for the repo.

**Docs updated:** `docs/audit_report.md` Part 57E table appended; `CLAUDE.md` Open Items.

---

## Part 58 - Analysis Pipeline (ANA-1..7 + RPT-1) + Test Fixes - 2026-06-15

**New `tools/` analysis pipeline, 7 modules + 1 report generator:**
- **ANA-1** `tools/metrics.py` (`compute_metrics`, `extract_final_iae`) + `compare_controllers.py`.
- **ANA-2** `tools/monte_carlo.py` (calls `sim.run_single(ctrl, params)`) + `mc_plots.py`.
- **ANA-3** `tools/fault_injector.py` (`FaultSpec`/`FaultInjector`) + `fault_sweep.py`
  (calls `sim.run_with_fault(ctrl, FaultSpec)`) + `fault_plots.py`.
- **ANA-4** `tools/anova.py` (one-way ANOVA + Tukey HSD via scipy/statsmodels).
- **ANA-5** `tools/wcet_report.py` (aggregates `wcet_*.csv`; p99 + WCET-at-99.9th-percentile).
- **ANA-6** `tools/model_validation.py` (`GreyBoxEstimator` fit + NRMSE; requires a study's
  own `grey_box_model()` hook).
- **ANA-7** `tools/mu_analysis.py` + `mu_plots.py` (ARMA(2,2) ID from logged closed-loop CSV;
  peak |S(z)|/|T(z)|).
- **RPT-1** `tools/generate_report.py` - self-contained HTML report, 8 sections, Plotly CDN
  interactive charts, degrades to plain tables without `plotly` installed.

**Bug fixes in the same part:**
- `RepetitiveController::setParams()` didn't reset `v_now_` when `periodSteps` changed -
  `correction()` kept returning a stale value across a period change.
- A `computeDoM` test measured the wrong signal (plant output `y` instead of peak control
  signal `u`); renamed and corrected.

**Non-obvious caveats:** `monte_carlo.py`/`fault_sweep.py`/`model_validation.py` each
require a specific hook in the study's `sim/main.py` (`run_single`, `run_with_fault`,
`grey_box_model` respectively) - see `tools/study_protocol.py` for the full contract.
`wcet_report.py` prints an instrumentation guide when no `wcet_*.csv` is found instead of
failing silently.

**Docs updated:** `CLAUDE.md` Open Items.

---

## Part 59 - Cross-Platform Scripts + Case Study Tracker - 2026-06-15

- **PLT-1** `setup.sh` (Linux/macOS bootstrap mirroring `setup.ps1`) + `compile.sh`
  (sequential full build mirroring `compile.bat`, same 120-target order). Both staged
  `git update-index --chmod=+x` so they land executable on a Linux clone.
- **TRK-1** `tools/case_study_tracker.py` completed from a stub: fixed `detect_language()`
  (extension-comparison bug, division-by-zero), added `detect_status()`
  (Complete/On-going/Incomplete/Not started - the 3-tier predecessor to Part 62's 4-tier
  rewrite), added `find_pdf_link()`/`find_readme_link()`, added `main()` writing
  `docs/case_study_status.md`. First run found 23 studies (16 On-going, 7 Incomplete) and
  **discovered 5 previously undocumented stubs** with PDF+README but no `sim/`: 6-DOF
  Stewart, Heavy-Duty Hydraulic VDC, Hybrid Tendon-Pneumatic, Underwater Manipulator, USV
  Wave-Predictive.

**Non-obvious caveats:** `setup.sh`'s eigen3 check is a non-fatal warning (falls back to
`CTRL_FETCH_EIGEN_IF_MISSING=ON`); `compile.sh --no-config` skips cmake re-configure and
assumes `build/` already exists; `case_study_tracker.py` must be run from the repo root
(`ROOT = "case-study"` is a relative path).

**Docs updated:** `docs/case_study_status.md` generated; `CLAUDE.md` stub tables updated.

---

## Part 60 - ROS2 Thin Wrapper + CI/CD Overhaul + Cross-Platform `run.py` - 2026-06-16

- **DIST-3** `ros2/ctrl_toolbox_ros2/` - `ament_cmake` package; `ControllerNode<T>`
  lifecycle-node template (header-only) wrapping any `ctrl::IController`. Topics:
  `~/setpoint`, `~/measurement`, `~/control_output` (all `std_msgs/Float64`); parameter
  `sample_time_s` (default 0.01s). Example: `pid_temperature_node.cpp`.
- **CI/CD overhaul:** 8 workflow files consolidated to 3 (`documentation.yml`,
  `benchmarks.yml`, `cross-platform-cicd.yml` covering linux/windows/macos/wheels/publish).
  Tag-triggered jobs gated on `startsWith(github.ref, 'refs/tags/v')`.
- **Bug fix:** `tests/test_embedded_subset.cpp` had 7 stale API calls against the Part 54
  `BasicPID`/`BasicSMC` API (single-arg constructor, `Ts` in Params, `sp.K` not `sp.eta`).
- **`run.py` made cross-platform:** bash on Linux for Phase 2, `conda run` for cmake in
  Phase 3, executable-detection in Phase 4 now handles no-extension + executable-bit
  binaries on Linux.

**Non-obvious caveats:** `ControllerNode<T>` uses `compute(setpoint - measurement)` - MRAC/
L1 controllers need an adapter; declared ROS params must come after `sample_time_s` is
already read in `on_configure`; `ros2/ctrl_toolbox_ros2` colcon build needs
`CMAKE_PREFIX_PATH` pointing at the `ctrl_toolbox` install prefix; no `.so`/`.dll` - header-
only, `ControllerNode<T>` instantiates in the user's own translation unit.

**Docs updated:** `CLAUDE.md` Open Items.

---

## Part 61 - 6-DOF Stewart Platform Vessel Motion Simulator - 2026-06-18

**New C++ case study** (`case-study/6-DOF Stewart Platform Vessel Motion Simulator/`,
target `stewart_sim`): 6-UPU hexagonal-hinge Stewart platform, closed-form inverse
kinematics + velocity Jacobian, 12-state per-rod spring-mass-damper actuator dynamics
(RK4, Ts=5ms). 12 controllers x 60 Douglas sea-state configs = 720 runs (largest run
count of any case study in the repo).

- **Architecture finding:** hinge geometry needs different base/platform half-angles
  (`delta_base=5^\circ`, `delta_platform=45^\circ`) - equal half-angles make every leg a pure
  rotation of every other leg, a genuine Jacobian singularity for vertical load at the
  home pose (`det(J^T)` collapsed to ~1e-49 for every phase/delta combination tried with
  equal half-angles, verified empirically).
- **Load-coupling sign bug caught before merge:** `F_load = -J^T.solve(wrench)`, not
  `+J^T.solve(wrench)` - verified against a vertical 1-rod example. Getting this backwards
  silently doubles the effective disturbance, producing 250-750mm tracking error that looks
  like a tuning problem but isn't.
- **Library bug fix** (`lib/NeuralPID.cpp`): `forward()` used a naive `log1p(exp(x))`
  softplus instead of the class's own numerically-stable two-branch `softplus()` helper -
  large `Kp0`/`Ki0` seeds (needed to match the other controllers' gain scale) overflowed to
  `inf`/`NaN`. One-line fix; zero behavioural change elsewhere in the repo.
- **L1AdaptiveController architectural ceiling documented, not chased:** `L1Adaptive` (like
  `MRACController`) is a relative-degree-1 SISO law; the rod plant is relative-degree-2
  (force -> position through a spring-mass-damper). An extensive gain sweep plateaus around
  115-190mm steady-state rod error regardless of tuning - accepted as architectural. MRAC
  reaches ~10mm with high gain via its direct algebraic law.

**Docs updated:** `tests/test_stewart_regression.cpp` added; `CLAUDE.md` roster + gotchas.

---

## Part 62 - `case_study_tracker.py` 4-Tier Rewrite + Documentation Reconciliation - 2026-06-18

- **TRK-2:** `detect_status()` rewritten from the Part 59 3-tier scheme (gated on file
  existence only) to 4-tier (Complete/On-going/Open placeholder/Not started). Root cause: a
  freshly-scaffolded `tools/new_case_study.py` study's placeholder plant (`x' = -a*x + b*u`)
  and `OpenLoop` controller actually run and write real `logs/*.csv`, so file-existence
  checks alone mis-classified untouched scaffolds as "On-going." New
  `_is_untouched_scaffold()` matches literal template strings from the generator against
  the study's actual plant/controller files. Re-scanning all 31 `case-study/*/` directories
  (up from 23 in Part 59) produced 18 On-going, 7 Open placeholder, 6 Not started.
- **C2-NEW finding:** of 8 newly-discovered directories, `Aircraft Engine Thermal
  Management` turned out to be a real, substantially-implemented study (not a placeholder),
  flagged for promotion in Part 63. The other 7 are untouched scaffolds.
- **DOC-1:** `docs/PROJECT_MASTER_STATE.md` reconciled - had drifted ~12 Parts behind
  CLAUDE.md. Stale references fixed: `prompt/prompt_enhanced.txt` (removed, doesn't exist),
  `docs/audit_report.md`/`docs/roadmap_deployment_frontend.md` (moved to `docs/archived/`),
  missing mentions of `scripts/`/`benchmark/`/`cheatsheet/`.

**Docs updated:** `docs/case_study_status.md` regenerated; `docs/PROJECT_MASTER_STATE.md`
Section 3 replaced with a pointer to the auto-generated tracker instead of a hand-
maintained case-study tree; `CLAUDE.md` Open Items.

---

## Part 63 - Aircraft Engine Thermal Management Promoted into Official Roster - 2026-06-18

**C2-NEW closed.** `Aircraft Engine Thermal Management` (discovered Part 62) added to
`README.md`'s and `CLAUDE.md`'s Python-only roster tables (17 -> 18 studies, 7 -> 8
Python-only). Pure documentation change - the study was already fully implemented (real
FTMS plant + 12 controllers + 5 scenarios) and already auto-discovered by `run.py` Phase 6;
no `lib/` or binding changes needed.

Four tribal-knowledge gotchas added to `CLAUDE.md` from this study's implementation:
state-choice `(m1, m2)` (not the paper's own `(m0, m1)`, which traps every SISO controller
at a shared-resource lock once both loops saturate simultaneously), negative-static-gain
sign convention, the safety-supervisor wrapper pattern (overrides `(u1, u2)` around the
controller rather than inside the plant), and `m1_min`/`m2_min` crossover-flow margins.

**Docs updated:** `CLAUDE.md` roster tables + gotchas; `README.md` intro/status baseline.

---

## Part 64 - CSV `error`/`iae_cumulative` Columns + C++ Robustness Analysis for 3 Case
Studies - 2026-06-19

- **CSV schema:** `Active Suspension Mathematical Modeling and Optimization 2025`,
  `Porous Fiber Plate Humidification System`, and `Solar-Driven Cooling System with
  Photovoltaic Evaporative Chimney` each gained trailing `error`/`iae_cumulative` columns -
  the exact names `tools/metrics.py`'s `extract_final_iae` already searches for.
- **New `case-study/common/RobustnessStats.h`** (header-only, shared by all three, the
  first file under a new `case-study/common/` directory): `FaultSpec`/`FaultKind` mirroring
  `tools/fault_injector.py`'s `FAULT_KINDS` 1:1, `MetricStats`/`computeStats()` (mean/std/
  percentiles/worst, no Eigen dependency), `SimSummary` common per-trial result type.
- **New `robustness_main.cpp` + `*_robustness` CMake target per study** (`susp_robustness`,
  `humidification_robustness`, `solar_cooling_robustness`), purely additive. Each runs WCET
  (per-step `std::chrono` timing -> `wcet_summary.csv`), Monte Carlo (30 samples/controller,
  +-15% Gaussian perturbation of real physical plant params, rerunning the actual nonlinear
  closed-loop sim - deliberately NOT `lib/RobustnessAnalysis.h`'s linearized-`StateSpace`
  approach, since that "isn't meaningful for the SMC/ADRC/Fuzzy/GA-tuned nonlinear
  controllers most case studies actually use"), and a fault sweep (3 magnitudes per
  applicable `FaultKind`, injected at 40% through a truncated analysis window). Output
  files (`mc_summary.csv`/`fault_sweep.csv`/`wcet_summary.csv`) land at the study root,
  matching the exact filenames `tools/generate_report.py` already reads.
- **`tools/mu_analysis.py` fix:** added `phi_measured`/`Tw1_C` to `y_candidates` and
  `u_fan_ms`/`m_dot_w_kgs` to `u_candidates` - these 3 studies' real column names weren't in
  the hardcoded candidate lists, so mu analysis silently fell back to a degenerate
  error-as-proxy path (or `status=no_columns` before this part added the `error` column).

**Docs updated:** `CLAUDE.md` ROB-1 entry + Open Items; this report's header table.

---

## Part 65 - Enabled `run_analysis.py` by Default + Fixed Multi-Study Analysis Bugs -
2026-06-19

- **`run.py` Phase 7 Step 2 flipped from opt-in to opt-out:** previously required
  `CTRL_RUN_ANALYSIS=1` to run `tools/run_analysis.py`; now runs by default every session,
  set `CTRL_SKIP_ANALYSIS=1` to opt back out.
- **Real bug found while turning this on:** `tools/run_analysis.py`'s `load_sim_module()`
  had a `sys.modules` cross-study collision, only surfaced once analysis ran across
  multiple studies in one process. Every case study's `sim/` package reuses the same
  generic file names (`controllers.py`, `simulation_runner.py`, ...); Python caches imports
  by short module name, not path, so the *second* study processed would silently get the
  *first* study's already-cached `controllers` module - wrong physics, usually surfacing as
  a `KeyError` swallowed by each study's own defensive fallback (looked like "no hooks
  found," not an import bug). Confirmed empirically: 5 of 7 studies failed this way
  depending on processing order. Fixed via a `cleanup()` closure that removes the study's
  `sim/` `sys.path` entry and purges only `sys.modules` entries whose `__file__` lives under
  that study's `sim/` directory (not third-party packages - an indiscriminate first attempt
  broke numpy's C-extension reload guard).
- **3 stale `nominal_scenario_id` values fixed** in `config/analysis.json` (Air-Cooled BTMS,
  Nonlinear Surface Ship, EHFS) - same root-cause pattern as a prior EV6x6 fix: a scenario
  got renamed and the analysis config was never updated, so every analysis call raised
  `ValueError` and filled `mc_summary.csv`/`fault_sweep.csv` with `iae=nan` rows.

**Docs updated:** `CLAUDE.md` Part 65 entry + non-obvious facts.

---

## Part 66 - ROB-1 Extended to All 10 C++ Case Studies + `generate_report.py` Empty-Data
Crash Fix - 2026-06-20

- **ROB-1 extended from 3 to 10 C++ case studies.** The Part 64 `RobustnessStats.h` /
  `robustness_main.cpp` / `*_robustness` CMake-target pattern replicated for `Boiler
  Control`, `Tug Boat Numerical Simulation`, `Non-Inverting Buck-Boost Converter`,
  `Solar Cooker with Reflector and Absorber`, `Solar Ocean Thermal Energy Conversion
  System`, `Separate Meter In Separate Meter Out`, and `6-DOF Stewart Platform Vessel
  Motion Simulator`. All 7 new targets built and run - `docs/case_study_status.md` now
  shows all 10 C++ studies as `Complete`.
- **Bug found and fixed - `tools/generate_report.py:148-150` `_section_comparison()`
  raised `KeyError`** on any study with zero parseable run rows (every "Open placeholder"/
  "Not started" scaffold: Bouyancy-Driven Airship, Differential Drive Robot Tracking,
  Dual-Arm IAUV Motion Planning, Residential Building Comfort SMPC, Underwater Glider
  Trajectory Tracking). The function checked `if not _HAS_PLOTLY or df.empty:` but then
  still indexed `df[["study","scenario","controller","iae"]]` on that same line before ever
  reaching `_df_to_html_table`'s own correct `df.empty` guard. Confirmed firing identically
  across two independent `generate_all_reports.bat` runs (2026-06-18 and 2026-06-19, logged
  in the gitignored `tools/generate_all_reports.log` - silent every time because the `.bat`
  loop continues past a failed `generate_report.py --study ...` call and the log has no
  tracked trace). Fixed by returning `_df_to_html_table(df)` immediately when `df.empty`,
  before any column selection. Verified with a direct repro against an empty `DataFrame`.
- **Aircraft Engine Thermal Management wired into the WCET pipeline:** `sim/main.py`
  gained `run_wcet_profile()`; `sim/simulation_runner.py`'s `run_simulation()` gained an
  optional `wcet_sink: list` parameter. New `config/analysis.json` brings this study in
  line with every other Python study's `tools/run_analysis.py` hook contract.
- **`tools/case_study_tracker.py`** excludes the new `case-study/common/` directory from
  being scanned as a case study.
- **`tools/mu_analysis.py`** reordered `y_candidates`/`u_candidates` so specific column
  names are checked before short generic fallbacks (`y`/`u`/`y1`/`u1`) - Tug Boat's own
  state variables are literally named `u`/`v` (surge/sway velocity), which was shadowing
  the real output/actuator columns under the old ordering.
- **Stray `docs/report.html`** (1653 lines, tracked since the initial commit) removed - a
  misplaced combined report from a `generate_report.py` run where `--study` didn't resolve
  to exactly one study (falls back to writing `report.html` into the cwd).
- **Two case-study-local hand-off planning docs added** (plans only, no implementation):
  `case-study/Bouyancy-Driven Airship in Vertical Plane/HANDOFF_PROMPT.md` and
  `case-study/Hybrid-Driven Tendon-Pneumatic Soft Manipulator/HANDOFF_PROMPT.md`.

**Docs updated:** `CLAUDE.md` Part 66 entry + Open Items; `docs/PROJECT_MASTER_STATE.md`
reconciled to Part 66; `docs/ALGORITHM_ROADMAP_PHASE2.md` status line corrected; this
report's header table + section.

---

## Part 67 - Debug-Build (`-g`) Toolchain Limitation Found While Adding ResonantController/
NotchFilter/PhaseLockedLoop - 2026-06-24

While verifying the new `ResonantController`/`NotchFilter`/`PhaseLockedLoop` classes
(`docs/superpowers/specs/2026-06-24-resonant-notch-pll-controllers-design.md`) under
`-DCMAKE_BUILD_TYPE=Debug`, two **pre-existing, unrelated** failures surfaced - confirmed
not caused by the new code:

- **`lib/DiscreteHinf.cpp` fails to compile with `-g`** (any level, including `-g1`) on this
  MSYS2 UCRT64 `g++` toolchain. `-O0` alone (no `-g`) compiles fine. The compiler exits 1
  with **no diagnostic text on stderr** - consistent with the GCC/MinGW DWARF debug-info
  emission pass choking on heavy Eigen template instantiation in a large translation unit,
  not a real code defect. Confirmed pre-existing by directly recompiling the file in
  isolation; the file was not touched by this session's changes.
- **`tests/test_catch2_advanced.cpp` fails the same way under `-g`.** Confirmed pre-existing
  (not caused by the 16 new test cases added this session) by extracting the file's content
  from immediately before this session's changes (commit `ccc1134`) and recompiling that
  exact pre-change version with the same flags - it fails identically.
- **Separately, linking `test_catch2_advanced.exe` under Debug (`-O0`, no `-g`) intermittently
  fails with `collect2.exe: error: ld returned 5 exit status`.** Windows error code 5 is
  `ERROR_ACCESS_DENIED` - consistent with a transient antivirus/file-lock race on the
  freshly-written `.exe`, not a real link error: the output binary is present at the correct
  size immediately after the reported "failure," and simply re-running the link (or just
  running the already-produced executable) succeeds.

**Net effect:** none of this blocks the new classes - all three compile cleanly individually
under `-g`, and the full Release build (`-DCMAKE_BUILD_TYPE=Release`) is unaffected (368/368
`ctest` cases pass, smoke test passes, all three new examples PASS). `CLAUDE.md`'s canonical
build/test commands are Release-only already; this is the first session to have exercised
`-DCMAKE_BUILD_TYPE=Debug` against this codebase, surfacing a toolchain limitation that
predates this work and was simply never triggered before.

**Not fixed - flagged for a future design iteration, not this session:** root-causing GCC's
`-g` DWARF-emission failure on Eigen-heavy translation units (likely requires a GCC version
change, splitting `DiscreteHinf.cpp`/`test_catch2_advanced.cpp` into smaller TUs, or different
debug-info flags) is a separate, materially larger investigation than the scope of adding
three new `lib/` classes. Workaround for anyone who needs a Debug build meanwhile: configure
with `-DCMAKE_CXX_FLAGS_DEBUG=-O0` to get Debug optimization semantics without `-g`, and retry
the link step if it hits a transient `ld returned 5`.

**Docs updated:** this report's header table + new Part 67 section.

---

## Part 68 - Algorithm Roadmap Phase 3, Phase 2 (7 items: OC1/SI1/EF2/EF3/MO1/MO3/DT4) - 2026-06-25

All 7 Phase 2 roadmap items implemented end-to-end (lib + bindings + Catch2 tests + C++/Python
examples + build wiring): `SelfTuningRegulator` (OC1, merges minimum-variance control/adaptive
pole placement/self-tuning regulators), `MLEIdentifier` (SI1, Gaussian/Laplace batch ID),
`SetMembershipEstimator` (EF2, ellipsoidal outer-bounding), `ParticleFilterV2` (EF3, Auxiliary +
Rao-Blackwellized variants - required making `ParticleFilter::predict/update/step/resample`
virtual), `NSGA2` (MO1, Pareto multi-objective), `tuneConstrained` (MO3, exterior-penalty
wrapper), `FaultClassifier`/`FTCSupervisor` (DT4, FDI + `ControllerStack` reconfiguration). See
`docs/superpowers/specs/2026-06-25-*-design.md` for the 4 design specs. Several real bugs
surfaced during implementation/verification, all fixed in this same session:

- **Bug found and fixed - dangling reference in `SelfTuningRegulator::estimatedNumerator()/
  estimatedDenominator()`.** Both returned `const Eigen::VectorXd&` bound to
  `RecursiveLeastSquares::numerator()/denominator()`, which return **by value** (computed
  slices, not member references) - caught via `-Wreturn-local-addr`. Fixed by returning
  `Eigen::VectorXd` by value.
- **Bug found and fixed - `SelfTuningRegulator` cold-start deadlock.** `RecursiveLeastSquares`'s
  `theta_` is zero-initialized, so the identified `b1`/`r0` is exactly 0 on the first call,
  hitting the "ill-conditioned -> hold `uPrev_`" guard; since `uPrev_` itself starts at 0, the
  plant is never excited and `b1` never leaves 0 - a permanent deadlock. Fixed by adding
  `fallbackProportional()` (a small proportional law on `r_ - yHist_(0)`) used by all three
  ill-conditioning guards instead of holding `uPrev_`.
- **Significant finding, not a bug - certainty-equivalence STR has no general persistent-
  excitation guarantee from closed-loop reference alone** (Astrom & Wittenmark, *Adaptive
  Control*, Ch. 3/7): both control-law modes can converge confidently (shrinking
  `covariance()`) to a *stabilizing but numerically wrong* parameter estimate. Verified via
  isolated diagnostics that the underlying RLS and Diophantine-solve math are both correct in
  isolation (instant convergence given true parameters or open-loop excitation) - the residual
  tracking error is the textbook closed-loop-identifiability limitation, corroborated by the
  roadmap's own (separately scoped, Phase 5) OC3 Dual Control item. Added `STRParams::
  probeAmplitude`/`probeSeed` persistent-dither support (helps in practice but is not a
  guarantee) and rewrote the 2 affected Catch2 tests + `ex101`/`ex118` to assert
  stability/boundedness through a mid-run plant change rather than exact parameter/setpoint
  convergence. Documented as class-level `@warning`s on `SelfTuningRegulator.h`.
- **Bug found and fixed - deadbeat (poles-at-origin) `desired_poles`/minimum-variance targets
  are fragile during the online-identification transient.** Both control laws divide by an
  identified leading coefficient; an aggressive deadbeat design amplifies identification-
  transient error into saturating control with the default `+/-1e9` actuator bounds, which feeds
  a corrupted (clipped) data point back into RLS. Fixed the 3 affected tests + `ex101`/`ex118`
  to use comfortably-damped `desired_poles` and realistic `uMin`/`uMax`; documented as a
  class-level `@warning`.
- **Bug found and fixed - `FaultClassifier::classify()`'s actuator/sensor discriminator
  conflated "no information" with "low correlation."** `corr(du_cmd, dy_meas)` defaulted to
  `0.0` whenever `duStd`/`dyStd` fell at or below the `1e-12` div-by-zero floor, which silently
  satisfied `|corr| < corr_threshold` and misclassified a perfectly healthy but momentarily
  quiescent closed loop as `ActuatorLoss`/`SensorNoise`. Fixed by gating the correlation check
  on `duStd` alone (no command variation -> nothing to correlate against either way -> fall
  through to the residual-amplitude sensor-fault check) and deciding the genuine broken-causal-
  link case directly (`duStd` meaningfully nonzero but `dyStd` collapses to ~0 -> `ActuatorLoss`,
  without dividing by a near-zero `dyStd`). Verified the existing `[fault_classifier]` Catch2
  cases (including the literal broken-causal-link `ActuatorLoss` signature, which exercises
  exactly the `dyStd~0` path) still pass unchanged.
- **Bug found and fixed - `FTCSupervisor::compute()` reconfigured `ControllerStack` on *any*
  fault-type change, including transient classifications with no registered response.**
  `ex107_ftc_supervisor`'s redundant-sensor-pair scenario hit this directly: once the primary
  sensor developed a bias, `FaultClassifier`'s small-sample (`confirm_window=5`) correlation
  estimate legitimately flickered between `SensorBias`/`SensorNoise`/`ActuatorLoss` as the
  still-settling 2-controller PID loop's residual statistics varied - and `compute()` was
  deactivating *every* registered stack entry on each flicker into an unregistered fault type
  (freezing the output), then bumplessly re-engaging against a further-diverged error once the
  classification flickered back. This compounded every cycle into a runaway divergence (output
  reached `~1e6` by step 190 in the diagnostic repro). Root-caused via an instrumented copy of
  `ex107` tracing `(yPrimary, yBackup, residual, fault, active, u)` per step, isolating the
  freeze/re-engage cycle before inspecting `FaultClassifier`'s internals with temporary debug
  prints. Fixed by only reconfiguring when the newly classified fault type has a *registered*
  response - an unregistered/transient classification now leaves the stack's current
  configuration untouched (mirrors `ControllerStack`'s own "hold last output when nothing
  eligible" philosophy). Updated `ex107`/`ex124`'s final assertions to check the reliably-true
  property (fault detected at least once, active controller correctly latched on the backup,
  bounded trajectory) instead of the literal last-step classifier output, which remains
  expected to flicker by design of a small-sample per-step heuristic.
- Verification: all 37 new Catch2 test cases (2429 assertions) pass; full `test_catch2_advanced`
  suite (7564 assertions, 344 cases) passes with no regressions; all 7 new C++ examples
  (`ex101`-`ex107`) compile and PASS.
- **Python-binding verification (separate build step) surfaced two more bugs:**
  1. *Wrong CMake Python interpreter.* A first bindings build configured outside the conda env
     picked up MSYS2's system `python3.exe` (3.14), producing a `ctrl_toolbox.cp314-*.pyd` that
     cannot be imported by the canonical `soft_robotics` env (Python 3.12, MSVC CPython). Fixed by
     reconfiguring the `build` dir entirely *within* `conda run -n soft_robotics` (per `run.py`
     Phase 3's own pattern) so pybind11 discovers the conda interpreter **and** `python312.lib`
     consistently - yielding an importable `cp312-win_amd64.pyd`. (Not a library bug; a build-env
     gotcha worth recording - the MinGW UCRT64 toolchain still cross-imports fine into the MSVC
     conda CPython thanks to the static libstdc++/libgcc link, exactly as CLAUDE.md Section 2 notes.)
  2. *`ss_step_copy` tuple misuse in 3 Python examples.* `ex122_nsga2.py`, `ex123_constrained_
     tuning.py`, and `ex124_ftc_supervisor.py` wrote `y = ctrl.ss_step_copy(ss, x, uv)[0]`,
     treating element `[0]` as the scalar plant output. `ss_step_copy` actually returns the tuple
     `(y_vec, x_next)` (output **first**, next state second; both `np.ndarray`), so `[0]` was the
     output *vector* - which then (a) got passed where a scalar `float` was expected (`compute()`
     / `feed_residual()` raised `TypeError`) and (b) meant `x` was never reassigned, so the plant
     state never advanced. These were latent because the examples had never run against a built
     `.pyd`. Fixed to the canonical idiom used everywhere else (`y_vec, x = ctrl.ss_step_copy(...)`
     then `y = float(y_vec[0])`); all three now PASS, with `ex124` reproducing the C++ `ex107`
     trajectory exactly (latches `backup_sensor_pid`, final `y = -22.8183`).
- Python verification result: `bindings/smoke_test.py` passes (all bound Phase 2 classes
  importable); all 7 new Python examples (`ex118`-`ex124`) PASS under `conda run -n soft_robotics`.

**Docs updated:** `docs/algorithm_backlog.md` (7 items moved to "Already done");
`docs/ALGORITHM_ROADMAP_PHASE3.md` status table (16/32 shipped, Phase 1 + Phase 2 complete);
this report's header table + new Part 68 section.

## Part 69 - Algorithm Roadmap Phase 3, Phase 3 (4 items: ML1/ML2/NC3/SI4) + Local Windows
Binding-Build Diagnosis - 2026-06-25

4 Phase 3 items implemented end-to-end (lib + bindings + Catch2 tests + C++/Python examples +
build wiring), deliberately chosen as the lowest compiler-interaction-risk subset of the 7 open
Phase 3 items - all four are **new files only** (`NeuralNetworkController`, `NNAdaptiveController`,
`NonlinearIMC`, `NARMAXIdentifier`), so none of them edit an existing hot class; FD2 (complex-pole
bookkeeping), SI3 (edits the shared `SubspaceID` free function), and ML3 (inherits the complex
`NonlinearMPC`) were skipped as higher-risk:

- `NeuralNetworkController` (ML1) - generic feedforward NN, fixed forward pass (arbitrary depth,
  Tanh/ReLU/Sigmoid/Linear/Softplus per layer), reusing `NeuralPID`'s activation forms. Exposes
  `protected hiddenFeatures()` (runs all layers except the last) specifically so `NNAdaptiveController`
  can reuse the forward-pass machinery without duplicating it.
- `NNAdaptiveController` (ML2) - inherits `NeuralNetworkController`; adapts only the output layer
  online via a Lyapunov gradient + sigma-modification law (mirrors `MRACController`'s sigma
  convention exactly: `theta[k+1] = theta[k] - Ts.(gamma.e_m.phi + sigma.theta)`). Requires the
  base network's input width to be exactly 2 (`[y_m - y, r]` convention) and the output layer to
  be `Linear` - both enforced as constructor `std::invalid_argument` checks.
- `NonlinearIMC` (NC3) - nonlinear analogue of `SmithPredictor`'s model-in-the-loop structure:
  parallel one-step `ModelFn`/`InverseModelFn` pair, IMC filter, mismatch feedback
  (`s = e + y_model`). Falls back to hold-last on a non-finite/singular inverse-model result.
- `NARMAXIdentifier` (SI4) - polynomial NARMAX via Orthogonal Forward Regression (Error Reduction
  Ratio term selection, Billings & Korenberg); Extended Least Squares residual-refinement pass when
  `nc > 0`. Static methods only (no `IController` base), which sidesteps the `std::shared_ptr<T>`
  binding-holder rule entirely.

**Verification (all green):** `bindings/smoke_test.py` (all 4 new classes, plus the full
pre-existing suite); full Catch2 suite - **358 test cases, 7592 assertions, 0 failures**
(`[neural_network_controller]`/`[nn_adaptive_control]`/`[nonlinear_imc]`/`[narmax]` tags: 14 cases,
28 assertions); all 8 examples (4 C++ `ex108`-`ex111` + 4 Python `ex125`-`ex128`) PASS.

**Bug found and fixed - smoke-test-only, not an implementation bug.** The `NNAdaptiveController`
smoke-test snippet built its test network with `n_input_features = 1`, violating the class's own
documented/enforced `[y_m - y, r]` 2-input contract; the constructor correctly raised
`std::invalid_argument` (caught via running the *actual* smoke test rather than just trusting the
C++ example, which used the correct 2-input shape throughout). Fixed by widening the smoke test's
hidden-layer `W` to `(3, 2)` and setting `n_input_features = 2`, matching the C++ example/Catch2
tests that were correct from the start.

**Local Windows binding-build issue diagnosed and fixed (machine-specific, not a code bug).**
Rebuilding `ctrl_toolbox` against the `soft_robotics` conda env surfaced a genuine link failure:
`collect2.exe: error: ld returned 1 exit status` with zero further diagnostic text reaching `make`'s
output (the real cause only appeared by extracting and re-running the failing `g++` link command
directly). Root cause: this `build/` directory's cached `PYTHON_EXECUTABLE` pointed at MSYS2
UCRT64's bundled `python3.exe` (3.14.6), not the conda env's Python (3.12.13) - `find_package
(Python3 ...)` (the modern finder, `bindings/CMakeLists.txt:15`) correctly found the conda
interpreter, but pybind11 v2.13.6's *legacy* `FindPythonLibs` (still in play because pybind11
explicitly sets `CMP0148 OLD`) independently searched system `PATH`/lib dirs and resolved
`PythonLibs` to `C:/msys64/ucrt64/lib/libpython3.14.dll.a` - a 3.14 import library linked against
a module built with 3.12 headers/interpreter. Confirmed MSYS2 UCRT64 has *no* 3.12 variant at all
(`ls /c/msys64/ucrt64/lib` shows only `libpython3.14.dll.a`), so the legacy finder had no correct
candidate to find. **Fix:** the conda env ships its own matching dev files
(`envs/soft_robotics/libs/python312.lib`, `envs/soft_robotics/include/Python.h`); pinning *both*
the modern and legacy CMake variable names to those files resolves both finders identically:
```
conda run -n soft_robotics -- cmake -S . -B build \
  -DPython3_LIBRARY="<conda_env>/libs/python312.lib" -DPython3_INCLUDE_DIR="<conda_env>/include" \
  -DPYTHON_LIBRARY="<conda_env>/libs/python312.lib" -DPYTHON_INCLUDE_DIR="<conda_env>/include" \
  -DCTRL_BUILD_PYTHON_BINDINGS=ON -DCMAKE_BUILD_TYPE=Release
```
MinGW `ld` links the MSVC-format `.lib` without issue (well-trodden combination for conda-on-Windows
+ MinGW). Considered and rejected two riskier alternatives (upgrading the conda env's Python to
3.14 to match MSYS2, or side-installing a pinned Python 3.12 into MSYS2 UCRT64 - the latter isn't
realistically supported since `pacman` tracks one rolling Python version per subsystem) per explicit
user direction to try the narrowest fix first.

**Separate, pre-existing issue found while diagnosing the above, since fixed in `run.py` and
`setup.ps1` per explicit user direction ("fix them before you or I forget about it"):** `run.py`'s
`phase_bindings()` and both `setup.ps1`/`setup.sh` hardcoded `-G Ninja` for their bindings-specific
`cmake` configure call. `compile.bat`'s own configure call (run earlier in the *same* `run.py`
invocation, Phase 2) passes no `-G` at all, so it silently inherits whatever generator is already
cached - `MinGW Makefiles` on this machine, the CMake default when MSYS2 UCRT64 GCC is on `PATH`
with no Visual Studio/Ninja forced. CMake refuses to switch an existing cache's generator without
wiping it, so `run.py`'s Phase 3 reliably failed with `CMake Error: Error: generator : Ninja /
Does not match the generator used previously: MinGW Makefiles` on any `build/` directory first
configured via CLAUDE.md Section 2's own documented manual recipe (`cmake -S . -B build
-DCMAKE_BUILD_TYPE=Release`, no `-G`) - exactly what the user's `run_20260625_150226.log` showed.
Confirmed independent of today's controller work (the cached generator predated this session) and
confirmed CI-unaffected (Windows CI never builds bindings at all - `has_python: false`; Linux/macOS
CI use a fresh `build_py` dir with a single-toolchain `apt`/`brew` Python, immune to both this and
the link issue above). **Fix applied to `run.py` and `setup.ps1`** (both Windows-relevant; `setup.sh`
left untouched - it and its Linux/macOS sibling `compile.sh` both already pass `-G Ninja`
consistently, so no clash exists there): drop the forced `-G`, letting the bindings configure step
inherit whatever Phase 2 (or a prior manual configure) already cached.

**Second bug found while scripting the Python-library-pin fix above into `run.py`/`setup.ps1`,
also fixed:** the first scripted attempt (`os.path.join(sys.prefix, 'libs', ...)` in Python;
native `"$pyPrefix\libs\..."` interpolation in PowerShell) reproduced the *exact* terse, contentless
`ld returned 1 exit status` failure from before - even though `linkLibs.rsp` now correctly listed
`python312.lib` instead of the mismatched `libpython3.14.dll.a`. Root cause: CMake bakes the pinned
path verbatim into `bindings/CMakeFiles/ctrl_toolbox.dir/linkLibs.rsp`, and GNU `ld`'s `@response-
file` parser treats backslashes as escape characters - a Windows-style `C:\Users\...\python312.lib`
silently loses every backslash, producing `ld.exe: cannot find C:Users...python312.lib: No such
file or directory` (this real message was being swallowed by both `cmake --build`'s output
buffering *and*, surprisingly, by Git-Bash's invocation of the linker directly - it only surfaced
by re-running the exact failing `g++`/`ld` command through native PowerShell with the `--%`
stop-parsing token). **Fix:** both scripts now build the pinned path with forward slashes
(`sys.prefix.replace('\\', '/')` in `run.py`; `.Replace('\', '/')` in `setup.ps1`) before handing it
to `cmake -D...`. **Verified end-to-end after both fixes**: `conda run -n soft_robotics -- python
-c "import run; run.phase_bindings()"` and `.\setup.ps1 -SkipCondaCreate` each independently
configure, build, and smoke-test `ctrl_toolbox` from a clean invocation - both exit 0, both report
"All smoke tests passed" including all 4 new Phase 3 classes.

**Docs updated:** `docs/ALGORITHM_ROADMAP_PHASE3.md` status table (20/32 shipped, Phase 1-3
complete for ML1/ML2/NC3/SI4; SI3/FD2/ML3 remain Open); this report's header table + new Part 69
section; `run.py`/`setup.ps1` patched as described above.

---

## Part 70 - Algorithm Roadmap Phase 3, Phase 4 (OC4: LPSolver + LPMPC) - 2026-06-27

OC4 (Linear-Programming-Based Control, second in the recommended Phase 4 order after OC2)
implemented end-to-end: `lib/LPSolver.h` (header-only two-phase simplex, Bland's rule, bounded
variables) and `lib/LPMPC.{h,cpp}` (SISO L1-cost linear MPC built on it), plus bindings,
`smoke_test.py`, 2 C++ + 2 Python examples (`ex118`/`ex119`, `ex135`/`ex136`), and 11 new Catch2
tests (`[lp_solver]` x5, `[lp_mpc]` x6). See
`docs/superpowers/specs/2026-06-27-lp-solver-lp-mpc-design.md` for the full design (including a
pre-implementation audit that corrected the roadmap's "extends `GradientProjectionQP`" claim -
verified false; `LPSolver` is a from-scratch simplex, since that QP solver has no general-
inequality machinery and is a first-order method that can't produce exact LP vertices).

**Four real numerical bugs found and fixed during local verification, none caught by code review
alone** - all reproduced via a textbook 2-variable LP (`maximize x1+x2 s.t. x1+2x2<=4, 3x1+x2<=6,
x>=0`, known optimum `(1.6, 1.2)`) and the LPMPC closed loop, before any of this shipped:

1. **Catastrophic cancellation from the box-row-augmentation design itself.** Every `LPProblem`
   variable's upper bound becomes an explicit inequality row (`y_i <= ub_i - lb_i`, the "shift to
   nonnegative" trick from the design doc). This toolbox's "unbounded" sentinel is `+-1e9`
   (`BacksteppingParams::uMin`/`uMax`, etc.) - injecting that literal magnitude into a row costs
   ~9 orders of magnitude of float64 precision to every Gauss-Jordan elimination step touching
   it. The textbook LP (trivially feasible, `x=(0,0)` alone proves it) came back `Infeasible`
   with `phase1_obj=1.19e-7` against `tol=1e-8` - a precision artifact, not a real infeasibility.
2. **The first fix (skip the box row entirely above a threshold) was wrong and reverted.**
   Skipping the row removes simplex's own per-variable safety net: a column with no explicit
   upper bound can hit "no eligible ratio-test row" (defensively read as `Unbounded`) when it's
   only *indirectly* bounded via another variable's cost trade-off - exactly `LPMPC`'s `DeltaU`
   when `duMin`/`duMax` are left at their default `+-1e9`. Confirmed by reproducing: the default-
   `LPMPCParams` Catch2 test passed with skipping but failed (false `Unbounded`-shaped behavior)
   once a second scenario exercised the default sentinel duMin/duMax.
3. **The real fix: clamp `lb`/`ub` to `+-1e6` *before* computing the shift, not just inside the
   box row.** The shift `y = x - lb` injects `lb`'s literal magnitude into *every* row referencing
   that variable's column, not just its own box row - capping only the box row's RHS left the
   shift itself contaminating the whole tableau. Root-caused via a standalone scratch harness
   reproducing `LPMPC`'s exact LP structure outside the class, printing the raw tableau: a
   trivially-feasible problem's Phase-1 "optimum" landed at `phase1_obj~=3e9` (matching `Nc=3`
   `DeltaU` columns each shifted by `~1e9`) - a wrong-answer bug, not rounding noise. Fixed by
   computing `lb_eff = lb.cwiseMax(-1e6)`, `ub_eff = ub.cwiseMin(1e6).cwiseMax(lb_eff)` (the
   second clamp guards the pathological case where the *entire* true interval sits beyond the
   cap on one side) and using these consistently everywhere a shift offset appears - the row
   RHS, the original inequality/equality rows' constant term, and the final un-shift
   `x = lb_eff + y_sol`. Documented as a deliberate scale limit: `LPSolver` is not intended for
   problems whose true optimal `x_i` needs `|x_i| > 1e6`.
4. **Phase-1 feasibility tolerance was too tight relative to ordinary pivot-count rounding.**
   Even after fix 3, a ~25-variable `LPMPC` instance (`Np=15`, `Nc=5`) left
   `phase1_obj=1.01e-8` after 65 pivots - *1% over* the default `tol=1e-8`, on a genuinely
   feasible problem. `tol` also governs the ratio-test cutoff and Phase-2 optimality check, where
   a tight value is exactly what a precision-seeking caller wants; decoupled the feasibility
   check onto `std::max(tol, 1e-6)` instead of literal `tol`, so a looser user `tol` is still
   honored while a floor protects the default.
5. **`LPMPCParams::lpMaxIter` default (200, matching `LPSolver`'s own default) was insufficient
   near a degenerate optimum.** Bland's rule (required for the cycle-free guarantee) converges
   more slowly than Dantzig's rule near tied/degenerate vertices, and L1-cost epigraph
   formulations are *structurally* prone to exactly that at their optimum (multiple `(t_y, t_u)`
   splits sharing the same cost at a kink). Caught as 51/1500 steps reporting `IterationLimit`
   at exactly 200 pivots once the closed loop settled near steady state, even though the
   practically-useful answer was already correct. Raised `LPMPCParams::lpMaxIter` default to 500.
6. **Not a bug - a genuine L1-MPC characteristic, "the deadzone."** Unlike QP/L2-cost MPC (which
   always takes an infinitesimal `DeltaU` for any nonzero gradient), an L1 cost only moves when
   the aggregate marginal benefit (`rho_y * sum` of the plant's per-step step-response
   coefficients across the horizon) clears the `rho_u` penalty; below that threshold the
   LP-optimal `DeltaU` is *exactly* zero, every step, forever. `ex119`'s plant (`G(s) =
   1/(s^2+1.5s+1)`, `Ts=0.01s` - same plant as `ex01_tf_pid.cpp`) has a tiny per-step
   sensitivity, so the first attempt at `rho_u=0.05` (a DiscreteMPC-style ratio) left the
   controller stuck at `u=0` forever despite `e=1.0` persisting - correctly optimal given those
   weights, not a solver fault. Fixed by retuning the example to `rho_u=0.001` (verified via a
   `rho_u` sweep: `0.05`/`0.001` -> stuck/`0.0001`/`1e-5`/`1e-6` -> track correctly) and documented
   as a class-level `@warning`-style note on `LPMPC.h`: rho_u typically needs to be 1-2 orders of
   magnitude smaller than a QP analogue would suggest. All 11 `[lp_mpc]`/`[lp_solver]` Catch2
   tests' chosen parameters were independently re-verified against this threshold and needed no
   changes (already comfortably in the "active" regime or testing a property - bound respect,
   NaN-guard, convergence flag - that doesn't depend on it).

**Verification:** full `test_catch2_advanced` suite (7781 assertions, 397 test cases) passes, no
regressions. Both new C++ examples PASS. `bindings/smoke_test.py` passes (built inside
`conda run -n soft_robotics` per the Part 68 Python-interpreter gotcha - this session hit the
*same* MSYS2-system-Python mis-detection again on the first configure attempt, plus a *second*,
independent stale-cache issue: re-running `cmake -B build` with `CTRL_BUILD_PYTHON_BINDINGS=ON`
inside `conda run` still picked up `PYTHON_EXECUTABLE` cached from an earlier failed attempt in
this same `build/` dir, requiring an explicit `-DPYTHON_EXECUTABLE=<env path>` plus `-U`-clearing
the stale `_Python3_*` internal cache entries to force fresh detection). Both new Python examples
PASS.

**Non-obvious facts added (Part 70):**
```
LPSolver bound scale     -> internally clamps to +-1e6; true optima outside that range are wrong
LPSolver feasibility tol -> max(tol, 1e-6), decoupled from the optimality/ratio-test tol
LPMPC rho_u tuning       -> 1-2 orders of magnitude smaller than a QP/DiscreteMPC analogue
LPMPC lpMaxIter default  -> 500, not LPSolver's own 200 (L1 epigraph degenerate-vertex slowdown)
LPMPC scope              -> SISO only; not a zero-allocation hot path (tableau built per call)
cmake -B build (same dir, 2nd attempt with bindings ON) -> stale PYTHON_EXECUTABLE cache can
  survive even inside conda run; pass -DPYTHON_EXECUTABLE explicitly + -U the _Python3_* keys
```

**Docs updated:** `docs/ALGORITHM_ROADMAP_PHASE3.md` status table (OC4 Open -> Done, 25/32
shipped); `docs/algorithm_backlog.md` (OC4 + the already-shipped-but-still-listed OC2 moved to
"Already done"); `docs/superpowers/specs/2026-06-27-lp-solver-lp-mpc-design.md` (new); this
report's new Part 70 section.
