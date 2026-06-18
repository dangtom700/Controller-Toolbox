# Controller Toolbox - Cumulative Bug Report (Part 51+)

**Active issues start at Part 51.** Earlier history is archived in two compact references:
- [`docs/compact_bug_report_parts_1-25.md`](compact_bug_report_parts_1-25.md) — Parts 1-25 (2026-05-19 through 2026-05-30)
- [`docs/compact_bug_report_parts_26-50.md`](compact_bug_report_parts_26-50.md) — Parts 26-50 (2026-05-31 through 2026-06-11)

Read both compact files for tribal knowledge before making any changes to controllers or case studies.

---

## Open Issues Log (Part 51+)

*(Append dated entries below as work proceeds.)*

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| **P1** | `DAESystem` struct + `dae2ode()` — Index-1 semi-explicit DAE; Newton solve on `g` | HIGH | **Done (Part 51)** |
| **P2** | `c2d()` overload for DAE — linearise + algebraic elimination + ZOH/Tustin | MED | **Done (Part 51)** |
| **P3** | DAE-aware EKF — post-update algebraic projection via `consistentInit()` | MED | **Done (Part 51)** |
| **E1** | `GreyBoxEstimator` — non-linear param estimation via Levenberg-Marquardt | HIGH | **Done (Part 52)** |
| **E2** | `RecursiveGreyBoxEstimator` — augmented-state UKF for online param tracking | HIGH | **Done (Part 52)** |
| **E3** | GP Residual Model — extend `GaussianProcess` with uncertainty output | MED | **Done (Part 52)** |
| **E4** | MHE Polytopic Constraints — extend MHE with `C_ineq`/`d_ineq` | MED | **Done (Part 53)** |
| **H1** | `HybridModel` base class — `IPlantModel` with `f_phys + f_data` | MED | **Done (Part 53)** |
| **H2** | `HybridMPC` — `NonlinearMPC` variant using `HybridModel` | MED | **Done (Part 53)** |
| **H3** | RL-MPC stitching Python example | LOW | **Done (Part 53)** |
| **H4** | `HybridModelTrainer` — hyperopt for `f_data` component | LOW | **Done (Part 53)** |
| **D1** | Mismatch Detector — CUSUM on KF/MHE innovation | LOW | **Done (Part 54)** |
| **D2** | Digital Twin Lite Python app | LOW | Open |
| **C2** | 8 spec-only stubs remain (BEMS + MEMS no blocker; DustControl/ModularEvap/SoftRobot/ControlTheory need plant-model design; Bioreactor/Nuclear thin specs). **DeePC closed (Part 57)** — `lib/DeePC.{h,cpp}` confirmed present and fully smoke-tested. | MED | Open |
| **C3** | Active Suspension 2-DOF: add `GAOptPIDCtrl` / `PSOOptPIDCtrl` / `DEOptPIDCtrl` using new lib/ GA/PSO/DE optimisers; 15→18 controllers, 75→90 runs | MED | **Done (Part 55)** |
| **C4** | SMISMO: modify plant for variable P_s; add `DOBEnergyCtrl` (Chen 2018 Eq. 29-30 DOB + adaptive supply pressure); 12→13 controllers, 60→65 runs | MED | **Done (Part 55)** |
| **C5** | EHFS: add `HinfODFCCtrl` + `HinfCascadeCtrl` (DiscreteHinf ODFC + local nLMS); 12→14 controllers, 60→70 runs | MED | **Done (Part 55)** |
| **C6** | Active Suspension 6×6 EV Full Model: NEW Python-only case study; 40-state plant (15-DOF vehicle + 5-DOF human biodynamic model), road time delays, 18 controllers, 90 runs | MED | **Done (Part 55)** |
| **B36-3** | Unify NaN-guard across controller fleet | MED | **Done (Part 53)** |
| R1 | Edge-case contract matrix tests for every controller family | MED | **Done (Part 53)** |
| T3 | Full DK-iteration with vector-fitting rational D(jω) | LOW | **Done (Part 53)** |
| B36-2 | `ex79_registry_monitor` monitors nothing (M3 telemetry mis-wired) | LOW | **Done (Part 39, confirmed Part 53)** |
| REL | Rebuild `ctrl_toolbox.pyd` in Release | LOW | Open |
| M4 | `template<typename Scalar>` leaf algorithms for embedded float target | Backlog | **Done (Part 54)** |

---

*(New parts appended below as work proceeds.)*

---

## Part 57E — Audit Iter E + DIST-1/2/4/5 — 2026-06-14

### Audit Iteration E (13 code changes, 29 verified/deferred — all 84 findings now closed)

All remaining audit findings from Iterations A–D were swept. See `docs/audit_report.md` Part 57E
section for full detail. Key code changes:
- **#46** `EchoStateNetwork.h` `W_out_` comment corrected (`n_out×n_res`, not `n_out×(n_res+n_in)`)
- **#54** `DiscreteSMC.h` `slidingSurface()` docstring clarified (returns s[k-1], not current s[k])
- **#55** `ParticleFilter.h` `w_` comment corrected (normalised probabilities, not log-sum-exp)
- **#59** `FeedforwardController.h` `rv_` promoted to pre-allocated member (no per-step heap alloc)
- **#64/#66** `test_catch2_advanced.cpp`: added `[pid]` tests (Kb anti-windup, N-filter decay, computeDoM comparative)
- **#65** `test_catch2_advanced.cpp`: added `[repetitive]` edge-case tests (invalid period, NaN hold-last, setParams reset)
- **#67** `test_catch2_advanced.cpp`: added `[extremum_seeker]` convergence test (J=(θ-2)²)
- **#68** `test_catch2_advanced.cpp`: renamed DynaController test + added bounded-range assertions
- **#69** `controllers_bindings.cpp`: `DynaController::inner_controller()` changed to `return_value_policy::copy`
- **#71** `smoke_test.py`: GreyBoxEstimator predict() shape assertion now checks both dimensions
- **#72** Created `_setup_bindings.py` (repo root); removed inline DLL block from 19 `examples/python/` + 9 `case-study/sim/` files
- **#74/#75** `doc.yml`: peaceiris action pinned to SHA `4f9cc6ed`; added `permissions: contents: write`

### DIST-1 — CMake install targets + vcpkg port (complete)

- **`lib/CMakeLists.txt`** — Added `EXPORT ControllerToolboxTargets` + `configure_package_config_file` +
  `write_basic_package_version_file` + install rules for cmake config files.
  Consumers use: `find_package(ControllerToolbox REQUIRED)` then `target_link_libraries(app ctrl::controller_toolbox)`
- **`cmake/ControllerToolboxConfig.cmake.in`** — Package config template; propagates Eigen3 dependency.
- **`cmake/ports/ctrl_toolbox/vcpkg.json`** — vcpkg port manifest (eigen3 dependency, python-bindings feature).
- **`cmake/ports/ctrl_toolbox/portfile.cmake`** — Standard `vcpkg_from_git` + configure + install pattern.
  *Note: SHA512 must be updated with actual hash after first v*.*.* tag push.*

### DIST-2 — Embedded header-only subset (complete)

- **`lib/embedded/DiscreteIntegrator.h`** — `template<Scalar>` backward-Euler integrator; `integrate()`, `value()`, `reset()`, `set()`.
- **`lib/embedded/FixedRateFilter.h`** — `template<Scalar, Order>` compile-time-order IIR LPF; backward Euler, stack state.
- **`lib/embedded/RingBuffer.h`** — `template<T, N>` fixed-capacity FIFO ring buffer; `push()`, `pop()`, `peek()`, `clear()`.
- **`lib/embedded/EmbeddedControllers.h`** — Umbrella include: re-exports `BasicPID.h`, `BasicSMC.h` + all 3 new files.
- **`examples/embedded/main.cpp`** — Demo: zero Eigen includes; verify with `grep -r "Eigen" examples/embedded/main.cpp`.
- **`tests/test_embedded_subset.cpp`** — 13 Catch2 `[basic_pid_embedded]`/`[basic_smc_embedded]`/`[discrete_integrator]`/`[fixed_rate_filter]`/`[ring_buffer]` tests; links only Catch2 (no controller_toolbox Eigen dep).
- **`CMakeLists.txt`** — Added `CTRL_BUILD_EMBEDDED_ONLY` option (early `return()` skips all Eigen targets) and
  `CTRL_FETCH_EIGEN_IF_MISSING` option (FetchContent fallback for CI wheel builds).
- **`tests/CMakeLists.txt`** — Added `test_embedded_subset` target (no Eigen, C++17, Catch2 only).

### DIST-4 — PyPI wheel distribution (complete)

- **`pyproject.toml`** — `scikit-build-core` backend; `cmake.args` enable Python bindings, disable tests;
  `CTRL_FETCH_EIGEN_IF_MISSING=ON` for CI containers. Triggered on `v*.*.*` tag.
- **`.github/workflows/publish.yml`** — cibuildwheel v2.21.3 (pinned SHA); builds cp39–cp312 on
  Linux/Windows/macOS; skips musl and 32-bit; publishes via PyPI trusted publishing (OIDC).
  *Requires "Trusted Publisher" configured in PyPI settings before first push.*

### DIST-5 — GitHub Release workflow (complete)

- **`.github/workflows/release.yml`** — Triggered on `v*.*.*` tag; builds Release on 3 platforms;
  `cmake --install` collects lib + headers + cmake config; zips and attaches to GitHub Release.
  Uses softprops/action-gh-release pinned to SHA `c062e08b` (v2.0.8).

---

## Part 59 — Cross-Platform Scripts + Case Study Tracker — 2026-06-15

### PLT-1 — `setup.sh` (Linux/macOS bootstrap)

Mirrors `setup.ps1` exactly. Five steps:

1. **Toolchain check** — accepts `gcc` or `clang`; cmake, ninja via `_need()`; eigen3 by
   header-path scan (`/usr/include/eigen3`, `/opt/homebrew/...`). Non-fatal eigen3 miss:
   emits `CTRL_FETCH_EIGEN_IF_MISSING=ON` to cmake (FetchContent fallback).
   Per-distro install hints printed for apt/dnf/pacman/brew.
2. **Conda check** — fails clearly with Miniconda install URL if not on PATH.
3. **Env create/update** — `conda env create -f environment.yml` (first time) or
   `conda env update --prune` (existing). Skipped by `--skip-conda-create`.
4. **Bindings build** — `conda run -n soft_robotics -- cmake -G Ninja ... --target ctrl_toolbox`.
   Locates built `.so`/`.dylib` under `build/bindings/`; fails clearly if absent.
5. **Smoke test** — `conda run -n soft_robotics -- python bindings/smoke_test.py`.
6. **Optional full build** — calls `compile.sh` when `--full-build` passed.

Staged with `git update-index --chmod=+x` (100755 mode) → lands executable on Linux clone.

### PLT-1 — `compile.sh` (Linux/macOS full build)

Mirrors `compile.bat`. Bash array of all 120 targets in dependency order; `cmake --build`
called once per target (sequential, no `--parallel`). Exits on first failure with clear error.
Flag: `--no-config` to skip cmake re-configure.

Staged with `git update-index --chmod=+x` (100755 mode).

### TRK-1 — `tools/case_study_tracker.py` + `docs/case_study_status.md`

Completed the tracker stub. Key fixes/additions:

- **`detect_language()`** — fixed extension comparison (was comparing `'cpp'` vs `'.cpp'`);
  fixed division-by-zero when no source files found; removed unreachable `return "mixed"`.
  Now follows 3-step spec: (1) check `sim/main.py` or `sim/src/main.cpp`; (2) depth heuristic
  on `sim/src/` existence; (3) extension count across whole tree.
- **`detect_status()`** — new: Complete (sim+logs+config+HTML), On-going (sim+logs+config),
  Incomplete (PDF or README on disk), Not started (default).
- **`find_pdf_link()`** / **`find_readme_link()`** — return `docs/`-relative markdown links.
- **`main()`** — walks `case-study/*/`; writes `docs/case_study_status.md` (Markdown table).

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
| `6-DOF Stewart Platform Vessel Motion Simulator/` | ✓ | ✓ | Stewart platform 6-DOF kinematics |
| `Heavy-Duty Parallel-Serial Hydraulic Manipulator VDC/` | ✓ | ✓ | VDC control; hydraulic parallel-serial arm |
| `Hybrid-Driven Tendon-Pneumatic Soft Manipulator/` | ✓ | ✓ | Adaptive kinematic + stiffness control |
| `Underwater Robotic Manipulator Trajectory Tracking/` | ✓ | ✓ | Implicit rigid TubeMPC + ASMC |
| `Unmanned Surface Vehicle Wave-Predictive Attitude Control/` | ✓ | ✓ | Short-time wave prediction MPC |

All added to the CLAUDE.md spec-only stubs table. Total stubs updated from 8 → 12
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

## Part 58 — ANA-1..7 + RPT-1 Analysis Pipeline + Test Bug Fixes — 2026-06-15

### Analysis pipeline (`tools/`) — ANA-1 through ANA-7 + RPT-1

All analysis and reporting tools are now implemented as standalone CLI scripts under `tools/`.

**ANA-1 — `tools/metrics.py`** (prerequisite module)
- `compute_metrics(t, y, u, ref)` → dict with `iae`, `rms_error`, `settle_time_s`, `overshoot_pct`,
  `max_u`, `energy_var`. Settling uses 2% band + 10-sample hysteresis.
- `compute_metrics_from_df(df)` — heuristic column detection; works on any case-study CSV.
- `extract_final_iae(df)` — reads last-row IAE from `iae_cumulative`, `IAE_y1..y3`, or computes from `error`.

**ANA-1 — `tools/compare_controllers.py`** (T7 re-implementation + ANA-1 extension)
- Auto-discovers `case-study/*/logs/run_*.csv`.
- Parses `run_{scenario}_{controller}.csv` naming convention (last `_`-token = controller).
- Flags: `--study`, `--scenario`, `--controller`, `--metric` (iae/rms_error/...), `--sort`, `--wide`, `--csv`.

**ANA-2 — `tools/monte_carlo.py` + `tools/mc_plots.py`**
- Imports study `sim/main.py` and calls `run_single(ctrl_name, perturbed_params)` if present.
- Perturbs `plant_params.json` numeric keys by `N(0, sigma)` per sample.
- Writes `mc_summary_*.csv`; `mc_plots.py` produces violin and scatter PNGs.

**ANA-3 — `tools/fault_injector.py` + `tools/fault_sweep.py` + `tools/fault_plots.py`**
- `FaultInjector`: composable sensor/actuator fault injection (bias, noise, loss, stuck, setpoint step).
- `fault_sweep.py` calls `sim.run_with_fault(ctrl_name, FaultSpec)` if present; writes `fault_sweep_*.csv`.
- `fault_plots.py` produces heatmaps and degradation curves per fault kind.

**ANA-4 — `tools/anova.py`**
- One-way ANOVA (scipy `f_oneway`) + Tukey HSD post-hoc (statsmodels `pairwise_tukeyhsd`).
- Reads any CSV with `controller` + metric column. Reports F, p, significance, and pairwise table.

**ANA-5 — `tools/wcet_report.py`**
- Discovers `wcet_*.csv` files (produced by optional timing instrumentation in `sim/main.py`).
- Aggregates mean, median, p99, WCET (q=0.999) per controller; writes `wcet_summary.csv` + optional bar chart.
- Includes instrumentation howto printed when no files found.

**ANA-6 — `tools/model_validation.py`**
- Uses `ctrl.GreyBoxEstimator` to fit ODE parameters to logged data; reports NRMSE.
- Studies must expose `grey_box_model()` → `(ode_fn, h_fn, x0, param_names, bounds)` in `sim/main.py`.
- Graceful fallback (IAE proxy) when hook is missing.

**ANA-7 — `tools/mu_analysis.py` + `tools/mu_plots.py`**
- Identifies discrete ARMA(2,2) model from each CSV; evaluates peak singular value of S(z) and T(z).
- Peak |T(z)| is an unstructured mu upper bound. Writes `mu_summary.csv`; plots bar + S vs T scatter.

**RPT-1 — `tools/generate_report.py`**
- Single self-contained HTML with 8 sections: Summary, Comparison, Heatmap, MC, Fault, ANOVA, WCET, Mu.
- Uses Plotly (inline CDN) for interactive charts. Gracefully degrades to plain tables if plotly absent.
- `--out report.html --open` writes and opens in browser.

### Test bug fixes

**`lib/RepetitiveController.cpp` — `setParams()` did not reset `v_now_`**
- When `periodSteps` changes, `v_buf_.assign(...)` cleared the buffer to zeros but `v_now_` was not
  updated. `correction()` returned the stale pre-change value. Fixed: added `v_now_ = 0.0` inside the
  `if (p.periodSteps != p_.periodSteps)` block.

**`tests/test_catch2_advanced.cpp` — `computeDoM` test measured wrong signal**
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

## Part 54 — D1 (MismatchDetector), M4 (BasicPID/BasicSMC) — 2026-06-12

**D1 — `MismatchDetector`** (`lib/MismatchDetector.h`, header-only)

- Wraps `CUSUMChart` (from `ControllerMonitor.h`) to run real-time CUSUM on the
  normalised innovation of a `KalmanFilter` or `MovingHorizonEstimator`.
- `MismatchDetectorParams`: `sigma` (in-control innovation RMS), `k_cusum` (slack, default 0.5),
  `h_threshold` (alarm level, default 5.0).
- `update(double)` feeds a scalar innovation (absolute value used internally to detect
  both upward and downward shifts in magnitude).
- `update(VectorXd&)` computes `‖innov‖/sqrt(p)` and feeds to scalar CUSUM.
- `detected()`: sticky bool — stays `true` until `reset()` is called.
- `score()`: current CUSUM statistic `max(C+, C-)`.
- `CTRL_REGISTER_FEATURE(mismatch_detector)`.

**D1 — Extensions to `KalmanFilter`** (`lib/KalmanFilter.{h,cpp}`)

- `enableMismatchDetection(params)`: attaches a `MismatchDetector` member; feeds
  `innov = y - C*x_pred - D*u` into CUSUM after every `update()` call.
- `mismatchDetected()`, `mismatchScore()`, `resetMismatchDetector()` accessors.
- Private: `std::optional<MismatchDetector> mismatch_det_`.

**D1 — Extensions to `MovingHorizonEstimator`** (`lib/MovingHorizonEstimator.{h,cpp}`)

- Same API: `enableMismatchDetection()`, `mismatchDetected()`, `mismatchScore()`,
  `resetMismatchDetector()`.
- Feeds `y_hist_[N] - C*x_est_` (one-step-ahead residual) into CUSUM after each
  `estimate()` call.

**D1 — Bindings / tests**

- `estimation_bindings.cpp`: `enable_mismatch_detection(sigma, k_cusum, h_threshold)`,
  `mismatch_detected()`, `mismatch_score()`, `reset_mismatch_detector()` on both
  `KalmanFilter` and `MovingHorizonEstimator`.
- `smoke_test.py`: D1 block — creates KF, enables detection, asserts no alarm on zero
  steps and `mismatch_score()` is float, checks registry.
- `tests/test_catch2_advanced.cpp`: 7 new `[mismatch_detector]` tests:
  (1) no alarm on white-noise innovation, (2) sustained shift triggers detection,
  (3) reset clears alarm, (4) vector innovation fires alarm, (5) KF disabled by default,
  (6) KF fires on wrong model, (7) MHE fires on mismatched model.

**M4 — `BasicPID<Scalar>` and `BasicSMC<Scalar>`** (`lib/BasicPID.h`, `lib/BasicSMC.h`,
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
- `tests/test_catch2_advanced.cpp`: 7 new tests — 4 `[basic_pid]` (step response, float
  saturation, reset, anti-windup bounded integrator) + 3 `[basic_smc]` (convergence,
  float saturation, reset reproducibility).

---

## Part 53 — Hybrid Models (H1-H4), E4, T3, B36-2/B36-3, R1 — 2026-06-12

**E4 — MHE Polytopic Inequality Constraints** (`lib/MovingHorizonEstimator.{h,cpp}`)

- `MHEParams::C_ineq` (m_c × n) and `d_ineq` (m_c): enforce `C_ineq * x_0 <= d_ineq` on
  arrival state after the FISTA solve via Hildreth's cyclic half-space projections.
- `ineq_proj_iters` (default 20): number of Hildreth sweeps. Box constraints (`xMin`/`xMax`) are
  re-applied after polytope projection so both are simultaneously satisfied.
- `projectX0Polytope()` private helper: no-op when `C_ineq` is empty; otherwise iterates
  half-space projections then re-clips to box.
- 3 `[mhe_polytopic]` Catch2 tests: half-space upper bound, simplex-coupled constraint,
  equivalence with xMax for pure box case.
- `estimation_bindings.cpp`: `C_ineq`, `d_ineq`, `ineq_proj_iters` exposed on `MHEParams`.
- `smoke_test.py`: E4 block asserts `C_ineq`/`d_ineq` shape round-trips correctly.

**H1 — `HybridModel`** (`lib/HybridModel.h`, header-only)

- `IPlantModel` abstract interface: `dynamics(x, u)`, `stateSize()`, `outputSize()`.
- `HybridModel` concrete class: `f_phys(x, u)` (physics, required) + optional `f_data(x, u)`
  (data-driven correction). `setDataModel()` / `clearDataModel()` swappable at runtime.
- RK4 `predict(x0, U, Ts)` helper on `IPlantModel`.

**H2 — `HybridMPC`** (`lib/HybridMPC.{h,cpp}`)

- Inherits `NonlinearMPC`; overrides the prediction rollout to call `HybridModel::dynamics`.
- `updateDataModel(DataFunc)`: hot-swaps the data correction without rebuilding the QP.
- `N_update` parameter: data model refreshed from new observations every N steps.

**H3 — RL-MPC Stitching** (`examples/python/ex101_rl_mpc_stitching.py`, Python-only)

- Lightweight DQN-style policy (<10 k params, numpy-only) that adjusts `rho_y` of `HybridMPC`
  in real time on a spring-mass-damper plant.
- Policy state: `[error, error_dot]`; actions: 4 discrete `rho_y` multipliers.
- Demonstrates H2 + H3 integration without PyTorch dependency.

**H4 — `HybridModelTrainer`** (`lib/HybridModelTrainer.{h,cpp}`)

- `trainGP(XU, residuals, gp)` → `DataFunc`: fits `GaussianProcess` residual and returns a
  lambda capturing the trained GP's `predict()` method.
- `trainESN(XU, residuals, esn)` → `DataFunc`: offline ridge-regression on `EchoStateNetwork`
  and returns the ESN forward-pass lambda.
- `trainRidge(XU, residuals, lambda_reg)` → `DataFunc`: lightweight fallback using Eigen
  ridge (no external dependency).

**T3 — VectorFitting + full DK-iteration** (`lib/VectorFitting.{h,cpp}`)

- Gustavsen SK iterative rational fitting of complex frequency response data → poles + residues.
- `solveMuSyn` in `DiscreteHinf`: `dFitOrder > 1` switches from first-order D-scaling to
  vector-fit rational D(jω), enabling full DK-iteration for structured-uncertainty mu-synthesis.

**B36-2 — `ex79_registry_monitor` fix** (Part 39, confirmed Part 53)

- `shared_ptr` monitor was copy-constructed before attachment; fixed to create `mon_ptr` first
  so callback and observer share a single instance. Observer now actually fires.

**B36-3 — NaN-guard hold-last fleet contract** (`lib/IController.h` + 7 controllers)

- `sanitize()` removed from contract. Hold-last NaN behaviour added to:
  ExtremumSeeker, MRAC, TubeMPC (scalar path), ScenarioMPC (scalar path),
  RepetitiveController, SmithPredictor, DiscreteHinf.
- `IController.h` documents the hold-last contract.
- `[nan_guard]` Catch2 tags added across all affected controllers.

**R1 — Contract matrix tests** (`tests/test_catch2_advanced.cpp`)

- `[nan_guard]` + `[health_contract]` extended to all controller families.
- Saturation-bounded-integral and non-stabilizable `isHealthy()` coverage added.

**M4 — BasicPID / BasicSMC (CLAIMED done in CLAUDE.md — NOT VERIFIED)**

- CLAUDE.md states `lib/BasicPID.h` and `lib/BasicSMC.h` (header-only `BasicPID<Scalar>` /
  `BasicSMC<Scalar>` for embedded float usage) were created in Part 53.
- **Files do NOT exist** in the repository as of the Part 53 audit (2026-06-12).
- Marked Open in the issues table. Must be implemented before closing M4.

---

## Part 51 — DAE Architecture (P1/P2/P3) — 2026-06-12

**P1 — `DAESystem` + `consistentInit` + `dae2ode`** (`lib/PlantModel.h/.cpp`)

- `DAESystem` struct: `f` (differential), `g` (algebraic), `h` (output), `n_diff`, `n_alg`, `Ts`.
- `consistentInit(dae, x1_init, u0, x2_guess)`: Newton-Raphson (LDLT) solving `g=0` for `x2`; up to 20 iters, tol=1e-9.
- `dae2ode(dae)`: returns discrete step function `x_aug_next = F(x_aug, u)`. Forward Euler for `x1`, Newton projection for `x2` at both current and next `x1`. `Ts` must be set on `DAESystem`.
- Three static central-difference Jacobian helpers (`algJacX1`, `algJacX2`, `algJacU`) in `PlantModel.cpp`.
- `CTRL_REGISTER_FEATURE(dae_system)` added after `namespace ctrl`.

**P2 — `c2d(DAESystem, x1_op, x2_op, u_op, Ts, method)`** (`lib/PlantModel.h/.cpp`)

- Index-1 algebraic elimination: `A_red = A11 - A12*G2⁻¹*G1`, `B_red = B1 - A12*G2⁻¹*B2` where all Jacobians are computed numerically via `algJac*` helpers.
- Checks `rcond(G2) > 1e-12`; throws `std::runtime_error("c2d(DAESystem): G2 is singular — DAE is not Index-1 at operating point.")` otherwise.
- Output matrix built from `h` Jacobians (or identity w.r.t. `x1` if `h` not set).
- Dispatches to existing `c2d(StateSpace, Ts, method)` for ZOH/Tustin.
- Python binding registered as `dae_c2d` (avoids `py::overload_cast` ambiguity with existing `c2d`).

**P3 — DAE-aware EKF projection** (`lib/ExtendedKalmanFilter.h/.cpp`)

- `setAlgebraicConstraint(g_alg, n_diff, n_alg, tol=1e-9)`: attaches algebraic constraint function; validates `n_diff + n_alg == n_states_`.
- `hasAlgebraicConstraint()`: bool accessor.
- `projectAlgebraicStates(u)`: called at end of `update()` when constraint is set. Newton-Raphson on `x2` block using `numericalJacobian`; then covariance projection `P = J_proj * P * J_proj'` where `J_proj = [[I, 0]; [-G2⁻¹G1, 0]]`.
- SISO assumption: `u_scalar = u(0)` (consistent with `DAESystem::AlgFunc` signature).
- Independent `AlgConstraintFn` type alias in EKF (does not depend on `PlantModel.h`).

**Bindings / tests**

- `plantmodel_bindings.cpp`: `DAESystem` class with `set_f/set_g/set_h`, `consistent_init`, `dae2ode`, `dae_c2d`.
- `estimation_bindings.cpp`: `set_algebraic_constraint`, `has_algebraic_constraint` on `ExtendedKalmanFilter`.
- `smoke_test.py`: 4 DAE assertions (`consistent_init`, `dae2ode`, `dae_c2d`, `registry_has('dae_system')`).
- `tests/test_catch2_advanced.cpp`: 7 Catch2 tests — `[dae_system]` ×3, `[dae_c2d]` ×2, `[dae_ekf]` ×2.

---

## Part 52 — Model Estimation E1/E2/E3 — 2026-06-12

**E1 — `GreyBoxEstimator`** (`lib/GreyBoxEstimator.{h,cpp}`)

- Batch Levenberg-Marquardt for user ODE `f(x,u,p)` and measurement `h(x,p)`.
- RK4 integration with `rk4_steps` substeps per `Ts`; central finite-difference Jacobian
  (step `eps_i = eps_jac * max(|p_i|, 1)` per parameter); box-constrained params via projection.
- LM normal equations: `(J'J + lambda*diag(J'J)) dp = -J'r`; accept/reject; `lambda *= nu` on
  reject, `lambda /= nu` on accept. Convergence: `max|J'r| < tol_grad`.
- `fit(x0, U, Y)` returns `Result{params, cost, iterations, converged}`.
- `predict(x0, U)` returns `Y_hat (n_y x N)` using current `p_est_`.

**E2 — `RecursiveGreyBoxEstimator`** (`lib/RecursiveGreyBoxEstimator.{h,cpp}`)

- Augmented state `z = [x; p]`; `f_aug` integrates ODE (RK4) for `x` and holds `p` constant
  with diffusion `Q_param`. `h_aug(z, u) = h(z.head(n_x), z.tail(n_p))`.
- UKF created at `initialize()` with `P0_aug = blkdiag(P0_state, P0_param)`.
- Compiled only under `CTRL_ENABLE_ADVANCED_KALMAN` (same guard as UKF/EKF).
- Default `alpha=0.1` (not 1e-3) — augmented `n_aug = n_state + n_param >= 3`.

**E3 — `GPResidualModel`** (`lib/GPResidualModel.{h,cpp}`)

- Composition over existing `GaussianProcess`; stores residuals `epsilon = y_true - y_model`.
- `addResidualPoint(xf, y_true, y_model)`: appends `(xf, epsilon)` to GP dataset.
- `residualFit(X_feat, Y_true, model_fn)`: batch version — resets GP, adds all points, calls `fit()`.
- `predictWithUncertainty(xf, model_pred)`: returns `{model_pred + gp_mean, gp_mean, gp_variance}`.
  Returns `{model_pred, 0.0, 0.0}` before first `fit()` call.

**Bindings / tests**

- `estimation_bindings.cpp`: `GreyBoxParams`, `GreyBoxResult`, `GreyBoxEstimator`; `RecursiveGreyBoxParams`,
  `RecursiveGreyBoxEstimator` (inside `CTRL_HAS_ADVANCED_KALMAN` guard).
- `controllers_bindings.cpp`: `GPResidualParams`, `GPResidualPrediction`, `GPResidualModel`.
  `residual_fit` wraps `model_fn` via `py::object` lambda capture.
- `smoke_test.py`: 3 assertion blocks (E1 fit, E2 step, E3 residual + batch).
- `tests/test_catch2_advanced.cpp`: 8 new Catch2 tests — `[grey_box]` ×3, `[recursive_grey_box]` ×2,
  `[gp_residual]` ×3. Bug fix: `Eigen::Vector1d` does not exist — replaced with explicit `VectorXd(1)`.
- `examples/ex80_grey_box_estimator.cpp`; `ex97_grey_box_estimator.py`, `ex98_recursive_grey_box.py`,
  `ex99_gp_residual_model.py`.

---

## Part 55 — Controller Gap Audit + C3–C6 (2026-06-13)

**Audit methodology:** All 15 case studies were cross-checked: each study's README was read to
identify the paper's proposed controller, then the actual `sim/` source was inspected to verify
presence. Studies that are pure plant-characterisation papers or literature reviews (no novel
controller proposed) were marked N/A. Three genuine gaps were identified (C3–C5) plus one new
study requested (C6).

---

**C3 — Active Suspension 2-DOF: metaheuristic-tuned PID controllers**

- Paper: Aydogan & Yildiz 2025 (*Alexandria Eng. J.* 127, 989-1003).
  Core contribution: GA/PSO/DE optimisation of suspension controller gains on a 15-DOF full vehicle.
- Gap: The 2-DOF sim (`susp_sim`, 15 controllers) has no parameter optimisation loop at all.
- Fix: Add `lib/GeneticAlgorithm.h`, `lib/ParticleSwarmOptimizer.h`, `lib/DifferentialEvolution.h`
  (header-only; reuse `TunerResult`/`CostFn` from `AutoTuner.h`); add `GAOptPIDCtrl`,
  `PSOOptPIDCtrl`, `DEOptPIDCtrl` to Active Suspension 2-DOF sim (15→18 controllers, 75→90 runs).
- New lib/ files require full 8-step checklist: bindings in `analysis_bindings.cpp`, smoke tests,
  Catch2 `[genetic_algorithm]`/`[pso]`/`[de]` tests (3 each), combined Rosenbrock example.
- Files: `lib/GeneticAlgorithm.h`, `lib/ParticleSwarmOptimizer.h`, `lib/DifferentialEvolution.h`,
  `lib/ControllerToolbox.h`, `bindings/analysis_bindings.cpp`, `bindings/smoke_test.py`,
  `tests/test_catch2_advanced.cpp`, `examples/exNN_metaheuristics.{cpp,py}`,
  `case-study/Active Suspension .../sim/{include/controllers.h, src/controllers.cpp, src/main.cpp}`,
  `tests/test_susp_regression.cpp`.

---

**C4 — SMISMO: disturbance-observer supply-pressure adaptation**

- Paper: Chen & Wang 2018. Core contribution: 2nd-order DOB (Eq. 29-30) estimates load force
  `F_hat`; adaptive supply pressure `P_s = k_f*|F_hat| + k_v*|v_L| + P_margin` (Eq. 37) achieves
  ~5/6 energy reduction vs. fixed supply. Grey predictor (GM(1,1), Eq. 52) drives pump speed.
- Gap: `smismo_sim` holds P_s constant at 60 bar; no disturbance observer; no energy comparison.
- Fix: (1) `smismo_plant.h/cpp` — add `setSupplyPressure(bar)`; replace hardcoded `P_s` with
  dynamic `P_s_dyn_` in valve flow equations; existing 12 controllers unaffected (never call setter).
  (2) Add `DOBEnergyCtrl` (#13): inner PID for position + DOB for load force estimation +
  adaptive P_s before each plant step. Observer gain L=30 (Chen Table 1). P_s clamped [22, 65] bar.
  (3) Grey predictor omitted (requires pump dynamics not in current sim; noted in comment).
- Files: `sim/include/smismo_plant.h`, `sim/src/smismo_plant.cpp`, `sim/include/controllers.h`,
  `sim/src/controllers.cpp`, `sim/src/main.cpp` (N_CONTROLLERS 12→13), `tests/test_smismo_regression.cpp`.
- Key invariant: `P_min = 22 bar > P_bd = 20 bar` — backpressure guard must be preserved.

---

**C5 — EHFS: PI + H∞ ODFC + nLMS 3-layer cascade**

- Paper: Shen et al. 2017. Core contribution: 3-layer cascade — base PI + H∞ offline-designed
  feedback controller (ODFC) + normalised LMS adaptive compensator (CSIA) for force ripple at
  velocity reversal and load stiffness variation.
- Gap: All 12 current EHFS controllers are generic toolbox algorithms; no H∞ ODFC or nLMS class.
- Fix: Python-only. Linearise 5-state EHFS plant around operating point (F0=3000N, x_v0=0);
  use `ctrl.DiscreteHinf.solve()` + `ctrl.MixedSensitivity.build()` offline at construction.
  `HinfODFCCtrl` (#13): H∞ controller as standalone feedback. `HinfCascadeCtrl` (#14): PI +
  H∞ (additive) + nLMS adaptive correction (additive). Fall back to PID if H∞ synthesis infeasible.
  Local `NormalisedLMS` class (~30 lines): regressor shift-register + normalised update rule.
- Files: `case-study/Tracking Control of Electro-Hydraulic Force Servo Systems/sim/main.py`
  (add 2 controllers; 12→14; update docstring 60→70 runs).
- Non-obvious: `DiscreteHinf` takes `HinfResult` in its constructor — use `ctrl.DiscreteHinf(result)`.
  EHFS Ts=0.5ms → linearised state-space and H∞ design must use the same Ts.

---

**C6 — Active Suspension 6×6 EV Full Model (new Python-only case study)**

- Paper: Aydogan & Yildiz 2025 (same as C3). Full 20-DOF (40-state) system: 15-DOF body
  (Z, θ, φ + 6 wheels + 6 in-wheel motors) + 5-DOF human biodynamic model (seat/pelvis/lower
  torso/upper torso/head). Paper proposes GA/PSO/DE-optimised PD control for all 6 wheel actuators.
- Directory: `case-study/Active Suspension 6x6 EV Full Model/` (Python-only; not in CMake or compile.bat).
- Plant: `EV6x6Plant` in `sim/ev6x6_plant.py` — assembles M (20×20 diagonal), K and C (20×20,
  includes pitch/roll-to-wheel coupling via geometric offsets a_i, e_i), B_road (20×6), B_act (20×6);
  state-space form `A=[0, I; -M^{-1}K, -M^{-1}C]` (40×40); discretised at Ts=0.005s via ZOH.
- Road model: `sim/road_model.py` — 2 independent channels with time-delay ring buffer for middle
  (delay=L1/v) and rear (delay=(L1+L2)/v) axles. L1=L2=2.0m, v=22.2m/s (80 km/h).
- Parameters: per-corner values inherit from 2-DOF study (k_s=16000, c_s=980, k_t=160000,
  M_w=36kg); human model from ISO 2631 Guide E / Griffin 1990; full-vehicle mass sourced from
  paper Table 2. Note in README if any parameters are estimated.
- Controllers (18): PassiveCtrl, PDCtrl, GAOptPDCtrl (paper's method), PSOOptPDCtrl, DEOptPDCtrl,
  PIDCtrl, LQRCtrl (full 40-state DARE), LQGCtrl (partial obs), MPCCtrl (12-state reduced),
  ADRCCtrl, SMCCtrl, MRACCtrl, FuzzyPIDCtrl, TubeMPCCtrl, ILCCtrl, CBFCtrl, L1AdaptiveCtrl,
  ScenarioMPCCtrl. Per-wheel strategies = 6 independent SISO instances; centralised = full model.
- 5 scenarios matching 2-DOF study (step bump, sine resonance, rough road, speed bump, comfort).
  Total: 18 × 5 = 90 runs.
- CSV columns: `t, Z_body, Z_head, theta, phi, Z_wr1-3, Z_wl1-3, F_act_1-6, z_r_1-6, seat_accel,
  head_accel, body_accel, iae_cumulative`.
- Key tribal knowledge: MPC/TubeMPC/ScenarioMPC use a 12-state reduced model for prediction;
  the full 40-state plant always runs for simulation (they diverge). LQR uses the full system
  (40 states, 6 inputs) — DARE is feasible since B_act has rank 6 (independent wheel actuators).
  ADRC omega_o*Ts < 0.5 constraint at Ts=0.005s → omega_o < 100.

---

## Part 56 — Test/Binding Bug Fixes — 2026-06-14

Six compile/binding/test errors found in `run_20260614_080012.log` and `run_20260614_084939.log`
and fixed in this session. No new algorithms added; no case-study logic changed.

---

**Fix 1 — `GradientProjectionQP` y_fista workspace (tests)**
- `solveGradientProjectionQP` signature has 12 parameters; the 12th (`y_fista` workspace
  `Eigen::Ref<VectorXd>`) was added in a prior session when FISTA momentum was introduced.
- Two tests in `tests/test_catch2_advanced.cpp` (lines ~109-112 and ~132-133) still called
  the 11-argument version → compile error "too few arguments".
- Fix: added `Eigen::VectorXd y_fista(n);` workspace and passed as 12th argument in both tests.
- `Eigen::Ref<VectorXd>` (not raw `VectorXd&`) is required for the workspace parameters so that
  block expressions (e.g., `.head(n)`) can be passed without a copy.

---

**Fix 2 — `ConsistentInitResult` binding (`plantmodel_bindings.cpp`)**
- `ctrl::consistentInit()` returns `ConsistentInitResult{VectorXd x, bool converged}`.
- The `consistent_init` Python binding at `plantmodel_bindings.cpp:197` returned the full
  struct, which pybind11 cannot auto-convert → `TypeError: Unregistered type: ctrl::ConsistentInitResult`.
- Fix: added `.x` to the return: `return ctrl::consistentInit(...).x;` — binding now returns
  a plain `np.ndarray`.
- Matching fix in `tests/test_catch2_advanced.cpp:4253`: test extracted `ci_result.x` manually.
- **Non-obvious:** The binding strips the struct; Python callers receive only the VectorXd.
  `converged` flag is not exposed in Python. C++ callers should always check `.converged`.

---

**Fix 3 — `TaylorApproximator::evaluate()` method name (test)**
- Test at `tests/test_catch2_advanced.cpp:5494` called `approx.eval(x)`.
- Actual method name is `evaluate(x)` → compile error "no member named eval".
- Fix: renamed call to `approx.evaluate(x)`.

---

**Fix 4 — ZPETC amplitude-flatness test (`test_catch2_advanced.cpp`)**
- The `[zpetc]` test checked `|arg(G * Gff)| < 1°` (near-zero phase). For a strictly proper
  2nd-order plant (relative degree 2), ZPETC yields `G * Gff = z^{-1}` (one-sample delay).
  At test frequency ω = 0.1π/T, the phase is `−0.1π rad ≈ −18°` — this is **correct ZPETC
  behaviour**, not a bug. The invariant is amplitude flatness, not zero phase.
- Fix: replaced phase assertion with amplitude assertion:
  `REQUIRE_THAT(std::abs(G * Gff), WithinAbs(1.0, 0.05))`
- **Tribal knowledge:** ZPETC (Tomizuka 1987) guarantees `|G(e^{jωT}) * G_ff(e^{jωT})| ≈ 1`
  for all frequencies below Nyquist (amplitude flat), at the cost of a group delay equal to
  the plant relative degree d. Absolute phase equals `−d·ω·T` — non-zero and unavoidable.
  The correct performance metric for ZPETC is tracking amplitude fidelity, not phase lead.

---

**Fix 5 — `ComputationalDelayWrapper` Python binding (`controllers_bindings.cpp`)**
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

**Fix 6 — `make_lqr_controller` Python binding (`controllers_bindings.cpp`)**
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
  captures state by reference — update the captured variables before each `compute()` call.

---

**Fix 7 — `HybridModel::makeDynamicsFunc` safe factory (`examples/ex81_hybrid_model_mpc.cpp`)**
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
ZeroPhaseTrackingFilter     -> ZPETC gives |G*Gff| ≈ 1 (amplitude flat, NOT zero phase);
                               phase = -d*omega*T; test amplitude, not phase
ComputationalDelayWrapper   -> Python: ctrl.ComputationalDelayWrapper(inner, initial_output=0.0)
make_lqr_controller         -> Python: ctrl.make_lqr_controller(plant_ss, lqr_params,
                               state_fn, ref_fn=None); callables must return np.ndarray
HybridModel.dynamicsFunc()  -> DEPRECATED; use ctrl.HybridModel.makeDynamicsFunc(model_sptr)
```

---

## Part 57 — Audit Iteration A Verification — 2026-06-14

No new algorithms or case-study logic added. All 17 Iteration A findings from `docs/audit_report.md`
(2026-06-13) were verified as **already resolved** in the codebase before this session.

**DeePC confirmed implemented.** The CRITICAL audit finding (#1) — `lib/DeePC.{h,cpp}` absent —
was invalidated: both files are present. The smoke test at `bindings/smoke_test.py:1029–1050`
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
| 70 | LOW | 4 classes not smoke-tested | `CUSUMChart`, `EWMAChart`, `SmithPredictor`, `ExtremumSeeker` asserted at `smoke_test.py:1022–1025` |
| 73 | LOW | `cxx_std_17` in examples CMake | `cxx_std_20` already in `examples/CMakeLists.txt:6` |
| 76 | LOW | `.gitignore` missing patterns | `*.pyd`, `*.exe`, `CMakeCache.txt`, `CMakeFiles/`, `.idea/`, `case-study/**/logs/` all present |
| 77 | LOW | `/tools` gitignored | `/tools` is not gitignored; `!/tools/*.py` negation is superfluous but harmless |

**Docs updated this session:**
- `docs/audit_report.md`: resolution status table appended (Part 57 section)
- `docs/cumulative_bug_report.md`: C2 DeePC note closed; Part 57 section added
- `CLAUDE.md`: Open Items updated to Part 57; Part 57 Done block added

**Remaining open:** 67 audit findings — #7–12 (HIGH Performance), #15–18, #22–33, #35, #37–69,
#71–72, #74–75, #78–84. See Iterations B–E in `prompt/prompt_enhanced.txt` for the work order.

---

## Part 57B — Audit Iteration B (Performance Pre-allocation) — 2026-06-14

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
                                      ramp-up (first N steps) still reallocates as dz grows —
                                      this is correct and identical to original behaviour
KoopmanEDMD liftRBF vs liftPoly   -> both now share mutable lift_psi_; only one path is
                                      active per call (Dict::RBF dispatches to liftRBF,
                                      Dict::PolyDeg1/2 dispatches to liftPoly) — no aliasing
```

**Audit progress:** 17 (Iter A) + 13 (Iter B) = 30 findings closed. 63 remain (Iter C-E, mostly LOW/MED).
**Docs updated:** `docs/audit_report.md` Part 57B table appended; this section added.

---

## Part 57C — Audit Iteration C (Correctness Sweep) — 2026-06-14

**Iteration C complete.** 12 correctness findings checked (#9, #15–18, #22, #27–28, #30–33); 10 already resolved, 2 required code changes:

| Finding | Controller / File | Code change |
|---------|------------------|-------------|
| #22 MHE general `EigenSolver` on symmetric PD Hessian | `MovingHorizonEstimator` | `Eigen::SelfAdjointEigenSolver` at both `estimate()` call sites; avoids complex arithmetic on guaranteed-real eigenvalues |
| #33 `MismatchDetector::update(VectorXd)` zero-size undocumented | `MismatchDetector.h` | `@note` added explaining zero-size → 0 fed to CUSUM (no alarm update) and `sqrt(max(p,1))` guards division-by-zero |

**Docs updated:** `docs/audit_report.md` Part 57C table appended; this section added.

---

## Part 57D — Audit Iteration D (LOW-severity batch) — 2026-06-14

**Iteration D complete.** 9 LOW-severity findings checked (#44, #45, #47, #50, #51, #53, #56, #58, #60); 5 already resolved/deferred, 4 required code changes:

| Finding | Controller / File | Code change |
|---------|------------------|-------------|
| #44 `ComputationalDelayWrapper::lastOutput()` naming ambiguity | `lib/ComputationalDelayWrapper.h` | Expanded docstring to clarify "last returned" == "next to be returned" equivalence under one-step delay semantics |
| #45 `EchoStateNetwork::extendedState()` dead private method | `lib/EchoStateNetwork.{h,cpp}` | Removed declaration + definition; method was never called after Part 31 |
| #51 `UnscentedKalmanFilter::sigmaPoints()` allocs n×(2n+1) per call | `lib/UnscentedKalmanFilter.{h,cpp}` | Added `mutable Eigen::MatrixXd sigma_pts_ws_` (sized in constructor); `sigmaPoints()` writes into it and returns `const Eigen::MatrixXd&`; callers in `predict()`/`update()` bind by const ref — eliminates two matrix allocs per `step()` |
| #56 `findEquilibrium()` silent 1000× tolerance fallback | `lib/AutoGainScheduler.h` | Added `#ifndef NDEBUG` `std::clog` warning when the `tol * 1e3` fallback fires; added `#include <iostream>` |

**Non-obvious caveats from Part 57D:**
```
UKF sigma_pts_ws_          -> mutable workspace; sigmaPoints() returns const&;
                               callers must bind const Eigen::MatrixXd& (not by value)
DiscreteADRC #58 deferred  -> runtime assert for "setReference() never called"
                               would fire spuriously in ControllerStack (r_=0 intentional);
                               @warning in header docstring is the correct guard
```

**Audit progress:** 51 total findings closed across Iterations A–D. 42 remain (Iter E).
**Docs updated:** `docs/audit_report.md` Part 57D table appended; this section added.
