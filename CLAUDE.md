# CLAUDE.md - Controller-Toolbox

> This file IS git-tracked and committed (it is **not** gitignored - a prior version of this
> header wrongly claimed it was; `git check-ignore CLAUDE.md` returns nothing and
> `git ls-files` lists it). So it survives a fresh clone - keep it accurate. Companions:
> `CONTRIBUTING.md` (conventions, sign-convention table, checklists, numerical-safety rules),
> `docs/DOCUMENTATION.md` (full API), `docs/deployment.md` (RT constraints), `docs/handoff.md`
> (cross-cutting caveats), `docs/forensic_reconstruction.md` (source-only architecture + v1->v2
> diff), `docs/cumulative_bug_report.md` (Part-numbered history),
> `docs/controller_selection_matrix.md` (plant-nature -> candidate-controller shortlist),
> `docs/control_strategies_deep_dive.md` (per-strategy behaviour + decision framework). Read those
> for depth; this file is the 2-minute map.

## 1. High-Level Project Philosophy

Discrete-time control library: a flat C++20/Eigen core (~125 controller/estimator/identification
classes across 119 `.h` + 89 `.cpp`; 46 are `IController` subclasses) exposed through one flat
`pybind11` module and exercised by 126 C++ + 152 Python examples and 31 tracked case studies
(21 complete, 10 open placeholder/not-started per `docs/case_study_status.md`), plus 1
MATLAB-native study tracked by hand. Almost every algorithm implements both its math and a single base interface
(`IController`) in one class; the only deliberate split is the stateless `DiscreteLQR` +
`LQRAdapter`. Cross-cutting behaviour (anti-windup, delay, gain scheduling, dead-time, multi-loop
supervision) is added by **composition wrappers that are themselves `IController`s**, so they
nest. Conventions (especially `compute()` sign) are intentionally *not* uniform across
controllers - they are made queryable, not homogenised. Production readiness is first-class: a
header-only no-Eigen embedded subset, a lock-free param buffer, HAL + RTOS schedulers, a ROS 2
lifecycle node, and WCET tooling all exist to move the same controllers to hardware.
**Scope discipline:** treat `lib/*.{h,cpp}` as a stable public API to *consume*; adding a new
controller is a heavier checklist (impl + build wiring + examples + Catch2 + bindings + smoke
test), not a quick edit `[Ref: CONTRIBUTING.md#adding-a-new-controller]`.

## 2. Essential Build & Environment Commands

```bash
# Canonical "is everything passing" (8 phases: 1 ASCII scan, 2 NaN-guard scan, 3 compile,
# 4 bindings+smoke, 5 run all .exe, 6 run all python examples, 7 python case studies, 8 regen
# status/report). Last green run: 175/175 .exe, 149/149 py examples, 10/10 py case studies.
conda run -n soft_robotics -- python run.py        # writes run_*.log; bug_report.txt on failure -
#   NOTE: bug_report.txt can false-positive by matching "nan" in the PASSING Phase 2 banner;
#   confirm against the log's "EXIT 0 - PASSED" lines before treating it as a real failure.

# Full sequential C++ build of every target (~155; hand-maintained list - keep .bat/.sh in sync):
./compile.sh          # Windows: compile.bat   (do NOT add --parallel locally; CI does that)

# Manual CMake (Release):
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure

# With Python bindings:
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTRL_BUILD_PYTHON_BINDINGS=ON
cmake --build build --target ctrl_toolbox
conda run -n soft_robotics -- python bindings/smoke_test.py

# One-time env setup: .\setup.ps1 (Win, PowerShell 7) | ./setup.sh (Linux/macOS)
# Single test: ctest --test-dir build -R test_catch2_advanced  | build/tests/test_catch2_advanced.exe [smc]
```

- **Toolchain (Windows):** MSYS2 **UCRT64** (`C:\msys64\ucrt64\bin`), *not* mingw64. MinGW builds
  statically link `libgcc`/`libstdc++`/`libwinpthread` so executables and `ctrl_toolbox*.pyd` run
  without MSYS2 on `PATH` - without it, `import ctrl_toolbox` fails outside an MSYS2 shell even
  when the build succeeds. `[Ref: CMakeLists.txt - if(MINGW) add_link_options(-static-*)]`
- **`lib/` feature flags** (all **ON** by default), each defines a `CTRL_HAS_*` macro:
  `CTRL_ENABLE_HINF`, `CTRL_ENABLE_SUBSPACE`, `CTRL_ENABLE_FUZZY`, `CTRL_ENABLE_FUNCTION_APPROX`,
  `CTRL_ENABLE_ADVANCED_KALMAN`. Build-mode flags (root): `CTRL_BUILD_EMBEDDED_ONLY`,
  `CTRL_FETCH_EIGEN_IF_MISSING`, `CTRL_BUILD_PYTHON_BINDINGS`, `CTRL_BUILD_BENCHMARKS`.
- **Deps (pinned):** Eigen >=3.4.0, Catch2 v3.5.4, pybind11 v2.13.6, nlohmann/json v3.11.3.
- **Never claim "passing" without a clean `run.py`** - README/CONTRIBUTING pass counts are stale until then.

## 3. Repository Topography & Physical Layering

**WARNING - the conventional `include/`, `src/`, `python/` directories do NOT exist here.** Map:

- `lib/` - **flat** core engine; every class is `lib/ClassName.{h,cpp}` (no subpackages, ~208 files: 119 `.h` + 89 `.cpp`). `[Ref: lib/]`
  - `lib/embedded/` - header-only, no-Eigen, no-virtual MCU subset (`BasicPID`, `BasicSMC`, `DiscreteIntegrator`, `FixedRateFilter`, `RingBuffer`).
  - `lib/hal/` - hardware abstraction (`ISensor`/`IActuator`/`ITimer`/`IScheduler`, Sim*/Safe*, FreeRTOS/Zephyr schedulers).
  - `lib/ControllerToolbox.h` - umbrella include; `lib/Features.h` - runtime feature registry (`ctrl.features()`).
- `bindings/` - the **C++<->Python boundary** (NOT `python/`): one flat `pybind11` module `ctrl_toolbox`. Entry `bindings/module.cpp`; dispatch to `plantmodel/controllers/estimation/advanced/analysis_bindings.cpp`. `[Ref: bindings/module.cpp:18]`
- `examples/` - 126 single-file C++ demos `exNN_*.cpp` (print PASS/FAIL); `examples/python/` (152 scripts); `examples/embedded/`.
- `tests/` - Catch2 suites + legacy hand-rolled + per-study regressions (21 `.cpp`, **513 `TEST_CASE()`**; `test_catch2_advanced.cpp` alone is 409). `[Ref: tests/CMakeLists.txt]`
- `case-study/<Study>/` - C++ (`*_sim` exe, run by `run.py` **Phase 5**) or Python-only (`sim/main.py`, **Phase 7**). 33 dirs -> **31 tracked**, 21 complete (11 C++ + 10 Python). Two dirs are excluded from `tools/case_study_tracker.py` by design: `case-study/common/` (shared code, not a study) and `Boiler Control MATLAB` (MATLAB-native: `matlab/` + `config/` + `logs/`, no `sim/`; run by hand via `run_all.m`, **not** by `run.py`). The tracker is **scoped to C++/Python only** - don't "fix" it to classify MATLAB studies.
- `tools/` - post-hoc analysis pipeline only (metrics, compare_controllers, monte_carlo, fault_sweep, wcet_report, generate_report, case_study_tracker - never hand-edit `docs/case_study_status.md`).
- `ros2/`, `docs/`, `cheatsheet/`, `scripts/`, `benchmark/`, `data/`, `cmake/` (minimal - just a config template).

## 4. Critical Architectural Backbone (The "Mental Model")

**Base interfaces** (there is **no** `DiscreteController`/`Estimator`/`Model` base - the template's
assumed names do not exist):
- `IController` `[Ref: lib/IController.h]` - the one controller base. Pure-virtual: `compute(double) -> double`, `reset()`, `sampleTime() const`. Virtual-with-default: `computeVec(VectorXd)`, `signConvention()`, `bumplessInit()`, `isHealthy()`, `hasInternalAntiWindup()`, `name()`.
- `IControllerObserver` `[Ref: lib/IControllerObserver.h]` - telemetry sink: `onCompute/onComputeVec/onReset/onState`.
- **Estimators and plant models share no base** - `KalmanFilter`/`EKF`/`UKF`/`MovingHorizonEstimator` and `PlantModel.h`'s `StateSpace`/`TransferFunction` each expose a purpose-fit API. `[Inferred from: lib/KalmanFilter.h, lib/PlantModel.h]`
- **Stateless exception:** `DiscreteLQR` is pure math (gain via `solveDARE`), wrapped by `LQRAdapter` (the `IController`). Do not copy this split for stateful controllers. `[Ref: lib/DiscreteLQR.h]`

**The Corrector-Pattern Suite** - these are **`ControllerStack` modes + one manual pattern**, NOT
standalone classes (no `Cascade`/`Additive`/`ObserverSF`/`Supervisory` types exist) `[Ref: lib/ControllerStack.h]`:
- **Cascade** = `ControllerStack` **Additive** mode, inner fast / outer setpoint. `[Ref: examples/ex42_pid_inner_mpc_outer.cpp]`
- **Additive** = Additive mode, `u = u_primary + u_corrector`. `[Ref: examples/ex47_esc_additive_pid.cpp]`
- **Supervisory** = **Supervisory** mode, first entry whose activation condition fires wins (health-aware fallback + bumpless transfer). `[Ref: examples/ex54_bumpless_transfer.cpp]`
- **Observer + state feedback (ObserverSF)** is **wired by hand**, not a class - estimator -> `setState()` -> feedback law:
  ```cpp
  kf.step(y, u_prev);  lqr_adapter.setState(kf.state());  u = lqr_adapter.compute(0);
  ```
  `[Ref: examples/ex50_ekf_mpc.cpp]`
- Other wrappers (all `IController`): `AntiWindupWrapper`, `ComputationalDelayWrapper`, `GainScheduledController`, `SmithPredictor`/`AdaptiveSmithPredictor`.

**Data flow:** `Eigen::VectorXd`/`MatrixXd` are the universal carriers. In `lib/` inputs are passed
by **`const&`**, results returned **by value**; predictive controllers pre-allocate work vectors at
construction and use `.noalias()` in the hot loop. The in-place stepper `ssStep` takes
`Eigen::Ref<VectorXd> x` (mutates `x`); `ssStepCopy` is non-mutating (Python-preferred).
`[Ref: lib/PlantModel.h]` `[Inferred from: lib/SystemAnalysis.h:144 return-by-value]`

## 5. Instantiation-to-Execution Lifecycle (The "Golden Path")

Every study/example follows this shape `[Ref: examples/ex01_tf_pid.cpp]` `[Inferred from: shared case-study/*/sim/main.* structure]`:

1. **Model:** `TransferFunction G({b...},{1,a...}, Ts); StateSpace sys = tf2ss(G);` (or hand-built `StateSpace`).
2. **Controller/estimator:** `DiscretePID pid(PIDParams{...}, Ts);` (tuning struct + `Ts`).
3. **Composition (optional):** wrap with stack/observer/Smith predictor - e.g. `auto s = make_shared<ControllerStack>(StackMode::Supervisory, Ts); s->addController(pid, "PID");`
4. **Step loop:** controllers use **`compute()`**, estimators use **`step()`/`update()`**, plant via `ssStep`/`SimPlant`. Mind the per-controller sign (CONTRIBUTING.md):
   ```cpp
   for (int k=0;k<N;++k){ double u = pid.compute(r - y);              // PID: e = r - y
                          Eigen::VectorXd uv(1); uv<<u; y = ssStep(sys, x, uv)(0); }
   ```
   For runtime param swaps from another thread, read via `AtomicParamBuffer::read()` (section 7).
5. **Termination:** no teardown - controllers/estimators are RAII value types; flush CSV/log buffers; `reset()` to reuse an instance across scenarios.

## 6. Python Bindings Mapping & Nuance

- **Import:** `import ctrl_toolbox as ctrl` - **one flat module, no submodules.** `[Ref: bindings/module.cpp:18]`
- **Naming:** C++ `ctrl::DiscretePID::computeDoM` -> Python `ctrl.DiscretePID.compute_dom` (CamelCase class, **snake_case methods**). `[Ref: bindings/controllers_bindings.cpp:47-61]`
- **Binding rule:** every `IController` subclass is bound with `std::shared_ptr<T>` as the 3rd `py::class_` template arg + `ctrl::IController` base, or `ControllerStack.add_controller()` throws at runtime. `[Ref: bindings/controllers_bindings.cpp:47-48]`
- **Eigen pitfall:** `pybind11/eigen.h` auto-converts NumPy <-> Eigen **by copy** at the boundary (`[Ref: bindings/module.cpp:2]`) - array in is copied, result copied out; fine per-call, but do not assume zero-copy or in-place mutation. `Eigen::Ref<>` out-params (e.g. `ssStep`) are not bindable - use `ss_step_copy`. Python subclassing works via trampolines (`PyIController`). `[Inferred from: bindings/trampoline.h]`

## 7. Real-Time & Deployment Constraints (Recovered Gotchas)

- **Zero-allocation in `compute()`/`step()`:** no `std::vector` `push_back`/`resize`, no `std::deque`, no STL streams, no `std::cout`/`cerr` in the hot path; pre-build MPC/QP matrices at construction; use `.noalias()`; for `n<=4` prefer fixed-size `Eigen::Matrix<double,N,N>`. `[Ref: docs/deployment.md - Zero-Allocation Checklist]`
- **Lock-free params:** the buffer is **`ctrl::AtomicParamBuffer<Params>`** (NOT `LockFreeParameterBuffer` - that name does not exist). Seqlock, single-writer/single-reader, `Params` must be trivially copyable. RT thread: `Params p = buf.read();` (copy under seqlock, ~0 retries). Background thread: `buf.publish(p);`. **Do NOT** use it for `DiscreteMPC`/`GPC` `setPlant()`/`setParams()` (non-trivial matrix rebuilds - double-buffer the whole controller or pause the loop). `[Ref: lib/AtomicParamBuffer.h:58]` `[Ref: docs/deployment.md - AtomicParamBuffer]`
- **NaN contract:** scalar controllers hold last output on non-finite input (PID, LeadLag, SMC, ADRC, LQG, Hinf/H2). Check `isHealthy()` / DARE-QP convergence flags after construction. `[Ref: lib/DiscreteLQG.cpp:63-64]`
- **HAL/RTOS:** implement `ISensor/IActuator/IScheduler`; RTOS schedulers `FreeRTOSScheduler` (`FREERTOS_VERSION`) / `ZephyrScheduler` (`CONFIG_ZEPHYR`). RT build flags `-O2 -fno-exceptions -fno-rtti -fstack-usage` (NaN guards use early-return, not exceptions). `[Ref: lib/hal/]`
- **ROS 2:** templated lifecycle node `ControllerNode<T>` (file is `controller_node.hpp`, **not** `ros2_lifecycle_node.hpp`): `on_configure -> on_activate -> on_deactivate -> on_cleanup/on_shutdown`; topics `~/setpoint`, `~/measurement`, `~/control_output`; applies `u = compute(r - y)`. `[Ref: ros2/ctrl_toolbox_ros2/include/ctrl_toolbox_ros2/controller_node.hpp]`

## 8. Recovered Deprecations & Migration Notes (from `version_1` diff)

`origin/version_1` is the archived **v1 snapshot**; `main` is the **v2 continuation** (no
merge-base - history was re-rooted to drop bloated CSV commits; diff via tree, not log). The
`lib/` delta is **small and entirely additive** (~735+/4-; **0 files deleted -> no removed
classes/methods -> nothing is deprecated**). `[Ref: docs/forensic_reconstruction.md - Phase 0]`

- **New in v2:** `signConvention()` + `SignConvention` enum on `IController` `[Ref: lib/IController.h:29-66]`; `SystemAnalysis::getSingularValues` `[Ref: lib/SystemAnalysis.h:144]`; files `FreqDomainIdentifier`, `LyapunovRobustness`, `WorstCaseSearch`; `DiscreteLQG::compute` NaN-guard.
- **`DiscreteH2`** (`[Ref: lib/DiscreteH2.h]`) and the modified `DiscreteHinf` were working-tree-only at the time of the v1/v2 forensic diff; **both are committed now** - treat them as normal `main` content.
- **Doc-bug fixed in v2:** `DiscreteSMC` takes `e = y - r` (v1 docs wrongly said `e = r - y`; the code was always `y - r`). `[Ref: lib/DiscreteSMC.h:97]`
- **Do NOT suggest** (never existed / template fictions): `include/`/`src/`/`python/` dirs; base classes `DiscreteController`/`Estimator`/`Model`; an `ObserverSF`/`Cascade`/`Supervisory` *class* (use `ControllerStack` modes / manual wiring); `LockFreeParameterBuffer` (use `AtomicParamBuffer`); `ros2_lifecycle_node.hpp` (use `controller_node.hpp`).
