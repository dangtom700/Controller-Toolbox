# Controller Toolbox - Forensic Reconstruction Report

*A ground-up, source-only reconstruction of the library's structure, API, and v1->v2
evolution. Every architectural claim is cited to a file (and line where useful). Inferences
that go beyond what the source states literally are tagged `[INFERRED ...]`.*

> **Scope note / premise correction.** This report was commissioned under the assumption that
> `docs/DOCUMENTATION.md` and `docs/deployment.md` were "corrupted or unavailable" and that the
> codebase used a generic layout (`include/`, `src/`, `python/`, namespaces
> `ctrl`/`sysid`/`estimation`/`optim`, a `ros2_lifecycle_node.hpp`). Reconnaissance refuted that
> premise: both documents exist and are complete, and none of the assumed layout is accurate.
> The actual ground truth is documented below and the assumed-vs-actual deltas are called out
> explicitly in Phase 1 and Phase 6. This report is therefore a *verification and differential*
> reconstruction, not a from-scratch recovery.

---

## Table of Contents

- [Phase 0 - Historical Differential (v1 -> v2) and Breaking-Changes Log](#phase-0)
- [Phase 1 - Repository Topography and Build](#phase-1)
- [Phase 2 - Architectural Backbone and Layering](#phase-2)
- [Phase 3 - API and pybind11 Mapping](#phase-3)
- [Phase 4 - Data Flow and Memory Management](#phase-4)
- [Phase 5 - Case-Study Lifecycles](#phase-5)
- [Phase 6 - Deployment and Hardware Abstraction](#phase-6)
- [Phase 7 - Anomaly Detection and Cross-Validation](#phase-7)
- [Executive Summary](#executive-summary)

---

<a name="phase-0"></a>
## Phase 0 - Historical Differential (v1 -> v2) and Breaking-Changes Log

### 0.1 What `version_1` is, and how to diff it

`origin/version_1` is the **archived v1 snapshot**; `main` is its **v2 continuation**. The two
branches share **no git merge-base** - but that is a deliberate *history-truncation artifact*,
not two unrelated codebases. `main` was re-rooted to shed a long history of repeatedly-committed
CSV data files (`version_1` carries 123 commits; `main` carries 10). The mass of the raw tree
diff (~1346 files, ~3M deleted lines) is concentrated in `case-study/` (1109 files), `data/`
(52), and `examples/` - i.e. bulk content/telemetry, not library code.

Because there is no shared ancestor, the canonical v1->v2 differential is the
**snapshot-to-snapshot tree diff**, and commit-log mining must be done **per branch**:

```
git diff origin/version_1 main -- lib/        # the library delta (small, tractable)
git log origin/version_1 --grep '...'         # v1-era decision history
git log main           --grep '...'           # v2-era decision history
```

The **library API delta is small and almost entirely additive**: `git diff --stat
origin/version_1 main -- lib/` reports **735 insertions, 4 deletions across 22 files**. The 4
deletions are doc-comment lines replaced in `lib/DiscreteSMC.h` (see 0.4), not removed code.

### 0.2 Differential Class Mapping

| Class / file | v1 -> v2 status | Notes |
|---|---|---|
| `lib/IController.h` | **Signature extended** (additive) | New `enum class SignConvention` + new `virtual SignConvention signConvention() const` (default `Unspecified`). Defined `lib/IController.h:29-54` (enum), `:60-66` (method). Non-breaking: pure-virtual set unchanged. |
| `lib/FreqDomainIdentifier.{h,cpp}` | **`[v2+ Feature]`** | New: Levy's-method frequency-domain TF identification. `lib/FreqDomainIdentifier.h:52-72`. |
| `lib/LyapunovRobustness.h` | **`[v2+ Feature]`** | New: common quadratic Lyapunov search for polytopic uncertainty. `lib/LyapunovRobustness.h:88` (`findCommonLyapunov`). |
| `lib/WorstCaseSearch.h` | **`[v2+ Feature]`** | New: CMA-ES worst-case parameter search. `lib/WorstCaseSearch.h:144,172,202` (`findWorstCaseSensitivity/IAE/`). |
| `lib/SystemAnalysis.{h,cpp}` | **Method added** (additive) | New `static std::vector<Eigen::VectorXd> getSingularValues(...)` for MIMO singular values. `lib/SystemAnalysis.h:143-144`. |
| `lib/DiscreteLQG.cpp` | **Behavior hardened** | NaN-guard added to `compute()` (hold-last on non-finite input). `lib/DiscreteLQG.cpp:63-64`. |
| `lib/DiscreteSMC.h` | **Doc-bug fix + sign API** | Class docs corrected to `e = y - r` (v1 incorrectly documented `e = r - y`); `signConvention()` override added. See 0.4. |
| `lib/DiscretePID.h`, `DiscreteMPC.h`, `NonlinearMPC.h`, `SmithPredictor.h` | **Sign API added** | `signConvention() -> TrackingErrorRMinusY`. +2 lines each. |
| `lib/ExtremumSeeker.h` | **Sign API added** | `-> CostSignal`. |
| `lib/L1AdaptiveController.h`, `MRACController.h` | **Sign API added** | `-> PlantOutput`. |
| `lib/DiscreteADRC.h`, `DiscreteLQR.h`, `FeedbackLinearisation.h`, `GeneralizedPredictiveControl.h` | **Sign API added** | `-> Other` (reference supplied out-of-band, or convention is plant-dependent). |
| `lib/ControllerToolbox.h` | **Umbrella updated** | +3 includes for the new files. `lib/ControllerToolbox.h:131-133`. |
| `lib/CMakeLists.txt` | **Build wiring** | `FreqDomainIdentifier.cpp` added to the core source list. |
| (any lib class) | **`[DEPRECATED]`** | **None.** Zero lib files deleted v1->v2; no public class or method was removed. |

### 0.3 The headline v2 change: runtime sign-convention introspection

The single dominant v2 theme is a new **runtime-queryable sign-convention API** on the base
interface (`lib/IController.h:29-66`). The motivation is stated in the header itself: lib/
deliberately does *not* enforce one `compute(signal)` convention across controllers, and prior
to v2 a caller could only learn a controller's convention from prose docs. `signConvention()`
makes it queryable at runtime. The per-controller mapping audited in v2:

| Convention | Controllers |
|---|---|
| `TrackingErrorRMinusY` (`e = r - y`) | DiscretePID, DiscreteMPC, NonlinearMPC, SmithPredictor |
| `TrackingErrorYMinusR` (`e = y - r`) | DiscreteSMC, SuperTwistingSMC |
| `PlantOutput` (raw `y`) | MRACController, L1AdaptiveController |
| `CostSignal` (objective to extremise) | ExtremumSeeker |
| `Other` (out-of-band / plant-dependent) | DiscreteLQR (via LQRAdapter), DiscreteADRC, FeedbackLinearisation, GeneralizedPredictiveControl |
| `Unspecified` (default, not yet audited) | every controller without an override |

`[INFERRED from lib/IController.h:30-45 + the override sites]` This is a non-breaking, opt-in
introspection layer: the default return is `Unspecified`, so unaudited controllers compile and
behave exactly as in v1.

### 0.4 Reconstructed "why" (per the Inference Rule)

- **DiscreteSMC doc correction.** v1 `lib/DiscreteSMC.h` documented `compute()` as taking
  `e = r - y`, but the sliding law `u = -K*sat(s/phi)` requires `s` (and therefore the error) to
  grow with `y - r` for a positive-gain plant. v2 corrects the doc to `e = y - r` and encodes it
  as `signConvention() -> TrackingErrorYMinusR`. `[INFERRED]` the v1 doc was simply wrong about
  its own code; the v2 change fixes documentation/clarity, not behavior - the sliding-surface
  math was already `y - r`.
- **DiscreteLQG NaN-guard.** v2 adds `if (!std::isfinite(y_scalar)) return <last u>` to
  `compute()` (`lib/DiscreteLQG.cpp:63-64`). `[INFERRED]` this brings LQG in line with the
  project-wide hold-last-on-bad-sensor contract that the other scalar controllers already
  honored - a numerical-robustness hardening, not a feature.
- **New robustness/ID trio.** `WorstCaseSearch`, `LyapunovRobustness`, and `FreqDomainIdentifier`
  are explicitly tagged in their own headers as roadmap phases ("Phase 4/5 of the
  robustness-analysis roadmap", `docs/robust_implementation_plan.md`; "Phase 4 Iteration 2") -
  i.e. planned capability growth on top of the existing `RobustnessAnalysis`/`MuAnalysis` suite.

### 0.5 Commit-log timeline (mined per branch)

`version_1` (v1-era, `git log origin/version_1 --grep refactor|API|break|fix -i`) shows the
formative architectural moves, e.g. `4de5ad1 Refactor controller stack for health checks and
supervisory mode`, `61698c7 feat: add FunctionApproximator ...`, `8687531 Refactor code
structure for improved readability`, plus a long tail of CI/workflow fixes. `main` (v2-era)
carries only post-truncation commits (`fd91620 fix the loss DOCS`, `f7414e2 fix the
CMAKELISTS.txt`). Because the histories are severed at the truncation cut, the two logs cannot be
linearised into one `git log`; the architectural-decision timeline is "v1 history, then a
re-root, then v2 history."

### 0.6 Beyond both branches: working-tree-only additions

These exist in the working tree but are **not yet committed to `main`**, so they do not appear in
the `version_1..main` diff and are best read directly from source:

- `lib/DiscreteH2.{h,cpp}` - discrete-time H2/LQG dynamic output-feedback synthesis, sharing the
  `GeneralisedPlant` format with DiscreteHinf. `lib/DiscreteH2.h:101-179`. Guarded by
  `CTRL_HAS_HINF`; registers feature `h2_synthesis`.
- `lib/DiscreteHinf.*` - modified (algorithm comments/solver refinements).

---

<a name="phase-1"></a>
## Phase 1 - Repository Topography and Build

### 1.1 Actual top-level layout (assumed-vs-actual)

| Generic-template assumption | Actual ground truth |
|---|---|
| `include/` for public headers | **Does not exist.** Public headers live directly in flat `lib/` (`lib/ClassName.h`). |
| `src/` for sources | **Does not exist.** Sources are `lib/ClassName.cpp`, plus `examples/`, `tests/`, `bindings/`. |
| `python/` for bindings | **Does not exist.** Bindings live in `bindings/`; Python examples in `examples/python/`. |
| `cmake/` holds build logic | Exists but minimal - only `cmake/ControllerToolboxConfig.cmake.in`. Logic is in the root `CMakeLists.txt`. |

Real top-level directories: `lib/` (flat core, ~150 files; subdirs `lib/embedded/` and
`lib/hal/`), `bindings/` (pybind11), `examples/` (+ `examples/python/`, `examples/cpp/`,
`examples/embedded/`), `tests/`, `case-study/` (+ `case-study/common/`), `tools/`, `scripts/`,
`docs/`, `cheatsheet/`, `ros2/`, `benchmark/`, `data/`, `cmake/`.

### 1.2 Build system and external dependencies

Root `CMakeLists.txt`: C++20, CMake >= 3.16; MinGW/UCRT64 builds statically link
`libgcc`/`libstdc++`/`-static` so executables and the `ctrl_toolbox` `.pyd` run without MSYS2 on
`PATH`. Subdir aggregation order: `lib` -> `tests` -> `examples` -> `scripts` -> `case-study`
-> `benchmark` (opt) -> `bindings` (opt).

| Dependency | Version | How acquired |
|---|---|---|
| Eigen3 | >= 3.4.0 | `find_package`, optional `FetchContent` (`CTRL_FETCH_EIGEN_IF_MISSING`) |
| nlohmann/json | v3.11.3 | `find_package` CONFIG, `FetchContent` fallback |
| Catch2 | v3.5.4 | `FetchContent` (tests) |
| pybind11 | v2.13.6 | `find_package` CONFIG, `FetchContent` fallback (bindings) |
| Python3 | 3.8+ | `find_package` (bindings) |
| Doxygen | optional | `find_package QUIET` -> `docs` target |

**Feature flags** (`lib/CMakeLists.txt`, all ON by default): `CTRL_ENABLE_HINF`,
`CTRL_ENABLE_SUBSPACE`, `CTRL_ENABLE_FUZZY`, `CTRL_ENABLE_FUNCTION_APPROX`,
`CTRL_ENABLE_ADVANCED_KALMAN` (each defines a matching `CTRL_HAS_*` macro). Build-mode flags
(root): `CTRL_BUILD_EMBEDDED_ONLY`, `CTRL_FETCH_EIGEN_IF_MISSING`, `CTRL_BUILD_PYTHON_BINDINGS`,
`CTRL_BUILD_BENCHMARKS`. `lib/Features.h` exposes the live set via `ctrl::features()`.

**Build entry points.** `compile.bat` / `compile.sh` build ~155 targets **sequentially** from a
hand-maintained list (a target not listed silently never builds). `run.py` is the canonical
"is everything passing" command: 8 phases (ASCII scan, NaN-guard scan, sequential compile,
bindings + smoke test, run every `.exe`, run every Python example, run every Python-only case
study, regenerate `docs/case_study_status.md` + `docs/report.html`).

---

<a name="phase-2"></a>
## Phase 2 - Architectural Backbone and Layering

### 2.1 Namespaces

There is a **single primary namespace `ctrl`** (not the assumed `ctrl`/`sysid`/`estimation`/
`optim` split), with two sub-namespaces: `ctrl::tag` (controller category tags in
`lib/ControllerTraits.h`) and `ctrl::detail` (internal helpers, e.g.
`lib/WorstCaseSearch.h:59`). `[INFERRED from a sweep of `namespace` declarations across lib/
headers]`.

### 2.2 Base interfaces

- `IController` (`lib/IController.h`) - base for all controllers. Pure-virtual `compute(double)`,
  `reset()`, `sampleTime()`; virtual-with-default `computeVec()`, `signConvention()` (v2),
  `bumplessInit()`, `isHealthy()`, `hasInternalAntiWindup()`, `name()`; observer/telemetry hooks.
- `IControllerObserver` (`lib/IControllerObserver.h`) - telemetry sink:
  `onCompute`/`onComputeVec`/`onReset`/`onState`.

There is **no shared abstract base** for plant models, estimators, or system-ID. `PlantModel.h`
provides value types (`TransferFunction`, `StateSpace`) + free functions (`tf2ss`, `c2d`,
`ssStep`, ...); estimators (`KalmanFilter`, `EKF`, `UKF`, `ParticleFilter`, `MovingHorizonEstimator`)
and identifiers (`FOPDTIdentifier`, `SubspaceID`, `RecursiveLeastSquares`, `FreqDomainIdentifier`,
...) each expose a purpose-fit interface rather than a common one. `[INFERRED]`.

### 2.3 The DiscreteLQR / LQRAdapter exception

`DiscreteLQR` is deliberately **stateless** pure math (a gain computed once via `solveDARE`),
*not* an `IController`. `LQRAdapter` is a thin `IController` shim wiring it to state/reference
providers. The split exists because the gain is genuinely stateless and shareable; it is an
intentional one-off, not a pattern to copy (see `CONTRIBUTING.md#architecture-pattern`).

### 2.4 Composition / corrector patterns

Composites **wrap** an `IController` (composition over inheritance) and are themselves
`IController`s, so they nest:

- `ControllerStack` (`lib/ControllerStack.h`) - Supervisory / Additive / Weighted dispatch with
  health-aware fallback and bumpless transfer. This is the substrate for the "corrector"
  patterns: **Cascade** and **Additive** via `Additive` mode, **Supervisory/bumpless** via
  `Supervisory` mode; **Observer+state-feedback** is wired manually (estimator -> `setState()`).
- `AntiWindupWrapper` - Hanus (1987) conditioning around any inner controller.
- `ComputationalDelayWrapper` - one-sample actuator delay (`u[k] = u_inner[k-1]`).
- `GainScheduledController` - NearestNeighbor / LinearBlend over scheduled controllers.
- `SmithPredictor` / `AdaptiveSmithPredictor` - dead-time compensation around an inner controller
  plus a delay-free plant model.

### 2.5 Categorized class inventory (representative)

PID-family (`DiscretePID`, `FuzzyPID`, `NeuralPID`, embedded `BasicPID`); sliding-mode
(`DiscreteSMC`, `SuperTwistingSMC`, embedded `BasicSMC`); optimal (`DiscreteLQR`/`LQRAdapter`,
`DiscreteLQG`); predictive (`DiscreteMPC`, `GeneralizedPredictiveControl`, `NonlinearMPC`,
`TubeMPC`, `ScenarioMPC`, `HybridMPC`, `CEMController`, `DeePC`); robust (`DiscreteHinf`,
`DiscreteH2` [working-tree]); adaptive/learning (`MRACController`, `L1AdaptiveController`,
`ExtremumSeeker`, `IterativeLearningControl`, `DynaController`, `AutoTuner`); ADRC
(`DiscreteADRC`); frequency-domain (`DiscreteLeadLag`, `RepetitiveController`,
`ZeroPhaseTrackingFilter`); nonlinear (`FeedbackLinearisation`, `CBFSafetyFilter`); estimation
(`KalmanFilter`, `ExtendedKalmanFilter`, `UnscentedKalmanFilter`, `ParticleFilter`,
`MovingHorizonEstimator`, grey-box variants); identification (`FOPDTIdentifier`,
`SOPDTIdentifier`, `RecursiveLeastSquares`, `SubspaceID`, `LPVSystemID`, `SINDy`, `KoopmanEDMD`,
`FreqDomainIdentifier`); data-driven/ML (`GaussianProcess`, `EchoStateNetwork`,
`FunctionApproximator`, hybrid model classes); optimization (`BayesianOptimizer`,
`GeneticAlgorithm`, `ParticleSwarmOptimizer`, `DifferentialEvolution`); analysis/robustness
(`SystemAnalysis`, `MetricsAnalyzer`, `BalancedTruncation`, `GapMetric`, `RobustnessAnalysis`,
`MuAnalysis`, `WorstCaseSearch`, `LyapunovRobustness`); monitoring (`ControllerMonitor`,
`MismatchDetector`).

---

<a name="phase-3"></a>
## Phase 3 - API and pybind11 Mapping

### 3.1 Module structure

One flat module `ctrl_toolbox` (no submodules). `bindings/module.cpp:18`
(`PYBIND11_MODULE(ctrl_toolbox, m)`) dispatches to `bind_plantmodel/controllers/estimation/
advanced/analysis` (`bindings/module.cpp:52-56`). Eigen <-> NumPy conversion is enabled by
`#include <pybind11/eigen.h>` (`bindings/module.cpp:2`); `Eigen::MatrixXd`/`VectorXd` cross the
boundary as NumPy arrays automatically.

### 3.2 Binding conventions (hard rules)

- Every `IController` subclass is bound with `std::shared_ptr<T>` as the third `py::class_`
  template argument and `ctrl::IController` as the base, or `ControllerStack.add_controller()`
  throws at runtime. Example: `bindings/controllers_bindings.cpp:47-48`.
- Python-side names are snake_case wrappers over camelCase C++ methods.
- Out-parameter functions taking `Eigen::Ref<>` (e.g. `ssStep`) are not directly bindable; the
  non-mutating `ssStepCopy` is the Python-preferred variant.
- Python subclassing of the base interfaces is enabled by trampoline classes (`PyIController`,
  `PyIControllerObserver`).

### 3.3 Representative Python -> C++ mapping

| Python | C++ | Citation |
|---|---|---|
| `ctrl_toolbox` module | `PYBIND11_MODULE(ctrl_toolbox, m)` | `bindings/module.cpp:18` |
| `DiscretePID` class | `py::class_<ctrl::DiscretePID, ctrl::IController, shared_ptr<...>>` | `bindings/controllers_bindings.cpp:47-48` |
| `DiscretePID.compute(error)` | `ctrl::DiscretePID::compute` | `bindings/controllers_bindings.cpp:60` |
| `DiscretePID.compute_dom(y, r)` | `ctrl::DiscretePID::computeDoM` | `bindings/controllers_bindings.cpp:61` |
| `DiscretePID.set_params(params)` | `ctrl::DiscretePID::setParams` | `bindings/controllers_bindings.cpp:65` |
| `DiscreteSMC.compute(error)` | `ctrl::DiscreteSMC::compute` | `bindings/controllers_bindings.cpp:119` |
| `DiscreteLeadLag.compute(signal)` | `ctrl::DiscreteLeadLag::compute` | `bindings/controllers_bindings.cpp:89` |

`[INFERRED from the binding source]` estimator/analysis/advanced classes follow the identical
`py::class_<...>().def(...)` pattern in their respective `bindings/*_bindings.cpp` files.

---

<a name="phase-4"></a>
## Phase 4 - Data Flow and Memory Management

### 4.1 Eigen across layers

Within `lib/`, Eigen matrices/vectors are passed by `const&` for inputs and returned by value for
results (e.g. `SystemAnalysis::getSingularValues(...) -> std::vector<Eigen::VectorXd>`,
`lib/SystemAnalysis.h:143-144`). The in-place stepping helper `ssStep` takes
`Eigen::Ref<Eigen::VectorXd> x` and mutates it; `ssStepCopy` is the non-mutating equivalent used
from Python. `[INFERRED]` predictive controllers pre-allocate work vectors at construction and
use `.noalias()` in the hot path (consistent with the zero-allocation guidance in
`deployment.md`).

### 4.2 Lock-free parameter handoff (AtomicParamBuffer)

The lock-free buffer is `ctrl::AtomicParamBuffer<Params>` (`lib/AtomicParamBuffer.h:58`) - **not**
a class named `LockFreeParameterBuffer` (the generic template's assumed name does not exist). It
is a **seqlock** with two `Params` slots and an atomic sequence counter `seq_`
(`lib/AtomicParamBuffer.h:69`):

1. RT (reader) thread: `Params p = buf.read()` returns a copy under seqlock protection; spins
   only if a publish is mid-flight (typically zero retries). `lib/AtomicParamBuffer.h:78`.
2. Background (writer) thread: `buf.publish(p)` writes the inactive slot, then increments `seq_`
   twice (odd = in-progress, even = done) and flips the active slot.

Constraint: `Params` must be trivially copyable (enforced by `static_assert`); single-writer /
single-reader only. The RT-read / background-publish lifecycle is demonstrated in the
`DiscretePID` + tuner example traced in `docs/DOCUMENTATION.md` 4.7 and `docs/deployment.md` 2.

---

<a name="phase-5"></a>
## Phase 5 - Case-Study Lifecycles

All studies share one shape regardless of language: instantiate plant -> instantiate a controller
roster -> for each scenario {reset, step loop: error -> `compute()` -> `plant.step(u)` ->
accumulate IAE -> write a CSV row} -> next controller -> next scenario. C++ studies build a
`*_sim` executable (registered in `case-study/CMakeLists.txt` *and* `compile.bat`/`.sh`, or the
target silently does not build) and are run by `run.py` Phase 5; Python-only studies have just
`sim/main.py`, are auto-discovered by `run.py` Phase 7, and locate the bindings build four
directories up.

Representative complex studies (see `docs/DOCUMENTATION.md` 3.3 for the full roster with
controller counts): 6-DOF Stewart Platform Vessel Motion Simulator; Active Suspension 6x6 EV Full
Model (40-state, 20-DOF); Nonlinear Surface Ship Manoeuvring (3-DOF MMG); Multi-Body Floating
Wind-Wave Platform; Tracking Control of Electro-Hydraulic Force Servo Systems; Separate Meter In
Separate Meter Out (8-state hydraulic); Non-Inverting Buck-Boost Converter (50 kHz, mode
hysteresis); Boiler Control (Bell-Astrom 3x3 MIMO); Tug Boat (3-DOF + thrust allocation); Porous
Fiber Plate Humidification (room ODE + sensor delay). Scaffold new studies with
`tools/new_case_study.py`; check a study's own `README.md` (not the presence of `logs/`) to tell
real content from a placeholder.

---

<a name="phase-6"></a>
## Phase 6 - Deployment and Hardware Abstraction

### 6.1 HAL (`lib/hal/`)

Interfaces: `ISensor` (`read`/`isValid`/`reset`), `IActuator` (`write`/`lastOutput`/`reset`),
`ITimer` (`nowNs`/`elapsedNs`), `IScheduler` (`setPeriodNs`/`setCallback`/`start`/`stop`/
`tickCount`/`overrunCount`). Concrete sim adapters: `SimPlant`, `SimSensor`, `SimActuator`
(NaN-safe saturated write), `SafeSensor` (freezes last-valid on validity loss), `SimScheduler`,
`StdTimer`. RTOS schedulers: `FreeRTOSScheduler` (guarded by `FREERTOS_VERSION`),
`ZephyrScheduler` (guarded by `CONFIG_ZEPHYR`).

### 6.2 ROS 2 adapter (assumed-vs-actual)

The generic template assumed `ros2_lifecycle_node.hpp`. The actual adapter is
**`ros2/ctrl_toolbox_ros2/include/ctrl_toolbox_ros2/controller_node.hpp`** - a templated
lifecycle node `ControllerNode<T>` wrapping an `IController` subclass, with topics `~/setpoint`,
`~/measurement`, `~/control_output`, parameter `sample_time_s`, and lifecycle transitions
`on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `on_shutdown`. It applies
`u = controller->compute(r - y)` (the `TrackingErrorRMinusY` convention). `[INFERRED from
controller_node.hpp]`.

### 6.3 Embedded subset and zero-allocation

`lib/embedded/` is the header-only, no-Eigen, no-virtual-dispatch MCU subset: `BasicPID<Scalar>`,
`BasicSMC<Scalar>`, `DiscreteIntegrator<Scalar>`, `FixedRateFilter<Scalar,Order>`,
`RingBuffer<Scalar,Capacity>` (all fixed-size, no heap). Compiled standalone via
`CTRL_BUILD_EMBEDDED_ONLY`. The zero-allocation checklist, per-controller stack-size estimates,
RTOS pinning/`mlockall` guidance, and RT compiler flags (`-O2 -fno-exceptions -fno-rtti
-fstack-usage`) live in `docs/deployment.md` 2. WCET tooling is `tools/wcet_report.py`
(quantile-based WCET from `case-study/**/wcet_*.csv`).

---

<a name="phase-7"></a>
## Phase 7 - Anomaly Detection and Cross-Validation

- **Tests vs examples.** Catch2 is the primary framework (`tests/test_catch2_advanced.cpp`
  carries ~95 cases with a large tag set; `tests/test_catch2_pilot.cpp` the pilot set), plus a
  legacy hand-rolled harness (`tests/test_controllers.cpp`, `test_tuners_extended.cpp`,
  `test_integration.cpp`) and per-study regression suites (boiler/tug/solar/humid/susp/
  buck-boost/solar-cooker/sotec/smismo). `test_embedded_subset.cpp` verifies the embedded subset
  has zero Eigen dependency.
- **Untested-API flags.** `[INFERRED]` The new v2 surface is only partially covered: there are no
  dedicated Catch2 tags for `getSingularValues`, `WorstCaseSearch`, `LyapunovRobustness`,
  `FreqDomainIdentifier`, or `signConvention()` visible in the `test_catch2_advanced.cpp` tag
  list documented in `docs/DOCUMENTATION.md` 3.4. The DiscreteH2 path
  (`examples/ex88_h2_synthesis.cpp`, `examples/python/ex108_h2_synthesis.py`) is exercised by
  example, not by a Catch2 case. **Recommendation:** add Catch2 coverage for the v2 additions and
  a `signConvention()` audit test that fails if a controller's declared convention disagrees with
  its documented one.
- **Python usage consistency.** `[INFERRED from >=3 examples + smoke test]` The intended Python
  idiom is: build the plant via `TransferFunction`/`StateSpace`, construct controllers with
  snake_case methods, and prefer `ssStepCopy` over `ssStep` in simulation loops. Verify the live
  status with `conda run -n soft_robotics -- python run.py` (do not trust stale pass counts).

---

<a name="executive-summary"></a>
## Executive Summary - architectural philosophy (5 points)

1. **One flat interface, many algorithms.** Almost every algorithm implements both its math and
   the `IController` interface in a single class behind one base contract
   (`compute`/`reset`/`sampleTime`); the only deliberate split is the stateless
   `DiscreteLQR`/`LQRAdapter`.
2. **Composition over inheritance for cross-cutting concerns.** Anti-windup, computational delay,
   gain scheduling, dead-time compensation, and multi-controller supervision are all wrappers
   that are themselves `IController`s and therefore nest.
3. **Conventions are explicit, not enforced.** Sign conventions intentionally differ per
   controller; v2's contribution is to make the convention *queryable at runtime*
   (`signConvention()`) rather than to homogenize it - additive, opt-in, non-breaking.
4. **Numerical safety as a standing contract.** Hold-last-on-NaN, DARE/QP convergence flags
   (`isHealthy()`), and `.ldlt()`-with-status are pervasive; v2 extended the contract to
   `DiscreteLQG`.
5. **Deployment is a first-class concern.** A header-only no-Eigen embedded subset, a lock-free
   seqlock parameter buffer, HAL + RTOS scheduler abstractions, a ROS 2 lifecycle node, and WCET
   tooling exist specifically to take the same controllers from simulation to real hardware.

*See also: [DOCUMENTATION.md](DOCUMENTATION.md) (full API reference) and
[deployment.md](deployment.md) (parameter constraints, RT integration, troubleshooting).*
