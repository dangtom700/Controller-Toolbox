# Case-Study Co-Pilot Reference

Supplementary quick-reference for scaffolding new case studies via public APIs only.
This does **not** replace `CLAUDE.md` (root) - that file is the canonical session guide
with sign conventions, tribal knowledge, and open items. Read it first. This file exists
purely as a condensed API/structure map for composing new simulations.

**Scope discipline:** treat `lib/` (84 headers / 59 .cpp files at repo root, plus
`lib/embedded/` and `lib/hal/`) as a stable public-API surface to *consume*, not edit.
New case-study work should only touch `case-study/<Study>/`, `bindings/*` (only if adding
a brand-new controller binding per the 8-step checklist in `CLAUDE.md`), and `tools/`.

**Verification status:** pass counts and "built" claims in `docs/PROJECT_MASTER_STATE.md`
are only as current as the last clean `conda run -n soft_robotics -- python run.py`. Don't
assert something is "verified" or "passing" without that run - see CLAUDE.md's "First
Things Every Session" section.

---

## 1. Library Core (consume via public headers)

`lib/` is **flat** - no `lib/control/` or `lib/estimation/` subdirectories. Every class is
`lib/ClassName.{h,cpp}`. Two real subdirs: `lib/embedded/` (header-only float-friendly
templates: `BasicPID.h`, `BasicSMC.h`, `DiscreteIntegrator.h`, `FixedRateFilter.h`,
`RingBuffer.h`) and `lib/hal/` (scheduler/sensor/actuator interfaces for deployment).

Base interface: `lib/IController.h` - `virtual double compute(double error)`,
`virtual void reset()`, `virtual double sampleTime() const`. Most controllers are
`shared_ptr<IController>`-compatible; MIMO ones (`DiscreteLQR`, `DiscreteMPC` via
`computeRef`) take `Eigen::VectorXd` directly and are not `IController` (use
`makeLQRController()` from `lib/DiscreteLQR.h` to adapt an LQR gain into an `IController`
for `design_fn` callbacks - see CLAUDE.md "makeLQRController" gotcha).

Representative classes (constructor + step method):
| Class | Header | Constructor | Step |
|---|---|---|---|
| `DiscretePID` | `DiscretePID.h` | `(const PIDParams&, double Ts)` | `compute(error)` |
| `DiscreteLQR` | `DiscreteLQR.h` | `(const StateSpace&, const LQRParams&)` | `compute(x, x_ref, u_ff)` |
| `DiscreteMPC` | `DiscreteMPC.h` | `(const StateSpace&, const MPCParams&)` | `compute(error)` / `computeRef(x, r_ref)` |
| `DiscreteLQG` | `DiscreteLQG.h` | `(const StateSpace&, LQRParams, Q_noise, R_noise)` | `compute(error)` |
| `DiscreteSMC` | `DiscreteSMC.h` | params + Ts | `compute(y - ref)` (reversed sign!) |
| `DiscreteADRC` | `DiscreteADRC.h` | params + Ts | `compute(r - y)`; needs `omega_o*Ts < 0.5` |
| `ExtremumSeeker` | `ExtremumSeeker.h` | `(const ExtremumSeekerParams&, double Ts)` | `compute(J)` - cost, NOT error |
| `DiscreteLeadLag` | `DiscreteLeadLag.h` | `(const LeadLagParams&, double Ts)` | `compute(error)` |
| `SmithPredictor` | `SmithPredictor.h` | `(shared_ptr<IController> inner, StateSpace delayModel, int delaySteps)` | `compute(r - y)` |
| `RepetitiveController` | `RepetitiveController.h` | params + Ts | `compute(error)` |
| `GeneralizedPredictiveControl` | `GeneralizedPredictiveControl.h` | params | `compute(error)` |
| `NonlinearMPC` / `TubeMPC` | `NonlinearMPC.h` / `TubeMPC.h` | plant + params | `computeRef(x, y_ref)` |
| `KalmanFilter` | `KalmanFilter.h` | `(StateSpace, Q_noise, R_noise, P0=...)` | `predict(u)` / `update(y, u)` |
| `MRACController` | `MRACController.h` | params | `compute(y_plant)` - NOT error |
| `FeedbackLinearisation` | `FeedbackLinearisation.h` | params | `compute(e)` + `setState(x)` |

Full inventory + sign conventions: `docs/DOCUMENTATION.md` (API reference) and the
"Critical Caveats" section of root `CLAUDE.md`.

## 2. Composition Patterns (real classes - verified, no invented names)

- **`GainScheduledController`** (`lib/GainScheduledController.h`) - `Ts` ctor +
  `addSchedulePoint(p, ctrl)`; modes `NearestNeighbor` / `LinearBlend`.
- **`AntiWindupWrapper`** (`lib/AntiWindupWrapper.h`) - wraps any `IController`; Hanus
  conditioning. **Never wrap `DiscretePID`** (it already has built-in `Kb` anti-windup).
- **`ComputationalDelayWrapper`** (`lib/ComputationalDelayWrapper.h`) - one-sample actuator
  delay; first `compute()` returns the held initial value, not a fresh inner output.
- **`ControllerStack`** (`lib/ControllerStack.h`) - `(StackMode, Ts)` with modes
  `Supervisory` (fallback chain), `Additive` (summed outputs), `Weighted` (normalized
  blend). This is the closest built-in to a "cascade/combiner" - there is no separate
  `Cascade` class; outer/inner loop composition is hand-wired in the case study's
  controller file (see Section 4) or via `ControllerStack::Additive`.
- **`AdaptiveSmithPredictor`** (`lib/AdaptiveSmithPredictor.h`) - `SmithPredictor` +
  online cross-correlation delay estimation.

No `Cascade`, `Observer+SF` combinator class, or `TLCS` base class exists in `lib/` -
those patterns are assembled per-case-study (e.g. SMISMO's dual-loop `ValveAllocator`,
or hand-rolled bumpless-transfer logic - see CLAUDE.md "TLCS bumpless transfer" gotcha).

## 3. Utilities & Simulation Scaffolding

- **Integrators**: no standalone RK4/Euler utility class for case-study plants - each
  case study's `*_plant.{h,cpp}` (C++) or `*_plant.py` (Python) implements its own
  `step()` with inline RK4 substeps (pattern: `N_SUBSTEPS` inner steps per outer `Ts`,
  e.g. Solar Cooker uses 10 substeps of 3s per Ts=30s). `lib/embedded/DiscreteIntegrator.h`
  is for embedded-target signal integration, not plant simulation.
- **Signal generators**: no built-in Step/Ramp/Sine/PRBS class - scenarios define
  reference profiles inline (conditional step, lambda for sine, JSON-driven ramp) directly
  in the simulation runner / `main.py`.
- **Logging**: no `DataRecorder` class - CSV is written directly: `std::ofstream` in C++
  runners (header row + per-step row, columns conventionally `t,ref,y,u,iae_cumulative`
  plus study-specific signals), or Python `csv.writer` in `sim/main.py`. Output goes to
  `case-study/<Study>/logs/*.csv`.
- **Template generator**: `tools/new_case_study.py` - scaffolds a brand-new case study:
  ```
  python tools/new_case_study.py case-study/MyPaper.pdf --lang cpp --name "My Study"
  python tools/new_case_study.py case-study/MyPaper.pdf --lang python
  ```
  Generates `CMakeLists.txt`, `README.md`, `config/plant_params.json`,
  `config/scenarios/s0N_*.json` (5 by default), and `sim/{include,src}` (C++) or
  `sim/{main.py, plant.py, controllers.py}` (Python) skeletons. **Use this instead of
  hand-writing boilerplate for a new study.** After generating, the manual work is:
  filling in the plant ODE, picking the controller roster, and tuning gains.
  **Gotcha:** the scaffold's placeholder plant (`x' = -a*x + b*u`) and `OpenLoop` controller
  actually run, so `python sim/main.py`/the built `*_sim` will happily produce real-looking
  `logs/*.csv` before you've implemented anything. `tools/case_study_tracker.py` (regenerate
  `docs/case_study_status.md`) detects this and reports "Open placeholder" until the
  placeholder body is actually replaced - use that as your real "have I started yet" signal,
  not the presence of log files.
- **Analysis tools** (post-hoc, not simulation-time): `tools/metrics.py`,
  `tools/compare_controllers.py`, `tools/monte_carlo.py`, `tools/fault_injector.py`,
  `tools/anova.py`, `tools/wcet_report.py`, `tools/generate_report.py` - see CLAUDE.md
  Part 58 notes for hook requirements (`run_single`, `run_with_fault`, `grey_box_model`).

## 4. Python Bindings

Single flat module: `import ctrl_toolbox as ctrl` - **no submodules** (no
`controller_toolbox.control`, etc.). `bindings/module.cpp` is the `PYBIND11_MODULE` entry
point dispatching to 5 internal (not Python-visible) bind functions:
`plantmodel_bindings.cpp`, `controllers_bindings.cpp`, `estimation_bindings.cpp`,
`advanced_bindings.cpp`, `analysis_bindings.cpp`.

```python
import ctrl_toolbox as ctrl
p = ctrl.PIDParams(); p.Kp = 2.0; p.Ki = 0.5
pid = ctrl.DiscretePID(p, 0.01)   # Ts=0.01
u = pid.compute(error)
```

Params structs use **snake_case** for multi-word fields even though C++ uses camelCase
in some places (`qp_max_iter` not `qpMaxIter` - see CLAUDE.md Python bindings table).
NumPy arrays must be `np.ndarray` (not list) when an Eigen `VectorXd`/`MatrixXd` is
expected (e.g. GA/PSO/DE `lower_bound`/`upper_bound`).

Plotting: examples use plain `matplotlib` (no project-specific plotting wrapper);
`tools/generate_report.py` optionally uses `plotly` for interactive HTML if installed.

## 5. Case-Study Pattern (skeleton)

Standard loop (same shape in C++ and Python): **instantiate plant -> instantiate
controller roster -> for each scenario: reset plant+controller -> step loop (read plant
output, compute error, `ctrl->compute()`, `plant.step(u)`, accumulate IAE, write CSV
row) -> next controller -> next scenario.**

Reference real example (C++): `case-study/Boiler Control/sim/src/main.cpp` +
`sim/include/{boiler_plant.h, controllers.h, simulation_runner.h}`, configs in
`config/plant_params.json` + `config/scenarios/s0N_*.json`.

Reference real example (Python-only): `case-study/Vertical Drill String Mathematical
Review 2025/sim/main.py` (see CLAUDE.md "Python-only case studies" gotcha - these set
`_ROOT = dirname(dirname(dirname(abspath(__file__))))` to find the `ctrl_toolbox` build,
and are discovered by `run.py` Phase 7 - they are **not** in `CMakeLists.txt`/`compile.bat`).

Config placement: per-study `config/plant_params.json` (physical constants, Ts, limits)
and `config/scenarios/s0N_name.json` (one file per scenario: ref profile, disturbance,
sim duration). Tuning gains for hand-tuned controllers live inline in the study's
`controllers.{h,cpp}` / `controllers.py`, not in JSON (GA/PSO/DE-optimized controllers
run their tuning cost function once in the constructor instead).

## 6. Hard Constraints

- **Never edit `lib/*.{h,cpp}`, `lib/embedded/`, `lib/hal/`** as part of case-study work.
  If a case study seems to need new library functionality, surface that as a question
  rather than implementing it directly - it's a different review bar (CLAUDE.md's 8-step
  checklist + Catch2 tests + Python bindings + smoke test, not a quick edit).
  [[exception]] Adding a brand-new controller class through the full 8-step checklist is
  a legitimate, separate workflow described in root `CLAUDE.md` - that's not "editing
  immutable internals," it's the documented extension path. Confirm with the user which
  mode you're in before touching `lib/`.
- **Don't assert verified/passing status** without a fresh `run.py`. Stale counts in
  `docs/PROJECT_MASTER_STATE.md` are explicitly flagged UNVERIFIED in root `CLAUDE.md` -
  that flag is real project signal, not a stale artifact to ignore.
- Use `tools/new_case_study.py` for new studies instead of hand-rolled boilerplate.
- `compile.bat` lists every C++ target explicitly - a new case study's `.exe`/regression
  target must be added there (and to `case-study/CMakeLists.txt`) or it silently won't build.
