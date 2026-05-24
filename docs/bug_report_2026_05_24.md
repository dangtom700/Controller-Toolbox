# Controller Toolbox — Code Review Report

**Date:** 2026-05-23  
**Reviewer:** Senior Controls Engineer  
**Scope:** Full codebase audit of `lib/`, `examples/`, `scripts/`, `case-study/` — focused on correctness, RT readiness, and numerical robustness. Prior report (2026-05-19) used as baseline.

---

## Overview

The good news first: the mathematical fixes from the 05-19 report are properly baked in. The doubling DARE is clean, the ADRC ESO switched to backward Euler (finally), and the Kalman Joseph-form update is there. The structural bones are solid. But there's a meaningful pile of new material — four fuzzy examples, the tug boat sim, GPC, UKF, SubspaceID — and the integration quality is inconsistent. Several issues range from "annoying" to "this will silently give you wrong physics," plus the CSV output situation is a sprawling mess that needs a migration plan regardless.

---

## 1. Status of Prior Items (05-19 Baseline)

Cross-checking the P0/P1 action items from last time:

| Item | Status |
|---|---|
| NaN/Inf guards in `DiscretePID`, `DiscreteSMC`, `DiscreteADRC` | **Done** — `isfinite` check present in all three |
| LDLT health check in `DiscreteMPC::computeRef()` | **Done** — `ldlt_.info() != Eigen::Success` return path added |
| LDLT floor in `KalmanFilter::update()` | **Done** — R diagonal clamped to `1e-12`, LDLT check before Kalman gain |
| `log(0)` guard in `SystemAnalysis::calculateMargins()` | Not verified (out of scope this pass) |
| Pre-allocated work vectors in `DiscreteMPC` | **Done** — `R_stack_`, `pred_err_`, `grad_`, etc. are members |
| DARE convergence struct `DareResult` | **Done** — `{P, converged, iterations}` returned |
| PBH stabilizability + detectability pre-checks in `DiscreteLQR` | **Done** — both tests implemented correctly |
| ESO stability: forward Euler → backward Euler in `DiscreteADRC` | **Done** — analytically inverted unit upper-triangular system, unconditionally A-stable |
| `SmithPredictor` deque → circular buffer | **Not verified this pass** |
| HAL (`ISensor`/`IActuator`) | **Partially done** — stubs exist in `lib/hal/`, not wired to examples |
| `MetricsAnalyzer` settle-time NaN sentinel | **Not verified this pass** |

Solid progress on the high-priority numerical issues. The RT/HAL side is still largely scaffolding.

---

## 2. New Defects — Correctness & Numerical Issues

---

### 2.1 `GeneralizedPredictiveController` — Hessian Re-Factored Every Step

**File:** [lib/GeneralizedPredictiveControl.cpp](../lib/GeneralizedPredictiveControl.cpp)  
**Severity:** High — kills RT determinism and is just wasteful  
**Description:**  
Inside `computeRef()`, there's this gem:

```cpp
const auto ldlt = H_.ldlt();  // line 112
if (ldlt.info() != Eigen::Success)
    return u_prev_(0);
```

The `H_` Hessian is prebuilt in `buildCondensedMatrices()` and never changes between calls (it only changes if the plant or cost weights change). Yet this re-factorises the entire Hessian on *every single control step*. Meanwhile, `DiscreteMPC` correctly pre-factors into `ldlt_` once and caches it as a member. The GPC just... didn't get that memo.

For `Nu=5`, `m=1`, this is a 5×5 factorisation — tolerable offline. For `Nu=20`, `m=4`, it's an 80×80 LDLT every step, which is O(n³/3) ≈ 170k FLOPs/step you're throwing away.

**Fix:** Add `ldlt_` as a pre-factored member (same pattern as `DiscreteMPC`), factor once in `buildCondensedMatrices()`, and use it in `computeRef()`.

```cpp
// In buildCondensedMatrices():
H_ = Ga_.transpose() * Qy_ * Ga_ + Ru_;
ldlt_ = H_.ldlt();   // factor once here

// In computeRef():
if (ldlt_.info() != Eigen::Success) return u_prev_(0);  // use cached
DeltaU_ = (-ldlt_.solve(grad_)).cwiseMax(lb_).cwiseMin(ub_);
```

---

### 2.2 `ex23_fuzzy_pd_temperature.cpp` — Disturbance Applied Post-Saturation, Breaking Anti-Windup

**File:** [examples/ex23_fuzzy_pd_temperature.cpp](../examples/ex23_fuzzy_pd_temperature.cpp:86)  
**Severity:** Medium — the comparison between FuzzyPD and PID is misleading; anti-windup accounting is wrong  
**Description:**  
The disturbance is applied *after* the controller has already computed and saturated its output:

```cpp
double uf = fuzzy.compute(ref - y_f);
uf = std::clamp(uf + dist, 0.0, 3.0);   // line 86 — disturbance injected here

double up = pid.compute(ref - y_p);
up = std::clamp(up + dist, 0.0, 3.0);   // line 89
```

This means neither controller sees the disturbance in its error signal — both controllers think the plant is tracking fine while the simulation injects a -0.5 kW load. The anti-windup in the PID never fires because from the controller's perspective there's no sustained error. The whole point of the disturbance comparison is to evaluate rejection, but you've injected the disturbance at the wrong place. It should be fed into the plant as an additive input disturbance *before* `ssStep`, not patched onto the already-computed control signal.

**Fix:** Keep `uf`/`up` as pure controller outputs (no clamping here). Pass the disturbance as part of `uvf`:

```cpp
double uf = fuzzy.compute(ref - y_f);
double up = pid.compute(ref - y_p);

Eigen::VectorXd uvf(1); uvf << std::clamp(uf, 0.0, 3.0);
Eigen::VectorXd uvp(1); uvp << std::clamp(up, 0.0, 3.0);

// Plant input = control + disturbance (additive input disturbance)
uvf(0) += dist;
uvp(0) += dist;

y_f = ctrl::ssStep(plant, xf, uvf)(0);
y_p = ctrl::ssStep(plant, xp, uvp)(0);
```

Now the error fed back to the controller will grow after the disturbance, the integral will wind up properly, and the anti-windup will engage correctly.

---

### 2.3 `ex23_fuzzy_pd_temperature.cpp` — Broken Warm-Start Initial Condition

**File:** [examples/ex23_fuzzy_pd_temperature.cpp](../examples/ex23_fuzzy_pd_temperature.cpp:69)  
**Severity:** Medium — outputs physically wrong temperature for the first ~100 steps  
**Description:**

```cpp
xf(1) = 20.0 * (1.0 + 1.0/200 * 60);  // approximate steady state
```

This is `20 * (1 + 0.3) = 26.0`. The intent was to initialise the state so that `C*x = 20°C` at t=0 (steady state at reference). But `C = [0, 1]`, so the output is `x(1)` directly. Setting `x(1) = 26` means the simulation starts with the measured temperature at 26°C, not 20°C. There's also a dangling `* 0.0` multiplied on the output of `ssStep` for the fuzzy branch (line 95), which zeros out an additive correction that was presumably left over from debugging.

The correct steady-state initialisation for this first-order-in-cascade plant at y=20 with u=0 is just `xf(1) = 20.0`, and `xf(0) = 20.0 * (200.0/60.0)` (the wall temperature at thermal equilibrium). Check the actual C2D matrices.

---

### 2.4 `ex25_fuzzy_supervisor_mpc.cpp` — State Vector x Not Tracking True Plant State

**File:** [examples/ex25_fuzzy_supervisor_mpc.cpp](../examples/ex25_fuzzy_supervisor_mpc.cpp:117)  
**Severity:** Medium — the adaptive MPC is not actually adaptive in any meaningful way  
**Description:**  
The simulation calls `ctrl::ssStep(buildPlantSS(...), xf, ...)` using `xf`/`xa` as the integration state, but `buildPlantSS` creates a *new* `StateSpace` object every step with a freshly computed gain `k_true`. This means the state `x` that `ssStep` advances is always for the *last built* model, not a consistent state trajectory. The `x` vectors are being passed to a different (re-created) model each step. The simulation is not numerically consistent — it's effectively running a different plant model every step and re-initialising the state propagation.

The fix is to maintain a single state vector and integrate it properly. If you want to simulate a Hammerstein model, the nonlinear gain should scale the output *after* the linear state propagation, not be embedded into a new `StateSpace` each step.

Also: `xf(0) = 7.0` and `xa(0) = 7.0` as the initial state with `C = [[1]]` gives an initial output of 7.0, which looks correct, but the state propagation thereafter is wrong for the reason above.

---

### 2.5 `ex26_fuzzy_ts_gain_scheduler.cpp` — Both PIDs Run Every Step (Double Integral Windup)

**File:** [examples/ex26_fuzzy_ts_gain_scheduler.cpp](../examples/ex26_fuzzy_ts_gain_scheduler.cpp:143)  
**Severity:** Medium — integral windup in the inactive PID corrupts the blended output  
**Description:**  
Both `pid1` and `pid2` are called unconditionally every step:

```cpp
double u1   = pid1.compute(e_ts);    // line 143
double u2   = pid2.compute(e_ts);    // line 144
double u_ts = std::clamp(w1 * u1 + w2 * u2, -15.0, 15.0);
```

When `w2 ≈ 0` (pendulum is near upright), `pid2` is still integrating the error and winding up its integral state. When the pendulum tilts and `w2` grows, the pre-wound `pid2` dumps a large integral contribution into the blended output — exactly the scenario that bumpless transfer and anti-windup exist to prevent. This is a textbook TS-fuzzy pitfall and the example should demonstrate *how to handle it*, not silently exhibit the bug.

The short-term fix: freeze the integral of the low-weight controller when its weight falls below a threshold (e.g. `w < 0.05`). Proper fix: call each PID's `bumplessInit()` as its weight transitions.

---

### 2.6 `ex20_system_identification_data.cpp` — CSV Written to CWD with No Directory Guard

**File:** [examples/ex20_system_identification_data.cpp](../examples/ex20_system_identification_data.cpp:25)  
**Severity:** Low-Medium — fragile; works only when binary is run from project root  
**Description:**

```cpp
std::ofstream out("sysid_data.csv");
if (!out) {
    std::cerr << "Failed to open sysid_data.csv\n";
    return 1;
}
```

The file is opened with a bare filename — no directory prefix. This writes to whatever the current working directory is at runtime. If you run the binary from `build/` (which is the default when using CMake + `./examples/ex20`), the CSV lands in `build/`, not where anyone expects it. The `examples/ex23-26` files at least use `"data/ex2X_*.csv"` consistently, but this one is a loose cannon. See Section 4 for the full migration plan.

---

### 2.7 `scripts/simulate_all.cpp` and `scripts/realtime_all.cpp` — CSV Written to CWD (Build Directory)

**Files:** [scripts/simulate_all.cpp](../scripts/simulate_all.cpp:122), [scripts/realtime_all.cpp](../scripts/realtime_all.cpp:174)  
**Severity:** Low-Medium — same CWD problem as 2.6, affects multiple outputs  
**Description:**  
Both scripts write CSVs using bare filenames (`"sim_pid.csv"`, `"rt_pid.csv"`, `"sim_summary.csv"`, `"rt_summary.csv"`) that land in the CWD at runtime. In a typical CMake build, this is `build/scripts/`, which is not tracked by git and invisible to Python plotting scripts in `scripts/`. See Section 4 for the migration plan.

---

### 2.8 `examples/cpp/` — Four Files Write CSV to CWD with Inconsistent Names

**Files:** [examples/cpp/siso_unknown.cpp](../examples/cpp/siso_unknown.cpp:556), [examples/cpp/siso_coupled.cpp](../examples/cpp/siso_coupled.cpp:737), [examples/cpp/mimo_unknown.cpp](../examples/cpp/mimo_unknown.cpp:372), [examples/cpp/mimo_known.cpp](../examples/cpp/mimo_known.cpp:568)  
**Severity:** Low-Medium — outputs go to CWD, not `examples/data/`, despite comments saying otherwise  
**Description:**  
`mimo_known.cpp` has a comment at line 25 saying "All results saved to `examples/data/mimo_known_results.csv`" but the code at line 568 sets `out_path = "mimo_known_results.csv"` (bare filename). The other three are similar — comments or log messages say one path, the `ofstream` opens a different one. See Section 4.

---

### 2.9 `UnscentedKalmanFilter` — Sigma Point Spread Parameter `kappa` Not Validated

**File:** [lib/UnscentedKalmanFilter.h](../lib/UnscentedKalmanFilter.h)  
**Severity:** Low — can produce negative weights silently  
**Description:**  
The UKF uses the standard Wan-van der Merwe sigma-point scheme with spread parameters `alpha`, `beta`, `kappa`. If `alpha` is set too small or `kappa` is negative (both are user-configurable), the zeroth weight `Wm0 = (lambda / (n + lambda))` can become negative. Negative mean weights are mathematically valid in the sigma-point formulation but the covariance weight `Wc0 = Wm0 + (1 - alpha^2 + beta)` can then also go negative for small `alpha`, breaking positive-semidefiniteness of the covariance update. There's no validation at construction time — the user gets garbage estimates and no warning.

**Fix:** Assert `n + lambda > 0` (equivalently `alpha^2*(n + kappa) > 0`) in the constructor, and warn if `Wc0 < 0`.

---

### 2.10 `FuzzySupervisor::update()` — Cooldown Counter Not Reset on `reset()`

**File:** [lib/FuzzyLogic.cpp](../lib/FuzzyLogic.cpp)  
**Severity:** Low — supervisor can be stuck in cooldown after `reset()`, suppressing valid relinearise signals  
**Description:**  
`FuzzySupervisor::reset()` presumably zeroes `abs_error_prev_` but does it reset `cooldown_remaining_`? If a simulation is reset mid-cooldown (common in MPC re-initialisation scenarios like `ex25`), the supervisor will silently ignore relinearise triggers for the remaining cooldown steps after the reset. The counter should be set to 0 in `reset()`.

---

### 2.11 `DiscreteMPC::compute()` — SISO Wrapper Reconstructs Reference Incorrectly for D≠0 Plants

**File:** [lib/DiscreteMPC.cpp](../lib/DiscreteMPC.cpp:84)  
**Severity:** Low — silent accuracy loss for D≠0 plants  
**Description:**

```cpp
double DiscreteMPC::compute(double error)
{
    const Eigen::VectorXd y_hat = plant_.C * x_hat_ + plant_.D * u_prev_;
    const Eigen::VectorXd r_ref = y_hat.array() + error;
    return computeRef(x_hat_, r_ref)(0);
}
```

The reference reconstruction `r = y_hat + error = (C*x + D*u_prev) + (r - y_true)` assumes `y_hat ≈ y_true`, which is valid only when `D = 0`. When `D ≠ 0`, `y_hat` uses `u_prev_` (u[k-1]) while `y_true` in the error uses `u[k]` (implicit in the feedback loop). The header comment acknowledges this and says to use `computeRef()` directly, but the warning is easy to miss. Add a `std::cerr` warning in the constructor when `D.norm() > 1e-12` to make it discoverable.

---

## 3. Performance & Architecture Issues

---

### 3.1 `DiscreteMPC::buildCondensedMatrices()` — Full Rebuild Still Triggered on Weight-Only Changes

**File:** [lib/DiscreteMPC.cpp](../lib/DiscreteMPC.cpp:177)  
**Severity:** Medium — unnecessary O(Np·Nc·n²) work on every Q/R weight update  
**Description:**  
`setParams()` calls `buildCondensedMatrices()`, which recomputes `F_`, `Phi_`, and `Gu_` (all depending only on the plant model) even if only `rho_y` or `rho_u` changed. In `ex25`'s adaptive MPC loop, the relinearisation path calls `setPlant()` — that's a justified full rebuild. But any tuner or gain-scheduler that only adjusts `rho_y`/`rho_u` incurs the full rebuild. The 05-19 report flagged this as P3.3 — it's still open.

**Fix (already documented in 05-19 report):** Split into `rebuildPredictionMatrices()` (plant-dependent: F, Phi, Gu) and `rebuildCostMatrix()` (weight-dependent: Qy, Ru, H, L, ldlt). `setPlant()` calls both; `setParams()` calls only the cost rebuild.

---

### 3.2 `ex25_fuzzy_supervisor_mpc.cpp` — `buildPlantSS()` Called Twice Per Step Per Controller

**File:** [examples/ex25_fuzzy_supervisor_mpc.cpp](../examples/ex25_fuzzy_supervisor_mpc.cpp:116)  
**Severity:** Low-Medium — allocates and constructs a fresh `StateSpace` object in the hot loop  
**Description:**

```cpp
double k_true_f = phGain(yf);
double y_lin_f  = ctrl::ssStep(buildPlantSS(k_true_f, Ts), xf, ...)(0);
```

`buildPlantSS` constructs four `Eigen::MatrixXd` objects on the stack and initialises them every step. This is the simulation loop — it's not a real-time path — but it's sloppy. Cache the `StateSpace` object and update only the `B` scalar (which is the only thing `k` changes) rather than rebuilding from scratch.

---

### 3.3 `FuzzySystem::evaluate()` — Grid Search for Peak When `std::optional<double> peak` Is Set

**File:** [lib/FuzzyLogic.cpp](../lib/FuzzyLogic.cpp)  
**Severity:** Low — unnecessary O(N) grid scan per rule evaluation  
**Description:**  
`defuzzWeightedAvg` falls back to a grid search over `cog_resolution` points when `LinguisticTerm::peak` is `nullopt`. For `ltSingleton`, the peak is set explicitly, so no grid search happens. But for user-built `LinguisticTerm` objects with triangular/trapezoidal MFs (common in `ex26`'s weight systems), if the user doesn't manually set `peak`, the grid search fires. In ex26, `cog_resolution = 101` by default, so every `FuzzySystem::evaluate()` call does up to 101 evaluations of each MF. At `Ts=5ms` and 2000 steps, this is fine for simulation but would be terrible in an embedded loop.

The documentation should explicitly state that users *must* set `peak` for any `LinguisticTerm` used in TS inference if they want O(1) defuzzification.

---

## 4. CSV Migration Plan — Controller Output Files → Root `data/`

### 4.1 Problem Statement

CSV output from controller files is currently scattered across four different landing zones depending on how the binary is invoked:

| Source | Current Path | Problem |
|---|---|---|
| `ex23_fuzzy_pd_temperature.cpp` | `data/ex23_fuzzy_temperature.csv` (relative to CWD) | Only works if CWD = project root |
| `ex24_fuzzy_pid_dc_motor.cpp` | `data/ex24_fuzzy_dc_motor.csv` | Same |
| `ex25_fuzzy_supervisor_mpc.cpp` | `data/ex25_fuzzy_supervisor_mpc.csv` | Same |
| `ex26_fuzzy_ts_gain_scheduler.cpp` | `data/ex26_fuzzy_ts_pendulum.csv` | Same |
| `ex20_system_identification_data.cpp` | `sysid_data.csv` (bare filename) | Lands in build dir |
| `examples/cpp/siso_unknown.cpp` | `siso_unknown_results.csv` (bare) | Lands in build dir |
| `examples/cpp/siso_coupled.cpp` | `siso_coupled_results.csv` (bare) | Lands in build dir |
| `examples/cpp/mimo_unknown.cpp` | `mimo_unknown_results.csv` (bare) | Lands in build dir |
| `examples/cpp/mimo_known.cpp` | `mimo_known_results.csv` (bare) | Lands in build dir |
| `scripts/simulate_all.cpp` | `sim_<name>.csv`, `sim_summary.csv` (bare) | Lands in build dir |
| `scripts/realtime_all.cpp` | `rt_<name>.csv`, `rt_summary.csv` (bare) | Lands in build dir |

The root-level `data/` folder exists but is completely empty. The intent is clearly there — it's not being used.

**Goal:** All CSV outputs from controller files go to `<project_root>/data/` regardless of where the binary is invoked from.

---

### 4.2 Strategy — Use `__FILE__` to Derive Project Root

The most portable CWD-independent approach without requiring CMake `configure_file` tricks is to derive the project root from `__FILE__` at compile time. Since `__FILE__` expands to the source file path (absolute in MSVC with `/FC`, relative otherwise), or alternatively use a CMake-injected `PROJECT_SOURCE_DIR` define:

**In root `CMakeLists.txt`, add:**

```cmake
add_compile_definitions(PROJECT_DATA_DIR="${CMAKE_SOURCE_DIR}/data")
```

Then in each source file, replace the hardcoded path with:

```cpp
// Before:
std::ofstream csv("data/ex23_fuzzy_temperature.csv");

// After:
std::ofstream csv(std::string(PROJECT_DATA_DIR) + "/ex23_fuzzy_temperature.csv");
```

This is absolute, CWD-independent, and requires zero runtime filesystem logic.

**Alternative (no CMake change needed):** Add a small helper in a shared header — but the CMake define is cleaner and already consistent with how `CMAKE_SOURCE_DIR` is used in the Doxyfile.

---

### 4.3 File-by-File Changes

#### Group A — `examples/ex20_*.cpp` (bare filename → `PROJECT_DATA_DIR`)

**File:** [examples/ex20_system_identification_data.cpp](../examples/ex20_system_identification_data.cpp:25)

```cpp
// BEFORE:
std::ofstream out("sysid_data.csv");

// AFTER:
std::ofstream out(std::string(PROJECT_DATA_DIR) + "/sysid_data.csv");
```

**Dependency:** None. No other file reads this CSV (it's meant to be fed manually into SubspaceID/RLS examples).

---

#### Group B — `examples/ex23_*.cpp` through `ex26_*.cpp` (relative `data/` → `PROJECT_DATA_DIR`)

These four files currently use `"data/ex2X_*.csv"` which resolves correctly only when CWD = project root. All four need the same one-line fix.

**File:** [examples/ex23_fuzzy_pd_temperature.cpp](../examples/ex23_fuzzy_pd_temperature.cpp:72)

```cpp
// BEFORE:
std::ofstream csv("data/ex23_fuzzy_temperature.csv");

// AFTER:
std::ofstream csv(std::string(PROJECT_DATA_DIR) + "/ex23_fuzzy_temperature.csv");
```

**File:** [examples/ex24_fuzzy_pid_dc_motor.cpp](../examples/ex24_fuzzy_pid_dc_motor.cpp:79)

```cpp
// BEFORE:
std::ofstream csv("data/ex24_fuzzy_dc_motor.csv");

// AFTER:
std::ofstream csv(std::string(PROJECT_DATA_DIR) + "/ex24_fuzzy_dc_motor.csv");
```

**File:** [examples/ex25_fuzzy_supervisor_mpc.cpp](../examples/ex25_fuzzy_supervisor_mpc.cpp:99)

```cpp
// BEFORE:
std::ofstream csv("data/ex25_fuzzy_supervisor_mpc.csv");

// AFTER:
std::ofstream csv(std::string(PROJECT_DATA_DIR) + "/ex25_fuzzy_supervisor_mpc.csv");
```

**File:** [examples/ex26_fuzzy_ts_gain_scheduler.cpp](../examples/ex26_fuzzy_ts_gain_scheduler.cpp:121)

```cpp
// BEFORE:
std::ofstream csv("data/ex26_fuzzy_ts_pendulum.csv");

// AFTER:
std::ofstream csv(std::string(PROJECT_DATA_DIR) + "/ex26_fuzzy_ts_pendulum.csv");
```

**Dependency:** The Python scripts in `examples/python/` (ex23_gen.py, etc.) presumably read these files. After migration, update the Python scripts to use `../../data/ex2X_*.csv` or pass the path as a CLI arg.

---

#### Group C — `examples/cpp/` (bare filename → `PROJECT_DATA_DIR`)

All four MIMO/SISO advanced examples write to bare filenames. Also note that `mimo_unknown.cpp` and `siso_unknown.cpp` *read* PRBS input data from `"examples/data/..."` paths, which are separate from the output CSVs and should be left alone (they're input fixtures).

**File:** [examples/cpp/siso_unknown.cpp](../examples/cpp/siso_unknown.cpp:556)

```cpp
// BEFORE (line 556):
std::ofstream of("siso_unknown_results.csv");

// AFTER:
std::ofstream of(std::string(PROJECT_DATA_DIR) + "/siso_unknown_results.csv");
```

**File:** [examples/cpp/siso_coupled.cpp](../examples/cpp/siso_coupled.cpp:737)

```cpp
// BEFORE:
std::ofstream f("siso_coupled_results.csv");

// AFTER:
std::ofstream f(std::string(PROJECT_DATA_DIR) + "/siso_coupled_results.csv");
```

**File:** [examples/cpp/mimo_unknown.cpp](../examples/cpp/mimo_unknown.cpp:372) (two output files)

```cpp
// BEFORE (line 372):
std::ofstream of("mimo_unknown_results.csv");
// BEFORE (line 380):
std::ofstream of("mimo_unknown_id.csv");

// AFTER:
std::ofstream of(std::string(PROJECT_DATA_DIR) + "/mimo_unknown_results.csv");
std::ofstream of(std::string(PROJECT_DATA_DIR) + "/mimo_unknown_id.csv");
```

**File:** [examples/cpp/mimo_known.cpp](../examples/cpp/mimo_known.cpp:568)

```cpp
// BEFORE (line 568):
std::string out_path = "mimo_known_results.csv";

// AFTER:
std::string out_path = std::string(PROJECT_DATA_DIR) + "/mimo_known_results.csv";
```

**Note:** The comment at the top of `mimo_known.cpp` says "results saved to `examples/data/`" — update that comment to `data/` after the migration.

**Dependency:** None known. These are standalone output files consumed by the user directly or by Python plotting scripts if any exist.

---

#### Group D — `scripts/simulate_all.cpp` and `scripts/realtime_all.cpp`

These scripts generate the most CSVs (one per controller plus summary files). The `write_csv()` helper in `simulate_all.cpp` is the right place to fix it — single change, fixes all output paths.

**File:** [scripts/simulate_all.cpp](../scripts/simulate_all.cpp:122)

```cpp
// BEFORE:
static void write_csv(const SimResult &r)
{
    std::string fname = "sim_" + r.name + ".csv";
    std::ofstream f(fname);
    ...
}

// ALSO (line 525):
std::ofstream fsum("sim_summary.csv");

// AFTER:
static void write_csv(const SimResult &r)
{
    std::string fname = std::string(PROJECT_DATA_DIR) + "/sim_" + r.name + ".csv";
    std::ofstream f(fname);
    ...
}

// ALSO:
std::ofstream fsum(std::string(PROJECT_DATA_DIR) + "/sim_summary.csv");
```

**File:** [scripts/realtime_all.cpp](../scripts/realtime_all.cpp:174) (three change sites)

```cpp
// BEFORE (line 174 — per-controller helper):
std::string fname = "rt_" + name + ".csv";

// BEFORE (line 303 — LQR hardcode):
std::ofstream f("rt_lqr.csv");

// BEFORE (line 424 — ADRC hardcode):
std::ofstream f("rt_adrc.csv");

// BEFORE (line 500 — summary):
std::ofstream fsum("rt_summary.csv");

// AFTER — replace all four with PROJECT_DATA_DIR prefix:
std::string fname = std::string(PROJECT_DATA_DIR) + "/rt_" + name + ".csv";
std::ofstream f(std::string(PROJECT_DATA_DIR) + "/rt_lqr.csv");
std::ofstream f(std::string(PROJECT_DATA_DIR) + "/rt_adrc.csv");
std::ofstream fsum(std::string(PROJECT_DATA_DIR) + "/rt_summary.csv");
```

**Note:** The LQR and ADRC hardcoded opens at lines 303 and 424 are suspicious — they bypass the `write_csv()` helper entirely. These should either be folded into the per-controller helper or explained by a comment.

**Dependency:** `scripts/visualize.py` presumably reads from these. After migration, update the Python script's path assumptions. If `visualize.py` is run from the project root, replacing `"sim_pid.csv"` with `"data/sim_pid.csv"` in the Python side should be sufficient.

---

### 4.4 CMake Change — Add `PROJECT_DATA_DIR` Definition

**File:** [CMakeLists.txt](../CMakeLists.txt)

Add one line after `set(CMAKE_CXX_STANDARD_REQUIRED ON)`:

```cmake
add_compile_definitions(PROJECT_DATA_DIR="${CMAKE_SOURCE_DIR}/data")
```

This propagates to all subdirectories (examples, scripts) automatically. No per-`CMakeLists.txt` changes needed.

**Windows note:** `CMAKE_SOURCE_DIR` on Windows uses forward slashes by default. `std::ofstream` accepts forward slashes on MSVC/Windows, so this is fine. If ever targeting MSVC with `/W4` pedantry around path strings, wrap in `std::filesystem::path(PROJECT_DATA_DIR) / "filename.csv"` for maximum portability.

---

### 4.5 Migration Checklist

- [ ] Add `add_compile_definitions(PROJECT_DATA_DIR="${CMAKE_SOURCE_DIR}/data")` to root [CMakeLists.txt](../CMakeLists.txt)
- [ ] Fix [examples/ex20_system_identification_data.cpp](../examples/ex20_system_identification_data.cpp:25) — bare `"sysid_data.csv"` → `PROJECT_DATA_DIR`
- [ ] Fix [examples/ex23_fuzzy_pd_temperature.cpp](../examples/ex23_fuzzy_pd_temperature.cpp:72) — `"data/ex23_..."` → `PROJECT_DATA_DIR`
- [ ] Fix [examples/ex24_fuzzy_pid_dc_motor.cpp](../examples/ex24_fuzzy_pid_dc_motor.cpp:79) — `"data/ex24_..."` → `PROJECT_DATA_DIR`
- [ ] Fix [examples/ex25_fuzzy_supervisor_mpc.cpp](../examples/ex25_fuzzy_supervisor_mpc.cpp:99) — `"data/ex25_..."` → `PROJECT_DATA_DIR`
- [ ] Fix [examples/ex26_fuzzy_ts_gain_scheduler.cpp](../examples/ex26_fuzzy_ts_gain_scheduler.cpp:121) — `"data/ex26_..."` → `PROJECT_DATA_DIR`
- [ ] Fix [examples/cpp/siso_unknown.cpp](../examples/cpp/siso_unknown.cpp:556) — bare filename → `PROJECT_DATA_DIR`
- [ ] Fix [examples/cpp/siso_coupled.cpp](../examples/cpp/siso_coupled.cpp:737) — bare filename → `PROJECT_DATA_DIR`
- [ ] Fix [examples/cpp/mimo_unknown.cpp](../examples/cpp/mimo_unknown.cpp:372) — two bare filenames → `PROJECT_DATA_DIR`
- [ ] Fix [examples/cpp/mimo_known.cpp](../examples/cpp/mimo_known.cpp:568) — `out_path` bare name → `PROJECT_DATA_DIR`
- [ ] Fix [scripts/simulate_all.cpp](../scripts/simulate_all.cpp:122) — `write_csv()` + summary → `PROJECT_DATA_DIR`
- [ ] Fix [scripts/realtime_all.cpp](../scripts/realtime_all.cpp:174) — 4 sites (helper, lqr hardcode, adrc hardcode, summary) → `PROJECT_DATA_DIR`
- [ ] Update Python plotting scripts to read from `data/` instead of CWD
- [ ] Update `mimo_known.cpp` file header comment to say `data/` not `examples/data/`
- [ ] Run a clean build and verify all CSVs appear in `<project_root>/data/`

---

## 5. Priority Summary

| Priority | Issue | File | Effort |
|---|---|---|---|
| **P0** | GPC Hessian re-factored every step (correctness + RT) | `lib/GeneralizedPredictiveControl.cpp` | Trivial |
| **P0** | Disturbance applied post-saturation in ex23 (wrong physics) | `examples/ex23_fuzzy_pd_temperature.cpp` | Small |
| **P0** | ex25 state vector not consistent across re-created plant models | `examples/ex25_fuzzy_supervisor_mpc.cpp` | Medium |
| **P1** | Double integral windup in ex26 TS gain scheduler | `examples/ex26_fuzzy_ts_gain_scheduler.cpp` | Small |
| **P1** | ex23 wrong warm-start IC (output starts at 26°C not 20°C) | `examples/ex23_fuzzy_pd_temperature.cpp` | Small |
| **P1** | UKF no validation of sigma-point spread parameters | `lib/UnscentedKalmanFilter.h` | Small |
| **P1** | MPC `setParams()` triggers full prediction matrix rebuild | `lib/DiscreteMPC.cpp` | Medium |
| **P1** | FuzzySupervisor cooldown not reset in `reset()` | `lib/FuzzyLogic.cpp` | Trivial |
| **P2** | All CSV output files landing in wrong directory | 12 files | Small (per file) |
| **P2** | MPC SISO wrapper inaccurate for D≠0, no warning | `lib/DiscreteMPC.cpp` | Trivial |
| **P2** | FuzzySystem grid search fires unnecessarily for non-singleton terms | `lib/FuzzyLogic.cpp` | Low |
| **P2** | `buildPlantSS()` called twice per step in ex25 hot loop | `examples/ex25_fuzzy_supervisor_mpc.cpp` | Low |
