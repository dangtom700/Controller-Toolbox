# Controller Toolbox - Code Review Report

**Date:** 2026-05-19  
**Reviewer:** Engineering Team  
**Scope:** Full codebase audit - `lib/`, `examples/`, `tests/`, `scripts/`

---

## Summary

The Controller Toolbox has matured into a comprehensive discrete-time control library comprising 11 controller implementations (PID, LQR, MPC, LQG, ADRC, SMC, Lead-Lag, Smith Predictor, Kalman Filter, Extremum Seeker, Controller Stack), a frequency- and time-domain analysis layer (`SystemAnalysis`, `MetricsAnalyzer`), a unified tuner suite (`TunerSuite`, `ControllerTuner`), and 22 example programs. All five control-theory gaps identified in the original bug report have been addressed at the implementation level.

However, the code review exposes **11 stability and correctness defects** in the newly implemented code, **5 performance bottlenecks**, and **3 open architectural issues** carried over from the original report. The top three priorities are: (1) adding NaN/Inf input guards and LDLT solver health checks across the critical compute paths to prevent silent corruption or crashes; (2) eliminating dynamic memory allocation inside `DiscreteMPC::buildCondensedMatrices()` and `SmithPredictor`'s delay buffer to meet real-time constraints; and (3) implementing the `ISensor`/`IActuator` hardware abstraction layer required for deployment on physical hardware.

---

## 1. Resolved Items

The following items from the original report are now implemented. Where the implementation has a known defect, the relevant entry in **Section 2** is cross-referenced.

| # | Original Item | Implementing Component | Status |
|---|---|---|---|
| 1 | Stability Margin Calculators (GM, PM) | `SystemAnalysis::calculateMargins()` | Implemented - see bugs 2.1, 2.2 |
| 2 | Automated Time-Domain Metric Extraction | `MetricsAnalyzer` class | Implemented - see bug 2.9 |
| 3 | Pole/Zero Extraction and Stability Check | `SystemAnalysis::getPoles()`, `isDiscreteStable()` | Implemented |
| 4 | Robust Sensitivity Analysis (Hinf norm) | `SystemAnalysis::calculateHInfinityNorm()` | Implemented (grid approximation) |
| 5 | Discrete Lyapunov Equation Solver | `SystemAnalysis::solveDiscreteLyapunov()` | Implemented |

> **Note on Hinf approximation:** `calculateHInfinityNorm()` evaluates the peak singular value over a fixed 10 000-point frequency grid. This is an approximation, not a rigorous upper bound. For formal robustness certification, the result should be treated as a lower bound only.

---

## 2. Bug Fixes & Stability Improvements

The following defects were identified in the current implementation. Each entry includes the affected file, a description of the failure mode, and a recommended fix.

---

### 2.1 `SystemAnalysis::calculateMargins()` - `log(0)` produces `-inf`

**File:** `lib/SystemAnalysis.cpp`, approximately line 135  
**Severity:** High - corrupts gain margin computation silently  
**Description:** The expression `std::log10(mag)` is evaluated unconditionally. When `mag` approaches zero (e.g., at a system resonance or numeric underflow), the result is `-inf`, which propagates into the margin calculation and returns a falsely infinite gain margin.  
**Recommended Fix:** Add a finite-check guard before the logarithm:
```cpp
if (mag < 1e-300) continue;
double gain_dB = 20.0 * std::log10(mag);
```

---

### 2.2 `SystemAnalysis::calculateMargins()` - Brittle Single-Crossover Heuristic

**File:** `lib/SystemAnalysis.cpp`  
**Severity:** Medium - returns incorrect margins on resonant or non-minimum-phase plants  
**Description:** The current implementation finds the first frequency where the gain crosses 0 dB (for phase margin) or the phase crosses -180^\circ (for gain margin), using a single threshold comparison. Systems with multiple gain crossovers - common in lightly damped plants, non-minimum-phase systems, or higher-order loops - will report the first crossover instead of the worst-case (smallest) margin, giving an overly optimistic stability assessment.  
**Recommended Fix:** Collect all sign-change pairs across the frequency grid and report the minimum margin found across all crossovers. Flag systems with more than one crossover in the output struct.

---

### 2.3 `DiscreteLQR::solveDARE()` - Unrecoverable Exception on Non-Convergence

**File:** `lib/DiscreteLQR.cpp`  
**Severity:** High - terminates the process in production  
**Description:** After 10 000 value-iteration steps, `solveDARE()` throws `std::runtime_error` with no recovery path. In an embedded control system, an unhandled exception at runtime is unacceptable. Additionally, the last iterate at the point of timeout may be a usable warm-start approximation, but it is discarded.  
**Recommended Fix:** Expose convergence state to the caller. Return a `struct DareResult { MatrixXd P; bool converged; int iterations; }` and let the caller decide whether to accept the approximate solution, fall back to a manual pole-placement gain, or log a warning.

---

### 2.4 `DiscreteLQR` - No Stabilizability Pre-Check

**File:** `lib/DiscreteLQR.cpp`  
**Severity:** Medium - wastes 10 000 iterations before failing on an unsolvable problem  
**Description:** If the pair `(A, B)` is not stabilizable, the DARE has no solution and the value-iteration cannot converge. There is no pre-flight check, so the solver exhausts its iteration budget before throwing. This makes debugging difficult because the error message does not distinguish "slow convergence" from "mathematically impossible."  
**Recommended Fix:** Implement a PBH (Popov-Belevitch-Hautus) test utility and assert stabilizability at the entry point of `solveDARE()`. This takes O(n^3) and eliminates the misleading timeout.

---

### 2.5 `DiscreteMPC::buildCondensedMatrices()` - Dynamic Allocation in Compute Path

**File:** `lib/DiscreteMPC.cpp`  
**Severity:** High - violates real-time determinism  
**Description:** `F_.resize()` and `Phi_.resize()` are called inside `buildCondensedMatrices()`, which is invoked during the control update. Eigen's `resize()` on `MatrixXd` calls `malloc` if the matrix needs to grow. In a hard real-time control loop, heap allocation introduces unbounded latency and is incompatible with MISRA-C++, AUTOSAR, and most RTOS deployment requirements.  
**Recommended Fix:** Pre-allocate `F_`, `Phi_`, and `H_` at maximum horizon size during the `DiscreteMPC` constructor. Inside the compute path, use `block()` views rather than resizing.

---

### 2.6 `DiscreteMPC` - Silent LDLT Solver Failure

**File:** `lib/DiscreteMPC.cpp`  
**Severity:** High - outputs corrupted control action without any diagnostic  
**Description:** `H_.ldlt().solve(rhs)` produces numerically garbage results if `H` is rank-deficient (e.g., when the prediction horizon is 1 with zero terminal cost weighting). Eigen's LDLT does not throw; it silently returns an invalid solution. The corrupted control signal is then passed to the plant actuator.  
**Recommended Fix:** Capture the decomposition object and verify `ldlt.info() == Eigen::Success` before using the result. On failure, revert to the previous control action and log an error.
```cpp
auto ldlt = H_.ldlt();
if (ldlt.info() != Eigen::Success) { /* fallback */ return u_prev_; }
u_opt = ldlt.solve(rhs);
```

---

### 2.7 `KalmanFilter::update()` - Unguarded Innovation Covariance Inversion

**File:** `lib/KalmanFilter.cpp`  
**Severity:** High - silent state estimate corruption  
**Description:** The innovation covariance `S = H*P*H' + R` is inverted via `S.ldlt().solve()` without checking for conditioning. When the measurement noise `R` is set very small (aggressive tuning), `S` becomes nearly singular and the Kalman gain `K` diverges, immediately corrupting all state estimates and covariance propagation.  
**Recommended Fix:** Apply the same LDLT health check as in Section 2.6. Additionally, enforce a minimum diagonal floor on `R` (`R.diagonal().array() = R.diagonal().array().max(1e-12)`) to prevent near-singular innovation matrices.

---

### 2.8 `DiscretePID::compute()` - NaN/Inf Propagation

**File:** `lib/DiscretePID.cpp`  
**Severity:** Medium - corrupts integral and derivative state permanently  
**Description:** `compute(error)` performs no input validation. A single `NaN` measurement (sensor dropout, ADC fault) corrupts the integral accumulator and derivative state, and the controller output becomes `NaN` for all subsequent steps even after the sensor recovers. The anti-windup back-calculation also produces `NaN`, preventing recovery.  
**Recommended Fix:** Add a finite-check at the top of `compute()`:
```cpp
if (!std::isfinite(error)) return u_prev_;
```
This applies equally to all other controllers that accept a scalar error input (`DiscreteLeadLag`, `DiscreteSMC`, `DiscreteADRC`).

---

### 2.9 `MetricsAnalyzer` - Incorrect Settling Time When Signal Never Settles

**File:** `lib/MetricsAnalyzer.cpp`, approximately line 44  
**Severity:** Low - misleading metric in unstable or underdamped closed-loop simulations  
**Description:** The settling time search loop records the last sample index at which the signal is within the 2% band. If the signal never enters that band (e.g., a test with an unstable controller), the function returns the final sample time as the settling time rather than indicating that settling was not achieved. Downstream analysis (tuner optimizers, test assertions) may interpret this as a valid but very slow response.  
**Recommended Fix:** Initialise `settle_idx = -1` before the loop. After the loop, if `settle_idx == -1`, return `std::numeric_limits<double>::quiet_NaN()` to signal non-convergence.

---

### 2.10 `DiscreteADRC` - Forward-Euler ESO Has No Stability Guard

**File:** `lib/DiscreteADRC.cpp`  
**Severity:** Medium - ESO can diverge at construction time parameters without warning  
**Description:** The Extended State Observer (ESO) is discretized using Forward Euler. The stability of Forward-Euler integration requires that the observer bandwidth satisfy `omega_o < 2/Ts`. No assertion or warning is issued if the user passes a bandwidth that violates this condition, causing the ESO poles to leave the unit circle and the observer states to diverge.  
**Recommended Fix:** Add a construction-time assertion:
```cpp
assert(omega_o * sample_time_ < 2.0 &&
       "ADRC: ESO bandwidth violates Forward-Euler stability. "
       "Reduce omega_o or use a smaller sample time.");
```

---

### 2.11 `SmithPredictor` - Dynamic `std::deque` Allocation in Real-Time Path

**File:** `lib/SmithPredictor.cpp`  
**Severity:** High - heap allocation on every control step  
**Description:** The dead-time compensation delay buffer is implemented as `std::deque<double>`. Each `push_back` / `pop_front` pair may invoke the allocator if the deque's internal chunked storage grows. In a real-time loop executing at 1 kHz or higher, this is a determinism violation.  
**Recommended Fix:** Replace the deque with a fixed-size circular buffer pre-allocated in the constructor to `delay_steps + 1` elements:
```cpp
// Constructor
delay_buf_.resize(delay_steps_ + 1, 0.0);
buf_head_ = 0;

// compute()
delay_buf_[buf_head_] = inner_output;
buf_head_ = (buf_head_ + 1) % delay_buf_.size();
double delayed = delay_buf_[buf_head_];
```

---

## 3. Performance Enhancements

The following areas have measurable performance deficiencies that should be addressed before the toolbox is used for rapid design iteration or deployed in time-sensitive workflows.

---

### 3.1 `SystemAnalysis::calculateMargins()` - O(10 000 * n^3) Frequency Sweep

**File:** `lib/SystemAnalysis.cpp`  
**Impact:** Approximately 10 000 eigenvalue evaluations per `calculateMargins()` call. For a 4-state system, each evaluation is O(64) FLOPs; for an 8-state system, O(512). Total: several million FLOPs per call - acceptable offline, but prohibitive in an adaptive gain-scheduling loop that calls it at each parameter update.  
**Recommendation:** Replace the uniform grid search with a bisection algorithm. Starting from coarse bracket points that bound the crossover frequencies, bisection converges in O(log₂(10 000) approx = 13) evaluations, reducing the cost by ~700*. The result should also be cached and invalidated only when the plant model changes.

---

### 3.2 `KalmanFilter::update()` - Per-Step O(n^3) Matrix Inversion

**File:** `lib/KalmanFilter.cpp`  
**Impact:** Full covariance update `P^+ = (I - KH)P` involves an n*n matrix multiply each step. For n >= 8, this becomes the dominant cost in the control loop.  
**Recommendation:** For the common case of small state vectors (n <= 4), switch `MatrixXd` to `Matrix<double, n, n>` fixed-size types. Eigen automatically applies fully unrolled, inlined inversions for small fixed sizes, eliminating function-call overhead and enabling SIMD auto-vectorization.

---

### 3.3 `DiscreteMPC` - Full Condensed Matrix Rebuild on Any Parameter Change

**File:** `lib/DiscreteMPC.cpp`  
**Impact:** `buildCondensedMatrices()` recomputes the prediction matrices `F` and `Φ` (which depend on the plant model) and the Hessian `H` (which depends on Q and R weights) in a single monolithic rebuild. Adaptive MPC that updates only Q/R weights at runtime triggers a full rebuild unnecessarily.  
**Recommendation:** Decompose the rebuild into two independent methods: `rebuildPredictionMatrices()` (called when the state-space model changes) and `rebuildCostMatrix()` (called when Q or R changes). This halves the work for the common case of weight adaptation.

---

### 3.4 Example Programs - Blocking CSV I/O Inside Control Loop

**Files:** All `examples/ex*.cpp`  
**Impact:** Every example writes step-response data via `std::ofstream` inside the simulation step. File system calls can block for milliseconds (kernel page cache misses, disk scheduling). On real hardware this destroys real-time performance; even in simulation it introduces non-determinism that obscures timing analysis.  
**Recommendation:** Implement a `LockFreeRingBuffer<LogEntry, 4096>` in `lib/logging/`. The real-time thread pushes a `LogEntry` struct in O(1) with no blocking. A dedicated low-priority background thread drains the ring buffer to disk. All 22 examples should be updated to use this pattern.

---

### 3.5 `ControllerTuner` Relay Experiment - Blocking Simulation Loop

**File:** `lib/ControllerTuner.cpp`  
**Impact:** The relay auto-tuner runs a closed-loop simulation to extract the ultimate gain and period. The entire experiment runs synchronously in a tight loop with no yield points. On real hardware, this means the relay experiment cannot coexist with a live control task and prevents RTOS integration.  
**Recommendation:** Refactor the relay tuner to expose a single-step `tick(double measurement) -> double` interface that advances the experiment by one sample. The RTOS task or test harness calls `tick()` at the sample rate and queries `isComplete()` to detect when identification is done.

---

## 4. Recommended Next Features

Features are ranked by production impact. **P0** items block deployment on physical hardware; **P1** items are high-value algorithmic completions; **P2** items improve developer experience and long-term maintainability.

---

### P0 - Safety & Real-Time Readiness

**`InputGuard` Decorator**  
A lightweight decorator class that wraps any `IController` and enforces NaN/Inf checking, output saturation, and a configurable fallback strategy (hold last output, switch to manual, or zero output). This provides a single enforcement point rather than requiring each controller to duplicate input validation code.

**`AtomicParamBuffer<Params>` - Lock-Free Parameter Updates**  
Updating controller parameters from a background thread (telemetry, auto-tuner, gain scheduler) while the real-time thread is inside `compute()` causes data races on every controller. Implement a double-buffered parameter container using `std::atomic` pointer swap: the background thread writes to the inactive buffer and atomically promotes it; the real-time thread reads from whichever buffer is current without acquiring a mutex.

**`ISensor` / `IActuator` Hardware Abstraction Layer (HAL)**  
The toolbox currently has no interface between controllers and physical I/O. Define two pure-virtual interfaces in `lib/hal/`:
- `ISensor { virtual double read() = 0; }` - abstracts ADC, encoder, CAN signal  
- `IActuator { virtual void write(double u) = 0; }` - abstracts DAC, PWM, EtherCAT drive  

Provide a `SimSensor` / `SimActuator` pair backed by `PlantModel` for testing, and document the contract for writing hardware adapters.

---

### P1 - Algorithm Completeness

**Fractional Delay in `SmithPredictor`**  
The current implementation supports only integer dead-times (`delay_steps` = integer multiple of `Ts`). Many real processes have non-integer delays. Add a Padé approximation option: for a fractional delay `L = k.Ts + delta`, the fractional part `delta` is approximated by a first- or second-order Padé filter, and `k` integer steps are handled by the circular buffer.

**DARE Warm-Start and Convergence Fallback in `DiscreteLQR`**  
After fixing bug 2.3, expose the last iterate `P` as a warm-start hint for subsequent `solveDARE()` calls (e.g., during adaptive weight updates). When the plant changes slowly, the previous solution is near the new one, and warm-starting can reduce iteration count by 10-100*.

**Multiple-Crossover Margin Reporting in `SystemAnalysis`**  
After fixing bug 2.2, extend `MarginResult` to include vectors of all crossover frequencies and per-crossover margins. This is essential for correctly assessing stability of higher-order or non-minimum-phase closed loops.

**`ExtremumSeeker` Convergence Detector**  
The extremum seeker has no convergence criterion. Add a stagnation detector: if the gradient estimate magnitude stays below a threshold `epsilon` for `N_stag` consecutive windows, set a `converged()` flag and optionally stop dithering to reduce steady-state perturbation of the plant.

---

### P2 - Developer Experience

**Catch2 / GoogleTest Migration**  
The current test suite uses custom `test::` macros defined locally. Migrating to Catch2 or GoogleTest provides BDD-style test naming, better assertion messages, death-test support (needed to verify that `solveDARE()` throws correctly), and CI integration. The migration is straightforward since the existing test structure is already logically organized.

**Benchmark Suite**  
Add a `benchmarks/` directory with one benchmark per controller measuring per-step `compute()` latency at state dimensions n \in {2, 4, 8, 16}. This provides regression detection and documents the real-time budget for each controller type. Google Benchmark is a natural fit alongside GoogleTest.

**Doxygen HTML API Reference**  
All public headers contain well-written doc comments, but no HTML reference is generated. Add a `Doxyfile` and a CI step that builds the API docs on every merge to `main` and publishes them to GitHub Pages. This eliminates the need to read header files to discover method signatures.

**Deployment Checklist**  
Add a `DEPLOYMENT.md` at the project root documenting: (1) parameter bound constraints for each controller (e.g., ADRC `omega_o < 2/Ts`, MPC horizon feasibility conditions); (2) RTOS integration guidance (zero-allocation checklist, stack size estimates); and (3) a troubleshooting section covering the most common failure modes (DARE non-convergence, MPC infeasibility, Kalman divergence).

---

## 3. Math Corrections Applied (2026-05-22)

The following mathematical errors and documentation gaps were identified and fully resolved. All five items were verified against the implementation.

---

### 3.1 `DiscreteMPC` — Missing G_u·u_prev Term in Condensed Prediction *(Resolved)*

**Files:** `lib/DiscreteMPC.cpp`, `lib/DiscreteMPC.h`  
**Severity:** High — systematically incorrect control action  
**Description:** The condensed prediction used `Y = F·x + Φ·ΔU`, omitting the `G_u·u_prev` offset. For an incremental MPC formulation (ΔU decision variables), the output prediction must include the cumulative step-response matrix `G_u` applied to the previous input to correctly account for the u_prev baseline. Without it, the gradient is computed from the wrong prediction error, producing a biased optimal move.  
**Fix:** Added `Gu_` matrix (`Np·p × m`) computed as the cumulative step response `G_u(i) = Σ_{j=0}^{i} C·A^j·B`. The prediction error in `computeRef()` is now:
```cpp
pred_err_.noalias() = F_ * x + Gu_ * u_prev_ - R_stack_;
```

---

### 3.2 `DiscretePID` — Forward-Euler Integral Despite Backward-Euler Claim *(Resolved)*

**Files:** `lib/DiscretePID.cpp`, `lib/DiscretePID.h`  
**Severity:** Medium — implementation inconsistent with documented integration method  
**Description:** The header declared backward Euler (`I[k] = I[k-1] + Ki·Ts·e[k]`), but the implementation computed the integral increment after forming `u_unsat`, making the current error effective only one step later (forward Euler behavior).  
**Fix:** Integral increment `ki_update = Ki·Ts·e[k]` is computed first and included in `u_unsat` before saturation, matching the declared law:
```
I[k] = I[k-1] + Ki·Ts·e[k] + Kb·(u_sat[k] − u_unsat[k])
u[k] = Kp·e[k] + I[k] + D[k]
```

---

### 3.3 `DiscreteLQG` — Kalman Innovation Uses Stale u When D ≠ 0 *(Resolved)*

**Files:** `lib/DiscreteLQG.cpp`, `lib/DiscreteLQG.h`  
**Severity:** Medium — silent accuracy degradation for non-zero D plants  
**Description:** The Kalman innovation `y − C·x̂ − D·u` requires `u[k]`, but at update time only `u[k-1]` is available. When D ≠ 0, this one-step staleness degrades state-estimate accuracy proportionally to D's magnitude. Previously the code used zeros for the D·u term without any diagnostic.  
**Fix:** Added a D≠0 guard in the constructor that emits a `std::cerr` warning and continues (programs are not terminated):
```
[DiscreteLQG] WARNING: plant.D != 0. Kalman innovation uses u[k-1] for the D*u term (one step stale). For accurate filtering, set D = 0 in the model.
```

---

### 3.4 `SmithPredictor` — Internal Model Output Uses D·zeros Instead of D·u_prev *(Resolved)*

**Files:** `lib/SmithPredictor.cpp`, `lib/SmithPredictor.h`  
**Severity:** Low — incorrect feedthrough term in Smith correction  
**Description:** The delay-free model output `ŷ[k] = C·x̂ + D·u` was evaluated with `D·zeros` instead of `D·u_prev`. For plants with a non-zero D matrix, the Smith correction term `ŷ_now − ŷ_delayed` was wrong, degrading dead-time compensation accuracy.  
**Fix:** Added `u_prev_` member (zero-initialized in constructor, updated each step after computing u). `y_now` is now:
```cpp
const double y_now = (model_.C * x_model_ + model_.D * u_prev_)(0);
```

---

### 3.5 `DiscreteSMC` — `c_de` Ts Absorption Not Documented *(Resolved)*

**File:** `lib/DiscreteSMC.h`  
**Severity:** Low — parameter mis-sizing risk for users working from continuous-time specs  
**Description:** The `c_de` parameter absorbs `Ts` (i.e., `c_de = λ·Ts` where `λ` is the desired continuous-time surface slope in 1/s). This was not documented, so users specifying `c_de` directly from a continuous-time design would over-weight the rate term by a factor of `1/Ts`.  
**Fix:** Updated the `c_de` field comment with explicit Ts absorption note and conversion formula:
```cpp
// Note: c_de absorbs the sample time Ts. To match a continuous-time
// slope lambda [1/s], set c_de = lambda * Ts.
double c_de = 0.1;
```

---

## Action Items

| Priority | Item | Affected Component | Estimated Effort |
|---|---|---|---|
| P0 | Add NaN/Inf guard to all `compute()` entry points | `DiscretePID`, `DiscreteLeadLag`, `DiscreteSMC`, `DiscreteADRC` | Small |
| P0 | Add LDLT solver health check in MPC and Kalman | `DiscreteMPC`, `KalmanFilter` | Small |
| P0 | Guard `log(0)` in `calculateMargins()` | `lib/SystemAnalysis.cpp` | Trivial |
| P0 | Pre-allocate condensed matrices in MPC constructor | `lib/DiscreteMPC.cpp` | Medium |
| P0 | Replace `SmithPredictor` deque with fixed circular buffer | `lib/SmithPredictor.cpp` | Small |
| P1 | Return `DareResult` struct with convergence flag from `solveDARE()` | `lib/DiscreteLQR.cpp` | Small |
| P1 | Return `NaN` sentinel from `MetricsAnalyzer` when signal never settles | `lib/MetricsAnalyzer.cpp` | Trivial |
| P1 | Replace uniform grid with bisection crossover search in `calculateMargins()` | `lib/SystemAnalysis.cpp` | Medium |
| P1 | Add stabilizability PBH pre-check in `solveDARE()` | `lib/DiscreteLQR.cpp` | Small |
| P1 | Add ESO stability guard assertion in `DiscreteADRC` constructor | `lib/DiscreteADRC.cpp` | Trivial |
| P1 | Implement `LockFreeRingBuffer` logging and update all examples | `lib/logging/`, `examples/` | Medium |
| P1 | Implement all-crossover margin reporting in `SystemAnalysis` | `lib/SystemAnalysis.cpp` | Medium |
| P2 | Implement `ISensor` / `IActuator` HAL with `SimSensor` / `SimActuator` | `lib/hal/` (new) | Large |
| P2 | Implement `AtomicParamBuffer` for lock-free parameter updates | `lib/` controllers | Medium |
| P2 | Migrate tests to Catch2 or GoogleTest with edge-case coverage | `tests/` | Medium |
| P2 | Add benchmark suite with per-controller latency profiles | `benchmarks/` (new) | Medium |
| P2 | Generate Doxygen HTML and publish via CI | Repository CI config | Small |
| P2 | Write `DEPLOYMENT.md` with RTOS checklist and troubleshooting guide | Project root | Small |
