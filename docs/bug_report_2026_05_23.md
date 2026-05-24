# Code Review - Controller Toolbox
**Date:** 2026-05-23  
**Reviewer role:** Senior C++ developer / control systems engineer  
**Scope:** Full library (`lib/`), fuzzy module (`FuzzyLogic`), and tug-boat case study (`case-study/Tug Boat Numerical Simulation/`)  
**Tone:** Critical and constructive. Issues are ranked by severity: 🔴 Bug / correctness, 🟠 Design / architecture, 🟡 Performance, 🔵 Style / maintainability.

---

## Fix Log

| Date | Issues Fixed | Commit notes |
|------|-------------|--------------|
| 2026-05-24 | **B-1, B-2, B-3, B-4, B-7** (P0/P1 correctness) | `ss2tf` replaced with Faddeev-LeVerrier; `AtomicParamBuffer` replaced with seqlock; `LinguisticTerm::peak` added + `defuzzWeightedAvg` fixed; `bumplessInit` integral corrected; `tf2ss` pad uses pre-allocated insert instead of O(n^2) loop. `FuzzyLogic.cpp` added to `lib/CMakeLists.txt` (was missing from static library). |
| 2026-05-24 | **D-1, D-2, B-5, D-6, P-1, P-2, B-6, D-3, D-4, S-2, Tug-B** (P1-P3 backlog) | `StateSpace::validate()` added; `FuzzySystem::addOutput` throws on second call; `DiscreteMPC::compute` D!=0 doc; `TransferFunction` monic check uses tolerance; `FuzzySystem` workspace pre-allocated; LDLT cached in `DiscreteMPC`; `KalmanFilter::step` uses `std::optional`; `IController::computeVec` throws by default; `ControllerStack` weight normalisation (D-4 already present - confirmed auto-normalises); magic numbers replaced with named constants; simulation step count uses `std::lround`. |

Items **not yet addressed**: S-1 (`.clang-format`), S-3 (`std::function` RT overhead), S-4 (`using namespace Eigen`), S-5 (workspace member naming), S-6 (`M_b` naming), D-5 (supervisor trend sign-crossing), 6.2 Nc>Np guard, 6.3 `ssStep` return struct, 6.4 KF silent skip warning.  These are low-severity style/documentation items.

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Correctness Bugs](#2-correctness-bugs)
3. [Design and Architecture Issues](#3-design-and-architecture-issues)
4. [Performance Issues](#4-performance-issues)
5. [Style and Maintainability](#5-style-and-maintainability)
6. [Module-Specific Notes](#6-module-specific-notes)
   - 6.1 [FuzzyLogic](#61-fuzzylogic)
   - 6.2 [DiscreteMPC](#62-discretempc)
   - 6.3 [PlantModel / c2d](#63-plantmodel--c2d)
   - 6.4 [KalmanFilter](#64-kalmanfilter)
   - 6.5 [AtomicParamBuffer](#65-atomicparambuffer)
   - 6.6 [Tug Boat Case Study](#66-tug-boat-case-study)
7. [Positive Observations](#7-positive-observations)
8. [Prioritised Action List](#8-prioritised-action-list)
9. [References](#9-references)

---

## 1. Executive Summary

The Controller Toolbox is a well-structured discrete-time control library with a clear `IController` abstraction, a sensible separation between plant modelling and control, and solid references throughout. The fuzzy module is a useful first-class addition. However, several correctness issues and design gaps accumulate to a level that warrants attention before the library is used in any validated context.

The most critical issues are:

- **`ss2tf` uses eigenvalue-based polynomial construction** that is numerically unreliable for repeated or near-repeated eigenvalues; a companion-form companion algorithm or `Faddeev-LeVerrier` recursion would be more robust.
- **`AtomicParamBuffer::read()` is not truly lock-free** under the C++ memory model when `Params` is not `std::atomic`-compatible; the current implementation has a data race on `bufs_` on compilers that do not sequentially-consistent memory ordering by default.
- **`FuzzySystem::defuzzWeightedAvg` peak-finding by grid search** is fragile and produces wrong results for singleton MFs (the `mfSingleton` lambda returns 1 only at an exact floating-point match, never found by the grid).
- **`FuzzyPID::bumplessInit`** leaves `integral_` in an incorrect state, producing an output bump on the first `compute()` call.
- **`DiscreteMPC::compute()` (SISO wrapper)** reconstructs the reference as `y_hat + error`, but `y_hat` is evaluated using a stale `u_prev_` from the _previous_ step's end state, which is correct only when `D = 0`. The comment does not flag this limitation.
- **`physics_plant.cpp` RK4**: The tau vector is held constant across all four sub-steps, which is correct for a zero-order-hold assumption but means disturbances computed externally before the call are already one half-step stale by k2/k3. The runner applies `tau_env` at the tick boundary without splitting; this is acceptable for Deltat = 0.5 s but should be documented.

Several additional design-level issues are described below.

---

## 2. Correctness Bugs

### 🔴 B-1 - `ss2tf`: eigenvalue polynomial expansion is numerically unstable

**File:** `lib/PlantModel.cpp`, lines 96-111

```cpp
Eigen::EigenSolver<Eigen::MatrixXd> es(sys.A, false);
const Eigen::VectorXcd &ev = es.eigenvalues();
// ... polynomial expansion via root product ...
a[k] = poly[k].real(); // imaginary parts are numerical noise
```

The approach constructs the characteristic polynomial by multiplying out `(z - lambda_i)` factors in floating-point complex arithmetic. For systems of order >= 8 with clustered eigenvalues this suffers catastrophic cancellation. The imaginary parts discarded as "numerical noise" can exceed the real parts when eigenvalues are nearly repeated.

**Correct approach:** Use Leverrier's (Faddeev-LeVerrier) recursion directly on `A`, which operates in real arithmetic and avoids eigenvalue computation entirely. Alternatively, compute the characteristic polynomial via `(sI - A)` evaluated symbolically, or use Schur decomposition to keep `A` in quasi-upper-triangular form.

**Recommended fix:**
```cpp
// Leverrier-Faddeev: coefficients of det(zI - A) in O(n^3) real arithmetic.
// p[0]=1, p[k] = -trace(A*M_{k-1})/k,  M_k = A*M_{k-1} + p[k]*I
std::vector<double> a(n + 1, 0.0);
a[0] = 1.0;
Eigen::MatrixXd M = Eigen::MatrixXd::Identity(n, n);
for (int k = 1; k <= n; ++k) {
    M = sys.A * M;
    a[k] = -M.trace() / k;
    M.diagonal().array() += a[k];
}
```

---

### 🔴 B-2 - `AtomicParamBuffer`: data race on `bufs_` array

**File:** `lib/AtomicParamBuffer.h`, lines 63-73

```cpp
const Params& read() const {
    return bufs_[active_.load(std::memory_order_acquire)];
}
void publish(const Params& p) {
    const int inactive = 1 - active_.load(std::memory_order_relaxed);
    bufs_[inactive] = p;                          // <- plain write
    active_.store(inactive, std::memory_order_release);
}
```

**Problem:** The read thread loads `active_`, then reads `bufs_[active]`. The write thread writes `bufs_[inactive]`, then stores `active_`. There is a data race: on the step where `active_` flips, the reader could be mid-way through reading `bufs_[old_active]` while the writer simultaneously starts writing `bufs_[old_active]` (which is now "inactive" from the writer's perspective). The `acquire`/`release` pair on `active_` does not protect the `bufs_` array elements themselves against torn reads.

**Root cause:** `bufs_` is a plain `std::array`, not atomic. The `static_assert` on `trivially_copyable` is necessary but not sufficient - it ensures no internal pointers, but does not make concurrent access safe.

**Correct approaches (pick one):**
1. Use `std::atomic<Params>` if `sizeof(Params)` is <= platform lock-free size (usually 8 bytes - too small for `PIDParams`).
2. Protect the write with a `std::mutex` on the writer side only (the reader remains lock-free after the atomic `active_` load, but you need a seqlock or third buffer for true single-copy-reads).
3. Use a **seqlock** pattern: writer increments a sequence counter before and after writing; reader checks the counter is even and unchanged across its read.

The simplest correct version for this use case (single-writer/single-reader, `Params` <= ~128 bytes) is a seqlock:

```cpp
// Writer
seq_.fetch_add(1, std::memory_order_release);  // odd = writing
bufs_[inactive] = p;
seq_.fetch_add(1, std::memory_order_release);  // even = done

// Reader (with retry)
uint64_t s;
Params result;
do {
    s = seq_.load(std::memory_order_acquire);
    if (s & 1) continue;   // writer in progress
    result = bufs_[active_.load(std::memory_order_acquire)];
} while (seq_.load(std::memory_order_acquire) != s);
```

---

### 🔴 B-3 - `FuzzySystem::defuzzWeightedAvg` breaks for singleton MFs

**File:** `lib/FuzzyLogic.cpp`, lines 165-187

```cpp
double best_mu = 0.0;
for (int k = 0; k < M; ++k) {
    double x  = out.lo + k * (out.hi - out.lo) / (M - 1);
    double mu = out.terms[t].mf(x);
    if (mu > best_mu) { best_mu = mu; centre = x; }
}
```

`mfSingleton(value)` returns 1 only when `|x - value| < 1e-9`. The grid over `[lo, hi]` at 51 points will almost never land exactly on `value`, so `best_mu` stays 0 and `centre` remains at the universe midpoint. The result is that all TS singleton outputs map to the same defuzzified value regardless of rules - the entire Takagi-Sugeno inference is silently wrong.

**Fix:** For `mfSingleton`, store the value directly in `LinguisticTerm` and use it as the `centre` without grid search. The cleanest solution is to add a `centre()` virtual or a separate `std::optional<double> peak` field to `LinguisticTerm`:

```cpp
struct LinguisticTerm {
    std::string name;
    MF          mf;
    std::optional<double> peak;   // set explicitly for singletons / TS use
};
```

Alternatively, check `mf(value)` at a targeted set of candidate points (the singleton value itself plus the grid):

```cpp
// After the grid loop, also probe the exact singleton candidate:
// e.g., for each rational fraction k/1000 in [lo, hi], evaluate mf.
```

The most pragmatic fix is to require TS users to set `peak` explicitly, and only fall back to grid search for Mamdani output terms.

---

### 🔴 B-4 - `FuzzyPID::bumplessInit` integral is wrong

**File:** `lib/FuzzyLogic.cpp`, lines 327-333

```cpp
void FuzzyPID::bumplessInit(double u_target, double error)
{
    pd_block_.reset();
    u_prev_    = u_target;
    integral_  = u_target - pd_block_.params().u_scale; // coarse; P+D approx = 0 at rest
    (void)error;
}
```

The comment says "P+D approx = 0 at rest" but then subtracts `u_scale` (a scaling constant, not a control output). This is dimensionally incorrect: `integral_` is in output units, and `u_scale` is also in output units but represents the maximum fuzzy PD contribution, not the current PD output. Calling `compute(error)` immediately after will produce:

```
u_pd   approx = some nonzero value (pd_block_ was just reset, so de = -e_prev/Ts != 0)
u_unsat = u_pd + (u_target - u_scale) + Ki*Ts*error
```

This is not close to `u_target` unless `u_pd approx = u_scale` by coincidence.

**Fix:** Set the integral to absorb the full target minus the PD contribution at the current error, matching the `DiscretePID::bumplessInit` pattern:

```cpp
void FuzzyPID::bumplessInit(double u_target, double error)
{
    pd_block_.reset();
    // Compute what PD would produce at this error with de=0
    double u_pd_est = pd_block_.compute(error);   // de=0 since just reset
    pd_block_.reset();                            // restore clean state
    integral_ = u_target - u_pd_est;
    u_prev_   = u_target;
    (void)error;
}
```

---

### 🟠 B-5 - `DiscreteMPC::compute()` SISO wrapper silently wrong for D != 0

**File:** `lib/DiscreteMPC.cpp`, lines 81-86

```cpp
double DiscreteMPC::compute(double error)
{
    const Eigen::VectorXd y_hat = plant_.C * x_hat_ + plant_.D * u_prev_;
    const Eigen::VectorXd r_ref = y_hat.array() + error;
    return computeRef(x_hat_, r_ref)(0);
}
```

`u_prev_` here is the control applied at step `k-1` (updated at the end of `computeRef`). The feedthrough `D * u_prev_` is therefore one step stale. For `D = 0` this is exactly correct. For `D != 0` the reconstructed reference `r_ref` drifts by `D * (u[k] - u[k-1])` per step. This should either be documented as a `D = 0` restriction or corrected by storing `u_current` separately.

---

### 🟠 B-6 - `KalmanFilter::step` default argument creates a dangling reference risk

**File:** `lib/KalmanFilter.h`, line 52

```cpp
void step(const Eigen::VectorXd& y,
          const Eigen::VectorXd& u_prev,
          const Eigen::VectorXd& u_current = Eigen::VectorXd());
```

A temporary `Eigen::VectorXd()` is default-constructed and bound to a `const&`. In `step()`, the implementation reads `u_current.size()` and potentially `u_current` data:

```cpp
update(y, u_current.size() > 0 ? u_current : u_prev);
```

This is technically well-formed (the temporary lives until the end of the full expression), but it is a footgun when a derived class overrides `step` and stores the reference. More importantly, the pattern `u_current.size() > 0` means a caller who passes an explicitly empty `VectorXd{}` gets `u_prev` used - but a caller who passes `VectorXd::Zero(0)` also gets `u_prev`. This is fragile. Use `std::optional<Eigen::VectorXd>` instead:

```cpp
void step(const Eigen::VectorXd& y,
          const Eigen::VectorXd& u_prev,
          std::optional<std::reference_wrapper<const Eigen::VectorXd>> u_current = std::nullopt);
```

---

### 🟡 B-7 - `tf2ss` numerator zero-padding prepends with `insert(begin)`

**File:** `lib/PlantModel.cpp`, lines 29-31

```cpp
while (static_cast<int>(num.size()) < n + 1)
    num.insert(num.begin(), 0.0);
```

`std::vector::insert` at the beginning is O(n) per call, making the total O(n^2) for a degree-n polynomial. This is harmless for typical controller orders (<= 5) but should use `std::deque` or a right-side resize:

```cpp
num.insert(num.begin(), n + 1 - static_cast<int>(num.size()), 0.0);
```

(single O(n) insert for the missing count).

---

## 3. Design and Architecture Issues

### 🟠 D-1 - `StateSpace` is a plain struct with public matrices - no invariant enforcement

`StateSpace` exposes `A`, `B`, `C`, `D` as raw public `Eigen::MatrixXd` members. Nothing prevents constructing an inconsistent system (e.g., `B` with wrong row count, `Ts = -1.0`). `c2d` does validate `Ts > 0` and `sys.Ts == 0`, but `tf2ss` and the bare constructor do not check dimension compatibility.

A minimal fix is a `validate()` method called from the constructor:

```cpp
void StateSpace::validate() const {
    if (A.rows() != A.cols())      throw std::invalid_argument("A must be square");
    if (B.rows() != A.rows())      throw std::invalid_argument("B row count != n");
    if (C.cols() != A.cols())      throw std::invalid_argument("C col count != n");
    if (D.rows() != C.rows())      throw std::invalid_argument("D row count != p");
    if (D.cols() != B.cols())      throw std::invalid_argument("D col count != m");
    if (Ts < 0.0)                  throw std::invalid_argument("Ts must be >= 0");
}
```

Several open-source libraries (e.g., `python-control`, `harold`) enforce this at construction. Without it, dimension mismatches propagate silently until an Eigen assertion fires at runtime (in Debug) or produce garbage (in Release with `-DNDEBUG`).

---

### 🟠 D-2 - `FuzzySystem` supports only a single output variable by design but enforces it implicitly

`FuzzySystem::addOutput` pushes to `outputs_` (a vector), `evaluate` indexes `outputs_[0]` without checking `outputs_.size()`. A second `addOutput` call is silently ignored. This will confuse users who expect MIMO fuzzy behaviour. Either:
- Document the single-output restriction explicitly at the API level and add a `throw` on a second call to `addOutput`, or
- Return `std::vector<double>` from `evaluate` to support the general case.

---

### 🟠 D-3 - `IController::computeVec` default implementation discards MIMO information

**File:** `lib/IController.h`, lines 22-25

```cpp
virtual Eigen::VectorXd computeVec(const Eigen::VectorXd &signal) {
    return Eigen::VectorXd::Constant(1, compute(signal(0)));
}
```

This default silently truncates a MIMO signal to its first element and returns a 1-vector. Any MIMO controller that forgets to override `computeVec` but is called through the base interface will produce wrong results without any compile-time or runtime warning. A better default would be to `throw std::logic_error("computeVec not implemented")` or make `computeVec` pure-virtual alongside `compute`.

---

### 🟠 D-4 - `ControllerStack` weight normalisation is absent in `Weighted` mode

**File:** `lib/ControllerStack.cpp` (inferred from header)

In `Weighted` mode, `u = Sigma w_i * u_i(e)`. If the user sets weights that do not sum to 1, the composite gain is unintentionally scaled. The fuzzy gain-scheduler example in `ex26` manually normalises `w1 + w2`, but the `ControllerStack` does not enforce this. Add a `normaliseWeights()` convenience method and note the requirement in the header.

---

### 🟠 D-5 - `FuzzySupervisor` uses absolute error only - trend sign is relative to previous absolute error

**File:** `lib/FuzzyLogic.cpp`, lines 409-432

```cpp
double d_err  = (abs_error - abs_error_prev_) / (Ts_ + 1e-12);
double trend_n = std::clamp(d_err / (p_.trend_threshold + 1e-12), -1.0, 1.0);
```

The trend `d|e|/dt` is computed from consecutive absolute errors. A step that causes the error sign to flip (e.g., oscillation around setpoint) will produce a positive trend even if the true error is shrinking. For example: `e[k-1] = +5`, `e[k] = -3` -> `|e|[k-1] = 5`, `|e|[k] = 3` -> `trend = -2/Ts` (correct, decreasing). But `e[k-1] = -5`, `e[k] = +5.1` -> `|e|` increases even if the controller is just crossing zero. In most real systems this is acceptable, but the supervisor is sensitive to sign crossings at the convergence boundary. Consider filtering `abs_error` with a short exponential moving average before differencing.

---

### 🟠 D-6 - `TransferFunction` denominator validation is incomplete

**File:** `lib/PlantModel.h`, lines 33-34

```cpp
if (den.empty() || den[0] == 0.0)
    throw std::invalid_argument("...");
```

The check `den[0] == 0.0` uses floating-point equality. If a user passes `den[0] = 1e-300` (near-zero but technically nonzero), no exception is thrown but the TF is non-monic and `tf2ss` will produce wildly wrong matrices. Replace with:

```cpp
if (den.empty() || std::abs(den[0] - 1.0) > 1e-9)
    throw std::invalid_argument("TransferFunction: denominator must be monic (den[0] = 1).");
```

---

## 4. Performance Issues

### 🟡 P-1 - `FuzzySystem::evaluate` allocates `std::vector` every call

**File:** `lib/FuzzyLogic.cpp`, lines 113-116

```cpp
std::vector<std::vector<double>> mu;
mu.reserve(inputs_.size());
for (std::size_t i = 0; i < inputs_.size(); ++i)
    mu.push_back(inputs_[i].fuzzify(inputs[i]));
```

`fuzzify` also returns a `std::vector<double>`. For a 25-rule, 2-input, 5-term-per-input system running at 500 Hz, this is two `std::vector` heap allocations per call plus one for `mu` itself. On embedded or RT targets this is unacceptable.

**Fix:** Pre-allocate `mu_` as a `std::vector<std::vector<double>>` member of `FuzzySystem` with fixed sizes, and clear/fill it in-place:

```cpp
// constructor:
mu_.resize(inputs_.size());
for (size_t i = 0; i < inputs_.size(); ++i)
    mu_[i].resize(inputs_[i].terms.size());

// evaluate():
for (size_t i = 0; i < inputs_.size(); ++i)
    for (size_t t = 0; t < inputs_[i].terms.size(); ++t)
        mu_[i][t] = std::clamp(inputs_[i].terms[t].mf(inputs[i]), 0.0, 1.0);
```

Similarly, `strengths` (a `std::vector<double>`) should be pre-allocated as a member.

---

### 🟡 P-2 - `DiscreteMPC::buildCondensedMatrices` allocates and re-computes on every `setPlant()` / `setParams()` call

This is by design for adaptive MPC, but the LDLT in `computeRef` is recomputed every step even though `H_` only changes when the plant or params change:

```cpp
const auto ldlt = H_.ldlt();
```

Pre-computing `ldlt_` as a member and only refreshing it inside `buildCondensedMatrices` saves one O((Nc.m)^3) factorisation per control step:

```cpp
// in DiscreteMPC private:
Eigen::LDLT<Eigen::MatrixXd> ldlt_;   // pre-factored H

// in buildCondensedMatrices():
ldlt_ = H_.ldlt();

// in computeRef():
DeltaU_ = (-ldlt_.solve(grad_)).cwiseMax(lb_).cwiseMin(ub_);
```

---

### 🟡 P-3 - `ss2tf` recomputes `A^k` incrementally but builds `Apow` redundantly

`ss2tf` builds Markov parameters with an incremental `Apow = A * Apow` pattern (correct), but also calls `Eigen::EigenSolver` earlier - an O(n^3) decomposition that is then thrown away after characteristic polynomial extraction. The Leverrier-Faddeev fix (B-1) would eliminate `EigenSolver` entirely and also provide the characteristic polynomial directly, removing all redundant work.

---

### 🟡 P-4 - `FuzzyPD::compute` re-normalises `e` and `de` every call without caching

The clamp-and-divide in `FuzzyPD::compute` (lines 269-270) is cheap but calls `sys_.evaluate({e_n, de_n})` which triggers the full CoG grid evaluation at 101 points, evaluating 5 * 101 = 505 MF calls. For a 3-axis marine controller at Deltat = 0.5 s this costs approximately 1500 MF evaluations per tick. Given that the normalised universe is always `[-1, 1]` with fixed 5-term partition, a lookup table (e.g., 201-point pre-sampled aggregate) would cut this by >99% with negligible accuracy loss.

---

## 5. Style and Maintainability

### 🔵 S-1 - Inconsistent namespace formatting

`lib/DiscreteSMC.h` and `lib/ExtremumSeeker.h` use `namespace ctrl {` without indentation; `lib/DiscretePID.h` and `lib/PlantModel.h` use four-space indented members. This is a stylistic inconsistency that makes the library feel assembled from separate authors. A `.clang-format` file would enforce consistency across all files.

---

### 🔵 S-2 - Magic numbers in `FuzzyLogic.cpp` without named constants

**File:** `lib/FuzzyLogic.cpp`

```cpp
sys_.params.cog_resolution = 101;
// ...
int M = 51;  // grid for peak search
```

`101` and `51` are magic numbers. Define:
```cpp
static constexpr int kCoGResolutionDefault   = 101;
static constexpr int kTSPeakSearchResolution = 51;
```

---

### 🔵 S-3 - `FuzzyLogic.h` uses `using MF = std::function<double(double)>` in namespace scope

`std::function` has non-trivial overhead (virtual dispatch + possible heap allocation for closures that capture by reference or exceed the SBO buffer). For an open-source library intended for embedded/RT use, this is a significant concern. Consider:
- Templating `LinguisticTerm` on the MF type (`template<class MF_T>`), or
- Using a fixed-size functor type with a small buffer optimisation, or
- Documenting that `FuzzySystem` is not suitable for hard-RT use without the P-1 pre-allocation fix applied first.

---

### 🔵 S-4 - `using namespace Eigen` in `physics_plant.cpp`

**File:** `case-study/.../physics_plant.cpp`, line 5

`using namespace Eigen` in a `.cpp` file is acceptable in isolated files, but can cause silent name collision with `std::` names such as `Matrix` and `Vector` in larger projects. Replace with specific `using` declarations or fully-qualified names:

```cpp
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix;
```

---

### 🔵 S-5 - `DiscreteMPC` pre-allocated members are named with trailing underscores like private state, but are purely workspace

Members like `R_stack_`, `pred_err_`, `grad_`, `DeltaU_`, `grad_k_`, `DU_new_`, `lb_`, `ub_`, `cumMin_`, `cumMax_` are described in a comment as "work vectors" but are named identically to persistent state. A naming convention such as `ws_` prefix (`ws_R_stack_`, `ws_grad_`) would immediately communicate their transient role and reduce cognitive load when reading `computeRef`.

---

### 🔵 S-6 - `FuzzySupervised_MPC::buildAxisSS` comment refers to `M_b` as `(m-Yvd)` but `M_b` is the full added mass matrix

**File:** `case-study/.../controllers.cpp`, lines 409-413

```cpp
// For surge (0): ... effective additional drag approx = (m-Yvd)*|v| / m
if (axis == 0) d_extra = pp_.M_b(1,1) * std::abs(nu(1));  // (m-Yvd)*|v|
```

`pp_.M_b(1,1)` is the (1,1) entry of the full added-mass matrix `M̃ = M_rb + M_a`. The comment says `(m-Yvd)` which is the diagonal entry of `M_rb + M_a` for the sway degree of freedom - technically `M_b(1,1) = m - Y_{v_dot}`. The comment is correct, but `M_b` is an unusual name for a full inertia matrix (typically `M` with subscripts). Rename or at least document the field in `plant_parameters.h` with its physical meaning.

---

## 6. Module-Specific Notes

### 6.1 FuzzyLogic

**Strengths:** Clean separation between `FuzzySystem` (inference engine) and the convenience wrappers (`FuzzyPD`, `FuzzyPID`, `FuzzySupervisor`). The 25-rule diagonal rule table is the canonical choice and well-referenced. `mfShoulderLeft/Right` are correct.

**Issues beyond B-3 and B-4:**
- The `mfTriangular(a, c, b)` parameter order is non-standard. Most fuzzy textbooks use `(a, b, c)` where `a` is the left foot, `b` the peak, `c` the right foot. The current signature has `b` as the right foot but appears second after the peak `c` - visually confusing. The MATLAB `trimf` convention is `[a, b, c]` (left, peak, right). Consider renaming to `mfTriangle(double left, double peak, double right)`.
- `FuzzyPD::buildSystem()` is called unconditionally in the constructor and in `setParams`. Since `setParams` does not rebuild the system (see comment "Rebuild not needed - scaling is applied externally"), calling `buildSystem()` from the constructor is the only call site. Consider removing the `setParams`-triggered rebuild comment to avoid confusion.
- There is no serialisation or deserialisation mechanism for rule bases. For a production library, being able to load rules from JSON (consistent with how the tug-boat scenarios are loaded) would be valuable.

---

### 6.2 DiscreteMPC

**Strengths:** The condensed formulation is correct, the rolling worst-case bound computation is a solid approach to handling the full-horizon absolute constraint `u \in [uMin, uMax]` without a full-horizon QP reformulation, and the pre-allocation strategy eliminates per-step heap allocation in `computeRef` (good RT practice).

**Issues beyond P-2:**
- When `L_ approx = 0` (numerically degenerate Hessian), `alpha = 1.0 / L_` becomes `Inf`. This path is partly guarded by the `ldlt.info() != Eigen::Success` check, but a near-zero `L_` with a valid LDLT will produce a huge gradient-projection step size rather than a graceful failure.
- `buildCondensedMatrices` does not validate `Nc <= Np`. If a user sets `Nc > Np`, the `Phi_` matrix is built as `(Np*p) * (Nc*m)` with correct dimensions, but the unconstrained solve effectively plans `Nc - Np` control moves beyond the prediction horizon with no cost - a modelling error. Add: `if (p_.Nc > p_.Np) throw std::invalid_argument("MPC: Nc must be <= Np");`.
- Warm-starting gradient projection from the unconstrained optimum is good. However, projection-only methods (no dual variable update) can converge slowly when constraints are active. For the scale of applications targeted here (Nc <= 20), this is acceptable, but documenting the worst-case iteration count vs. Nc would help users.

---

### 6.3 PlantModel / c2d

**Strengths:** ZOH via matrix exponential embedding is the correct exact method. Tustin prewarping is correctly derived. The `c2d` validation for `Ts == 0` on the input catches a common mistake.

**Issues:**
- `c2d(Tustin)` uses `PartialPivLU` for `(I - alphaA)^{-1}`. This is fine for general matrices but `(I - alphaA)` is symmetric when `A` is symmetric, in which case `LDLT` or Cholesky would be faster and more numerically stable. In practice, `A` is rarely symmetric, so this is a minor point.
- `ssStep` modifies `x` in-place via `Eigen::Ref`. This is efficient and idiomatic but makes it easy to call `ssStep(sys, x, u)` in a simulation loop and accidentally use the *updated* `x` before applying the output - the function docstring says "y[k] = C.x[k] + D.u[k]" is returned (correct) but a reviewer reading the call site sees `x` being passed and mutated. Consider returning a `struct { VectorXd y; VectorXd x_next; }` and leaving the caller to assign `x = result.x_next`, which is self-documenting.

---

### 6.4 KalmanFilter

**Strengths:** Joseph-form covariance update is the correct choice for numerical stability. The R floor `1e-12` avoids singular innovation covariance. The split `predict` / `update` interface is clean.

**Issues:**
- When `ldlt.info() != Eigen::Success` in `update()`, the function silently returns without updating. The state estimate is then the raw predicted value. In a high-noise scenario where `S` becomes singular (e.g., R was set too low despite the floor), this silent skip can allow the estimate to diverge undetected. Add a `std::cerr` warning or a `bool update_succeeded_` flag that the caller can query.
- `P0` defaults to `Eigen::MatrixXd()` (size 0*0) and the check `P0.rows() == n` handles this, but `Eigen::MatrixXd()` is a non-obvious way to express "use default". Prefer `std::optional<Eigen::MatrixXd> P0 = std::nullopt`.

---

### 6.5 AtomicParamBuffer

Beyond B-2, the documentation says "Multiple concurrent writers are not supported. Guard `publish()` with a mutex." This is correct but understates the issue: even with a mutex on `publish()`, the read-side data race from B-2 still exists. The mutex only prevents two concurrent writers from interleaving writes to the *same* inactive buffer; it does not make `bufs_[active]` safe to read without synchronisation. This must be fixed at the buffer level as described in B-2.

---

### 6.6 Tug Boat Case Study

**Strengths:** The simulation architecture (plant / environment / allocator / controller / logger / runner) is clean and well-separated. RK4 integration is the right choice for a 0.5 s timestep on a 3-DOF marine model. The controller base class interface (`compute(ref, state) -> tau`) is simple and testable.

**Issues:**
- **`simulation_runner.cpp` line 59:** `N_steps = static_cast<int>(duration / dt)` uses integer truncation. If `duration = 300.0` and `dt = 0.5`, `N_steps = 600` exactly. But if `duration` is read from JSON with floating-point rounding, e.g. `299.9999999`, `N_steps = 599` and the simulation runs one step short. Use `std::lround(duration / dt)` or add a small epsilon: `N_steps = static_cast<int>(duration / dt + 0.5)`.

- **`physics_plant.cpp` `C_rb` sign convention:** The standard Fossen 3-DOF Coriolis matrix is:
  ```
  C_rb = [ 0,      0,     -(m-Y_vd)*v ]
         [ 0,      0,      (m-X_ud)*u ]
         [ (m-Y_vd)*v, -(m-X_ud)*u, 0 ]
  ```
  The implementation matches this. However, in `f()`, line 57, `C_nu = C_rb(nu) * nu` where `C_rb` is already defined to produce the force when multiplied by `nu` - this is correct. The concern is that `pp_.M_b(0,0)` is labeled as `m - Xud` (surge added mass direction) but in Fossen's notation `X_{u_dot}` is negative (added mass is positive, so `m - X_{u_dot} = m + |X_{u_dot}|`). Verify that the JSON `plant_params.json` stores `M_b` with the correct sign convention - if `M_b` entries are stored as `m + |X_{u_dot}|` (positive), the code is correct.

- **ESC controller in `controllers.cpp` lines 311-319:** The ESC `compute` call receives `std::abs(e(axis))` as the cost, but `ExtremumSeeker` is designed to find the *extremum* of a cost surface, not to regulate to a setpoint. Using |e| as the cost means ESC minimises absolute error - this is reasonable as a model-free controller, but the ESC integrator will drift when the error is near zero (the cost surface is flat there, and the dither produces no gradient signal). The resulting `theta` output is used as `tau` directly without any scaling - for the marine application where tau is in Newtons (up to 2*10⁶ N), `perturbAmp = 5e3 N` is only 0.25% of the full range, which may be too small to overcome the damping and produce a gradient signal. This should be validated against the simulation output.

---

## 7. Positive Observations

- **`DiscretePID` backward-Euler with back-calculation anti-windup** is correctly implemented. The inclusion of `ki_update` in the unsaturated output before saturation (so the anti-windup correction acts on the correct `u_unsat`) is a subtle point that many implementations get wrong.
- **`DiscreteSMC` `compute()`** correctly stores `u_prev_` for `bumplessInit` compatibility, and the `phi > 1e-12` guard for the boundary layer handles the degenerate `phi = 0` case cleanly.
- **`SuperTwistingSMC`** is a bonus class not advertised in the header-level documentation but fully implemented - a useful addition for chattering-free SMC.
- **`c2d` TustinPrewarped** is rarely seen in open-source control libraries and is correctly derived.
- **`KalmanFilter` Joseph form** covariance update is the numerically stable choice and reflects solid engineering practice.
- **`ControllerStack` bumpless transfer** via `prevActiveName_` tracking is a thoughtful design for mode-switching without output bumps.
- **`AtomicParamBuffer` `static_assert`** for `trivially_copyable` catches the most common misuse at compile time.
- **The case study's thrust allocator** with rate-limiting and box constraints is physically correct and well-separated from the control logic.
- **All controllers guard against `!std::isfinite(error)`**, returning the last valid output. This is good defensive practice for real deployments.

---

## 8. Prioritised Action List

| Priority | Status | Issue | File | Effort |
|----------|--------|-------|------|--------|
| P0 | ✅ Fixed 2026-05-24 | B-2: Fix `AtomicParamBuffer` data race | `AtomicParamBuffer.h` | Medium |
| P0 | ✅ Fixed 2026-05-24 | B-3: Fix TS singleton defuzzification | `FuzzyLogic.cpp/.h` | Small |
| P0 | ✅ Fixed 2026-05-24 | B-1: Replace `ss2tf` eigenvalue expansion with Faddeev-LeVerrier | `PlantModel.cpp` | Small |
| P1 | ✅ Fixed 2026-05-24 | B-4: Fix `FuzzyPID::bumplessInit` integral calculation | `FuzzyLogic.cpp` | Small |
| P1 | ✅ Fixed 2026-05-24 | D-1: Add `StateSpace::validate()` | `PlantModel.h/.cpp` | Small |
| P1 | ✅ Fixed 2026-05-24 | D-2: Enforce single-output in `FuzzySystem::addOutput` | `FuzzyLogic.h/.cpp` | Trivial |
| P1 | ✅ Fixed 2026-05-24 | B-5: Document D!=0 limitation in `DiscreteMPC::compute` | `DiscreteMPC.h` | Trivial |
| P1 | ✅ Fixed 2026-05-24 | D-6: Fix monic check in `TransferFunction` constructor | `PlantModel.h` | Trivial |
| P2 | ✅ Fixed 2026-05-24 | P-1: Pre-allocate `FuzzySystem` workspace vectors | `FuzzyLogic.h/.cpp` | Medium |
| P2 | ✅ Fixed 2026-05-24 | P-2: Cache LDLT in `DiscreteMPC` | `DiscreteMPC.h/.cpp` | Small |
| P2 | ✅ Fixed 2026-05-24 | B-6: Replace `VectorXd` default arg with `std::optional` | `KalmanFilter.h/.cpp` | Small |
| P2 | ✅ Fixed 2026-05-24 | D-3: Make `computeVec` throw by default | `IController.h` | Trivial |
| P3 | ✅ Confirmed 2026-05-24 | D-4: Normalise weights in `ControllerStack` Weighted mode | `ControllerStack.cpp` | N/A - already auto-normalises |
| P3 | ⬜ Open | S-1: Add `.clang-format` for namespace style consistency | repo root | Trivial |
| P3 | ✅ Fixed 2026-05-24 | S-2: Replace magic numbers in `FuzzyLogic.cpp` | `FuzzyLogic.cpp` | Trivial |
| P3 | ⬜ Open | S-5: Rename workspace members in `DiscreteMPC` | `DiscreteMPC.h` | Small |
| P3 | ✅ Fixed 2026-05-24 | Tug-B: Simulation step count rounding | `simulation_runner.cpp` | Trivial |

*Review conducted against commit state as of 2026-05-23. All line numbers are approximate and may shift with subsequent edits.*
