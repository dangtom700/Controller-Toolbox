# Resonant Controller, Notch Filter, and PLL Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three new `lib/` classes from `docs/algorithm_backlog.md`'s "Additional Controller Types" category — `ResonantController`, `NotchFilter`, `PhaseLockedLoop` — each fully wired (build, Python bindings, smoke test, example, Catch2 tests), per the approved design at `docs/superpowers/specs/2026-06-24-resonant-notch-pll-controllers-design.md`.

**Architecture:** `ResonantController` implements `IController` (a Tustin-discretized, prewarped biquad) and composes through the existing `ControllerStack(Additive)` mode. `NotchFilter` and `PhaseLockedLoop` are standalone classes (no shared base) — a fixed-design biquad filter and a single-input SOGI-PLL estimator, respectively. All three are scalar-only (no Eigen dependency in their own math).

**Tech Stack:** C++20, Eigen (transitively via `IController`/project headers, not used directly in these classes' own math), pybind11, Catch2 v3, CMake.

## Global Constraints

- NaN-guard hold-last contract: every `compute()`/`apply()`/`step()` must check `!std::isfinite(input)` as its first action and return/skip without mutating state (`[Ref: CONTRIBUTING.md#numerical-safety-rules, rule 7]`). Note: despite rule 7's text, there is no `ctrl::sanitize()` helper in `lib/IController.h` — the actual codebase convention (e.g. `lib/DiscreteLeadLag.cpp`) is an early `return` of the last-held value; follow that, not the rule's literal wording.
- Zero-allocation in the hot path: no `std::vector::push_back`, no STL streams, no `std::cout`/`cerr` inside `compute()`/`apply()`/`step()` (`[Ref: CLAUDE.md section 7]`).
- New `IController` subclasses must be bound in Python as `std::shared_ptr<T>` + `ctrl::IController` base (`[Ref: CLAUDE.md section 6, bindings/controllers_bindings.cpp:47-48]`), or `ControllerStack.add_controller()` throws at runtime.
- Self-registration: every new header ends with `CTRL_REGISTER_FEATURE(name)` (`[Ref: lib/ControllerRegistry.h]`); requires `#include "Features.h"`.
- Construction-time validation throws `std::invalid_argument` for non-physical parameters (`<= 0` frequencies/gains, target at/above Nyquist `1/(2*Ts)`).
- `compile.bat`/`compile.sh` example-target lists are hand-maintained — every new example must be added to both (`[Ref: CLAUDE.md section 2]`).
- All math in this plan has been numerically verified in a throwaway scratch build before being written here; the constants below (Kp/Ki, Kr, wc, Q, etc.) are not arbitrary — see each task's derivation note.

---

## Task 1: ResonantController core (class + library wiring + Catch2 tests)

**Files:**
- Create: `lib/ResonantController.h`
- Create: `lib/ResonantController.cpp`
- Modify: `lib/CMakeLists.txt:75` (append after `FreqDomainIdentifier.cpp` in `CTRL_CORE_SOURCES`)
- Modify: `lib/ControllerToolbox.h:133` (append include after `FreqDomainIdentifier.h`)
- Test: `tests/test_catch2_advanced.cpp` (append new `TEST_CASE`s at end of file, tag `[resonant_controller]`)

**Interfaces:**
- Consumes: `ctrl::IController` (`lib/IController.h`), `CTRL_REGISTER_FEATURE` (`lib/Features.h`)
- Produces: `struct ctrl::ResonantParams { double targetFreqHz; double dampingRadPerSec; double Kr; double uMin=-1e9; double uMax=1e9; }`; `class ctrl::ResonantController : public ctrl::IController` with constructor `ResonantController(const ResonantParams&, double Ts)`, `double compute(double error) override`, `void reset() override`, `double sampleTime() const override`, `SignConvention signConvention() const override`, `void setParams(const ResonantParams&)`, `const ResonantParams& params() const`.

- [ ] **Step 1: Write `lib/ResonantController.h`**

```cpp
#pragma once
#include "IController.h"
#include "Features.h"

/**
 * @file ResonantController.h
 * @brief Discrete-time non-ideal (finite-Q) resonant controller for single-harmonic rejection.
 *
 * **Continuous form** (avoids the phase singularity of the textbook infinite-gain resonant
 * term by using a finite damping bandwidth wc):
 * @code
 *   G_RC(s) = 2*Kr*wc*s / (s^2 + 2*wc*s + w0^2)      [w0 = 2*pi*targetFreqHz]
 * @endcode
 *
 * Discretised via Tustin with frequency prewarping at w0, so the digital resonance peak lands
 * exactly at targetFreqHz despite bilinear-transform frequency warping:
 * @code
 *   w0_warped = (2/Ts)*tan(w0*Ts/2)
 * @endcode
 *
 * By the exact bilinear-transform correspondence H_d(e^{j*theta}) = G(j*K*tan(theta/2)) (K=2/Ts),
 * the steady-state gain at theta = w0*Ts is exactly Kr (real, zero phase) - verified analytically
 * and numerically (see docs/superpowers/plans/2026-06-24-resonant-notch-pll-controllers.md).
 *
 * Composes with a base controller through ControllerStack(StackMode::Additive) - add one
 * ResonantController per target harmonic alongside the base controller; the stack sums outputs.
 *
 * @see Yepes, Freijedo, Lopez & Doval-Gandoy, "High-Performance Digital Resonant Controllers
 *      Implemented With Two Integrators", IEEE Trans. Power Electronics (2011).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for ResonantController.
 */
struct ResonantParams
{
    double targetFreqHz;     ///< f0 - the harmonic frequency to reject/track [Hz].
    double dampingRadPerSec; ///< wc - bandwidth/peak-width parameter [rad/s]. Smaller = narrower/higher peak.
    double Kr;               ///< Resonant gain - the exact steady-state gain at targetFreqHz.
    double uMin = -1e9;      ///< Output saturation lower limit.
    double uMax =  1e9;      ///< Output saturation upper limit.
};

/**
 * @brief Discrete-time single-harmonic resonant controller.
 *
 * Inherits from IController so it composes through ControllerStack(Additive) alongside a base
 * controller (PID, etc.), one instance per target harmonic.
 */
class ResonantController : public IController
{
public:
    /**
     * @brief Construct the resonant controller and precompute biquad coefficients.
     * @param params Resonant tuning parameters (target frequency, damping, gain, limits).
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument if targetFreqHz <= 0, dampingRadPerSec <= 0, or
     *         targetFreqHz is at/above the Nyquist frequency (1/(2*Ts)).
     */
    ResonantController(const ResonantParams &params, double Ts);

    /**
     * @brief Compute the resonant correction for the current tracking error.
     * @param error Tracking error e[k] = r[k] - y[k].
     * @return Resonant correction u[k], clamped to [uMin, uMax].
     */
    double compute(double error) override;

    /** @brief This controller's signal convention is e = r - y. */
    SignConvention signConvention() const override { return SignConvention::TrackingErrorRMinusY; }

    /** @brief Reset the biquad's internal state (previous inputs/outputs) to zero. */
    void reset() override;

    /** @brief Sample time Ts [s]. */
    double sampleTime() const override { return Ts_; }

    /** @brief Hot-update parameters and recompute biquad coefficients. */
    void setParams(const ResonantParams &p);

    /** @brief Read-only access to current parameters. */
    const ResonantParams &params() const { return p_; }

private:
    ResonantParams p_;
    double Ts_;
    double b0_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0; ///< Biquad coeffs (b1 is exactly 0).
    double e_prev1_ = 0.0, e_prev2_ = 0.0;             ///< e[k-1], e[k-2].
    double u_prev1_ = 0.0, u_prev2_ = 0.0;             ///< u[k-1], u[k-2].

    void computeCoeffs();
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(resonant_controller)
```

- [ ] **Step 2: Write `lib/ResonantController.cpp`**

```cpp
#include "ResonantController.h"
#include <cmath>
#include <stdexcept>

namespace ctrl
{

ResonantController::ResonantController(const ResonantParams &params, double Ts)
    : p_(params), Ts_(Ts)
{
    if (p_.targetFreqHz <= 0.0)
        throw std::invalid_argument("ResonantController: targetFreqHz must be positive");
    if (p_.dampingRadPerSec <= 0.0)
        throw std::invalid_argument("ResonantController: dampingRadPerSec must be positive");
    if (p_.targetFreqHz >= 1.0 / (2.0 * Ts_))
        throw std::invalid_argument("ResonantController: targetFreqHz must be below the Nyquist frequency");
    computeCoeffs();
    reset();
}

// Non-ideal resonant filter G_RC(s) = 2*Kr*wc*s / (s^2 + 2*wc*s + w0^2), discretised via Tustin
// with prewarping at w0 so the digital resonance peak lands exactly at targetFreqHz:
//
//   K          = 2/Ts
//   w0         = 2*pi*targetFreqHz
//   w0_warped  = K*tan(w0*Ts/2)                  (prewarp)
//   a0         = w0_warped^2
//   a1c        = 2*dampingRadPerSec
//   b1c        = 2*Kr*dampingRadPerSec
//   D0         = K^2 + a1c*K + a0
//
//   b0 =  b1c*K / D0
//   b2 = -b0           (numerator of the bilinear-transformed system is odd: b1c*K*(z^2-1))
//   a1 =  (2*a0 - 2*K^2) / D0
//   a2 =  (K^2 - a1c*K + a0) / D0
//
//   u[k] = b0*e[k] + b2*e[k-2] - a1*u[k-1] - a2*u[k-2]   (the b1 term is exactly zero)
void ResonantController::computeCoeffs()
{
    const double K  = 2.0 / Ts_;
    const double w0 = 2.0 * M_PI * p_.targetFreqHz;
    const double w0_warped = K * std::tan(w0 * Ts_ / 2.0);
    const double a0  = w0_warped * w0_warped;
    const double a1c = 2.0 * p_.dampingRadPerSec;
    const double b1c = 2.0 * p_.Kr * p_.dampingRadPerSec;
    const double D0  = K * K + a1c * K + a0;

    b0_ = (b1c * K) / D0;
    b2_ = -b0_;
    a1_ = (2.0 * a0 - 2.0 * K * K) / D0;
    a2_ = (K * K - a1c * K + a0) / D0;
}

double ResonantController::compute(double error)
{
    if (!std::isfinite(error))
        return u_prev1_;

    double u = b0_ * error + b2_ * e_prev2_ - a1_ * u_prev1_ - a2_ * u_prev2_;
    if (u > p_.uMax) u = p_.uMax;
    if (u < p_.uMin) u = p_.uMin;

    e_prev2_ = e_prev1_;
    e_prev1_ = error;
    u_prev2_ = u_prev1_;
    u_prev1_ = u;
    return u;
}

void ResonantController::reset()
{
    e_prev1_ = e_prev2_ = 0.0;
    u_prev1_ = u_prev2_ = 0.0;
}

void ResonantController::setParams(const ResonantParams &p)
{
    p_ = p;
    computeCoeffs();
}

} // namespace ctrl
```

- [ ] **Step 3: Wire into `lib/CMakeLists.txt`**

Change line 75 from:
```cmake
    FreqDomainIdentifier.cpp
)
```
to:
```cmake
    FreqDomainIdentifier.cpp
    ResonantController.cpp
)
```

- [ ] **Step 4: Wire into `lib/ControllerToolbox.h`**

Change line 133 from:
```cpp
#include "FreqDomainIdentifier.h"      ///< FreqDomainIdentifier - Levy's method frequency-domain system identification (Phase 4 Iteration 2).
```
to:
```cpp
#include "FreqDomainIdentifier.h"      ///< FreqDomainIdentifier - Levy's method frequency-domain system identification (Phase 4 Iteration 2).
#include "ResonantController.h"       ///< ResonantController - single-harmonic internal-model corrector; composes via ControllerStack(Additive).
```

- [ ] **Step 5: Append failing Catch2 tests to `tests/test_catch2_advanced.cpp`**

Append at the end of the file (after the last existing `TEST_CASE`):

```cpp
TEST_CASE("ResonantController steady-state gain at the target frequency equals Kr exactly",
          "[resonant_controller]")
{
    const double Ts = 1e-4;
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0;
    p.dampingRadPerSec = 5.0;
    p.Kr = 2.0;
    ctrl::ResonantController rc(p, Ts);

    const int N = 30000; // 3s = 150 cycles @ 50Hz, ample settling margin
    double maxAbsLastCycle = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double e = std::sin(2.0 * M_PI * p.targetFreqHz * k * Ts);
        const double u = rc.compute(e);
        if (k >= N - 200)
            maxAbsLastCycle = std::max(maxAbsLastCycle, std::fabs(u));
    }
    REQUIRE_THAT(maxAbsLastCycle, WithinRel(p.Kr, 0.001));
}

TEST_CASE("ResonantController attenuates strongly away from the target frequency",
          "[resonant_controller]")
{
    const double Ts = 1e-4;
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0;
    p.dampingRadPerSec = 5.0;
    p.Kr = 2.0;
    ctrl::ResonantController rc(p, Ts);

    const int N = 30000;
    double maxAbsLastCycle = 0.0;
    const double fOff = 150.0;
    for (int k = 0; k < N; ++k)
    {
        const double e = std::sin(2.0 * M_PI * fOff * k * Ts);
        const double u = rc.compute(e);
        if (k >= N - 200)
            maxAbsLastCycle = std::max(maxAbsLastCycle, std::fabs(u));
    }
    REQUIRE(maxAbsLastCycle < 0.1); // << Kr=2.0 at resonance
}

TEST_CASE("ResonantController holds last output on a non-finite input and leaves state unchanged",
          "[resonant_controller]")
{
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0; p.dampingRadPerSec = 5.0; p.Kr = 2.0;
    ctrl::ResonantController rc(p, 1e-4);

    const double u1 = rc.compute(1.0);
    const double u_nan = rc.compute(std::numeric_limits<double>::quiet_NaN());
    REQUIRE(u_nan == u1);

    const double u2 = rc.compute(1.0);
    ctrl::ResonantController rc_ref(p, 1e-4);
    rc_ref.compute(1.0);
    const double u2_ref = rc_ref.compute(1.0);
    REQUIRE_THAT(u2, WithinAbs(u2_ref, 1e-12));
}

TEST_CASE("ResonantController reset() clears internal state", "[resonant_controller]")
{
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0; p.dampingRadPerSec = 5.0; p.Kr = 2.0;
    ctrl::ResonantController rc(p, 1e-4);

    for (int k = 0; k < 100; ++k)
        rc.compute(std::sin(2.0 * M_PI * 50.0 * k * 1e-4));
    rc.reset();

    ctrl::ResonantController rc_fresh(p, 1e-4);
    REQUIRE_THAT(rc.compute(1.0), WithinAbs(rc_fresh.compute(1.0), 1e-12));
}

TEST_CASE("ResonantController throws on invalid construction parameters", "[resonant_controller]")
{
    ctrl::ResonantParams p;
    p.targetFreqHz = 50.0; p.dampingRadPerSec = 5.0; p.Kr = 2.0;

    ctrl::ResonantParams bad1 = p; bad1.targetFreqHz = 0.0;
    REQUIRE_THROWS_AS(ctrl::ResonantController(bad1, 1e-4), std::invalid_argument);

    ctrl::ResonantParams bad2 = p; bad2.dampingRadPerSec = -1.0;
    REQUIRE_THROWS_AS(ctrl::ResonantController(bad2, 1e-4), std::invalid_argument);

    ctrl::ResonantParams bad3 = p; bad3.targetFreqHz = 6000.0; // Nyquist = 5000Hz at Ts=1e-4
    REQUIRE_THROWS_AS(ctrl::ResonantController(bad3, 1e-4), std::invalid_argument);
}

TEST_CASE("ResonantController reports TrackingErrorRMinusY as its sign convention",
          "[resonant_controller]")
{
    ctrl::ResonantParams p; p.targetFreqHz = 50.0; p.dampingRadPerSec = 5.0; p.Kr = 2.0;
    ctrl::ResonantController rc(p, 1e-4);
    REQUIRE(rc.signConvention() == ctrl::SignConvention::TrackingErrorRMinusY);
}
```

Also add `#include "ResonantController.h"` to the include block near the top of `tests/test_catch2_advanced.cpp` (after line 56, the `FreqDomainIdentifier.h` include) — though `ControllerToolbox.h` (already included at line 24) now pulls it in transitively via Step 4, so this is optional; add it anyway for explicitness, matching how other recently-added classes (`FreqDomainIdentifier.h` at line 56) are listed individually despite the umbrella include.

- [ ] **Step 6: Configure and build the test target**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release` then `cmake --build build --target test_catch2_advanced`
Expected: clean compile (no errors). If `ResonantController.cpp`/`.h` have a typo, this is where it surfaces.

- [ ] **Step 7: Run the new tests and verify they pass**

Run: `build/tests/test_catch2_advanced.exe [resonant_controller]`
Expected: `All tests passed (N assertions in 6 test cases)` — all 6 `TEST_CASE`s from Step 5 green.

- [ ] **Step 8: Commit**

```bash
git add lib/ResonantController.h lib/ResonantController.cpp lib/CMakeLists.txt lib/ControllerToolbox.h tests/test_catch2_advanced.cpp
git commit -m "Add ResonantController core (IController, Tustin-prewarped biquad)"
```

---

## Task 2: ResonantController integration (bindings, smoke test, example, docs)

**Files:**
- Modify: `bindings/controllers_bindings.cpp` (add binding section after the `DiscretePID` section, ~line 70)
- Modify: `bindings/smoke_test.py` (append new numbered section)
- Create: `examples/ex89_resonant_controller.cpp`
- Modify: `examples/CMakeLists.txt:135` (append `add_example(ex89_resonant_controller)`)
- Modify: `compile.bat:129` (append `ex89_resonant_controller` to the target list)
- Modify: `compile.sh:170` (append `ex89_resonant_controller` to the target list)
- Modify: `CONTRIBUTING.md` (append a row to the sign-convention table, ~line 162)

**Interfaces:**
- Consumes: `ctrl::ResonantController`/`ctrl::ResonantParams` (Task 1), `ctrl::ControllerStack`/`ctrl::StackMode::Additive` (existing), `ctrl::DiscretePID`/`ctrl::PIDParams` (existing)
- Produces: Python `ctrl.ResonantController`/`ctrl.ResonantParams`; example binary `ex89_resonant_controller`

- [ ] **Step 1: Add the Python binding to `bindings/controllers_bindings.cpp`**

Insert after the `DiscretePID` binding block (after line 70, before the `DiscreteLeadLag` section comment at line 72-73):

```cpp
    // -----------------------------------------------------------------------
    // ResonantController
    // -----------------------------------------------------------------------
    py::class_<ctrl::ResonantParams>(m, "ResonantParams",
        "Tuning parameters for ResonantController (single-harmonic internal-model corrector).")
        .def(py::init<>())
        .def_readwrite("targetFreqHz",     &ctrl::ResonantParams::targetFreqHz,
                       "Harmonic frequency to reject/track [Hz].")
        .def_readwrite("dampingRadPerSec", &ctrl::ResonantParams::dampingRadPerSec,
                       "Bandwidth/peak-width parameter wc [rad/s].")
        .def_readwrite("Kr",               &ctrl::ResonantParams::Kr,
                       "Resonant gain - the exact steady-state gain at targetFreqHz.")
        .def_readwrite("uMin",             &ctrl::ResonantParams::uMin, "Output saturation lower limit.")
        .def_readwrite("uMax",             &ctrl::ResonantParams::uMax, "Output saturation upper limit.");

    py::class_<ctrl::ResonantController, ctrl::IController,
               std::shared_ptr<ctrl::ResonantController>>(m, "ResonantController", R"doc(
Discrete-time non-ideal (finite-Q) resonant controller for single-harmonic rejection.

Composes through ControllerStack(Additive) alongside a base controller, one instance per
target harmonic.

Example
-------
>>> rp = ctrl.ResonantParams(); rp.targetFreqHz = 50.0; rp.dampingRadPerSec = 5.0; rp.Kr = 2.0
>>> rc = ctrl.ResonantController(rp, Ts=1e-4)
>>> u = rc.compute(error)
)doc")
        .def(py::init<const ctrl::ResonantParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("compute",     &ctrl::ResonantController::compute, py::arg("error"))
        .def("reset",       &ctrl::ResonantController::reset)
        .def("sample_time", &ctrl::ResonantController::sampleTime)
        .def("set_params",  &ctrl::ResonantController::setParams, py::arg("params"))
        .def("params",      &ctrl::ResonantController::params, py::return_value_policy::copy);
```

- [ ] **Step 2: Append to `bindings/smoke_test.py`**

Add after the existing `RepetitiveController` section (after line 253):

```python
# 15. ResonantController
rp = ctrl.ResonantParams()
rp.targetFreqHz = 50.0
rp.dampingRadPerSec = 5.0
rp.Kr = 2.0
rc = ctrl.ResonantController(rp, 1e-4)
u_rc = rc.compute(1.0)
assert math.isfinite(u_rc)
print(f'ResonantController compute(1.0) = {u_rc:.4f}')
```

(`math` is already imported earlier in the file for other sections; if this section ends up before any `import math` line in your edit location, add `import math` directly above it.)

- [ ] **Step 3: Rebuild and run the smoke test**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTRL_BUILD_PYTHON_BINDINGS=ON` then `cmake --build build --target ctrl_toolbox`
Run: `conda run -n soft_robotics -- python bindings/smoke_test.py`
Expected: no exceptions; output includes `ResonantController compute(1.0) = ...` with a finite number.

- [ ] **Step 4: Write `examples/ex89_resonant_controller.cpp`**

```cpp
/**
 * ex89_resonant_controller.cpp
 * Additional Controller Types: ResonantController multi-harmonic disturbance rejection.
 *
 * Demonstrates the composition pattern from
 * docs/superpowers/specs/2026-06-24-resonant-notch-pll-controllers-design.md: a
 * ControllerStack(Additive) holds a base PID plus one ResonantController per harmonic. A
 * periodic process disturbance (5th + 7th harmonic of a 0.5 Hz fundamental) enters at the
 * plant input; the PID alone leaves a steady-state ripple, the stack with the resonant
 * correctors added largely cancels it - the stack, not a one-off manual sum, does the work.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

namespace
{

double runScenario(const ctrl::StateSpace &sys, double Ts, bool withResonant)
{
    ctrl::PIDParams pp;
    pp.Kp = 2.0; pp.Ki = 5.0; pp.Kd = 0.0;
    pp.uMin = -1e6; pp.uMax = 1e6;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::ControllerStack stack(ctrl::StackMode::Additive, Ts);
    stack.addController(pid, "PID-base", 1.0);

    if (withResonant)
    {
        ctrl::ResonantParams rp5;
        rp5.targetFreqHz = 2.5; // 5th harmonic of the 0.5Hz fundamental
        rp5.dampingRadPerSec = 5.0;
        rp5.Kr = 200.0;
        auto rc5 = std::make_shared<ctrl::ResonantController>(rp5, Ts);

        ctrl::ResonantParams rp7;
        rp7.targetFreqHz = 3.5; // 7th harmonic
        rp7.dampingRadPerSec = 5.0;
        rp7.Kr = 200.0;
        auto rc7 = std::make_shared<ctrl::ResonantController>(rp7, Ts);

        stack.addController(rc5, "5th-harmonic-RC", 1.0);
        stack.addController(rc7, "7th-harmonic-RC", 1.0);
    }

    const double r = 1.0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
    double y = 0.0;
    const int N = 60000;                  // 60s = 30 cycles of the 0.5Hz fundamental
    const int samplesPerFundCycle = 2000;  // 1/0.5Hz / Ts
    double maxAbsErrLastCycle = 0.0;

    for (int k = 0; k < N; ++k)
    {
        const double e = r - y;
        const double u = stack.compute(e);
        const double d = 0.3 * std::sin(2.0 * M_PI * 2.5 * k * Ts)
                        + 0.2 * std::sin(2.0 * M_PI * 3.5 * k * Ts);
        Eigen::VectorXd uv(1); uv(0) = u + d; // process disturbance, enters with the control input
        y = ctrl::ssStep(sys, x, uv)(0);
        if (k >= N - samplesPerFundCycle)
            maxAbsErrLastCycle = std::max(maxAbsErrLastCycle, std::fabs(e));
    }
    return maxAbsErrLastCycle;
}

} // namespace

int main()
{
    const double Ts = 1e-3;
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1, 1, -1.0),
        Eigen::MatrixXd::Constant(1, 1,  1.0),
        Eigen::MatrixXd::Constant(1, 1,  1.0),
        Eigen::MatrixXd::Zero(1, 1), 0.0);
    const ctrl::StateSpace sys = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    const double ripplePidOnly = runScenario(sys, Ts, false);
    const double rippleWithRC  = runScenario(sys, Ts, true);

    std::cout << "PID-only steady-state ripple:     " << ripplePidOnly << "\n";
    std::cout << "PID+resonant steady-state ripple: " << rippleWithRC << "\n";
    std::cout << "ratio (with/without):              " << (rippleWithRC / ripplePidOnly) << "\n";

    const bool ok = std::isfinite(rippleWithRC) && std::isfinite(ripplePidOnly)
                  && rippleWithRC < 0.3 * ripplePidOnly;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
```

This has been verified numerically in a scratch build: `ripplePidOnly ≈ 0.0282`, `rippleWithRC ≈ 0.0065` (ratio ≈ 0.23 at the originally-tried `Kr=50`; the `Kr=200` used here measured ratio ≈ 0.065), both comfortably under the `0.3` threshold.

- [ ] **Step 5: Wire the example into `examples/CMakeLists.txt`**

Insert after line 135 (`add_example(ex88_h2_synthesis)`):

```cmake
# Additional Controller Types: ResonantController multi-harmonic disturbance rejection
add_example(ex89_resonant_controller)
```

- [ ] **Step 6: Wire the example into `compile.bat` and `compile.sh`**

In `compile.bat`, insert after `ex88_h2_synthesis` in the target list (line 129):
```
    ex89_resonant_controller
```

In `compile.sh`, insert after `ex88_h2_synthesis` in the `TARGETS` list (line 170):
```
    ex89_resonant_controller
```

- [ ] **Step 7: Build and run the example**

Run: `cmake --build build --target ex89_resonant_controller`
Run: `build/examples/ex89_resonant_controller.exe` (path may be `build/examples/Release/...` depending on generator)
Expected: prints the three ripple lines, then `PASS`.

- [ ] **Step 8: Add the sign-convention table row to `CONTRIBUTING.md`**

In the table at line 152-162, insert a new row after the `NonlinearMPC` row (line 162):
```markdown
| `ResonantController` | `e = r - y` (tracking error) | Composes via `ControllerStack(Additive)`; one instance per target harmonic |
```

- [ ] **Step 9: Commit**

```bash
git add bindings/controllers_bindings.cpp bindings/smoke_test.py examples/ex89_resonant_controller.cpp examples/CMakeLists.txt compile.bat compile.sh CONTRIBUTING.md
git commit -m "Wire ResonantController into Python bindings, smoke test, and ex89"
```

---

## Task 3: NotchFilter core (class + library wiring + Catch2 tests)

**Files:**
- Create: `lib/NotchFilter.h`
- Create: `lib/NotchFilter.cpp`
- Modify: `lib/CMakeLists.txt` (append after `ResonantController.cpp`, added in Task 1 Step 3)
- Modify: `lib/ControllerToolbox.h` (append include after `ResonantController.h`, added in Task 1 Step 4)
- Test: `tests/test_catch2_advanced.cpp` (append new `TEST_CASE`s, tag `[notch_filter]`)

**Interfaces:**
- Consumes: `CTRL_REGISTER_FEATURE` (`lib/Features.h`) — no `IController` dependency
- Produces: `struct ctrl::NotchFilterParams { double centerFreqHz; double Q; }`; `class ctrl::NotchFilter` (standalone) with constructor `NotchFilter(const NotchFilterParams&, double Ts)`, `double apply(double x)`, `void reset()`, `void setParams(const NotchFilterParams&)`, `const NotchFilterParams& params() const`.

- [ ] **Step 1: Write `lib/NotchFilter.h`**

```cpp
#pragma once
#include "Features.h"

/**
 * @file NotchFilter.h
 * @brief Discrete biquad notch filter (Bristow-Johnson "Audio EQ Cookbook" formulation).
 *
 * Attenuates a single frequency band in any sampled signal - no setpoint/error semantics, so
 * this is a standalone filter, not an IController. Apply it to a measurement, an actuator
 * command, or any other signal that needs a known resonance/disturbance frequency suppressed.
 *
 * **Digital design** (computed directly from the warped digital center frequency - the
 * standard "cookbook" approach, no separate continuous-to-discrete step):
 * @code
 *   omega = 2*pi*centerFreqHz*Ts
 *   alpha = sin(omega) / (2*Q)
 *   b0 =  1,            b1 = -2*cos(omega),  b2 =  1
 *   a0 =  1 + alpha,     a1 = -2*cos(omega),  a2 =  1 - alpha
 *   (normalise b0,b1,b2,a1,a2 by dividing through by a0)
 *
 *   y[k] = b0.x[k] + b1.x[k-1] + b2.x[k-2] - a1.y[k-1] - a2.y[k-2]
 * @endcode
 *
 * The numerator has exact zeros at z = e^{+-j*omega} (on the unit circle), so a sinusoid fed
 * in at exactly centerFreqHz is attenuated to (numerically) zero in steady state - verified
 * numerically in docs/superpowers/plans/2026-06-24-resonant-notch-pll-controllers.md.
 *
 * @see Bristow-Johnson, "Cookbook formulae for audio EQ biquad filter coefficients".
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for NotchFilter.
 */
struct NotchFilterParams
{
    double centerFreqHz; ///< f0 to attenuate [Hz].
    double Q;             ///< Quality factor = f0 / bandwidth. Higher Q = narrower notch.
};

/**
 * @brief Discrete biquad notch filter.
 */
class NotchFilter
{
public:
    /**
     * @brief Construct the filter and precompute biquad coefficients.
     * @param params Center frequency and Q.
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument if centerFreqHz <= 0, Q <= 0, or centerFreqHz is at/above
     *         the Nyquist frequency (1/(2*Ts)).
     */
    NotchFilter(const NotchFilterParams &params, double Ts);

    /**
     * @brief Filter one sample.
     * @param x Input sample x[k].
     * @return Filtered output y[k].
     */
    double apply(double x);

    /** @brief Reset previous inputs/outputs to zero. */
    void reset();

    /** @brief Hot-update parameters and recompute biquad coefficients. */
    void setParams(const NotchFilterParams &p);

    /** @brief Read-only access to current parameters. */
    const NotchFilterParams &params() const { return p_; }

private:
    NotchFilterParams p_;
    double Ts_;
    double b0_ = 0.0, b1_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0;
    double x_prev1_ = 0.0, x_prev2_ = 0.0;
    double y_prev1_ = 0.0, y_prev2_ = 0.0;

    void computeCoeffs();
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(notch_filter)
```

- [ ] **Step 2: Write `lib/NotchFilter.cpp`**

```cpp
#include "NotchFilter.h"
#include <cmath>
#include <stdexcept>

namespace ctrl
{

NotchFilter::NotchFilter(const NotchFilterParams &params, double Ts)
    : p_(params), Ts_(Ts)
{
    if (p_.centerFreqHz <= 0.0)
        throw std::invalid_argument("NotchFilter: centerFreqHz must be positive");
    if (p_.Q <= 0.0)
        throw std::invalid_argument("NotchFilter: Q must be positive");
    if (p_.centerFreqHz >= 1.0 / (2.0 * Ts_))
        throw std::invalid_argument("NotchFilter: centerFreqHz must be below the Nyquist frequency");
    computeCoeffs();
    reset();
}

void NotchFilter::computeCoeffs()
{
    const double omega = 2.0 * M_PI * p_.centerFreqHz * Ts_;
    const double alpha = std::sin(omega) / (2.0 * p_.Q);
    const double cos_omega = std::cos(omega);
    const double a0 = 1.0 + alpha;

    b0_ = 1.0 / a0;
    b1_ = (-2.0 * cos_omega) / a0;
    b2_ = 1.0 / a0;
    a1_ = (-2.0 * cos_omega) / a0;
    a2_ = (1.0 - alpha) / a0;
}

double NotchFilter::apply(double x)
{
    if (!std::isfinite(x))
        return y_prev1_;

    const double y = b0_ * x + b1_ * x_prev1_ + b2_ * x_prev2_
                    - a1_ * y_prev1_ - a2_ * y_prev2_;

    x_prev2_ = x_prev1_;
    x_prev1_ = x;
    y_prev2_ = y_prev1_;
    y_prev1_ = y;
    return y;
}

void NotchFilter::reset()
{
    x_prev1_ = x_prev2_ = 0.0;
    y_prev1_ = y_prev2_ = 0.0;
}

void NotchFilter::setParams(const NotchFilterParams &p)
{
    p_ = p;
    computeCoeffs();
}

} // namespace ctrl
```

- [ ] **Step 3: Wire into `lib/CMakeLists.txt`**

Append `NotchFilter.cpp` to `CTRL_CORE_SOURCES` (after `ResonantController.cpp` from Task 1 Step 3):
```cmake
    ResonantController.cpp
    NotchFilter.cpp
)
```

- [ ] **Step 4: Wire into `lib/ControllerToolbox.h`**

Append after the `ResonantController.h` include (from Task 1 Step 4):
```cpp
#include "ResonantController.h"       ///< ResonantController - single-harmonic internal-model corrector; composes via ControllerStack(Additive).
#include "NotchFilter.h"              ///< NotchFilter - fixed-design biquad notch filter (Bristow-Johnson cookbook); no IController base.
```

- [ ] **Step 5: Append failing Catch2 tests to `tests/test_catch2_advanced.cpp`**

```cpp
TEST_CASE("NotchFilter attenuates a sinusoid exactly at the center frequency to near zero",
          "[notch_filter]")
{
    const double Ts = 1e-4;
    ctrl::NotchFilterParams p;
    p.centerFreqHz = 50.0;
    p.Q = 10.0;
    ctrl::NotchFilter nf(p, Ts);

    const int N = 6000; // 30 cycles - enough for the near-unit-circle poles (Q=10) to settle
    double maxAbsLastCycle = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double x = std::sin(2.0 * M_PI * p.centerFreqHz * k * Ts);
        const double y = nf.apply(x);
        if (k >= N - 200)
            maxAbsLastCycle = std::max(maxAbsLastCycle, std::fabs(y));
    }
    REQUIRE(maxAbsLastCycle < 0.01);
}

TEST_CASE("NotchFilter passes a sinusoid far from the center frequency through largely unchanged",
          "[notch_filter]")
{
    const double Ts = 1e-4;
    ctrl::NotchFilterParams p;
    p.centerFreqHz = 50.0;
    p.Q = 10.0;
    ctrl::NotchFilter nf(p, Ts);

    const int N = 3000;
    double maxAbsLastCycle = 0.0;
    const double fFar = 200.0;
    for (int k = 0; k < N; ++k)
    {
        const double x = std::sin(2.0 * M_PI * fFar * k * Ts);
        const double y = nf.apply(x);
        if (k >= N - 200)
            maxAbsLastCycle = std::max(maxAbsLastCycle, std::fabs(y));
    }
    REQUIRE(maxAbsLastCycle > 0.9);
    REQUIRE(maxAbsLastCycle < 1.1);
}

TEST_CASE("NotchFilter holds last output on a non-finite input", "[notch_filter]")
{
    ctrl::NotchFilterParams p; p.centerFreqHz = 50.0; p.Q = 10.0;
    ctrl::NotchFilter nf(p, 1e-4);

    const double y1 = nf.apply(1.0);
    const double y_nan = nf.apply(std::numeric_limits<double>::quiet_NaN());
    REQUIRE(y_nan == y1);
}

TEST_CASE("NotchFilter reset() clears internal state", "[notch_filter]")
{
    ctrl::NotchFilterParams p; p.centerFreqHz = 50.0; p.Q = 10.0;
    ctrl::NotchFilter nf(p, 1e-4);

    for (int k = 0; k < 100; ++k)
        nf.apply(std::sin(2.0 * M_PI * 50.0 * k * 1e-4));
    nf.reset();

    ctrl::NotchFilter nf_fresh(p, 1e-4);
    REQUIRE_THAT(nf.apply(1.0), WithinAbs(nf_fresh.apply(1.0), 1e-12));
}

TEST_CASE("NotchFilter throws on invalid construction parameters", "[notch_filter]")
{
    ctrl::NotchFilterParams p; p.centerFreqHz = 50.0; p.Q = 10.0;

    ctrl::NotchFilterParams bad1 = p; bad1.centerFreqHz = 0.0;
    REQUIRE_THROWS_AS(ctrl::NotchFilter(bad1, 1e-4), std::invalid_argument);

    ctrl::NotchFilterParams bad2 = p; bad2.Q = -1.0;
    REQUIRE_THROWS_AS(ctrl::NotchFilter(bad2, 1e-4), std::invalid_argument);

    ctrl::NotchFilterParams bad3 = p; bad3.centerFreqHz = 6000.0;
    REQUIRE_THROWS_AS(ctrl::NotchFilter(bad3, 1e-4), std::invalid_argument);
}
```

- [ ] **Step 6: Configure and build the test target**

Run: `cmake --build build --target test_catch2_advanced`
Expected: clean compile.

- [ ] **Step 7: Run the new tests and verify they pass**

Run: `build/tests/test_catch2_advanced.exe [notch_filter]`
Expected: all 5 test cases pass. (Numerically verified in scratch: at-center peak ≈ `1.01e-4`, far-from-center peak ≈ `0.999`.)

- [ ] **Step 8: Commit**

```bash
git add lib/NotchFilter.h lib/NotchFilter.cpp lib/CMakeLists.txt lib/ControllerToolbox.h tests/test_catch2_advanced.cpp
git commit -m "Add NotchFilter core (standalone biquad, Bristow-Johnson cookbook)"
```

---

## Task 4: NotchFilter integration (bindings, smoke test, example)

**Files:**
- Modify: `bindings/estimation_bindings.cpp` (add binding section after the `ParticleFilter` section, ~line 503)
- Modify: `bindings/smoke_test.py` (append new numbered section)
- Create: `examples/ex90_notch_filter.cpp`
- Modify: `examples/CMakeLists.txt` (append `add_example(ex90_notch_filter)` after `ex89_resonant_controller`)
- Modify: `compile.bat` (append `ex90_notch_filter` after `ex89_resonant_controller`)
- Modify: `compile.sh` (append `ex90_notch_filter` after `ex89_resonant_controller`)

**Interfaces:**
- Consumes: `ctrl::NotchFilter`/`ctrl::NotchFilterParams` (Task 3)
- Produces: Python `ctrl.NotchFilter`/`ctrl.NotchFilterParams`; example binary `ex90_notch_filter`

- [ ] **Step 1: Add the Python binding to `bindings/estimation_bindings.cpp`**

Insert after the `ParticleFilter` section (after line 503, before the `GreyBoxEstimator` section comment):

```cpp
    // -----------------------------------------------------------------------
    // NotchFilter
    // -----------------------------------------------------------------------
    py::class_<ctrl::NotchFilterParams>(m, "NotchFilterParams",
        "Tuning parameters for NotchFilter.")
        .def(py::init<>())
        .def_readwrite("centerFreqHz", &ctrl::NotchFilterParams::centerFreqHz,
                       "Center frequency to attenuate [Hz].")
        .def_readwrite("Q",            &ctrl::NotchFilterParams::Q,
                       "Quality factor = centerFreqHz / bandwidth.");

    py::class_<ctrl::NotchFilter>(m, "NotchFilter", R"doc(
Discrete biquad notch filter (Bristow-Johnson "Audio EQ Cookbook" formulation).

No setpoint/error semantics - applies to any sampled signal.

Example
-------
>>> p = ctrl.NotchFilterParams(); p.centerFreqHz = 50.0; p.Q = 10.0
>>> nf = ctrl.NotchFilter(p, Ts=1e-4)
>>> y = nf.apply(x)
)doc")
        .def(py::init<const ctrl::NotchFilterParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("apply",      &ctrl::NotchFilter::apply, py::arg("x"))
        .def("reset",      &ctrl::NotchFilter::reset)
        .def("set_params", &ctrl::NotchFilter::setParams, py::arg("params"))
        .def("params",     &ctrl::NotchFilter::params, py::return_value_policy::copy);
```

- [ ] **Step 2: Append to `bindings/smoke_test.py`**

Add after the `ResonantController` section (Task 2 Step 2):

```python
# 16. NotchFilter
nfp = ctrl.NotchFilterParams()
nfp.centerFreqHz = 50.0
nfp.Q = 10.0
nf = ctrl.NotchFilter(nfp, 1e-4)
y_nf = nf.apply(1.0)
assert math.isfinite(y_nf)
print(f'NotchFilter apply(1.0) = {y_nf:.4f}')
```

- [ ] **Step 3: Rebuild and run the smoke test**

Run: `cmake --build build --target ctrl_toolbox`
Run: `conda run -n soft_robotics -- python bindings/smoke_test.py`
Expected: output includes `NotchFilter apply(1.0) = ...` with a finite number.

- [ ] **Step 4: Write `examples/ex90_notch_filter.cpp`**

```cpp
/**
 * ex90_notch_filter.cpp
 * Additional Controller Types: NotchFilter removing a known resonance from a synthetic signal.
 *
 * Feeds a synthetic sinusoid at the filter's center frequency (an unwanted resonance): the
 * notch attenuates it almost completely. A sinusoid well outside the notch band passes
 * through largely unchanged.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 1e-4;
    ctrl::NotchFilterParams p;
    p.centerFreqHz = 50.0;
    p.Q = 10.0;

    const int N = 6000; // 30 cycles of the 50Hz resonance - ample settling for Q=10

    ctrl::NotchFilter nf_at(p, Ts);
    double peakAtResonance = 0.0;
    for (int k = 0; k < N; ++k)
    {
        const double x = std::sin(2.0 * M_PI * p.centerFreqHz * k * Ts);
        const double y = nf_at.apply(x);
        if (k >= N - 200)
            peakAtResonance = std::max(peakAtResonance, std::fabs(y));
    }

    ctrl::NotchFilter nf_far(p, Ts);
    double peakFarFromResonance = 0.0;
    const double fFar = 200.0;
    const int N2 = 3000;
    for (int k = 0; k < N2; ++k)
    {
        const double x = std::sin(2.0 * M_PI * fFar * k * Ts);
        const double y = nf_far.apply(x);
        if (k >= N2 - 200)
            peakFarFromResonance = std::max(peakFarFromResonance, std::fabs(y));
    }

    std::cout << "Unwanted 50Hz resonance: amplitude 1.0 before, " << peakAtResonance << " after notch\n";
    std::cout << "Unrelated 200Hz content: amplitude 1.0 before, " << peakFarFromResonance << " after notch\n";

    const bool ok = std::isfinite(peakAtResonance) && std::isfinite(peakFarFromResonance)
                   && peakAtResonance < 0.01 && peakFarFromResonance > 0.9;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
```

(Numerically verified in scratch: `peakAtResonance ≈ 1.01e-4`, `peakFarFromResonance ≈ 0.999`.)

- [ ] **Step 5: Wire the example into `examples/CMakeLists.txt`**

```cmake
# Additional Controller Types: NotchFilter removing a known resonance
add_example(ex90_notch_filter)
```

- [ ] **Step 6: Wire the example into `compile.bat` and `compile.sh`**

In both files, insert after `ex89_resonant_controller`:
```
    ex90_notch_filter
```

- [ ] **Step 7: Build and run the example**

Run: `cmake --build build --target ex90_notch_filter`
Run: `build/examples/ex90_notch_filter.exe`
Expected: prints the two attenuation lines, then `PASS`.

- [ ] **Step 8: Commit**

```bash
git add bindings/estimation_bindings.cpp bindings/smoke_test.py examples/ex90_notch_filter.cpp examples/CMakeLists.txt compile.bat compile.sh
git commit -m "Wire NotchFilter into Python bindings, smoke test, and ex90"
```

---

## Task 5: PhaseLockedLoop core (class + library wiring + Catch2 tests)

**Files:**
- Create: `lib/PhaseLockedLoop.h`
- Create: `lib/PhaseLockedLoop.cpp`
- Modify: `lib/CMakeLists.txt` (append after `NotchFilter.cpp`, added in Task 3 Step 3)
- Modify: `lib/ControllerToolbox.h` (append include after `NotchFilter.h`, added in Task 3 Step 4)
- Test: `tests/test_catch2_advanced.cpp` (append new `TEST_CASE`s, tag `[pll]`)

**Interfaces:**
- Consumes: `CTRL_REGISTER_FEATURE` (`lib/Features.h`) — no `IController` dependency
- Produces: `struct ctrl::PLLParams { double nominalFreqHz; double Kp; double Ki; double sogiK=1.41421356; }`; `class ctrl::PhaseLockedLoop` (standalone) with constructor `PhaseLockedLoop(const PLLParams&, double Ts)`, `void step(double sample)`, `double phase() const`, `double frequencyHz() const`, `double amplitude() const`, `bool locked() const`, `void reset()`, `const PLLParams& params() const`.

**Derivation note (read before implementing):** the SOGI quadrature generator's states settle to `x1 ≈ A*sin(theta_true)`, `x2 ≈ A*cos(theta_true)` in steady state (verified numerically — *not* `x1≈A*cos`, `x2≈-A*sin` as a naive phasor derivation might suggest). The Park-transform error signal that correctly drives `theta_hat -> theta_true` with the right feedback sign is `v_q = x1*cos(theta_hat) - x2*sin(theta_hat)` (`= A*sin(theta_true - theta_hat)`, vanishing at lock), fed *directly* (no extra negation) into a positive-gain PI loop filter. Getting either the `x1`/`x2` roles or the sign of `v_q` wrong still produces a PLL that "locks" (small `v_q`) but at a phase offset by roughly 90 degrees from the true phase — the bug is silent unless you check the phase value itself, not just `locked()`. The code below has already had this bug found and fixed.

- [ ] **Step 1: Write `lib/PhaseLockedLoop.h`**

```cpp
#pragma once
#include "Features.h"

/**
 * @file PhaseLockedLoop.h
 * @brief Single-input SOGI-PLL: tracks the phase and frequency of one sampled sinusoid.
 *
 * Produces an estimate (phase, frequency, amplitude), not a control action, so this is a
 * standalone estimator - same philosophy as KalmanFilter, no shared base with IController.
 *
 * **Pipeline per sample** (Ciobotaru, Teodorescu & Blaabjerg, "A New Single-Phase PLL
 * Structure Based on Second Order Generalized Integrator", IEEE PESC 2006):
 * 1. SOGI quadrature signal generator (forward-Euler discretised, tuned to the current
 *    frequency estimate w_hat) manufactures states x1, x2 from the single input sample. In
 *    steady state x1 ~= A*sin(theta_true), x2 ~= A*cos(theta_true) (verified numerically -
 *    this is the opposite role/sign pairing a naive continuous-domain phasor derivation
 *    suggests; see the implementation plan this class was specified from for the full
 *    derivation and the bug it caught).
 * 2. Park-transform-style error signal v_q = x1*cos(theta_hat) - x2*sin(theta_hat)
 *    (= A*sin(theta_true - theta_hat)), which vanishes when theta_hat == theta_true.
 * 3. A PI loop filter on v_q (direct, no extra sign flip) produces a frequency correction.
 * 4. The corrected frequency drives an NCO (phase integrator), wrapped to [-pi, pi).
 *
 * @see Ciobotaru, Teodorescu & Blaabjerg (2006).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for PhaseLockedLoop.
 */
struct PLLParams
{
    double nominalFreqHz;        ///< Expected/center frequency [Hz].
    double Kp;                   ///< PI loop-filter proportional gain.
    double Ki;                   ///< PI loop-filter integral gain.
    double sogiK = 1.41421356;   ///< SOGI damping gain (default sqrt(2)).
};

/**
 * @brief Single-input SOGI-based phase-locked loop.
 */
class PhaseLockedLoop
{
public:
    /**
     * @brief Construct the PLL and initialise its internal state at the nominal frequency.
     * @param params Nominal frequency and PI loop-filter gains.
     * @param Ts     Sample time [s].
     * @throws std::invalid_argument if nominalFreqHz <= 0, or nominalFreqHz is at/above the
     *         Nyquist frequency (1/(2*Ts)).
     */
    PhaseLockedLoop(const PLLParams &params, double Ts);

    /**
     * @brief Advance the PLL by one sample.
     * @param sample Measured sinusoid value v[k]. A non-finite sample is skipped entirely -
     *               the estimate holds at its last value and internal state is unchanged.
     */
    void step(double sample);

    /** @brief Current phase estimate theta_hat [rad], wrapped to [-pi, pi). */
    double phase() const { return theta_hat_; }

    /** @brief Current frequency estimate [Hz]. */
    double frequencyHz() const { return w_hat_ / (2.0 * M_PI); }

    /** @brief Estimated input amplitude sqrt(x1^2 + x2^2) - a free byproduct of the SOGI. */
    double amplitude() const;

    /** @brief True once |v_q| has stayed below a small fraction of the amplitude for long enough. */
    bool locked() const { return lockCounter_ >= kLockCountRequired; }

    /** @brief Reset all internal state; frequency estimate returns to nominalFreqHz. */
    void reset();

    /** @brief Read-only access to current parameters. */
    const PLLParams &params() const { return p_; }

private:
    static constexpr int kLockCountRequired = 50;

    PLLParams p_;
    double Ts_;
    double x1_ = 0.0, x2_ = 0.0; ///< SOGI quadrature-generator states.
    double integral_ = 0.0;      ///< PI loop-filter integrator state.
    double theta_hat_ = 0.0;     ///< Phase estimate [rad].
    double w_hat_ = 0.0;         ///< Frequency estimate [rad/s].
    int    lockCounter_ = 0;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(phase_locked_loop)
```

- [ ] **Step 2: Write `lib/PhaseLockedLoop.cpp`**

```cpp
#include "PhaseLockedLoop.h"
#include <cmath>
#include <stdexcept>

namespace ctrl
{

PhaseLockedLoop::PhaseLockedLoop(const PLLParams &params, double Ts)
    : p_(params), Ts_(Ts)
{
    if (p_.nominalFreqHz <= 0.0)
        throw std::invalid_argument("PhaseLockedLoop: nominalFreqHz must be positive");
    if (p_.nominalFreqHz >= 1.0 / (2.0 * Ts_))
        throw std::invalid_argument("PhaseLockedLoop: nominalFreqHz must be below the Nyquist frequency");
    reset();
}

void PhaseLockedLoop::step(double sample)
{
    if (!std::isfinite(sample))
        return;

    // SOGI quadrature generator (forward-Euler), tuned to the current frequency estimate:
    //   dx1/dt = w_hat*x2 + sogiK*w_hat*(v_in - x1)
    //   dx2/dt = -w_hat*x1
    const double x1_old = x1_;
    const double x2_old = x2_;
    x1_ = x1_old + Ts_ * (w_hat_ * x2_old + p_.sogiK * w_hat_ * (sample - x1_old));
    x2_ = x2_old + Ts_ * (-w_hat_ * x1_old);

    // Error signal: v_q = x1*cos(theta_hat) - x2*sin(theta_hat) = A*sin(theta_true-theta_hat),
    // vanishing at lock. See the header's derivation note for why this pairing/sign (not the
    // naive cos/-sin one) is the one that's actually correct.
    const double cosT = std::cos(theta_hat_);
    const double sinT = std::sin(theta_hat_);
    const double v_q = x1_ * cosT - x2_ * sinT;

    // PI loop filter, direct (no extra negation - see derivation note).
    integral_ += p_.Ki * v_q * Ts_;
    const double delta_w = p_.Kp * v_q + integral_;

    // NCO: corrected frequency drives the phase integrator, wrapped to [-pi, pi).
    w_hat_ = 2.0 * M_PI * p_.nominalFreqHz + delta_w;
    theta_hat_ += w_hat_ * Ts_;
    theta_hat_ = std::atan2(std::sin(theta_hat_), std::cos(theta_hat_));

    // Lock detection: |v_q| small relative to amplitude for kLockCountRequired consecutive samples.
    const double amp = std::sqrt(x1_ * x1_ + x2_ * x2_);
    if (std::fabs(v_q) < 0.02 * amp)
    {
        if (lockCounter_ < kLockCountRequired) ++lockCounter_;
    }
    else
    {
        lockCounter_ = 0;
    }
}

double PhaseLockedLoop::amplitude() const
{
    return std::sqrt(x1_ * x1_ + x2_ * x2_);
}

void PhaseLockedLoop::reset()
{
    x1_ = x2_ = 0.0;
    integral_ = 0.0;
    theta_hat_ = 0.0;
    w_hat_ = 2.0 * M_PI * p_.nominalFreqHz;
    lockCounter_ = 0;
}

} // namespace ctrl
```

- [ ] **Step 3: Wire into `lib/CMakeLists.txt`**

```cmake
    NotchFilter.cpp
    PhaseLockedLoop.cpp
)
```

- [ ] **Step 4: Wire into `lib/ControllerToolbox.h`**

```cpp
#include "NotchFilter.h"              ///< NotchFilter - fixed-design biquad notch filter (Bristow-Johnson cookbook); no IController base.
#include "PhaseLockedLoop.h"          ///< PhaseLockedLoop - single-input SOGI-PLL phase/frequency estimator; no IController base.
```

- [ ] **Step 5: Append failing Catch2 tests to `tests/test_catch2_advanced.cpp`**

```cpp
TEST_CASE("PhaseLockedLoop converges to the true phase and frequency of a synthetic sinusoid",
          "[pll]")
{
    const double Ts = 1e-4;
    ctrl::PLLParams p;
    p.nominalFreqHz = 50.0;
    p.Kp = 90.0;
    p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, Ts);

    const int N = 20000; // 2s, ~22x the ~90ms loop settling time
    double phaseTrue = 0.7; // nonzero initial offset to exercise convergence
    for (int k = 0; k < N; ++k)
    {
        pll.step(std::sin(phaseTrue));
        phaseTrue += 2.0 * M_PI * 50.0 * Ts;
    }
    const double trueWrapped = std::atan2(std::sin(phaseTrue), std::cos(phaseTrue));
    const double diff = std::atan2(std::sin(pll.phase() - trueWrapped),
                                    std::cos(pll.phase() - trueWrapped));

    REQUIRE(pll.locked());
    REQUIRE_THAT(pll.frequencyHz(), WithinAbs(50.0, 0.5));
    REQUIRE(std::fabs(diff) < 0.1);
}

TEST_CASE("PhaseLockedLoop re-converges after a step change in the true input frequency", "[pll]")
{
    const double Ts = 1e-4;
    ctrl::PLLParams p;
    p.nominalFreqHz = 50.0;
    p.Kp = 90.0;
    p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, Ts);

    const int N = 20000; // 2s total, 1s at 50Hz then 1s at 53Hz
    double phaseTrue = 0.0;
    double fTrue = 50.0;
    for (int k = 0; k < N; ++k)
    {
        if (k == N / 2) fTrue = 53.0;
        pll.step(std::sin(phaseTrue));
        phaseTrue += 2.0 * M_PI * fTrue * Ts;
    }

    REQUIRE(pll.locked());
    REQUIRE_THAT(pll.frequencyHz(), WithinAbs(53.0, 0.5));
}

TEST_CASE("PhaseLockedLoop holds its estimate on a non-finite sample", "[pll]")
{
    ctrl::PLLParams p; p.nominalFreqHz = 50.0; p.Kp = 90.0; p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, 1e-4);

    for (int k = 0; k < 100; ++k)
        pll.step(std::sin(2.0 * M_PI * 50.0 * k * 1e-4));
    const double freqBefore = pll.frequencyHz();
    const double phaseBefore = pll.phase();

    pll.step(std::numeric_limits<double>::quiet_NaN());

    REQUIRE(pll.frequencyHz() == freqBefore);
    REQUIRE(pll.phase() == phaseBefore);
}

TEST_CASE("PhaseLockedLoop reset() returns the frequency estimate to nominal", "[pll]")
{
    ctrl::PLLParams p; p.nominalFreqHz = 50.0; p.Kp = 90.0; p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, 1e-4);

    for (int k = 0; k < 1000; ++k)
        pll.step(std::sin(2.0 * M_PI * 53.0 * k * 1e-4)); // off-nominal input
    pll.reset();

    REQUIRE_THAT(pll.frequencyHz(), WithinAbs(50.0, 1e-9));
    REQUIRE_THAT(pll.phase(), WithinAbs(0.0, 1e-9));
    REQUIRE(!pll.locked());
}

TEST_CASE("PhaseLockedLoop throws on invalid construction parameters", "[pll]")
{
    ctrl::PLLParams p; p.nominalFreqHz = 50.0; p.Kp = 90.0; p.Ki = 4000.0;

    ctrl::PLLParams bad1 = p; bad1.nominalFreqHz = 0.0;
    REQUIRE_THROWS_AS(ctrl::PhaseLockedLoop(bad1, 1e-4), std::invalid_argument);

    ctrl::PLLParams bad2 = p; bad2.nominalFreqHz = 6000.0;
    REQUIRE_THROWS_AS(ctrl::PhaseLockedLoop(bad2, 1e-4), std::invalid_argument);
}
```

- [ ] **Step 6: Configure and build the test target**

Run: `cmake --build build --target test_catch2_advanced`
Expected: clean compile.

- [ ] **Step 7: Run the new tests and verify they pass**

Run: `build/tests/test_catch2_advanced.exe [pll]`
Expected: all 5 test cases pass. (Numerically verified in scratch: basic-lock freq diff ≈ `0.027 Hz`, phase diff ≈ `0.025 rad`; freq-step final freq ≈ `53.13 Hz`.)

- [ ] **Step 8: Commit**

```bash
git add lib/PhaseLockedLoop.h lib/PhaseLockedLoop.cpp lib/CMakeLists.txt lib/ControllerToolbox.h tests/test_catch2_advanced.cpp
git commit -m "Add PhaseLockedLoop core (single-input SOGI-PLL estimator)"
```

---

## Task 6: PhaseLockedLoop integration (bindings, smoke test, example)

**Files:**
- Modify: `bindings/estimation_bindings.cpp` (add binding section after the `NotchFilter` section from Task 4 Step 1)
- Modify: `bindings/smoke_test.py` (append new numbered section)
- Create: `examples/ex91_phase_locked_loop.cpp`
- Modify: `examples/CMakeLists.txt` (append `add_example(ex91_phase_locked_loop)` after `ex90_notch_filter`)
- Modify: `compile.bat` (append `ex91_phase_locked_loop` after `ex90_notch_filter`)
- Modify: `compile.sh` (append `ex91_phase_locked_loop` after `ex90_notch_filter`)

**Interfaces:**
- Consumes: `ctrl::PhaseLockedLoop`/`ctrl::PLLParams` (Task 5)
- Produces: Python `ctrl.PhaseLockedLoop`/`ctrl.PLLParams`; example binary `ex91_phase_locked_loop`

- [ ] **Step 1: Add the Python binding to `bindings/estimation_bindings.cpp`**

Insert after the `NotchFilter` binding section (added in Task 4 Step 1):

```cpp
    // -----------------------------------------------------------------------
    // PhaseLockedLoop
    // -----------------------------------------------------------------------
    py::class_<ctrl::PLLParams>(m, "PLLParams",
        "Tuning parameters for PhaseLockedLoop.")
        .def(py::init<>())
        .def_readwrite("nominalFreqHz", &ctrl::PLLParams::nominalFreqHz, "Expected/center frequency [Hz].")
        .def_readwrite("Kp",            &ctrl::PLLParams::Kp, "PI loop-filter proportional gain.")
        .def_readwrite("Ki",            &ctrl::PLLParams::Ki, "PI loop-filter integral gain.")
        .def_readwrite("sogiK",         &ctrl::PLLParams::sogiK, "SOGI damping gain (default sqrt(2)).");

    py::class_<ctrl::PhaseLockedLoop>(m, "PhaseLockedLoop", R"doc(
Single-input SOGI-PLL: tracks the phase and frequency of one sampled sinusoid.

Example
-------
>>> p = ctrl.PLLParams(); p.nominalFreqHz = 50.0; p.Kp = 90.0; p.Ki = 4000.0
>>> pll = ctrl.PhaseLockedLoop(p, Ts=1e-4)
>>> pll.step(sample)
>>> pll.frequency_hz(), pll.phase(), pll.locked()
)doc")
        .def(py::init<const ctrl::PLLParams &, double>(),
             py::arg("params"), py::arg("Ts"))
        .def("step",         &ctrl::PhaseLockedLoop::step, py::arg("sample"))
        .def("phase",        &ctrl::PhaseLockedLoop::phase)
        .def("frequency_hz", &ctrl::PhaseLockedLoop::frequencyHz)
        .def("amplitude",    &ctrl::PhaseLockedLoop::amplitude)
        .def("locked",       &ctrl::PhaseLockedLoop::locked)
        .def("reset",        &ctrl::PhaseLockedLoop::reset)
        .def("params",       &ctrl::PhaseLockedLoop::params, py::return_value_policy::copy);
```

- [ ] **Step 2: Append to `bindings/smoke_test.py`**

```python
# 17. PhaseLockedLoop
pllp = ctrl.PLLParams()
pllp.nominalFreqHz = 50.0
pllp.Kp = 90.0
pllp.Ki = 4000.0
pll = ctrl.PhaseLockedLoop(pllp, 1e-4)
pll.step(1.0)
assert math.isfinite(pll.frequency_hz())
print(f'PhaseLockedLoop frequency_hz() after one step = {pll.frequency_hz():.4f}')
```

- [ ] **Step 3: Rebuild and run the smoke test**

Run: `cmake --build build --target ctrl_toolbox`
Run: `conda run -n soft_robotics -- python bindings/smoke_test.py`
Expected: output includes `PhaseLockedLoop frequency_hz() after one step = ...` with a finite number.

- [ ] **Step 4: Write `examples/ex91_phase_locked_loop.cpp`**

```cpp
/**
 * ex91_phase_locked_loop.cpp
 * Additional Controller Types: PhaseLockedLoop tracking a synthetic AC signal's phase/frequency.
 *
 * Feeds a synthetic sinusoid at 50Hz for the first second, then steps the true frequency to
 * 53Hz for the second second, and shows the PLL's frequency estimate re-converging.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 1e-4;
    ctrl::PLLParams p;
    p.nominalFreqHz = 50.0;
    p.Kp = 90.0;
    p.Ki = 4000.0;
    ctrl::PhaseLockedLoop pll(p, Ts);

    const int N = 20000; // 2s total: 1s @ 50Hz, then a step to 53Hz for the 2nd second
    double phaseTrue = 0.7; // nonzero initial phase offset
    double fTrue = 50.0;
    for (int k = 0; k < N; ++k)
    {
        if (k == N / 2)
            fTrue = 53.0;
        pll.step(std::sin(phaseTrue));
        phaseTrue += 2.0 * M_PI * fTrue * Ts;

        if (k % 2000 == 0)
            std::cout << "t=" << k * Ts << "s  freq_est=" << pll.frequencyHz()
                      << "Hz  locked=" << pll.locked() << "\n";
    }

    std::cout << "Final frequency estimate: " << pll.frequencyHz() << " Hz (true 53.0 Hz)\n";

    const bool ok = pll.locked() && std::isfinite(pll.frequencyHz())
                   && std::fabs(pll.frequencyHz() - 53.0) < 0.5;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
```

(Numerically verified in scratch: final frequency estimate ≈ `53.13 Hz`.)

- [ ] **Step 5: Wire the example into `examples/CMakeLists.txt`**

```cmake
# Additional Controller Types: PhaseLockedLoop tracking a synthetic AC signal
add_example(ex91_phase_locked_loop)
```

- [ ] **Step 6: Wire the example into `compile.bat` and `compile.sh`**

In both files, insert after `ex90_notch_filter`:
```
    ex91_phase_locked_loop
```

- [ ] **Step 7: Build and run the example**

Run: `cmake --build build --target ex91_phase_locked_loop`
Run: `build/examples/ex91_phase_locked_loop.exe`
Expected: prints periodic frequency estimates, then the final estimate, then `PASS`.

- [ ] **Step 8: Run the full test suite and full example list once, to confirm nothing else broke**

Run: `ctest --test-dir build --output-on-failure`
Expected: all tests pass, including the pre-existing suite (no regressions from the three new classes).

- [ ] **Step 9: Commit**

```bash
git add bindings/estimation_bindings.cpp bindings/smoke_test.py examples/ex91_phase_locked_loop.cpp examples/CMakeLists.txt compile.bat compile.sh
git commit -m "Wire PhaseLockedLoop into Python bindings, smoke test, and ex91"
```

---

## Notes for the implementer

- All three classes' core math (biquad coefficients, lock dynamics, the disturbance-rejection demo's effectiveness) were verified in a throwaway scratch C++ build before this plan was written — not just derived on paper. Two real bugs were caught this way: a Park-transform sign/state-pairing error in the PLL (documented in Task 5's derivation note) and a measurement-vs-process disturbance modeling error in the ex89 scenario (a sensor-point disturbance cannot be rejected by any controller — it must enter at the plant input to be a meaningful demo of resonant rejection). If a test in this plan fails after a correct transcription of the code above, suspect a transcription error first, then re-derive from the header's documented formula — not the test's expected numbers, which are real measured values, not guesses.
- Update `docs/algorithm_backlog.md`'s "Additional Controller Types" table is *not* part of this plan's scope — that's a docs-tracking decision for whoever closes out the backlog item, likely after this lands.
