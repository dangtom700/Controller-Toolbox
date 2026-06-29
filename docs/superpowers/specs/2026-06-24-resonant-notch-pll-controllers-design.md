# Design: Resonant Controller, Notch Filter, and Phase-Locked Loop

**Date:** 2026-06-24
**Status:** Approved, not yet implemented

## Motivation

`docs/algorithm_backlog.md`'s "Additional Controller Types" category lists three open items:
Resonant controllers, Notch filters, and a Phase-Locked Loop (PLL) - picked together because the
backlog groups them as related, and each is a compact, self-contained DSP/controls class with
well-known math and no foundational dependency (unlike the LMI-solver-gated Robust Control items
or the explicitly-deferred Deep RL item in the same backlog).

The three do not share a role, so they do not share a base class:

- **ResonantController** is a genuine feedback controller - it consumes a tracking error and
  produces a correction - so it implements `IController` and composes through the existing
  `ControllerStack` Additive mode, the same way `RepetitiveController` and the `ex47` additive-PID
  pattern already work (`[Ref: CLAUDE.md "The Corrector-Pattern Suite"]`).
- **NotchFilter** has no setpoint or error - it attenuates a frequency band in *any* sampled
  signal - so it is a standalone DSP class with no shared base, like `lib/embedded/FixedRateFilter`
  but as a tunable design tool rather than a generic filter shell.
- **PhaseLockedLoop** produces an estimate (phase/frequency), not a control action, so it follows
  the estimator pattern (`KalmanFilter`/`ParticleFilter`-style: its own `step()`/getter API, no
  shared base - `[Ref: CLAUDE.md "Estimators and plant models share no base"]`).

## Scope

- One target frequency per `ResonantController` instance. Multi-harmonic rejection is achieved by
  adding several instances to an existing `ControllerStack(Additive)` alongside a base controller
  - this reuses existing composition machinery instead of building a second one.
- `NotchFilter` is a single fixed-design biquad (one center frequency, one Q) per instance.
  Multiple notches are multiple instances applied in series (caller's responsibility, same as
  cascading any other filter).
- `PhaseLockedLoop` assumes a single sampled sinusoidal input (no pre-existing quadrature pair).
  It internally manufactures the missing orthogonal component via a Second-Order Generalized
  Integrator (SOGI) - the standard approach when only one channel is available, which is the
  common case for this toolbox's case studies (single sensor/vibration/grid-voltage channels).

## Components

### 1. `lib/ResonantController.h` / `.cpp` - implements `IController`

Non-ideal (finite-Q) resonant transfer function, the form used in grid-tied-inverter / power-
quality literature to avoid the phase singularity of the textbook infinite-gain resonant term:

```
G_RC(s) = 2*Kr*wc*s / (s^2 + 2*wc*s + w0^2)      [w0 = 2*pi*targetFreqHz]
```

Discretized via Tustin with frequency prewarping at `w0`, producing a 2nd-order biquad evaluated
in direct-form-II each `compute()` call.

```cpp
struct ResonantParams {
    double targetFreqHz;       // f0 - the harmonic to reject/track
    double dampingRadPerSec;   // wc - bandwidth/peak-width parameter
    double Kr;                 // resonant gain
    double uMin = -1e9;
    double uMax =  1e9;        // output saturation, mirrors RepetitiveParams
};

class ResonantController : public IController {
public:
    ResonantController(const ResonantParams &p, double Ts);
    double compute(double error) override;     // signConvention() = TrackingErrorRMinusY
    void reset() override;
    double sampleTime() const override { return Ts_; }
    void setParams(const ResonantParams &p);
    const ResonantParams &params() const { return p_; }
};
```

Hold-last NaN guard on non-finite input/output, consistent with the rest of the `compute()` fleet
(`[Ref: CLAUDE.md "NaN contract"]`). Constructor throws `std::invalid_argument` for
`targetFreqHz <= 0`, `dampingRadPerSec <= 0`, or `targetFreqHz` at/above Nyquist (`1/(2*Ts)`).

Usage (multi-harmonic, reusing existing composition):
```cpp
auto stack = std::make_shared<ControllerStack>(StackMode::Additive, Ts);
stack->addController(pid, "PID-base");
stack->addController(resonant_5th, "5th-harmonic-RC");
stack->addController(resonant_7th, "7th-harmonic-RC");
```

### 2. `lib/NotchFilter.h` / `.cpp` - standalone, no shared base

Classic digital biquad notch (Bristow-Johnson "Audio EQ Cookbook" formulation), computed directly
in the digital domain from the warped center frequency - no separate continuous-to-discrete step
needed.

```cpp
struct NotchFilterParams {
    double centerFreqHz;  // f0 to attenuate
    double Q;             // quality factor = f0 / bandwidth; higher Q = narrower notch
};

class NotchFilter {
public:
    NotchFilter(const NotchFilterParams &p, double Ts);
    double apply(double x);     // direct-form-II-transposed biquad
    void reset();
    void setParams(const NotchFilterParams &p);
    const NotchFilterParams &params() const { return p_; }
};
```

Hold-last guard on non-finite input (same defensive posture as the `IController` fleet, applied
by convention even though `NotchFilter` isn't one). Constructor throws `std::invalid_argument` for
`centerFreqHz <= 0`, `Q <= 0`, or `centerFreqHz` at/above Nyquist.

### 3. `lib/PhaseLockedLoop.h` / `.cpp` - standalone estimator, no shared base

Single-input SOGI-PLL (Ciobotaru, Teodorescu & Blaabjerg, "A New Single-Phase PLL Structure Based
on Second Order Generalized Integrator," 2006). Per sample:

1. **SOGI quadrature generator** - from input `v[k]`, produces an in-phase component `v_alpha`
   and a 90-degree-lagging `v_beta`, tuned to the current frequency estimate `w_hat`.
2. **Park transform** using the current phase estimate `theta_hat`: `v_d`/`v_q` from
   `v_alpha`/`v_beta`. `v_q` is the phase-error signal (drives toward 0 when locked).
3. **PI loop filter** on `v_q` produces a frequency correction `delta_w`.
4. **NCO**: `w_hat = w0_nominal + delta_w`; `theta_hat[k+1] = theta_hat[k] + w_hat*Ts`, wrapped to
   `[-pi, pi)`.

```cpp
struct PLLParams {
    double nominalFreqHz;     // expected/center frequency
    double Kp;
    double Ki;                // PI loop-filter gains
    double sogiK = 1.41421356; // SOGI damping gain, default sqrt(2)
};

class PhaseLockedLoop {
public:
    PhaseLockedLoop(const PLLParams &p, double Ts);
    void step(double sample);
    double phase() const;        // theta_hat, rad, wrapped to [-pi, pi)
    double frequencyHz() const;  // current frequency estimate
    double amplitude() const;    // sqrt(v_alpha^2 + v_beta^2) - free byproduct
    bool locked() const;         // heuristic: |v_q| below threshold for N consecutive samples
    void reset();
};
```

Hold-last guard on a non-finite input sample (skip the update, keep prior `theta_hat`/`w_hat`,
same defensive posture as above). Constructor throws `std::invalid_argument` for
`nominalFreqHz <= 0` or `nominalFreqHz` at/above Nyquist.

## Explicitly out of scope (this phase)

- **Built-in multi-harmonic resonant bank** - deliberately rejected in favor of composing several
  single-frequency `ResonantController` instances through `ControllerStack(Additive)`; avoids a
  second composition mechanism alongside the one the project already has.
- **Two-input (pre-existing quadrature) PLL variant** - only the single-input SOGI form is built;
  callers who already have an orthogonal pair can add a simpler PLL later if a real use case
  appears, but speculative now.
- **Three-phase abc/dq PLL** - multi-channel power-systems variant; out of scope for a
  single-channel toolbox addition.
- **Variable-depth/partial notch** (attenuation less than a full null) and **self-tuning/adaptive
  notch center frequency** - `NotchFilter` is a fixed-design full notch; adaptive retuning is a
  separate, materially different feature.
- **General anti-windup wrapper integration for the PLL's internal PI loop filter** -
  `AntiWindupWrapper` only wraps `IController`, and `PhaseLockedLoop` isn't one; the loop filter
  gets a simple internal clamp, not the shared wrapper.

## Implementation checklist

**ResonantController** (full `IController` checklist, per `CONTRIBUTING.md`'s
"adding a new controller" workflow):
1. `lib/ResonantController.h` / `.cpp` + `CTRL_REGISTER_FEATURE(resonant_controller)`
2. `lib/CMakeLists.txt` - add to `CTRL_CORE_SOURCES`
3. `lib/ControllerToolbox.h` - add `#include "ResonantController.h"`
4. `bindings/controllers_bindings.cpp` - bind as `shared_ptr<ResonantController>` + `IController`
   base (required for `ControllerStack.add_controller()` from Python)
5. `bindings/smoke_test.py` - construct, call `compute()`, confirm callable
6. `examples/ex89_resonant_controller.cpp` - demonstrates the multi-harmonic composition
   pattern from the Components section explicitly: a `ControllerStack(StackMode::Additive)`
   holding a base PID plus two `ResonantController` instances (e.g. 5th + 7th harmonic),
   rejecting a multi-harmonic sinusoidal disturbance that the PID alone leaves a steady-state
   ripple on. PASS/FAIL based on residual ripple amplitude after convergence, confirming the
   stack - not a one-off manual summation - is what's doing the composition.
7. `tests/test_catch2_advanced.cpp` - tests under `[resonant_controller]` (see Testing plan)
8. `CONTRIBUTING.md` sign-convention table - add `ResonantController` -> `TrackingErrorRMinusY`

**NotchFilter** and **PhaseLockedLoop** (lighter non-`IController` utility-class checklist, same
shape as `FreqDomainIdentifier`/`SubspaceID`):
1. `lib/NotchFilter.h`/`.cpp`, `lib/PhaseLockedLoop.h`/`.cpp` +
   `CTRL_REGISTER_FEATURE(notch_filter)` / `CTRL_REGISTER_FEATURE(phase_locked_loop)`
2. `lib/CMakeLists.txt` - add both to `CTRL_CORE_SOURCES`
3. `lib/ControllerToolbox.h` - add both `#include`s
4. `bindings/estimation_bindings.cpp` - bind both standalone (alongside `KalmanFilter`/
   `ParticleFilter`)
5. `bindings/smoke_test.py` - construct + one `apply()`/`step()` call each
6. `examples/ex90_notch_filter.cpp` - notch removing a known resonance from a synthetic signal
7. `examples/ex91_phase_locked_loop.cpp` - PLL tracking a synthetic AC signal's phase/frequency
8. `tests/test_catch2_advanced.cpp` - tests under `[notch_filter]` / `[pll]` (see Testing plan)

`compile.bat`/`compile.sh` get the three new example targets added to their hand-maintained lists
(`[Ref: CLAUDE.md "hand-maintained list - keep .bat/.sh in sync"]`).

## Testing plan

**`[resonant_controller]`**
1. Frequency response of the discretized biquad has its gain peak at `targetFreqHz` (evaluate the
   transfer function directly, no closed-loop sim needed for this check).
2. Non-finite input -> hold-last output, internal state unchanged.
3. `reset()` clears state back to construction-time values.
4. Invalid construction params (`targetFreqHz <= 0`, `dampingRadPerSec <= 0`, at/above Nyquist)
   throw `std::invalid_argument`.

**`[notch_filter]`**
1. A sinusoid exactly at `centerFreqHz`, run to steady state -> output amplitude attenuated below
   a fixed threshold relative to input amplitude.
2. A sinusoid well outside the notch band -> output amplitude close to input (within a
   Q-dependent tolerance), confirming the filter doesn't over-attenuate unrelated content.
3. Non-finite input -> hold-last output. `reset()` clears state.
4. Invalid construction params throw `std::invalid_argument`.

**`[pll]`**
1. Synthetic sinusoid `v[k] = A*sin(w0*k*Ts + phi0)` at the nominal frequency, run for enough
   cycles -> `phase()`/`frequencyHz()` converge close to the true values and `locked()` becomes
   true.
2. A step change in the true input frequency mid-run -> the PLL re-converges (tests the loop
   filter's tracking dynamics, not just steady-state lock).
3. A non-finite input sample -> estimate holds at its last value, state not corrupted; PLL
   resumes tracking once finite samples resume.
4. `reset()` reinitializes phase/frequency state. Invalid construction params throw
   `std::invalid_argument`.
