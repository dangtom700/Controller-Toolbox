# Design: Fault-Tolerant Control Reconfiguration

**Date:** 2026-06-25
**Status:** Approved, not yet implemented

## Motivation

DT4 closes the loop from fault detection to controller reconfiguration, built directly on
`ControllerStack`'s existing Supervisory mode (health-aware fallback + bumpless transfer already
built in) — driven by a fault classifier instead of a static activation condition.
`lib/ControllerStack.h` and `lib/MismatchDetector.h`/`KalmanFilter::mismatchDetected()`/
`mismatchScore()` were read in full before writing this spec; both exist exactly as the roadmap
assumes, with no corrections needed to the reuse claims. The one genuine design gap the roadmap's
bare sketch leaves open — **how does `classify()` actually tell `SensorBias` apart from
`ActuatorLoss` given only `(innovation, u_cmd, y_meas)`?** — is resolved below; this is a real FDI
(fault detection *and isolation*) problem, not a rubber-stamp of the sketch.

## Scope

This is a **heuristic** classifier over rolling-window statistics, not a rigorous geometric/
structured-residual FDI observer bank (the academically rigorous approach to fault *isolation*,
which needs one dedicated observer per fault direction — far beyond this item's ~300-line
budget). It distinguishes the four `tools/fault_injector.py` fault kinds using two structurally
different signatures that *are* recoverable from a scalar residual stream without a observer
bank:
- **Actuator faults** (loss, stuck) break the causal link between commanded `u` and measured `y`
  — detectable via the correlation between consecutive changes in `u_cmd` and `y_meas`.
- **Sensor faults** (bias, noise) leave that causal link intact but corrupt the residual's own
  distribution — a persistent offset (bias) vs. elevated variance with no offset (noise).

A true ambiguous case (e.g. a fault that mimics both signatures) can still misclassify — this is
inherent to working from 3 scalars instead of a full structured-residual bank, and is documented
as a limitation, not silently assumed away.

## Components

### 1. `lib/FaultClassifier.h` / `.cpp` — standalone, no `IController`/`ControllerStack` dependency

```cpp
enum class FaultType { None, SensorBias, SensorNoise, ActuatorLoss, ActuatorStuck };

struct FaultDetectorParams {
    double residual_threshold = 3.0;    // absolute RMS(innovation) over the window above which a
                                          // fault is suspected at all (same "user must tune to the
                                          // known noise floor" convention as MismatchDetectorParams.sigma)
    int    confirm_window      = 5;      // samples of rolling history (>= 3, since correlation needs
                                          // consecutive differences)
    double corr_threshold      = 0.3;    // |corr(du_cmd, dy_meas)| below this -> actuator suspected
    double bias_threshold      = 2.0;    // |mean(innovation)| / stddev(innovation) above this,
                                          // with the causal link intact -> SensorBias else SensorNoise
    double stuck_du_threshold  = 1e-3;   // stddev(u_cmd) below this, with an actuator fault suspected
                                          // -> ActuatorStuck else ActuatorLoss
};

class FaultClassifier {
public:
    explicit FaultClassifier(const FaultDetectorParams &p = {});
    FaultType classify(double innovation, double u_cmd, double y_meas);   // call once per step
    void reset();
private:
    FaultDetectorParams p_;
    Eigen::VectorXd innovHist_, uHist_, yHist_;  // fixed-size circular buffers, size confirm_window
                                                  // (construction-time allocation only)
    int count_ = 0, head_ = 0;
};
```

**`classify()` per-step algorithm:**
```
1. Push (innovation, u_cmd, y_meas) into the circular buffers (overwrite oldest at head_).
2. If count_ < confirm_window: return FaultType::None (insufficient history).
3. rmsInnov = sqrt(mean(innovHist_.^2))
   If rmsInnov < residual_threshold: return FaultType::None   // nothing unusual
4. meanInnov = mean(innovHist_); sigmaInnov = stddev(innovHist_)  (epsilon-guarded)
5. if stddev(uHist_) < stuck_du_threshold: return ActuatorStuck   // u_cmd itself frozen
6. du = diff(uHist_); dy = diff(yHist_)                 // consecutive first differences
   if stddev(du) > stuck_du_threshold:                  // u_cmd is actually varying...
       if stddev(dy) <= stuck_du_threshold: return ActuatorLoss   // ...but y_meas isn't responding
                                                                    // at all - broken causal link
       corr = PearsonCorr(du, dy)
       if |corr| < corr_threshold: return ActuatorLoss   // causal link looks broken
   // else: u_cmd isn't varying enough to say anything about causality either way - fall through
7. return (|meanInnov| > bias_threshold * sigmaInnov) ? SensorBias : SensorNoise
```
**Correction found during implementation/testing:** the original step 5/6 above computed `corr`
unconditionally and defaulted it to `0.0` whenever `stddev(du)`/`stddev(dy)` fell at or below a
bare `1e-12` div-by-zero floor - which then satisfied `|corr| < corr_threshold` and silently
misclassified a perfectly healthy but momentarily quiescent closed loop (both diffs near zero, no
real correlation evidence either way) as `ActuatorLoss`/`SensorNoise`. The fix above distinguishes
"no information" (`stddev(du)` itself near zero - skip the correlation check, fall through to the
sensor-fault check) from the genuine broken-causal-link signature (`stddev(du)` meaningfully
nonzero but `stddev(dy)` collapses to ~0 - decide `ActuatorLoss` directly, without dividing by a
near-zero `stddev(dy)`). The existing `actuator_loss`/`actuator_stuck` test signatures (constant
`y_meas` against varying/frozen `u_cmd` respectively) still hit the intended branches unchanged.
**Rationale recap (the part the roadmap's sketch left unresolved):** `ActuatorStuck`/`ActuatorLoss`
both break the `u_cmd -> y_meas` correlation (the plant stops responding proportionally to
commands); they're told apart by whether the *controller's own* `u_cmd` has also collapsed to
near-constant (a wound-up integrator pushing against an unresponsive actuator settles near a
rail — low `stddev(uHist_)` — vs. `ActuatorLoss`, where `u_cmd` keeps varying normally since the
controller is still actively, if ineffectively, correcting through an attenuated actuator).
`SensorBias`/`SensorNoise` both preserve the causal link (the controller's commands still
correctly correlate with what the corrupted sensor reports); they're told apart by the classic
mean-shift-vs-variance-inflation distinction on the innovation itself.

### 2. `lib/FTCSupervisor.h` / `.cpp` — implements `IController`, composes a `ControllerStack`

```cpp
class FTCSupervisor : public IController {
public:
    FTCSupervisor(std::shared_ptr<ControllerStack> stack,
                  const FaultDetectorParams &fp, double Ts);

    // FaultType::None registers the nominal/default controller - required exactly like every
    // other fault type, so the supervisor always has a well-defined entry to fall back to.
    // `controllerName` must already have been added to `stack` via addController() with no
    // activationCondition (eligibility is driven entirely by FTCSupervisor's setActive() calls
    // below, not by per-entry conditions).
    void registerFaultResponse(FaultType fault, const std::string &controllerName);

    // Feeds the latest residual triple - call once per step BEFORE compute(), typically right
    // after a KalmanFilter::update() call: feedResidual(kf.innovation(), u_prev, y_meas).
    void feedResidual(double innovation, double u_cmd, double y_meas);

    double compute(double error) override;   // re-activates the registered entry for the
                                                // current fault (if changed since last call),
                                                // then delegates to stack_->compute(error)
    void   reset() override;
    double sampleTime() const override { return Ts_; }
    FaultType currentFault() const { return currentFault_; }

private:
    std::shared_ptr<ControllerStack> stack_;
    FaultClassifier classifier_;
    std::array<std::string, 5> faultResponse_;  // indexed by static_cast<int>(FaultType); empty = unregistered
    FaultType currentFault_  = FaultType::None;
    FaultType lastApplied_   = FaultType::None;  // sentinel distinct from any real value isn't
                                                   // needed: None is itself a valid registered
                                                   // state, so the very first compute() call always
                                                   // re-applies once (harmless, idempotent setActive calls)
    double Ts_;
};
```

**Why `feedResidual()` is a separate call (a small, necessary deviation from the roadmap's bare
`compute(double error)` sketch):** `compute()`'s only input is the tracking error — it has no
access to the model-based innovation, nor to the `u_cmd`/`y_meas` pair the classifier needs.
`SelfTuningRegulator` (this phase's OC1) hit the same gap and resolved it with a `setReference()`
pre-call; `FTCSupervisor` follows the same precedent with `feedResidual()`.

**`compute(double error)`:**
```
1. currentFault_ = classifier_.classify(...)   // already updated by the most recent feedResidual()
2. if currentFault_ != lastApplied_ (or this is the first call):
     for each FaultType f in {None, SensorBias, SensorNoise, ActuatorLoss, ActuatorStuck}:
         if faultResponse_[f] is non-empty:
             stack_->setActive(faultResponse_[f], f == currentFault_)
     lastApplied_ = currentFault_
3. return stack_->compute(error)
```
No `ControllerStack` changes needed — `setActive()` (already public) plus Supervisory mode's
existing "first `active && isHealthy()` entry wins, with automatic `bumplessInit()` on switch"
behavior (`ControllerStack.h:30-46`) does all of the actual reconfiguration and bump-free
switching. `FTCSupervisor` only decides *which single entry* should be `active` at any time —
exactly the "most of the orchestration machinery is pure reuse" claim the roadmap makes, now made
concrete.

**Correction from an earlier draft of this spec:** `ControllerStack::setActive()` (confirmed by
reading `ControllerStack.cpp`) silently no-ops on an unknown name (`if (auto *e = findEntry(name))
e->active = active;` - no `else` branch), it does **not** throw. `registerFaultResponse()`
therefore validates the name against `stack_->entries()` itself and throws
`std::invalid_argument` at *registration* time, rather than relying on (and incorrectly assuming)
an exception from `compute()`'s `setActive()` call.

**Correction found during implementation/testing - step 2 of `compute()`'s pseudocode above was
wrong.** Reconfiguring on *every* `currentFault_ != lastApplied_` transition - including a
transition into a fault type with no registered response - deactivates every registered stack
entry simultaneously (the `for` loop's `f == currentFault_` test is false for all of them), which
silently freezes the stack's output. `ex107_ftc_supervisor`'s redundant-sensor-pair scenario hit
this directly: `FaultClassifier` is a small-sample (`confirm_window=5`) per-step heuristic, and
once the loop is reconfigured onto a different controller its own residual statistics keep
shifting, so it legitimately keeps flickering between `SensorBias` (registered) and
`SensorNoise`/`ActuatorLoss` (not registered in this example). Every flicker into an unregistered
type froze the output; every flicker back re-engaged via `bumplessInit()` against an
already-diverged error - compounding every cycle into a runaway divergence (confirmed via an
instrumented repro: output reached `~1e6` by step 190). Fixed by adding an `actionable` guard -
`compute()` only runs the reconfiguration loop when `faultResponse_[currentFault_]` is
non-empty - so an unregistered/transient classification now leaves the stack's current active
entry untouched instead of clearing it. This also means the example's pass criterion can't check
the literal last-step `currentFault()` (still expected to flicker by design); it checks the
reliably-true property instead - fault detected at least once, active controller latched on the
correct entry, bounded trajectory throughout (see the Testing Plan note below).

## Explicitly out of scope (this phase)

- **A rigorous structured/directional-residual FDI observer bank** — see Scope; the heuristic
  classifier can misclassify genuinely ambiguous faults, documented rather than hidden.
- **Multi-fault (simultaneous) classification** — `classify()` returns one `FaultType` per step;
  simultaneous sensor+actuator faults are not disambiguated.
- **Automatic `ControllerStack` entry creation** — the user must `addController()` every
  fault-response controller themselves before calling `registerFaultResponse()`; `FTCSupervisor`
  never constructs controllers on its own.

## Implementation checklist

**`FaultClassifier`** (standalone utility, lightest checklist — same shape as `MismatchDetector`):
1. `lib/FaultClassifier.h`/`.cpp` + `CTRL_REGISTER_FEATURE(fault_classifier)`
2. `lib/CMakeLists.txt` — add `FaultClassifier.cpp`
3. `lib/ControllerToolbox.h` — `#include "FaultClassifier.h"` near `MismatchDetector.h`

**`FTCSupervisor`** (full `IController` checklist, depends on `FaultClassifier`):
4. `lib/FTCSupervisor.h`/`.cpp` + `CTRL_REGISTER_FEATURE(ftc_supervisor)`
5. `lib/CMakeLists.txt` — add `FTCSupervisor.cpp`
6. `lib/ControllerToolbox.h` — `#include "FTCSupervisor.h"` near `ControllerStack.h`
7. `bindings/controllers_bindings.cpp` — bind `FaultType`, `FaultDetectorParams`,
   `FaultClassifier`, `FTCSupervisor` (`std::shared_ptr<T>` + `ctrl::IController` base)
8. `bindings/smoke_test.py` — build a 2-entry stack, register `None`+`ActuatorLoss`, feed a
   synthetic fault residual sequence, assert `current_fault()` flips as expected
9. `examples/ex107_ftc_supervisor.cpp` + `examples/python/ex124_ftc_supervisor.py` — redundant
   sensor pair scenario per the roadmap's own example use case (injected `sensor_bias`, supervisor
   switches to a controller relying on the healthy sensor)
10. `tests/test_catch2_advanced.cpp` — tests under `[fault_classifier]` and `[ftc_supervisor]`
11. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` — add `ex107_ftc_supervisor`

## Testing plan

**`[fault_classifier]`**
1. Synthetic `sensor_bias` residual stream (persistent offset, causal link intact) — classifies
   as `SensorBias` within `confirm_window` samples.
2. Synthetic `sensor_noise` residual stream (zero-mean, elevated variance, causal link intact) —
   classifies as `SensorNoise`.
3. Synthetic `actuator_loss` stream (attenuated but still-varying `u_cmd`, broken correlation) —
   classifies as `ActuatorLoss`.
4. Synthetic `actuator_stuck` stream (near-constant `u_cmd`, broken correlation) — classifies as
   `ActuatorStuck`.
5. No fault (nominal residual below `residual_threshold`) — classifies as `None` throughout.
6. Insufficient history (`count_ < confirm_window`) — returns `None`, never a false positive on
   the first few calls.

**`[ftc_supervisor]`**
1. Injected `actuator_loss` fault (matching `tools/fault_injector.py`'s taxonomy) — switches to
   the registered fallback controller within `confirm_window` steps of `feedResidual()` calls.
2. No fault — behaves identically to a plain `ControllerStack` in Supervisory mode with one
   always-active entry (regression; confirms the one-time initial `setActive()` pass doesn't
   alter steady-state behavior).
3. Fault clears (residual returns to nominal) — supervisor switches back to the `None`-registered
   controller with no bump in `u` (verified via `ControllerStack`'s existing bumpless-transfer
   guarantee — i.e. this test is really confirming `FTCSupervisor` doesn't bypass it).
4. `registerFaultResponse()` called with a controller name never added to `stack` — throws
   `std::invalid_argument` immediately (see the correction above), surfacing the misconfiguration
   at registration time instead of silently doing nothing at `compute()` time.

**Note on `ex107_ftc_supervisor`/`ex124_ftc_supervisor` (found during implementation/testing):**
the redundant-sensor-pair example's closed loop keeps oscillating after the switch (the example's
PID gains aren't specially tuned), so `FaultClassifier`'s small-sample heuristic keeps flickering
between `SensorBias` and unregistered types step-to-step even after the supervisor has correctly
latched onto the backup controller (see the `FTCSupervisor` correction above). The example
therefore asserts the reliably-true property - `SensorBias` detected at least once, final active
controller is the backup, trajectory stays bounded - not that the literal last-step
`currentFault()` still reads `SensorBias`.
