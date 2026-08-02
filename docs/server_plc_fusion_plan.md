# Server/PLC Master-Slave Controller Fusions - Implementation Plan

Authored 2026-07-25. Codebase state: 51 `IController` subclasses, 131 C++ examples
(highest `ex130_fuzzy_smc`), 22 complete case studies.

**Status: Stages 0, 1a, 1b, 2, 3 and 5 complete - all four Tier 1 fusions plus Tier 2.1 are
built and passing, and `NetworkChannel` is bound into Python. Only Stage 4 (full `run.py` pass
and doc/count reconciliation) remains open.** See the Staging table below.

This plan takes a proposal for four "Tier 1" server/PLC controller fusions - designs meant to be
immediately implementable by wiring existing `lib/` components together - and turns it into
buildable work. The proposal's framing was mostly right, but it rested on several wrong premises
about what this repo actually contains, one of which was load-bearing.

---

## 1. Inventory corrections

**This is the most important section of the document.** The original proposal's component
inventory contained five errors. They are recorded here so the same wrong premises are not
rebuilt later. All claims below were verified directly against `lib/`.

| Proposal claim | Reality |
|---|---|
| `LockFreeParameterBuffer`, "already production-ready" | **Does not exist under that name.** It is `ctrl::AtomicParamBuffer` ([lib/AtomicParamBuffer.h](../lib/AtomicParamBuffer.h)), a header-only template. It *is* production-ready, but it has **no example, no Python binding, and is deliberately commented out of the umbrella header** ([lib/ControllerToolbox.h:77](../lib/ControllerToolbox.h#L77), grouped with `hal/HAL.h` as opt-in). `CLAUDE.md` section 8 already warns that `LockFreeParameterBuffer` is a template fiction. |
| `ComputationalDelayWrapper` "models the variable delay between sensor read and actuator write" | **It is a fixed ONE-sample delay** ([lib/ComputationalDelayWrapper.h:40](../lib/ComputationalDelayWrapper.h#L40)). It cannot express jitter at all. This was the load-bearing error - proposal design 1.1 depended on it entirely. |
| `Cascade` / `Supervisory` are corrector *patterns*, not classes | Half right. `Supervisory` is still a `StackMode`, but **`CascadeController` is now a real `IController`** ([lib/CascadeController.h:63](../lib/CascadeController.h#L63)) with a `CascadeParams::outerDecimation` field - so proposal design 1.2's dual-rate cascade is *already built*. |
| "46 `IController` subclasses" | **51.** Missing from the proposal's table: `AdaptiveSMC`, `SuperTwistingSMC`, `NonsingularTerminalSMC`, `FuzzySlidingModeController`, `DisturbanceObserverController`, `TwoDOFController`, `LearningFeedforwardController`, `LPMPC`. |
| `StackMode::Weighted`, `LoopShapingTuner`, `DAESystem`, `GPResidualModel` exist | **Confirmed correct** - all four are present. |

### The actual gap

There is **no network or transport abstraction anywhere** in `lib/` or `lib/hal/` - no latency,
no jitter, no packet loss, no out-of-order delivery, and no master/slave harness. `lib/hal/`
covers sensors, actuators, timers and RTOS schedulers, but stops at the device boundary.

All four Tier 1 designs depend on that missing piece. It is therefore the one genuinely new
component this plan introduces; everything else is composition of existing parts.

---

## 2. Scope

| Decision | Choice |
|---|---|
| Which designs | All four Tier 1 server/PLC fusions |
| Where the demos live | `examples/` (`ex131`-`ex134`), matching the "integration exercise" framing |
| Network channel | A **full `lib/` component** with bindings, Catch2 coverage and smoke test |

Explicitly **out of scope**: all Tier 2 and Tier 3 designs (PLC-adaptive retuning, distributed
MPC, sensor-fusion voting, Bayesian self-optimisation, hierarchical tube MPC, predictive
maintenance); any real networking (this is a deterministic *simulation* of a link - no sockets,
no fieldbus driver, no server process); any change to an existing `lib/` controller.

---

## 3. Staging

| Stage | Contents | Status |
|---|---|---|
| **0** | This document + `docs/index.md` entry | **Done** |
| **1a** | `lib/NetworkChannel.h` + umbrella include + 12 `[network]` Catch2 cases | **Done** |
| **1b** | pybind11 binding + `smoke_test.py` + `docs/deployment.md` section | **Done** |
| **2** | `ex131` jitter-MPC, `ex132` dual-rate cascade + registration | **Done** |
| **3** | `ex133` event-triggered estimation, `ex134` bumpless redundancy + registration | **Done** |
| 4 | Full `run.py` green pass, doc/count reconciliation | Open |
| **5** | `ex135` PLC-adaptive online retuning (Tier 2.1) | **Done** |

### Measured results (Stages 1a + 2 + 3)

| Check | Result |
|---|---|
| `[network]` Catch2 guards | 12/12, 504 assertions |
| `test_catch2_advanced` total | 436/436, 10,502 assertions (was 424 cases) |
| `ex131` jitter-compensated MPC | **PASS** - IAE 12.888 -> 0.335, a 97.4% improvement, paired trace verified |
| `ex132` dual-rate cascade + supervisor | **PASS** - fault detected 36 ms after injection, 0 false alarms, final \|e\| 0.0007 vs 0.0449 degraded |
| `ex133` event-triggered estimation | **PASS** - uplink traffic -77.5% (2000 -> 449 packets), pre-fault RMS estimate error 0.019 vs 0.023 periodic, sensor fault caught 30 ms after injection |
| `ex134` bumpless redundancy | **PASS** - IAE 0.582 vs 6.680 wedged (11x), 0.824 command gap spread to 0.180 max per tick, 10 supervisor publishes |
| `ex135` adaptive online retuning | **PASS** - post-drift IAE 2.367 -> 0.572 (-75.8%), 2 detector-triggered sessions, identified worn model exact to 4 dp |
| `check_build_target_drift.py` | `compile.bat` / `compile.sh` agree on 191 targets |

Stage 1a deliberately excludes the bindings: those need a
`-DCTRL_BUILD_PYTHON_BINDINGS=ON` rebuild, a separate and slower build path. Keeping them in 1b
lets Stage 1a be verified with a single fast Release build.

### Stage 1b as built

`ctrl.NetworkChannel` / `ctrl.NetworkChannelParams` in
[bindings/plantmodel_bindings.cpp](../bindings/plantmodel_bindings.cpp) (beside `ControllerMonitor`,
its closest analogue), a `NetworkChannel` subsection in
[docs/deployment.md](deployment.md), and a smoke-test block covering the fixed-latency trace,
seed determinism under jitter + loss, latest-wins supersession, the NaN contract and parameter
sanitisation. `ctrl.features()` reports `network_channel`. The binding is scalar-payload
(`NetworkChannel<double>`) at the default 64-slot capacity; `try_receive(t_now)` returns the
payload or `None`, since the C++ out-parameter form does not cross the pybind11 boundary.

**One gotcha, found by the smoke test failing:** do not assert exact arrival ticks. A packet sent
at `t` with latency exactly `n*Ts` lands precisely on `tryReceive`'s `t_deliver > t_now`
comparison, and `(t + latency) <= (t + n*Ts)` is a coin flip in binary floating point - so some
packets slip a tick, arrive two-at-a-time, and the older one is correctly discarded as
superseded. The first version of the test asserted 95 deliveries and got 87. The channel was
right; the test was measuring the FPU. Use a latency that is deliberately *not* a whole number of
ticks (the block uses 45 ms against a 10 ms tick) so the comparison is never on the boundary.

---

## 4. Stage 1 - `lib/NetworkChannel.h`

**Header-only**, following the precedent of `AtomicParamBuffer.h`, `MismatchDetector.h` and
`ControllerMonitor.h` (all header-only, no `.cpp`). This matters practically:
`lib/CMakeLists.txt` installs headers by glob (`FILES_MATCHING PATTERN "*.h"`), so **no
`CTRL_CORE_SOURCES` edit is required** - a header-only component is materially cheaper to add
than the "full lib component" label suggests.

```cpp
namespace ctrl {

struct NetworkChannelParams {
    double   latency_mean  = 0.010;  ///< mean one-way latency [s]
    double   jitter_sigma  = 0.002;  ///< std dev of latency [s]; truncated at >= 0
    double   loss_prob     = 0.0;    ///< per-packet drop probability [0,1]
    bool     allow_reorder = false;  ///< permit out-of-order delivery
    unsigned seed          = 42u;    ///< deterministic: same seed => same trace
};

template <typename T, int Capacity = 64>
class NetworkChannel {
public:
    explicit NetworkChannel(const NetworkChannelParams& p);
    void send(const T& value, double t_now);   ///< schedule delivery at t_now + latency
    bool tryReceive(T& out, double t_now);     ///< newest packet due by t_now; false if none
    void reset();
    unsigned sent() const;  unsigned delivered() const;
    unsigned dropped() const;  unsigned reordered() const;
    double   lastLatency() const;
};

}  // namespace ctrl
CTRL_REGISTER_FEATURE(network_channel)
```

### Design rules

Matching the repo's RT contract (`docs/deployment.md`, Zero-Allocation Checklist):

- **Zero allocation.** Fixed `std::array<Packet, Capacity>` ring. Overflow drops the oldest slot
  and increments `dropped()`. No `push_back`, no `deque`, no streams in any path.
- **Deterministic.** Owns its `std::mt19937`, seeded from `params.seed`, so a run reproduces
  exactly. Required for the Catch2 guards and for `run.py` stability - a channel seeded from
  `random_device` would make every example flaky.
- **Latest-wins.** `tryReceive()` pops every packet due by `t_now` and returns the one with the
  **highest sequence number**, discarding stale ones. This is the correct semantic for control
  telemetry (an old setpoint is worthless once a newer one has arrived) and it is what makes
  `allow_reorder` safe to enable.
- **NaN-safe.** A non-finite payload or `t_now` is rejected at `send()` and counted as dropped,
  never propagated. `run.py` Phase 2 scans for this contract.
- **Causality.** Sampled latency is truncated at `>= 0`, so jitter can never deliver a packet
  before it was sent.

### Registration

1. `lib/ControllerToolbox.h` - add `#include "NetworkChannel.h"`, **uncommented**. Unlike
   `AtomicParamBuffer.h` and `hal/HAL.h`, which are opt-in because they pull in threading and
   hardware concerns, `NetworkChannel` is a pure simulation utility depending only on `<random>`,
   `<array>` and `<cmath>`. Including it in the umbrella is also what makes
   `CTRL_REGISTER_FEATURE` fire for plain C++ consumers.
2. `bindings/plantmodel_bindings.cpp` - bind `NetworkChannel<double>` as `ctrl.NetworkChannel`
   plus `ctrl.NetworkChannelParams`. That file already hosts `ControllerMonitor`, the closest
   analogue. Methods in **snake_case** per `CLAUDE.md` section 6. `try_receive` cannot use an
   out-parameter across the boundary - return `std::optional<double>` instead. Not an
   `IController`, so no `shared_ptr` holder and no base-class argument.
3. `tests/test_catch2_advanced.cpp` - `TEST_CASE`s under tag `[network]`.
4. `bindings/smoke_test.py` - a short assert block in the file's existing plain-`assert` style.
5. `docs/deployment.md` - a `NetworkChannel` subsection beside the existing
   `AtomicParamBuffer - Lock-Free Parameter Updates` section.

`CONTRIBUTING.md` needs **no** sign-convention row - `NetworkChannel` is not an `IController`.

### Catch2 guards (tag `[network]`)

- **Determinism** - same seed gives an identical delivery trace; different seeds diverge.
- **Latency bounds** - every delivered packet has latency `>= 0`; with `jitter_sigma = 0` the
  latency equals `latency_mean` exactly.
- **Ordering** - with `allow_reorder = false` sequence numbers arrive monotonically; with it
  enabled, `tryReceive` still returns the newest available packet and `reordered()` increments.
- **Loss accounting** - `sent() == delivered() + dropped()` for every configuration.
- **Capacity** - sending far more than `Capacity` without receiving drops the excess and never
  corrupts the ring.
- **Starvation** - `tryReceive` returns `false` before the first delivery time.
- **NaN contract** - non-finite payload or `t_now` is rejected and counted, never delivered.

---

## 5. Stages 2-3 - the four fusion demos

Numbering continues from `ex130_fuzzy_smc`. Each demo is a single self-contained `.cpp` that
prints a metrics block and ends `PASS`/`FAIL` with exit code 0/1, exactly like
`examples/ex126_cascade_controller.cpp`. No shared harness header - this repo's examples are
deliberately single-file.

Common master/slave shape:

```
slave (PLC)  : plant + fast inner loop, 1 ms tick
   |  sensor packets -> NetworkChannel (latency + jitter + loss)
master (srv) : outer controller, 10-30 ms tick
   |  actuator command -> NetworkChannel
slave (PLC)  : applies newest command; holds last value on starvation
```

**Every demo's PASS criterion is a *paired* comparison** against an uncompensated baseline over
the *same seeded channel trace*. Without pairing, a green result only proves the scenario was
easy, not that the fusion helped.

### ex131 - Network-Jitter-Compensated MPC

`DiscreteMPC` + `NetworkChannel` + `SmithPredictor` + `ComputationalDelayWrapper`

The proposal assigned jitter modelling to `ComputationalDelayWrapper`, which cannot do it.
Corrected split: `NetworkChannel` supplies the variable round-trip transport delay,
`SmithPredictor` compensates the *nominal* dead time so the master predicts plant state at the
moment the command will actually land, and `ComputationalDelayWrapper` keeps its honest role -
the one-tick compute delay of the master itself, applied identically to both arms.

**Deviation from the original plan: the inner predictive controller is `DiscreteMPC`, not
`NonlinearMPC`.** `SmithPredictor` drives its inner controller purely through
`compute(error)`, whereas `NonlinearMPC` additionally requires `setState()` before every call
(`CONTRIBUTING.md` sign-convention table) - the two do not compose. `DiscreteMPC` carries its own
internal state estimate across `compute()` calls and does.

*Guard:* compensated IAE materially lower than the uncompensated arm on the same seed, plus an
explicit paired-trace assertion that both arms saw identical uplink delivered/dropped counts.
*Measured:* IAE `12.888 -> 0.335` (97.4% better); the uncompensated arm rings to `max |y| = 2.58`
against a setpoint of 1.0.

### ex132 - Dual-Rate Cascade with Supervisor

`CascadeController` + `ControllerStack` (Supervisory) + `ControllerMonitor`

Mostly wiring: `CascadeController` already provides multi-rate via `CascadeParams::outerDecimation`.
Inner `DiscretePID` at the 1 ms slave tick, outer controller every 10th tick. `ControllerMonitor`
is an `IControllerObserver` - attach it to the inner loop and let its CUSUM/EWMA charts flag
degradation; a Supervisory `ControllerStack` falls back to a single-loop PI whose
`activationCondition` fires on that alarm.

*Guard:* fault detected, **zero false alarms before the fault**, fallback engages after the
fault, axis stays bounded, and the recovered steady-state error beats the degraded cascade.

**Three corrections this demo forced** (each broke a revision before being fixed):

1. **The monitored signal must be stationary.** `ControllerMonitor::onCompute()` feeds the
   controller *output* to its charts. On a servo axis the steady command is
   `u = (c*v + load)/k_act`, so it tracks the velocity setpoint: at `k_act = 12`, `v = 2` the
   healthy command is `1.25` - **the same value as the faulted at-rest command**. A chart
   centred on the at-rest value therefore alarms on every motion. Fixed by making the scenario
   station keeping, so `v ~ 0` and `u` is genuinely stationary.
2. **The fault needs a steady-state signature.** With no load the steady command is zero both
   before and after an actuator-gain collapse, so no chart on `u` could ever see it. A constant
   load makes the signature `0.25 -> 1.25`.
3. **SPC must go live after commissioning.** The startup transient saturates the inner command
   at `uMax`; charting it produced 25-26 false alarms at `t ~ 0.02 s` and engaged the fallback
   before the fault existed. The monitor is attached only after the axis settles.

Also note `ControllerMonitor::nAlarms()` only increments when an alarm callback is registered
(see `ControllerMonitor::feed`) - `setAlarmCallback()` is not optional if you intend to count.

### ex133 - Event-Triggered Estimation

`KalmanFilter` + `EventTriggeredWrapper` + `MismatchDetector` + `NetworkChannel`

**Semantics correction:** `EventTriggeredWrapper` gates *controller execution* on a deadband
(`EventTriggeredParams::sigma`); it does not gate telemetry, and it does not sit between sensor
I/O and a filter as the proposal assumed. The demo uses it for its real purpose on the control
side and implements the send-on-delta *measurement* gate explicitly on the slave, using the same
deadband semantics. The master's KF runs predict-only between arrivals;
`MismatchDetector::update(innov)` / `detected()` flags a sensor fault.

*Guard:* transmitted-packet count falls substantially versus periodic transmission while RMS
estimation error stays inside a stated bound; the detector fires on an injected sensor bias and
stays quiet during nominal operation.
*Measured:* uplink `2000 -> 449` packets (-77.5%), pre-fault RMS estimate error `0.019` (event)
vs `0.023` (periodic), fault flagged 30 ms after injection with a pre-fault peak score of 0.03.

**Two corrections this demo forced:**

1. **The fault must be abrupt, not a drift.** The first revision ramped a sensor bias at
   0.25 m/s and the detector never fired. A Kalman filter *tracks* slow drift: the per-tick bias
   increment is 0.0025 m, four times smaller than the 0.01 m noise it hides in, so the innovation
   never leaves its null distribution. Innovation-CUSUM catches abrupt faults; slow parametric
   drift is a job for `RecursiveGreyBoxEstimator`. Changed to a 0.35 m step.
2. **Estimation quality must be measured pre-fault.** Averaging estimate error over the whole run
   let the post-fault period (where the filter is being fed a lie) dominate the RMS at ~0.46 m,
   swamping the thing actually under test - whether event-triggering degrades the estimate.

**Deviation from the plan:** no standalone `ctrl::MismatchDetector` is constructed.
`ctrl::KalmanFilter` already embeds one - `enableMismatchDetection()` / `mismatchDetected()` /
`mismatchScore()` - and feeds it the innovation on every `update()`. Hand-feeding a separate
detector would duplicate that.

### ex134 - Bumpless Redundant Transfer

`ControllerStack` (Weighted) + `AtomicParamBuffer` + `NetworkChannel`

Primary and standby controllers both run every tick; only the blended output is applied.
`ControllerStack::setWeight(name, w)` ramps primary 1 -> 0 and standby 0 -> 1 over ~100 ms on
failover. A background supervisor publishes the failover command through
`AtomicParamBuffer<FailoverCmd>` - a trivially-copyable POD, which the template's `static_assert`
enforces - and the RT loop reads it with `read()`.

**This is the first example in the repo to exercise `AtomicParamBuffer` at all.** It must
`#include "AtomicParamBuffer.h"` **directly**: the umbrella header does not pull it in.

*Guard:* the handover gap must be non-trivial, the worst single-tick step must be a small
fraction of it, the standby must recover the setpoint, and the failover must beat doing nothing
by a real margin (25%, not noise).
*Measured:* IAE `0.582` vs `6.680` wedged (11x better), an `0.824` command gap spread to a
maximum `0.180` per tick, 10 supervisor publishes.

**The correction this demo forced: the primary must wedge MID-TRANSIENT.** The first revision
hung it at steady state, where the frozen command is still the *correct* command - so failing
over gained nothing (IAE 0.3233 vs 0.3234, a 0.03% difference that is pure noise) and the
measured bump was exactly 0.00000 because both controllers, fed identical inputs, produced
identical outputs. There was literally nothing to transfer. Stepping the setpoint 50 ms before
the hang makes the frozen command wrong and lets the live standby's command diverge from it,
which is what makes both the benefit and the bumplessness measurable.

Note also that "bumpless" is asserted **relative to what a hard switch would have delivered**
(the total command change across the handover), not against an absolute constant - an absolute
threshold passes trivially whenever the gap happens to be small.

### Registration

- `examples/CMakeLists.txt` - one `add_example(...)` line per demo; the file defines that helper
  and needs no other boilerplate.
- `compile.bat` **and** `compile.sh` - matching entries after `ex130_fuzzy_smc`. Both must stay in
  sync; `tools/check_build_target_drift.py` enforces it.

---

## 5b. Stage 5 - ex135, PLC-adaptive online retuning

`AutoTuner` (CMA-ES) + `RecursiveLeastSquares` + `MismatchDetector` + `AtomicParamBuffer`
+ `NetworkChannel`

This is Tier 2.1 from the original proposal, unblocked by `NetworkChannel` and by the
`AtomicParamBuffer` pattern proven in `ex134`.

```
PLC (slave)  : plant whose dynamics DRIFT mid-run; PID at the fast tick.
               Reads its gains from AtomicParamBuffer every tick - O(1), never stalls.
   |  telemetry (y, u) -> NetworkChannel
srv (master) : RecursiveLeastSquares identifies the live plant from (y, u).
               MismatchDetector watches the RLS residual.
               On a sustained alarm, AutoTuner::tune() runs a CMA-ES session against the
               freshly identified model and publishes the new gains.
```

The point is the *separation of timescales*: identification and CMA-ES optimisation are
expensive and run on the master's own schedule, while the real-time loop only ever does a
lock-free `read()`. That is exactly what `AtomicParamBuffer` exists for - and `PIDParams` is
all-doubles, so it satisfies the template's trivially-copyable `static_assert` directly.

*Guard:* paired against a fixed-gain arm over the identical drift and disturbance sequence.
The retuned arm must (a) actually retune, (b) end with materially different gains, (c) beat the
fixed arm on post-drift IAE by a real margin, and (d) the tuning session must be triggered by
the detector rather than scheduled at a hard-coded time - otherwise the demo is a timer, not an
adaptive controller.

*Measured:* post-drift IAE `2.3671 -> 0.5723` (**-75.8%**), 2 tuning sessions, first triggered at
`t = 34.12 s` during the drift ramp. Gains `Kp 1.200 -> 6.918`, `Ki 1.800 -> 12.000`. The
identified worn model matches the truth to four decimals: `a_d = 0.9860, b_d = 0.0199` against a
true `0.9860, 0.0200`.

**Two corrections this demo forced:**

1. **An adaptive estimator's residual cannot detect the drift it is absorbing.** The first
   revision fed `MismatchDetector` the residual of the tracking RLS. At `lambda = 0.995` the
   effective memory is `1/(1-lambda) = 200` samples = 4 s, so RLS simply tracked the 6 s drift
   ramp and the residual never left the noise floor - zero retunes, and the two arms came out
   byte-identical. Fixed with a **two-model split**: an adaptive RLS tracks the live plant and
   supplies the model `AutoTuner` optimises against, while a **frozen commissioning snapshot**
   scores the live data, and it is *that* residual the detector watches. This is the same failure
   class as `ex133`'s undetectable sensor drift.
2. **Re-baseline after a successful retune.** Without it the frozen commissioning model stays
   permanently wrong once the plant has drifted, so the detector alarms forever and the loop
   retunes on every cooldown expiry - 7 sessions instead of the 2 the fault warrants. Accepting
   the adapted plant as the new reference closes the detect -> adapt -> settle cycle.

Note the `Ki` upper bound is **binding** at the optimum (it lands exactly on 12.0). That is
expected, not an artefact: the worn plant is slow with high DC gain, so the cost function
genuinely wants large integral action. The bound is a safety limit on how aggressive an
unattended tuner may become.

---

## 6. Verification

```bash
# Stage 1a
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target test_catch2_advanced
./build/tests/test_catch2_advanced.exe "[network]"   # all green
./build/tests/test_catch2_advanced.exe               # no regression elsewhere

# Stages 2-3
cmake --build build --target ex131_plc_jitter_mpc ex132_plc_dual_rate_cascade \
                            ex133_plc_event_triggered_estimation ex134_plc_bumpless_redundancy
for e in ex131_plc_jitter_mpc ex132_plc_dual_rate_cascade \
         ex133_plc_event_triggered_estimation ex134_plc_bumpless_redundancy; do
  ./build/examples/$e.exe; echo "$e -> exit $?"
done

# Stage 1b
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTRL_BUILD_PYTHON_BINDINGS=ON
cmake --build build --target ctrl_toolbox
conda run -n soft_robotics -- python bindings/smoke_test.py
conda run -n soft_robotics -- python -c "import ctrl_toolbox as c; assert 'network_channel' in c.features()"

# Stage 4
conda run -n soft_robotics -- python tools/check_build_target_drift.py
conda run -n soft_robotics -- python run.py
```

**Acceptance:**

- `[network]` cases green; `test_catch2_advanced` `TEST_CASE` count rises from 424.
- Four new `.exe` files print `PASS`; `run.py`'s C++ execution phase count rises by 4.
- `ctrl.features()` contains `network_channel`.
- `check_build_target_drift.py` reports `compile.bat` and `compile.sh` agree.
- Phase 1 (ASCII) and Phase 2 (NaN-guard) scans stay clean.

## Build note

The canonical build is **Release** (`CLAUDE.md` section 2). `lib/DiscreteHinf.cpp` fails to
compile in Debug/`-O0` on MinGW with "too many sections"; build Release, or add `-Wa,-mbig-obj`
for that translation unit.
