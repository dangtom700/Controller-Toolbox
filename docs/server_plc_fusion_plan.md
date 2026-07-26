# Server/PLC Master-Slave Controller Fusions - Implementation Plan

Authored 2026-07-25. Codebase state: 51 `IController` subclasses, 131 C++ examples
(highest `ex130_fuzzy_smc`), 22 complete case studies.

**Status: Stage 0 + Stage 1a complete. Stages 1b-4 open.** See the Staging table below.

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
| **1a** | `lib/NetworkChannel.h` + umbrella include + `[network]` Catch2 cases | **Done** |
| 1b | pybind11 binding + `smoke_test.py` + `docs/deployment.md` section | Open |
| 2 | `ex131` jitter-MPC, `ex132` dual-rate cascade + registration | Open |
| 3 | `ex133` event-triggered estimation, `ex134` bumpless redundancy + registration | Open |
| 4 | Full `run.py` green pass, doc/count reconciliation | Open |

Stage 1a deliberately excludes the bindings: those need a
`-DCTRL_BUILD_PYTHON_BINDINGS=ON` rebuild, a separate and slower build path. Keeping them in 1b
lets Stage 1a be verified with a single fast Release build.

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

`NonlinearMPC` + `NetworkChannel` + `SmithPredictor` + `ComputationalDelayWrapper`

The proposal assigned jitter modelling to `ComputationalDelayWrapper`, which cannot do it.
Corrected split: `NetworkChannel` supplies the variable round-trip transport delay,
`SmithPredictor` compensates the *nominal* dead time so the master predicts plant state at the
moment the command will actually land, and `ComputationalDelayWrapper` keeps its honest role -
the one-tick compute delay of the master itself.

*Guard:* compensated IAE materially lower than the uncompensated arm on the same seed.

### ex132 - Dual-Rate Cascade with Supervisor

`CascadeController` + `ControllerStack` (Supervisory) + `ControllerMonitor`

Mostly wiring: `CascadeController` already provides multi-rate via `CascadeParams::outerDecimation`.
Inner `DiscretePID` at the 1 ms slave tick, outer controller every 10th tick. `ControllerMonitor`
is an `IControllerObserver` - attach it to the inner loop and let its CUSUM/EWMA charts flag
degradation; a Supervisory `ControllerStack` falls back to a detuned PI whose
`activationCondition` fires on that alarm.

*Guard:* fallback engages under an injected inner-loop fault, transfer is bumpless
(`bumplessInit()` on switch), loop stays bounded throughout.

### ex133 - Event-Triggered Estimation

`KalmanFilter` + `EventTriggeredWrapper` + `MismatchDetector` + `NetworkChannel`

**Semantics correction:** `EventTriggeredWrapper` gates *controller execution* on a deadband
(`EventTriggeredParams::sigma`); it does not gate telemetry, and it does not sit between sensor
I/O and a filter as the proposal assumed. The demo uses it for its real purpose on the control
side and implements the send-on-delta *measurement* gate explicitly on the slave, using the same
deadband semantics. The master's KF runs predict-only between arrivals;
`MismatchDetector::update(innov)` / `detected()` flags a sensor fault.

*Guard:* transmitted-packet count falls substantially versus periodic transmission while RMS
estimation error stays inside a stated bound; `MismatchDetector` fires on an injected sensor bias
and stays quiet during nominal operation.

### ex134 - Bumpless Redundant Transfer

`ControllerStack` (Weighted) + `AtomicParamBuffer` + `NetworkChannel`

Primary and standby controllers both run every tick; only the blended output is applied.
`ControllerStack::setWeight(name, w)` ramps primary 1 -> 0 and standby 0 -> 1 over ~100 ms on
failover. A background supervisor publishes the failover command through
`AtomicParamBuffer<FailoverCmd>` - a trivially-copyable POD, which the template's `static_assert`
enforces - and the RT loop reads it with `read()`.

**This is the first example in the repo to exercise `AtomicParamBuffer` at all.** It must
`#include "AtomicParamBuffer.h"` **directly**: the umbrella header does not pull it in.

*Guard:* no output discontinuity beyond a stated bump threshold across the transfer; tracking
error stays bounded while the primary is dead.

### Registration

- `examples/CMakeLists.txt` - four `add_example(...)` lines; the file defines that helper and
  needs no other boilerplate.
- `compile.bat` **and** `compile.sh` - four entries after `ex130_fuzzy_smc`. Both must stay in
  sync; `tools/check_build_target_drift.py` enforces it.

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
