# Embedded and Real-Time Control

### BasicPID\<Scalar\>, BasicSMC\<Scalar\>, and ComputationalDelayWrapper

---

## Introduction

The main toolbox algorithms - `DiscretePID`, `DiscreteSMC`, `DiscreteMPC` - depend on Eigen 3.4, virtual dispatch, and `std::function`. These are appropriate for desktop simulation and Linux/Windows embedded Linux targets with substantial memory. They are too heavy for microcontroller targets with 32-bit single-precision FPUs, no heap allocator, and cycle-budget constraints of a few hundred nanoseconds per step.

Two header-only templates address this gap:

| Class | Header | Replaces | Eigen? | Virtual? | `float` support? |
|---|---|---|---|---|---|
| `BasicPID<Scalar>` | `lib/BasicPID.h` | `DiscretePID` | No | No | Yes |
| `BasicSMC<Scalar>` | `lib/BasicSMC.h` | `DiscreteSMC` | No | No | Yes |

A third class, `ComputationalDelayWrapper`, wraps any `IController` to simulate the one-sample actuator delay inherent in real digital control loops - essential for accurate controller-in-the-loop simulation on desktop.

---

## BasicPID\<Scalar\>

### Purpose

A header-only, template PID controller in the ISA parallel form with anti-windup and derivative filtering. Designed for embedded targets (ARM Cortex-M, ESP32, RISC-V) where:
- Only the FPU is available (single-precision `float`).
- No dynamic memory allocation is allowed.
- `std::function`, virtual calls, and Eigen matrix operations must be avoided.
- The controller must fit in a few hundred bytes of stack.

### API

```cpp
#include "BasicPID.h"

// Instantiate for float (embedded) or double (desktop validation)
ctrl::BasicPID<float> pid(Kp, Ki, Kd, Ts,
                          u_min, u_max,    // output clamps
                          N_filter,        // derivative filter coefficient (0 = no filter)
                          Kb);             // anti-windup back-calculation gain

// Per-step call - error = r - y (same sign convention as DiscretePID)
float u = pid.compute(error);

// State inspection
float I_state = pid.integrator();   // current integrator accumulator
void  pid.reset();                  // zero integrator and derivative memory
```

### Template parameter

`Scalar` can be `float` or `double`. Use `float` on MCU targets for FPU alignment; use `double` for desktop validation so results match `DiscretePID`.

### API equivalence with DiscretePID

| `DiscretePID` parameter | `BasicPID<Scalar>` parameter |
|---|---|
| `Kp, Ki, Kd` | `Kp, Ki, Kd` |
| `Ts` | `Ts` |
| `uMin, uMax` | `u_min, u_max` |
| `N` (derivative filter) | `N_filter` |
| `Kb` (back-calculation) | `Kb` |
| `compute(r - y)` | `compute(error)` where `error = r - y` |

### Features NOT available in BasicPID

- 2-DOF setpoint weighting (`b_weight`): pre-compute `error = b_weight*r - y` externally.
- Derivative-on-measurement mode: compute `de = -(y - y_prev)/Ts` externally and pass `Kd*de/Ts` as a feedforward addition.
- Observer interface (`attachObserver`, `notifyObserver`): not present - header-only, no virtual dispatch.

### Minimal embedded C++ example

```cpp
// On a Cortex-M4 (single precision FPU)
#include "BasicPID.h"

static ctrl::BasicPID<float> pid(2.0f, 0.5f, 0.1f, 0.01f,
                                  -100.0f, 100.0f, 5.0f, 0.1f);

extern "C" void TIM1_UP_IRQHandler(void) {
    float y = read_sensor();
    float r = get_setpoint();
    float u = pid.compute(r - y);
    write_actuator(u);
}
```

### Desktop validation pattern

```cpp
// Validate BasicPID against DiscretePID on the same signal
ctrl::BasicPID<double>  basic(Kp, Ki, Kd, Ts, u_min, u_max, N, Kb);
ctrl::DiscretePID       ref(Kp, Ki, Kd, Ts, u_min, u_max, N, Kb);

for (int k = 0; k < N; ++k) {
    double u_basic = basic.compute(e[k]);
    double u_ref   = ref.compute(e[k]);
    assert(std::abs(u_basic - u_ref) < 1e-10);  // should match to machine epsilon
}
```

---

## BasicSMC\<Scalar\>

### Purpose

A header-only, template first-order sliding mode controller with saturation boundary layer. The sliding surface is:
$$s(k) = c_e \cdot e(k) + c_{de} \cdot (e(k) - e(k-1)) / T_s$$
and the control law is:
$$u(k) = K_s \cdot \text{sat}(s(k) / \phi)$$
where $\phi$ is the boundary layer width that replaces the discontinuous sign function.

### API

```cpp
#include "BasicSMC.h"

ctrl::BasicSMC<float> smc(c_e, c_de, Ks, phi, Ts, u_min, u_max);

// error = r - y  (same sign convention as DiscreteSMC which uses y - ref;
// BasicSMC uses r - y to match BasicPID sign convention)
float u = smc.compute(error);

// State
float s = smc.slidingSurface(error);  // inspect surface BEFORE calling compute()
void  smc.reset();
```

**Sign convention note.** `DiscreteSMC` computes with `compute(y - ref)` (error is plant output minus reference). `BasicSMC<Scalar>` computes with `compute(r - y)` (reference minus output), matching `BasicPID` and `DiscretePID`. This difference is intentional - `BasicPID` and `BasicSMC` share the same sign convention so they can be swapped without changing the calling code.

### API equivalence with DiscreteSMC

| `DiscreteSMC` parameter | `BasicSMC<Scalar>` parameter | Note |
|---|---|---|
| `c_e` | `c_e` | Surface error coefficient |
| `c_de` | `c_de` | Surface derivative coefficient |
| `Ks` | `Ks` | Switching gain |
| `phi` | `phi` | Boundary layer width |
| `compute(y - ref)` | `compute(r - y)` | **Sign flip - negate input** |

### Minimal embedded example

```cpp
#include "BasicSMC.h"

static ctrl::BasicSMC<float> smc(1.0f, 0.05f, 50.0f, 0.1f, 0.005f, -1.0f, 1.0f);

void control_isr(void) {
    float e = setpoint - read_sensor();
    float u = smc.compute(e);
    write_pwm(u);
}
```

---

## ComputationalDelayWrapper

### Purpose

In a real digital control loop there is always a one-sample computational delay between the moment the sensor is read and the moment the computed output reaches the actuator. This delay is often ignored in simulation, causing the simulated controller to outperform the real one - especially for fast loops or controllers with high bandwidth.

`ComputationalDelayWrapper` wraps any `IController` and introduces exactly one sample of delay: the output at step $k$ is the computation from step $k-1$.

```
Without wrapper:  y[k] -> controller -> u[k]    (applied at step k)
With wrapper:     y[k] -> controller -> u[k]    (held; applied at step k+1)
```

### Non-obvious facts

- **First call returns 0.0.** The held value is initialised to 0. The *real* controller output is computed on the first `compute()` call but not applied until the second. Warm up for one step before trusting the output.
- **Output is a held value.** `lastOutput()` returns the currently held (delayed) output, not the inner controller's fresh computation.
- Wraps any `shared_ptr<IController>` - works with `DiscretePID`, `DiscreteMPC`, `DiscreteADRC`, and all other `IController` subclasses.
- Does **not** double-wrap - check before applying if the inner controller already models delay internally.

### API

```cpp
#include "ComputationalDelayWrapper.h"

auto pid = std::make_shared<ctrl::DiscretePID>(Kp, Ki, Kd, Ts);
ctrl::ComputationalDelayWrapper delayed(pid);

// Simulation loop
for (int k = 0; k < N; ++k) {
    double u_applied = delayed.compute(error[k]);  // returns u from step k-1
    // apply u_applied to plant
    // plant returns y[k+1]
}
```

### Python usage

```python
import ctrl_toolbox as ctrl

pid     = ctrl.DiscretePID(Kp, Ki, Kd, Ts)
delayed = ctrl.ComputationalDelayWrapper(pid)

for k in range(N):
    u = delayed.compute(error[k])
    # u is the output computed from error[k-1]
```

### When to use ComputationalDelayWrapper

| Scenario | Use wrapper? |
|---|---|
| High-bandwidth loop (BW > 0.1 * sample rate) | **Yes** - delay has significant phase impact |
| Low-bandwidth PID (BW < 0.01 * sample rate) | Optional - delay effect is negligible |
| Comparing simulation vs. hardware results | **Yes** - aligns simulation model with hardware |
| MPC with long horizon | **Yes** - first step of the MPC horizon should account for delay |
| ILC (learning controller) | **Yes** - delay shifts the convergence point |

### Effect on stability margins

A one-sample delay $e^{-j\omega T_s}$ reduces the phase margin by $\omega_c T_s$ radians at the crossover frequency $\omega_c$. For a loop with $\omega_c T_s = 0.1$ rad (10* oversampling), this is about 5.7^\circ - negligible. For $\omega_c T_s = 0.5$ rad, the loss is 28.6^\circ - significant and must be compensated in the design.

---

## Deployment Checklist

When porting a controller from desktop simulation to an embedded target:

| Step | Action |
|---|---|
| 1 | Replace `DiscretePID` with `BasicPID<float>`; verify outputs match using `BasicPID<double>` first |
| 2 | Replace `DiscreteSMC` with `BasicSMC<float>`; note the sign-convention flip |
| 3 | Add `ComputationalDelayWrapper` to the desktop simulation to match hardware latency |
| 4 | Remove `AntiWindupWrapper` - `BasicPID` has built-in back-calculation anti-windup |
| 5 | Verify `omega_o * Ts < 0.5` if using ADRC - the constraint applies regardless of target |
| 6 | Check fixed-point overflow: `float` has 23 bits of mantissa; gains > 1e6 lose precision |

---

*See also:* `controller-tuning-reference.md` (gain tuning for PID and SMC), `mismatch_detection.md` (ControllerMonitor for runtime SPC on embedded targets), `control_design_pipeline.md` (Step 7: implementation and real-time considerations).
