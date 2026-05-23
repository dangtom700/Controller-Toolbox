# Controller Toolbox - Deployment Guide

*Version 1.0 - 2026-05-19*

This document covers three areas required before deploying this library to a production control
system: (1) per-controller parameter constraints and stability conditions; (2) real-time
integration guidelines (zero-allocation checklist, stack-size estimates, RTOS considerations);
and (3) a troubleshooting section for the most common runtime failure modes.

---

## 1. Parameter Constraints by Controller

### DiscretePID

| Parameter | Constraint | Consequence if violated |
|-----------|-----------|------------------------|
| `Kp`, `Ki`, `Kd` | All finite, `Kp > 0` | NaN output; state corruption |
| `N` (derivative filter) | `N > 0`; practical range `[1, 1000]` | `N -> inf` ≡ ideal derivative; amplifies high-freq noise |
| `Ts` | `Ts > 0`; must match actual sample period | Integral and derivative scale incorrectly |
| `Ki * Ts < 2 * Kp` | Discrete integrator stability criterion | Integral term can destabilise the closed loop |
| `uMin < uMax` | Hard requirement | Anti-windup back-calculation undefined |
| `Kb` | `0 <= Kb <= 1/Ts` recommended | `Kb = 0` disables anti-windup; `Kb` too large causes back-calculation oscillation |

**Bumpless parameter update:** `setParams()` changes gains instantly. To avoid a step in the
derivative term, reset after changing `Kd` or `N`, or ramp parameter changes over several samples.

---

### DiscreteLeadLag

| Parameter | Constraint |
|-----------|-----------|
| `continuousZero` | `> 0` (lead) or `< continuousPole` (lag) |
| `continuousPole` | `> 0`; `continuousPole != continuousZero` |
| `gain` | finite, non-zero |
| `Ts` | `Ts < pi / continuousPole` (Nyquist on the fastest pole) |

---

### DiscreteLQR

| Requirement | Check |
|-------------|-------|
| `(A, B)` stabilizable | Verified at construction via PBH test; stderr warning on failure |
| `Q` positive semi-definite | Caller responsibility; no runtime check - use `Q = Q.transpose()` and ensure non-negative eigenvalues |
| `R` positive definite | Enforced implicitly by DARE solver; add a floor: `R = R + epsilon.I` if uncertain |
| DARE convergence | Check `lqr.dareConverged()` before use; unconverged result may be unstable |

**DARE convergence check pattern:**
```cpp
ctrl::DiscreteLQR lqr(plant, lqr_params);
if (!lqr.dareConverged()) {
    // Log, fall back to a known-safe PID, or hold last output.
}
```

**DARE convergence conditions:**
- `(A, B)` must be stabilizable.
- `(A, C_q)` must be detectable, where `Q = C_q' * C_q`.
- All eigenvalues of `A` on the unit circle must be controllable from `B`.

---

### DiscreteMPC

| Parameter | Constraint | Guidance |
|-----------|-----------|----------|
| `Np` (prediction horizon) | `1 <= Np <= ~50` (memory) | Covers at least one settling time in samples |
| `Nc` (control horizon) | `1 <= Nc <= Np` | Smaller `Nc` gives smoother control; `Nc = 1` gives minimum-variation control |
| `rho_y` | `> 0` | Weight on output tracking |
| `rho_u` | `> 0` | Must be strictly positive to keep Hessian `H = Φ'Qy Φ + Ru` positive definite |
| `uMin / uMax` | `uMin < uMax` | Box constraint on absolute output |
| `duMin / duMax` | `duMin < duMax` | Box constraint on move |

**Hessian conditioning:**  If `rho_u` is very small (< 1e-6), the Hessian becomes ill-conditioned.
`computeRef()` checks `ldlt.info() != Eigen::Success` and returns the last safe output on failure.
Increase `rho_u` if this check triggers frequently.

**Real-time latency:** MPC solve time scales as O(Nc^3.m^3) for the LDLT solve plus O(Np.Nc.n.m)
for the matrix products. For `n=4, Np=10, Nc=3` expect ~5 µs on a 3 GHz core (see benchmark
output). Budget at least 3* this for worst-case jitter.

---

### KalmanFilter

| Parameter | Constraint |
|-----------|-----------|
| `Q_noise` (process noise) | Symmetric, positive semi-definite; elements >= 0 |
| `R_noise` (measurement noise) | Symmetric, positive definite; a floor of 1e-12 per diagonal element is applied automatically |
| `P0` (initial covariance) | Symmetric, positive definite; large `P0` (e.g., `10.I`) is safe when initial state is uncertain |

**Divergence symptoms:** residuals grow unboundedly; innovations `y - C*x_hat` grow larger than
expected. Common causes: `Q_noise` too small (plant model error not covered), `R_noise` too small
(sensor better than modelled), or non-linear plant violating the linear model assumption.

**Recovery:** call `reset()` and restart from a plausible `P0`. In RTOS code, implement a
watchdog that resets the filter if max(|residual|) > threshold for more than K consecutive samples.

---

### DiscreteADRC

| Parameter | Constraint | Note |
|-----------|-----------|------|
| `omega_o` | **`omega_o * Ts < 2`** (strict) | Forward-Euler ESO poles exit unit circle otherwise; constructor issues stderr warning |
| `omega_c` | `omega_c < omega_o / 3` (rule of thumb) | Separation principle: observer >= 3* faster than controller |
| `b0` | `b0 != 0`, same sign as plant DC gain | Wrong sign or zero causes division by zero in control law |
| `uMin / uMax` | `uMin < uMax` | Output saturation - ADRC has no built-in anti-windup; add margin |

**Example safe parameterisation** for `Ts = 0.01 s`:
```cpp
ADRCParams p;
p.omega_o = 50.0;  // 50 < 2/0.01 = 200 (check)
p.omega_c = 10.0;  // < 50/3 (check)
p.b0      = 1.0;
```

---

### SmithPredictor

| Parameter | Constraint |
|-----------|-----------|
| `delaySteps` | `>= 0`; pre-allocated at construction - do not change at runtime |
| Inner controller | Any `IController`; must be tuned for the delay-free plant |

The internal delay buffer is a fixed-size circular buffer allocated at construction.
**No dynamic allocation in the control loop.**

---

### ExtremumSeeker

| Parameter | Constraint |
|-----------|-----------|
| `dither_amp` | `> 0`; amplitude of the probing sinusoid |
| `dither_freq` | `0 < dither_freq < 0.5/Ts` (below Nyquist) |
| `lpf_cutoff` | `< dither_freq` (demodulation low-pass must be slower than dither) |
| `hpf_cutoff` | `< lpf_cutoff` |
| `integrator_gain` | `> 0`; governs convergence speed; too large causes oscillation around optimum |

**Convergence check:** ESC does not declare convergence autonomously. Implement a stagnation window:
```cpp
// If |theta[k] - theta[k-W]| < eps for W samples, optimum is found.
```

---

## 2. Real-Time Integration

### Zero-Allocation Checklist

Before deploying a controller in a real-time control loop, verify each item:

- [ ] **SmithPredictor:** constructed with the correct `delaySteps`; delay buffer pre-allocated.
- [ ] **DiscreteMPC:** `buildCondensedMatrices()` called at construction or on model change, never inside the timed loop. Work vectors `R_stack_`, `pred_err_`, `grad_`, `DeltaU_` are pre-allocated.
- [ ] **No `std::vector::push_back` / `resize`** in `compute()` paths.
- [ ] **No `std::deque`** in control paths (replaced by circular buffer in `SmithPredictor`).
- [ ] **No dynamic Eigen temporaries:** use `.noalias()` on all `MatrixXd * VectorXd` expressions. For small, fixed-size systems (`n <= 4`), switch to `Eigen::Matrix<double, N, N>` to enable compile-time allocation.
- [ ] **AtomicParamBuffer** used for any parameter that is updated from a non-RT thread (e.g., tuner output, operator override). RT thread calls `buf.read()`; background thread calls `buf.publish()`.
- [ ] **No STL streams in the control loop.** Use a `LockFreeRingBuffer<LogEntry, N>` fed by the control loop and drained by a low-priority writer thread (see `scripts/` for reference implementation).
- [ ] **No `std::cout` / `std::cerr`** in the hot path. The warning prints added to ADRC and LQR constructors fire once at construction time - not in `compute()`.

### Stack-Size Estimates

These are approximate peak stack depths for a single `compute()` call, excluding the Eigen
internal stack for small matrices (which is stack-allocated for `Matrix<double,N,N>` with N<=16).

| Controller | Approx. stack (n=4) |
|------------|-------------------|
| DiscretePID | < 64 bytes |
| DiscreteLeadLag | < 64 bytes |
| DiscreteSMC | < 64 bytes |
| DiscreteADRC | ~256 bytes (Vector3d ESO) |
| SmithPredictor | < 128 bytes (wraps inner controller) |
| KalmanFilter | O(n^2) - ~512 bytes for n=4 |
| DiscreteLQR | < 128 bytes (gain multiply only) |
| DiscreteMPC | O(Nc.m) - ~256 bytes for Nc=3, m=1 |
| ExtremumSeeker | < 256 bytes |
| ControllerStack | Sum of active controllers |

Add a 2* safety margin for OS/RTOS frame overhead and nested function calls.

### RTOS Considerations

**Task priority:** The control task should run at the highest real-time priority in the RTOS.
Non-RT work (logging, tuning, UI updates) belongs in a separate, lower-priority task.

**CPU isolation (Linux RT):** Pin the control task to a dedicated CPU core:
```
SCHED_FIFO priority 99, CPU affinity mask = 0x02 (core 1)
```
and disable IRQ affinity for that core in `/proc/irq/*/smp_affinity`.

**Timer source:** Use a hardware timer interrupt or a POSIX `timer_create(CLOCK_MONOTONIC, ...)`
with `SIGEV_THREAD_ID` to drive the sample-rate tick. Do **not** use `std::this_thread::sleep_for`
- it does not guarantee wakeup precision.

**Memory locking:** Call `mlockall(MCL_CURRENT | MCL_FUTURE)` before the control loop starts
to pin all pages in RAM and eliminate page-fault jitter.

**Compiler flags for RT builds:**
```
-O2 -fno-exceptions -fno-rtti -fstack-usage
```
Review the `.su` stack-usage report to verify per-function stack consumption.
Use `-fno-exceptions` only if all library code paths have been audited to not rely on RAII
for unwinding - the NaN guards use early-return, not exceptions, so this is safe.

---

## 3. Troubleshooting

### DARE Non-Convergence (DiscreteLQR)

**Symptom:** `lqr.dareConverged()` returns `false`; gain matrix `K_` may be large or irregular.

**Cause 1 - Plant not stabilizable:** One or more unstable modes of `A` are not reachable from `B`.
Check: `ctrl::SystemAnalysis::getPoles(plant)` - any `|lambda| >= 1` that cannot be shifted by `B`.
Fix: redesign the actuator placement or use output feedback with a full-order observer.

**Cause 2 - `R` nearly singular:** DARE iteration ill-conditioned.
Fix: add a floor `R = R + 1e-6 * I` before constructing `DiscreteLQR`.

**Cause 3 - `Q` not positive semi-definite:** Negative eigenvalue from floating-point asymmetry.
Fix: symmetrise: `Q = 0.5 * (Q + Q.transpose())` and project: `Q = Q.cwiseMax(0)` on diagonal.

**Fallback strategy:**
```cpp
ctrl::DiscreteLQR lqr(plant, params);
if (!lqr.dareConverged()) {
    // Option A: use a known-safe pre-computed gain
    lqr_gain_fallback.setRows(...);
    // Option B: switch ControllerStack to the backup PID
    stack.setActiveController("PID_backup");
    // Option C: hold last control output and alarm
}
```

---

### MPC Infeasibility / LDLT Failure

**Symptom:** `computeRef()` returns last safe output repeatedly; tracking error grows.

**Cause 1 - Hessian not positive definite:** `rho_u` too small or zero.
Fix: increase `rho_u`; minimum safe value is approximately `1e-6 * rho_y * max(eigenvalue(Φ'Φ))`.

**Cause 2 - Model mismatch:** `F_` and `Phi_` built from a linearised model that no longer
represents the plant. For gain-scheduled MPC, call `setPlant()` after each re-linearisation.

**Cause 3 - Horizons too aggressive for the plant dynamics:** Large `Np` on a fast-unstable
plant can cause Phi to have very large entries, making `H` ill-conditioned.
Fix: reduce `Np` to cover 1-2 settling times, or increase `rho_u`.

**Monitoring:** log the LDLT return status count per minute. More than occasional failures
indicate a tuning or model problem, not a numerical one.

---

### Kalman Filter Divergence

**Symptom:** State estimate `stateEstimate()` diverges from true state; covariance `P_` grows.

**Cause 1 - `Q_noise` underestimated:** Filter trusts the model too much; disturbances accumulate.
Fix: increase `Q_noise` diagonal entries. A useful heuristic: `Q_ii approx = sigma^2_{disturbance,i}`.

**Cause 2 - `R_noise` underestimated:** Filter gives innovation too much weight; noisy
measurement drives the estimate.
Fix: increase `R_noise`; the floor of 1e-12 prevents exact zero but does not prevent small values.

**Cause 3 - Plant non-linearity:** True dynamics are not captured by the linear model.
Fix: use an Extended Kalman Filter (EKF) or Unscented KF - not currently in this toolbox.

**Reset policy:**
```cpp
// Watchdog: if residual RMS over last W steps > threshold, reset
double rms = residual_buffer.rms();
if (rms > 5.0 * sqrt(R_noise(0,0))) {
    kf.reset();
    // Re-inject last known good state estimate or zero
}
```

---

### ADRC ESO Instability (Forward-Euler)

**Symptom:** ESO states `z1`, `z2`, `z3` grow without bound; control output saturates.

**Cause:** `omega_o * Ts >= 2`. The constructor prints a warning; check stderr at startup.

**Fix:** Reduce `omega_o` or decrease `Ts`. For `Ts = 0.01 s`, the hard limit is `omega_o < 200 rad/s`.
Practically, keep `omega_o <= 100 rad/s` for a 2* stability margin.

**Rule of thumb:** Start with `omega_o = 10 / Ts` (i.e., 1000 rad/s for `Ts=0.001`) - this is
the upper practical limit before ESO pole locations become sensitive to round-off error.

---

### SmithPredictor Delay Mismatch

**Symptom:** Oscillation or poor transient performance despite stable inner loop.

**Cause:** `delaySteps` does not match the true plant delay. A mismatch of even one sample can
destabilize the predictor for lightly damped plants.

**Fix:** Measure the transport delay experimentally (step test, cross-correlation) and ensure
`delaySteps = round(L / Ts)` where `L` is the measured delay in seconds.

For non-integer `L / Ts` (fractional delay), a first-order Padé approximation in the inner model
is recommended - not yet implemented in this toolbox.

---

### NaN / Inf Propagation

All scalar-input controllers (`DiscretePID`, `DiscreteLeadLag`, `DiscreteSMC`, `DiscreteADRC`)
now return the last valid output on `NaN` or `Inf` input rather than propagating the corruption.

If you observe the output "freezing" at an unexpected value, check upstream:
1. Sensor: `sensor.isValid()` - `SimSensor` always returns `true`; real sensors should override this.
2. Reference generator: ensure `r` is finite before passing to the controller.
3. Actuator: `SimActuator::write()` applies NaN hold-last before stepping the plant; log `lastOutput()` to detect frozen actuator commands.

---

## 4. Quick-Start Parameter Tables

### Typical starting points for an unknown SISO plant

| Controller | Starting point |
|------------|---------------|
| PID (Ziegler-Nichols) | Use `RelayAutoTuner` - provides `Ku`, `Tu` -> `Kp = 0.6.Ku`, `Ti = 0.5.Tu`, `Td = 0.125.Tu` |
| Lead-Lag | Phase margin < 30^\circ: add lead. `zero = omega_c / 5`, `pole = 5.omega_c`. Adjust `gain` for 0 dB at `omega_c`. |
| LQR | Start with `Q = diag(1/y_max^2,...) `, `R = diag(1/u_max^2,...)` (Bryson's rule) |
| MPC | `Np = round(4.τ / Ts)`, `Nc = max(1, round(Np/3))`, `rho_u = 0.01.rho_y` |
| ADRC | `omega_c approx = desired_bandwidth`, `omega_o = 5.omega_c`, `b0 = plant_DC_gain` |
| Kalman | `Q = 1e-3.I`, `R = variance_of_sensor_noise.I`, `P0 = 10.I` - then tune Q/R ratio |

---

*End of deployment guide.*
