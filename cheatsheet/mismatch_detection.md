# Mismatch Detection and Runtime Monitoring

### MismatchDetector (D1), ControllerMonitor (SPC), and DAE Plant Models (P1/P2/P3)

---

## Introduction

A controller designed for a nominal model will degrade - sometimes catastrophically - when the real plant drifts away from that model. Two complementary toolbox components address this:

- **`MismatchDetector`** (D1) monitors the innovation sequence of a running `KalmanFilter` or `MovingHorizonEstimator` using a CUSUM chart. When innovation statistics deviate from the expected zero-mean white-noise pattern, it raises a sticky `mismatchDetected()` flag that can trigger re-identification or parameter adaptation.
- **`ControllerMonitor`** provides broader statistical process control (SPC) for any controller: CUSUM and EWMA charts on the control error or any scalar signal, attachable as an `IControllerObserver`.

A third component, `DAESystem`, is included in this file because it directly affects the plant model structure that both estimators consume.

---

## Part I - MismatchDetector (D1)

### Motivation

The Kalman filter innovation $\nu_k = y_k - C\hat{x}_{k|k-1}$ should be zero-mean white noise with covariance $S_k = C P_{k|k-1} C^T + R$ when the model is correct. A persistent bias or growing variance in $\|\nu_k\| / \sqrt{p}$ (where $p$ is the output dimension) is a reliable indicator that the model has drifted from the plant.

`MismatchDetector` wraps a `CUSUMChart` from `ControllerMonitor.h` and feeds it the normalised innovation norm after each filter update.

### Key API (C++)

Enabled on `KalmanFilter`:
```cpp
#include "KalmanFilter.h"

ctrl::MismatchDetectorParams mp;
mp.sigma       = 1.0;   // expected std of ||innov||/sqrt(p) when model is correct
mp.k_cusum     = 0.5;   // CUSUM reference (half the expected shift to detect)
mp.h_threshold = 5.0;   // decision threshold - alarm when S_k > h_threshold * sigma

kf.enableMismatchDetection(mp);

// after each kf.update(y):
if (kf.mismatchDetected()) {
    double score = kf.mismatchScore();   // current CUSUM accumulator value
    kf.resetMismatchDetector();          // clear after re-identification
    // trigger GreyBoxEstimator re-fit or parameter reset
}
```

Enabled on `MovingHorizonEstimator`:
```cpp
mhe.enableMismatchDetection(mp);
// identical API: mismatchDetected(), mismatchScore(), resetMismatchDetector()
```

### Key API (Python)

```python
import ctrl_toolbox as ctrl
import numpy as np

kf = ctrl.KalmanFilter(A, B, C, Q, R, P0, x0)
kf.enable_mismatch_detection(sigma=1.0, k_cusum=0.5, h_threshold=5.0)

for k in range(N):
    kf.predict(u[k])
    kf.update(y[k])

    if kf.mismatch_detected():
        print(f"Mismatch at step {k}, score={kf.mismatch_score():.2f}")
        kf.reset_mismatch_detector()
        # re-identification logic here
```

### Non-obvious facts

- **Sticky flag**: `mismatchDetected()` returns `true` until `resetMismatchDetector()` is called. The underlying CUSUM accumulator is also reset by that call.
- **Normalised input**: The CUSUM receives $\|\nu_k\|_2 / \sqrt{p}$, not the raw norm. This makes `sigma` and `h_threshold` approximately dimension-independent - the same threshold works regardless of whether $p = 1$ or $p = 6$.
- **CUSUM direction**: Only the upper-sided CUSUM is used (detects upward shift in innovation norm). A model that suddenly fits *better* than expected is not flagged.
- **One-sided sensitivity**: Setting `k_cusum = sigma/2` gives the standard CUSUM design for detecting a one-sigma shift. Increase `k_cusum` to be less sensitive to small drifts.
- **MHE innovation**: For `MovingHorizonEstimator`, the innovation is computed from the arrival-cost state estimate vs. the first measurement in the current window (not the full window residual).

### Tuning guidelines

| Parameter | Conservative (few false alarms) | Sensitive (catch early drift) |
|---|---|---|
| `sigma` | From KF steady-state $S_k$ | From KF steady-state $S_k$ |
| `k_cusum` | `0.75 * sigma` | `0.25 * sigma` |
| `h_threshold` | `8.0 * sigma` | `3.0 * sigma` |

A good initial calibration: run the closed-loop system for 200-500 steps in normal operation, compute `std(||innov||/sqrt(p))`, and use that as `sigma`.

---

## Part II - ControllerMonitor (SPC)

### Purpose

`ControllerMonitor` implements two statistical process control charts - CUSUM and EWMA - as an `IControllerObserver`. Attach it to any `IController` that calls `notifyObserver(output)` each step; it accumulates the control output (or you can feed it arbitrary scalars directly via `update(scalar)`).

### Architecture

```
IController --notifyObserver(u)--> ControllerMonitor
                                       |
                               CUSUMChart (upper + lower sided)
                               EWMAChart
                               onState("eso", z)   <- DiscreteADRC ESO state
                               onState("surface", s) <- DiscreteSMC surface
```

### Key classes and methods

```cpp
#include "ControllerMonitor.h"

// CUSUM chart
ctrl::CUSUMChart cusum(k_ref, h_threshold);
cusum.update(value);
bool alarm = cusum.alarm();   // sticky until reset()
double score = cusum.score(); // current accumulator
cusum.reset();

// EWMA chart
ctrl::EWMAChart ewma(lambda, L);  // lambda=smoothing, L=sigma multiplier for limits
ewma.update(value);
bool alarm = ewma.alarm();
double smoothed = ewma.ewma();

// Full ControllerMonitor as observer
ctrl::ControllerMonitor mon(cusum_params, ewma_params);
controller.attachObserver(std::make_shared<ctrl::ControllerMonitor>(mon));
```

### Python usage

```python
mon = ctrl.ControllerMonitor(
    k_cusum=0.5, h_threshold=5.0,   # CUSUM params
    ewma_lambda=0.2, ewma_L=3.0     # EWMA params
)
pid.attach_observer(mon)

for k in range(N):
    u = pid.compute(error[k])
    if mon.cusum_alarm():
        print(f"CUSUM alarm at step {k}")
    if mon.ewma_alarm():
        print(f"EWMA alarm at step {k}")
```

### CUSUM vs EWMA - when to use which

| Property | CUSUM | EWMA |
|---|---|---|
| Best for | Detecting sustained mean shift | Detecting gradual drift or increased variance |
| Sensitivity to outliers | Low (accumulator is monotone) | Moderate (smoothing damps isolated spikes) |
| False alarm rate at ARL0 | ~370 (standard design) | ~370 (standard design with L=3) |
| Reset after alarm | Manual (`reset()`) | Manual (`reset()`) |
| Computational cost | O(1) per step | O(1) per step |

### Attaching to DiscreteADRC / DiscreteSMC

These controllers emit internal state via `notifyObserverState()`:
- `DiscreteADRC` emits `"eso"` -> the 3-component ESO state vector $z = [z_1, z_2, z_3]$.
- `DiscreteSMC` emits `"surface"` -> the sliding surface scalar $s$.

These can be monitored by overriding `onState(key, vec)` in a custom observer derived from `IControllerObserver`.

---

## Part III - DAE System Models (P1/P2/P3)

### Motivation

Many physical systems have algebraic constraints alongside differential equations - chemical equilibrium relations, kinematic constraints in robotics, network power-balance equations. Lumping these into the ODE by substitution can cause numerical ill-conditioning or destroy the sparse structure of the Jacobian. The toolbox provides a first-class `DAESystem` type alongside three operations:

| ID | Operation | Function |
|---|---|---|
| P1 | Consistent initialisation + discrete step | `consistentInit()`, `dae2ode()` in `PlantModel.h` |
| P2 | Algebraic elimination -> `StateSpace` | `c2d(DAESystem, Ts, ...)` -> `ctrl::StateSpace` |
| P3 | DAE-constrained EKF | `ExtendedKalmanFilter::setAlgebraicConstraint()` |

### P1 - DAESystem: Structure and consistent initialisation

```cpp
#include "PlantModel.h"

ctrl::DAESystem dae;
dae.F = [](const VectorXd& x, const VectorXd& z, const VectorXd& u) -> VectorXd {
    // differential part: xdot = F(x, z, u)
    return A11*x + A12*z + B1*u;
};
dae.G = [](const VectorXd& x, const VectorXd& z, const VectorXd& u) -> VectorXd {
    // algebraic part: 0 = G(x, z, u)  [must be zero]
    return G1*x + G2*z;
};
dae.n_diff = 4;   // number of differential states x
dae.n_alg  = 2;   // number of algebraic variables z

// Find consistent (x0, z0) satisfying G(x0, z0, u0) = 0
auto [x0, z0] = ctrl::consistentInit(dae, x0_guess, z0_guess, u0, tol=1e-10);

// Step forward one Ts using forward-Euler + Newton projection
auto [x_next, z_next] = ctrl::dae2ode(dae, x0, z0, u, Ts);
```

### P2 - Linearisation and discretisation of DAE

`c2d(DAESystem, Ts)` performs algebraic elimination: computes the reduced state matrix
$$A_\text{red} = A_{11} - A_{12} G_2^{-1} G_1$$
and dispatches to the standard `c2d(StateSpace, Ts, ZOH)` to produce a discrete-time `StateSpace` object suitable for LQR, MPC, or Kalman filter design.

```cpp
ctrl::StateSpace sys_d = ctrl::dae_c2d(dae, Ts);
// sys_d.A, sys_d.B, sys_d.C, sys_d.D are fully populated

// Python
sys_d = ctrl.dae_c2d(dae, Ts)
```

### P3 - EKF with algebraic constraint projection

After the UKF/EKF update step, the state estimate may violate the algebraic constraints. `setAlgebraicConstraint` adds a Newton projection step that maps $\hat{x}$ back onto the constraint manifold and applies the covariance correction $P = J_\text{proj} P J_\text{proj}^\top$.

```cpp
#include "ExtendedKalmanFilter.h"

ekf.setAlgebraicConstraint(
    g_alg,   // std::function<VectorXd(VectorXd)> returning G(x)
    n_diff,  // number of differential states
    n_alg    // number of algebraic states
);

// After each ekf.update(y, u), the state estimate satisfies G(x_hat) approx = 0
```

**Non-obvious facts.**

- P2 `dae_c2d` assumes $G_2$ (the Jacobian of $G$ w.r.t. algebraic variables $z$) is square and invertible. If the DAE is index > 1, this assumption fails; reduce the index first by differentiation.
- P3 projection is Newton-Raphson with one iteration by default. For loosely constrained systems (large residual after one step), increase `n_newton_steps` in `AlgebraicConstraintParams`.
- `consistentInit` uses LDLT decomposition internally - it is not suitable for indefinite $G_2$ Jacobians.

---

## Integration: Mismatch Detection + DAE + Re-Identification

The recommended closed-loop monitoring architecture:

```
[DAESystem] --consistent init--> [EKF with algebraic projection]
                                         |
                               [MismatchDetector enabled on EKF]
                                         |
                         mismatchDetected() == true?
                                   /         \
                                 YES          NO
                                  |            |
                    [GreyBoxEstimator.fit()]  continue
                    [update DAESystem params]
                    [resetMismatchDetector()]
```

This loop provides automatic model maintenance: the DAE EKF runs in real time, the CUSUM accumulates innovation statistics, and upon alarm the batch estimator re-fits the uncertain parameters using recent closed-loop data.

---

*See also:* `phase2_hybrid_modeling.md` (GreyBoxEstimator, HybridMPC), `model_evaluation.md` (offline residual analysis and cross-validation), `controller-tuning-reference.md` (CUSUM tuning for SPC).
