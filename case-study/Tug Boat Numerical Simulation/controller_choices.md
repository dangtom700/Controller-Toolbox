# Tug Boat Numerical Simulation - Controller Choices

**Document:** Controller Choice for Modeling
**Audience:** Technical engineers familiar with control systems and C++ programming
**Date:** 2026-05-23

---

## 1. Selection Criteria

Controllers were selected to span four design philosophies - proportional-integral-derivative
feedback, model-based sliding mode, constrained predictive control, and model-free
optimization - while remaining implementable using existing Controller Toolbox classes.
All five modes share the same plant, disturbance realization, and thrust allocator to
ensure a fair comparison.

---

## 2. Controller Portfolio

### Mode 1 - PID Baseline (`DiscretePID`)

**Design rationale:** Establishes a performance lower bound using the simplest
model-independent feedback strategy. Three decoupled discrete-time PID loops operate on
the surge, sway, and yaw error signals independently.

**Error computation:**
$$
\mathbf{e} = R^\top(\psi)\,(\boldsymbol{\eta}_\mathrm{ref} - \boldsymbol{\eta})
$$

Errors are expressed in body frame to decouple the three axes approximately.

**Control law (backward-Euler discrete PID):**
$$
\tau_{c,j,k} = K_{p,j}\,e_{j,k} + K_{i,j}\sum_{n=0}^{k} e_{j,n}\,\Delta t + K_{d,j}\,\frac{e_{j,k} - e_{j,k-1}}{\Delta t}
$$

with back-calculation anti-windup and filtered derivative (filter coefficient $N=10$).

**Gains (Bryson's method, tuned offline):**

| Axis | $K_p$ | $K_i$ | $K_d$ |
|------|-------|-------|-------|
| Surge $x$ | $3\times10^5$ N/m | $1\times10^4$ N/(m.s) | $8\times10^5$ N.s/m |
| Sway $y$ | $3\times10^5$ N/m | $1\times10^4$ N/(m.s) | $8\times10^5$ N.s/m |
| Yaw $\psi$ | $8\times10^6$ N.m/rad | $5\times10^5$ N.m/(rad.s) | $2\times10^7$ N.m.s/rad |

**Output saturation:** $\tau_{x,y} \in [-2\times10^6,\,2\times10^6]\;\text{N}$;
$\tau_\psi \in [-5\times10^7,\,5\times10^7]\;\text{N.m}$.

**Toolbox class:** `DiscretePID` (three instances, one per axis).

**Expected behavior:** Moderate steady-state error under sustained disturbance; derivative
chattering amplified in noisy-measurement mode. Serves as reference point for all other
controllers.

---

### Mode 2 - Kalman Filter + PID (`KalmanFilter` + `DiscretePID`)

**Design rationale:** Isolates the benefit of state estimation from changes in control
structure. Identical PID gains to Mode 1; only the measurement input is filtered.

**Linearized state-space model** (around zero velocity, used to construct $A$, $B$):

$$
\mathbf{X}_{k+1} = A\,\mathbf{X}_k + B\,\mathbf{u}_k + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(0,Q_\mathrm{kf})
$$
$$
\mathbf{z}_k = H\,\mathbf{X}_k + \mathbf{v}_k, \quad \mathbf{v}_k \sim \mathcal{N}(0,R_\mathrm{kf}), \quad H = I_6
$$

where $A = I + \Delta t \cdot f_\mathrm{lin}$ and $B = \Delta t \cdot M_\mathrm{re}^{-1}$.

**Kalman predict-update:**
$$
\hat{\mathbf{X}}_{k|k-1} = A\,\hat{\mathbf{X}}_{k-1} + B\,\mathbf{u}_{k-1}, \quad
P_{k|k-1} = A\,P_{k-1}\,A^\top + Q_\mathrm{kf}
$$
$$
K_k = P_{k|k-1}H^\top(HP_{k|k-1}H^\top + R_\mathrm{kf})^{-1}
$$
$$
\hat{\mathbf{X}}_k = \hat{\mathbf{X}}_{k|k-1} + K_k(\mathbf{z}_k - H\,\hat{\mathbf{X}}_{k|k-1}), \quad
P_k = (I - K_kH)\,P_{k|k-1}
$$

**Covariance defaults:**
$Q_\mathrm{kf} = \mathrm{diag}[10^{-3},10^{-3},10^{-5},10^{-2},10^{-2},10^{-4}]$,
$R_\mathrm{kf} = \mathrm{diag}[10^{-2},10^{-2},10^{-4},10^{-1},10^{-1},10^{-3}]$.

**Toolbox classes:** `KalmanFilter` (for estimation) + `DiscretePID` (identical to Mode 1).

**Expected behavior:** Reduced derivative chattering and lower saturation count relative to
Mode 1 under noisy measurements. Demonstrates Tier-1 improvement without full model use.

---

### Mode 3 - Sliding Mode Controller (`DiscreteSMC`)

**Design rationale:** Directly replicates the paper's proposed controller (Li et al. Eqs. 24-27).
This is the primary cross-validation benchmark against published IAE results (paper Table 7).
Requires full plant model knowledge.

**Sliding surface:**
$$
\mathbf{s} = \dot{\mathbf{e}} + \Lambda\,\mathbf{e} + K_{i,s}\int_0^t \mathbf{e}\,d\tau
$$

**Equivalent control** (model-cancellation term):
$$
\boldsymbol{\tau}_\mathrm{eq} = -M_\mathrm{re}\,\Lambda\,\dot{\mathbf{e}} + D_\mathrm{re}\,\boldsymbol{\nu} + C_\mathrm{re}(\boldsymbol{\nu})\,\boldsymbol{\nu}
$$

**Switching control** (boundary-layer saturation):
$$
\boldsymbol{\tau}_\mathrm{sw} = -K_\mathrm{sw}\,\mathrm{sat}\!\left(\frac{\mathbf{s}}{\Phi}\right), \quad
\mathrm{sat}(x) = \begin{cases} x & |x|\leq1 \\ \mathrm{sgn}(x) & |x|>1 \end{cases}
$$

**Combined output:**
$$
\boldsymbol{\tau}_c = \boldsymbol{\tau}_\mathrm{eq} + \boldsymbol{\tau}_\mathrm{sw}
$$

**Parameters (paper Table 4):**

| Parameter | Surge | Sway | Yaw |
|-----------|-------|------|-----|
| $\Lambda$ | 0.05 | 0.05 | 0.10 |
| $K_{i,s}$ | $10^{-4}$ | $10^{-4}$ | $10^{-4}$ |
| $K_\mathrm{sw}$ | $8\times10^5$ N/m | $8\times10^5$ N/m | $2\times10^7$ N.m/rad |
| $\Phi$ | 0.5 m | 0.5 m | 0.05 rad |

**Toolbox class:** `DiscreteSMC`.

**Expected behavior:** IAE values within +/-10% of paper Table 7 (S2-S4 scenarios) with ideal
observer. Chattering suppressed by boundary layer $\Phi$.

---

### Mode 4 - Model Predictive Control (`DiscreteMPC`)

**Design rationale:** Horizon-based optimal control that explicitly accounts for actuation
limits. Expected to achieve the lowest fuel consumption by anticipating disturbances and
avoiding saturation-inducing overshoot.

**Condensed QP cost function:**
$$
J = \sum_{k=0}^{N_p} \mathbf{e}_k^\top Q_\mathrm{mpc}\,\mathbf{e}_k + \sum_{k=0}^{N_c-1} \boldsymbol{\tau}_{c,k}^\top R_\mathrm{mpc}\,\boldsymbol{\tau}_{c,k}
$$

**Linearized prediction model** (same A, B as Mode 2):
$$
\mathbf{e}_{k+i|k} = \Phi^i\,\mathbf{e}_k + \sum_{j=0}^{i-1}\Phi^{i-1-j}B\,\boldsymbol{\tau}_{c,j}
$$

The horizon is condensed into a single QP and solved at each control tick (receding horizon).
First move $\boldsymbol{\tau}_{c,0}^*$ is applied; the rest are discarded.

**Parameters:**

| Parameter | Value |
|-----------|-------|
| Prediction horizon $N_p$ | 10 steps (5 s) |
| Control horizon $N_c$ | 3 steps |
| $Q_\mathrm{mpc}$ | $\mathrm{diag}[10^3,\,10^3,\,10^5]$ |
| $R_\mathrm{mpc}$ | $\mathrm{diag}[1,\,1,\,10^{-3}]$ |

**Toolbox class:** `DiscreteMPC` (with internal condensed QP solver or OSQP backend).

**Expected behavior:** Lowest $E_\mathrm{fuel}$ and $\mathrm{Sat}_\mathrm{count}$ among all
modes. Slightly higher compute time per tick than PID/SMC.

---

### Mode 5 - Extremum Seeking Control (`ExtremumSeeker`)

**Design rationale:** Model-free gradient-based optimizer. Serves as the lower-performance
reference for data-free approaches - requires no plant knowledge and no tuning of
model-based parameters.

**Dither injection:**
$$
\boldsymbol{\tau}_c = \boldsymbol{\tau}_\mathrm{nom} + a_d\sin(\omega_d t)
$$

**Gradient estimation via demodulation:**
$$
\hat{\nabla} J = \mathrm{HPF}\!\left[\mathrm{cost}_k \cdot a_d\sin(\omega_d t_k)\right]
$$

**Descent update:**
$$
\dot{\boldsymbol{\tau}}_\mathrm{nom} = -k_\mathrm{esc}\,\hat{\nabla} J
$$

**Parameters:**

| Parameter | Value |
|-----------|-------|
| Dither amplitude $a_d$ | $5\times10^3$ N (position axes), $1\times10^5$ N.m (yaw) |
| Dither frequency $\omega_d$ | 0.016-0.020 Hz (per axis, staggered) |
| High-pass cutoff | 0.05 rad/s |
| Low-pass cutoff | 0.02 rad/s |
| Descent gain $k_\mathrm{esc}$ | -1 per axis |

**Toolbox class:** `ExtremumSeeker`.

**Expected behavior:** Slowest convergence (~10* longer than PID). Useful only when plant
knowledge is entirely unavailable. Documents the cost of model-free operation on this task.

---

## 3. Controller Comparison Summary

| Mode | Controller | Plant Knowledge | Toolbox Class | Key Metric Advantage |
|------|-----------|----------------|---------------|----------------------|
| 1 | PID | None (FOPDT) | `DiscretePID` | Simplicity, reference |
| 2 | KF-PID | Linearized | `KalmanFilter` + `DiscretePID` | Noise rejection |
| 3 | SMC | Full $M_\mathrm{re}$, $D_\mathrm{re}$ | `DiscreteSMC` | Paper benchmark (IAE) |
| 4 | MPC | Linearized, horizon | `DiscreteMPC` | Fuel efficiency |
| 5 | ESC | None (model-free) | `ExtremumSeeker` | Zero model requirement |

---

## 4. Shared Interface

All five controllers implement the `IController` interface:

```cpp
// compute(reference, measurement) -> generalized force command
Eigen::Vector3d compute(const Eigen::Vector3d& ref,
                        const Eigen::VectorXd& state);
```

`ref` is $[\,x_\mathrm{target},\,y_\mathrm{target},\,\psi_\mathrm{target}\,]^\top$.
`state` is $[\,x,\,y,\,\psi,\,u,\,v,\,r\,]^\top$ (filtered for Mode 2, raw for others).
Return value is $\boldsymbol{\tau}_c = [\tau_x,\,\tau_y,\,\tau_\psi]^\top$ before saturation.
