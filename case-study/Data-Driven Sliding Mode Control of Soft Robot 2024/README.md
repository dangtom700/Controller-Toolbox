# Data-Driven Sliding Mode Control of a Soft Robot

## Reference
Dimitrios Papageorgiou, Guðrún Þóra Sigurðardóttir, Egidio Falotico, Silvia Tolu (2024). "Data-driven sliding mode control of soft robots." *Control Engineering Practice* 144, 105836.

---

## Plant Model

A **continuum soft robot module** (205 mm length) actuated by a combination of **cable tendons and McKibben pneumatic muscles**. The continuum kinematics are highly nonlinear and hysteretic, making first-principles modelling impractical. The paper uses **SINDYc** (Sparse Identification of Nonlinear Dynamics with control inputs) to identify a sparse, interpretable data-driven state-space model from operational measurements at 40 Hz.

### Physical Description

- **Actuators:** Cable tendons and McKibben artificial muscles providing bending in two planes
- **Sensing:** End-effector 3D position tracked via electromagnetic sensors or motion capture
- **Model identification:** SINDYc with a polynomial + trigonometric function library; identified model is `xdot = Ξ Theta(x, u)` where `Ξ` is the sparse coefficient matrix learned from data
- **Operating rate:** 40 Hz (Ts = 0.025 s); quasi-static assumption does not apply - inertial and viscoelastic dynamics matter

### State / Output Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `x` | End-effector x-position | mm |
| `y` | End-effector y-position | mm |
| `z` | End-effector z-position | mm |
| `q1, q2` | Bending curvature coordinates (cable tensions or muscle pressures) | N or kPa |

### Control Input

| Symbol | Range | Description |
|--------|-------|-------------|
| `u1, u2` | [0, u_max] | Cable tension or muscle pressure commands in two bending planes |

### Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Sampling time | Ts = 0.025 s (40 Hz) | Data acquisition and control loop rate |
| Module length | ~205 mm | Continuum segment |
| SINDYc library | Poly deg 1-3 + trig | Sparse function library for model identification |
| Model states | 4-6 | Curvature + velocity states in reduced-order model |

---

## Control Objective

Track a desired 3D end-effector trajectory `[x_d(t), y_d(t), z_d(t)]` subject to:
- **Hysteresis** in cable/muscle actuation coupling
- **Creep** at constant input over extended periods
- **Unmodelled dynamics** (material fatigue, temperature-dependent stiffness)
- **Disturbances** (payload changes, contact forces)

The paper's primary contribution is a **Super-Twisting Sliding Mode Controller (STSMC)** that uses the SINDYc model for the equivalent control component and an **online input estimator** to compensate for unknown input disturbances in real time. The STSMC + estimator combination is compared against PID and standard SMC baselines.

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | STSMC | `DiscreteSMC` | c=1.0, K=0.5, phi=0.05 | Equivalent control from SINDYc model; super-twisting gains; sign convention: compute(y - ref) |
| 2 | PID | `DiscretePID` | Kp=2.0, Ki=0.1, Kd=0.05 | Baseline; separate x/y/z channels; anti-windup clamp |
| 3 | ADRC | `DiscreteADRC` | omega_o=8, omega_c=2, b0=0.5 | ESO lumps hysteresis + creep as total disturbance; omega_o*Ts = 8*0.025 = 0.2 < 0.5 (check) |
| 4 | MRAC | `MRACController` | gamma=0.5, a_m=-2.0, b_m=2.0 | Adapts to time-varying plant gain from material changes; call compute(y_plant) NOT compute(e) |
| 5 | L1Adaptive | `L1AdaptiveController` | a_m=-2.0, b_m=2.0, omega_c=5.0 | Low-pass filtered adaptation; fast Gamma=50 for stiffness variation |
| 6 | NeuralPID | `NeuralPID` | n_h=8, lr=1e-4 | Online PID gain adaptation from tip error features |
| 7 | DynaCtrl | `DynaController` | n_collect=100, n_refit=50 | Dyna-MBRL wrapping PID; refits SINDy error model from online data |
| 8 | FuzzyPID | `FuzzyPIDController` | e_max=20 mm, de_max=10 mm/s | Gain-scheduled for large-deflection nonlinearity |
| 9 | MPC | `DiscreteMPC` | Np=10, Nu=3, rho_y=100, rho_u=0.01 | Linearised model from SINDYc Jacobian at current operating point |
| 10 | SINDy+MPC | `SINDy` + `DiscreteMPC` | poly_deg=2, Np=8 | Identifies sparse dynamics from exploration data; embeds SINDyModel in MPC predictor |
| 11 | CBFSafety | `CBFSafetyFilter` | h(x) = R_max^2 - (x^2+y^2+z^2); alpha=2.0 | Wraps STSMC; enforces 3D workspace boundary as control barrier |
| 12 | ILC | `ILCController` | Lp=0.6, P-type; trial_length=N | For periodic trajectories (circle, lemniscate); learns feedforward correction trial-to-trial |

---

## Scenarios

| ID | Description | Reference Signal | Stress Factor |
|----|-------------|-----------------|---------------|
| s01_circle | Circular tip trajectory r=20 mm, f=0.1 Hz in x-y plane | Sinusoidal x/y | Smooth periodic; tests steady-state hysteresis rejection |
| s02_step | Step commands: 0->30 mm x-displacement | Step | Transient; large input change triggers creep |
| s03_lemniscate | Figure-8 (lemniscate of Bernoulli) trajectory | Periodic nonlinear | Direction reversal; tests switching control chattering |
| s04_disturbance | Circular tracking + lateral point load at t=5 s | Sinusoidal + step disturbance | Robustness to external push |
| s05_material_drift | Circle tracking as stiffness drifts +30% over 60 s | Sinusoidal | Slow parameter variation; tests adaptation |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **SINDYc model:** The SINDYc library should include polynomial terms up to degree 2 plus sinusoidal basis functions. The identified model `xdot = Ξ Theta(x, u)` can be wrapped in a `SINDyModel::stateFunc()` for use in MPC/DynaController.
- **STSMC design:** For the super-twisting algorithm, the sliding surface is `s = c*e + ė`; the super-twisting control law is `u_eq = -K1*sqrt(|s|)*sign(s) - K2*\intsign(s)dt`. Use `DiscreteSMC` for the equivalent control framework with `compute(y - ref)`.
- **Online input estimator:** The paper's input estimator runs a recursive identification step each period to update the effective input gain. This can be approximated by `RecursiveLeastSquares` updating the `b0` parameter for ADRC.
- **ADRC b0:** Estimate from SINDYc Jacobian at operating point: `b0 = df/du` evaluated at current state.
- **Workspace constraint:** Hard constraint on combined actuation `|u| <= u_max`. CBFSafetyFilter wraps STSMC with `h(x) = R_max^2 - ||tip||^2` where `g = -2*tip^T * dtip/du` from SINDYc Jacobian.
- **Sampling time:** Ts = 0.025 s (40 Hz). ADRC constraint: `omega_o * 0.025 < 0.5` -> `omega_o < 20 rad/s`. Use omega_o = 8.
- **CSV columns:** `t, x_ref, y_ref, z_ref, x, y, z, u1, u2, error_norm, iae_cumulative`

---

## Status

Spec only - `sim/` not present, not registered, not built.
