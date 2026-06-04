# Multi-Body Floating Wind-Wave Platform

## Reference
**Title:** A mathematical model for the dynamic analysis of multi-body floating platforms with complex mechanical constraints  
**Authors:** Thiago S. Hallak, Jose F. Gaspar, C. Guedes Soares  
**Journal:** Ocean Engineering, Vol. 314, 2024, Article 119640  
**DOI:** https://doi.org/10.1016/j.oceaneng.2024.119640

---

## System Description

A **Floating Wind-Wave Platform (FWWP)** is a hybrid offshore energy device that couples a Floating Offshore Wind Turbine (FOWT) with an articulated Wave Energy Converter (WEC) that drives a hydraulic power take-off (PTO) piston. The complete system is a constrained multi-body structure subject to ocean waves, where hydrodynamic interactions among the submerged geometries and the nonlinear mechanical joints must be modelled simultaneously.

The paper's key contribution is a generalised-coordinates formulation that yields **explicit, low-dimensional ODEs** for the multi-body system-suitable for real-time simulation, optimisation, and control synthesis.

---

## Mathematical Model

### Degrees of freedom

Each rigid body *i* has 6 DOF (surge, sway, heave, roll, pitch, yaw). Mechanical constraints (hinges, articulated arms) reduce the total DOF by enforcing geometric equations:

```
g(q) = 0          (holonomic constraint vector)
J(q) q. = 0        (velocity-level Jacobian form)
```

### Equations of motion (generalised coordinates)

Using Lagrangian mechanics with hydrodynamic impulse-response functions:

```
M q̈ + B q. + K q + \int0ᵗ h(t-tau) q.(tau) dtau = F_wave + F_ext + F_constraint
```

where:
- `q` - generalised coordinate vector (reduced DOF after constraints)
- `M` - generalised mass matrix (rigid-body inertia + added mass at inf)
- `B` - linear damping matrix
- `K` - hydrostatic restoring stiffness
- `h(t)` - retardation (impulse-response) kernel from hydrodynamic memory effect
- `F_wave` - first-order wave excitation force
- `F_ext` - external loads (mooring, PTO force)
- `F_constraint` - constraint reaction forces via Lagrange multipliers lambda

### Constraint handling - Gauss Principle

Constraint forces are computed via:

```
J Mⁱ Jᵀ lambda = J Mⁱ (F_wave + F_ext) - J. q.
```

### Hydraulic PTO (power take-off)

The WEC drives a hydraulic piston; the PTO force is:

```
F_PTO = Cpto * xdot_rel        (passive linear damper, simplest model)
F_PTO = -k_spring * x_rel   (spring stiffness variation)
```

where `x_rel` is the relative displacement at the articulated arm.

### Natural frequency (closed-form)

For the simplified two-body case an analytical expression for the natural frequencies of the constrained system is derived and verified numerically.

---

## State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `q` | Generalised position vector | m, rad |
| `q.` | Generalised velocity vector | m/s, rad/s |
| `x_rel` | Relative WEC arm displacement | m |
| `xdot_rel` | Relative WEC arm velocity | m/s |

## Inputs / Actuators

| Signal | Description |
|--------|-------------|
| `F_PTO` | Hydraulic PTO force (controllable) |
| `u_mooring` | Active mooring tension (if actuated) |

## Outputs / Measurements

| Signal | Description |
|--------|-------------|
| `eta` (surge/heave/pitch of FOWT) | Platform motion for fatigue / comfort |
| `P_WEC = F_PTO * xdot_rel` | Instantaneous WEC power |
| `P_wind` | Aerodynamic power of turbine |

---

## Control Objectives

1. **WEC power maximisation** - choose `F_PTO(t)` to maximise mean harvested power subject to stroke and force limits.
2. **FOWT load reduction** - use WEC as an active mass damper to reduce wind-turbine tower loads and accelerations.
3. **Combined optimisation** - jointly maximise renewable energy output while respecting structural fatigue margins.

---

## Relevant Control Methods

| Method | Notes |
|--------|-------|
| **Complex-conjugate control** | Optimal passive PTO for regular waves |
| **Model Predictive Control (MPC)** | Handles stroke/force constraints with wave preview |
| **LQR / LQG** | Linear state-feedback using linearised model around operating point |
| **Reactive control** | Spring–damper parameter scheduling |
| **Reinforcement Learning** | Handles irregular seas without exact wave model |

---

## Key Parameters (from paper)

| Parameter | Value / Range | Description |
|-----------|--------------|-------------|
| Water depth | >= 50 m | Offshore deepwater site |
| FOWT displacement | O(10^3) t | Typical semi-sub FOWT |
| WEC arm length | O(10) m | Articulated arm geometry |
| Wave period | 6–20 s | Typical North Sea spectrum |
| PTO damping `Cpto` | Optimised per sea state | Passive case |

---

## Scenarios

- **Design verification**: low-DOF analytical case for code validation
- **Regular waves**: monochromatic wave with known period and amplitude
- **Irregular waves** (JONSWAP): realistic stochastic sea state
- **Combined wind + wave** loading

---

## Implementation Notes

- The retardation-function integral requires state-augmentation or IRF truncation for real-time control; a Prony-series approximation converts the convolution to a finite-order ODE.
- The constraint Jacobian `J(q)` is configuration-dependent and must be updated at each time step for large-amplitude motions.
- The C++ `lib/` toolbox can contribute: `DiscreteMPC` for PTO force scheduling, `KalmanFilter` / `UKF` for state estimation from noisy wave gauges, `ExtremumSeeker` for gradient-free power optimisation.
