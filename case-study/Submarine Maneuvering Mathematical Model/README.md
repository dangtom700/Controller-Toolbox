# Submarine Maneuvering Mathematical Model

## Reference
**Title:** A study on a physical based manoeuvring mathematical model for submarines  
**Authors:** Sungwook Lee, Jin-Hyeong Ahn  
**Journal:** Ocean Engineering, Vol. 311, 2024, Article 118839  
**DOI:** https://doi.org/10.1016/j.oceaneng.2024.118839

---

## System Description

This paper develops and validates a **Karasuno-type physical-based maneuvering model** for submarines, generalising a technique previously applied only to fishing vessels. Submarines operate in full 6-DOF (surge, sway, heave, roll, pitch, yaw) and, unlike surface vessels, must handle **large drift angles and angles of attack** during depth control, emergency rising, and low-speed harbor maneuvers, where Taylor-series based models break down.

The model characterises hydrodynamic forces and moments using **lift, drag, and cross-flow drag** coefficients (physically interpretable) rather than polynomial regression on captive-model coefficients. Validation uses CFD data for the MARIN BB2 benchmark submarine.

---

## Mathematical Model

### 6-DOF equations of motion

```
(m - Xᵤ.) u. = X_hydro + X_prop
(m - Yᵥ.) v. = Y_hydro
(m - Zẇ.) ẇ = Z_hydro + B_z
(Iₓₓ - Kₚ.) ṗ = K_hydro + K_righting
(Iyy - Mq..) q. = M_hydro
(Izz - Nṙ.) ṙ = N_hydro
```

where `u, v, w` are body-frame linear velocities (surge, sway, heave) and `p, q, r` are angular rates (roll, pitch, yaw). Added-mass terms `Xᵤ., Yᵥ., ...` come from potential theory.

### Kinematic equations

```
xdot = u cos(ψ)cos(theta) + v(...) + w(...)
ydot = ...
ż = -u sin(theta) + v sin(phi)cos(theta) + w cos(phi)cos(theta)
phi. = p + (q sin(phi) + r cos(phi)) tan(theta)
theta. = q cos(phi) - r sin(phi)
ψ. = (q sin(phi) + r cos(phi)) / cos(theta)
```

### Karasuno-type hydrodynamic force model

Hydrodynamic forces are decomposed into **lift (L), drag (D), and cross-flow drag (Cd)**:

```
Y_hydro = ½ rho L_pp^2 V^2 [Cy_L(alpha) sin(2alpha) + Cy_D(alpha) sin(alpha)|sin(alpha)|
           + Cy_cf(alpha) sin(alpha)|sin(alpha)|] + Cdelta_r deltar
N_hydro = ½ rho L_pp^3 V^2 [Cn_L(alpha) sin(2alpha) + ... ] + Cdelta_r_n deltar
```

where:
- `alpha` - drift angle (horizontal) or angle of attack (vertical)
- `L_pp` - between-perpendiculars length
- `V` - resultant speed
- `Cy_L, Cy_D, Cy_cf` - lift, drag, cross-flow coefficients (functions of `alpha`)

Vertical-plane forces use the same structure with `beta` (angle of attack).

### Propulsion

```
X_prop = (1 - t_w) T(n, Va)
T = rho n^2 D^4 Kt(J)    where J = Va / (n D)
```

### Control surfaces

Rudder and stern/sail planes modelled as thin foils:

```
Y_rudder = ½ rho V^2 A_r Cy_r(deltar, alpha_r)
M_stern  = ½ rho V^2 A_s Cz_s(deltas, beta_s)
```

---

## State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `u, v, w` | Body-frame linear velocities | m/s |
| `p, q, r` | Angular rates (roll, pitch, yaw) | rad/s |
| `x, y, z` | Earth-frame position | m |
| `phi, theta, ψ` | Euler angles (roll, pitch, yaw) | rad |

**State vector dimension:** 12

## Inputs / Actuators

| Signal | Description | Range |
|--------|-------------|-------|
| `deltar` | Rudder angle | +/-35^\circ |
| `deltas_stern` | Stern plane angle | +/-35^\circ |
| `deltas_sail` | Sail plane angle | +/-35^\circ |
| `n` | Propeller speed | rpm |

## Outputs / Measurements

| Signal | Description |
|--------|-------------|
| `z` | Depth | m |
| `theta` | Pitch angle | rad |
| `ψ` | Heading | rad |
| `u` | Speed | m/s |

---

## Control Objectives

1. **Depth control** - regulate `z(t)` to setpoint with minimal pitch excursion.
2. **Heading control** - track `ψ_ref` during turning maneuvers.
3. **Speed control** - maintain target surge velocity against drag and sea-state variation.
4. **Emergency ascent** - maximum-rate rise while keeping pitch within safe bounds.

---

## Relevant Control Methods

| Method | Notes |
|--------|-------|
| **PID (classical)** | Depth–pitch cascade; industry standard |
| **LQR** | Linear state-feedback at trim condition |
| **MPC** | Handles plane saturation constraints; enables coupled depth + heading |
| **Sliding Mode Control (SMC)** | Robust to large-angle model uncertainty |
| **ADRC** | Treats hydrodynamic nonlinearity as "total disturbance" |
| **MRAC** | Adapts to changing speed/depth/loading conditions |

---

## Key Parameters

| Parameter | Description |
|-----------|-------------|
| `L_pp` | Between-perpendiculars length [m] - MARIN BB2 benchmark |
| `rho` | Water density approx = 1025 kg/m^3 |
| `m` | Submarine mass (neutrally buoyant: m = rho * nabla) |
| `Ixx, Iyy, Izz` | Moments of inertia in body frame |
| Validation basis | MARIN BB2 geometry; CFD data for coefficients |

---

## Scenarios (from paper)

- **Turning circle** - constant rudder angle from straight-ahead
- **Zig-zag maneuver** - alternating rudder for heading overshoot assessment
- **Vertical plane** - stern-plane step for depth change
- **Large drift angle** - static drift up to +/-20^\circ, angle of attack up to +/-15^\circ (outside Taylor validity)

---

## Implementation Notes

- The Karasuno model requires **fewer captive-test matrices** than Taylor-polynomial models: only drift-motion tests are needed; rotation coefficients can be estimated empirically.
- For a C++ case study: a 12-state ODE integrator (4th-order Runge-Kutta) with the Karasuno force equations drives the plant; `DiscreteADRC` or a cascade PID can be the depth controller.
- The `KalmanFilter` / `EKF` in `lib/` can be used for state estimation from depth+pitch sensors and noisy velocity measurements.
