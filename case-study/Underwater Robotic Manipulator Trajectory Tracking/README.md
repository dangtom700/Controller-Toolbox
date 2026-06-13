# Implicit Rigid Tube MPC for Underwater Robotic Manipulator Trajectory Tracking

## Reference

Boyang Xu, Wenlong Ding, Yongping Hao, Hao Fang, and Mao Ye (2025). "Implicit rigid tube model predictive control for underwater manipulators with adaptive sliding mode strategy." *Ocean Engineering* 324, 120682. https://doi.org/10.1016/j.oceaneng.2025.120682

---

## Plant Model

A **3-DOF underwater hydraulic manipulator** rated to 300 m depth, mounted on a remotely operated vehicle (ROV). The manipulator arm is subject to significant hydrodynamic drag and added-mass effects from the surrounding seawater, which Xu et al. model via Morrison's equation applied to each link treated as a cylinder. The paper addresses trajectory tracking via the **Implicit Rigid Tube MPC (IRTMPC)** method combined with an **Adaptive Sliding Mode Control (ASMC)** auxiliary disturbance-compensation law. A **Least Squares Input-Output (LS-IO)** parameter identification method avoids the need to differentiate acceleration signals, making it suitable for real ROV hardware.

### Physical Description

- **Configuration:** 3-revolute-DOF (shoulder, elbow, wrist) planar arm; each joint driven by a hydraulic rotary actuator.
- **Depth rating:** 300 m; water column pressure applied to actuator seals and structural elements.
- **Drag model (Morrison's equation per link):** Each link is approximated as a cylinder of diameter `D_i` and length `L_i`. The drag force and added mass on link i are:
  ```
  F_drag_i = 0.5 * rho * C_D * D_i * L_i * |dq_i| * dq_i   (projected to joint space via Jacobian)
  M_added_i = rho * C_M * pi/4 * D_i^2 * L_i                (added mass contribution)
  ```
- **LS-IO identification:** The combined inertia matrix `M(q) + M_added(q)` and the friction/gravity parameters are identified offline from joint velocity/torque histories without requiring double differentiation.

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `q1, q2, q3` | Joint angles (shoulder, elbow, wrist) | rad |
| `dq1, dq2, dq3` | Joint angular velocities | rad/s |

Full state: `x = [q1, q2, q3, dq1, dq2, dq3]^T` (6 states)

### Governing Equations

**Rigid-body dynamics with hydrodynamics:**
```
[M(q) + M_added(q)] * ddq + C(q, dq) * dq + G(q) + F_drag(q, dq) + F_fric(dq) = tau
```
where:
- `M(q)` - standard rigid-body inertia matrix (DH parameters below)
- `M_added(q)` - position-dependent added mass from Morrison's model
- `C(q,dq)` - Coriolis and centrifugal matrix
- `G(q)` - gravity minus buoyancy (net gravity in seawater)
- `F_drag` - velocity-squared hydrodynamic drag (projected to joint torques via Jacobian)
- `F_fric` - viscous + Coulomb friction in actuator seals and joints
- `tau` - actuator torque vector (control input)

**D-H Parameters (from paper Table / geometric model):**

| Link | a (m) | alpha (rad) | D_link (m) | Description |
|------|-------|-------------|------------|-------------|
| L1 | 0.15 | -pi/2 | 0.145 | Shoulder (largest cross-section) |
| L2 | 0.52 | 0 | 0.051 | Elbow (main structural link) |
| L3 | 0.42 | pi | 0.030 | Wrist (distal, smallest) |

(d-offsets and theta-offsets set to zero for the planar configuration modelled in simulation)

**Morrison's coefficients (seawater rho = 1025 kg/m^3):**
```
C_D = 1.0   (drag coefficient, cylindrical body in turbulent flow)
C_M = 2.0   (added-mass coefficient, includes potential-flow C_M = 1 + 1 for inertia)
```

**IRTMPC formulation (paper's method):**
```
min_{U} sum J(x_k+i, u_k+i)
s.t. x_{k+1} = A*x_k + B*u_k + disturbance          [linearised ZOH model]
     ||x - x_nom|| <= rho_tube                        [implicit rigid tube constraint]
     u \in [u_min, u_max]
```
The implicit rigid tube size `rho_tube` is set to enclose the ASMC auxiliary law's residual tracking error, removing the need to pre-compute the explicit tube polytope.

### Key Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Water density | rho | 1025 kg/m^3 | Seawater at 300 m |
| Gravity (net) | g_net | 9.81 * (1 - rho_body/rho) | Effective gravity after buoyancy correction |
| Drag coefficient | C_D | 1.0 | Per link, cylindrical model |
| Added-mass coeff. | C_M | 2.0 | Per link |
| Link masses | m1,m2,m3 | ~3-8 kg | Including housing |
| Link inertias | I1,I2,I3 | ~0.05-0.4 kg.m^2 | About joint axis |
| Actuator torque limit | tau_max | +/-150 N.m | Per joint |
| MPC horizon | N | 10 | Prediction steps |
| MPC output weight | Q | I⁶ | State tracking |
| MPC input weight | R | 0.1 * I⁶ | Control effort |
| Sampling time | delta_t | 0.01 s (100 Hz) | Controller rate |

---

## Control Objective

Track a desired joint-space trajectory `q_ref(t)` - typically sinusoidal excitation trajectories for parameter identification, or circular Cartesian endpoint paths for inspection tasks - in the presence of:

1. **Hydrodynamic drag** growing quadratically with joint velocity at depth.
2. **Added mass** that increases the effective inertia beyond the rigid-body model, especially at higher joint speeds.
3. **Model uncertainty** from imprecise buoyancy estimation and variation in drag coefficients with joint angle (changing projected area).
4. **Disturbance from ROV thruster wash** and sea current acting on the manipulator.

The **Xu et al. (2025)** IRTMPC+ASMC achieves MSE reduction of ~8.91% over IRTMPC alone and ~18% over standard MPC in simulation, with field experiments confirming practical deployability. The ASMC auxiliary law runs in parallel with the MPC to cancel residual disturbances within the tube.

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | OpenLoop | - | Pre-computed feedforward torques from inverse dynamics | Gravity + inertia compensation only; no feedback |
| 2 | PID | `DiscretePID` | Kp=50, Ki=5, Kd=8; e = q_ref - q | Per-joint independent; drag ignored |
| 3 | ADRC | `DiscreteADRC` | omega_o=30, omega_c=10, b0approx =1/I_eff; omega_o*Ts=0.30<0.5 | ESO absorbs drag + added-mass coupling; per joint |
| 4 | SMC | `DiscreteSMC` | c=10, K=80, phi=0.01 rad | Reaching law accounts for bounded drag disturbance; compute(y - ref) |
| 5 | LQR | `DiscreteLQR` | Q=diag(100,100,100,1,1,1), R=0.01*I | ZOH linearised at mid-stroke, nominal buoyancy |
| 6 | LQG | `DiscreteLQG` | Q_w=diag(0.01,...), R_v=diag(1e-4,...) | Kalman filter for joint velocities (noisy encoder + actuator pressure) |
| 7 | MPC | `DiscreteMPC` | Np=10, Nu=4, rho_y=100, rho_u=0.01 | Paper's baseline; ZOH linearised model; torque saturation constraint |
| 8 | TubeMPC | `TubeMPC` | Q=100*I, R=0.01, K_tube=LQR gain | Models drag uncertainty as bounded additive disturbance; explicit tube |
| 9 | MRAC | `MRACController` | gamma=10, a_m=-10, b_m=10 | Adapts to unknown drag scaling and buoyancy offset; compute(y_plant) |
| 10 | L1Adaptive | `L1AdaptiveController` | a_m=-10, b_m=10, omega_c=8 | Fast adaptation for current-induced disturbance during ROV translation |
| 11 | FeedbackLinearisation | `FeedbackLinearisationController` | g = M_eff_inv; f = Coriolis + G + F_drag_estimated | Full inverse dynamics; relies on LS-IO identified parameters for M and G |
| 12 | ILC | `ILCController` | Lp=0.6, P-type; trial_length=N | For repeated inspection sweeps along the same joint trajectory; converges drag compensation trial-to-trial |

---

## Scenarios

| ID | Description | Reference Signal | Load / Stress |
|----|-------------|-----------------|---------------|
| s01_joint_step | Simultaneous step in all three joints | q_ref: [0,0,0] -> [pi/4, pi/3, -pi/6] rad | Nominal depth 300 m, still water |
| s02_sine_excitation | Sinusoidal excitation for parameter identification (paper trajectory) | q_ref_i = A_i * sin(w1*t) + B_i * sin(w2*t); A1=0.4, A2=0.3, A3=0.2 rad | Still water, nominal drag |
| s03_deep_current | Sine tracking under 0.5 m/s horizontal current (increased effective drag) | Same sine as s02 | Current adds C_D * rho * v_c^2 / 2 offset drag torque at all joints |
| s04_endpoint_circle | Cartesian circle at distal tip using inverse kinematics references | 3D circle r=0.1 m at 0.05 Hz in YZ plane | Nominal depth, payload 0 kg |
| s05_payload | P2P tracking; 5 kg inspection tool attached to wrist | q_ref: step [0,0,0] -> [pi/4, pi/4, -pi/4] | Mass added at L3 tip: modifies G and M; tests parameter adaptation |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **RK4 integration:** The hydrodynamic dynamics are stiff near joint-velocity reversals (drag term changes sign). Use RK4 with Ts = 10 ms (delta_t from paper); 4 substeps of 2.5 ms each for numerical stability at high drag.
- **ADRC b0 estimate:** Use `b0 approx = 1 / I_eff_i` where `I_eff_i = M_ii + M_added_ii` evaluated at the nominal operating point. The ESO will compensate for the remainder.
- **Buoyancy correction:** In seawater, each link experiences an upward buoyancy force `F_buoy_i = rho * g * V_link_i`. Subtract from gravity vector: `G_net(q) = G_gravity(q) - J^T * F_buoy_world`. This significantly changes the gravity bias at horizontal configurations.
- **Morrison's drag in joint space:** Project drag forces to joint torques via the geometric Jacobian: `tau_drag = J^T(q) * F_drag_cartesian`. Since drag acts along each link's local velocity, this requires the per-link Jacobians, not just the end-effector Jacobian.
- **LS-IO identification (simplified):** For simulation, use the true plant parameters rather than identification. For a more faithful implementation, generate noisy joint position/torque data from an open-loop sweep and apply `RecursiveLeastSquares` to identify a linear-in-parameters model of the form `Y(q,dq,ddq) * theta = tau`.
- **SMC compute convention:** `compute(y - ref)` i.e. pass `q - q_ref` (not the negative). See CLAUDE.md sign conventions.
- **MRAC convention:** `set_reference(q_ref)` then `compute(q_plant)` - NOT `compute(q_ref - q)`.
- **CSV columns:** `t, q1_ref, q1, q2_ref, q2, q3_ref, q3, dq1, dq2, dq3, tau1, tau2, tau3, F_drag_norm, iae_cumulative`

---

## Status

Spec only - `sim/` not present, not registered, not built.
