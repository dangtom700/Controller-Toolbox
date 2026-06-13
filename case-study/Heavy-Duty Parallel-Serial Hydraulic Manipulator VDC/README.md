# Heavy-Duty Parallel-Serial Hydraulic Manipulator - Virtual Decomposition Control

## Reference

Goran Petrović and Jouni Mattila (2022). "Mathematical modelling and virtual decomposition control of heavy-duty parallel-serial hydraulic manipulators." *Mechanism and Machine Theory* 170, 104680. https://doi.org/10.1016/j.mechmachtheory.2021.104680

---

## Plant Model

A **heavy-duty hydraulic parallel-serial manipulator** consisting of coupled revolute and prismatic segments driven by hydraulic actuators. The distinguishing feature of this class of machine is that its kinematic chain is neither purely serial nor purely parallel: loop-closure constraints couple the dynamics of parallel sub-chains, while the overall tip motion follows a serial chain. Petrović & Mattila's Virtual Decomposition Control (VDC) framework handles this structure by recursively decomposing the system into virtual stubs at each cut-point, proving stability via a composite Lyapunov function built from per-subsystem virtual stability conditions.

For simulation purposes, the representative two-segment model (one prismatic hydraulic cylinder + one revolute hydraulic rotary actuator in a parallel-serial arrangement) captures the essential closed-chain coupling without the full N-segment complexity.

### Physical Description

- **Revolute segment:** A hydraulic rotary actuator (vane motor or piston + linkage) producing joint torque proportional to differential pressure. The angular position, velocity, and supply-side pressure are the three states per revolute DOF.
- **Prismatic segment:** A double-acting hydraulic cylinder. Piston position, piston velocity, and both chamber pressures form the four states per prismatic DOF.
- **Servo valve:** Proportional directional valve; spool position proportional to command voltage `u_v`; flow Q proportional to `u_v * sqrt(DeltaP)` (Bernoulli-type valve equation).
- **Loop-closure:** A rigid link connecting two joints imposes an algebraic constraint between the joint angles, reducing the system's independent DOF and introducing constraint forces computed from the loop-closure Jacobian.
- **Load:** Payload at the endpoint modelled as a lumped mass `m_L`; gravity vector projects through the kinematic chain to produce joint-space loading.

### State Variables (2-DOF parallel-serial sub-model)

| Symbol | Description | Unit |
|--------|-------------|------|
| `q_r` | Revolute joint angle | rad |
| `dq_r` | Revolute joint angular velocity | rad/s |
| `P_r` | Hydraulic pressure, revolute actuator working chamber | Pa |
| `q_p` | Prismatic joint displacement | m |
| `dq_p` | Prismatic joint velocity | m/s |
| `P_Ap` | Cap-side pressure, prismatic cylinder | Pa |
| `P_Bp` | Rod-side pressure, prismatic cylinder | Pa |

### Governing Equations

**Revolute segment (rotary hydraulic actuator):**
```
J_r * ddq_r = D_m * (P_r - P_return) - tau_fric(dq_r) - tau_load(q_r)
V_r / beta * dP_r/dt = Q_valve(u_r, P_s, P_r) - D_m * dq_r
```
where `D_m` is the motor displacement [m^3/rad], `V_r` is the actuated chamber volume, `beta` is the bulk modulus.

**Prismatic segment (double-acting cylinder):**
```
m_p * ddq_p = A_A * P_Ap - A_B * P_Bp - F_fric(dq_p) - F_load(q)
V_Ap / beta * dP_Ap/dt = Q_A(u_p, P_s, P_Ap) - A_A * dq_p
V_Bp / beta * dP_Bp/dt = -Q_B(u_p, P_Bp) + A_B * dq_p
```

**Valve flow equation (both segments):**
```
Q = Cd * w * x_v(u) * sqrt(2 * |P_s - P_work| / rho) * sign(P_s - P_work)
```

**Loop-closure constraint (parallel sub-chain):**
```
J_lc(q) * dq = 0    [velocity-level]
F_constraint = (J_lc)^T * lambda   [constraint forces via Lagrange multiplier]
```

### Key Parameters

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Bulk modulus | beta | 1.4 GPa | Hydraulic oil stiffness |
| Supply pressure | P_s | 250-350 bar | Hydraulic power unit |
| Motor displacement | D_m | 30-80 cm^3/rev | Rotary actuator |
| Piston area (cap) | A_A | 30-80 cm^2 | Prismatic cylinder |
| Piston area (rod) | A_B | 25-65 cm^2 | Rod-side area |
| Revolute inertia | J_r | 5-50 kg.m^2 | Link + reflected load |
| Prismatic mass | m_p | 20-200 kg | Moving mass |
| Payload mass | m_L | 0-500 kg | Endpoint tool + workpiece |
| Coulomb friction | F_c | 50-500 N | Seal + guide friction |
| Sampling time | Ts | 1-5 ms | Hydraulic bandwidth |

---

## Control Objective

Trajectory tracking of the manipulator's endpoint (or individual joint coordinates) across pick-and-place or continuous-path tasks in the presence of:

1. **Strong kinematic coupling** through loop-closure constraints - controllers must account for constraint forces or treat them as disturbance.
2. **Hydraulic nonlinearity** - valve flow is proportional to `sqrt(DeltaP)`, creating input-gain variation over the operating range.
3. **Payload uncertainty** - tool mass may vary from 0 to 500 kg, shifting the gravity loading significantly.
4. **High-stiffness position hold** - heavy-duty machines must resist large static loads with near-zero position error.

The **Petrović & Mattila (2022)** method uses VDC to decompose the parallel-serial chain into independent subsystems, each proved virtually stable via a segment-level Lyapunov function. The composite Lyapunov function guarantees asymptotic stability of the full coupled system without requiring a global inertia matrix inversion. The paper validates the approach against MATLAB SimMechanics with a multi-segment test manipulator.

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | PID | `DiscretePID` | Kp=200, Ki=50, Kd=10; e = q_ref - q | Per-joint independent; baseline |
| 2 | ADRC | `DiscreteADRC` | omega_o=100, omega_c=30, b0approx =A_A/m_p | ESO absorbs loop-closure coupling and friction; omega_o*Ts<0.5 |
| 3 | SMC | `DiscreteSMC` | c=40, K=300, phi=0.02 m | Sliding surface sigma = dq + c*q_err; compute(y - ref) |
| 4 | LQR | `DiscreteLQR` | Q=diag(1e4,1e4,1,1,...), R=I | ZOH linearised at mid-stroke; coupled joint SS model |
| 5 | LQG | `DiscreteLQG` | Q_w=diag(1e-4,...), R_v=diag(1e-6,...) | Kalman filter for chamber pressures from position + force sensors |
| 6 | MPC | `DiscreteMPC` | Np=15, Nu=4, rho_y=1e5, rho_u=0.01 | Box constraints: P_A \in [P_min, P_s], q_p \in stroke limits |
| 7 | MRAC | `MRACController` | gamma=5, a_m=-20, b_m=20 | Adapts to payload mass variation; compute(y_plant) |
| 8 | L1Adaptive | `L1AdaptiveController` | a_m=-20, b_m=20, omega_c=15 | Fast bandwidth for rapid load changes |
| 9 | FeedbackLinearisation | `FeedbackLinearisationController` | g = A_A*sqrt(P_s-P_A)/(m_p); f = friction_model | Dynamic inversion of hydraulic nonlinearity; inner PD loop |
| 10 | ILC | `ILCController` | Lp=0.5, P-type; trial_length=N | For repetitive pick-and-place; trial-to-trial feedforward buildup |
| 11 | NeuralPID | `NeuralPID` | n_h=8, lr=1e-5, plant_gainapprox =A_A/(m_p*Ts) | Online gain adaptation for joint coupling variation |
| 12 | TubeMPC | `TubeMPC` | Q=I*100, R=0.1, K_tube=LQR gain | Tube accounts for loop-closure constraint-force uncertainty |

---

## Scenarios

| ID | Description | Reference Signal | Load / Stress |
|----|-------------|-----------------|---------------|
| s01_point_to_point | Step to target endpoint; hold for 5 s | q_ref: step from 0 to 60^\circ (revolute), 0.3 m (prismatic) | Nominal payload 100 kg |
| s02_sinusoidal_tracking | Sinusoidal joint trajectory at 0.5 Hz | q_ref = A * sin(2pi*0.5*t) for each joint | Nominal payload; tests bandwidth |
| s03_payload_change | P2P tracking; payload varies mid-motion | Step reference; m_L steps 50 -> 250 kg at t = 5 s | Heavy load insertion/extraction |
| s04_joint_coupling | Coupled motion activating loop-closure | Coordinated q_r and q_p to track Cartesian circle | Nominal payload; tests coupling compensation |
| s05_energy_compare | P2P cycle; measure hydraulic energy E = \int P_s * Q_s dt | Repeated 10-s P2P cycles | Nominal 100 kg; metric: actuator energy |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Hydraulic bandwidth:** The pressure dynamics have time constants on the order of 1-10 ms. Use Ts = 2 ms with RK4 integration to capture the stiff pressure ODE.
- **ADRC omega_o constraint:** With Ts = 2 ms, require `omega_o * Ts < 0.5` -> `omega_o < 250 rad/s`. Use omega_o = 100, omega_c = 30 as safe starting values.
- **Loop-closure DAE reduction:** The prismatic-revolute parallel sub-chain introduces an algebraic constraint. In simulation, apply `dae2ode()` (P1 open item) or pre-solve the constraint analytically: `q_p = f(q_r)` from geometry, reducing the system to a single generalized coordinate with modified effective inertia.
- **Valve saturation:** Clamp valve command `u_v \in [-1, 1]`. Near valve center, the `sqrt(DeltaP)` gain drops sharply; controllers must handle this reduced gain at low-velocity reversals.
- **FeedbackLinearisation `g` field:** `g(x) = Cd * w * x_v_gain * sqrt(P_s - P_A) * A_A / m_p`; evaluate numerically each step as pressures evolve.
- **ILC trial length:** Set to exactly one period of the sinusoidal reference (scenario s02) or the P2P cycle duration (scenarios s01/s04). D-type ILC with Lp = 0.5 should converge in ~15-30 trials.
- **LQR loop-closure coupling:** Linearise the reduced-DOF model (after constraint elimination) at the operating point for DARE. Do not use full unconstrained state matrix - constraint modes are not stabilisable in the unconstrained model.
- **CSV columns:** `t, q_r_ref, q_r, dq_r, q_p_ref, q_p, dq_p, P_Ap, P_Bp, P_r, u_r, u_p, F_ext, energy_J, iae_cumulative`

---

## Status

Spec only - `sim/` not present, not registered, not built.

The key implementation challenge is the loop-closure constraint handling. Options:
1. Analytical constraint elimination (reduces to 1-DOF ODE per parallel sub-chain) - simpler but restricts to fixed-geometry parallel chains.
2. Index-1 DAE formulation using the `DAESystem` struct (P1 open item) - general but requires Newton solve per step.
3. Penalty-based relaxation (add large constraint stiffness) - simple but introduces stiff ODE.

Option 1 is recommended for initial implementation of the spec study.
