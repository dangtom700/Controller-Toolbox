# Physical System Modeling and Optimized Control of 6-DOF Vessel Motion Simulator

## Reference

Weibin Ma, Peng Wang, Huachao Dong, Xinjing Wang, Hengzhi Fang, and Xianxian Fan (2025). "Physical system modeling and optimized control strategy of 6-DOF vessel motion simulator based on MBD and LBM." *Ocean Engineering* 334, 121595. https://doi.org/10.1016/j.oceaneng.2025.121595

---

## Plant Model

A **6-DOF vessel motion simulator** based on the **Stewart parallel mechanism (6-UPU configuration)**: a fixed base and a moving platform connected by six independently actuated limb actuators (hydraulic linear cylinders), each attached via universal (Hooke) joints at both ends. The simulator replicates sea-wave-induced vessel motions to enable laboratory testing of shipboard equipment under realistic multi-axis excitation. Ma et al. develop an integrated physical system model that couples: (i) a Multi-Body Dynamics (MBD) model in Simscape/Simulink, (ii) an inverse kinematics resolver that maps desired 6-DOF platform pose to individual actuator stroke commands, (iii) a sensor model for pose feedback, and (iv) the control system. CFD simulations via the Lattice-Boltzmann Method (LBM) provide realistic vessel motion inputs at sea states 3-5.

### Physical Description

- **Platform geometry (6-UPU):** Base platform and moving platform are each hexagonal frames. The six limb actuators connect the lower (base) and upper (platform) hinges via Hooke (universal) joints. Actuator `i` spans from base-hinge `A_i` to platform-hinge `B_i`.
- **Inverse kinematics:** Given the desired 6-DOF platform pose `(P, R)` (translation P \in ℝ^3, rotation matrix R \in SO(3)), the actuator stroke change is:
  ```
  DeltaL_i = || R * B_i - A_i + P || - l0
  ```
  where `l0` is the neutral actuator length.
- **Workspace (from paper Table 1):** Surge +/-366 mm, Sway +/-422 mm, Heave 621-985 mm, Roll +/-27^\circ, Pitch +/-23^\circ, Yaw +/-30^\circ.
- **Dynamic equation:** The platform's equation of motion in generalised coordinates:
  ```
  M(q) * ddq + C(q, dq) + G(q) = F
  ```
  where `M` is the total mass/inertia, `C` the centrifugal/coupling term, `G` gravity, and `F` the driving force from the six limb actuators projected via the Jacobian.
- **Shipboard equipment load:** 12,000 N concentrated at the platform centre, dominating the gravitational and inertial loading.
- **CFD vessel inputs:** 6-DOF time-series of vessel position/attitude from LBM simulation (XFlow software). Sea state 3: H=1.25 m, T=3.8 s; sea state 4: H=2.5 m, T=5.4 s; sea state 5: H=4.0 m, T=7.0 s. Heave amplitudes: 0.112 m, 0.208 m, 0.991 m respectively.

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `L_i` (i=1...6) | Instantaneous length of limb actuator i | mm |
| `dL_i` | Velocity of limb actuator i | mm/s |
| `x_p, y_p, z_p` | Platform centre translation (surge, sway, heave) | mm |
| `phi, theta, psi` | Platform orientation (roll, pitch, yaw) | deg |

Each limb actuator is controlled independently; the control variable is the rod displacement error `e_i = L_i_cmd - L_i`.

### Governing Equations

**Inverse kinematics (per limb i):**
```
L_i_cmd = || R(phi,theta,psi) * B_i - A_i + P(x_p,y_p,z_p) ||
DeltaL_i = L_i_cmd - l0
```
where `(B_i)` is the upper hinge in platform frame, `(A_i)` in base frame, `l0` nominal length.

**Actuator dynamics (each rod, simplified 2nd-order hydraulic):**
```
m_rod * ddL_i = F_drive_i(u_i) - k_spring * DeltaL_i - b_damp * dL_i - F_load_i(q, m_platform)
F_drive_i = K_act * u_i   [linearised hydraulic force; u_i \in [-1, 1]]
```

**Platform MBD coupling (virtual work principle):**
```
J_limb^T(q) * F_limb = M(q) * ddq + C(q,dq) + G(q)
```
where `J_limb` is the Jacobian mapping limb forces to platform generalised forces.

### Key Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Base hinge radius | Ra | ~500 mm | Distribution circle radius, lower hinges |
| Platform hinge radius | Rb | ~350 mm | Distribution circle radius, upper hinges |
| Nominal stroke | l0 | ~800 mm (mid-range) | Neutral actuator length |
| Platform mass | m_p | ~500 kg | Moving platform structural mass |
| Equipment load | F_eq | 12,000 N | Shipboard equipment weight |
| Actuator stroke range | DeltaL | 621-985 mm (heave) | Per workspace Table 1 |
| Actuator bandwidth | f_act | ~10 Hz | Hydraulic linear actuator |
| UDP communication delay | tau_comm | 1 ms | Controller-to-actuator latency |
| Sampling time | Ts | 5 ms | Control loop rate |

---

## Control Objective

Reproduce the 6-DOF vessel motion profile (derived from CFD sea-state simulation) on the moving platform with:

1. **Low tracking error** - platform pose must follow the desired wave-induced motion accurately across all sea states; maximum 6-DOF error at sea state 5 should remain below ~58 mm / 0.5^\circ (paper test values).
2. **Smooth actuator response** - avoid overshoot and oscillation in rod displacement that would damage the mechanism or disturb the shipboard equipment under test.
3. **Robustness to load disturbance** - the 12,000 N equipment load introduces coupling between limbs, and any vibration from equipment-under-test must be rejected.
4. **Rapid settling** - for sea state changes or step inputs, the platform must stabilise within 1-2 oscillation periods.

The **Ma et al. (2025)** paper proposes a **GA-fuzzy PID** controller: a Mamdani-style fuzzy PID (error `e` and error rate `ec` -> incremental gains Deltakp, Deltaki, Deltakd) whose 147 fuzzy rule parameters are optimised offline by a Genetic Algorithm to minimise root-mean-square rod displacement error. The GA-FuzzyPID outperforms both PID and hand-tuned FuzzyPID in all six DOFs at sea state 5.

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | PID | `DiscretePID` | Kp=200, Ki=20, Kd=15; e = L_i_cmd - L_i | Per-rod independent; paper baseline |
| 2 | FuzzyPID | `FuzzyPID` | e \in [-3,3], ec \in [-3,3]; Mamdani 7*7 rules; dkp \in [-0.3,0.3] | Hand-tuned rules (Table 7 from paper); paper comparison |
| 3 | ADRC | `DiscreteADRC` | omega_o=60, omega_c=20, b0=K_act/m_rod; omega_o*Ts=0.30<0.5 | ESO treats platform coupling + load as total disturbance per rod |
| 4 | SMC | `DiscreteSMC` | c=15, K=200, phi=0.5 mm | Robust to coupling forces from other rods; compute(y - ref) |
| 5 | LQR | `DiscreteLQR` | Q=diag(1e4,1e4,1,...), R=0.01*I | Linearised 12-state (6 rods * position+velocity) decoupled model |
| 6 | MPC | `DiscreteMPC` | Np=10, Nu=3, rho_y=1e4, rho_u=0.01 | Stroke constraints: L_i \in [stroke_min, stroke_max]; per-rod model |
| 7 | MRAC | `MRACController` | gamma=2, a_m=-15, b_m=15 | Adapts to equipment load variation and payload shifts; compute(y_plant) |
| 8 | L1Adaptive | `L1AdaptiveController` | a_m=-15, b_m=15, omega_c=10 | Fast adaptation for sudden load changes when equipment activates |
| 9 | GainScheduled | `GainScheduledController` | Schedule on heave stroke |DeltaL_heave|; 3 gain sets for calm/moderate/extreme | Reduced integral gain in large-amplitude sea state 5 to prevent windup |
| 10 | TubeMPC | `TubeMPC` | Q=1e4*I, R=0.01, K_tube=LQR gain | Tube accounts for inter-limb coupling forces as bounded disturbance |
| 11 | NeuralPID | `NeuralPID` | n_h=8, lr=5e-6, plant_gainapprox =K_act*Ts/m_rod | Online gain adaptation across sea states without manual retuning |
| 12 | ScenarioMPC | `ScenarioMPC` | Np=10, Nu=3, N_samples=20, Sigma_w=wave noise | Averages optimisation over Gaussian actuator-force uncertainty scenarios |

---

## Scenarios

| ID | Description | Reference Signal | Load / Stress |
|----|-------------|-----------------|---------------|
| s01_sea_state_3 | Replicate sea-state-3 vessel motion | 6-DOF CFD time-series: heave 0.112 m, pitch 1.18^\circ, roll 0.148^\circ | Equipment load 12 kN; tests low-sea-state tracking |
| s02_sea_state_4 | Replicate sea-state-4 vessel motion | 6-DOF CFD: heave 0.208 m, pitch 2.27^\circ, roll 0.307^\circ | Same equipment load; medium sea state |
| s03_sea_state_5 | Replicate sea-state-5 vessel motion (most extreme) | 6-DOF CFD: heave 0.991 m, pitch 4.32^\circ, roll 0.662^\circ | Most demanding; rod stroke near limits |
| s04_harmonic_heave | Pure sinusoidal heave, +/-300 mm at 0.3 Hz | `z_ref(t) = 300 * sin(2pi * 0.3 * t)` mm; other DOF = 0 | No equipment load; isolates heave performance |
| s05_compound_6dof | Simultaneous 6-DOF compound motion | Superposition of individual DOF sinusoids at different frequencies | Equipment load 12 kN; tests cross-coupling and load disturbance |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Per-rod control architecture:** The Stewart mechanism is controlled in rod-space: each controller receives one rod's displacement error `e_i = L_i_cmd - L_i` and outputs one driving force `F_i`. The 6 controllers (one per rod) run independently in the simplified model; coupling appears only through the platform MBD model.
- **Inverse kinematics at every step:** Compute `L_i_cmd(t)` from the CFD reference pose using the closed-form inverse kinematics (Eq. 2 from paper) at each sample step. This does not require online Newton iteration.
- **Communication delay:** The 1 ms UDP delay in the physical system is significant at Ts = 5 ms (one-fifth of the sample period). Include as a one-sample `ComputationalDelayWrapper` on the controller output for realistic comparison.
- **ADRC omega_o constraint:** With Ts = 5 ms, require `omega_o * Ts < 0.5` -> `omega_o < 100 rad/s`. Use omega_o = 60, omega_c = 20.
- **FuzzyPID inner bounds:** Fundamental domains of error `e` and rate `ec` are +/-3 (normalised); output deltas dkp \in [-0.3, 0.3], dki \in [-0.06, 0.06], dkd \in [-0.6, 0.6]. The centroid defuzzification method is used. The hand-tuned rule table is documented as paper Table 7.
- **MPC stroke constraints:** Hard constraints on `L_i \in [L_min_i, L_max_i]` based on the workspace limits. For heave: `z_p \in [621, 985]` mm. Other DOF limits convert to rod-space constraints via the forward kinematics Jacobian.
- **Metric:** 6-DOF platform pose RMS error (mm for translation, deg for rotation); also report per-rod RMS displacement error for direct comparison with paper Table 9.
- **CSV columns:** `t, z_ref, z_p, phi_ref, phi, theta_ref, theta, psi_ref, psi, x_p, y_p, L1, L2, L3, L4, L5, L6, u1, u2, u3, u4, u5, u6, iae_cumulative`

---

## Status

Spec only - `sim/` not present, not registered, not built.

The paper's full physical system model is implemented in MATLAB Simscape/Simulink with a 1 ms CFD-driven input loop and hardware-in-the-loop test rig. For the Python-only implementation, the recommended approach is to (i) analytically compute the inverse kinematics at each step to get 6 desired rod lengths, (ii) simulate each rod as a second-order hydraulic actuator driven by its controller, and (iii) use the forward kinematics Jacobian to propagate rod forces back to platform pose for the error signal. This approximation ignores the platform-limb coupling but captures the single-rod dynamics and controller comparison faithfully.
