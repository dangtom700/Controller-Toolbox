# Adaptive Kinematic and Stiffness Control of a Hybrid-Driven Soft Manipulator

## Reference

Wenhao Fu, Pengbo Liu, Yang Li, Xin Li, and Zhongbo Sun (2025). "Adaptive kinematic and stiffness control of a hybrid-driven soft robot for enhanced interactions under external loads." *Results in Engineering* 28, 107955. https://doi.org/10.1016/j.rineng.2025.107955

---

## Plant Model

A **two-segment hybrid-driven soft manipulator** that combines tendon-driven bending (for shape control) with pneumatic pressurisation (for active stiffness modulation). Each segment is modelled using the **Piecewise Constant Curvature (PCC)** kinematic framework, where the segment deforms into an arc of constant curvature κ whose magnitude is determined by the net tendon tension and resisted by the segment's pressure-dependent bending stiffness.

Fu et al. introduce the **Weighted Bias Least Squares (WBLS)** algorithm to predict the minimum driving pressure `P_min` required to maintain a desired curvature against an external load, then use an adaptive admittance-style controller to regulate both the endpoint trajectory and the effective stiffness by coordinating the two actuation channels.

### Physical Description

- **Tendon channel:** Two antagonistic tendons per segment apply bending moments about the segment's neutral axis. Net tendon force `DeltaT = T_flex - T_ext` bends the segment; tension is bounded below by preload `T_min > 0` to avoid slack.
- **Pneumatic channel:** Internal air pressure `P_pneu` radially inflates the silicone body, increasing the flexural stiffness `K_s(P_pneu)`. Pressure range 0.9-1.5 bar gives a 3.6* stiffness variation.
- **PCC kinematics:** The arc of curvature κ (rad/m) maps to tip position and orientation via the standard PCC forward kinematics. The length of each segment is `L = 150 mm`.
- **External load:** A lumped mass `m_L` attached to the distal tip exerts a gravitational torque that shifts the equilibrium curvature. Tested experimentally at 0 g, 100 g, and 200 g.
- **WBLS predictor:** At each step, WBLS solves a weighted least-squares problem using a library of curvature-pressure-load observations to predict `P_min(κ_des, m_L_est)`, reducing steady-state stiffness error under varying loads.

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `kappa_1` | Curvature of segment 1 | rad/m |
| `kappa_2` | Curvature of segment 2 | rad/m |
| `dkappa_1` | Curvature rate, segment 1 | rad/(m.s) |
| `dkappa_2` | Curvature rate, segment 2 | rad/(m.s) |
| `P_1` | Pneumatic pressure, segment 1 | bar (gauge) |
| `P_2` | Pneumatic pressure, segment 2 | bar (gauge) |
| `x_tip, y_tip` | Tip Cartesian position (2D plane) | mm |

### Governing Equations

**PCC curvature dynamics (per segment i):**
```
I_s * d^2kappa_i/dt^2 = tau_tendon_i(T_i) - K_s(P_i) * kappa_i - B_s * dkappa_i - tau_gravity_i(kappa, m_L)
```
where `I_s` is the segment's second moment of area * density (effective rotational inertia), `B_s` is damping.

**Stiffness-pressure coupling:**
```
K_s(P_i) = K_s0 + k_P * P_i       [linearised around P_0 = 1.2 bar]
K_s0 approx = 0.05 N.m/rad,  k_P approx = 0.08 N.m/(rad.bar)
```

**Pneumatic pressure dynamics:**
```
(V_0 / (gamma * P_atm)) * dP_i/dt = Q_valve_i(u_P_i) - kappa_i * dkappa_i * V_rate_coeff
```
(first-order pressure build-up; Q_valve is proportional to command `u_P_i` with valve bandwidth ~5 Hz)

**PCC forward kinematics (two segments in series):**
```
x_tip = L * (sin(kappa_1*L)/(kappa_1*L)) + rotation(kappa_1*L) * [L*sin(kappa_2*L)/(kappa_2*L)]
y_tip = L * (1 - cos(kappa_1*L))/(kappa_1*L) + ...
```
(standard PCC composition formula; reduce to straight-segment case when kappa -> 0)

**WBLS minimum pressure prediction:**
```
P_min = W * [kappa_des, m_L_est, 1]^T         [linear basis with load-weighted Gram matrix W]
```

### Key Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Segment length | L | 150 mm | Each segment |
| Nominal stiffness | K_s0 | ~0.05 N.m/rad | At P = 0 (zero gauge) |
| Stiffness pressure gain | k_P | ~0.08 N.m/(rad.bar) | From characterisation data |
| Pressure range | P_pneu | 0.9-1.5 bar (gauge) | ~3.6* stiffness variation |
| Tendon preload | T_min | 0.5 N | Prevents slack |
| Max tendon tension | T_max | 20 N | Motor stall limit |
| Tested payload | m_L | 0, 100, 200 g | Distal tip mass |
| Effective segment inertia | I_s | ~5*10^-^5 kg.m^2 | From silicone density + geometry |
| Sampling time | Ts | 20 ms (50 Hz) | Controller update rate |

---

## Control Objective

Simultaneously regulate the **endpoint trajectory** (tip Cartesian position) and the **effective structural stiffness** (pneumatic channel pressure) of the two-segment soft manipulator under varying external loads, achieving:

1. **Trajectory tracking** - follow prescribed tip paths (spiral, circle, P2P) with bounded position error despite the nonlinear PCC kinematics and gravity-induced curvature shifts.
2. **Stiffness regulation** - maintain a commanded stiffness target `K_des` by controlling `P_pneu`, enabling the robot to stiffen for contact tasks and soften for compliant interaction.
3. **Load adaptation** - detect and compensate for unknown payload mass changes (0-200 g range) using WBLS-based pressure feedforward, preventing the tip from drooping below the reference path.

The **Fu et al. (2025)** approach coordinates WBLS minimum-pressure prediction with an admittance-style feedback controller on tip position error, validated experimentally for spiral tracking and apple pick-and-place tasks. The paper proposes a single integrated architecture; the controller roster below surveys alternative approaches for comparison.

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | PID | `DiscretePID` | Kp=0.5, Ki=0.1, Kd=0.02; e = kappa_ref - kappa | Independent curvature loops; gravity modelled as disturbance |
| 2 | ADRC | `DiscreteADRC` | omega_o=8, omega_c=3, b0approx =1/(I_s); omega_o*Ts=0.16<0.5 | ESO absorbs gravity + stiffness-pressure coupling as total disturbance |
| 3 | SMC | `DiscreteSMC` | c=5, K=0.1, phi=0.005 | Sliding surface on curvature error; robust to payload uncertainty; compute(y - ref) |
| 4 | MRAC | `MRACController` | gamma=2, a_m=-3, b_m=3 | Adapts curvature loop gain as effective stiffness K_s changes with pressure; compute(y_plant) |
| 5 | L1Adaptive | `L1AdaptiveController` | a_m=-3, b_m=3, omega_c=2 | Fast stiffness-change adaptation; decoupled low-pass filter for each segment |
| 6 | FeedbackLinearisation | `FeedbackLinearisationController` | g = 1/I_s; f = -K_s*kappa/I_s - B_s*dkappa/I_s - g_gravity/I_s | Inverts PCC dynamics; inner PD controller in curvature space |
| 7 | ILC | `ILCController` | Lp=0.5, P-type; trial_length=N_spiral | Learns curvature feedforward for repeated spiral trajectory; converges in ~10 trials |
| 8 | NeuralPID | `NeuralPID` | n_h=6, lr=5e-6, plant_gainapprox =Ts/I_s | Online gain tuning to compensate varying stiffness across pressure range |
| 9 | CBFSafetyFilter | `CBFSafetyFilter` | h = P_max - P_pneu; g=1, alpha=2; nominal = PID | Prevents over-pressurisation; barrier function on pneumatic pressure limit |
| 10 | GainScheduled | `GainScheduledController` | Schedule on m_L_est \in {0g, 100g, 200g}; 3 PID sets | PID gains designed per payload level; bumpless transfer at threshold |
| 11 | MPC | `DiscreteMPC` | Np=10, Nu=3, rho_y=100, rho_u=1; P_pneu \in [0.9, 1.5] | Linearised PCC model; pressure constraint as hard MPC bound |
| 12 | DynaController | `DynaController` | wraps PID; n_collect=50, n_refit_every=25 | MBRL: SINDy error-dynamics identification from curvature history; warmup ~1000 s |

---

## Scenarios

| ID | Description | Reference Signal | Load / Stress |
|----|-------------|-----------------|---------------|
| s01_spiral_no_load | Spiral endpoint trajectory; 3 revolutions over 30 s | Archimedean spiral r(t) = 0.1*t/30, theta = 6pi*t/30 in Cartesian [m] | m_L = 0 g |
| s02_spiral_100g | Same spiral; 100 g tip payload | Same spiral reference | m_L = 100 g; gravity droops trajectory |
| s03_spiral_200g | Same spiral; 200 g tip payload | Same spiral reference | m_L = 200 g; heaviest tested condition |
| s04_stiffness_sweep | Circle trajectory; P_des ramps 0.9 -> 1.5 bar | Endpoint circle r=40 mm at 0.2 Hz; K_des ramp | m_L = 100 g; tests stiffness regulation decoupled from trajectory |
| s05_pick_and_place | P2P between 3 waypoints; gentle contact force at endpoint | Waypoints [0, 0], [0.1, 0.05], [0.08, 0.12] m; dwell 2 s at each | m_L varies 0 -> 100 g at second waypoint (tool pickup) |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **PCC kinematics singularity:** When curvature κ -> 0 (straight segment), the PCC formulas have a 0/0 form for `sin(κL)/(κL)`. Use the limit expansion: `x_tip -> L + O(κ^2)`. Guard with `if |kappa| < 1e-6: use Taylor expansion`.
- **WBLS implementation:** The WBLS predictor requires an offline characterisation dataset (curvature * pressure * load observations). For simulation, substitute with the linearised model `P_min = K_s_inv * tau_gravity` to approximate the WBLS output without the dataset.
- **Stiffness channel decoupling:** In the simplified simulation, treat the pneumatic channel as a slow (5 Hz bandwidth) first-order lag. The trajectory controller runs faster (50 Hz) and sees a slowly varying stiffness.
- **ADRC omega_o constraint:** With Ts = 20 ms, require `omega_o * Ts < 0.5` -> `omega_o < 25 rad/s`. Use omega_o = 8, omega_c = 3.
- **Gravity torque:** `tau_gravity_i = m_L * g * L_moment_arm(kappa_1, kappa_2, i)` via PCC geometry. For segment 1, moment arm depends on both curvatures; for segment 2, only on kappa_2. Compute analytically from PCC composition.
- **MRAC sign convention:** Soft robot has a positive curvature gain (more tendon tension -> larger curvature). Use `set_reference(kappa_ref)` then `compute(kappa_plant)` - NOT `compute(kappa_ref - kappa)`.
- **ILC trial definition:** One trial = one full spiral revolution (10 s at 0.1 Hz). Store kappa error history per trial; apply P-type correction with Lp = 0.5.
- **CSV columns:** `t, kappa1_ref, kappa1, kappa2_ref, kappa2, P1, P2, T1, T2, x_tip, y_tip, x_ref, y_ref, pos_err_mm, iae_cumulative`

---

## Status

Spec only - `sim/` not present, not registered, not built.

The key implementation challenge is characterising the PCC curvature dynamics (stiffness `K_s` vs. pressure `P_pneu`) from the paper's experimental data. The paper provides a characterisation plot but does not tabulate the parameters explicitly. Recommended approach: fit `K_s(P) = K_s0 + k_P * P` to the reported 3.6* stiffness ratio across the 0.9-1.5 bar range as the initial parameter estimate; refine with MRAC or online identification in scenario s04.
