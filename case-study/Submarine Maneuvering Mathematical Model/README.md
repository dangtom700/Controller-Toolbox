# Submarine Maneuvering Mathematical Model

## Reference

**Title:** A study on a physical based manoeuvring mathematical model for submarines  
**Authors:** Sungwook Lee, Jin-Hyeong Ahn  
**Journal:** Ocean Engineering, Vol. 311, 2024, Article 118839  
**DOI:** https://doi.org/10.1016/j.oceaneng.2024.118839

---

## Overview

This case study simulates **heading and depth autopilots** for the MARIN BB2 benchmark submarine using the Karasuno physical-based hydrodynamic force model.  10 controllers * 5 scenarios = **50 runs**. All results are logged as CSV files to `logs/`.

The plant is a **4-DOF decoupled model**: the horizontal plane (sway `v`, yaw rate `r`, heading `ψ`) and vertical plane (heave `w`, pitch rate `q`, pitch `theta`, depth `z`) are integrated simultaneously but their hydrodynamic coupling is approximated via centripetal kinematic terms only. Earth-frame position `(x, y)` is tracked for trajectory visualisation. This simplified model is sufficient to assess autopilot performance without the full 6-DOF nonlinear complexity.

---

## MARIN BB2 Plant Parameters

The model uses the model-scale MARIN BB2 geometry (scale 1:18.348) operating at design speed:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `L` | 3.826 m | Between-perpendiculars length (model scale) |
| `m` | 737.0 kg | Displaced mass (neutrally buoyant) |
| `Iyy` | 300.0 kg.m^2 | Pitch moment of inertia |
| `Izz` | 300.0 kg.m^2 | Yaw moment of inertia |
| `U` | 1.4 m/s | Design forward speed |
| `BG` | 0.100 m | Vertical separation of buoyancy and gravity centres |
| `m22` | 1000.0 kg | Sway added mass (circular-cylinder potential theory) |
| `m33` | 1000.0 kg | Heave added mass |
| `m55` | 1250.0 kg.m^2 | Pitch added moment of inertia |
| `m66` | 1250.0 kg.m^2 | Yaw added moment of inertia |

### Re-dimensionalised Hydrodynamic Derivatives (SI units, Table 11)

Derivatives are re-dimensionalised from non-dimensional form as:  
`F_dim = F'_coeff * 0.5 * rho * L^n * U` (n=2 for force, n=3 for moment, n=4 for yaw damping, etc.)

**Horizontal plane:**

| Derivative | Value [SI] | Physical interpretation |
|------------|-----------|------------------------|
| `Yv` | -586 N.s/m | Sway damping (linear) |
| `Yr` | -346 N.s | Sway-yaw cross-coupling |
| `Yvv` | -734 N/m^2 | Cross-flow sway drag |
| `Yrr` | -209 N.s^2/m | Cross-flow yaw drag on sway |
| `Nv` | -696 N.s | Yaw moment from sway |
| `Nr` | -895 N.m.s | Yaw damping (stable: Nr < 0) |
| `Nvv` | +391 N/m | Cross-coupling |
| `Nrr` | +378 N.m.s^2 | Cross-coupling |
| `Ydr` | -200 N/rad | Rudder sway force |
| `Ndr` | +1014 N.m/rad | Rudder yaw moment (Ndr > 0 -> positive deltar = starboard turn) |

**Vertical plane:**

| Derivative | Value [SI] | Physical interpretation |
|------------|-----------|------------------------|
| `Zw` | -290 N.s/m | Heave damping |
| `Zq` | +453 N.s | Heave-pitch cross-coupling |
| `Zww` | -405 N/m^2 | Cross-flow heave drag |
| `Mw` | +350 N.s | Pitch from heave |
| `Mq` | -877 N.m.s | Pitch damping (stable: Mq < 0) |
| `Mqq` | -93 N.m.s^2 | Nonlinear pitch damping |
| `Zds` | -200 N/rad | Stern plane heave force |
| `Mds` | +400 N.m/rad | Stern plane pitch moment |

### Equations of Motion

**Horizontal plane** (centripetal coupling from body-frame kinematics: `Y = m(v. + Ur)`):

```
(m + m22).v. = Yv.v + Yr.r + Yvv.v|v| + Yrr.r|r| + Ydr.deltar + dist_Y - m.U.r
(Izz + m66).ṙ = Nv.v + Nr.r + Nvv.v|v| + Nrr.r|r| + Ndr.deltar
ψ. = r
```

**Vertical plane** (centripetal coupling: `Z = m(ẇ - Uq)`):

```
(m + m33).ẇ = Zw.w + Zq.q + Zww.w|w| + Zds.deltas + m.U.q
(Iyy + m55).q. = Mw.w + Mq.q + Mqq.q|q| + Mds.deltas - m.g.BG.sin(theta)
theta. = q
ż = U.sin(theta) - w.cos(theta)
```

**Earth-frame position:**

```
xdot = U.cos(ψ) - v.sin(ψ)
ydot = U.sin(ψ) + v.cos(ψ)
```

**Stability analysis at design speed (linearised):**

- Horizontal eigenvalues: lambda1 approx = +0.15 s^-^1 (mildly unstable yaw, tau approx = 6.6 s), lambda2 approx = -0.80 s^-^1 (stable sway)
- Vertical eigenvalues: all stable (BG = 0.1 m restoring moment dominates)
- All controllers must actively stabilise the yaw channel

**Control surface saturation:** +/-35^\circ = +/-0.611 rad (rudder and stern planes).

---

## Cascade Depth Control Architecture

All 10 controllers share the same **two-loop depth channel**:

```
z_ref --► [Outer P: Kz=0.05] --► theta_ref (clamped +/-10^\circ) --► [Inner PID] --► deltas --► Plant
                                                                    ▲
                                                               theta feedback
```

- Outer P gain `Kz = 0.05` maps depth error (m) to pitch reference (rad), clamped to +/-0.175 rad
- Inner PID: `Kp=0.4, Ki=0.05, Kd=0.5, N=5`, limits +/-0.611 rad

Each controller overrides only the **heading channel** (deltar), while the depth channel above runs identically for all 10.

---

## Controller Roster

| # | Name | Class | Heading Strategy | Key Parameters |
|---|------|-------|-----------------|----------------|
| 1 | `PID_ZN` | `PIDZNCtrl` | Discrete PID (Ziegler-Nichols aggressive) | Kp=3.0, Ki=0.3, Kd=8.0 |
| 2 | `PID_Cons` | `PIDConsCtrl` | Discrete PID (conservative) | Kp=1.5, Ki=0.1, Kd=5.0 |
| 3 | `MPC` | `MPCHeadingCtrl` | DiscreteMPC on linearised 3-state heading model [v, r, ψ] | Np=20, Nc=5, rho_y=10, rho_u=0.01 |
| 4 | `LQR` | `LQRHeadingCtrl` | DiscreteLQR full-state feedback [v, r, ψ] | Q=diag(0.1,0.01,100), R=0.001 |
| 5 | `ADRC` | `ADRCHeadingCtrl` | DiscreteADRC - treats yaw nonlinearity as total disturbance | b0=0.654, omegac=0.5, omegao=2.0 (omegao.Ts=0.10 < 0.5 (check)) |
| 6 | `SMC` | `SMCHeadingCtrl` | DiscreteSMC with boundary layer | ce=1.0, cde=0.015, K=1.5, phi=0.1 |
| 7 | `MRAC` | `MRACHeadingCtrl` | MRACController - adapts to speed/depth changes | am=0.70, bm=0.30, gammar=gammay=0.002 |
| 8 | `L1` | `L1HeadingCtrl` | L1AdaptiveController - fast adaptation with LP filter | am=0.985, Gamma=50, omegac=0.3 |
| 9 | `GainSched` | `GainSchedHeadingCtrl` | GainScheduledController (NearestNeighbor) - 3 PIDs scheduled on \|ψ_err\| | p0=0^\circ->Kp=1.5; p1=15^\circ->Kp=2.5; p2=30^\circ->Kp=3.5 |
| 10 | `Smith` | `SmithHeadingCtrl` | SmithPredictor compensating 1-step heading sensor delay | Inner PID: Kp=2.5, Ki=0.25, Kd=7.0 |

### Linearised Heading Model for MPC / LQR / Smith

The 3-state continuous model `xdot = Ac.x + Bc.deltar, y = Cc.x` is derived by linearising at U:

```
        ⎡ Yv/A22    (Yr-mU)/A22   0 ⎤         ⎡ Ydr/A22 ⎤
Ac  =   ⎢ Nv/A66    Nr/A66        0 ⎥  ,  Bc = ⎢ Ndr/A66 ⎥ ,  Cc = [0  0  1]
        ⎣   0          1           0 ⎦         ⎣    0    ⎦
```

Discretised via ZOH at Ts = 0.05 s (20 Hz).  The (Yr-mU)/A22 term is the centripetal correction that makes the heading model slightly non-minimum-phase and must not be neglected.

### Sign Convention Reminders (from CLAUDE.md)

| Controller | compute() argument |
|---|---|
| PID, ADRC, Smith | `compute(wrapAngle(ψ_ref - ψ))` - error |
| SMC | `compute(wrapAngle(ψ - ψ_ref))` - reversed |
| MRAC, L1 | `setReference(ψ_ref); compute(ψ)` - plant output, not error |

---

## Scenarios

| ID | Description | Duration | ψ_ref | z_ref | Disturbance |
|----|-------------|----------|-------|-------|-------------|
| `s01_turn20` | 20^\circ port heading step | 300 s | -0.349 rad (-20^\circ) | 0 m | none |
| `s02_turn90` | 90^\circ port heading step | 400 s | -1.571 rad (-90^\circ) | 0 m | none |
| `s03_zigzag10` | 10/10 zig-zag (time-based alternating +/-10^\circ) | 300 s | +/-0.175 rad, switch every 30 s | 0 m | none |
| `s04_disturbance` | Course keeping with ocean current | 300 s | 0 rad | 0 m | 150 N sway at t >= 50 s |
| `s05_depth_dive` | Depth change 0 -> 20 m, heading hold | 400 s | 0 rad | 20 m at t >= 30 s | none |

**Sample time:** Ts = 0.05 s (20 Hz, consistent with model-scale dynamics at U = 1.4 m/s)

**Initial state:** All scenarios start from rest (v = r = w = q = 0, ψ = theta = z = 0) except `s05_depth_dive` which also starts at z = 0 and dives.

---

## CSV Output Format

Each run produces `logs/run_<scenario>_<controller>.csv` with 15 columns:

| Column | Symbol | Unit | Description |
|--------|--------|------|-------------|
| `t` | t | s | Simulation time |
| `psi_ref` | ψ_ref | rad | Heading reference |
| `psi` | ψ | rad | Actual heading |
| `psi_err` | ψ_err | rad | Wrapped heading error (ref - actual) |
| `delta_r` | deltar | rad | Rudder angle command |
| `r` | r | rad/s | Yaw rate |
| `v` | v | m/s | Sway velocity |
| `z_ref` | z_ref | m | Depth reference |
| `z` | z | m | Actual depth |
| `delta_s` | deltas | rad | Stern plane command |
| `theta` | theta | rad | Pitch angle |
| `w` | w | m/s | Heave velocity |
| `q` | q | rad/s | Pitch rate |
| `x` | x | m | Earth-frame forward position |
| `y` | y | m | Earth-frame lateral position |

Per-run metrics (printed to console): IAE_ψ [rad.s], IAE_z [m.s].

---

## Build and Run

```bash
# From the repo root - run.py handles compile + all case studies automatically
conda run -n soft_robotics -- python run.py

# Or build just this target (from the build directory)
cmake --build build --target submarine_sim
./build/submarine_sim.exe
```

Logs are written to `case-study/Submarine Maneuvering Mathematical Model/logs/`.

---

## Implementation Notes

### Karasuno vs Taylor-series Model

The paper's Karasuno model decomposes forces into lift/drag/cross-flow components that remain physically valid at large drift angles (alpha > 15^\circ) where Taylor polynomials diverge. The case study uses a simpler **linearised + nonlinear cross-flow drag** approximation: linear sway/yaw derivatives (from the Taylor coefficients of Table 11, valid for small angles) plus the quadratic cross-flow terms `Yvv.v|v|` and `Yrr.r|r|`. This is sufficient for autopilot evaluation in the operating range of the scenarios (heading errors up to 90^\circ, moderate drift).

### Mildly Unstable Yaw Channel

The linearised horizontal model has an **open-loop unstable yaw mode** (lambda1 approx = +0.15 s^-^1). All controllers must provide active stabilisation - pure observers or integral-only designs will fail. ADRC, MRAC, and L1 handle this via their inherent robustness; LQR and MPC explicitly stabilise the [v, r, ψ] state.

### ADRC Stability Constraint

`omega_o * Ts = 2.0 * 0.05 = 0.10 < 0.5` - the backward-Euler stability condition for DiscreteADRC is satisfied with margin.

### Heading Angle Wrapping

All heading errors are wrapped to (-pi, pi] via `wrapAngle()` defined in `sim/include/controllers.h`. This is essential for 90^\circ turns and zig-zag maneuvers where the unwrapped error would exceed +/-pi.

### Depth-Heading Decoupling

The lateral (heading) and vertical (depth) dynamics are coupled via `U.q` in the heave equation and `U.r` in the sway equation, but this coupling is small at model scale. The cascade PID depth loop and the heading autopilot are designed independently; coupled MPC could improve performance on the `s05_depth_dive` scenario where simultaneous heading hold and depth change exercises both loops.
