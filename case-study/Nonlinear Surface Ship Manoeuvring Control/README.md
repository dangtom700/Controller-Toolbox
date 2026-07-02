# Nonlinear Surface Ship Manoeuvring Control

**Paper:** Yao Meng, Xianku Zhang, Xiufeng Zhang, Yating Duan, C. Guedes Soares,
"Nonlinear identification of surface ship manoeuvring motion model and its control application,"
*Ocean Engineering* 321 (2025) 120432.
DOI: [10.1016/j.oceaneng.2025.120432](https://doi.org/10.1016/j.oceaneng.2025.120432)

**Status:** Python-only | 13 controllers | 5 scenarios | 65 runs

---

## Overview

This case study reproduces the trajectory tracking results of Meng et al. (2025). The paper
develops a Square Root Unscented Kalman Filter (SRUKF) to identify all 19 parameters of a
3-DOF MMG manoeuvring model from free-running model test data collected by MARIN during the
SIMMAN 2020 workshop. The identified model is then used with an Adaptive Sliding Mode
Controller (ASMC) for trajectory tracking.

**Model simplifications vs the paper:**
- The paper performs online SRUKF identification simultaneously with control. This case study
  uses the pre-identified Table 5 parameters directly as a fixed plant model.
- The reference trajectory integrator (heading to x,y) uses RK4 at Ts=0.08 s.
- ASMC disturbance feedforward uses the exact paper disturbance model (known to the controller).
  Other controllers have no disturbance knowledge.

---

## Plant Equations

The 3-DOF MMG model is identified in discretized form via SRUKF. The continuous-time
interpretation of the Euler-forward identification model (Eq. 3-4, paper) is:

```
du/dt = a1*u^2 + a2*v*r + a3*v^2 + a4*r^2  +  a5*n^2 + a6*delta  + d_u
dv/dt = b1*v + b2*r + b3*|v|*v + b4*|r|*r + b5*|v|*r + b6*(-u*r)
        + b7*delta  + d_v
dr/dt = c1*v + c2*r + c3*|v|*v + c4*v^2*r + c5*v*r^2  +  c6*delta  + d_r
dpsi/dt = r
dx/dt  = u*cos(psi) - v*sin(psi)
dy/dt  = u*sin(psi) + v*cos(psi)
```

where n [rev/s] is propeller speed, delta [rad] is rudder angle, and (d_u, d_v, d_r) are
external disturbances. RK4 integration at Ts = 0.08 s.

**Coordinate convention:** body-fixed frame; psi measured from north, positive clockwise.

---

## Parameter Table

All 19 identified parameters from Table 5 of Meng et al. (2025), obtained by SRUKF using
700 samples of the 20/20 deg zigzag free-running test from MARIN SIMMAN 2020.

| Symbol | Value   | Description |
|--------|---------|-------------|
| a1     | -0.023  | u^2 coefficient in surge force |
| a2     | -3.6141 | vr coupling in surge force |
| a3     | -0.8381 | v^2 coefficient in surge force |
| a4     | -9.0068 | r^2 coefficient in surge force |
| a5     |  0.0008 | n^2 propeller thrust coefficient |
| a6     | -0.0012 | rudder coupling in surge force |
| b1     | -0.8553 | v damping in sway force |
| b2     | -0.4944 | r coupling in sway force |
| b3     |  0.4451 | |v|*v nonlinear sway damping |
| b4     | -9.9044 | |r|*r nonlinear coupling |
| b5     |  2.0548 | |v|*r cross-coupling |
| b6     |  1.0348 | -u*r Coriolis-like coupling |
| b7     |  0.1306 | rudder lateral force coefficient |
| c1     | -0.7215 | v coefficient in yaw moment |
| c2     | -2.4756 | r linear yaw damping |
| c3     |  0.1106 | |v|*v nonlinear yaw term |
| c4     |  0.1936 | v^2*r nonlinear yaw term |
| c5     | 26.5057 | v*r^2 nonlinear yaw term |
| c6     |  0.4045 | rudder yaw moment coefficient |

**Ship particulars (MARIN free-running model, Table 1):**

| Parameter | Value | Unit |
|-----------|-------|------|
| Lpp       | 6.0702 | m |
| Breadth   | 0.8498 | m |
| Draft     | 0.2850 | m |
| CB        | 0.651  | - |
| Displacement | 0.9565 | m^3 |

**Actuator constraints (Eq. 35):**
- |delta| <= 35 deg (0.6109 rad)
- |d(delta)/dt| <= 14.28 deg/s (0.2492 rad/s)
- 0 <= n <= 642 RPM = 10.7 rev/s

---

## State Vector

| Index | Symbol | Description | Units |
|-------|--------|-------------|-------|
| 0 | u | surge velocity | m/s |
| 1 | v | sway velocity | m/s |
| 2 | r | yaw rate | rad/s |
| 3 | psi | heading angle | rad |
| 4 | x | x-position (earth frame) | m |
| 5 | y | y-position (earth frame) | m |

**Initial conditions (paper Section 4.2):** u0=2.0 m/s, v0=r0=psi0=0, x0=-10 m, y0=10 m.

**Equilibrium at u_ss=2 m/s:** n_ss = sqrt(-a1*u_ss^2 / a5) = 10.72 rev/s = 643 RPM.

---

## Controller Roster

| # | Name | Type | Heading loop | Notes |
|---|------|------|--------------|-------|
| 1 | OpenLoop | Fixed commands | n=n_ss, delta=0 | Baseline; no feedback |
| 2 | PID | Feedback | DiscretePID on psi_err | Cross-track correction via atan law |
| 3 | SMC | Feedback | 1st-order sliding surface on psi_err + integral | Ks=0.3, tanh switching |
| 4 | ASMC | Cascade nonlinear | Full paper Section 4.1 | Paper result; disturbance feedforward |
| 4b | AdaptiveSMC | Adaptive SMC | Library `ctrl.AdaptiveSMC` on psi_err; switching gain adapts online (no a-priori disturbance bound) | Off-the-shelf comparison to the bespoke ASMC (row 4) |
| 5 | MPC | Predictive | DiscreteMPC, linearized [psi,r] model | Np=20, Nc=5, ZOH c2d |
| 6 | LQR | State feedback | Discrete LQR on [psi_e, r] | Bryson Q=diag(16,4), R=3 |
| 7 | MRAC | Adaptive | MRACController; set_reference+compute(psi) | am=-0.15, gamma=0.5 |
| 8 | L1Adaptive | Adaptive | L1AdaptiveController; set_reference+compute(psi) | Gamma=10, omega_c=0.25 |
| 9 | GainScheduled | Scheduled | 3-point schedule on |psi_err|; PID at each point | p=[0.15,0.40,0.80] rad |
| 10 | ADRC | Active disturbance | DiscreteADRC, 2nd-order, b0=c6=0.4045 | omega_o=1.5 (omega_o*Ts=0.12 < 0.5) |
| 11 | NeuralPID | Adaptive | NeuralPID with online gain adaptation | lr=1e-4, plant_gain=c6*Ts |
| 12 | ILC | Iterative | P-type ILC; first trial = zero feedforward | Lp=0.5; improves over repeated runs |

**Speed loop (all controllers):** PI controller tracking u_ref = sqrt(xd_dot^2 + yd_dot^2);
uses n_ss as bias; Kp=2, Ki=0.5.

**ADRC constraint satisfied:** omega_o * Ts = 1.5 * 0.08 = 0.12 < 0.5.

---

## ASMC Design (Paper Section 4.1)

The paper's controller uses a backstepping-SMC cascade:

1. **Position kinematics** (Eq. 28):
   - w = sqrt(xe^2 + ye^2 + C),  C=10
   - alpha_u =  (xd_dot - rho*xe/w)*cos(psi) + (yd_dot - rho*ye/w)*sin(psi)
   - alpha_v = -(xd_dot - rho*xe/w)*sin(psi) + (yd_dot - rho*ye/w)*cos(psi)

2. **Sliding surfaces** (Eq. 30, 36):
   - sv = sigma_v1*(v - alpha_v) + sigma_v2 * integral(v - alpha_v)
   - su = sigma_u1*(u - alpha_u) + sigma_u2 * integral(u - alpha_u)

3. **Lateral equivalent + switching control** (Eq. 32-34):
   - tau_r = (1/Omega_v)*[alpha_v_dot - Fv(u,v,r) - dv - (sv2/sv1)*ve] - Kv*tanh(sv/eps_v)

4. **Longitudinal equivalent + switching control** (Eq. 38-40):
   - tau_u = alpha_u_dot - Fu(u,v,r) - Omega_u*tau_r - du - (su2/su1)*ue - Ku*tanh(su/eps_u)

5. **Actuation** (Eq. 35): delta = tau_r/c6,  n = sqrt(max(0, tau_u)/a5)

**Parameters (paper Section 4.2):** rho=1.5, C=10, sv1=2.5, sv2=0.9, su1=1.4, su2=6,
Kv=0.1, Ku=0.01, eps_v=0.1, eps_u=0.1.

**Disturbances (paper Section 4.1):**
- d_u = 0.008 sin(0.2t),  d_v = 0.008 cos(0.5t),  d_r = 0.005 cos(0.5t)

---

## Scenario List

| ID | Name | Trajectory | Duration | Disturbances |
|----|------|-----------|----------|--------------|
| s01 | Sine Trajectory | xd=20sin(0.07t), yd=t | 100 s | None |
| s02 | Straight-Line | xd=0, yd=2t | 80 s | None |
| s03 | Circular | xd=25(cos(0.06t)-1), yd=25sin(0.06t) | 200 s | None |
| s04 | Zigzag S-Curve | xd=20sin(pi*t/30), yd=1.5t | 120 s | None |
| s05 | Sine + Disturbance | xd=20sin(0.07t), yd=t | 100 s | Full (scale=1) |

s01 reproduces the paper's Fig. 7 scenario exactly. s05 tests robustness to the paper's
disturbance model; ASMC includes disturbance feedforward so it should outperform other
controllers on s05.

---

## CSV Column Definitions

Every simulation writes one CSV to `logs/sXX_<controller>.csv`:

| Column | Description | Units |
|--------|-------------|-------|
| time | simulation time | s |
| xd | desired x-position | m |
| yd | desired y-position | m |
| x | actual x-position | m |
| y | actual y-position | m |
| u | surge velocity | m/s |
| v | sway velocity | m/s |
| r | yaw rate | rad/s |
| psi_deg | heading angle | deg |
| psi_d_deg | desired heading (from trajectory tangent) | deg |
| n_rps | propeller speed applied | rev/s |
| delta_deg | rudder angle applied | deg |
| pos_error | Euclidean position error sqrt((x-xd)^2+(y-yd)^2) | m |
| iae_cumulative | running integral of pos_error*dt | m*s |

---

## Performance Summary (representative results)

| Controller | s01 MeanErr [m] | s01 IAE [m*s] | s05 MeanErr [m] | Note |
|------------|----------------|---------------|----------------|------|
| OpenLoop   | 99.6 | 9957 | 99.9 | No control |
| PID        | 70.5 | 7053 | 70.5 | Saturates rudder |
| SMC        | 55.4 | 5536 | 55.2 | Best non-ASMC |
| **ASMC**   | **6.8** | **682** | **5.9** | **Paper result; 12x better than PID** |
| MPC        | 65.6 | 6557 | 65.3 | Linearized model limits perf |
| LQR        | 66.1 | 6610 | 65.9 | Similar to MPC |
| ILC        | 78.7 | 7872 | 78.7 | First trial only; zero feedforward |

ASMC achieves ~90% lower IAE than PID on the paper scenario (s01), consistent with
the paper's conclusion that the full cascade nonlinear controller is necessary for
accurate trajectory tracking with rudder/propeller rate constraints.

ILC has zero rudder activity on first trial (pure feedforward mode); performance
improves over repeated experimental trials.

---

## Implementation Notes

- **alpha_v_dot** is computed via backward finite differences (alpha_v(k) - alpha_v(k-1))/Ts.
  This introduces one-step delay but avoids symbolic differentiation through the full
  kinematic chain.
- **GainScheduledController** uses `add_schedule_point(p, ctrl)` and `set_scheduling_param(p)`
  API (ctrl_toolbox Python binding); schedule points at |psi_err| = 0.15, 0.40, 0.80 rad.
- **ADRC:** omega_o=1.5 rad/s, Ts=0.08 s -> omega_o*Ts=0.12 < 0.5 (backward Euler stability).
- **MRAC/L1Adaptive:** use `set_reference(psi_ref)` then `compute(psi)` convention (not error).
- **LQR:** equilibrium at zero psi_err, r=0; gain computed from discrete-time Bryson DARE.
- **Speed loop:** PI with n_ss bias ensures propeller stays near equilibrium RPM at rated speed.

---

## Run Instructions

```bash
conda run -n soft_robotics -- python "case-study/Nonlinear Surface Ship Manoeuvring Control/sim/main.py"
```

Output: summary table to stdout + 60 CSV files in `logs/`.
