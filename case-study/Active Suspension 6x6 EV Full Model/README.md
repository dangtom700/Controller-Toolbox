# Active Suspension 6*6 EV Full Model

**Paper:** Aydogan & Yildiz (2025), "Active suspension control optimization for 6*6 electric vehicles using metaheuristic algorithms," *Alexandria Engineering Journal*, 127, 2025.

**Type:** Python-only case study (sim/main.py). Discovered by run.py Phase 7.
**Runs:** 18 controllers * 5 scenarios = **90 runs**
**Reference:** case-study\Active Suspension Mathematical Modeling and Optimization 2025

---

## Plant Model: 40-State Second-Order System

The plant is a 20-DOF (40-state) lumped-parameter model of a 6*6 EV. The state vector is:

```
x = [q; q_dot]   (40*1)
```

where **q** (20*1) contains the generalised displacements:

| Index | Symbol      | Description                              |
|-------|-------------|------------------------------------------|
| 0     | Z           | Body vertical displacement [m]           |
| 1     | theta           | Body pitch angle [rad]                   |
| 2     | phi           | Body roll angle [rad]                    |
| 3-5   | Z_wr1-3     | Right-side wheel vertical (front/mid/rear) [m] |
| 6-8   | Z_wl1-3     | Left-side wheel vertical [m]             |
| 9-11  | Z_admr1-3   | Right in-wheel motor displacements [m]   |
| 12-14 | Z_adml1-3   | Left in-wheel motor displacements [m]    |
| 15    | Z_seat      | Seat vertical displacement [m]           |
| 16    | Z_p         | Pelvis [m]                               |
| 17    | Z_lt        | Lower torso [m]                          |
| 18    | Z_ut        | Upper torso [m]                          |
| 19    | Z_h         | Head [m]                                 |

### Equations of Motion

```
M q̈ + C q. + K q = B_u z_r + B_f F_act
```

- **M** (20*20 diagonal): body mass M, moments of inertia I_x/I_y; wheel mass M_w per corner; in-wheel motor mass M_adm; human segment masses M_seat, M_p, M_lt, M_ut, M_h
- **K, C** (20*20): suspension springs k_s / dampers c_s coupling body (with pitch/roll geometry at offsets a_i, +/-W/2) to wheels; tyre springs k_t; motor mount k_adm/c_adm; ISO 2631 biodynamic springs k1-k5 / dampers c1-c5
- **B_u** (20*6): tyre stiffness k_t couples each road input z_r[i] to wheel DOF i+3
- **B_f** (20*6): active actuator F_act[i] couples wheel corner to body (with pitch/roll coefficients)

### Body-Wheel Coupling Geometry

For right wheel i at longitudinal offset `a_i` (front +, rear -) and lateral offset `+W/2`:
```
F_spring = k_s (Z + a_i theta - (W/2)phi - Z_wri)
```
Left wheel i at `-W/2`:
```
F_spring = k_s (Z + a_i theta + (W/2)phi - Z_wli)
```

### Continuous State-Space (40*40)

```
A = [[0_{20},    I_{20}  ],
     [-M^-^1K,   -M^-^1C   ]]

B_road = [[0_{20*6}  ],   B_act = [[0_{20*6}  ],
          [M^-^1B_u   ]]             [M^-^1B_f   ]]
```

Discretised at **Ts = 0.005 s** (ZOH, `scipy.signal.cont2discrete`).

### Parameters (Table 2, Aydogan & Yildiz 2025)

| Parameter | Value | Description |
|-----------|-------|-------------|
| M         | 1600 kg | Sprung body mass |
| I_x       | 600 kg.m^2 | Roll inertia |
| I_y       | 1200 kg.m^2 | Pitch inertia |
| a         | [1.5, 0.0, -1.5] m | Axle longitudinal offsets |
| W         | 1.8 m | Track width |
| k_s       | 16 000 N/m | Suspension spring |
| c_s       | 980 N.s/m | Suspension damper |
| k_t       | 160 000 N/m | Tyre spring |
| M_w       | 36 kg | Wheel assembly mass |
| M_adm     | 50 kg | In-wheel motor mass |
| k_adm     | 120 000 N/m | Motor mount stiffness |
| F_max     | 3 000 N | Peak active force per actuator |
| Ts        | 0.005 s | Sample time |
| v_vehicle | 22.2 m/s | Vehicle speed (80 km/h) |

Human biodynamic model: ISO 2631-5 Guide E 5-segment values (Griffin 1990).

---

## Road Model

Two independent road channels (left/right track) generated per scenario. The 6 wheel inputs are derived via ring-buffer time delays based on axle spacing (L1 = L2 = 2.0 m) and vehicle speed:

```
delay_mid  = L1 / v_vehicle   approx = 90 ms
delay_rear = (L1+L2) / v_vehicle approx = 180 ms
z_r = [rF, rM, rR, lF, lM, lR]
```

---

## Controller Roster (18 controllers)

Each controller outputs **F_act[6]** - one active force per corner.

| # | Name | Strategy | Notes |
|---|------|----------|-------|
| 1 | PassiveCtrl | F_act = 0 | Baseline |
| 2 | PDCtrl | Per-wheel PD, hand-tuned Kp=800, Kd=120 | |
| 3 | GAOptPDCtrl | GA-optimises (Kp, Kd) offline | **Paper's core contribution** |
| 4 | PSOOptPDCtrl | PSO-optimises (Kp, Kd) offline | |
| 5 | DEOptPDCtrl | DE-optimises (Kp, Kd) offline | |
| 6 | PIDCtrl | Per-wheel PID Kp=600, Ki=80, Kd=100 | |
| 7 | LQRCtrl | Full-state 40->6 LQR (DARE, Bryson weights) | |
| 8 | LQGCtrl | LQR + KF (4-output partial observation) | |
| 9 | MPCCtrl | Condensed QP on 24-state reduced model | |
| 10 | ADRCCtrl | Per-wheel ADRC, omega_o=80, omega_o.Ts=0.40<0.5 | |
| 11 | SMCCtrl | Per-wheel SMC, compute(y-ref) convention | |
| 12 | MRACCtrl | Per-wheel MRAC, set_reference+compute(y) | |
| 13 | FuzzyPIDCtrl | Per-wheel FuzzyPID | |
| 14 | TubeMPCCtrl | Tube MPC on 2-state per-wheel model | |
| 15 | ILCCtrl | Per-wheel P-type ILC, Lp=0.5 | |
| 16 | CBFCtrl | Per-wheel CBF wrapping PD, barrier on susp. travel | |
| 17 | L1AdaptiveCtrl | Per-wheel L1 Adaptive | |
| 18 | ScenarioMPCCtrl | Scenario MPC on 2-state per-wheel model, N_s=30 | |

### Optimisation cost function (controllers #3-5):

```
cost = RMS(Z_body) + 0.5 . RMS(head_accel) + 0.1 . max_tyre_deflect / tyre_clearance
Bounds: Kp \in [100, 20000], Kd \in [10, 2000]  (same for all 6 wheels)
```

Evaluated on a 1-second step-bump simulation at construction time (offline). Population: GA pop=30 gen=60, PSO n_p=20 iter=60, DE pop=20 gen=60.

---

## Scenarios (5 * 18 = 90 runs)

| ID | Profile | Parameters | Primary metric |
|----|---------|------------|----------------|
| s01_bump | Single step bump | 25 mm, onset t=0.5 s, T_sim=5 s | Settling time of Z_body |
| s02_resonance | Sine at body resonance | 1.3 Hz, 15 mm, T_sim=10 s | RMS(Z_body) |
| s03_rough | High-frequency rough road | 8 Hz, 5 mm, T_sim=10 s | RMS(head_accel) |
| s04_speedbump | Versine speed bump | 50 mm, 0.5 m length, T_sim=5 s | max(Z_body) |
| s05_comfort | ISO 2631 sine sweep | 1->10 Hz at 0.6 Hz/s, 10 mm, T_sim=15 s | Weighted RMS seat_accel |

---

## CSV Columns

```
t, Z_body, Z_head, theta, phi,
Z_wr1, Z_wr2, Z_wr3, Z_wl1, Z_wl2, Z_wl3,
F_act_1, F_act_2, F_act_3, F_act_4, F_act_5, F_act_6,
z_r_1, z_r_2, z_r_3, z_r_4, z_r_5, z_r_6,
seat_accel, head_accel, body_accel, iae_cumulative
```

---

## Non-obvious Implementation Notes

- **ZOH discretisation via scipy.signal.cont2discrete:** the full 40*40 system is discretised once in the plant constructor. Controllers that build their own reduced model (MPC, TubeMPC, ScenarioMPC) use `scipy.signal.cont2discrete` on a 2-state per-wheel approximation.
- **LQR reference:** full-state LQR drives the state toward zero (equilibrium). No reference tracking.
- **LQG partial observation:** body Z, pitch, roll, and head Z are observed (4 outputs). Remaining states are estimated by the Kalman filter.
- **MRAC / L1 convention:** `set_reference(z_r[i])` then `compute(wheel_disp[i])` - NOT error-based.
- **SMC sign:** `compute(wheel_disp[i] - z_r[i])` (y - ref convention).
- **ADRC:** omega_o = 80, Ts = 0.005 s -> omega_o . Ts = 0.40 < 0.5 (stability constraint satisfied).
- **FuzzyPID inner bounds:** inner FuzzyPDParams.uMin/uMax = +/-1.0 (loose) so overshoot suppression works.
- **ILC:** each scenario run is treated as a fresh trial (episode). No cross-scenario learning.
- **CBF barrier:** h = |Z_wheel - Z_body| (simplified, ignores pitch/roll coupling). Intentional approximation - provides a conservative safety bound.
