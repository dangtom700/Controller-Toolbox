# Active Suspension Mathematical Modeling and Optimization 2025

**Reference:** Berk Aydogan, Ahmet Yildiz (2025). "Mathematical modeling and optimization of the active suspension system of a 6×6 electric vehicle." *Alexandria Engineering Journal* 127, 989–1003.

**Note on plant model:** The paper analyses a full 6×6 in-wheel electric vehicle with 15-DOF body + 5-DOF human model optimised via GA/PSO/DE. The simulation here uses a standard **2-DOF quarter-car** active suspension derived from the per-corner sprung/unsprung mass subsystem of such models, which is the well-established benchmark for active suspension controller design and comparison.

---

## Plant Model

A 2-DOF quarter-car active suspension model with 4 states representing the body (sprung mass)
and wheel assembly (unsprung mass). The actuator force is applied between the two masses.

### State Vector

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | z_s | Sprung mass (body) displacement | m |
| 1 | dz_s | Sprung mass velocity | m/s |
| 2 | z_u | Unsprung mass (wheel) displacement | m |
| 3 | dz_u | Unsprung mass velocity | m/s |

### Inputs and Disturbances

**Control input:**

| Symbol | Description | Saturation |
|--------|-------------|------------|
| F_act | Active actuator force between body and wheel | +/- 2000 N |

**Road disturbance:**
- z_r(t): road surface profile [m]; enters through tyre stiffness k_t

### Governing Equations

```
m_s * ddz_s = -k_s*(z_s - z_u) - c_s*(dz_s - dz_u) + F_act
m_u * ddz_u =  k_s*(z_s - z_u) + c_s*(dz_s - dz_u) - k_t*(z_u - z_r) - F_act
```

State-space form (continuous):
```
A = [[0,          1,          0,           0        ],
     [-k_s/m_s, -c_s/m_s,  k_s/m_s,   c_s/m_s  ],
     [0,          0,          0,           1        ],
     [k_s/m_u,  c_s/m_u, -(k_s+k_t)/m_u, -c_s/m_u]]

B_act = [0, 1/m_s, 0, -1/m_u]^T   (actuator column)
B_road = [0, 0, 0, k_t/m_u]^T     (road disturbance column)
```

**Integration:** Classical RK4 at Ts = 0.005 s (200 Hz).

### Parameter Values

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Sprung mass (quarter-car) | m_s | 240 kg | Standard compact car quarter |
| Unsprung mass (wheel assembly) | m_u | 36 kg | Wheel + axle + hub |
| Suspension spring | k_s | 16 000 N/m | Typical passenger car |
| Suspension damper | c_s | 980 N.s/m | Passive zeta = 0.25 |
| Tyre stiffness | k_t | 160 000 N/m | Standard radial tyre |
| Actuator saturation | F_max | 2 000 N | Electrohydraulic actuator |

### Natural Frequencies

| Mode | Symbol | Value |
|------|--------|-------|
| Body resonance | f_n_body | 1.30 Hz (8.16 rad/s) |
| Wheel hop | f_n_wheel | 11.1 Hz (69.8 rad/s) |
| Passive damping ratio | zeta_body | 0.25 (underdamped) |

---

## Scenarios

All scenarios target z_s = 0 (body at equilibrium position regardless of road profile).

| ID | Description | Road Profile | Duration |
|----|-------------|-------------|----------|
| S1 | Step bump (25 mm at t=0.5 s) | Smooth-step onset, holds | 5 s |
| S2 | Sinusoidal at body resonance (1.3 Hz, 15 mm) | Sine | 5 s |
| S3 | Rough road near wheel hop (8 Hz, 5 mm) | Sine | 5 s |
| S4 | Speed bump (50 mm versine, 0.5 m at 30 km/h) | Versine | 3 s |
| S5 | Compound: 10 mm sine 1.3 Hz + 15 mm step at t=2 s | Composite | 5 s |

**Total runs: 15 controllers x 5 scenarios = 75**

S1 tests settling time and overshoot suppression. S2 is the primary comfort test -
body resonance amplification by the passive suspension is the core problem active
suspension solves. S3 tests road isolation near wheel hop. S4 tests transient impact
response to an isolated obstacle. S5 combines continuous disturbance with a sudden
change to test combined tracking and rejection.

---

## Controller Roster

Each controller subclasses `susp::ControllerBase`. Its `compute(state, z_r)` receives
the 4-element state and the road height, and returns actuator force F_act [N].

| # | Name | lib/ Algorithm(s) | Design Notes |
|---|------|--------------------|--------------|
| 1 | Passive | (none) | F_act = 0 always; baseline reference |
| 2 | PID | `DiscretePID` | Body displacement feedback; Kp=2000 N/m, Ki=30 N/(m.s), Kd=500 N.s/m, N=10 |
| 3 | ADRC | `DiscreteADRC` | 2nd-order LADRC; b0=1/m_s; ESO treats road as total disturbance; omega_o=20, omega_c=8, omega_o*Ts=0.10<0.5 |
| 4 | SMC | `DiscreteSMC` | Sliding surface s = z_s + lambda*Ts*dz_s; lambda=5 rad/s; phi=0.005 m; K_sw=1200 N |
| 5 | LQR | `DiscreteLQR` | Full 4-state Bryson-tuned; x_max=[0.025m, 0.5m/s, 0.025m, 1m/s]; u_max=2000N |
| 6 | LQG | `DiscreteLQG` | KF measures z_s + z_u (2 outputs); same Bryson Q/R; process noise on wheel states |
| 7 | MPC | `DiscreteMPC` | 2-state body model; Np=20, Nu=5; rho_y=1.0, rho_u=1e-7; u in [-2000,2000] N |
| 8 | MRAC | `MRACController` | sigma-modification; a_m=exp(-4*Ts)~0.980; gamma=0.01 (conservative for 4th-order coupling) |
| 9 | FuzzyPID | `FuzzyPID` | e_scale=0.03m; de_scale=0.5m/s; u_scale=2000N; Ki=50 N/(m.s) |
| 10 | TubeMPC | `TubeMPC` | 2-state body model; K=-K_lqr; wMax=[0.002m, 0.040m/s] (wheel coupling bound); Np=10, Nu=3 |
| 11 | ILC | `ILCController` | P-type ILC; Lp=0.6; N_trial=1000; learns periodic road disturbance feedforward trial-to-trial |
| 12 | CBFSafety | `CBFSafetyFilter` | Barrier on dz_s: h=v_max-dz_s, v_max=0.5 m/s; alpha=5; PID nominal; g=1/m_s (approximate - ignores coupling) |
| 13 | L1Adaptive | `L1AdaptiveController` | a_m=exp(-4*Ts)~0.980, Gamma=200, omega_c=2.0, sigma_max=5000; adapts to payload variation |
| 14 | ScenarioMPC | `ScenarioMPC` | 2-state body SS; Np=10, Nu=3; Sigma_w=diag(4e-6,1.6e-3) (wheel noise); N_samples=30 |
| 15 | DynaCtrl | `DynaController` | Wraps DiscretePID; n_collect=50, n_refit=25; error=-z_s |

### Key Implementation Notes

- **Sign conventions:** PID/FuzzyPID use `compute(-z_s)` (r-y with r=0); SMC uses
  `compute(z_s)` (y-ref convention); ADRC/MRAC use `compute(-z_s)` / `compute(z_s)` respectively.
- **ADRC stability:** omega_o * Ts = 20 * 0.005 = 0.10 < 0.5 (backward-Euler stable).
- **MPC and TubeMPC model:** Both use a 2-state reduced body model (z_s, dz_s) that
  treats wheel dynamics as a disturbance; the full 4-state plant is used for simulation.
- **LQR/LQG matrix construction:** `makeFull4SS` assembles the exact continuous-time
  state-space from plant parameters and calls `ctrl::c2d(..., ZOH)` for discretization.
- **TubeMPC K convention:** `u_tube = K*(x-x_nom)`. K = -K_lqr (LQR gain negated) per
  toolbox convention. wMax = [0.002, 0.040] bounds the per-step wheel-coupling disturbance.

---

## Metrics

Each run prints and logs:

```
[Sk | Controller]  IAE_zs=<>  RMS_acc=<>  MaxDefl=<>  RMS_force=<>  sat=<>  wall=<> ms
```

| Metric | Description | Unit |
|--------|-------------|------|
| IAE_zs | Integral absolute body displacement | m.s |
| RMS_acc | RMS body vertical acceleration (ride comfort) | m/s^2 |
| MaxDefl | Maximum suspension deflection |z_s - z_u| (travel limit) | m |
| RMS_force | RMS actuator force (energy proxy) | N |
| sat | Actuator saturation event count | - |

CSV logs written to `case-study/Active Suspension.../logs/`.
CSV columns: `t, z_s, dz_s, z_u, dz_u, z_r, F_act, body_acc, susp_defl, tyre_defl`

---

## Build and Run

```bash
conda run -n soft_robotics -- python run.py
```

The `susp_sim` target is built by `compile.bat`. Individual run:

```bash
build\case-study\"Active Suspension Mathematical Modeling and Optimization 2025"\susp_sim.exe
```
