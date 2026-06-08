# Separate Meter-In Separate Meter-Out Hydraulic Actuator Control

## Reference
Guangrong Chen, Junzheng Wang, Shoukun Wang, Jiangbo Zhao, Wei Shen (2018). "Indirect adaptive robust dynamic surface control for separate meter-in separate meter-out hydraulic system." *Control Engineering Practice* 72, 138-150.

---

## Plant Model

A **Separate Meter-In Separate Meter-Out (SMISMO)** hydraulic actuator circuit where the supply flow to and the return flow from a hydraulic cylinder are controlled by **independent proportional directional valves** (or separate 2/2 valves). This decouples the metering-in (supply) and metering-out (return) functions, enabling load-independent flow control and energy recovery through the return line.

### Physical System

- Double-acting hydraulic cylinder (piston + rod chambers)
- Independent proportional valves: V_mi (meter-in, supply) and V_mo (meter-out, return)
- Load: vertical or inclined (gravity + inertia, potentially resistive or overrunning)
- Pressure sensors on both cylinder chambers and supply line
- Displacement/velocity sensor on piston

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `x_p(t)` | Piston position | m |
| `v_p(t)` | Piston velocity | m/s |
| `P_A(t)` | Cap (A-side) chamber pressure | Pa or bar |
| `P_B(t)` | Rod (B-side) chamber pressure | Pa or bar |

### Governing Equations

**Piston dynamics (Newton):**
```
m_eff * dv_p/dt = A_A * P_A - A_B * P_B - F_L - B_v * v_p - F_f
```
where `A_A`, `A_B` = piston and rod-side areas; `F_L` = external load force; `B_v` = viscous friction; `F_f` = Coulomb friction.

**Pressure dynamics (continuity, with bulk modulus beta):**
```
V_A(x) / beta * dP_A/dt = Q_mi - A_A * v_p - Q_leak_A
V_B(x) / beta * dP_B/dt = A_B * v_p - Q_mo - Q_leak_B
```
where `V_A(x) = V_A0 + A_A * x_p`, `V_B(x) = V_B0 - A_B * x_p` are chamber volumes (position-dependent).

**Valve flow equations:**
```
Q_mi = Cd * w_mi * x_v_mi(u_mi) * sqrt(2 * (P_S - P_A) / rho)
Q_mo = Cd * w_mo * x_v_mo(u_mo) * sqrt(2 * P_B / rho)
```
where `x_v(u)` is the valve spool position (proportional to input signal with saturation and dead-band).

### Key Parameters

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Piston mass + load | m_eff | 50-500 kg | Includes reflected inertia |
| Piston area (cap side) | A_A | 20-80 cm^2 | Larger than rod side |
| Rod area (rod side) | A_B | 15-60 cm^2 | A_B = A_A - A_rod |
| Bulk modulus | beta | 1.0-1.7 GPa | Hydraulic oil at temperature |
| Supply pressure | P_S | 100-210 bar | Fixed pump pressure |
| Valve flow gain | Kq | 1e-3-1e-2 m^2/s | At nominal opening |
| Viscous damping | B_v | 500-5000 N.s/m | |
| Sampling time | Ts | 0.5-2 ms | Fast pressure dynamics |

---

## Control Objective

**Position/velocity tracking** of the hydraulic cylinder with:
1. **Energy saving:** Exploit SMISMO freedom to minimise throttling losses (meter-in valve operates near fully open for resistive loads; differential circuit for overrunning loads).
2. **Load-independent motion:** Decouple piston velocity from load force variations.
3. **Pressure feedback:** Maintain supply pressure margin and prevent cavitation in the return chamber.

The SMISMO configuration opens a second degree of freedom: the split of control effort between V_mi and V_mo can be optimised for energy efficiency at each instant, while the net piston velocity tracks the reference.

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | PID (velocity) | `DiscretePID` | Kp=80, Ki=500, Kd=0.02 | Outer velocity loop; single valve (baseline, no SMISMO) |
| 2 | CascadePID | `DiscretePID` (outer pos) + `DiscretePID` (inner vel) | Outer Kp=5; Inner Kp=80, Ki=200 | Position outer / velocity inner cascade |
| 3 | LQR | `DiscreteLQR` | Q=diag(1e6,1e4,1,1), R=diag(1,1) | Full 4-state {x_p, v_p, P_A, P_B}; two inputs {u_mi, u_mo} |
| 4 | LQG | `DiscreteLQG` | Q_w=1e-4*I, R_v=diag(1e-6,...) | Kalman filter for pressure + velocity from noisy sensors |
| 5 | MPC | `DiscreteMPC` | Np=20, Nu=5, rho_y=1e4, rho_u=1 | MIMO (u_mi, u_mo); add energy-cost term in objective |
| 6 | ADRC | `DiscreteADRC` | omega_o=500, omega_c=150, b0=Kq/m_eff | ESO estimates load force F_L as total disturbance; omega_o*Ts < 0.5 for Ts=1 ms -> omega_o < 500 |
| 7 | SMC | `DiscreteSMC` | c=200, K=5000, phi=0.01 | Sliding surface on velocity error; robust to Coulomb friction; compute(y - ref) |
| 8 | FeedbackLinearisation | `FeedbackLinearisationController` | g(x)=Kq*A_A/m_eff; f(x)=-(B_v*v+F_L)/m_eff | Inverts nonlinear valve-pressure-force chain; enables linear inner loop |
| 9 | TubeMPC | `TubeMPC` | Np=15, rho_y=1e4, Q=1e4*I, R=1 | Robust to load uncertainty; constraint tightening for pressure limits |
| 10 | L1Adaptive | `L1AdaptiveController` | a_m=-200, b_m=Kq_nom, omega_c=100 | Adapts to Kq variation with temperature and valve wear |
| 11 | GainScheduled | `GainScheduledController` | Schedule on load direction (resistive vs. overrunning) | Two gain sets: resistive load (high back-pressure) and overrunning (regenerative) |
| 12 | NonlinearMPC | `NonlinearMPC` | Np=10, Nu=3; StateFunc = full nonlinear 4-state ODE | Handles position-dependent volumes and pressure-dependent flow gains |

---

## Scenarios

| ID | Description | Load Profile | Energy Challenge |
|----|-------------|-------------|-----------------|
| s01_resistive_step | Step position command, resistive load (opposing motion) | F_L = +5 kN constant | Standard meter-in mode |
| s02_overrunning | Extension against gravity (load assists motion) | F_L = -8 kN (gravity) | Risk of cavitation; SMISMO meter-out mode |
| s03_sine_tracking | Sinusoidal position r=0.1*sin(2*pi*t) m, f=0.5 Hz | F_L = +3 kN | Velocity reversal; valve dead-band traversal |
| s04_load_step | Position hold; load steps 0 -> 10 kN at t=2 s | F_L step | Load stiffness; tests integral action |
| s05_energy_compare | 3-cycle sinusoidal motion; log total hydraulic energy | F_L = +5 kN | Primary metric: energy consumed per cycle |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Fast dynamics:** Pressure dynamics have time constants of 1-10 ms. Use Ts = 1 ms and RK4 integration. The 4-state system is stiff due to large bulk modulus.
- **Position-dependent volumes:** `V_A(x)` and `V_B(x)` change with piston position - this makes the pressure Jacobian position-dependent. LQR/MPC should be linearised at nominal mid-stroke position, or use the full nonlinear StateFunc.
- **Valve dead-band:** Real proportional valves have a dead-band of +/-2-5% of command range. Model as: `x_v = max(|u| - dead_band, 0) * sign(u)`. This causes limit cycling with integral control - add dead-band compensation.
- **SMISMO energy allocation:** For energy-optimal operation, at each step partition total velocity demand between V_mi and V_mo to minimise `P_S * Q_mi - P_B * Q_mo`. The Chen et al. (2018) paper achieves this via Indirect Adaptive Robust Dynamic Surface Control (IARDSC) with a grey predictor for supply flow estimation and a disturbance observer; the MPC/NonlinearMPC objective captures the same energy-saving intent.
- **ADRC omega_o:** With Ts = 1 ms, require `omega_o * Ts < 0.5` -> `omega_o < 500 rad/s`. Use omega_o = 400, omega_c = 120.
- **Cavitation prevention:** Hard constraint `P_B >= P_min = 2 bar` for all controllers. Saturate valve opening before cavitation threshold.
- **Coulomb friction:** Include Stribeck friction model `F_f = (F_c + (F_s - F_c)*exp(-(v/v_s)^2)) * sign(v)` for realistic velocity reversal behaviour.
- **CSV columns:** `t, x_ref, x_p, v_p, P_A, P_B, u_mi, u_mo, F_load, energy_cumul, iae_cumulative`

---

## Status

Spec only - `sim/` not present, not registered, not built.
