# Tug Boat Numerical Simulation Case Study

**Reference:** Li et al. (2026) - 3-DOF unified barge-tugboat dynamic positioning model.
Plant parameters and scenario conditions from Table 5 of the paper.

---

## Plant Model

A 6-state nonlinear model of a barge under tug-assist dynamic positioning (DP). Four tugs
are attached at fixed stations around the barge hull. The model combines rigid-body inertia,
hydrodynamic added mass, linear viscous damping, and a nonlinear kinematics mapping from
body-frame velocities to world-frame positions.

### State Vector

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | x | World-frame surge position | m |
| 1 | y | World-frame sway position | m |
| 2 | psi | Heading angle | rad |
| 3 | u | Body-frame surge velocity | m/s |
| 4 | v | Body-frame sway velocity | m/s |
| 5 | r | Yaw rate | rad/s |

### Inputs and Disturbances

**Control inputs (allocated to 4 tugs):**

| Symbol | Description | Saturation |
|--------|-------------|------------|
| tau_x | Generalized surge force | +/- 2.0e6 N |
| tau_y | Generalized sway force | +/- 2.0e6 N |
| tau_psi | Generalized yaw moment | +/- 5.0e7 N.m |

**Environmental disturbances (body frame):**
- Wind drag (aerodynamic force + moment) - function of wind speed and bearing
- Current drag (hydrodynamic force + moment) - function of current speed and bearing
- JONSWAP wave drift - 2-component spectrum with Hs, Tp, and random phase seed

### Governing Equations

**Kinematics (body -> world):**
```
eta_dot = R(psi) * nu

where:
  eta = [x, y, psi]
  nu  = [u, v, r]
  R(psi) = [[cos(psi), -sin(psi), 0],
             [sin(psi),  cos(psi), 0],
             [       0,         0, 1]]
```

**Dynamics (Newton-Euler in body frame):**
```
(M_rb + M_a) * nu_dot = tau_control + tau_env - C_rb(nu)*nu - D*nu

where:
  M_rb = barge rigid-body inertia matrix
  M_a  = hydrodynamic added mass (frequency-independent, infinite-frequency)
  C_rb = Coriolis-centripetal matrix (skew-symmetric, depends on nu)
  D    = linear damping matrix
  M_re = M_rb + M_a  (effective mass, precomputed)
  D_re = effective damping (precomputed)
```

**Coriolis terms:**
```
C_rb(nu) has off-diagonal coupling:
  -( m - Yv_dot ) * v  and  ( m - Xu_dot ) * u
```

**Integration:** Classical RK4 with simulation dt (from scenario JSON).

### Thrust Allocator

A least-squares pseudoinverse allocates the 3-DOF generalized force vector to individual
tug thrust commands. Each tug has independent thruster dynamics with rate limits `dT_max [N/step]`
and thrust bounds `[T_min, T_max]` clipped after allocation. Fuel cost is tracked as
`E_fuel = sum(|thrust| * dt)` over the run.

### System Matrix Values

Loaded from `config/plant_params.json`. Key diagonal entries:

| Parameter | Value | Description |
|-----------|-------|-------------|
| M_re(0,0) | 1.349e8 kg | Surge effective mass |
| M_re(1,1) | 8.718e7 kg | Sway effective mass |
| M_re(2,2) | 4.669e13 kg.m^2 | Yaw effective inertia |
| D_re(0,0) | 6.0e5 N.s/m | Surge linear damping |
| D_re(1,1) | 1.34e6 N.s/m | Sway linear damping |
| D_re(2,2) | 3.119e12 N.m.s/rad | Yaw linear damping |

> Note: `[DiscreteLQR] WARNING: (A,B) failed PBH stabilizability test` appears at startup
> for 4 controllers that use inner LQR designs (TubeMPC, AutoGS-LQR). This is a diagnostic
> for the slow wave-drift modes (neutrally stable at z=1). DARE still converges and the
> controllers run correctly.

---

## Context and Motivation

Tug-assisted dynamic positioning of a barge is a safety-critical offshore operation.
The control challenge is multi-axis coupled positioning against persistent environmental
loads (wind, current, waves) with actuator saturation and rate limits. The JONSWAP wave
spectrum introduces colored stochastic disturbances with energy concentrated around the
peak period Tp.

The study benchmarks the full library roster - from decoupled PID to full MIMO NMPC -
under identical environmental conditions, measuring position IAE, heading IAE, and
fuel energy as primary metrics.

---

## Scenarios

All scenarios target the origin `[x=0, y=0, psi=0]` (station-keeping).

| ID | Description | Wind [m/s] | Current [kn] | Hs [m] | Tp [s] | Duration |
|----|-------------|------------|--------------|--------|--------|----------|
| S1 | Calm water - zero-input stability test | 0 | 0 | 0 | - | 300 s |
| S2 | 90^\circ wind + current (paper Table 5 baseline) | 10 | 1.0 | 2.0 | 10 | 5400 s |
| S3 | 135^\circ quartering disturbance (primary validation) | 10 | 1.0 | 2.0 | 10 | 5400 s |
| S4 | 180^\circ head-on disturbance | 10 | 1.0 | 2.0 | 10 | 5400 s |

**Total runs: 16 controllers x 4 scenarios = 64**

S1 is a numerical stability check - all controllers should produce IAE = 0 with no
environmental forcing. S2-S4 replicate Table 5 conditions from Li et al. (2026).

---

## Controller Roster

Each controller subclasses `TugControllerBase`. Its `compute(state, ref)` receives the
6-element state vector and a 3-element reference `[x_ref, y_ref, psi_ref]`, and returns
generalized forces `tau = [tau_x, tau_y, tau_psi]` which are then allocated to individual tugs.

| # | Name | lib/ Algorithm(s) | Design Notes |
|---|------|--------------------|--------------|
| 1 | PID | `DiscretePID` x3 | Per-axis decoupled surge/sway/yaw; Ziegler-Nichols on linearized step |
| 2 | KF-PID | `KalmanFilter` + `DiscretePID` x3 | 6-state linear SS KF; same PID feedback on estimated state |
| 3 | SMC | Sliding mode (paper Eqs. 24-27) | Integral surface + boundary layer; Lambda=[0.05,0.05,0.10]; switching gains K_sw=[8e5,8e5,2e7] |
| 4 | MPC | `DiscreteMPC` x3 | Per-axis decoupled SISO linearized models; `notifyApplied()` syncs with thrust allocator saturation |
| 5 | ESC | `ExtremumSeeker` x3 | Model-free gradient descent on per-axis IAE |
| 6 | FuzzyPID | `FuzzyPID` x3 | 25-rule Mamdani FuzzyPD + crisp integral with anti-windup |
| 7 | FuzzySup-MPC | `DiscreteMPC` x3 + `FuzzySupervisor` x3 | Supervisor monitors error and triggers MPC re-linearization |
| 8 | ADRC | `DiscreteADRC` x3 | b0_xy = 1/M_re(i,i), b0_psi = 1/M_re(2,2); omega_o=0.5; omega_o*dt=0.25 (backward-Euler stable) |
| 9 | RepetitiveCtrl | `RepetitiveController` x3 | On top of PID; period = Tp steps to cancel periodic JONSWAP wave drift in S2-S4 |
| 10 | LQR | `DiscreteLQR` | Full 6-state MIMO; Bryson weights: xmax=[10,10,0.1,1,1,0.05], umax=[TAU_XY_MAX, TAU_XY_MAX, TAU_PSI_MAX] |
| 11 | LQG | `DiscreteLQG` | Kalman estimates full 6-state from position+heading measurement; same Bryson Q/R |
| 12 | TubeMPC | `TubeMPC` x3 | Per-axis robust MPC; tube sized to JONSWAP wave disturbance bound; K = -K_lqr convention |
| 13 | EKF-LQR | `ExtendedKalmanFilter` + `DiscreteLQR` | EKF on nonlinear kinematics J(psi)*nu; LQR on 6-state linearization |
| 14 | MRAC | `MRACController` x3 | Conservative adaptation (gamma=1e-8, large theta_max); handles relative-degree-2 ship dynamics |
| 15 | AutoGS-LQR | `GainScheduledController` (surge) + `DiscretePID` x2 | Surge axis scheduled on |u_v| in [0,1.5] m/s via nu-gap clustering; sway/yaw remain PID |
| 16 | NMPC | `NonlinearMPC` | RTI on 6-state discrete nonlinear dynamics; Np=20, Nu=5; C=[I_3, 0_3x3] position tracking |

### Key Implementation Notes

- **Regression test:** This is the only case study with a Catch2 regression (S2 baseline):
  `IAE_x=806.5, IAE_y=116786.1, IAE_psi=0.5` at 5% tolerance (`tests/test_tugsim_regression.cpp`).
- **TubeMPC K convention:** `u_tube = K*(x - x_nom)`. For LQR-designed K (which gives positive
  feedback in MATLAB convention), negate: `K = -K_lqr`.
- **PBH stabilizability warning:** 4 controllers trigger `[DiscreteLQR] WARNING: PBH test failed`
  on startup due to the slow/neutral wave-drift modes. DARE converges correctly; warning is benign.
- **MRAC convergence:** The double-integrator-like ship dynamics (relative degree 2) require very
  conservative adaptation rates. Aggressive gamma causes oscillatory adaptation.
- **SMC implementation:** Uses the paper equations directly (Eqs. 24-27), not `DiscreteSMC` from
  lib/, because the sliding surface is an integral type (`s = int(e) + Lambda*e`) rather than the
  `DiscreteSMC` derivative-gain surface.

---

## Metrics

Each run prints and logs:

```
[Sk | Controller]  IAE_x=<>  IAE_y=<>  IAE_psi=<>  IAE_total=<>  E_fuel=<>  sat=<>  wall=<> ms
```

`E_fuel` = sum(|thrust|*dt) over all tugs [N.s], a proxy for fuel consumption.
`sat` = total thrust saturation events across all tugs.
CSV logs written to `case-study/Tug Boat Numerical Simulation/logs/`.

---

## Build and Run

```bash
conda run -n soft_robotics -- python run.py
```

The `tug_sim` target is built by `compile.bat`. Individual run:

```bash
build\case-study\"Tug Boat Numerical Simulation"\tug_sim.exe
```
