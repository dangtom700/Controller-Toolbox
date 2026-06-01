# Meter-In / Meter-Out Hydraulic Actuator Case Study

**Reference:** Separate Meter-In / Meter-Out (SMISMO) electrohydraulic actuator - standard
industrial configuration for independent metering of supply and return flows via two
proportional directional-control valves.

---

## Plant Model

A 9-state nonlinear hydraulic system representing a double-acting cylinder with independent
meter-in and meter-out valve control. The meter-out valve allows active pressure control on
the rod side, enabling energy recovery and improved damping characteristics compared to
conventional single-valve layouts.

### State Vector

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| x0 | x_p | Piston position | m |
| x1 | v_p | Piston velocity | m/s |
| x2 | p1 | Piston-side (bore) chamber pressure | Pa |
| x3 | p2 | Rod-side chamber pressure | Pa |
| x4 | p_s | Supply line pressure | Pa |
| x5 | z_mi | Meter-in spool position | [-1, 1] |
| x6 | dz_mi | Meter-in spool velocity | 1/s |
| x7 | z_mo | Meter-out spool position | [-1, 1] |
| x8 | dz_mo | Meter-out spool velocity | 1/s |

### Input Vector

| Index | Symbol | Description | Range |
|-------|--------|-------------|-------|
| u0 | cmd_mi | Meter-in valve command | [-1, 1] |
| u1 | cmd_mo | Meter-out valve command | [-1, 1] |

### Output Vector

Position `x_p`, velocity `v_p`, bore pressure `p1`, rod pressure `p2`.

### Governing Equations

**Cylinder dynamics (Newton's second law):**
```
M * dv/dt = A1*p1 - A2*p2 - Bv*v - F_load
```

**Pressure dynamics (compressibility):**
```
dp1/dt = (beta_e / V1(x)) * (Q_mi - A1*v)
dp2/dt = (beta_e / V2(x)) * (A2*v - Q_mo)
```
where `V1(x) = V10 + A1*x`, `V2(x) = V20 + A2*(L-x)` are stroke-dependent chamber volumes.

**Orifice flow (turbulent through valve spool):**
```
Q_mi = Cd * Amax * |z_mi| * sign(z_mi) * sqrt(2/rho * |p_s - p1|)
Q_mo = Cd * Amax * |z_mo| * sign(z_mo) * sqrt(2/rho * |p2 - p_t|)
```

**Valve spool dynamics (2nd-order oscillator):**
```
d^2z/dt^2 = wn_v^2 * (cmd - z) - 2*zeta_v*wn_v * dz/dt
```

**Supply dynamics (pump + relief valve):**
```
dp_s/dt = (beta_e / Vs) * (Qp - Q_mi - Kr*max(0, p_s - Pr))
```

### Physical Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| A1 | 3.117e-3 m^2 | Bore area (D=63 mm) |
| A2 | 1.526e-3 m^2 | Rod-side area (D=45 mm) |
| L | 0.30 m | Stroke |
| M | 15.0 kg | Piston + load mass |
| Bv | 500.0 N.s/m | Viscous damping |
| beta_e | 300.0 MPa | Effective bulk modulus |
| rho | 870.0 kg/m^3 | Fluid density |
| Cd | 0.70 | Valve discharge coefficient |
| Amax | 2.252e-6 m^2 | Maximum orifice area |
| wn_v | 251.3 rad/s | Valve natural frequency (40 Hz) |
| zeta_v | 0.70 | Valve damping ratio |
| Qp | 2.5e-4 m^3/s | Pump ideal flow (15 L/min) |
| Pr | 14.5 MPa | Relief valve cracking pressure (145 bar) |

### Equilibrium Operating Point

| Variable | Value | Condition |
|----------|-------|-----------|
| x_p | 0.15 m | Mid-stroke |
| p1 | 1.0 MPa (10 bar) | Bore pressure |
| p2 | 2.044 MPa | Force balance: A1*p1 = A2*p2 |
| p_s | 15.0 MPa (150 bar) | Supply pressure |

### Simulation Details

- **Sample time:** Ts = 5 ms (200 Hz control loop)
- **Inner integration:** RK4 with dt_inner = 1 ms (5 sub-steps per control cycle) to resolve
  the ~60 Hz hydraulic resonance from the 300 MPa bulk modulus.

---

## Context and Motivation

Electrohydraulic actuators are used in construction machinery, aerospace control surfaces,
and industrial presses. The separate-metering architecture decouples supply and return flows,
enabling:

1. **Energy recovery** - the rod-side can throttle independently of the bore side.
2. **Active damping** - the meter-out valve can apply back-pressure to damp the hydraulic
   resonance driven by compressibility.
3. **Constraint handling** - supply pressure is a shared resource; the meter-in must respect it.

The 60 Hz hydraulic resonance, strongly nonlinear flow equations, and 9-state complexity make
this a demanding benchmark for both model-based and model-free controllers.

---

## Scenarios

| ID | Description | Reference Trajectory | Disturbance | Duration |
|----|-------------|----------------------|-------------|----------|
| S1_step | 5 cm position step (0.10 -> 0.20 m at t=1 s) | Step | None | 8 s |
| S2_multi_step | Multi-step sequence: 0.05 -> 0.15 -> 0.10 -> 0.25 -> 0.20 m | Staircase | None | 12 s |
| S3_disturbance | Step to 0.20 m at t=1 s + 1 kN load step at t=4 s | Step | F_ext = 1000 N | 10 s |

**Total runs: 14 controllers x 3 scenarios = 42**

---

## Controller Roster

Each controller subclasses `SMISMOControllerBase`. Its `compute(x, ref)` receives the full
9-element state and a position reference, and returns the 2-element command `[cmd_mi, cmd_mo]`.

| # | Name | lib/ Algorithm(s) | Design Notes |
|---|------|--------------------|--------------|
| 1 | PID | `DiscretePID` | PI on position error; derivative disabled (noisy velocity) |
| 2 | LQR | `DiscreteLQR` | 2-state [x,v] reduced model; position integral outer loop |
| 3 | LQG | `DiscreteLQG` | Position-only measurement; Kalman estimates velocity; same LQR gains |
| 4 | SMC | `DiscreteSMC` | Sliding mode on position error; boundary layer phi=0.002 m |
| 5 | ADRC | `DiscreteADRC` | 2nd-order; ESO treats valve and pressure dynamics as disturbance; b0 = A1*beta_e/(M*V10) |
| 6 | TubeMPC | `TubeMPC` | Robust MPC on 2-state model; tube sized to 1 kN load uncertainty |
| 7 | LeadLag-PID | `DiscreteLeadLag` + `DiscretePID` | Lead zero=10 rad/s, pole=50 rad/s; cascaded with PI |
| 8 | GPC-RLS | `GeneralizedPredictiveController` + `RecursiveLeastSquares` | Np=50, Nu=10; RLS na=2, nb=1, lambda=0.995; updates every 50 steps after 100-step warmup |
| 9 | MRAC | `MRACController` | Sigma-modification; adapts to hydraulic gain variation; a_m=0.85, b_m=0.15 |
| 10 | GainScheduledLQR | `GainScheduledController` | 5 position-scheduled PI gains at x=0.05..0.25 m; NearestNeighbor mode |
| 11 | EKF-LQR | `ExtendedKalmanFilter` + `DiscreteLQR` | EKF on 4-state linear model; LQR 2-state design; position integral outer |
| 12 | H-inf | `DiscreteHinf` | Mixed-sensitivity W1/W2/W3 on 2-state model; fallback to PID if synthesis fails |
| 13 | NMPC | `NonlinearMPC` | RTI on 4-state model with V1(x), V2(x) volume nonlinearity; Np=30, Nu=5 |
| 14 | FL | `FeedbackLinearisationController` | Velocity-servo inner loop; outer P position Kp_pos=20 s^-1; f(x)=-Bv/M*v, g(x)=b0 |

### Key Implementation Notes

- **Two-output plant, one-DOF control:** Most controllers only command `cmd_mi` (meter-in);
  `cmd_mo` is fixed at a nominal back-pressure value unless the controller explicitly uses both.
- **9-state complexity vs. reduced design models:** Controllers are designed on 2- or 4-state
  linearizations. The full 9-state nonlinear plant is the test vehicle. NMPC uses a 4-state
  model with nonlinear volume terms but still ignores spool dynamics (modeled as direct input).
- **RK4 inner loop:** The simulation plant uses 5 sub-steps per control cycle. Controller
  models do NOT see this internal state; they see only the output at the end of each Ts.
- **GPC-RLS warm-up:** First 100 steps use a fixed model; RLS adaptation begins at step 100
  and re-tunes every 50 steps. The SMISMO plant gain varies with piston position, so RLS
  prevents the GPC from losing authority at stroke extremes.

---

## Metrics

Each run prints:

```
ISE=<value>  IAE=<value>  settling_time=<value> s  (within +-3 mm band)
```

CSV logs written to `case-study/Meter In Meter Out Control/logs/`.

---

## Build and Run

```bash
conda run -n soft_robotics -- python run.py
```

The `smismo_sim` target is built by `compile.bat`. Individual run:

```bash
build\case-study\"Meter In Meter Out Control"\smismo_sim.exe
```
