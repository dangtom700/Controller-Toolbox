# Boiler Control Case Study

**Reference:** Bell & Astrom (1987), "A Low Order Nonlinear Dynamic Model of a Power Plant Boiler-Turbine Unit"

---

## Plant Model

The Bell-Astrom nonlinear boiler-turbine model is a 3-state, 3-input MIMO system representing
a drum-boiler connected to a steam turbine-generator. The model is widely used as a MIMO
nonlinear benchmark in the process control literature.

### State Vector

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| x1 | p | Drum pressure | bar |
| x2 | P_e | Electric power output | MW |
| x3 | h | Drum water level (steam quality proxy) | cm |

### Input Vector (valve positions, all in [0, 1])

| Index | Symbol | Description | Rate limit |
|-------|--------|-------------|------------|
| u1 | q_f | Fuel flow valve | +/- 0.007 / step |
| u2 | q_s | Steam control valve | +/- 0.020 / step |
| u3 | q_w | Feedwater valve | +/- 0.050 / step |

### Output Vector

| Index | Description |
|-------|-------------|
| y1 | Drum pressure x1 [bar] |
| y2 | Electric power x2 [MW] |
| y3 | Boiler efficiency proxy (nonlinear function of x, u) |

### Governing Equations

```
dx1/dt = -0.0018 * u2 * x1^(9/8) + 0.9*u1 - 0.15*u3
dx2/dt = (0.073*u2 - 0.016) * x1^(9/8) - 0.1*x2
dx3/dt = (141*u3 - (1.1*u2 - 0.19)*x1) / 85
```

Sample time: **Ts = 1.0 s** (1-Hz control loop)

### Operating Points

| OP | Load Level | p [bar] | P_e [MW] | h [cm] | u1 | u2 | u3 |
|----|-----------|---------|----------|--------|----|----|-----|
| A | Low Load | 75.6 | 15.3 | 508.97 | 0.1193 | 0.3806 | 0.1226 |
| B | Medium Load | 97.2 | 50.5 | 469.51 | 0.2705 | 0.6208 | 0.3398 |
| C | High Load | 140.0 | 128.0 | 323.68 | 0.5959 | 0.8945 | 0.7883 |

---

## Context and Motivation

A coal or gas-fired boiler must simultaneously regulate drum pressure (energy storage),
turbine power output (grid demand), and drum water level (safety). These three outputs
are strongly coupled: opening the steam valve drops pressure, raises power, and lowers
level simultaneously. Fast setpoint steps are constrained by valve rate limits, creating
control timing conflicts typical of real plant startup/load-change sequences.

The study covers three operating regimes (low/medium/high load), load-step tracking tasks,
operating-point transitions, and a periodic-grid-load scenario designed to expose the
structural advantage of RepetitiveController over standard feedback.

---

## Scenarios

| ID | Description | Starting OP | Mode | Duration |
|----|-------------|-------------|------|----------|
| s01_lowload_regulation | Perturbation rejection: initial offset [+5 bar, +3 MW, -10 cm] from OP-A | A | Regulation | 3600 s |
| s02_medload_regulation | Perturbation rejection at Medium Load (OP-B) | B | Regulation | 3600 s |
| s03_highload_regulation | Perturbation rejection at High Load (OP-C) | C | Regulation | 3600 s |
| s04_lowload_tracking | Load-step tracking: +2 bar, +3 MW from OP-A | A | Tracking | 3600 s |
| s05_medload_tracking | Load-step tracking at Medium Load | B | Tracking | 3600 s |
| s06_highload_tracking | Load-step tracking at High Load | C | Tracking | 3600 s |
| s07_multiop_transition | OP-A -> OP-B transition at t = 1800 s | A->B | Tracking | 3600 s |
| s08_periodic_load | Sinusoidal grid demand: +/-5 bar / +/-2 MW at 0.005 Hz (period=200 s, 7200 s run) | B | Periodic | 7200 s |

**Total runs: 27 controllers x 8 scenarios = 216**

---

## Controller Roster

Each controller wraps one or more algorithms from `lib/` inside the boiler-specific
`BoilerControllerBase`. The outer `compute()` receives the 3-element error vector and
returns a 3-element valve-delta command.

| # | Name | lib/ Algorithm(s) | Design Notes |
|---|------|--------------------|--------------|
| 1 | PID | `DiscretePID` x3 | Per-axis decoupled; Ziegler-Nichols tuned |
| 2 | LQR | `DiscreteLQR` | Full MIMO 3x3 state-feedback on linearized model at OP-B |
| 3 | LQG | `DiscreteLQG` | KF estimates full state; LQR feedback (same Q/R as #2) |
| 4 | MPC | `DiscreteMPC` x3 | Per-axis FOPDT condensed QP; Np=30, Nc=8, qp_max_iter=1000 |
| 5 | SMC | `DiscreteSMC` x3 | compute(y - ref) sign convention; per-axis phi=[0.10,0.20,0.05], K=0.10 |
| 6 | ESC | `ExtremumSeeker` | Model-free gradient descent maximizing negative IAE sum |
| 7 | ADRC | `DiscreteADRC` x3 | omega_o=0.45 rad/s; omega_o*Ts=0.45 < 0.50 (backward-Euler stable with 10% margin); omega_c=0.09 |
| 8 | LeadLag-PID | `DiscreteLeadLag` + `DiscretePID` x3 | Lead compensator cascaded with PI |
| 9 | SmithPredictor | `SmithPredictor` x3 | Per-axis 1-step transport delay model |
| 10 | GPC-RLS | `GeneralizedPredictiveController` + `RecursiveLeastSquares` x3 | Adaptive self-tuning GPC; NaN if RLS not yet converged |
| 11 | EKF-LQR | `ExtendedKalmanFilter` + `DiscreteLQR` | EKF on nonlinear plant; LQR feedback on estimated state |
| 12 | UKF-LQR | `UnscentedKalmanFilter` + `DiscreteLQR` | UKF sigma-point state estimation; same LQR gains |
| 13 | FuzzyPID | `FuzzyPID` x3 | 25-rule Mamdani per axis |
| 14 | FuzzySup-MPC | `FuzzySupervisor` + `DiscreteMPC` x3 | Supervisor triggers MPC re-linearization on large error |
| 15 | SupervisoryStack | `ControllerStack` x3 (SMC->LQR) | Switches SMC->LQR when sliding surface converges |
| 16 | AdditiveStack | `ControllerStack` x3 (PID + LeadLag) | Blended additive over 300 steps |
| 17 | WeightedStack | `ControllerStack` x3 (PID + LQR) | Weights scheduled by pressure x1 |
| 18 | RepetitiveCtrl | `RepetitiveController` x3 | Period=200 steps; designed for s08_periodic_load |
| 19 | MRAC | `MRACController` x3 | Lyapunov + sigma-modification; adapts across OP-A/B/C gain changes |
| 20 | H-inf | `DiscreteHinf` x3 | Mixed-sensitivity; falls back to PID if synthesis infeasible at OP |
| 21 | AdaptiveSP | `AdaptiveSmithPredictor` x3 | Online cross-correlation delay estimator + Smith compensation |
| 22 | NMPC | `NonlinearMPC` | RTI on full nonlinear Bell-Astrom model; Np=10, Nu=3 |
| 23 | FL | `FeedbackLinearisationController` x3 | Channel-specific nonlinearities; f(x,u) linearizes dx/dt |
| 24 | MHE-LQR | `MovingHorizonEstimator` (N=10) + `DiscreteLQR` | MHE state estimation on linearized model; LQR feedback |
| 25 | LPV-GS | `GainScheduledController` | Pressure-dependent LPV scheduling [60,150] bar; NearestNeighbor |
| 26 | SubspaceID-LQG | `n4sid` + `DiscreteLQG` | 3rd-order model identified from step response; adaptive LQG |
| 27 | AutoGS | `GainScheduledController` (via `buildAutoGainScheduler`) | Nu-gap clustered scheduling across pressure sweep [60,150] bar |

### Key Implementation Notes

- **SMC sign convention:** `compute(y - ref)` - opposite of PID - matches `DiscreteSMC` contract.
- **GPC-RLS warm-up:** GPC-RLS outputs NaN for the first ~10-20 steps until RLS has enough data; the
  simulation runner tolerates this. Scenarios s02/s08 show NaN in early steps - expected behavior.
- **H-inf fallback:** If the DGKF Riccati fails to converge at an extreme operating point, the
  controller silently falls back to the OP-B PID gains. This is visible as identical IAE to PID.
- **AutoGS / LPV-GS:** `DiscreteLQR` is not an `IController`; LQR gains are wrapped in a `DiscretePID`
  proxy (Kp = K(0,0)) for the `design_fn` interface (TK26-1).
- **ADRC stability:** `omega_o * Ts` must be strictly < 0.5 for the backward-Euler ESO.
  At Ts=1 s, `omega_o = 0.45` rad/s gives `omega_o*Ts = 0.45` with ~10% margin. Do not
  increase `omega_o` above 0.49 without reducing `Ts`.

---

## Metrics

Each run produces a CSV log in `logs/run_<scenario>_<controller>.csv` and prints:

```
IAE=[y1, y2, y3]  ISE=[y1, y2, y3]  E_valve=<fuel_energy_cost>
```

`E_valve` is the L1 norm of cumulative valve movements -- a proxy for actuator wear.

---

## Build and Run

```bash
conda run -n soft_robotics -- python run.py
```

The case study target `boiler_sim` is built by `compile.bat` and run automatically.
Individual run:

```bash
build\case-study\Boiler Control\boiler_sim.exe
```

Logs written to `case-study/Boiler Control/logs/`.
