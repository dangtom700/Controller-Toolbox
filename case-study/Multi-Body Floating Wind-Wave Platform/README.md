# Multi-Body Floating Wind-Wave Platform

## Reference
**Title:** A mathematical model for the dynamic analysis of multi-body floating platforms with complex mechanical constraints  
**Authors:** Thiago S. Hallak, Jose F. Gaspar, C. Guedes Soares  
**Journal:** Ocean Engineering, Vol. 314, 2024, Article 119640  
**DOI:** https://doi.org/10.1016/j.oceaneng.2024.119640

---

## System Description

A **Floating Wind-Wave Platform (FWWP)** is a hybrid offshore energy device that couples a Floating Offshore Wind Turbine (FOWT) with an articulated Wave Energy Converter (WEC) that drives a hydraulic power take-off (PTO) piston. The complete system is a constrained multi-body structure subject to ocean waves, where hydrodynamic interactions among the submerged geometries and the nonlinear mechanical joints must be modelled simultaneously.

The paper's key contribution is a generalised-coordinates formulation that yields **explicit, low-dimensional ODEs** for the multi-body system-suitable for real-time simulation, optimisation, and control synthesis.

---

## Mathematical Model

### Degrees of freedom

Each rigid body *i* has 6 DOF (surge, sway, heave, roll, pitch, yaw). Mechanical constraints (hinges, articulated arms) reduce the total DOF by enforcing geometric equations:

```
g(q) = 0          (holonomic constraint vector)
J(q) q. = 0        (velocity-level Jacobian form)
```

### Equations of motion (generalised coordinates)

Using Lagrangian mechanics with hydrodynamic impulse-response functions:

```
M q̈ + B q. + K q + \int0ᵗ h(t-tau) q.(tau) dtau = F_wave + F_ext + F_constraint
```

where:
- `q` - generalised coordinate vector (reduced DOF after constraints)
- `M` - generalised mass matrix (rigid-body inertia + added mass at inf)
- `B` - linear damping matrix
- `K` - hydrostatic restoring stiffness
- `h(t)` - retardation (impulse-response) kernel from hydrodynamic memory effect
- `F_wave` - first-order wave excitation force
- `F_ext` - external loads (mooring, PTO force)
- `F_constraint` - constraint reaction forces via Lagrange multipliers lambda

### Constraint handling - Gauss Principle

Constraint forces are computed via:

```
J Mⁱ Jᵀ lambda = J Mⁱ (F_wave + F_ext) - J. q.
```

### Hydraulic PTO (power take-off)

The WEC drives a hydraulic piston; the PTO force is:

```
F_PTO = Cpto * xdot_rel        (passive linear damper, simplest model)
F_PTO = -k_spring * x_rel   (spring stiffness variation)
```

where `x_rel` is the relative displacement at the articulated arm.

### Natural frequency (closed-form)

For the simplified two-body case an analytical expression for the natural frequencies of the constrained system is derived and verified numerically.

---

## State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `q` | Generalised position vector | m, rad |
| `q.` | Generalised velocity vector | m/s, rad/s |
| `x_rel` | Relative WEC arm displacement | m |
| `xdot_rel` | Relative WEC arm velocity | m/s |

## Inputs / Actuators

| Signal | Description |
|--------|-------------|
| `F_PTO` | Hydraulic PTO force (controllable) |
| `u_mooring` | Active mooring tension (if actuated) |

## Outputs / Measurements

| Signal | Description |
|--------|-------------|
| `eta` (surge/heave/pitch of FOWT) | Platform motion for fatigue / comfort |
| `P_WEC = F_PTO * xdot_rel` | Instantaneous WEC power |
| `P_wind` | Aerodynamic power of turbine |

---

## Control Objectives

1. **WEC power maximisation** - choose `F_PTO(t)` to maximise mean harvested power subject to stroke and force limits.
2. **FOWT load reduction** - use WEC as an active mass damper to reduce wind-turbine tower loads and accelerations.
3. **Combined optimisation** - jointly maximise renewable energy output while respecting structural fatigue margins.

---

## Relevant Control Methods

| Method | Notes |
|--------|-------|
| **Complex-conjugate control** | Optimal passive PTO for regular waves |
| **Model Predictive Control (MPC)** | Handles stroke/force constraints with wave preview |
| **LQR / LQG** | Linear state-feedback using linearised model around operating point |
| **Reactive control** | Spring-damper parameter scheduling |
| **Reinforcement Learning** | Handles irregular seas without exact wave model |

---

## Key Parameters (from paper)

| Parameter | Value / Range | Description |
|-----------|--------------|-------------|
| Water depth | >= 50 m | Offshore deepwater site |
| FOWT displacement | O(10^3) t | Typical semi-sub FOWT |
| WEC arm length | O(10) m | Articulated arm geometry |
| Wave period | 6-20 s | Typical North Sea spectrum |
| PTO damping `Cpto` | Optimised per sea state | Passive case |

---

## Python-Only Implementation

This study runs as a **Python-only case study** via `sim/main.py` (Phase 7 of `run.py`). It uses `ctrl_toolbox` Python bindings directly. No C++ compilation needed; NOT in `CMakeLists.txt` or `compile.bat`.

Plant: 4-state simplified FOWT heave + WEC arm model (Ts = 0.5 s):
- States: `[z, zdot, x_rel, xrel_dot]` - FOWT heave and WEC arm relative displacement
- Input: `F_PTO` - hydraulic PTO force
- Wave forcing: sinusoidal `F_wave(t)` driving heave; WEC resonant at T = 10 s
- Integration: RK4 at Ts = 0.5 s

---

## Controller Roster

| # | Name | lib/ Python Algorithm | Design Notes |
|---|------|--------------------|--------------|
| 1 | Passive | - | Optimal passive damping `B_opt = sqrt(k_w * m_w)`; baseline |
| 2 | Reactive | - | Cancel spring stiffness + optimal damping: `F_PTO = -k_w*x_rel + B_opt*xrel_dot` |
| 3 | PID | `ctrl.DiscretePID` | e = -x_rel (minimise relative displacement); standard tuning |
| 4 | ADRC | `ctrl.DiscreteADRC` | omega_o=0.8, Ts=0.5 -> omega_o*Ts=0.40 < 0.5 (check); ESO estimates wave forcing |
| 5 | SMC | `ctrl.DiscreteSMC` | compute(y - ref) convention; sliding on WEC velocity tracking |
| 6 | LQR | `ctrl.DiscreteLQR` | Full 4-state; LQR equilibrium compensation: u = u_ss + lqr.compute(x, x_ref)[0] |
| 7 | MPC | `ctrl.DiscreteMPC` | ZOH linearised 4-state; Np=20, Nu=5 |
| 8 | MRAC | `ctrl.MRACController` | `ctrl.set_reference(r)` then `compute(y_plant)` - NOT compute(r-y) |
| 9 | L1Adaptive | `ctrl.L1AdaptiveController` | `set_reference(r)` then `compute(y_plant)`; adapts to wave period variation |
| 10 | ILC | `ctrl.ILCController` | Periodic wave learning; trial_length = one wave period |
| 11 | DynaCtrl | `ctrl.DynaController` | Wraps PID; online SINDy model of WEC-wave coupling |
| 12 | CEM | `ctrl.CEMController` | Derivative-free NMPC via elite-sample rollout; 50 samples, 10% elite |
| 13 | ScenarioMPC | `ctrl.ScenarioMPC` | N_samples=20; Sigma_w = wave height uncertainty |
| 14 | KoopmanMPC | `ctrl.KoopmanEDMD` + `ctrl.DiscreteMPC` | EDMD lifts WEC dynamics to linear; MPC on lifted state |
| 15 | ESNCtrl | `ctrl.EchoStateNetwork` | Reservoir readout trained on wave-PTO data; W_out via ridge regression |
| 16 | CBFSafety | `ctrl.CBFSafetyFilter` | Barrier on PTO stroke limit; wraps Reactive controller |

**Total runs: 16 controllers * 5 scenarios = 80**

---

## Scenarios

| ID | Description | Wave Period | Wave Height | Notes |
|----|-------------|-------------|-------------|-------|
| s01_regular_waves | Regular wave at WEC resonance | T = 10 s | H = 2 m | Maximum power extraction point |
| s02_storm_waves | Storm condition | T = 12 s | H = 5 m | High energy; PTO force limits |
| s03_short_period | Short-period waves (off resonance) | T = 7 s | H = 1 m | Low WEC response |
| s04_irregular_wave | Bi-chromatic wave (10 s + 6 s) | Mixed | Mixed | Tests adaptation to non-periodic forcing |
| s05_freq_change | Wave period steps 10 s -> 14 s at t = 150 s | 10->14 s | H = 2 m | Tests adaptation to frequency shift |

---

## Implementation Notes

- **LQR equilibrium compensation:** Must compute `x_ref = [phi_ss, omega_ref]` and `u_ss` from steady-state WEC equations; then `u = u_ss + lqr.compute(x, x_ref)[0]`. Plain `lqr.compute(x)` will not work.
- **ADRC omega_o constraint:** With Ts = 0.5 s, require `omega_o * 0.5 < 0.5` -> `omega_o < 1.0 rad/s`. Use omega_o = 0.8.
- **MRAC/L1 convention:** `ctrl.set_reference(r)` then `ctrl.compute(y_plant)` - the controller outputs absolute F_PTO, not a correction.
- **GainScheduledController Python:** Constructor needs Ts: `ctrl.GainScheduledController(Ts)`.
- **Module path:** `sim/` sets binding path 4 levels up from `sim/`: `_ROOT = dirname(dirname(dirname(abspath(__file__))))`.
- **CSV columns:** `time, x_ref, z, zdot, x_rel, xrel_dot, F_pto, power, fowt_rms_cumul`
- **Run via:** `conda run -n soft_robotics -- python "case-study/Multi-Body Floating Wind-Wave Platform/sim/main.py"`
