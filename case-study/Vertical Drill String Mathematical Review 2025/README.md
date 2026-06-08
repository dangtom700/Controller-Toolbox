# Vertical Drill String Mathematical Review

## Reference
Jasem M. Kamel, Asan G.A. Muthalif, Abdulazim H. Falah (2025). "A review of vertical drill-string mathematical modelling." *Applications in Engineering Science* 22, 100227.

**Note:** This is a literature review paper covering lumped-parameter and distributed models of drill-string vibrations (torsional, lateral, axial). The simulation uses a 2-DOF torsional lumped-parameter model with Stribeck friction — the standard benchmark model reviewed in the paper (Section 2.1, Belokobyl'skii & Prokopov type model).

---

## System Description

A **2-DOF torsional drill-string model** representing a drilling column where the top-drive motor (surface) twists the drill string, and the drill bit experiences **Stribeck friction** from the rock formation. This friction causes the well-known **stick-slip** instability: the bit alternately sticks (zero velocity) and slips (high-speed spin), which is the primary source of premature bit wear and borehole quality degradation in oil and gas drilling.

### Plant Model

- **States:** `[phi, omega_b]` — torsional twist angle (rad) and bit angular velocity (rad/s)
- **Input:** `omega_t` — top-drive angular velocity command (rad/s); treated as a directly-tracked setpoint (surface drive dynamics not modelled)
- **Integration:** RK4 at Ts = 0.1 s
- **Stribeck friction model:** exponential Stribeck form capturing static-peak and kinetic-viscous regimes

### Governing Equations

```
dphi/dt    = omega_t - omega_b
J_b * d(omega_b)/dt = k_t*phi + c_t*(omega_t - omega_b) - T_bit(omega_b)

T_bit(omega_b) = WOB * R_b
               * [mu_k + (mu_s - mu_k) * exp(-|omega_b| / eps_v)]
               * tanh(omega_b / eps_tanh)
```

where:
- `phi` = torsional twist between top-drive and bit [rad]
- `omega_b` = bit angular velocity [rad/s]; `omega_t` = top-drive command [rad/s]
- `k_t` = string torsional stiffness [N·m/rad]
- `c_t` = torsional damping coefficient [N·m·s/rad]
- `WOB` = weight-on-bit [N]; `R_b` = bit radius [m]
- `mu_s`, `mu_k` = static and kinetic friction coefficients
- `eps_v` = Stribeck velocity decay constant [rad/s]
- `eps_tanh` = tanh smoothing width [rad/s] (prevents discontinuity at zero)

### Key Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Sampling time | Ts | 0.1 s | RK4 integration step |
| Bit inertia | J_b | 374 kg·m² | BHA + drill collar (bottom-hole assembly) |
| String stiffness | k_t | 861 N·m/rad | Torsional stiffness of drill string |
| Torsional damping | c_t | 100 N·m·s/rad | String viscous damping |
| Weight-on-bit | WOB | 97 440 N | Axial force on formation |
| Bit radius | R_b | 0.155 m | Moment arm for friction torque |
| Static friction coeff | mu_s | 0.8 | Peak Stribeck friction |
| Kinetic friction coeff | mu_k | 0.5 | Coulomb sliding friction |
| Stribeck decay | eps_v | 1.0 rad/s | Velocity scale for static→kinetic transition |
| tanh smoothing | eps_tanh | 0.05 rad/s | Prevents torque discontinuity at zero speed |

---

## Python-Only Implementation

This study runs as a **Python-only case study** via `sim/main.py` (Phase 6 of `run.py`). It uses `ctrl_toolbox` Python bindings directly. No C++ compilation needed; NOT in `CMakeLists.txt` or `compile.bat`.

The `sim/` module sets the binding path 4 levels up: `_ROOT = dirname(dirname(dirname(abspath(__file__))))`.

---

## Controller Roster

| # | Name | lib/ Python Algorithm | Design Notes |
|---|------|--------------------|--------------|
| 1 | OpenLoop | — | Fixed omega_t = omega_ref; no feedback; shows raw stick-slip |
| 2 | PID | `ctrl.DiscretePID` | Error e = omega_ref - omega_b; proportional-integral on bit speed |
| 3 | ADRC | `ctrl.DiscreteADRC` | omega_o=3.0, Ts=0.1 → omega_o*Ts=0.30 < 0.5 (check); ESO estimates Stribeck friction as total disturbance |
| 4 | SMC | `ctrl.DiscreteSMC` | compute(y - ref) convention; robust switching against stick-slip friction |
| 5 | LQR | `ctrl.DiscreteLQR` | Full-state feedback; x_ref=[phi_ss, omega_ref]; u = u_ss + lqr.compute(x, x_ref)[0] |
| 6 | MPC | `ctrl.DiscreteMPC` | ZOH linearised model; Np=20, Nu=5; handles omega_t actuator limits |
| 7 | MRAC | `ctrl.MRACController` | `set_reference(r)` then `compute(y_plant)` — NOT compute(r-y) |
| 8 | GainScheduled | `ctrl.GainScheduledController` | GainScheduledController(Ts); two PIDs blended on |omega_ref|; low-speed (high friction gain) vs. high-speed |
| 9 | L1Adaptive | `ctrl.L1AdaptiveController` | `set_reference(r)` then `compute(y_plant)`; fast Gamma adapts to unknown friction torque |
| 10 | NeuralPID | `ctrl.NeuralPID` | Online gain adaptation from bit speed error features; n_h=8, lr=1e-4 |
| 11 | ILC | `ctrl.ILCController` | Periodic reference cycle learning; trial-to-trial feedforward for stick-slip suppression |
| 12 | DynaCtrl | `ctrl.DynaController` | Wraps PID; n_collect=50, n_refit=25; SINDy error model of stick-slip dynamics |
| 13 | CEM | `ctrl.CEMController` | Derivative-free NMPC; 50 samples, 10% elite; handles non-convex friction landscape |
| 14 | ScenarioMPC | `ctrl.ScenarioMPC` | N_samples=20; Sigma_w = formation friction uncertainty |
| 15 | KoopmanMPC | `ctrl.KoopmanEDMD` + `ctrl.DiscreteMPC` | EDMD lifts torsional dynamics to linear; MPC on lifted state |
| 16 | ESNCtrl | `ctrl.EchoStateNetwork` | Reservoir trained on stick-slip data; W_out via ridge regression |
| 17 | CBFSafety | `ctrl.CBFSafetyFilter` | Barrier on omega_b overshoot; wraps PID; prevents over-speed |

**Total runs: 17 controllers × 5 scenarios = 85**

---

## Scenarios

| ID | Description | omega_ref [rad/s] | Notes |
|----|-------------|-------------------|-------|
| s01_step_ref | Step reference 0 → 10 rad/s | 10 | Basic tracking; most controllers stabilise |
| s02_slow_ramp | Slow ramp 2 → 18 rad/s | 2→18 | Tests ramp tracking and creep at low speed |
| s03_stick_slip | Low-speed target in high-friction regime | 0→4 | Hardest scenario; stick-slip most severe |
| s04_high_speed | Step to 20 rad/s | 0→20 | High-speed; Stribeck friction less dominant |
| s05_reversal | Step 10 → −5 rad/s at t = 25 s | 10→-5 | Direction reversal; bit decelerates through zero |

---

## Implementation Notes

- **LQR equilibrium compensation:** Compute `phi_ss = T_bit(omega_ref)/k_t` and `u_ss` at steady state; then `u = u_ss + lqr.compute(x, x_ref)[0]`. Plain `lqr.compute(x)` will not stabilise at a non-zero reference.
- **ADRC omega_o constraint:** With Ts = 0.1 s, require `omega_o * 0.1 < 0.5` → `omega_o < 5.0 rad/s`. Default omega_o = 3.0 is safe.
- **MRAC/L1 convention:** `ctrl.set_reference(r)` then `ctrl.compute(y_plant)` — the controller outputs absolute omega_t, not a correction.
- **GainScheduledController Python:** Constructor needs Ts: `ctrl.GainScheduledController(Ts)`.
- **Stick-slip severity:** In s03, `omega_b` oscillates between ~0 and ~2×omega_ref. IAE_cumulative is the primary metric; ADRC and L1Adaptive typically suppress stick-slip best.
- **CSV columns:** `time, omega_ref, omega_b, phi, omega_t, error, iae_cumulative`
- **Run via:** `conda run -n soft_robotics -- python "case-study/Vertical Drill String Mathematical Review 2025/sim/main.py"`

---

## Status

Python-only implementation present in `sim/main.py`. Discovered by Phase 6 of `run.py`.
