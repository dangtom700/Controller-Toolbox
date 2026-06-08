# Real-Time Tracking Control of Electro-Hydraulic Force Servo Systems

## Reference
Gang Shen, Zhencai Zhu, Jinsong Zhao, Weidong Zhu, Yu Tang, Xiang Li (2017). "Real-time tracking control of electro-hydraulic force servo systems using offline feedback control and adaptive control." *ISA Transactions* 67, 356–370.

---

## Plant Model

An **electro-hydraulic force servo system** (EHFS) used for material testing, hardware-in-loop simulation, and structural loading. A servo valve commands hydraulic flow to a double-acting actuator whose rod force is measured and fed back. The control objective is high-bandwidth, precise force tracking — a harder problem than position control because force depends on the actuator's environment (load stiffness) and exhibits nonlinear valve dynamics and friction.

### Physical Description

- **Servo valve:** 4/3 proportional directional valve (electrohydraulic); spool position proportional to electrical command `u_v` with bandwidth ~100–300 Hz
- **Actuator:** Double-acting hydraulic cylinder; force = `(P_A * A_A - P_B * A_B) - F_friction`
- **Load:** Test specimen modelled as a spring `k_L` (stiffness load) or a spring-mass-damper; compliance of the load determines the force-position coupling
- **Force sensor:** Load cell on actuator rod; bandwidth 500+ Hz; low-noise

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `F(t)` | Actuator output force (measured) | N or kN |
| `x_p(t)` | Piston displacement | mm |
| `v_p(t)` | Piston velocity | mm/s |
| `P_A(t)` | Cap-side chamber pressure | bar |
| `P_B(t)` | Rod-side chamber pressure | bar |
| `x_v(t)` | Servo valve spool position | mm or normalised |

### Governing Equations

**Valve spool dynamics (1st-order approximation):**
```
tau_v * dx_v/dt + x_v = k_v * u_v
```
where `tau_v = 1/(2*pi*f_v)` with valve bandwidth `f_v ≈ 100 Hz`.

**Actuator flow continuity:**
```
V_A/beta * dP_A/dt = Cd * w * x_v * sqrt((P_S - P_A)/rho)  -  A_A * v_p  -  C_t * (P_A - P_B)
V_B/beta * dP_B/dt = A_B * v_p  -  Cd * w * x_v * sqrt(P_B/rho)  -  C_t * (P_A - P_B)
```

**Piston dynamics:**
```
m_eff * dv_p/dt = A_A * P_A - A_B * P_B - k_L * x_p - B_v * v_p - F_fric(v_p)
```

**Force output:**
```
F = A_A * P_A - A_B * P_B - m_rod * g  [or F = k_L * x_p if elastic load]
```

### Key Parameters

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Supply pressure | P_S | 210 bar | Hydraulic power unit |
| Piston area | A_A | 20–50 cm^2 | Cap side |
| Rod area | A_B | 15–40 cm^2 | Rod side (A_B = A_A - pi/4 * d_rod^2) |
| Bulk modulus | beta | 1.5 GPa | Hydraulic oil (includes trapped air) |
| Valve bandwidth | f_v | 100 Hz | Typical proportional servo valve |
| Load stiffness | k_L | 1e4–1e8 N/m | Rigid to very compliant |
| Effective mass | m_eff | 10–200 kg | Piston + rod + attached mass |
| Coulomb friction | F_c | 100–500 N | Seal friction |
| Sampling time | Ts | 0.2–1 ms | High-bandwidth force servo |

---

## Control Objective

Track a desired force reference `F_ref(t)` — typically sinusoidal (fatigue testing) or arbitrary waveform (earthquake simulation, hardware-in-loop) — with:
- High bandwidth (up to 50 Hz tracking)
- Low phase lag (critical for hardware-in-loop fidelity)
- Robustness to load stiffness variation (the specimen stiffness `k_L` changes as material yields/cracks)
- Rejection of friction-induced force ripple at velocity reversal

The **Shen et al. (2017)** approach combines a **PI** base loop with an **H∞ offline designed feedback controller (ODFC)** and an online **normalised LMS (nLMS) adaptive compensator** that identifies and cancels residual nonlinear force disturbances in real time via a continuous system identification algorithm (CSIA). This three-layer architecture (PI + H∞ ODFC + nLMS) is the paper's key result and is benchmarked against the ±15 kN test rig at 0–20 Hz.

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | PID | `DiscretePID` | Kp=0.002, Ki=1.0, Kd=2e-5; e = F_ref - F | Baseline force loop; anti-windup clamp; Ts=0.5 ms |
| 2 | LQR | `DiscreteLQR` | Q=diag(1e6,100,1,1), R=1 | States {F, v_p, P_A, P_B}; designed at nominal k_L |
| 3 | LQG | `DiscreteLQG` | Q_w=diag(10,...), R_v=diag(1e4,...) | Kalman filter for pressure + velocity from load cell + encoder |
| 4 | ADRC | `DiscreteADRC` | omega_o=2000, omega_c=600, b0=K_valve | ESO treats load variation + friction as total disturbance; omega_o*Ts < 0.5 for Ts=0.5 ms -> omega_o < 1000 rad/s |
| 5 | SMC | `DiscreteSMC` | c=200, K=500, phi=5 N | Robust to Coulomb friction discontinuity; saturation function prevents chattering; compute(y - ref) |
| 6 | MPC | `DiscreteMPC` | Np=15, Nu=4, rho_y=1e6, rho_u=0.01 | Linearised valve-actuator model; pressure constraint P_B >= 0 |
| 7 | MRAC | `MRACController` | gamma=10, a_m=-300, b_m=300 | Adapts to load stiffness variation as specimen yields; compute(y_plant) |
| 8 | L1Adaptive | `L1AdaptiveController` | a_m=-300, b_m=300, omega_c=200 | Fast adaptation bandwidth; handles rapid k_L changes during crack propagation |
| 9 | FeedbackLinearisation | `FeedbackLinearisationController` | g=K_valve*A_A*sqrt(P_S/rho)/(m_eff); f=-B_v*v/m_eff | Inverts valve flow nonlinearity; inner loop P/PI controller |
| 10 | NeuralPID | `NeuralPID` | n_h=8, lr=5e-5 | Adapts force loop gains to real-time bandwidth demand |
| 11 | ILC | `ILCController` | Lp=0.6, D-type; trial_length=N | For periodic reference (sinusoidal fatigue test); learns feedforward correction trial-to-trial |
| 12 | GainScheduled | `GainScheduledController` | Schedule on |F_ref| (light vs. heavy load) | Reduces integral windup risk at small force commands |

---

## Scenarios

| ID | Description | Reference Signal | Load / Stress |
|----|-------------|-----------------|---------------|
| s01_sine_50hz | Sinusoidal force 0.5 kN amplitude, 50 Hz | F_ref = 500*sin(2*pi*50*t) N | High-frequency tracking; tests bandwidth |
| s02_sine_5hz | Sinusoidal force 5 kN amplitude, 5 Hz | F_ref = 5000*sin(2*pi*5*t) N | High amplitude; pressure saturation risk |
| s03_step | Step force commands 0 -> 10 kN at t=0.01 s | Step function | Transient response; overshoot |
| s04_stiffness_change | Sine tracking; k_L steps 1e5 -> 1e7 N/m at t=0.5 s | F_ref = 2000*sin(2*pi*2*t) N | Specimen stiffness change (specimen yielding) |
| s05_earthquake | Irregular broadband waveform (El Centro ground motion scaled to ±15 kN) | Measured earthquake data replay | Broadband; hardware-in-loop fidelity |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Fast dynamics:** Pressure dynamics have time constants of 1–5 ms. Use Ts = 0.5 ms with RK4 integration. The 5-state system {F, v_p, P_A, P_B, x_v} is appropriate.
- **ADRC omega_o constraint:** With Ts = 0.5 ms, require `omega_o * Ts < 0.5` → `omega_o < 1000 rad/s`. Use omega_o = 800, omega_c = 240.
- **Valve dead-band:** Proportional servo valves have dead-band ~0–5% of stroke. Include `x_v = max(|u| - dead_band, 0) * sign(u)` in the plant model for realism.
- **Force vs. position control:** Unlike position control where a stiff actuator is forgiving, force control on a stiff load (high k_L) becomes position control — small position error generates large force error. This makes force control on stiff specimens inherently less stable; reduce integral gain accordingly.
- **ILC applicability:** ILC is ideal for scenario s01/s02 (periodic fatigue tests). Set trial length to exactly one period of the sinusoidal reference. D-type ILC with Lp=0.6 converges in ~10 trials for clean sinusoidal reference.
- **Pressure saturation:** Enforce `P_A >= 0` and `P_B >= 0` (cavitation prevention) in plant simulation. At velocity reversal, if return chamber pressure drops to zero, the force servo loses authority suddenly — this is a key failure mode to test.
- **Load stiffness uncertainty:** The main source of model uncertainty in this system is the unknown and time-varying load stiffness `k_L`. Adaptive controllers (MRAC, L1Adaptive) are expected to outperform fixed-gain designs in scenario s04.
- **CSV columns:** `t, F_ref, F, x_p, v_p, P_A, P_B, u_v, phase_error_deg, iae_cumulative`

---

## Status

Spec only — `sim/` not present, not registered, not built.
