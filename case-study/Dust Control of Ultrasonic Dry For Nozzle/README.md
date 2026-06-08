# Dust Suppression via Ultrasonic Dry Fog Nozzle

## Reference
Xinzhe Wang, Pengfei Wang, Yun Peng, Yongjun Li, YaFei Luo, Shilin Li (2026). "Study on the dust control of ultrasonic dry fog for nozzle." *Powder Technology* 476, 122382.

---

## Plant Model

An **ultrasonic dry fog dust suppression system** in which a high-frequency (ultrasonic) nozzle atomises water into sub-10 mum droplets (dry fog). These micro-droplets collide with and capture airborne dust particles (typically PM10 and PM2.5), causing them to agglomerate and settle. The paper develops a mathematical model linking system operating parameters (water flow rate, ultrasonic frequency, air pressure) to **dust suppression efficiency** eta.

### Governing Physics

**Droplet size distribution** (Sauter mean diameter d_32):
```
d_32 = C1 * (sigma / (rho_l * f^2))^(1/3)
```
where `sigma` = surface tension (N/m), `rho_l` = liquid density (kg/m^3), `f` = ultrasonic frequency (Hz), `C1` = empirical constant.

**Collision efficiency** between droplet (diameter d_d) and dust particle (diameter d_p):
```
eta_c = f(St, Re, d_d/d_p)
```
Stokes number `St = rho_p * d_p^2 * v_rel / (18 * mu * d_d)` governs inertial impaction.

**Overall dust suppression efficiency:**
```
eta = 1 - exp(-K * n_d * A_d * eta_c * L)
```
where `n_d` = droplet number density, `A_d` = droplet cross-section, `L` = travel path length.

### State / Output Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `C_dust(t)` | Airborne dust concentration (outlet) | mg/m^3 |
| `eta(t)` | Instantaneous dust suppression efficiency | % |
| `d_32(t)` | Sauter mean droplet diameter | mum |
| `Q_w(t)` | Water flow rate | L/min |

### Control Inputs

| Symbol | Range | Description |
|--------|-------|-------------|
| `Q_w` | [0.5, 5.0] L/min | Water supply flow rate |
| `P_air` | [0.1, 0.6] MPa | Compressed air pressure driving atomisation |
| `f_us` | [20, 100] kHz | Ultrasonic excitation frequency (if variable-frequency system) |

### Key Parameters

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Ultrasonic frequency | f | 40 kHz | Fixed or variable |
| Water surface tension | sigma | 0.072 N/m | At 20^\circC |
| Liquid density | rho_l | 1000 kg/m^3 | Water |
| Dust particle density | rho_p | 1500-2600 kg/m^3 | Coal dust / silica |
| Dust particle diameter | d_p | 1-50 mum | PM10/PM2.5 target |
| Droplet diameter (target) | d_d | 1-10 mum | Dry fog regime |
| Air velocity in duct | v_air | 0.5-3 m/s | Mine tunnel or workshop |
| Sampling time | Ts | 1-5 s | Dust sensor response time |

---

## Control Objective

Regulate the outlet dust concentration `C_dust` to below a regulatory threshold (e.g., 4 mg/m^3 for coal mines in China) by manipulating water flow rate `Q_w` and/or air pressure `P_air`. A secondary objective is minimising water consumption (operating cost).

The key challenge is a **nonlinear, time-delayed** relationship between the control input (water flow) and the measured dust concentration (sensor response lag 2-30 s depending on sensor placement distance).

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | PID | `DiscretePID` | Kp=0.5, Ki=0.05, Kd=0; e = C_ref - C_dust | Baseline; reference is target concentration (lower is better); reverse-acting (increase Q_w when C_dust > C_ref) |
| 2 | SmithPredictor | `SmithPredictor` | Inner PID + delay model L=10 s | Compensates sensor/transport delay; compute(r - y) convention |
| 3 | ADRC | `DiscreteADRC` | omega_o=0.3, omega_c=0.1, b0=0.2 | ESO estimates dust load disturbance; check omega_o*Ts < 0.5 for Ts=1 s |
| 4 | FuzzyPID | `FuzzyPIDController` | e_max=4 mg/m^3, de_max=1 mg/m^3/s | Gain-scheduled; reduces water waste at low dust levels |
| 5 | MPC | `DiscreteMPC` | Np=20, Nu=5, rho_y=10, rho_u=1 | FOPDT linearised dust transport model; output constraint C_dust <= 4 mg/m^3 |
| 6 | GPC | `GPCController` | Np=15, Nu=4, lambda=0.8 | Generalised predictive control; handles variable dust generation rate |
| 7 | MRAC | `MRACController` | gamma=0.3, a_m=-0.5, b_m=0.5 | Adapts to seasonal variation in dust load; compute(y_plant) convention |
| 8 | SMC | `DiscreteSMC` | c=1.0, K=0.3, phi=0.1 | Robust to step changes in dust generation; compute(y - ref) convention |
| 9 | L1Adaptive | `L1AdaptiveController` | a_m=-0.5, b_m=0.5, omega_c=0.5 | Fast adaptation to sudden dust burst events |
| 10 | DynaCtrl | `DynaController` | n_collect=60, n_refit=30 | Builds SINDy model of dust-Q_w relationship online |
| 11 | GainScheduled | `GainScheduledController` | Schedule on dust generation rate proxy | Two operating modes: low-dust (economy) and high-dust (full suppression) |
| 12 | NeuralPID | `NeuralPID` | n_h=6, lr=5e-5 | Online adaptation; useful when dust particle size distribution shifts |

---

## Scenarios

| ID | Description | Dust Generation | Duration |
|----|-------------|----------------|----------|
| s01_steady | Steady dust source, regulate to 4 mg/m^3 | Constant 20 mg/m^3 raw | 300 s |
| s02_step_load | Step increase in dust concentration at t=60 s | 10 -> 30 mg/m^3 | 300 s |
| s03_pulse | Periodic dust bursts (blasting simulation) every 60 s | Pulse +40 mg/m^3 for 5 s | 600 s |
| s04_ramp_down | Gradual reduction in dust activity (end-of-shift) | 25 -> 5 mg/m^3 over 150 s | 300 s |
| s05_low_water | Water supply pressure reduced 50%; test robustness | Constant 20 mg/m^3 | 300 s |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Transport delay:** Model the dust sensor as a first-order lag plus pure delay: `C_sensor(s) = K_s / (tau_s*s + 1) * exp(-L_d*s)` with L_d = 5-30 s. Discretise with ZOH for MPC/GPC prediction model.
- **Reverse-acting loop:** Higher `Q_w` reduces `C_dust`. PID error should be `e = C_ref - C_dust` but the output polarity must drive `Q_w` up when dust is high. If using `DiscretePID`, wrap with sign inversion or negate the output before clamping.
- **ADRC b0:** For a first-order plant model `dC/dt = -a*C + b*Q_w + d`, set `b0 = b = K_plant / tau_plant`.
- **Water consumption metric:** Log cumulative `integral(Q_w * dt)` per scenario for efficiency comparison.
- **SmithPredictor delay estimation:** Transport delay varies with air velocity. RLS can track delay online.
- **CSV columns:** `t, C_ref, C_dust, Q_w, P_air, eta_suppression, iae_cumulative, water_used`

---

## Status

Spec only - `sim/` not present, not registered, not built.
