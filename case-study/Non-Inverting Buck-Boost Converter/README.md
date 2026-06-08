# Non-Inverting Buck-Boost Converter - Bump-less Two-Level T-S Fuzzy PI Control

## Reference
Omid Naghash Almasi, Vahid Fereshtehpoor, Mohammad Hassan Khooban, Frede Blaabjerg (2017). "Analysis, control and design of a non-inverting buck-boost converter: A bump-less two-level T-S fuzzy PI control." *ISA Transactions* 67, 515-527.

---

## System Description

DC-DC converters must often handle input voltages that can be either lower or higher than the desired output (e.g., Li-ion battery: 2.7-4.2 V; solar panels with variable irradiance). A **non-inverting buck-boost converter** combines a buck stage and a boost stage with two switches (`S1`, `S2`), an inductor `L`, and a capacitor `C`. It provides positive output voltage and can operate in three modes: **buck** (only `S1` switching, `S2` off), **boost** (`S1` always on, `S2` switching), and **buck-boost** (both switching, avoided here due to efficiency loss).

The converter's dynamics differ fundamentally between buck and boost modes:
- **Buck mode** - minimum phase, stable, allows wide-bandwidth control.
- **Boost mode** - non-minimum phase (right-half-plane zero), conditionally stable, requires narrow bandwidth.

Using a single PI controller tuned for boost mode degrades performance in buck mode. The paper proposes a **two-level control scheme (TLCS)** with:
- Two dedicated fuzzy PI controllers (one for buck, one for boost).
- A **TSK fuzzy switch** to select the active controller.
- A **bump-less transfer modification** that eliminates output oscillations when switching.

The approach is validated by simulations and DSP-based experiments (TMS320F28335).

---

## Mathematical Model

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `i_L` | Inductor current | A |
| `v_C` | Capacitor voltage (= output voltage) | V |

### Averaged State-Space Model (Buck mode, D = duty cycle of S1)

The simulation uses the ideal lossless averaged model (no parasitic R_L or R_C):

```
L * di_L/dt = D*V_in - v_C
C * dv_C/dt = i_L - v_C/R
```

### Averaged State-Space Model (Boost mode, D = duty cycle of S2)

```
L * di_L/dt = V_in - (1-D)*v_C
C * dv_C/dt = (1-D)*i_L - v_C/R
```

The right-half-plane zero in the boost transfer function imposes a bandwidth limit.

### Small-Signal Transfer Functions (ideal model)

**Buck mode:**
```
G_buck(s) = V_in/(LC) / (s^2 + s/(R*C) + 1/(LC))
```

**Boost mode (around operating point D0, I_L0):**
```
G_boost(s) = [I_L0/L * (V_in/I_L0 - s)] / [s^2 + s/(R*C) + (1-D0)^2/(LC)]
```

Note: The paper's full model includes parasitic inductor resistance R_L and capacitor ESR R_C.
The simulation omits these for clarity; they shift the pole-zero locations slightly but do not
change the qualitative control challenge (RHP zero in boost, different dynamics in each mode).

### Bump-Less Transfer

```
e_bump1 = V_ctrl - u_Buck     -> added (scaled by K_b1) to Buck integral
e_bump2 = V_ctrl - u_Boost    -> added (scaled by K_b2) to Boost integral
```

Forces the inactive controller output to continuously track the active one, eliminating switching transients.

### Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `V_in` | 10 V | Supply voltage (constant in paper; scenarios vary V_ref) |
| Switching frequency `f_s` | 50 kHz (Ts = 20 mus) | PWM carrier |
| Inductance `L` | 50 muH | |
| Capacitance `C` | 1.8 mF | |
| Load `R_o` | 2 Omega | |
| Classic PI (buck) | Kp=0.0163, Ki=29.86 | ZOH-discretised |
| Classic PI (boost) | Kp=0.0058, Ki=15.05 | ZOH-discretised |
| Bump gains (fuzzy TLCS) | K_b1=25, K_b2=1200 | |
| Mode hysteresis band | +/-0.1 V | BUCK->BOOST when V_ref > V_in+0.1; BOOST->BUCK when V_ref < V_in-0.1 |

All parameters loaded from `config/plant_params.json`.

---

## Control Objectives

- Regulate `v_C` to track step changes in `V_ref`, including transitions where `V_ref` crosses `V_in` (mode change).
- Achieve **smooth, bump-less transitions** - no output oscillations at mode boundaries.
- Faster transient response in buck mode than a single boost-tuned PI.
- Stable operation in boost mode despite RHP zero.

---

## Scenarios

| ID | Description | V_in [V] | V_ref [V] | Mode |
|----|-------------|----------|-----------|------|
| s01_buck | Buck-mode regulation | 10 | 8 | Buck only |
| s02_boost | Boost-mode regulation | 10 | 15 | Boost only |
| s03_crossing_up | Step up: 8 V -> 15 V (crosses V_in) | 10 | 8->15 | Buck->Boost |
| s04_crossing_down | Step down: 15 V -> 4 V (crosses V_in) | 10 | 15->4 | Boost->Buck |
| s05_full | Full sequence: 8 V -> 15 V -> 4 V | 10 | 8->15->4 | Both transitions |

**Total runs: 12 controllers * 5 scenarios = 60**

---

## Controller Roster

Each controller subclasses `buck::ControllerBase`. Its `compute(v_out, v_ref, v_in)` returns
duty cycle `d \in [0, 1]`. Internally, mode is determined by comparing V_ref to V_in +/- 0.1 V hysteresis band.

| # | Name | lib/ Algorithm(s) | Design Notes |
|---|------|--------------------|--------------|
| 1 | OpenLoop | - | Fixed d = 0.5; baseline only |
| 2 | PI-Buck | `DiscretePID` | Kp=0.0163, Ki=29.86; buck gains; operates in all modes (ignores boost instability) |
| 3 | PI-Boost | `DiscretePID` | Kp=0.0058, Ki=15.05; boost gains; operates in all modes (poor in buck) |
| 4 | TLCS-ClassicPI | `DiscretePID` x2 | Buck PI + Boost PI with bump-less transfer; K_b1=100, K_b2=120 (classic TLCS) |
| 5 | FuzzyPD | `FuzzyPDController` | Feed-forward fuzzy PD; inner uMin/uMax=+/-1.0 (loose) to allow correction in both directions |
| 6 | FuzzyPID-Buck | `FuzzyPIDController` | TSK fuzzy PI tuned for buck mode only; serves as buck-only baseline |
| 7 | FuzzyPID-Boost | `FuzzyPIDController` | TSK fuzzy PI tuned for boost mode only; serves as boost-only baseline |
| 8 | TLCS-FuzzyPI | `FuzzyPIDController` x2 | **Paper result** - buck fuzzy PI + boost fuzzy PI with bump-less transfer; K_b1=25, K_b2=1200 |
| 9 | GainScheduled | `GainScheduledController` | Scheduling variable xi = V_in/V_ref; two brackets: xi>1 (buck) and xi<1 (boost) |
| 10 | ADRC | `DiscreteADRC` | b0 = V_in/(L*C_approx) approx = 1.11e8; omega_o=0.04*f_s; omega_o*Ts = 0.04*20e-6 < 0.5 |
| 11 | MPC | `DiscreteMPC` | ZOH state-space per active mode; Np=10, Nu=3; u \in [0,1] |
| 12 | LQR | `DiscreteLQR` | Bryson-tuned; x_max=[I_L_max, V_ref]; u_max=1; mode-dependent SS |

### Key Implementation Notes

- **Bump-less TLCS:** Call `inactive_ctrl.bumplessInit(d_active, e)` **every step** (not just at
  switch) so the inactive controller's integrator continuously tracks the active one. This is
  critical - doing it only at the switch instant leaves a large integrator mismatch.
- **FuzzyPID inner bounds:** The inner `FuzzyPDParams.uMin/uMax` must be `+/-1.0` (loose) so both
  overshoot and undershoot generate proportional correction. Using `[0,1]` for the inner bounds
  kills overshoot suppression. Outer `FuzzyPID` clamps to `[0,1]` (duty cycle).
- **Mode hysteresis:** BUCK->BOOST when `V_ref > V_in + 0.1 V`; BOOST->BUCK when `V_ref < V_in - 0.1 V`.
  Hold current mode within the +/-0.1 V band to prevent chatter at the boundary.
- **ADRC b0:** `b0 = V_in / L * (1/C)` approximation; for boost mode the effective gain changes.
  Use ADRC as a SISO loop with the ESO compensating mode-dependent dynamics as total disturbance.
- **Sampling time:** Ts = 20 mus (50 kHz). ADRC constraint: `omega_o * 20e-6 < 0.5` -> `omega_o < 25000 rad/s`.

---

## Metrics

Each run prints and logs:

```
[Sk | Controller]  IAE=<>  RMS_err=<>  MaxErr=<>  sat_d=<>  settle_ms=<>  wall=<> ms
```

CSV columns: `t, v_in, v_ref, v_out, i_L, d, mode, error`

CSV logs written to `case-study/Non-Inverting Buck-Boost Converter/logs/`.

---

## Build and Run

```bash
conda run -n soft_robotics -- python run.py
```

The `buck_boost_sim` target is built by `compile.bat`. Individual run:

```bash
build\case-study\"Non-Inverting Buck-Boost Converter"\buck_boost_sim.exe
```
