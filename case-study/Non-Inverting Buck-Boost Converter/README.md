```markdown
# Non-Inverting Buck-Boost Converter - Bump-less Two-Level T-S Fuzzy PI Control

## Reference
**Title:** Analysis, control and design of a non-inverting buck-boost converter: A bump-less two-level T-S fuzzy PI control  
**Authors:** Omid Naghash Almasi, Vahid Fereshtehpoor, Mohammad Hassan Khooban, Frede Blaabjerg  
**Journal:** ISA Transactions, Vol. 67, 2017, pp. 515-527  
**DOI:** (available from publisher)

---

## System Description

DC-DC converters must often handle input voltages that can be either lower or higher than the desired output (e.g., Li-ion battery: 2.7 V-4.2 V; solar panels with variable irradiance). A **non-inverting buck-boost converter** combines a buck stage and a boost stage with two switches (`S1`, `S2`), an inductor `L`, and a capacitor `C`. It provides positive output voltage and can operate in three modes: **buck** (only `S1` switching, `S2` off), **boost** (`S1` always on, `S2` switching), and **buck-boost** (both switching, avoided here due to efficiency loss).

The converter's dynamics differ fundamentally between buck and boost modes:
- **Buck mode** - minimum phase, stable, allows wide-bandwidth control.
- **Boost mode** - non-minimum phase (right-half-plane zero), conditionally stable, requires narrow bandwidth.

Using a single PI controller tuned for boost mode degrades performance in buck mode. The paper proposes a **two-level control scheme (TLCS)** with:
- Two dedicated fuzzy PI controllers (one for buck, one for boost).
- A **TSK fuzzy switch** to select the active controller.
- A **bump-less transfer modification** that eliminates output oscillations when switching between controllers.

The approach is validated by simulations and DSP-based experiments (TMS320F28335).

---

## Mathematical Model

### Small-signal transfer functions (linearised state-space averaging)

**Buck mode** (neglecting input voltage disturbances):

```
Ĝ_buck(s) = V_in/(LC) . (1 + R_C C s) / (s^2 + s/(R_o C) + 1/(LC))
```

**Boost mode**:

```
Ĝ_boost(s) = [i_L/L . (V_in/i_L - s) . (1 + R_C C s)] / [s^2 + s/(R_o C) + (1-D2)^2/(LC)]
```

where:
- `R_o` - load resistance
- `L`, `C` - inductance, capacitance
- `R_C` - equivalent series resistance of capacitor
- `D2` - duty cycle of `S2` (boost switch)
- `i_L` - inductor current (steady state)
- `V_in` - input voltage

The right-half-plane zero in the boost transfer function imposes a bandwidth limit.

### Classic PI controller

```
V_ctrl(t) = K_p e(t) + K_i \int0ᵀ e(tau) dtau
```

### Fuzzy PI controller (Takagi-Sugeno-Kang)

Fuzzy PI rules are extracted from input-output data pairs of the tuned classic PI controllers using **subtractive clustering** (no need to pre-specify number of clusters). Each rule has the form:

```
IF e is Aⁱ AND ė is Bⁱ THEN y = b0ⁱ + b1ⁱ e + b2ⁱ ė
```

The final output is the weighted average of rule consequents. The consequent coefficients are optimised with **Recursive Least Squares (RLS)** to minimise MSE.

### Bump-less transfer modification

To avoid oscillations when switching between buck and boost controllers, an additional error signal `e_bump` is defined:

```
e_bump1 = V_ctrl - u_Buck
e_bump2 = V_ctrl - u_Boost
```

These are multiplied by gains `K_b1`, `K_b2` and added to the integral term of the respective PI controller. This forces the two controller outputs to be close at the switching instant, eliminating bumps.

---

## State / Signal Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `V_in` | Input voltage (can be > or < `V_ref`) | V |
| `V_out` | Output voltage (controlled) | V |
| `V_ref` | Reference voltage | V |
| `e = V_ref - V_out` | Voltage error | V |
| `ė` | Derivative of error | V/s |
| `d` | Duty cycle (control input) | - |
| `V_ctrl` | Controller output (to PWM generator) | V |
| `u_Buck`, `u_Boost` | Outputs of buck and boost fuzzy PI controllers | V |

---

## Inputs

| Signal | Description |
|--------|-------------|
| `V_ref` | Desired output voltage (step changes: 8 V -> 15 V -> 4 V in experiments) |
| `V_out` (feedback) | Measured output voltage |
| `V_in` | Measured input voltage (used by fuzzy switch) |

## Outputs

| Signal | Description |
|--------|-------------|
| `V_out` | Regulated output voltage |
| `G1`, `G2` | PWM gate signals for `S1` (buck) and `S2` (boost) |
| Active mode indicator | Buck / Boost (determined by fuzzy switch) |

---

## Control Objectives

- Regulate `V_out` to track step changes in `V_ref` even when `V_in` crosses `V_ref` (transition between buck and boost modes).
- Achieve **smooth, bump-less transitions** - no oscillations at mode boundaries.
- Obtain **faster transient response** in buck mode than a single PI controller tuned for boost.
- Maintain stability in boost mode despite RHP zero.
- Provide a **model-free, rule-based** design that can handle nonlinearities.

---

## Relevant Control/Estimation Methods in `lib/`

| Method | Role |
|--------|------|
| **Fuzzy Logic Controller (TSK)** | Core controller - two fuzzy PI regulators (buck / boost) |
| **RecursiveLeastSquares (RLS)** | Optimisation of consequent parameters in TSK rules |
| **BayesianOptimizer** | Could replace manual tuning of `K_b1`, `K_b2` for bump-less transfer |
| **Kalman Filter** | Not used in paper, but could estimate inductor current `i_L` for improved boost control |
| **Particle Filter** | Alternative for non-Gaussian noise in PWM or sensor signals |

---

## Key Parameters

| Parameter | Value (simulation / experiment) |
|-----------|----------------------------------|
| `V_in` | 10 V (constant, reference steps 8 V -> 15 V -> 4 V) |
| Switching frequency `f_s` | 50 kHz |
| Inductance `L` | 50 muH |
| Capacitance `C` | 1.8 mF |
| Load `R_o` | 2 Omega |
| Classic PI gains (buck) | `K_p = 0.0163`, `K_i = 29.86` |
| Classic PI gains (boost) | `K_p = 0.0058`, `K_i = 15.05` |
| Bump gains `K_b1`, `K_b2` | Fuzzy: 25, 1200 ; Classic: 100, 120 |
| Rise time (buck, fuzzy TLCS) | 5.1 ms |
| Settling time (buck, fuzzy TLCS) | 9.9 ms |

---

## Scenarios

- **Step-up transition** - `V_ref` changes from 8 V (< `V_in`) to 15 V (> `V_in`): boost mode activated.
- **Step-down transition** - `V_ref` changes from 15 V to 4 V: back to buck mode.
- **Comparison** - one-level PI (boost-tuned) vs. classic TLCS vs. fuzzy TLCS.
- **Start-up transient** - from 0 V to 8 V in buck mode.
- **Load / input voltage disturbance rejection** (discussed but not the main focus).

---

## Implementation Notes

- The **TSK fuzzy switch** uses two inputs: `e_in = V_in - V_out` and `e_ref = V_ref - V_out`. Membership functions are Gaussian (Fig. 6). Four rules determine whether the buck or boost controller is active.
- **PWM generation** uses two non-overlapping triangular carriers to avoid the inefficient buck-boost mode (`V_H1 = V_L2`).
- The **bump-less modification** is implemented by adding `K_b * (V_ctrl - u_other)` to the integral accumulator of each PI. This does **not** reset the integral term, just biases it to match the other controller's output at switching instants.
- In `lib/`, a `FuzzyTSK` class could implement subtractive clustering and RLS learning. For real-time embedded use, pre-trained rule bases are stored as lookup tables.
- The controller is **model-free** after training; no online parameter estimation is required.

**Source:**  
Almasi, O.N., Fereshtehpoor, V., Khooban, M.H., Blaabjerg, F. (2017). Analysis, control and design of a non-inverting buck-boost converter: A bump-less two-level T-S fuzzy PI control. *ISA Transactions*, 67, 515-527.
```