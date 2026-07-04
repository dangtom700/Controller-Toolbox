# Air-Cooled Battery Thermal Management System

## Reference

> Zhang, J., Zhang, Z., Wu, X., Song, M., & Chen, K. (2026).
> Operation optimization of battery thermal management systems based on
> transient heat transfer model and self-adaptive control strategy.
> *Applied Thermal Engineering*, **298**, 130921.
> https://doi.org/10.1016/j.applthermaleng.2026.130921

**Authors:** Jiajun Zhang, Zhenli Zhang, Xiaoling Wu, Mengxuan Song, Kai Chen
(South China University of Technology / Shanghai Polytechnic University)

---

## Physical System

The BTMS J-U-L packs **12 * 2 prismatic lithium-ion battery cells** into a
parallel air-cooled enclosure with one inlet and three valve-controlled outlets.
Opening/closing the three outlet valves selects one of three flow patterns:

| Pattern | Active outlet(s) | Max-flow region | Best for hot-spot |
|---------|-----------------|-----------------|-------------------|
| **J**   | Right only       | Right channels  | Hot spot right    |
| **U**   | Left only        | Left channels   | Hot spot left     |
| **L**   | Both sides       | Middle channels | Hot spot center   |

The **self-adaptive control strategy** switches the flow pattern based on the
real-time position of the hottest battery cell and the temperature difference
$\Delta T$ of the pack, keeping $\Delta T < \Delta T_{\text{lim}} = 1$ K under
5C discharge and randomly varying operating conditions.

### Model Simplifications

The paper uses 2-D CFD for performance evaluation.  This case study implements
the **analytical transient heat transfer model** developed in Section 3 of the
paper (Eqs. 13-16), which achieves < 7% deviation from CFD in under 1 s of
compute time.  The 12 * 2 cell arrangement is reduced to a **1-D row of
N = 9 cells** (10 parallel channels) to match the 10-element channel-width
array tabulated in [31].

---

## Plant Equations (Eqs. 13-16)

### Battery cell *i* (thermal ODE)

$$\rho_b c_{p,b} V_b \frac{dT_{b,i}}{dt}
  = \phi_{b,i} V_b
  - h_{PC,i} S \Delta T_{\text{left},i}
  - h_{PC,i+1} S \Delta T_{\text{right},i}$$

### Air in parallel channel *j* (quasi-static, $\tau_{air} \approx 3\,\text{ms} \ll T_s$)

$$\rho_a c_{p,a} Q_{PC,j} (T_{a,\text{out},j} - T_{a,\text{in}})
  = h_{PC,j} S \left(\Delta T^{(j)}_{\text{left}} + \Delta T^{(j)}_{\text{right}}\right)$$

Solved by fixed-point iteration each time step.

### LMTD temperature differences (Eq. 15-16)

$$\Delta T_{\text{left},i}
  = \frac{(T_{b,i} - T_{a,\text{in}}) - (T_{b,i} - T_{a,\text{out},i})}
         {\ln\!\left(\frac{T_{b,i} - T_{a,\text{in}}}{T_{b,i} - T_{a,\text{out},i}}\right)},
  \quad T_{b,i} > T_{a,\text{out},i}$$

(L'Hôpital limit used when numerator and denominator -> 0.)

### Battery heat generation (Eqs. 1-2)

$$\phi_{b,i} = \frac{1}{V_b}\!\left(I^2 R - I T_b \frac{dU_e}{dT}\right)$$

$$R(\text{SOC}) = 0.00705 - 0.01853\,s + 0.05894\,s^2 - 0.09151\,s^3
                 + 0.06579\,s^4 - 0.01707\,s^5$$

---

## Parameter Table

| Symbol | Value | Units | Source |
|--------|-------|-------|--------|
| $N$ | 9 | - | Case study (1-D model) |
| $T_{a,\text{in}} = T_0$ | 298.15 | K | Table 1 / Section 2 |
| $Q_{\text{in}}$ | 0.015 | m^3/s | Section 2.1 |
| $\rho_b$ | 1542.9 | kg/m^3 | Table 1 |
| $c_{p,b}$ | 1337 | J/(kg.K) | Table 1 |
| $\lambda_b$ (x) | 1.05 | W/(m.K) | Table 1 |
| $w_b$ | 16 | mm | Section 2.1 |
| $L_b$ | 151 | mm | Section 2.1 |
| $H_b$ | 65 | mm | Section 2.1 |
| $w_{PC}$ | [2.5, 3.1*8, 2.5] | mm | Section 2.1 / [31] |
| $h_{PC,1}^J$ | 43.4 | W/(m^2.K) | Section 3.2 |
| $h_{PC,N+1}^U$ | 34.2 | W/(m^2.K) | Section 3.2 |
| $h_{PC,1}^L$ | 32.6 | W/(m^2.K) | Section 3.2 |
| $dU_e/dT$ | -0.22 | mV/K | [33] |
| $I_{1C}$ | 7.0 | A | Derived (7 Ah cell) |
| $\Delta T_{\text{lim}}$ | 1.0 | K | Section 4.1 |
| $\delta$ | 0.07 | - | Eq. 17 |
| $\Delta\varepsilon$ | 0.01 | K | Section 4.1 |
| $\Delta t = T_s$ | 1.0 | s | Section 4.1 |

---

## State Vector

| Index | Symbol | Description |
|-------|--------|-------------|
| 0-8 | $T_{b,0} \ldots T_{b,8}$ | Battery cell temperatures [K] |
| 9-18 | $T_{a,\text{out},0} \ldots T_{a,\text{out},9}$ | Channel air outlet temperatures [K] |

---

## Controller Roster (12 controllers)

| # | Name | Type | Key parameters | Notes |
|---|------|------|----------------|-------|
| 1 | OpenLoop | Fixed J | - | Baseline: no switching |
| 2 | SelfAdaptive | Position-rule | $\Delta T_{\text{thresh}} = 0.92\,K$ | Paper's strategy (Eq. 17) |
| 3 | PID | DiscretePID | $K_p=0.4,\,K_i=0.02,\,K_d=0.05$ | Threshold adaptation |
| 4 | ADRC | DiscreteADRC | $\omega_o=0.3,\,\omega_c=0.1,\,\omega_o T_s=0.30$ | ESO-based disturbance rejection |
| 5 | SMC | DiscreteSMC | $c_e=1.0,\,K=0.3,\,\phi=0.05$ | Sliding surface on $\Delta T$ |
| 6 | MPC | 1-step lookahead | Native Python, 3-mode search | Selects pattern minimising $\Delta T(t+T_s)$ |
| 7 | LQR | DiscreteLQR | 1-state $\Delta T$ model, $Q=2,R=1$ | Deviation-form |
| 8 | MRAC | MRACController | $a_m=\exp(-T_s/120),\,\gamma_r=0.01$ | Tracks $\Delta T_{\text{ref}}=0.5\,K$ |
| 9 | L1Adaptive | L1AdaptiveController | $\Gamma=0.05,\,\omega_c=0.08$ | Fast adaptation law |
| 10 | NeuralPID | NeuralPID | $K_{p0}=0.4,\,\text{lr}=5\times10^{-4}$ | Online gain adaptation |
| 11 | GainScheduled | GainScheduledController | Scheduling on SOC \in [0,1] | Aggressive at low SOC |
| 12 | ILC | ILC | P-type, $L_p=0.5,\,N=720$ | Learns over one discharge cycle |

**Sign convention:** error $e = \Delta T_{\text{ref}} - \Delta T$ (positive when below
target).  Positive $u$ raises the switching threshold (less aggressive);
negative $u$ lowers it (more aggressive).  All ctrl_toolbox controllers output
$u \in [-0.5, 0.5]\,K$; threshold $= \max(0.1,\,\Delta T_{\text{lim}} + u)$.

---

## Scenario List

| ID | Profile | Duration | Description |
|----|---------|----------|-------------|
| s01_5C_discharge | constant 5C | 720 s | Paper Section 4.1 baseline |
| s02_varying_conditions | random 5-8*10^4 W/m^3 | 3600 s | Paper Section 4.2 |
| s03_2C_steady | constant 2C | 1800 s | Mild discharge, sensitivity test |
| s04_high_rate_pulse | 7C -> 2C at 300 s | 900 s | Transient stress test |
| s05_battery_aging | 5C, $R \times 1.5$ | 720 s | Aged-cell extra heat |

---

## CSV Column Definitions

| Column | Units | Description |
|--------|-------|-------------|
| `t` | s | Simulation time |
| `DeltaT_ref` | K | Reference (target $\Delta T = 0$) |
| `DeltaT` | K | Pack temperature difference $T_{\max} - T_{\min}$ |
| `T_max` | K | Maximum cell temperature |
| `T_min` | K | Minimum cell temperature |
| `T_avg` | K | Mean cell temperature |
| `flow_pattern` | - | Active flow pattern: J, U, or L |
| `phi_b_kWm3` | kW/m^3 | Battery heat generation rate |
| `soc` | - | Approximate state of charge [0, 1] |
| `n_switches` | - | Cumulative number of flow pattern switches |
| `iae_cumulative` | K.s | Running integral of $|\Delta T|$ |

---

## Build and Run

This is a **Python-only** case study (Phase 7 of `run.py`).  No C++ compilation needed.

```bash
# Run from project root
conda run -n soft_robotics -- python "case-study/Air-Cooled Battery Thermal Management System/sim/main.py"

# Or as part of full test suite
conda run -n soft_robotics -- python run.py
```

Output CSV files are written to `case-study/Air-Cooled Battery Thermal Management System/logs/`.
