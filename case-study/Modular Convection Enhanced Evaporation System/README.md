# Modular Convection-Enhanced Evaporation System

## Reference
Mustafa F. Kaddoura, Matthew Chosa, Prakash Bhalekar, Natasha C. Wright (2021). "Modular convection-enhanced evaporation for brine management." *Desalination* 510, 115057.

---

## Plant Model

A **modular convection-enhanced evaporation (CEE) system** for brine concentration and desalination, consisting of stacked evaporation modules where heated saline water is exposed to a forced air flow to enhance evaporative mass transfer. Each module consists of a thin falling liquid film on a structured surface exposed to an air stream; multiple modules are arranged in series or parallel to scale capacity.

### Physical Description

- Saline feed water flows down a structured packing surface (falling film evaporator)
- Hot dry air is forced across the packing surface, driving evaporation
- Output is concentrated brine (reject) and humidified air carrying evaporated water vapour
- A condenser downstream recovers fresh water from the humid air

### State Variables (per module)

| Symbol | Description | Unit |
|--------|-------------|------|
| `T_w(t)` | Water film temperature | °C |
| `T_a(t)` | Air bulk temperature leaving module | °C |
| `omega_a(t)` | Air humidity ratio leaving module | kg_w / kg_dry |
| `C_s(t)` | Salt concentration in brine | g/L |
| `m_w(t)` | Water film mass flow rate | kg/s |

### Governing Equations

**Water-side energy balance:**
```
rho_w * cp_w * V_w * dT_w/dt = Q_in - h_wa * A * (T_w - T_a) - m_evap * h_fg
```

**Air-side energy and mass balance (module outlet):**
```
m_a * cp_a * dT_a/dt = h_wa * A * (T_w - T_a) + m_evap * h_fg
m_a * d(omega_a)/dt = m_evap
```

**Evaporation rate (Lewis analogy):**
```
m_evap = h_m * A * (omega_s(T_w) - omega_a)
```
where `omega_s(T_w)` = saturation humidity at film surface temperature (Antoine equation), `h_m` = convective mass transfer coefficient.

**Salt balance:**
```
d(C_s * V_w)/dt = C_s_in * m_w_in - C_s * m_w_out
```

### Key Parameters

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Air mass flow rate | m_a | 0.5–3.0 kg/s | Forced convection fan |
| Feed water flow rate | m_w | 0.1–1.0 kg/s per module | Falling film |
| Inlet water temperature | T_w0 | 40–80 °C | Pre-heated by solar or waste heat |
| Inlet air humidity | omega_in | 0.005–0.015 kg/kg | Ambient |
| Heat transfer coefficient | h_wa | 50–200 W/(m^2 K) | Depends on packing and flow |
| Packing surface area | A | 2–10 m^2 per module | Modular design |
| Number of modules | N | 2–8 | Series/parallel arrangement |
| Sampling time | Ts | 5–30 s | Slow thermal dynamics |

---

## Control Objective

Regulate the **brine outlet concentration** `C_s_out` (or equivalently the **evaporation rate** per module) to a setpoint by manipulating:
1. **Air flow rate** `m_a` (fan speed) — primary manipulated variable
2. **Feed water flow rate** `m_w` — secondary manipulated variable
3. **Inlet water temperature** `T_w_in` — tertiary (heat input from solar/waste heat)

Secondary objectives: maximise water recovery ratio (WRR = evaporate / feed), minimise fan energy consumption (operating cost), prevent salt precipitation (`C_s < C_s_max`).

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | PID (air flow) | `DiscretePID` | Kp=0.3, Ki=0.02, Kd=0; e = C_s_ref - C_s | Manipulates fan speed; primary loop |
| 2 | Cascade PID | `DiscretePID` (outer) + `DiscretePID` (inner) | Outer: C_s->T_w_ref; Inner: T_w->m_a | Inner loop thermal, outer loop concentration |
| 3 | ADRC | `DiscreteADRC` | omega_o=0.15, omega_c=0.05, b0=0.1 | ESO estimates solar irradiance disturbance; omega_o*Ts < 0.5 for Ts=10 s requires omega_o < 0.05 |
| 4 | MPC | `DiscreteMPC` | Np=20, Nu=5, rho_y=50, rho_u=1 | Multi-input (m_a, m_w); output constraint C_s <= C_s_precip |
| 5 | LQR | `DiscreteLQR` | Q=diag(100,1,10), R=diag(1,1) | Linearised multi-state model; state = [T_w, T_a, omega_a] |
| 6 | LQG | `DiscreteLQG` | Q_w=diag(0.1,...), R_v=diag(0.01,...) | Noisy temperature / concentration sensors |
| 7 | GainScheduled | `GainScheduledController` | Schedule on inlet air temperature | Winter (low T_a) vs. summer (high T_a) operating modes |
| 8 | FuzzyPID | `FuzzyPIDController` | e_max=10 g/L, de_max=2 g/L/s | Handles nonlinear evaporation rate near saturation |
| 9 | MRAC | `MRACController` | gamma=0.2, a_m=-0.1, b_m=0.1 | Adapts to seasonal changes in ambient humidity and temperature |
| 10 | DynaCtrl | `DynaController` | n_collect=80, n_refit=40 | Builds SINDy model of evaporation dynamics from operating data |
| 11 | ScenarioMPC | `ScenarioMPC` | N_samples=20, Sigma_w=solar variability | Robust MPC for stochastic solar irradiance |
| 12 | NeuralPID | `NeuralPID` | n_h=8, lr=1e-5 | Online adaptation to fouling-induced changes in h_wa |

---

## Scenarios

| ID | Description | Operating Condition | Stress Factor |
|----|-------------|--------------------|--------------| 
| s01_nominal | Regulate C_s to 70 g/L, steady solar | T_w_in=60°C, m_a=1.5 kg/s | Baseline |
| s02_solar_step | Solar input step ±20% at t=60 s | T_w_in 60->72°C | Thermal disturbance from irradiance change |
| s03_ramp_concentration | Ramp C_s setpoint from 50 to 90 g/L over 300 s | Full range traverse | Tests integral action; precipitation boundary |
| s04_fouling | h_wa degrades 30% over 600 s (packing fouling) | Nominal setpoint | Slow parametric drift; tests adaptation |
| s05_multi_module | Two modules in series; setpoint on outlet of module 2 | T_w_in=65°C | MIMO interaction; upstream disturbance propagates |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Slow dynamics:** Thermal time constants are 50–300 s. Ts = 10–30 s is appropriate; use Ts = 10 s for a reasonable balance between prediction horizon length and step count.
- **ADRC omega_o constraint:** With Ts = 10 s, require `omega_o * Ts < 0.5` → `omega_o < 0.05 rad/s`. Use omega_o = 0.04, omega_c = 0.013.
- **Salt precipitation:** Hard output constraint `C_s <= C_s_precip ≈ 200 g/L` (NaCl saturation at 60°C). MPC should enforce this as an inequality. Non-MPC controllers should saturate the setpoint.
- **Nonlinear evaporation:** The Antoine equation for `omega_s(T_w)` is exponential in T_w — significant nonlinearity above 70°C. Gain-scheduled or adaptive controllers outperform fixed-gain PID here.
- **Module coupling:** In series arrangement, the humidity and temperature of air exiting module 1 becomes the inlet condition for module 2. Model this as a cascade with shared disturbance.
- **LQR/LQG linearisation:** Linearise the 3-state per-module ODE around the nominal operating point. Use `c2d(A, B, Ts, 'zoh')` to obtain discrete-time matrices.
- **CSV columns:** `t, C_s_ref, C_s_out, T_w, T_a, omega_a, m_a_cmd, m_w, WRR, iae_cumulative`

---

## Status

Spec only — `sim/` not present, not registered, not built.
