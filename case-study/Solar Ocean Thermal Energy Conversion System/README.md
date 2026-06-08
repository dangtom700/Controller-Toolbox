# Solar Ocean Thermal Energy Conversion (SOTEC) System

## Reference
Wenzhong Gao, Jiangfeng Wang, Yiping Dai, Jie Yan, Peng Liu, Rui Wang, Yansong Ying (2024). "Experimental investigation on the performance of a solar ocean thermal energy conversion system based on the organic Rankine cycle." *Applied Thermal Engineering* 245, 122776.

**Note:** This is a small-scale experimental characterisation study, not a control design paper. The actual system uses PID temperature control for the solar collector hot water loop and variable-frequency drive (VFD) speed control for the working fluid pump. The proposed controller roster below represents a potential simulation study based on the paper's plant model.

---

## Plant Model

A **small-scale Solar Ocean Thermal Energy Conversion (S-OTEC)** experimental system combining a **solar collector** (flat plate or evacuated tube) with an **Organic Rankine Cycle (ORC)** using **R134a as working fluid** and a **scroll expander** as the power device. Solar energy heats the warm source water; cold water simulates deep ocean water. The ORC converts the temperature differential into shaft power via the scroll expander. The paper characterises the system experimentally across multiple operating conditions.

### System Schematic

```
Solar Collectors -> Hot reservoir (T_h) -> ORC Evaporator -> Turbine/Expander -> Generator
                                                                       |
                                         Cold Ocean Water -> ORC Condenser -> Pump -> back
```

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `T_h(t)` | Hot-side working fluid / evaporator temperature | °C |
| `T_c(t)` | Cold-side condenser temperature | °C |
| `T_coll(t)` | Solar collector outlet temperature | °C |
| `P_net(t)` | Net ORC power output | W |
| `m_dot_wf(t)` | Working fluid (ORC) mass flow rate | kg/s |

### Governing Equations

**Solar collector energy balance:**
```
m_coll * cp_f * dT_coll/dt = G_b * eta_coll * A_coll - m_dot_f * cp_f * (T_coll - T_f_in) - U_coll * A_coll * (T_coll - T_amb)
```

**Hot reservoir (buffer tank):**
```
m_tank * cp_f * dT_h/dt = m_dot_f * cp_f * (T_coll - T_h) - Q_evap
```

where `Q_evap` = heat transferred to ORC evaporator.

**ORC thermodynamic model (steady-state sub-model, quasi-static):**
```
eta_ORC = eta_Carnot * eta_internal = (T_h - T_c) / T_h * eta_int
P_gross = eta_ORC * Q_evap
P_net = P_gross - W_pump - W_cooling_pump
```

**Working fluid flow (turbine speed proportional control):**
```
m_dot_wf = f(N_turbine, P_h, T_h)   [pump/turbine characteristic]
```

### Key Parameters

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Solar collector area | A_coll | 10–100 m^2 | Flat plate or evacuated tube |
| Collector efficiency | eta_coll | 0.55–0.75 | Depends on working fluid temperature |
| ORC working fluid | — | R134a | Refrigerant; low boiling point, scroll-expander compatible |
| Hot source temperature | T_h | 40–70°C | Solar-augmented warm water (simulates OTEC warm surface) |
| Cold sink temperature | T_c | 4–10°C | Deep ocean water (500–1000 m) |
| Temperature differential | delta_T | 30–60°C | Drives ORC efficiency |
| Nominal ORC efficiency | eta_ORC | 0.04–0.10 | Low grade heat; limited by small delta_T |
| ORC power output | P_net | 1–50 kW | Depends on scale |
| Sampling time | Ts | 10–60 s | Slow thermal + ORC dynamics |

---

## Control Objective

The paper's actual control: a **PID controller** regulates hot water inlet temperature `T_h` by adjusting the solar collector pump speed, and a **VFD** (variable-frequency drive) sets the R134a working fluid pump speed. These are the two real degrees of freedom in the experimental rig.

For a simulation study, the expanded objective is to maximise **net ORC power output** `P_net` subject to:
1. Maintaining evaporator temperature `T_h` above minimum ORC operating threshold (`T_h >= T_h_min ≈ 35°C`)
2. Preventing collector overheating (`T_coll <= T_coll_max` to protect collectors and R134a)
3. Coordinating collector pump (`m_dot_f`) and R134a pump speed (`m_dot_wf`) for maximum power point

The primary manipulated variables are:
- `m_dot_f`: collector loop pump speed (controls `T_coll → T_h`)
- `m_dot_wf`: R134a working fluid pump speed via VFD (controls `P_net`, `T_h`)
- `m_dot_cold`: cold water flow rate (controls `T_c`)

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | OpenLoop | — | Fixed pump speeds | Baseline; no feedback |
| 2 | PID (T_h) | `DiscretePID` | Kp=0.02, Ki=0.001, Kd=0; e = T_h_ref - T_h | Controls collector pump to maintain T_h; Ts=30 s |
| 3 | ADRC | `DiscreteADRC` | omega_o=0.03, omega_c=0.01, b0=0.001 | ESO lumps solar variability as disturbance; omega_o*Ts < 0.5 for Ts=30 s |
| 4 | MPC | `DiscreteMPC` | Np=20, Nu=5, rho_y=10, rho_u=1 | Linearised ORC-collector model; maximise P_net in objective; collector overheat constraint |
| 5 | LQR | `DiscreteLQR` | Q=diag(100,100,1), R=diag(1,0.1) | States {T_h, T_coll, T_c}; inputs {m_dot_f, m_dot_wf} |
| 6 | FuzzyPID | `FuzzyPIDController` | e_max=20°C, de_max=0.5°C/s | Handles nonlinear ORC efficiency curve; gain schedule near T_h_min |
| 7 | MRAC | `MRACController` | gamma=0.05, a_m=-0.02, b_m=0.02 | Adapts to seasonal variation in ocean water temperature; compute(y_plant) |
| 8 | L1Adaptive | `L1AdaptiveController` | a_m=-0.02, b_m=0.02, omega_c=0.02 | Robust adaptation to collector fouling (eta_coll drift) |
| 9 | GainScheduled | `GainScheduledController` | Schedule on G_b (cloud condition) | Morning ramp-up (low gain) vs. clear-sky steady (high gain) |
| 10 | ScenarioMPC | `ScenarioMPC` | N_samples=15, Sigma_w=solar forecast uncertainty | Robust to cloud prediction errors |
| 11 | DynaCtrl | `DynaController` | n_collect=60, n_refit=30 | Learns ORC-solar coupling from daily operation data |
| 12 | NeuralPID | `NeuralPID` | n_h=8, lr=1e-5 | Adapts to time-of-day and seasonal variation in performance map |

---

## Scenarios

| ID | Description | Solar / Ocean Conditions | Stress Factor |
|----|-------------|------------------------|---------------|
| s01_clear_day | Maximise P_net, clear sky G_b=800 W/m^2, T_ocean=6°C | Steady high irradiance | Baseline; test steady-state optimisation |
| s02_cloud_transient | G_b drops 800->150 W/m^2 for 10 min at t=300 s | Cloud shadow | Thermal buffer management; prevent ORC shutdown |
| s03_morning_startup | G_b ramps 0->900 W/m^2 over 90 min (dawn startup) | Ramp from zero | Minimum T_h threshold; startup management |
| s04_low_delta_T | T_ocean warms to 20°C (reduced sink); delta_T = 25°C only | Reduced delta_T | ORC near lower efficiency limit; maximise P_net |
| s05_fouling | Collector efficiency eta_coll degrades 10% over 2 hours | Slow drift | Adaptation to gradual performance degradation |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Very slow dynamics:** Collector and tank thermal time constants are 5–30 min. Ts = 30 s is appropriate. ADRC constraint: `omega_o < 0.5/30 = 0.0167 rad/s`; use omega_o = 0.012, omega_c = 0.004.
- **ORC quasi-static:** The ORC thermodynamic cycle is much faster than the thermal dynamics (turbine response ~1 s, thermal ~minutes). Model the ORC as a static map `P_net = f(T_h, T_c, m_dot_wf)` with look-up from the working fluid property tables.
- **Carnot limit:** Maximum theoretical efficiency = `(T_h - T_c)/T_h` with T in Kelvin. For T_h=65°C (338 K) and T_c=8°C (281 K), eta_Carnot = 16.9%. Practical ORC achieves ~40–60% of Carnot, so eta_ORC ≈ 6–10%.
- **Control degrees of freedom:** The system has 3 pumps (collector, ORC working fluid, cold water). All controllers here manipulate the collector pump `m_dot_f` as primary input, and optionally the ORC pump `m_dot_wf` as secondary.
- **Maximum power point tracking:** The optimum `m_dot_wf` that maximises P_net for a given T_h can be found by the ADRC or MPC objective. This is analogous to MPPT in PV systems.
- **Working fluid properties:** Use NIST-Refprop or CoolProp tables for enthalpy/entropy of the ORC working fluid. For simulation, a simplified polynomial fit `h = a + b*T + c*T^2` is sufficient.
- **CSV columns:** `t, T_ref, T_h, T_coll, T_c, G_b, m_dot_f_cmd, m_dot_wf_cmd, P_net, eta_ORC, iae_cumulative`

---

## Status

Spec only — `sim/` not present, not registered, not built.
