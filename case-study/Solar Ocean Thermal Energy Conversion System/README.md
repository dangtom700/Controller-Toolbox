# Solar Ocean Thermal Energy Conversion (SOTEC) System

## Reference
Wenzhong Gao, Jiangfeng Wang, Yiping Dai, Jie Yan, Peng Liu, Rui Wang, Yansong Ying (2024). "Experimental investigation on the performance of a solar ocean thermal energy conversion system based on the organic Rankine cycle." *Applied Thermal Engineering* 245, 122776.

**Note:** This is a small-scale experimental characterisation study, not a control design paper. The paper explicitly frames its findings as a benchmark reference for the control and operation of S-OTEC systems. The actual rig uses a dedicated PID controller for solar hot water temperature and variable-frequency drives (VFDs) on the working fluid and solar pumps. The proposed controller roster below represents a simulation study built on that plant model.

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

| Parameter | Symbol | Value / Range | Description |
|-----------|--------|--------------|-------------|
| ORC working fluid | — | R134a | Low boiling point refrigerant; scroll-expander compatible |
| Solar hot water temperature | T_h | 54–72 °C | Tested range; drives superheating degree ΔT_Super |
| Cold sink temperature | T_c | ~4–10 °C | Simulated deep ocean water |
| Temperature differential | delta_T | ~44–68 °C | Primary ORC driving force |
| Superheating degree | ΔT_Super | 0–18 °C | T_h above saturation; broadens expander enthalpy drop |
| Expander inlet pressure limit | P_inlet_max | **1.38 MPa** | Hard mechanical safety constraint on scroll expander |
| Nominal ORC thermal efficiency | η_th | 0.04–0.10 | Low-grade heat; raised >150% vs. non-solar OTEC by superheating |
| Control actuator 1 | m_dot_wf | via VFD | Working fluid pump speed; primary power-maximisation variable |
| Control actuator 2 | m_dot_f | via VFD | Solar collector loop pump; regulates T_h and ΔT_Super |
| Sampling time | Ts | 10–60 s | Slow thermal + ORC dynamics |

---

## Control Objectives

The paper provides the experimental benchmark; a deployed controller built around this plant
would pursue three coupled objectives derived directly from what the paper characterises.

### 1 — Maximize Net Power Output and System Efficiencies

The ultimate performance targets are:
- **W_net** — net electrical output (expander work minus all pump loads)
- **W_ele** — electrical generation delivered to load
- **η_th, η_ele, η_ex** — thermal, electrical, and exergy efficiencies

**Control action:** continuously seek the operating point where expander output heavily
outweighs working fluid pump consumption.
**Why it matters:** the ORC operating point is a function of both T_h and m_dot_wf
simultaneously. The maximum of W_net is not a simple setpoint — it is a moving optimum that
shifts as solar irradiance and ocean temperature change, requiring an MPPT-style
optimisation loop rather than a fixed reference track.

### 2 — Dynamically Regulate Working Fluid Flow Rate (m_dot_wf) Within Pressure Constraints

The paper tests how varying the R134a mass flow rate (controlled via a VFD on the working
fluid pump) changes system behaviour. Increasing m_dot_wf raises evaporation rates and
increases vapor throughput to the expander, but it simultaneously raises the expander
inlet pressure.

**Control mechanism:** variable-frequency drive on the working fluid pump.
**Hard safety constraint:** expander inlet pressure **must not exceed 1.38 MPa** — the scroll
expander's mechanical design limit. Violating this constraint damages the machine.
**Control objective:** track the optimal m_dot_wf that maximises W_net for the current T_h
while keeping P_inlet ≤ 1.38 MPa. This is a constrained optimisation inner loop.

### 3 — Optimise Superheating Degree (ΔT_Super) via Solar Heat Regulation

Unlike pure OTEC systems that use only the ocean temperature differential, this S-OTEC
system uses solar collectors to superheat the R134a above its saturation temperature before
entering the expander. The paper tests solar hot water temperatures from **54°C to 72°C**.

**Effect:** raising T_h broadens the specific enthalpy drop Δh_exp across the expander and
elevates its isentropic efficiency. The paper shows this can boost thermal efficiency by
**over 150%** compared to a non-solar baseline OTEC operating on the same cold-source.
**Control mechanism:** a PID controller (in the real rig) regulates hot water temperature
by adjusting the solar collector loop pump speed m_dot_f.
**Control objective:** dynamically adjust solar heat delivery to track the ΔT_Super that
maximises W_net given the current solar irradiance — while staying within the pressure
constraint of Objective 2, since higher T_h at fixed m_dot_wf also raises P_inlet.

### Summary

The compound control objective is an **MPPT-style optimisation loop**:
> *Sense solar hot water temperature and cold ocean state; simultaneously adjust the working
> fluid pump (m_dot_wf) and solar collector pump (m_dot_f) to extract maximum W_net while
> keeping expander inlet pressure below 1.38 MPa.*

The two VFD actuators are coupled: increasing T_h (via m_dot_f) and increasing m_dot_wf
both raise P_inlet. A controller that moves only one without the other will either leave
power on the table or trip the pressure limit.

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | OpenLoop | — | Fixed m_dot_f, m_dot_wf | Baseline at paper's nominal operating point; no MPPT |
| 2 | PID (T_h) | `DiscretePID` | Kp=0.02, Ki=0.001, Kd=0; e = T_h_ref - T_h | Replicates paper's actual rig: regulates collector pump to hold T_h; Ts=30 s |
| 3 | ADRC | `DiscreteADRC` | omega_o=0.012, omega_c=0.004, b0=0.001 | ESO lumps solar variability and ocean T drift as disturbance; omega_o*Ts < 0.5 |
| 4 | MPC | `DiscreteMPC` | Np=20, Nu=5, rho_y=10, rho_u=1 | Linearised ORC-solar model; P_inlet ≤ 1.38 MPa as hard output constraint in QP |
| 5 | LQR | `DiscreteLQR` | Q=diag(100,100,1), R=diag(1,0.1) | States {T_h, T_coll, T_c}; inputs {m_dot_f, m_dot_wf}; linearised at mid-range T_h=63°C |
| 6 | FuzzyPID | `FuzzyPIDController` | e_max=18°C, de_max=0.5°C/s | Handles nonlinear ORC efficiency curve; softer gain near P_inlet limit |
| 7 | MRAC | `MRACController` | gamma=0.05, a_m=-0.02, b_m=0.02 | Adapts to seasonal drift in ocean sink temperature and collector fouling; compute(y_plant) |
| 8 | L1Adaptive | `L1AdaptiveController` | a_m=-0.02, b_m=0.02, omega_c=0.02 | Fast adaptation to step changes in solar irradiance while maintaining pressure safety margin |
| 9 | GainScheduled | `GainScheduledController` | Schedule on T_h brackets: <58°C, 58–66°C, >66°C | Matches the three operating regions tested in paper (54, 63, 72°C hot-water temperatures) |
| 10 | ScenarioMPC | `ScenarioMPC` | N_samples=15, Sigma_w=solar forecast uncertainty | Robust to cloud prediction errors; constraint tightened to P_inlet ≤ 1.30 MPa (safety margin) |
| 11 | DynaCtrl | `DynaController` | n_collect=60, n_refit=30 | Learns ORC-solar coupling from daily m_dot_wf / T_h / P_net data |
| 12 | NeuralPID | `NeuralPID` | n_h=8, lr=1e-5 | Online gain adaptation tracking the shifting W_net optimum across T_h operating range |

---

## Scenarios

Each scenario exercises one or more of the three control objectives.

| ID | Description | Conditions | Primary Objective Tested |
|----|-------------|------------|--------------------------|
| s01_mppt_steady | Steady clear-sky; track W_net optimum by co-adjusting m_dot_f and m_dot_wf | G=800 W/m^2, T_c=6°C, T_h_range=54–72°C | Obj 1: MPPT steady-state optimisation |
| s02_solar_step | T_h steps 54→72°C (high solar superheating); must raise m_dot_wf without breaching 1.38 MPa | T_h step, fixed cold source | Obj 2 + Obj 3: pressure-constrained superheating |
| s03_cloud_transient | G_b drops 800→150 W/m^2 at t=300 s for 10 min; T_h falls; prevent ORC under-speed | Cloud shadow step | Obj 2: m_dot_wf regulation to prevent shutdown |
| s04_low_delta_T | Ocean sink warms to 20°C; reduced delta_T; must maintain P_inlet within limits at lower T_h | Reduced cold source | Obj 1: efficiency maximisation at reduced driving force |
| s05_solar_ramp | G_b ramps 0→900 W/m^2 over 90 min (dawn startup); coordinate both pumps from cold start | Irradiance ramp from zero | Obj 3: superheating regulation during startup |

**Total runs:** 12 controllers × 5 scenarios = 60.

---

## Implementation Notes

- **Very slow dynamics:** Collector and tank thermal time constants are 5–30 min. Ts = 30 s is appropriate. ADRC constraint: `omega_o < 0.5/30 = 0.0167 rad/s`; use omega_o = 0.012, omega_c = 0.004.
- **ORC quasi-static:** The thermodynamic cycle responds on ~1 s timescales; the thermal plant responds on ~minutes. Model the ORC as a static algebraic map `P_net = f(T_h, T_c, m_dot_wf)` and `P_inlet = g(T_h, m_dot_wf)` fitted from the paper's experimental data.
- **Pressure safety constraint (1.38 MPa):** This is the hardest constraint in the problem. MPC/ScenarioMPC encode it as a QP inequality `C*u ≤ d`. All other controllers must clamp m_dot_wf before it would drive P_inlet above 1.38 MPa — model the P_inlet map and enforce the clamp explicitly.
- **MPPT analogy:** The optimum m_dot_wf that maximises W_net for a given T_h moves as T_h changes. This is structurally identical to Maximum Power Point Tracking in PV systems. ESC (ExtremumSeeker) is a natural fit for this objective; MPC internalises it through the objective function.
- **Coupled actuators:** Raising T_h (via m_dot_f) and raising m_dot_wf both increase P_inlet. Controllers that move only one DOF at a time risk saturation or missed optimum. LQR/MPC exploit the coupling explicitly.
- **Three operating regions from paper:** T_h ≈ 54°C, 63°C, 72°C were the tested hot-water temperatures. GainScheduled uses these as bracket boundaries. NeuralPID and DynaCtrl learn across the full 54–72°C range.
- **Efficiency reference:** The paper reports η_th rising by >150% when solar superheating is used vs. non-solar OTEC on the same cold source. This is the quantitative benchmark a simulation controller should aim to match or exceed.
- **CSV columns:** `t, T_h_ref, T_h, T_coll, T_c, G_b, m_dot_f_cmd, m_dot_wf_cmd, P_net, P_inlet, eta_th, delta_T_super, iae_cumulative`

---

## Status

Spec only — `sim/` not present, not registered, not built.
