# Solar-Driven Cooling System with Photovoltaic Evaporative Chimney

**Reference:** Ruiz, Martinez, Aguilar & Lucas, "Analytical modelling and optimisation of a
solar-driven cooling system with photovoltaic evaporative chimney," *Applied Thermal Engineering*
245, 2024.

---

## Plant Model

An algebraic steady-state plant representing a building cooling system powered entirely by
solar energy. Three sub-systems are solved sequentially each time step:

1. **Evaporative zone + vapor-compression chiller** - spray water cools the condenser inlet
   via direct evaporation; chiller coefficient of performance depends on condenser and
   evaporator temperatures.
2. **Photovoltaic panel in the chimney airflow** - solar irradiance generates electricity
   that offsets the compressor and pump consumption.
3. **Condenser hydraulic loop** - a VFD-controlled pump recirculates condenser water;
   pump-curve / system-curve intersection determines actual flow rate.

The global metric is **net grid EER** (energy efficiency ratio):
```
EER_grid = Q_evap / (W_comp + W_pump - W_PV)
```

A higher EER_grid means more cooling delivered per unit of grid electricity purchased.

### Control Inputs

| Symbol | Description | Valid Range |
|--------|-------------|-------------|
| m_dot_w | Chimney spray water mass flow | [0.02, 0.22] kg/s |
| kr | Pump VFD speed ratio | [0.30, 1.00] |

### Weather Disturbances (inputs)

| Symbol | Description | Unit |
|--------|-------------|------|
| G | Solar irradiance | W/m^2 |
| T_amb | Ambient dry-bulb temperature | ^\circC |
| phi_amb | Ambient relative humidity | - |
| v_w | Wind speed | m/s |

### Primary Controlled Variable

- **Tw1** - warm condenser water temperature [^\circC] returning from the chiller to the evaporative
  chimney. Higher Tw1 reduces chiller EER; lower Tw1 requires more spray water.

---

## Sub-System Equations

### Evaporative Zone (Poppe Method)

Water-to-air heat and mass transfer through the chimney packing, integrated by RK4:

```
dTw/dma = (cp_w * Tw - h_fg * beta) / (Le * (h_s_ma - h_ma))   [Eq. 13-18]

Merkel number (packing correlation): Me = a * (mw/ma)^b
Air mass flow:       ma = a*mw^2 + b*mw + c
```

### Chiller EER Model (quadratic surface)

```
EER = a + b*Tw2e + c*Tw2e^2 + d*Tw1 + e*Tw1^2 + f*Tw2e*Tw1   [Eq. 20]

Q_cond = m_dot_w * cp_w * (Tw1 - Tw2)
```

Iterative solve for equilibrium Tw1 at the current spray rate and irradiance.

### PV Panel Energy Balance (4-layer model)

```
Layers: glass | PV cell | tedlar | air stream
Energy balance at each layer [Eqs. 1-8]:
  G * tau_glass = Q_cell + Q_glass_out
  Q_cell = G * eta_PV + conduction to tedlar

PV efficiency:  eta_PV = eta_ref * (1 - beta_ref * (Tc - T_ref))
PV power:       W_PV = G * A_PV * eta_PV
```

### Pump / Hydraulic Loop

```
System curve: Hm_sys = sys_a + sys_b * Q^2
Pump curve:   Hm_pump = kr^2 * H0 * (1 - (Q / (kr*Q0))^2)   [Eqs. 22-27]
Intersection gives actual flow Q; pump power W_pump = rho*g*Q*Hm / eta_p

Pump efficiency (parabolic BEP model):
  ratio = Q / (kr * Q0)
  eta_p = eta_p0 * ratio * (2 - ratio)      [= eta_p0 at rated flow, 0 at no flow]
```

### Key Plant Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| A_PV | (from JSON) | PV panel area [m^2] |
| eta_ref | (from JSON) | PV reference efficiency at T_ref, G_ref |
| beta_ref | (from JSON) | PV temperature coefficient [1/^\circC] |
| T_ref | 25 ^\circC | STC reference temperature |
| Q_evap_kW | (from JSON) | Chiller design cooling capacity [kW] |
| T_w2_evap | (from JSON) | Chiller evaporator outlet temperature [^\circC] |
| H0, Q0 | (from JSON) | Pump rated head [m] and flow [l/s] |
| eta_p0 | (from JSON) | Pump rated efficiency |

All parameters loaded from `config/plant_params.json`.

---

## Context and Motivation

Building HVAC accounts for roughly 40% of global electricity consumption; vapor-compression
cooling is the largest single load in hot climates. By coupling a solar chimney (passive
airflow + evaporative cooling) with a PV panel that partially offsets the compressor,
the system aims for near-net-zero grid consumption on peak irradiance days.

The control challenge is that **spray water flow** (the primary manipulated variable) affects
three competing objectives simultaneously:
- Higher m_dot_w cools the condenser inlet -> better chiller EER
- Higher m_dot_w pumps more water -> higher pump energy W_pump
- Higher chimney air flow -> higher PV cell temperature -> lower eta_PV

The net-EER optimum is therefore a non-trivial function of irradiance, humidity, and
temperature - making it a natural benchmark for extremum-seeking control, adaptive control,
and model-predictive control with model uncertainty.

---

## Scenarios

| ID | Description | G [W/m^2] | T_amb [^\circC] | Humidity | Setpoint Tw1 |
|----|-------------|----------|-----------|----------|--------------|
| s01_design_steady | Design-point steady-state regulation | 920 | 35.6 | 0.35 | 40 ^\circC |
| s02_irradiance_ramp | Rising irradiance 250->1000 W/m^2 over 1 hr (FF-PID advantage) | Ramp | 30 | 0.50 | 40 ^\circC |
| s03_cloudy_disturbance | Cloud step: G drops 900->500 W/m^2 for t=[600,1800] s then recovers | 900/500 | 28 | 0.60 | 38 ^\circC |
| s04_setpoint_step | Setpoint step 42->38 ^\circC at t=1800 s | 750 | 32 | 0.45 | 42->38 ^\circC |
| s05_high_humidity | High humidity reduces evaporative effectiveness | 800 | 38 | 0.90 | 40 ^\circC |

**Total runs: 9 controllers x 5 scenarios = 45**

Scenario s05 (high humidity, phi=0.90) is the harshest condition: the wet-bulb temperature
approaches the dry-bulb, limiting evaporative cooling. Controllers designed for nominal
conditions may saturate m_dot_w without achieving the setpoint.

---

## Controller Roster

Each controller subclasses `SolarControllerBase`. Its `compute(Tw1_measured, ref_Tw1)`
returns `m_dot_w [kg/s]`. The pump speed `kr` is either fixed or updated separately.

The runner calls `controller->setLastEER(EER_grid)` after each plant step so ESC can
feed the plant efficiency metric back without changing the standard `compute()` interface.

| # | Name | lib/ Algorithm(s) | Design Notes |
|---|------|--------------------|--------------|
| 1 | PID | `DiscretePID` | PI on Tw1 error; kr fixed at 0.85 |
| 2 | FFPID | `DiscretePID` + solar feedforward | Feedforward: dm_dot_w = G_to_mw * G [1.5e-4 kg/s per W/m^2]; kr scheduled: 0.80 + 0.35*(e/10) |
| 3 | MPC | `DiscreteMPC` | FOPDT model: a=0.018, b=4.5 [Tw1[k+1]=(1-a*Ts)*Tw1[k]+b*Ts*m_dot_w]; Np=30, Nc=8; kr=0.85 |
| 4 | ADRC | `DiscreteADRC` | b0=4.5, omega_o=0.04 rad/s; omega_o*Ts=0.40 < 0.5 (backward-Euler stable); ESO estimates irradiance disturbance; kr=0.85 |
| 5 | FuzzyPID | `FuzzyPID` | 25-rule Mamdani; e_scale=10^\circC, de_scale=1^\circC/step, u_scale=0.10 kg/s; kr=0.85 |
| 6 | SmithPredictor | `SmithPredictor` | 2-step dead-time (~20 s thermal lag); inner PID Kp=0.003, Ki=0.0003; FOPDT inner model a=0.018, b=4.5; kr=0.85 |
| 7 | MRAC | `MRACController` | Sigma-modification adapts spray-flow gain; reference model a_m=0.70, b_m=0.30; compute(y_plant) not error; kr=0.85 |
| 8 | ESC | `ExtremumSeeker` | Maximizes EER_grid via m_dot_w perturbation (seekMinimum=false, cost=-EER_grid); EER fed back via setLastEER(); inner PID keeps Tw1 near ref; m_dot_w in [0.02, 0.22]; kr=0.85 |
| 9 | GPC-RLS | `GeneralizedPredictiveController` + `RecursiveLeastSquares` | Np=20, Nu=5; RLS na=1, nb=1, lambda=0.98; updates every 20 steps after 50-step warmup; tracks gain variation with irradiance; kr=0.85 |

### Key Implementation Notes

- **MRAC sign convention:** `compute(y_plant)` - passes the raw plant output, not the error.
  This is the MRAC convention (`setReference(ref)` called separately each step).
- **ESC feedback path:** `SolarControllerBase` declares `virtual void setLastEER(double)` as
  a no-op. Only `ESCSolarCtrl` overrides it. This is the pattern for any "controller needs a
  plant metric that isn't the tracked output" (TK26-7).
- **s05 high-humidity saturation:** At phi=0.90, evaporative effectiveness drops sharply.
  Most controllers will saturate m_dot_w at 0.22 kg/s and still miss the setpoint. This is
  a physically correct outcome, not a controller failure.
- **ADRC stability:** omega_o = 0.04 rad/s with step size Ts gives omega_o*Ts = 0.40 < 0.50,
  within the backward-Euler LADRC stability bound.

---

## Metrics

Each run prints and logs:

```
EER_grid=<>  IAE_Tw1=<>  ISE_Tw1=<>  W_net_kW=<>  m_dot_w_mean=<>
```

CSV logs written to `case-study/Solar-Driven Cooling System with Photovoltaic Evaporative Chimney/logs/`.
`check_logs.py` in the logs directory can aggregate results across scenarios.

---

## Build and Run

This case study uses a C++ simulation (`solar_cooling_sim` target) built via cmake.
A parallel Python implementation is in `module/` for exploratory analysis.

```bash
conda run -n soft_robotics -- python run.py
```

The `solar_cooling_sim` target is built by `compile.bat`. Individual run:

```bash
build\case-study\"Solar-Driven Cooling System with Photovoltaic Evaporative Chimney"\solar_cooling_sim.exe
```

Python prototype (standalone, does not use ctrl_toolbox bindings):
```bash
conda run -n soft_robotics -- python "case-study/Solar-Driven Cooling System with Photovoltaic Evaporative Chimney/module/solar_cooling_sim.py"
```
