# PCM Thermal Energy Storage Control

Cost-optimal cooling control of a **Phase-Change-Material (PCM) thermal-energy
store** buffered onto a variable-speed **Heat Pump (HP)**, managing the store's
state of charge to shift cooling load into low-price hours.

## Source

- **Reference:** Y. Chen, I. Marotta, V. Palomba, T. Ohlson Timoudas, Q. Wang,
  *"Model predictive control guided imitation learning for optimal control of PCM
  thermal energy storage"*, Applied Thermal Engineering **295** (2026) 130741.
- Paper PDF in this folder (extract text with `tools/extract_text.py` if needed; extraction is lossy).

## Plant model

Single-state store **State of Charge** (SoC) integrator driven by the HP cooling
minus the building load, using the paper's reduced-order quadratic HP maps
(Eqs. 1-2) and hysteretic capacity (Sec. 2.1.2):

```
Q_hp(r,T_o) = -0.207 + 5.33e-3 r - 0.137 T_o - 7.0e-7 r^2 + 1.24e-3 T_o^2   [kW]
e_hp(r,T_o) =  0.015 + 1.2e-4 r - 9.0e-3 T_o - 1.5e-7 r^2 + 2.3e-5 r T_o + 1.1e-3 T_o^2  [kWh]
SoC[k+1]    = SoC[k] + Ts (Q_hp - P_load) / C,   C = 32 kWh (charge) | 27 kWh (discharge)
```

| Symbol | Meaning | Units |
|---|---|---|
| `SoC` | store state of charge (state), clamped [0,1] | - |
| `u` | normalised HP compressor speed (**control**, `r = 2900 u` rpm), clamped [0,1] | - |
| `T_o` | outdoor temperature (disturbance) | degC |
| `P_load` | building cooling load (disturbance) | kW |
| `price` | normalised electricity price (cost only) | - |

Time step `Ts = 1 h` (day-ahead-market resolution). Forward-Euler integrator.

### Model simplifications (control-oriented reconstruction)

1. The paper's headline method is an MPC **expert** whose optimal actions train
   two neural **imitation-learning** agents (Behavior Cloning, GAIL) against a
   high-fidelity Modelica/FMU co-simulation. That FMU is not distributed, so this
   study uses the paper's own reduced-order HP maps (Eqs. 1-2) as the plant - the
   same reduced model the MPC expert uses internally - and represents the learned
   imitation policy with the roster's neural controller (**NeuralPID**).
2. The economic objective (min `sum price*e_hp`, Eq. 3a) is realised in the
   tracking harness as a **SoC reference schedule** that pre-charges the store in
   the low-price valleys (paper Fig. 14: twice-daily charging ~2-5 am and ~12-3 pm).
   Controllers are compared on how tightly they track that schedule (**IAE** on
   SoC) and on the resulting electricity **cost** (logged).
3. `SoC in [0,1]` (Eq. 3c), `u in [0,1]` / `r in [0,2900]` rpm (Eq. 3g).

## Control objective

Track the load-shifting SoC schedule while meeting the cooling load. Primary
metric: **IAE** = integral of `|SoC_ref - SoC| dt`; secondary: cumulative
**cost** = integral of `price * e_hp dt`. Sign: positive-gain plant, `e = ref - SoC`
with positive gains. The store's true hold-point is `u ~= u_ff` (the compressor
speed whose cooling matches the load, not `u = 0`), so the model-based controllers
(MPC/GPC/LQR/ScenarioMPC) use a **load feedforward** (`load_feedforward()`,
inverting the `Q_hp` map) and supply only the tracking correction on top.

## Controllers (12)

| # | Controller | Notes |
|---|---|---|
| 1 | OpenLoop | fixed nominal compressor speed - no storage management |
| 2 | PID | integral action finds the load hold-point automatically |
| 3 | **MPC (proposed)** | **constrained 12-h-horizon expert** - the paper's headline method |
| 4 | NeuralPID | stands in for the paper's imitation-learning policy (BC / GAIL) |
| 5 | GainScheduled | PID gains scheduled on outdoor temperature |
| 6 | LQR | 1-state optimal feedback + load feedforward |
| 7 | FOPID | fractional-order PID |
| 8 | SMC | boundary-layer sliding mode (biased by the operating point) |
| 9 | ADRC | active disturbance rejection (load as estimated disturbance) |
| 10 | GPC | generalized predictive control (CARIMA) |
| 11 | LeadLag | classical lead compensator + load feedforward |
| 12 | ScenarioMPC | stochastic MPC hedging the paper's forecast uncertainty (Gaussian noise, Eq. 4) |

## Scenarios (5)

| id | Description |
|---|---|
| `s01_typical_summer` | nominal Mediterranean summer day (nominal MC/analysis scenario) |
| `s02_heatwave` | peak-season heatwave (high temp + load; store throughput-limited) |
| `s03_price_volatility` | large price peak/valley spread; rewards tight schedule tracking |
| `s04_weekend_low_load` | low-occupancy day; must avoid over-charging (SoC saturating) |
| `s05_shoulder_season` | cooler, flatter load, 3-day horizon; steady-state regulation/drift |

## Run

```
conda run -n soft_robotics -- python "case-study/PCM Thermal Energy Storage Control/sim/main.py"
```

Analysis artifacts (Monte Carlo, fault sweep, WCET, mu-analysis, HTML report) are
produced by `tools/run_analysis.py` + `tools/generate_report.py` per
`config/analysis.json`.

## CSV columns (`logs/run_<scenario>_<controller>.csv`)

`time, soc_ref, soc, u, r, T_o, P_load, price, Q_hp, e_hp, cost_cumulative, error, iae_cumulative`
