# Solar Cooker with Bottom Parabolic Reflector and Absorber

## Reference
M.A. Tawfik, Atul A. Sagade, A.A. El-Sebaii, A.M. Khallaf, N.M. Saxena, A. El-Shal, Rashad Abd Allah (2026). "Energy, exergy, and economic analysis of a box solar cooker integrated with a tracking-type bottom parabolic reflector and phase change material." *Energy* 344, 140064.

---

## Plant Model

A **box-type solar cooker** enhanced with two optional additions: a **tracking-type bottom parabolic reflector (TBPR)** that redirects ground-level solar radiation into the cooker box from below, and a **phase change material (PCM)** layer (paraffin wax) integrated into the absorber plate for thermal storage. The paper evaluates four configurations (SC1-SC4) and provides energy/exergy/economic analysis. A future simulation study would use the SC4 (TBPR + PCM) configuration as the plant and regulate cooking pot temperature via TBPR tilt angle.

### Physical Description

- **Box cooker:** Insulated rectangular box with double-glazed lid; collects both direct and diffuse solar radiation
- **TBPR:** Parabolic reflector mounted beneath the cooker, tracking the sun; redirects additional solar flux into the box through the base glazing; increases effective aperture area without focusing to a point
- **PCM absorber:** Absorber plate with integrated paraffin wax layer; stores excess heat during peak irradiance and releases it during cloud passage or after sunset
- **Four configurations:** SC1 = baseline, SC2 = TBPR only, SC3 = PCM only, SC4 = TBPR + PCM (best performance)
- **Cooking load:** Water/food mass in insulated pot; primary controlled variable is pot temperature
- **Losses:** Convective and radiative losses from absorber and glazing to ambient

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `T_abs(t)` | Absorber/collector plate temperature | ^\circC |
| `T_pot(t)` | Cooking pot contents (water/food) temperature | ^\circC |
| `T_amb(t)` | Ambient temperature (measured disturbance) | ^\circC |

### Governing Equations

**Absorber/PCM energy balance:**
```
m_abs * cp_abs * dT_abs/dt = (eta_box + eta_TBPR * f_TBPR) * G * A_box
                             - h_abs_amb * A_abs * (T_abs - T_amb)
                             - h_abs_pot * A_c * (T_abs - T_pot)
                             - eps_abs * sigma * A_abs * (T_abs^4 - T_sky^4)
                             - m_pcm * lambda_pcm * d(phi_pcm)/dt   [PCM melting term]
```

**Cooking pot energy balance:**
```
m_pot * cp_pot * dT_pot/dt = h_abs_pot * A_c * (T_abs - T_pot) - U_loss * A_pot * (T_pot - T_amb)
```

where:
- `G` = global horizontal irradiance [W/m^2] - exogenous disturbance input
- `eta_box` = optical efficiency of box cooker glazing (transmittance * absorptance)
- `eta_TBPR` = additional efficiency contribution of TBPR (depends on tilt angle `theta`)
- `f_TBPR` = 1 if TBPR present, 0 otherwise
- `A_box` = box aperture area [m^2]
- `phi_pcm` = PCM liquid fraction [0, 1]; melting absorbs `m_pcm * lambda_pcm` [J]
- `h_abs_pot` = contact conductance between absorber and pot [W/(m^2 K)]
- `U_loss` = overall heat loss coefficient from pot [W/(m^2 K)]

### Control Input

| Symbol | Range | Description |
|--------|-------|-------------|
| `theta(t)` | [0, theta_max] degrees | TBPR tilt angle (servo motor); adjusts effective solar flux entering the box from below |
| `f_shade(t)` | [0, 1] | Fraction of box aperture shaded (0 = fully open, 1 = fully blocked) - optional overheating protection |

### Key Parameters

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Box aperture area | A_box | 0.25-0.50 m^2 | Double-glazed box lid |
| Box optical efficiency | eta_box | 0.50-0.65 | Glazing transmittance * absorptance |
| TBPR efficiency gain | eta_TBPR | 0.10-0.25 | Additional flux from reflector |
| Absorber absorptance | alpha_abs | 0.90-0.96 | Blackened selective coating |
| PCM latent heat | lambda_pcm | 210-250 kJ/kg | Paraffin wax |
| PCM melting temperature | T_melt | 52-58 ^\circC | Paraffin grade |
| Cooking load mass | m_pot | 1-5 kg | Water equivalent |
| Pot heat capacity | cp_pot | 4186 J/(kg K) | Water dominant |
| Heat loss coefficient | U_loss | 5-15 W/(m^2 K) | Pot insulation quality |
| Sampling time | Ts | 10-30 s | Slow thermal dynamics |

---

## Control Objectives

Three coupled objectives drive any control system for this plant, all derived directly from the
physical operation of the TBPR+PCM configuration (SC4):

### 1 - Maximize Solar Radiation Capture via TBPR Tracking

The TBPR is a parabolic booster reflector that must continuously track the sun's position to
redirect the maximum ground-level solar flux into the box through the base glazing. As the solar
angle changes throughout the day, the optimal tilt angle `theta_opt(t)` of the TBPR changes.

**Control action:** adjust `theta(t)` in real time to keep the reflected flux on-target.
**Why it matters:** the reflector gain is nonlinear in tilt error - even a 10^\circ deviation from
`theta_opt` can drop the TBPR contribution by 15-30%. A tracking controller that corrects for
sun motion, mechanical backlash, and wind-induced deflection is the primary performance lever.

### 2 - Maximize Net Useful Thermal Energy Against Weather Disturbances

The governing energy balance takes three exogenous climatic inputs:
- `I` (solar irradiance) - drives the heat input term
- `T_a` (ambient temperature) - sets the temperature gradient for all loss terms
- `V` (wind speed) - amplifies convective losses through both glazed covers

**Control action:** modulate TBPR tilt (or a supplementary shade) to maximise the *net* rate of
useful heat delivery to the absorber while compensating for the increased losses that high wind
or low ambient temperature impose.
**Why it matters:** a fixed-tilt or open-loop strategy over-heats on calm, sunny days and
under-performs on windy days. Closed-loop feedback on absorber or pot temperature integrates
all three disturbance effects automatically.

### 3 - Regulate PCM Charge/Discharge Cycle

The paraffin-wax PCM layer in the absorber plate acts as latent heat storage:
- **Charging mode** (high irradiance): excess solar energy melts the PCM (phi_pcm: 0->1),
  absorbing heat at constant T_melt without raising T_abs, extending the absorber's effective
  heat capacity.
- **Discharging mode** (solar drop or post-sunset): the solidifying PCM (phi_pcm: 1->0) releases
  latent heat to the pot, sustaining cooking temperatures when no radiation is available.

**Control action:** coordinate TBPR tilt to ensure the PCM charges fully during peak irradiance
and that the stored energy is released at the right rate to meet the cooking load during the
discharging window.
**Why it matters:** without regulation, the PCM may only partially charge (wasted storage
capacity) or discharge too quickly (cooking stops before the meal is done). The PCM also
introduces a thermal plateau near T_melt that acts as a temporary plant gain drop - controllers
that do not account for this will over-drive the actuator during melting.

### Summary

The core compound objective is:
> *Maximize solar capture via continuous TBPR tracking under variable irradiance, ambient
> temperature, and wind; and optimally buffer that energy through the PCM charge/discharge
> cycle to minimize cooking time, prevent absorber overheating, and extend usability into
> periods with no direct solar input.*

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | OpenLoop | - | Fixed tilt theta = 45^\circ | No feedback; baseline showing drift with sun movement and wind losses |
| 2 | PID | `DiscretePID` | Kp=0.05, Ki=0.001, Kd=0.5; e = T_ref - T_pot | Shade fraction output; clamp [0,1]; slow sampling Ts=30 s |
| 3 | ADRC | `DiscreteADRC` | omega_o=0.04, omega_c=0.013, b0=0.002 | ESO lumps cloud, wind, and PCM plateau into one extended disturbance; omega_o*Ts < 0.5 for Ts=30 s |
| 4 | MPC | `DiscreteMPC` | Np=20, Nu=5, rho_y=10, rho_u=1 | Includes irradiance persistence forecast; enforces absorber constraint explicitly |
| 5 | FuzzyPID | `FuzzyPIDController` | e_max=30^\circC, de_max=2^\circC/s | Gain-scheduled for charging plateau (low gain) vs. standard tracking (high gain) |
| 6 | SMC | `DiscreteSMC` | c=0.5, K=0.02, phi=0.5 | Robust to step irradiance changes (cloud passage); compute(y - ref) convention |
| 7 | GainScheduled | `GainScheduledController` | Schedule on T_pot (below/above T_melt, then near boiling) | Three brackets: heat-up, PCM-plateau, post-melt |
| 8 | MRAC | `MRACController` | gamma=0.1, a_m=-0.03, b_m=0.03 | Adapts to seasonal variation in reflector gain and U_loss; compute(y_plant) |
| 9 | L1Adaptive | `L1AdaptiveController` | a_m=-0.03, b_m=0.03, omega_c=0.05 | Handles uncertainty in U_loss as pot fills/empties and wind speed changes |
| 10 | NeuralPID | `NeuralPID` | n_h=6, lr=1e-5 | Online gain adaptation; learns PCM plateau timing from daily thermal signature |
| 11 | DynaCtrl | `DynaController` | n_collect=60, n_refit=30 | Builds SINDy error model of solar-thermal dynamics including PCM nonlinearity |
| 12 | ScenarioMPC | `ScenarioMPC` | N_samples=15, Sigma_w=cloud uncertainty | Robust to cloud-cover prediction error; scenarios = sampled G_b trajectories |

---

## Scenarios

Each scenario exercises one or more of the three control objectives.

| ID | Description | Solar / Weather Profile | Primary Objective Tested |
|----|-------------|------------------------|--------------------------|
| s01_clear_sky_tracking | Full day, clear sky; sun angle sweeps 6-18 h; TBPR must track | G = 800 W/m^2, T_a = 30^\circC, V = 1 m/s | Obj 1: TBPR tracking across day |
| s02_wind_disturbance | Constant sun; wind gusts V = 1->6 m/s at t = 600 s; elevated convective losses | G = 750 W/m^2, V step 1->6 m/s, T_a = 28^\circC | Obj 2: weather disturbance rejection |
| s03_pcm_charge_discharge | Peak irradiance charges PCM fully; G drops to 0 at t = 2700 s; pot must hold temperature via PCM discharge alone | G: 900->0 W/m^2 step | Obj 3: PCM charge then discharge cycle |
| s04_cloudy_morning | Intermittent cloud cover (G oscillates 200-700 W/m^2); partial PCM charge; combined disturbance | G: sinusoidal +/-250 W/m^2 around 450 W/m^2 | Obj 2 + Obj 3 combined |
| s05_overtemp_protection | Summer noon peak irradiance; must limit T_abs <= 280^\circC while still completing cooking | G = 1050 W/m^2, V = 0.5 m/s | Obj 2: absorber protection constraint |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Slow thermal dynamics:** Time constants are 3-15 minutes. Ts = 30 s is appropriate. ADRC omega_o must satisfy `omega_o < 0.5/Ts = 0.0167 rad/s`; use omega_o = 0.013.
- **PCM plateau:** The latent heat term `m_pcm * lambda_pcm * d(phi_pcm)/dt` creates a thermal plateau near T_melt approx = 55^\circC where `dT_abs/dt approx = 0`. This appears to feedback controllers as a temporary near-zero plant gain. ADRC ESO compensates automatically; PID/MPC designs should be tested through s03 specifically to verify they do not saturate during the plateau.
- **TBPR nonlinearity:** `eta_TBPR(theta)` is approximately cosine-shaped in tilt error - peak at `theta_opt(t)`, falling off on either side. For MPC/LQR, linearise at nominal tracking angle; for ADRC/SMC, treat as total disturbance absorbed by the ESO.
- **Wind as measured disturbance:** In scenarios s02/s04, `V(t)` is available as a measured input. MPC and ADRC feedforward benefit from including it. PID and SMC treat increased losses as unmeasured disturbance - their IAE will be higher but non-zero response is expected, not failure.
- **SC4 configuration:** All simulations use SC4 (TBPR + PCM), which the paper identifies as the best-performing configuration. This maximises the control challenge (both energy inputs and the PCM storage nonlinearity are active).
- **Irradiance forecast:** For MPC, use persistence forecast: `G(k+j|k) = G(k)` for j > 0. This is conservative but standard. ScenarioMPC samples G uncertainty around the persistence prediction.
- **Absorber hard constraint:** `T_abs <= T_abs_max = 280^\circC` is safety-critical. MPC/ScenarioMPC enforce it as a hard constraint in the QP. Other controllers clamp shade output manually when T_abs approaches the limit.
- **Two-state model:** For simulation, the 2-state ODE {T_abs, T_pot} is sufficient. RK4 at Ts = 30 s with sub-stepping (10 * 3 s) for accuracy without stiffness.
- **CSV columns:** `t, T_ref, T_pot, T_abs, G_b, T_amb, V_wind, phi_pcm, f_shade, Q_absorbed, iae_cumulative`

---

## Status

Spec only - `sim/` not present, not registered, not built.
