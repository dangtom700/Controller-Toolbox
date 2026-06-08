# Solar Cooker with Bottom Parabolic Reflector and Absorber

## Reference
M.A. Tawfik, Atul A. Sagade, A.A. El-Sebaii, A.M. Khallaf, N.M. Saxena, A. El-Shal, Rashad Abd Allah (2026). "Energy, exergy, and economic analysis of a box solar cooker integrated with a tracking-type bottom parabolic reflector and phase change material." *Energy* 344, 140064.

---

## Plant Model

A **box-type solar cooker** enhanced with two optional additions: a **tracking-type bottom parabolic reflector (TBPR)** that redirects ground-level solar radiation into the cooker box from below, and a **phase change material (PCM)** layer (paraffin wax) integrated into the absorber plate for thermal storage. The paper evaluates four configurations (SC1–SC4) and provides energy/exergy/economic analysis. A future simulation study would use the SC4 (TBPR + PCM) configuration as the plant and regulate cooking pot temperature via TBPR tilt angle.

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
| `T_abs(t)` | Absorber/collector plate temperature | °C |
| `T_pot(t)` | Cooking pot contents (water/food) temperature | °C |
| `T_amb(t)` | Ambient temperature (measured disturbance) | °C |

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
- `G` = global horizontal irradiance [W/m^2] — exogenous disturbance input
- `eta_box` = optical efficiency of box cooker glazing (transmittance × absorptance)
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
| `f_shade(t)` | [0, 1] | Fraction of box aperture shaded (0 = fully open, 1 = fully blocked) — optional overheating protection |

### Key Parameters

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Box aperture area | A_box | 0.25–0.50 m^2 | Double-glazed box lid |
| Box optical efficiency | eta_box | 0.50–0.65 | Glazing transmittance × absorptance |
| TBPR efficiency gain | eta_TBPR | 0.10–0.25 | Additional flux from reflector |
| Absorber absorptance | alpha_abs | 0.90–0.96 | Blackened selective coating |
| PCM latent heat | lambda_pcm | 210–250 kJ/kg | Paraffin wax |
| PCM melting temperature | T_melt | 52–58 °C | Paraffin grade |
| Cooking load mass | m_pot | 1–5 kg | Water equivalent |
| Pot heat capacity | cp_pot | 4186 J/(kg K) | Water dominant |
| Heat loss coefficient | U_loss | 5–15 W/(m^2 K) | Pot insulation quality |
| Sampling time | Ts | 10–30 s | Slow thermal dynamics |

---

## Control Objective

Regulate or track the cooking pot temperature `T_pot` to a desired setpoint (e.g., boiling at 100°C, or a lower simmering temperature), while:
- Preventing absorber overheating (`T_abs <= T_abs_max ≈ 300°C` for typical materials)
- Rejecting disturbances from cloud cover (step/ramp changes in `G_b`)
- Minimising energy waste (overcooking / boiling dry)

The actuator is either a **reflector tilt servo** (adjusting effective concentration) or a **shading mechanism** (reducing absorbed flux).

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | OpenLoop | — | Fixed tilt theta = 45° | No feedback; baseline performance in clear-sky conditions |
| 2 | PID | `DiscretePID` | Kp=0.05, Ki=0.001, Kd=0.5; e = T_ref - T_pot | Shade fraction output; clamp [0,1]; slow sampling Ts=30 s |
| 3 | ADRC | `DiscreteADRC` | omega_o=0.04, omega_c=0.013, b0=0.002 | ESO treats cloud cover + ambient T changes as disturbance; omega_o*Ts < 0.5 for Ts=30 s |
| 4 | MPC | `DiscreteMPC` | Np=20, Nu=5, rho_y=10, rho_u=1 | Uses irradiance forecast (measured G_b, persistence model); absorber constraint |
| 5 | FuzzyPID | `FuzzyPIDController` | e_max=30°C, de_max=2°C/s | Gain-scheduled for large deviation (rapid cloud clearing) vs. near-setpoint |
| 6 | SMC | `DiscreteSMC` | c=0.5, K=0.02, phi=0.5 | Robust to step irradiance changes; compute(y - ref) convention |
| 7 | GainScheduled | `GainScheduledController` | Schedule on T_pot (below/above boiling) | Low-gain at simmering, high-gain for initial heat-up |
| 8 | MRAC | `MRACController` | gamma=0.1, a_m=-0.03, b_m=0.03 | Adapts to seasonal variation in CR (reflector dust, tilt accuracy); compute(y_plant) |
| 9 | L1Adaptive | `L1AdaptiveController` | a_m=-0.03, b_m=0.03, omega_c=0.05 | Handles uncertainty in U_loss as pot fills/empties |
| 10 | NeuralPID | `NeuralPID` | n_h=6, lr=1e-5 | Online gain adaptation for different food loads (m_pot variation) |
| 11 | DynaCtrl | `DynaController` | n_collect=60, n_refit=30 | Learns solar-thermal dynamics from daily operation |
| 12 | ScenarioMPC | `ScenarioMPC` | N_samples=15, Sigma_w=cloud uncertainty | Robust to cloud prediction uncertainty; scenarios = possible G_b trajectories |

---

## Scenarios

| ID | Description | Solar Profile | Stress Factor |
|----|-------------|--------------|---------------|
| s01_clear_sky | Clear day; heat water from 20°C to 95°C | G_b=800 W/m^2 constant | Baseline; tests heat-up time |
| s02_cloud_passage | Clouds at t=300 s reduce G_b 800->200 W/m^2 for 5 min | Step disturbance | Temperature drop rejection |
| s03_simmering | Hold T_pot at 80°C (simmering), G_b varies ±30% | Sinusoidal G_b ±240 W/m^2 | Precision regulation below boiling |
| s04_morning_ramp | G_b ramps 0->900 W/m^2 over 60 min (dawn cook) | Ramp | Tracking during large input ramp |
| s05_overtemp | G_b = 1000 W/m^2 (summer noon); must limit T_abs <= 280°C | High irradiance | Absorber protection via shade actuation |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Slow thermal dynamics:** Time constants of the cooking system are 3–15 minutes. Ts = 30 s is appropriate. ADRC omega_o must satisfy `omega_o < 0.5/Ts = 0.0167 rad/s`; use omega_o = 0.013.
- **PCM nonlinearity:** The latent heat term `m_pcm * lambda_pcm * d(phi_pcm)/dt` creates a thermal plateau near T_melt ≈ 55°C where `dT_abs/dt ≈ 0`. MPC should include phi_pcm as a state and handle the bilinear melting term. For PID/ADRC, the plateau appears as a temporary plant gain drop — ADRC ESO compensates it automatically.
- **Radiation nonlinearity:** The `T_abs^4` radiation loss term creates significant nonlinearity above 100°C. Linearise at nominal T_abs = 100°C for LQR/MPC design.
- **TBPR actuator:** The TBPR tilt angle `theta` adjusts `eta_TBPR(theta)` nonlinearly (approximately `eta_TBPR * cos(theta - theta_opt)`). For MPC/LQR, linearise at nominal tracking angle. For ADRC/SMC, treat the nonlinearity as total disturbance.
- **Four configurations:** SC4 (TBPR + PCM) has the best performance per the paper. For a fair controller comparison, simulate on SC4 which has both energy inputs and the PCM storage nonlinearity.
- **Irradiance as measured disturbance:** Include `G_b(t)` as a measured disturbance in the MPC prediction model for feed-forward benefit. Use persistence forecast: `G_b(k+j|k) = G_b(k)` for j > 0.
- **Absorber constraint:** Hard output constraint `T_abs <= T_abs_max = 280°C` is safety-critical. MPC and TubeMPC can enforce this explicitly; other controllers need a manual clamp on the shade output.
- **Two-state model:** For simulation, the 2-state ODE {T_abs, T_pot} is sufficient. RK4 at Ts = 30 s with sub-stepping (10 × 3 s) gives accurate dynamics without stiffness issues.
- **CSV columns:** `t, T_ref, T_pot, T_abs, G_b, T_amb, f_shade, Q_absorbed, iae_cumulative`

---

## Status

Spec only — `sim/` not present, not registered, not built.
