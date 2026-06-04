
# Water-Absorbing Bags for High-Altitude Aerial Firefighting - Drop Pattern Modelling

## Reference
**Title:** Water-absorbing bags for high-altitude aerial firefighting: Mathematical modeling and drop pattern analysis  
**Authors:** Haoran Sun, Chengyun Wu, Dachuan Zhang, Yang Wu, Renyu Hu, Yanmeng Wang, Yaru Tian, Chuyun Luo, Haitao Hu  
**Journal:** Results in Engineering, Vol. 27, 2025, Article 105940  
**DOI:** (to be assigned) - 10.1016/j.rineng.2025.105940 (inferred)

---

## System Description

Forest firefighting using fixed-wing aircraft typically requires low-altitude drops (approx =30 m) to avoid atomisation of liquid retardants, which reduces ground coverage thickness and effectiveness. Low-altitude operations are hazardous – 47 aircraft accidents (39 fatalities) in the US from 2000-2023.  

A novel solution uses **water-absorbing bags** – large encapsulated "droplets" that resist aerodynamic breakup. Bags can be dropped from higher altitudes (>=90 m) without significant atomisation, improving flight safety while maintaining ground coverage.  

The paper develops a **mathematical model** to predict bag trajectory, rotational attitude, and the resulting ground drop pattern (coverage length, width, and uniformity). The model is validated with full-scale drop tests.

---

## Mathematical Model

### Kinetic model (bag trajectory and attitude)

The bag is modelled as a rigid body with 6 degrees of freedom:

$$
d/dt [x, v, theta, omega] = f(m, I, F_{aero}, F_{gravity}, M_{aero})
$$

where:
- `x \in ℝ^3` – position vector  
- `v \in ℝ^3` – linear velocity  
- `theta \in ℝ^3` – Euler angles (attitude)  
- `omega \in ℝ^3` – angular velocity  
- `m` – bag mass (water + bag material)  
- `I` – inertia tensor  
- `F_aero` – aerodynamic force (dependent on orientation, wind)  
- `M_aero` – aerodynamic moment  

The aerodynamic forces are computed from the relative wind velocity and bag shape.

### Ground pattern model

Given the impact point distribution (from Monte Carlo runs of the kinetic model with initial condition uncertainties), the ground coverage is characterised by:

- **Pattern length** (distance from first to last impact along flight direction)  
- **Pattern width** (lateral spread)  
- **Coverage level** (e.g., gallons per 100 ft^2, or thickness in mm)

The model outputs a 2D coverage map `C(x,y)`.

### Validation metrics

| Quantity | Prediction deviation (vs. full-scale test) |
|----------|---------------------------------------------|
| Pattern length | 20%, 16.1%, 18.75% (three test cases) |
| 95% pattern width | -0.2 m, +0.1 m, -0.8 m |

---

## State / Signal Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `(x, y, z)` | Bag position (downrange, lateral, altitude) | m |
| `(v_x, v_y, v_z)` | Bag velocity components | m/s |
| `(phi, theta, ψ)` | Roll, pitch, yaw angles | rad |
| `(p, q, r)` | Angular velocities | rad/s |
| `C(x,y)` | Ground coverage level | mm or GPC |
| `L_pattern` | Drop pattern length | m |
| `W_pattern` | Drop pattern width (e.g., 95% quantile) | m |

---

## Inputs

| Signal | Description |
|--------|-------------|
| `h_drop` | Release altitude above ground (e.g., 30–120 m) |
| `V_aircraft` | Aircraft speed and heading at release |
| `wind` | Wind velocity profile (speed, direction, turbulence) |
| Bag properties | Mass, dimensions, inertia, aerodynamic coefficients |
| Water absorption | Saturated mass (water + bag) |

## Outputs

| Signal | Description |
|--------|-------------|
| `C(x,y)` | Predicted ground coverage map |
| `L_pattern`, `W_pattern` | Pattern length and width |
| Impact point distribution | Ensemble of landing positions (for probabilistic planning) |

---

## Control / Estimation Objectives

This is a **mission planning and design optimisation** problem, not real-time control. The model supports:

1. **Airdrop mission planning** – choose drop height and release parameters to achieve required coverage while maximising flight safety.
2. **Bag design optimisation** – adjust bag shape, mass, and water absorption to minimise pattern dispersion.
3. **Real-time trajectory estimation** (optional) – if bags carry sensors, estimate impact point from initial release conditions.

---

## Relevant Control/Estimation Methods in `lib/`

| Method | Role |
|--------|------|
| **Extended Kalman Filter (EKF)** | Estimate bag trajectory from noisy onboard accelerometer/GPS (if equipped) |
| **Particle Filter** | Handle non-Gaussian wind turbulence and model uncertainties |
| **Bayesian Optimizer** | Tune bag design parameters (mass, shape) to minimise pattern width |
| **Gaussian Process Regression** | Build surrogate model of pattern coverage as function of drop height and wind |
| **Monte Carlo simulation** | Propagate initial condition uncertainties to impact distribution |

---

## Key Parameters

| Parameter | Typical Range / Value |
|-----------|------------------------|
| Drop altitude | 30–120 m (conventional low: 30 m; bag high: 90–120 m) |
| Aircraft speed | 60–120 m/s (typical for airtankers) |
| Bag mass (wet) | 10–50 kg (depending on water absorption) |
| Wind speed | 0–15 m/s (critical for lateral drift) |
| Prediction error (length) | approx =20% (validated) |
| Prediction error (width) | <1 m (validated) |

---

## Scenarios

- **High-altitude drop** (90 m) vs. conventional liquid retardant (30 m) – comparison of ground coverage and safety.
- **Crosswind sensitivity** – pattern elongation and lateral shift.
- **Bag design optimisation** – minimise pattern width for a given drop height.
- **Monte Carlo uncertainty quantification** – wind gusts, release timing errors.

---

## Implementation Notes

- The kinetic model is a set of ODEs (6-DOF rigid body). Solve with **Runge-Kutta (RK4)** or adaptive solver (e.g., `scipy.integrate.solve_ivp`).
- Aerodynamic coefficients (drag, lift, moment) can be obtained from CFD or wind tunnel tests.
- The ground pattern model is essentially a **2D histogram** of impact points from multiple simulations (Monte Carlo). For real-time use, precompute a lookup table mapping `(h_drop, V_aircraft, wind)` to pattern parameters.
- The `lib/` classes can be used:
  - `ParticleFilter` for online trajectory prediction with wind uncertainty.
  - `BayesianOptimizer` to find bag mass/shape that minimises `W_pattern` subject to `L_pattern <= max_length`.
  - `MonteCarloSimulator` to generate impact distributions.

**Source:**  
Sun, H., Wu, C., Zhang, D., Wu, Y., Hu, R., Wang, Y., Tian, Y., Luo, C., Hu, H. (2025). Water-absorbing bags for high-altitude aerial firefighting: Mathematical modeling and drop pattern analysis. *Results in Engineering*, 27, 105940.
