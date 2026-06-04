# Optimizing CO2 Mitigation and Biodiesel Production Using *Bacillus cereus* in a Bubble Column Bioreactor

## Reference
**Title:** Optimizing CO2 mitigation and biodiesel production using *Bacillus cereus* in bubble column bioreactor: A mathematical modeling approach for industrial upscaling  
**Authors:** Rachael Jovita Barla, Suresh Gupta, Smita Raghuvanshi  
**Journal:** Bioresource Technology Reports, Vol. 34, 2026, Article 102779  
**DOI:** https://doi.org/10.1016/j.biteb.2026.102779

---

## System Description

Industrial flue gas CO2 emissions continue to rise, requiring sustainable carbon capture and utilisation technologies. **Chemolithotrophic bacteria** such as *Bacillus cereus* SSLMC2 can sequester CO2 and convert it into lipids, which are then transesterified to biodiesel. This approach operates under ambient conditions, avoiding the energy penalty of conventional capture technologies.

The system uses a **bubble column bioreactor** – a simple, low-shear design suitable for bacterial growth – where CO2-rich gas is sparged from the bottom. The bacteria consume dissolved CO2 and O2, producing biomass rich in intracellular lipids. The paper develops a **mathematical model** (solved with MATLAB `pdepe`) that predicts:

- Substrate uptake rates (CO2, O2, nutrients)
- Biomass formation and lipid accumulation
- Optimal operating parameters for maximum CO2 bio-fixation and biodiesel yield

The model is used to optimise process parameters for industrial upscaling, balancing biomass productivity, lipid content, and biodiesel quality (FAME profile meeting international standards).

---

## Mathematical Model

### 1. Mass balance equations (bubble column, axial dispersion)

For each component (biomass, dissolved CO2, dissolved O2, nutrients):

$$
dC_i/dt = D_ax . d^2C_i/dz^2 - u_g . dC_i/dz + r_i(C, T, P)
$$

where:
- `C_i` – concentration of component *i* (g/L or mol/m^3)
- `D_ax` – axial dispersion coefficient (m^2/s)
- `u_g` – superficial gas velocity (m/s)
- `z` – axial coordinate (m)
- `r_i` – reaction term (production/consumption rate)

### 2. Bacterial growth kinetics (Monod-type with inhibition)

$$
mu = mu_max . [S_CO2/(K_S_CO2 + S_CO2)] . [S_O2/(K_S_O2 + S_O2)] . I(T, pH)
$$

where:
- `mu` – specific growth rate (h^-^1)
- `mu_max` – maximum specific growth rate
- `S_CO2, S_O2` – dissolved substrate concentrations
- `K_S` – half-saturation constants
- `I(T, pH)` – inhibition factors (e.g., temperature deviation)

### 3. Lipid accumulation and biodiesel yield

Lipid productivity (g.L^-^1.h^-^1) is linked to growth phase and nutrient limitation (e.g., nitrogen stress). After harvest, transesterification converts lipids to fatty acid methyl esters (FAME):

$$
Biodiesel yield = (mass of FAME) / (mass of dry biomass) * 100%
$$

The paper reports:
- Lipid content = 56% of dry biomass
- Lipid productivity = 0.1 g.L^-^1.h^-^1
- Biodiesel content = 89.3%
- Biodiesel productivity = 0.2 g.L^-^1.h^-^1

### 4. Numerical solution

The PDE system is solved using MATLAB's `pdepe` solver for 1D transient problems. The solver optimises parameters (temperature, pressure, gas flow rate, inlet CO2/O2 concentrations) to maximise biomass concentration and CO2 fixation rate.

---

## State / Signal Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `X` | Biomass concentration | g.L^-^1 |
| `S_CO2` | Dissolved CO2 concentration | mg.L^-^1 or % |
| `S_O2` | Dissolved O2 concentration | mg.L^-^1 |
| `S_N` | Nutrient (e.g., nitrogen) concentration | g.L^-^1 |
| `L` | Lipid concentration | g.L^-^1 |
| `mu` | Specific growth rate | h^-^1 |
| `P_biodiesel` | Biodiesel productivity | g.L^-^1.h^-^1 |
| `Y_CO2` | CO2 fixation efficiency | % or g.g^-^1 |

---

## Inputs

| Signal | Description |
|--------|-------------|
| Inlet gas composition | CO2, O2, N2 fractions (e.g., 5–15% CO2 from flue gas) |
| Gas flow rate | Superficial velocity (m/s) or volumetric (L/min) |
| Temperature | Bioreactor temperature (K or ^\circC) |
| Pressure | Headspace or hydrostatic pressure (bar) |
| Nutrient feed | Nitrogen, phosphorus, trace metals |
| Inoculum size | Initial bacterial concentration |

## Outputs

| Signal | Description |
|--------|-------------|
| `X(t,z)` | Predicted biomass concentration profile |
| `S_CO2(t,z)` | Dissolved CO2 profile – determines fixation rate |
| `L(t)` | Total lipid concentration (for downstream biodiesel) |
| Biodiesel FAME profile | Composition (C14:0, C16:0, C18:1, etc.) and compliance with EN 14214 / ASTM D6751 |
| Optimal operating setpoints | T, P, gas flow rate, dilution rate (for continuous operation) |

---

## Control / Estimation Objectives

This is a **process optimisation and scale-up** problem, but the model can underpin:

1. **Model-based control** – maintain dissolved CO2 and O2 at setpoints that maximise lipid productivity (e.g., via gas flow rate or inlet composition).
2. **Real-time estimation** – infer biomass and lipid concentrations from dissolved O2 uptake rate (indirect sensing).
3. **Industrial upscaling** – predict performance of larger bubble columns (diameter, height) based on pilot-scale validated model.
4. **Feed-forward optimisation** – adjust nutrient feed to trigger lipid accumulation (nitrogen stress) at optimal growth phase.

---

## Relevant Control/Estimation Methods in `lib/`

| Method | Role |
|--------|------|
| **Extended Kalman Filter (EKF)** | Estimate unmeasured states (biomass, lipid) from online DO and pH sensors |
| **Model Predictive Control (MPC)** | Regulate gas flow and nutrient feed to maximise CO2 fixation while avoiding oxygen limitation |
| **RecursiveLeastSquares** | Online update of kinetic parameters (mu_max, K_S) as bacterial strain evolves |
| **BayesianOptimizer** | Find optimal temperature/pressure/flow rate that maximises biodiesel productivity |
| **Parameter Estimation (e.g., `scipy.optimize`)** | Fit Monod parameters to batch experimental data |

---

## Key Parameters

| Parameter | Typical Value / Range (from paper) |
|-----------|-------------------------------------|
| Lipid content | 56% (w/w of dry biomass) |
| Lipid productivity | 0.1 g.L^-^1.h^-^1 |
| Biodiesel content | 89.3% (from transesterification) |
| Biodiesel productivity | 0.2 g.L^-^1.h^-^1 |
| CO2 concentration in inlet gas | 5–15% (v/v) |
| Temperature | 25–37 ^\circC (optimal for *B. cereus*) |
| Gas flow rate | 0.5–2 vvm (volume per volume per minute) |
| Reactor height-to-diameter ratio | Typical bubble column: 3–10 |

---

## Scenarios

- **Batch vs. continuous operation** – model predicts steady-state biomass and lipid productivity for continuous mode.
- **Flue gas composition variation** – simulate CO2 fixation when inlet gas changes (e.g., 5% CO2 from natural gas vs. 15% from coal).
- **Nitrogen stress for lipid accumulation** – reduce nitrogen feed to shift metabolism from growth to lipid storage.
- **Scale-up from lab (5 L) to pilot (500 L)** – use dimensionless groups (e.g., gas holdup, axial dispersion) to predict performance.

---

## Implementation Notes

- The `pdepe` solver in MATLAB solves the 1D axial dispersion model. For Python, use `scipy.integrate.solve_ivp` for ODEs (if well-mixed assumption) or `scipy.fft` for spectral methods if PDE is required.
- Kinetic parameters (mu_max, K_S_CO2, K_S_O2) must be estimated from batch experiments. Use `lib/ParameterEstimation` (e.g., Levenberg-Marquardt) to fit model to measured biomass and dissolved CO2 profiles.
- For real-time estimation, an **EKF** can be implemented using the process model as the prediction step, with measurements from DO probes and pH sensors (inferring CO2 from pH via bicarbonate equilibrium).
- The validated model can be used for **economic optimisation**: maximise net present value (NPV) of CO2 credits + biodiesel sales – operating costs.

**Source:**  
Barla, R.J., Gupta, S., Raghuvanshi, S. (2026). Optimizing CO2 mitigation and biodiesel production using *Bacillus cereus* in bubble column bioreactor: A mathematical modeling approach for industrial upscaling. *Bioresource Technology Reports*, 34, 102779.
