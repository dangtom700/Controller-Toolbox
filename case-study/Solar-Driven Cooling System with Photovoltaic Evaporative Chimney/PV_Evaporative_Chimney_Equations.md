# Mathematical Equations Extracted from:
**"Analytical Modelling and Optimisation of a Solar-Driven Cooling System Enhanced with a Photovoltaic Evaporative Chimney"**
J. Ruiz, P. Martínez, F. Aguilar, M. Lucas - *Applied Thermal Engineering 245 (2024) 122878*

---

## 1. PV Convective Area Model (Section 2.1.1)

### Assumptions
- 1D steady-state model
- PV panel layers: glass (front) -> silicon cell -> tedlar (rear)
- All radiation transmitted through glass is absorbed by the cell
- Glass reflectivity: $\tau = 0.9$ (constant, from Fresnel/Snell analysis)
- Wind-driven and buoyancy-driven forced convection dominate

### Energy Balance Equations

**Eq. (1)** - Front glass heat balance (external convection = conduction through glass):
$$h_e A \left(T_g - T_\text{amb}\right) = \frac{A k_g}{x_g}\left(T_c - T_g\right)$$

**Eq. (2)** - Cell energy balance (absorbed irradiance = electrical output + front conduction + rear conduction):
$$G A \tau = \eta_\text{PV} G A + \frac{A k_g}{x_g}\left(T_c - T_g\right) + \frac{A k_c}{x_c}\left(T_c - T_t\right)$$

**Eq. (3)** - Tedlar energy balance (conduction from cell = conduction through tedlar):
$$\frac{A k_c}{x_c}\left(T_c - T_t\right) = \frac{A k_t}{x_t}\left(T_t - T_r\right)$$

**Eq. (4)** - Rear surface heat balance (internal convection = conduction through tedlar):
$$h_i A\left(T_r - T_i\right) = \frac{A k_t}{x_t}\left(T_t - T_r\right)$$

### Convection Coefficients (empirical, from Aguilar [29])

**Eq. (5)** - External (front, wind-driven):
$$h_e = 0.841\, v_w + 4.61 \quad [\text{W m}^{-2}\text{K}^{-1}]$$

**Eq. (6)** - Internal (rear, buoyancy-driven channel):
$$h_i = 1.97\, v_i + 10 \quad [\text{W m}^{-2}\text{K}^{-1}]$$

**Eq. (7)** - Convective area air velocity as a function of water mass flow (momentum-driven, from Lucas et al. [25]):
$$v_i = -207.325\,\dot{m}_w^2 + 67.978\,\dot{m}_w - 2.380 \quad [\text{m s}^{-1}]$$

### PV Electrical Efficiency Correlations

**Eq. (8)** - Linear (temperature only; **selected as best fit**, used in global model):
$$\eta_\text{PV} = \eta_\text{PV,ref}\left(1 - \beta_\text{ref}(T_c - T_\text{ref})\right)$$

**Eq. (9)** - Linear with irradiance correction:
$$\eta_\text{PV} = \eta_\text{PV,ref}\left[1 - \beta_\text{ref}(T_c - T_\text{ref}) + \gamma \log_{10}\!\left(\frac{G}{G_\text{ref}}\right)\right]$$

**Eq. (10)** - NOCT-based (when $T_c$ is not available):
$$\eta_\text{PV} = \eta_\text{PV,ref}\left[1 - \beta_\text{ref}\!\left(T_\text{amb} - T_\text{ref} + \left(T_\text{NOCT} - T_\text{ref,NOCT}\right)\frac{G}{G_\text{ref,NOCT}}\right)\right]$$

**Eq. (11)** - Cell temperature from NOCT (substitute into Eq. 9 to derive Eq. 10):
$$T_c = T_\text{amb} + \left(T_\text{NOCT} - T_\text{ref,NOCT}\right)\frac{G}{G_\text{ref,NOCT}}$$

**Eq. (12)** - PV power output:
$$\eta_\text{PV} = \frac{\dot{W}_\text{PV}}{G A}$$

### Parameter Values

| Symbol | Value | Source |
|---|---|---|
| $\eta_\text{PV,ref}$ | 0.157 | Manufacturer |
| $\beta_\text{ref}$ | 0.0044 ^\circC$^{-1}$ | Manufacturer |
| $T_\text{ref}$ | 25 ^\circC | Standard |
| $G_\text{ref}$ | 1000 W m$^{-2}$ | Standard |
| $T_\text{NOCT}$ | 47 ^\circC | Manufacturer |
| $T_\text{ref,NOCT}$ | 20 ^\circC | Standard |
| $G_\text{ref,NOCT}$ | 800 W m$^{-2}$ | Standard |
| $\gamma$ | 0.02748 | Fitted to [27] |
| $\tau$ | 0.9 | Fresnel/Snell analysis |

---

## 2. PV Evaporative Area Model - Poppe Theory (Section 2.1.2)

### Assumptions
- Parallel flow arrangement (water and air both flow downward)
- Poppe theory used (more accurate than Merkel): Lewis factor $\neq 1$, evaporation mass loss is accounted for, air exit need not be saturated
- Lewis number: $\text{Le} = h_C / (h_D\, c_{p_{m_a}})$
- Solved via 4th-order Runge-Kutta

### Merkel Number Definition

**Eq. (13)**:
$$\text{Me} = \frac{h_D\, a_V\, V}{\dot{m}_w}$$

### Poppe Theory Governing ODEs

**Eq. (14)** - Evolution of air humidity ratio with water temperature:
$$\frac{d\omega}{dT_w} = \frac{c_{p_w}\dfrac{\dot{m}_w}{\dot{m}_a}\left(\omega_{sw} - \omega\right)}{\left(h_{sw} - h\right) + (\text{Le}-1)\left[\left(h_{sw}-h\right) - \left(\omega_{sw}-\omega\right)h_v\right] - \left(\omega_{sw}-\omega\right)h_w}$$

**Eq. (15)** - Evolution of air enthalpy with water temperature:
$$\frac{dh}{dT_w} = c_{p_w}\frac{\dot{m}_w}{\dot{m}_a} \times \left[1 + \frac{\left(\omega_{sw}-\omega\right)c_{p_w} T_w}{\left(h_{sw}-h\right) + (\text{Le}-1)\left[\left(h_{sw}-h\right) - \left(\omega_{sw}-\omega\right)h_v\right] - \left(\omega_{sw}-\omega\right)h_w}\right]$$

**Eq. (16)** - Evolution of Merkel number with water temperature:
$$\frac{d\,\text{Me}}{dT_w} = \frac{c_{p_w}}{\left(h_{sw}-h\right) + (\text{Le}-1)\left[\left(h_{sw}-h\right) - \left(\omega_{sw}-\omega\right)h_v\right] - \left(\omega_{sw}-\omega\right)h_w}$$

> **Note:** Subscript $sw$ denotes saturated air properties at water temperature $T_w$. The denominator is common across Eqs. (14)-(16).

### Experimental Correlations for Prototype

**Eq. (17)** - Merkel number vs. water-to-air mass flow ratio (from Lucas et al. [25]):
$$\text{Me} = 0.7099\left(\frac{\dot{m}_w}{\dot{m}_a}\right)^{-0.3254}$$

**Eq. (18)** - Air mass flow rate driven by water momentum (from Lucas et al. [25]):
$$\dot{m}_a = -6.1751\,\dot{m}_w^2 + 2.2469\,\dot{m}_w - 0.07653 \quad [\text{kg s}^{-1}]$$

---

## 3. Water-Cooled Chiller Model (Section 2.2)

### Assumptions
- EER is a function of condenser inlet water temperature $T_{w1,\text{cond}}$ and evaporator outlet water temperature $T_{w2,\text{evap}}$
- Condenser inlet/outlet water temperatures match evaporative area outlet/inlet: $T_{w1,\text{cond}} = T_{w2}$, $T_{w2,\text{cond}} = T_{w1}$

### EER Definition

**Eq. (19)**:
$$\text{EER} = \frac{\dot{Q}_\text{evap}}{\dot{W}_\text{comp}} = \frac{\dot{Q}_\text{cond} - \dot{W}_\text{comp}}{\dot{W}_\text{comp}} = \frac{\dot{Q}_\text{evap}}{\dot{Q}_\text{cond} - \dot{Q}_\text{evap}}$$

### EER Correlations (fitted to experimental data from Ruiz et al. [27])

**Eq. (20)** - Quadratic (selected as best fit, max difference < 1.30%):
$$\text{EER} = a + b\,T_{w2,\text{evap}} + c\,T_{w2,\text{evap}}^2 + d\,T_{w1,\text{cond}} + e\,T_{w1,\text{cond}}^2 + f\,T_{w2,\text{evap}}\,T_{w1,\text{cond}}$$

**Eq. (21)** - Linear (max difference < 2.03%):
$$\text{EER} = a + b\,T_{w2,\text{evap}} + c\,T_{w1,\text{cond}}$$

### Fitting Coefficients (Table 2)

| Coefficient | $a$ | $b$ | $c$ | $d$ | $e$ | $f$ |
|---|---|---|---|---|---|---|
| Eq. (20) | 12.15 | -1.826 | 0.2108 | 0.1166 | 0.004213 | -0.05686 |
| Eq. (21) | 6.331 | 0.2215 | 0.1163 | - | - | - |

---

## 4. Condenser (Hot) Loop Hydraulic Model (Section 2.3)

### System Curve (from EPANET simulation)

**Eq. (22)** - Condenser loop pressure curve ($H_m$ in m, $Q$ in l s$^{-1}$):
$$H_m = 1 + 89.649\,Q^2$$

### Pump Model

**Eq. (23)** - Pump efficiency definition:
$$\eta_\text{pump} = \frac{\rho_w\, g\, Q\, H_m}{\dot{W}_\text{pump}}$$

**Eq. (24)** - Pump head curve at nominal speed:
$$H_m = H_0\left[1 - \left(\frac{Q}{Q_0}\right)^2\right]$$

**Eq. (25)** - Pump efficiency curve at nominal speed:
$$\eta_\text{pump} = \eta_{\text{pump}_0}\left(\frac{Q}{Q_0}\right)\!\left(1 - \frac{Q}{Q_0}\right)$$

**Eq. (26)** - Pump head at variable speed (affinity laws, VFD-controlled):
$$H'_m = H_0\,k_r^2\left[1 - \left(\frac{Q}{k_r Q_0}\right)^2\right]$$

**Eq. (27)** - Pump efficiency at variable speed:
$$\eta'_\text{pump} = \eta_{\text{pump}_0}\left(\frac{Q}{k_r Q_0}\right)\!\left(1 - \frac{Q}{k_r Q_0}\right)$$

where $k_r = \Omega'/\Omega$ is the rotational speed ratio.

### Pump Nominal Parameters (from manufacturer data)

| Symbol | Value |
|---|---|
| $H_0$ | 45.05 m |
| $Q_0$ | 1.308 l s$^{-1}$ |
| $\eta_{\text{pump}_0}$ | 3.4 |

---

## 5. Global Performance Indicator (Section 3.2)

**Eq. (28)** - Grid Energy Efficiency Ratio (key optimisation objective):
$$\text{EER}_\text{grid} = \frac{\dot{Q}_\text{evap}}{\dot{W}_\text{grid}} = \frac{\dot{Q}_\text{evap}}{\dot{W}_\text{comp} + \dot{W}_\text{pump} - \displaystyle\sum \dot{W}_\text{PV}}$$

---

## 6. Solution Procedure for MATLAB

The paper's solution sequence (Fig. 3) maps directly to three sequential solver calls:

| Step | Equations Solved Simultaneously | Outputs |
|---|---|---|
| **1** | Eqs. (14)-(17) [evaporative] + Eqs. (19),(20) [EER] | $T_{w1}$, $T_{w2}$, $\dot{Q}_\text{cond}$, $\dot{W}_\text{comp}$, EER, $\varphi_{a_i}$ |
| **2** | Eqs. (1)-(4) + Eq. (8) [convective panel] | $T_c$, $\eta_\text{PV}$, $\dot{W}_\text{PV}$ |
| **3** | Eqs. (22)-(27) [hydraulic loop] sequential | $\dot{W}_\text{pump}$ |

**Coupling variable:** $T_{a_i}$ (air temperature leaving the evaporative area, entering the convective area as $T_i$) links Steps 1 and 2.

**Inputs required:** $T_\text{amb}$, $\phi_\text{amb}$, $G$, $v_w$, $Q$, $\dot{Q}_\text{evap}$, $T_{w2,\text{evap}}$

**Recommended solver:** `fsolve` (MATLAB nonlinear system solver); RK4 for Eqs. (14)-(16)

---

## 7. Design and Validation Conditions

### Optimisation Design Point (Table 4)

| Parameter | Value |
|---|---|
| $T_\text{amb}$ | 35.6 ^\circC |
| $\phi_\text{amb}$ | 35% |
| $G$ | 920.17 W m$^{-2}$ |
| $v_w$ | 3.028 m s$^{-1}$ |
| $\dot{Q}_\text{evap}$ | 3.8 kW |
| $T_{w2,\text{evap}}$ | 7 ^\circC |
| $Q$ (optimal) | 1.05 m^3 h$^{-1}$ |

### Parametric Analysis Range (Table 5)

| $T_\text{amb}$ (^\circC) | $\phi_\text{amb}$ (%) | $G$ (W m$^{-2}$) | $v_w$ (m s$^{-1}$) |
|---|---|---|---|
| 25 | 25 | 250 | 2.5 |
| 30 | 50 | 500 | 5 |
| 35 | 75 | 750 | 7.5 |
| 40 | 100 | 1000 | 10 |

### Model Validation Accuracy (Table 3)

| Variable | Max difference (%) | Avg difference (%) |
|---|---|---|
| $T_{w1}$ | 2.221 | 0.8597 |
| $T_{w2}$ | 2.347 | 1.005 |
| $T_{a_i}$ | 6.336 | 2.802 |
| $\dot{W}_\text{PV}$ | 3.467 | 2.469 |
| $\eta_\text{PV}$ | 5.830 | 2.478 |
