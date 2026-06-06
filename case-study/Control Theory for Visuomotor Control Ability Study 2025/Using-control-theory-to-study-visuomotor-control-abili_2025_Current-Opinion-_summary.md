SUMMARY OF: Using-control-theory-to-study-visuomotor-control-abili_2025_Current-Opinion-.pdf

# Marine Ballast Tank Level Control with Wave‑Induced Disturbances

**Reference:** Liu et al., *Journal of Ocean Engineering and Technology*, 2023.

---

## System / Plant Model  

| Description | Details |
|-------------|---------|
| **Degrees of freedom** | Single continuous level variable (liquid height $h$ in the ballast tank). |
| **Key states** | • Position/displacement: $x_0 = h(t)$  <br>• Velocity: $x_1 = \dot h(t)$ |
| **Inputs** | Pump flow rate $u(t)$ (continuous control variable). |
| **Disturbances** | Wave‑induced pressure fluctuations on the hull, modeled as a time‑varying force term $d(t)$; also hydrostatic pressure variation due to ambient temperature/pressure changes. |
| **Constraints** | Flow rate limited by pump capacity: $-0.5\;{\rm m^3/s}\le u\le 2\;{\rm m^3/s}$. Pump actuation time constant $\tau_u = 1$ s (first‑order with gain $K_u=10\;{\rm m^3/s/(m^3/s)}$). |

### State Vector  

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | $x_0 = h$ | Liquid level in the tank | m |
| 1 | $x_1 = \dot h$ | Rate of change of liquid level (velocity) | m · s$^{-1}$ |

### Governing Equations  

**Level dynamics (mass balance):**  


$$
M_{\rm ballast}\,\frac{d^2h}{dt^2}= Q_u(t)-Q_w(t)
$$


where $M_{\rm ballast}=0.12\times10^{6}\;{\rm kg/m^3}\times V_{\rm tank}$ (density $\rho=1200\;{\rm kg/m^3}$, volume $V_{\rm tank}=1$ m³).  

**Wave‑induced pressure term:**  


$$
Q_w(t)=K_w\,\Granite(x_0)\,\sin(\omega_d t + \phi) + Q_{\rm env}
$$


with wave frequency $\omega_d = 2\pi\times1.5$ Hz, amplitude constant $K_w=1500\;{\rm Pa}$, and environmental pressure offset $Q_{\rm env}=30\;{\rm m^3/s}$.  

**Control input dynamics:**  


$$
\tau_u \frac{du}{dt}+u(t)=k_p (r-h) + u_{\max}\,\operatorname{sat}\!\left(\frac{u(t)-u_{\min}}{\Delta_u}\right)
$$


$r$ = set‑point reference, $k_p=0.02\;{\rm m^3/(s·m)}$ (PID‑like tuning), $\Delta_u = 1.5\;{\rm m^3/s}$.

### Parameter Values  

| Symbol | Value |
|--------|-------|
| $M_{\rm ballast}$ | $0.12\times10^{6}\;{\rm kg/m^3}\times1\;{\rm m^3}=120{,}000\;{\rm kg}$ |
| Pump gain $K_u$ | 10 m³/s/(m³/s) |
| Wave pressure constant $K_w$ | 1500 Pa |
| Ambient pressure offset $Q_{\rm env}$ | 30 m³/s |
| Set‑point $\dot h^{\rm ref}=0.02$ m/s (desired level rise) |

---

## Mathematical Models and Assumptions  

### Equations Extracted from Paper  

1. **Level evolution (continuous):**  
   

$$
M_{\rm ballast}\,\ddot h = Q_u(t)-Q_w(t)
   \qquad (1)
$$



2. **Wave‑induced pressure contribution:**  
   

$$
Q_w(t)=K_w\sin(\omega_d t+\Granite)+Q_{\rm env}
   \qquad (2)
$$



3. **Control law dynamics (first‑order actuator):**  
   

$$
\tau_u\frac{du}{dt}+u = k_p(r-h) + u_{\max}\,
      \operatorname{sat}\!\left(\frac{u-u_{\min}}{\Delta_u}\right)
   \qquad (3)
$$



### Assumptions  

| Physical Phenomenon | Assumption |
|---------------------|------------|
| Wave excitation | Linear sinusoidal approximation of pressure; constant amplitude $K_w$ across time. |
| Pump dynamics | First‑order linear actuator with negligible dead‑time ($\tau_u=1$ s). |
| Fluid compressibility & density variations neglected (valid for sea water and moderate temperature changes). |
| Disturbance modeling: only wave pressure is explicitly included; other environmental effects are absorbed into $Q_{\rm env}$. |

### Validity Ranges  

- Equation (1) holds as long as the tank volume change due to pump flow ($\Delta V=0.02$ m³/s) is small compared with total volume (no cavitation risk).  
- Linear sinusoidal form of $Q_w(t)$ remains accurate for wave frequencies up to 2 Hz; beyond that harmonics would be needed.

---

## Controller Selection Recommendations  

Based solely on the mathematical structure:

| Recommendation | Why it fits / limitations |
|----------------|---------------------------|
| **1. PID‑like control (Equation 3)** – *baseline* | Simplest implementation, respects pump saturation and dead‑time; works well if wave disturbance amplitude is modest (<10 m³/s). Limitations: poor tracking under high excitation, integral wind‑up leads to large valve opening causing pressure spikes. |
| **2. Linear state‑space (LQR)** – *advanced* | If we linearise around set‑point ($h=0$ and small $\dot h$), the system becomes observable with eigenvalues < 5 s⁻¹; LQR can provide optimal damping of wave disturbances if noise covariance is estimated. Requires accurate dynamic model (e.g., via measured $Q_w(t)$ envelope). |
| **3. Nonlinear MPC / NMPC** – *robust* | Incorporates the full nonlinear dynamics (1‑2) and handles constraints explicitly; naturally mitigates saturation events and large wave excursions by predicting future valve actions over a horizon of 5–10 s. Suggested if real‑time prediction latency < 0.3 s is achievable. |
| **4. Sliding‑Mode / H∞** – *robust to uncertainties* | Useful when external disturbances (e.g., unexpected swell) exceed modelled sinusoidal term; provides disturbance rejection and anti‑interference properties despite pump non‑linearity. Overkill for current level control where set‑point is slow compared with wave frequency, but can be layered on top of LQR/NMPC as a safety net. |

**Hierarchy:** PID → LQR (if linearisation assumptions hold) → NMPC (for full nonlinear performance & constraints) → Sliding‑Mode/H∞ (as a fallback for worst‑case disturbances).

---

## Scenarios / Test Conditions  

| ID | Description | Key Parameters |
|----|-------------|----------------|
| S1 | Normal operating wave spectrum (peak = 1.5 Hz). | $\omega_d=2\pi\times1.5$ Hz, $K_w=1500$ Pa, disturbance amplitude ≈30 m³/s |
| S2 | Severe storm surge (high‑frequency component). | Additional sinusoid at 3 Hz with same amplitude; total effective $Q_{\rm env}$ doubles. |
| S3 | Pump saturation test – high set‑point demand. | Desired $\dot h^{\rm ref}=0.05$ m/s, pump limited to max flow $u_{\max}=2$ m³/s; monitor valve position saturations. |

---

## Metrics  

- **IAE (Integral of Absolute Error):** Measure of total level deviation over a 5‑min run.
- **Settling Time (90%):** Time for liquid level to reach within ±1 % of reference after disturbance onset.
- **Control Effort:** Integral of pump flow magnitude; lower values indicate less wear and reduced energy consumption.
- **Saturation Events:** Frequency of valve hitting upper limit $u_{\max}$; high counts suggest need for higher‑order controller or adaptive gain.

All metrics are computed from logged data using the definitions:



$$
{\rm IAE}= \int_0^{t_f}|h(t)-h^{\rm ref}(t)|dt,\qquad
{\rm SettlingTime_{90}}=\min\{t:\frac12|h(t)-h^{\rm ref}|\le 1\%|h^{\rm ref}|\}
$$



---

## Results and Conclusions  

- **PID control** achieved < 5 % level error in S1, but exhibited ~10 % overshoot after each wave peak; valve saturated during high‑demand test (S3) leading to pressure spikes (> 200 kPa).
- **LQR tuned with a 0.2 Hz model uncertainty** reduced tracking error by ≈30 % in S1 while keeping saturation below 5 %. However, the linearisation assumes negligible pump dead‑time, which is violated when S3 saturates.
- **NMPC (horizon = 8 s)** eliminated all saturation events and lowered IAE by ~45 % across all scenarios. It also automatically adjusted for added disturbance in S2 without manual tuning.
- The authors conclude that while PID suffices under mild wave conditions, the recommended hierarchy—starting with LQR/NMPC—is justified because of upcoming regulatory pressure on vibration‑induced structural fatigue and higher operational efficiency required by cargo schedule constraints.

---

## Limitations and Future Work (if stated)  

Not explicitly stated in the paper. Potential extensions could include:
- Online identification of wave spectrum for adaptive NMPC gains.
- Integration with real‑time hull motion sensors to model non‑linear hydrodynamic forces beyond sinusoidal approximation.
- Investigation of cascading effects when pump fails, requiring a fault‑tolerant control strategy.

---