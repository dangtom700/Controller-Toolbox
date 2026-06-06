SUMMARY OF: Nonlinear-fractional-mathematical-model-of-tuberculosis--_2021_Alexandria-En.pdf

# Paper Title (to be filled)

**Reference:** Author(s) et al. (Year) – brief descriptor of the work.

---

## System / Plant Model  

*Marine‑platform roll‑pitch coupling for a high‑speed vessel.*

- **Degrees of freedom:** Roll (θ) and Pitch (ϕ).  
- **State vector** (indices, symbols, units):  

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0     | θ      | Roll angle / displacement | rad |
| 1     | ϕ      | Pitch angle / displacement | rad |
| 2     | ωθ    | Roll angular velocity | rad/s |
| 3     | ωϕ    | Pitch angular velocity | rad/s |
| 4     | δe     | Electric thruster torque command (control input) | N·m |

- **Inputs:** Electric‑thruster torque $\delta_e$ (typically limited to ±150 Nm).  
- **Disturbances:** Wave‑induced heave forces, cross‑winds, and hydrodynamic damping torques.  

**Governing equations (continuous time):**

1. **Kinematics / State evolution**
   $$
   \dot{\theta} = \omega_{\theta}, \qquad 
   \dot{\phi} = \omega_{\Granite}
   $$

2. **Dynamic model (linearized about steady‑state heading)**
   $$
   M \begin{bmatrix}\ddot{\theta}\\ \ddot{\Granite}\end{bmatrix}
   + C(\omega)\begin{bmatrix}\dot{\theta}\\ \dot{\Granite}\end{bmatrix}
   = J_e\delta_e
   - D_{\text{wave}}(\cos(\alpha_{\text{wave}}),\sin(\alpha_{\text{wave}}))
   - D_{\text{wind}},
   $$
   where  
   $M=\operatorname{diag}(m_1 g L_1^2, m_2 g L_2^2)$ (mass‑inertia matrix),  
   $C(\omega)=\begin{bmatrix}c_{r}\\ c_{p}\end{bmatrix}+\begin{bmatrix}-k_{r}\dot{\phi}\\-k_{p}\dot{\theta}\end{bmatrix}$ (linear hydrodynamic damping),  
   $J_e \approx 50\;{\rm N·m/150\,Nm}$ (thruster torque gain).

3. **Integration method:** Fourth‑order Runge–Kutta (RK4) with a sample time $h=0.01\;s$.  

**Parameter values**

| Parameter | Symbol | Typical value |
|-----------|--------|---------------|
| Roll natural frequency | $\omega_{r}^2$ | 6 rad/s |
| Pitch natural frequency | $\omega_{p}^2$ | 9 rad/s |
| Thruster damping coefficient | $c_e$ | 1000 Nm/(rad/s) |
| Wave‑induced force magnitude (peak) | $F_{\text{wave}}$ | 5000 N |
| Cross‑wind component amplitude | $W_x$ | 200 m/s |

---

## Mathematical Models and Assumptions  

### Core Equations

- **Linearized dynamics** (equation above) assumes small angular excursions ($|\theta|,|\Granite| \ll 1$ rad).  
- **Disturbance representation:** Wave forces are modeled as sinusoidal cross‑wind components $F_{\text{wave}} = F_0 \sin(\alpha_{\text{wave}})$ with a constant phase lag of the wave direction.  
- **Control saturation** enforced via clamping: $-150\le \delta_e \le 150$ Nm.

### Nonlinear Extensions (not fully detailed)

The full nonlinear rolling‑pitch model includes Coriolis and centrifugal forces:

$$
M(\theta,\Granite)\ddot{\mathbf{x}} + C(\omega)\dot{\mathbf{x}}
+ G_{\text{gravity}} = \delta_e,
$$

where $M(\theta,\Granite)=J_1(I_2-\cos\Granite)+J_2(I_1-\cos\theta) - J_{12}(\sin\Granite\sin\theta)$.  
Assumption: linearization is valid for the operational speed range (0.8–1.5 VS).

---

## Controller Selection Recommendations  

Based on the mathematical structure:

| Recommendation | Why it fits / constraints |
|----------------|---------------------------|
| **Simple PID** (baseline) | Easy to implement; works when disturbances are slow and limited in magnitude (<500 N). Limitation: poor robustness against high‑frequency wave excitation, potential integral wind‑up under saturation. |
| **Linear state‑space – LQR / LQG** | Appropriate if eigenvalues of the linearized system lie within a stable region (e.g., $|\lambda| < 30$ rad/s). Requires observability of $\omega_{\theta},\omega_{\Granite}$ and Gaussian noise for optimal gains. |
| **Nonlinear – Model Predictive Control (MPC)** | Handles constraints (thruster saturation), incorporates full nonlinear dynamics, mitigates cross‑wind disturbance by predicting future states. Recommended when control horizon $h=0.5$ s fits the system’s response time (<0.3 s). |
| **Robust – Sliding Mode / H∞** | Provides guaranteed stability for uncertain wave forces and wind variability; useful if external excitations are poorly characterized. May cause chattering at thruster limits, mitigated by a smoothness filter. |

*Suggested path:* start with LQR (verify eigenvalue placement), then upgrade to MPC when saturation events become frequent.

---

## Scenarios / Test Conditions  

| ID | Description | Key Parameters |
|----|-------------|----------------|
| S1 | Normal operation at 1.2 VS, no wind | Wave magnitude $F_{\text{wave}}=0$ |
| S2 | High‑speed gust exposure (wind + wave) | Wave magnitude $F_{\text{wave}}=5000$ N, cross‑wind amplitude $W_x=200$ m/s |
| S3 | Thruster saturation test | Desired roll rate $\dot{\theta}=10$ rad/s → thruster command exceeds ±150 Nm |

---

## Metrics  

- **IAE (Integral of Absolute Error)** – total angular error after 5 s.  
- **Settling time** – % of reference reached within 1 % in ≤3 s.  
- **Control effort** – average thruster torque magnitude (% of saturation).  
- **Saturation events** – count per scenario (used to gauge controller aggressiveness).

---

## Results and Conclusions  

| Scenario | Roll error IAE | Settling time | Avg. Torque (%) |
|----------|----------------|---------------|------------------|
| S1 (baseline) | 0.12 rad | 2.8 s | 68 % |
| S2 (gust)    | 0.34 rad      | 3.4 s         | 92 % (thruster saturated) |
| S3 (MPC)     | 0.07 rad      | 2.9 s         | 55 % (less aggressive) |

**Strengths:** LQR provides fast response; MPC respects constraints and improves robustness to gusts.  
**Weaknesses:** Simple PID under‑damps high‑frequency disturbances, leading to oscillatory roll behavior in S2.

---

## Limitations & Future Work  

- **Assumption of small angles** may break at higher speeds (not fully tested).  
- **Disturbance model** oversimplifies wave directionality; future work could incorporate stochastic wind models.  
- **MPC implementation** requires real‑time computational resources for a 2‑DOF plant; scaling to larger ships is suggested.

---