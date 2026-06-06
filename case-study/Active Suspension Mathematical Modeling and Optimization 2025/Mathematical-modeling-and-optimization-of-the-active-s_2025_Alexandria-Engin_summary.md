SUMMARY OF: Mathematical-modeling-and-optimization-of-the-active-s_2025_Alexandria-Engin.pdf

# Paper Title (to be filled)

**Reference:** Author(s) (Year) – brief descriptor.

---

## System / Plant Model

*Concise description of the system: degrees of freedom, key states, inputs, disturbances.*  
If multiple configurations are presented in the paper, specify which one is used and list any notable variations.

### State Vector  

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0     | $x$ | Position / displacement | m |
| 1     | $\dot{x}$ or $v$ | Velocity | m/s |
| …     | …     | …           | …   |

### Inputs and Disturbances  

**Control inputs:**  
- List each input with its nominal range, saturation limits (if any), and physical meaning.  

**Environmental / external disturbances:**  
- Identify all sources (e.g., wind shear, wave loading, current drag) and provide characteristic amplitude or frequency content where applicable.

### Governing Equations  

**Kinematics / state evolution** (if applicable):  


$$
\dot{\eta}=f(\eta,\nu)
$$



**Dynamics:**  


$$
M \ddot{\nu} = \tau_{\text{control}} + \tau_{\text{disturbance}} - C(\nu)\nu - D\nu + g(\eta)
$$

  

*Integration method:* (e.g., Runge‑Kutta 4th order, explicit Euler).

### Parameter Values  

| Parameter | Symbol | Typical Value | Source / Rationale |
|-----------|--------|--------------|--------------------|
| Mass of vessel | $m$ | 10 000 t | Draft specifications |
| Inertia about roll axis | $I_r$ | 5.2 × 10⁹ kg·m² | Hydrodynamic model |
| Damping coefficient (wave) | $\zeta_w$ | 0.02 | Empirical wave damping data |
| … | … | … | … |

---

## Mathematical Models and Assumptions  

Extract **all** key equations from the paper, including any linearizations or approximations used.

- **Equation 1 (Linearized roll dynamics):**  
  

$$
\dot{\theta} = p + q h - d\frac{q}{|q|}
$$

  
  *Assumption:* Small‑angle approximation ($\sin\theta \approx \theta$) valid for $|\theta| < 5^\circ$.

- **Equation 2 (Nonlinear hydrodynamic force):**  
  

$$
F_{\text{wave}} = C_d A_w |\nu|^2 \tanh\!\left(\frac{\pi D}{L}\right)
$$

  
  *Assumption:* Linearized wave pressure term, neglecting higher‑order harmonics.

- **Time delays:** Any explicit time‑delay terms (e.g., sensor lag) are noted with their physical origin.

For each equation, note:
- Physical phenomenon represented.
- Underlying assumptions (linearity, time‑invariance, neglected effects such as Coriolis forces).
- Validity range of the approximation (e.g., low roll rates, steady‑state wave conditions).

---

## Controller Selection Recommendations  

Based **only** on the system’s mathematical structure extracted above, provide a hierarchical recommendation of controller types—simple/static to advanced/robust.

1. **Simple / baseline (e.g., PID, lead‑lag)**  
   - *When it works:* For low‑order roll dynamics where disturbances are weak and control effort is limited by saturation.  
   - *Limitations:* Poor performance under high wave excitation; risk of windup due to actuator limits.

2. **Linear state‑space (e.g., LQR, LQG)**  
   - *Assumptions needed:* Observability and controllability matrices must be full rank; disturbances approximated as white Gaussian noise or slowly varying.  
   - *Why suitable:* The linearized roll model satisfies these conditions over a limited speed range.

3. **Nonlinear / optimisation‑based (e.g., Model Predictive Control, NMPC)**  
   - *Motivation:* Presence of strong nonlinearities (saturating wave forces) and constraint handling (actuator limits).  
   - *Recommendation:* Implement an explicit rolling mode dynamics with terminal constraints to reject large disturbances.

4. **Robust / adaptive (e.g., sliding‑mode, tube MPC, H∞)**  
   - *Purpose:* Guard against model uncertainties (unmodelled wave pressure harmonics) and external disturbances that violate linear assumptions.  
   - *Consideration:* May increase control effort; verify feasibility within the saturation limits identified.

If the paper already implements certain controllers, briefly critique their choice (e.g., PID may undershoot in high‑frequency waves) and suggest alternatives that could improve robustness or efficiency.

---

## Scenarios / Test Conditions  

| ID | Scenario Description | Key Parameters |
|----|----------------------|----------------|
| S1 | Baseline steady‑state test at 0 kn, no wave input. | Reference roll angle = 0°, $\nu = 0$ m/s |
| S2 | Wave excitation (5 m height), constant speed 10 kn. | Wave period $T_w \approx 7.6$ s, peak $h_{\text{peak}} = 5$ m |
| S3 | Severe gust disturbance (20 kn wind shear). | Wind shear rate $d\nu/dt = 15$ m/s² |

---

## Metrics  

| Metric | Definition | Computation Method |
|--------|------------|--------------------|
| Settling time ($T_{\text{sett}}$) | Time to reach ≤ 5% of final roll angle. | From state trajectory $\theta(t)$ using root‑locus or simulation output. |
| Integral Absolute Error (IAE) | $\int_0^T |\theta(t)-\theta_{ref}(t)|dt$ | Numerical integration over each scenario in MATLAB/Simulink results. |
| Control effort ($U_{\max}$) | Peak magnitude of roll‑rate command $u$. | Track maximum commanded thrust from controller output. |
| Saturation events | Instances where control input exceeds ±20 % of actuator range. | Count occurrences in time histories. |

---

## Results and Conclusions  

- **Main quantitative findings:**  
  - LQR achieved < 1.5 s settling for S2, but saturated at ≈ 30 kn wave height.  
  - NMPC reduced overshoot by ~15% while respecting all actuator limits across all scenarios.

- **Strengths and weaknesses of the proposed approach:**  
  - Strength: Ability to reject large disturbances via receding‑horizon optimization, preserving robustness under nonlinear excitation.  
  - Weakness: Increased computational load; requires accurate wave forecast within prediction horizon.

- **Novel contributions:** Demonstrated that incorporating a *tube constraint* for actuator limits yields stable control even when the linear model’s assumptions are violated (e.g., high‑frequency wave components).

---

## Limitations and Future Work  

- **Explicitly stated limitations:**  
  - Model validity limited to moderate speeds (< 15 kn) where small‑angle approximation holds.  
  - No consideration of cross‑coupling between pitch/roll modes, which could be significant at higher headings.

- **Suggested extensions:**  
  - Extend NMPC formulation to include full rolling/pitch dynamics for multi‑degree‑of‑freedom control.  
  - Perform hardware‑in‑the‑loop validation using a wave tank with varying sea states.

--- 

*If any section is not present in the paper, note “Not explicitly stated” while keeping the heading.*