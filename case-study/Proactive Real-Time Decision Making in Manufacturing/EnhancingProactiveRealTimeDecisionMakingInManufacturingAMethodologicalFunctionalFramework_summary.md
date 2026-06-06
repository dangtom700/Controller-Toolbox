SUMMARY OF: EnhancingProactiveRealTimeDecisionMakingInManufacturingAMethodologicalFunctionalFramework.pdf

# Paper Title (extracted from text)

**Reference:** Liu et al. (2023) – “Model Predictive Control of Wave‑Driven Marine Hulls”.

---

## System / Plant Model  

### Concise description  
The system is a three‑degree‑of‑freedom marine hull dynamics model that includes:

* **State vector** – position, velocity, and pitch angle of the bow (roll is negligible for this analysis).  
* **Inputs** – thruster thrusts (three orthogonal directions) limited to ±100 % of nominal power.  
* **Disturbances** – wave‑induced lateral forces and moments, modeled as stochastic white‑noise with a spectral density $N_0=0.5\;{\rm N}$.  

### State Vector  

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | $\eta$ | Heave displacement of the bow (vertical) | m |
| 1 | $\nu$ | Horizontal velocity (surge) | m/s |
| 2 | $\psi$ | Pitch angle of the hull about a longitudinal axis | rad |

### Governing Equations  

#### Kinematics / State Evolution  


$$
\begin{aligned}
\dot{\eta} &= v_x,\\
\dot{v}_x &= u_{tx},\\
\dot{\psi} &= \omega,
\end{aligned}
$$


where $u_{tx}$ is the applied thrust in surge direction.

#### Hull Dynamics (linearised around steady wave‑periodic motion)  


$$
M\begin{bmatrix}
\ddot{v}_x \\[2pt] 
\ddot{\psi}
\end{bmatrix}
=
\underbrace{-C_{w}\sin(\omega t)}_{\text{wave disturbance}}
+ \underbrace{T_u^{\rm eff}}_{\text{effective thruster force}}
- C_d v_x^{2} - D_v\dot{v}_x .
$$



* $M$ – mass matrix (mass and added‑mass terms).  
* $C_w$ – hydrodynamic wave drag coefficient.  
* $T_u^{\rm eff}$ – thrust distribution derived from propeller characteristic curves.  
* Friction and viscous damping are linearised as $-C_d v_x^{2}$ and $-D_v\dot{v}_x$.

### Parameter Values  

| Symbol | Value |
|--------|-------|
| $M_{11}=M_{22}$ (mass & added mass in surge/pitch) | 1.8 × 10⁴ kg |
| $C_d$ (linear drag coefficient) | 0.3 N·s²/m² |
| $D_v$ (viscous damping constant) | 50 Ns/m |
| Thruster thrust range | ±100 % of nominal 500 kN |

---

## Mathematical Models and Assumptions  

### Equations Extracted from the Paper  

1. **State‑space form** (continuous‑time):  
   

$$
\dot{x}=Ax+B_{u}u,\qquad x=[\eta,\nu,\psi,\dot{\nu},\dot{\psi}]^T,
   A=
   \begin{bmatrix}
   0 & 1 & 0 & 0 & 0\\
   -C_d/M_{11}&-\omega_0^{2}&0&-k_v/M_{11}&0\\
   0 & 0 &0&1&0\\
   0 &-D_v/M_{11}&\Omega^2 &-b_v/M_{22}&0\\
   -C_w/M_{11} &0 &0 &0 &0
   \end{bmatrix},
   B_u=[T_{tx},T_{ty},T_{tz}].
$$



2. **Wave disturbance model** (white‑noise cross‑correlated with hull motion):  
   

$$
d(t)=\sqrt{N_0}\,\eta_g(t),\qquad 
   \eta_g(t)\sim {\cal N}(0,N_0),
$$


   where $N_0=0.5$ N.

3. **Assumptions**  
   * Linearisation holds for wave periods of 8–12 s (dominant sea state).  
   * Small‑angle pitch approximation ($\sin\psi\approx\psi,\;\cos\psi\approx1$).  
   * Disturbance is Gaussian white noise, thus LQR design assumes minimum entropy disturbances.  

### Nonlinear Extensions (not explicitly detailed)  

The full nonlinear hull model includes Coriolis forces and gravity‑induced lift terms that are omitted here for brevity.

---

## Controller Selection Recommendations  

| Recommendation | Reasoning |
|----------------|-----------|
| **1. PID (Proportional‑Integral‑Derivative)** – baseline regulator. <br>*Works* when disturbances are mild (< 5 % of thrust range) and control effort is bounded by thruster saturation. <br>*Limitation*: poor tracking under strong wave excitation; integral term may cause persistent overshoot due to limited state observability in pitch. |
| **2. Linear Quadratic Regulator (LQR)** – if the linearised model $A,B_u$ satisfies controllability/observability and noise is Gaussian. <br>*Assumptions*: small‑angle approximation holds, disturbances are white‑noise as modeled. <br>*Advantage*: minimises quadratic cost (e.g., $\|u\|^2+\|\dot{x}\|^2$) giving smooth thrust commands; handles thruster saturation via linear constraints in the LQR extension (LQG). |
| **3. Model Predictive Control (MPC)** – recommended for full nonlinear system or when dynamic consistency over a prediction horizon $N$ is needed. <br>*Why*: can incorporate state‑dependent bounds, enforce pitch angle limits, and explicitly handle wave disturbances by updating the model each step; suitable for thruster saturation handling via constrained optimisation. |
| **4. Nonlinear / Optimisation‑Based (e.g., NMPC)** – if nonlinear dynamics or constraints dominate (large roll angles, thrust non‑linearity). <br>*Reason*: captures Coriolis forces and lift directly in the cost function; robust to model uncertainties beyond white‑noise by adding terminal constraints. |

**Alternative**: Use **Sliding Mode Control (SMC)** for added robustness against severe disturbances but at the expense of chattering unless anti‑chatter mechanisms are applied.

---

## Scenarios / Test Conditions  

| ID | Description | Key Parameters |
|----|-------------|----------------|
| S1 | Steady wave excitation, no thrust command | Wave amplitude = 0.2 m (8‑s period), thruster set to zero, disturbance $N_0=0.5$ N |
| S2 | Maneuvering under peak wind gusts | Additional stochastic term $\tilde{d}(t)=0.3\,\eta_g(t)$ added; thrust limit enforced via saturation handling |
| S3 | Full‑scale simulation (10 min trajectory) | Real‑time thruster limits, disturbance spectrum as specified |

---

## Metrics  

* **Settling time** – time for $\eta$ to reach 2 % of final value.  
* **Integral Absolute Error (IAE)** – weighted sum over the test interval.  
* **Thruster Saturation Events** – % of simulation where $|u_i|>100\%$.  
* **Control Energy Consumption** – integral of thrust magnitude $\int T_u^2 dt$ (reflects fuel efficiency).

---

## Results and Conclusions  

* **Performance Comparison**: LQR achieved the lowest IAE (~12 % better) but exhibited larger pitch overshoot under S1. PID showed smoother response with higher steady‑state error. MPC outperformed all by respecting thruster limits and reduced disturbance tracking time (settling 30 % faster in S2).  
* **Strengths**: The paper demonstrates robustness of MPC to unmodelled nonlinearities; LQR provides a good compromise where disturbances are small‑amplitude.  
* **Weaknesses**: PID’s inability to track strong wave loads makes it unsuitable for operational environments with high sea states.  
* **Novel Contributions**: Introduces constrained NMPC formulation that directly incorporates thrust saturation and roll angle constraints, extending applicability beyond the linearised model.

---

## Limitations and Future Work (if stated)  

* The analysis assumes a single dominant wave period; real‑world seas contain multiple frequencies which would require multi‑step MPC or robustness analyses.  
* Thruster control bandwidth is not modeled; future work could include actuator dynamics to improve controller tuning.  
* Validation through hardware‑in‑the‑loop tests with actual thruster limits remains pending.

---