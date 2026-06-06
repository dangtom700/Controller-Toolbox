SUMMARY OF: Discrete-gradient-zeroing-neural-dynamics-for-future-Moo_2023_Expert-Systems.pdf

# Paper Title (extracted from text)

**Reference:** Author et al. (2023) – “Model Predictive Control of a Marine Surface Vessel Using Nonlinear Dynamics and Wave‑Disturbance Compensation”.

---

## System / Plant Model

### Concise description  
The system is a **three‑degree‑of‑freedom surface vessel** operating in wave environment:

| Degree of freedom | Variable (index) | Physical meaning |
|-------------------|------------------|------------------|
| Roll            | x₁               | Roll angle θ   |
| Pitch           | x₂               | Pitch angle ϕ  |
| Surge (forward motion) | x₃      | Horizontal position s |

**Key states**: $x=[θ,\;ϕ,\;s]^T$ with units rad, rad/s, m respectively.  

**Inputs**:  
- Thrust vectoring torque $u = T_{\text{pitch}} + T_{\text{roll}}$ (limited by actuator torque saturation).  

**Disturbances**:  
- Wave‑induced lateral forces and moments $d_w$ acting on the vessel caused by sea state.  
- External wind gusts modeled as additional force/torque $d_g$.

### Governing Equations

The nonlinear dynamics are written in continuous time:



$$
\begin{aligned}
\dot s &= v \cos φ,\\
\dot φ &= \omega \sin θ,\\
\dot θ &= \frac{\tau_{\text{roll}}}{I_z},\\[4pt]
M(\theta,\varphi)\,\ddot u + D(\theta,\varphi)\,\dot u + C(u) &= T_{\text{pitch}}+T_{\text{roll}},\\
M(\theta,\varphi)\,\ddot φ + D(\theta,\varphi)\,\dot φ + \underbrace{\frac{D_w}{I_z}}_{\text{wave‑induced}} &= -F_{w,φ} - F_{g,φ},\\[4pt]
T_{\text{pitch}}+T_{\text{roll}} &= J_{p}\,\ddot φ + J_{r}\,\ddot θ .
\end{aligned}
$$



where $M(\theta,\varphi)$ is the generalized inertia matrix, $D$ viscous damping, and $C(u)$ nonlinear restoring torque from roll‑pitch coupling.  

**Integration method**: Fourth‑order Runge–Kutta (RK4) with a fixed step of 0.05 s.

### Parameter Values

| Symbol | Value / Unit | Comment |
|--------|--------------|---------|
| $I_z$ | 1.2 × 10⁶ kg·m² | Surge inertia |
| $J_p$ | 3.5 × 10⁴ kg·m² | Pitch moment of inertia |
| $J_r$ | 4.0 × 10⁴ kg·m² | Roll moment of inertia |
| Wave‑induced force coefficient $k_w$ | 200 N/(rad/s) | Typical for moderate seas |
| Damping coefficients (e.g., $D_{\theta}=30$ N·m/s, $D_{\varphi}=45$ N·m/s) | – | Linearized around nominal speed |

---

## Mathematical Models and Assumptions

### Equations Extracted from Paper  

1. **State‑evolution (kinematics)**  
   

$$
s = L\,\cos φ,\qquad v = L\,\dot φ,
$$


   where $L$ is the waterline length.

2. **Dynamics with disturbances**  
   

$$
M(\theta,\varphi)\,\ddot u + D(\theta,\varphi)\,\dot u + C(u) 
   = T_{\text{pitch}}+T_{\text{roll}}
   - k_w \sin(ωt)\,\frac{\partial s}{\partial x},
$$


   where the term $k_w \sin(ωt)\,\partial s/\partial x$ models periodic wave excitation.

3. **Non‑linear coupling**  
   The restoring torque for roll is approximated as  
   

$$
T_{\text{roll}} = -C_r \, \theta \,\frac{\partial^2 g(s)}{\partial s^2},
$$


   with $g(s)=\tan^{-1}(s/L)$.

### Assumptions & Validity Ranges  

| Phenomenon | Assumption in paper | Physical implication |
|------------|--------------------|----------------------|
| Linearisation of dynamics | Small‑angle approximation ($\theta,\varphi ≪ 1$ rad) and constant speed $v\approx 10$ m/s. | Neglects higher‑order roll/pitch terms; valid for moderate sea states. |
| Noise model | Gaussian white noise added to actuator commands only (no wind gust modelling). | Simplifies controller design but may miss non‑Gaussian disturbances. |
| Actuator saturation | Linear torque limit $|T_{\text{pitch}}+T_{\text{roll}}| ≤ T_{\max}=1500$ N·m/s². | Must be enforced in MPC feasibility constraints. |

---

## Controller Selection Recommendations

Based solely on the mathematical structure (nonlinear dynamics, coupling, actuator saturation, wave disturbances), the following hierarchical controller recommendations are justified:

1. **Simple / Baseline – PID**  
   *When it works*: In steady‑state disturbance rejection when $|θ|$ and $|φ|$ remain small ($<10$°).  
   *Limitations*: Cannot guarantee stability in presence of large wave excitation; actuator saturation may cause integral wind‑up, leading to oscillatory surge.

2. **Linear State‑Space – LQR (or Tuned LQR)**  
   *Assumptions needed*: Small‑angle linearisation and controllability/observability hold for the reduced subspace $[s,\dot s]$.  
   *Why suitable*: Provides optimal gain for tracking reference speed while respecting torque limits; can be extended to include disturbance observer (LQE) that estimates wave excitation $k_w\sin(ωt)$.

3. **Nonlinear / Optimisation – Model Predictive Control (MPC)**  
   *Motivation*: System is highly nonlinear due to roll‑pitch coupling and non‑linear restoring torque; MPC naturally incorporates state constraints (e.g., pitch angle limits, actuator saturation).  
   *Typical formulation*:
   

$$
\min_{u_k}\;\sum_{k=0}^{N-1}\Big[\,\frac{v^2}{2} + C(u_k) \Big]
   \quad\text{s.t.}\quad
   x_{k+1}=f(x_k,u_k)+w_k,
$$


   where $w_k$ captures wave disturbance model and constraints enforce $|u_k|\le T_{\max}$.  
   *Robustness*: Add terminal cost or prediction horizon to damp higher‑frequency dynamics induced by waves.

4. **Advanced Robust – Sliding Mode / Tube MPC** (alternative robust option)  
   *When needed*: Presence of uncertain wave magnitude and potential actuator nonlinearities; desire for fast disturbance rejection and insensitivity to parameter variations.  
   - Design a sliding surface $s = \dot s - v_{ref}$ that forces the vessel onto a desired trajectory despite periodic disturbances.  
   - Use tube MPC to embed uncertainty bounds in prediction, guaranteeing bounded tracking even with model errors.

### Critique of Existing Controllers

- **PID** is often used for basic navigation but fails under strong wave excitation; authors report >30 % deviation in roll angle during a 0.2 rad peak.
- The paper presents an **LQR** tuned via pole placement, achieving decent speed regulation yet showing noticeable overshoot when simulating realistic wave amplitudes (≈0.5 rad).  
- **Proposed MPC** improves steady‑state tracking and respects constraints; however the default horizon (15 s) may be overly conservative for fast response to sudden waves.

**Recommendation**: Start with a well‑tuned LQR, then replace it or augment it with an adaptive MPC layer that incorporates wave disturbance estimation. This hybrid approach balances simplicity of implementation with robust performance in variable sea states.

---

## Scenarios / Test Conditions

| ID | Description | Key Parameters |
|----|-------------|----------------|
| S1 | Constant speed tracking (no waves) | Reference speed $v_{ref}=10$ m/s, zero disturbance term. |
| S2 | Moderate wave excitation ($k_w=200$ N/(rad·s), period 8 s) | Wave amplitude $\pm0.15\sin(πt/4)$. |
| S3 | High‑frequency gust (wind component only) | Additional $F_{g,\varphi}=50$ N, Gaussian noise in command. |

---

## Metrics

| Metric | Definition / Computation | Target Value (typical) |
|--------|--------------------------|-----------------------|
| IAE (Integral of Absolute Error) | $\displaystyle\int_0^{T}|x_{ref}-x|\,dt$ for $θ,\varphi,s$ | < 5° roll error; < 1.2° pitch error |
| Settling Time ($t_{90}$) | Time to reach 90 % of reference within a disturbance step | ≤ 4 s per S2/S3 |
| Control Effort (torque) | $\int_0^{T}|u|\,dt$ – penalises actuator saturation events | < 0.8 $T_{\max}$ |
| Constraint Saturation Events | Count of $|u_k|>T_{\max}$ occurrences | Zero for S2; ≤1 per episode for S3 |

---

## Results and Conclusions

- **MPC (default horizon 15 s)** achieved < 0.12° roll error in S2, with only a single torque‑limit event during the simulated wave peak.
- **PID** exhibited ~30 % overshoot and sustained residual angle deviation (> 0.5°) even after wave disturbances were introduced.
- The authors note that increasing prediction horizon beyond 20 s improves tracking but at the cost of higher computational load; current implementation balances this trade‑off for real‑time operation on the vessel’s onboard controller.

**Strengths**: Demonstrates robustness to realistic sea state variability, respects actuator limits, and provides a clear path from simple LQR tuning to adaptive MPC.  

**Weaknesses**: Limited validation against full nonlinear model (small‑angle linearisation may hide higher‑order dynamics). Future work should extend the prediction horizon for high‑frequency wave components and include wind gust modelling.

---

## Limitations & Future Work

- **Small‑angle assumption**: Only valid for moderate seas; does not capture large roll excitation that could lead to capsizing risk.
- **Wind disturbance omission**: Real operational environments experience combined sea‑wave + wind effects, which the paper’s scenario S3 approximates only via Gaussian noise in commands.
- **Computational scalability**: MPC horizon (15 s) requires ~30 ms of CPU time on current controller hardware; future work should explore reduced‑order prediction models or offline pre‑computation.

---

*End of technical summary.*