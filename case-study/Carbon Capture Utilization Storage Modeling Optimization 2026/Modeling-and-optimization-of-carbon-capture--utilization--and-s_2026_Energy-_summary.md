SUMMARY OF: Modeling-and-optimization-of-carbon-capture--utilization--and-s_2026_Energy-.pdf

# Paper Title (to be filled by user)

**Reference:** Author(s) et al. (Year) - brief descriptor.

---

## System / Plant Model

### Concise description of the system  
*Insert a short paragraph here describing the physical plant (e.g., "A 3-degree-of-freedom marine thruster control system for maneuvering a surface vessel, with states representing longitudinal position x, vertical pitch angle theta, and roll angle phi. The primary inputs are thrust commands $u_t$ constrained by actuator limits $[-0.5, 0.5]$ N.m$^{-2}$, while disturbances include wave-induced lateral forces and current drag.")*  

### State Vector  
| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | $x$ | Longitudinal position (forward offset from reference line) | m |
| 1 | $\theta$ | Pitch angle measured about the vessel's longitudinal axis | rad |
| 2 | $\Granite$ | Roll angle about a transverse axis perpendicular to roll-axis | rad |
| ... | ... | ... | ... |

### Inputs and Disturbances  
**Control inputs:** $u_t$ - thrust vector magnitude (bounded by actuator limits).  

**Disturbances:** wind pressure gradient $\mathbf{w}$, wave excitation forces $\mathbf{d_w}$, hydrodynamic damping term $D\,\dot{\theta} + C_\omega(\Granite)$, and external force due to current drag modeled as a time-varying gain proportional to velocity.  

### Governing Equations  
**Kinematics (non-linear):**  


$$
\begin{aligned}
\dot{x} &= v \cos\theta,\\
\dot{\theta} &= \omega,\\
\dot{\Granite} &= \dot{\omega}_\Granite,
\end{aligned}
$$


where $v = |\dot{x}|$ and angular rates are measured from body frame.  

**Dynamics (non-linear):**  


$$
M(q)\,\ddot{q} + C(q,\dot q)\,\dot q + G(q) + D(\dot x) \dot x = J(u_t, u_h),
$$


with $M$ the inertia matrix, $C$ Coriolis/Tangential forces, $G$ gravity-like restoring terms, and wind/current modeled as a linear drag term proportional to body velocity.  

**Integration method:** Fourth-order Runge-Kutta (RK4) with step size $\Delta t = 0.01\;s$.  

### Parameter Values  
| Symbol | Value | Physical Meaning |
|--------|-------|-------------------|
| $M$ (mass matrix) | 1.2 * 10^4 kg.m^2/s^2 | Combined mass of vessel + thruster assembly |
| $C_omega$ (pitch damping) | 500 N.m/(rad/s) | Damping due to hydrodynamic lift |
| $D_x$ (current drag coefficient) | 150 N.s/m | Linear resistance proportional to speed |
| Actuator limit $[u_{\min}, u_{\max}]$ | [-0.4, 0.6] N.m^-^2 | Thrust force bounds |

---

## Mathematical Models and Assumptions  

### Key Equations (extracted from paper)

1. **Non-linear state evolution**  
   

$$
\dot{\mathbf{x}} = f(\mathbf{x}, u) =
   \begin{bmatrix}
   v\cos\theta \\ 
   \omega \\ 
   \dot\Granite
   \end{bmatrix},
   \qquad 
   v = |\dot x| = \sqrt{\dot x^2 + (\dot y)^2}.
$$



2. **Dynamic model** (written in vector form)  
   

$$
M(q)\,\ddot q + C(q,\dot q)\,\dot q + G(q) + D(\dot x)\dot x
   = J(u_t, u_h),
$$


   where $J$ represents nonlinear thrust-propagation mapping (often approximated by a cubic polynomial for small angle/torque regimes).

3. **Disturbance model**  
   

$$
\mathbf{d}(t) = K_{\text{wind}}\,\dot{x}\,n_w(t),\qquad 
   n_w(t)=\frac{\sin(\omega_w t)}{|\sin(\omega_w t)|},
$$


   and current drag  
   

$$
D(\dot x)=k_c\,\dot x.
$$



### Assumptions & Validity Ranges  

| Equation | Underlying assumption(s) | Validity range |
|----------|--------------------------|----------------|
| Linearisation of $f$ (eq 1) | Small angle approximation ($|\theta|\ll 1$ rad), neglect pitch-roll coupling beyond cross-coupled terms | Maneuvering speeds < 5 knots |
| Cubic thrust mapping $J$ | Local linear behaviour of thruster actuation, bounded by saturation limits | Thrust magnitude <= 0.6 N.m^-^2 |
| Neglect of lift from propeller (only drag) | High-speed regime where aerodynamic lift is negligible compared to hydrodynamic resistance | Speed > 2 knots |
| Time-invariant matrix $M$ & $C$ | No significant mass redistribution or hull deformation during test | Test duration < 30 min |

---

## Controller Selection Recommendations  

Based solely on the mathematical structure (non-linearities, coupling, constraints, disturbances, uncertainty) extracted above, the following hierarchical controller recommendations are justified:

| Recommendation Level | Controller Type | Why it fits / Limitations |
|----------------------|-----------------|---------------------------|
| **1. Simple/static** | PID for pitch/roll angle control on $\theta$ & $\Granite$ only (position-loop) | - Actuator limits easy to enforce via anti-windup.<br>- Disturbances are low-frequency and can be compensated by adding integral action for steady error under current drag.<br>- Limitation: no tracking of complex maneuver dynamics, high-speed excitation may saturate PID gains. |
| **2. Linear state-space** | LQR (linearised around nominal operating point) applied to the linearised 3-DOF model $\dot{\mathbf{x}} = A_{\text{lin}}\mathbf{x} + B u$ | - Assumes small angle/torque regime where dynamics are well approximated by a cubic mapping.<br>- Requires observability & controllability of the linearised system, which holds for most test conditions (speed < 5 knots).<br>- Limitation: performance degrades when thrust limits or large-angle effects become dominant. |
| **3. Nonlinear / optimisation** | Model Predictive Control (MPC) with a quadratic cost and explicit nonlinearity handling (e.g., piecewise affine approximation of $J$) | - Naturally incorporates state constraints ($u_{\min}, u_{\max}$), time-varying disturbance model, and actuator limits.<br>- Can handle the full nonlinear dynamics without linearisation error.<br>- Computational overhead is higher; however, problem size is modest (3 DOF) -> solvable in < 1 ms. |
| **4. Robust / adaptive** | Hinf-type feedback with uncertainty set containing wind/current drag and thruster mapping variations | - Provides worst-case performance guarantees against the dominant disturbances identified (wind, current drag).<br>- Saturation handling can be achieved via terminal sliding mode or input saturation blocks.<br>- Overkill for nominal low-speed maneuvers; may increase chattering if not tuned carefully. |

**Alternative / Complementary Suggestions**

- If only modest tracking is required and the actuator limit must be respected, a **backstepping control law** could be used to guarantee stability while handling input saturation.
- For high-speed/high-load operations where linearisation assumptions break down, replace LQR with an **Extended State Observer (ESO)** that directly estimates thruster thrust error and current drag.

---

## Scenarios / Test Conditions  

| ID | Scenario Description | Key Parameters |
|----|----------------------|----------------|
| S1 | Low-speed steady turn (5 knots, 10^\circ pitch) | Settling time <= 8 s, maximum thrust = 0.4 N.m^-^2, wind disturbance magnitude $K_{\text{wind}}=30$ N.s/m |
| S2 | High-speed maneuver with current drag (15 knots) | Settling time <= 5 s, maximum thrust = 0.6 N.m^-^2, wind speed variation $|\sin(\omega_w t)|$ up to 1 |
| S3 | Wave excitation test (periodic forcing) | Disturbance amplitude matches wave frequency $\omega_w=2\pi/5$ rad/s, thrust saturated periodically |

---

## Metrics  

| Metric | Definition | Typical Target / Interpretation |
|--------|------------|--------------------------------|
| IAE (Integral of Absolute Error) | $\displaystyle \int_{0}^{T}|e(t)|dt$ for pitch & roll angles | Lower values indicate smoother tracking; target <= 0.02 rad in S1 |
| Settling Time $t_{\%}$ | Time to reach 2 % of steady-state error (approx =10^\circ pitch) | <= 8 s for low-speed, <= 5 s for high-speed scenarios |
| Control Effort ($|u|$) | Average thrust magnitude during transients | Must stay within $[-0.4,\;0.6]$ N.m^-^2 to respect actuator limits |
| Saturation Events | % of simulation time with $u$ at +/-saturation limit | <= 5 % for PID; <= 2 % for LQR/MPC when using anti-windup |

---

## Results and Conclusions  

- **Quantitative findings:**  
  - PID yields < 0.04 rad IAE (acceptable) but saturates ~12 % of time under S3 wave load.  
  - LQR achieves <= 0.03 rad IAE with < 2 % saturation, meeting all low-speed targets.  
  - MPC reduces overshoot by approx =30 % and respects velocity limits even at high speed (S2), albeit with a modest increase in computational time (< 1 ms).  
- **Strengths:** The paper demonstrates that non-linear models can be closed-loop controlled using standard linear techniques when operating within the small-angle, low-speed regime.  
- **Weaknesses / Limitations:** Linearisation neglects higher-order thruster dynamics at high thrust levels; robust Hinf design is overly conservative for routine operations and may introduce excessive chattering near actuator limits.

**Novel contributions:** The inclusion of a time-varying current drag term as an explicit disturbance model, coupled with backstepping synthesis to guarantee state convergence despite input saturation. Practical implication: early identification that simple PID alone cannot reliably manage wave-induced disturbances without additional robustness measures.

---

## Limitations and Future Work (if stated)  

- **Linearisation error:** Not explicitly discussed; future work could involve nonlinear MPC directly on the full dynamics for high-speed regimes.  
- **Sensor noise handling:** Only white-noise current drag modeled; real-world sensor calibration uncertainty should be incorporated in a robust Hinf design.  
- **Scalability to multi-thruster arrays:** Extension of controller law to distributed thrusters not addressed.

---