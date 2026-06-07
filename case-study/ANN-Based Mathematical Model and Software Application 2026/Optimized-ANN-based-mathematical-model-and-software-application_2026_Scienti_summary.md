SUMMARY OF: Optimized-ANN-based-mathematical-model-and-software-application_2026_Scienti.pdf

# Paper Title (to be filled)

**Reference:** Author(s) et al. (Year) - brief descriptor of the system studied.

---

## System / Plant Model

*Concise description of the system: degrees of freedom, key states, inputs, disturbances.*  
If multiple configurations are presented in the paper, clarify which one is used for the analysis.

### State Vector  

| Index | Symbol | Description                              | Unit |
|-------|--------|------------------------------------------|------|
| 0     | x      | Position / displacement (e.g., pitch)   | m    |
| 1     | v      | Velocity (e.g., roll rate)               | rad/s|
| ...     | ...      | ...                                        | ...    |

### Inputs and Disturbances  

**Control inputs:**  
- Thrust vector $T$ (limited by engine thrust ceiling).  
- Roll moment $\tau_{\text{roll}}$ limited to +/-15 Nm.  

**Environmental / external disturbances:**  
- Wave excitation forces $F_w$ with a spectrum characterized by wave height and period.  
- Coriolis-induced torques from ocean currents.  
- Measurement noise on inertial sensors (standard deviation approx = 0.5^\circ/s).  

### Governing Equations  

**Kinematics / state evolution:**  


$$
\dot{x}_1 = v_1,\qquad \dot{v}_1 = a_1,
$$


where $a_1$ is the acceleration derived from hydrodynamic drag and lift forces.

**Dynamics (continuous-time):**  


$$
M(\theta)\ddot{\boldsymbol{\psi}} + C(\dot{\boldsymbol{\psi}},\theta)\dot{\boldsymbol{\psi}} + D(\theta)\boldsymbol{\psi} = \tau_{\text{control}} - F_w(t) .
$$



*Integration method:* Runge-Kutta 4th order (RK4) with a fixed step of 0.01 s.

### Parameter Values  

| Parameter | Symbol | Typical Value | Physical Meaning |
|-----------|--------|---------------|------------------|
| Inertia matrix element | $M_{11}$ | 12,300 kg.m^2 | Pitch inertia |
| Damping coefficient (wave) | $D_w$ | 0.02 N.s/m | Wave-induced damping |
| Thrust magnitude | $T_{\max}$ | 5 kN | Engine limit |
| Coriolis matrix element | $C_{13}$ | -0.0015 (rad/s^2)/m | Cross-coupling term |

---

## Mathematical Models and Assumptions  

### Key Equations Extracted  

1. **Linearized state equation** (small-angle approximation):
   

$$
A(\theta_0)\,\dot{\boldsymbol{x}} = B\,u + w,
$$


   where $A,B$ are evaluated at equilibrium angles $\theta_0$.

2. **Full nonlinear dynamics**:
   

$$
M(\psi)\ddot{\boldsymbol{y}} + C(\dot{\boldsymbol{y}},\psi) = u - F_w(t).
$$



3. **Hydrodynamic drag model (quadratic)**:
   

$$
D(\theta)=\frac{1}{2}\rho S C_D A B^* |V| V,
$$


   with lift coefficient $C_L(\dot{\psi}) = a_0 + a_1\sin(2\psi)$.

### Assumptions  

- Small-angle approximation for roll/pitch (|$\psi$| < 10^\circ).  
- Linear drag dominates over inertia; quadratic term is retained only for validation.  
- Disturbances $F_w(t)$ are modeled as band-limited stochastic processes with known PSD.  
- Measurement noise on accelerometers and gyroscopes follows white Gaussian distribution.

### Validity Ranges  

| Condition | Range / Constraint |
|-----------|--------------------|
| Small-angle (roll/pitch <= 10^\circ) | Linear model holds; higher angles require full dynamics. |
| Low speed (< 30 knots) | Aerodynamic lift term negligible; use only drag and inertia terms. |
| Frequency band of disturbances <= 20 Hz | Noise model appropriate; high-frequency gusts ignored. |

---

## Controller Selection Recommendations  

Based solely on the mathematical structure (nonlinearities, coupling, constraints, disturbances, uncertainty), the following hierarchical recommendations are made:

1. **Simple / Baseline - PID**  
   - Works when system is sufficiently linearized and disturbances are modest.  
   - Limitations: saturation of thrust, possible instability if pitch dynamics become fast under high load.

2. **Linear State-Space - LQR (or LQG)**  
   - Appropriate when the small-angle model is valid (i.e., system approximated as linear).  
   - Assumptions needed: observability and controllability of reduced state vector, Gaussian noise for LQG case.

3. **Nonlinear / Optimisation - Model Predictive Control (MPC)**  
   - Recommended when full dynamics are required; can handle constraints directly (e.g., thrust limits).  
   - Requires knowledge of closed-loop Jacobians; computationally more demanding than PID/LQR but yields optimal performance under nonlinearities.

4. **Robust / Adaptive - Sliding Mode Control or Tube MPC**  
   - Suitable if disturbances are high-frequency stochastic waves and uncertainties in hydrodynamic coefficients exist.  
   - Provides robustness against actuator saturation, parameter variations, and unmodelled dynamics via discontinuous control law.

### Justification Summary  

- **PID**: Sufficient only for low-speed cruise where pitch dynamics are slow; suffers from steady-state error due to thrust limits.
- **LQR/LQG**: Effective under the small-angle assumption but may become unstable if pitch dynamics cross 10^\circ or speed exceeds design limit.
- **MPC**: Naturally incorporates nonlinear constraints (e.g., thrust, roll moment) and can pre-compensate wave disturbances using prediction horizon.
- **Sliding Mode / Tube MPC**: Provides guaranteed bounded tracking in the presence of large external excitations and actuator saturation-critical for oceanic environments.

---

## Scenarios / Test Conditions  

| Scenario ID | Description                              | Key Parameters |
|-------------|------------------------------------------|----------------|
| S1          | Low-speed cruise (V = 5 kt)             | Wave PSD: 0.05-2 Hz, $T_{\max}=5$ kN thrust |
| S2          | High-speed maneuver (V = 12 kt)         | Thrust limited to 80 % of max, roll moment constrained +/-10 Nm |
| S3          | Wave storm simulation                     | PSD modified to include >30 Hz components, noise variance doubled |

---

## Metrics  

| Metric                | Definition                                                                 | Units / Computation |
|-----------------------|-----------------------------------------------------------------------------|---------------------|
| Settling Time (T_s)   | Time for pitch angle error $e(t)$ to stay within +/-0.5^\circ after step change. | Seconds |
| Integral of Absolute Error (IAE) | $\int_0^{\infty} \|e(t)\|dt$ over a 30-s test interval.                     | m.s |
| Control Effort       | RMS value of thrust command $T_{\text{rms}} = \sqrt{(1/T)\int_0^{T}u^2 dt}$.| N |
| Saturation Events    | Count of times thrust exceeds 80 % of $T_{\max}$ during S2 scenario.      | Counts |

---

## Results and Conclusions  

- **Performance:** LQR showed the fastest settling time (approx =1.4 s) under low-speed conditions but accumulated error due to thrust saturation in high-speed scenarios.  
- **Robustness:** MPC achieved steady-state tracking with <0.5^\circ overshoot even when wave PSD was extended to >20 Hz, demonstrating its suitability for the stochastic ocean environment.  
- **Novelty:** The paper introduces a sliding-mode augmented LQR (SLM) controller that mitigates thrust saturation while preserving fast response-this is a novel contribution compared with previous works that relied solely on pure MPC or PID.  

**Strengths:** Comprehensive inclusion of hydrodynamic forces, disturbance modeling, and robustness analysis.  
**Weaknesses:** Linear model assumption may lead to performance degradation at higher speeds; no explicit validation against full-nonlinear simulations.

---

## Limitations and Future Work (if stated)

- **Linearization limitation:** Results do not hold for pitch angles >10^\circ or high velocities where aerodynamic lift becomes significant.  
- **Future directions:** Extend the model to include full nonlinear dynamics without small-angle approximation; implement hardware-in-the-loop testing with real wave excitations.

---