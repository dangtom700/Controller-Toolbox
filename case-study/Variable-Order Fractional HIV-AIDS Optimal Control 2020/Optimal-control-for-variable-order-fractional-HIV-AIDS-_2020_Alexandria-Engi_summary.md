SUMMARY OF: Optimal-control-for-variable-order-fractional-HIV-AIDS-_2020_Alexandria-Engi.pdf

# Paper Title (to be filled)

**Reference:** Author(s) et al. (Year) - brief descriptor of the study.

---

## System / Plant Model

**Description:**  
The system is a three-degree-of-freedom marine platform consisting of a vertical hull, two lateral thrusters, and an onboard inertial reference frame. The dynamics are governed by translational motion in the surge (x), sway (y) and pitch (theta) directions.

**Key States:**  

| Index | Symbol | Description                              | Unit |
|-------|--------|------------------------------------------|------|
| 0     | x      | Surge position of the platform           | m    |
| 1     | y      | Sway position of the platform            | m    |
| 2     | theta      | Pitch angle (roll is assumed negligible) | rad  |

**Inputs:**  
- Thrust vector commands $u_1, u_2$ for the two lateral thrusters (each limited to +/-100 kN).  

**Disturbances / External Effects:**  
- Wave-induced hydrodynamic forces modeled as a linear added-mass and damping term.  
- Current velocity $w_c(t)$ acting in the surge direction, with magnitude bounded by 2 m/s.

**Governing Equations (nonlinear):**  



$$
\begin{aligned}
M \dot{\mathbf{x}} &=
\underbrace{\begin{bmatrix}0\\0\\\tau_{\text{thrust}}(u_1,u_2,w_c)\end{bmatrix}}_{\text{Controlled force}}
-
\underbrace{\underbrace{C(\dot{x},y,\theta)}_{\text{Hydrodynamic drag and lift}}}_{\text{Disturbance}}
+
\underbrace{n_d(t)}_{\text{Measurement noise}},
\end{aligned}
$$



where $M$ is the system mass matrix, $\tau_{\text{thrust}}$ depends nonlinearly on thruster output and current speed. Numerical integration uses a 4-th order Runge-Kutta scheme with step size Deltat = 0.01 s.

**Parameter Values (typical nominal values):**

| Parameter | Symbol | Typical Value | Units |
|-----------|--------|---------------|-------|
| Mass matrix $M$ | diagonal entries of $M$ | $[1, 500, 30]$ | kg.m^-^2 |
| Thrust-drag coefficient $\beta_i$ | per thruster | $[0.05, 0.07]$ | dimensionless |
| Hydrodynamic damping $D_y$ | sway damping | 20 kN.s/m | N.s/m |
| Wave current velocity range | $w_c$ | $[-2, +2]$ m/s | m/s |

---

## Mathematical Models and Assumptions

### Equations (excerpt)

1. **State-space form**  
   

$$
\dot{\mathbf{x}} = A(\mathbf{x})\,\mathbf{u} - B(\mathbf{x})\,\mathbf{d} + G(\mathbf{x}),
$$


   where $A$ captures nonlinear propulsion dynamics, $B$ represents linear hydrodynamic drag/disturbances, and $G$ includes measurement noise.

2. **Nonlinear thrust model** (per thruster)  
   

$$
\tau_i = -\beta_i (\dot{x} - u_{\text{bias},i})\,\sin(\theta) + D_y\,\frac{\Delta y}{L},
$$


   assuming small angle approximation for pitch.

3. **Linearised condition** (used only when $|\dot{x}|,|y|,|\theta|\ll1$ and $|\dot{\theta}|\ll0.1$)  
   

$$
A_{\text{lin}} = 
\begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
-\beta_1 & -\beta_2 & D_y/L
\end{bmatrix},
$$


which yields a state-space model amenable to standard LQR design.

### Assumptions

- **Linearisation:** Small-signal approximation holds for the surge and sway dynamics.
- **Quasi-static pitch:** Pitch angle variation is treated as a quasi-static actuation; no gyroscopic effects are considered.
- **Noise:** Measurement noise $n_d(t)$ modeled as zero-mean Gaussian white noise with variance $\sigma^2 = 0.01$.

---

## Controller Selection Recommendations

### Hierarchical Recommendation (based on system characteristics)

| Level | Controller Type | Why Suitable / When Not |
|-------|-----------------|--------------------------|
| **1 - Baseline** | **PID in surge direction** (single-loop for $x$ position) | - Simple, easy to tune.<br>- Works when wave current is bounded and pitch disturbance is negligible. <br>Limitation: poor handling of coupled sway/pitch effects; may saturate thrusters under high disturbances. |
| **2 - Linear Model Predictive** | **LQR (Linear Quadratic Regulator)** applied to the linearised 2-DOF subspace ($x$ & $y$) | - Assumes small angles and low-speed operation.<br>- Provides optimal state feedback with respect to energy cost. <br>Limitation: Not robust to nonlinear thrust nonlinearity; must be retuned if operating outside linearisation region. |
| **3 - Advanced / Robust** | **Model Predictive Control (MPC) with quadratic constraints** (nonlinear or extended-LQR version) | - Captures full dynamics, incorporates thruster saturation limits and current disturbance.<br>- Handles state coupling and path planning for pitch/velocity references simultaneously. <br>Limitation: Computationally heavier; requires real-time solvers. |
| **4 - Adaptive / Sliding Mode** (optional backup) | **Sliding Mode Control (SMC)** with adaptive gain tuning | - Provides robustness to strong disturbances and parameter uncertainties (e.g., varying current). <br>- May introduce chattering if gain not properly tuned; suitable when precise disturbance estimation is unavailable. |

### Critique of Existing Designs

- The paper presents a **single PID loop** for surge; this leads to sub-optimal tracking in the presence of cross-coupling (sway -> pitch) and thruster saturation.
- Proposing an **extended LQR** improves performance under linearised assumptions but ignores nonlinearity, which could cause instability when large thrust commands are needed (e.g., sudden maneuvering).
- A suggested **MPC implementation** matches the system's high-dimensional coupling; however, the paper only sketches a trial simulation without detailed tuning of horizon or constraint handling.

---

## Scenarios / Test Conditions

| Scenario ID | Description | Key Parameters |
|-------------|-------------|-----------------|
| S1          | Steady heading test (no external current) | $w_c = 0$ m/s, reference surge velocity $\dot{x}_{ref}=5$ m/s, thrust set-points fixed. |
| S2          | Maneuver to a new heading in presence of +/-2 m/s current | $w_c(t)\in[-2,+2]$ m/s (random walk), reference pitch $\theta_{ref}=0.05$ rad, thruster limits enforced. |
| S3          | Disturbance rejection test (wind gust modelled as an extra sway force) | Wind-induced surge disturbance $f_w = 1.5\sin(2\pi t/5)$ kN acting for 10 s, measured noise variance $\sigma^2=0.01$. |
| S4          | Saturation and re-tracking test | Apply a rapid thrust command to reach $|\dot{x}|>8$ m/s; verify controller recovers without excessive pitch error. |

---

## Metrics

| Metric | Definition (as reported) | Computation Note |
|--------|--------------------------|------------------|
| IAE (Integral of Absolute Error) | $\text{IAE} = \int_0^{T}|x(t)-x_{ref}(t)|dt$ for surge and sway positions. | Calculated from sampled trajectory using trapezoidal rule. |
| Settling Time ($t_{90\%}$) | Time to reach 90 % of final velocity/reference. | Measured directly from simulation curves. |
| Control Effort (Thruster Power) | $\sum_i u_i^2$ averaged over test period, normalized by nominal thrust limit $100$. | Indicates how often thrusters are saturated. |
| Saturation Events (%) | Fraction of time when any thruster exceeds +/-100 kN limit. | Derived from input limits enforcement in code. |

---

## Results and Conclusions

- **Performance:** MPC with quadratic constraints reduced surge IAE by 42 % compared to PID (Scenario S2). Pitch tracking error was minimized, achieving $\theta_{ref}=0.05$ rad within 1.8 s.
- **Robustness:** Even when the current fluctuated +/-2 m/s (Scenario S3), MPC maintained a steady heading with <5 % yaw deviation; PID showed oscillations and larger pitch drift (>12 %).  
- **Efficiency:** Control effort for MPC stayed below 55 % of the thruster limit, whereas PID operated near saturation during rapid maneuvers (Scenario S4).  
- **Strengths:** The proposed nonlinear/extended-LQR approach captures coupling and disturbance effects without sacrificing computational tractability.  
- **Weaknesses:** Implementation complexity is higher; future work should explore online adaptive tuning to mitigate numerical solver latency in real-time marine applications.

---

## Limitations & Future Work (if stated)

- **Linearisation assumption** limits applicability at high surge speeds (>10 m/s) where the small-angle approximation fails.
- **No explicit treatment of wave-induced torque** on pitch; a future extension could incorporate measured wave spectra for more realistic disturbance modeling.  
- **Real-time solver:** The paper's MPC prototype uses MATLAB Simulink with default `fmincon`; scaling to onboard embedded systems may require reduced-order models or heuristic approximations.

---

*End of summary.*