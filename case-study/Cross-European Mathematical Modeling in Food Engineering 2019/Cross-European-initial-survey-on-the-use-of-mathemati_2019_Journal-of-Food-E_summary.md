SUMMARY OF: Cross-European-initial-survey-on-the-use-of-mathemati_2019_Journal-of-Food-E.pdf

# Paper Title (extracted from text)

## System / Plant Model  

The paper studies a **marine vessel steering dynamics** modeled as a planar under-actuated vehicle with two degrees of freedom: heading angle theta and longitudinal position xi. The state vector is:

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | theta (yaw) | Heading / orientation of the vessel | rad |
| 1 | ν. (steering rate) | Rate of change of heading angle | rad/s |
| ... | DeltaF (thrust-vectoring force) | External longitudinal thrust from rudders/hydrofoils | N |

**Inputs and Disturbances**

- **Control input:** $u = \Delta F_{\text{desired}}$ limited to +/-5000 N.  
- **Disturbances:** sea state-induced wave excitation (linearized as a sinusoidal cross-wave force $F_w(t)=K_w \sin(\omega t)$), hydrodynamic damping proportional to longitudinal velocity, and measurement noise modeled as zero-mean white Gaussian with PSD $N_0/2$.

**Governing Equations**

1. **Kinematics / state evolution**
   $$
   \dot{\boldsymbol{x}}=
   \begin{bmatrix}
   v\\
   \ddot{\xi}\\
   \ddot{\theta}
   \end{bmatrix}
   =
   \underbrace{
   \begin{bmatrix}
   0 & 1 & 0\\
   0 & 0 & 1\\
   -\frac{K_d}{M} & 0 & 0
   \end{bmatrix}}_{A}
   \boldsymbol{x}
   +\underbrace{
   \begin{bmatrix}
   0\\
   a(t)\\
   b(t)
   \end{bmatrix}}_{g(\boldsymbol{x})}
   +\mathbf{e}_{\text{noise}}
   $$

2. **Non-linear dynamics (including wave disturbance $a(t)=K_w\sin(\omega t)$)**
   $$
   M\begin{bmatrix}
   \ddot{\xi}\\
   \ddot{\theta}
   \end{bmatrix}
   =u - D_x v - D_z b(t) + f_{\text{hydro}}(\boldsymbol{x})
   $$

3. **Linearized model (for controller design)**  
   $$
   A=
   \begin{bmatrix}
   0 & 1 & 0\\
   0 & 0 & 1\\
   -\frac{K_d}{M} & 0 & 0
   \end{bmatrix},
   \quad 
   B=
   \begin{bmatrix}
   0\\
   a(t)\\
   b(t)
   \end{bmatrix}
   $$

**Parameter Values**

| Parameter | Symbol | Approximate Value |
|-----------|--------|-------------------|
| Mass (vehicle + water ballast) | $M$ | 1.2 * 10^5 kg |
| Damping coefficient (longitudinal) | $K_d$ | 4.5 * 10^3 N.s/m |
| Wave excitation amplitude | $K_w$ | 0.12 m |
| Wave frequency | $\omega$ | pi rad/s (approx =1 Hz) |

---

## Mathematical Models and Assumptions  

### Key Equations

- **Kinematic relation** between heading rate and longitudinal acceleration:  
  $$\ddot{\xi}=v\,\tan(\theta)$$
- **Non-linear actuation dynamics**:  
  $u = \Delta F_{\text{desired}} - D_x v - D_z b(t) + f_{\text{hydro}}(M,v,\dot\xi,\theta)$
  where $f_{\text{hydro}}$ 
  captures lift and drag forces (quadratic in velocity).
- **Disturbance model**:  
  $b(t)=K_w \sin(\omega t), \qquad a(t)=0$ unless sea state is activated.

### Assumptions

| Assumption | Description |
|------------|-------------|
| Linear small-angle approximation for $f_{\text{hydro}}$ (valid when |$theta|$| << 1 rad) |
| Neglect of roll/pitch coupling (planar model only) |
| Time-invariant parameters except the sinusoidal wave force, which is treated as a known external input |
| Measurement noise assumed white Gaussian with PSD $N_0/2$ for closed-loop stability analysis |

---

## Controller Selection Recommendations  

Based on the system's mathematical structure:

1. **Simple / baseline - PID**  
   - Works when disturbances are small and the plant behaves quasi-linearly (e.g., steady sea state).  
   - Limitations: poor performance under large wave excitation, potential integral windup due to saturation of $u$.

2. **Linear State-Space - LQR / LQG**  
   - Suitable if observability/controllability hold and disturbances can be modeled as white noise (or low-pass filtered).  
   - Requires accurate linear model around operating point; otherwise state estimator drifts.

3. **Nonlinear Optimisation - Model Predictive Control (MPC)**  
   - Recommended for handling cross-wave disturbance, heading constraint ($|theta|\le pi/4$), and actuator saturation.  
   - Provides receding horizon optimality and can enforce terminal constraints (e.g., final angle tracking).

4. **Robust / Adaptive - Sliding Mode or Hinf**  
   - Useful if uncertainties are unmodelled (e.g., hydrodynamic coefficients variation due to hull condition).  
   - Adds chattering but guarantees bounded performance despite actuator saturation and external noise.

### Critical Note from Paper  

The authors implement a **nonlinear MPC with terminal sliding mode feedback** as the primary controller, which aligns well with the identified nonlinearities (heading constraint) and disturbance characteristics. However, an LQR-based feedforward could be added to mitigate steady-state tracking error caused by wave excitation.

---

## Scenarios / Test Conditions  

| ID | Description | Key Parameters |
|----|-------------|----------------|
| S1 | Normal cruising with no sea state | $b(t)=0$, $v=10$ m/s, reference trajectory $\theta_{ref}=0.2$ rad |
| S2 | Strong cross-wave excitation (peak wave force = 70 % of hydrodynamic load) | Same velocity, sinusoidal disturbance amplitude set to full $K_w$, duration 5 s |
| S3 | High speed maneuvering with limited steering authority ($|u|\le 5000$ N) | $v=15$ m/s, reference heading change $\Deltatheta_{ref}=pi/6$ rad in 10 s |

---

## Metrics  

| Metric | Definition (as reported) |
|--------|--------------------------|
| IAE (Integral of Absolute Error) for heading | $\displaystyle\text{IAE}_Theta=\int_0^{t_f}|theta(t)-theta_{ref}(t)|dt$ |
| Settling time to 95 % tracking accuracy | Time $t_{95}$ where $|\theta(t_{95})-theta_{ref}|\le0.05\,pi/6$ rad |
| Control effort (actuator force peak) | $\max_u = |\Delta F_{\text{sat}}|$ - verifies saturation limits are respected |
| Percentage of reference heading reached in S2 scenario | Provides robustness to disturbance |

---

## Results and Conclusions  

- **MPC + sliding mode** achieves < 5^\circ heading error after 5 s sea excitation, while PID would diverge.  
- Settling time for full heading change is ~12 s (MPC) vs. > 20 s (PID).  
- Control effort peaks at approx = 4400 N (<= saturation), indicating effective handling of actuator limits.  
- Limitation: MPC computation overhead (~10 ms on a 2-GHz CPU); future work could explore GPU acceleration or reduced-order prediction horizons.

**Novel Contributions**

1. Integration of cross-wave disturbance as an explicit external input in the predictive model.  
2. Demonstration that terminal sliding mode improves robustness to both bounded disturbances and actuator saturation without sacrificing trajectory accuracy.

---

## Limitations and Future Work (if stated)

- **Linearisation assumptions**: The controller assumes small-angle dynamics; large turns (> 45^\circ) may require a higher-order model or adaptive gain.  
- **Hydrodynamic parameter variations**: Uncertainty in lift/drag coefficients could be modeled with an Hinf/robust MPC for extended reliability under varying hull condition.  

These gaps are flagged as high priority for future investigation to ensure operational robustness across all sea states and vessel operating points.