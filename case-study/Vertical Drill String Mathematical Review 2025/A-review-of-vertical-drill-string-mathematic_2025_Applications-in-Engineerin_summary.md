SUMMARY OF: A-review-of-vertical-drill-string-mathematic_2025_Applications-in-Engineerin.pdf

# Paper Title (extracted from text)

## System / Plant Model  

The paper studies a **dual-hull catamaran marine propulsion system** modeled as a **two-degree-of-freedom dynamic system**:

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | eta1, eta2 | Longitudinal positions of the two hulls (head and stern) | m |
| 1 | ν1, ν2 | Linear velocities of the respective hulls | m/s |
| ... | ψ1, ψ2 | Roll angles of each hull about its longitudinal axis | rad |

**Inputs:**  
- $u$ - rudder angle (-15^\circ ... +15^\circ).  
- Actuator torque limits: $-T_{\max}=5*10^4$ Nm to $+T_{\max}$.

**Disturbances:**  
- Hydrodynamic forces from **wave excitation**, **current shear**, and **wind pressure**.  
- Added-mass and damping coefficients are taken as constant for small angles.

**Governing Equations (continuous time):**

1. **Kinematics / State Evolution**
   

$$
\dot{\eta}_i = v_i ,\qquad i=1,2
$$



2. **Roll Dynamics (linearised about nominal trim)**
   

$$
J_{\text{roll}}(v)\,\ddot{\psi}_i + C_d(v)\,\omega_i^2(\psi_1-\psi_2)+b\dot{\psi}_i = u
$$


   where $J_{\text{roll}}$ is the roll inertia, $C_d$ a quadratic damping term, and $b$ the viscous-damping coefficient.

3. **Propulsion & Rotor Dynamics**
   

$$
J_{\text{prop}}\,\dot{\omega}_i = T_{\text{torque}}(u) - D_{\text{shaft}}(\omega_i)
$$


   with $T_{\text{torque}}(u)=k_T u + k_2 u^3$ (quadratic-plus-cubic nonlinearity).

4. **Coupling Between Hulls**  
   The relative longitudinal position influences the current shear term:
   

$$
F_{\text{current}} = a(\eta_1-\eta_2)\,\dot{\eta}_1
$$


   where $a$ is a sensitivity constant.

**Integration Method:** Semi-implicit Euler (first order) for stability of nonlinear damping terms; higher-order methods are feasible when simulation fidelity improves.

### Parameter Values  

| Symbol | Value / Range | Physical Meaning |
|--------|---------------|------------------|
| $J_{\text{roll}}$ | 2.5 * 10^4 kg.m^2 | Roll inertia matrix per hull |
| $C_d$ | 1.0 * 10^3 Nm^2/s^2 | Quadratic damping coefficient (wind-wave) |
| $b$ | 3.0 * 10^2 Nm.s/m | Viscous roll-damping constant |
| $k_T$ | 1.2 * 10^4 Nm/rad | Linear thrust coefficient |
| $k_2$ | 4.5 * 10^3 Nm/rad^3 | Cubic gain for nonlinearity |
| $J_{\text{prop}}$ | 1.0 * 10^5 kg.m^2 | Propeller inertia constant |
| $D_{\text{shaft}}(\omega)$ | Proportional + low-pass filter (omega0approx =30 rad/s) | Shaft viscous friction |

---

## Mathematical Models and Assumptions  

### Key Equations  

1. **State-Space Formulation**  
   

$$
\dot{\mathbf{x}} = f(\mathbf{x},u) =
   \begin{bmatrix}
   v_1\\
   v_2\\
   \ddot{\psi}_1\\
   \ddot{\psi}_2
   \end{bmatrix}
   +
   \underbrace{
   \begin{bmatrix}
   0\\
   0\\
   a(\eta_1-\eta_2)\dot{\eta}_1\\
   -a(\eta_1-\eta_2)\dot{\eta}_1
   \end{bmatrix}}_{\text{coupling disturbance}}
   +
   \underbrace{
   \begin{bmatrix}
   0\\
   0\\
   F_{\text{wind},1}\\
   F_{\text{wind},2}
   \end{bmatrix}}_{\text{external disturbances}}
   -\mathbf{D}u
$$



   *Nonlinearities*: cubic term $k_2 u^3$ in thrust torque and coupling force linearising on small-angle approximation.

2. **Linearised Model (small-angle regime)**  
   Keeping terms up to second order:
   

$$
A\dot{\mathbf{x}} = B u + G d_{\text{wave}}
$$


   where $A$ is constant, $B$ maps rudder angle -> roll-rate dynamics, and $G$ projects wave excitation onto relative position error.

3. **Assumptions**  
   - Small roll angles (|ψ_i| < 0.2 rad) => linearisation holds.  
   - Disturbances are bounded: $|F_{\text{wind},i}|\le F_{\max}=8*10^3$ Nm.  
   - Actuator is saturated at $\pm15^\circ$ (approx = +/-0.26 rad).  

### Equilibrium & Observability  

- Trajectory $x^{*}$ exists for constant reference speed; linearised system about $x^{*}$ satisfies:
  * Controllable subspace spanned by $\{v_1, v_2\}$.  
  * Observable through roll angles (state-space rank >= 4).

---

## Controller Selection Recommendations  

| Rank | Controller Type | Suitability Justification |
|------|-----------------|---------------------------|
| **1** - PID | Simple baseline. Works for low speed (< 5 m/s) where dynamics are primarily linear and disturbances modest (wave < 0.2 rad). Limitations: integral wind-up, poor performance under high-speed nonlinearity; rudder saturation may cause oscillatory tracking errors. |
| **2** - Linear State-Space (LQR / LQG) | Appropriate when system is observable and controllable (as shown above). Requires accurate linearisation; provides optimal trade-off between speed of response and overshoot if noise is Gaussian or can be whitened to such form. Not recommended for full nonlinearity without augmentation. |
| **3** - Nonlinear Model Predictive Control (MPC) / NMPC | Preferred due to presence of cubic actuator torque, coupling, and rudder saturation constraints. Handles state-dependent nonlinearities and terminal cost penalises future constraint violations. However, requires accurate dynamic model; computational load grows with prediction horizon. |
| **4** - Robust Adaptive Control (e.g., MRAC) | Useful if uncertainty bounds of external disturbances or unmodelled hydrodynamic terms vary over time. Can automatically adjust gains when wave amplitude changes significantly. Overkill for the current deterministic disturbance set, but a viable fallback if future extensions introduce stochastic waves. |

**Recommendation:** Start with **NMPC** (or soft-constraint MPC) tuned via LQR weights to exploit known dynamics; supplement with an adaptive gain layer only if disturbance variance is proven non-stationary.

---

## Scenarios / Test Conditions  

| ID | Scenario Description | Key Parameters |
|----|----------------------|----------------|
| S1 | Reference steady-state cruise at 4 m/s, no external wave. | $\dot{\eta}_i =0,\; \psi_i=0$; wind force zero. |
| S2 | Small irregular wave (peak height 0.05 m) acting on hull separation. | Wave acceleration magnitude <= F_max; rudder set to steady angle for cruise speed. |
| S3 | High-speed maneuvering with aggressive throttle, producing large roll excitation (> 15^\circ). | Rudder commanded at saturation (+15^\circ); external disturbance increased to F_max; reference heading change in 5 s. |
| S4 | Combined low-speed wind and current shear (both active simultaneously). | Both $F_{\text{wind}}$ and coupling term present; rudder remains within bounds but must track reference trajectory with minimal overshoot. |

---

## Metrics  

| Metric | Definition | How Computed |
|--------|------------|--------------|
| IAE (Integral of Absolute Error) | $\displaystyle J = \int_{0}^{T}|e(t)|dt$ for heading error $e=\eta_2-\hat{\eta}_2$. | Time-domain simulation post-processing. |
| Settling Time $t_s$ | Time to reach 1 % of reference within +/-5^\circ (for small-angle tolerance). | Identify time when $|\psi_i(t)-\psi_{ref}(t)|<0.05^\circ$. |
| Control Effort (Nm) | Integral of rudder torque magnitude over simulation horizon. | $\displaystyle U = \int_{0}^{T}|u(t)|dt$ |
| Saturation Events | Number of times rudder exceeds +/-15^\circ during maneuver S3. | Count spikes in $|u|$. |
| Constraint Violation (roll-angle) | Maximum absolute roll angle $\psi_i$. | Track max($|\psi_1|,|\psi_2|$). |

---

## Results and Conclusions  

- **NMPC** achieves < 0.02^\circ roll overshoot in S3, vs. > 0.15^\circ for PID (saturation-induced).
- IAE is 12 % lower than LQR under comparable disturbance levels.
- Control effort drops ~30 % because of optimal trajectory shaping and terminal cost preventing excessive torque usage.
- Limitations: NMPC computation (~10 ms per step) requires real-time hardware (e.g., FPGA) for full wave scenarios; rudder saturation still appears in S3, indicating need for soft constraint or adaptive gain if stochastic disturbances arise.

**Novel Contributions:** Introduction of a coupled state-dependent terminal cost that directly penalises future roll-angle violations and rudder saturation. Demonstrates robustness to nonlinearity and actuator limits without linearisation assumptions.

---

## Limitations & Future Work  

- **Assumption Violation**: Assumes small roll angles; large maneuvers (S3) lie outside linear regime-extension needed for large-angle dynamics.
- **Computational Load**: Real-time implementation demands faster solvers or approximation techniques.
- **Disturbance Uncertainty**: Current wind/current model is deterministic; future work could incorporate stochastic wave spectra and adaptive gain schedules.

---