SUMMARY OF: Control-theory-in-the-era-of-intelligent-systems--Applic_2026_Annual-Reviews.pdf

# Paper Title: “Model Predictive Control of a Wave‑Driven Marine Current Turbine”

**Reference:** Smith et al., 2023 – Model predictive control for wave‑driven marine current turbines.

---

## System / Plant Model

The plant is a **three‑axis (pitch, yaw, heave) tidal turbine** operating in a deep‑water environment.  

| Quantity | Description |
|----------|-------------|
| Degrees of freedom | 3 translational (heave), 3 rotational (pitch, yaw, azimuth). Total state vector has 6 continuous states plus optional wake state variables if included. |
| Key states | `η = [x₁,x₂,…,x₆]ᵀ` where x₁–x₃ are displacements and x₄–x₆ are angular velocities (pitch, yaw, azimuth). |
| Inputs | Electrical torque command *τ_e* (limited by generator maximum power transfer), hydraulic valve position for pitch control. |
| Disturbances | Wave‑induced hydrodynamic forces (nonlinear drag + added mass), current shear, turbulence modeled as stochastic wind gusts with white‑noise characteristics. |

### Governing Equations

**Kinematics / State Evolution**

```
η̇ = f(η, τ_e) 
where
f(x)= [ v₁ … v₆ ]
vᵢ_dot = g(η_i, η_{i+1})   (nonlinear hydrodynamic coupling)
```

**Dynamics (mechanical‑electrical)**  

```
M * ν̇ = τ_e + C(ν)·ν + D*ω - B(η)*η_bar  + g_wave
τ_e = J_gα   (α is pitch angle, α_dot = ω_pitch)
```

- `M` – inertia matrix of the rotor‑blades system.  
- `C(ν)` – nonlinear damping vector accounting viscous drag & added mass.  
- `D*ω` – hydrodynamic torque from current shear.  
- `-B(η)*η_bar` – wake effect (nonlinear coupling between blade pitch and upstream flow).  
- `g_wave` – stochastic disturbance representing wave excitation.

**Integration Method** – Fourth‑order Runge–Kutta (RK4) with fixed step Δt = 0.01 s for numerical simulation; analytical Jacobian available for MPC formulation.

### Parameter Values

| Symbol | Value | Source / Assumption |
|--------|-------|---------------------|
| M      | Diagonal matrix of blade mass moments: [2.1,1.8,2.5] kg·m² per blade (total 6) | Manufacturer specifications |
| D      | Viscous drag coefficient vector: [0.015,0.014,0.016] per blade rad⁻¹ | Empirical correlation for Reynolds ≈10⁵ |
| B(η)   | Wake‑induced torque gain: `B = 1.2 * η̄` (linearized around steady operating point) | Linearised state‑space model |
| Noise σ_wave | Standard deviation of wave‑induced force magnitude: 0.045 N per blade element | Wave climate statistical analysis |

---

## Mathematical Models and Assumptions

### Equations Extracted from Paper

1. **State Dynamics (continuous):**  
   $$\dot{\eta} = f(\eta, \tau_e)$$ where $f_i(\eta,\tau_e)=v_i(\eta,\omega)$ with nonlinear drag terms.

2. **Torque Balance:**  
   $$\tau_e + C(\nu)\cdot\nu + D\omega - B(η)*η_{\bar{}} = g_{wave}.$$  
   *Assumption:* Linearised wake effect (`B`) valid for operation within ±15 % of nominal power.

3. **Disturbance Model (stochastic):**  
   $$g_{wave}= \boldsymbol{\eta}_w(t) + n(t),\qquad 
   \mathbb{E}[n(t)]=0,\;\operatorname{Var}(n)=σ^2_{wave}I.$$  
   *Assumption:* White‑noise approximation; no long‑term correlation.

4. **Nonlinear Damping (per blade):**  
   $$C(\nu_i)=k_d\bigl(|ν_i|+α\bigr),\qquad k_d=0.0125,$$  
   *Assumption:* Linearised around steady state, neglecting higher‑order harmonics.

### Assumptions & Validity Ranges

- **Linear wake model** (`B(η)`) is valid when turbine operates below rated power (≤ 80 % of design point).  
- **Small‑angle approximation** for pitch dynamics: `sinα ≈ α` and `cosα ≈ 1`.  
- **Time‑invariance of drag coefficients** holds over the operational wave spectrum considered.  
- **Noise magnitude** is bounded; no extreme wave events beyond historical maximum amplitudes are assumed.

---

## Controller Selection Recommendations

### Hierarchical Recommendation (based on system characteristics)

| Level | Controller Type | Reasoning / Suitability |
|-------|------------------|--------------------------|
| 1️⃣ Simple/Static | **PID for pitch** only (single‑input, single‑output) | - Directly addresses the dominant torque disturbance `Dω`. <br>- Linear in state space; easy to tune. <br>- Limited by nonlinear coupling (`B(η)*η_bar`) but can be supplemented with feed‑forward if needed. |
| 2️⃣ Linear State‑Space | **Linear Quadratic Regulator (LQR)** or **H₂ optimal controller** on a linearised plant around steady state | - Handles multiple inputs/outputs simultaneously. <br>- Requires observability & controllability; can be verified by solving the algebraic Riccati equation using the Jacobian `A(η)` derived from dynamics. |
| 3️⃣ Nonlinear / Optimisation‑Based | **Model Predictive Control (MPC)** with a first‑order Taylor expansion of nonlinearities (or constrained MPC) | - Explicitly accounts for wake model `B(η)*η_bar` and disturbance noise, providing optimal control within physical limits. <br>- Allows constraints on torque command saturation (`τ_e ≤ τ_{max}`). |
| 4️⃣ Robust / Adaptive | **Sliding‑Mode Controller** or **Tube MPC** for guaranteed robustness to unmodelled uncertainties (e.g., extreme wave events) | - Provides high disturbance rejection and anti‑interference ability when linearised model assumptions are violated. <br>- Useful as a fallback if plant exhibits large nonlinearity deviations beyond the first‑order expansion. |

### Critique of Existing Implementation

- The paper presents an **MPC** with only *linearised* dynamics (ignoring `B(η)*η_bar`). This is sufficient for most operating points but may cause chattering or constraint violations under high wave loads.
- **Alternative:** Introduce a *state‑dependent weighting* in the cost function to penalise large pitch angles, reducing chattering while preserving robustness.

---

## Scenarios / Test Conditions

| ID | Scenario Description | Key Parameters |
|----|-----------------------|----------------|
| S1 | Baseline steady operation at 50 % rated power, nominal wave spectrum. | Disturbance amplitude σ_wave = 0.045 N, reference signal η̄ = (target) |
| S2 | High‑wave excitation event with instantaneous max σ_wave = 0.12 N for 5 s bursts. | Same as S1, but stochastic term scaled by factor 2.7 |
| S3 | Pitch saturation test: impose torque command limit τ_e ≤ τ_{max} (≈120 % of nominal). | Duty cycle = 30 % over simulation time |

---

## Metrics

- **Settling Time:** Time for turbine power output to reach 99 % of setpoint after disturbance.  
- **Integral Absolute Error (IAE):** Cumulative pitch error, reflecting total torque excursions.  
- **Control Effort:** Number of PID tuning steps or MPC replanning cycles executed per hour.  
- **Constraint Saturation Events:** Frequency and duration of torque command saturation during S2 scenario.

Metrics are computed using standard numerical integration over simulation windows (10 s for each step) with error tolerance ε = 0.01 % power deviation.

---

## Results and Conclusions

### Quantitative Findings
- **Baseline LQR** reduces IAE by ~35 % relative to pure PID on steady conditions, but exhibits noticeable overshoot due to ignoring wake dynamics.
- **MPC (linearised)** improves set‑up time by 12 % while keeping torque within limits; however, during S2 it triggers ≈45 % of replanning cycles because of the unmodelled wake effect.
- **Sliding‑Mode** variant eliminates chattering under high wave loads but increases control effort (≈30 % higher IAE) due to aggressive switching.

### Strengths & Weaknesses
- Strength: Model Predictive Control captures nonlinear coupling and disturbance noise, offering robust performance across operating points.  
- Weakness: Linearisation may be insufficient during extreme excitation events; additional state augmentation or adaptive gain tuning is required for full reliability.

### Novel Contributions
- First application of **tube‑constrained MPC** to marine turbines under stochastic wave loading.  
- Demonstration that a *state‑dependent weighting* in the MPC cost function mitigates chattering without sacrificing robustness.

### Practical Implications
The recommended hybrid approach (linearised MPC with tube constraints) provides designers with a scalable solution for real‑world deployment, balancing computational tractability and disturbance rejection needed in wave energy environments.

---

## Limitations & Future Work

- **Linearisation** assumption may degrade performance during high‑intensity wave events; future work could incorporate *full nonlinear dynamics* or *real‑time adaptive MPC*.  
- No consideration of mechanical wear effects (blade fatigue) is included; a reliability layer would be beneficial for long‑term operation.  

---

This technical summary follows the requested structure, using markdown tables and LaTeX formatting where appropriate, while reflecting the full content extracted from the paper on marine current turbine control.