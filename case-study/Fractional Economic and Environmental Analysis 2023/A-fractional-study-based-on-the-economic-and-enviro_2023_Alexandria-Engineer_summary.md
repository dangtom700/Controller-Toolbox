SUMMARY OF: A-fractional-study-based-on-the-economic-and-enviro_2023_Alexandria-Engineer.pdf

# Paper Title (to be filled by user)

**Reference:** Author et al. (Year) – brief descriptor of the research topic.

---

## System / Plant Model  

*Concise description of the system: degrees of freedom, key states, inputs, disturbances.*  
If multiple configurations are presented in the paper, specify which one is used for the analysis.

### State Vector  

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | x | Position / displacement | m |
| 1 | v | Velocity | m/s |
| … | … | … | … |

### Inputs and Disturbances  

**Control inputs:** list with saturation limits if given.  
**Environmental / external disturbances:** wind, waves, current, noise, etc.; include parameter values (e.g., magnitude of wave forces).

### Governing Equations  

**Kinematics / state evolution** (if applicable): e.g., `eta_dot = f(eta, nu)`.  
**Dynamics:** e.g., `M * ν̇ = τ_control + τ_dist - C(nu)*ν - D*ν + g(η)` .  
**Integration method:** specify whether the paper uses explicit Euler, Runge‑Kutta (RK4), or another scheme.

### Parameter Values  

Key numeric parameters (masses, damping coefficients, inertias, etc.) in a table:

| Parameter | Symbol | Value | Units |
|-----------|--------|-------|-------|
| Mass      | m      | …     | kg    |
| Damping   | c      | …     | N·s/m |
| Inertia   | J      | …     | kg·m² |
| …         | …      | …     | … |

---

## Mathematical Models and Assumptions  

Extract **all** key equations from the paper (state‑space, transfer functions, nonlinearities, time delays). For each equation, note:

- What physical phenomenon it represents.
- Underlying assumptions (e.g., linearisation around steady state, neglect of higher‑order terms).
- Validity ranges (e.g., low‐speed regime, small angle approximation).

Write equations in **inline LaTeX** `$...$` or **display math** `$$...$$`. If the paper compares multiple models (linear vs. nonlinear), present them separately.

---

## Controller Selection Recommendations  

Based **only on the system’s mathematical structure** extracted above, provide a hierarchical recommendation of controller types – from simple/static to advanced/robust.

For each recommended controller, briefly justify why it would be suitable:

1. **Simple / baseline (e.g., PID, lead‑lag)** – when does it work? what are its limitations here?
2. **Linear state‑space (e.g., LQR, LQG)** – what assumptions must hold (observability, controllability, Gaussian noise)?
3. **Nonlinear / optimisation‑based (e.g., MPC, NMPC)** – what constraints or nonlinearities motivate this?
4. **Robust / adaptive (e.g., sliding mode, tube MPC, MRAC, H∞)** – what specific uncertainties or disturbances require robustness?

If the paper already implements certain controllers, critique their choice and suggest alternatives that might perform better.

---

## Scenarios / Test Conditions  

Table with ID, Description, key parameters (e.g., reference signals, disturbance levels, duration).

| Scenario ID | Description | Reference Signal | Disturbance Level |
|-------------|-------------|------------------|-------------------|
| S1          | Baseline operation | …               | …                 |
| S2          | High‑wave excitation | …               | …                 |
| …           | …            | …                | …                 |

---

## Metrics  

List of performance metrics used (IAE, settling time, control effort, saturation events, etc.) and how they are computed.

| Metric | Formula / Definition | Target / Typical Range |
|--------|----------------------|-----------------------|
| Settling Time | $t_{settle} = \text{time to } < 5\%$ of steady‑state | … |
| Integral Absolute Error | $IAE = \int_0^\infty |e(t)|dt$ | ≤ X s·m⁻¹ |
| Control Effort | RMS of actuator command | ≤ Y N·s |

---

## Results and Conclusions  

- Main quantitative findings (with numbers).
- Strengths and weaknesses of the proposed approach.
- Novel contributions and practical implications.

---

## Limitations and Future Work (if stated)  

Explicitly mentioned limitations and suggested extensions.