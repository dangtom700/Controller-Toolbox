SUMMARY OF: Sliding-mode-control-of-a-soft-robot-based-on-data-d_2024_Control-Engineerin.pdf

# Paper Title (to be filled)

**Reference:** Author et al. (Year) – brief descriptor.

---

## System / Plant Model  

*Concise description of the system: degrees of freedom, key states, inputs, disturbances.*  
If multiple configurations are discussed, specify which one is used and why it was chosen.

### State Vector  

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | x | Position / displacement | m |
| 1 | v | Velocity | m/s |
| … | … | … | … |

### Inputs and Disturbances  

**Control inputs:** list with saturation limits if given.  
**Environmental / external disturbances:** wind, waves, current, noise, etc.; specify parameters such as magnitude ranges or statistical properties.

### Governing Equations  

**Kinematics / state evolution** (if applicable): e.g., `η̇ = f(η, ν)`.  
**Dynamics:** e.g., `M * ν̈ = τ_control + τ_dist - C(ν)*ν - D*ν + g(η)`.

**Integration method** (e.g., RK4, Euler).

### Parameter Values  

Key numeric parameters in a table:

| Parameter | Symbol | Value | Units |
|-----------|--------|-------|-------|
| Mass      | m      | …     | kg    |
| Damping   | c      | …     | N·s/m |
| Inertia   | J      | …     | kg·m² |
| Wave height | H_w  | …     | m    |

---

## Mathematical Models and Assumptions  

Extract **all** key equations from the paper (state‑space, transfer functions, nonlinearities, time delays). For each equation note:

- What physical phenomenon it represents.
- Underlying assumptions (linearity, time‑invariance, neglected effects).
- Validity ranges (e.g., low speed, small angles).

Write equations in inline LaTeX or display math as appropriate.

---

## Controller Selection Recommendations  

Based **only** on the system’s mathematical structure extracted above, provide a hierarchical recommendation of controller types – from simple/static to advanced/robust.

For each recommended controller, briefly justify suitability:

1. **Simple / baseline (e.g., PID, lead‑lag)** – when does it work? what are its limitations here?
2. **Linear state‑space (e.g., LQR, LQG)** – assumptions required (observability, controllability, Gaussian noise)?
3. **Nonlinear / optimisation‑based (e.g., MPC, NMPC)** – constraints or nonlinearities motivating this approach?
4. **Robust / adaptive (e.g., sliding mode, tube MPC, MRAC, H∞)** – specific uncertainties or disturbances requiring robustness?

If the paper already implements certain controllers, critique their choice and suggest alternatives that might perform better.

---

## Scenarios / Test Conditions  

Table with ID, Description, key parameters (reference signals, disturbance levels, duration).

| Scenario | Description | Reference Signal | Disturbance Level |
|----------|-------------|------------------|-------------------|
| S1       | …           | …                | …                 |

---

## Metrics  

List of performance metrics used (IAE, settling time, control effort, saturation events, etc.) and how they are computed.

---

## Results and Conclusions  

- Main quantitative findings (with numbers).  
- Strengths and weaknesses of the proposed approach.  
- Novel contributions and practical implications.

---

## Limitations and Future Work (if stated)  

Explicitly mentioned limitations and suggested extensions.