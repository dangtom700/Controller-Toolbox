SUMMARY OF: Mathematical-analysis-of-fractional-order-alcoh_2023_Alexandria-Engineering-.pdf

# Paper Title (to be filled)

**Reference:** Author et al. (Year) - brief descriptor.

---

## System / Plant Model

Concise description of the system: degrees of freedom, key states, inputs, disturbances.  
If the paper describes multiple configurations, clarify which one is used.

### State Vector  

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | x | Position / displacement | m |
| 1 | v | Velocity | m/s |
| ... | ... | ... | ... |

### Inputs and Disturbances  

**Control inputs:** list with saturation limits if given.  
**Environmental / external disturbances:** wind, waves, current, noise, etc. - specify parameters.

### Governing Equations  

**Kinematics / state evolution** (if applicable):  
`eta_dot = f(eta, ν)`

**Dynamics:**  
`M . ν. = tau_control + tau_dist - C(ν).ν - D.ν + g(eta)`  

**Integration method** (e.g., RK4, Euler).

### Parameter Values  

Key numeric parameters (mass, damping, inertia, etc.) in a table.

---

## Mathematical Models and Assumptions

Extract **all** key equations from the paper (state-space, transfer functions, nonlinearities, time delays).  
For each equation, note:

- What physical phenomenon it represents.
- Underlying assumptions (linearity, time-invariance, neglect of certain effects).
- Validity ranges (e.g., low speed, small angles).

Write equations in **inline LaTeX** `$...$` or **display math** `$$...$$`.  
If the paper compares multiple models (e.g., linear vs. nonlinear), present them separately.

---

## Controller Selection Recommendations

Based **only on the system's mathematical structure** (nonlinearities, coupling, constraints, disturbances, uncertainty) extracted above, provide a **hierarchical recommendation** of controller types - from **simple/static** to **advanced/robust**.

For each recommended controller, briefly justify why it would be suitable:

1. **Simple / baseline (e.g., PID, lead-lag)** - when does it work? what are its limitations here?
2. **Linear state-space (e.g., LQR, LQG)** - what assumptions must hold (observability, controllability, Gaussian noise)?
3. **Nonlinear / optimisation-based (e.g., MPC, NMPC)** - what constraints or nonlinearities motivate this?
4. **Robust / adaptive (e.g., sliding mode, tube MPC, MRAC, Hinf)** - what specific uncertainties or disturbances require robustness?

If the paper already implements certain controllers, critique their choice and suggest alternatives that might perform better.

---

## Scenarios / Test Conditions  

Table with ID, Description, key parameters (e.g., reference signals, disturbance levels, duration).

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