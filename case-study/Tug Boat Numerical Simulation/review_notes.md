# Tug Boat Numerical Simulation - Review, Feedback, and Improvement Notes

**Document:** Additional Review / Feedback / Improvement Notes
**Audience:** Technical engineers familiar with control systems and C++ programming
**Date:** 2026-05-23

---

## 1. Known Gaps Between This Simulation and the Reference Paper

### 1.1 Wave Drift Approximation (High Priority)

The reference paper (Li et al. 2026) uses a full Quadratic Transfer Function (QTF) model
for second-order wave drift forces, evaluated at each simulation step using the complete
directional wave spectrum. The present implementation substitutes a two-component JONSWAP
approximation:

$$
\boldsymbol{\tau}_\mathrm{drift} \approx \left(\sum_{j=1}^{2} a_j\cos(\omega_j t+\phi_j)\right)^2 \mathbf{d}
$$

This approximation underestimates mean drift forces at off-axis wave angles and omits the
cross-coupling between wave direction and vessel heading. Expect systematic IAE
underestimation in scenarios S3 (135^\circ) and S4 (180^\circ) relative to paper Table 7.

**Mitigation (v2):** Import QTF coefficient tables from the paper's supplementary data or
from WAMIT/AQWA panel method output. Replace the scalar drift coefficients with a
direction-dependent $3\times3$ matrix $\mathbf{D}_\mathrm{QTF}(\beta)$.

### 1.2 Tug-Tug Hydrodynamic Interaction

The paper's reconstructed matrices treat each tugboat's added mass and damping as
independent. In reality, tugboats in close proximity to each other and to the barge
experience hydrodynamic interaction effects (pressure field interference). These are not
modeled. The error is expected to be small (< 5%) at the tugboat separations used in
paper Figure 5, but should be noted in the validation report.

### 1.3 Coriolis Term for Combined System

The `C_re(nu)` matrix implemented here retains only the barge's rigid-body Coriolis term.
The tug Coriolis contributions ($\tilde{C}^{t,i}$) are omitted in the current prototype
because they are second-order in velocity and negligible for the low-speed positioning
task. If scenarios with faster transit speeds are added, this term should be reinstated
from Li et al. Eq. (20).

---

## 2. SMC Implementation Risk: Sign Convention

The GDScript prototype (Tug Boat Game) exhibited IAE values 500-5000* larger than paper
Table 7 during initial testing. Root-cause analysis identified a probable sign error in
the SMC equivalent control term: the model-cancellation contribution
$M_\mathrm{re}\Lambda\dot{\mathbf{e}}$ may be added rather than subtracted, causing the
equivalent control to drive the barge away from the target.

**Action required:** Before any performance comparison is published, validate the C++ SMC
implementation against paper Table 7 scenario S2 (90^\circ, ideal observer, 5400 s). Accept
IAE within +/-10% of the paper value. If the mismatch exceeds 10%, inspect the sign of
$\boldsymbol{\tau}_\mathrm{eq}$ and the heading-error unwrapping in the sliding surface
computation.

---

## 3. Linearization Domain for KF and MPC

Both Mode 2 (KF-PID) and Mode 4 (MPC) linearize the plant about the zero-velocity
equilibrium. This linearization is valid when $|u|$, $|v| \ll U_\mathrm{char}$ and
$|r| \ll r_\mathrm{char}$ (low-speed station-keeping). It will degrade for:

- Transit scenarios where the barge is moving at speed to a new target position
- Heading changes greater than approximately 20^\circ from the linearization point

**Mitigation options:**
- Schedule multiple linearization points around the trajectory (gain scheduling)
- Replace the linear KF with an Extended Kalman Filter (`ExtendedKalmanFilter.h` is
  available in the Toolbox) using the nonlinear RK4 dynamics as the prediction model
- Accept the linearization error as a known limitation and document it in the validation
  report for large-heading-change scenarios

---

## 4. Thrust Allocation: Pseudo-Inverse vs. QP

The current thrust allocation uses a Moore-Penrose pseudo-inverse as the baseline:

$$
\mathbf{T}^* = B^\top(BB^\top)^{-1}\boldsymbol{\tau}_c
$$

followed by ad-hoc clamping to $[T_\min,\,T_\max]$ and rate-limiting. This approach does
not minimize a cost function and does not account for the interaction between box
constraints and the force balance equation - clamping one tug breaks the equality
$B\mathbf{T} = \boldsymbol{\tau}_c$.

**Improvement:** Formulate the allocation as a quadratic program:

$$
\min_{\mathbf{T}}\;\mathbf{T}^\top W \mathbf{T} \quad \text{s.t.}\; B\mathbf{T} = \boldsymbol{\tau}_c,\; T_\min \leq T_i \leq T_\max,\; |\Delta\mathbf{T}|\leq\Delta T_\max
$$

The `DiscreteMPC.h` QP infrastructure (or a standalone OSQP call) can be reused for this.
Adding a slack variable $\mathbf{s}$ with large penalty allows the constraint to be relaxed
when $\boldsymbol{\tau}_c$ is unachievable, replacing silent clamping with an explicit
force-error signal.

---

## 5. Scenario Coverage Recommendations

The three scenarios (S2, S3, S4) replicate paper Table 5 conditions and are essential for
validation. Two additional scenarios are recommended for a complete controller comparison:

| ID | Description | Purpose |
|----|-------------|---------|
| S1 | Zero disturbance, zero initial error | Verify plant integration stability (no drift) |
| S5 | Step wind direction change (90^\circ -> 135^\circ at $t=300\,\text{s}$) | Test disturbance rejection and transient response |

S1 also serves as a unit test: with no disturbance and no error, all controllers should
output $\boldsymbol{\tau}_c = \mathbf{0}$ and the plant state should remain stationary.

---

## 6. Performance Metric: Weighted IAE Composite

The paper reports per-axis IAE values (IAE$_x$, IAE$_y$, IAE$_\psi$) separately. For
ranking controllers in a single scalar, the GDScript prototype used:

$$
\mathrm{IAE}_\mathrm{total} = \mathrm{IAE}_x + \frac{\mathrm{IAE}_y}{10} + \mathrm{IAE}_\psi
$$

The factor of 1/10 on IAE$_y$ was chosen heuristically. This weighting should be justified
or replaced before results are presented formally. Options:
- Equal weights (sum of normalized per-axis IAE)
- Weights derived from the MPC $Q_\mathrm{mpc}$ diagonal (position cost ratio)
- Report all three axes separately and avoid aggregation

---

## 7. Deterministic Reproducibility

All stochastic components (wave phase initialization, sensor noise injection) must use a
seeded pseudo-random number generator initialized from the scenario configuration. The
same scenario JSON with the same seed must produce bit-identical output on repeated runs.
This is required for:

- Cross-controller comparison (all controllers face the same disturbance realization)
- Debugging (a specific run can be reproduced exactly from its seed)
- Paper-result replication (seed matches the paper's simulation setup if specified)

Use `std::mt19937` seeded from the scenario `seed` field. Do not use `std::rand`.

---

## 8. Potential Controller Toolbox Additions

The following Controller Toolbox classes exist but are not included in the initial five-mode
comparison. They are candidates for a v2 extension:

| Class | Mode | Rationale for Deferral |
|-------|------|------------------------|
| `DiscreteLQR.h` | LQR | Requires discrete Riccati solve; less instructive than MPC for this task |
| `DiscreteLQG.h` | LQG | KF + LQR combined; superseded by the separate KF-PID comparison |
| `ExtendedKalmanFilter.h` | EKF-PID | Nonlinear KF; valuable for transit scenarios, deferred until S5 is added |
| `DiscreteADRC.h` | ADRC | Active Disturbance Rejection; strong alternative to SMC for validation |
| `GeneralizedPredictiveControl.h` | GPC | Closely related to MPC; would require separate $N_2$ horizon tuning |

---

## 9. CSV Logging and Analysis Pipeline

The analysis Python scripts (`plot_run.py`) should produce at minimum:

- **Position trace plot:** $x(t)$, $y(t)$, $\psi(t)$ for all five controllers overlaid,
  with disturbance level annotated
- **Thruster trace:** $T_1(t),\ldots,T_4(t)$ showing saturation events
- **Score bar chart:** IAE$_x$, IAE$_y$, IAE$_\psi$, $E_\mathrm{fuel}$, Sat$_\mathrm{count}$
  grouped by controller and scenario
- **Phase-portrait:** $(e_x, e_y)$ scatter per controller to visualize trajectory quality

All plots should be saved to `logs/figures/` and included in the project documentation.

---

## 10. Build and Integration Checklist

Before coding begins, verify the following integration prerequisites:

- [ ] Controller Toolbox headers compile cleanly with C++17 and Eigen 3.4 on target platform
- [ ] `nlohmann/json` single-header available at `include/nlohmann/json.hpp`
- [ ] `plant_params.json` parses correctly and produces physical $M_\mathrm{re}$ (positive
      definite, diagonal dominant)
- [ ] RK4 integrator passes zero-input stability test: state remains within $10^{-6}$ of
      initial value after 100 steps with $\boldsymbol{\tau}=\mathbf{0}$
- [ ] SMC IAE on S2 (ideal observer) within +/-10% of Li et al. Table 7 before any other
      controller is evaluated
- [ ] CSV output contains correct column count and no NaN/Inf values across a full 5400 s run
