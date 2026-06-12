# Controller Toolbox - Cumulative Bug Report (Part 51+)

**Active issues start at Part 51.** Earlier history is archived in two compact references:
- [`docs/compact_bug_report_parts_1-25.md`](compact_bug_report_parts_1-25.md) — Parts 1-25 (2026-05-19 through 2026-05-30)
- [`docs/compact_bug_report_parts_26-50.md`](compact_bug_report_parts_26-50.md) — Parts 26-50 (2026-05-31 through 2026-06-11)

Read both compact files for tribal knowledge before making any changes to controllers or case studies.

---

## Open Issues Log (Part 51+)

*(Append dated entries below as work proceeds.)*

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| **P1** | `DAESystem` struct + `dae2ode()` — Index-1 semi-explicit DAE; Newton solve on `g` | HIGH | **Done (Part 51)** |
| **P2** | `c2d()` overload for DAE — linearise + algebraic elimination + ZOH/Tustin | MED | **Done (Part 51)** |
| **P3** | DAE-aware EKF — post-update algebraic projection via `consistentInit()` | MED | **Done (Part 51)** |
| **E1** | `GreyBoxEstimator` — non-linear param estimation via Levenberg-Marquardt | HIGH | Open |
| **E2** | `RecursiveGreyBoxEstimator` — augmented-state UKF for online param tracking | HIGH | Open |
| **E3** | GP Residual Model — extend `GaussianProcess` with uncertainty output | MED | Open |
| **E4** | MHE Polytopic Constraints — extend MHE with `C_ineq`/`d_ineq` | MED | Open |
| **H1** | `HybridModel` base class — `IPlantModel` with `f_phys + f_data` | MED | Open |
| **H2** | `HybridMPC` — `NonlinearMPC` variant using `HybridModel` | MED | Open |
| **H3** | RL-MPC stitching Python example | LOW | Open |
| **H4** | `HybridModelTrainer` — hyperopt for `f_data` component | LOW | Open |
| **D1** | Mismatch Detector — CUSUM on KF/MHE innovation | LOW | Open |
| **D2** | Digital Twin Lite Python app | LOW | Open |
| **C2** | 6 spec-only stubs (BEMS + MEMS no blocker; others need plant design) | MED | Open |
| **B36-3** | Unify NaN-guard across controller fleet | MED | Open |
| R1 | Edge-case contract matrix tests for every controller family | MED | Open |
| T3 | Full DK-iteration with vector-fitting rational D(jω) | LOW | Open |
| B36-2 | `ex79_registry_monitor` monitors nothing (M3 telemetry mis-wired) | LOW | Open |
| REL | Rebuild `ctrl_toolbox.pyd` in Release | LOW | Open |
| M4 | `template<typename Scalar>` leaf algorithms for embedded float target | Backlog | Open |

---

*(New parts appended below as work proceeds.)*

---

## Part 51 — DAE Architecture (P1/P2/P3) — 2026-06-12

**P1 — `DAESystem` + `consistentInit` + `dae2ode`** (`lib/PlantModel.h/.cpp`)

- `DAESystem` struct: `f` (differential), `g` (algebraic), `h` (output), `n_diff`, `n_alg`, `Ts`.
- `consistentInit(dae, x1_init, u0, x2_guess)`: Newton-Raphson (LDLT) solving `g=0` for `x2`; up to 20 iters, tol=1e-9.
- `dae2ode(dae)`: returns discrete step function `x_aug_next = F(x_aug, u)`. Forward Euler for `x1`, Newton projection for `x2` at both current and next `x1`. `Ts` must be set on `DAESystem`.
- Three static central-difference Jacobian helpers (`algJacX1`, `algJacX2`, `algJacU`) in `PlantModel.cpp`.
- `CTRL_REGISTER_FEATURE(dae_system)` added after `namespace ctrl`.

**P2 — `c2d(DAESystem, x1_op, x2_op, u_op, Ts, method)`** (`lib/PlantModel.h/.cpp`)

- Index-1 algebraic elimination: `A_red = A11 - A12*G2⁻¹*G1`, `B_red = B1 - A12*G2⁻¹*B2` where all Jacobians are computed numerically via `algJac*` helpers.
- Checks `rcond(G2) > 1e-12`; throws `std::runtime_error("c2d(DAESystem): G2 is singular — DAE is not Index-1 at operating point.")` otherwise.
- Output matrix built from `h` Jacobians (or identity w.r.t. `x1` if `h` not set).
- Dispatches to existing `c2d(StateSpace, Ts, method)` for ZOH/Tustin.
- Python binding registered as `dae_c2d` (avoids `py::overload_cast` ambiguity with existing `c2d`).

**P3 — DAE-aware EKF projection** (`lib/ExtendedKalmanFilter.h/.cpp`)

- `setAlgebraicConstraint(g_alg, n_diff, n_alg, tol=1e-9)`: attaches algebraic constraint function; validates `n_diff + n_alg == n_states_`.
- `hasAlgebraicConstraint()`: bool accessor.
- `projectAlgebraicStates(u)`: called at end of `update()` when constraint is set. Newton-Raphson on `x2` block using `numericalJacobian`; then covariance projection `P = J_proj * P * J_proj'` where `J_proj = [[I, 0]; [-G2⁻¹G1, 0]]`.
- SISO assumption: `u_scalar = u(0)` (consistent with `DAESystem::AlgFunc` signature).
- Independent `AlgConstraintFn` type alias in EKF (does not depend on `PlantModel.h`).

**Bindings / tests**

- `plantmodel_bindings.cpp`: `DAESystem` class with `set_f/set_g/set_h`, `consistent_init`, `dae2ode`, `dae_c2d`.
- `estimation_bindings.cpp`: `set_algebraic_constraint`, `has_algebraic_constraint` on `ExtendedKalmanFilter`.
- `smoke_test.py`: 4 DAE assertions (`consistent_init`, `dae2ode`, `dae_c2d`, `registry_has('dae_system')`).
- `tests/test_catch2_advanced.cpp`: 7 Catch2 tests — `[dae_system]` ×3, `[dae_c2d]` ×2, `[dae_ekf]` ×2.
