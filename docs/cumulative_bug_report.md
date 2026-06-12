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
| **P1** | `DAESystem` struct + `dae2ode()` — Index-1 semi-explicit DAE; Newton solve on `g` | HIGH | Open |
| **P2** | `c2d()` overload for DAE — linearise + algebraic elimination + ZOH/Tustin | MED | Open |
| **P3** | DAE-aware EKF — post-update algebraic projection via `consistentInit()` | MED | Open |
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
