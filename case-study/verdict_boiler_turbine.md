# Boiler-Turbine Case Study: Verdict & Recommendations

## Overview of the Implementation
Based on the provided ISA Transactions (2017) paper regarding the nonlinear predictive control of a boiler-turbine unit, the core challenge is controlling a highly nonlinear 3x3 multivariable plant (Drum Pressure, Electric Power, Fluid Density) across wide operating ranges (OP A, B, and C). The fixed-model linear MPC fails at points distant from its linearization because the plant gains vary by up to 24x across the operating envelope.

To recreate and validate this study in the current C++ controller toolbox:
1. **Model Updates:** We successfully extended `DiscreteMPC` with a `setPlant()` capability, allowing for the substitution of the internal prediction matrices (`F`, `Φ`, and `Hessian`) dynamically during operation.
2. **Case Study Implementation:** Added `examples/ex21_boiler_turbine_case_study.cpp` which implements the 3x3 nonlinear Bell-Åström differential equations, an online RK4 integrator, and an online finite-difference Jacobian lineariser. 
3. **Execution:** The plant tests `DiscreteMPC` configured with a fixed linear model versus one configured with the newly implemented successive online linearisation (Adaptive MPC). The Adaptive MPC achieves vastly superior tracking during massive load transitions (e.g., OP B to OP C), perfectly aligning with the paper's central thesis.

---

## Verdict & Recommendations for Controller Improvement

Having analysed the existing toolbox capabilities against the demands of highly nonlinear processes like the boiler-turbine, here are the targeted recommendations to further improve the controller quality:

### 1. Augment MPC with Explicit Integral Action or Disturbance Estimation
- **Current State:** The toolbox's `DiscreteMPC` uses an incremental formulation (`ΔU`) which provides *some* inherent integral action. However, the study shows that in highly nonlinear systems (or when persistent unmeasured disturbances occur), this is insufficient.
- **Recommendation:** Implement a disturbance observer (e.g., a steady-state Kalman filter estimating load disturbances) and explicitly feed this disturbance estimate into the MPC prediction horizon. This ensures zero steady-state error even when model mismatch persists.

### 2. Transition from Condensed QP to Sparse Active-Set or Interior Point Solvers
- **Current State:** The `DiscreteMPC` currently uses a condensed formulation and an unconstrained `.ldlt().solve()` followed by hard clamping for constraints.
- **Recommendation:** While adequate for small steps, hard clamping invalidates the optimality of the sequence when multiple constraints are active (as they often are during the massive abrupt transitions in TC1). Upgrading the MPC to use a proper active-set QP solver (like `OSQP` or `qOASES` linked as an optional C++ dependency) will dramatically improve large-signal transient tracking and constraint enforcement.

### 3. Expand the `ControllerStack` for Nonlinear Gain Scheduling
- **Current State:** Gain-Scheduled PID performed surprisingly well for wide-range tracking. Currently, the `ControllerStack` supports `Supervisory` (hard switching) and `Weighted` modes.
- **Recommendation:** Add a `Scheduled` mode to `ControllerStack` where continuous linear interpolation across a defined grid of operating points automatically blends the outputs or gains of multiple PIDs (or LQI controllers). This directly solves the degradation of standard PID at high loads (OP C).

### 4. Provide Analytical Jacobians for the MPC 
- **Current State:** Our example uses finite differences for the online linearisation, which requires 13 calls to the nonlinear `dynamics` function per step.
- **Recommendation:** If the user supplies the algebraic Jacobian, the successive linearisation overhead drops to almost zero. The MPC class could be extended to accept an `std::function` for evaluating analytical Jacobians directly. 

### Conclusion
The addition of the `setPlant()` dynamic update to `DiscreteMPC` elevates the toolbox from handling only piece-wise linear systems to being capable of controlling deeply nonlinear, complex multi-physics systems (like power plants) smoothly. The framework is mechanically sound; the next evolutionary step is integrating a robust QP solver to fully leverage the predictive horizons.
