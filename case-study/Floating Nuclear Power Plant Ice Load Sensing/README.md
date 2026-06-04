# Floating Nuclear Power Plant - Ice Load Sensing

## Reference
**Title:** Ice load sensing of floating nuclear power plant using a reduced mathematical model  
**Authors:** Donghui Wang, Jiajun Wang, Meng Zhang, Xianqiang Qu  
**Journal:** Ocean Engineering, Vol. 312, 2024, Article 119350  
**DOI:** https://doi.org/10.1016/j.oceaneng.2024.119350

---

## System Description

A **Floating Nuclear Power Plant (FNPP)** moored in an arctic or sub-arctic sea (e.g., Bohai Sea near Yantai, China) is subject to **distributed quasi-static ice loads** during winter freeze-up. These loads - thermal expansion pressure from ice sheets and crushing ice pressure on the reactor compartment bow - cannot be measured directly and must be inferred from **structural strain measurements** via an inverse problem.

The paper develops an **indirect ice-load sensing** framework that:
1. Constructs a **reduced-order structural model** (influence coefficient matrix) mapping distributed ice pressures to measurable strains.
2. Optimises **sensor placement** using an Improved Simulated Annealing Algorithm (ISAA) to minimise the ill-posedness of the inverse problem.
3. Reconstructs the full distributed ice-load field from a sparse set of strain gauge readings.

---

## Mathematical Model

### Forward problem (structural response)

```
epsilon = A . p + noise
```

where:
- `epsilon \in ℝ^m` - measured strain vector at *m* sensor locations
- `p \in ℝ^n` - unknown distributed ice pressure vector at *n* load points (n >> m in general)
- `A \in ℝ^mˣ^n` - influence coefficient (transfer) matrix, computed from FEM

Each column `A[:,j]` is the strain field produced by a unit pressure at load point `j`.

### Inverse problem (load reconstruction)

```
p* = argmin_p { ||A p - epsilon||^2 + lambda ||Gamma p||^2 }
```

Tikhonov regularised least-squares. In practice the system is **ill-posed** (condition number of `A` very high), so sensor placement is critical.

### Ill-posedness metric

The **D-optimal** design criterion maximises `det(AᵀA)` (maximises information content of selected sensors). The paper uses:

```
ISAA: maximise   det(A_S^T A_S)
      subject to |S| = m   (select m rows / sensor positions)
```

where `A_S` is the sub-matrix of rows corresponding to chosen sensors.

The improved simulated annealing (ISAA) uses:
- **Adaptive temperature schedule** to escape local optima
- **Block-wise candidate generation** for large-dimension problems (n > 100)

### Reduced model

The full FEM model (thousands of nodes) is reduced to a compact influence-coefficient matrix via static condensation, enabling fast online inversion.

---

## State / Signal Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `p(x,t)` | Distributed ice pressure | Pa or kN/m^2 |
| `epsilon(x,t)` | Structural strain at gauge locations | muepsilon |
| `A` | Influence coefficient matrix (FEM-derived) | muepsilon / (kN/m^2) |
| `S` | Selected sensor index set | - |

## Inputs

| Signal | Description |
|--------|-------------|
| `epsilon_meas` | Measured strains from onboard strain gauges |
| FEM model | Geometry + material properties of reactor compartment |

## Outputs

| Signal | Description |
|--------|-------------|
| `p*` | Reconstructed distributed ice pressure field |
| Condition number `κ(A_S)` | Ill-posedness indicator of selected sensor set |

---

## Control / Estimation Objectives

This is primarily a **structural health monitoring / load estimation** problem, not a classical feedback control problem. However, the estimated ice load can feed:

1. **Structural integrity monitoring** - alert if estimated ice pressure exceeds design limit.
2. **Thermal management** - regulate reactor cooling power in response to hull stress.
3. **Station-keeping control** - adjust mooring tensions to counteract asymmetric ice loading.

---

## Relevant Control/Estimation Methods in lib/

| Method | Role |
|--------|------|
| **Kalman Filter / EKF** | Recursive load estimation from streaming strain data |
| **Particle Filter** | Non-Gaussian ice-load posterior (e.g., intermittent crushing) |
| **Bayesian Optimizer** | Sensor placement optimisation (alternative to ISAA) |
| **RecursiveLeastSquares** | Online update of influence-coefficient model as ice conditions change |
| **GaussianProcess** | Spatial interpolation of ice pressure field from sparse measurements |

---

## Key Parameters

| Parameter | Description |
|-----------|-------------|
| Reactor compartment length | ~30–50 m (typical FNPP section) |
| Ice pressure range | 0.1–2.0 MPa (Bohai Sea, mild ice conditions) |
| Number of load points `n` | Hundreds (FEM resolution) |
| Number of sensors `m` | Tens (practical installation limit) |
| Condition number threshold | Minimised by ISAA sensor placement |

---

## Scenarios

- **Thermal expansion ice load** - slow quasi-static pressure build-up as ice sheet freezes around hull
- **Dynamic ice crushing** - impulsive loads from ice-breaking events at low frequency (< 2.5 Hz)
- **Comparative methods** - ISAA vs. D-optimal, C-optimal, block C-optimal (BCOD) sensor selection

---

## Implementation Notes

- The influence-coefficient matrix `A` is computed once offline from a validated FEM model.
- For real-time estimation, the regularised inverse `(AᵀA + lambdaI)^-^1Aᵀ` can be precomputed and applied to incoming strain streams.
- The `RecursiveLeastSquares` class in `lib/` with `(na=0, nb=n, lambda=0.99)` can perform online sequential estimation of `p*` as new strain readings arrive.
- The `BayesianOptimizer` in `lib/` can replace ISAA for sensor placement: define cost as negative `det(A_S^T A_S)` and optimise over the discrete sensor-index space.
