# Expanded Control Strategies & Plant Model Interference

*Target audience: control engineers and C++ architects extending the Controller Toolbox (C++20, Eigen 3.4+, pybind11). All equations are in discrete time unless noted. File references map to `lib/`, `examples/`, and `tests/` in the repository root.*

---

## Table of Contents

1. [Control Strategy Families (A1–A7)](#1-control-strategy-families-a1a7)
2. [Plant Model Function Interference (B1–B8)](#2-plant-model-function-interference-b1b8)
3. [Deep Behaviour Interference (C1–C6)](#3-deep-behaviour-interference-c1c6)
4. [Decision Framework](#4-decision-framework)
5. [Proposed Toolbox Extensions](#5-proposed-toolbox-extensions)

---

## 1. Control Strategy Families (A1–A7)

Each family is assessed for: mathematical core, toolbox mapping, a worked example, and whether it inherently requires a corrector (observer, model) or is self-contained.

---

### A1. Adaptive Control — RLS, MRAC, Gain Scheduling

**Core idea.** Parameters evolve online so the controller compensates for plant variation or uncertainty without offline re-design.

#### 1.1 Self-Tuning via RLS + GPC

The plant is identified as an ARX model online:

```
A(q⁻¹) y[k] = B(q⁻¹) u[k] + e[k]
```

RLS update (exponential forgetting, factor λ ∈ (0,1]):

```
K[k]      = P[k-1] φ[k] / (λ + φ[k]ᵀ P[k-1] φ[k])
θ̂[k]     = θ̂[k-1] + K[k] (y[k] - φ[k]ᵀ θ̂[k-1])
P[k]      = (I - K[k] φ[k]ᵀ) P[k-1] / λ
```

The identified θ̂ is converted to a `StateSpace` and fed to `GPC::setPlant()`, closing the adaptive loop.

**Toolbox mapping:**

| Component | Class | File |
|-----------|-------|------|
| Online identification | `RecursiveLeastSquares` | `lib/RecursiveLeastSquares.h` |
| Adaptive MPC | `GeneralizedPredictiveController::setPlant()` | `lib/GeneralizedPredictiveControl.h` |
| Adaptive MPC (LQR style) | `DiscreteMPC::setPlant()` | `lib/DiscreteMPC.h` |

**Example:** `ex28_gpc_adaptive.cpp` — PRBS excitation → N4SID ID → GPC hot-swap every 50 steps. Python mirror: `examples/python/ex39_gpc_adaptive_rls.py`.

**Prefers over corrector when:** The plant gain or time constant drifts by more than ±30% over the mission lifetime, making a fixed corrector ineffective.

**Corrector needed?** Optional. Without an observer the state is estimated from output only via CARIMA predictor. Adding a KF on top gives LQG-quality state estimation.

---

#### 1.2 Gain Scheduling

A scheduling variable ρ (e.g., airspeed, temperature, load) selects or interpolates among pre-designed gain tables:

```
K(ρ) = Σᵢ wᵢ(ρ) Kᵢ,   Σ wᵢ = 1,   wᵢ ≥ 0
```

**Toolbox mapping:** `ControllerStack::Weighted` mode with `wᵢ` as runtime weights, or direct matrix blending `K_blend = (1-ρ)*K_slow + ρ*K_fast` (see `ex41_lpv_gain_scheduling.cpp`).

**Prefers over simple PID when:** The plant has two or more distinct operating regimes with substantially different dynamics, and a single linear controller cannot satisfy performance at both extremes.

---

#### 1.3 MRAC — Model Reference Adaptive Control

Force the closed-loop to track a stable reference model M(s):

```
ė_m = A_m e_m + B_m r      (reference model error dynamics)
u = θᵀ(t) ω                (parameterised control law)
θ̇ = -Γ e_m Pᵀ B_m ω       (MIT/Lyapunov gradient adaptation)
```

**Toolbox mapping:** `RecursiveLeastSquares` provides the parameter update infrastructure; the Lyapunov gradient law is not yet wrapped — see [Proposed Extension E3](#e3-mrac-controller).

**Self-contained?** Yes — no separate observer required, though full-state feedback requires that all states are measurable or estimated.

---

### A2. Robust Control — H∞, mu-Synthesis, QFT

**Core idea.** Guarantee a worst-case performance bound for all plants in a norm-bounded uncertainty set.

#### 2.1 H∞ Synthesis

Mixed-sensitivity formulation: choose weighting functions W₁ (sensitivity), W₂ (control effort), W₃ (complementary sensitivity) and solve:

```
min_{K} ‖ [ W₁ S ]  ‖
        ‖ [ W₂ KS ] ‖∞
        ‖ [ W₃ T  ] ‖
```

where S = (I + GK)⁻¹, T = GK(I + GK)⁻¹. Solved by γ-bisection via Hamiltonian or discrete Riccati equations.

**Toolbox mapping:**

| Method | Call | File |
|--------|------|------|
| Direct H∞ | `DiscreteHinf::solve(plant, W1, W2, W3)` | `lib/DiscreteHinf.h` |
| Mixed sensitivity | `DiscreteHinf::mixedSensitivity(plant, W1, W2, W3)` | `lib/DiscreteHinf.h` |
| mu-synthesis | `DiscreteHinf::solveMuSyn(plant, params)` | `lib/DiscreteHinf.h` |

**mu-synthesis DK-iteration** (implemented in Part 18): at each iteration, scale the plant uncertainty channels by diagonal D matrices to minimise μ(Δ) upper bound, then re-solve H∞. With `MuSynParams::useRationalD = true`, first-order rational D_j(z) filters are fitted per channel:

```
D_j(z) = K · (z - z₀) / (z - p),   p = 0.85
```

Plant state augmented to n + n_z + n_w states.

**Example:** `ex34_mu_synthesis_full_dk.cpp` — constant vs rational D-scaling comparison.

**Self-contained?** Yes — the synthesised K is a fixed dynamic controller. No corrector required, though K is typically of order n_plant + n_weights.

**Prefers over corrector when:** There is a structured parametric uncertainty (e.g., ±20% gain, ±15% resonant frequency) that must be handled by design, not rejected online.

---

### A3. Optimal Control — LQR, MPC, GPC

**Core idea.** Minimise a cost functional over finite or infinite horizon subject to dynamics and constraints.

#### 3.1 LQR (Infinite Horizon)

```
J = Σ_{k=0}^{∞} (xₖᵀ Q xₖ + uₖᵀ R uₖ)
```

Solved offline via DARE → gain K = (R + BᵀPB)⁻¹BᵀPA. Online cost: one matrix-vector multiply per step.

**Toolbox:** `DiscreteLQR` + optional `LQRAdapter` for use inside `ControllerStack`.

#### 3.2 MPC (Finite Horizon, Constrained)

```
min_{ΔU} ½ ΔUᵀ H ΔU + fᵀ ΔU
s.t.     u_min ≤ u_prev + Γ ΔU ≤ u_max
         Δu_min ≤ ΔU ≤ Δu_max
```

Condensed prediction: Y = F x + G_u u_prev + Φ ΔU. Solved via `GradientProjectionQP` (shared backend with GPC and MHE).

**Corrector required?** Output-feedback MPC requires a state estimator. The certainty-equivalence pattern — `mpc.setState(kf.state())` — is demonstrated in `ex50_ekf_mpc.cpp` (EKF+MPC) and `ex53_mhe_mpc_dual.cpp` (MHE+MPC).

**Prefers over LQR when:** Hard input/state constraints must be satisfied, or the prediction horizon matters (unstable plants, time-varying references).

---

### A4. Intelligent Control — Fuzzy Logic, Repetitive, Learning-based

**Core idea.** Use human-interpretable rules, memory of prior periods, or offline-trained policies to handle cases where analytic models are unavailable or unreliable.

#### 4.1 Fuzzy Logic

Mamdani: rule firing strength `αᵢ = min(μ_A(e), μ_B(ė))` → aggregated output clipped MF → CoG defuzzification.
Takagi-Sugeno: weighted average of linear consequents.

**Toolbox:** `FuzzySystem`, `FuzzyPD`, `FuzzyPID`, `FuzzySupervisor` — all in `lib/FuzzyLogic.h`.

#### 4.2 Repetitive Control

Internal Model Principle: to reject a periodic disturbance of period T, include a model of that signal in the open-loop transfer function. RC adds a delay-line memory:

```
u_rc[k] = Q(z) · u_rc[k - N_p] + L(z) · e[k]
```

where N_p = T/Ts (period in samples), Q(z) is a low-pass robustness filter, and L(z) is a phase-lead compensator. Converges in O(1/L_gain) periods.

**Toolbox:** `RepetitiveController` in `lib/RepetitiveController.h`. Demonstrated paired with LeadLag in `ex46_leadlag_inner_repetitive_outer.cpp`.

**Self-contained?** RC is a corrector by nature — it adds to a stabilising primary controller's output (additive stack). The primary controller must first stabilise the plant.

**Prefers over PID when:** The disturbance or reference is periodic (e.g., motor ripple, power-line frequency, conveyor belt profile). PID cannot achieve zero steady-state error for sinusoidal inputs.

---

### A5. Event-Based Control — Send-on-Delta, Self-Triggered

**Core idea.** Reduce computation and communication by triggering updates only when a condition is met, rather than periodically.

**Send-on-delta trigger:**
```
trigger[k] = 1  iff  ‖x[k] - x[last]‖ > δ
```

**Self-triggered schedule:** next update at k* = argmin{k > k_last : V(x[k]) ≥ γ · V(x[k_last])} for a Lyapunov function V.

**Toolbox mapping:** Not yet implemented. Could be a thin wrapper around any `IController`:

```cpp
class EventTriggeredController : public IController {
    double sigma_;                        // deadband threshold [engineering units]
    double y_last_;
    std::shared_ptr<IController> inner_;
public:
    double compute(double e) override {
        if (std::abs(e - (y_last_ - /* ref */)) > sigma_)
            return inner_->compute(e);
        return inner_->lastOutput();      // zero-order hold
    }
};
```

**Status:** [FUTURE]. No existing example; closest is `AtomicParamBuffer` for non-RT parameter updates.

**Self-contained?** Yes — wraps any controller without requiring an observer.

**Prefers over periodic when:** Communication bandwidth is scarce (networked control, IoT sensors), or computation is expensive and must be budgeted.

---

### A6. Nonlinear Control — SMC, Backstepping, Feedback Linearisation

**Core idea.** Exploit known nonlinear structure to guarantee global or large-region stability properties unavailable to linear designs.

#### 6.1 Sliding Mode Control (Implemented)

Sliding surface: `s[k] = c_e · e[k] + c_de · (e[k] - e[k-1])`

Control: `u[k] = -K · sat(s[k] / φ)` — saturation replaces discontinuous sign to limit chattering.

`c_de` absorbs Ts: to match continuous-time slope λ [rad/s], set `c_de = λ · Ts`.

**Toolbox:** `DiscreteSMC` in `lib/DiscreteSMC.h`. See [Deployment Guide Section 1](DEPLOYMENT.md#discretesmc) for `c_de` sizing constraint.

**Self-contained?** Yes — but requires full state (or at least position and velocity). With an estimator: `DiscreteSMC` fed by `UnscentedKalmanFilter` (see `ex51_ukf_smc.cpp`).

#### 6.2 Backstepping and Feedback Linearisation

Not yet implemented — see [Proposed Extension E4](#e4-feedbacklinearisationh). Both require:
- Full nonlinear state equations `ẋ = f(x) + g(x)u` in symbolic or functional form.
- For FL: relative degree r = n and minimum-phase zeros.
- For backstepping: strict-feedback (triangular) structure.

**Prefers over SMC when:** The plant has well-known nonlinear dynamics and smooth, noise-free state measurements. SMC is preferable when disturbances are the primary concern.

---

### A7. Hybrid / Switched Control — Supervisory, Hybrid Automata

**Core idea.** A finite-state machine selects or blends controllers based on discrete logic, enabling mode changes that a single continuous controller cannot handle.

**Formal model:**
```
q[k+1] = δ(q[k], x[k], r[k])        (discrete state transition)
u[k]   = f_{q[k]}(x[k], r[k])       (mode-specific continuous control)
```

Stability condition (Lyapunov): there exists a common Lyapunov function V(x) such that V decreases along trajectories of each mode, or dwell-time condition T_d ≥ τ_d is satisfied.

**Toolbox mapping:** `ControllerStack::Supervisory` implements the mode-selection logic. Condition callbacks `[](double e, double last_out) → bool` encode the transition guard.

**Bumpless transfer** is the key practical concern: when switching from controller i to j, initialise j's integrator state to match i's last output (see `ex54_bumpless_transfer.cpp`).

**Self-contained?** Yes — the supervisory layer coordinates existing `IController` instances.

**Prefers over a single controller when:** The plant operates in qualitatively different modes (e.g., heating vs. cooling, saturation vs. linear regime, fault mode) that each demand different control strategies.

---

## 2. Plant Model Function Interference (B1–B8)

"Interfering with the plant model function" means modifying the effective dynamics seen by the controller — without changing the physical plant. Ordered from shallowest to deepest interference.

---

### B1. Model Reduction — Balanced Truncation, Hankel Approximation

**What it does.** Replaces a high-order state-space model (n = 50–1000, e.g., FEM modal model) with a low-order approximation (n = 2–8) that preserves the most input-output energetically significant modes.

**Algorithm (balanced truncation):**

1. Solve the discrete Lyapunov equations for controllability gramian P_c and observability gramian P_o:
   ```
   A P_c Aᵀ + B Bᵀ = P_c
   Aᵀ P_o A + Cᵀ C = P_o
   ```
2. Compute balanced realisation: `T = P_c^{1/2} U Σ^{-1/2}` where `P_c^{1/2} P_o P_c^{1/2} = U Σ² Uᵀ`.
3. Truncate states with Hankel singular values `σᵢ < ε_tol`.

**Error bound (H∞):** `‖G - G_r‖∞ ≤ 2 Σᵢ₌ᵣ₊₁ⁿ σᵢ` — provides a-priori guarantee on approximation quality.

**Simplifies controller design:** Reduces a 50-state flexible structure to a 4-state rigid-body + first-flex-mode model → enables PID or LQR on the reduced plant, with the truncation error treated as H∞ uncertainty in a robust design.

**C++ / Eigen implementation sketch:**
```cpp
// Solve Lyapunov: A*Pc*A' + B*B' = Pc  (discrete)
Eigen::MatrixXd solveDLyapunov(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q);
// Then:  svd(sqrtm(Pc) * Po * sqrtm(Pc))  ->  Hankel singular values
```

**Status:** [FUTURE] — `lib/BalancedTruncation.h` proposed in [Section 5](#5-proposed-toolbox-extensions).

**Prefers over using the full model when:** The high-order model is required for physical fidelity (FEM, CFD-linearised) but real-time QP (MPC) would be infeasible at the full order.

---

### B2. Linearisation — Jacobian and Feedback Linearisation

**What it does.** Approximates the nonlinear plant `ẋ = f(x,u)` by its Jacobian at an operating point (x₀, u₀):

```
A = ∂f/∂x |_{x₀,u₀},   B = ∂f/∂u |_{x₀,u₀}
```

enabling design of LQR, MPC, or H∞ on the linear model.

**Simplifies:** Reduces nonlinear design to linear tools. Multiple operating points → gain scheduling (A1.2) or adaptive (A1.1) re-linearisation.

**C++ implementation:** The EKF already computes Jacobians numerically at each step (scaled-epsilon finite differences). The same `numericalJacobian()` utility can be used standalone for offline linearisation.

**Toolbox mapping:**
- `FuzzySupervisor::update()` returns `SupervisorDecision::relinearize == true` → caller calls `mpc.setPlant(reLinearise(x))`. Pattern in `ex25_fuzzy_supervisor_mpc.cpp`.
- Jacobian utility: `ExtendedKalmanFilter::numericalJacobian()` (private, but extractable).

**Status:** [EXISTS partial] — re-linearisation is supported via `setPlant()` on MPC and GPC. A standalone `LinearisationHelper.h` is proposed in [Section 5](#5-proposed-toolbox-extensions).

---

### B3. Integrator Backstepping — Recursive Virtual Control

**What it does.** For a system in strict-feedback (triangular) form:

```
ẋ₁ = f₁(x₁) + g₁(x₁) x₂
ẋ₂ = f₂(x₁,x₂) + g₂(x₁,x₂) x₃
⋮
ẋₙ = fₙ(x) + gₙ(x) u
```

Design Lyapunov functions `V₁, V₂, ..., Vₙ` recursively. At each step, the next state `xᵢ₊₁` acts as a virtual control for the `xᵢ` subsystem. The final control law `u` drives the last virtual tracking error to zero.

**Simplifies:** Provides a systematic, stability-guaranteed design for cascaded nonlinear systems (robot links, flight control) without requiring full feedback linearisation (no need for relative degree = n).

**Feasibility condition:** Each `gᵢ(x) ≠ 0` for all reachable x (non-singular virtual actuators).

**C++ / Eigen sketch:**
```cpp
// Step 1: alpha1(x1) = -(c1*e1 + f1(x1)) / g1(x1)    [virtual control for x2]
// Step 2: e2 = x2 - alpha1; alpha2(x1,x2) = -(c2*e2 + ...)
// Step n: u = (alpha_n - fn(x) - (...)) / gn(x)
```

**Status:** [FUTURE] — no current implementation. Proposed as `lib/BacksteppingController.h` in Section 5.

---

### B4. Passivity-Based Control — Energy Shaping and Damping Injection

**What it does.** Reshapes the plant's stored energy function H(x) → H_d(x) so the minimum of H_d coincides with the desired equilibrium x*. Then injects virtual damping to guarantee asymptotic stability:

```
u = u_ES + u_DI = g⊥(x) − kd ẋ_output
```

where g⊥ is the interconnection matrix for energy shaping and kd > 0 is the damping gain.

**Intended effect:** Inherent robustness to parameter variation (energy-based stability). Particularly effective for underactuated mechanical systems (pendulums, quadrotors, robotic arms).

**Feasibility:** Requires a Hamiltonian or port-Hamiltonian system structure and the ability to solve a PDE for the energy shaping terms (matching conditions).

**C++ / Eigen:** Requires symbolic manipulation or offline solution of the PDE; online component is a function evaluation + damping injection.

**Status:** [FUTURE] — not implemented. Requires a port-Hamiltonian system description interface not currently in the toolbox.

---

### B5. Disturbance Observer (DOB) and Extended State Observer (ESO)

**What it does.** Estimates the lumped disturbance d acting on the plant output (DOB) or the total disturbance including model uncertainty as an extended state (ESO), and cancels it in feedforward.

**DOB Q-filter approach:**

```
d̂(z) = Q(z) · [y(z) - G_nom(z) · u(z)]
u(z)  = u_primary(z) - d̂(z)
```

Q(z) is a low-pass filter (bandwidth ω_Q below plant bandwidth). The effective plant seen by the primary controller is G_nom(z) for all disturbances below ω_Q.

**ADRC ESO (3-state for 2nd-order plant):**

```
[ẑ₁]   [1-β₁Ts    Ts    Ts·Ts/2] [ẑ₁]   [β₁Ts] [  0  ]
[ẑ₂] = [  -β₂Ts   1      Ts   ] [ẑ₂] + [β₂Ts] [b₀·Ts] u
[ẑ₃]   [  -β₃Ts   0      1   ] [ẑ₃]   [β₃Ts] [  0  ]

u = (r_dd - ẑ₂·ω_c² - ẑ₃) / b₀   [PD + total-disturbance cancellation]
```

**Simplifies:** The primary controller is designed for the nominal model G_nom only. All disturbances, model errors, and nonlinearities below ω_Q are handled transparently.

**Toolbox mapping:**

| Component | Class | File |
|-----------|-------|------|
| ADRC (ESO built in) | `DiscreteADRC` | `lib/DiscreteADRC.h` |
| DOB pattern (manual) | — | `ex52_dob_pi.cpp` |

**Status:** [EXISTS] for both ESO and DOB pattern.

**Prefers over an observer+feedback correction when:** The disturbance structure is unknown, but its magnitude is bounded. ESO requires only a rough plant model (integrator + delay suffices for many process plants).

---

### B6. Model Predictive Control — Model-Heavy Design

MPC is not strictly a "model interference" technique, but it is the most model-dependent control strategy in the toolbox. Its condensed prediction matrices F, G_u, Φ are rebuilt from the plant model at construction or on `setPlant()` call.

**Key design trade-off:** Unlike PID (model-free), MPC explicitly uses the model to predict future outputs. Model mismatch → prediction error → suboptimal or unsafe control. Adaptive MPC (RLS + `setPlant()`) closes this loop.

**Toolbox:** `DiscreteMPC`, `GeneralizedPredictiveController`. See `lib/DiscreteMPC.h` and DEPLOYMENT.md Section 3 (MPC infeasibility/LDLT failure).

---

### B7. Inversion-Based Feedforward — Stable Plant Inverse

**What it does.** For a minimum-phase plant G(z) (all zeros strictly inside the unit circle), compute the stable inverse G⁻¹(z) and apply it to the reference signal:

```
u_ff(z) = G⁻¹(z) · r(z)   →   y(z) ≈ r(z) after the plant delay
```

The feedback controller only needs to reject disturbances and compensate model error — not drive tracking.

**Feasibility condition:** All zeros of G(z) must satisfy |zᵢ| < 1 (minimum phase). For non-minimum-phase plants, the inverse is unstable — a partial (causal) inversion or zero-phase preview filter is needed instead.

**Simplifies:** Feedback controller bandwidth can be reduced (robustness), since the inversion handles the nominal reference tracking.

**C++ sketch:**
```cpp
// G(z) = B(z) / A(z), B minimum phase
// G_inv(z) = A(z) / B(z)  ->  difference equation for u_ff given r
// Implement as a TransferFunction with num=A coeffs, den=B coeffs
ctrl::TransferFunction G_inv(A_coeffs, B_coeffs, Ts);
// Use ssStep(ctrl::tf2ss(G_inv), x_inv, r_vec) each sample
```

**Toolbox mapping:**
- `FeedforwardController` with static gain covers the DC case.
- Dynamic inversion is not yet wrapped as a dedicated class.

**Status:** [EXISTS partial] — static feedforward in `FeedforwardController`. Dynamic inversion [PLANNED] as a `StateSpace`-based wrapper.

---

### B8. Constraint Enforcement — Projection and Anti-Windup

**What it does.** Projects the unconstrained control signal onto the feasible set, or uses back-calculation to de-wind the integrator when the output saturates.

**Back-calculation anti-windup (PID):**

```
I[k] = I[k-1] + Ki·Ts·e[k] + Kb·(u_sat[k] - u_unsat[k])
```

`Kb > 0` drives the integrator state back toward the limit boundary, preventing windup without disabling integral action.

**Gradient projection (MPC/GPC/MHE):**

```
ΔU[n+1] = Π_{𝒞}(ΔU[n] - α · ∇J(ΔU[n]))
```

where `Π_{𝒞}` projects onto the box constraint set 𝒞.

**Toolbox mapping:**

| Constraint type | Component | File |
|-----------------|-----------|------|
| PID integrator windup | `PIDParams::Kb`, `DiscretePID` | `lib/DiscretePID.h` |
| MPC input/move bounds | `GradientProjectionQP` | `lib/GradientProjectionQP.h` |
| MHE process-noise bounds | `MHEParams::wMin/wMax` | `lib/MovingHorizonEstimator.h` |

**Status:** [EXISTS] — anti-windup and QP projection are fully implemented.

**Prefers over soft constraints when:** Actuator limits are physical (valve fully open/closed, motor at max torque) and violation would damage hardware or endanger safety.

---

## 3. Deep Behaviour Interference (C1–C6)

These techniques force the plant to behave in a prescribed way that strongly deviates from its natural dynamics. Each carries specific feasibility conditions and real-world risks.

---

### C1. Sliding Mode Control — First-Order and Super-Twisting

**Intended effect.** Drive the state to a sliding surface s = 0 in finite time, then keep it there despite bounded matched disturbances:

```
s[k] = c_e · e[k] + c_de · (e[k] - e[k-1])   [first-order SMC]
u[k] = -K · sat(s[k] / φ)
```

On the surface, the dynamics are `ṡ = 0 → reduced-order stable system` — independent of matched disturbances d with |d| ≤ D < K.

Super-Twisting (second-order SMC) eliminates the sign discontinuity entirely by differentiating u:

```
u̇ = -α · sign(s)
u  = -λ√|s| · sign(s) + ∫ u̇
```

→ higher-order sliding reduces chattering without a boundary layer.

**Feasibility condition:** Sign-controllability: g(x) (input gain) must be non-zero and same sign as desired. Matching condition: disturbances enter through the control channel.

**Real-world risks:**

| Risk | Mechanism | Mitigation |
|------|-----------|------------|
| Chattering | High-frequency switching at sensor noise level | Saturation boundary layer (φ > 0); super-twisting |
| Actuator wear | Rapid sign reversals in u | Increase φ; filter u through low-pass |
| Noise amplification | c_de multiplies the difference e[k]-e[k-1] (= numerical derivative) | Low-pass filter e before SMC; reduce c_de |
| Surface non-invariance | Discretisation of sign function allows limit cycles | Use `c_de = λ·Ts` correctly (see DEPLOYMENT.md) |

**Wise use:** Robust position servo with bounded Coulomb friction and load variation — SMC guarantees invariance on the surface regardless of friction magnitude.

**Do NOT use:** High-noise sensor environments (accelerometers without filtering), soft/pneumatic actuators that cannot switch rapidly, non-minimum-phase plants where the zero dynamics are unstable (forcing s=0 destabilises the internal dynamics).

**Toolbox:** `DiscreteSMC` in `lib/DiscreteSMC.h`. Parameter pitfall: `SMCParams::c_de` must absorb Ts (`c_de = λ·Ts`), not the continuous-time slope λ directly. See `test_catch2_advanced.cpp [smc]`.

---

### C2. Exact Feedback Linearisation — Full Nonlinear Cancellation

**Intended effect.** Cancel all nonlinear terms and reduce the closed-loop to a chain of integrators, on which a simple linear controller (e.g., pole placement) is applied:

```
u = (v - f(x)) / g(x)   →   ẋ = v   (linear double-integrator)
```

The plant's natural response is completely replaced by the designer's chosen linear dynamics.

**Feasibility conditions:**

1. **Minimum phase:** All zero dynamics (internal dynamics when y = reference) must be asymptotically stable. Unstable zeros cause the internal states to grow unboundedly when the output is forced.
2. **Relative degree = n:** The output must be differentiated exactly n times before u appears (full relative degree = full-state linearisation).
3. **Full state measurement:** f(x) and g(x) must be computable from measured (or estimated) state x.
4. **g(x) ≠ 0** for all reachable x (no input singularity).

**Real-world risks:**

| Risk | Mechanism | Mitigation |
|------|-----------|------------|
| Model error → instability | Perfect cancellation of f(x) depends on exact model; residual error enters as unmatched disturbance | Robust FL: add SMC or H∞ outer loop |
| Algebraic singularity | g(x) → 0 at some states (e.g., inverted pendulum at vertical) | Restrict operating region; use backstepping instead |
| Actuator saturation | The algebraic inversion u = (v - f(x))/g(x) can be large | Add saturation + integrate anti-windup back in |
| Noise on x | f(x) often contains high-order terms amplified by numerical differentiation | Pre-filter states; use EKF/UKF |

**Wise use:** Robot joint control with well-calibrated dynamics, slow enough that numerical inversion is stable, sensor noise filtered via EKF.

**Do NOT use:** Systems with uncertain zeros near the unit circle, gas turbines or chemical reactors where model error is structural, or any plant with non-minimum-phase zeros.

**Toolbox status:** [FUTURE] — requires a user-supplied `f(x)` and `g(x)` functor. Proposed as `lib/FeedbackLinearisation.h` in Section 5.

---

### C3. MRAC — Force Closed-Loop to Track a Reference Model

**Intended effect.** The output y(t) converges to the output y_m(t) of a reference model M(s) driven by the same reference r(t), despite plant parameter uncertainty:

```
e_m = y - y_m  →  0   as t → ∞
```

The adaptation law (MIT rule / Lyapunov gradient):

```
θ̇(t) = -γ · e_m(t) · ∂y/∂θ      (MIT rule)
θ̇(t) = -Γ · e_m · Pᵀ Bₘ ω       (Lyapunov-based, guaranteed stable)
```

**Feasibility conditions:**

1. Reference model M(s) must be stable and of the same relative degree as the plant.
2. Plant must be minimum phase (unknown zeros must all be stable).
3. **Persistent excitation** of the reference r(t) — without PE, parameter convergence is not guaranteed even if tracking error converges. Consequence: the parameter estimates θ may stagnate away from true values, creating a dormant instability.
4. High-gain adaptation (large Γ) → fast tracking but risk of high-frequency oscillation.

**Real-world risks:**

| Risk | Mechanism | Mitigation |
|------|-----------|------------|
| Bursting / transient instability | Slow adaptation + unmodelled dynamics → temporary divergence | Add a deadzone ‖e_m‖ < δ; use σ-modification |
| PE requirement | Constant reference → zero gradient → parameter drift | Inject small PRBS dither on reference |
| Control saturation | Adaptation may demand large u temporarily | Bound θ; add projection operator to keep θ in known feasible set |

**Wise use:** Aircraft autopilot with slowly changing mass/CG due to fuel consumption — reference model M(s) defines the desired short-period response; adaptation tracks the changing plant.

**Do NOT use:** Plants with unstable zeros, rapidly changing plants (faster than the adaptation rate Γ), or when exact y_m tracking carries safety-critical timing constraints that cannot tolerate transient bursting.

**Toolbox status:** [FUTURE] — `RecursiveLeastSquares` provides the parameter update infrastructure; the Lyapunov/MIT gradient law is not yet wrapped. Proposed as `lib/MRACController.h` in Section 5.

---

### C4. Extremum Seeking — Inject Oscillation to Find Optimum

**Intended effect.** Drive the plant to the operating point that minimises (or maximises) an unknown static cost function J(y) by injecting a persistent sinusoidal dither:

```
u(t) = θ(t) + a · sin(ωₛ t)         [dithered input]
J_filtered = LPF{J(y(t))}
∇̂J = HPF{J(y(t))} · sin(ωₛ t)       [demodulated gradient]
θ̇ = -k · ∇̂J                         [integrator]
```

The ESC asymptotically converges to a neighbourhood of size O(a²) around the true extremum — the dither cannot be removed without losing gradient information.

**Feasibility conditions:**

1. J(θ) has a unique local minimum (or maximum) in the operating region.
2. Plant dynamics G(s) are much faster than the dither frequency ωₛ (quasi-static approximation). Rule of thumb: ωₛ < ω_bandwidth / 10.
3. The cost J(y) is smooth (differentiable) — discontinuous cost breaks demodulation.

**Real-world risks:**

| Risk | Mechanism | Mitigation |
|------|-----------|------------|
| Persistent oscillation at optimum | Dither is never removed — it is the sensing mechanism | Set a ≪ operating range; accept O(a²) neighbourhood |
| Slow convergence | Integrator gain k small for stability, but this slows learning | Tune ωₛ and k jointly: ωₛ ≈ 10·k for separability |
| Multiple local optima | Gradient demodulation points to nearest local minimum | Initialise near global optimum; use large a initially then ramp down |
| Interaction with closed-loop poles | If ωₛ is near a closed-loop resonance, the loop can amplify dither | Check open-loop frequency response at ωₛ |

**Wise use:** Maximum Power Point Tracking (MPPT) in photovoltaic arrays — the I-V curve has a smooth, unique maximum; plant dynamics are much faster than MPPT update rate.

**Do NOT use:** Systems where continuous oscillation is unacceptable (precision machining, medical devices), or when the cost landscape has multiple local minima and the initial condition is unknown.

**Toolbox:** `ExtremumSeeker` in `lib/ExtremumSeeker.h`. `seekMinimum` flag toggles minimisation vs. maximisation. Demonstrated in `ex04_esc_minimum.cpp`, `ex15_esc_moving_minimum.cpp`, `ex47_esc_additive_pid.cpp`.

---

### C5. Hard Constraint Enforcement — MPC with Mandatory Feasibility

**Intended effect.** Guarantee that the control input u[k] and state x[k] satisfy hard physical constraints at every time step, even at the cost of tracking performance:

```
u_min ≤ u[k] ≤ u_max      (actuator limits)
Δu_min ≤ Δu[k] ≤ Δu_max  (rate of change limits)
x_min ≤ x[k+i] ≤ x_max   (predicted state constraints, i=1..Np)
```

When constraints cannot be simultaneously satisfied with the objective, the QP returns infeasible → requires a fallback.

**Feasibility conditions:**

1. The initial state x[0] must be inside the feasible set (constraint qualification).
2. For recursive feasibility (guarantee that x[k] feasible → x[k+1] feasible), terminal constraints or a terminal invariant set are required.
3. `rho_u > 0` is mandatory — otherwise the Hessian is singular and the LDLT fails.

**Real-world risks:**

| Risk | Mechanism | Mitigation |
|------|-----------|------------|
| QP infeasibility | Operating point outside feasible set; horizon too long | Soft-constraint penalty on state bounds; reduce Np |
| Constraint chatter | Plant near constraint boundary → frequent small corrections | Increase `rho_u`; add move suppression |
| LDLT failure | `rho_u` too small → ill-conditioned Hessian | Floor: `rho_u ≥ 1e-6 * rho_y * max_eig(Φᵀ Φ)` |
| Horizon too short | Constraint violated at k+Np+1 (outside horizon) | Set Np ≥ 2·settling_time/Ts |

**Wise use:** Battery charge/discharge management — voltage and current must stay within manufacturer limits at all times; MPC with hard bounds provides formal guarantees.

**Do NOT use:** When constraint limits are engineering preferences (soft), not physical hard limits — soft constraints via quadratic penalty (slack variables) are more numerically robust and avoid infeasibility.

**Toolbox:** `DiscreteMPC` with `uMin/uMax` and `duMin/duMax`; solved via `GradientProjectionQP`. LDLT failure handling documented in `docs/DEPLOYMENT.md` Section 3.

---

### C6. Perfect Inversion Feedforward — Zero Tracking Error

**Intended effect.** Pre-filter the reference r[k] through the stable inverse G⁻¹(z) so the output y[k] tracks r[k] with zero steady-state error and minimal transient error, without requiring the feedback controller to drive the tracking:

```
u_ff[k] = G⁻¹(z) · r[k]   →   y[k] = G(z) · u_ff[k] = r[k]   (exact, no delay)
```

The feedback controller u_fb handles only disturbances and model error → can be designed for robustness with lower bandwidth.

**Feasibility condition:** All zeros of G(z) must satisfy |zᵢ| < 1 (minimum phase). A non-minimum-phase zero at z₀ with |z₀| > 1 creates an unstable pole in G⁻¹(z) → the feedforward diverges for any non-trivial reference.

**For non-minimum-phase plants:** Use zero-phase error tracking (ZPETC):

```
G_ff(z) = B⁺(z) · B⁻(1/z) / (B⁻(1/z) B⁻(z) |_{z=1})
```

where B⁺ contains minimum-phase zeros and B⁻ contains non-minimum-phase zeros. This achieves zero-phase error at the cost of amplitude error (the gain is not exactly 1 away from DC).

**Real-world risks:**

| Risk | Mechanism | Mitigation |
|------|-----------|------------|
| Zeros near unit circle | G⁻¹ has poles near unit circle → near-resonance amplification | Regularise: `G_ff = G⁻¹ · (1 + ε·G⁻¹)⁻¹` |
| Noise amplification | G⁻¹ is high-pass for typical low-pass plants | Pre-filter r[k] with a reference governor |
| Non-causal preview needed | Zero-phase inversion requires future r[k+N] | Use preview MPC or record reference trajectory offline |

**Wise use:** CNC machine-tool axis control — reference trajectory is known in advance; minimum-phase plant model; feedback keeps errors small during disturbances; inversion provides high-precision reference tracking.

**Do NOT use:** Closed-loop systems where the reference is not known in advance (reactive systems), non-minimum-phase plants (flexible structures with non-collocated sensors/actuators), or plants with zeros that drift due to temperature variation.

**Toolbox status:** [EXISTS partial] — static feedforward in `FeedforwardController`. Dynamic inversion via `StateSpace` wrapper is [PLANNED].

---

## 4. Decision Framework

### 4.1 Primary Decision Table

Use the following table to select the level of interference. Start at the top and stop at the first row that matches the plant/design situation.

| Condition | Strategy | Toolbox path | Typical risk |
|-----------|----------|--------------|--------------|
| Well-identified SISO plant, stable, little noise | PID + lead-lag | `DiscretePID` + `DiscreteLeadLag` | Overshoot, integral windup |
| Periodic disturbance or reference | Add Repetitive Control corrector | `RepetitiveController` (additive stack) | Period estimation error |
| Unknown bounded disturbance | Add ESO / DOB corrector | `DiscreteADRC` or ex52 DOB pattern | ESO bandwidth vs. noise trade-off |
| Measurable disturbance | Add feedforward corrector | `FeedforwardController` (additive) | Model error in feedforward path |
| Output-feedback LQR needed | Add Kalman filter corrector | `KalmanFilter` → `DiscreteLQR::compute(x_hat)` | KF divergence on model mismatch |
| Nonlinear plant (smooth, well-known) | EKF/UKF + observer+SF | `ExtendedKalmanFilter` + `DiscreteSMC` | Jacobian error, noise amplification |
| Constrained MIMO plant | MPC with box constraints | `DiscreteMPC` | QP infeasibility, Hessian conditioning |
| Time-varying or uncertain gain (≤30% drift) | Adaptive GPC | `RecursiveLeastSquares` → `GPC::setPlant()` | RLS forgetting factor choice |
| Structured parametric uncertainty (quantified) | H∞ / mu-synthesis | `DiscreteHinf::solveMuSyn()` | Conservative (γ larger than needed) |
| Plant operates in qualitatively different modes | Hybrid supervisory | `ControllerStack::Supervisory` | Bumpless transfer, dwell-time stability |
| Must guarantee zero steady-state error for sinusoid | Repetitive or RC + IMP | `RepetitiveController` | Requires stable primary controller |
| Plant gain drift is large or structural | MRAC | [FUTURE] `MRACController` | Persistent excitation, bursting |
| Unknown static cost optimum | Extremum Seeking | `ExtremumSeeker` | Persistent dither, local minima |
| Strong nonlinearity, robust disturbance rejection | SMC | `DiscreteSMC` | Chattering, sensor noise |
| High-order model from FEM/CFD | Balanced truncation first | [FUTURE] `BalancedTruncation` | Truncation error vs. controller order |

---

### 4.2 Depth-of-Interference Flowchart

```
Start: Can a well-tuned PID achieve the specs?
│
├─ YES → Use PID (DiscretePID). Possibly add Lead-Lag for phase margin.
│        Stop — minimal interference.
│
└─ NO: What is the primary obstacle?
       │
       ├─ Periodic disturbance or reference
       │   └─ Add RepetitiveController corrector (additive stack).
       │       [Shallow interference — IMP added, plant untouched]
       │
       ├─ Unknown disturbance / model uncertainty  
       │   ├─ If disturbance is measurable → FeedforwardController
       │   ├─ If disturbance is unknown, plant ≈ integrator → DiscreteADRC (ESO)
       │   └─ If disturbance is unknown, full model available → DOB (ex52 pattern)
       │       [Moderate interference — effective plant model seen by controller changes]
       │
       ├─ Constraints on u or x are binding
       │   └─ DiscreteMPC with hard bounds + GradientProjectionQP
       │       [Moderate-to-deep — controller overrides plant trajectory]
       │
       ├─ Plant parameters vary significantly over time
       │   ├─ Slow drift (minutes) → RLS + GPC::setPlant() (adaptive)
       │   ├─ Fast switching (seconds) → Gain scheduling (ControllerStack::Weighted)
       │   └─ Model reference required → [FUTURE] MRACController
       │       [Deep — controller adapts its own structure/gains]
       │
       ├─ Structured parametric uncertainty (model set known)
       │   └─ H∞ (DiscreteHinf::solve) or mu (solveMuSyn)
       │       [Deep — worst-case performance guaranteed across model set]
       │
       ├─ Strong nonlinearity, bounded disturbances
       │   ├─ Known structure, smooth model → Feedback Linearisation [FUTURE]
       │   ├─ Known structure, disturbance-dominated → SMC (DiscreteSMC)
       │   └─ Unknown structure → Fuzzy+SMC or UKF+SMC (ex51, ex69)
       │       [Deepest — natural plant dynamics partially or fully cancelled]
       │
       └─ Unknown static optimum (extremum)
           └─ ExtremumSeeker (ESC)
               [Deepest — inject persistent oscillation to probe cost surface]
```

---

### 4.3 Wise vs. Unwise Summary

| Technique | Wise when | Unwise when |
|-----------|-----------|-------------|
| SMC | Bounded, matched disturbances; actuator can switch fast | Noisy sensors; soft actuators; non-minimum-phase internal dynamics |
| Feedback Linearisation | Accurate model; minimum phase; full state | Model error > 5%; unstable zeros; actuator saturation |
| MRAC | Slowly varying parameters; persistent reference excitation | Rapidly changing plant; unstable zeros; no persistent excitation |
| ESC | Smooth, unimodal cost; plant faster than dither | Multiple local optima; cost is discontinuous; oscillation intolerable |
| Hard MPC constraints | Physical actuator limits; safety-critical bounds | Soft engineering preferences; rapidly varying feasible set |
| Balanced truncation | High-order FEM model; design with low-order methods needed | Plant order already low (≤6); truncated modes are near crossover frequency |

---

## 5. Proposed Toolbox Extensions

Five new features are proposed, prioritised by: (a) existence of a reference implementation in `reference/pdc-master/` or current Eigen infrastructure, and (b) expected return on test-suite investment.

---

### E1. `lib/BalancedTruncation.h` — Model Order Reduction

**Motivation.** High-order FEM or multi-body linearisations cannot directly feed MPC or LQR in real time. Balanced truncation with an a-priori error bound is the standard industrial method.

**API sketch:**

```cpp
namespace ctrl {

struct TruncationResult {
    StateSpace  reduced;              // order-r approximation
    VectorXd    hankelSingularValues; // all n values
    double      errorBound;           // 2 * sum(sigma_{r+1}..sigma_n) [H∞ norm]
};

TruncationResult balancedTruncate(const StateSpace& full, int r);
// r: desired reduced order
// Internally: solveGramians() -> balance() -> truncate()

int suggestOrder(const TruncationResult& result, double tol = 0.01);
// Returns smallest r such that errorBound < tol * DC gain
}
```

**Implementation notes:**
- `solveGramians()`: discrete Lyapunov solve via eigendecomposition (Eigen's `SelfAdjointEigenSolver`).
- Balanced transformation: Cholesky of P_c, SVD of `chol(Pc)ᵀ · Po · chol(Pc)`.
- All operations are O(n³) offline — not in the control loop.

**Test idea:**

```cpp
// [balanced_truncation] tag
// Plant: 6th-order oscillatory system (3 conjugate pole pairs at 1, 2, 5 rad/s)
// Truncate to order 2 (keep dominant poles at 1 rad/s)
// REQUIRE: DC gain of reduced model within 1% of full model
// REQUIRE: errorBound < 2 * sigma_3
// REQUIRE: DiscreteLQR on reduced model closes loop on full model to within 0.05 tolerance
```

**Example:** `ex55_balanced_truncation.cpp` (C++), `examples/python/ex71_balanced_truncation.py`.

---

### E2. `lib/LinearisationHelper.h` — Jacobian and Re-Linearisation Utilities

**Motivation.** The EKF already computes numerical Jacobians internally (scaled-epsilon finite differences). Making this utility public and pairing it with a `reLinearise()` helper removes boilerplate from every adaptive MPC example.

**API sketch:**

```cpp
namespace ctrl {

// Compute Jacobian of f: R^n x R^m -> R^n at (x0, u0)
// Uses scaled epsilon: eps_i = max(1e-5 * |x0_i|, 1e-8)
MatrixXd numericalJacobian(
    const std::function<VectorXd(VectorXd, VectorXd)>& f,
    const VectorXd& x0, const VectorXd& u0);

// Build discrete StateSpace by Jacobian linearisation + ZOH
StateSpace lineariseAtPoint(
    const std::function<VectorXd(VectorXd, VectorXd)>& f_continuous,
    const std::function<VectorXd(VectorXd, VectorXd)>& h,
    const VectorXd& x0, const VectorXd& u0, double Ts);
}
```

**Implementation notes:**
- Reuse the same scaled-epsilon logic already in `ExtendedKalmanFilter` (internal `numericalJacobian()`).
- `lineariseAtPoint()` calls `numericalJacobian()` for A and B, then `c2d()` for discretisation.

**Test idea:**

```cpp
// [linearisation] tag
// Nonlinear plant: Van der Pol oscillator (known A at equilibrium)
// REQUIRE: Jacobian A matches analytical A within 1e-5 relative error
// REQUIRE: c2d of linearised model matches ZOH reference from python-control to 1e-6
```

**Example:** Extend `ex30_ekf_nonlinear.cpp` with a side-by-side linearisation validation.

---

### E3. `lib/MRACController.h` — Model Reference Adaptive Control

**Motivation.** MRAC is the most widely taught adaptive control method but is absent from the toolbox. The Lyapunov-based version (with σ-modification for robustness) is implementable with the existing Eigen infrastructure.

**API sketch:**

```cpp
namespace ctrl {

struct MRACParams {
    StateSpace referenceModel;  // M(s): desired closed-loop model
    MatrixXd   Gamma;           // adaptation rate matrix (n x n, PD)
    double     sigma;           // σ-modification gain (0 = off, ~0.01 recommended)
    double     thetaMax;        // parameter projection bound
    double     uMin, uMax;
};

class MRACController : public IController {
public:
    MRACController(const StateSpace& plant_nominal,
                   const MRACParams& params, double Ts);
    double compute(double error) override;  // error = r - y
    void   reset() override;
    VectorXd parameters() const;            // current θ̂
    double   modelError() const;            // current e_m = y - y_m
private:
    StateSpace  referenceModel_;
    VectorXd    theta_;         // adaptive parameters
    VectorXd    xm_;            // reference model state
    double      em_prev_;
};
}
```

**Implementation notes:**
- Lyapunov adaptation: `theta_[k] = theta_[k-1] + Ts * (-Gamma * em * phi - sigma * theta_)` where `phi` is the regressor vector and σ-modification prevents parameter drift.
- Reference model state updated via `ssStep()` each sample.
- Projection operator clamps `‖theta‖ ≤ thetaMax` to prevent finite-escape.

**Test idea:**

```cpp
// [mrac] tag
// Plant: G(s) = 2/(s+1) with unknown gain; reference model M(s) = 1/(0.5s+1)
// REQUIRE: |y - y_m| < 0.05 after 200 steps (model tracking)
// REQUIRE: parameters() converges to known values within 10% (with PE reference)
// REQUIRE: sigma-modification keeps theta.norm() < thetaMax at all times
```

**Example:** `ex56_mrac_first_order.cpp` (C++), `examples/python/ex72_mrac_first_order.py`.

---

### E4. `lib/FeedbackLinearisation.h` — Nonlinear Cancellation + Linear Inner Loop

**Motivation.** Feedback linearisation (FL) and integrator backstepping are the core tools of nonlinear control design. A generic implementation using `std::function` for f(x,u) and g(x,u) fits cleanly in the existing `IController` framework.

**API sketch:**

```cpp
namespace ctrl {

// Single-input single-output FL for relative degree r = 1:
//   u = (v - f(x)) / g(x)
// v from inner linear controller on the linearised output error
struct FLParams {
    double uMin, uMax;
    double regularisationEps;  // prevents division by zero: g_eff = max(|g|, eps)
};

class FeedbackLinearisationController : public IController {
public:
    FeedbackLinearisationController(
        std::function<double(VectorXd, double)> f,   // drift term
        std::function<double(VectorXd, double)> g,   // input gain
        std::shared_ptr<IController>            innerCtrl, // linear inner loop (e.g., PID)
        const FLParams& params, double Ts);

    double compute(double error) override;        // error = r - y
    void   setState(const VectorXd& x);           // must be called before compute()
    void   reset() override;
private:
    std::function<double(VectorXd,double)> f_, g_;
    std::shared_ptr<IController>           inner_;
    VectorXd x_;
};
}
```

**Implementation notes:**
- `compute(error)`:
  1. Compute `v = inner_->compute(error)` (linear virtual control).
  2. Evaluate `u = clamp((v - f_(x_, last_u)) / max(g_(x_, last_u), eps), uMin, uMax)`.
  3. Update internal state via user-supplied integrator or external `setState()`.
- No Eigen matrix operations in the hot path for SISO case — fast enough for embedded.

**Test idea:**

```cpp
// [feedback_linearisation] tag
// Plant: ẋ = -x³ + u (strongly nonlinear, relative degree 1, minimum phase)
// Linearised: ẋ = v with inner PID Kp=5, Ki=2
// REQUIRE: |y - 1.0| < 0.05 after 500 steps (step to y=1)
// REQUIRE: internal state x remains bounded (|x| < 5)
```

**Example:** `ex57_feedback_linearisation.cpp`, Python: `ex73_feedback_linearisation.py`.

---

### E5. `lib/ZeroPhaseTrackingFilter.h` — Non-Minimum-Phase Feedforward

**Motivation.** Flexible structures, aerosurfaces, and robotic manipulators frequently have non-minimum-phase zeros. Perfect inversion is unstable for such plants. Zero-Phase Error Tracking Control (ZPETC, Tomizuka 1987) is the standard industry solution.

**API sketch:**

```cpp
namespace ctrl {

struct ZPETCResult {
    StateSpace forwardFilter;   // G_ff(z) = B+(z) * B-(1/z) / (normalisation)
    double     dcGainError;     // expected amplitude error at DC (< 1 for NMP)
    bool       hasNMPZeros;     // true if any |z_i| >= 1 in B(z)
};

ZPETCResult designZPETC(const StateSpace& plant, double Ts);
// Factorises B(z) into minimum-phase B+(z) and non-minimum-phase B-(z)
// Builds ZPETC pre-filter: achieves zero phase shift at all frequencies
// at the cost of |G_ff · G|  ≠ 1 away from DC
}
```

**Implementation notes:**
- Polynomial root finding: compute eigenvalues of companion matrix of B(z) (Eigen's `EigenSolver`).
- Partition roots into |zᵢ| < 1 (B+) and |zᵢ| ≥ 1 (B-).
- Normalise so DC gain = 1 (divide by B-(1)² ).
- The ZPETC filter is a `TransferFunction` → `StateSpace` → use `ssStep()` in the loop.

**Test idea:**

```cpp
// [zpetc] tag  
// Plant: G(z) = (z - 1.2) / (z² - 1.5z + 0.56)  (NMP zero at z=1.2)
// REQUIRE: hasNMPZeros == true
// REQUIRE: phase of (G_ff · G) at ω=0.1·π/Ts is within 0.1° of 0°
// REQUIRE: amplitude of (G_ff · G) at DC is within 1% of 1.0 (normalised)
// REQUIRE: open-loop step with ZPETC reaches 0.95 of reference in fewer steps than without
```

**Example:** `ex58_zpetc_flexible_link.cpp`, Python: `ex74_zpetc_flexible_link.py`.

---

### Implementation Status (all 5 DONE — 2026-05-28)

| Extension | Status | Key files | Tests |
|-----------|--------|-----------|-------|
| E2 `LinearisationHelper.h` | **DONE** | `lib/LinearisationHelper.h/.cpp` | `[linearisation]` × 2 |
| E4 `FeedbackLinearisation.h` | **DONE** | `lib/FeedbackLinearisation.h/.cpp` | `[fl]` × 2 |
| E3 `MRACController.h` | **DONE** | `lib/MRACController.h/.cpp` | `[mrac]` × 2 |
| E1 `BalancedTruncation.h` | **DONE** | `lib/BalancedTruncation.h/.cpp` | `[btm]` × 2 |
| E5 `ZeroPhaseTrackingFilter.h` | **DONE** | `lib/ZeroPhaseTrackingFilter.h/.cpp` | `[zpetc]` × 3 |

**Final counts after all extensions:** 78 C++ passed | 79 Python passed | 0 failures.

**Key fixes discovered during implementation:**
- E5 `evalTF`: C matrix must be `MatrixXcd`, not `VectorXcd` — Eigen's implicit reshape transposes a (1×n) row to (n×1) column, producing ×50 amplitude error.
- E5 `suggest_order` Python binding: name clash with SubspaceID's `suggestOrder(VectorXd)` — resolved by wrapping both in lambdas so pybind11 type dispatch selects the correct overload at runtime.
- FL pendulum (relative degree 2): inner PID must be tuned for a virtual double integrator, not the plant directly; use characteristic polynomial placement.

All five are compatible with the existing `IController` interface, `StateSpace` / `ssStep()` infrastructure, and `GradientProjectionQP` backend. No external dependencies beyond Eigen 3.4.

---

*End of document. Cross-reference: `docs/DOCUMENTATION.md` (Section 5 Class Reference), `docs/DEPLOYMENT.md` (parameter constraints), `cheatsheet/tuning_methods.md` (SOPDT IMC-PID), `cheatsheet/controller_categories.md` (implementation tiers).*
