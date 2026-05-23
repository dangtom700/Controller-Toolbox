# Tuning Methods
### A Practical Reference for Control Engineering Professionals

---

## Introduction

Selecting the right tuning method is as consequential as selecting the right controller structure. A well-tuned LQR on a poorly characterised plant can outperform a poorly tuned Hinf controller on a well-characterised one. This document surveys the principal tuning strategies used in discrete-time control practice, organised by the information they require and the cost function they minimise. Engineers familiar with PID controllers and state-space models will find each method described at the level of working implementation rather than derivation.

The tuning strategies are divided into three families: **heuristic/frequency-domain methods** (require little model information), **optimisation-based methods** (require a model or simulation), and **adaptive/online methods** (operate during closed-loop execution).

---

## 1. Ziegler-Nichols Relay Tuning

**Mechanism.** Apply a relay of amplitude +/-*d* in a closed loop. The plant self-oscillates at its ultimate frequency. Measure the peak-to-peak output amplitude *a* and oscillation period *T_u*.

**Key equations.**

$$K_u = \frac{4d}{\pi a}, \quad T_u = \text{average period}$$

| Rule | $K_p$ | $T_i$ | $T_d$ |
|---|---|---|---|
| Classic ZN | $0.6\,K_u$ | $0.5\,T_u$ | $0.125\,T_u$ |
| Tyreus-Luyben | $K_u/2.2$ | $2.2\,T_u$ | $T_u/6.3$ |
| AMIGO | $0.4\,K_u$ | $T_u/0.8$ | $0.1\,K_p T_u$ |

**Applications.** Process control (temperature, flow, pressure loops), embedded systems where only a relay test is practical.

**Pros.** No explicit plant model required; fast; runs online.

**Cons.** Aggressive ZN settings produce ~25 % overshoot; Tyreus-Luyben is more conservative but slower; AMIGO offers the best balance. Dead-time-dominant plants require the Smith Predictor extension.

---

## 2. IMC-PID (Internal Model Control)

**Mechanism.** Invert the invertible part of the plant model, then add a low-pass filter of bandwidth lambda to ensure robustness and properness. The result is algebraically reduced to PID form.

**Key equations.** For a FOPDT model $G(s) = Ke^{-\theta s}/(\tau s+1)$:

$$K_p = \frac{2\tau + \theta}{2K(\lambda + \theta)}, \quad T_i = \tau + \frac{\theta}{2}, \quad T_d = \frac{\tau\theta}{2\tau+\theta}$$

where lambda is the closed-loop time constant (tuning knob). Increasing lambda sacrifices speed for robustness.

**Applications.** Chemical process loops; anywhere a FOPDT step-response identification is available (use `StepResponseTuner::identify` + `computePIDParams(IMC)`).

**Pros.** Single tuning parameter (lambda); predictable closed-loop bandwidth; guaranteed stability for stable, minimum-phase plants.

**Cons.** Requires an explicit FOPDT model; performance degrades when the true plant deviates significantly from the model.

---

## 3. LQR and Bryson's Rule

**Mechanism.** Minimise the infinite-horizon quadratic cost $J = \sum_k (x_k^T Q x_k + u_k^T R u_k)$ by solving the Discrete Algebraic Riccati Equation (DARE). Bryson's rule provides a principled starting point for the weight matrices.

**Key equations.**

$$Q_{ii} = \frac{1}{(\text{max allowable } x_i)^2}, \quad R_{jj} = \frac{1}{(\text{max allowable } u_j)^2}$$

$$P = A^T P A - (A^T P B)(R + B^T P B)^{-1}(B^T P A) + Q, \quad K = (R + B^T P B)^{-1} B^T P A$$

**Applications.** Mechatronic systems (DC motors, robot joints), aerospace attitude control, any plant with known state-space model and interpretable state/input bounds.

**Pros.** Globally optimal for linear systems; Bryson weights have physical interpretations; single unified DARE solve.

**Cons.** Requires full-state feedback (observer needed for output-only measurement); not directly applicable to nonlinear plants; Q/R choice is still a design decision.

---

## 4. LQG (LQR + Kalman Filter)

**Mechanism.** Combine an LQR gain (from DARE on the plant model) with a Kalman filter gain (from DARE on the noise covariance matrices). The Separation Principle guarantees the two designs are independent.

**Key equations.** Process noise $Q_f$, measurement noise $R_f$; Kalman gain:

$$L = P_e C^T (C P_e C^T + R_f)^{-1}$$

where $P_e$ is the steady-state error covariance. Controller: $u_k = -K\hat{x}_k$.

**Applications.** Output-feedback control with sensor noise (inertial navigation, power electronics, vibration control).

**Pros.** Optimal for linear Gaussian systems; noise covariances have physical interpretations (process noise variance, sensor noise variance).

**Cons.** Sensitive to model mismatch; LQG controllers can be non-robust (Doyle 1978 counterexample). Supplement with robustness checks (LQG/LTR or Hinf).

---

## 5. MPC Weight and Horizon Tuning

**Mechanism.** Minimise a finite-horizon cost over prediction horizon $N_p$ and control horizon $N_c$:

$$J = \sum_{i=1}^{N_p} \rho_y \|y_{k+i} - r\|^2 + \sum_{j=0}^{N_c-1} \rho_u \|\Delta u_{k+j}\|^2$$

**Tuning procedure:**
1. Estimate plant settling time $t_s$; set $N_p \geq t_s / T_s$.
2. Set $N_c \approx N_p/3$ (smoother control with fewer free variables).
3. Start with $\rho_y = 1$, $\rho_u = 0.1$; increase $\rho_u$ to suppress aggressive moves.

**Applications.** Multi-variable process control (distillation, HVAC, battery management), constrained plants.

**Pros.** Handles hard constraints on $u$ and $\Delta u$ explicitly; MIMO-capable.

**Cons.** Computationally intensive for long horizons; stability guarantees require terminal constraints; weight tuning remains iterative.

---

## 6. Frequency-Domain Loop Shaping

**Mechanism.** Design a lead-lag compensator $C(s) = K(s+z_c)/(s+p_c)$ so that the open-loop gain crossover occurs at a desired frequency $\omega_c$ with a target phase margin.

**Key equations.** For phase addition $\phi$ at $\omega_c$:

$$\alpha = \frac{1+\sin\phi}{1-\sin\phi}, \quad z_c = \frac{\omega_c}{\sqrt{\alpha}}, \quad p_c = \omega_c\sqrt{\alpha}, \quad K = \frac{\sqrt{\alpha}}{|G(j\omega_c)|}$$

**Applications.** Servo drives, audio amplifiers, any system where bandwidth and phase margin are the primary specifications.

**Pros.** Direct frequency-domain interpretation; classical Bode-plot design.

**Cons.** SISO only; requires knowledge of open-loop plant magnitude at $\omega_c$.

---

## 7. GA / PSO Optimisation-Based Tuning

**Mechanism.** Treat controller gains as a parameter vector $\theta$. Evaluate a simulation-based cost (IAE, ITAE, $H_2$ norm) and update $\theta$ via evolutionary or swarm operations.

$$\text{ITAE} = \int_0^T t\,|e(t)|\,dt$$

**Pros.** Model-free (cost evaluated via simulation); handles nonlinear, non-convex parameter spaces; applicable to any controller structure.

**Cons.** Computationally expensive; no stability guarantees during search; solution quality depends on cost function design.

---

## Conclusion

No single tuning method dominates across all scenarios. Relay-based ZN/AMIGO methods are the practitioner's first resort when model information is scarce. IMC-PID and loop shaping offer structured, bandwidth-centric design once a FOPDT or frequency-response model is available. LQR/LQG and MPC methods provide optimality and constraint handling at the cost of requiring an accurate state-space model. Optimisation-based methods (GA, PSO) serve as a fallback for nonlinear or highly coupled systems. Effective tuning practice typically combines one or more of these approaches in sequence: identify first, shape second, optimise last.
