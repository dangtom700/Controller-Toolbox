# System Identification Methods
### A Practical Reference for Control Engineering Professionals

---

## Introduction

System identification is the process of constructing a mathematical model of a dynamic system from measured input-output data. The choice of model structure determines what dynamics can be captured, how the parameters are estimated, and how well the model generalises to unseen operating conditions. Identification results feed directly into controller design: a misspecified model structure produces a model whose uncertainty is not characterised, which in turn leads to a poorly tuned or non-robust controller.

This document organises the principal identification model structures from the simplest (FIR) to the most expressive (Gaussian Process, NARX), with emphasis on what distinguishes each structure and when to prefer it.

---

## 1. FIR and Step-Response Models

**Mechanism.** The output is a finite weighted sum of past inputs - no output feedback. The model is non-parametric in the sense that no pole/zero structure is assumed.

$$y(k) = \sum_{i=0}^{n} b_i\, u(k-i) + e(k)$$

The step-response variant (used in DMC/MPC):

$$y(k) = \sum_{i=1}^{N} s_i\, \Delta u(k-i)$$

where $s_i$ are the sampled step-response coefficients.

**Estimation.** Ordinary least squares (OLS); the regressor matrix is a Toeplitz matrix of past inputs.

**Applications.** Direct use in step-response MPC (DMC); initial non-parametric identification before fitting parametric models.

**Pros.** Linear in parameters; trivially estimated; no structural assumptions.

**Cons.** Many parameters for long-memory processes; cannot represent unstable plants; no noise model.

---

## 2. ARX (AutoRegressive with eXogenous Input)

**Mechanism.** Past outputs feed back into the prediction. The noise enters as white noise on the output - equivalent to assuming the noise is filtered by $1/A(q)$.

$$A(q)\,y(k) = B(q)\,u(k) + e(k)$$

where $A(q) = 1 + a_1 q^{-1} + \cdots + a_{na} q^{-na}$ and $B(q) = b_1 q^{-nk} + \cdots$.

**Estimation.** OLS; the regressor is $[y(k-1), \ldots, u(k-nk), \ldots]$. Computationally the cheapest parametric estimator.

**Applications.** Real-time adaptive control (RLS-based ARX); first-cut parametric identification; MATLAB `arx()`.

**Pros.** Linear in parameters; globally convergent OLS; fast.

**Cons.** Noise model is tied to plant denominator ($A$); biased estimates when noise is coloured and order is misspecified.

---

## 3. ARMAX

**Mechanism.** Adds an independent moving-average noise polynomial $C(q)$ to handle coloured disturbances (e.g., load disturbances filtered through integrators).

$$A(q)\,y(k) = B(q)\,u(k) + C(q)\,e(k)$$

**Estimation.** Iterative prediction-error minimisation (PEM); not globally convex - requires good initialisation (ARX solution is standard).

**Applications.** Processes with coloured process noise (chemical reactors, power systems).

**Pros.** More accurate noise model than ARX; widely supported (MATLAB `armax()`).

**Cons.** Non-convex optimisation; slower than ARX; requires order selection for $C$.

---

## 4. OE (Output Error)

**Mechanism.** Separates plant dynamics from noise entirely. The predictor is a pure simulation of the plant - no output feedback in the noise path.

$$y(k) = \frac{B(q)}{F(q)}\,u(k) + e(k), \quad \hat{y}(k|\theta) = \frac{B(q)}{F(q)}\,u(k)$$

**Estimation.** PEM; non-linear in parameters (iterative).

**Applications.** MPC model identification (simulation-based predictor matches MPC internal model); servo drive identification.

**Pros.** Unbiased plant estimate independent of noise colour; simulation-consistent (no output feedback divergence).

**Cons.** Non-convex; can diverge with poor initialisation; no disturbance model for feedforward.

---

## 5. Box-Jenkins (BJ)

**Mechanism.** The most general single-input single-output polynomial structure. Plant and noise dynamics are fully decoupled.

$$y(k) = \frac{B(q)}{F(q)}\,u(k) + \frac{C(q)}{D(q)}\,e(k)$$

**Estimation.** PEM; four polynomial orders to select ($n_B, n_F, n_C, n_D$) plus delay.

**Applications.** High-accuracy identification for robust controller design; situations where independent plant and noise models are required.

**Pros.** Maximum flexibility; asymptotically efficient when orders are correct.

**Cons.** Most parameters to tune; most sensitive to local minima; slow.

---

## 6. Innovations State-Space (N4SID / Subspace)

**Mechanism.** A MIMO state-space model is identified directly from block-Hankel matrices of input-output data without iterative optimisation.

$$x(k+1) = A\,x(k) + B\,u(k) + K\,e(k), \quad y(k) = C\,x(k) + D\,u(k) + e(k)$$

**Estimation.** Singular value decomposition of a weighted projection; MATLAB `n4sid()`, `ssest()`.

**Applications.** MIMO process identification; LQG/MPC model development; any situation where the model order is unknown.

**Pros.** Non-iterative; MIMO-capable; provides direct state-space matrices for LQR/MPC/Kalman design.

**Cons.** Order selection requires SVD inspection; accuracy limited by data length and signal-to-noise ratio.

---

## 7. Process Model (FOPDT / SOPDT)

**Mechanism.** Fit a continuous-time low-order transfer function with explicit dead time to step-response or frequency-response data.

$$G(s) = \frac{K}{(\tau s + 1)}\,e^{-T_d s} \quad \text{(FOPDT)}$$

Parameters $K$, $\tau$, $T_d$ estimated via the tangent/area method or nonlinear least squares (`procest` in MATLAB).

**Applications.** Process control tuning (IMC-PID, Cohen-Coon, AMIGO); wherever a physical interpretation of gain, time constant, and dead time is needed.

**Pros.** Interpretable parameters; directly feeds IMC and Cohen-Coon PID tuning rules.

**Cons.** Inadequate for oscillatory, integrating, or unstable plants; dead-time estimation is sensitive to noise.

---

## 8. Nonlinear Structures (NARX, Hammerstein-Wiener, Takagi-Sugeno, GP)

| Structure | Nonlinearity location | Estimation |
|---|---|---|
| **Hammerstein-Wiener** | Static input/output nonlinearity around linear dynamics | Iterative linear/nonlinear separation |
| **NARX** | Fully nonlinear map of past I/O | Neural network, gradient descent |
| **Takagi-Sugeno** | Blended local linear models via fuzzy membership | Fuzzy c-means + local linear regression |
| **Gaussian Process** | Non-parametric; posterior mean + variance over functions | Marginal likelihood maximisation |

**Common applications.** Actuator saturation modelling (Hammerstein); engine/power electronics (NARX); gain-scheduled MPC linearisation (TS); data-efficient identification under uncertainty (GP).

**Key trade-off.** Expressiveness vs. data requirements. NARX and GP models can represent any smooth function but require substantial excitation data and careful regularisation.

---

## Conclusion

Model structure selection should be guided by three questions: (1) How much prior structural knowledge is available? (2) Is the noise model important for the downstream controller design? (3) Is the system linear, mildly nonlinear, or strongly nonlinear?

For linear plants with unknown noise, start with ARX for speed, refine with OE if simulation accuracy matters, and use BJ when both plant and disturbance models must be accurate. For MIMO systems, N4SID subspace methods are the default. For nonlinear plants, begin with Hammerstein-Wiener if the nonlinearity location is known, and escalate to NARX or GP only when simpler structures cannot fit validation data. In all cases, validate the identified model on a separate dataset not used for estimation - a model that fits training data perfectly but fails on validation data is an overfit model, not a system model.
