# Model Predictive Attitude Control of USV Based on Short-Time Wave Prediction

## Reference

Liang Hong, Haitao Liu, Quanshun Yang, and Jiaxuan Yao (2024). "Model predictive attitude control of unmanned surface vehicle based on short-time wave prediction." *Ocean Engineering* 314, 119727. https://doi.org/10.1016/j.oceaneng.2024.119727

---

## Plant Model

The **S175 standard model USV** (a publicly available ship hull form used widely in seakeeping research), modelled using the full 6-DOF nonlinear equations of motion developed in Fossen (2011). The control objective is **attitude stabilisation**: reducing roll and pitch amplitudes and maintaining heading (yaw) in high sea states. Wave disturbances are modelled from the **Pierson-Moskowitz (PM) wave spectrum** and applied via interpolated Response Amplitude Operator (RAO) data. The paper's key contribution is a parallel **dual-LSTM short-term wave prediction** scheme that feeds predicted disturbances `Np` steps ahead into the MPC cost function, enabling proactive compensation rather than reactive correction.

### Physical Description

- **Hull model:** S175 container ship hull scaled to USV dimensions; full-load displacement, draught, and RAO data are publicly available.
- **Dynamics (Fossen 2011 framework):**
  - Rigid-body inertia `M_RB` (mass matrix + rotational inertia)
  - Added mass `M_A = A(inf)` at infinite frequency
  - Coriolis matrices `C_RB(υ)` and `C_A(υ_r)`
  - Damping `D = B_total(inf)` (linear viscous at infinite frequency)
  - Fluid memory effects `mu = \int K(t-tau)(υ(tau) - U*e1) dtau` modelled as a state-space retardation function `{A_r, B_r, C_r}`
  - Strip-theory drag force `tau_drag` applied to sway and yaw via sectional integration
  - Wave force `tau_wave` from RAO data (first-order + second-order mean drift)
- **Linearisation for MPC:** Around nominal surge speed U = 5.1 m/s, the 6-DOF equations reduce to a **9th-order continuous state-space model** with states `x = [p, q, r, phi, theta, ψ, z, v, w]^T` and inputs `u = [tau₄, tau₅, tau₆]^T` (roll, pitch, yaw control forces/moments). The linearised model is discretised at Ts = 0.1 s.
- **Wave model:** Pierson-Moskowitz spectrum S(omega) with significant wave height H_s = 8 m and modal period T_0 derived from ITTC formula. Wave realisation via 50-frequency sum of cosines with random phases. Wave forces computed per Eq. (13) using RAO amplitudes and phases interpolated by (U, beta, omega).

### State Variables (linearised 9-state model)

| Symbol | Description | Unit |
|--------|-------------|------|
| `p` | Roll rate | rad/s |
| `q` | Pitch rate | rad/s |
| `r` | Yaw rate | rad/s |
| `phi (phi)` | Roll angle | rad |
| `theta (theta)` | Pitch angle | rad |
| `psi (ψ)` | Yaw (heading) angle | rad |
| `z` | Heave displacement | m |
| `v` | Sway velocity | m/s |
| `w` | Heave velocity | m/s |

Control input `u = [tau₄, tau₅, tau₆]^T` - roll, pitch, and yaw moments/forces from active fin stabilisers, roll thrusters, or rudder combinations.

### Governing Equations

**Nonlinear 6-DOF (simulation plant):**
```
(M_RB + M_A) * υ. + C_RB*(υ) * υ + C_A*(υ_r) * υ_r + D * υ_r + mu + G(eta) = tau_wave + tau_control + tau_drag
eta. = J(eta) * υ
```

**Fluid memory state-space (retardation function):**
```
xdot_r = A_r * x_r + B_r * (υ - U*e1)
mu   = C_r * x_r
```

**Linearised discrete model for MPC (at U = 5.1 m/s):**
```
x_{k+1} = A * x_k + B * u_k + B_w * w_k
y_k     = C * x_k          [C selects phi, theta, ψ from state vector]
```
where `A = exp(A_c * Ts)`, `B = \int0ᵀˢ exp(A_c*tau)B_c dtau`, `n=9, p=3, m=3`.

**Wave disturbance (PM spectrum, linear irregular sea):**
```
eta_wave(t) = Sigma_i \sqrt(2 * S(omega_i) * Deltaomega) * cos(omega_ei * t + epsilon_i - k_i*(x*cos beta + y*sin beta))
```
`tau_wave,d` = RAO(omega_i, beta, U) * H_i, summed over 50 frequency components.

**LSTM disturbance prediction (paper's method):**
```
w_{k-1} = C * (x_k - A * x_{k-1} - B * u_{k-1})   [disturbance observer]
ŵ_{k:k+Np-1} = LSTM_parallel(w_{k-1}, history)     [parallel single+multistep LSTM]
```
LSTM parameters: 200 cells, 3 input channels, 3 output channels, Adam solver, 160 training epochs on 200 s of disturbance history.

### Key Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Surge speed | U | 5.1 m/s | Linearisation and simulation speed |
| Sampling time | Ts | 0.1 s | MPC discretisation |
| Prediction horizon | Np | 20 | MPC look-ahead steps (2 s) |
| Input weight | R | diag(1,1,1)*10^-^2 | MPC control cost |
| Output weight | Q, S | diag(1,1,1) | MPC tracking and terminal cost |
| Input limit | u_max | [0.1, 2, 4]^T | Roll, pitch, yaw actuator limits |
| Input rate limit | Deltau_max | [0.05, 0.5, 1]^T /s | Actuator slew rate |
| Significant wave height | H_s | 8 m | PM spectrum parameter |
| Wave frequencies | n | 50 | Frequency components in wave realisation |
| LSTM cells | - | 200 | Per LSTM layer |
| LSTM training iterations | - | 160 | Adam solver, lr0=0.01 |
| Scaling factor | k_ratio | 10⁹ | B-matrix scaling for numerical conditioning |

---

## Control Objective

Stabilise the USV attitude - **reduce roll and pitch amplitudes and maintain heading (yaw)** - against large irregular wave disturbances in high sea states (H_s = 8 m), while satisfying:

1. **Actuator constraints** - roll fin, pitch stabiliser, and yaw thruster have hard amplitude and slew-rate limits.
2. **Robustness to sensor noise** - realistic state feedback noise (sigma = 0.0017 rad for angle states, sigma = 0.1 for velocity states) must not degrade the controller.
3. **Adaptation to sea state changes** - if H_s changes significantly (e.g., 4 m -> 8 m), the control strategy should degrade gracefully or retrain.

**Benchmark from paper (Table 4, no input rate limit, no noise):**

| Strategy | Roll SD (^\circ) | Pitch SD (^\circ) | Yaw SD (^\circ) |
|----------|-------------|--------------|------------|
| Uncontrolled | 9.46 | 0.94 | 13.10 |
| PID | 0.0637 | 0.148 | 0.175 |
| LQR | 0.0022 | 0.00542 | 0.0088 |
| MPC | 0.00194 | 0.00120 | 0.00204 |
| MPC+FC1 | 2.70*10^-^4 | 1.70*10^-^4 | 2.43*10^-^4 |
| MPC+FC2 | 2.69*10^-^4 | 1.30*10^-^4 | 2.44*10^-^4 |
| **MPC+LSTM** | **1.79*10^-^4** | **1.18*10^-^4** | **1.47*10^-^4** |

MPC+LSTM improves ~34% over MPC+FC1 (best of the prior methods) under ideal conditions. Under input rate limits with sensor noise, LQR diverges and MPC+FC2 becomes unstable; MPC+LSTM matches plain MPC (the most robust baseline) while retaining ~12% advantage over MPC+FC1.

---

## Proposed Controller Roster

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | OpenLoop | - | Zero control; uncontrolled USV response | Baseline drift reference; establishes wave disturbance magnitude |
| 2 | PID | `DiscretePID` | Roll: Kp=1e8, Ki=5e7, Kd=2e8; Pitch/Yaw: see Table 3 | Per-DOF independent; paper baseline |
| 3 | LQR | `DiscreteLQR` | Q=diag(100,100,10,100,100,10,1,1,1), R=I*100 | Bryson design on 9-state linearised model; full-state feedback; unstable under rate limits |
| 4 | MPC | `DiscreteMPC` | Np=20, Nu=20, rho_y=1 (Q), rho_u=0.01 (R); u_max as above | No disturbance compensation; paper baseline MPC |
| 5 | MPC_FC1 | `DiscreteMPC` + FF | Same MPC; add y_d,k+l = w_{k-1} feedforward | MPC+FC1: Liang & Wen (2017) nominal-system feedforward; assumes constant w |
| 6 | MPC_FC2 | `DiscreteMPC` + FB | Same MPC; w_hat = C*(x_k - A*x_{k-1} - B*u_{k-1}) feedback | MPC+FC2: Jimoh et al. (2021) observed current disturbance; amplifies noise |
| 7 | MPC_LSTM | `DiscreteMPC` + LSTM | Same MPC; ŵ_{k:k+Np-1} from parallel LSTM predictor | Paper proposed; LSTM trained on 200 s disturbance history; parallel single+multistep |
| 8 | ADRC | `DiscreteADRC` | omega_o=2, omega_c=0.8, b0approx =B_m_roll; omega_o*Ts=0.20<0.5 | ESO absorbs wave disturbance as total external disturbance; per DOF (roll, pitch, yaw) |
| 9 | SMC | `DiscreteSMC` | c=5, K=0.05, phi=0.01 | Sliding surface on attitude error; saturation prevents chattering; compute(y - ref) |
| 10 | MRAC | `MRACController` | gamma=0.5, a_m=-2, b_m=2 | Adapts to effective wave forcing gain changes across sea states; compute(y_plant) |
| 11 | L1Adaptive | `L1AdaptiveController` | a_m=-2, b_m=2, Gamma=5, omega_c=1.5 | Fast wave-disturbance adaptation; low-pass filter bandwidth set to wave peak frequency |
| 12 | NeuralPID | `NeuralPID` | n_h=8, lr=1e-6, plant_gainapprox =B_tau_roll*Ts | Online gain adaptation to time-varying wave spectrum; per-DOF |

---

## Scenarios

| ID | Description | Reference Signal | Load / Stress |
|----|-------------|-----------------|---------------|
| s01_ideal | H_s = 8 m, beta = 0^\circ (head seas); no input rate limit; no sensor noise | phi_ref = theta_ref = ψ_ref = 0 (attitude stabilisation) | Uncontrolled SD: roll=9.46^\circ, pitch=0.94^\circ |
| s02_quartering | H_s = 8 m, beta = 45^\circ (quartering seas); no input rate limit; no sensor noise | Same zero-reference | Wave hits port bow; roll + yaw coupling increased |
| s03_rate_limited | H_s = 8 m, beta = 0^\circ; input rate limits active (Table 1); no sensor noise | Same zero-reference | Actuator slew constraint degrades LQR; tests MPC+LSTM vs MPC+FC2 |
| s04_noisy | H_s = 8 m, beta = 0^\circ; input rate limits active; Deltax ~ N(0, sigma^2) added to all states | Same zero-reference | sigma = [0.0017*6, 0.1*3] rad or m/s; MPC+FC2 becomes unstable |
| s05_sea_change | H_s ramps from 4 m to 8 m at t = 100 s (LSTM trained on 4 m history) | Same zero-reference | Tests LSTM generalisation; expect degradation if LSTM not retrained |

**Total runs:** 12 controllers * 5 scenarios = 60.

---

## Implementation Notes

- **Linearisation validity:** The 9-state linearised model is valid for small attitude perturbations around U = 5.1 m/s straight-line sailing. For scenario s02 (quartering seas), the wave-encounter coupling changes; the linearised B_w matrix should be recomputed for beta = 45^\circ. To keep the model simple, use the beta = 0^\circ linearisation for all scenarios and treat the heading-change-induced coupling as unmodelled disturbance.
- **Retardation function:** The fluid memory effect `mu = C_r * x_r` adds n_r additional states to the simulation plant. For an approximate simulation, truncate to 3-5 retardation states by fitting the kernel K(t) with a finite state-space model. This is sufficient for frequency-domain accuracy up to 2* the wave peak frequency.
- **LSTM not implemented in lib/:** The MPC+LSTM controller requires an LSTM neural network for wave prediction. For the Python-only sim, use `torch.nn.LSTM` (PyTorch) or `tensorflow.keras.LSTM` (TensorFlow) pre-trained on simulated wave disturbance history, called at each step to produce the Np-step disturbance forecast `ŵ_{k:k+Np-1}`. Pass this as an offset into the MPC cost function.
- **MPC+FC1 and MPC+FC2 as library controllers:** Both are variants of `DiscreteMPC` with modified cost-function disturbance terms. Implement as a Python wrapper around `ctrl.DiscreteMPC` that pre-computes the `ŵ` vector (constant or Kalman-estimated) and injects it into the QP before each `compute()` call.
- **ADRC omega_o constraint:** With Ts = 0.1 s, require `omega_o * Ts < 0.5` -> `omega_o < 5 rad/s`. Use omega_o = 2, omega_c = 0.8. The slow dynamics of roll/pitch (natural periods 2-10 s) are compatible with these low-bandwidth ESO settings.
- **k_ratio scaling:** The linearised B matrix for S175 produces very small control inputs relative to state magnitudes. Multiply B by k_ratio = 10⁹ and divide the MPC output u by 10⁹ to improve QP solver conditioning. This is a numerical convenience; the physical torques remain unchanged.
- **MRAC and L1 convention:** `set_reference(0)` then `compute(phi_plant)` - the reference is zero (attitude stabilisation to upright), NOT `compute(phi_ref - phi)`.
- **Evaluation metric:** Standard deviation of phi, theta, ψ over the steady-state portion (t > 50 s) of the simulation window. This is the metric from the paper's Tables 4-7.
- **CSV columns:** `t, phi_ref, phi, theta_ref, theta, psi_ref, psi, p, q, r, tau4, tau5, tau6, w_roll, w_pitch, w_yaw, sd_phi_cumul, iae_cumulative`

---

## Status

Spec only - `sim/` not present, not registered, not built.

The main implementation complexity is the LSTM wave predictor. For initial validation without LSTM:
1. Implement scenarios s01-s04 using only the 8 library controllers (PID, LQR, MPC, ADRC, SMC, MRAC, L1Adaptive, NeuralPID).
2. Add the three MPC variants (FC1, FC2, LSTM) as a second phase once the base simulation is validated.
3. For the LSTM predictor, pre-train on a simulated 200 s wave disturbance history at the target sea state before the main simulation begins; freeze weights during the control run.
