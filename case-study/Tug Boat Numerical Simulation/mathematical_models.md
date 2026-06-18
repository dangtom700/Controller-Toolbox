# Tug Boat Numerical Simulation - Mathematical Models

**Document:** Mathematical Models and Input/Output Parameters
**Audience:** Technical engineers familiar with control systems and C++ programming
**Date:** 2026-05-23

All notation follows Li et al. (2026) unless stated otherwise.

---

## 1. Coordinate Frames and State Variables

Two reference frames are used:

- **World frame** (NED or fixed inertial): position and heading
  $\boldsymbol{\eta} = [x,\; y,\; \psi]^\top$

- **Body frame** (barge-fixed): velocity and angular rate
  $\boldsymbol{\nu} = [u,\; v,\; r]^\top$

The full simulation state vector is:

$$
\mathbf{X} = [\,x,\; y,\; \psi,\; u,\; v,\; r\,]^\top \in \mathbb{R}^6
$$

---

## 2. Kinematics

The relationship between world-frame position rate and body-frame velocity is:

$$
\dot{\boldsymbol{\eta}} = R(\psi)\,\boldsymbol{\nu}
$$

where the rotation matrix is:

$$
R(\psi) = \begin{bmatrix}
\cos\psi & -\sin\psi & 0 \\
\sin\psi & \phantom{-}\cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

Heading $\psi$ is wrapped to $(-\pi,\,\pi]$ after each integration step.

---

## 3. Unified Plant Equation (Li et al. Eq. 21)

The combined barge-tugboat system obeys:

$$
M_\mathrm{re}\,\dot{\boldsymbol{\nu}} + C_\mathrm{re}(\boldsymbol{\nu})\,\boldsymbol{\nu} + D_\mathrm{re}\,\boldsymbol{\nu} = \boldsymbol{\tau}_\mathrm{re,env} + \boldsymbol{\tau}^\mathrm{b}_\mathrm{main}
$$

where the reconstructed $3\times3$ matrices absorb all four tugboat contributions:

$$
M_\mathrm{re} = M^\mathrm{b} + \sum_{i=1}^{4} \tilde{M}^{t,i}, \qquad
D_\mathrm{re} = D^\mathrm{b} + \sum_{i=1}^{4} \tilde{D}^{t,i}
$$

$$
C_\mathrm{re}(\boldsymbol{\nu}) = C^\mathrm{b}(\boldsymbol{\nu}) + \sum_{i=1}^{4} \tilde{C}^{t,i}(\boldsymbol{\nu})
$$

$$
\boldsymbol{\tau}_\mathrm{re,env} = \boldsymbol{\tau}^\mathrm{b}_\mathrm{env} + \sum_{i=1}^{4} \tilde{\boldsymbol{\tau}}^{t,i}_\mathrm{env}, \qquad
\boldsymbol{\tau}^\mathrm{b}_\mathrm{main} = \sum_{i=1}^{4} \tilde{\boldsymbol{\tau}}^{t,i}_\mathrm{main}
$$

### 3.1 Barge Generalized Mass

$$
M^\mathrm{b} = \begin{bmatrix}
m^\mathrm{b} - X_{\dot{u}} & 0 & 0 \\
0 & m^\mathrm{b} - Y_{\dot{v}} & 0 \\
0 & 0 & I^\mathrm{b}_{zz} - N_{\dot{r}}
\end{bmatrix}
$$

with $m^\mathrm{b} = 1.39\times10^8\;\text{kg}$, $I^\mathrm{b}_{zz} = \frac{m^\mathrm{b}}{12}(L^2+B^2)$,
and added-mass coefficients $X_{\dot{u}} = 1.39\times10^7$, $Y_{\dot{v}} = 9.73\times10^7$,
$N_{\dot{r}} = 2.78\times10^{10}\;\text{kg}$.

### 3.2 Tug Lever-Arm Projection (Eq. 20)

For tug $i$ at station $(x_i,\,y_i)$ with nominal thrust angle $\alpha_i$, the tug-to-barge
projection matrices are constructed via:

$$
J_i = \begin{bmatrix} \cos\alpha_i \\ \sin\alpha_i \\ -y_i\cos\alpha_i + x_i\sin\alpha_i \end{bmatrix}
\begin{bmatrix} \cos\alpha_i & \sin\alpha_i & -y_i\cos\alpha_i + x_i\sin\alpha_i \end{bmatrix}
$$

so that $\tilde{M}^{t,i} = J_i(m^{t,i} + X^t_{\dot{u}}) \cdot [\text{appropriate block}]$ per
the full expression in Li et al. Eq. (20).

### 3.3 Barge Linear Damping

$$
D^\mathrm{b} = \begin{bmatrix}
-X_u & 0 & 0 \\
0 & -Y_v & 0 \\
0 & 0 & -N_r
\end{bmatrix}, \qquad
X_u = 5\times10^5,\; Y_v = 8\times10^5,\; N_r = 5\times10^9\;\text{(SI)}
$$

### 3.4 Coriolis-Centripetal Term

$$
C^\mathrm{b}(\boldsymbol{\nu}) = \begin{bmatrix}
0 & 0 & -(m^\mathrm{b} - Y_{\dot{v}})v \\
0 & 0 & (m^\mathrm{b} - X_{\dot{u}})u \\
(m^\mathrm{b} - Y_{\dot{v}})v & -(m^\mathrm{b} - X_{\dot{u}})u & 0
\end{bmatrix}
$$

---

## 4. RK4 State Integration

The ODE right-hand side is:

$$
f(\mathbf{X}) = \begin{bmatrix}
R(\psi)\,\boldsymbol{\nu} \\
M_\mathrm{re}^{-1}\!\left(\boldsymbol{\tau}_\mathrm{re,env} + \boldsymbol{\tau}^\mathrm{b}_\mathrm{main}
- C_\mathrm{re}(\boldsymbol{\nu})\,\boldsymbol{\nu} - D_\mathrm{re}\,\boldsymbol{\nu}\right)
\end{bmatrix}
$$

Classical fourth-order Runge-Kutta integration at fixed step $\Delta t = 0.5\;\text{s}$:

$$
\begin{aligned}
k_1 &= f(\mathbf{X}_k) \\
k_2 &= f\!\left(\mathbf{X}_k + \tfrac{\Delta t}{2}\,k_1\right) \\
k_3 &= f\!\left(\mathbf{X}_k + \tfrac{\Delta t}{2}\,k_2\right) \\
k_4 &= f\!\left(\mathbf{X}_k + \Delta t\,k_3\right) \\
\mathbf{X}_{k+1} &= \mathbf{X}_k + \frac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4)
\end{aligned}
$$

---

## 5. Environmental Disturbances

All disturbance forces are expressed in body frame as
$\boldsymbol{\tau}_\mathrm{env} = [\tau_x,\;\tau_y,\;\tau_\psi]^\top$.

### 5.1 Wind Load (Eq. 3)

Dynamic pressure: $q_w = \tfrac{1}{2}\rho_\mathrm{air}U_\mathrm{wr}^2$, where $U_\mathrm{wr}$
is the magnitude of the relative wind velocity in body frame.

$$
\boldsymbol{\tau}_\mathrm{wind} = q_w \begin{bmatrix}
A_F \cdot C_{wx}(\gamma_\mathrm{wr}) \\
A_L \cdot C_{wy}(\gamma_\mathrm{wr}) \\
A_L L_\mathrm{oa} \cdot C_{w\psi}(\gamma_\mathrm{wr})
\end{bmatrix}
$$

with analytic coefficient functions:

$$
C_{wx}(\gamma) = -C_{wx,\max}\cos\gamma, \qquad
C_{wy}(\gamma) = C_{wy,\max}\sin\gamma, \qquad
C_{w\psi}(\gamma) = C_{w\psi,\max}\sin 2\gamma
$$

Parameters: $\rho_\mathrm{air}=1.225\;\text{kg/m}^3$, $A_F=862.5\;\text{m}^2$,
$A_L=3450\;\text{m}^2$, $C_{wx,\max}=0.85$, $C_{wy,\max}=1.10$, $C_{w\psi,\max}=0.20$.

### 5.2 Current Load (Eq. 4)

Dynamic pressure: $q_c = \tfrac{1}{2}\rho_\mathrm{water}U_\mathrm{cr}^2$.

$$
\boldsymbol{\tau}_\mathrm{current} = q_c \begin{bmatrix}
A_{F,u} \cdot C_{cx}(\gamma_\mathrm{cr}) \\
A_{L,u} \cdot C_{cy}(\gamma_\mathrm{cr}) \\
A_{L,u} L_\mathrm{oa} \cdot C_{c\psi}(\gamma_\mathrm{cr})
\end{bmatrix}
$$

where $A_{F,u}=\text{beam}\times\text{draft}=862.5\;\text{m}^2$,
$A_{L,u}=L_\mathrm{oa}\times\text{draft}=3450\;\text{m}^2$,
$\rho_\mathrm{water}=1025\;\text{kg/m}^3$, and coefficients $C_{cx}=-C_u\cos\gamma$,
$C_{cy}=C_v\sin\gamma$, $C_{c\psi}=C_\psi\sin2\gamma$ with
$C_u=0.25$, $C_v=0.45$, $C_\psi=0.10$.

### 5.3 Wave Drift (Simplified JONSWAP)

A two-component JONSWAP approximation of the second-order mean drift force:

$$
\boldsymbol{\tau}_\mathrm{drift} = \left(\sum_{j=1}^{2} a_j\cos(\omega_j t + \phi_j)\right)^2
\begin{bmatrix} d_x \\ d_y \\ d_\psi \end{bmatrix}
$$

with JONSWAP spectral amplitudes $a_j$ derived from $H_s=2\;\text{m}$, $T_p=10\;\text{s}$,
$\gamma_J=3.3$; drift coefficients $d_x=1.2\times10^4$, $d_y=2.0\times10^4$,
$d_\psi=5.0\times10^5$; and random phase $\phi_j$ seeded per scenario.

---

## 6. Thrust Allocation

The controller outputs a three-axis generalized force/moment vector
$\boldsymbol{\tau}_c = [\tau_x,\;\tau_y,\;\tau_\psi]^\top$, which is distributed among
the four tugs by solving a constrained least-squares problem.

### 6.1 Direction Matrix

For tug $i$ at station $(x_i,y_i)$ with thrust angle $\alpha_i$:

$$
B = \begin{bmatrix}
\cos\alpha_1 & \cos\alpha_2 & \cos\alpha_3 & \cos\alpha_4 \\
\sin\alpha_1 & \sin\alpha_2 & \sin\alpha_3 & \sin\alpha_4 \\
\ell_1 & \ell_2 & \ell_3 & \ell_4
\end{bmatrix}, \quad \ell_i = -y_i\cos\alpha_i + x_i\sin\alpha_i
$$

### 6.2 Pseudo-Inverse Allocation with Constraints

Unconstrained solution (minimum-norm baseline):

$$
\mathbf{T}^* = B^\top(BB^\top)^{-1}\,\boldsymbol{\tau}_c
$$

Per-tug box constraint: $T_{i,\min}=200\;\text{kN} \;\leq\; T_i \;\leq\; 2500\;\text{kN}=T_{i,\max}$

Rate constraint: $|\Delta T_i| \leq 75\;\text{kN}$ per $\Delta t = 0.5\;\text{s}$
(equivalently, $\leq 150\;\text{kN/s}$).

Constrained allocations exceeding box limits are clamped and the residual force error is
logged as the loss rate $\mathcal{L}_{\Delta T}$.

---

## 7. Measurement Model

In noisy-measurement mode, the observed state $\mathbf{z}_k$ is:

$$
\mathbf{z}_k = \mathbf{X}_k + \mathbf{v}_k, \qquad \mathbf{v}_k \sim \mathcal{N}(\mathbf{0},\,R_\mathrm{meas})
$$

$$
R_\mathrm{meas} = \mathrm{diag}\!\left[\sigma_x^2,\;\sigma_y^2,\;\sigma_\psi^2,\;\sigma_u^2,\;\sigma_v^2,\;\sigma_r^2\right]
$$

Default values: $\sigma_x = \sigma_y = 0.1\;\text{m}$, $\sigma_\psi = 1.7\times10^{-3}\;\text{rad}$
(0.1^\circ), $\sigma_u = \sigma_v = 0.05\;\text{m/s}$, $\sigma_r = 5\times10^{-4}\;\text{rad/s}$.

---

## 8. Input and Output Parameters

### 8.1 Plant Inputs

| Symbol | Description | Units | Typical Range |
|--------|-------------|-------|---------------|
| $\boldsymbol{\tau}_c$ | Controller generalized force/moment command | N, N.m | $\pm2\times10^6$ N; $\pm5\times10^7$ N.m |
| $T_i$ | Per-tug thrust magnitude (after allocation) | kN | $[200,\;2500]$ |
| $\boldsymbol{\tau}_\mathrm{env}$ | Total environmental disturbance | N, N.m | scenario-dependent |

### 8.2 Plant Outputs / Controller Inputs

| Symbol | Description | Units |
|--------|-------------|-------|
| $\eta = [x,\,y,\,\psi]$ | World-frame position and heading | m, rad |
| $\nu = [u,\,v,\,r]$ | Body-frame velocity and yaw rate | m/s, rad/s |
| $\mathbf{z}_k$ | Noisy measurement (if noise enabled) | same |

### 8.3 Performance Metric Outputs

| Metric | Definition |
|--------|-----------|
| $\mathrm{IAE}_x = \int|e_x|\,dt$ | Integral absolute error in surge (m.s) |
| $\mathrm{IAE}_y = \int|e_y|\,dt$ | Integral absolute error in sway (m.s) |
| $\mathrm{IAE}_\psi = \int|e_\psi|\,dt$ | Integral absolute heading error (rad.s) |
| $E_\mathrm{fuel} = \sum_i\sum_k T_{i,k}^2\,\Delta t$ | Total thrust energy (kN^2.s) |
| $\mathrm{Sat}_\mathrm{count}$ | Count of ticks with $|\Delta T_i|\geq\Delta T_\mathrm{max}$ | - |

---

## 9. Numerical Plant Parameters Summary

| Parameter | Barge | Each Tug | Units |
|-----------|-------|----------|-------|
| Displacement $m$ | $1.39\times10^8$ | $8.92\times10^6$ | kg |
| Length $L_\mathrm{oa}$ | 200 | 80 | m |
| Beam $B$ | 50 | 20 | m |
| Draft | 17.25 | 6.9 | m |
| Added mass $X_{\dot{u}}$ | $1.39\times10^7$ | $8.92\times10^5$ | kg |
| Added mass $Y_{\dot{v}}$ | $9.73\times10^7$ | $6.24\times10^6$ | kg |
| Added mass $N_{\dot{r}}$ | $2.78\times10^{10}$ | $1.14\times10^9$ | kg |
| Damping $X_u$ | $5.0\times10^5$ | $1.0\times10^5$ | N.s/m |
| Damping $Y_v$ | $8.0\times10^5$ | $1.8\times10^5$ | N.s/m |
| Damping $N_r$ | $5.0\times10^9$ | $5.0\times10^8$ | N.m.s/rad |

Tug station positions (from barge center) and nominal thrust angles:

| Tug | $x_i$ (m) | $y_i$ (m) | $\alpha_{i,\mathrm{nom}}$ (^\circ) |
|-----|-----------|-----------|-------------------------------|
| 1 | +60 | +25 | -120 |
| 2 | -60 | +25 | -60 |
| 3 | -60 | -25 | +60 |
| 4 | +60 | -25 | +120 |
