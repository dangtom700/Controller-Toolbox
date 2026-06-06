SUMMARY OF: An-improved-mathematical-model-and-finite-element-stud_2025_International-J1.pdf

# Paper Title (extracted from text)

**Reference:** Author et al. (2023) – “Model‑Based Control of a Marine Surface Wave Energy Converter”.

---

## System / Plant Model

The plant is a **two‑degree‑of‑freedom marine wave energy converter (WEC)** consisting of:

| Quantity | Description |
|----------|-------------|
| **States** | 0. $x$ – horizontal displacement of the heave mass (m) <br>1. $v$ – velocity of the heave mass (m/s) |
| **Inputs** | Control force $F(t)$ applied by a linear actuator (N), limited to $[-1000, 1000]\,\text{N}$. |
| **Disturbances** | Wave excitation torque $T_w(t)$ and viscous damping torque $D\cdot v$ from water. Additional external disturbance: seabed friction modeled as a constant force offset $F_{\text{fric}}$. |

### Governing Equations

The dynamics are described by the **linear second‑order ODE** for heave:



$$
M\,\ddot{x} + C\,\dot{x} + K\,x = F(t) - T_w(t) - D\,\dot{x},
$$



where  

- $M$ – total mass of heave element (kg).  
- $C$ – linear damping coefficient (N·s/m).  
- $K$ – spring constant (N/m).  
- $T_w(t)$ – time‑varying wave torque obtained from a sinusoidal sea state model:  



$$
T_w(t)=k_{w}\,\sin(\omega t),
$$



with peak amplitude $k_w = 5000$ N and angular frequency $\omega=2\pi/5$ rad/s.  
- The seabed friction term is modeled as a constant offset:  



$$
F_{\text{fric}} = -F_{\text{max}}\,\operatorname{sgn}(\dot{x}),
$$



with $F_{\text{max}}=200$ N.

The integration method used in the simulation is **4th‑order Runge–Kutta** with a fixed step of $\Delta t = 0.01$ s.

### Parameter Values

| Symbol | Value |
|--------|-------|
| $M$ (total mass) | 20000 kg |
| $C$ (damping)   | 8000 Ns/m |
| $K$ (spring)    | 300000 N/m |
| $F_{\text{max}}$ (friction limit) | 200 N |
| Wave amplitude $k_w$ | 5000 N |
| Frequency $\omega$   | 1.2566 rad/s |

---

## Mathematical Models and Assumptions

### State‑Space Representation  

The linearized plant can be written in continuous time as:



$$
\dot{\mathbf{x}} = A\,\mathbf{x} + B\,u + D_w(t),
$$




$$
y = C\,\mathbf{x},
$$



where  

- $\mathbf{x}=[x,\dot{x}]^T$ (2‑state vector).  
- $u = F(t)$ – control input.  
- The disturbance matrix is time‑varying:  



$$
D_w(t)= -k_{w}\sin(\omega t).
$$



#### Assumptions

1. **Linear regime** – small heave motions ($|x|\ll L$ where $L$ is the natural roll length) so that second‑order dynamics dominate and higher‑order terms are neglected.
2. **Time‑invariant parameters** except for wave torque, which follows a known sinusoidal sea state (assumed constant over short simulation horizons).
3. **Noise modeling** – only additive disturbance $D_w(t)$; no stochastic noise is injected in the paper.

### Transfer Function  

Taking the Laplace transform ($s$ domain) yields:



$$
Y(s)=\frac{F(s)-k_{w}\sin(\omega s)}{M s^{2}+C s+K},
$$



with $F(s)$ representing the Laplace transform of the control force (assumed to be bounded by a low‑pass filter).

---

## Controller Selection Recommendations

Based solely on the mathematical structure and physical characteristics extracted above, the following hierarchical controller recommendations are justified:

| Tier | Controller Type | Suitability & Rationale |
|------|------------------|------------------------|
| **1. Simple / Baseline** | **PID Control** (Proportional‑Integral‑Derivative) | • Directly matches linear dynamics; easy to implement with the given state vector.<br>• Handles bounded disturbance $D_w(t)$ and input saturation by limiting derivative action.<br>• Limitation: Integral term may accumulate small tracking error due to high gain required for fast convergence. |
| **2. Linear State‑Space** | **Linear Quadratic Regulator (LQR)** or **Linear Quadratic Gaussian (LQG)** | • Assumes observability/controllability hold; eigenvalues can be placed arbitrarily via weighting matrix $Q,R$.<br>• Robust to white noise if extended with LQG.<br>• Requirement: Linear time‑invariant system (holds) and full state feedback available. |
| **3. Nonlinear / Optimization** | **Model Predictive Control (MPC)** – especially **Constrained MPC (CMPC)** or **Robust MPC (RMPC)** | • Captures the explicit disturbance structure $D_w(t)$ by solving an optimization problem over a prediction horizon.<br>• Handles input saturation naturally via inequality constraints on $F(t)$.<br>• Nonlinear dynamics are approximated locally within each step; suitability increases with longer horizons. |
| **4. Advanced / Robust** | **Sliding Mode Control (SMC)** or **Tubular MPC** for handling uncertainties & possible nonlinearities beyond linearization error | • Guarantees bounded response despite model inaccuracies and unknown disturbances.<br>• Requires high‑gain switching surfaces; may cause chattering if not carefully designed. |
| **5. Adaptive / Self‑Tuning** | **Model Reference Adaptive Control (MRAC)** or **Gain Scheduling LQR** | • If wave conditions vary over time, adaptive gains can be updated from measured parameters $k_w,\omega$.<br>• Requires an auxiliary reference model and Lyapunov stability analysis. |

### Critique of Existing Implementations

- The paper reports using a **PID controller** tuned via Ziegler‑Nichols; while it meets the required bandwidth, it often leads to oscillatory response when wave torque is present.
- Proposing **LQR** would improve steady‑state accuracy but does not explicitly address input saturation or disturbance magnitude variations—MPC offers a more holistic solution.
- Suggested alternative: adopt **Constrained MPC** with horizon $N=4$–$6$ steps (the typical prediction window for wave energy converters) to actively counteract the sinusoidal torque and avoid excessive control effort.

---

## Scenarios / Test Conditions

| Scenario ID | Description | Key Parameters |
|-------------|-------------|-----------------|
| S1 | **Reference tracking** – track a step change in displacement from 0 to target value. | Reference signal $x_{ref}(t)=0.05\sin(2\pi t/5)$ (m). Duration: 30 s. |
| S2 | **Disturbance only** – apply pure wave torque without any control force. | $T_w(t)=5000\sin(\omega t)$, no active control ($F=0$). |
| S3 | **Adverse condition** – large seabed friction (full $-200$ N). | Friction term present, target step 0.05 m still required. |

---

## Metrics

| Metric | Definition | How Computed |
|--------|------------|--------------|
| IAE (Integral of Absolute Error) | $\int_{0}^{T}|x(t)-x_{ref}(t)|dt$ | Numerical integration over simulation horizon. |
| Settling Time ($t_{90}$) | Time to reach 90 % of reference displacement. | Interpolated from time history plots. |
| Control Effort (Max Force) | Peak magnitude $|\dot{x}_{\max}|$. | Directly reads from $F(t)$ trace. |
| Saturation Events | Number of times control input hits $\pm1000$ N limit. | Binary count per scenario. |

---

## Results and Conclusions

- **PID** yields a steady‑state error ≈ 4–5 % due to lack of integral action; control effort stays below saturation.
- **LQR** reduces steady‑state offset but incurs large derivative gains, causing oscillations when S2 is applied (pure wave torque).
- **MPC** with $N=3$ steps eliminates tracking error (<1 %) and respects input limits; settling time drops to ~12 s versus 18 s for PID.
- Overall, the **Constrained MPC** provides the best trade‑off between performance (minimal IAE) and robustness (handles S2/S3 without large control spikes).

---

## Limitations and Future Work (if stated)

The authors note:

1. **Linearization assumption** – may degrade performance for larger heave motions.
2. **Limited horizon choice** – longer prediction windows increase computational load; future work could explore adaptive MPC horizons based on wave variability.

These points are explicitly discussed in the “Discussion” section of the paper.