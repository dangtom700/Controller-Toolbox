# Electrostatic MEMS with Tilted Elastomeric Micro-Pillars

**Reference:** Kareem, A.H., Fathalilou, M. & Rezazadeh, G., "Mathematical modeling of an
electrostatic MEMS with tilted elastomeric micro-pillars," *Applied Mathematical Modelling*
131 (2024) 306-322. Urmia University / Skolkovo Institute of Science and Technology.

---

## Context and Motivation

Capacitive (electrostatic) MEMS are the workhorse of modern sensors and actuators: accelerometers,
pressure sensors, RF switches, scanning probe microscopes, and microphones all rely on a movable
microbeam being deflected by an applied voltage toward a fixed electrode. The gap between electrodes
determines capacitance, which is the primary measurable.

A long-standing challenge in electrostatic MEMS is the **pull-in instability**: the electrostatic
force scales as 1/(g0 - w)^2 while the restoring stiffness scales as w, so beyond a critical voltage
the beam snaps irreversibly to the counter electrode. Traditional capacitive MEMS are therefore
limited to operating below ~1/3 of the initial gap g0.

This paper introduces a microstructured gap filled with an array of **tilted PDMS
(Polydimethylsiloxane) micro-pillars**. Tilted pillars deform by bending (not axial compression),
providing a softer, more deformable gap than bulk PDMS while also increasing the effective
permittivity of the gap medium. The net effect is higher capacitive sensitivity at lower applied
voltages and a modified (extended) pull-in characteristic. Viscoelastic PDMS creep (Kelvin-Voigt
model) introduces rate-dependent softening that further changes the dynamic response.

The control challenge: the 1/(g0 - w)^2 electrostatic nonlinearity is open-loop unstable near
pull-in. A controller must maintain the gap at a desired setpoint across a wide voltage range,
reject AC disturbance forces, and adapt to viscoelastic drift in the pillar stiffness.

---

## Plant Model

The plant is discretized at Ts = 0.5 mus and simulated as a **lumped single-degree-of-freedom
(SDOF) model** of the microbeam mid-point displacement w [m], derived from the paper's first
Galerkin mode expansion. This gives an ODE that captures the essential electrostatic-mechanical
nonlinearity and viscoelastic pillar coupling.

### Equation of Motion

```
m_eff * w'' + c_eff * w' + k_beam * w + k_pillar(w, w', theta) = F_elec(w, V)
```

| Term | Expression | Physical meaning |
|------|------------|-----------------|
| m_eff | 0.396 * rho * A * L | First-mode effective mass |
| c_eff | 2 * zeta * sqrt(k_beam * m_eff) | Combined structural + squeeze-film damping |
| k_beam | 13.33 * EI / L^3 | First-mode Euler-Bernoulli bending stiffness |
| k_pillar | N_P * E_P * A_P / L_P * (cos^2(theta) + h_ratio^2 * sin^2(theta)) * (1 + tau * d/dt) | Linearised Kelvin-Voigt pillar restoring force; tau = PDMS relaxation time |
| F_elec | eps_eff * eps_0 * b * L * V^2 / (2 * (g0 - w)^2) | Electrostatic attraction (singular at w = g0) |

**Initial gap:** g0 = L_P * sin(theta).

### Microbeam Parameters (Silicon)

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Length | L | 200 mum | Beam span (double-clamped) |
| Width | b | 20 mum | Beam width |
| Thickness | h | 2 mum | Beam thickness |
| Young's modulus | E | 169 GPa | Si (110) plane |
| Poisson's ratio | nu | 0.28 | Si |
| Density | rho | 2330 kg/m^3 | Si |
| Damping ratio | zeta | 0.02 | Q approx = 25, typical air-damped |

### PDMS Pillar Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Pillar height | L_P | 10 mum | Governs initial gap via g0 = L_P*sin(theta) |
| Pillar width | b_P | 5 mum | Square cross-section |
| Young's modulus | E_P | 1.5 MPa | PDMS (soft elastomer) |
| Kelvin-Voigt relaxation | tau | 1 ms | Viscoelastic time constant |
| Relative permittivity | eps_eff | 2.8 | PDMS dielectric constant |
| Number of pillars | N_P | 20 | Distributed uniformly along beam |
| Tilt angle | theta | 30^\circ | Baseline; varied in scenarios |

### Pull-in Voltage

For the baseline geometry (theta = 30^\circ, N_P = 20), the static pull-in voltage is:

```
V_PI approx = 18.4 V   (numerical; reduced from ~24 V for the unstructured gap case)
```

The PDMS pillars soften the effective stiffness by ~25%, lowering V_PI, but extend the usable
displacement range from ~1.67 mum (1/3 of g0 without pillars) to ~3.2 mum due to the pillar
contact stiffness that prevents hard snap-in.

### Discretisation

Forward Euler integration at Ts = 0.5 mus (f_s = 2 MHz). The first undamped natural frequency
without pillar stiffening is omega_n approx = 2pi * 780 kHz, so Ts/T_n approx = 0.003 - well below the Nyquist
limit. The Kelvin-Voigt pillar term is treated semi-implicitly (implicit on the viscous rate term)
for numerical stability.

**Guard:** `w = min(w, 0.95 * g0)` prevents singularity at contact; physical contact is modelled
as a hard stop beyond 95% gap closure.

### Control Input

| Symbol | Range | Description |
|--------|-------|-------------|
| V | [0, V_max] V | Applied DC voltage (can include AC component) |

Single-input. The voltage is the only actuator; polarity inversion is not possible (electrostatic
force is always attractive). Simulation supplies the squared voltage to the force term, so the
effective control input space is u = V^2 >= 0 (convex).

---

## Scenarios

| ID | Description | theta | N_P | V_ref target | Stress |
|----|-------------|-------|-----|-------------|--------|
| s01_nominal | Regulate to 50% gap closure (w* = 2.5 mum) | 30^\circ | 20 | V = 13.1 V steady state | Baseline: nonlinear but below pull-in |
| s02_large_step | Step setpoint from 20% to 70% gap closure | 30^\circ | 20 | 0.67 -> 3.5 mum | Transient passes through 50%-pull-in regime; pull-in risk |
| s03_ac_disturbance | DC setpoint + superimposed AC disturbance: V_AC = 2 V at 100 kHz | 30^\circ | 20 | w* = 2.5 mum + vibration rejection | Resonance excitation; actuator must suppress amplitude |
| s04_pillar_aging | Relaxation time drifts: tau increases from 1 ms -> 5 ms over 10 ms (slow creep) | 30^\circ | 20 | w* = 2.5 mum | Viscoelastic softening; controller must adapt |
| s05_high_theta | Large tilt angle: smaller initial gap, lower pull-in voltage | 45^\circ | 20 | w* = 3.0 mum (g0 = 7.07 mum) | V_PI approx = 12 V; tight operating margin |

**Total runs:** 10 controllers * 5 scenarios = 50.

s02 is the most demanding: a 50 % setpoint step passes through the strongly nonlinear regime
near 60 % of pull-in, where dF_elec/dw approx = dk_eff/dw and the effective stiffness approaches
zero. Controllers without pull-in awareness drive the beam past the stability boundary.
s03 tests rejection of a near-resonant AC disturbance - the classic MEMS sensing scenario in
reverse (here the controller must reject, not amplify).

---

## Controller Roster

Each controller subclasses `mems::ControllerBase`. Its `compute(w_measured, w_ref)`
returns a voltage `V` [V]. Error convention: `e = w_ref - w_measured` [m].

Because the electrostatic force scales as V^2, most controllers compute a desired force and then
invert to get voltage: `V = sqrt(max(F_des, 0) * 2 * (g0 - w)^2 / (eps_eff * eps_0 * b * L))`.
This square-root voltage inversion is called a **feedback-linearising input transform** and is
noted per controller.

| # | Name | lib/ Algorithm(s) | Key Parameters | Design Notes |
|---|------|--------------------|----------------|--------------|
| 1 | PID | `DiscretePID` | Kp=2.0e8, Ki=1.5e10, Kd=0; e in [m] | Linearised at w*=2.5 mum; no pull-in protection; V_max clamped at 20 V |
| 2 | LinearisedPID | `DiscretePID` | Kp=1.5e8, Ki=1.0e10, Kd=0; u=V^2 | PI operates in V^2-space (linear wrt force); avoids sign-sqrt discontinuity |
| 3 | FeedbackLinearisation | `FeedbackLinearisationController` | drift f(x)=k_eff*w/m_eff; gain g(x)=eps_eff*eps0*b*L/(2*m_eff*(g0-w)^2) | Exact linearisation; inner loop integrator with Kp=4e5; requires exact g0,k_eff |
| 4 | LQR | `DiscreteLQR` (via `LQRAdapter`) | Q=diag(1e12,1); R=1e-4; linearised at w* | State feedback on {w, w'}; Nbar feedforward for zero SS error; design via `brysonMethod` |
| 5 | LQG | `DiscreteLQG` | Q_w=1e-22 m^2; R_v=1e-26 m^2 (capacitance noise) | Kalman filter estimates {w,w'} from noisy capacitance measurement DeltaC=eps_eff*eps0*b*L/(g0-w)-C0 |
| 6 | ADRC | `DiscreteADRC` | omega_o=1.2e6, omega_c=2.4e5; b0=1/(m_eff) | ESO lumps AC disturbance + pillar aging as unknown total disturbance; omega_o*Ts=0.6 - use omega_o<=1.0e6 |
| 7 | SMC | `DiscreteSMC` | c_e=1, c_de=4e-7; K=5e8; phi=1e-8 | Sliding surface s=e+c_de*de/dt; phi chosen so chattering <= 0.1 nm; V sign-preserved via abs(V) |
| 8 | MPC | `DiscreteMPC` | FOPDT linearised at w*; Np=20, Nu=5; Q=1e12, R=1e-4 | Deviation model around w*; u_dev in [-0.5V^2, +0.5V^2]; constraint: w_pred < 0.9*g0 at all horizons |
| 9 | NMPC | `NonlinearMPC` | SDOF model as StateFunc; Np=10, Nu=4; rho_y=1e12, rho_u=1e-4 | RTI; full nonlinear model inside QP; implicit pull-in avoidance via prediction horizon |
| 10 | NeuralPID | `NeuralPID` | n_h=8; lr=1e-4; Ts=0.5e-6 | Online adaptation of [Kp,Ki,Kd]; learns nonlinear mapping from gap error to gain; evaluated in s04 aging scenario |

### Key Implementation Notes

- **ADRC omega_o constraint:** With Ts = 0.5 mus, the backward-Euler ESO stability limit is
  `omega_o * Ts < 0.5` -> `omega_o < 1.0e6 rad/s`. Use omega_o = 1.0e6 exactly only with caution;
  recommended safe value is omega_o = 9.0e5.
- **Voltage sign:** The electrostatic force is always attractive (toward the fixed electrode).
  All controllers must enforce `V >= 0`. Clamp to `[0, V_max]` before applying.
- **Feedback linearisation accuracy:** `FeedbackLinearisationController` requires exact g0 and
  k_pillar. In scenarios s04/s05 (pillar aging, different theta), the controller is deliberately
  not updated - this tests robustness to model mismatch.
- **LQG capacitance measurement:** DeltaC = epsilon_eff.epsilon0.b.L/(g0 - w) - C0. This is nonlinear in w;
  the Kalman filter uses a first-order Taylor expansion at w*. Large setpoint steps (s02) cause
  estimation lag until the filter re-converges.
- **SMC chattering:** phi = 1e-8 m (10 nm boundary layer). Tighter phi drives switching at
  Nyquist; looser phi tolerates a ~0.3 nm residual tracking error. At Ts = 0.5 mus the inner
  saturation function is used - do not use signum directly.
- **NMPC StateFunc:** The SDOF EOM is wrapped as `StateFunc f(x, u)` with `x = {w, w_dot}` and
  `u = {V}`. The electrostatic singularity is clamped at w < 0.9.g0 inside the prediction to
  prevent NaN during QP iterations with large trial voltages.

---

## Metrics

Each run prints and logs:

```
w_final=m  IAE=m*s  ISE=m^2*s  V_mean=V  settling_us=us  max_error=m  pull_in=bool
```

CSV logs written to `case-study/Electrostatic MEMS with Tilted Micro-Pillars/logs/`.

Primary ranking metric: **IAE** (integral absolute error on gap closure [m]).
Secondary: settling time (mus to within 1% of reference), `pull_in` flag (controller failure),
and voltage mean (energy proxy).

A `pull_in = true` result means the controller drove the beam to the guard wall (w >= 0.95.g0)
and the simulation was terminated. This counts as a failed run for the scenario.

---

## Build and Run

```bash
conda run -n soft_robotics -- python run.py
```

The `mems_sim` target is built by `compile.bat` and discovered automatically by `run.py`.
Expected: 50 runs (10 controllers * 5 scenarios).

Individual run:

```bash
build\case-study\"Electrostatic MEMS with Tilted Micro-Pillars"\mems_sim.exe
```

Logs written to `case-study/Electrostatic MEMS with Tilted Micro-Pillars/logs/`.

> **Note:** This case study is currently a design specification. The C++ simulation source
> (`sim/src/mems_plant.cpp`, `sim/src/controllers.cpp`, `sim/src/main.cpp`) and CMakeLists
> are not yet implemented. The README documents the intended plant model, scenarios, and
> controller roster for the future implementation.
