# Separate Meter-In Separate Meter-Out Hydraulic Actuator Control

## References

1. **Guangrong Chen, Junzheng Wang, Shoukun Wang, Jiangbo Zhao, Wei Shen (2018).**
   "Energy saving control in separate meter in and separate meter out control system."
   *Control Engineering Practice* 72, 138-150.
   (PDF: `Energy-saving-control-in-separate-meter-in-and-separ_2018_Control-Engineerin.pdf`)
   Primary source for the plant equations, rig parameters (Table 1), identified Stribeck
   friction, the IARDSC tracking controller, and the two energy-saving techniques
   (disturbance-observer supply-pressure control + grey-predictor supply-flow control).

2. **Yingjie Liu, Bing Xu, Huayong Yang, Dingrong Zeng (2009).**
   "Modeling of Separate Meter In and Separate Meter Out Control System."
   *2009 IEEE/ASME International Conference on Advanced Intelligent Mechatronics*, 227-232.
   (PDF: `Modeling_of_separate_meter_in_and_Separate_Meter_out_control_system.pdf`)
   Source for the experimentally verified component models (pump, relief valve, low-DP
   valve characteristic), the friction identification method, and the dual-loop controller
   structure (velocity loop on the working valve + back-pressure feedforward/feedback loop
   on the off-side valve, Fig. 10).

---

## Plant Model

A **Separate Meter-In Separate Meter-Out (SMISMO)** hydraulic circuit: a single-rod
double-acting cylinder whose two chambers are each served by an **independent
proportional directional control valve** (PDCV1 on the cylinder-end/cap chamber, PDCV2
on the rod-end chamber). Decoupling the metering-in and metering-out orifices removes
the mechanical linkage of a conventional 4-way valve and opens a second control degree
of freedom that can be spent on energy saving (low back-pressure regulation).

### Physical System (Chen et al. 2018, Fig. 1)

- Single-rod hydraulic cylinder with hanging inertia load (vertical motion)
- Two proportional directional control valves (PDCV1: cap side, PDCV2: rod side),
  each able to connect its chamber to **either supply or tank** depending on spool sign
- Fluid source: fixed-displacement pump driven by a servo motor + proportional relief
  valve (supply pressure regulation)
- Pressure transducers on supply, cap chamber, and rod chamber; displacement sensor
  on the load

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `x_L(t)` | Load (piston) position | m |
| `v_L(t)` | Load velocity | m/s |
| `P_1(t)` | Cap (cylinder-end) chamber pressure | Pa |
| `P_2(t)` | Rod-end chamber pressure | Pa |
| `x_v1, x_v2` | PDCV1/PDCV2 spool positions (2nd-order valve dynamics) | - |

### Governing Equations

**Load dynamics (Chen Eq. 2, Newton):**
```
P_1*A_1 - P_2*A_2 = m * dv_L/dt + F_f(v_L) + F + Delta_f
```
where `A_1`, `A_2` = cap and rod-side areas, `F_f(v_L)` = identified Stribeck friction,
`F` = constant load force (gravity, `F = m*g`), `Delta_f` = external disturbance.

**Identified Stribeck friction (Chen Sec. 5.1, [N]):**
```
F_f(v) =  68 + 13*v + 11*exp(-|3v/0.5|)    v > 0
F_f(v) = -79 + 24*v - 16*exp(-|3v/0.6|)    v < 0
```
(Liu et al. 2009 use the simpler `f = f_0 + k*v` with f_0 = 200 N, k = 980 N.s/m for
their rig - same identification procedure, Fig. 6.)

**Pressure dynamics (Chen Eq. 3, continuity):**
```
(V_1(x)/beta_e) * dP_1/dt = Q_1 - A_1*v_L + DQ_10
(V_2(x)/beta_e) * dP_2/dt = Q_2 + A_2*v_L + DQ_20
```
with chamber volumes `V_1(x) = V_10 + A_1*x_L`, `V_2(x) = V_20 - A_2*x_L`.
`Q_1` is the (signed) supply flow **into** the cap chamber through PDCV1, `Q_2` the
(signed) ingress flow **into** the rod chamber through PDCV2; `DQ_10/DQ_20` lump
internal/external leakage (neglected in the sim, per Liu hypothesis 5).

**Valve orifice flow (Chen Eqs. 4-5, four-quadrant):**
```
Q_i = C_d * W * x_vi * sqrt(2*DP_i/rho),   DP_i = { P_s - P_i   if x_vi >= 0   (from supply)
                                                  { P_i - P_r   if x_vi <  0   (to tank)
```
The simulation uses the equivalent rated-flow normalisation (Liu Eq. 14):
`Q_i = xbar_vi * Q_nom_i * sqrt(DP_i / DP_nom)` with normalised spool `xbar_vi in [-1,1]`,
rated flow `Q_nom` at rated pressure drop `DP_nom = 3.5 MPa`
(`C_d*W*x_v,max = Q_nom / sqrt(2*DP_nom/rho)`).

**Valve spool dynamics (Chen Eq. 7 / Liu Eq. 2, 2nd-order):**
```
x_vi(s)/u_i(s) = k_vi * w_vi^2 / (s^2 + 2*xi_vi*w_vi*s + w_vi^2)
PDCV1: xi_v1 = 0.70, w_v1 = 86.2 rad/s      PDCV2: xi_v2 = 0.68, w_v2 = 91.4 rad/s
```

**Supply (Liu Eqs. 1, 7, 9 - modeled as ideal constant-pressure source in the sim):**
```
q_p = n*D - k_leak*p_s                      (pump)
q_r = (p_s - (i_r/i_c)*p_crack - p_t)*grad  (relief valve)
```

### Key Parameters (Chen et al. 2018, Table 1)

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Supply pressure | P_s | 60 bar (6 MPa) | Relief-valve regulated |
| Tank pressure | P_r | 0 bar | Reference |
| Cap-side area | A_1 | 4.91e-4 m^2 | |
| Rod-side area | A_2 | 2.9e-4 m^2 | |
| Initial chamber volumes | V_10, V_20 | 1.0e-3 m^3 | |
| Load mass | m | 50 kg | Hanging inertia load |
| Effective bulk modulus | beta_e | 890 MPa | |
| Discharge coefficient | C_d | 0.62 | |
| Spool area gradient | W | 0.0314 m | |
| Oil density | rho | 870 kg/m^3 | |
| Desired backpressure | P_bd | 20 bar (2 MPa) | Off-side setpoint |
| Gravity | g | 9.8 m/s^2 | F = m*g = 490 N |
| Rated valve flow | Q_nom | 40 L/min @ 3.5 MPa | Liu rig (PDCV2 scaled by k_v2/k_v1 = 0.67) |
| Sampling time | Ts | 1 ms | Chen experiments |

(The Liu 2009 rig differs: 50/25 mm cylinder -> A_1 = 1963 mm^2, A_2 = 1473 mm^2,
beta_e = 9000 bar, valve w_n = 40 Hz, zeta = 0.8, p_crack = 150 bar, backpressure 8 bar.)

---

## Control Objective

Dual objectives (Chen Sec. 2):

1. **Tracking (working side):** the load position `x_L` tracks the desired trajectory
   `x_Ld` despite parametric uncertainty, friction, and external disturbance. Paper
   trajectory: `x_Ld = 0.25 + 0.25*sin(pi*t/2 - pi/2)` m; 500 N disturbance added at t = 9 s.
2. **Backpressure regulation (off-side):** keep the off-side (discharging) chamber at a
   low constant backpressure `P_bd = 20 bar` to minimise throttling loss while preventing
   cavitation. The working/off-side roles swap with motion direction (Liu & Yao 2002
   working-mode selection).

**Energy saving (Chen Sec. 4).** Hydraulic energy `E = Int P_s(t)*Q_s(t) dt` (Eq. 1).
Throttling loss `DW = DW_1 + DW_2 = v_L*A_1*(P_s - P_1) + v_L*A_2*(P_2 - P_r)` (Eq. 38);
minimising over P_s subject to the force balance gives the minimal supply pressure
```
P_s,min = (A_2/A_1)*P_r + P_L/A_1 + DP_s          (Eq. 41, P_L = load force)
```
realised by `P_s = k_f*|f_d_hat| + k_v*|v_L|` (Eq. 37) with a 2nd-order disturbance
observer for f_d (Eq. 29-30; k_f = 0.0048 MPa/N, k_v = 1.3 MPa/(m/s)), and the supply
flow rate is matched to demand by a GM(1,1) grey predictor driving the pump servo speed
`n = (Q_S + C_P*P_s)/D + Dn` (Eq. 52). Experimentally: pressure control saves ~1/3,
flow control ~2/3, both ~5/6 of the power.

The simulation keeps `P_s` constant (60 bar) and logs `E = Int P_s*Q_s dt` so the
controller roster is compared at fixed supply; the s05 scenario makes the per-cycle
energy the primary metric.

---

## Controller Roster (12 implemented)

All controllers command the **working-side valve** (scalar `u_ctrl in [-10, 10] V`,
sign = motion direction). A shared mode-selection + backpressure-PI allocator (the
Liu Fig. 10 "controller2" loop, generalised to both directions) maps `u_ctrl` to the
two valve commands `(u_1, u_2)` and regulates the off-side chamber to `P_bd`.

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | PID | `DiscretePID` | Kp=60, Ki=40, Kd=4, N=10 | Position loop; ~8 rad/s crossover (plant gain ~0.14 (m/s)/V) |
| 2 | CascadePID | `DiscretePID` x2 | Outer P: Kp=8 (1/s), v_ref clamp +/-0.45 m/s; inner PI: Kp=10, Ki=100 | Position outer / velocity inner (velocity from plant state); inner crossover ~39 rad/s under the valve lag |
| 3 | LQR | `DiscreteLQR` (design) | Bryson: x_max=[0.02 m, 0.4 m/s], u_max=10 V | 2-state design model [x_L, v_L]; u = -K(x - x_ref) |
| 4 | LQG | `DiscreteLQG` | Same Bryson Q/R; R_kf=1e-8 (0.1 mm sensor) | Kalman filter estimates v_L from position only |
| 5 | MPC | `DiscreteMPC` | Np=60, Nc=5, rho_y=1, rho_u=0.01, du +/-2 V | 2-state ZOH model; 60 ms prediction at Ts=1 ms |
| 6 | ADRC | `DiscreteADRC` | omega_o=200, omega_c=30, b0=5.6 | 2nd-order LADRC; omega_o*Ts=0.2<0.5; b0 = K_v/tau_v |
| 7 | SMC | `DiscreteSMC` | c_e=1, c_de=50 (lead 0.05 s), K=4, phi=0.05 | compute(y - ref); robust to Stribeck friction |
| 8 | FeedbackLinearisation | `FeedbackLinearisationController` | g(x) = K_q/(10*A)*sqrt(DP) direction-dependent; inner PID -> v_cmd | Compensates sqrt(DP) valve-gain variation (Liu calc-flow control) |
| 9 | TubeMPC | `TubeMPC` | Np=10, Nu=3, wMax=[1e-4, 5e-3], K=-K_lqr | Robust to load-pressure model error |
| 10 | L1Adaptive | `L1AdaptiveController` | a_m=exp(-5Ts), Gamma=100, omega_c=20 | setReference(x_ref) + compute(x_L); adapts to K_q variation |
| 11 | GainScheduled | `GainScheduledController` | 3 PIDs scheduled on v_L in {-0.3, 0, +0.3} | Resistive vs. overrunning regimes |
| 12 | NonlinearMPC | `NonlinearMPC` | Np=12, Nu=3, rho_u=0.05; internal 10 ms model step | RTI on 2-state nonlinear model with flow-saturation (tanh) nonlinearity |

---

## Scenarios

| ID | Description | Reference | Load Profile |
|----|-------------|-----------|--------------|
| s01_resistive_step | Step extension against resistive load | x_ref: 0.05 -> 0.25 m at t=0.5 s | F_ext = +500 N constant |
| s02_overrunning | Step extension, load assists motion (cavitation risk) | x_ref: 0.05 -> 0.35 m at t=0.5 s | F_ext = -800 N (overrunning) |
| s03_sine_tracking | Paper trajectory x_Ld = 0.25 + 0.25*sin(pi*t/2 - pi/2) | 16 s (4 cycles) | F_ext = 0 (gravity only) |
| s04_load_step | Position hold, paper disturbance test | x_ref = 0.25 m hold | F_ext: 0 -> +500 N at t=9 s |
| s05_energy_compare | 3-cycle paper sine; energy E = Int P_s*Q_s dt is the metric | 12 s sine | F_ext = +300 N |

**Total runs:** 12 controllers * 5 scenarios = 60. Target: `smismo_sim`.

---

## Implementation Notes

- **Integration:** RK4 at Ts = 1 ms with 4 substeps (dt = 0.25 ms). 8 plant states:
  [x_L, v_L, P_1, P_2, x_v1, dx_v1, x_v2, dx_v2]. Hydraulic natural frequency
  ~75 rad/s at mid-stroke - not stiff at this step size.
- **Mode selection + hysteresis:** extend if u_ctrl > +0.05 V, retract if < -0.05 V,
  else hold mode. EXTEND: u_1 = u_ctrl (cap from supply), PDCV2 = backpressure PI
  discharging rod chamber to tank (u_2 <= 0). RETRACT: u_2 = -u_ctrl (rod from supply),
  PDCV1 = backpressure PI discharging cap chamber (u_1 <= 0). The off-side PI gets a
  flow-matching feedforward `u_ff = 10*A_off*|v_L| / (K_q,off*sqrt(P_bd))` (Liu Fig. 10:
  feedforward + feedback pressure control) and bumpless integrator reset on mode switch.
- **Stribeck regularisation:** the identified friction law is discontinuous at v = 0;
  for |v| < 5e-3 m/s the sim interpolates linearly between F_f(+eps) = +79 N and
  F_f(-eps) = -95 N (static friction band).
- **sqrt(DP) regularisation:** orifice flow uses `dp/sqrt(|dp| + 1e3)` (signed,
  smooth at dp = 0, allows reverse flow / anti-cavitation backfill from tank).
- **Cavitation/limits:** P clamped to [0, 50 MPa]; piston end stops at x in [0, 0.5 m]
  (velocity zeroed on contact). Backpressure regulation to P_bd = 20 bar is what keeps
  the overrunning s02 scenario cavitation-free - do not lower P_bd below ~5 bar.
- **Energy accounting:** Q_s = Q_1*[x_v1>0] + Q_2*[x_v2>0] (flow drawn from supply by
  whichever valve is connected to it); E += P_s*Q_s*dt each substep.
- **ADRC omega_o:** Ts = 1 ms requires omega_o < 500 rad/s (omega_o*Ts < 0.5).
  Implemented: omega_o = 200, omega_c = 30, b0 = K_v/tau_v ~ 5.6 (m/s^2)/V.
- **Working-side plant gain (for tuning):** v_L/u ~ K_q1*sqrt(P_s - P_1)/(10*A_1)
  ~ 0.14 (m/s)/V at nominal load pressure; velocity lag tau_v ~ 25 ms (valve + hydraulics).
- **CSV columns:** `t,x_ref,x_p,v_p,P1_bar,P2_bar,u1,u2,F_ext,Q_s_lpm,energy_J,iae_cumulative`

---

## Status

**Implemented (Part 44).** `sim/{include,src}` present, registered in
`case-study/CMakeLists.txt` (`smismo_sim`) and `compile.bat`; regression guard
`tests/test_smismo_regression.cpp`. 12 controllers x 5 scenarios = 60 runs; CSV
telemetry to `logs/`.
