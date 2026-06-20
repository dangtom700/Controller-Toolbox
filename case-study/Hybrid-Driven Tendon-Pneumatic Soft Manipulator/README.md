# Adaptive Kinematic and Stiffness Control of a Hybrid-Driven Soft Manipulator

## Reference

Xin Fu, Daohui Zhang, Naijia Xu, Shuheng Ren, Yaqi Chu, Dezhen Xiong, Xingang Zhao (2025).
"Adaptive kinematic and stiffness control of a hybrid-driven soft robot for enhanced
interactions under external loads." *Results in Engineering* 28, 107955.
https://doi.org/10.1016/j.rineng.2025.107955

> Corrects the previous revision of this README, whose author list ("Wenhao Fu, Pengbo Liu,
> Yang Li, Xin Li, Zhongbo Sun") and entire "Governing Equations" section (a second-order
> curvature ODE with inertia `I_s`, damping `B_s`, and a pneumatic pressure-buildup ODE) were
> not present in the source PDF and do not match it - verified directly against the paper's
> own text (`adaptive kinematic and stiffness control....txt`, the PyMuPDF extraction sitting
> next to the PDF in this folder) for this rewrite. The real paper has **no dynamic ODE at
> all** - it is a quasi-static kinematic + stiffness framework (PCC kinematics + a learned
> minimum-pressure predictor + a Jacobian-based stiffness model), not a curvature-dynamics
> control problem. See `HANDOFF_PROMPT.md` in this folder for what implementing this as a
> Controller Toolbox case study (which does need a discrete-time "plant" of some kind) requires.

---

## Plant Model

A **two-segment hybrid-driven soft manipulator** (Fig. 1 of the paper): two silicone bellow
segments in series, each actuated by 3 tendons (the proximal segment's skeleton has 6 tendon
channels - 3 for its own actuation, 3 pass-through for the distal segment - giving 6
independently motor-driven tendons in total) plus a shared pneumatic chamber per segment.
Tendon tension sets the segment's bending/elongation; chamber pressure independently sets the
segment's *stiffness* without materially changing its rest shape. There is **no inertial
dynamics model anywhere in the paper** - every relationship below is an algebraic (kinematic
or quasi-static elastic) map; bending/elongation/stiffness are all treated as reaching
equilibrium fast relative to the control loop, consistent with the paper's own framing of
soft-body deformation as instantaneous compliance, not a mass-spring-damper response.

### Physical Description

- **Tendon channel:** 3 tendons per segment (`l_k1, l_k2, l_k3`, `k` = segment index)
  bend/elongate the segment via Eq. (15)/(16) below; tendons are inextensible, routed through
  a 3D-printed base by 6 DC motors (Dynamixel XM430-W350-R) and winding wheels.
- **Pneumatic channel:** one chamber per segment; gauge pressure `P_k` is regulated by a
  proportional valve (EPV2-50MD2, ~10 ms response) and raises the segment's bending/axial
  stiffness roughly linearly (Eq. 22) without itself driving the nominal shape - this is the
  paper's central "actuation-stiffness decoupling" idea. Max permissible pressure in the
  paper's own rig: 150 kPa (1.5 bar gauge) for safety; minimum driving pressure (pressure
  needed just to keep tendons taut for a given configuration) was characterised up to 0.5 bar.
- **WBLS predictor:** a Weighted Broad Learning System (a shallow random-feature network with
  ridge-regression readout, *not* a deep net) learns `P_min(theta_k, phi_k)` - the minimum
  pressure needed to sustain a configuration without tendon slack - from an offline
  characterisation dataset (n=10 feature nodes, m=20 enhancement nodes, Sigmoid activation).
  Reported accuracy: average bending-angle prediction error 1.2 deg.
- **External load:** a lumped mass at the distal tip (tested at 0 g, 100 g, 200 g) deflects
  the tip away from its commanded (no-load) position by an amount set by the *current*
  Cartesian stiffness `K_X` - i.e. the same load produces a smaller deflection at higher
  chamber pressure. Validated stiffness-variation ratio: **>3.6x** (axial) between 0.9 bar and
  1.5 bar gauge.
- **Headline experimental result:** under a 200 g tip load, WBLS-driven adaptive pressure
  control tracked the reference trajectory with comparable accuracy to a fixed-1.5-bar
  baseline while running at an average pressure of 1.02 bar - a **31.83% pressure reduction**.

### Configuration, task, and actuator spaces (paper's full generality)

| Space | Symbol | Per-segment contents |
|---|---|---|
| Task space | `X(t) = [x(t), y(t), z(t)]^T` | end-effector Cartesian position |
| Configuration space | `q_k = [theta_k, phi_k, gamma_k]^T` | bending angle [rad], deflection-plane angle [rad], segment length [m] |
| Actuator space | `[l_k1, l_k2, l_k3]`, `P_k` | 3 tendon lengths [m] + chamber pressure [Pa] |

`h` = cross-section radius [m] (tendon routing radius within the segment); not given a
numeric value in the extracted text - treat as an assumed/measured fabrication parameter,
flagged in `HANDOFF_PROMPT.md`.

### Governing Equations (verified against the paper's own equation numbers 1-27)

**1. Weighted Broad Learning System (WBLS) - minimum-pressure predictor**

Training set `{x_i, y_i}`, `x_i = [theta_i, phi_i]^T` (a configuration), `y_i = P_min_i`.
```
Z_i = phi(X W_ei + beta_ei),     i = 1..n          (feature nodes)      [Eq 1]
H_j = xi(Z^n W_hj + beta_hj),    j = 1..m          (enhancement nodes)  [Eq 2]
A = [Z^n | H^m] ;  Y = A W                                              [Eq 3]
```
`W_ei, beta_ei, W_hj, beta_hj` are random weights/biases drawn from `[-zeta, zeta]`; `phi`,
`xi` are Sigmoid. Connection weights `W` solved by **weighted ridge regression**, robust to
outliers via Huber weighting:
```
W* = argmin_W  || sigma*A*W - sigma*Y ||_2^2 + lambda*||W||_2^2          [Eq 4]

sigma_i = 1            if |u_i| <= b                                    [Eq 5]
        = b / |u_i|    if |u_i| >  b
u_i = r_i / zeta_hat                                                    [Eq 6]
zeta_hat = 1.4826 * MAD = 1.4826 * median(|r_i - median(r)|)            [Eq 7]

W = (lambda*I + A^T*sigma^2*A)^-1 * A^T*sigma^2*Y                       [Eq 8]
P_min* = f_WBLS(X) = [Z_1*, .., Z_n*, H_1*, .., H_m*] * W                [Eq 9]
```
`r_i` = residual between the network's prediction and the measured `P_min` for sample `i`.

**2. Piecewise Constant Curvature (PCC) kinematics, robot-independent (per segment k)**
```
p = [ (gamma_k/theta_k)*cos(phi_k)*(1-cos(theta_k)),
      (gamma_k/theta_k)*sin(phi_k)*(1-cos(theta_k)),
      (gamma_k/theta_k)*sin(theta_k) ]^T                                [Eq 10]

R = Rz(phi_k) * Ry(theta_k) * Rz(-phi_k)
  = [[ 1 + cos(phi_k)^2*(cos(theta_k)-1),   sin(phi_k)*cos(phi_k)*(cos(theta_k)-1),   cos(phi_k)*sin(theta_k) ],
     [ sin(phi_k)*cos(phi_k)*(cos(theta_k)-1),   1 + sin(phi_k)^2*(cos(theta_k)-1),   sin(phi_k)*sin(theta_k) ],
     [ -cos(phi_k)*sin(theta_k),   -sin(phi_k)*sin(theta_k),   cos(theta_k) ]]                              [Eq 11]

T_k^{k-1} = [[ R(3x3),  p(3x1) ], [ 0(1x3), 1 ]]                         [Eq 12]
T_2^0 = T_1^0 * T_2^1                          (two segments in series)  [Eq 13]
Xdot(t) = J(q(t)) * qdot(t)                    (kinematic Jacobian)      [Eq 14]
```
`c(.)`/`s(.)` abbreviate `cos(.)`/`sin(.)` in the source. As `theta_k -> 0` (straight
segment), `(1-cos(theta_k))/theta_k -> 0` and `sin(theta_k)/theta_k -> 1` (both well-behaved
limits - L'Hopital / Taylor, not a 0/0 blow-up at first order, but a naive float divide by a
`theta_k` that underflows to exactly 0.0 will still NaN; guard it, see Implementation Notes).

**3. Actuator-space <-> configuration-space mapping (robot-specific, per segment k)**
```
theta_k = 2*sqrt(l_k1^2 + l_k2^2 + l_k3^2 - l_k1*l_k2 - l_k2*l_k3 - l_k1*l_k3) / (3*h)
phi_k   = atan2( sqrt(3)*(l_k2 + l_k3 - 2*l_k1),  3*(l_k2 - l_k3) )
gamma_k = (l_k1 + l_k2 + l_k3) / 3                                       [Eq 15]

l_ki = gamma_k - h*theta_k*cos( delta_k + 2*pi*(i-1)/3 - phi_k ),  i in {1,2,3}
delta_1 = pi/2,  delta_2 = 5*pi/6        (per-SEGMENT mounting offset, k=1,2 - there is
                                           no delta_3; the manipulator has only 2 segments)
                                                                          [Eq 16]
```

**4. External-load deflection and stiffness model**
```
X = f_hat(q)                                     (no-load kinematics)    [Eq 17]
f(q) = X + dX = f_hat(q) + K_X^-1 * F_ext         (loaded kinematics)    [Eq 18]

K_q = d(J^T*K_X*dX)/dq = J^T*K_X*J - (dJ/dq)*K_X*dX                      [Eq 19]
K_q ~= J^T*K_X*J                    (small-deformation / static-equilibrium approx)
                                                                          [Eq 20]
K_X    = J^{+T} * K_q * J^+
K_X^-1 = J * K_q^-1 * J^T            (pseudoinverse, non-square J)       [Eq 21]

k_bending(dP)     = k1_Pmin + g1_Pmin * dP
k_compression(dP) = k2_Pmin + g2_Pmin * dP        dP = P - P_min          [Eq 22]

U(P) = (1/2)*k_bending(dP)*theta^2 + (1/2)*k_compression(dP)*gamma^2     [Eq 23]

K_q' = grad^2(U) = A * dP'          dP' = [1, dP]^T  (augmented pressure)
K_q  = diag(K_q')                  K_q > 0 (must stay positive-definite) [Eq 24]
```

**5. Coordinated motion + stiffness optimisation (the controller, Sec. 5)**
```
argmin_{dq, dP}  || dX - J_q*dq ||_2^2  +  nu * || J_s*[dq; dP] ||_2^2

subject to:
  X_c     = f(q) + K_X^-1 * F_ext
  K_X^-1  = J(q) * K_q^-1 * J(q)^T
  K_q     = diag(A * dP')
  P_new   = P + dP
  q_new   = q + dq                                                       [Eq 25]
```
`J_q` = kinematic Jacobian, `J_s` = sensitivity Jacobian of the stiffness model, `nu > 0`
trades off position accuracy against stiffness change (`nu = 0` collapses to plain inverse
kinematics with no stiffness compensation).

**6. Online adaptive stiffness update (error-feedback, no direct force sensing needed)**
```
dX_n = X_d - X_n                                  (n = control step)
K_q^{n+1} = K_q^n + eta * J_n^+ * dX_n                                   [Eq 26]

argmin_A  ||dA||_2
subject to:  K_q' = A^{n+1} * dP',   A^{n+1} = A^n + dA,   K_q > 0       [Eq 27]
```
`||dA||_2` = Frobenius norm of the update, kept small so the stiffness-pressure coefficient
estimate `A` drifts smoothly rather than jumping step to step.

### Key Parameters

| Parameter | Symbol | Value | Source |
|---|---|---|---|
| Number of segments | - | 2 | Sec. 2.1 |
| Tendons per segment | - | 3 (6 motors total, distal segment's pass through the proximal's 6 channels) | Sec. 2.1 |
| WBLS feature / enhancement nodes | `n`, `m` | 10, 20 | Sec. 4.2 |
| WBLS activation | `phi, xi` | Sigmoid | Sec. 4.2 |
| Segment mounting offsets | `delta_1, delta_2` | pi/2, 5pi/6 rad | Eq. 16 |
| Max chamber pressure (safety limit) | `P_max` | 150 kPa (1.5 bar gauge) | Sec. 6.1 |
| Characterised min-driving-pressure range | `P_min` | 0 - 0.5 bar gauge | Sec. 4.1 |
| Stiffness characterisation pressures | `P` | 0.9, 1.2, 1.5 bar gauge | Sec. 6.2 |
| Axial stiffness variation ratio | `k_max/k_min` | > 3.6x (0.65 N / 4 mm @ 0.9 bar -> 2.35 N / 4 mm @ 1.5 bar) | Sec. 6.2 |
| Tested tip payload | `m_L` | 0, 100, 200 g | Sec. 6.3 |
| Average pressure under 200 g load (adaptive vs. fixed 1.5 bar) | - | 1.02 bar (-31.83%) | Sec. 6.3 |
| WBLS bending-angle prediction error | - | 1.2 deg average | Sec. 4.2 |
| Uniaxial test module length | - | 166 mm | Sec. 4.1 (one segment's tested free length - not necessarily the deployed `gamma_k`) |
| Valve response time | - | ~10 ms | Sec. 2.1 |
| Stereo camera calibration error | - | 0.08 mm | Sec. 4.1 |

`h` (cross-section/tendon-routing radius) and the per-bar `k1_Pmin, g1_Pmin, k2_Pmin,
g2_Pmin` regression coefficients of Eq. 22 are **not tabulated numerically anywhere in the
extracted text** (the paper presents them as a characterisation plot, Fig. 9, not a table) -
these must be assumed/fit, see `HANDOFF_PROMPT.md`.

---

## Control Objective

The paper's own controller (Sec. 5) jointly solves for a configuration update `dq` and a
pressure update `dP` every step (Eq. 25), using the adaptive stiffness law (Eq. 26/27) to
keep its internal `K_q` estimate honest as pose and load change, and the WBLS predictor (Eq.
9) to seed `P_min` for the current configuration. This case study benchmarks that approach
against generic library controllers on three goals:

1. **Trajectory tracking** - drive the tip through prescribed paths (spiral, circle, P2P)
   despite the load-dependent elastic droop `K_X^-1 * F_ext` (Eq. 18).
2. **Stiffness/pressure efficiency** - achieve that tracking accuracy at the *lowest*
   pressure consistent with the load (the paper's own 31.83% headline number), not just by
   running stiff (high pressure) all the time.
3. **Load adaptation** - keep tracking error bounded as the tip payload changes (0/100/200 g,
   including a mid-trajectory pickup) without a direct force sensor, using only the
   measured position-error feedback that drives Eq. 26.

See `HANDOFF_PROMPT.md` for how this gets reduced to a concrete discrete-time "plant" that
the toolbox's `IController` roster can actually run against - the paper itself has no
control-loop plant in the sense every other case study in this repo uses (no ODE, no
fixed sample-and-hold actuator dynamics), so that reduction is itself a real design decision,
not a transcription step.

---

## Proposed Controller Roster (12)

All entries assume the per-segment, planar-bending reduction in `HANDOFF_PROMPT.md` Section
1 (`phi_k = 0`, `gamma_k` fixed, `theta_1`/`theta_2` are the only actively tracked DOF; `P_1`,
`P_2` are the stiffness DOF). Library class names verified against `lib/` headers for this
rewrite (the previous revision had two wrong: it called these `FeedbackLinearisationController`
and `ILCController` - the real classes are `ctrl::FeedbackLinearisation` and
`ctrl::IterativeLearningControl`).

| # | Name | lib/ Algorithm | Pressure policy | Design Notes |
|---|------|---------------|---|--------------|
| 1 | OpenLoop | - | Fixed 1.5 bar | `theta_k_cmd` held at the scenario's initial reference; baseline |
| 2 | PID | `DiscretePID` | Fixed 1.5 bar (paper's own rigid-stiffness baseline pressure) | One instance per segment; `compute(theta_k_ref - theta_k)` |
| 3 | ADRC | `DiscreteADRC` | Fixed 1.5 bar | One instance per segment; `compute(theta_k_ref - theta_k)`; check `omega_o*Ts < 0.5` |
| 4 | SMC | `DiscreteSMC` | Fixed 1.5 bar | One instance per segment; `compute(theta_k - theta_k_ref)` per this repo's SMC sign convention |
| 5 | LQR | `makeLQRController` factory | Fixed 1.5 bar | Linearised single-segment theta-loop (the plant is already a static map plus a load bias - "A" matrix is just the identity/gain term, see HANDOFF_PROMPT.md) |
| 6 | MPC | `DiscreteMPC` | Fixed 1.5 bar | Same linearised per-segment model as LQR; `P_k` held fixed so MPC's only decision is `theta_k_cmd` |
| 7 | MRAC | `MRACController` | Fixed 1.5 bar | `set_reference(theta_k_ref)` then `compute(theta_k)` (this plant is positive-gain: more commanded `theta_k` -> more achieved `theta_k`) |
| 8 | L1Adaptive | `L1AdaptiveController` | Fixed 1.5 bar | Same `set_reference`/`compute` pattern as MRAC |
| 9 | FeedbackLinearisation | `FeedbackLinearisation` | Fixed 1.5 bar | Plant is already algebraic (no inertia to invert) - degenerates to an exact one-step inverse of the load-bias term; documented as a near-trivial case for this plant, see HANDOFF_PROMPT.md |
| 10 | GainScheduled | `GainScheduledController` | Fixed 1.5 bar, 3-point schedule by `m_L_est` | 3 PID sets tuned for 0g/100g/200g; bumpless transfer at threshold |
| 11 | ILC | `IterativeLearningControl` | Fixed 1.5 bar | P-type; one trial = one full spiral revolution; natural fit for the repeated-revolution scenarios |
| 12 | **AdaptiveWBLSCtrl** (new, local class - the paper's own method) | n/a (case-study-local, like SMISMO's `DOBEnergyCtrl`) | **Adaptive**, via Eq. 9 WBLS seed + Eq. 26/27 online update | Implements Eq. 25-27 directly; the only controller that touches `P_k`. This is the comparison point for the paper's "31.83% less pressure, comparable MSE" result |

Removed from the previous revision: `NeuralPID`, `CBFSafetyFilter`, `DynaController` (12 slots
are already spoken for by the roster above without inventing a curvature-dynamics role for
them; nothing stops a future session from swapping one in if a 13th comparison point is
wanted, but `CBFSafetyFilter`'s natural job here - a hard barrier on `P_max` - is already
covered structurally since every controller here holds pressure fixed below `P_max` except
controller 12, which already respects `K_q > 0`/`P <= P_max` inside its own optimisation).

---

## Scenarios (5)

| ID | Description | Reference | Load |
|----|-------------|-----------|------|
| s01_spiral_no_load | Spiral tip trajectory, 3 revolutions over 30 s | Archimedean spiral, planar reduction of the paper's helical path | `m_L` = 0 g |
| s02_spiral_100g | Same spiral | Same | `m_L` = 100 g |
| s03_spiral_200g | Same spiral (heaviest tested condition) | Same | `m_L` = 200 g |
| s04_stiffness_sweep | Static pose (no tracking motion); commanded pressure ramps 0.9 -> 1.5 bar at fixed `theta_1, theta_2` | constant | `m_L` = 0 g, with a swept external force probe instead - reproduces the paper's own Sec. 6.2 stiffness-characterisation test and should recover the >3.6x ratio |
| s05_pick_and_place | P2P between 3 waypoints, dwell 2 s each | 3 waypoints | `m_L` steps 0 -> 100 g at the second waypoint (tool pickup), tests transient load-adaptation response |

**Total runs:** 12 controllers x 5 scenarios = 60.

---

## Implementation Notes

- **No ODE - the "plant" is a quasi-static algebraic map.** Every other discrete-time
  case study in this repo has a real ODE/difference equation for `Plant::step()`; this one
  doesn't, by the paper's own design. `HANDOFF_PROMPT.md` works through the specific
  reduction used here (per-segment `theta_k` loop with a load-dependent elastic bias term
  derived from Eq. 19/20, not invented).
- **PCC singularity at `theta_k -> 0`:** Eq. 10's `(1-cos(theta_k))/theta_k` and
  `sin(theta_k)/theta_k` terms are analytically well-behaved (limits `0` and `1`
  respectively) but will divide-by-zero in floating point at exactly `theta_k = 0`. Guard
  with a small-angle Taylor branch, e.g. `if |theta_k| < 1e-6` use the limiting values
  directly.
- **`phi_k` formula uses `atan2`, not `atan`:** Eq. 15's deflection angle is
  `atan2(sqrt(3)*(l_k2+l_k3-2*l_k1), 3*(l_k2-l_k3))` - using plain `atan` on the ratio (as one
  of the source extraction attempts for this README incorrectly did) loses the quadrant and
  will silently give the wrong deflection plane whenever `l_k2 < l_k3`.
- **Rotation matrix sign convention (Eq. 11):** the off-diagonal `(1,3)`/`(3,1)` entries are
  `+cos(phi_k)*sin(theta_k)` and `-cos(phi_k)*sin(theta_k)` respectively (not the other way
  around) - verified against the source text and by direct symbolic re-derivation of
  `Rz(phi)*Ry(theta)*Rz(-phi)` for this rewrite, since one of the source extraction attempts
  had this sign flipped.
- **Per-segment mounting offset is per-SEGMENT, not per-tendon:** `delta_1 = pi/2` and
  `delta_2 = 5*pi/6` in Eq. 16 index the two segments (`k = 1, 2`); there is no `delta_3` -
  do not invent one for a 3rd tendon, the tendon index `i in {1,2,3}` is handled entirely by
  the `2*pi*(i-1)/3` term.
- **Sign convention (add to CLAUDE.md's table once implemented):**
  `SoftManipulatorPlant: compute(theta_k_ref - theta_k)` for the bending loop - positive-gain
  plant (more commanded bending -> more achieved bending), unlike the negative-gain pattern
  seen in some other case studies (Solar Cooker, Aircraft Engine).
- **CSV columns:** `t, theta1_ref, theta1, theta2_ref, theta2, P1, P2, x_tip, z_tip, x_ref,
  z_ref, m_L, error, iae_cumulative` (the trailing two names are required verbatim for
  `tools/metrics.py` auto-detection, per the Part 64 lesson already in CLAUDE.md).

---

## Status

Spec only - `sim/` not present, not registered, not built. See `HANDOFF_PROMPT.md` in this
folder for the full implementation plan, including the plant-architecture reduction needed
before any controller code can be written, the controller-by-controller mapping above in
more implementation-level detail, scenario parameters, and the file/registration checklist.
