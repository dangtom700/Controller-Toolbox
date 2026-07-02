# Bouyancy-Driven Airship in Vertical Plane

## Reference

Xiaotao Wu, Claude H. Moog, Luis Alejandro Marquez-Martinez, Yueming Hu (2013). "Full model
of a buoyancy-driven airship and its control in the vertical plane." *Aerospace Science and
Technology* 26, 138-152. (Verified directly against the PDF's own text extraction,
`extracted_text.txt`, in this folder - lines 8-17 give the exact author list and venue.)

> Implemented from the `tools/new_case_study.py` scaffold per `HANDOFF_PROMPT.md`'s plan -
> see "Status" at the bottom of this file for what was built and a few real deviations from
> that plan discovered during implementation (gain signs, tuning, one MPC redesign).

---

## Plant Model

A novel airship concept with **no thrust, elevator, or rudder** - actuated purely by (1) an
internal moving mass that shifts the center of gravity to control pitch attitude, and (2) an
internal air bladder of adjustable mass that changes the net lift to climb or descend. The
paper develops the model in increasing generality (fixed-center idealisation -> liberated
center with ballistic CG motion -> liberated center with added mass -> complete model with
aerodynamic forces) and proves a nonlinear feedback-linearization control law with provably
stable internal (zero) dynamics for the cases it analyses.

**This case study implements the "liberated center point" model (paper's Eq. 35, Sec. 4.3)**
- the first variant with non-trivial translational (ballistic) dynamics, and the one the
paper itself validates numerically with concrete parameters and initial conditions. The
added-mass (Sec. 4.4) and aerodynamic-forces (Sec. 4.5) extensions are **deferred scope**,
not implemented here - see "Model simplifications" below for why.

### Physical Description

- **Moving mass (`m_bar`):** slides along the body x-axis at position `rp1`, at a fixed
  lever arm `rp3` above/below the hull centerline. An actuator applies force `u` to this
  mass along the body x-axis; sliding it shifts the airship's center of gravity and produces
  a pitching moment - this is the *only* attitude actuator (no aerodynamic control surfaces).
- **Hull (`ms`):** the rigid/stationary mass of the airship body; together with the moving
  mass, `mv = ms + m_bar` is the total vehicle mass.
- **Net lift (`m0`):** the imbalance between total vehicle mass and buoyancy
  (`m0 = mv - rho_a*nabla`); changing the internal air bladder's mass changes `m0` and hence
  whether the airship climbs, descends, or holds altitude. `m0` is treated as a parameter in
  this model (constant per scenario, except a scripted bang-bang switch for the sawtooth
  scenario - see below); the bladder's own mass dynamics (`dot(m_b) = u4`) are out of scope
  (the paper itself treats the bladder release/inflation as instantaneous).
- **Ballistic translation (`v1, v3`):** because the moving mass is "liberated" (not pinned to
  a fixed center of volume), the whole vehicle's center of gravity is free to translate under
  gravity and the net-lift imbalance - the paper calls this "ballistic motion of CG" (its Fig.
  13). There is no aerodynamic lift in this model (that only appears once Sec. 4.5's
  aerodynamic forces are added - out of scope here), so the airship is not in trimmed,
  non-drifting flight; `v1`/`v3` evolve under gravity/net-lift rather than holding steady.

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `theta` | Pitch angle | rad |
| `q = d(theta)/dt` | Pitch rate (paper's `Omega2`; `Omega1 = Omega3 = 0` in pure vertical-plane motion) | rad/s |
| `rp1` | Position of the moving mass along the body x-axis | m |
| `w = d(rp1)/dt` | Slider rate | m/s |
| `v1` | Body-frame forward velocity | m/s |
| `v3` | Body-frame heave (vertical) velocity | m/s |

`rp3` (slider lever arm) is a constant parameter, not a state. `m0` (net lift) is a
parameter, piecewise-constant in time (constant per scenario, switched for the sawtooth
scenario).

### Control Input and Output

| Symbol | Description | Unit |
|---|---|---|
| `u` | Force the actuator applies to the moving mass along the body x-axis - the single scalar control signal for every controller in the roster | N |
| `y = theta` | Regulated output | rad |

Sign convention (standard for this repo): `compute(theta_ref - theta)`, like
`DiscretePID`/`DiscreteADRC` elsewhere in this project - except SMC and NeuralPID, which need
the opposite (`theta - theta_ref`); see "Implementation Notes" below for why.

`rp1` is **not** independently commanded - it is an internal state coupled to `theta`
through the single shared actuator `u` (the system is genuinely underactuated: one input,
two coupled mechanical DOF). It is logged on every run regardless: the paper's own
feedback-linearization controller keeps `rp1` bounded *by construction* (it regulates a
composite output whose zero dynamics are provably stable, see Eq. (24) below), while the 11
generic SISO controllers in the roster have no explicit mechanism for this and may let `rp1`
drift - a legitimate, paper-grounded comparison this case study is built to surface, not a
bug to fix in the generic controllers.

### Governing Equations

**Liberated-center model (paper's Eq. 35, Sec. 4.3):**
```
theta_ddot = rho1 + (1 / (J + m_bar*rp1^2)) * (v3_dot - q*v1) * m_bar*rp1

rp1_ddot   = sigma1 + (1 / (J + m_bar*rp1^2))
             * ( m_bar*rp1*rp3*(q*v1 - v3_dot) - (J + m_bar*rp1^2)*(q*v3 + v1_dot) )

v1_dot = (1/ms) * ( -ms*q*v3 + (m_bar - m0)*g*sin(theta) - u )

v3_dot = (1/(ms+m_bar)) * ( (ms+m_bar)*q*v1 + m0*g*cos(theta)
                             + m_bar*rp3*q^2 + 2*m_bar*q*w + m_bar*rp1*theta_ddot )
```
where `rho1`, `sigma1` are the fixed-center model's closed forms (paper's Eq. 24, Sec. 4.2 -
the degenerate case with no translation, `v1 = v3 = 0`):
```
rho1   = -(1/(J + m_bar*rp1^2)) * ( m_bar*rp3*rp1*q^2 + 2*m_bar*rp1*w*q
                                      + m_bar*g*rp1*cos(theta) + rp3*u )

sigma1 = (1/(J + m_bar*rp1^2)) * ( (J*rp1 + m_bar*rp1^3 + m_bar*rp3^2*rp1)*q^2
                                     + 2*m_bar*rp3*rp1*w*q - (J + m_bar*rp1^2)*g*sin(theta)
                                     + m_bar*g*rp3*rp1*cos(theta)
                                     + (J/m_bar + rp1^2 + rp3^2)*u )
```

**Implicit coupling - `theta_ddot` and `v3_dot` are mutually dependent** (the
`m_bar*rp1*theta_ddot` term inside `v3_dot`, and the `v3_dot` term inside `theta_ddot`).
`rp1_ddot` and `v1_dot` do not feed back into any other acceleration's right-hand side, so
only `theta_ddot`/`v3_dot` form a genuine 2x2 linear system - solved here in closed form
(do not implement this as 4 sequential assignments, the circular dependency will silently
give the wrong dynamics):
```
A = m_bar*rp1 / (J + m_bar*rp1^2)
B = m_bar*rp1 / (ms + m_bar)
C = (1/(ms+m_bar)) * ( (ms+m_bar)*q*v1 + m0*g*cos(theta) + m_bar*rp3*q^2 + 2*m_bar*q*w )

v3_dot     = ( C + B*rho1 - A*B*q*v1 ) / ( 1 - A*B )
theta_ddot = rho1 + A*v3_dot - A*q*v1
```
`rp1_ddot` and `v1_dot` then follow directly from their own (non-coupled) formulas above
once `theta_ddot`/`v3_dot` are known.

**Trim point** (fixed-center subsystem, no `v1`/`v3` dependence, used to linearize for LQR/MPC):
```
u_ss(theta_ref, rp1_ref) = -m_bar * g * rp1_ref * cos(theta_ref) / rp3
```
There is no nontrivial steady trim for `v1`/`v3` in this model - with `m0 != 0` or
`theta_ref != 0` the airship is always on a slow ballistic drift (no aerodynamic lift to
balance it in this model variant; true non-drifting trimmed flight only appears once Sec.
4.5's aerodynamic forces are added, out of scope here). LQR/MPC are designed on the 4-state
fixed-center model (Eq. 24) at this trim and deployed on the full 6-state plant above,
treating `v1`/`v3` as a slow, unmodeled disturbance on the design model.

**Sign convention:** `d(theta_ddot)/du = -rp3 / (J + m_bar*rp1^2)`. With `rp3 = 2 m > 0` this
is **negative** - increasing `u` decreases pitch acceleration, the same "negative-gain
plant" pattern documented elsewhere in this repo for Solar Cooker and Aircraft Engine.

**Feedback-linearization output (paper's headline result, Sec. 4.3.2, Theorem 2)** - a
composite output whose zero dynamics are provably asymptotically stable for any `k_tilde > 0`:
```
phi1_tilde = J*q + (rp1^2*q + rp3^2*q + rp3*w) * (m_bar*ms)/(m_bar+ms)

phi2_tilde = theta + rp3/sqrt((m_bar+ms)/(m_bar*ms)*J + rp3^2)
                    * arctan( rp1 / sqrt((m_bar+ms)/(m_bar*ms)*J + rp3^2) )

d(phi2_tilde)/dt = phi1_tilde / ( J + (rp1^2+rp3^2)*(m_bar*ms)/(m_bar+ms) )

y = phi1_tilde + k_tilde*phi2_tilde            (k_tilde > 0)
```
The control law solves `y_dddot = alpha(xi,zeta) + beta(xi,zeta)*u` for `u`, with the
third-order error dynamics `y_dddot + lambda2*y_ddot + lambda1*y_dot + lambda0*(y - y_e) = 0`.
The paper computed `alpha`/`beta` via computer algebra and only published the result in an
Appendix that is not present in this case study's PDF text extraction - see "Implementation
Notes" below for how this case study gets `alpha`/`beta` without that Appendix.

### Sawtooth flight extension (Sec. 4.6)

`m0` switches between two trim values to drive a climb/descend cycle:
```
m0 = +34.66 kg   (net lift force = +340 N, ascent)
m0 = -34.66 kg   (net lift force = -340 N, descent)
```
(the paper narrates the switch as "+-340 N"; `m0` itself is a mass per its role in the
equations above, so the consistent value is `340 N / g = 34.66 kg`). The paper treats
bladder release/inflation as instantaneous - `m0` is simply flipped at scripted switch times,
no separate bladder-mass ODE is modelled.

### Key Parameters

| Symbol | Value | Symbol | Value |
|---|---|---|---|
| `m_bar` (moving mass) | 30 kg | `J` (pitch inertia) | 8000 kg m^2 |
| `ms` (hull mass) | 269 kg | `rp3` (slider lever arm) | 2 m |
| `rho_a` (air density) | 1.29 kg/m^3 | `nabla` (airship volume) | 296 m^3 |
| `g` | 9.81 m/s^2 (standard, not separately tabulated) | | |

(`Cx0, Cx_alpha, Cz0, Cz_alpha, Cm0, Cm_alpha, Cm_Omega2` are the paper's aerodynamic
coefficients, Table 1 - only needed once the Sec. 4.5 extension is implemented; not used by
this case study's plant.)

### Model simplifications

- **Added mass (Sec. 4.4) and aerodynamic forces (Sec. 4.5) are deferred, not implemented.**
  The paper's own LaTeX-reconstruction sources used to draft this case study contained
  visible transcription corruption in those two sections specifically (a dimensionally
  suspect term in the added-mass slider equation, and a fraction in the aerodynamic
  `phi_2'` relationship that's identically `1` - both clearly mangled OCR, not real
  physics), and this session's PDF text extraction did not independently re-verify them.
  The liberated-center model implemented here is clean in both the extraction and an
  independent cross-check, and is the one the paper itself validates numerically in Sec.
  4.3.3. See `HANDOFF_PROMPT.md` for the full reasoning.
- **`u_max`/`u_min` = +-400 N** (assumed - not stated by the paper; its own simulated `u`
  traces peak around 150-220 N, so this gives headroom without being a soft constraint
  that's never hit).
- **`rp1` track limits = +-1.5 m** (assumed - a physical slider track must end somewhere
  and the paper doesn't say where; its own scenarios use `rp1` around -1.15 m).
- **`Ts` = 0.05 s (20 Hz)** (assumed - this is a slow mechanical system, `J = 8000 kg m^2`,
  settling times ~20 s, comparable in dynamics class to this repo's Drill String (`Ts=0.1s`)
  or Wind-Wave (`Ts=0.5s`) studies, not a fast electrical/hydraulic system).

---

## Control Objective

Regulate pitch attitude `theta` to a commanded reference via the single moving-mass
actuator `u`, across:

1. **Attitude step regulation** - drive `theta` to a new commanded value with the paper's
   own qualitative target (settles in ~20-30 s, no overshoot) despite the coupled,
   underactuated `rp1`/`v1`/`v3` dynamics.
2. **Robustness to ballistic translation** - maintain attitude tracking while the vehicle's
   center of gravity is on a slow, uncontrolled ballistic drift (`v1`, `v3`), since this
   model has no aerodynamic lift to arrest it.
3. **Sawtooth altitude cycling** - track an alternating ascend/descend pitch-attitude
   schedule synchronized with a bang-bang net-lift (`m0`) switch, the paper's own
   trajectory-tracking flight demonstration (Sec. 4.6).

The paper's own method (Sec. 4.3's feedback-linearization law on the composite output
`phi1_tilde + k_tilde*phi2_tilde`) is included in the roster below as the controller with
the strongest theoretical guarantee (provably stable zero dynamics); the other 11
controllers are generic comparison baselines that only see `theta` error.

---

## Controller Roster (12)

The paper's method is included as controller #12. Every wrapper class lives in this study's
own `sim/include/controllers.h` / `sim/src/controllers.cpp` (`ClassNameAirshipCtrl`), and
encapsulates its own sign handling internally - the simulation runner just calls
`compute(x, theta_ref, rp1_ref, m0)` uniformly on whichever `ControllerBase` is active.

| # | Name | lib/ Algorithm | Sign / Interface | Tuning notes (final, after empirical retuning) |
|---|------|----------------|-------------------|--------------|
| 1 | OpenLoop | - | `u = 0` | Baseline |
| 2 | PID | `DiscretePID` | `compute(theta_ref - theta)` | **Negative** `Kp=-251, Ki=-6, Kd=-1808` (negative-gain plant - positive gains are positive feedback here, see "Sign convention" above); `pid_.bumplessInit(lastOutput, newError)` called on every `theta_ref` change (defensive re-init, see Implementation Notes); explicit trim feedforward `u_ss = trimInput(theta_ref, rp1_ref)` added on top |
| 3 | ADRC | `DiscreteADRC` | `compute(theta_ref - theta)` | `b0` negative, evaluated at a representative `rp1=-1.15` m; `omega_o=3.0, omega_c=0.3` (`omega_o*Ts=0.15<0.5`, check); trim feedforward added on top |
| 4 | SMC | `DiscreteSMC` | `compute(theta - theta_ref)` | Per this repo's SMC sign convention; **negative** `K=-220`; `c_de=15` (much larger than `SMCParams`' own "lambda*Ts" rule of thumb - see Implementation Notes); trim feedforward added on top |
| 5 | LQR | `makeLQRController` factory | state feedback | 4-state design model `[theta, q, rp1, w]` (Eq. 24), numerically linearized + re-trimmed whenever `(theta_ref, rp1_ref)` changes; Bryson weights `Q=diag(1/0.3^2, 1/0.3^2, 1/0.5^2, 1/0.3^2)`, `R=1/400^2` |
| 6 | MPC | `DiscreteMPC` | state feedback | Same 4-state design model as LQR, deviation form; `Np=20, Nc=5`, and **the same Bryson `rho_y`/`rho_u` weights as LQR** rather than hand-picked MPC weights - see Implementation Notes for why several longer-horizon attempts diverged first |
| 7 | MRAC | `MRACController` | `set_reference(theta_ref)` then `compute(theta)` | **Negative** `gamma_r=gamma_y=-2` (gentle - the trim feedforward already supplies most of the load, see Implementation Notes); trim feedforward added on top |
| 8 | GainScheduled | `GainScheduledController(Ts)` | 3-point PID schedule on `abs(theta_ref - theta)` (`0.05/0.30/0.80` rad breakpoints) | All 3 inner PIDs **defensively `bumplessInit()`'d on every `theta_ref` change** (not just whichever one `GainScheduledController` itself would protect - see Implementation Notes); negative gains; trim feedforward added on top |
| 9 | L1Adaptive | `L1AdaptiveController` | `set_reference`/`compute` | **Positive** `Gamma=200` (L1's `sigma_hat` law does not need MRAC's gamma-negation rule); `omega_c=1.5`; trim feedforward added on top; expect a steady-state-error plateau on the harder scenarios (relative-degree-1 law on a relative-degree-2 plant - same architectural ceiling noted for Stewart Platform's rod tracking elsewhere in this repo, not a bug) |
| 10 | NeuralPID | `NeuralPID` | `compute(theta - theta_ref)` (**y - ref**, mirroring Solar Cooker - see Implementation Notes) | `Kd0=0` (NeuralPID has no derivative-on-measurement option - see Implementation Notes); `Kp0=251, Ki0=6` seeded to match PID #2; negative `plant_gain`; trim feedforward added on top |
| 11 | ILC | `IterativeLearningControl` (P-type, two-phase) | `compute(theta_ref - theta)` | `N_TRIAL=600` (30 s): PID feedback while learning (phase 1), PID + learned feedforward after (phase 2); inner PID defensively `bumplessInit()`'d on `theta_ref` change, same as PID #2; trim feedforward added on top |
| 12 | **AirshipFBLCtrl** (new local class, the paper's headline method) | n/a - case-study-local, same pattern as SMISMO's `DOBEnergyCtrl` | full state | Implements the `phi1_tilde + k_tilde*phi2_tilde` law above; gets `alpha`/`beta` **numerically** (finite differences on the plant's own known closed-form dynamics) rather than the paper's unpublished Appendix-A symbolic expressions - see Implementation Notes. `k_tilde=50, lambda0=1, lambda1=2, lambda2=2` (paper's Sec. 4.2.3 starting values, unchanged - already converges well) |

All 12 controllers run cleanly (no NaN/divergence) across all 5 scenarios; PID, ADRC, LQR,
MPC, and AirshipFBLCtrl converge well within `theta0_deg -> theta_ref_deg` step scenarios.
SMC/MRAC/GainScheduled/L1Adaptive/NeuralPID/ILC are functional but track noticeably less
tightly on the larger commanded swings (`s03`) and especially the sawtooth flight (`s05`,
the hardest scenario - alternating reference + bang-bang `m0` every 37.5 s) - a normal first
tuning pass, not a defect; see Implementation Notes and `HANDOFF_PROMPT.md`'s Open Risks.

---

## Scenarios (5)

All scenarios use the liberated-center model above; only initial conditions, references,
and `m0` differ. Suggested `T_sim`: 60 s for s01-s04, 150 s for s05.

| ID | Description | theta0 -> theta_ref | rp1_ref | v1(0), v3(0) | m0 |
|----|---|---|---|---|---|
| `s01_calm_step` | Paper's own Sec. 4.2.3 validation case (translation-free check) | 41.5 deg -> 30 deg | -1.15 m | 0, 0 | 1 kg |
| `s02_ballistic_step` | Same step, paper's Sec. 4.3.3 ballistic-CG case | 41.5 deg -> 30 deg | -1.15 m | 1.8 m/s, 0 | 1 kg |
| `s03_large_maneuver` | Bigger commanded swing, stresses saturation/nonlinearity | 55 deg -> 15 deg | -1.0 m | 1.8 m/s, 0 | 1 kg |
| `s04_disturbance` | Hold `theta_ref` constant; inject a transient pitch-moment disturbance mid-run to test rejection | 30 deg -> 30 deg | -1.15 m | 1.0 m/s, 0 | 1 kg |
| `s05_sawtooth` | Sec. 4.6 trajectory-tracking flight: 2 ascend/descend cycles, `m0` bang-bang switches +-34.66 kg at each cycle's midpoint, `theta_ref` alternates ascent/descent pitch each half-cycle | alternating, e.g. 25 deg / -15 deg | -1.15 m | 0, 0 | +-34.66 kg (switched) |

**Total runs:** 12 controllers x 5 scenarios = 60 (all complete cleanly, no NaN/divergence).

The `s04` disturbance is a one-shot pitch-rate impulse (`+0.05 rad/s` added directly to `q`
via `Plant::setState()`) at `t=20 s`; `s05`'s half-cycle length is 37.5 s (2 full
ascend/descend cycles over the 150 s run). Both were hand-tuned during implementation so the
responses are visually interesting (neither trivial nor saturating), the same way every other
case study's scenario set in this repo was tuned, not algorithmically derived.

---

## Implementation Notes

- **Do not hand-derive `AirshipFBLCtrl`'s third-order `alpha(xi,zeta)`/`beta(xi,zeta)`
  terms.** The paper computed them via computer algebra and only published the result in an
  Appendix A that is not present in this case study's PDF text extraction. Since the plant's
  full closed-form vector field `f(x,u)` is already known exactly (Governing Equations
  above), `alpha`/`beta` are obtained **numerically**: a short (`H=1e-3 s`) constant-input RK4
  rollout from the current state, with `xi1/xi2/xi3/alpha` read off via the standard forward
  finite-difference stencils for the 0th-3rd derivatives, and `beta` from a second rollout at
  a probe input `eps=1.0 N`. This is exact (the dynamics are exact, only the *differentiation*
  is numerical), self-correcting by construction (the control law's `-(1/beta)` factor
  automatically accounts for whatever sign `beta` comes out to be), and far less error-prone
  than re-deriving the symbolic algebra by hand. `AirshipFBLCtrl::normalForm()` is exposed
  (not just used internally) specifically so `tests/test_bouyancy_driven_airship_regression.cpp`
  can sanity-check it two ways without re-deriving the closed-form algebra: (1) at the trivial
  all-zero drift equilibrium, every rollout sample is exactly 0 by construction, so
  `xi1=xi2=xi3=alpha=0` to floating-point precision; (2) away from that point, `beta` must be
  independent of the probe magnitude `eps` since the plant is provably affine in `u` (every
  term containing `u` in the governing equations above is linear in `u`, with no `u^2` or
  `trig(u)` terms) - a differencing bug would generically break that invariance.
- **CSV columns:** `t, theta_ref, theta, rp1, v1, v3, u, m0, error, iae_cumulative` - the
  trailing two names are required verbatim for `tools/metrics.py` auto-detection (per the
  Part 64 lesson already in CLAUDE.md).
- **Per-controller sign handling is encapsulated in each wrapper class, not branched in the
  simulation runner.** SMC needs `compute(theta - theta_ref)`, NeuralPID needs
  `compute(theta - theta_ref)` too (despite the roster table's general "compute(theta_ref -
  theta)" convention - NeuralPID's `[Kp,Ki,Kd]` are softplus-activated and therefore always
  positive, so the negative-gain plant has to be handled through the error sign instead, the
  same way Solar Cooker's `NeuralPIDCookerCtrl` does), and everything else uses
  `compute(theta_ref - theta)` - each `ClassNameAirshipCtrl::compute()` in `controllers.cpp`
  decides its own convention internally; `simulation_runner.cpp` just calls
  `ctrl.compute(x, theta_ref, rp1_ref, m0)` uniformly on whichever `ControllerBase` is active.
- **A large, roughly constant trim torque dominates this plant.** Holding any pitch angle
  needs `u_ss = trimInput(theta_ref, rp1_ref)` approx= 120-150 N just to counteract the
  slider's own gravity moment - confirmed directly from the governing equations (the
  fixed-center `rho1=0` condition at `q=w=0` reduces exactly to the trim formula above).
  ADRC's ESO and `AirshipFBLCtrl`'s `alpha` term estimate/cancel this automatically; LQR/MPC
  feed it forward through their own design-model trim; the remaining controllers
  (PID/SMC/MRAC/GainScheduled/L1Adaptive/NeuralPID/ILC) do not have an equivalent built-in
  mechanism and were given an explicit `u_ss` feedforward on top of their normal feedback
  output - without it, every one of them converged far too slowly against this disturbance
  using integral action alone (confirmed empirically: this was the single most impactful fix
  applied during tuning).
- **Large `theta_ref` steps (10-40 deg, every scenario) create a derivative "kick" wherever a
  controller's derivative term is computed on the tracking error rather than the
  measurement.** The error jumps instantly on the step; `DiscretePID`'s filtered derivative
  amplifies that single-step jump into a huge transient, which saturates `u` and then the
  resulting anti-windup back-calculation slams the integrator the wrong way for 1-2 s -
  confirmed empirically (the first build of PID/ILC/GainScheduled ran straight to the
  actuator rail after every reference step and took 10s of seconds to recover, or never did).
  `computeDoM()` (derivative-on-measurement) looked like the fix but has the *same* problem
  at `k=0`, since `DiscretePID::reset()` zeroes its internal `y_prev_` rather than seeding it
  from the plant's actual initial measurement. The robust fix needs no library change:
  `pid_.bumplessInit(pid_.lastOutput(), newError)` is called explicitly on every detected
  `theta_ref` change (including the implicit "change" from an initial NaN sentinel), which
  re-seeds `e_prev_`/`deriv_`/`integral_` so the very next `compute()` call starts smoothly -
  the same mechanism `GainScheduledController` already uses internally for bracket switches,
  just invoked manually here since these are single fixed controllers, not a schedule.
  `GainScheduledController`'s own bumplessInit-on-switch protection only fires for a
  controller *newly entering* the active bracket pair (LinearBlend mode) - a controller that
  stays part of the pair across the jump (e.g. the "mid" bracket, which is "hi" just below
  the jump and "lo" just above it) is not re-initialised by that built-in mechanism, so
  `GainScheduledAirshipCtrl` defensively calls `bumplessInit()` on *all three* inner PIDs
  itself rather than relying on it. `NeuralPID` has no equivalent re-init hook at all (no
  `bumplessInit` override), so its seed `Kd0` is `0` instead - a real, if blunt, accepted
  workaround.
- **`DiscreteSMC`'s sliding-surface derivative term needs an unusually large `c_de` on this
  plant.** Inside the boundary layer, SMC's law reduces to a PD form with **no `1/Ts`
  scaling** on the derivative-like `(e - e_prev)` term, unlike `DiscretePID`'s filtered
  derivative (effective multiplier approx= `Kd*N*alpha/(1-alpha)` approx= `Kd*20` for this
  `Ts`/`N`). Matching comparable damping to the well-tuned PID above therefore needs `c_de`
  roughly 20x larger than the "`c_de = lambda*Ts`" rule of thumb in `SMCParams`' own
  docstring would suggest (`c_de=15` here, vs. the docstring's typical `0.01-1` range) -
  confirmed empirically: smaller `c_de` left SMC critically under-damped, overshooting
  straight through `theta_ref` into the far negative range.
- **MPC needed the *same* Bryson weights as LQR, not independently hand-picked MPC weights.**
  Several longer-horizon / differently-weighted attempts (`Np` from 40 to 150, various
  `rho_y`/`rho_u` ratios) all diverged the same way: theta sails straight through `theta_ref`
  and keeps accelerating past it. Root cause: the shared 4-state design model excludes
  `v1`/`v3` entirely, so the QP's own prediction has no way to "see" the growing ballistic
  coupling that eventually arrests/reverses `theta` in the real plant (confirmed empirically:
  `v3` grows monotonically through every divergent run). LQR uses the *same* mismatched
  design model yet stays robust because it commits to nothing beyond a single proportional
  reaction to the current real state every step, not an open-loop-optimal sequence; setting
  `rho_y = 1/theta_max^2` and `rho_u = 1/u_max^2` (LQR's own Bryson `Q`/`R`, see roster table)
  with a short `Np=20` reproduced that same robustness almost exactly (IAE dropped from
  25-47 down to 5.4 on `s01`).

---

## Status

**Implemented and registered.** `sim/` is the real liberated-center-model plant + 12
controllers described above (no longer the scaffold placeholder); built as
`bouyancy_driven_airship_in_vertical_plan_sim` (60 runs, registered in
`case-study/CMakeLists.txt` and `compile.bat`/`compile.sh`); regression-tested by
`tests/test_bouyancy_driven_airship_regression.cpp` (8 `[airship]` Catch2 test cases -
PID/ADRC/LQR/MPC/AirshipFBLCtrl convergence checks, 2 `AirshipFBLCtrl::normalForm()` sanity
checks, and an all-12-controller smoke test - registered in `tests/CMakeLists.txt` and
`compile.bat`/`compile.sh`). See `HANDOFF_PROMPT.md` for the original implementation plan -
the plant equations, trim formula, and 12-controller roster were all implemented exactly as
planned there; the real deviations discovered along the way (controller gain signs, the MPC
weight redesign, the trim-feedforward and bumplessInit fixes) are documented in
"Implementation Notes" above rather than in that file, since they were tuning/empirical
findings rather than plan-level decisions.

Not implemented (deferred per "Model simplifications" above, consistent with the original
plan): the paper's Sec. 4.4 added-mass and Sec. 4.5 aerodynamic-force model extensions.

**Robustness analysis added.** `sim/src/robustness_main.cpp` (built as
`bouyancy_driven_airship_in_vertical_plan_robustness`, registered in this study's
`CMakeLists.txt` and `compile.bat`/`compile.sh`) follows the same `case-study/common/
RobustnessStats.h` pattern as Boiler Control/Tug Boat/etc.: WCET (per-step `compute()` timing
on the nominal `s01_calm_step` scenario), Monte Carlo (30 samples/controller, +-15% Gaussian
perturbation of `m_bar`/`ms`/`J`, controllers built from the original nominal `PlantParams`,
only the simulated `Plant` perturbed), and a fault sweep (3 magnitudes per applicable fault
kind - sensor bias/noise on the measured `theta`, actuator loss/stuck on `u`, setpoint step on
`theta_ref` - injected at 40% through the 60 s run). All 12 controllers stayed stable across
every MC sample and fault trial (`P(unstable)=0` throughout). Output lands at the study root
(`mc_summary.csv`, `fault_sweep.csv`, `wcet_summary.csv`, alongside the pre-existing
`mu_analysis.csv`) and `report.html` was regenerated via `tools/generate_report.py` -
`tools/case_study_tracker.py` now reports this study's status as `Complete`.
