# Physical System Modeling and Optimized Control of 6-DOF Vessel Motion Simulator

## Reference

Weibin Ma, Peng Wang, Huachao Dong, Xinjing Wang, Hengzhi Fang, and Xianxian Fan (2025). "Physical system modeling and optimized control strategy of 6-DOF vessel motion simulator based on MBD and LBM." *Ocean Engineering* 334, 121595. https://doi.org/10.1016/j.oceaneng.2025.121595

---

## Plant Model

A **6-DOF vessel motion simulator** based on the **Stewart parallel mechanism (6-UPU configuration)**: a fixed base and a moving platform connected by six independently actuated limb actuators (hydraulic linear cylinders), each attached via universal (Hooke) joints at both ends. The simulator replicates sea-wave-induced vessel motions to enable laboratory testing of shipboard equipment under realistic multi-axis excitation. Ma et al. develop an integrated physical system model that couples: (i) a Multi-Body Dynamics (MBD) model in Simscape/Simulink, (ii) an inverse kinematics resolver that maps desired 6-DOF platform pose to individual actuator stroke commands, (iii) a sensor model for pose feedback, and (iv) the control system. CFD simulations via the Lattice-Boltzmann Method (LBM) provide realistic vessel motion inputs at sea states 3-5.

### Physical Description

- **Platform geometry (6-UPU):** Base platform and moving platform are each hexagonal frames. The six limb actuators connect the lower (base) and upper (platform) hinges via Hooke (universal) joints. Actuator `i` spans from base-hinge `A_i` to platform-hinge `B_i`.
- **Inverse kinematics:** Given the desired 6-DOF platform pose `(P, R)` (translation P \in ℝ^3, rotation matrix R \in SO(3)), the actuator stroke change is:
  ```
  DeltaL_i = || R * B_i - A_i + P || - l0
  ```
  where `l0` is the neutral actuator length.
- **Workspace (from paper Table 1):** Surge +/-366 mm, Sway +/-422 mm, Heave 621-985 mm, Roll +/-27^\circ, Pitch +/-23^\circ, Yaw +/-30^\circ.
- **Dynamic equation:** The platform's equation of motion in generalised coordinates:
  ```
  M(q) * ddq + C(q, dq) + G(q) = F
  ```
  where `M` is the total mass/inertia, `C` the centrifugal/coupling term, `G` gravity, and `F` the driving force from the six limb actuators projected via the Jacobian.
- **Shipboard equipment load:** 12,000 N concentrated at the platform centre, dominating the gravitational and inertial loading.
- **CFD vessel inputs:** 6-DOF time-series of vessel position/attitude from LBM simulation (XFlow software). Sea state 3: H=1.25 m, T=3.8 s; sea state 4: H=2.5 m, T=5.4 s; sea state 5: H=4.0 m, T=7.0 s. Heave amplitudes: 0.112 m, 0.208 m, 0.991 m respectively.

### State Variables

| Symbol | Description | Unit |
|--------|-------------|------|
| `L_i` (i=1...6) | Instantaneous length of limb actuator i | mm |
| `dL_i` | Velocity of limb actuator i | mm/s |
| `x_p, y_p, z_p` | Platform centre translation (surge, sway, heave) | mm |
| `phi, theta, psi` | Platform orientation (roll, pitch, yaw) | deg |

Each limb actuator is controlled independently; the control variable is the rod displacement error `e_i = L_i_cmd - L_i`.

### Governing Equations

**Inverse kinematics (per limb i):**
```
L_i_cmd = || R(phi,theta,psi) * B_i - A_i + P(x_p,y_p,z_p) ||
DeltaL_i = L_i_cmd - l0
```
where `(B_i)` is the upper hinge in platform frame, `(A_i)` in base frame, `l0` nominal length.

**Actuator dynamics (each rod, simplified 2nd-order hydraulic):**
```
m_rod * ddL_i = u_i - k_spring * DeltaL_i - b_damp * dL_i - F_load_i(q, m_platform)
u_i = driving force [N], directly bounded by +/-F_rod_max   [see Implementation Notes #2]
```

**Platform MBD coupling (virtual work principle):**
```
J_limb^T(q) * F_limb = M(q) * ddq + C(q,dq) + G(q)
```
where `J_limb` is the Jacobian mapping limb forces to platform generalised forces. The
case study uses the static (quasi-equilibrium) form of this relation directly - see
Implementation Notes #3.

### Key Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Base hinge radius | Ra | 500 mm | Distribution circle radius, lower hinges |
| Platform hinge radius | Rb | 350 mm | Distribution circle radius, upper hinges |
| Base hinge pair half-angle | delta_base | 5 deg | See Implementation Notes #1 |
| Platform hinge pair half-angle | delta_platform | 45 deg | See Implementation Notes #1 |
| Platform pattern phase offset | phase | 0 deg | See Implementation Notes #1 |
| Nominal stroke | l0 | 866 mm (uniform across all 6 rods) | Neutral actuator length |
| Rod mass | m_rod | 15 kg | Moving rod mass (not given in paper) |
| Rod parasitic stiffness | k_spring | 2.0e4 N/m | Not given in paper |
| Rod damping | b_damp | 800 N.s/m | Not given in paper |
| Rod actuator force limit | F_rod_max | 6000 N | Not given in paper |
| Platform mass | m_p | 500 kg | Moving platform structural mass |
| Equipment load | F_eq | 12,000 N | Shipboard equipment weight |
| Actuator stroke range | DeltaL | 621-985 mm (heave) | Per workspace Table 1 |
| UDP communication delay | tau_comm | 1 ms (= 1 sample at Ts) | Controller-to-actuator latency |
| Sampling time | Ts | 5 ms | Control loop rate |

---

## Control Objective

Reproduce the 6-DOF vessel motion profile (derived from CFD sea-state simulation) on the moving platform with:

1. **Low tracking error** - platform pose must follow the desired wave-induced motion accurately across all sea states; maximum 6-DOF error at sea state 5 should remain below ~58 mm / 0.5^\circ (paper test values).
2. **Smooth actuator response** - avoid overshoot and oscillation in rod displacement that would damage the mechanism or disturb the shipboard equipment under test.
3. **Robustness to load disturbance** - the 12,000 N equipment load introduces coupling between limbs, and any vibration from equipment-under-test must be rejected.
4. **Rapid settling** - for sea state changes or step inputs, the platform must stabilise within 1-2 oscillation periods.

The **Ma et al. (2025)** paper proposes a **GA-fuzzy PID** controller: a Mamdani-style fuzzy PID (error `e` and error rate `ec` -> incremental gains Deltakp, Deltaki, Deltakd) whose 147 fuzzy rule parameters are optimised offline by a Genetic Algorithm to minimise root-mean-square rod displacement error. The GA-FuzzyPID outperforms both PID and hand-tuned FuzzyPID in all six DOFs at sea state 5.

---

## Controller Roster (implemented in `sim/include/controllers.h` + `.cpp`)

All controllers operate per-rod (`std::array<ctrl::X, 6>`, one independent SISO instance
per limb) except LQR, which is a single true 12-state block-diagonal controller. Gains
were retuned from the README's original placeholder values against the actual physical
constants chosen for this case study (k_spring=2e4 N/m, F_rod_max=6000 N) - see
Implementation Notes #2 and #9 for why several gains differ substantially from a naive
reading of the table below.

| # | Name | lib/ Algorithm | Key Parameters | Design Notes |
|---|------|---------------|----------------|--------------|
| 1 | PID | `DiscretePID` | Kp=8e4, Ki=4e4, Kd=4e3; e = L_i_cmd - L_i | Per-rod independent |
| 2 | FuzzyPID | `FuzzyPID` | e_scale=3mm, de_scale=50mm/s, u_scale=F_rod_max, Ki=4e4 | Toolbox's standard 5-term Mamdani FuzzyPD + crisp integral (not the paper's GA-optimised 147-parameter rule table - out of scope) |
| 3 | ADRC | `DiscreteADRC` | omega_o=60, omega_c=20, b0=1/m_rod; omega_o*Ts=0.30<0.5 | ESO treats spring/damping/load as total disturbance per rod |
| 4 | SMC | `DiscreteSMC` | c_e=1, c_de=50*Ts, K=4000, phi=1mm | compute(L_i - L_i_cmd) [y-ref convention] |
| 5 | LQR | `DiscreteLQR` | Q=diag(1e6,5 x6 pairs), R=1e-7*I_6 | Single 12-state block-diagonal model, deviation coords z_i=L_i-l0_i; empirically retuned (see #9) |
| 6 | MPC | `DiscreteMPC` | Np=10, Nc=3, rho_y=1e6, rho_u=1e-7, qpMaxIter=2000 | 6x 2-state per-rod model, deviation coords; force bounds only (no explicit stroke constraint QP) |
| 7 | MRAC | `MRACController` | gamma_r=gamma_y=5e5, a_m=exp(-15*Ts), b_m=1-a_m | setReference(L_cmd_i) + compute(L_i); positive-gain plant |
| 8 | L1Adaptive | `L1AdaptiveController` | Gamma=2e4, omega_c=10, k_g=10, a_m=exp(-15*Ts) | Relative-degree-1 architecture on a relative-degree-2 rod - see Implementation Notes #9 |
| 9 | GainScheduled | `GainScheduledController` | Scheduled on \|heave deviation\|; 3 PID gain sets (Ki=4e4/3e4/1e4) at p=0/0.10/0.16 m | Reduced integral gain at large heave amplitude to limit windup |
| 10 | TubeMPC | `TubeMPC` | Np=10, Nu=3, Q=1e6, R=1e-7, K_tube=-K_lqr, wMax=[1e-4,1e-2] | 6x 2-state per-rod model; tube accounts for inter-limb coupling as bounded disturbance |
| 11 | NeuralPID | `NeuralPID` | n_h=8, lr=5e-6, plant_gain=1/k_spring, Kp0/Ki0/Kd0 matching PID | Online gain adaptation; also fixed a library overflow bug (see Implementation Notes #9) |
| 12 | ScenarioMPC | `ScenarioMPC` | Np=10, Nu=3, N_samples=20, Q=1e6, R=1e-7, qpMaxIter=2000 | 6x 2-state per-rod model, Gaussian process noise on rod position/velocity |

---

## Scenarios - Douglas Sea State Matrix

Replaces the originally-proposed 5 ad-hoc scenarios with a comprehensive matrix built from
a standalone **CFD-input stand-in module** (`sim/include/cfd_input_model.h` + `.cpp`),
deliberately separated from the rod/geometry plant (`stewart_plant.h`) - mirroring the
paper's own architecture (Fig. 6: a distinct "input and inverse kinematic system model"
block feeding the MBD model). See Implementation Notes #7/#8 for the full derivation.

**10 Douglas Sea Scale states (0-9) x 3 wave directions (Head/Following/Beam) x 2 swell
conditions (with/without) = 60 configurations**, each generated at runtime by
`buildSeaStateMatrix()` (no per-scenario JSON files - `config/scenarios/sea_state_matrix.json`
holds only the tunable knobs: duration clamps, workspace margin, direction weights, swell
ratios).

| Douglas state | Description | Representative Hs | Wave period T(Hs) |
|---|---|---|---|
| 0 | Calm (glassy) | 0.0 m | 0.77 s (floor) |
| 1 | Calm (rippled) | 0.05 m | 0.77 s |
| 2 | Smooth (wavelets) | 0.30 m | 1.89 s |
| 3 | Slight | 0.875 m | 3.23 s |
| 4 | Moderate | 1.875 m | 4.73 s |
| 5 | Rough | 3.25 m | 6.22 s |
| 6 | Very rough | 5.0 m | 7.72 s |
| 7 | High | 7.5 m | 9.45 s |
| 8 | Very high | 11.5 m | 11.70 s |
| 9 | Phenomenal | 16.0 m (representative cap) | 13.80 s |

`T(Hs) = 3.45*sqrt(max(Hs,0.05))` is calibrated directly against the paper's 3 actual
(Hs,T) data points (1.25m->3.8s, 2.5m->5.4s, 4.0m->7.0s; `T/sqrt(Hs)` = 3.40/3.42/3.50 for
these three - a remarkably tight fit). Heave/roll/pitch amplitudes use a 4-anchor
piecewise-linear law through `(0,0)` plus the same 3 paper data points, extrapolated beyond
Hs=4.0m. Direction (Head/Following/Beam) redistributes energy between pitch/surge and
roll/sway via a fixed heuristic weight table; swell adds a bichromatic secondary component
at 1.8x the primary period and 40% of its amplitude. A workspace-margin scaling step
(Implementation Notes #8) guarantees every one of the 60 generated trajectories stays
within the paper's Table 1 limits, even at the most extreme Douglas states.

**Total runs:** 12 controllers * 60 sea-state configs = **720**. Scenario durations scale
with wave period (`10*T(Hs)`, clamped to [20s, 150s]), so this study has a materially
longer wall-clock runtime than other case studies in this repo - an accepted consequence of
the requested Douglas-matrix coverage.

---

## Implementation Notes

1. **Geometry (decision #1).** The paper gives workspace limits (Table 1) but not exact
   hinge angles. Uses a 3-hinge-pair-per-platform layout (pairs at 0/120/240 deg, each
   split by a half-angle). `platform_phase_deg=0` (platform pattern NOT rotated relative to
   the base pattern) gives every leg the **same** neutral length `l0_i` regardless of
   `delta_base_deg` vs `delta_platform_deg`, because `l0_i` depends only on
   `cos(beta_i-alpha_i)` and that angle has the same magnitude for the "lo"/"hi" leg of
   every pair. **The two half-angles must differ substantially** (`delta_base=5 deg`,
   `delta_platform=45 deg` here): equal half-angles make every leg a pure rotation of every
   other leg, which is a genuine kinematic (architecture) singularity for a pure-vertical
   load at the home pose - empirically verified across a wide phase/delta sweep, `det(J^T)`
   collapses to ~1e-49 (machine-precision zero, scaled) for every combination tried with
   equal half-angles, and the required per-rod load-support force blows up to absurd values
   (>1e19 N). The chosen geometry gives `l0=866mm` uniformly and a well-conditioned Jacobian
   (max load force ~3.3kN, well under `F_rod_max`).
2. **Actuator command units (decision #2).** Every controller outputs a **direct force in
   Newtons** (`uMin=-F_rod_max, uMax=F_rod_max`), matching every other per-actuator
   controller in this repo - not the README's original `F_drive_i = K_act*u_i, u_i\in[-1,1]`
   normalised convention. This also means ADRC's `b0 = 1/m_rod` (not `K_act/m_rod`).
3. **Load coupling (decision #3).** At every step, the static-equivalent rod load
   `F_load(6,)` is computed from the *current commanded pose's* Jacobian via the virtual-work
   equilibrium relation `J^T*f + wrench_ext = 0 => f = -J^-T*wrench_ext` (sign verified
   against a vertical 1-rod example: gravity pulling down must be met by `f=+W`, pushing
   up). `wrench_ext = [0,0,-(F_eq+m_platform*g),0,0,0]`. This realizes "12,000 N equipment
   load introduces coupling between limbs" without a full nonlinear MBD integrator.
4. **Achieved platform pose (decision #4).** True forward kinematics has no closed form
   (paper: "18 simultaneous nonlinear equations"), so the linearized relation
   `q_actual = q_ref - J^-1*(L_cmd - L_actual)` approximates it each step for the RMSE/error
   CSV columns - the README's own recommended Jacobian-based approach.
5. **Communication delay (decision #5).** A one-sample held delay on the combined 6-vector
   actuator command is applied generically inside `simulation_runner.cpp` (equivalent to,
   simpler than, wrapping all 72 sub-controller instances individually in
   `ComputationalDelayWrapper`).
6. **LQR (decision #6).** The only controller implemented as a single true 12-state
   block-diagonal `ctrl::DiscreteLQR` (matching the README's literal "12-state decoupled
   model"); all 11 others follow the `std::array<ctrl::X, 6>` per-rod pattern.
7. **CFD-input stand-in, separated from the plant (decision #7).** See "Scenarios" above -
   `cfd_input_model.h/.cpp` is architecturally independent of `stewart_plant.h`.
8. **Workspace-aware amplitude scaling (decision #8).** Paper sec. 4.1: "the input system
   model takes the CFD motion values...in an appropriate ratio." After computing the raw
   direction/swell-weighted 6-DOF amplitudes (including the swell contribution in the peak
   estimate), if any axis would exceed 90% of its Table-1 limit, all 6 amplitudes are scaled
   by one common factor so the worst axis lands at exactly 90%. Verified by a regression
   test (`[stewart][regression][cfd_input]`) at Douglas states 7-9.
9. **Gain retuning (decision #9, beyond the original plan).** Most gains in the controller
   roster table were **substantially retuned from the README's original placeholder values**
   once the actual physical constants were chosen (`k_spring=2e4 N/m`, `F_rod_max=6000 N`):
   the literal `Kp=200`-style numbers were sized for an unspecified different unit
   convention and produced negligible control authority against the chosen spring/load
   scale. LQR/MPC/TubeMPC/ScenarioMPC weights were swept empirically against the real
   plant+CFD reference rather than derived purely from Bryson's rule, because strict Bryson
   weights left LQR with a large steady-state offset under the constant load disturbance
   (pure state feedback has no integral action). Two additional findings during this
   process:
   - **L1Adaptive's tracking ceiling is architectural, not a tuning gap.**
     `L1AdaptiveController` (like `MRACController`) is documented as a relative-degree-1
     SISO adaptive law; the rod plant is relative-degree-2 (force -> position through a
     spring-mass-damper). A sweep of `Gamma` (200-1e6), `omega_c` (5-80), `k_g` (1-1000),
     `Q_lyap` (1-1e6), and the reference-model pole (1-60 rad/s) plateaus around 115-190mm
     steady-state rod error at sea state 5 regardless of tuning - accepted as an expected
     architectural limitation and documented in `controllers.cpp` rather than chased
     further. MRAC, despite the same relative-degree mismatch, reaches ~10mm with a much
     larger adaptation rate (`gamma_r=gamma_y=5e5` vs the README's placeholder `gamma=2`)
     because its direct algebraic control law (`u=theta_r*r+theta_y*y`) can still exploit
     brute-force gain in a way L1's low-pass-filtered architecture cannot.
   - **Library bug fix:** `ctrl::NeuralPID::forward()` used a naive `log1p(exp(x))` for
     softplus instead of the numerically-stable two-branch `softplus()` static helper
     already defined in the same class - large `Kp0`/`Ki0` seeds (needed here to match the
     other controllers' gain scale) overflowed to `inf`/`NaN`. Fixed in `lib/NeuralPID.cpp`
     (one-line change, zero behavioral impact on existing small-`Kp0` usages elsewhere in
     the repo).
- **ADRC omega_o constraint:** With Ts = 5 ms, require `omega_o * Ts < 0.5` -> `omega_o < 100 rad/s`. Use omega_o = 60, omega_c = 20.
- **Metric:** 6-DOF platform pose RMS error (mm for translation, deg for rotation); also report per-rod RMS displacement error for direct comparison with paper Table 9/10.
- **CSV columns:** `t, z_ref, z_p, phi_ref, phi, theta_ref, theta, psi_ref, psi, x_p, y_p, L1, L2, L3, L4, L5, L6, u1, u2, u3, u4, u5, u6, iae_cumulative`

---

## Status

**Implemented (C++).** Built and registered as `stewart_sim` in `case-study/CMakeLists.txt`
and `compile.bat`; regression tests in `tests/test_stewart_regression.cpp`
(`test_stewart_regression`). 12 controllers x 60 Douglas sea-state configs = 720 runs.

The paper's full physical system model is implemented in MATLAB Simscape/Simulink with a
1 ms CFD-driven input loop and hardware-in-the-loop test rig. This case study approximates
it with: (i) closed-form inverse kinematics at each step to get 6 desired rod lengths from
a synthetic CFD-input stand-in (Douglas sea-state matrix, see above), (ii) each rod
simulated as a 2nd-order spring-mass-damper actuator driven independently by its
controller, and (iii) the inverse-kinematics Jacobian used both to compute the
quasi-static equipment-load coupling force and to reconstruct an approximate achieved
platform pose from the rod tracking errors. This approximation ignores full nonlinear
platform-limb MBD coupling but captures the single-rod dynamics, load-disturbance
rejection, and 12-controller comparison faithfully - per-rod RMS tracking error at sea
state 5 ranges from ~4mm (ScenarioMPC) to ~190mm (L1Adaptive, an accepted architectural
limitation - see Implementation Notes #9), bracketing the paper's own PID/GA-fuzzy-PID
range of ~1.5-2.4mm (full MBD model, not directly comparable in absolute terms).
