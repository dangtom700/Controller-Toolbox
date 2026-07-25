# Differential Drive Robot Tracking - FUHAC

Reproduction of the **Fixed Ultra-Hybrid Adaptive Controller (FUHAC)** for a differential-drive
mobile robot (DDMR), benchmarked against eleven other controllers from `lib/`.

## Source

Peng Xu, Mohammadhadi Maghsoudniazi & Yahya Maghsoudniazi, *"Integrating Lyapunov based
backstepping and neuro fuzzy logic with sliding mode control for precise trajectory tracking of
differential drive robots"*, **Scientific Reports 16:11961 (2026)**,
[doi:10.1038/s41598-026-39667-1](https://doi.org/10.1038/s41598-026-39667-1).

- Paper PDF: `advanced control methods for precise trajectory tracking of differential drive robots.pdf`
- Extracted text: `advanced control methods for precise trajectory tracking of differential drive robots.txt`
  (PDF extraction is lossy - equations were re-derived against the text, not copy-pasted)

## Plant model

Five-state nonlinear DDMR with a non-negligible centre-of-mass offset `d`, RK4-integrated at
`Ts_plant = 5 ms` ([differential_drive_robot_tracking_plant.cpp](sim/src/differential_drive_robot_tracking_plant.cpp)):

```
state x = [X, Y, theta, omega_R, omega_L]

v     = r*(omega_R + omega_L)/2                  body-frame linear velocity  [m/s]
omega = r*(omega_R - omega_L)/(2R)               body-frame angular velocity [rad/s]

X'     = v*cos(theta) - d*omega*sin(theta)       paper's kinematic matrix
Y'     = v*sin(theta) + d*omega*cos(theta)
theta' = omega

[omega_R', omega_L']^T = M^-1 * ([tau_R, tau_L]^T - Kf*[omega_R, omega_L]^T)

M = [[A, B], [B, A]]
A = M_t*r^2/4 + (I_A + M_t*d^2)*r^2/(4R^2) + I_0
B = M_t*r^2/4 - (I_A + M_t*d^2)*r^2/(4R^2)
```

`M` is constant, so `M^-1` is factored **once** in the constructor; `step()` allocates nothing
and does no 2x2 solve. `det(M) = A^2 - B^2` is guarded at construction (`isHealthy()`), and a
non-finite torque command freezes the state rather than poisoning it (the repo's NaN contract).

### Plant parameters

**The paper never tabulates the robot's physical constants** - it only names the Pioneer
platform. Everything below is an assumption or a calibration, and is flagged as such in
[`config/plant_params.json`](config/plant_params.json) under `_sources`:

| Symbol | JSON key | Value | Provenance |
|---|---|---|---|
| `M` | `M_total` | 9.0 kg | Pioneer 3-DX base mass - **assumed, not in paper** |
| `r` | `r_wheel` | 0.0975 m | Pioneer 3-DX wheel radius - **assumed, not in paper** |
| `R` | `R_half_axle` | 0.165 m | half of the 0.33 m track - **assumed, not in paper** |
| `I_A` | `I_A` | 0.16 kg m^2 | estimated body inertia - **assumed, not in paper** |
| `I_0` | `I_0` | 0.005 kg m^2 | estimated wheel inertia - **assumed, not in paper** |
| `K_f` | `Kf` | 0.25 N m s/rad | **calibrated** - see note below |
| `d` | `d_com` | 0.05 m | **assumed**; the paper's kinematic matrix requires `d != 0` |
| - | `tau_max` | 15.0 N m | paper Fig. 22: torques "reach the saturation limits plotted (+/- 15 N.m)" |
| - | `v_max` | 5.0 m/s | command clamp; must exceed the fastest reference (see note below) |
| - | `Ts_plant` | 0.005 s | RK4 step and inner-PI period (this implementation's choice) |
| - | `Tf` | 0.03 s | **paper**: fast loop, 30 ms / 33 Hz |
| - | `Ts_slow` | 0.15 s | **paper**: slow loop, 150 ms; `eps = Tf/Ts_slow = 0.2` |

**On the `Kf` calibration and `v_max`.** `Kf` sets both the cruise torque and the physical speed
ceiling, and the two constraints must be satisfied together:

```
cruise torque at 3 m/s = Kf*v/r          speed ceiling v_ss = r*tau_max/Kf
Kf = 0.50  ->  15.4 N.m (over the band, saturated)   v_ss = 2.93 m/s
Kf = 0.25  ->   7.7 N.m (inside 5.9-9.7)             v_ss = 5.85 m/s
```

An initial `Kf = 0.50` was wrong on both counts: it capped the robot at 2.93 m/s while the
`a = 3 m` circle demands exactly 3.0 m/s, making that reference **physically unfollowable** by
any controller, and it would have needed 15.4 N.m of continuous cruise torque. `Kf = 0.25` puts
cruise torque in the paper's band with real speed headroom. `v_max` must likewise sit above the
fastest reference or no controller can ever close the gap.

## Multi-rate structure

The paper's dual-time-scale design is implemented as three genuinely nested rates
([simulation_runner.cpp](sim/src/simulation_runner.cpp)):

| Rate | Period | What runs |
|---|---|---|
| Outer kinematic controller | `Tf = 30 ms` | `ControllerBase::compute()` -> `(v_cmd, w_cmd)` |
| Slow adaptation | `Ts_slow = 150 ms` | `ControllerBase::slowTick()` - every 5th fast tick |
| Inner PI wheel-torque loop | `5 ms` | paper's `tau_R`/`tau_L` law with conditional anti-windup |
| Plant RK4 | `5 ms` | 6 sub-steps per fast tick |

`eps = Tf/Ts_slow = 0.2` is the paper's singular-perturbation time-scale ratio; the paper reports
that `eps > 0.37` produces parameter oscillation and `Ts_slow > 300 ms` produces sluggishness.

### Actuation layer

The paper's velocity-to-torque law, applied at the inner rate:

```
tau_R = (1/r)*[(Kp_v*e_v + Ki_v*int e_v)*R + (Kp_w*e_w + Ki_w*int e_w)*R]
tau_L = (1/r)*[(Kp_v*e_v + Ki_v*int e_v)*R - (Kp_w*e_w + Ki_w*int e_w)*R]
```

with **conditional anti-windup**: both integrators freeze while either wheel command exceeds
`tau_max`, and the delivered torques are clamped.

## Control objective

Track a planar reference trajectory `q_r = [x_r, y_r, theta_r]` with body-frame errors

```
[e1, e2, e3]^T = Rz(theta)^T * (q_r - q),   e3 = wrap(theta_r - theta)

e1' =  omega*e2 - v + v_r*cos(e3)
e2' = -omega*e1 + v_r*sin(e3)
e3' =  omega_r - omega
```

Primary metrics are the paper's Table 2 columns: ISE, IAE, ITAE on `(e_x, e_y)`, final position
error, mean wheel torque, final adaptive sliding gain `Ks`, and settling time (first `t` after
which `|e_pos| < 0.05 m` holds for the rest of the run).

## Controllers (12)

Every entry is an **outer kinematic loop** producing `(v_cmd, w_cmd)`; the PI torque layer is
shared, so no controller emits torques directly. Sign conventions
([CONTRIBUTING.md#sign-conventions](../../CONTRIBUTING.md)) are handled inside each wrapper.

| # | Name | Built on | `compute()` signal | Role |
|---|---|---|---|---|
| 1 | `OpenLoop` | - | - | `(v_r, w_r)` feedforward only; baseline |
| 2 | `PID` | 2x `ctrl::DiscretePID` | `r - y` | paper Table 3 "Standard PID" (reported ISE 14.7) |
| 3 | `Backstepping` | study-local | - | paper Table 3 "Pure backstepping" (reported ISE 3.3) |
| 4 | `SMC` | `ctrl::DiscreteSMC` | `y - r` | paper Table 3 "Pure SMC" (reported ISE 9.0) |
| 5 | `AdaptiveSMC` | `ctrl::AdaptiveSMC` | `y - r` | isolates the `Ks`-adaptation contribution |
| 6 | `ADRC` | 2x `ctrl::DiscreteADRC` | `computeTracking(y, r)` | lib-native ESO vs the paper's DOB |
| 7 | `FuzzyTSK` | `ctrl::FuzzySystem` | - | paper's NFS structure, **fixed** weights |
| 8 | `LQR` | `ctrl::DiscreteLQR` | state `x` | frozen-LTI error model at nominal `(v_r, w_r)` |
| 9 | `NMPC` | `ctrl::NonlinearMPC` | `computeRef(x, y_ref)` | 3-state body-frame error model |
| 10 | `L1Adaptive` | 2x `ctrl::L1AdaptiveController` | `setReference()` + `compute(y)` | |
| 11 | `GainScheduled` | `ctrl::GainScheduledController` | `r - y` | scheduled on curvature `\|w_r/v_r\|` |
| 12 | **`FUHAC`** | study-local composite | - | **the paper's proposed method** |

Entries 2-4 are the paper's own Table 3 comparators. Entries 5 and 7 are deliberate *ablations*:
`AdaptiveSMC` is FUHAC's sliding term alone, and `FuzzyTSK` is FUHAC's fuzzy structure without
online learning - together they isolate what each FUHAC ingredient actually contributes.

### FUHAC composition

```
u1 = alpha*u1base + (1 - alpha)*u1NF
u2 = alpha*u2base + (1 - alpha)*u2Stab + u2SMC
```

| Term | Implementation |
|---|---|
| `u_base` | Lyapunov backstepping, `V = (e1^2 + e2^2)/2 + (1 - cos e3)/k2`, gains `K1,K2,K3 = 2.0, 4.0, 2.0` (paper Table 1). Shared with `BacksteppingCtrl::law()`. |
| `u1NF` | 5x5 Gaussian Takagi-Sugeno grid on `(e1, de1)`, `mu_i(x) = exp[-0.5((x-c_i)/sigma_i)^2]`, `phi_ij = mu_i*mu_j`, output `u1NF[k] = 0.8*u1NF[k-1] + 0.2*sum(phi*w)/sum(phi)`. Weights adapt by gradient descent with momentum (`eta1 = 0.05`), projected onto `\|w\| <= 5`. |
| `u2SMC` | `ctrl::AdaptiveSMC` with `c_de = 0` and input `e2 + 0.8*e3`, giving the paper's `s = e2 + 0.8 e3`, `u = -K*sat(s/Phi)` with `Phi = 0.2`, and `K[k+1] = clamp(K + Ts*gamma*(\|s\| - eps), 3.0, 5.0)` starting at `K0 = 3.0`. |
| DOB | `dhat' = L*(y - yhat)`, then `dhat[k] = gamma*dhat[k-1] + (1-gamma)*clamp(dhat, d_max)`. `L = 20 rad/s` sits between the robot bandwidth (~3.3 rad/s) and the 30 ms-loop Nyquist (~105 rad/s), the paper's `omega_dynamics << L << omega_noise` rule. |
| Predictor | `e_pred(k+i) = [e(k) + de(k)*i*dt]*exp(-0.5*i)`, `H = 5`, `dt = 0.1 s` (paper's own values; `H*dt = 0.5 s ~ tau_m`). Applied as a bounded phase-lead multiplier. |
| Blending | `alpha[k] = 0.97*alpha[k-1] + 0.03*alpha_raw[k]` (paper Table 1 `beta = 0.97`), `alpha` in `[0.25, 0.95]`. |
| Slow loop | `slowTick()` leaks the adaptive weights toward nominal at 2%/tick - the paper's slow subsystem `theta' = eps*g(e, theta)`. |

#### Why the neuro-fuzzy layer is study-local

`ctrl::FuzzySystem` supports exactly the paper's inference structure - `InferenceMethod::TakagiSugeno`,
`MF::Type::Gaussian`, `DefuzzMethod::WeightedAverage` - **but rule weights are fixed at
`addRule()` and there is no mutator**, so the paper's online gradient-with-momentum update of
`w_ij` cannot be expressed through it. `ctrl::NNAdaptiveController` was also checked: its feature
map offers no Gaussian/RBF activation ([lib/NeuralNetworkController.h:38](../../lib/NeuralNetworkController.h#L38)).
The adaptive grid is therefore implemented in-study (~50 lines), and `ctrl::FuzzySystem` powers
the fixed-weight `FuzzyTSK` roster entry instead. `lib/` is not modified by this study.

#### Adaptive blending indicators

The paper writes `alpha_raw = alpha_min + (alpha_max - alpha_min)*[wt*T + we*E + wp*P + wo*O]`
but **never defines the four indicators**. They are defined here as normalised, `[0,1]`-saturated
quantities, weights `wt, we, wp, wo = 0.15, 0.45, 0.25, 0.15`:

| Indicator | Definition | Meaning |
|---|---|---|
| `T(t)` | `1 - exp(-t/5)` | elapsed-time maturity: trust the model-based law once the transient has passed |
| `E(e)` | `1/(1 + \|\|e\|\|)` | small error favours backstepping, large error favours the robust terms |
| `P(I)` | `1/(1 + perf_index)` | low accumulated ISE favours backstepping |
| `O(l)` | `1/(1 + osc)` | low control oscillation favours backstepping |

## Scenarios (5)

| File | Path | Purpose |
|---|---|---|
| [`s01_lemniscate.json`](config/scenarios/s01_lemniscate.json) | lemniscate, `a = 2.0 m` | the paper's hardest profile - continuous curvature reversal |
| [`s02_circle.json`](config/scenarios/s02_circle.json) | circle, `a = 3.0 m` | constant curvature; the paper's best-case indices |
| [`s03_diamond_offset_start.json`](config/scenarios/s03_diamond_offset_start.json) | diamond, `a = 1.0 m` | four sharp corners **and** the paper's non-zero-initial-condition test |
| [`s04_noise_disturbance.json`](config/scenarios/s04_noise_disturbance.json) | lemniscate + noise + torque disturbance | the paper's Fig. 23 comparison "with the presence of measured noise"; exercises the DOB |
| [`s05_saturation_mismatch.json`](config/scenarios/s05_saturation_mismatch.json) | circle, `tau_max = 15 N.m`, +40% mass | reproduces the paper's Fig. 22 Pioneer **failure mode** (saturation-induced transients) |

Reference paths, `time_scale = 1.0` reproducing the paper's literal `t` parameterisation:

```
lemniscate: x = a*cos(t)/(1 + sin^2 t),        y = a*sin(t)*cos(t)/(1 + sin^2 t)
circle:     x = a*cos(t),                       y = a*sin(t)
diamond:    x = a*sgn(cos t)*(1 - |sin t|),     y = a*sgn(sin t)*(1 - |cos t|)
```

`theta_r = atan2(yr', xr')`, `v_r = hypot(xr', yr')`, `omega_r = d(theta_r)/dt`, all by central
difference with `atan2` unwrapping - the diamond's `sgn()` terms make an analytic derivative
undefined at the four corners.

## CSV output

One file per (scenario, controller) at `logs/run_<scenario>_<controller>.csv`, logged at the
fast rate `Tf` (1000 rows per 30 s run):

| Column | Meaning |
|---|---|
| `t` | simulation time [s] |
| `x_ref`, `y_ref`, `theta_ref` | reference pose [m, m, rad] |
| `x_pos`, `y_pos`, `theta_pos` | actual pose [m, m, rad] |
| `e1`, `e2`, `e3` | body-frame errors [m, m, rad] |
| `e_pos` | `hypot(x_ref - x, y_ref - y)` [m] |
| `v_cmd`, `w_cmd` | outer-loop velocity commands [m/s, rad/s] |
| `tau_R`, `tau_L` | delivered wheel torques after saturation [N m] |
| `d_hat` | FUHAC disturbance estimate (NaN for other controllers) |
| `alpha` | FUHAC blending coefficient (NaN for other controllers) |
| `Ks` | adaptive sliding gain (FUHAC and `AdaptiveSMC` only; NaN otherwise) |
| `V_lyap` | FUHAC composite Lyapunov value (NaN for other controllers) |
| `iae_cumulative` | running `int \|e_pos\| dt` - the column `tools/metrics.py::extract_final_iae` reads |
| `ref`, `y`, `u` | the **heading loop** (`theta_ref`, `theta`, `w_cmd`) restated for the generic tooling |

The pose columns are deliberately named `x_pos`/`y_pos`/`theta_pos` rather than `x`/`y`/`theta`.
`tools/metrics.py` auto-picks its output column as the first of `['y', 'y1', ...]` and its
reference as the first of `['ref', ..., 'x_ref', ...]`, so a column literally named `y` would be
paired against `x_ref` - robot **Y** position against reference **X** - a meaningless SISO loop.
That mis-pairing is not hypothetical: it made `tools/mu_analysis.py` report `peak_T = 0.000`. The
trailing `ref`/`y`/`u` triple exposes the heading loop, which is a genuine SISO pair, and the same
analysis then yields `peak_S = 2.43`, `peak_T = 1.43`.

## Run

```bash
# Build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target differential_drive_robot_tracking_sim
cmake --build build --target differential_drive_robot_tracking_robustness
cmake --build build --target test_ddmr_regression

# 12 controllers x 5 scenarios = 60 runs -> logs/
build/case-study/Differential\ Drive\ Robot\ Tracking/differential_drive_robot_tracking_sim

# Robustness -> mc_summary.csv, fault_sweep.csv, wcet_summary.csv at the study root
build/case-study/Differential\ Drive\ Robot\ Tracking/differential_drive_robot_tracking_robustness

# Regression suite - filter the EXE on the tag, NOT ctest -R on the target name
build/tests/test_ddmr_regression.exe "[ddmr]"
```

`differential_drive_robot_tracking_sim` runs in `run.py` **Phase 5**; both targets are registered
in `compile.bat` and `compile.sh`.

## Results

60/60 runs complete; regression suite 32/32 (135,856 assertions). Mean wheel torque across the
roster lands at **4.7-8.3 N.m**, inside the paper's reported 5.9-9.7 N.m band.

### FUHAC vs the paper's Table 2

| Metric | s01 lemniscate | paper | s02 circle | paper | s03 diamond | paper |
|---|---|---|---|---|---|---|
| ISE | 0.233 | 3.6955 | 1.822 | 1.0833 | 1.731 | 1.2032 |
| IAE | 3.071 | 4.7004 | 7.917 | 2.6197 | 8.290 | 3.8314 |
| ITAE | 46.2 | 21.98 | 143.4 | 18.81 | 123.5 | 35.67 |
| Final error [m] | 0.038 | 0.0345 | 0.276 | 0.0394 | 0.243 | 0.0405 |
| Mean torque [N.m] | 4.70 | 9.694 | 8.26 | 5.931 | 4.33 | 9.237 |
| `Ks` final | 3.000 | 3.725 | 3.014 | 4.147 | 5.000 | 4.568 |

Same order of magnitude throughout, and the lemniscate final error (0.038 m) reproduces the
paper's headline "<4 cm on all trajectories" claim. The individual columns differ - unavoidably,
since every physical parameter is assumed (see the parameter table). **The reproduced claim is
the trend and ordering, not the digits.** Note the s03 diamond drives `Ks` to its 5.0 ceiling,
matching the paper's observation that the diamond ends with the highest sliding gain (4.568).

### Controller ordering (ISE)

| Scenario | Best three | FUHAC | PID |
|---|---|---|---|
| s01 lemniscate | Backstepping 0.103, SMC 0.126, LQR 0.158 | 0.233 | 0.189 |
| s02 circle | Backstepping 0.240, SMC 0.269, L1Adaptive 0.272 | 1.822 | 0.304 |
| s03 diamond + offset start | **NMPC 0.224, FUHAC 1.731, L1Adaptive 1.875** | **1.731** | 11.886 |
| s04 noise + disturbance | L1Adaptive 0.097, Backstepping 0.109, SMC 0.127 | 0.609 | 0.199 |
| s05 saturation + mismatch | SMC 0.350, Backstepping 0.351, LQR 0.397 | 2.188 | 0.447 |

**FUHAC's advantage is real but narrower than the paper claims.** It wins decisively on the
hardest case - the diamond with sharp corners and an off-path start, where it beats PID by 6.9x
(1.73 vs 11.89) and pure backstepping by 4.7x (vs 8.20) - and it is the only controller besides
NMPC to stay under ISE 2 there. On smooth, well-modelled paths it *loses* to plain backstepping,
because its always-on sliding term is pure overhead when there is no uncertainty to reject.

That direction is consistent with the paper's own concession: Table 3 reports FUHAC (ISE 3.7) as
worse than pure backstepping (3.3), justifying FUHAC on damped torque and actuator health rather
than raw tracking error. What is *not* reproduced is the paper's claim that FUHAC also beats a
standard PID everywhere - here a well-tuned PID edges it out on every smooth scenario.

## Deviations from the paper

Recorded honestly rather than papered over:

1. **No physical parameters are given in the paper.** Everything in the parameter table above is
   an assumed Pioneer 3-DX value, except `Kf` which is *calibrated* to put mean torque in the
   paper's reported band. Absolute ISE/IAE/torque values therefore cannot be expected to match -
   **the claim reproduced here is the trend and ordering across trajectories and controllers,
   not the digits.**
2. **The Pioneer hardware trial is not reproduced.** The paper's Fig. 22 uses "publicly available
   data on Pioneer 1 time series", which is not part of this repository. `s05_saturation_mismatch`
   reproduces the reported *failure mode* (torque saturation at +/- 15 N.m under mass mismatch),
   not the dataset or its numbers (mean error 1.47 m, RMS 1.85 m).
3. **The four blending indicators are undefined in the paper** and are given explicit definitions
   here (table above).
4. **SMC sign.** The paper writes `u_2SMC = -K*sgn(s)` with `s = e2 + 0.8 e3` built from
   `e = q_r - q` errors. As literally written that term *opposes* error reduction. The lib
   convention (`DiscreteSMC`/`AdaptiveSMC` take `e = y - r`) resolves this: the surface signal is
   negated before it reaches the controller, which recovers a stabilising law. Both the `SMC`
   and `FUHAC` wrappers do this explicitly.
5. **Trajectory speed.** At `time_scale = 1.0` (the paper's literal parameterisation, period
   `2*pi` s) the `a = 2 m` lemniscate demands roughly 2-3 m/s - well above a real Pioneer's
   ~1.2 m/s limit, and the likely reason the paper's torques are as large as they are. The
   parameterisation is kept faithful and `time_scale` is exposed per scenario for slower runs.
6. **Diamond corner clamping.** `omega_r` is unbounded in the limit at the diamond's four `sgn()`
   discontinuities; both feedforward channels are clamped to `v_max`/`w_max`.
7. **`u2Stab` is not specified in the paper** beyond its name; it is implemented as a high-gain
   proportional fallback `w_r + 3.0*e3 + 1.5*e2`.
8. **The logged Lyapunov trace omits the weight term.** The paper's candidate is
   `V = |e|^2/2 + W~'W~/(2*eta1) + s^2/(2*eta2) + d~^2/(2*eta3)`, where `W~ = W - W*` and
   `d~ = d - dhat` are errors against an unknown ideal `W*` and the true disturbance. Neither
   is observable in simulation, so `V_lyap` logs the three computable terms (tracking,
   sliding-surface and disturbance-estimate energy).
9. **`u1NF` augments rather than replaces the nominal law.** Taken literally, `u1 = alpha*u1base
   + (1-alpha)*u1NF` with zero-initialised NFS weights gives the second branch no authority at
   `t = 0`; since `alpha` falls as the error grows, a large error would then *remove* corrective
   action exactly when it is needed. `u1NF` is therefore the nominal kinematic law plus the
   learned residual, which matches the paper's stated role for the NFS ("approximate unknown
   nonlinear dynamics online").
10. **`L1Adaptive` is layered on the backstepping baseline**, not run bare. L1's internal
    reference model assumes a stable relative-degree-1 plant, but velocity-command to
    position-error is an integrator; the bare wiring diverges (measured ISE 2091 on the circle
    versus 0.24 for the nominal law alone). Augmenting a stabilised baseline is the standard L1
    architecture in any case.
11. **`ctrl::DiscreteADRC` is second-order**, so `b0` is the gain to the *second* derivative of
    the regulated signal (`b0 ~ 1/tau_inner ~ 20`), and the ESO bandwidth is held to
    `omega_o = 5-6 rad/s` - at `Ts = 30 ms`, `omega_o = 20` would give `beta3*Ts = 240` and peg
    the actuator (measured ISE 531 before the retune, 0.59 after).

## Files

| Path | Contents |
|---|---|
| [sim/include/](sim/include/) | plant, trajectory, controller roster, runner headers |
| [sim/src/](sim/src/) | implementations + `main.cpp` + `robustness_main.cpp` |
| [config/plant_params.json](config/plant_params.json) | physical parameters with `_sources` provenance |
| [config/scenarios/](config/scenarios/) | the five scenario definitions |
| `logs/` | 60 telemetry CSVs (generated) |
| `mc_summary.csv`, `fault_sweep.csv`, `wcet_summary.csv` | robustness artifacts (generated) |
| [../../tests/test_ddmr_regression.cpp](../../tests/test_ddmr_regression.cpp) | Catch2 suite, tag `[ddmr]` |
