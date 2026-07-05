# Satellite Launch Vehicle Systems

Pitch-plane attitude control of an aerodynamically **unstable, time-varying**
Satellite Launch Vehicle (SLV) during the atmospheric phase of ascent.

## Source

- **Reference:** A.P. Nair, N. Selvaganesan, V.R. Lalithambika, *"Lyapunov based
  PD/PID in model reference adaptive control for satellite launch vehicle
  systems"*, Aerospace Science and Technology **51** (2016) 70-77.
- Paper PDF in this folder (extract text with `tools/extract_text.py` if needed; extraction is lossy).

## Plant model

Single-plane rigid-body short-period dynamics (paper Eq. 1-3), with the lateral
drift term `z_dot/V` neglected in the high-dynamic-pressure regime:

```
theta_ddot = mu_alpha(t) * (theta + alpha_w) + mu_c(t) * delta
```

| Symbol | Meaning | Units |
|---|---|---|
| `theta`, `theta_dot` | pitch attitude / rate (state) | rad, rad/s |
| `delta` | thrust-deflection (gimbal) angle (**control input**, \|delta\| <= 8 deg) | rad |
| `alpha_w = -Vw/V` | wind angle of attack (disturbance) | rad |
| `mu_alpha(t)` | aerodynamic-moment coeff `L_alpha l_alpha / I` (**> 0 => unstable**, pole at `+sqrt(mu_alpha)`) | 1/s^2 |
| `mu_c(t)` | control-moment coeff `T_c l_c / I` | 1/s^2 |

Both `mu_alpha` and `mu_c` are **time varying** (thrust, mass, inertia, dynamic
pressure all change during ascent - paper Remark 1 / Fig. 3). `mu_alpha` peaks in
the transonic band (~50 s), making the plant most unstable there. Integrator:
RK4 at `Ts = 0.05 s`; runs for `T_sim = 100 s`.

### Model simplifications (control-oriented reconstruction)

1. **`mu_alpha(t)`, `mu_c(t)` profiles.** The paper gives them graphically only
   (Fig. 3); numeric values are not recoverable from the PDF. This study uses a
   smooth, physically-representative reconstruction: `mu_alpha` a Gaussian bump
   peaking transonic, `mu_c` a slow linear decay as propellant depletes. All
   profile parameters live in `config/plant_params.json` (queryable, not baked in).
2. **Drift/force equation** dropped exactly as the paper does for the aerodynamic
   phase, leaving the `theta/delta` 2nd-order pair.
3. **Actuator** second-order dynamics + slew/position limits reduced to a slew
   limit (`delta_rate_max`) + position limit (`delta_max = 8 deg`).
4. **Breakup/tumble saturation** clamps `|theta| <= 1 rad`, `|theta_dot| <= 5 rad/s`
   so the uncontrolled `OpenLoop` baseline gives a finite (large) IAE, not an
   inf/NaN overflow.

## Control objective

Track the guidance attitude command `theta_ref(t)` while stabilising the unstable,
time-varying plant using no more than 8 deg of gimbal authority. Primary metric:
**IAE** = integral of `|theta_ref - theta| dt` (the runner also logs ISE, the
paper's "integral absolute square error", and peak control demand).

Sign convention: the plant is **positive-gain** (`mu_c > 0`), so error-driven
controllers use `e = ref - theta` with positive gains. Note this is opposite to
the negative-gain Aircraft/Solar-Cooker studies - the first-order `DiscreteSMC`
and super-twisting SMC set up a divergent discrete bang-bang limit cycle here and
are therefore **not** in the roster (LQG takes the robust-control slot instead).

## Controllers (12)

| # | Controller | Notes |
|---|---|---|
| 1 | OpenLoop | zero gimbal - uncontrolled unstable baseline (diverges to the tumble clamp) |
| 2 | PID | fixed-gain attitude autopilot |
| 3 | GainScheduledPD | paper baseline: PD gains scheduled on flight time (no integral - weakest practical controller, as in the paper) |
| 4 | GainScheduledPID | paper baseline: PID gains scheduled on flight time |
| 5 | **AdaptivePID (proposed)** | **Lyapunov MRAC augmentation of a stabilising PID** - the paper's adaptive-PID-in-MRAC method |
| 6 | L1Adaptive | fast adaptive augmentation of the same PID baseline (ref [5] L1 architecture) |
| 7 | LQR | full-state optimal feedback on the transonic design model |
| 8 | LeadLag | classical phase-lead compensator |
| 9 | LQG | Kalman estimator + LQR (output feedback; H2 candidate from the selection matrix) |
| 10 | ADRC | active disturbance rejection (ESO lumps `mu_alpha*theta` as total disturbance) |
| 11 | MPC | constrained receding-horizon control (hard 8-deg gimbal limit) |
| 12 | SelfTuningRegulator | online RLS identification + pole placement (targets the time-varying theme) |

The proposed **AdaptivePID** realises the paper's Lyapunov-based adaptive PID as an
MRAC augmentation of a fixed stabilising PID (the augmentation architecture of
refs [10]/[11]), because `ctrl_toolbox`'s `MRACController` is a first-order
model-reference law that cannot stabilise a 2nd-order unstable plant unaided.

## Scenarios (5)

| id | Description |
|---|---|
| `s01_step_command` | attitude step 0 -> 0.10 rad at t=5 s |
| `s02_ramp_command` | ramp 0 -> 0.12 rad over 60 s (paper Fig. 4); nominal MC/analysis scenario |
| `s03_guidance_command` | piecewise-constant guidance program (paper Fig. 7) |
| `s04_param_perturbation` | robustness: transonic instability raised (`mu_alpha_peak` 4.0->6.0), paper Sec. 4.2.1 |
| `s05_wind_gust` | wind-disturbance rejection: raised-cosine gust in the transonic band (paper Sec. 4.2.2) |

## Run

```
conda run -n soft_robotics -- python "case-study/Satellite Launch Vehicle Systems/sim/main.py"
```

Analysis artifacts (Monte Carlo, fault sweep, WCET, mu-analysis, HTML report) are
produced by `tools/run_analysis.py` + `tools/generate_report.py` per
`config/analysis.json`.

## CSV columns (`logs/run_<scenario>_<controller>.csv`)

`time, theta_ref, theta, theta_dot, delta, alpha_w, error, iae_cumulative`
