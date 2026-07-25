# Boiler Control - MATLAB-native case study

A **MATLAB-only** re-implementation of the [Boiler Control](../Boiler%20Control/) case
study, built directly on the MATLAB R2026a toolboxes instead of the C++ `lib/`
core. It is a *separate, parallel* study: it reads its own `config/`, writes its
own `logs/` and `mc_summary.csv`, and **never touches the C++ study's data**.

> **Why this exists.** With the full R2026a stack installed (Control System, MPC,
> Robust Control, System Identification, Optimization, Fuzzy Logic toolboxes),
> hand-porting the C++ controller classes into MATLAB would be wasted effort -
> MATLAB already ships `dlqr`, `kalman`/`dlqe`, `c2d`, `quadprog`, `mixsyn`,
> `n4sid`, etc. So this study leverages those toolboxes to reproduce the same
> plant, scenarios, and 27-controller roster natively. See
> [`../../MATLAB/HANDOFF.md`](../../MATLAB/HANDOFF.md) Section 0 for the decision record.

## What it produces

The **exact 23-column telemetry schema** of the C++ `telemetry_logger`, one file
per (scenario * controller):

```
logs/run_<scenario_id>_<controller_name>.csv
t,y1,y2,y3,u1,u2,u3,du1,du2,du3,ref_y1,ref_y2,ref_y3,e1,e2,e3,IAE_y1,IAE_y2,IAE_y3,ISE_y1,ISE_y2,ISE_y3,E_valve
```

plus a root `mc_summary.csv`
(`study,controller,sample_id,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var,stable`)
so the study is discovered by the Python report pipeline
(`tools/generate_report.py` globs `case-study/*/mc_summary.csv`;
`tools/compare_controllers.py` globs `case-study/*/**/logs/run_*.csv`) with **no
Python changes**.

## How to run

```matlab
% From MATLAB, anywhere:
addpath('case-study/Boiler Control MATLAB/matlab');
run_all                                              % full 8 x 27 sweep -> logs/, mc_summary.csv
run_all('scenarios', {'s01_lowload_regulation'})     % one scenario
run_all('controllers', {'PID','LQR','MPC'})          % subset (class names, see below)
run_all('duration', 120)                             % short smoke run

analyze                                              % rank controllers per scenario from the logs
```

Headless:

```bash
matlab -batch "addpath('case-study/Boiler Control MATLAB/matlab'); run_all()"
```

Last verified: **27/27 controllers construct, run, and log stably** on R2026a
Update 3 (smoke: s01, 60 steps, 0 failures).

## Architecture (`matlab/`)

- **`+boiler/`** - plant + infrastructure (toolbox-light):
  `BoilerTurbine` (nonlinear forward-Euler plant, faithful port of
  `boiler_plant.cpp`), `linearize` (analytic Jacobians + `c2d` ZOH),
  `computeNbar`, `diagonalChannel`, `loadScenario` (`jsondecode`),
  `TelemetryLogger` (conformant CSV), `runSimulation` (the step loop).
- **`+bctrl/`** - the 27 controllers as `bctrl.Controller` subclasses plus shared
  building blocks (`DPID`, `LeadLag`, `SMCax`, `KF`, `CondensedMPC`). Toolbox
  usage: `dlqr` (LQR/LQG/gain-scheduling), `dlqe`/`idare` (Kalman), `quadprog`
  (all MPC/GPC/NMPC), `mixsyn`/`d2c`/`c2d` (H-inf), `n4sid`/`iddata` (subspace ID).
- **`run_all.m`** - the driver. **`analyze.m`** - per-scenario IAE/RMS ranking.

Controller class names (for the `'controllers'` filter), in sim order:
`PID LQR LQG MPC SMC ESC ADRC LeadLagPID SmithPredictor GPCRLS EKFLQR UKFLQR
FuzzyPID FuzzySupMPC SupervisoryStack AdditiveStack WeightedStack RepetitiveCtrl
MRAC Hinf AdaptiveSP NMPC FL MHELQR LPVGS SubspaceIDLQG AutoGS`.

## Deliberate deviations from the C++ study

These are intentional - a *cleaner* parallel, not a bug-for-bug clone:

1. **s07 transition reference.** The MATLAB loop regulates to the **new**
   operating point after the transition (`ref_dy` stays relative to the current
   op). The C++ log encodes `ref_abs = y0 + (y_new - y_old)`, doubling the target.
2. **MRAC in deviation space.** The C++ MRAC drives an absolute valve from an
   absolute (pressure-scaled) reference, which is ill-scaled; here MRAC adapts in
   deviation space (`du = theta_r.ref_dy - theta_y.dy`), which is numerically sane.
3. **Toolbox algorithms, not the C++ classes.** Gains/behaviour will not match the
   C++ logs sample-for-sample; the point is a native-MATLAB rendering of the same
   *methods* on the same plant and scenarios. A few controllers (`FuzzyPID`,
   RLS-in-`GPC-RLS`, delay-estimation in `AdaptiveSP`) are faithful-in-spirit
   MATLAB renderings rather than exact ports.

Robustness: every controller clamps its increment to +/-0.5, the plant clamps
valves to [0,1] with per-step rate limits, and all toolbox synthesis
(`mixsyn`, `n4sid`, per-point `dlqr`) is wrapped so a failure degrades to a
PID/LQR fallback - a full `run_all` never aborts.
