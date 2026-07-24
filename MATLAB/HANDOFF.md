# MATLAB Conversion — Handoff & Future Plan

> Status: **proposal / handoff**, written 2026-07-12. Nothing here changes `lib/`, `tools/`,
> `run.py`, or the existing case studies — it is the roadmap for the MATLAB port plus a small
> working starter kit (see [§9](#9-starter-kit-whats-already-here)). The roadmap is a proposal for
> whoever owns the effort; the data-contract facts in [§4](#4-the-data-contract-matlab-must-honour)
> are load-bearing and should not be changed lightly.

---

## 0. Decision record — pivot away from the core-library port (2026-07-12)

**Verdict (owner call):** *do not* spend effort recreating the `lib/` core in MATLAB. With the full
R2026a stack installed (Control System, MPC, Robust Control, System Identification, Optimization,
Fuzzy Logic toolboxes), hand-porting `DiscretePID`/`DiscreteLQR`/`KalmanFilter`/… duplicates what
`dlqr`, `kalman`/`dlqe`, `c2d`, `quadprog`, `mixsyn`, `n4sid`, … already provide. Instead, build
**separate MATLAB-native case studies** directly on the toolboxes.

Consequences:

- The `MATLAB/+ctrl/` native port (§9, Phase 2) is **deprecated / frozen** — kept for reference, no
  longer extended. `+ctrlanalysis/` (log analysis + metrics) **stays** and is reused by the new
  studies.
- **First realized study:** [`case-study/Boiler Control MATLAB/`](../case-study/Boiler%20Control%20MATLAB/)
  — a self-contained MATLAB twin of Boiler Control: nonlinear Bell-Åström plant, all 8 scenarios,
  the full **27-controller** roster rebuilt on the toolboxes (`quadprog` MPC/GPC/NMPC, `mixsyn`
  H-inf, `n4sid` subspace-ID LQG, EKF/UKF, gain scheduling, …). It writes conformant
  `logs/run_*.csv` + `mc_summary.csv` into its **own** directory (never clobbers the C++ study) and
  is discovered by the report pipeline as a distinct study. Smoke-verified 27/27 stable on R2026a.
  Run: `matlab -batch "addpath('case-study/Boiler Control MATLAB/matlab'); run_all()"`.
- The Tier-1/2/3 framing (§5) still applies, but the target is now *native re-simulation per study*
  (Tier 3) built on toolboxes, not a monolithic `+ctrl` re-expression of `lib/`.

Everything below §1 is the **original** (pre-verdict) plan; read it as background, but §0 supersedes
the "native `.m` port of `lib/`" goal wherever they conflict.

---

## 1. Purpose & audience

`MATLAB/README.md` states the intent: *"MATLAB conversion of the C++ controller toolbox for the
MATLAB-main user … to have a better look into what is already inside the codebase."*

Read literally, the goal is **legibility for a MATLAB-first reader**, not a second production
runtime. So the target is:

- A **native, readable `.m` port** of the `lib/` core — controllers, plant models, estimators —
  written the way a MATLAB user would read them (namespace package `+ctrl`, `classdef`,
  vectorised), so the algorithms can be *studied* in MATLAB.
- A **MATLAB analysis layer** over the existing case studies, so a MATLAB user can load, re-derive,
  visualise, and eventually re-simulate the results that the C++/Python pipeline already produces.

Explicit **non-goals** (call these out to avoid scope creep):

- Not a MEX / `codegen` interop shim around the C++ (`lib/` stays the source of truth; the point is
  a readable re-expression, not calling the same binary).
- Not a hardware / RT target — the embedded, HAL, RTOS, and ROS 2 surfaces (`lib/embedded/`,
  `lib/hal/`, `ros2/`) are out of scope for MATLAB.
- Not a big-bang parity port of all ~90 classes + 126 C++ / 152 Python examples + Catch2 tests up
  front. Port **incrementally**, driven by what the case studies actually exercise.

---

## 2. Current state

- `MATLAB/` contains only `README.md` (2 lines) and this file. No `.m` code beyond the starter kit
  in [§9](#9-starter-kit-whats-already-here).
- The repo has **zero `.m` awareness**: no MATLAB runner, no registration, and the status tracker
  ([`../tools/case_study_tracker.py`](../tools/case_study_tracker.py)) detects only C++ vs Python.
- MATLAB appears in the codebase today only as *citations* — cheatsheets and
  [`../docs/algorithm_backlog.md`](../docs/algorithm_backlog.md) benchmark this toolbox against the
  MATLAB Control System / Robust Control Toolboxes. That backlog is a useful parity checklist when
  deciding port order.

---

## 3. Target architecture

```
MATLAB/
  README.md                 # keep; now points here
  HANDOFF.md                # this file
  +ctrl/                    # native .m port of lib/ core -> called as ctrl.DiscretePID(...)
    TransferFunction.m  StateSpace.m  tf2ss.m                                             [DONE]
    DiscretePID.m  DiscreteLQR.m  LQRAdapter.m  KalmanFilter.m                            [DONE]
    (more controllers ported incrementally; port-order in §6)
  +ctrlanalysis/            # reusable analysis toolbox (language-agnostic on the data side)
    load_run.m              # read case-study/<S>/logs/run_*.csv           -> struct/table   [DONE]
    compute_metrics.m       # faithful port of tools/metrics.py::compute_metrics (6 keys)   [DONE]
    plot_scenarios.m        # per-scenario / per-controller comparison plots               [TODO]
    write_summary.m         # emit mc_summary/fault_sweep/... in the fixed schemas          [TODO]
  examples/                 # MATLAB verification demos
    verify_ex01_tf_pid.m    # reproduces build/examples/ex01_tf_pid.exe (plant + PID)       [DONE]
    verify_lqr_kalman.m     # DARE residual + dlqr cross-check + observer convergence       [DONE]

case-study/<Study>/matlab/  # per-study drivers, co-located with sim/ config/ logs/
    analyze.m               # load this study's logs, compute metrics, plot, print PASS/FAIL
```

Two conventions worth stating up front:

- **`+ctrl` / `+ctrlanalysis` are MATLAB *namespace packages*** (the leading `+`). Callers write
  `ctrl.DiscretePID(...)` / `ctrlanalysis.compute_metrics(...)` after adding `MATLAB/` to the path.
  This mirrors the flat-but-namespaced feel of the C++ `ctrl::` namespace and avoids polluting the
  global function namespace.
- **Per-study MATLAB lives in `case-study/<Study>/matlab/`**, next to that study's `sim/`,
  `config/`, and `logs/` — the same way `sim/` already sits inside each study. A study stays
  self-contained, and the driver reads its neighbour `../logs/run_*.csv` with a relative path.

---

## 4. The data contract MATLAB must honour

This is the single most important section: the Python report pipeline discovers data **by filename
glob, not by language**. If MATLAB writes files that match the conventions below, they flow into the
existing reports with **zero changes to any Python tool**.

**Time-series logs** — one file per (scenario × controller):

```
case-study/<Study>/logs/run_{scenario_id}_{controller_name}.csv
```

- The `run_` prefix is mandatory — discovery is `rglob("logs/run_*.csv")` in
  [`../tools/generate_report.py`](../tools/generate_report.py) and
  [`../tools/compare_controllers.py`](../tools/compare_controllers.py). Filename parsing splits on
  `_` to recover scenario and controller. Contract documented in
  [`../tools/study_protocol.py`](../tools/study_protocol.py) (lines 70-82).
- **Column schema is study-specific**, not uniform. IAE is recovered heuristically by
  [`../tools/metrics.py`](../tools/metrics.py) `extract_final_iae` / `IAE_COL_CANDIDATES`: it reads
  a scalar `iae_cumulative` / `iae` / `IAE_cumulative`, or **sums** MIMO `IAE_y1,IAE_y2,…` columns,
  or trapezoid-integrates an `error`/`e1`/`e` column. **So a MATLAB-produced log must expose at
  least one recognised IAE column** (a cumulative `IAE_y*` per output is the safest for MIMO).
  Example header (Boiler, C++ `telemetry_logger`):
  `t,y1,y2,y3,u1,u2,u3,du1,du2,du3,ref_y1,ref_y2,ref_y3,e1,e2,e3,IAE_y1,IAE_y2,IAE_y3,ISE_y1,ISE_y2,ISE_y3,E_valve`.

**Per-study summary CSVs** (written at study root; consumed by report sections 4-8). Match these
headers exactly:

| File | Header (exact) |
|------|----------------|
| `mc_summary.csv` | `study,controller,sample_id,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var,stable` |
| `fault_sweep.csv` | `study,controller,fault_kind,magnitude,iae,rms_error,settle_time_s,overshoot_pct,max_u,energy_var` |
| `wcet_summary.csv` | `controller,n_samples,mean_us,median_us,p99_us,wcet_us,max_us` |
| `mu_analysis.csv` | `study,scenario,peak_S,peak_T,status` |

> Python-only studies emit a wider `mc_summary` (extra result-dict keys pass through); the columns
> above are the **C++ self-contained variant** and the minimum the report needs. A MATLAB path
> should follow the C++ variant (see [§5](#5-case-study-analysis-integration-plan)).

**Metric definitions** MATLAB must reproduce (from `compute_metrics`, error `e = ref - y`):

- `iae` = Σ |e[k]|·Δt (left Riemann, drops last sample).
- `rms_error` = sqrt(mean(e²)).
- `settle_time_s` = first `t` after which |e| stays within `settle_band·|e[0]|` for
  `settle_hysteresis` consecutive samples (defaults 0.02 and 10); `-1` if never.
- `overshoot_pct` = signed peak beyond `ref[end]`, in % of |ref[end]|; 0 if |ref[end]| < 1e-9.
- `max_u` = max|u|. `energy_var` = var(u).

The starter kit's `compute_metrics.m` already ports these exactly — keep it in sync if
`tools/metrics.py` ever changes.

---

## 5. Case-study analysis integration plan

Three tiers, ordered easiest → deepest. This is the recommended sequence for "adding MATLAB files
into case-study analysis."

**Tier 1 — Read & cross-validate (no re-sim).** `case-study/<Study>/matlab/analyze.m` loads the
existing `logs/run_*.csv` via `ctrlanalysis.load_run`, recomputes the 6 metrics with
`ctrlanalysis.compute_metrics`, and asserts they match the values already baked into the logs /
Python tooling within tolerance. This is the fastest "look into what's inside the codebase" and
doubles as a regression check on the metric definitions. **Start with one Complete study — Boiler
Control** (8 scenarios × 27 controllers already logged). The starter kit ships exactly this for
Boiler.

**Tier 2 — MATLAB visualisation.** The same driver adds native MATLAB plots (per-scenario tracking
`y` vs `ref`, IAE bar chart across controllers, control-effort/`u` traces) — a MATLAB-native
parallel to the plotly `report.html`, for users who prefer to explore in-tool. Factor the shared
parts into `+ctrlanalysis/plot_scenarios.m`.

**Tier 3 — MATLAB re-simulation.** Using the ported `+ctrl` classes and a MATLAB plant model, re-run
plant + controller and write **conformant** `logs/run_{scenario}_{controller}.csv` (plus, if you add
a robustness driver, the four summary CSVs in the exact schemas above). Because discovery is by
glob, these feed `docs/report.html` automatically. Cross-check MATLAB trajectories against the
C++/Python logs to validate the port class-by-class.

### How MATLAB reaches the aggregate report — two options

- **(a) Passive — recommended.** MATLAB only writes conformant CSVs; the existing Python tools pick
  them up unchanged. **No `run.py` change, no tracker change.** This is the whole payoff of the
  discovery-by-glob design and should be the default through Tier 3.
- **(b) Active — deferred to Phase 4.** Add a MATLAB execution phase to
  [`../run.py`](../run.py) (a sibling of Phase 5 / Phase 7) that shells out to
  `matlab -batch` or `octave --no-gui`, plus a `.m` language tier + status detection in
  [`../tools/case_study_tracker.py`](../tools/case_study_tracker.py). Only worth doing once MATLAB
  (or Octave) is reliably available in CI. Note the **analysis-hook contract**
  ([`../tools/study_protocol.py`](../tools/study_protocol.py)) is Python-`importlib`-based and
  cannot import `.m` files — a MATLAB study must instead mirror the self-contained C++
  `*_robustness` executable pattern (compute and write the four summary CSVs itself), not try to
  join `tools/run_analysis.py`'s hook path.

---

## 6. Phased roadmap

| Phase | Goal | First concrete files | Exit criterion |
|-------|------|----------------------|----------------|
| **0. Starter kit** | Prove Tier 1 on one study | `+ctrlanalysis/{load_run,compute_metrics}.m`, `case-study/Boiler Control/matlab/analyze.m` | `analyze.m` runs on checked-in logs and prints PASS |
| **1. Analysis layer** | Tier 1+2 across all Complete studies | `+ctrlanalysis/plot_scenarios.m`; an `analyze.m` per Complete study | Every Complete study has a working `matlab/analyze.m` |
| **2. Core port** | Readable `+ctrl` for the controllers the studies use | `+ctrl/{DiscretePID,StateSpace,DiscreteLQR,KalmanFilter}.m` | A MATLAB step-loop reproduces one C++ example within tolerance — **DONE** (`verify_ex01_tf_pid`, max dev 4.5e-4) |
| **3. Per-study re-sim** | Tier 3 for a pilot study | `case-study/Boiler Control/matlab/simulate.m` + `+ctrlanalysis/write_summary.m` | MATLAB-written `run_*.csv` appears in `docs/report.html` |
| **4. Pipeline integration** | Option (b) active path | `run.py` MATLAB phase + `.m` tier in `case_study_tracker.py` | `python run.py` exercises MATLAB studies in CI |

Suggested **port order for `+ctrl`** (Phase 2), driven by case-study usage and the
[`../docs/controller_selection_matrix.md`](../docs/controller_selection_matrix.md) roster: start
with the controllers that appear in nearly every study's roster — `DiscretePID`, `StateSpace` /
`tf2ss`, `DiscreteLQR` (+ `LQRAdapter` split), `KalmanFilter` — then branch to `DiscreteSMC`,
`DiscreteMPC`, `ADRC` as specific studies need them. Mind the per-controller sign conventions
(`CONTRIBUTING.md`): e.g. PID uses `e = r - y` but `DiscreteSMC` uses `e = y - r`.

---

## 7. Open decisions (need an owner's call)

1. **MATLAB vs GNU Octave, and toolbox dependence.** The starter kit is written to run in **both**
   base MATLAB and Octave (no Control System Toolbox calls — pure array math + `readtable`). Decide
   whether the `+ctrl` port may use Control System Toolbox objects (`ss`, `tf`, `dlqr`, `kalman`)
   for legibility, or must stay toolbox-free for portable CI. Recommendation: **toolbox-free for
   `+ctrlanalysis`; toolbox-optional for `+ctrl` study demos.**
2. **Runner path.** Passive (a) vs active (b) from [§5](#5-case-study-analysis-integration-plan).
   Recommendation: stay passive until Phase 4.
3. **Tracker tier.** Whether `case_study_tracker.py` should report a MATLAB tier / column. Deferred
   with Phase 4.

---

## 8. Known discrepancies to fix separately

- **Stale case-study count.** `../CLAUDE.md` (§1) says "31 case studies (19 complete, 12 open)", but
  the freshly generated [`../docs/case_study_status.md`](../docs/case_study_status.md) shows **21
  complete / 10 not** (11 C++ + 10 Python complete). The status file is auto-generated and
  authoritative; CLAUDE.md's count is stale. Not a MATLAB issue — flagged here because the port's
  Phase 1 iterates over "Complete" studies and should trust the status file, not CLAUDE.md.

---

## 9. Starter kit — what's already here

Committed alongside this doc to make the plan concrete and testable (Phase 0 analysis layer
**and** the Phase 2 core-port kickoff):

*Analysis layer (Phase 0):*
- [`+ctrlanalysis/load_run.m`](+ctrlanalysis/load_run.m) — reads a `logs/run_*.csv` into a struct of
  named columns (Octave/MATLAB `readtable`), tolerant of the non-uniform schemas.
- [`+ctrlanalysis/compute_metrics.m`](+ctrlanalysis/compute_metrics.m) — faithful port of
  `tools/metrics.py::compute_metrics`, returning the same 6 fields.
- [`../case-study/Boiler Control/matlab/analyze.m`](../case-study/Boiler%20Control/matlab/analyze.m)
  — worked Tier-1 driver: loads a few Boiler runs, computes metrics, cross-checks the recomputed
  cumulative IAE against the logged `IAE_y*` columns, and prints `PASS`/`FAIL`.

*Core port (Phase 2):*
- [`+ctrl/`](+ctrl/) — first native plant/controller/estimator classes, faithful line-by-line ports
  of `lib/`: `TransferFunction`, `StateSpace` (+ `tf2ss`, controllable-canonical), `DiscretePID`
  (filtered derivative, back-calculation anti-windup, DoM), `DiscreteLQR` (doubling-algorithm DARE)
  + `LQRAdapter`, and `KalmanFilter` (Joseph-form covariance).
- [`examples/verify_ex01_tf_pid.m`](examples/verify_ex01_tf_pid.m) — reproduces the compiled
  `ex01_tf_pid.cpp` closed loop; plant matrices/DC gain match exactly and the y/e/u trajectory
  matches the C++ to **max deviation 4.5e-4** (limited by the demo's 3-decimal printed gains).
- [`examples/verify_lqr_kalman.m`](examples/verify_lqr_kalman.m) — asserts the DARE residual ≈ 0,
  cross-checks the LQR gain against MATLAB `dlqr()` (agree to ~1e-14), and shows the Kalman observer
  converging on a noiseless double integrator.

> Porting note surfaced by the verification: `StepResponseTuner::computePIDParams`
> (`lib/ControllerTuner.cpp`) sets `Kb = sqrt(|Ki*Kd|)`, and `ex01` overrides `N`/`uMin`/`uMax`
> but **not** `Kb` — so a faithful reproduction must keep the tuner's `Kb`, not the `PIDParams`
> default `1.0`. Getting this wrong slows the closed-loop rise ~2×.

**Run it:**

```matlab
% from repo root, MATLAB or Octave:
addpath('MATLAB');
run('case-study/Boiler Control/matlab/analyze.m')   % Phase 0: per-run metrics + PASS/FAIL
run('MATLAB/examples/verify_ex01_tf_pid.m')         % Phase 2: +ctrl reproduces ex01_tf_pid.cpp
run('MATLAB/examples/verify_lqr_kalman.m')          % Phase 2: LQR/Kalman consistency + dlqr check
```

or headless: `matlab -batch "addpath('MATLAB'); run('case-study/Boiler Control/matlab/analyze.m')"`
(Octave: `octave --no-gui --eval "addpath('MATLAB'); run('case-study/Boiler Control/matlab/analyze.m')"`).
