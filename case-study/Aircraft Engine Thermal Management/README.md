# Aircraft Engine Thermal Management

## Reference

**Title:** Model predictive control for aircraft engine thermal management under nonlinear heat transfer and time delay
**Authors:** Ke-Lun He, Tian-Yi Zhang, Xiao-Guang Zhang, Qun Chen
**Journal:** Aerospace Science and Technology 169 (2026), Article 111491
**DOI:** https://doi.org/10.1016/j.ast.2025.111491

Source PDF: [`MPC for aircraft engine thermal management under nonlinear heat transfer and time delay.pdf`](MPC%20for%20aircraft%20engine%20thermal%20management%20under%20nonlinear%20heat%20transfer%20and%20time%20delay.pdf). Extracted text: [`extracted_text.txt`](extracted_text.txt) (lossy OCR - cross-checked against the PDF for the equations used below).

---

## System Description

A fuel thermal management system (FTMS) intermediate circulation loop (paper Fig. 1/Fig. 6): a water tank feeds a main loop that splits into two branches, each through an air-water heat exchanger (AHE1, AHE2) that picks up heat from hot bleed air. The two branches remerge and pass through a third exchanger (OHE/FHE) that dumps the collected heat into the fuel before the cooled water returns to the tank. Fluid transport between components introduces a delay; the paper identifies and collapses all the individual pipeline delays into a single "equivalent" delay of 60 s (Section 4.3). The controlled variables are the two AHE air-side outlet temperatures (Eq. 13); the manipulated variables are the two branch flow rates (Eq. 14).

## Model Simplifications

This case study implements a **reduced, control-oriented model**, not a byte-for-byte reproduction of the paper's symbolic derivation. Five deliberate simplifications, in order of significance:

1. **Heat exchangers.** The paper derives a "thermal resistance R" form of the counter-flow exchanger via the heat-current/entransy method (Eq. 7). That equation is a stacked fraction with repeated subscripts in both the raw PDF text extraction and a cleaner manual LaTeX re-extraction - it could not be reliably recovered. This model instead uses the textbook effectiveness-NTU counter-flow heat-exchanger model (Kays & London), the standard closed form for the *same* underlying physical exchanger (a counter-flow exchanger of conductance `K*A` is `K*A` regardless of which equivalent algebraic form expresses it).
2. **Transport delay.** The paper derives per-pipeline delays (`tau1`, `tau2`, `taua`, `taub`, ...) then explicitly collapses them, noting `tau1 ~= tau2` and "this paper temporarily takes 60 s as the equivalent delay time of the system" (Section 4.3). This model applies that single lumped delay directly to the tank temperature seen at each AHE's water inlet - the paper's own end-state simplification (Eq. 25/26).
3. **Actuator.** `u` is used exactly as the paper defines it (Eq. 14): the direct time derivative of the flow rates, integrated by the plant. A physical rate limit (`u_rate_max`) bounds `|u|` as a stand-in for the pump/valve's finite slew rate.
4. **Safety supervisor.** The "forced cooling" / "valve shutdown" rule-based overrides described in the paper (Section 3.3, Fig. 8) are implemented as a supervisor wrapped around every controller in `simulation_runner.py`, not inside the plant - matching the paper's own framing of it as a layer above the predictive controller ("a hybrid control architecture... overrides the MPC when temperatures approach the critical limit").
5. **State choice.** The paper's own state is `(TT, m0, m1)` with `m2 = m0 - m1` a *dependent* quantity. Driving `Thout1` (via `m1`) and `Thout2` (via `m0`) with two independent SISO loops on that state choice means the loop that raises `m1` to cool AHE1 simultaneously *steals* flow from branch 2 (`m2 = m0 - m1` falls), and a hard `m1 <= m0` saturation can drive `m2` to zero and freeze AHE2 entirely - an artefact of the state choice, not a physical limitation, that traps every SISO controller (PID through MPC) tested. This model instead uses the independent branch flows `(m1, m2)` as states (`m0 = m1 + m2` recovered wherever needed): identical heat-exchanger/tank physics, but each loop now has its own actuator with no shared-resource lock. See `aircraft_engine_thermal_management_plant.py` module docstring for the full derivation.

**Reference targets.** The paper states a single 400 K design target for both AHE outlets. With the exact identified parameters (`KA1=713.7 W/K`, `mh1=0.48 kg/s`, `Thin1=723 K`) and the effectiveness-NTU substitution above, AHE1's achievable outlet-temperature range in the well-behaved (monotonic, `m > crossover`) operating branch is approximately `[432, 482] K` - 400 K is below the asymptotic floor (~412 K) reachable at *any* finite flow. `Thout1_ref`/`Thout2_ref` are therefore set to values centred in each exchanger's own achievable range (445 K / 340 K) rather than a literal 400 K, consistent with the paper's own generic per-channel reference framing (Eq. 13 defines `Thout1*`, `Thout2*` as independent design choices, not a single shared constant).

---

## Plant Model

**States** `x = [TT, m1, m2]`:

| Symbol | Description | Unit |
|---|---|---|
| `TT` | Water tank temperature | K |
| `m1` | Branch-1 mass flow rate (through AHE1) | kg/s |
| `m2` | Branch-2 mass flow rate (through AHE2) | kg/s |

`m0 = m1 + m2` (main loop flow, Eq. 6) is recovered algebraically wherever needed (e.g. the OHE hot-side capacity rate).

**Inputs** `u = [u1, u2] = [m1_dot, m2_dot]` [kg/s^2], rate-limited to `+/- u_rate_max`.

**Disturbances** (boundary conditions, vary by scenario): `Thin1(t)`, `Thin2(t)` - AHE1/AHE2 hot (air) inlet temperatures [K]; `mh1(t)`, `mh2(t)` - AHE1/AHE2 hot-side mass flow rates [kg/s].

**Outputs** (controlled variables, paper Eq. 13): `Thout1`, `Thout2` - AHE1/AHE2 air-side outlet temperatures [K].

**Governing equations:**

Counter-flow effectiveness-NTU (per exchanger, hot/cold capacity rates `Gh = m_hot*cp_hot`, `Gc = m_cold*cp_cold`, conductance `KA`):
```
C_min = min(Gh, Gc);  C_max = max(Gh, Gc);  Cr = C_min / C_max;  NTU = KA / C_min
epsilon = (1 - exp(-NTU*(1-Cr))) / (1 - Cr*exp(-NTU*(1-Cr)))      (Cr < 1)
epsilon = NTU / (1 + NTU)                                         (Cr = 1)
Q = epsilon * C_min * (Thin - Tcin)
```

AHE1/AHE2 (hot = air, cold = water branch; `TT_d` = tank temperature delayed by `tau_d`):
```
Thout_i = Thin_i - epsilon_i * (Thin_i - TT_d)
Twater_out_i = TT_d + (epsilon_i * Gh_i / Gc_i) * (Thin_i - TT_d)
```

Mixing (Eq. 4, paper):
```
Tmix = (m1*Twater_out1 + m2*Twater_out2) / m0
```

OHE (hot = merged water, cold = fuel at fixed `mh3`, `Tcin3`):
```
TTi = Tmix - epsilon_3 * (Tmix - Tcin3)        # tank inlet temperature
```

Water tank (Eq. 8, paper - unchanged):
```
TT_dot = m0 * (TTi - TT) / mT
```

Integration: 4th-order Runge-Kutta at `Ts`; the lumped transport delay is applied via a ring buffer of past `TT` samples (`round(tau_d/Ts)` entries).

**Parameter table** (mirrors `config/plant_params.json`):

| Symbol | Value | Source |
|---|---|---|
| `Ts` | 0.5 s | Sample time |
| `mT` | 11.22 kg | Identified tank dynamic capacity (paper Section 4.1) |
| `cp_water`, `cp_air`, `cp_fuel` | 3100, 1050, 2300 J/kg/K | Paper Section 4.2 |
| `KA1`, `KA2`, `KA3` | 713.7, 1276.1, 11409.2 W/K | Identified AHE1/AHE2/OHE conductances (paper Table 2) |
| `tau_d` | 60 s | Identified equivalent transport delay (paper Section 4.3) |
| `u_rate_max` | 0.02 kg/s^2 | Pump/valve slew-rate limit (design choice) |
| `m1_min`/`m1_max`, `m2_min`/`m2_max` | 0.25 / 0.55 kg/s | Kept above each exchanger's Cr=1 crossover (see Implementation Notes) |
| `mh3`, `Tcin3` | 1.30 kg/s, 310 K | Fuel-side flow/inlet temperature (paper Section 5.1) |
| `T_boil_limit`, `T_boil_margin` | 503 K, 10 K | Boiling point at 3 MPa pressurisation (paper Section 5.1); supervisor trigger margin |
| `Thout1_ref`, `Thout2_ref` | 445 K, 340 K | See "Reference targets" above |

---

## Controllers (12)

All controllers share `compute(refs, plant_state, t, outs) -> (u1, u2)`. Sign convention: increasing flow rate (`m1` or `m2`) reduces the corresponding outlet temperature (negative-static-gain process within the monotonic operating branch) - mirrors the documented "Solar Cooker sign convention" precedent (CLAUDE.md).

| # | Name | lib/ Algorithm | Design Notes |
|---|------|--------------------|--------------|
| 1 | OpenLoop | - | Holds flow rates constant (`u=0`); baseline |
| 2 | PID | `ctrl.DiscretePID` | `e = y - ref`, positive gains; m1/m2 are already integrators of u (Eq. 14), so P alone gives zero steady-state error - Ki kept tiny, Kd small (60s delay makes derivative action on recent samples unreliable) |
| 3 | ADRC | `ctrl.DiscreteADRC` | `omega_o=0.04, omega_c=0.015`, `Ts=0.5 -> omega_o*Ts=0.02<0.5` (check); bandwidth kept well below `1/tau_d` so the ESO does not chase the 60s delay |
| 4 | SMC | `ctrl.DiscreteSMC` | Native `compute(y-ref)` convention; boundary layer `phi` sized to the small (~10K) typical operating error |
| 5 | LQR | `ctrl.DiscreteLQR` | Full 3-state regulator toward the open-loop trim point; no integral action - does not re-solve the trim if the setpoint changes or under sustained disturbance |
| 6 | MPC | `ctrl.DiscreteMPC` | The paper's headline method: deviation-form constrained MPC (`Np=15, Nc=5`) tracking `[Thout1_ref, Thout2_ref]` on a numerically-linearized 3-state model; `uMin/uMax`, `duMin/duMax` mirror the paper's Eq. 50 flow-rate constraint |
| 7 | MRAC | `ctrl.MRACController` | `set_reference(r)` + `compute(y)`; **negative** `gamma_r`/`gamma_y` (negative-gain plant - mirrors the documented Solar Cooker MRAC precedent) |
| 8 | L1Adaptive | `ctrl.L1AdaptiveController` | `set_reference(r)` + `compute(y)` |
| 9 | GainScheduled | `ctrl.GainScheduledController` | 2-point schedule on `TT` (gentle below 380K, more aggressive above 480K, anticipating the anti-boiling margin) |
| 10 | SmithPredictor | `ctrl.SmithPredictor` | Delay-compensated PID; internal 2-state model (`m_dev`, `Thout_dev`) built from the linearized static sensitivity and a tank-residence-time lag; `delay_steps = round(tau_d/Ts) = 120` |
| 11 | NeuralPID | `ctrl.NeuralPID` | Standard `compute(r-y)` convention, **negative** `plant_gain` (mirrors the documented Solar Cooker NeuralPID precedent) |
| 12 | ILC | `ctrl.ILC` | Two-phase P-type learning on the m1/Thout1 loop (`N_TRIAL=400`, 200s at Ts=0.5); m2/Thout2 loop uses plain PID |

**Total runs: 12 controllers * 5 scenarios = 60**

---

## Scenarios

| ID | Description | Notes |
|---|---|---|
| `s01_steady_condition1` | Steady flight condition 1/3 boundary values (paper Table 5) | Regulation test at the nominal operating point; `T_sim=1200s` |
| `s02_condition_switch` | Paper Table 5/6 strong time-varying experiment: condition 1 (0-2000s) -> condition 2 (2000-2300s, AHE1 air OFF, AHE2 derated) -> condition 3 = condition 1 (2300-3000s) | Exercises the transport delay and the AHE1-off structural disturbance; `T_sim=3000s` |
| `s03_thermal_ramp` | Smooth climb/acceleration heat-load ramp, complementary to s02's hard switches | `T_sim=1200s` |
| `s04_high_heat_stress` | Above-Table-5 heat load combined with a hot fuel-side inlet (`Tcin3=485K`) | Pushes the tank toward the anti-boiling supervisor margin (493K) since the OHE's high effectiveness otherwise always pulls the tank temperature toward the fuel inlet temperature; `T_sim=1200s` |
| `s05_setpoint_step` | Boundary conditions held at condition 1; air-outlet setpoints step to a new achievable target at t=400s | Reference-tracking test distinct from disturbance rejection; `T_sim=900s` |

---

## CSV Columns

```
time, Thout1_ref, Thout1, Thout2_ref, Thout2, TT, m1, m2, m0,
u1, u2, Thin1, Thin2, override_active, iae_cumulative
```

## Run

```bash
conda run -n soft_robotics -- python "case-study/Aircraft Engine Thermal Management/sim/main.py"
```

This is a **Python-only case study** (Phase 6 of `run.py`). It does not appear in `CMakeLists.txt` or `compile.bat`.

---

## Implementation Notes (tribal knowledge)

- **Crossover/non-monotonic effectiveness:** for a counter-flow exchanger, effectiveness is *minimised*, not maximised, when the two capacity rates are balanced (`Cr=1`, the textbook "balanced counter-flow" worst case). Below that crossover flow, effectiveness *decreases* as flow increases (water-side-limited regime); above it, effectiveness *increases* as flow increases (air-side-limited regime) and `Thout` is then monotonically decreasing in flow. `m1_min`/`m2_min` (0.25 kg/s) are set safely above the highest crossover flow across all five scenarios' `mh1`/`mh2` values (`crossover_m = mh*cp_air/cp_water`, max ~0.20 kg/s) so every controller operates in the well-behaved monotonic branch.
- **Why `(m1, m2)` instead of `(m0, m1)`:** see Model Simplifications #5. Confirmed empirically - the original `(m0, m1)` state choice trapped every SISO controller (and even MPC, despite its linear model correctly capturing the `m0`/`m1` cross-coupling) at `m1 = m0` (`m2 = 0`, AHE2 frozen) when both loops simultaneously saturated their shared flow budget.
- **Safety supervisor is a wrapper, not a controller feature:** `simulation_runner.py` overrides `(u1, u2)` to `+u_rate_max` for *every* controller when `TT > T_boil_limit - T_boil_margin`, exactly mirroring the paper's hybrid-architecture description. `override_active` and cumulative override counts are logged so a controller's "true" unassisted performance can be distinguished from supervisor intervention.
- **`mh3`/`Tcin3` scenario overrides:** `simulation_runner.run_simulation` merges any scenario-level `mh3`/`Tcin3` keys into the plant params for that run only (used by `s04_high_heat_stress` to raise the fuel inlet temperature) - the OHE's high effectiveness (`NTU` is large for any realistic flow given `KA3=11409.2 W/K`) means the tank temperature always tracks close to `Tcin3` regardless of `mh3`, so raising `Tcin3` (not lowering `mh3`) is the physically meaningful way to threaten the boiling margin.
- **Trim/linearization:** `controllers.find_trim()` runs the plant open-loop (`u=0`, condition-1 boundary values) for 900s to reach a self-consistent equilibrium; `controllers.linearize()` takes a central-difference numeric Jacobian of `_derivs`/`outputs` there. LQR and MPC operate in deviation form around this trim (`u_trim = 0` since flow rates are constant at any equilibrium).
- **Module path:** `sim/` sets the binding path 3 levels up from `sim/`: `_ROOT = dirname(dirname(dirname(abspath(__file__))))`.
