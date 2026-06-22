# Design: Classical Frequency-Domain Analysis & Plotting

**Date:** 2026-06-22
**Status:** Approved, not yet implemented

## Motivation

A broad feature wishlist (LMI solvers, backstepping, NARMAX, deep RL, code generation, ~50
items across 11 categories) was proposed for the toolbox. Cross-checking against `lib/`
(80 headers) showed several items already exist under different names (`MRACController`,
`FeedbackLinearisation`, `SINDy`, `RepetitiveController`, `GaussianProcess`/`GPResidualModel`,
`WorstCaseSearch`/`LyapunovRobustness`, `SystemAnalysis::calculateDiskMargin`). The wishlist
was far too large for one implementation phase — the rest is tracked in
[docs/algorithm_backlog.md](../../algorithm_backlog.md) for future phases.

This spec scopes the first phase: **classical frequency-domain analysis plots** (Bode,
Nyquist, Nichols, root locus, singular-value), the user's own "High priority — essential for
adoption" item, and the one category where the underlying math is *already* implemented and
bound to Python, so this phase is almost entirely a thin visualization layer rather than new
control-theory code.

## Scope

| Plot | Data source | New code? |
|---|---|---|
| Bode (mag/phase vs ω) | `SystemAnalysis.get_frequency_response` (existing, SISO) | None — pure Python |
| Nyquist (Re vs Im) | same | None |
| Nichols (phase vs mag dB) | same | None |
| Root locus (closed-loop poles vs swept gain) | `feedback(k*C, k*D)` + `get_poles`, looped over gain `k` in Python — scaling `C`/`D` scales loop gain; `A`/`B`/`C`/`D` are `def_readwrite` already | None |
| Singular-value plot (MIMO, σ vs ω) | new `SystemAnalysis::getSingularValues(sys, freqs)` | ~60-80 lines C++ — `get_frequency_response` is explicitly SISO-only |

Backend: matplotlib, Agg backend (matches `tools/mu_plots.py`, `fault_plots.py`, `mc_plots.py`
exactly — no new dependency, no second plotting convention alongside `generate_report.py`'s
Plotly).

## Components

1. **`lib/SystemAnalysis.h` / `.cpp`** — add
   ```cpp
   static std::vector<Eigen::VectorXd> getSingularValues(const StateSpace& sys,
                                                          const std::vector<double>& freqs);
   ```
   Evaluates the full `G(e^{jωTs}) = C(zI-A)^-1 B + D` matrix (not just the SISO scalar
   `get_frequency_response` returns) at each frequency, then `Eigen::JacobiSVD` for singular
   values, descending order. Treated as an **extension to an existing class** per the
   `CONTRIBUTING.md` / Phase-2-roadmap precedent (`docs/ALGORITHM_ROADMAP_PHASE2.md`'s E3/E4/D1
   items): no new feature-flag entry, no new example-only-for-this-method — just the method,
   its binding, and tests.

2. **`tools/freq_domain_plots.py`** — new module, *not* a CSV-driven CLI like `mu_plots.py`
   (those visualize a sweep-result file written by a paired analysis script; here the input is
   a live `StateSpace`, computed on the fly — there's no natural file to point a CLI at).
   Five functions, each takes a `StateSpace` (+ a frequency or gain array) and returns a
   matplotlib `Figure` for the caller to `savefig`/`show`:
   - `bode(sys, freqs) -> Figure`
   - `nyquist(sys, freqs) -> Figure`
   - `nichols(sys, freqs) -> Figure`
   - `root_locus(open_loop, gains) -> Figure`
   - `sigma_plot(sys, freqs) -> Figure`

3. **`examples/python/ex106_frequency_domain_plots.py`** — demonstrates all five against the
   README's minimal-example plant (`TransferFunction({0.0048, 0.0047}, {1.0, -1.81, 0.819})`)
   closed with `DiscretePID`. This is the functional verification: `run.py` Phase 5 auto-runs
   every `examples/python/exNN_*.py`, so this script must run clean and exit 0 — same bar as
   every other example in the repo, no separate test harness needed for the plotting module.

4. **Tests:**
   - 2 new Catch2 cases under the existing `[system_analysis_ext]` tag in
     `tests/test_catch2_advanced.cpp`: (a) SISO system — `getSingularValues` result matches
     `|get_frequency_response|` at the same frequencies; (b) known diagonal 2x2 MIMO system —
     singular values equal the sorted absolute values of the two independent SISO channels.
   - 1 new assertion in `bindings/smoke_test.py` calling `SystemAnalysis.get_singular_values`.
   - 1 new `.def_static` line in `bindings/analysis_bindings.cpp`.

## Explicitly out of scope (this phase)

- Nichols-chart M-circle/N-circle gridlines (cosmetic; can be layered on later without
  changing the function signatures).
- A CLI wrapper for `freq_domain_plots.py` (no natural file-based input).
- MIMO Bode (the per-channel matrix Bode is a different, larger visualization problem; SV
  plot is the standard MIMO substitute).
- Embedding these into `generate_report.py`'s Plotly pipeline (different backend, separate
  concern — these are standalone diagnostic plots, not part of the unified HTML report).
- Every other category from the original wishlist (robust-control LMI/H2/structured-Hinf,
  nonlinear backstepping/passivity, system-ID expansion, optimal-control DP/dual-control,
  adaptive pole placement, estimation variants, ML/RL, deployment/codegen, multi-objective
  optimization, additional controller types) — tracked in
  [docs/algorithm_backlog.md](../../algorithm_backlog.md), not this phase.

## Implementation checklist

Per `CONTRIBUTING.md`'s "extension to an existing class" rule (full 8-step new-controller
checklist not required since `SystemAnalysis` isn't an `IController` and already exists):

1. `lib/SystemAnalysis.h`/`.cpp` — add `getSingularValues`
2. `bindings/analysis_bindings.cpp` — bind it
3. `bindings/smoke_test.py` — assert it's callable
4. `tests/test_catch2_advanced.cpp` — 2 tests under `[system_analysis_ext]`
5. `tools/freq_domain_plots.py` — new module, 5 functions
6. `examples/python/ex106_frequency_domain_plots.py` — new example exercising all 5
7. `run.py` — no changes needed; Phase 5 auto-discovers the new example
