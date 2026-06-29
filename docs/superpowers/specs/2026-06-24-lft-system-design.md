# Design: General LFT Uncertainty Representation

**Date:** 2026-06-24
**Status:** Approved, not yet implemented

## Motivation

`docs/algorithm_backlog.md`'s Robust Control section flags this directly: `MuAnalysis.h`'s
`peakMu()` hardcodes the single canonical "`M = sigma_rel * T`" multiplicative-output-uncertainty
loop - the entire output space is implicitly treated as *one* uncertainty block. A plant with
*simultaneous* uncertainty at two different loop locations (e.g. multiplicative input uncertainty
**and** additive output uncertainty together) cannot be represented today; `peakMu()` has no way
to say "this `Delta` block touches these channels, that one touches those."

This is the highest design-risk item in Phase 1: `MuAnalysis::peakMu()` is **not** built on a
general LFT/`GeneralisedPlant` interconnection at all (confirmed: it takes `(StateSpace G,
StateSpace K, ...)`, internally calls `SystemAnalysis::gangOfFour` to get `T`, then scales it) -
so there is no existing sibling implementation to "generalize." The design decision below
resolves this from first principles against `MuAnalysis`'s actual, narrower contract.

## Scope

**Key simplification, resolved during this design pass:** `computeMu()`/`UncertaintyStructure`
(`MuAnalysis.h`) never take `Delta`'s actual numeric realization - only its *structure* (block
sizes/types). Mu-analysis only needs the matrix `M` that `Delta` would multiply against, indexed
into the right block order. **There is therefore no algebraic loop-closing/elimination to
perform** - generalizing `peakMu()` is purely a *channel-gather* problem: given an open-loop
map's full frequency response, select and reorder the rows/columns that each `Delta` block
touches into the canonical block-ordered matrix `computeMu()` expects. This is materially simpler
than the roadmap's "general LFT interconnection builder" framing suggested, and means
`DiscreteHinf::buildClosedLoop` (which *does* perform real algebraic elimination, but only
against one fixed-position feedback block - a controller, not an uncertainty structure) is not
the right reference pattern; it solves a different problem.

**Deviation from the roadmap sketch:** `LFTSystem`'s constructor takes a plain `StateSpace`
(named `M0` below - the *nominal open-loop map* whose rows are an output/`z`-like vector and
whose columns are an input/`w`-like vector), not a `GeneralisedPlant`. `GeneralisedPlant`'s extra
`B2`/`C2`/`D12`/`D21`/`D22` channels exist specifically to support closing a *controller* loop
(`DiscreteHinf::solve()`'s job) - `LFTSystem` has no controller parameter in its constructor at
all (per the roadmap's own sketch) and never uses those channels, so requiring callers to
populate a 9-field struct just to leave 5 of its fields empty is needless ceremony. A caller who
needs *both* an active control loop and a multi-block uncertainty representation closes the
control loop first (e.g. via `DiscreteHinf`, or any fixed `K`, exactly the way `peakMu()`
internally calls `gangOfFour` to close `G`/`K` before scaling `T`), then hands the resulting
closed-loop `StateSpace` to `LFTSystem` as `M0` - the same two-stage pattern `peakMu()` already
uses internally, just made explicit and reusable for arbitrary block structures instead of one
hardcoded one.

- Robust **stability** analysis only (does `mu_Delta(M0) < 1` hold for the given block
  structure) - matching `peakMu()`'s own scope exactly, generalized to multiple/arbitrary block
  placement. Robust **performance** through a separate, non-`Delta` exogenous-disturbance-to-
  performance channel pair (true LFT elimination of an algebraic loop) is a structurally
  different, harder problem and is out of scope (see below).
- Block placement is **row/column sub-ranges of `M0`'s own frequency-response matrix** - blocks
  may be non-contiguous and need not span `M0`'s full dimension (channels not claimed by any
  block are simply not part of the gathered matrix handed to `computeMu`).

## Components

### `lib/LFTSystem.h` / `.cpp` - standalone, no shared base

```cpp
struct LFTChannelMap {
    // Per-block (same order/length as UncertaintyStructure::blocks):
    std::vector<int> rowStart;  // M0's row offset where block i's output-side range starts
                                 // (range is [rowStart[i], rowStart[i] + blocks[i].r_out)).
    std::vector<int> colStart;  // M0's column offset where block i's input-side range starts
                                 // (range is [colStart[i], colStart[i] + blocks[i].r_in)).
};

class LFTSystem {
public:
    // M0: the nominal open-loop map (rows = z-like output channels Delta blocks read from,
    //     cols = w-like input channels Delta blocks write into). Any control/measurement
    //     port must already be closed by the caller - see Scope.
    LFTSystem(const StateSpace &M0, const UncertaintyStructure &struc,
              const LFTChannelMap &map);

    // Gathered M(jw) at each omega, sized struc.totalOutputs() x struc.totalInputs(),
    // block-ordered to match `struc` - directly feedable to computeMu().
    std::vector<Eigen::MatrixXcd> closedLoopFreqResponse(const std::vector<double> &omegas) const;

    // Convenience: log-spaced frequency grid + computeMu(), mirroring peakMu()'s own signature
    // minus sigma_rel (relative scaling, if needed per-block, is baked into M0 or struc by the
    // caller before construction - see "sigma_rel" note below).
    PeakMuResult peakMu(int freq_points = 200, double omega_min = 1e-2) const;
};
```

**`closedLoopFreqResponse()` implementation:** evaluate `M0`'s full frequency response via the
existing `freqResponseGrid(M0, omegas)` (`GapMetric.h:43`, confirmed public, reused as-is - zero
duplication), then for each frequency point, gather the matrix at row indices
`concat([rowStart[i], rowStart[i]+r_out_i) for each block i]` and column indices
`concat([colStart[i], colStart[i]+r_in_i) for each block i]`, in block order. This single gather
(via Eigen fancy-indexing or an explicit double loop) produces exactly the matrix
`computeMu()`/`peakMu()`'s degenerate single-block path already builds by construction (the
entire `T` matrix, no gather needed since one block spans everything) - generalized to arbitrary,
possibly-disjoint, possibly-partial block placement.

**Validation at construction:** throws `std::invalid_argument` if `map.rowStart`/`colStart` don't
have one entry per `struc.blocks`, if any block's row/col range falls outside `M0`'s output/input
dimensions, or if any two blocks' ranges overlap (a channel claimed by two `Delta` blocks
simultaneously is ill-posed) - this is the roadmap's test plan item 3 ("mis-sized channel map
throws").

**`sigma_rel` note:** `peakMu(G, K, struc, sigma_rel, ...)`'s relative-uncertainty scaling is a
uniform multiplier applied to the *entire* `M`. `LFTSystem` has no equivalent parameter - for the
degenerate single-block reproduction test (below), the test constructs `M0` as `sigma_rel * T`
directly (scale `T`'s `C`/`D` by `sigma_rel` before passing it in), which is mathematically
identical to what `peakMu()` does internally. Per-block-heterogeneous relative scaling (different
blocks scaled differently) isn't addressed by the roadmap's `peakMu()` either and is left to the
caller (scale the relevant rows of `M0` per block before construction) rather than added as a new
per-block parameter that nothing in this phase needs.

## Explicitly out of scope (this phase)

- **Robust performance via a non-`Delta` exogenous/performance channel pair** - would require
  actually eliminating an algebraic loop (true `F_u(M0, Delta)` LFT closure with `Delta`'s
  *symbolic* structure, not just gathering `M0`'s existing frequency response) to get a residual
  disturbance-to-performance map under worst-case `Delta`. Structurally a different, harder
  feature; not needed by `computeMu()`'s structure-only contract and not requested by the
  roadmap's own test plan (all three items are robust-*stability* checks).
- **`RealScalar` block support** - already out of scope in `MuAnalysis.h` itself (G-scaling not
  implemented); `LFTSystem` inherits this limitation unchanged, it doesn't attempt to lift it.
- **Per-block heterogeneous relative scaling parameter** - see `sigma_rel` note above; caller
  pre-scales `M0`/block rows directly instead.
- **Closing a controller loop internally** - `LFTSystem` takes an already-closed `M0`; it does
  not accept a `GeneralisedPlant` + controller pair and call `buildClosedLoop`-style elimination
  itself. Callers needing that compose it externally before constructing `LFTSystem` (see Scope).

## Implementation checklist

(Lighter non-`IController` utility-class checklist, same shape as `MuAnalysis`'s free
functions/structs.)

1. `lib/LFTSystem.h`/`.cpp` + `CTRL_REGISTER_FEATURE(lft_system)`
2. `lib/CMakeLists.txt` - add `LFTSystem.cpp` to `CTRL_CORE_SOURCES`
3. `lib/ControllerToolbox.h` - add `#include "LFTSystem.h"` near `MuAnalysis.h`
4. `bindings/analysis_bindings.cpp` - bind alongside `MuAnalysis`'s `UncertaintyStructure`/
   `UncertaintyBlock`/`computeMu`/`peakMu` (confirmed binding home, `analysis_bindings.cpp:
   753-818`)
5. `bindings/smoke_test.py` - construct with a 2-block degenerate map, call
   `closed_loop_freq_response()`/`peak_mu()`, confirm finite output
6. `examples/ex96_lft_system.cpp` - the two-simultaneous-blocks scenario from the Scope section
   (multiplicative input + additive output uncertainty on the same plant, via two disjoint
   `LFTChannelMap` ranges) + `examples/python/ex113_lft_system.py`
7. `tests/test_catch2_advanced.cpp` - tests under `[lft_system]`
8. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` - add `ex96_lft_system`

## Testing plan

**`[lft_system]`**
1. Degenerate single-block case - construct `M0 = sigma_rel * T` (scaling `T`'s `C`/`D`
   directly) for the same `G`/`K`/`struc` used by an existing `peakMu(G, K, struc, sigma_rel)`
   call; `LFTSystem(M0, struc, map).peakMu()` with `map` spanning `M0`'s full range reproduces
   the existing `peakMu()` result exactly (same `peak.upper`, same `peak_omega_rad_s`).
2. Two simultaneous blocks at disjoint row/col ranges of a small (e.g. 2x2) synthetic `M0` -
   `closedLoopFreqResponse()`'s gathered matrix at each frequency matches a hand-derived
   block-selection of `M0`'s known frequency response exactly (direct numerical comparison, not
   just a sanity bound).
3. Mis-sized/overlapping/out-of-range channel map - throws `std::invalid_argument` at
   construction, not at first use.
4. Channels left unclaimed by any block (`M0` larger than the union of all block ranges) - the
   gathered matrix still has exactly `struc.totalOutputs() x struc.totalInputs()` size, ignoring
   the unclaimed channels, confirming partial coverage doesn't error or silently include
   unrelated channels.
