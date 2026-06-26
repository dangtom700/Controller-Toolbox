# Design: Subspace ID Method Variants (Phase 3, SI3 — MOESP / N4SID / CVA)

**Date:** 2026-06-25
**Roadmap source:** `docs/ALGORITHM_ROADMAP_PHASE3.md` SI3 section; `docs/algorithm_backlog.md`
System Identification table (`MOESP / CVA (subspace ID variants)`).
**Status:** Approved design, ready for implementation plan.

## Goal

Give `lib/SubspaceID.h` a real choice of subspace-identification weighting, instead of the single
hardcoded pipeline `n4sid()` exposes today, by adding a `SubspaceMethod` enum (`MOESP`, `N4SID`,
`CVA`) and a new `subspaceID()` entry point that reuses the existing Hankel/LQ/SVD machinery and
only branches at the weighting-before-SVD step.

## What's actually wrong with the roadmap's original framing

The roadmap sketch invented a free function `subspaceID(..., method=N4SID, ...)` and assumed
`n4sid()` was the deterministic-stochastic-combined N4SID algorithm being "extended" with MOESP
and CVA as additions. Reading the real code shows the opposite: `n4sid()`'s own docstring says it
implements **MOESP** (Verhaegen & Dewilde 1992) — an *unweighted* oblique projection — not weighted
N4SID at all (`lib/SubspaceID.h:16-62`). So "MOESP mode" is not new work; it is exactly today's
existing `n4sid()`, mislabeled. The real new work is true (weighted) N4SID and CVA, which the
unifying-theorem literature (Van Overschee & De Moor) frames as differing from MOESP **only** by
weighting matrices applied to the oblique-projection matrix (`L32` in this codebase's notation)
before its SVD — everything downstream (extracting `A`/`C` from shift-invariance, the `B`/`D`
regression, the stochastic realization) is identical regardless of method.

## Why this needed a numpy prototype before writing it into a plan

A first prototype attempt used the textbook-clean choice for CVA's "future" weighting —
`Sigma_{Yf|Uf}^{-1/2}` computed from the existing LQ decomposition's `L32`/`L33` blocks — and it
was numerically broken: `Sigma_{Yf|Uf}` is an `(i*p) x (i*p)` matrix whose **true rank is only the
system order** (`n_order`, typically << `i*p`), so inverting it (even with ridge regularization)
amplified noise directions by factors of 1000x+ in a near-noiseless sanity check that should have
recovered the exact known system. This is not a contrived edge case — `i_horizon > n_order` is the
normal, recommended operating regime (the docstring itself recommends `i >= 2*n_order/p`), so this
failure mode would hit real use, not just adversarial inputs.

The fix that survived testing: instead of whitening the full multivariate future-block covariance,
weight **per output channel** using a noise-scale estimate derived from `L33` (the part of each
future output unexplained by either the input or the past data — i.e., the genuinely
unpredictable/noise component). `L33`'s rows are indexed 1:1 by `Yf`'s own row layout (`i` repeated
blocks of `p` output channels), so per-channel noise variance is `mean over the i repetitions of
each channel's squared L33 row norm` — `p` scalars, trivially invertible, no rank-deficiency risk.
This directly implements the plain-language motivation the roadmap itself gives for CVA ("preferred
when output channels have very different noise scales") without the full canonical-correlation
machinery's conditioning problem. Verified against a synthetic 2-output system with deliberately
mismatched noise levels (`std = 0.005` vs `0.3`): CVA beats both MOESP and N4SID specifically on the
high-noise channel (mean abs frequency-response error `0.157` vs `0.179` vs `0.313`), while tracking
N4SID closely on equal-noise data and in a near-noiseless sanity check (no blow-ups, complex
conjugate eigenvalue pair preserved, close to the true system). This is a narrower, regularized
claim than full Larimore/Van Overschee-De Moor CVA, and is documented as such below — not a
bit-for-bit reproduction of the original paper's weighting.

## Algorithm

All three methods share Steps 1-2 (build the `2i`-block Hankel matrices, partition past/future,
LQ-decompose `Z = [Uf; Wp; Yf]`) and Steps 4-6 (extract `A`/`C` from `Gamma`'s shift-invariance,
regress `B`/`D`, compute the stochastic realization) **completely unchanged** from today's
`n4sid()`. Only Step 3 (weighting before the SVD that produces `Gamma`) branches by method:

- **MOESP:** `SVD(L32)` directly — bit-identical to today's `n4sid()`.
- **N4SID:** `SVD(L32 @ L22^-1)` — right-weight only. `L22` (the `(Wp,Wp)` block of the LQ
  factor `L`) is already a valid Cholesky-style square root of `Sigma_{Wp|Uf} = L22 @ L22^T / s`
  (Wp's covariance with the input's contribution regressed out) — no extra computation needed
  beyond a triangular solve, and no rank-deficiency risk since `Wp` (raw past data) is not expected
  to be low-rank the way a future-output projection is.
- **CVA:** `SVD(diag(w) @ L32 @ L22^-1)`, where `w` is a length-`i*p` vector built by tiling a
  length-`p` per-channel weight `i` times: `w_j = 1 / sqrt(mean_{r=0..i-1}(L33[r*p+j, :])^2-row-norm)`
  for channel `j = 0..p-1`. After truncating to `n_order` singular vectors, undo the (purely
  diagonal, trivially invertible) left weighting before forming `Gamma`, by multiplying back by
  `1/w` element-wise — unlike a generic matrix weight, this never requires inverting an
  ill-conditioned object.
- `suggestOrder()` is reused unchanged for all three methods' `singularValues` (no method-specific
  branch). Its elbow heuristic is somewhat less sharp for CVA's reweighted spectrum in some
  scenarios (verified in the prototype) — this is an existing, documented heuristic limitation
  (`suggestOrder`'s own docstring already frames it as a heuristic with a secondary threshold
  guard), not a new defect introduced by this design.

## API

`lib/SubspaceID.h` / `lib/SubspaceID.cpp`. `n4sid()` keeps its exact current name, signature, and
behavior — it is called from ~10 other locations (`lib/LPVSystemID.cpp`, the Boiler Control case
study, `examples/ex31_subspace_id.cpp`, `examples/ex20_system_identification_data.cpp`, tests,
3 binding files) and none of them need to change. Internally, `n4sid()` becomes a one-line
delegate to the new shared pipeline at `SubspaceMethod::MOESP` — a behavior-preserving refactor
that also deletes ~150 lines of now-duplicate Hankel/LQ/extraction code.

```cpp
namespace ctrl {

enum class SubspaceMethod { MOESP, N4SID, CVA };

// Existing signature, existing behavior, now implemented as subspaceID(..., MOESP, svd_tol).
SubspaceIDResult n4sid(const Eigen::MatrixXd &Y, const Eigen::MatrixXd &U,
                       int n_order, int i_horizon, double Ts, double svd_tol = -1.0);

/**
 * @brief Batch subspace identification with a choice of weighting (MOESP / N4SID / CVA).
 *
 * MOESP: unweighted oblique projection (identical to n4sid()).
 * N4SID:  right-weights the past block by its Uf-conditioned covariance (Cholesky-clean).
 * CVA:    additionally left-weights each output channel by an estimated noise scale,
 *         derived from the LQ factor's residual block -- helps when output channels have
 *         very different noise levels. See the design doc for the numerical reasoning;
 *         this is a regularized, per-channel-scale variant, not full canonical-variate
 *         (cross-covariance) whitening, which proved numerically ill-conditioned in
 *         prototyping (the matrix it would require inverting is rank-deficient by
 *         construction whenever i_horizon > n_order, the normal operating regime).
 *
 * kalmanGain/innovCov (SubspaceIDResult's stochastic-realization diagnostics) are computed
 * for all three methods, including MOESP -- even though textbook MOESP has no stochastic
 * step, this machinery is already built from the same residuals regardless of method, and
 * n4sid() (today's mislabeled MOESP) already always populates it; leaving it empty only for
 * "true" MOESP would be a surprising, valueless inconsistency.
 *
 * @param method  Weighting variant. Defaults to MOESP (n4sid()'s existing behavior).
 */
SubspaceIDResult subspaceID(const Eigen::MatrixXd &Y, const Eigen::MatrixXd &U,
                             int n_order, int i_horizon, double Ts,
                             SubspaceMethod method = SubspaceMethod::MOESP,
                             double svd_tol = -1.0);

int suggestOrder(const Eigen::VectorXd &sv, double threshold = 0.01, int maxOrder = -1); // unchanged

} // namespace ctrl
```

Internal refactor (private to `SubspaceID.cpp`, not part of the public API): the existing
`buildHankel()` helper is reused as-is; a new private helper performs the shared Steps 1-2
(Hankel build + LQ decomposition, returning `L` and the `r_uf`/`r_wp`/`r_yf` block sizes), and
another shared helper performs Steps 4-6 given a `Gamma`. `subspaceID()`'s body is: call the
Steps-1-2 helper, branch on `method` to build the weighted matrix and its SVD, form `Gamma`
(undoing any left weighting), call the Steps-4-6 helper. `n4sid()` becomes
`return subspaceID(Y, U, n_order, i_horizon, Ts, SubspaceMethod::MOESP, svd_tol);`.

## Edge cases / numerical safety

- All of `n4sid()`'s existing checks apply unchanged (mismatched `Y`/`U` column counts,
  insufficient data relative to `i_horizon`/`n_order`, `i_horizon` too small, `n_order < 1`).
- N4SID/CVA: if `L22` is near-singular (degenerate/non-persistent excitation in the input), the
  triangular solve `L32 @ L22^-1` is checked via `L22`'s smallest diagonal magnitude; below a
  floor (`1e-10` relative to the largest diagonal entry), return `success=false` with a message
  ("input excitation too weak for weighted subspace ID; check persistence of excitation") rather
  than silently dividing by a near-zero pivot.
- CVA: per-channel noise variance is floored at `1e-12 * max_channel_variance` before the
  `1/sqrt(.)` weight (prevents a division blow-up for a channel with literally zero estimated
  noise, e.g. a synthetic noiseless test channel), mirroring the floor already used in the
  prototype.
- This is offline, batch identification code (not a `compute()`/`step()` hot path) — the RT
  zero-allocation rules (`CLAUDE.md` section 7) do not apply, the same exemption `n4sid()` already
  has.

## Wiring

This is an **extension to an existing class** (`docs/ALGORITHM_ROADMAP_PHASE3.md`'s own
Implementation Checklist already calls this out for SI3) — only the files below need updating,
not the full 8-step new-class checklist:

1. `lib/SubspaceID.h` / `lib/SubspaceID.cpp` — `SubspaceMethod` enum, `subspaceID()`, internal
   refactor, `n4sid()` becomes a delegating one-liner.
2. `bindings/advanced_bindings.cpp` (where `n4sid()`/`suggestOrder()` are already bound, under
   `CTRL_HAS_SUBSPACE`) — bind the `SubspaceMethod` enum and `subspaceID()`.
3. `bindings/smoke_test.py` — extend the existing `n4sid` smoke-test section with a `subspaceID`
   call across all 3 methods.
4. `tests/test_catch2_advanced.cpp` — new Catch2 tests tagged `[subspace_id_variants]`.
5. `examples/ex113_subspace_id_variants.cpp` + `examples/python/ex130_subspace_id_variants.py`
   (next free numbers as of this writing, after FD2's reserved `ex112`/`ex129`), plus
   `examples/CMakeLists.txt`, `compile.bat`, `compile.sh` updates.
6. `docs/algorithm_backlog.md` / `docs/ALGORITHM_ROADMAP_PHASE3.md` — mark SI3 done once shipped.

No changes to `lib/CMakeLists.txt` or `lib/ControllerToolbox.h` (no new source file — `SubspaceID.h`/
`.cpp` already exist and are already wired in).

## Test plan (`[subspace_id_variants]`)

1. Known 2-output state-space system, equal noise on both channels — all 3 methods recover
   realizations whose `A` eigenvalues match the true system within tolerance (similarity-invariant
   comparison, per `SubspaceID.h`'s own documented invariance contract); `subspaceID(..., MOESP)`
   matches `n4sid()`'s output bit-for-bit (regression, confirms the refactor is behavior-preserving).
2. Mismatched output-channel noise scales (`std` differing by 60x between channels) — CVA's
   recovered model has lower frequency-response error than both MOESP and N4SID specifically on
   the high-noise channel (verified in the prototype: `0.157` vs `0.179` vs `0.313`).
3. `suggestOrder()` runs unchanged (same function, no method-specific branch) across all 3
   methods' `singularValues` outputs and returns a value `>= 1` for each.
4. Degenerate excitation (near-constant input, near-singular `L22`) — N4SID/CVA return
   `success=false` with a descriptive message rather than a NaN/garbage model; MOESP (which
   doesn't use `L22`) is unaffected by this specific degeneracy and is checked separately for its
   own existing failure modes (already covered by `n4sid()`'s current tests).

## Out of scope

- Full Larimore/Van Overschee-De Moor canonical-variate (cross-covariance) whitening — proved
  numerically ill-conditioned in prototyping (see "Why this needed a numpy prototype" above); the
  per-channel noise-scale weighting shipped instead is a deliberate, documented simplification.
- Changing `n4sid()`'s name or signature — stays fully backward compatible for all existing callers.
- A `SubspaceMethod`-aware variant of `kalmanGain`/`innovCov` semantics (e.g. leaving them empty
  for "true" MOESP) — computed identically for all 3 methods, per the API section's reasoning.
- Re-deriving `i_horizon`/`n_order` selection guidance per method — `suggestOrder()`'s existing
  heuristic is reused as-is; no new order-selection logic.
