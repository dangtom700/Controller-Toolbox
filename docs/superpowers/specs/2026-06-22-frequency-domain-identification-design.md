# Design: Frequency-Domain System Identification (Levy's Method)

**Date:** 2026-06-22
**Status:** Implemented and verified - see
[2026-06-22-frequency-domain-identification-handoff.md](2026-06-22-frequency-domain-identification-handoff.md)

## Motivation

`docs/algorithm_backlog.md`'s System Identification category lists frequency-domain
identification as open: the toolbox has time-domain identification (`RecursiveLeastSquares`,
`SubspaceID`, `SINDy`, `GreyBoxEstimator`) but nothing that fits a model directly to
frequency-response data. This is also the natural follow-up to Phase 4 Iteration 1
(classical frequency-domain analysis plots, `2026-06-22-frequency-domain-analysis-plots-design.md`):
that work built tooling *around* `SystemAnalysis::getFrequencyResponse`/`getSingularValues`;
this phase fits a model the other direction - from frequency-response samples back to a
parametric `TransferFunction`.

Per the "keep iterations small and self-contained" decision for Phase 4, this scopes only
**Levy's method** - the classical single-shot linear-least-squares fit (the same algorithm
behind MATLAB's `invfreqz`). Two related, more advanced methods (Sanathanan-Koerner iteration,
Vector Fitting) are deliberately deferred - see "Explicitly out of scope" - rather than shipped
as partial/stubbed code, per this repo's "no half-finished implementations" rule.

## Scope

- **Input:** frequency-response data the caller already has - `(freqs, response)` arrays of
  matching length, e.g. produced by `SystemAnalysis::getFrequencyResponse` on a black-box
  system, or from an external sine-sweep/FFT measurement. No FFT/spectral-estimation
  preprocessing is in scope (see below).
- **Output:** a SISO discrete-time `TransferFunction` (reuses the existing class directly -
  num/den coefficient arrays, already bound to Python, already convertible to `StateSpace` via
  `tf2ss()` if the caller needs state-space form).
- **Algorithm:** Levy's method only.

## Components

**New class `lib/FreqDomainIdentifier.h` / `.cpp`** - a static-method utility class (mirrors
`SystemAnalysis`'s style: a one-shot deterministic computation needs no persistent state,
unlike `GreyBoxEstimator`'s iterative nonlinear solve):

```cpp
struct FreqDomainFitResult {
    TransferFunction tf;   // fitted discrete-time model
    double rmse;           // RMS of |H_data - H_fit| across the sample frequencies
    bool full_rank;        // false if the linear system was rank-deficient (still solved,
                            // via least-norm solution, but the fit may be unreliable)
};

class FreqDomainIdentifier {
public:
    // Fit num_order/den_order TransferFunction coefficients to (freqs, response) via
    // Levy's method (linear least squares). den's constant term is fixed to 1, matching
    // this codebase's existing TransferFunction convention (e.g. {1.0, -1.81, 0.819}).
    static FreqDomainFitResult fitLevy(const std::vector<double> &freqs,
                                        const std::vector<std::complex<double>> &response,
                                        int num_order, int den_order, double Ts);
};
```

**Algorithm:** target model `H(z^-1) = N(z^-1)/D(z^-1)`, `D`'s constant term fixed to 1.
Minimizing `||H_data - N/D||^2` directly is nonlinear; Levy linearizes by minimizing
`N(z_k^-1) - H_data,k * D(z_k^-1)` instead, which is linear in the unknown coefficients.
Stacking real and imaginary parts of this residual across all sample frequencies gives a
real linear least-squares system `Phi*x = y` (size `2*M` rows by `num_order+1+den_order`
columns), solved once via `Eigen::ColPivHouseholderQR` - the same solver pattern already used
in `SystemAnalysis::solveDiscreteLyapunov`. `rmse` is computed by re-evaluating the fitted
`N(z^-1)/D(z^-1)` at each input frequency and comparing to the original `response` data.

**Error handling:** throws `std::invalid_argument` if `freqs.size() != response.size()`, or if
`freqs.size() < num_order + 1 + den_order` (system underdetermined) - consistent with how
`SystemAnalysis::feedback`/`calculateDiskMargin` already throw on invalid input. `full_rank`
flags numerically rank-deficient-but-technically-solvable cases without throwing.

## Explicitly out of scope (this phase)

- **Generalizing SK iteration to full complex-response fitting** - `lib/VectorFitting.h`
  already implements Sanathanan-Koerner iteration, but only for fitting a *real positive
  magnitude* profile to a *real-pole* filter (built for `DiscreteHinf::solveMuSyn`'s D-scaling
  fits, not general identification). Extending SK to fit the full complex frequency response
  (both numerator and denominator, recovering phase as well as magnitude) for arbitrary
  `TransferFunction` identification is still open and tracked as a new backlog item
  (`docs/algorithm_backlog.md`, new "Frequency-Domain Identification Extensions" section) -
  it reuses `fitLevy`'s linear-system-build step with iteration/reweighting added around it,
  not a from-scratch effort.
- **Complex-conjugate-pole Vector Fitting** - `VectorFitting::fitMagnitude` only places real
  poles, so it can't represent resonant/lightly-damped systems with complex-conjugate pole
  pairs. A general Vector Fitting with complex pole-pair bookkeeping and relocation is
  genuinely unstarted and a materially bigger lift than the SK generalization above; tracked
  in the same new backlog section.
- **FFT/spectral-estimation preprocessing** from raw time-domain I/O (Welch's method, windowing,
  averaging) - a separate concern from the curve-fit itself; this phase assumes the caller
  already has frequency-response samples.
- **MIMO fitting** - Levy's method is classically SISO (matrix-fraction descriptions for MIMO
  are a materially larger problem); out of scope here as it was for `getFrequencyResponse`
  before Iteration 1 added the separate MIMO-capable `getSingularValues`.
- **StateSpace-direct fitting** - `TransferFunction` is the natural fit target for a rational
  frequency-domain fit; `tf2ss()` already exists for callers who need a state-space form.

## Implementation checklist

New `lib/` class (not an `IController`, so the full controller checklist doesn't apply - this
follows the same shape as `GreyBoxEstimator`/`SubspaceID`/`SOPDTIdentifier`):

1. `lib/FreqDomainIdentifier.h` / `.cpp` - implement `fitLevy`
2. `lib/CMakeLists.txt` - add `FreqDomainIdentifier.cpp` to `CTRL_CORE_SOURCES`
3. `lib/ControllerToolbox.h` - add `#include "FreqDomainIdentifier.h"`
4. `lib/Features.h` - add a feature-flag entry
5. `bindings/estimation_bindings.cpp` - bind `FreqDomainFitResult` + `FreqDomainIdentifier.fit_levy`
6. `bindings/smoke_test.py` - assert it's callable and recovers a known model
7. `tests/test_catch2_advanced.cpp` - 3 tests under `[freq_domain_id]` (see Testing plan)
8. `examples/python/ex107_frequency_domain_identification.py` - fit the README's minimal
   plant from its own (noiseless) frequency response and confirm recovery

## Testing plan (`[freq_domain_id]`)

1. Known 1st-order system (`num={0.2}`, `den={1,-0.8}`, `Ts=0.1` - the same minimal example
   already used in `bindings/smoke_test.py`), exact noiseless frequency response at correctly
   assumed orders -> recovered coefficients match the true ones to tight tolerance (the linear
   system is exactly solvable here, no fitting error to absorb).
2. Known 2nd-order system (the README's own plant: `num={0.0048,0.0047}`,
   `den={1,-1.81,0.819}`, `Ts=0.01`) -> same exact-recovery check at higher order.
3. Fewer frequency samples than unknown coefficients -> throws `std::invalid_argument`.
