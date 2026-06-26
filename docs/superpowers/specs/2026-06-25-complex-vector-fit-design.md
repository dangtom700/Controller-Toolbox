# Design: `ComplexVectorFit` (Phase 3, FD2 — Complex-Conjugate-Pole Vector Fitting)

**Date:** 2026-06-25
**Roadmap source:** `docs/ALGORITHM_ROADMAP_PHASE3.md` FD2 section; `docs/algorithm_backlog.md`
Frequency-Domain Identification Extensions table (`Complex-conjugate-pole Vector Fitting`).
**Status:** Approved design, ready for implementation plan.

## Goal

Fit a stable discrete-time SISO model to complex-valued (magnitude **and** phase) frequency
response data using complex-conjugate pole pairs. `VectorFitting::fitMagnitude` (`lib/VectorFitting.h`)
only places real poles and only fits a real magnitude profile, so it cannot represent
resonant/lightly-damped systems (flexible structures, multiple resonant peaks). This closes that
gap with a real Vector Fitting pole-relocation algorithm (Gustavsen & Semlyen 1999), not a
power-basis polynomial fit.

## Why this is not redundant with `SKFit` (FD1, already shipped)

`SKFit::fitSK` (`lib/SKFit.h`) already fits a general complex response to an arbitrary
`num_order`/`den_order` `TransferFunction`, and its denominator's roots can already be complex
pairs — there is no real-pole restriction in `SKFit` at all. The difference is **conditioning**,
not capability: `SKFit`/Levy fit in the monomial power basis (`z^j`), which becomes ill-conditioned
at higher order or near resonance (large coefficient dynamic range). Vector Fitting's actual
contribution is reformulating the same kind of fit in a **partial-fraction basis** relative to the
current pole estimate (`phi_j(z) = z^j / D_prev(z)`), which stays well-conditioned for
multi-resonance systems — exactly the property `algorithm_backlog.md` cites when it says VF-style
pole relocation is "more robust... than Levy/SK" for these cases. `VectorFitting::fitMagnitude`
already implements that pole-relocation SK loop for real poles; `ComplexVectorFit` ports the same
loop, generalized to a complex response target and complex pole pairs.

## Algorithm

1. **Initialization:** `n_real_poles` real poles, log-spaced like `VectorFitting::initPoles`, plus
   `n_complex_pairs` complex-conjugate pairs at log-spaced center frequencies `omega_n` (same
   `[1e-3, 0.9*pi/Ts]` log-spaced range `VectorFitting::initPoles` uses) with a fixed initial
   damping ratio `zeta = 0.1` (lightly damped, matching the resonant systems this is built for),
   mapped to the discrete-time unit disk via
   `p = exp((-zeta*omega_n +/- j*omega_n*sqrt(1-zeta^2)) * Ts)` — the standard continuous-to-discrete
   pole map, generalizing `VectorFitting::initPoles`'s real-only `p = exp(-sigma*Ts)`.
2. **Each SK iteration:**
   - Build `D_prev(z) = prod_k (z - p_k)` from the current pole set (now genuinely complex factors
     for the complex-pair poles; the product over a conjugate pair is itself a real-coefficient
     quadratic, so `D_prev` evaluated at any `z` on the unit circle combines consistently).
   - Divide by `D_prev(z)` to linearise, exactly as `VectorFitting::buildSKSystem` does — but
     because the target response `H_i` is now complex (not a real magnitude), stack the real and
     imaginary parts of each frequency sample as **two real rows** (generalizing
     `buildSKSystem`'s real-part-only row), the same real-stacking trick
     `FreqDomainIdentifier::buildLevySystem` already uses.
   - Solve one real least-squares system for the numerator coefficients `c_0..c_n` and denominator
     adjustment `a_1..a_n`. These coefficients are the actual `TransferFunction` numerator/denominator
     — always real-valued, regardless of whether the resulting roots are real or a complex pair.
   - Build the companion matrix from the new `a_coeff` and eigendecompose it to get next
     iteration's poles. A real companion matrix's eigenvalues are **already exact conjugate
     pairs** — `VectorFitting`'s existing perturbation hack (`ev.real() + 1e-6*im`, used only to
     force a fake-real approximation) is dropped entirely; complex eigenvalues are kept as-is.
   - **Stability:** reflect any pole with `|p| >= 1` via the single unified formula
     `p <- 1 / conj(p)`, which reduces to `VectorFitting`'s existing `1/p` reflection for real `p`
     and correctly preserves conjugate-pair structure for complex `p` (if `p* = conj(p)` is also a
     pole, reflecting both yields `1/conj(p)` and `1/conj(p*) = 1/p`, which are still conjugates of
     each other).
3. **Convergence:** max absolute displacement of the solved coefficient vector `[c; a]` between
   iterations, below `tol` — matching `SKFit::fitSK`'s convergence criterion. This is deliberately
   *not* pole-displacement (what `VectorFitting::fitMagnitude` uses): matching individual complex
   poles across iterations when pairs can appear/disappear or swap order is ambiguous;
   coefficient displacement is order-invariant and avoids that problem.
4. **Model output:** the final iteration's `[c_coeff; a_coeff]` already *is* the fitted
   `TransferFunction`'s real numerator/denominator — no pole/residue reconstruction needed to
   build the model itself.
5. **Diagnostics (computed once, after the loop, not needed for the model):** final poles via the
   same companion-matrix eigendecomposition; residues via the closed-form partial-fraction formula
   `r_k = N(p_k) / D'(p_k)` (`D'` = derivative of the final denominator polynomial). This is simpler
   than `VectorFitting::fitMagnitude`'s residue step, which needs a second least-squares solve only
   because that class reconstructs a magnitude-evaluation `StateSpace`; here, residues are
   diagnostic-only.

## API

`lib/ComplexVectorFit.h` / `lib/ComplexVectorFit.cpp`. Naming parallels `SKFit`/`SKFitResult`/
`fitSK` exactly (the precedent this design follows most directly):

```cpp
namespace ctrl {

struct ComplexVectorFitResult
{
    TransferFunction model;                        // fitted discrete-time model
    std::vector<std::complex<double>> poles;       // diagnostic: final pole locations
    std::vector<std::complex<double>> residues;     // diagnostic: partial-fraction residues
    std::vector<double> iterError;                  // RMSE per iteration (FreqDomainIdentifier::fitRMSE)
    bool converged = false;
};

class ComplexVectorFit
{
public:
    static ComplexVectorFitResult fit(const std::vector<double> &omega,
                                       const std::vector<std::complex<double>> &response,
                                       int n_real_poles, int n_complex_pairs, double Ts,
                                       int max_iter = 20, double tol = 1e-6);

private:
    // Build the SK divided LS system for a complex target response (real+imag stacked rows),
    // generalizing VectorFitting::buildSKSystem from a real-magnitude target.
    static void buildSystem(const std::vector<std::complex<double>> &z_grid,
                             const std::vector<std::complex<double>> &response,
                             const std::vector<std::complex<double>> &poles,
                             Eigen::MatrixXd &A_out, Eigen::VectorXd &b_out);

    // p <- 1/conj(p) if |p| >= 1, else p unchanged. Reduces to VectorFitting's 1/p for real p.
    static std::complex<double> reflectPole(std::complex<double> p);

    // n_real_poles log-spaced real poles + n_complex_pairs log-spaced conjugate pairs,
    // mapped into the unit disk the same way VectorFitting::initPoles does.
    static std::vector<std::complex<double>> initPoles(int n_real_poles, int n_complex_pairs,
                                                         double omega_max, double Ts);
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(complex_vector_fit)
```

Reuses `FreqDomainIdentifier::fitRMSE` directly for `iterError` (no new RMSE code).

## Edge cases / numerical safety

- `omega.size() != response.size()`, or either empty -> `std::invalid_argument`.
- `n_real_poles + n_complex_pairs <= 0` -> `std::invalid_argument`.
- Fewer frequency samples than unknowns (underdetermined system) -> `std::invalid_argument`,
  matching `SKFit::fitSK`'s existing check.
- `D_prev(z) approx 0` at a sample frequency -> zero that row out, same guard
  `VectorFitting::buildSKSystem` already has (`Dk_abs2 < 1e-30`).
- This is offline identification code, not a `compute()`/`step()` hot path — the RT
  zero-allocation rules (`CLAUDE.md` section 7) do not apply, same exemption as `VectorFitting`,
  `SKFit`, and `FreqDomainIdentifier`.

## Wiring (full treatment, matching how FD1/`SKFit` shipped)

1. `lib/ComplexVectorFit.h`, `lib/ComplexVectorFit.cpp`
2. `lib/CMakeLists.txt` — add `ComplexVectorFit.cpp` to `CTRL_CORE_SOURCES`
3. `lib/ControllerToolbox.h` — `#include "ComplexVectorFit.h"`
4. `lib/Features.h` — feature registry entry (`complex_vector_fit`, matching `SKFit`'s `sk_fit`)
5. `bindings/estimation_bindings.cpp` (where `SKFit` is bound) — add `ComplexVectorFit` /
   `ComplexVectorFitResult` pybind11 bindings
6. `bindings/smoke_test.py` — add an assertion exercising `ctrl.ComplexVectorFit.fit(...)`
7. `tests/test_catch2_advanced.cpp` — Catch2 tests tagged `[complex_vector_fit]`
8. `examples/ex112_complex_vector_fit.cpp` + `examples/python/ex129_complex_vector_fit.py`
   (next free numbers as of this writing), plus `examples/CMakeLists.txt`, `compile.bat`,
   `compile.sh` updates

## Test plan

1. Synthetic response generated from a known 2-resonance system (2 complex-conjugate pole pairs)
   — `ComplexVectorFit::fit`'s recovered poles match the known poles within tolerance, and its
   fit RMSE is lower than `SKFit::fitSK` at the same order on the same data (validates the
   "more robust than Levy/SK for resonance" claim from `algorithm_backlog.md`).
2. Conjugate-pair integrity: every returned pole's conjugate is also present in `poles` (no
   orphaned complex pole, which would imply a non-real-valued time response).
3. Mixed case (`n_real_poles=1, n_complex_pairs=1`) — correctly identifies which poles are real
   vs. complex-paired.
4. Argument validation: mismatched/empty input lengths, zero total pole count, and an
   underdetermined system each throw `std::invalid_argument`.

## Out of scope

- `initial_poles` caller-supplied override (present in the roadmap doc's original sketch as
  `VectorFitComplexParams::initial_poles`) — dropped from this design; auto-init covers the test
  plan and example use case, and a caller-supplied override can be added later as a small
  additive follow-up if a real use case needs it (consistent with this codebase's YAGNI
  preference elsewhere).
- `StateSpace` output — the model is represented as a `TransferFunction` only (real polynomial
  coefficients, never needs a complex-valued state-space realization); no `StateSpace` conversion
  is in scope here, matching the roadmap sketch's `TransferFunction model` field.
