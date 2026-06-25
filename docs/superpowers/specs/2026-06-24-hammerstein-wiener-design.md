# Design: Hammerstein-Wiener Model Identification

**Date:** 2026-06-24
**Status:** Approved, not yet implemented

## Motivation

`docs/algorithm_backlog.md`'s System Identification section flags this directly: "no current
equivalent," despite Hammerstein (static input nonlinearity -> linear dynamics) and Wiener
(linear dynamics -> static output nonlinearity) structures being extremely common in practice
(valve/actuator deadzone or saturation; sensor saturation). Standalone spec — distinct (system-
ID) subject matter from every other Phase 1 item, no shared design content.

The roadmap's "reuse `RecursiveLeastSquares` wholesale... run in batch mode each outer iteration"
claim doesn't hold up: `RecursiveLeastSquares::update(double y, double u)` (confirmed,
`RecursiveLeastSquares.h:67`) is strictly sample-by-sample/online — there is no batch entry
point, and repeatedly looping `update()` over the same dataset with a `reset()` between outer
iterations would be repurposing a recursive *filter* as a batch solver (forgetting-factor and
covariance-reset semantics that have no meaning in a batch context). The design below uses a
direct batch ARX least-squares helper instead — simpler and more numerically direct.

## Scope

- Single-input/single-output. Hammerstein and Wiener are two separate static methods
  (`fitHammerstein`/`fitWiener`) sharing one internal batch-ARX helper, not a combined
  Hammerstein-**and**-Wiener (nonlinearity on both ends) model — matching the roadmap's own
  `HammersteinWienerResult` shape (`nl_output_coeffs` empty for Hammerstein-only).
- Polynomial static nonlinearity only (`nl_degree`), matching the roadmap sketch.
- Alternating least squares (linear sub-step / nonlinear sub-step), not a joint nonlinear
  optimizer — matching the roadmap's explicit "fit via alternating linear/nonlinear least
  squares."

## Components

### `lib/HammersteinWienerIdentifier.h` / `.cpp` — standalone, no shared base

```cpp
struct HammersteinWienerParams {
    int    na, nb;             // linear ARX orders
    int    nl_degree = 3;      // polynomial degree for the static nonlinearity
    int    max_iter  = 20;
    double tol        = 1e-6;
};

struct HammersteinWienerResult {
    Eigen::VectorXd nl_input_coeffs;    // Hammerstein static map, [c0..c_d], c1 fixed = 1.0
    Eigen::VectorXd nl_output_coeffs;   // Wiener static map, [d0..d_d], d1 fixed = 1.0 (empty
                                          // for fitHammerstein's result)
    TransferFunction linear_part;
    bool converged;
    int  iters;
};

class HammersteinWienerIdentifier {
public:
    static HammersteinWienerResult fitHammerstein(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                                                    double Ts, const HammersteinWienerParams &params = {});
    static HammersteinWienerResult fitWiener(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                                               double Ts, const HammersteinWienerParams &params = {});
};
```

**Scale-ambiguity normalization (resolves a gap the roadmap sketch doesn't address):**
Hammerstein/Wiener separation has a fundamental scale ambiguity — multiplying the static
nonlinearity's coefficients by `k` and the linear part's gain by `1/k` produces an identical
input-output map, so an unconstrained alternating fit has no unique fixed point. Resolved by
fixing the polynomial's **linear-term coefficient to `1.0`** (`nl_input_coeffs[1]` for
Hammerstein, `nl_output_coeffs[1]` for Wiener) after every outer iteration, rescaling the
just-updated nonlinearity coefficients so that term is exactly 1 and absorbing the corresponding
factor into the linear part's gain instead. **Catch2 test data must be constructed under this
same convention** (or rescaled before comparison) — a standard requirement for testing any
Hammerstein-Wiener identification method, not a workaround specific to this implementation.

**Shared internal batch-ARX helper** (`HammersteinWienerIdentifier.cpp`, not in the public
header):
```cpp
// Solves y[k] = -sum_i a_i*y[k-i] + sum_j b_j*v[k-j] + e[k] via one batch QR solve
// (regressor matrix built once, Eigen::ColPivHouseholderQR) - the direct batch analogue of
// RecursiveLeastSquares's regressor convention, NOT a loop over RecursiveLeastSquares::update().
static void fitBatchARX(const Eigen::VectorXd &v, const Eigen::VectorXd &y,
                         int na, int nb, Eigen::VectorXd &a_out, Eigen::VectorXd &b_out);
```

**`fitHammerstein` algorithm** (Narendra-Gallman-style alternating LS):
1. Initialize the static map as identity (`c = [0, 1, 0, ..., 0]`).
2. **Linear sub-step:** compute `v[k] = sum_m c_m * u[k]^m` (current nonlinearity applied to
   `u`), call `fitBatchARX(v, y, na, nb, a, b)`.
3. **Nonlinear sub-step:** with `a`/`b` now fixed, the model `y[k] + sum_i a_i*y[k-i] = sum_j b_j
   * v[k-j] = sum_j b_j * sum_m c_m*u[k-j]^m = sum_m c_m * (sum_j b_j*u[k-j]^m)` is **linear in
   `c`** — build the regressor `R_m[k] = sum_j b_j*u[k-j]^m` for each power `m = 0..nl_degree`
   and the target `y[k] + sum_i a_i*y[k-i]`, solve the resulting `(nl_degree+1)`-unknown LS
   problem via QR for a new `c`. Rescale per the normalization above.
4. Repeat steps 2-3 until `max_iter` or `max(||a_new-a_old||_inf, ||b_new-b_old||_inf,
   ||c_new-c_old||_inf) < tol`.

**`fitWiener` algorithm** (symmetric structure, with the practical simplification noted below):
1. Initialize the forward static map as identity (`d = [0, 1, 0, ..., 0]`), `w_est = y`.
2. **Linear sub-step:** `fitBatchARX(u, w_est, na, nb, a, b)` (fit `u -> w_est`).
3. Compute `w_pred[k]` by simulating the just-fit linear ARX model forward from `u` alone
   (`w_pred[k] = -sum_i a_i*w_pred[k-i] + sum_j b_j*u[k-j]`).
4. **Nonlinear sub-step (forward map):** fit `y[k] = sum_m d_m * w_pred[k]^m` via batch LS
   (regress `y` against powers of `w_pred` directly — no inversion needed here, since `w_pred`
   is already known). Rescale per the normalization above; this `d` is the result's
   `nl_output_coeffs`.
5. **Approximate-inverse refresh** (resolves needing `h^{-1}(y)` for the next iteration's
   `w_est` without symbolic/numeric polynomial inversion, which risks non-existent or
   multiple real roots for higher-degree polynomials): fit a **separate** small auxiliary
   polynomial `g` via batch LS regressing the just-computed `w_pred[k]` (as the target) against
   powers of `y[k]` (as the regressor) — i.e. `w_pred[k] ~= sum_m e_m * y[k]^m`. Set
   `w_est[k] = sum_m e_m * y[k]^m` for the next iteration's linear sub-step. `g`/`e` are
   internal-only, never exposed in `HammersteinWienerResult` — this is a deliberate, documented
   simplification of the textbook alternating-LS approach (which would invert `h` directly),
   trading one extra cheap LS solve per outer iteration for avoiding polynomial-root-finding
   edge cases (non-monotonic fits, complex roots) entirely.
6. Repeat steps 2-5 until `max_iter` or coefficient-change `< tol` (same criterion as
   Hammerstein, over `a`/`b`/`d`).

**Polynomial static-nonlinearity basis** (resolves the roadmap's "reuse `SINDy`'s polynomial-
basis pattern" into a concrete, *not*-`SINDy`-dependent helper): a direct scalar
`[1, v, v^2, ..., v^d]` evaluation, ~5 lines — `SINDy::libraryRow` (`SINDy.h:190`) builds a
**multivariate** monomial library jointly over a state vector `x` and input `u`, sized for full
state-vector regression; instantiating a `SINDy` object with `n_state=1, n_input=0` just to get a
1-D polynomial row would be a needless dependency for something `Eigen::VectorXd` power
iteration handles directly. Only the *pattern* (basis-function library idea) is reused, not the
class.

## Explicitly out of scope (this phase)

- **Non-polynomial static nonlinearities** (piecewise-linear deadzone/saturation as a distinct
  parameterization, splines) — polynomial only, matching the roadmap sketch.
- **Combined Hammerstein-Wiener (nonlinearity on both input and output simultaneously)** —
  two separate methods, matching `HammersteinWienerResult`'s own shape.
- **Symbolic/numeric inversion of the output nonlinearity** — the approximate-inverse-via-
  auxiliary-fit approach (step 5 above) is used instead; see rationale there.
- **MIMO** — SISO only.

## Implementation checklist

(Lighter non-`IController` utility-class checklist, same shape as `FreqDomainIdentifier`/
`SubspaceID`.)

1. `lib/HammersteinWienerIdentifier.h`/`.cpp` + `CTRL_REGISTER_FEATURE(hammerstein_wiener)`
2. `lib/CMakeLists.txt` — add `HammersteinWienerIdentifier.cpp` to `CTRL_CORE_SOURCES`
3. `lib/ControllerToolbox.h` — add `#include "HammersteinWienerIdentifier.h"`
4. `bindings/estimation_bindings.cpp` — bind alongside `RecursiveLeastSquares`
5. `bindings/smoke_test.py` — fit a tiny synthetic Hammerstein dataset, confirm `converged`
   field is accessible and `linear_part` is a valid `TransferFunction`
6. `examples/ex100_hammerstein_wiener.cpp` — a valve-with-deadzone-style Hammerstein scenario
   (known cubic input nonlinearity + known 2nd-order linear part) + `examples/python/
   ex117_hammerstein_wiener.py`
7. `tests/test_catch2_advanced.cpp` — tests under `[hammerstein_wiener]`
8. `examples/CMakeLists.txt`, `compile.bat`/`compile.sh` — add `ex100_hammerstein_wiener`

## Testing plan

**`[hammerstein_wiener]`**
1. Synthetic Hammerstein system (known cubic input nonlinearity, linear term normalized to 1 +
   known 2nd-order linear part) — both `nl_input_coeffs` and `linear_part` recovered within
   tolerance of the (correspondingly-normalized) ground truth.
2. Synthetic Wiener system, symmetric test — `nl_output_coeffs`/`linear_part` recovered within
   tolerance.
3. Pure-linear system (nonlinearity = identity, i.e. `c = [0, 1, 0, ...]`) — alternating fit
   converges to a near-identity nonlinearity (`c_0`, `c_2..c_d` near zero), doesn't overfit
   spurious higher-order terms onto a system that has none.
4. `max_iter` exhausted without reaching `tol` — `converged = false`, `iters = max_iter`, result
   still returned (best estimate so far), not a thrown exception or garbage output.
