# ComplexVectorFit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `ComplexVectorFit`, a `lib/` class that fits a stable discrete-time SISO model to complex (magnitude+phase) frequency-response data using complex-conjugate pole pairs, fully wired (build, Python bindings, smoke test, C++ + Python examples, Catch2 tests, doc status updates), per the approved design at `docs/superpowers/specs/2026-06-25-complex-vector-fit-design.md`.

**Architecture:** One new standalone class (no shared base — it's a static-method identifier, like its siblings `SKFit`/`FreqDomainIdentifier`/`VectorFitting`). The algorithm is a Sanathanan-Koerner-reweighted least-squares loop in `TransferFunction`'s native `zinv = z^-1` basis (matching `FreqDomainIdentifier::buildLevySystem`'s convention, **not** `VectorFitting::buildSKSystem`'s internal ascending-`z^{+j}` convention, which is private to that class and would not produce valid `TransferFunction` coefficients if copied directly). Each iteration divides by the current denominator estimate `D_prev(zinv) = prod_k(1 - poles[k]*zinv)`, built from real and/or complex-conjugate-pair pole estimates; poles are relocated each iteration via a companion-matrix eigendecomposition (real matrix, so conjugate pairs come out exact — no perturbation hack needed, unlike `VectorFitting.cpp`'s real-pole-only workaround). Diagnostics (final poles + partial-fraction residues) are computed once after the loop via a closed-form formula, no second least-squares solve.

**Tech Stack:** C++20, Eigen (`Eigen::EigenSolver`, `Eigen::ColPivHouseholderQR`), pybind11, Catch2 v3, CMake, Python (numpy) for the example/smoke test.

## Global Constraints

- This is offline system-identification code (not a `compute()`/`step()` hot path) — the RT zero-allocation rules (`CLAUDE.md` section 7) do **not** apply here, the same exemption `VectorFitting`/`SKFit`/`FreqDomainIdentifier` already have.
- `TransferFunction`'s native convention is `H(zinv) = N(zinv)/D(zinv)`, `D[0]=1`, `zinv = exp(-j*omega*Ts)` — every basis function and companion-matrix construction in this plan is derived fresh for that convention, not copied from `VectorFitting.cpp`'s different internal convention.
- `compile.bat`/`compile.sh` example-target lists are hand-maintained — every new example must be added to both.
- Construction-time/call-time validation throws `std::invalid_argument` for invalid inputs (mismatched/empty lengths, zero total pole count, underdetermined systems) — matching `SKFit::fitSK`'s existing checks exactly.
- The algorithm and all numeric thresholds below were verified in a throwaway numpy prototype before being written into this plan (mirrors `2026-06-24-resonant-notch-pll-controllers.md`'s precedent of pre-verifying math); see the design spec's "Why this is not redundant with SKFit" section for what that prototyping found and corrected.

---

## Task 1: ComplexVectorFit core (class + library wiring + Catch2 tests)

**Files:**
- Create: `lib/ComplexVectorFit.h`
- Create: `lib/ComplexVectorFit.cpp`
- Modify: `lib/CMakeLists.txt:84` (append after `SKFit.cpp` in `CTRL_CORE_SOURCES`)
- Modify: `lib/ControllerToolbox.h:155` (append include after `SKFit.h`)
- Modify: `tests/test_catch2_advanced.cpp` (add `#include "ComplexVectorFit.h"` after line 57's `#include "ResonantController.h"`; append new `TEST_CASE`s at end of file, tag `[complex_vector_fit]`)

**Interfaces:**
- Consumes: `ctrl::TransferFunction` (`lib/PlantModel.h`), `CTRL_REGISTER_FEATURE` (`lib/Features.h`), `ctrl::FreqDomainIdentifier::fitRMSE`/`fitLevy` (`lib/FreqDomainIdentifier.h`)
- Produces: `struct ctrl::ComplexVectorFitResult { TransferFunction model; std::vector<std::complex<double>> poles; std::vector<std::complex<double>> residues; std::vector<double> iterError; bool converged = false; }`; `class ctrl::ComplexVectorFit` with `static ComplexVectorFitResult fit(const std::vector<double>& omega, const std::vector<std::complex<double>>& response, int n_real_poles, int n_complex_pairs, double Ts, int max_iter = 20, double tol = 1e-6)`

- [ ] **Step 1: Write `lib/ComplexVectorFit.h`**

```cpp
#pragma once
#include "PlantModel.h"
#include "Features.h"
#include <Eigen/Dense>
#include <vector>
#include <complex>

/**
 * @file ComplexVectorFit.h
 * @brief Vector Fitting with complex-conjugate pole pairs for resonant frequency-response data
 *        (Phase 3 FD2).
 *
 * Generalizes VectorFitting::fitMagnitude's real-pole/magnitude-only Sanathanan-Koerner loop to
 * (a) a full complex (magnitude+phase) target response and (b) complex-conjugate pole pairs, with
 * explicit pole/residue tracking - the one combination this codebase doesn't otherwise cover
 * (SKFit.h already fits a general complex response, but only ever returns polynomial
 * coefficients, not explicit pole locations).
 *
 * Basis convention matches TransferFunction's native form (H(zinv) = N(zinv)/D(zinv), D[0]=1,
 * zinv = z^-1), the same convention FreqDomainIdentifier::buildLevySystem/SKFit use - NOT
 * VectorFitting::buildSKSystem's internal ascending-z^{+j} convention, which is private to that
 * class's own StateSpace construction and would not produce valid TransferFunction coefficients
 * if copied directly.
 *
 * @see Gustavsen & Semlyen, "Rational approximation of frequency domain responses by vector
 *      fitting," IEEE Trans. Power Del. 14(3), 1999.
 * @see VectorFitting.h for the real-pole/magnitude-only sibling this generalizes.
 * @see SKFit.h for the complex-response sibling (Phase 3 FD1) - same input, but returns only
 *      polynomial coefficients, no explicit pole/residue tracking.
 * @see docs/superpowers/specs/2026-06-25-complex-vector-fit-design.md
 */

namespace ctrl {

/**
 * @brief Result of @ref ComplexVectorFit::fit.
 */
struct ComplexVectorFitResult
{
    TransferFunction model;                      ///< Fitted discrete-time model.
    std::vector<std::complex<double>> poles;     ///< Diagnostic: final pole locations.
    std::vector<std::complex<double>> residues;  ///< Diagnostic: partial-fraction residues.
    std::vector<double> iterError;                ///< RMSE per SK iteration.
    bool converged = false;                       ///< True if coefficient displacement < tol.
};

/**
 * @brief Vector Fitting with complex-conjugate pole pairs for a complex frequency response.
 */
class ComplexVectorFit
{
public:
    /**
     * @brief Fit n_real_poles real poles plus n_complex_pairs complex-conjugate pole pairs to a
     *        complex (magnitude+phase) frequency response.
     *
     * @param omega           Frequency grid [rad/s].
     * @param response        Complex frequency response H(e^{j*omega*Ts}) at each frequency.
     * @param n_real_poles    Number of real poles (>= 0).
     * @param n_complex_pairs Number of complex-conjugate pole pairs (>= 0).
     * @param Ts              Sample time [s].
     * @param max_iter        SK iteration limit.
     * @param tol             Stop when max coefficient displacement drops below this.
     * @return ComplexVectorFitResult with the fitted model, diagnostic poles/residues, the
     *         per-iteration RMSE history, and a convergence flag.
     * @throws std::invalid_argument If @p omega/@p response are empty or mismatched in length,
     *         if n_real_poles + n_complex_pairs <= 0, or if the system is underdetermined.
     */
    static ComplexVectorFitResult fit(const std::vector<double> &omega,
                                       const std::vector<std::complex<double>> &response,
                                       int n_real_poles, int n_complex_pairs, double Ts,
                                       int max_iter = 20, double tol = 1e-6);

private:
    // Build the SK divided LS system for a complex target response (real+imag stacked rows) in
    // the zinv domain: divides by D_prev(zinv) = prod_k(1 - poles[k]*zinv); D_prev=1 reproduces
    // FreqDomainIdentifier::buildLevySystem's unweighted basis exactly.
    static void buildSystem(const std::vector<std::complex<double>> &zinv_grid,
                             const std::vector<std::complex<double>> &response,
                             const std::vector<std::complex<double>> &poles,
                             Eigen::MatrixXd &A_out, Eigen::VectorXd &b_out);

    // p <- 1/conj(p) if |p| >= 1, else p unchanged.
    static std::complex<double> reflectPole(std::complex<double> p);

    // n_real_poles log-spaced real poles + n_complex_pairs log-spaced conjugate pairs (fixed
    // initial damping zeta=0.1), mapped into the unit disk.
    static std::vector<std::complex<double>> initPoles(int n_real_poles, int n_complex_pairs,
                                                         double omega_max, double Ts);
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(complex_vector_fit)
```

- [ ] **Step 2: Write `lib/ComplexVectorFit.cpp`**

```cpp
#include "ComplexVectorFit.h"
#include "FreqDomainIdentifier.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

std::complex<double> ComplexVectorFit::reflectPole(std::complex<double> p)
{
    if (std::abs(p) >= 1.0)
        return 1.0 / std::conj(p);
    return p;
}

std::vector<std::complex<double>> ComplexVectorFit::initPoles(
    int n_real_poles, int n_complex_pairs, double omega_max, double Ts)
{
    std::vector<std::complex<double>> poles;
    poles.reserve(n_real_poles + 2 * n_complex_pairs);

    const double omega_min = 1e-3;
    const double sig_lo = omega_min;
    const double sig_hi = std::min(omega_max * 0.8, M_PI / Ts * 0.9);

    for (int k = 0; k < n_real_poles; ++k)
    {
        const double t = (n_real_poles == 1) ? 0.5
                          : static_cast<double>(k) / (n_real_poles - 1);
        const double sigma = std::exp(std::log(sig_lo) * (1.0 - t) + std::log(sig_hi) * t);
        poles.emplace_back(std::exp(-sigma * Ts), 0.0);
    }

    constexpr double kZeta = 0.1;
    for (int k = 0; k < n_complex_pairs; ++k)
    {
        const double t = (n_complex_pairs == 1) ? 0.5
                          : static_cast<double>(k) / (n_complex_pairs - 1);
        const double wn = std::exp(std::log(sig_lo) * (1.0 - t) + std::log(sig_hi) * t);
        const double sigma = kZeta * wn;
        const double wd = wn * std::sqrt(1.0 - kZeta * kZeta);
        const std::complex<double> s(-sigma, wd);
        const std::complex<double> p = std::exp(s * Ts);
        poles.push_back(p);
        poles.push_back(std::conj(p));
    }
    return poles;
}

void ComplexVectorFit::buildSystem(
    const std::vector<std::complex<double>> &zinv_grid,
    const std::vector<std::complex<double>> &response,
    const std::vector<std::complex<double>> &poles,
    Eigen::MatrixXd &A_out,
    Eigen::VectorXd &b_out)
{
    const int N = static_cast<int>(zinv_grid.size());
    const int n_poles = static_cast<int>(poles.size());
    const int nc = n_poles + 1;
    const int ncols = nc + n_poles;

    A_out.resize(2 * N, ncols);
    b_out.resize(2 * N);

    for (int i = 0; i < N; ++i)
    {
        const std::complex<double> zinv = zinv_grid[i];
        const std::complex<double> H_i = response[i];

        std::complex<double> Dprev(1.0, 0.0);
        for (int k = 0; k < n_poles; ++k)
            Dprev *= (1.0 - poles[k] * zinv);

        if (std::norm(Dprev) < 1e-30)
        {
            A_out.row(2 * i).setZero();
            A_out.row(2 * i + 1).setZero();
            b_out(2 * i) = 0.0;
            b_out(2 * i + 1) = 0.0;
            continue;
        }

        std::complex<double> zinv_pow(1.0, 0.0);
        for (int j = 0; j < nc; ++j)
        {
            const std::complex<double> basis = zinv_pow / Dprev;
            A_out(2 * i, j)     = basis.real();
            A_out(2 * i + 1, j) = basis.imag();
            zinv_pow *= zinv;
        }

        zinv_pow = zinv;
        for (int j = 0; j < n_poles; ++j)
        {
            const std::complex<double> basis = -H_i * zinv_pow / Dprev;
            A_out(2 * i, nc + j)     = basis.real();
            A_out(2 * i + 1, nc + j) = basis.imag();
            zinv_pow *= zinv;
        }

        const std::complex<double> rhs = H_i / Dprev;
        b_out(2 * i)     = rhs.real();
        b_out(2 * i + 1) = rhs.imag();
    }
}

ComplexVectorFitResult ComplexVectorFit::fit(
    const std::vector<double> &omega,
    const std::vector<std::complex<double>> &response,
    int n_real_poles, int n_complex_pairs, double Ts,
    int max_iter, double tol)
{
    const int N = static_cast<int>(omega.size());
    if (N == 0 || response.size() != static_cast<size_t>(N))
        throw std::invalid_argument(
            "ComplexVectorFit::fit: omega and response must be non-empty and same size.");
    if (n_real_poles < 0 || n_complex_pairs < 0 || (n_real_poles + n_complex_pairs) <= 0)
        throw std::invalid_argument(
            "ComplexVectorFit::fit: need at least one pole (n_real_poles + n_complex_pairs > 0).");

    const int n_poles = n_real_poles + 2 * n_complex_pairs;
    const int n_unknowns = 2 * n_poles + 1;
    if (N < n_unknowns)
        throw std::invalid_argument(
            "ComplexVectorFit::fit: fewer frequency samples than unknown coefficients "
            "(system underdetermined).");

    std::vector<std::complex<double>> zinv_grid(N);
    for (int i = 0; i < N; ++i)
        zinv_grid[i] = std::polar(1.0, -omega[i] * Ts);

    std::vector<std::complex<double>> poles =
        initPoles(n_real_poles, n_complex_pairs, omega.back(), Ts);

    Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(n_poles, n_poles);
    for (int k = 0; k < n_poles - 1; ++k)
        companion(k + 1, k) = 1.0;
    Eigen::EigenSolver<Eigen::MatrixXd> es;

    const int nc = n_poles + 1;
    std::vector<double> num(nc, 0.0);
    std::vector<double> den(nc, 0.0);
    den[0] = 1.0;
    Eigen::VectorXd xPrev;
    std::vector<double> iterError;
    bool converged = false;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        Eigen::MatrixXd A;
        Eigen::VectorXd b;
        buildSystem(zinv_grid, response, poles, A, b);

        const Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

        for (int i = 0; i < nc; ++i)
            num[i] = x(i);
        den[0] = 1.0;
        for (int j = 1; j <= n_poles; ++j)
            den[j] = x(nc + j - 1);

        iterError.push_back(FreqDomainIdentifier::fitRMSE(omega, response, num, den, Ts));

        if (iter > 0)
        {
            const double maxDelta = (x - xPrev).cwiseAbs().maxCoeff();
            if (maxDelta < tol)
            {
                converged = true;
                xPrev = x;
                break;
            }
        }
        xPrev = x;

        for (int k = 0; k < n_poles; ++k)
            companion(0, k) = -den[k + 1];
        es.compute(companion, false);
        const auto &evals = es.eigenvalues();
        for (int k = 0; k < n_poles; ++k)
            poles[k] = reflectPole(evals(k));
    }

    for (int k = 0; k < n_poles; ++k)
        companion(0, k) = -den[k + 1];
    es.compute(companion, false);
    std::vector<std::complex<double>> finalPoles(n_poles);
    for (int k = 0; k < n_poles; ++k)
        finalPoles[k] = es.eigenvalues()(k);

    // Residues (diagnostic only, no model dependency): r_k = Nz(p_k)/Dz'(p_k), where
    // Nz(z) = sum_j num[j]*z^(n_poles-j), Dz(z) = sum_j den[j]*z^(n_poles-j) are num/den
    // re-expressed as polynomials in z (clearing the zinv powers) - H(1/z) = Nz(z)/Dz(z)
    // exactly, with Dz's roots being the companion matrix's eigenvalues found above.
    std::vector<std::complex<double>> residues(n_poles);
    for (int k = 0; k < n_poles; ++k)
    {
        const std::complex<double> p = finalPoles[k];
        std::complex<double> Nz(0.0, 0.0);
        for (int j = 0; j < nc; ++j)
            Nz = Nz * p + num[j];
        std::complex<double> Dzp(0.0, 0.0);
        for (int j = 0; j < n_poles; ++j)
            Dzp = Dzp * p + den[j] * static_cast<double>(n_poles - j);
        residues[k] = (std::abs(Dzp) > 1e-12) ? (Nz / Dzp) : std::complex<double>(0.0, 0.0);
    }

    TransferFunction model(num, den, Ts);
    return ComplexVectorFitResult{model, finalPoles, residues, iterError, converged};
}

} // namespace ctrl
```

- [ ] **Step 3: Wire into `lib/CMakeLists.txt`**

Insert after line 84 (`SKFit.cpp`):
```cmake
    ComplexVectorFit.cpp
```

- [ ] **Step 4: Wire into `lib/ControllerToolbox.h`**

Insert after line 155 (`#include "SKFit.h"`):
```cpp
#include "ComplexVectorFit.h"        ///< ComplexVectorFit - complex-conjugate-pole Vector Fitting (Phase 3 FD2).
```

- [ ] **Step 5: Add the include and write Catch2 tests in `tests/test_catch2_advanced.cpp`**

Add after line 57 (`#include "ResonantController.h"`):
```cpp
#include "ComplexVectorFit.h"
```

Append at the end of the file (this file already includes `<random>` at line 62 and `<complex>` transitively via `ControllerToolbox.h`, so no new includes are needed for the test bodies below):

```cpp
// -----------------------------------------------------------------------------
// ComplexVectorFit - complex-conjugate-pole Vector Fitting (Phase 3 FD2)
// -----------------------------------------------------------------------------

namespace
{
std::vector<double> cvfPolyMulPair(const std::vector<double> &p, double a1, double a2)
{
    std::vector<double> result(p.size() + 2, 0.0);
    for (std::size_t i = 0; i < p.size(); ++i)
    {
        result[i]     += p[i];
        result[i + 1] += p[i] * a1;
        result[i + 2] += p[i] * a2;
    }
    return result;
}

// Builds H(zinv) = N(zinv)/D(zinv) from `pairs` complex-conjugate pole pairs (plus one
// optional real pole), evaluates it on `freqs` via tf2ss + SystemAnalysis::getFrequencyResponse,
// and adds Gaussian measurement noise - mirrors SKFit's existing test-data convention.
std::vector<std::complex<double>> cvfSyntheticResponse(
    const std::vector<std::pair<double, double>> &pairs,
    double realPole, bool hasRealPole,
    const std::vector<double> &freqs, double Ts, unsigned seed, double noiseStd)
{
    std::vector<double> den{1.0};
    for (const auto &pr : pairs)
        den = cvfPolyMulPair(den, -2.0 * pr.first * std::cos(pr.second), pr.first * pr.first);
    if (hasRealPole)
    {
        std::vector<double> next(den.size() + 1, 0.0);
        for (std::size_t i = 0; i < den.size(); ++i)
        {
            next[i]     += den[i];
            next[i + 1] += den[i] * (-realPole);
        }
        den = next;
    }
    std::vector<double> num(den.size(), 0.0);
    num[1] = 0.05;

    const ctrl::TransferFunction tf(num, den, Ts);
    const auto sys = ctrl::tf2ss(tf);
    auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, noiseStd);
    for (auto &h : response) h += std::complex<double>(noise(rng), noise(rng));
    return response;
}
} // namespace

TEST_CASE("ComplexVectorFit recovers known poles of a 3-resonance system and far outperforms "
          "a one-shot Levy fit",
          "[complex_vector_fit]")
{
    const double Ts = 0.1;
    const std::vector<std::pair<double, double>> specs{{0.99, 0.4}, {0.985, 0.55}, {0.99, 0.75}};

    std::vector<double> freqs;
    for (int i = 1; i <= 80; ++i) freqs.push_back(0.25 * i);

    const auto response = cvfSyntheticResponse(specs, 0.0, false, freqs, Ts, 11, 0.02);

    const auto cvfResult  = ctrl::ComplexVectorFit::fit(freqs, response, 0, 3, Ts, 30);
    const auto levyResult = ctrl::FreqDomainIdentifier::fitLevy(freqs, response, 6, 6, Ts);

    REQUIRE(cvfResult.iterError.size() >= 1u);
    REQUIRE(std::isfinite(cvfResult.iterError.back()));
    // Verified in a numpy prototype: ~600x improvement on this exact scenario; 2x is a
    // generous margin against this test's different (C++) RNG stream producing different noise.
    REQUIRE(cvfResult.iterError.back() < 0.5 * levyResult.rmse);

    REQUIRE(cvfResult.poles.size() == 6u);
    std::vector<std::complex<double>> truePoles;
    for (const auto &pr : specs)
    {
        truePoles.emplace_back(pr.first * std::cos(pr.second),  pr.first * std::sin(pr.second));
        truePoles.emplace_back(pr.first * std::cos(pr.second), -pr.first * std::sin(pr.second));
    }
    for (const auto &p : cvfResult.poles)
    {
        double bestDist = 1e9;
        for (const auto &tp : truePoles)
            bestDist = std::min(bestDist, std::abs(p - tp));
        // Verified in the prototype to recover poles within ~1e-3; 0.05 leaves generous margin.
        REQUIRE(bestDist < 0.05);
    }
}

TEST_CASE("ComplexVectorFit's returned poles always include each pole's conjugate partner",
          "[complex_vector_fit]")
{
    const double Ts = 0.1;
    const std::vector<std::pair<double, double>> specs{{0.97, 0.6}, {0.95, 1.5}};

    std::vector<double> freqs;
    for (int i = 1; i <= 40; ++i) freqs.push_back(0.5 * i);

    const auto response = cvfSyntheticResponse(specs, 0.0, false, freqs, Ts, 7, 0.01);
    const auto result = ctrl::ComplexVectorFit::fit(freqs, response, 0, 2, Ts);

    REQUIRE(result.poles.size() == 4u);
    for (const auto &p : result.poles)
    {
        bool foundConjugate = false;
        for (const auto &q : result.poles)
            if (std::abs(q - std::conj(p)) < 1e-6) { foundConjugate = true; break; }
        REQUIRE(foundConjugate);
    }
}

TEST_CASE("ComplexVectorFit correctly identifies a mixed real-pole + complex-pair system",
          "[complex_vector_fit]")
{
    const double Ts = 0.1;
    const std::vector<std::pair<double, double>> specs{{0.97, 0.6}};
    const double realPole = 0.8;

    std::vector<double> freqs;
    for (int i = 1; i <= 40; ++i) freqs.push_back(0.5 * i);

    const auto response = cvfSyntheticResponse(specs, realPole, true, freqs, Ts, 7, 0.01);
    const auto result = ctrl::ComplexVectorFit::fit(freqs, response, 1, 1, Ts);

    REQUIRE(result.poles.size() == 3u);

    int realCount = 0, complexCount = 0;
    for (const auto &p : result.poles)
    {
        if (std::abs(p.imag()) < 1e-3) ++realCount;
        else ++complexCount;
    }
    REQUIRE(realCount == 1);
    REQUIRE(complexCount == 2);

    double bestRealDist = 1e9;
    for (const auto &p : result.poles)
        if (std::abs(p.imag()) < 1e-3)
            bestRealDist = std::min(bestRealDist, std::abs(p.real() - realPole));
    REQUIRE(bestRealDist < 0.05);
}

TEST_CASE("ComplexVectorFit throws on invalid inputs", "[complex_vector_fit]")
{
    const std::vector<double> freqs{1.0, 5.0, 10.0};
    const std::vector<std::complex<double>> response{{0.1, 0.0}, {0.2, -0.1}, {0.1, 0.05}};

    REQUIRE_THROWS_AS(ctrl::ComplexVectorFit::fit({}, {}, 1, 0, 0.1), std::invalid_argument);
    REQUIRE_THROWS_AS(
        ctrl::ComplexVectorFit::fit(freqs, {{0.1, 0.0}, {0.2, -0.1}}, 1, 0, 0.1),
        std::invalid_argument);
    REQUIRE_THROWS_AS(
        ctrl::ComplexVectorFit::fit(freqs, response, 0, 0, 0.1),
        std::invalid_argument);
    // n_real_poles=1, n_complex_pairs=1 -> n_poles=3, n_unknowns=7, but only 3 samples given.
    REQUIRE_THROWS_AS(
        ctrl::ComplexVectorFit::fit(freqs, response, 1, 1, 0.1),
        std::invalid_argument);
}
```

- [ ] **Step 6: Configure and build the test target**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release` then `cmake --build build --target test_catch2_advanced`
Expected: clean compile (no errors). If `ComplexVectorFit.cpp`/`.h` have a typo, this is where it surfaces.

- [ ] **Step 7: Run the new tests and verify they pass**

Run: `build/tests/test_catch2_advanced.exe [complex_vector_fit]`
Expected: `All tests passed (N assertions in 4 test cases)`. If a test fails on a marginal threshold (e.g. the RMSE-vs-Levy margin or a pole-distance tolerance), the algorithm itself has already been verified correct in the design spec's prototyping — check first whether it's a genuinely-unlucky noise draw from this run's RNG stream (try re-running; `std::mt19937`/`std::normal_distribution` differ from the numpy prototype's RNG, so exact convergence numbers will differ run-to-run only if the seed changes, but are deterministic for a fixed seed) before suspecting the implementation. Document any real discrepancy in `docs/cumulative_bug_report.md` per this repo's workflow rather than re-deriving the algorithm from scratch.

- [ ] **Step 8: Commit**

```bash
git add lib/ComplexVectorFit.h lib/ComplexVectorFit.cpp lib/CMakeLists.txt lib/ControllerToolbox.h tests/test_catch2_advanced.cpp
git commit -m "Add ComplexVectorFit core (complex-conjugate-pole Vector Fitting, Phase 3 FD2)"
```

---

## Task 2: ComplexVectorFit integration (bindings, smoke test, examples, docs)

**Files:**
- Modify: `bindings/estimation_bindings.cpp:1056` (add binding section before the closing `}` of `bind_estimation`, right after the `SKFit` block)
- Modify: `bindings/smoke_test.py` (append new section after the `SKFit` section)
- Create: `examples/ex112_complex_vector_fit.cpp`
- Modify: `examples/CMakeLists.txt:184` (append `add_example(ex112_complex_vector_fit)` after `ex111_narmax`)
- Modify: `compile.bat:152` (append `ex112_complex_vector_fit` to the target list)
- Modify: `compile.sh:193` (append `ex112_complex_vector_fit` to the target list)
- Create: `examples/python/ex129_complex_vector_fit.py`
- Modify: `docs/algorithm_backlog.md` (move FD2 from "open" to "Already done")
- Modify: `docs/ALGORITHM_ROADMAP_PHASE3.md` (mark FD2 `Done` in the status table and top status line)

**Interfaces:**
- Consumes: `ctrl::ComplexVectorFit`/`ctrl::ComplexVectorFitResult` (Task 1), `ctrl::FreqDomainIdentifier::fitLevy` (existing), `ctrl::TransferFunction`/`ctrl::tf2ss`/`ctrl::SystemAnalysis::getFrequencyResponse` (existing)
- Produces: Python `ctrl.ComplexVectorFit`/`ctrl.ComplexVectorFitResult`; example binaries `ex112_complex_vector_fit` (C++) and `ex129_complex_vector_fit.py` (Python)

- [ ] **Step 1: Add the Python binding to `bindings/estimation_bindings.cpp`**

Insert after the `SKFit` binding block (after line 1056, the `.def_static("fit_sk", ...)` call's closing `;`, and before the file's final closing `}` at line 1057):

```cpp

    // -----------------------------------------------------------------------
    // ComplexVectorFit - complex-conjugate-pole Vector Fitting
    // (Phase 3 FD2)
    // -----------------------------------------------------------------------
    py::class_<ctrl::ComplexVectorFitResult>(m, "ComplexVectorFitResult",
        "Result from ComplexVectorFit.fit().")
        .def_readonly("model",      &ctrl::ComplexVectorFitResult::model,
                      "Fitted discrete-time model (TransferFunction) after the final iteration.")
        .def_readonly("poles",      &ctrl::ComplexVectorFitResult::poles,
                      "Diagnostic: final pole locations (real or complex-conjugate pairs).")
        .def_readonly("residues",   &ctrl::ComplexVectorFitResult::residues,
                      "Diagnostic: partial-fraction residues corresponding to poles.")
        .def_readonly("iter_error", &ctrl::ComplexVectorFitResult::iterError,
                      "RMSE per SK iteration.")
        .def_readonly("converged",  &ctrl::ComplexVectorFitResult::converged,
                      "True if coefficient displacement dropped below tol.");

    py::class_<ctrl::ComplexVectorFit>(m, "ComplexVectorFit", R"doc(
Vector Fitting with complex-conjugate pole pairs for a complex (magnitude+phase) frequency
response.

Generalizes VectorFitting's real-pole/magnitude-only Sanathanan-Koerner loop to complex poles and
a full complex response, tracking explicit pole/residue locations each iteration (unlike SKFit,
which only returns polynomial coefficients).

Example
-------
>>> result = ctrl.ComplexVectorFit.fit(omega, response, n_real_poles=0, n_complex_pairs=2, Ts=0.1)
>>> print(result.model.num, result.model.den, result.poles)
)doc")
        .def_static("fit", &ctrl::ComplexVectorFit::fit,
             py::arg("omega"), py::arg("response"),
             py::arg("n_real_poles"), py::arg("n_complex_pairs"), py::arg("Ts"),
             py::arg("max_iter") = 20, py::arg("tol") = 1e-6,
             "Fit n_real_poles real poles plus n_complex_pairs complex-conjugate pole pairs to a "
             "complex frequency response via SK-reweighted least squares.");
```

- [ ] **Step 2: Append to `bindings/smoke_test.py`**

Add after the existing `SKFit` section (after the `print('SKFit smoke test passed.')` line):

```python
# ComplexVectorFit - complex-conjugate-pole Vector Fitting (Phase 3 FD2) smoke test
assert hasattr(ctrl, 'ComplexVectorFit'), "ComplexVectorFit not bound"
assert hasattr(ctrl, 'ComplexVectorFitResult'), "ComplexVectorFitResult not bound"
assert ctrl.registry_has('complex_vector_fit'), "complex_vector_fit not registered"

_cvf_r, _cvf_theta = 0.97, 0.6
_cvf_tf = ctrl.TransferFunction(
    [0.0, 1.0 - _cvf_r, 0.0],
    [1.0, -2.0 * _cvf_r * np.cos(_cvf_theta), _cvf_r ** 2],
    0.1)
_cvf_sys = ctrl.tf2ss(_cvf_tf)
_cvf_freqs = list(np.linspace(0.5, 20.0, 40))
_cvf_response = ctrl.SystemAnalysis.get_frequency_response(_cvf_sys, _cvf_freqs)
_cvf_result = ctrl.ComplexVectorFit.fit(_cvf_freqs, _cvf_response, n_real_poles=0, n_complex_pairs=1, Ts=0.1)
assert len(_cvf_result.iter_error) >= 1, "fit: expected at least one iteration"
assert len(_cvf_result.poles) == 2, "fit: expected 2 poles (1 complex-conjugate pair)"
print('ComplexVectorFit smoke test passed.')
```

- [ ] **Step 3: Rebuild and run the smoke test**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTRL_BUILD_PYTHON_BINDINGS=ON` then `cmake --build build --target ctrl_toolbox`
Run: `conda run -n soft_robotics -- python bindings/smoke_test.py`
Expected: no exceptions; output includes `ComplexVectorFit smoke test passed.`

- [ ] **Step 4: Write `examples/ex112_complex_vector_fit.cpp`**

```cpp
/**
 * @file ex112_complex_vector_fit.cpp
 * @brief Phase 3 (FD2): complex-conjugate-pole Vector Fitting vs. a one-shot Levy fit.
 *
 * Fits a 3-resonance system (3 lightly-damped complex-conjugate pole pairs) from a noisy
 * frequency-response sample set with ComplexVectorFit::fit, comparing against
 * FreqDomainIdentifier::fitLevy (one-shot, same order) on the same data.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <random>

namespace
{
std::vector<double> polyMulPair(const std::vector<double> &p, double a1, double a2)
{
    std::vector<double> result(p.size() + 2, 0.0);
    for (std::size_t i = 0; i < p.size(); ++i)
    {
        result[i]     += p[i];
        result[i + 1] += p[i] * a1;
        result[i + 2] += p[i] * a2;
    }
    return result;
}
} // namespace

int main()
{
    const double Ts = 0.1;
    const std::vector<std::pair<double, double>> specs{{0.99, 0.4}, {0.985, 0.55}, {0.99, 0.75}};

    std::vector<double> den{1.0};
    for (const auto &pr : specs)
        den = polyMulPair(den, -2.0 * pr.first * std::cos(pr.second), pr.first * pr.first);
    std::vector<double> num(den.size(), 0.0);
    num[1] = 0.05;

    const ctrl::TransferFunction tf_true(num, den, Ts);
    const auto sys = ctrl::tf2ss(tf_true);

    std::vector<double> freqs;
    for (int i = 1; i <= 80; ++i) freqs.push_back(0.25 * i);
    auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    std::mt19937 rng(11);
    std::normal_distribution<double> noise(0.0, 0.02);
    for (auto &h : response) h += std::complex<double>(noise(rng), noise(rng));

    const auto cvfResult  = ctrl::ComplexVectorFit::fit(freqs, response, 0, 3, Ts, 30);
    const auto levyResult = ctrl::FreqDomainIdentifier::fitLevy(freqs, response, 6, 6, Ts);

    std::printf("Levy (one-shot, order 6)    rmse = %.5f\n", levyResult.rmse);
    std::printf("ComplexVectorFit            rmse = %.5f after %zu iterations (converged=%s)\n",
                cvfResult.iterError.back(), cvfResult.iterError.size(),
                cvfResult.converged ? "yes" : "no");
    std::cout << "Recovered poles (magnitude @ angle [rad]):\n";
    for (const auto &p : cvfResult.poles)
        std::printf("  %.4f @ %.4f\n", std::abs(p), std::arg(p));

    const bool ok = std::isfinite(cvfResult.iterError.back())
                  && cvfResult.iterError.back() < 0.5 * levyResult.rmse;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
```

- [ ] **Step 5: Wire the example into `examples/CMakeLists.txt`**

Insert after line 184 (`add_example(ex111_narmax)`):
```cmake
# Phase 3 (Algorithm Roadmap Phase 3): FD2 complex-conjugate-pole Vector Fitting
add_example(ex112_complex_vector_fit)
```

- [ ] **Step 6: Wire the example into `compile.bat` and `compile.sh`**

In `compile.bat`, insert after `ex111_narmax` in the target list (line 152):
```
    ex112_complex_vector_fit
```

In `compile.sh`, insert after `ex111_narmax` in the `TARGETS` list (line 193):
```
    ex112_complex_vector_fit
```

- [ ] **Step 7: Build and run the example**

Run: `cmake --build build --target ex112_complex_vector_fit`
Run: `build/examples/ex112_complex_vector_fit.exe` (path may be `build/examples/Release/...` depending on generator)
Expected: prints the Levy/CVF RMSE lines, the recovered poles, then `PASS`.

- [ ] **Step 8: Write `examples/python/ex129_complex_vector_fit.py`**

```python
"""
ex129_complex_vector_fit.py

Phase 3 (FD2): complex-conjugate-pole Vector Fitting vs. a one-shot Levy fit.

Mirrors ex112_complex_vector_fit.cpp - fits a 3-resonance system (3 lightly-damped
complex-conjugate pole pairs) from a noisy frequency-response sample set with
ComplexVectorFit.fit(), comparing against FreqDomainIdentifier.fit_levy() (one-shot, same order)
on the same data.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'ComplexVectorFit'):
        raise AttributeError("ComplexVectorFit not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

Ts = 0.1
specs = [(0.99, 0.4), (0.985, 0.55), (0.99, 0.75)]


def poly_mul_pair(p, a1, a2):
    result = np.zeros(len(p) + 2)
    for i, c in enumerate(p):
        result[i] += c
        result[i + 1] += c * a1
        result[i + 2] += c * a2
    return result


den = np.array([1.0])
for r, theta in specs:
    den = poly_mul_pair(den, -2.0 * r * np.cos(theta), r * r)
num = np.zeros(len(den))
num[1] = 0.05

tf_true = ctrl.TransferFunction(list(num), list(den), Ts)
sys_ss = ctrl.tf2ss(tf_true)

freqs = list(0.25 * np.arange(1, 81))
response = np.array(ctrl.SystemAnalysis.get_frequency_response(sys_ss, freqs))

rng = np.random.default_rng(11)
response = response + (rng.normal(0.0, 0.02, len(response)) + 1j * rng.normal(0.0, 0.02, len(response)))

cvf_result = ctrl.ComplexVectorFit.fit(freqs, list(response), n_real_poles=0, n_complex_pairs=3,
                                        Ts=Ts, max_iter=30)
levy_result = ctrl.FreqDomainIdentifier.fit_levy(freqs, list(response), num_order=6, den_order=6, Ts=Ts)

print(f"Levy (one-shot, order 6)    rmse = {levy_result.rmse:.5f}")
print(f"ComplexVectorFit            rmse = {cvf_result.iter_error[-1]:.5f} after "
      f"{len(cvf_result.iter_error)} iterations (converged={cvf_result.converged})")
print("Recovered poles (magnitude @ angle [rad]):")
for p in cvf_result.poles:
    print(f"  {abs(p):.4f} @ {np.angle(p):.4f}")

ok = np.isfinite(cvf_result.iter_error[-1]) and cvf_result.iter_error[-1] < 0.5 * levy_result.rmse
print("[PASS] All checks passed." if ok else "[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
```

- [ ] **Step 9: Run the Python example**

Run: `conda run -n soft_robotics -- python examples/python/ex129_complex_vector_fit.py`
Expected: prints the same style of output as the C++ example, ending in `[PASS] All checks passed.`

- [ ] **Step 10: Update `docs/algorithm_backlog.md` status**

Insert a new row after line 87 (the `NARMAX` row in the "Already done" table):
```markdown
| Complex-conjugate-pole Vector Fitting | `lib/ComplexVectorFit.h`/`.cpp` (generalizes `VectorFitting`'s pole-relocation SK loop to a complex magnitude+phase target with complex-conjugate pole pairs; explicit pole/residue diagnostics) — Phase 3 FD2, `examples/ex112_complex_vector_fit.cpp`. See [2026-06-25-complex-vector-fit-design.md](superpowers/specs/2026-06-25-complex-vector-fit-design.md). |
```

Replace lines 89-91 (the "Shipped" summary):
```markdown
**Shipped:** `ALGORITHM_ROADMAP_PHASE3.md` Phase 3 partial (5 designs: ML1, ML2, NC3, SI4, FD2),
see `docs/cumulative_bug_report.md` Part 69 and
[2026-06-25-complex-vector-fit-design.md](superpowers/specs/2026-06-25-complex-vector-fit-design.md).
SI3 (MOESP/CVA) and ML3 (GP-MPC) remain open.
```

Replace lines 139-144 (the "Frequency-Domain Identification Extensions" section's now-resolved
"What's left" table):
```markdown
Generalizing SK iteration to full complex-response fitting, and complex-conjugate-pole Vector
Fitting, are both **done** — see the "Already done" table above (`lib/SKFit.h`,
`lib/ComplexVectorFit.h`). No items remain open in this category.
```

- [ ] **Step 11: Update `docs/ALGORITHM_ROADMAP_PHASE3.md` status**

Replace lines 4-5 (top status line):
```markdown
**Status:** Planning — 21 of 32 items shipped (Phase 1 and Phase 2 complete; Phase 3 partial:
ML1/ML2/NC3/SI4/FD2 done, SI3/ML3 open).
```

Replace line 52 (the FD2 status-table row):
```markdown
| FD2 | Complex-Conjugate-Pole Vector Fitting | 3 | Done |
```

- [ ] **Step 12: Commit**

```bash
git add bindings/estimation_bindings.cpp bindings/smoke_test.py examples/ex112_complex_vector_fit.cpp examples/CMakeLists.txt compile.bat compile.sh examples/python/ex129_complex_vector_fit.py docs/algorithm_backlog.md docs/ALGORITHM_ROADMAP_PHASE3.md
git commit -m "Wire ComplexVectorFit into Python bindings, examples, and roadmap docs"
```
