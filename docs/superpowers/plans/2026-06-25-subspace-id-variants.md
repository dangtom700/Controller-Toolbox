# Subspace ID Method Variants Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `SubspaceMethod` enum (`MOESP`/`N4SID`/`CVA`) and a new `subspaceID()` entry point to `lib/SubspaceID.h`/`.cpp`, giving callers a real choice of subspace-ID weighting while keeping `n4sid()` byte-for-byte backward compatible, per the approved design at `docs/superpowers/specs/2026-06-25-subspace-id-variants-design.md`.

**Architecture:** `n4sid()`'s existing 6-step body (Hankel build, LQ decomposition, SVD of the oblique projection `L32`, shift-invariance extraction of `A`/`C`, `B`/`D` regression, stochastic realization) becomes the new `subspaceID()` function, with Step 3 (the SVD step) parameterized by `method`: MOESP uses `L32` unweighted (bit-identical to today); N4SID right-weights by `L22^-1` (a triangular solve, `L22` being the existing LQ factor's `(Wp,Wp)` block); CVA additionally left-weights each output channel by a per-channel noise-scale estimate derived from `L33`'s row norms (the existing LQ factor's `(Yf,Yf)` block). `n4sid()` becomes a one-line delegate to `subspaceID(..., SubspaceMethod::MOESP, ...)`. Steps 1-2 and 4-6 are untouched.

**Tech Stack:** C++20, Eigen (`Eigen::HouseholderQR`, `Eigen::JacobiSVD`, `Eigen::TriangularView::solve`), pybind11, Catch2 v3, CMake, Python (numpy) for the example/smoke test.

## Global Constraints

- This is offline, batch identification code (not a `compute()`/`step()` hot path) - the RT zero-allocation rules (`CLAUDE.md` section 7) do **not** apply here, the same exemption `n4sid()` already has.
- `n4sid()`'s exact current name, signature, and behavior must not change - it is called from `lib/LPVSystemID.cpp`, `examples/ex31_subspace_id.cpp`, `examples/ex20_system_identification_data.cpp`, the Boiler Control case study, and existing tests/bindings.
- This is an **extension to an existing class** (no new `lib/` source file, no `lib/CMakeLists.txt`/`lib/ControllerToolbox.h` changes) - only the files listed per task need updating.
- Construction-time validation throws nothing (matches `n4sid()`'s existing contract of returning `SubspaceIDResult{success=false, message=...}` rather than throwing) - N4SID/CVA's new near-singular-`L22` guard follows this same return-not-throw convention.

---

## Task 1: Core refactor (`SubspaceMethod`, `subspaceID()`, `n4sid()` delegation) + Catch2 tests

**Files:**
- Modify: `lib/SubspaceID.h` (add enum + new declaration, after line 133's `n4sid()` declaration)
- Modify: `lib/SubspaceID.cpp` (refactor body)
- Modify: `tests/test_catch2_advanced.cpp` (append new `TEST_CASE`s at end of file, tag `[subspace_id_variants]`; no new `#include` needed - `SubspaceID.h` is already pulled in via `ControllerToolbox.h`)

**Interfaces:**
- Consumes: nothing new (only Eigen + the existing `StateSpace`/`SubspaceIDResult`).
- Produces: `enum class ctrl::SubspaceMethod { MOESP, N4SID, CVA }`; `ctrl::SubspaceIDResult ctrl::subspaceID(const Eigen::MatrixXd& Y, const Eigen::MatrixXd& U, int n_order, int i_horizon, double Ts, SubspaceMethod method = SubspaceMethod::MOESP, double svd_tol = -1.0)`.

- [ ] **Step 1: Add the enum and declaration to `lib/SubspaceID.h`**

Insert after line 133 (the closing `;` of the existing `n4sid()` declaration), before `int suggestOrder(...)`:

```cpp
/**
 * @brief Weighting variant for subspaceID().
 *
 * All three share the same Hankel/LQ/extraction pipeline n4sid() already uses; they differ
 * only in how the oblique projection L32 is weighted before its SVD (Step 3).
 */
enum class SubspaceMethod
{
    MOESP,  ///< Unweighted oblique projection (Verhaegen & Dewilde 1992). Identical to n4sid().
    N4SID,  ///< Right-weights the past block by its Uf-conditioned covariance (Cholesky-clean).
    CVA     ///< Additionally weights each output channel by an estimated noise scale.
};

/**
 * @brief Batch subspace identification with a choice of weighting (MOESP / N4SID / CVA).
 *
 * MOESP reproduces n4sid() exactly. N4SID right-weights the past data block by its
 * Uf-conditioned covariance. CVA additionally left-weights each *output channel* by a
 * noise-scale estimate (derived from the LQ factor's residual block) -- helps when output
 * channels have very different noise levels. This is a regularized, per-channel-scale
 * variant, not full canonical-variate (cross-covariance) whitening: that requires inverting
 * an (i*p)x(i*p) matrix whose true rank is only n_order (<< i*p whenever i_horizon >
 * n_order, the normal operating regime), which is numerically ill-conditioned even with
 * ridge regularization (verified in prototyping -- see the design doc).
 *
 * kalmanGain/innovCov are computed for all three methods, including MOESP -- the same
 * "free" diagnostic n4sid() already always populates, regardless of whether the chosen
 * method has a textbook stochastic step.
 *
 * @param Y         Output data matrix (p * N): rows = outputs, columns = time samples.
 * @param U         Input data matrix  (m * N): rows = inputs,  columns = time samples.
 * @param n_order   Desired model order n.
 * @param i_horizon Block-row count in Hankel matrices. Recommend i >= 2.n_order/p, minimum n_order+1.
 * @param Ts        Sample time [s].
 * @param method    Weighting variant. Defaults to MOESP (n4sid()'s existing behavior).
 * @param svd_tol   SVD truncation tolerance; if > 0, n_order is capped at the number of
 *                  singular values exceeding svd_tol. Pass -1.0 to disable (default).
 * @return SubspaceIDResult containing the model, singular values, and diagnostics.
 *         Check result.success before using result.model. success=false (with a
 *         descriptive message) on a near-singular weighting matrix (N4SID/CVA only,
 *         indicating non-persistent input excitation), in addition to n4sid()'s existing
 *         failure modes.
 */
SubspaceIDResult subspaceID(const Eigen::MatrixXd &Y,
                            const Eigen::MatrixXd &U,
                            int n_order,
                            int i_horizon,
                            double Ts,
                            SubspaceMethod method = SubspaceMethod::MOESP,
                            double svd_tol = -1.0);

```

- [ ] **Step 2: Refactor `lib/SubspaceID.cpp`**

Replace the entire `n4sid()` function (the whole block from `SubspaceIDResult n4sid(...)  { ... }`, currently lines 29-271) with:

```cpp
SubspaceIDResult subspaceID(const Eigen::MatrixXd &Y,
                            const Eigen::MatrixXd &U,
                            int n_order,
                            int i,
                            double Ts,
                            SubspaceMethod method,
                            double svd_tol)
{
    SubspaceIDResult res;
    const int p = Y.rows(); // output dimension
    const int m = U.rows(); // input dimension
    const int N = Y.cols(); // number of samples
    const int s = N - 2 * i; // number of Hankel columns

    // Input checks
    if (U.cols() != N)
    {
        res.message = "U and Y must have the same number of columns (time samples).";
        return res;
    }
    if (s <= n_order)
    {
        res.message = "Not enough data: need N > 2*i_horizon + n_order. "
                      "Increase N or reduce i_horizon / n_order.";
        return res;
    }
    if (i <= n_order / p)
    {
        res.message = "i_horizon too small: need i_horizon > n_order / p. "
                      "Increase i_horizon.";
        return res;
    }
    if (n_order < 1)
    {
        res.message = "n_order must be >= 1.";
        return res;
    }

    // ------------------------------------------------------------------
    // Step 1: build 2i-block Hankel matrices and partition past/future
    // ------------------------------------------------------------------
    const Eigen::MatrixXd Yh = buildHankel(Y, 2 * i); // 2i*p * s
    const Eigen::MatrixXd Uh = buildHankel(U, 2 * i); // 2i*m * s

    const Eigen::MatrixXd Yp = Yh.topRows(i * p);      // i*p * s  (past)
    const Eigen::MatrixXd Yf = Yh.bottomRows(i * p);   // i*p * s  (future)
    const Eigen::MatrixXd Up = Uh.topRows(i * m);      // i*m * s  (past)
    const Eigen::MatrixXd Uf = Uh.bottomRows(i * m);   // i*m * s  (future)

    // Wp = [Up; Yp]  (i*(m+p) * s)
    const int r_uf = i * m;
    const int r_wp = i * (m + p);
    const int r_yf = i * p;
    const int r_tot = r_uf + r_wp + r_yf;

    Eigen::MatrixXd Z(r_tot, s);
    Z.topRows(r_uf)              = Uf;
    Z.middleRows(r_uf, i * m)    = Up;
    Z.middleRows(r_uf + i * m, i * p) = Yp;
    Z.bottomRows(r_yf)           = Yf;

    // ------------------------------------------------------------------
    // Step 2: thin LQ decomposition of Z via QR of Z'.  See SubspaceID.h's
    // class docstring for the full derivation of why L32 equals the oblique
    // projection Yf /_{Uf} Wp (Verhaegen & Dewilde 1992, Eq. 4.3).
    // ------------------------------------------------------------------
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(Z.transpose()); // QR of (s * r_tot)
    const Eigen::MatrixXd L =
        qr.matrixQR()
          .topRows(r_tot)
          .triangularView<Eigen::Upper>()
          .transpose(); // r_tot * r_tot, lower triangular

    const Eigen::MatrixXd L32 = L.block(r_uf + r_wp, r_uf, r_yf, r_wp);
    const Eigen::MatrixXd L22 = L.block(r_uf, r_uf, r_wp, r_wp);
    const Eigen::MatrixXd L33 = L.block(r_uf + r_wp, r_uf + r_wp, r_yf, r_yf);

    // ------------------------------------------------------------------
    // Step 3: weight L32 by method, then SVD to get extended observability
    // matrix Gamma. See SubspaceID.h's subspaceID() docstring and
    // docs/superpowers/specs/2026-06-25-subspace-id-variants-design.md for
    // the full numerical reasoning (in particular why CVA uses a per-channel
    // scalar weight rather than full canonical-variate matrix whitening).
    // ------------------------------------------------------------------
    Eigen::MatrixXd M;
    Eigen::VectorXd cva_w1; // per-row weight (length r_yf), populated only for CVA

    if (method == SubspaceMethod::MOESP)
    {
        M = L32;
    }
    else
    {
        const double l22_max_diag = L22.diagonal().cwiseAbs().maxCoeff();
        const double l22_min_diag = L22.diagonal().cwiseAbs().minCoeff();
        if (l22_max_diag < 1e-300 || l22_min_diag < 1e-10 * l22_max_diag)
        {
            res.message = "Input excitation too weak for weighted subspace ID "
                          "(L22 near-singular); check persistence of excitation.";
            return res;
        }

        const Eigen::MatrixXd L22_inv =
            L22.triangularView<Eigen::Lower>().solve(Eigen::MatrixXd::Identity(r_wp, r_wp));

        if (method == SubspaceMethod::N4SID)
        {
            M = L32 * L22_inv;
        }
        else // CVA
        {
            Eigen::VectorXd noise_var = Eigen::VectorXd::Zero(p);
            for (int r = 0; r < i; ++r)
                for (int j = 0; j < p; ++j)
                    noise_var(j) += L33.row(r * p + j).squaredNorm();
            noise_var /= static_cast<double>(i);

            const double floor_val = 1e-12 * std::max(noise_var.maxCoeff(), 1e-300);
            noise_var = noise_var.cwiseMax(floor_val);
            const Eigen::VectorXd sigma = noise_var.cwiseSqrt();

            cva_w1.resize(r_yf);
            for (int r = 0; r < i; ++r)
                cva_w1.segment(r * p, p) = sigma.cwiseInverse();

            M = (cva_w1.asDiagonal() * L32) * L22_inv;
        }
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    res.singularValues = svd.singularValues();

    if (svd_tol > 0.0)
    {
        int valid_rank = 0;
        for (int k = 0; k < svd.singularValues().size(); ++k)
        {
            if (svd.singularValues()(k) > svd_tol)
                valid_rank++;
        }
        if (n_order > valid_rank)
            n_order = std::max(1, valid_rank);
    }

    if (n_order > svd.singularValues().size())
    {
        res.message = "n_order exceeds rank of L32. Reduce n_order.";
        return res;
    }

    const Eigen::VectorXd S_sqrt = svd.singularValues().head(n_order).cwiseSqrt();
    Eigen::MatrixXd Gamma = svd.matrixU().leftCols(n_order) * S_sqrt.asDiagonal();
    if (method == SubspaceMethod::CVA)
        Gamma = cva_w1.cwiseInverse().asDiagonal() * Gamma; // undo per-channel left weighting

    // ------------------------------------------------------------------
    // Step 4: extract C and A from shift invariance of Gamma
    // ------------------------------------------------------------------
    const Eigen::MatrixXd C = Gamma.topRows(p);
    const Eigen::MatrixXd Gamma_up   = Gamma.topRows((i - 1) * p);
    const Eigen::MatrixXd Gamma_down = Gamma.bottomRows((i - 1) * p);
    const Eigen::MatrixXd A = Gamma_up.colPivHouseholderQr().solve(Gamma_down);

    // ------------------------------------------------------------------
    // Step 5: least-squares regression for B and D.
    // ------------------------------------------------------------------
    const Eigen::MatrixXd X_hat = Gamma.colPivHouseholderQr().solve(Yf); // n * s

    const int T = s - 1;
    Eigen::MatrixXd Phi_BD(T, m);
    Eigen::MatrixXd Lhs_D(p, T);
    Eigen::MatrixXd Lhs_B(n_order, T);

    for (int k = 0; k < T; ++k)
    {
        const Eigen::VectorXd u_k = U.col(i + k);
        const Eigen::VectorXd x_k   = X_hat.col(k);
        const Eigen::VectorXd x_k1  = X_hat.col(k + 1);
        const Eigen::VectorXd y_k   = Y.col(i + k);

        Phi_BD.row(k)  = u_k.transpose();
        Lhs_D.col(k)   = y_k   - C * x_k;
        Lhs_B.col(k)   = x_k1  - A * x_k;
    }

    const Eigen::MatrixXd D =
        (Phi_BD.colPivHouseholderQr().solve(Lhs_D.transpose())).transpose();
    const Eigen::MatrixXd B =
        (Phi_BD.colPivHouseholderQr().solve(Lhs_B.transpose())).transpose();

    // ------------------------------------------------------------------
    // Step 6: Stochastic realisation - Kalman gain K and innovation
    // covariance Lambda. Computed for all methods (see subspaceID()'s
    // docstring for why this is intentional even for MOESP).
    // ------------------------------------------------------------------
    {
        Eigen::MatrixXd eps(p, T);
        Eigen::MatrixXd eta(n_order, T);

        for (int k = 0; k < T; ++k)
        {
            const Eigen::VectorXd u_k  = U.col(i + k);
            const Eigen::VectorXd x_k  = X_hat.col(k);
            const Eigen::VectorXd x_k1 = X_hat.col(k + 1);
            const Eigen::VectorXd y_k  = Y.col(i + k);

            eps.col(k) = y_k  - C * x_k  - D * u_k;
            eta.col(k) = x_k1 - A * x_k  - B * u_k;
        }

        const Eigen::MatrixXd innov_cov = (eps * eps.transpose()) / static_cast<double>(T);
        const Eigen::MatrixXd ee = eps * eps.transpose();
        const Eigen::MatrixXd ne = eta * eps.transpose();
        const Eigen::MatrixXd K_est = ee.ldlt().solve(ne.transpose()).transpose();

        res.kalmanGain = K_est;
        res.innovCov   = innov_cov;
    }

    // ------------------------------------------------------------------
    // Pack result
    // ------------------------------------------------------------------
    res.model   = std::make_optional<StateSpace>(A, B, C, D, Ts);
    res.success = true;
    return res;
}

SubspaceIDResult n4sid(const Eigen::MatrixXd &Y,
                       const Eigen::MatrixXd &U,
                       int n_order,
                       int i_horizon,
                       double Ts,
                       double svd_tol)
{
    return subspaceID(Y, U, n_order, i_horizon, Ts, SubspaceMethod::MOESP, svd_tol);
}
```

Leave `buildHankel()` (the anonymous-namespace helper) and `suggestOrder()` exactly as they are - neither changes.

- [ ] **Step 3: Configure and build the test target**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release` then `cmake --build build --target test_catch2_advanced`
Expected: clean compile. If `CTRL_HAS_SUBSPACE` is OFF in your build config (it's ON by default per `CLAUDE.md` section 2 - `CTRL_ENABLE_SUBSPACE`), this whole file is conditionally excluded; check `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -LA | grep SUBSPACE` if anything related to `subspaceID` fails to link.

- [ ] **Step 4: Run the existing n4sid tests to confirm the refactor is behavior-preserving**

Run: `build/tests/test_catch2_advanced.exe [n4sid]`
Expected: `All tests passed` - these tests call `ctrl::n4sid()` directly and must pass unchanged, since `n4sid()` now just delegates to `subspaceID(..., MOESP, ...)`.

- [ ] **Step 5: Add the new tests to `tests/test_catch2_advanced.cpp`**

Append at the end of the file (no new `#include` needed; `SubspaceID.h` is already pulled in via `ControllerToolbox.h`, and `<random>`/`<complex>` are already included):

```cpp
// -----------------------------------------------------------------------------
// SubspaceID method variants - MOESP / N4SID / CVA (Phase 3 SI3)
// -----------------------------------------------------------------------------

namespace
{
// Simulates a discrete-time LTI system and adds independent Gaussian noise with a
// possibly different std per output channel, mirroring the design spec's prototype.
void simulateForSubspaceVariants(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                                  const Eigen::MatrixXd &C, const Eigen::MatrixXd &D,
                                  const Eigen::MatrixXd &U, const Eigen::VectorXd &noiseStd,
                                  unsigned seed, Eigen::MatrixXd &Y_out)
{
    const int n = A.rows();
    const int p = C.rows();
    const int N = U.cols();
    Y_out.resize(p, N);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 1.0);

    for (int k = 0; k < N; ++k)
    {
        const Eigen::VectorXd u_k = U.col(k);
        Eigen::VectorXd y_k = C * x + D * u_k;
        for (int j = 0; j < p; ++j)
            y_k(j) += noise(rng) * noiseStd(j);
        Y_out.col(k) = y_k;
        x = A * x + B * u_k;
    }
}
} // namespace

TEST_CASE("subspaceID(MOESP) matches n4sid() bit-for-bit (regression)",
          "[subspace_id_variants]")
{
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    std::mt19937 rng_u(42);
    std::normal_distribution<double> u_dist(0.0, 1.0);
    Eigen::MatrixXd U(1, 2000);
    for (int k = 0; k < U.cols(); ++k) U(0, k) = u_dist(rng_u);

    Eigen::MatrixXd Y;
    simulateForSubspaceVariants(A_true, B_true, C_true, D_true, U,
                                Eigen::Vector2d(0.01, 0.01), 7, Y);

    const auto r1 = ctrl::n4sid(Y, U, 2, 10, Ts);
    const auto r2 = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::MOESP);

    REQUIRE(r1.success);
    REQUIRE(r2.success);
    REQUIRE(r1.model.value().A.isApprox(r2.model.value().A, 1e-12));
    REQUIRE(r1.model.value().B.isApprox(r2.model.value().B, 1e-12));
    REQUIRE(r1.model.value().C.isApprox(r2.model.value().C, 1e-12));
    REQUIRE(r1.singularValues.isApprox(r2.singularValues, 1e-12));
}

TEST_CASE("subspaceID recovers a known 2-output system with equal noise (all 3 methods)",
          "[subspace_id_variants]")
{
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    std::mt19937 rng_u(42);
    std::normal_distribution<double> u_dist(0.0, 1.0);
    Eigen::MatrixXd U(1, 2000);
    for (int k = 0; k < U.cols(); ++k) U(0, k) = u_dist(rng_u);

    Eigen::MatrixXd Y;
    simulateForSubspaceVariants(A_true, B_true, C_true, D_true, U,
                                Eigen::Vector2d(0.01, 0.01), 7, Y);

    const double true_eig_mag = std::abs(A_true.eigenvalues()(0));

    for (auto method : {ctrl::SubspaceMethod::MOESP, ctrl::SubspaceMethod::N4SID,
                        ctrl::SubspaceMethod::CVA})
    {
        const auto res = ctrl::subspaceID(Y, U, 2, 10, Ts, method);
        REQUIRE(res.success);
        const double est_eig_mag = std::abs(res.model.value().A.eigenvalues()(0));
        REQUIRE_THAT(est_eig_mag, WithinAbs(true_eig_mag, 0.05));
    }
}

TEST_CASE("subspaceID(CVA) beats MOESP and N4SID on the high-noise channel "
          "when output noise scales are mismatched",
          "[subspace_id_variants]")
{
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    std::mt19937 rng_u(42);
    std::normal_distribution<double> u_dist(0.0, 1.0);
    Eigen::MatrixXd U(1, 2000);
    for (int k = 0; k < U.cols(); ++k) U(0, k) = u_dist(rng_u);

    // Channel 1 (index 1) has 60x the noise std of channel 0.
    Eigen::MatrixXd Y;
    simulateForSubspaceVariants(A_true, B_true, C_true, D_true, U,
                                Eigen::Vector2d(0.005, 0.3), 7, Y);

    // Frequency-response error on the high-noise channel only, vs. the true system.
    auto chanError = [&](const ctrl::StateSpace &model) {
        const std::vector<double> freqs{0.5, 1.0, 2.0, 4.0, 8.0, 12.0, 16.0, 20.0};
        const auto resp_true = ctrl::SystemAnalysis::getFrequencyResponse(
            ctrl::StateSpace(A_true, B_true, C_true, D_true, Ts), freqs);
        const auto resp_est = ctrl::SystemAnalysis::getFrequencyResponse(model, freqs);
        double err = 0.0;
        for (std::size_t k = 0; k < freqs.size(); ++k)
            err += std::abs(std::abs(resp_est[k]) - std::abs(resp_true[k]));
        return err / static_cast<double>(freqs.size());
    };

    const auto moesp = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::MOESP);
    const auto n4sidv = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::N4SID);
    const auto cva = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::CVA);
    REQUIRE(moesp.success);
    REQUIRE(n4sidv.success);
    REQUIRE(cva.success);

    const double err_moesp = chanError(moesp.model.value());
    const double err_n4sid = chanError(n4sidv.model.value());
    const double err_cva   = chanError(cva.model.value());

    REQUIRE(err_cva < err_moesp);
    REQUIRE(err_cva < err_n4sid);
}

TEST_CASE("suggestOrder runs unchanged across all 3 subspaceID methods", "[subspace_id_variants]")
{
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    std::mt19937 rng_u(42);
    std::normal_distribution<double> u_dist(0.0, 1.0);
    Eigen::MatrixXd U(1, 2000);
    for (int k = 0; k < U.cols(); ++k) U(0, k) = u_dist(rng_u);

    Eigen::MatrixXd Y;
    simulateForSubspaceVariants(A_true, B_true, C_true, D_true, U,
                                Eigen::Vector2d(0.005, 0.3), 7, Y);

    for (auto method : {ctrl::SubspaceMethod::MOESP, ctrl::SubspaceMethod::N4SID,
                        ctrl::SubspaceMethod::CVA})
    {
        const auto res = ctrl::subspaceID(Y, U, 6, 10, Ts, method);
        REQUIRE(res.success);
        const int order = ctrl::suggestOrder(res.singularValues, 0.01);
        REQUIRE(order >= 1);
    }
}

TEST_CASE("subspaceID(N4SID/CVA) reports failure (not a crash/NaN) on degenerate excitation",
          "[subspace_id_variants]")
{
    // Constant input -> Wp's Uf-conditioned covariance (L22) is singular.
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    Eigen::MatrixXd U = Eigen::MatrixXd::Constant(1, 500, 1.0);
    Eigen::MatrixXd Y;
    simulateForSubspaceVariants(A_true, B_true, C_true, D_true, U,
                                Eigen::Vector2d(0.01, 0.01), 7, Y);

    const auto n4sidv = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::N4SID);
    const auto cva = ctrl::subspaceID(Y, U, 2, 10, Ts, ctrl::SubspaceMethod::CVA);

    REQUIRE_FALSE(n4sidv.success);
    REQUIRE_FALSE(n4sidv.message.empty());
    REQUIRE_FALSE(cva.success);
    REQUIRE_FALSE(cva.message.empty());
}
```

- [ ] **Step 6: Rebuild and run the new tests**

Run: `cmake --build build --target test_catch2_advanced` then `build/tests/test_catch2_advanced.exe [subspace_id_variants]`
Expected: `All tests passed (N assertions in 5 test cases)`. If the degenerate-excitation test (`Step 5`'s last `TEST_CASE`) doesn't trigger `success=false` with the constant-input data given, increase the constant-input length's redundancy is not the fix - verify `i_horizon=10` still satisfies `s > n_order` for `N=500` (`s = N - 2*i = 480`, fine); the near-singularity comes from `L22`'s conditioning on a Uf-correlated/non-exciting input, not from data quantity.

- [ ] **Step 7: Commit**

```bash
git add lib/SubspaceID.h lib/SubspaceID.cpp tests/test_catch2_advanced.cpp
git commit -m "Add SubspaceMethod (MOESP/N4SID/CVA) variants to SubspaceID (Phase 3 SI3)"
```

---

## Task 2: Integration (bindings, smoke test, examples, docs)

**Files:**
- Modify: `bindings/advanced_bindings.cpp:398` (insert before `#endif  // CTRL_HAS_SUBSPACE`, right after the existing `suggest_order` binding)
- Modify: `bindings/smoke_test.py` (extend the existing SubspaceID smoke-test section)
- Create: `examples/ex113_subspace_id_variants.cpp`
- Modify: `examples/CMakeLists.txt:184` (append `add_example(ex113_subspace_id_variants)` after `ex112_complex_vector_fit` if FD2 has already been wired by the time this task runs - otherwise after `ex111_narmax`; check the file's current tail before editing)
- Modify: `compile.bat` / `compile.sh` (append `ex113_subspace_id_variants` to the target lists, same position rule as above)
- Create: `examples/python/ex130_subspace_id_variants.py`
- Modify: `docs/algorithm_backlog.md` (move MOESP/CVA from "open" to "Already done")
- Modify: `docs/ALGORITHM_ROADMAP_PHASE3.md` (mark SI3 `Done`)

**Interfaces:**
- Consumes: `ctrl::subspaceID`/`ctrl::SubspaceMethod`/`ctrl::SubspaceIDResult` (Task 1), `ctrl::suggestOrder` (existing).
- Produces: Python `ctrl.subspace_id(..., method=...)`, `ctrl.SubspaceMethod.MOESP/.N4SID/.CVA`; example binaries `ex113_subspace_id_variants` (C++) and `ex130_subspace_id_variants.py` (Python).

**Numbering note:** `ex112`/`ex129` are reserved by FD2's plan (`docs/superpowers/plans/2026-06-25-complex-vector-fit.md`). Before starting this task, check whether `examples/ex112_complex_vector_fit.cpp` already exists - if FD2 shipped first, use `ex113`/`ex130` as written below; if this task runs before FD2's Task 2, use `ex112`/`ex129` instead and let FD2 claim `ex113`/`ex130` when it ships. Whichever runs second must check the other's actual file existence, not just this plan's text.

- [ ] **Step 1: Add the Python binding to `bindings/advanced_bindings.cpp`**

Insert after line 397 (the `suggest_order` binding's closing `)doc");`), before `#endif  // CTRL_HAS_SUBSPACE`:

```cpp

    py::enum_<ctrl::SubspaceMethod>(m, "SubspaceMethod",
        "Weighting variant for subspace_id(). MOESP reproduces n4sid() exactly.")
        .value("MOESP", ctrl::SubspaceMethod::MOESP)
        .value("N4SID", ctrl::SubspaceMethod::N4SID)
        .value("CVA",   ctrl::SubspaceMethod::CVA);

    m.def("subspace_id",
          &ctrl::subspaceID,
          py::arg("Y"), py::arg("U"),
          py::arg("n_order"), py::arg("i_horizon"), py::arg("Ts"),
          py::arg("method") = ctrl::SubspaceMethod::MOESP,
          py::arg("svd_tol") = -1.0,
          R"doc(
Batch subspace state-space identification with a choice of weighting (MOESP/N4SID/CVA).

method=MOESP reproduces n4sid() exactly. N4SID right-weights the past data block by its
Uf-conditioned covariance. CVA additionally weights each output channel by an estimated
noise scale -- helps when output channels have very different noise levels.

Parameters
----------
Y         : Output data (p x N) - rows=outputs, cols=time.
U         : Input data  (m x N) - rows=inputs,  cols=time.
n_order   : Desired model order.  Choose by inspecting result.singular_values.
i_horizon : Block-row count (recommend >= 2*n_order/p, minimum n_order+1).
Ts        : Sample time [s].
method    : ctrl.SubspaceMethod.MOESP / .N4SID / .CVA (default MOESP).
svd_tol   : SVD truncation floor (pass -1 to disable).

Returns SubspaceIDResult. Check result.success before using result.get_model().

Example
-------
>>> res = ctrl.subspace_id(Y, U, n_order=2, i_horizon=10, Ts=0.01, method=ctrl.SubspaceMethod.CVA)
)doc");
```

- [ ] **Step 2: Extend the SubspaceID smoke-test section in `bindings/smoke_test.py`**

Replace the existing `if feats.get('subspace', False):` block's body (the one ending in `print('SubspaceID smoke tests passed.')`) by appending these lines just before that print statement (keep the existing `n4sid`/`suggest_order` assertions as-is, add below them, still inside the same `if` block):

```python
    assert hasattr(ctrl, 'subspace_id'), "subspace_id not bound"
    assert hasattr(ctrl, 'SubspaceMethod'), "SubspaceMethod not bound"
    for method in (ctrl.SubspaceMethod.MOESP, ctrl.SubspaceMethod.N4SID, ctrl.SubspaceMethod.CVA):
        res_v = ctrl.subspace_id(Y_mat, U_mat, n_order=1, i_horizon=10, Ts=0.01, method=method)
        assert res_v.success, f"subspace_id({method}) failed: {res_v.message}"
    moesp_res = ctrl.subspace_id(Y_mat, U_mat, n_order=1, i_horizon=10, Ts=0.01, method=ctrl.SubspaceMethod.MOESP)
    assert np.allclose(moesp_res.model.A, res.get_model().A), "subspace_id(MOESP) should match n4sid()"
```

- [ ] **Step 3: Rebuild and run the smoke test**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTRL_BUILD_PYTHON_BINDINGS=ON` then `cmake --build build --target ctrl_toolbox`
Run: `conda run -n soft_robotics -- python bindings/smoke_test.py`
Expected: no exceptions; output includes `SubspaceID smoke tests passed.`

- [ ] **Step 4: Write `examples/ex113_subspace_id_variants.cpp`**

```cpp
/**
 * @file ex113_subspace_id_variants.cpp
 * @brief Phase 3 (SI3): MOESP/N4SID/CVA subspace-ID weighting variants.
 *
 * Identifies a known 2-output system from noisy I/O data with all three SubspaceMethod
 * variants, demonstrating CVA's advantage when output channels have mismatched noise scales.
 */

#include "ControllerToolbox.h"
#include <iostream>
#include <random>

#if !defined(CTRL_HAS_SUBSPACE)
int main() { std::puts("Skipped: CTRL_HAS_SUBSPACE not enabled."); return 0; }
#else

int main()
{
    Eigen::Matrix2d A_true;
    A_true << 0.9, 0.1, -0.05, 0.85;
    Eigen::MatrixXd B_true(2, 1); B_true << 0.5, 0.2;
    Eigen::MatrixXd C_true(2, 2); C_true << 1.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd D_true = Eigen::MatrixXd::Zero(2, 1);
    const double Ts = 0.1;

    const int N = 2000;
    std::mt19937 rng_u(42);
    std::normal_distribution<double> u_dist(0.0, 1.0);
    Eigen::MatrixXd U(1, N);
    for (int k = 0; k < N; ++k) U(0, k) = u_dist(rng_u);

    // Channel 1 has 60x the measurement noise std of channel 0.
    Eigen::VectorXd noiseStd(2); noiseStd << 0.005, 0.3;
    Eigen::MatrixXd Y(2, N);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    std::mt19937 rng_n(7);
    std::normal_distribution<double> noise(0.0, 1.0);
    for (int k = 0; k < N; ++k)
    {
        const Eigen::VectorXd u_k = U.col(k);
        Eigen::VectorXd y_k = C_true * x + D_true * u_k;
        for (int j = 0; j < 2; ++j) y_k(j) += noise(rng_n) * noiseStd(j);
        Y.col(k) = y_k;
        x = A_true * x + B_true * u_k;
    }

    std::cout << "=== Subspace ID method variants (Phase 3 SI3) ===\n"
              << "Mismatched output noise: channel 0 std=0.005, channel 1 std=0.3\n\n";

    const auto freqs = std::vector<double>{0.5, 1.0, 2.0, 4.0, 8.0, 12.0, 16.0, 20.0};
    const auto resp_true = ctrl::SystemAnalysis::getFrequencyResponse(
        ctrl::StateSpace(A_true, B_true, C_true, D_true, Ts), freqs);

    bool ok = true;
    double err_moesp = 0.0, err_cva = 0.0;
    for (auto [name, method] : std::initializer_list<std::pair<const char *, ctrl::SubspaceMethod>>{
             {"MOESP", ctrl::SubspaceMethod::MOESP},
             {"N4SID", ctrl::SubspaceMethod::N4SID},
             {"CVA",   ctrl::SubspaceMethod::CVA}})
    {
        const auto res = ctrl::subspaceID(Y, U, 2, 10, Ts, method);
        if (!res.success)
        {
            std::cerr << name << " failed: " << res.message << "\n";
            ok = false;
            continue;
        }
        const auto resp_est = ctrl::SystemAnalysis::getFrequencyResponse(res.model.value(), freqs);
        double err = 0.0;
        for (std::size_t k = 0; k < freqs.size(); ++k)
            err += std::abs(std::abs(resp_est[k]) - std::abs(resp_true[k]));
        err /= static_cast<double>(freqs.size());
        std::cout << name << ": high-noise-channel-weighted freq-response error = " << err << "\n";
        if (std::string(name) == "MOESP") err_moesp = err;
        if (std::string(name) == "CVA")   err_cva   = err;
    }

    ok = ok && (err_cva < err_moesp);
    std::cout << "\n" << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
#endif
```

- [ ] **Step 5: Wire the example into `examples/CMakeLists.txt`, `compile.bat`, `compile.sh`**

Append `add_example(ex113_subspace_id_variants)` to `examples/CMakeLists.txt` (after whichever of `ex111_narmax`/`ex112_complex_vector_fit` is currently last - check the file's tail first, per the Numbering Note above), and append `ex113_subspace_id_variants` to the target lists in `compile.bat` and `compile.sh` at the matching position.

- [ ] **Step 6: Build and run the example**

Run: `cmake --build build --target ex113_subspace_id_variants`
Run: `build/examples/ex113_subspace_id_variants.exe` (path may be `build/examples/Release/...` depending on generator)
Expected: prints the 3 methods' errors, then `PASS`.

- [ ] **Step 7: Write `examples/python/ex130_subspace_id_variants.py`**

```python
"""
ex130_subspace_id_variants.py

Phase 3 (SI3): MOESP/N4SID/CVA subspace-ID weighting variants.

Mirrors ex113_subspace_id_variants.cpp -- identifies a known 2-output system with
mismatched per-channel measurement noise, demonstrating CVA's advantage on the
high-noise channel.
"""

import sys
import _setup_bindings  # noqa: F401
import numpy as np

try:
    import ctrl_toolbox as ctrl
    if not hasattr(ctrl, 'subspace_id'):
        raise AttributeError("subspace_id not found - rebuild ctrl_toolbox binding")
except (ImportError, AttributeError) as _e:
    print(f"SKIP: {_e}")
    sys.exit(0)

A_true = np.array([[0.9, 0.1], [-0.05, 0.85]])
B_true = np.array([[0.5], [0.2]])
C_true = np.array([[1.0, 0.0], [0.0, 1.0]])
D_true = np.zeros((2, 1))
Ts = 0.1
N = 2000

rng_u = np.random.default_rng(42)
U = rng_u.normal(0.0, 1.0, size=(1, N))

noise_std = np.array([0.005, 0.3])  # channel 1 has 60x channel 0's noise
rng_n = np.random.default_rng(7)
x = np.zeros(2)
Y = np.zeros((2, N))
for k in range(N):
    u_k = U[:, k]
    y_k = C_true @ x + D_true @ u_k
    y_k = y_k + rng_n.normal(0.0, 1.0, size=2) * noise_std
    Y[:, k] = y_k
    x = A_true @ x + B_true @ u_k

freqs = [0.5, 1.0, 2.0, 4.0, 8.0, 12.0, 16.0, 20.0]
sys_true = ctrl.StateSpace(A_true, B_true, C_true, D_true, Ts)
resp_true = np.array(ctrl.SystemAnalysis.get_frequency_response(sys_true, freqs))

print("=== Subspace ID method variants (Phase 3 SI3) ===")
print("Mismatched output noise: channel 0 std=0.005, channel 1 std=0.3\n")

errs = {}
for name, method in [("MOESP", ctrl.SubspaceMethod.MOESP),
                      ("N4SID", ctrl.SubspaceMethod.N4SID),
                      ("CVA", ctrl.SubspaceMethod.CVA)]:
    res = ctrl.subspace_id(Y, U, n_order=2, i_horizon=10, Ts=Ts, method=method)
    if not res.success:
        print(f"{name} failed: {res.message}")
        errs[name] = float("inf")
        continue
    model = res.get_model()
    resp_est = np.array(ctrl.SystemAnalysis.get_frequency_response(model, freqs))
    err = float(np.mean(np.abs(np.abs(resp_est) - np.abs(resp_true))))
    errs[name] = err
    print(f"{name}: high-noise-channel-weighted freq-response error = {err:.5f}")

ok = errs["CVA"] < errs["MOESP"]
print("\n[PASS] All checks passed." if ok else "\n[FAIL] One or more checks failed.")
sys.exit(0 if ok else 1)
```

- [ ] **Step 8: Run the Python example**

Run: `conda run -n soft_robotics -- python examples/python/ex130_subspace_id_variants.py`
Expected: prints the same style of output as the C++ example, ending in `[PASS] All checks passed.`

- [ ] **Step 9: Update `docs/algorithm_backlog.md` status**

Insert a new row after line 87 (the NARMAX row in the "Already done" table):
```markdown
| MOESP / CVA (subspace ID variants) | `lib/SubspaceID.h`/`.cpp` (`SubspaceMethod` enum + `subspaceID()`; `n4sid()` becomes a delegating one-liner at `SubspaceMethod::MOESP`) - Phase 3 SI3, `examples/ex113_subspace_id_variants.cpp`. See [2026-06-25-subspace-id-variants-design.md](superpowers/specs/2026-06-25-subspace-id-variants-design.md). |
```

Replace lines 89-91 (the "Shipped" summary) with:
```markdown
**Shipped:** `ALGORITHM_ROADMAP_PHASE3.md` Phase 3 partial (5 designs: ML1, ML2, NC3, SI4, SI3),
see `docs/cumulative_bug_report.md` Part 69 and
[2026-06-25-subspace-id-variants-design.md](superpowers/specs/2026-06-25-subspace-id-variants-design.md).
FD2 (complex-pole Vector Fitting) and ML3 (GP-MPC) remain open (or are tracked separately, if
they have already shipped by the time you read this - check the status table at the top of
`ALGORITHM_ROADMAP_PHASE3.md`).
```

Replace line 127 (the MOESP/CVA "What's left" row in the System Identification section) - remove the row entirely and change the section's lead-in sentence:
```markdown
Correlation-based identification, Hammerstein-Wiener models, Maximum Likelihood / MAP
identification, NARMAX, and MOESP/CVA subspace ID variants are **done** - see the "Already
done" table above (`lib/CorrelationID.h`, `lib/HammersteinWienerIdentifier.h`,
`lib/MLEIdentifier.h`, `lib/NARMAXIdentifier.h`, `lib/SubspaceID.h`). Nothing left in this
category.
```//
(delete the now-empty `| Item | Notes |` table that followed it)

- [ ] **Step 10: Update `docs/ALGORITHM_ROADMAP_PHASE3.md` status**

Replace lines 4-5 (top status line) - exact wording depends on whether FD2 has already shipped by the time this runs; if FD2 is still open:
```markdown
**Status:** Planning - 21 of 32 items shipped (Phase 1 and Phase 2 complete; Phase 3 partial:
ML1/ML2/NC3/SI4/SI3 done, FD2/ML3 open).
```

Replace line 50 (the SI3 status-table row):
```markdown
| SI3 | MOESP / CVA Subspace ID Variants | 3 | Done |
```

- [ ] **Step 11: Commit**

```bash
git add bindings/advanced_bindings.cpp bindings/smoke_test.py examples/ex113_subspace_id_variants.cpp examples/CMakeLists.txt compile.bat compile.sh examples/python/ex130_subspace_id_variants.py docs/algorithm_backlog.md docs/ALGORITHM_ROADMAP_PHASE3.md
git commit -m "Wire SubspaceMethod variants into Python bindings, examples, and roadmap docs"
```
