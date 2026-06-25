#pragma once
#include "FreqDomainIdentifier.h"
#include "Features.h"
#include <vector>
#include <complex>

/**
 * @file SKFit.h
 * @brief Sanathanan-Koerner-reweighted complex-response fitting (Phase 3 FD1).
 *
 * Generalizes `FreqDomainIdentifier::fitLevy`'s one-shot linear least-squares fit into an
 * iteratively-reweighted (Sanathanan-Koerner, 1963) fit against the full complex frequency
 * response, removing most of Levy's high-frequency bias. Reuses
 * `FreqDomainIdentifier::buildLevySystem`'s linear-system-build step (extracted from
 * `fitLevy` for exactly this purpose) inside an outer reweighting loop, rather than
 * `VectorFitting`'s real-pole/magnitude-only SK machinery (`VectorFitting::buildSKSystem`
 * is private and not applicable to a full complex-response, general num/den fit).
 *
 * @see FreqDomainIdentifier.h for the unweighted (iteration-0-equivalent) Levy fit.
 * @see docs/superpowers/specs/2026-06-24-small-foundational-utilities-design.md
 */

namespace ctrl {

/**
 * @brief Result of @ref SKFit::fitSK.
 */
struct SKFitResult
{
    TransferFunction     model;     ///< Fitted discrete-time model after the final iteration.
    std::vector<double>  iterCost;  ///< RMSE per SK iteration (index 0 = unweighted baseline).
    bool                 converged = false; ///< True if coefficient displacement dropped below tol.
};

/**
 * @brief Iteratively-reweighted (Sanathanan-Koerner) complex-response fitting.
 */
class SKFit
{
public:
    /**
     * @brief Fit a num_order/den_order TransferFunction via SK-reweighted least squares.
     *
     * Iteration 0 is an unweighted Levy fit (identical to @ref FreqDomainIdentifier::fitLevy).
     * Each subsequent iteration reweights every sample by `1 / |D_prev(zinv_k)|`, where
     * `D_prev` is the previous iteration's fitted denominator - the classical SK reweighting,
     * which minimizes the residual `|N_k - H_k*D_k|^2 / |D_prev,k|^2` once squared inside the
     * least-squares solve.
     *
     * @param freqs     Frequency vector [rad/s].
     * @param response  Complex frequency response at each frequency in @p freqs.
     * @param num_order Numerator order m.
     * @param den_order Denominator order n.
     * @param Ts        Sample time [s].
     * @param max_iter  Maximum SK iterations.
     * @param tol       Stop when the coefficient vector's max absolute change drops below this.
     * @return SKFitResult with the final fitted model, the per-iteration RMSE history, and a
     *         convergence flag.
     * @throws std::invalid_argument If @p freqs/@p response have mismatched lengths, or if the
     *         system is underdetermined (same checks as `fitLevy`).
     */
    static SKFitResult fitSK(const std::vector<double> &freqs,
                              const std::vector<std::complex<double>> &response,
                              int num_order, int den_order, double Ts,
                              int max_iter = 20, double tol = 1e-4);
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(sk_fit)
