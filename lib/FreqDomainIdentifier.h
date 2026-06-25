#pragma once
#include "PlantModel.h"
#include "Features.h"
#include <vector>
#include <complex>

/**
 * @file FreqDomainIdentifier.h
 * @brief Frequency-domain system identification via Levy's method.
 *
 * Fits a SISO discrete-time @ref ctrl::TransferFunction directly to a set of
 * complex frequency-response samples `(freqs, response)` - the inverse direction
 * of `SystemAnalysis::getFrequencyResponse`.
 *
 * **Algorithm (Levy 1959):** the target model is `H(z^-1) = N(z^-1)/D(z^-1)` with
 * `D`'s constant term fixed to 1 (matching this codebase's `TransferFunction`
 * convention). Minimising `||H_data - N/D||^2` directly is nonlinear in the
 * unknown coefficients; Levy's method linearises by instead minimising the
 * equation residual `N(z_k^-1) - H_data,k * D(z_k^-1)` at each sample frequency,
 * which is linear. Stacking the real and imaginary parts of this residual across
 * all sample frequencies gives a real linear least-squares system, solved once
 * via `Eigen::ColPivHouseholderQR`.
 *
 * This is the classical single-shot fit behind MATLAB's `invfreqz`. It does not
 * iteratively reweight the residual (see Sanathanan-Koerner iteration in
 * `VectorFitting.h`, which fits only a real magnitude profile to real poles -
 * generalising it to a full complex-response fit is a deferred follow-up, see
 * `docs/algorithm_backlog.md`), so Levy's fit carries a known high-frequency
 * bias relative to SK/Vector-Fitting variants.
 *
 * @see Levy, "Complex curve fitting", IRE Trans. Automatic Control (1959).
 * @see VectorFitting.h for the SK-iteration / real-pole magnitude-fitting sibling.
 */

namespace ctrl {

/**
 * @brief Result of @ref FreqDomainIdentifier::fitLevy.
 */
struct FreqDomainFitResult
{
    TransferFunction tf;             ///< Fitted discrete-time model.
    double           rmse = 0.0;     ///< RMS of |H_data - H_fit| across the sample frequencies.
    bool             full_rank = true; ///< False if the linear system was rank-deficient (still
                                        ///< solved via the least-norm solution, but the fit may
                                        ///< be unreliable).
};

/**
 * @brief Frequency-domain system identification (Levy's method).
 */
class FreqDomainIdentifier
{
public:
    /**
     * @brief Fit a `num_order`/`den_order` TransferFunction to frequency-response data.
     *
     * @param freqs     Frequency vector [rad/s].
     * @param response  Complex frequency response H(e^{j*omega*Ts}) at each frequency in
     *                   @p freqs (same length).
     * @param num_order Numerator order m (N has m+1 coefficients).
     * @param den_order Denominator order n (D has n+1 coefficients; D's constant term is
     *                   fixed to 1).
     * @param Ts        Sample time [s] of the fitted model.
     * @return FreqDomainFitResult with the fitted TransferFunction, fit RMSE, and a rank flag.
     * @throws std::invalid_argument If @p freqs and @p response have different lengths, or if
     *         `freqs.size() < num_order + 1 + den_order` (system underdetermined).
     */
    static FreqDomainFitResult fitLevy(const std::vector<double> &freqs,
                                        const std::vector<std::complex<double>> &response,
                                        int num_order, int den_order, double Ts);

    /**
     * @brief Build the real-stacked Levy linear system Phi*x = y, optionally row-weighted.
     *
     * Extracted from @ref fitLevy so @ref SKFit can reuse the same linear-system-build step
     * inside an outer Sanathanan-Koerner reweighting loop. @p weights defaults to empty
     * (unweighted, i.e. identical to @ref fitLevy's own system) - passing an empty vector
     * reproduces @ref fitLevy's behavior exactly, with zero change for existing callers.
     *
     * @param freqs     Frequency vector [rad/s].
     * @param response  Complex frequency response at each frequency in @p freqs.
     * @param num_order Numerator order m.
     * @param den_order Denominator order n.
     * @param Ts        Sample time [s].
     * @param weights   Per-sample weight (size 0 = unweighted, size `freqs.size()` otherwise);
     *                  each sample's stacked real/imaginary row pair is scaled by its weight.
     * @param[out] Phi  Real-stacked system matrix (2*freqs.size() x (num_order+1+den_order)).
     * @param[out] y    Real-stacked right-hand side (2*freqs.size()).
     * @throws std::invalid_argument If @p freqs/@p response/@p weights have mismatched
     *         lengths (when @p weights is non-empty), or if the system is underdetermined.
     */
    static void buildLevySystem(const std::vector<double> &freqs,
                                 const std::vector<std::complex<double>> &response,
                                 int num_order, int den_order, double Ts,
                                 const std::vector<double> &weights,
                                 Eigen::MatrixXd &Phi, Eigen::VectorXd &y);

    /**
     * @brief RMS of |H_data - N/D| across the sample frequencies for a fitted num/den pair.
     *
     * Shared scoring step used by both @ref fitLevy and @ref SKFit::fitSK.
     */
    static double fitRMSE(const std::vector<double> &freqs,
                           const std::vector<std::complex<double>> &response,
                           const std::vector<double> &num, const std::vector<double> &den,
                           double Ts);
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(freq_domain_identifier)
