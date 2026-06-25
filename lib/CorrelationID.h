#pragma once
#include "Features.h"
#include <Eigen/Dense>

/**
 * @file CorrelationID.h
 * @brief Cross-correlation impulse-response identification (Phase 3 SI2).
 *
 * Classical non-parametric impulse-response estimation: drive the plant with a
 * (near-white) test input and recover g_hat(k) = R_uy(k) / R_uu(0). The standard
 * first step in a classical system-ID workflow, before committing to a parametric
 * ARX/state-space structure.
 *
 * @see docs/superpowers/specs/2026-06-24-small-foundational-utilities-design.md
 */

namespace ctrl
{

/**
 * @brief Parameters for @ref CorrelationID::identify.
 */
struct CorrelationIDParams
{
    int  max_lag      = 50;    ///< Impulse-response length to estimate (lags 0..max_lag).
    bool whiten_input = false; ///< Apply a first-order AR pre-whitening pass to u and y before
                                 ///< correlating (reduces bias when u isn't already near-white).
};

/**
 * @brief Result of @ref CorrelationID::identify.
 */
struct CorrelationIDResult
{
    Eigen::VectorXd impulse_response; ///< g_hat(0..max_lag).
    Eigen::VectorXd autocorr_u;       ///< R_uu(0..max_lag).
    Eigen::VectorXd crosscorr_uy;     ///< R_uy(0..max_lag).
};

/**
 * @brief Cross-correlation impulse-response identification.
 */
class CorrelationID
{
public:
    /**
     * @brief Estimate the impulse response of a SISO LTI system from (u, y) data.
     *
     * @param u      Input samples u[0..N-1].
     * @param y      Output samples y[0..N-1] (same length as @p u).
     * @param Ts     Sample time [s] (recorded for caller convenience; not used in the
     *               correlation sums themselves).
     * @param params Estimation parameters.
     * @return CorrelationIDResult with the estimated impulse response and the underlying
     *         correlation sequences.
     * @throws std::invalid_argument If @p u and @p y differ in length, or if
     *         `params.max_lag >= u.size()`.
     */
    static CorrelationIDResult identify(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                                         double Ts, const CorrelationIDParams &params = {});

    /**
     * @brief Generate a maximal-length pseudo-random binary sequence (PRBS).
     *
     * Fibonacci LFSR with a standard primitive-polynomial tap set, period
     * `2^n_bits - 1`, values in {-1, +1} - the standard near-white test input for
     * @ref identify. If @p length exceeds the period, the sequence repeats.
     *
     * @param length Number of samples to generate.
     * @param n_bits LFSR register width (supported range: 2..20).
     * @param seed   Initial register state (forced to 1 if it would otherwise be 0).
     * @return PRBS sequence of length @p length, values in {-1, +1}.
     * @throws std::invalid_argument If `n_bits` is outside [2, 20] or `length <= 0`.
     */
    static Eigen::VectorXd generatePRBS(int length, int n_bits, unsigned seed = 42);
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(correlation_id)
