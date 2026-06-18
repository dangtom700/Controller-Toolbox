#pragma once
#include <Eigen/Dense>
#include <complex>
#include <vector>
#include "PlantModel.h"

/**
 * @file ZeroPhaseTrackingFilter.h
 * @brief Zero-phase error tracking control (ZPETC) feedforward filter design.
 *
 * ZPETC (Tomizuka 1987) designs a causal prefilter G_ff(z) such that the composite
 * transfer function G(z).G_ff(z) has:
 *  - **Zero phase error** for the minimum-phase zeros of G(z) (inverted stably).
 *  - **Unit DC gain** by normalising the non-minimum-phase (NMP) contribution.
 *  - **Bounded frequency response** - no unstable pole-zero cancellations.
 *
 * For a plant G(z) = B(z)/A(z) with factorisation B(z) = B^+(z).B^-(z):
 * @code
 *   G_ff(z) = K . A(z) / (B^+(z) . z^{n-m})
 *   K = 1 / B^-(1)    (DC normalisation; B^-(1) = \prod (1-z_i) for NMP zeros z_i)
 * @endcode
 *
 * Composite response: G.G_ff = B^-(z)/B^-(1) (unit DC; NMP amplitude error only).
 *
 * @par When to use
 *  - **Minimum-phase plant (all |z_i| < 1):** G_ff gives exact causal inversion.
 *    G.G_ff \equiv 1 at all frequencies. No preview needed.
 *  - **Non-minimum-phase plant (some |z_i| >= 1):** G_ff cancels B^+ stably and
 *    normalises DC. The remaining B^-(z) factor contributes amplitude error away
 *    from DC but introduces NO phase lag. Compare with trying to invert B^- directly:
 *    that produces unstable poles.
 *
 * @par Limitations
 *  - Prefilter assumes reference trajectory r[k] is available (feedforward only).
 *    Combine with a feedback controller for disturbance rejection.
 *  - For plants with no zeros (all-pole): G_ff reduces to A(z)/1, which differentiates
 *    the reference. Add the returned `filter` in cascade with a reference pre-smoother.
 *  - Requires SISO plant (throws for MIMO).
 *
 * @par Usage
 * @code
 *   ctrl::StateSpace plant = ...;   // SISO discrete-time
 *   ctrl::ZPETCResult res = ctrl::designZPETC(plant);
 *
 *   if (res.hasNMPZeros)
 *       std::cout << "NMP zeros present; DC amplitude error: " << res.dcAmplitudeError << "\n";
 *
 *   // Use res.filter as a feedforward prefilter on the reference:
 *   Eigen::VectorXd x_ff = Eigen::VectorXd::Zero(res.filter.stateSize());
 *   Eigen::VectorXd r_vec(1); r_vec(0) = r[k];
 *   double u_ff = ctrl::ssStep(res.filter, x_ff, r_vec)(0);
 *   double u    = u_feedback + u_ff;
 * @endcode
 *
 * @see Tomizuka, "Zero Phase Error Tracking Algorithm for Digital Control", ASME JDSMC (1987).
 * @see Franklin, Powell & Emami-Naeini, "Feedback Control of Dynamic Systems", Ch. 8.
 */

namespace ctrl
{

/** @brief Result of a ZPETC prefilter design. */
struct ZPETCResult
{
    StateSpace              filter;           ///< Causal ZPETC prefilter G_ff(z) as a StateSpace.
    double                  dcAmplitudeError; ///< Max |||G.G_ff|(omega) - 1|| over [0, Nyquist]. 0 for min-phase.
    bool                    hasNMPZeros;      ///< True if any transmission zero satisfies |z| >= 1.
    std::vector<std::complex<double>> zeros;    ///< All transmission zeros of the plant.
    std::vector<std::complex<double>> nmpZeros; ///< Non-minimum-phase zeros only (|z| >= 1).
};

/**
 * @brief Compute the transmission zeros of a SISO discrete-time system.
 *
 * Uses the generalised eigenvalue problem for the system matrix pencil:
 * @code
 *   det([ A - lambdaI   B ] ) = 0
 *       ([   C     D ] )
 * @endcode
 *
 * @param sys SISO discrete-time state-space model.
 * @return Complex vector of transmission zeros (may include zeros at infinity - these are excluded).
 * @throws std::invalid_argument If the system is not SISO.
 */
std::vector<std::complex<double>> transmissionZeros(const StateSpace &sys);

/**
 * @brief Design a ZPETC prefilter for a SISO discrete-time plant.
 *
 * @param plant SISO discrete-time state-space model.
 * @return ZPETCResult with the causal prefilter and diagnostic information.
 * @throws std::invalid_argument If the plant is not SISO.
 */
ZPETCResult designZPETC(const StateSpace &plant);

} // namespace ctrl
