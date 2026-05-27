#pragma once
#include "PlantModel.h"
#include <Eigen/Dense>
#include <vector>
#include <complex>

/**
 * @file SystemAnalysis.h
 * @brief Frequency-domain and stability analysis tools for discrete-time state-space systems.
 */

namespace ctrl
{

/**
 * @brief Stability margins for a SISO open-loop plant.
 *
 * A value of infinity indicates the corresponding frequency crossing was not found on the
 * evaluation grid — a system with very high margins or a critically damped plant may
 * legitimately have no phase or gain crossover.
 */
struct StabilityMargins
{
    double gainMarginDb;    ///< Gain margin GM [dB]: how much loop gain can increase before instability.
    double phaseMarginDeg;  ///< Phase margin PM [°]: phase above −180° at the gain-crossover frequency.
    double wCrossoverGain;  ///< Gain-crossover frequency ωc [rad/s] where |G(jωc)| = 1 (0 dB).
    double wCrossoverPhase; ///< Phase-crossover frequency ωp [rad/s] where ∠G(jωp) = −180°.
};

/**
 * @brief Static analysis utilities for discrete-time state-space systems.
 */
class SystemAnalysis
{
public:
    /**
     * @brief Compute the poles (eigenvalues of A) of a state-space system.
     *
     * For discrete-time systems, stability requires all |poles| < 1.
     *
     * @param sys Discrete-time state-space model.
     * @return Vector of poles (eigenvalues of A).
     */
    static std::vector<std::complex<double>> getPoles(const StateSpace &sys);

    /**
     * @brief Test discrete-time stability.
     *
     * Returns @c true iff all eigenvalues of A satisfy |λ| < 1 (strict stability).
     * Marginally stable systems (|λ| = 1) return @c false.
     *
     * @param sys Discrete-time state-space model.
     * @return @c true if strictly stable.
     */
    static bool isDiscreteStable(const StateSpace &sys);

    /**
     * @brief Solve the discrete Lyapunov equation A·P·Aᵀ − P + Q = 0.
     *
     * Solved via Kronecker product vectorisation: (I − A⊗A)·vec(P) = vec(Q).
     *
     * @par Complexity note
     * O(n⁶) due to the n²×n² linear system — suitable for n ≤ 10. For n > 10, the
     * Bartels-Stewart algorithm (O(n³) via Schur decomposition) is strongly preferred;
     * MATLAB's dlyap() uses Bartels-Stewart internally.
     *
     * @param A System matrix (must be strictly stable; unstable A gives a singular system).
     * @param Q Symmetric positive semi-definite matrix (same dimensions as A).
     * @return Solution P.
     *
     * @see Golub, Nash & Van Loan, "A Hessenberg-Schur method for AX+XB=C", IEEE TAC (1979).
     */
    static Eigen::MatrixXd solveDiscreteLyapunov(const Eigen::MatrixXd &A,
                                                  const Eigen::MatrixXd &Q);

    /**
     * @brief Compute the frequency response G(e^{jωTs}) at each frequency in @p freqs.
     *
     * Uses the direct formula G(z) = C·(z·I − A)⁻¹·B + D at z = e^{jωTs}.
     * SISO only; throws for MIMO plants.
     *
     * @param sys   SISO discrete-time state-space model.
     * @param freqs Frequency vector [rad/s].
     * @return Complex frequency response at each frequency (same length as freqs).
     * @throws std::invalid_argument If the system is not SISO.
     */
    static std::vector<std::complex<double>> getFrequencyResponse(const StateSpace &sys,
                                                                   const std::vector<double> &freqs);

    /**
     * @brief Compute gain and phase margins for a SISO open-loop plant.
     *
     * Algorithm: coarse logarithmic grid (200 points) to bracket crossings, followed by
     * bisection (50 iterations) for accuracy. Phase is continuously unwrapped across grid
     * points, so the result is correct for higher-order and non-minimum-phase plants where
     * phase can cross −180° multiple times. Returns the worst-case (smallest) margin when
     * multiple crossings exist.
     *
     * @param sys SISO discrete-time state-space model.
     * @return StabilityMargins with GM, PM, and crossover frequencies.
     */
    static StabilityMargins calculateMargins(const StateSpace &sys);

    /**
     * @brief Compute the peak H∞ norm of a state-space model.
     *
     * For SISO plants, equals the peak frequency-response magnitude.
     * For MIMO plants, equals the peak induced L₂-gain (maximum singular value over all frequencies).
     *
     * Uses a coarse grid followed by golden-section search; typical accuracy is better than 0.1%.
     *
     * @param sys Discrete-time state-space model.
     * @return Peak H∞ norm.
     */
    static double calculateHInfinityNorm(const StateSpace &sys);
};

} // namespace ctrl
