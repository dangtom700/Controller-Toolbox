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
 * evaluation grid - a system with very high margins or a critically damped plant may
 * legitimately have no phase or gain crossover.
 */
struct StabilityMargins
{
    double gainMarginDb;    ///< Gain margin GM [dB]: how much loop gain can increase before instability.
    double phaseMarginDeg;  ///< Phase margin PM [^\circ]: phase above -180^\circ at the gain-crossover frequency.
    double wCrossoverGain;  ///< Gain-crossover frequency omegac [rad/s] where |G(jomegac)| = 1 (0 dB).
    double wCrossoverPhase; ///< Phase-crossover frequency omegap [rad/s] where \angleG(jomegap) = -180^\circ.
};

/**
 * @brief Simultaneous gain/phase margin from a disk-shaped uncertainty region (Seiler et al. 2020).
 *
 * Unlike @ref StabilityMargins (which finds independent worst-case gain and phase crossings),
 * the disk margin gives a single number @c alpha describing the largest simultaneous gain AND
 * phase perturbation the loop tolerates without instability - valid for SISO and MIMO loops.
 */
struct DiskMargin
{
    double alpha;            ///< Disk radius alpha = 1 / ||S||_inf; alpha >= 1 is a "good" loop.
    double gain_margin;      ///< Simultaneous gain margin (linear, NOT dB): (1+alpha)/(1-alpha).
    double phase_margin_deg; ///< Simultaneous phase margin [deg]: 2*asin(alpha/2).
};

/**
 * @brief The "Gang of Four" closed-loop transfer matrices for a unity-feedback loop.
 *
 * For plant @c G and controller @c K closing a negative-feedback loop (e = r - y, u = K*e,
 * y = G*u), the four matrices fully characterise closed-loop behaviour:
 *  - @c S  (sensitivity)            attenuates @c r and output disturbances.
 *  - @c T  (complementary sens.)    = I - S; reference-tracking transfer.
 *  - @c GS (load-disturbance sens.) response at @c y to a disturbance injected at the plant input.
 *  - @c KS (control/noise sens.)    response at @c u to noise injected at the plant output.
 */
struct GangOfFour
{
    StateSpace S;  ///< Sensitivity:            (I + G*K)^-1.
    StateSpace T;  ///< Complementary sens.:    G*K*(I + G*K)^-1  (= I - S).
    StateSpace GS; ///< Load-disturbance sens.: S*G (disturbance at plant input -> output).
    StateSpace KS; ///< Control/noise sens.:    K*S (noise at plant output -> control signal).
};

/**
 * @brief Peak (Hinf) norms of the four Gang-of-Four transfer matrices.
 */
struct GangOfFourNorms
{
    double norm_S;  ///< ||S||_inf  - smaller is better tracking/disturbance rejection.
    double norm_T;  ///< ||T||_inf  - large values indicate poor robustness (resonant peaking).
    double norm_GS; ///< ||GS||_inf - peak load-disturbance response.
    double norm_KS; ///< ||KS||_inf - peak control effort from sensor noise.
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
     * Returns @c true iff all eigenvalues of A satisfy |lambda| < 1 (strict stability).
     * Marginally stable systems (|lambda| = 1) return @c false.
     *
     * @param sys Discrete-time state-space model.
     * @return @c true if strictly stable.
     */
    static bool isDiscreteStable(const StateSpace &sys);

    /**
     * @brief Solve the discrete Lyapunov equation A.P.Aᵀ - P + Q = 0.
     *
     * Solved via Kronecker product vectorisation: (I - A\otimesA).vec(P) = vec(Q).
     *
     * @par Complexity note
     * O(n⁶) due to the n^2*n^2 linear system - suitable for n <= 10. For n > 10, the
     * Bartels-Stewart algorithm (O(n^3) via Schur decomposition) is strongly preferred;
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
     * @brief Compute the frequency response G(e^{jomegaTs}) at each frequency in @p freqs.
     *
     * Uses the direct formula G(z) = C.(z.I - A)^-^1.B + D at z = e^{jomegaTs}.
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
     * phase can cross -180^\circ multiple times. Returns the worst-case (smallest) margin when
     * multiple crossings exist.
     *
     * @param sys SISO discrete-time state-space model.
     * @return StabilityMargins with GM, PM, and crossover frequencies.
     */
    static StabilityMargins calculateMargins(const StateSpace &sys);

    /**
     * @brief Compute the peak Hinf norm of a state-space model.
     *
     * For SISO plants, equals the peak frequency-response magnitude.
     * For MIMO plants, equals the peak induced L2-gain (maximum singular value over all frequencies).
     *
     * Uses a coarse grid followed by golden-section search; typical accuracy is better than 0.1%.
     *
     * @param sys Discrete-time state-space model.
     * @return Peak Hinf norm.
     */
    static double calculateHInfinityNorm(const StateSpace &sys);

    /**
     * @brief Series (cascade) connection: output of @p G1 feeds the input of @p G2.
     *
     * Represents the matrix product `G2*G1` applied to a signal (apply @p G1 first).
     * Requires `G1.outputSize() == G2.inputSize()` and equal sample times.
     *
     * @param G1 Upstream system.
     * @param G2 Downstream system.
     * @return Combined state-space with state `[x1; x2]` (G1's states first).
     * @throws std::invalid_argument On dimension or sample-time mismatch.
     */
    static StateSpace series(const StateSpace &G1, const StateSpace &G2);

    /**
     * @brief Parallel connection: `sys_out = G1 + G2` (same input feeds both, outputs summed).
     *
     * Requires matching input/output sizes and equal sample times.
     *
     * @throws std::invalid_argument On dimension or sample-time mismatch.
     */
    static StateSpace parallel(const StateSpace &G1, const StateSpace &G2);

    /**
     * @brief Close a unity-negative-feedback loop around a single forward-path system.
     *
     * Given the open-loop (forward-path) system @p GK mapping error `e` to output `y`,
     * returns the complementary sensitivity `T = GK*(I+GK)^-1` (closed loop from
     * reference `r` to output `y`, with `e = r - y`). Requires @p GK to be square
     * (`inputSize() == outputSize()`).
     *
     * @note `S = I - T` shares the same `(A, B)` realisation as `T` (`C_S = -C_T`,
     *       `D_S = I - D_T`, by the push-through identity) - see @ref gangOfFour.
     * @param GK Forward-path (open-loop) system, square.
     * @return Closed-loop system `T` (reference to output).
     * @throws std::invalid_argument If @p GK is not square.
     */
    static StateSpace feedback(const StateSpace &GK);

    /**
     * @brief Build the Gang of Four for a unity-feedback loop with plant @p G and controller @p K.
     *
     * Loop convention: `e = r - y`, `u = K*e`, `y = G*u` (same as the rest of the codebase,
     * see CLAUDE.md sign-convention table). Requires `K.outputSize() == G.inputSize()` and
     * `G.outputSize() == K.inputSize()`.
     *
     * @throws std::invalid_argument On dimension or sample-time mismatch.
     */
    static GangOfFour gangOfFour(const StateSpace &G, const StateSpace &K);

    /**
     * @brief Peak Hinf norms of all four Gang-of-Four matrices.
     */
    static GangOfFourNorms gangOfFourNorms(const GangOfFour &g4);

    /**
     * @brief Simultaneous gain/phase disk margin of an open-loop transfer @p open_loop_L.
     *
     * `alpha = 1/||S||_inf` where `S = (I+L)^-1`; converts to an equivalent simultaneous
     * gain margin `(1+alpha)/(1-alpha)` and phase margin `2*asin(alpha/2)` [deg].
     * Valid for SISO and square MIMO loops.
     *
     * @see Seiler, Packard & Gahinet, "An Introduction to Disk Margins", IEEE Control
     *      Systems Magazine (2020).
     * @throws std::invalid_argument If @p open_loop_L is not square.
     */
    static DiskMargin calculateDiskMargin(const StateSpace &open_loop_L);
};

} // namespace ctrl
