#pragma once
#include "PlantModel.h"
#include <complex>
#include <vector>
#include <Eigen/Dense>

/**
 * @file GapMetric.h
 * @brief Nu-gap (Vinnicombe gap) metric for SISO discrete-time linear systems.
 *
 * The nu-gap delta_nu(P1, P2) measures the distance between two plants' closed-loop
 * behaviours on the Riemann sphere of transfer functions. A distance below 0.5
 * means a single robust controller can stabilise both plants simultaneously
 * (Vinnicombe 2001, "Uncertainty and Feedback").
 *
 * **Implementation:** Uses the frequency-domain chordal metric:
 * @code
 *   delta_nu(P1, P2) ~= sup_omega |P1(e^{j*omega*Ts}) - P2(e^{j*omega*Ts})|
 *                       / sqrt((1 + |P1|^2) * (1 + |P2|^2))
 * @endcode
 * over a log-spaced grid from omega_min to pi/Ts. This gives an upper bound on
 * the exact nu-gap (which also requires a winding-number check). The upper bound
 * is sufficient for clustering: if it is below the chosen threshold, a single
 * robust controller works for the entire cluster.
 *
 * @par Reference
 * Vinnicombe, G. (2001). *Uncertainty and Feedback: H-infinity Loop-Shaping and
 * the nu-Gap Metric*. Imperial College Press.
 *
 * @note Only SISO (p=1, m=1) systems are accepted. For MIMO, extend using
 *       the subspace chordal distance (Skogestad & Postlethwaite Ch. 9).
 */

namespace ctrl {

/**
 * @brief Evaluate the discrete-time frequency response P(e^{j*omega*Ts}).
 *
 * Computes P(z) = C * (z*I - A)^{-1} * B + D at z = e^{j*omega*Ts} for each
 * frequency in omega_grid. Uses full-pivot LU for numerical robustness near
 * resonant frequencies.
 *
 * @param sys         Discrete-time StateSpace model (any MIMO dimensions).
 * @param omega_grid  Frequencies [rad/s].
 * @return            Vector of p_out*m_in complex matrices, one per frequency.
 */
std::vector<Eigen::MatrixXcd> freqResponseGrid(const StateSpace& sys,
                                                const std::vector<double>& omega_grid);

/**
 * @brief SISO chordal distance on the Riemann sphere.
 *
 * d(p1, p2) = |p1 - p2| / sqrt((1 + |p1|^2) * (1 + |p2|^2))
 *
 * Range: [0, 1]. Equals 0 iff p1 == p2; approaches 1 for antipodal points
 * (p2 = -1/conj(p1)).
 *
 * @param p1  First complex frequency-response value.
 * @param p2  Second complex frequency-response value.
 * @return    Chordal distance in [0, 1].
 */
double chordalDist(std::complex<double> p1, std::complex<double> p2);

/**
 * @brief Compute the nu-gap upper bound between two SISO discrete-time systems.
 *
 * @param P1          First system (must be SISO: outputSize=1, inputSize=1).
 * @param P2          Second system (same Ts and SISO as P1).
 * @param freq_points Frequency grid size (default 200). More points = tighter bound.
 * @param omega_min   Minimum frequency [rad/s] for the grid (default 0.01).
 * @return            Nu-gap upper bound in [0, 1].
 * @throws std::invalid_argument If systems are not SISO or have different Ts.
 */
double nuGap(const StateSpace& P1, const StateSpace& P2,
             int freq_points = 200, double omega_min = 1e-2);

/**
 * @brief Compute the nu-gap upper bound between two SISO discrete TF models.
 *
 * Converts both to state-space form internally.
 *
 * @param P1  First transfer function.
 * @param P2  Second transfer function (same Ts as P1).
 * @return    Nu-gap upper bound.
 */
double nuGap(const TransferFunction& P1, const TransferFunction& P2,
             int freq_points = 200, double omega_min = 1e-2);

/**
 * @brief Compute the symmetric N*N nu-gap distance matrix.
 *
 * Diagonal entries are 0. Off-diagonal (i,j) is nuGap(models[i], models[j]).
 * All models must be SISO with the same Ts.
 *
 * @param models      Collection of discrete-time SISO state-space models.
 * @param freq_points Frequency grid resolution per pair.
 * @return            N*N symmetric Eigen matrix of gap distances.
 */
Eigen::MatrixXd nuGapMatrix(const std::vector<StateSpace>& models,
                             int freq_points = 200);

} // namespace ctrl
