#pragma once
#include "PlantModel.h"
#include <complex>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>

/**
 * @file GapMetric.h
 * @brief Nu-gap (Vinnicombe gap) metric for SISO and MIMO discrete-time linear systems.
 *
 * The nu-gap delta_nu(P1, P2) measures the distance between two plants' closed-loop
 * behaviours. A value below 0.5 means a single robust controller can stabilise both
 * (Vinnicombe 2001, "Uncertainty and Feedback").
 *
 * **SISO implementation:** chordal metric on the Riemann sphere:
 * @code
 *   d_chordal(p1, p2) = |p1 - p2| / sqrt((1 + |p1|^2) * (1 + |p2|^2))
 * @endcode
 *
 * **MIMO implementation:** subspace chordal distance via normalised graph:
 * @code
 *   G_i(jw) = col-span of [P_i(jw); I]  (normalised via thin SVD: G_i = U_i S_i V_i^H)
 *   d_subspace(P1, P2) = sqrt(1 - sigma_min(U1^H * U2)^2)
 * @endcode
 * This equals the SISO formula for p=m=1 (verified analytically). For MIMO it measures
 * the largest principal angle between the two normalised-graph subspaces.
 *
 * @par Reference
 * Vinnicombe, G. (2001). *Uncertainty and Feedback*. Imperial College Press, Ch. 5.
 * Skogestad, S. & Postlethwaite, I. (2005). *Multivariable Feedback Design*, Ch. 9.
 */

namespace ctrl {

/**
 * @brief Evaluate the discrete-time frequency response P(e^{j*omega*Ts}).
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
 * Range [0, 1]. Equals 0 iff p1 == p2.
 */
double chordalDist(std::complex<double> p1, std::complex<double> p2);

/**
 * @brief Subspace chordal distance between two MIMO frequency-response matrices.
 *
 * Forms the normalised graphs [P1; I] and [P2; I], computes their thin SVDs
 * to extract column spaces U1, U2, and returns sqrt(1 - sigma_min(U1^H * U2)^2).
 *
 * Reduces to chordalDist() for p=m=1 (SISO).
 *
 * @param H1  p*m complex frequency-response matrix of P1 at one frequency.
 * @param H2  p*m complex frequency-response matrix of P2 at one frequency.
 * @return    Subspace chordal distance in [0, 1].
 */
double subspaceDist(const Eigen::MatrixXcd& H1, const Eigen::MatrixXcd& H2);

/**
 * @brief Nu-gap upper bound between two discrete-time systems (SISO or MIMO).
 *
 * Dispatches to the scalar chordal formula (SISO) or the subspace formula (MIMO).
 * Both systems must have identical dimensions and the same Ts.
 *
 * @param P1          First discrete-time system.
 * @param P2          Second system (same p, m, Ts).
 * @param freq_points Log-spaced frequency grid size (default 200).
 * @param omega_min   Minimum frequency [rad/s] (default 0.01).
 * @return            Nu-gap upper bound in [0, 1].
 * @throws std::invalid_argument On dimension or Ts mismatch.
 */
double nuGap(const StateSpace& P1, const StateSpace& P2,
             int freq_points = 200, double omega_min = 1e-2);

/**
 * @brief Nu-gap upper bound between two SISO discrete TF models.
 *
 * Converts both to state-space internally.
 */
double nuGap(const TransferFunction& P1, const TransferFunction& P2,
             int freq_points = 200, double omega_min = 1e-2);

/**
 * @brief Symmetric N*N nu-gap distance matrix for a collection of systems.
 *
 * All systems must have the same p, m, and Ts.
 *
 * @param models      Collection of discrete-time state-space models.
 * @param freq_points Frequency resolution per pair.
 * @return            N*N symmetric Eigen matrix of gap distances.
 */
Eigen::MatrixXd nuGapMatrix(const std::vector<StateSpace>& models,
                             int freq_points = 200);

} // namespace ctrl
