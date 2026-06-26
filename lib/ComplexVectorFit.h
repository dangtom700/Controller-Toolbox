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
