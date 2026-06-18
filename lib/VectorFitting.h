#pragma once
#include "PlantModel.h"
#include "Features.h"
#include <Eigen/Dense>
#include <vector>
#include <complex>

/**
 * @file VectorFitting.h
 * @brief Rational approximation of frequency-domain magnitude data via Sanathanan-Koerner
 *        (SK) iterative weighted least-squares - the core step of full DK-iteration.
 *
 * Given a set of (omega, magnitude) samples, VectorFitting::fitMagnitude() returns a
 * stable discrete-time StateSpace H(z) such that |H(e^{jomegaTs})| approx = magnitude(omega) over the
 * provided frequency grid.
 *
 * **Algorithm (SK iteration, real-pole variant):**
 * 1. Initialise n_poles real poles: log-uniformly spaced on (0, pi/Ts) -> mapped inside
 *    the unit disk via p_k = exp(-sigma_k * Ts).
 * 2. Build the linear system in the SK frame (divides the numerator/denominator model by
 *    the previous-iteration denominator to re-linearise).
 * 3. Solve the over-determined real LS problem for numerator and denominator coefficients.
 * 4. Relocate poles to roots of the new denominator polynomial; enforce stability by
 *    reflecting any roots outside the unit disk back inside.
 * 5. Repeat for max_iter rounds or until pole displacement < tol.
 * 6. Return the rational model in first-companion state-space form.
 *
 * **Use in DK-iteration:** set MuSynParams::dFitOrder > 1.  The D-step fits a dFitOrder-pole
 * filter to the per-channel magnitude profile instead of the first-order
 * fitFirstOrderDFilter() heuristic.  This enables accurate approximation of structured
 * D-scalings with multiple frequency inflections.
 *
 * @see Gustavsen & Semlyen, "Rational approximation of frequency domain responses by
 *      vector fitting," IEEE Trans. Power Del. 14(3), 1999.
 * @see DiscreteHinf::solveMuSyn (MuSynParams::dFitOrder).
 */

namespace ctrl
{

/**
 * @brief Tuning parameters for VectorFitting::fitMagnitude().
 */
struct VectorFittingParams
{
    int    max_iter = 10;   ///< SK iteration limit.
    double tol      = 1e-5; ///< Stop when max pole displacement (|Deltap|) < tol.
    double eps_mag  = 1e-9; ///< Regularisation floor for near-zero magnitudes.
};

/**
 * @brief Result of VectorFitting::fitMagnitude().
 */
struct VectorFittingResult
{
    bool   converged = false; ///< @c true if pole displacement < VectorFittingParams::tol.
    int    iterations = 0;    ///< SK rounds performed.
    double rms_error  = 0.0;  ///< RMS relative error |fit - target| / |target| at data points.
};

/**
 * @brief Rational approximation of real positive frequency-domain magnitude profiles.
 *
 * All poles in the returned StateSpace are real and inside the unit disk (stable).
 * The filter has unity gain at omega = 0 (DC) by default - useful for D-scaling normalization.
 */
class VectorFitting
{
public:
    /**
     * @brief Fit a stable rational filter H(z) to the given magnitude profile.
     *
     * @param omega_grid  Frequency grid [rad/s] - strictly increasing, in [0, pi/Ts).
     * @param magnitude   Positive target magnitudes |H(jomega)| at each grid point.
     * @param n_poles     Number of real poles (order of the rational approximant).
     * @param Ts          Sample period [s].
     * @param[out] result Diagnostic information (convergence, iterations, RMS error).
     * @param params      Algorithm tuning (default suitable for D-scaling profiles).
     * @return            Discrete-time StateSpace H(z) of order n_poles.
     */
    static StateSpace fitMagnitude(
        const std::vector<double>      &omega_grid,
        const std::vector<double>      &magnitude,
        int                             n_poles,
        double                          Ts,
        VectorFittingResult            &result,
        const VectorFittingParams      &params = {});

    /**
     * @brief Evaluate |H(e^{jomegaTs})| for a given StateSpace and frequency.
     * @param sys StateSpace to evaluate.
     * @param omega Frequency [rad/s].
     * @return |H(e^{jomegaTs})| - positive real scalar.
     */
    static double evalMagnitude(const StateSpace &sys, double omega);

private:
    // Build the SK divided LS matrix and RHS for one iteration.
    static void buildSKSystem(
        const std::vector<std::complex<double>> &z_grid,   // z = exp(j*omega*Ts)
        const std::vector<double>               &mag,      // target magnitudes
        const Eigen::VectorXd                   &poles,    // current poles (real, inside disk)
        int n_poles,
        Eigen::MatrixXd &A_out,
        Eigen::VectorXd &b_out);

    // Stabilise poles: reflect |p| > 1 to 1/p.
    static void enforcePoleStability(Eigen::VectorXd &poles);

    // Build state-space from partial-fraction residues and real poles.
    static StateSpace buildStateSpace(
        const Eigen::VectorXd &poles,
        const Eigen::VectorXd &residues,
        double d_gain,
        double Ts);

    // Log-spaced initial poles inside the unit disk.
    static Eigen::VectorXd initPoles(int n_poles, double omega_max, double Ts);
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(vector_fitting)
