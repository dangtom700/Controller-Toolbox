#pragma once
#include "PlantModel.h"
#include <vector>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>

/**
 * @file FunctionApproximator.h
 * @brief Data-driven Taylor (polynomial) and Padé (rational) function approximation.
 *
 * Both backends fit an approximant by solving a linear least-squares problem against supplied
 * (x, y) sample pairs — no symbolic function is required. This matches the toolbox's
 * identification-from-data philosophy (same as RLS and N4SID).
 *
 * **Taylor (polynomial) backend** — fits f(x) ≈ a₀ + a₁x + … + aₙxⁿ via a Vandermonde
 * least-squares system. Best for smooth functions away from poles; watch out for Runge's
 * phenomenon with high degrees (keep n ≤ 8 unless data is very dense and well-behaved).
 *
 * **Padé (rational) backend** — fits f(x) ≈ P(x)/Q(x) with deg(P) = m, deg(Q) = n,
 * Q(0) = 1, via the linearised Sanathanan–Koerner least-squares approach. Best for
 * functions with poles or asymptotes; check hasPoleInDomain() before evaluation to avoid
 * division by zero.
 *
 * @see Baker & Graves-Morris, "Padé Approximants" (1996).
 * @see Åström & Wittenmark, "Computer Controlled Systems" §6.4 (1997).
 */

namespace ctrl
{

/**
 * @brief Polynomial function approximator (Taylor / Vandermonde least squares).
 */
class TaylorApproximator
{
public:
    /**
     * @brief Fit a polynomial of degree @p degree to data (xs, ys).
     * @param xs     Input sample vector (strictly increasing recommended for stability).
     * @param ys     Output sample vector, same length as xs.
     * @param degree Polynomial degree n. Require xs.size() > degree.
     * @throws std::invalid_argument On dimension mismatch or underdetermined system.
     */
    TaylorApproximator(const std::vector<double> &xs,
                       const std::vector<double> &ys,
                       int degree);

    /**
     * @brief Evaluate the fitted polynomial at @p x.
     * @param x Evaluation point.
     * @return Approximate f(x).
     */
    double evaluate(double x) const;

    /**
     * @brief Polynomial coefficients a[0…degree]: f(x) = a[0] + a[1]·x + … + a[n]·xⁿ.
     */
    const std::vector<double> &coefficients() const { return coeffs_; }

    /** @brief Polynomial degree n. */
    int degree() const { return static_cast<int>(coeffs_.size()) - 1; }

    /**
     * @brief RMS residual of the fit over the training data.
     *
     * Lower is better; useful for comparing degree choices.
     */
    double fitRMSE() const { return rmse_; }

private:
    std::vector<double> coeffs_;
    double              rmse_;
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Rational function approximator (Padé / Sanathanan–Koerner least squares).
 *
 * Fits f(x) ≈ P(x)/Q(x), deg(P) = m, deg(Q) = n, Q(0) = 1. The denominator normalisation
 * avoids the trivial all-zero solution of the unconstrained bilinear problem.
 */
class PadeApproximator
{
public:
    /**
     * @brief Fit a [m/n] Padé rational approximant to data (xs, ys).
     * @param xs         Input sample vector.
     * @param ys         Output sample vector, same length as xs.
     * @param num_degree Numerator degree m. Require xs.size() > m + n.
     * @param den_degree Denominator degree n.
     * @throws std::invalid_argument On dimension mismatch or underdetermined system.
     */
    PadeApproximator(const std::vector<double> &xs,
                     const std::vector<double> &ys,
                     int num_degree,
                     int den_degree);

    /**
     * @brief Evaluate P(x)/Q(x) at @p x.
     * @param x Evaluation point.
     * @return Approximate f(x).
     * @throws std::domain_error If Q(x) ≈ 0 (pole near evaluation point).
     */
    double evaluate(double x) const;

    /**
     * @brief Numerator coefficients p[0…m]: P(x) = p[0] + p[1]·x + … + p[m]·xᵐ.
     */
    const std::vector<double> &numerator()   const { return p_; }

    /**
     * @brief Denominator coefficients q[0…n] with q[0] = 1: Q(x) = 1 + q[1]·x + … + q[n]·xⁿ.
     */
    const std::vector<double> &denominator() const { return q_; }

    int numDegree() const { return static_cast<int>(p_.size()) - 1; } ///< Numerator degree m.
    int denDegree() const { return static_cast<int>(q_.size()) - 1; } ///< Denominator degree n.

    /**
     * @brief RMS residual of the fit over the training data.
     */
    double fitRMSE() const { return rmse_; }

    /**
     * @brief Check whether Q(x) has a real root inside [x_lo, x_hi].
     *
     * Samples Q at @p n_pts points and returns @c true if the sign changes anywhere in the
     * interval, indicating a pole that would cause evaluate() to diverge.
     *
     * @param x_lo  Lower bound of the evaluation domain.
     * @param x_hi  Upper bound of the evaluation domain.
     * @param n_pts Number of sample points (default 1000).
     * @return @c true if a real pole exists in [x_lo, x_hi].
     */
    bool hasPoleInDomain(double x_lo, double x_hi, int n_pts = 1000) const;

    /**
     * @brief Convert a [1/1] Padé approximant to a discrete-time StateSpace filter.
     *
     * Treats the approximant as a rational function of z⁻¹ and converts via controllable
     * canonical form.
     *
     * @param Ts Sample time [s].
     * @return Discrete-time StateSpace (1 state, 1 input, 1 output).
     * @throws std::logic_error If the approximant is not [1/1].
     */
    StateSpace toDiscreteFilter(double Ts) const;

private:
    std::vector<double> p_; ///< Numerator coefficients [p0, p1, …, pm].
    std::vector<double> q_; ///< Denominator coefficients [1, q1, …, qn].
    double              rmse_;
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Build a first-order Padé filter for fractional dead-time compensation.
 *
 * Returns a 1-state discrete StateSpace representing e^{−θ_frac·s} to first order,
 * discretised via Tustin:
 * @code
 *   Continuous:  H(s) = (1 − 0.5·θ_frac·s) / (1 + 0.5·θ_frac·s)
 * @endcode
 *
 * Typical use — absorbing a fractional delay into a SmithPredictor:
 * @code
 *   int    d_int    = static_cast<int>(theta / Ts);   // integer steps
 *   double theta_fr = theta - d_int * Ts;             // sub-sample remainder
 *   StateSpace Hfrac = ctrl::padeDelayFilter(theta_fr, Ts);
 *   ctrl::SmithPredictor sp(inner, G0, d_int, Hfrac);
 * @endcode
 *
 * @param theta_frac Fractional dead time θ_frac ∈ (0, Ts) [s].
 *                   If 0, returns a unit-gain static StateSpace (no dynamics).
 * @param Ts         Sample time [s].
 * @return 1-state discrete StateSpace approximating e^{−θ_frac·s}.
 */
StateSpace padeDelayFilter(double theta_frac, double Ts);

} // namespace ctrl
