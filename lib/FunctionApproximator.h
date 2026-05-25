#pragma once
#include "PlantModel.h"
#include <vector>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>

// FunctionApproximator - data-driven Taylor (polynomial) and Padé (rational)
// approximation from a dataset of (x, y) sample pairs.
//
// Both backends fit the approximant by solving a linear least-squares problem
// against the supplied data — no symbolic function required.  This matches
// the toolbox's identification-from-data philosophy (same as RLS and N4SID).
//
// ─────────────────────────────────────────────────────────────────────────────
// Taylor (polynomial) backend
// ─────────────────────────────────────────────────────────────────────────────
//   Fits:  f(x) ≈ P(x) = a₀ + a₁x + a₂x² + … + aₙxⁿ
//
//   Method: Vandermonde matrix V (rows = data points, cols = powers 0..n),
//           QR least-squares solve → coefficient vector a.
//
//   Best for: smooth functions away from poles, evaluation at any x, cheap.
//   Watch out: high-degree polynomials can overfit and oscillate (Runge's
//              phenomenon).  Keep n ≤ 8 unless you have many data points and
//              the function is well-behaved.
//
// ─────────────────────────────────────────────────────────────────────────────
// Padé (rational) backend
// ─────────────────────────────────────────────────────────────────────────────
//   Fits:  f(x) ≈ P(x)/Q(x),  deg(P)=m, deg(Q)=n, Q(x)=1+q₁x+…+qₙxⁿ
//   (Q is normalised so that Q(0)=1, avoiding the trivial all-zero solution.)
//
//   Method: Rewrite as a linear system in (p₀…pₘ, q₁…qₙ) by moving the
//           denominator to the right-hand side:
//               P(xᵢ) − f(xᵢ)·Q(xᵢ) = 0   for each data point i
//           Collect into a matrix equation and solve in least-squares sense.
//           This is the standard Sanathanan-Koerner / linearised approach.
//
//   Best for: functions with poles or asymptotes, step-response transients,
//             fractional dead-time filters.
//   Watch out: denominator roots can land inside your evaluation domain,
//              causing division-by-zero.  Use hasPoleInDomain() to check.
//
// ─────────────────────────────────────────────────────────────────────────────
// Fractional dead-time helper
// ─────────────────────────────────────────────────────────────────────────────
//   padeDelayFilter(theta_frac, Ts) builds a discrete-time first-order Padé
//   approximant for the fractional-sample dead-time (theta_frac < Ts) and
//   returns a 1-state StateSpace you can absorb into a SmithPredictor model.
//
//   Formula (Tustin-discretised continuous [1,1] Padé):
//       H_c(s) = (1 - 0.5·theta_frac·s) / (1 + 0.5·theta_frac·s)
//   This exactly matches the delay e^{-theta_frac·s} to first order.
//
// Ref: Vetter, "Numerical Continuation Methods" (1991);
//      Baker & Graves-Morris, "Padé Approximants" (1996);
//      Åström & Wittenmark, "Computer Controlled Systems" Sec. 6.4 (1997).

namespace ctrl
{

// ─────────────────────────────────────────────────────────────────────────────
// TaylorApproximator
// ─────────────────────────────────────────────────────────────────────────────
class TaylorApproximator
{
public:
    // Fit a polynomial of degree n to data (xs, ys).
    // xs and ys must have the same length; length must be > n.
    // Throws std::invalid_argument on dimension mismatch or underdetermined system.
    TaylorApproximator(const std::vector<double> &xs,
                       const std::vector<double> &ys,
                       int degree);

    // Evaluate the fitted polynomial at x.
    double evaluate(double x) const;

    // Coefficients a[0..degree]: f(x) = a[0] + a[1]*x + ... + a[n]*x^n
    const std::vector<double> &coefficients() const { return coeffs_; }

    int degree() const { return static_cast<int>(coeffs_.size()) - 1; }

    // RMS residual of the fit over the training data.
    double fitRMSE() const { return rmse_; }

private:
    std::vector<double> coeffs_;
    double              rmse_;
};


// ─────────────────────────────────────────────────────────────────────────────
// PadeApproximator
// ─────────────────────────────────────────────────────────────────────────────
class PadeApproximator
{
public:
    // Fit a [m/n] Padé rational approximant to data (xs, ys).
    // xs and ys must have the same length; length must be > m + n.
    // Throws std::invalid_argument on dimension mismatch or underdetermined system.
    PadeApproximator(const std::vector<double> &xs,
                     const std::vector<double> &ys,
                     int num_degree,
                     int den_degree);

    // Evaluate P(x)/Q(x) at x.
    // Throws std::domain_error if Q(x) is zero (pole in evaluation domain).
    double evaluate(double x) const;

    // Numerator coefficients p[0..m]: P(x) = p[0] + p[1]*x + ... + p[m]*x^m
    const std::vector<double> &numerator()   const { return p_; }

    // Denominator coefficients q[0..n] with q[0]=1:
    //   Q(x) = 1 + q[1]*x + ... + q[n]*x^n
    const std::vector<double> &denominator() const { return q_; }

    int numDegree() const { return static_cast<int>(p_.size()) - 1; }
    int denDegree() const { return static_cast<int>(q_.size()) - 1; }

    // RMS residual of the fit over the training data.
    double fitRMSE() const { return rmse_; }

    // Check whether Q(x) has a real root inside [x_lo, x_hi] (sampled at n_pts points).
    // Returns true (has a pole) if the sign of Q(x) changes anywhere in the interval.
    bool hasPoleInDomain(double x_lo, double x_hi, int n_pts = 1000) const;

    // Convert the [1/1] Padé approximant to a discrete-time StateSpace filter.
    // Only valid when numDegree()==1 and denDegree()==1.
    // The approximant is treated as a rational function of z^{-1} and converted
    // via the controllable-canonical form.
    // Throws std::logic_error if the approximant is not [1/1].
    StateSpace toDiscreteFilter(double Ts) const;

private:
    std::vector<double> p_; // numerator coefficients [p0, p1, ..., pm]
    std::vector<double> q_; // denominator coefficients [1, q1, ..., qn]
    double              rmse_;
};


// ─────────────────────────────────────────────────────────────────────────────
// Free function: first-order Padé filter for fractional dead-time
// ─────────────────────────────────────────────────────────────────────────────
//
// Returns a 1-state discrete StateSpace representing the first-order Padé
// approximant of a pure delay e^{-theta_frac * s}, discretised with Tustin:
//
//   Continuous:  H(s) = (1 - 0.5*theta_frac*s) / (1 + 0.5*theta_frac*s)
//   Discrete (Tustin, alpha = Ts/2):
//     numerator  z-coeffs: [1 - theta_frac/Ts,   -(1 - theta_frac/Ts)]  (approx)
//
// theta_frac must be in (0, Ts).  For theta_frac == 0, returns a unit-gain
// static gain StateSpace (D=1, no dynamics).
//
// Typical use — absorbing a fractional delay into a SmithPredictor model:
//
//   StateSpace G0 = ...;          // integer-delay-free plant
//   double theta    = 0.73;       // true dead time [s]
//   double Ts       = 0.5;        // sample time [s]
//   int    d_int    = static_cast<int>(theta / Ts);          // = 1
//   double theta_fr = theta - d_int * Ts;                    // = 0.23 s
//   StateSpace Hfrac = ctrl::padeDelayFilter(theta_fr, Ts);  // 1-state filter
//   // Series connection: absorb Hfrac into G0 before passing to SmithPredictor.
//   // (Simple SISO cascade: multiply TFs or augment the state-space manually.)
//   ctrl::SmithPredictor sp(inner, G0, d_int, Hfrac);        // see SmithPredictor
//
StateSpace padeDelayFilter(double theta_frac, double Ts);

} // namespace ctrl
