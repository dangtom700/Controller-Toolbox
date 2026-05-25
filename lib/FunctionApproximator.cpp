#include "FunctionApproximator.h"
#include <cmath>
#include <stdexcept>
#include <string>

namespace ctrl
{

// ─────────────────────────────────────────────────────────────────────────────
// TaylorApproximator
// ─────────────────────────────────────────────────────────────────────────────

TaylorApproximator::TaylorApproximator(const std::vector<double> &xs,
                                       const std::vector<double> &ys,
                                       int degree)
{
    const int N = static_cast<int>(xs.size());
    if (static_cast<int>(ys.size()) != N)
        throw std::invalid_argument("TaylorApproximator: xs and ys must have the same length.");
    if (degree < 0)
        throw std::invalid_argument("TaylorApproximator: degree must be >= 0.");
    if (N <= degree)
        throw std::invalid_argument(
            "TaylorApproximator: need at least degree+1 data points (have " +
            std::to_string(N) + ", need " + std::to_string(degree + 1) + ").");

    // Build Vandermonde matrix V (N x (degree+1))
    // V[i][j] = xs[i]^j
    const int cols = degree + 1;
    Eigen::MatrixXd V(N, cols);
    for (int i = 0; i < N; ++i)
    {
        double xpow = 1.0;
        for (int j = 0; j < cols; ++j)
        {
            V(i, j) = xpow;
            xpow   *= xs[i];
        }
    }

    Eigen::VectorXd y_vec(N);
    for (int i = 0; i < N; ++i) y_vec(i) = ys[i];

    // Solve V * a = y in least-squares sense via QR decomposition.
    // ColPivHouseholderQR handles rank-deficient Vandermonde (e.g., duplicate x-values).
    const Eigen::VectorXd a = V.colPivHouseholderQr().solve(y_vec);

    coeffs_.resize(cols);
    for (int j = 0; j < cols; ++j) coeffs_[j] = a(j);

    // RMS residual
    const Eigen::VectorXd resid = V * a - y_vec;
    rmse_ = std::sqrt(resid.squaredNorm() / N);
}

double TaylorApproximator::evaluate(double x) const
{
    // Horner's method: p = a[n] + x*(a[n-1] + x*(...))
    double p = 0.0;
    for (int j = static_cast<int>(coeffs_.size()) - 1; j >= 0; --j)
        p = p * x + coeffs_[j];
    return p;
}


// ─────────────────────────────────────────────────────────────────────────────
// PadeApproximator
// ─────────────────────────────────────────────────────────────────────────────

// Linearised Padé fit (Sanathanan-Koerner approach, one iteration).
//
// We seek P(x)/Q(x) ≈ f(x) with Q(0)=1 (normalised).
// Rearranging:  P(x) - f(x)*Q(x) = 0
//   p₀ + p₁x + … + pₘxᵐ - f(x)*(1 + q₁x + … + qₙxⁿ) = 0
//
// Writing the unknown vector as θ = [p₀ … pₘ, q₁ … qₙ]ᵀ, each data point
// (xᵢ, yᵢ) contributes one row to the linear system A·θ = b where:
//   A[i, 0..m]   = [1, xᵢ, xᵢ², …, xᵢᵐ]
//   A[i, m+1..m+n] = [-yᵢ·xᵢ, -yᵢ·xᵢ², …, -yᵢ·xᵢⁿ]
//   b[i]         = yᵢ   (the Q(0)=1 constant term absorbs yᵢ)
//
// This is solved in the least-squares sense via QR.

PadeApproximator::PadeApproximator(const std::vector<double> &xs,
                                   const std::vector<double> &ys,
                                   int num_degree,
                                   int den_degree)
{
    const int N = static_cast<int>(xs.size());
    if (static_cast<int>(ys.size()) != N)
        throw std::invalid_argument("PadeApproximator: xs and ys must have the same length.");
    if (num_degree < 0 || den_degree < 0)
        throw std::invalid_argument("PadeApproximator: degrees must be >= 0.");
    const int unknowns = (num_degree + 1) + den_degree; // p's + q's (q0=1 fixed)
    if (N <= unknowns)
        throw std::invalid_argument(
            "PadeApproximator: underdetermined system — need more data points than unknowns "
            "(have " + std::to_string(N) + ", need > " + std::to_string(unknowns) + ").");

    const int m = num_degree;
    const int n = den_degree;
    const int cols = (m + 1) + n;

    Eigen::MatrixXd A(N, cols);
    Eigen::VectorXd b(N);

    for (int i = 0; i < N; ++i)
    {
        const double xi = xs[i];
        const double yi = ys[i];

        // Numerator columns: 1, xi, xi^2, ..., xi^m
        double xpow = 1.0;
        for (int j = 0; j <= m; ++j)
        {
            A(i, j) = xpow;
            xpow   *= xi;
        }

        // Denominator columns (q1..qn): -yi*xi, -yi*xi^2, ..., -yi*xi^n
        xpow = xi;
        for (int j = 1; j <= n; ++j)
        {
            A(i, m + j) = -yi * xpow;
            xpow        *= xi;
        }

        b(i) = yi; // right-hand side: yi * Q(0) = yi * 1
    }

    const Eigen::VectorXd theta = A.colPivHouseholderQr().solve(b);

    // Unpack numerator coefficients p[0..m]
    p_.resize(m + 1);
    for (int j = 0; j <= m; ++j) p_[j] = theta(j);

    // Unpack denominator coefficients q[0..n] with q[0] = 1
    q_.resize(n + 1);
    q_[0] = 1.0;
    for (int j = 1; j <= n; ++j) q_[j] = theta(m + j);

    // Compute RMS residual
    double sse = 0.0;
    for (int i = 0; i < N; ++i)
    {
        double err = evaluate(xs[i]) - ys[i];
        sse += err * err;
    }
    rmse_ = std::sqrt(sse / N);
}

double PadeApproximator::evaluate(double x) const
{
    // Evaluate numerator P(x) via Horner
    double P = 0.0;
    for (int j = static_cast<int>(p_.size()) - 1; j >= 0; --j)
        P = P * x + p_[j];

    // Evaluate denominator Q(x) via Horner
    double Q = 0.0;
    for (int j = static_cast<int>(q_.size()) - 1; j >= 0; --j)
        Q = Q * x + q_[j];

    if (std::abs(Q) < 1e-300)
        throw std::domain_error(
            "PadeApproximator::evaluate: denominator Q(x) is zero at x = " +
            std::to_string(x) + " — pole in evaluation domain.");

    return P / Q;
}

bool PadeApproximator::hasPoleInDomain(double x_lo, double x_hi, int n_pts) const
{
    if (n_pts < 2) n_pts = 2;
    const double dx = (x_hi - x_lo) / (n_pts - 1);

    // Evaluate Q at the first point
    auto evalQ = [&](double x) {
        double Q = 0.0;
        for (int j = static_cast<int>(q_.size()) - 1; j >= 0; --j)
            Q = Q * x + q_[j];
        return Q;
    };

    double Q_prev = evalQ(x_lo);
    for (int i = 1; i < n_pts; ++i)
    {
        double Q_curr = evalQ(x_lo + i * dx);
        if (Q_prev * Q_curr < 0.0) return true; // sign change → real root
        Q_prev = Q_curr;
    }
    return false;
}

StateSpace PadeApproximator::toDiscreteFilter(double Ts) const
{
    if (numDegree() != 1 || denDegree() != 1)
        throw std::logic_error(
            "PadeApproximator::toDiscreteFilter: only [1/1] Padé supported. "
            "Got [" + std::to_string(numDegree()) + "/" + std::to_string(denDegree()) + "].");

    // Rational function in x-domain: (p0 + p1*x) / (1 + q1*x)
    // Treat x as z^{-1} and convert to state-space (controllable canonical form).
    // H(z^{-1}) = (p0 + p1*z^{-1}) / (1 + q1*z^{-1})
    // Multiply through by z: H(z) = (p0*z + p1) / (z + q1)
    // State-space: A = -q1, B = 1, C = p1 - p0*q1, D = p0
    const double p0 = p_[0], p1 = p_[1];
    const double q1 = q_[1];

    Eigen::MatrixXd A(1, 1), B(1, 1), C(1, 1), D(1, 1);
    A(0, 0) = -q1;
    B(0, 0) = 1.0;
    C(0, 0) = p1 - p0 * q1;
    D(0, 0) = p0;

    return StateSpace(A, B, C, D, Ts);
}


// ─────────────────────────────────────────────────────────────────────────────
// padeDelayFilter — fractional dead-time first-order Padé filter
// ─────────────────────────────────────────────────────────────────────────────

StateSpace padeDelayFilter(double theta_frac, double Ts)
{
    if (Ts <= 0.0)
        throw std::invalid_argument("padeDelayFilter: Ts must be positive.");
    if (theta_frac < 0.0 || theta_frac >= Ts)
        throw std::invalid_argument(
            "padeDelayFilter: theta_frac must be in [0, Ts). Got theta_frac=" +
            std::to_string(theta_frac) + ", Ts=" + std::to_string(Ts) + ".");

    if (theta_frac < 1e-12)
    {
        // No fractional delay — return unity static gain (D=1, no dynamics).
        Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
        A(0,0)=0.0; B(0,0)=0.0; C(0,0)=0.0; D(0,0)=1.0;
        return StateSpace(A, B, C, D, Ts);
    }

    // Continuous [1/1] Padé approximant of e^{-theta_frac * s}:
    //   H_c(s) = (1 - (theta_frac/2)*s) / (1 + (theta_frac/2)*s)
    //
    // Discretise with Tustin (bilinear) transform:  s = (2/Ts)*(z-1)/(z+1)
    // Let  r = theta_frac / Ts  (fractional ratio, 0 < r < 1)
    //
    //   H(z) = (1 - r*(z-1)/(z+1)) / (1 + r*(z-1)/(z+1))
    //        = ((z+1) - r*(z-1)) / ((z+1) + r*(z-1))
    //        = ((1-r)*z + (1+r)) … wait — let me expand cleanly:
    //
    //  Numerator:    (z+1) - r*(z-1) = z(1-r) + (1+r)
    //  Denominator:  (z+1) + r*(z-1) = z(1+r) + (1-r)
    //
    //  Divide by (1+r) to get monic denominator:
    //    b0 = (1-r)/(1+r),  b1 = 1
    //    a1 = (1-r)/(1+r)   (the z^0 / z^1 ratio in denominator)
    //
    //  In z^{-1} form (multiply through by z^{-1}):
    //    H(z^{-1}) = ((1-r) + (1+r)*z^{-1}) / ((1+r) + (1-r)*z^{-1})
    //    Normalise denominator: divide through by (1+r):
    //    H(z^{-1}) = (b0_n + b1_n*z^{-1}) / (1 + a1_n*z^{-1})
    //    where  b0_n = (1-r)/(1+r),  b1_n = 1,  a1_n = (1-r)/(1+r)
    //
    // State-space (controllable canonical, 1 state):
    //   A = -a1_n,  B = 1,  C = b1_n - b0_n*a1_n,  D = b0_n

    const double r    = theta_frac / Ts;
    const double b0_n = (1.0 - r) / (1.0 + r); // = a1_n by symmetry
    const double a1_n = b0_n;
    const double b1_n = 1.0;

    Eigen::MatrixXd A(1,1), B(1,1), C(1,1), D(1,1);
    A(0,0) = -a1_n;
    B(0,0) =  1.0;
    C(0,0) =  b1_n - b0_n * a1_n;
    D(0,0) =  b0_n;

    return StateSpace(A, B, C, D, Ts);
}

} // namespace ctrl
