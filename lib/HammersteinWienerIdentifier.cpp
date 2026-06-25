#include "HammersteinWienerIdentifier.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

namespace {

double evalPoly(const Eigen::VectorXd &coeffs, double v)
{
    double result = 0.0, p = 1.0;
    for (int m = 0; m < coeffs.size(); ++m)
    {
        result += coeffs(m) * p;
        p *= v;
    }
    return result;
}

// Batch ARX least squares: y[k] = -sum_i a_i*y[k-i] + sum_j b_j*v[k-j] + e[k], solved once
// via QR over k = max(na,nb)..N-1 - the direct batch analogue of RecursiveLeastSquares's
// regressor convention (phi[k] = [-y[k-1..k-na], v[k-1..k-nb]]), not a loop over update().
void fitBatchARX(const Eigen::VectorXd &v, const Eigen::VectorXd &y, int na, int nb,
                  Eigen::VectorXd &a_out, Eigen::VectorXd &b_out)
{
    const int N = static_cast<int>(y.size());
    const int kStart = std::max(na, nb);
    const int M = N - kStart;
    const int nTheta = na + nb;

    Eigen::MatrixXd Phi(M, nTheta);
    Eigen::VectorXd target(M);
    for (int k = kStart; k < N; ++k)
    {
        const int row = k - kStart;
        for (int i = 1; i <= na; ++i) Phi(row, i - 1)      = -y(k - i);
        for (int j = 1; j <= nb; ++j) Phi(row, na + j - 1) = v(k - j);
        target(row) = y(k);
    }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Phi);
    const Eigen::VectorXd theta = qr.solve(target);
    a_out = theta.head(na);
    b_out = theta.tail(nb);
}

void validateInputs(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                     const HammersteinWienerParams &params)
{
    if (u.size() != y.size())
        throw std::invalid_argument(
            "HammersteinWienerIdentifier: u and y must have the same length.");
    const int N = static_cast<int>(u.size());
    const int kStart = std::max(params.na, params.nb);
    const int M = N - kStart;
    if (M < params.na + params.nb)
        throw std::invalid_argument(
            "HammersteinWienerIdentifier: too few samples for the requested na/nb.");
    if (M < params.nl_degree + 1)
        throw std::invalid_argument(
            "HammersteinWienerIdentifier: too few samples for the requested nl_degree.");
}

TransferFunction buildTF(const Eigen::VectorXd &a, const Eigen::VectorXd &b, double Ts)
{
    const int na = static_cast<int>(a.size());
    const int nb = static_cast<int>(b.size());
    std::vector<double> num(nb + 1, 0.0); // num[0] = 0 (no direct feedthrough, matches the
                                            // RLS/ARX convention's b1..bnb starting at lag 1)
    for (int j = 0; j < nb; ++j) num[j + 1] = b(j);
    std::vector<double> den(na + 1, 0.0);
    den[0] = 1.0;
    for (int i = 0; i < na; ++i) den[i + 1] = a(i);
    return TransferFunction(num, den, Ts);
}

} // namespace

HammersteinWienerResult HammersteinWienerIdentifier::fitHammerstein(
    const Eigen::VectorXd &u, const Eigen::VectorXd &y, double Ts,
    const HammersteinWienerParams &params)
{
    validateInputs(u, y, params);
    const int N = static_cast<int>(u.size());
    const int d = params.nl_degree;
    const int kStart = std::max(params.na, params.nb);
    const int M = N - kStart;

    // HammersteinWienerResult holds a TransferFunction (no default constructor), so it
    // cannot be default-constructed and incrementally filled in across the loop -
    // accumulate into plain locals instead and build the result via aggregate-init only
    // at the return.
    Eigen::VectorXd c = Eigen::VectorXd::Zero(d + 1);
    c(1) = 1.0; // identity nonlinearity initial guess
    Eigen::VectorXd a, b, cPrev, aPrev, bPrev;
    bool converged = false;
    int itersDone = 0;

    for (int iter = 0; iter < params.max_iter; ++iter)
    {
        Eigen::VectorXd v(N);
        for (int k = 0; k < N; ++k) v(k) = evalPoly(c, u(k));

        fitBatchARX(v, y, params.na, params.nb, a, b);

        // Nonlinear sub-step: y[k] + sum_i a_i*y[k-i] = sum_m c_m * (sum_j b_j*u[k-j]^m),
        // linear in c - build the regressor and solve via QR.
        Eigen::MatrixXd Phi(M, d + 1);
        Eigen::VectorXd target(M);
        for (int k = kStart; k < N; ++k)
        {
            double t = y(k);
            for (int i = 1; i <= params.na; ++i) t += a(i - 1) * y(k - i);
            target(k - kStart) = t;
            for (int m = 0; m <= d; ++m)
            {
                double Rm = 0.0;
                for (int j = 1; j <= params.nb; ++j)
                    Rm += b(j - 1) * std::pow(u(k - j), m);
                Phi(k - kStart, m) = Rm;
            }
        }
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Phi);
        Eigen::VectorXd cNew = qr.solve(target);

        const double scale = cNew(1);
        if (std::fabs(scale) > 1e-12)
        {
            cNew /= scale;
            b *= scale;
        }

        bool converged_this_iter = false;
        if (iter > 0)
        {
            const double delta = std::max({(a - aPrev).cwiseAbs().maxCoeff(),
                                            (b - bPrev).cwiseAbs().maxCoeff(),
                                            (cNew - cPrev).cwiseAbs().maxCoeff()});
            converged_this_iter = delta < params.tol;
        }
        c = cNew;
        itersDone = iter + 1;
        aPrev = a; bPrev = b; cPrev = c;

        if (converged_this_iter)
        {
            converged = true;
            break;
        }
    }

    return HammersteinWienerResult{c, Eigen::VectorXd(), buildTF(a, b, Ts), converged, itersDone};
}

HammersteinWienerResult HammersteinWienerIdentifier::fitWiener(
    const Eigen::VectorXd &u, const Eigen::VectorXd &y, double Ts,
    const HammersteinWienerParams &params)
{
    validateInputs(u, y, params);
    const int N = static_cast<int>(u.size());
    const int d = params.nl_degree;
    const int kStart = std::max(params.na, params.nb);
    const int M = N - kStart;

    // See fitHammerstein's comment above re: avoiding early default-construction of a
    // struct holding a TransferFunction.
    Eigen::VectorXd wEst = y; // identity forward map initial guess
    Eigen::VectorXd a, b, dCoef, dPrev, aPrev, bPrev;
    bool converged = false;
    int itersDone = 0;

    for (int iter = 0; iter < params.max_iter; ++iter)
    {
        fitBatchARX(u, wEst, params.na, params.nb, a, b);

        Eigen::VectorXd wPred = Eigen::VectorXd::Zero(N);
        for (int k = kStart; k < N; ++k)
        {
            double val = 0.0;
            for (int i = 1; i <= params.na; ++i) val -= a(i - 1) * wPred(k - i);
            for (int j = 1; j <= params.nb; ++j) val += b(j - 1) * u(k - j);
            wPred(k) = val;
        }

        // Forward nonlinear sub-step: y[k] = sum_m d_m * wPred[k]^m, direct batch LS.
        Eigen::MatrixXd Phi(M, d + 1);
        Eigen::VectorXd target(M);
        for (int k = kStart; k < N; ++k)
        {
            double p = 1.0;
            for (int m = 0; m <= d; ++m)
            {
                Phi(k - kStart, m) = p;
                p *= wPred(k);
            }
            target(k - kStart) = y(k);
        }
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Phi);
        Eigen::VectorXd dNew = qr.solve(target);

        const double scale = dNew(1);
        if (std::fabs(scale) > 1e-12)
        {
            dNew /= scale;
            b *= scale;
            wPred *= scale; // keep wPred consistent with the now-rescaled b for the inverse refit
        }

        bool converged_this_iter = false;
        if (iter > 0)
        {
            const double delta = std::max({(a - aPrev).cwiseAbs().maxCoeff(),
                                            (b - bPrev).cwiseAbs().maxCoeff(),
                                            (dNew - dPrev).cwiseAbs().maxCoeff()});
            if (delta < params.tol) converged_this_iter = true;
        }
        dCoef = dNew;
        itersDone = iter + 1;
        aPrev = a; bPrev = b; dPrev = dCoef;

        if (converged_this_iter)
        {
            converged = true;
            break;
        }

        // Approximate-inverse refresh: fit an auxiliary polynomial g via batch LS regressing
        // wPred (target) against powers of y (regressor) - avoids symbolic/numeric inversion
        // of the forward map d, which risks non-existent/multiple real roots for higher
        // degrees. g/e are internal-only, never exposed in HammersteinWienerResult.
        Eigen::MatrixXd Phi2(M, d + 1);
        Eigen::VectorXd target2(M);
        for (int k = kStart; k < N; ++k)
        {
            double p = 1.0;
            for (int m = 0; m <= d; ++m)
            {
                Phi2(k - kStart, m) = p;
                p *= y(k);
            }
            target2(k - kStart) = wPred(k);
        }
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr2(Phi2);
        const Eigen::VectorXd eCoef = qr2.solve(target2);

        for (int k = 0; k < N; ++k) wEst(k) = evalPoly(eCoef, y(k));
    }

    return HammersteinWienerResult{Eigen::VectorXd(), dCoef, buildTF(a, b, Ts), converged, itersDone};
}

} // namespace ctrl
