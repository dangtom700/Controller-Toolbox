#include "MLEIdentifier.h"
#include "AutoTuner.h"
#include <cmath>
#include <stdexcept>

namespace ctrl
{

namespace
{

void buildRegressor(const Eigen::VectorXd &u, const Eigen::VectorXd &y, int na, int nb,
                     Eigen::MatrixXd &Phi, Eigen::VectorXd &Y)
{
    const int m = std::max(na, nb);
    const int N = static_cast<int>(y.size()) - m;
    Phi.resize(N, na + nb);
    Y.resize(N);
    for (int k = 0; k < N; ++k)
    {
        const int t = k + m; // absolute sample index for y[t]
        for (int i = 0; i < na; ++i) Phi(k, i) = -y(t - 1 - i);
        for (int i = 0; i < nb; ++i) Phi(k, na + i) = u(t - 1 - i);
        Y(k) = y(t);
    }
}

double negLogLikelihood(const Eigen::VectorXd &theta, const Eigen::MatrixXd &Phi,
                         const Eigen::VectorXd &Y, const MLEParams &p)
{
    const Eigen::VectorXd e = Y - Phi * theta;
    const double N = static_cast<double>(e.size());

    double nll = 0.0;
    if (p.noise == NoiseModel::Gaussian)
    {
        const double sse = e.squaredNorm();
        nll = 0.5 * N * std::log(std::max(sse / N, 1e-300));
    }
    else // Laplace
    {
        const double sad = e.cwiseAbs().sum();
        nll = N * std::log(std::max(sad, 1e-300));
    }

    if (p.prior_mean.size() > 0 && p.prior_cov.size() > 0)
    {
        const Eigen::VectorXd d = theta - p.prior_mean;
        nll += 0.5 * d.dot(p.prior_cov.ldlt().solve(d));
    }
    return nll;
}

Eigen::MatrixXd numericalHessian(const Eigen::VectorXd &theta, const Eigen::MatrixXd &Phi,
                                  const Eigen::VectorXd &Y, const MLEParams &p)
{
    const int n = static_cast<int>(theta.size());
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd h(n);
    for (int i = 0; i < n; ++i) h(i) = 1e-4 * std::max(std::fabs(theta(i)), 1.0);

    const double f0 = negLogLikelihood(theta, Phi, Y, p);
    for (int i = 0; i < n; ++i)
    {
        for (int j = i; j < n; ++j)
        {
            double d2;
            if (i == j)
            {
                Eigen::VectorXd tp = theta, tm = theta;
                tp(i) += h(i);
                tm(i) -= h(i);
                const double fpp = negLogLikelihood(tp, Phi, Y, p);
                const double fmm = negLogLikelihood(tm, Phi, Y, p);
                d2 = (fpp - 2.0 * f0 + fmm) / (h(i) * h(i));
            }
            else
            {
                Eigen::VectorXd tpp = theta, tpm = theta, tmp = theta, tmm = theta;
                tpp(i) += h(i); tpp(j) += h(j);
                tpm(i) += h(i); tpm(j) -= h(j);
                tmp(i) -= h(i); tmp(j) += h(j);
                tmm(i) -= h(i); tmm(j) -= h(j);

                const double fpp = negLogLikelihood(tpp, Phi, Y, p);
                const double fpm = negLogLikelihood(tpm, Phi, Y, p);
                const double fmp = negLogLikelihood(tmp, Phi, Y, p);
                const double fmm = negLogLikelihood(tmm, Phi, Y, p);
                d2 = (fpp - fpm - fmp + fmm) / (4.0 * h(i) * h(j));
            }
            H(i, j) = d2;
            H(j, i) = d2;
        }
    }
    return H;
}

} // namespace

MLEResult MLEIdentifier::fit(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                              double /*Ts*/, const MLEParams &params)
{
    if (u.size() != y.size())
        throw std::invalid_argument("MLEIdentifier::fit: u and y must have the same length");
    const int m = std::max(params.na, params.nb);
    if (static_cast<int>(y.size()) <= m)
        throw std::invalid_argument("MLEIdentifier::fit: not enough samples for na/nb");

    Eigen::MatrixXd Phi;
    Eigen::VectorXd Y;
    buildRegressor(u, y, params.na, params.nb, Phi, Y);

    const Eigen::VectorXd theta0 = (Phi.transpose() * Phi).ldlt().solve(Phi.transpose() * Y);

    AutoTunerParams atp;
    atp.n = params.na + params.nb;
    atp.maxIter = params.max_iter;
    atp.tol = params.tol;
    AutoTuner tuner(atp);

    auto cost = [&](const Eigen::VectorXd &theta) {
        return negLogLikelihood(theta, Phi, Y, params);
    };
    const TunerResult tr = tuner.tune(cost, theta0);

    MLEResult result;
    result.theta = tr.params;
    result.logLikelihood = -tr.cost;
    result.converged = tr.converged;

    const Eigen::MatrixXd H = numericalHessian(result.theta, Phi, Y, params);
    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    if (ldlt.info() == Eigen::Success && ldlt.isPositive())
    {
        result.covariance = ldlt.solve(Eigen::MatrixXd::Identity(H.rows(), H.cols()));
    }
    else
    {
        result.covariance =
            H.completeOrthogonalDecomposition().pseudoInverse();
    }

    return result;
}

} // namespace ctrl
