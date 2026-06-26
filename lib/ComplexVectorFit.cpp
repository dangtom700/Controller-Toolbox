#include "ComplexVectorFit.h"
#include "FreqDomainIdentifier.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl
{

std::complex<double> ComplexVectorFit::reflectPole(std::complex<double> p)
{
    if (std::abs(p) >= 1.0)
        return 1.0 / std::conj(p);
    return p;
}

std::vector<std::complex<double>> ComplexVectorFit::initPoles(
    int n_real_poles, int n_complex_pairs, double omega_max, double Ts)
{
    std::vector<std::complex<double>> poles;
    poles.reserve(n_real_poles + 2 * n_complex_pairs);

    const double omega_min = 1e-3;
    const double sig_lo = omega_min;
    const double sig_hi = std::min(omega_max * 0.8, M_PI / Ts * 0.9);

    for (int k = 0; k < n_real_poles; ++k)
    {
        const double t = (n_real_poles == 1) ? 0.5
                          : static_cast<double>(k) / (n_real_poles - 1);
        const double sigma = std::exp(std::log(sig_lo) * (1.0 - t) + std::log(sig_hi) * t);
        poles.emplace_back(std::exp(-sigma * Ts), 0.0);
    }

    constexpr double kZeta = 0.1;
    for (int k = 0; k < n_complex_pairs; ++k)
    {
        const double t = (n_complex_pairs == 1) ? 0.5
                          : static_cast<double>(k) / (n_complex_pairs - 1);
        const double wn = std::exp(std::log(sig_lo) * (1.0 - t) + std::log(sig_hi) * t);
        const double sigma = kZeta * wn;
        const double wd = wn * std::sqrt(1.0 - kZeta * kZeta);
        const std::complex<double> s(-sigma, wd);
        const std::complex<double> p = std::exp(s * Ts);
        poles.push_back(p);
        poles.push_back(std::conj(p));
    }
    return poles;
}

void ComplexVectorFit::buildSystem(
    const std::vector<std::complex<double>> &zinv_grid,
    const std::vector<std::complex<double>> &response,
    const std::vector<std::complex<double>> &poles,
    Eigen::MatrixXd &A_out,
    Eigen::VectorXd &b_out)
{
    const int N = static_cast<int>(zinv_grid.size());
    const int n_poles = static_cast<int>(poles.size());
    const int nc = n_poles + 1;
    const int ncols = nc + n_poles;

    A_out.resize(2 * N, ncols);
    b_out.resize(2 * N);

    for (int i = 0; i < N; ++i)
    {
        const std::complex<double> zinv = zinv_grid[i];
        const std::complex<double> H_i = response[i];

        std::complex<double> Dprev(1.0, 0.0);
        for (int k = 0; k < n_poles; ++k)
            Dprev *= (1.0 - poles[k] * zinv);

        if (std::norm(Dprev) < 1e-30)
        {
            A_out.row(2 * i).setZero();
            A_out.row(2 * i + 1).setZero();
            b_out(2 * i) = 0.0;
            b_out(2 * i + 1) = 0.0;
            continue;
        }

        std::complex<double> zinv_pow(1.0, 0.0);
        for (int j = 0; j < nc; ++j)
        {
            const std::complex<double> basis = zinv_pow / Dprev;
            A_out(2 * i, j)     = basis.real();
            A_out(2 * i + 1, j) = basis.imag();
            zinv_pow *= zinv;
        }

        zinv_pow = zinv;
        for (int j = 0; j < n_poles; ++j)
        {
            const std::complex<double> basis = -H_i * zinv_pow / Dprev;
            A_out(2 * i, nc + j)     = basis.real();
            A_out(2 * i + 1, nc + j) = basis.imag();
            zinv_pow *= zinv;
        }

        const std::complex<double> rhs = H_i / Dprev;
        b_out(2 * i)     = rhs.real();
        b_out(2 * i + 1) = rhs.imag();
    }
}

ComplexVectorFitResult ComplexVectorFit::fit(
    const std::vector<double> &omega,
    const std::vector<std::complex<double>> &response,
    int n_real_poles, int n_complex_pairs, double Ts,
    int max_iter, double tol)
{
    const int N = static_cast<int>(omega.size());
    if (N == 0 || response.size() != static_cast<size_t>(N))
        throw std::invalid_argument(
            "ComplexVectorFit::fit: omega and response must be non-empty and same size.");
    if (n_real_poles < 0 || n_complex_pairs < 0 || (n_real_poles + n_complex_pairs) <= 0)
        throw std::invalid_argument(
            "ComplexVectorFit::fit: need at least one pole (n_real_poles + n_complex_pairs > 0).");

    const int n_poles = n_real_poles + 2 * n_complex_pairs;
    const int n_unknowns = 2 * n_poles + 1;
    if (N < n_unknowns)
        throw std::invalid_argument(
            "ComplexVectorFit::fit: fewer frequency samples than unknown coefficients "
            "(system underdetermined).");

    std::vector<std::complex<double>> zinv_grid(N);
    for (int i = 0; i < N; ++i)
        zinv_grid[i] = std::polar(1.0, -omega[i] * Ts);

    std::vector<std::complex<double>> poles =
        initPoles(n_real_poles, n_complex_pairs, omega.back(), Ts);

    Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(n_poles, n_poles);
    for (int k = 0; k < n_poles - 1; ++k)
        companion(k + 1, k) = 1.0;
    Eigen::EigenSolver<Eigen::MatrixXd> es;

    const int nc = n_poles + 1;
    std::vector<double> num(nc, 0.0);
    std::vector<double> den(nc, 0.0);
    den[0] = 1.0;
    Eigen::VectorXd xPrev;
    std::vector<double> iterError;
    bool converged = false;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        Eigen::MatrixXd A;
        Eigen::VectorXd b;
        buildSystem(zinv_grid, response, poles, A, b);

        const Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

        for (int i = 0; i < nc; ++i)
            num[i] = x(i);
        den[0] = 1.0;
        for (int j = 1; j <= n_poles; ++j)
            den[j] = x(nc + j - 1);

        iterError.push_back(FreqDomainIdentifier::fitRMSE(omega, response, num, den, Ts));

        if (iter > 0)
        {
            const double maxDelta = (x - xPrev).cwiseAbs().maxCoeff();
            if (maxDelta < tol)
            {
                converged = true;
                xPrev = x;
                break;
            }
        }
        xPrev = x;

        for (int k = 0; k < n_poles; ++k)
            companion(0, k) = -den[k + 1];
        es.compute(companion, false);
        const auto &evals = es.eigenvalues();
        for (int k = 0; k < n_poles; ++k)
            poles[k] = reflectPole(evals(k));
    }

    for (int k = 0; k < n_poles; ++k)
        companion(0, k) = -den[k + 1];
    es.compute(companion, false);
    std::vector<std::complex<double>> finalPoles(n_poles);
    for (int k = 0; k < n_poles; ++k)
        finalPoles[k] = es.eigenvalues()(k);

    // Residues (diagnostic only, no model dependency): r_k = Nz(p_k)/Dz'(p_k), where
    // Nz(z) = sum_j num[j]*z^(n_poles-j), Dz(z) = sum_j den[j]*z^(n_poles-j) are num/den
    // re-expressed as polynomials in z (clearing the zinv powers) - H(1/z) = Nz(z)/Dz(z)
    // exactly, with Dz's roots being the companion matrix's eigenvalues found above.
    std::vector<std::complex<double>> residues(n_poles);
    for (int k = 0; k < n_poles; ++k)
    {
        const std::complex<double> p = finalPoles[k];
        std::complex<double> Nz(0.0, 0.0);
        for (int j = 0; j < nc; ++j)
            Nz = Nz * p + num[j];
        std::complex<double> Dzp(0.0, 0.0);
        for (int j = 0; j < n_poles; ++j)
            Dzp = Dzp * p + den[j] * static_cast<double>(n_poles - j);
        residues[k] = (std::abs(Dzp) > 1e-12) ? (Nz / Dzp) : std::complex<double>(0.0, 0.0);
    }

    TransferFunction model(num, den, Ts);
    return ComplexVectorFitResult{model, finalPoles, residues, iterError, converged};
}

} // namespace ctrl
