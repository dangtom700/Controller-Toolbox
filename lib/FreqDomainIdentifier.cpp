#include "FreqDomainIdentifier.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

void FreqDomainIdentifier::buildLevySystem(const std::vector<double> &freqs,
                                            const std::vector<std::complex<double>> &response,
                                            int num_order, int den_order, double Ts,
                                            const std::vector<double> &weights,
                                            Eigen::MatrixXd &Phi, Eigen::VectorXd &y)
{
    if (freqs.size() != response.size())
        throw std::invalid_argument(
            "FreqDomainIdentifier::buildLevySystem: freqs and response must have the same length.");
    if (!weights.empty() && weights.size() != freqs.size())
        throw std::invalid_argument(
            "FreqDomainIdentifier::buildLevySystem: weights must be empty or match freqs in length.");

    const int n_unknowns = num_order + 1 + den_order;
    const std::size_t M  = freqs.size();
    if (static_cast<int>(M) < n_unknowns)
        throw std::invalid_argument(
            "FreqDomainIdentifier::buildLevySystem: fewer frequency samples than unknown "
            "coefficients (system underdetermined).");

    // Build the real-stacked linear system Phi*x = y from the Levy residual
    // N(zinv_k) - H_data,k * D(zinv_k) = 0, with D's constant term fixed to 1:
    //   sum_i b_i*zinv_k^i - sum_j a_j*H_data,k*zinv_k^j = H_data,k
    // Each sample's stacked row pair is scaled by weights[k] (1.0 when unweighted).
    Phi.resize(2 * M, n_unknowns);
    y.resize(2 * M);

    for (std::size_t k = 0; k < M; ++k)
    {
        const std::complex<double> zinv = std::polar(1.0, -freqs[k] * Ts);
        const std::complex<double> &Hk  = response[k];
        const double w = weights.empty() ? 1.0 : weights[k];

        std::complex<double> zinv_pow(1.0, 0.0); // zinv^0
        for (int i = 0; i <= num_order; ++i)
        {
            Phi(2 * k,     i) = w * zinv_pow.real();
            Phi(2 * k + 1, i) = w * zinv_pow.imag();
            zinv_pow *= zinv;
        }

        zinv_pow = zinv; // zinv^1
        for (int j = 1; j <= den_order; ++j)
        {
            const std::complex<double> coeff = -(Hk * zinv_pow);
            Phi(2 * k,     num_order + j) = w * coeff.real();
            Phi(2 * k + 1, num_order + j) = w * coeff.imag();
            zinv_pow *= zinv;
        }

        y(2 * k)     = w * Hk.real();
        y(2 * k + 1) = w * Hk.imag();
    }
}

double FreqDomainIdentifier::fitRMSE(const std::vector<double> &freqs,
                                      const std::vector<std::complex<double>> &response,
                                      const std::vector<double> &num, const std::vector<double> &den,
                                      double Ts)
{
    const std::size_t M = freqs.size();
    const int num_order = static_cast<int>(num.size()) - 1;
    const int den_order = static_cast<int>(den.size()) - 1;

    double sse = 0.0;
    for (std::size_t k = 0; k < M; ++k)
    {
        const std::complex<double> zinv = std::polar(1.0, -freqs[k] * Ts);

        std::complex<double> N(0.0, 0.0), zinv_pow(1.0, 0.0);
        for (int i = 0; i <= num_order; ++i)
        {
            N += num[i] * zinv_pow;
            zinv_pow *= zinv;
        }

        std::complex<double> D(1.0, 0.0);
        zinv_pow = zinv;
        for (int j = 1; j <= den_order; ++j)
        {
            D += den[j] * zinv_pow;
            zinv_pow *= zinv;
        }

        const std::complex<double> diff = response[k] - N / D;
        sse += std::norm(diff);
    }
    return std::sqrt(sse / static_cast<double>(M));
}

FreqDomainFitResult FreqDomainIdentifier::fitLevy(const std::vector<double> &freqs,
                                                   const std::vector<std::complex<double>> &response,
                                                   int num_order, int den_order, double Ts)
{
    Eigen::MatrixXd Phi;
    Eigen::VectorXd y;
    buildLevySystem(freqs, response, num_order, den_order, Ts, {}, Phi, y);

    const int n_unknowns = num_order + 1 + den_order;
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Phi);
    const Eigen::VectorXd x = qr.solve(y);
    const bool full_rank    = qr.rank() == n_unknowns;

    std::vector<double> num(num_order + 1);
    for (int i = 0; i <= num_order; ++i)
        num[i] = x(i);

    std::vector<double> den(den_order + 1);
    den[0] = 1.0;
    for (int j = 1; j <= den_order; ++j)
        den[j] = x(num_order + j);

    TransferFunction tf(num, den, Ts);
    const double rmse = fitRMSE(freqs, response, num, den, Ts);

    return FreqDomainFitResult{std::move(tf), rmse, full_rank};
}

} // namespace ctrl
