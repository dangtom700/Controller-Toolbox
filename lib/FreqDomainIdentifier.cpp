#include "FreqDomainIdentifier.h"
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace ctrl {

FreqDomainFitResult FreqDomainIdentifier::fitLevy(const std::vector<double> &freqs,
                                                   const std::vector<std::complex<double>> &response,
                                                   int num_order, int den_order, double Ts)
{
    if (freqs.size() != response.size())
        throw std::invalid_argument(
            "FreqDomainIdentifier::fitLevy: freqs and response must have the same length.");

    const int n_unknowns = num_order + 1 + den_order;
    const std::size_t M  = freqs.size();
    if (static_cast<int>(M) < n_unknowns)
        throw std::invalid_argument(
            "FreqDomainIdentifier::fitLevy: fewer frequency samples than unknown coefficients "
            "(system underdetermined).");

    // Build the real-stacked linear system Phi*x = y from the Levy residual
    // N(zinv_k) - H_data,k * D(zinv_k) = 0, with D's constant term fixed to 1:
    //   sum_i b_i*zinv_k^i - sum_j a_j*H_data,k*zinv_k^j = H_data,k
    Eigen::MatrixXd Phi(2 * M, n_unknowns);
    Eigen::VectorXd y(2 * M);

    for (std::size_t k = 0; k < M; ++k)
    {
        const std::complex<double> zinv = std::polar(1.0, -freqs[k] * Ts);
        const std::complex<double> &Hk  = response[k];

        std::complex<double> zinv_pow(1.0, 0.0); // zinv^0
        for (int i = 0; i <= num_order; ++i)
        {
            Phi(2 * k,     i) = zinv_pow.real();
            Phi(2 * k + 1, i) = zinv_pow.imag();
            zinv_pow *= zinv;
        }

        zinv_pow = zinv; // zinv^1
        for (int j = 1; j <= den_order; ++j)
        {
            const std::complex<double> coeff = -(Hk * zinv_pow);
            Phi(2 * k,     num_order + j) = coeff.real();
            Phi(2 * k + 1, num_order + j) = coeff.imag();
            zinv_pow *= zinv;
        }

        y(2 * k)     = Hk.real();
        y(2 * k + 1) = Hk.imag();
    }

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

    return FreqDomainFitResult{std::move(tf), std::sqrt(sse / static_cast<double>(M)), full_rank};
}

} // namespace ctrl
