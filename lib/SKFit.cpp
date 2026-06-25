#include "SKFit.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl {

SKFitResult SKFit::fitSK(const std::vector<double> &freqs,
                          const std::vector<std::complex<double>> &response,
                          int num_order, int den_order, double Ts,
                          int max_iter, double tol)
{
    if (freqs.size() != response.size())
        throw std::invalid_argument("SKFit::fitSK: freqs and response must have the same length.");

    const int n_unknowns = num_order + 1 + den_order;
    const std::size_t M  = freqs.size();
    if (static_cast<int>(M) < n_unknowns)
        throw std::invalid_argument(
            "SKFit::fitSK: fewer frequency samples than unknown coefficients (system underdetermined).");

    constexpr double kEpsD = 1e-9; // floor on |D_prev| to avoid divide-by-near-zero weights

    // SKFitResult holds a TransferFunction (no default constructor), so it cannot be
    // default-constructed and incrementally filled in across the loop - accumulate into
    // plain locals instead and build the result via aggregate-init only at the return.
    std::vector<double> weights; // empty = unweighted (iteration 0)
    std::vector<double> num(num_order + 1);
    std::vector<double> den(den_order + 1, 0.0);
    den[0] = 1.0;
    Eigen::VectorXd xPrev;
    std::vector<double> iterCost;
    bool converged = false;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        Eigen::MatrixXd Phi;
        Eigen::VectorXd y;
        FreqDomainIdentifier::buildLevySystem(freqs, response, num_order, den_order, Ts,
                                               weights, Phi, y);

        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Phi);
        const Eigen::VectorXd x = qr.solve(y);

        for (int i = 0; i <= num_order; ++i)
            num[i] = x(i);
        den[0] = 1.0;
        for (int j = 1; j <= den_order; ++j)
            den[j] = x(num_order + j);

        iterCost.push_back(FreqDomainIdentifier::fitRMSE(freqs, response, num, den, Ts));

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

        // Reweight for the next iteration: w_k = 1 / max(|D_prev(zinv_k)|, eps).
        weights.assign(M, 1.0);
        for (std::size_t k = 0; k < M; ++k)
        {
            const std::complex<double> zinv = std::polar(1.0, -freqs[k] * Ts);
            std::complex<double> D(1.0, 0.0), zinv_pow = zinv;
            for (int j = 1; j <= den_order; ++j)
            {
                D += den[j] * zinv_pow;
                zinv_pow *= zinv;
            }
            weights[k] = 1.0 / std::max(std::abs(D), kEpsD);
        }
    }

    return SKFitResult{TransferFunction(num, den, Ts), iterCost, converged};
}

} // namespace ctrl
