/**
 * @file ex114_complex_vector_fit.cpp
 * @brief Phase 3 (FD2): complex-conjugate-pole Vector Fitting vs. a one-shot Levy fit.
 *
 * Fits a 3-resonance system (3 lightly-damped complex-conjugate pole pairs) from a noisy
 * frequency-response sample set with ComplexVectorFit::fit, comparing against
 * FreqDomainIdentifier::fitLevy (one-shot, same order) on the same data.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <random>

namespace
{
std::vector<double> polyMulPair(const std::vector<double> &p, double a1, double a2)
{
    std::vector<double> result(p.size() + 2, 0.0);
    for (std::size_t i = 0; i < p.size(); ++i)
    {
        result[i]     += p[i];
        result[i + 1] += p[i] * a1;
        result[i + 2] += p[i] * a2;
    }
    return result;
}
} // namespace

int main()
{
    const double Ts = 0.1;
    const std::vector<std::pair<double, double>> specs{{0.99, 0.4}, {0.985, 0.55}, {0.99, 0.75}};

    std::vector<double> den{1.0};
    for (const auto &pr : specs)
        den = polyMulPair(den, -2.0 * pr.first * std::cos(pr.second), pr.first * pr.first);
    std::vector<double> num(den.size(), 0.0);
    num[1] = 0.05;

    const ctrl::TransferFunction tf_true(num, den, Ts);
    const auto sys = ctrl::tf2ss(tf_true);

    std::vector<double> freqs;
    for (int i = 1; i <= 80; ++i) freqs.push_back(0.25 * i);
    auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    std::mt19937 rng(11);
    std::normal_distribution<double> noise(0.0, 0.02);
    for (auto &h : response) h += std::complex<double>(noise(rng), noise(rng));

    const auto cvfResult  = ctrl::ComplexVectorFit::fit(freqs, response, 0, 3, Ts, 30);
    const auto levyResult = ctrl::FreqDomainIdentifier::fitLevy(freqs, response, 6, 6, Ts);

    std::printf("Levy (one-shot, order 6)    rmse = %.5f\n", levyResult.rmse);
    std::printf("ComplexVectorFit            rmse = %.5f after %zu iterations (converged=%s)\n",
                cvfResult.iterError.back(), cvfResult.iterError.size(),
                cvfResult.converged ? "yes" : "no");
    std::cout << "Recovered poles (magnitude @ angle [rad]):\n";
    for (const auto &p : cvfResult.poles)
        std::printf("  %.4f @ %.4f\n", std::abs(p), std::arg(p));

    const bool ok = std::isfinite(cvfResult.iterError.back())
                  && cvfResult.iterError.back() < 0.5 * levyResult.rmse;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
