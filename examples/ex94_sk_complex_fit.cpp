/**
 * @file ex94_sk_complex_fit.cpp
 * @brief Phase 3 (FD1): SK-reweighted complex-response fitting vs. a one-shot Levy fit.
 *
 * Fits a lightly-damped 2nd-order resonance from a noisy frequency-response sample set,
 * comparing FreqDomainIdentifier::fitLevy (one-shot) against SKFit::fitSK (iteratively
 * reweighted) on the same data - SK reweighting should reduce the RMSE relative to the
 * unweighted Levy baseline.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>
#include <random>

int main()
{
    const double Ts = 0.1;
    const double r = 0.97, theta = 0.6; // lightly-damped pole pair r*exp(+-j*theta)
    const double a1 = -2.0 * r * std::cos(theta);
    const double a2 = r * r;
    ctrl::TransferFunction tf_true({0.0, 1.0 - r, 0.0}, {1.0, a1, a2}, Ts);
    ctrl::StateSpace sys = ctrl::tf2ss(tf_true);

    std::vector<double> freqs;
    for (int i = 1; i <= 40; ++i) freqs.push_back(0.5 * i);
    auto response = ctrl::SystemAnalysis::getFrequencyResponse(sys, freqs);

    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.01);
    for (auto &h : response) h += std::complex<double>(noise(rng), noise(rng));

    const auto levyResult = ctrl::FreqDomainIdentifier::fitLevy(freqs, response, 2, 2, Ts);
    const auto skResult   = ctrl::SKFit::fitSK(freqs, response, 2, 2, Ts);

    std::printf("Levy (one-shot)   rmse = %.5f\n", levyResult.rmse);
    std::printf("SK   (reweighted) rmse = %.5f after %zu iterations (converged=%s)\n",
                skResult.iterCost.back(), skResult.iterCost.size(),
                skResult.converged ? "yes" : "no");
    std::cout << "Per-iteration RMSE: ";
    for (double c : skResult.iterCost) std::printf("%.5f ", c);
    std::cout << "\n";

    const bool ok = std::isfinite(skResult.iterCost.back())
                  && skResult.iterCost.back() < levyResult.rmse;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
