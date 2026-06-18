/**
 * @file ex85_mu_analysis.cpp
 * @brief Robustness Phase 3: structured singular value (mu) D-scaling analysis.
 *
 * Two checks against analytically-known values:
 *   1. A classic 2x2 off-diagonal example (Skogestad & Postlethwaite) with two
 *      independent SISO complex-full uncertainty blocks: mu = sqrt(|m12*m21|) = 1,
 *      strictly below the unstructured sigma_max(M) = 2.
 *   2. The Gang-of-Four fixture from ex84: peak mu of the closed loop under scaled
 *      output multiplicative uncertainty M(jw) = sigma_rel * T(jw), for a single
 *      ComplexFull block spanning the SISO output, reduces exactly to
 *      sigma_rel * ||T||_inf = sigma_rel / 3. The robust stability radius
 *      (largest sigma_rel keeping peak mu < 1) is therefore 3.
 *
 * Expected output:
 *   mu (D-scaled)      ~= 1.0   (vs. unstructured sigma_max = 2.0)
 *   peak mu @ sigma_rel=1 ~= 0.3333
 *   robust stability radius ~= 3.0
 *   [PASS] All checks passed.
 */

#include <ControllerToolbox.h>
#include <cmath>
#include <iomanip>
#include <iostream>

static ctrl::StateSpace firstOrderPlant(double a, double b, double ts)
{
    return ctrl::StateSpace((Eigen::MatrixXd(1, 1) << a).finished(),
                            (Eigen::MatrixXd(1, 1) << b).finished(),
                            (Eigen::MatrixXd(1, 1) << 1.0).finished(),
                            (Eigen::MatrixXd(1, 1) << 0.0).finished(), ts);
}

static ctrl::StateSpace staticController(double gain, double ts)
{
    return ctrl::StateSpace(Eigen::MatrixXd::Zero(1, 1),
                            Eigen::MatrixXd::Zero(1, 1),
                            Eigen::MatrixXd::Zero(1, 1),
                            (Eigen::MatrixXd(1, 1) << gain).finished(), ts);
}

int main()
{
    std::cout << "\n=== Robustness Phase 3: Structured Singular Value (mu) Analysis ===\n";
    std::cout << std::fixed << std::setprecision(4);

    bool ok = true;

    // ----- Part 1: classic 2x2 off-diagonal example -----------------------
    ctrl::UncertaintyStructure struc2;
    struc2.blocks = {
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1},
        ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1},
    };
    Eigen::MatrixXcd M(2, 2);
    M << 0.0, 2.0,
         0.5, 0.0;

    const auto bounds = ctrl::computeMu({M}, struc2, /*compute_lower_bound=*/true);
    const auto &mu2 = bounds[0];

    std::cout << "\n-- Classic 2x2 off-diagonal example --\n";
    std::cout << "  sigma_max(M) (unstructured) = 2.0000\n";
    std::cout << "  mu upper bound (D-scaled)   = " << mu2.upper << "\n";
    std::cout << "  mu lower bound (rho(M))     = " << mu2.lower << "\n";

    const bool mu2_ok = std::abs(mu2.upper - 1.0) < 1e-3 && std::abs(mu2.lower - 1.0) < 1e-9;
    std::cout << "  [mu=1, tighter than sigma_max] " << (mu2_ok ? "PASS" : "FAIL") << "\n";
    ok = ok && mu2_ok;

    // ----- Part 2: closed-loop peak mu + robust stability radius ----------
    const double Ts = 0.1;
    const auto G = firstOrderPlant(0.6, 0.4, Ts);
    const auto K = staticController(0.5, Ts);

    ctrl::UncertaintyStructure struc1;
    struc1.blocks = {ctrl::UncertaintyBlock{ctrl::UncertaintyBlock::Type::ComplexFull, 1, 1}};

    const auto peak = ctrl::peakMu(G, K, struc1, /*sigma_rel=*/1.0, /*freq_points=*/50, /*omega_min=*/1e-4);
    const double radius = ctrl::robustStabilityRadius(G, K, struc1, /*sigma_max=*/5.0);

    std::cout << "\n-- Closed-loop peak mu (single ComplexFull block) --\n";
    std::cout << "  peak mu @ sigma_rel=1 (== ||T||_inf) = " << peak.peak.upper << "\n";
    std::cout << "  peak frequency [rad/s]                = " << peak.peak_omega_rad_s << "\n";
    std::cout << "  robust stability radius (1/||T||_inf) = " << radius << "\n";

    const bool peak_ok   = std::abs(peak.peak.upper - 1.0 / 3.0) < 1e-3;
    const bool radius_ok = std::abs(radius - 3.0) < 1e-2;
    std::cout << "  [peak == ||T||_inf]      " << (peak_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "  [radius == 1/||T||_inf]  " << (radius_ok ? "PASS" : "FAIL") << "\n";
    ok = ok && peak_ok && radius_ok;

    if (ok)
    {
        std::cout << "\n[PASS] All checks passed.\n";
        return 0;
    }
    std::cout << "\n[FAIL] One or more checks failed.\n";
    return 1;
}
