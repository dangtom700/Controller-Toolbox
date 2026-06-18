/**
 * @file ex83_robustness_mc.cpp
 * @brief Robustness Phase 1: Monte-Carlo closed-loop robustness analysis.
 *
 * Spawns an ensemble of perturbed first-order plants around a nominal model,
 * closes each one around a fixed proportional controller, and aggregates
 * closed-loop robustness statistics (stability probability, gain/phase margins,
 * peak sensitivity, step IAE, nu-gap from nominal).
 *
 *   Nominal plant:  x[k+1] = 0.6 x[k] + 0.4 u[k],  y = x   (open-loop stable)
 *   Controller:     u = 0.5 * e   (static gain; closed-loop pole 0.4)
 *
 * Two ensembles are compared:
 *   (a) small uncertainty (sigma_A = 0.05)  -> expect ~0% instability
 *   (b) large uncertainty (sigma_A = 0.40)  -> expect a non-zero instability tail
 *
 * Expected output:
 *   [A] small sigma: instability_probability == 0, finite sensitivity peak
 *   [B] large sigma: wider sensitivity spread, possible instability tail
 *   [PASS] All checks passed.
 */

#include <ControllerToolbox.h>
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

static void printResult(const char* tag, const ctrl::MonteCarloResult& r)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  [" << tag << "] samples=" << r.n_samples
              << "  unstable=" << r.n_unstable
              << "  P(unstable)=" << r.instability_probability << "\n";
    std::cout << "        ||S||_inf  mean=" << r.sensitivity_peak_stats.mean
              << "  p95="  << r.sensitivity_peak_stats.p95
              << "  worst=" << r.sensitivity_peak_stats.worst << "\n";
    std::cout << "        GM[dB]     mean=" << r.gm_stats.mean
              << "  worst=" << r.gm_stats.worst << "\n";
    std::cout << "        nu-gap     mean=" << r.nu_gap_stats.mean
              << "  worst=" << r.nu_gap_stats.worst << "\n";
}

int main()
{
    const double Ts = 0.1;
    const auto nominal = firstOrderPlant(0.6, 0.4, Ts);
    const auto ctl     = staticController(0.5, Ts);

    std::cout << "\n=== Robustness Phase 1: Monte-Carlo analysis ===\n";

    const auto res_small = ctrl::monteCarloAnalysis(nominal, ctl, 500, /*sigma_A=*/0.05,
                                                    0.0, 0.0, 0.0, /*seed=*/42);
    printResult("A small sigma", res_small);

    const auto res_large = ctrl::monteCarloAnalysis(nominal, ctl, 500, /*sigma_A=*/0.40,
                                                    0.0, 0.0, 0.0, /*seed=*/42);
    printResult("B large sigma", res_large);

    // ----- Checks --------------------------------------------------------
    bool a_ok = res_small.n_samples == 500 &&
                res_small.n_unstable == 0 &&
                std::isfinite(res_small.sensitivity_peak_stats.mean) &&
                res_small.sensitivity_peak_stats.mean > 0.0 &&
                res_small.nu_gap_stats.worst >= 0.0 &&
                res_small.nu_gap_stats.worst < 1.0;

    // Larger uncertainty must broaden the sensitivity spread and the nu-gap.
    bool b_ok = res_large.n_samples == 500 &&
                res_large.sensitivity_peak_stats.p95 >= res_small.sensitivity_peak_stats.p95 &&
                res_large.nu_gap_stats.mean >= res_small.nu_gap_stats.mean &&
                res_large.instability_probability >= 0.0 &&
                res_large.instability_probability <= 1.0;

    std::cout << "  [A] " << (a_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "  [B] " << (b_ok ? "PASS" : "FAIL") << "\n";

    if (a_ok && b_ok)
    {
        std::cout << "\n[PASS] All checks passed.\n";
        return 0;
    }
    std::cout << "\n[FAIL] One or more checks failed.\n";
    return 1;
}
