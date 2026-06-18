/**
 * @file ex84_gang_of_four.cpp
 * @brief Robustness Phase 2: Gang of Four + Disk Margin via SystemAnalysis block algebra.
 *
 * Builds the closed-loop Gang of Four (S, T, GS, KS) for a unity-feedback loop using
 * the new series()/parallel()/feedback() block-algebra primitives, then derives the
 * simultaneous gain/phase disk margin from the same loop.
 *
 *   Plant:      x[k+1] = 0.6 x[k] + 0.4 u[k],  y = x   (open-loop stable)
 *   Controller: u = 0.5 * e   (static gain; closed-loop pole 0.4)
 *
 * Expected output:
 *   S + T == I pointwise (push-through identity)
 *   ||T||_inf == T(DC) = 1/3 (monotonic 1st-order loop, peak at DC)
 *   ||S||_inf > 1 (waterbed effect away from DC - not a bug)
 *   disk margin alpha == 1/||S||_inf
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
    const double Ts = 0.1;
    const auto G = firstOrderPlant(0.6, 0.4, Ts);
    const auto K = staticController(0.5, Ts);

    std::cout << "\n=== Robustness Phase 2: Gang of Four + Disk Margin ===\n";

    const auto g4  = ctrl::SystemAnalysis::gangOfFour(G, K);
    const auto g4n = ctrl::SystemAnalysis::gangOfFourNorms(g4);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  ||S||_inf  = " << g4n.norm_S  << "\n";
    std::cout << "  ||T||_inf  = " << g4n.norm_T  << "\n";
    std::cout << "  ||GS||_inf = " << g4n.norm_GS << "\n";
    std::cout << "  ||KS||_inf = " << g4n.norm_KS << "\n";

    const auto L  = ctrl::SystemAnalysis::series(K, G);
    const auto dm = ctrl::SystemAnalysis::calculateDiskMargin(L);

    std::cout << "  disk margin alpha       = " << dm.alpha << "\n";
    std::cout << "  simultaneous gain margin = " << dm.gain_margin << " (linear)\n";
    std::cout << "  simultaneous phase margin = " << dm.phase_margin_deg << " deg\n";

    // ----- Checks --------------------------------------------------------
    const std::vector<double> freqs{0.5, 3.0, 10.0, 25.0};
    const auto S_resp = ctrl::SystemAnalysis::getFrequencyResponse(g4.S, freqs);
    const auto T_resp = ctrl::SystemAnalysis::getFrequencyResponse(g4.T, freqs);
    bool st_identity_ok = true;
    for (std::size_t i = 0; i < freqs.size(); ++i)
    {
        const auto sum = S_resp[i] + T_resp[i];
        if (std::abs(sum.real() - 1.0) > 1e-9 || std::abs(sum.imag()) > 1e-9)
            st_identity_ok = false;
    }

    const bool norm_t_ok = std::abs(g4n.norm_T - 1.0 / 3.0) < 1e-3;
    const bool norm_s_ok = g4n.norm_S > 1.0;
    const bool disk_ok   = std::abs(dm.alpha - 1.0 / g4n.norm_S) < 1e-6;

    std::cout << "  [S+T=I]   " << (st_identity_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "  [norm_T]  " << (norm_t_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "  [norm_S]  " << (norm_s_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "  [disk]    " << (disk_ok ? "PASS" : "FAIL") << "\n";

    if (st_identity_ok && norm_t_ok && norm_s_ok && disk_ok)
    {
        std::cout << "\n[PASS] All checks passed.\n";
        return 0;
    }
    std::cout << "\n[FAIL] One or more checks failed.\n";
    return 1;
}
