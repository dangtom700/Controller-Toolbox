/**
 * @file ex86_worst_case.cpp
 * @brief Robustness Phase 4: CMA-ES worst-case parameter search.
 *
 * Treats the plant's pole location as an uncertain parameter and uses
 * findWorstCaseSensitivity / findWorstCaseIAE / findWorstCase to find the value, inside
 * a relative search box around the nominal, that degrades closed-loop performance the
 * most against a fixed controller.
 *
 *   Nominal plant:  x[k+1] = 0.6 x[k] + 0.4 u[k],  y = x
 *   Controller:     u = 0.5 * e   (closed-loop pole 0.4)
 *
 * Expected output:
 *   [A] worst-case ||S||_inf  >= nominal ||S||_inf
 *   [B] worst-case step IAE   >= nominal step IAE
 *   [C] bounded search stays within the supplied parameter box
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

int main()
{
    const double Ts = 0.1;
    const auto ctl = staticController(0.5, Ts);
    auto plant_factory = [Ts](const Eigen::VectorXd& p) {
        return firstOrderPlant(p(0), 0.4, Ts);
    };

    const Eigen::VectorXd nominal = (Eigen::VectorXd(1) << 0.6).finished();
    const Eigen::VectorXd sigma   = (Eigen::VectorXd(1) << 0.3).finished();

    const auto nominal_plant  = plant_factory(nominal);
    const auto nominal_sample = ctrl::evaluateSample(0, nominal_plant, ctl, nominal_plant);

    std::cout << "\n=== Robustness Phase 4: CMA-ES worst-case parameter search ===\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Nominal: a=" << nominal(0)
              << "  ||S||_inf=" << nominal_sample.hinf_sensitivity
              << "  IAE=" << nominal_sample.iae << "\n";

    ctrl::WorstCaseSearchParams wp;
    wp.max_evals = 400;
    wp.seed = 7;

    const auto res_sens = ctrl::findWorstCaseSensitivity(plant_factory, ctl, nominal, sigma, {}, {}, wp);
    std::cout << "  [A] worst-case sensitivity: a=" << res_sens.worst_params(0)
              << "  ||S||_inf=" << res_sens.worst_cost
              << "  evals=" << res_sens.n_evals << "\n";

    const auto res_iae = ctrl::findWorstCaseIAE(plant_factory, ctl, nominal, sigma, {}, {}, 20.0, wp);
    std::cout << "  [B] worst-case IAE:         a=" << res_iae.worst_params(0)
              << "  IAE=" << res_iae.worst_cost
              << "  evals=" << res_iae.n_evals << "\n";

    const Eigen::VectorXd lower = (Eigen::VectorXd(1) << 0.55).finished();
    const Eigen::VectorXd upper = (Eigen::VectorXd(1) << 0.65).finished();
    const auto res_bounded =
        ctrl::findWorstCaseSensitivity(plant_factory, ctl, nominal, (Eigen::VectorXd(1) << 0.5).finished(),
                                       lower, upper, wp);
    std::cout << "  [C] bounded search:         a=" << res_bounded.worst_params(0)
              << "  (box [" << lower(0) << ", " << upper(0) << "])\n";

    bool a_ok = std::isfinite(res_sens.worst_cost) &&
                res_sens.worst_cost >= nominal_sample.hinf_sensitivity - 1e-9;
    bool b_ok = res_iae.worst_cost >= nominal_sample.iae - 1e-9;
    bool c_ok = res_bounded.worst_params(0) >= lower(0) - 1e-9 &&
                res_bounded.worst_params(0) <= upper(0) + 1e-9;

    std::cout << "  [A] " << (a_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "  [B] " << (b_ok ? "PASS" : "FAIL") << "\n";
    std::cout << "  [C] " << (c_ok ? "PASS" : "FAIL") << "\n";

    if (a_ok && b_ok && c_ok)
    {
        std::cout << "\n[PASS] All checks passed.\n";
        return 0;
    }
    std::cout << "\n[FAIL] One or more checks failed.\n";
    return 1;
}
