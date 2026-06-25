/**
 * @file ex105_nsga2.cpp
 * @brief Phase 3 Roadmap Phase 2 (MO1): NSGA-II tuning a PID for settling time vs control effort.
 *
 * TunerSuite's cost functions give single-objective costs today; NSGA2 returns the actual
 * tradeoff curve instead of forcing a weighted-sum compromise upfront.
 */

#include "ControllerToolbox.h"
#include <iostream>

namespace
{
// Simulate a fixed first-order plant under a PID(Kp, Ki) and return [settling-error-energy,
// control-effort-energy] for a unit step reference.
Eigen::VectorXd simulate(const Eigen::VectorXd &gains)
{
    ctrl::PIDParams pp;
    pp.Kp = gains(0);
    pp.Ki = gains(1);
    pp.Kd = 0.0;
    const double Ts = 0.05;
    ctrl::DiscretePID pid(pp, Ts);

    const ctrl::TransferFunction tf({0.0, 1.0}, {1.0, -0.9}, Ts);
    const auto sys = ctrl::tf2ss(tf);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(sys.stateSize());

    double errEnergy = 0.0, effortEnergy = 0.0;
    double y = 0.0;
    for (int k = 0; k < 100; ++k)
    {
        const double e = 1.0 - y;
        const double u = pid.compute(e);
        Eigen::VectorXd uv(1); uv << u;
        y = ctrl::ssStep(sys, x, uv)(0);
        errEnergy += e * e;
        effortEnergy += u * u;
    }
    Eigen::VectorXd obj(2);
    obj(0) = errEnergy;
    obj(1) = effortEnergy;
    return obj;
}
} // namespace

int main()
{
    ctrl::NSGA2Params params;
    params.n_dim = 2;
    params.n_objectives = 2;
    params.population = 40;
    params.max_gen = 40;
    params.lower = Eigen::Vector2d(0.1, 0.0);
    params.upper = Eigen::Vector2d(5.0, 5.0);

    ctrl::NSGA2 nsga(params);
    const auto result = nsga.optimize(simulate);

    std::cout << "Pareto front (" << result.front_params.rows() << " points):\n";
    for (int i = 0; i < std::min<int>(5, static_cast<int>(result.front_params.rows())); ++i)
    {
        std::cout << "  Kp=" << result.front_params(i, 0) << " Ki=" << result.front_params(i, 1)
                  << "  -> errEnergy=" << result.front_objectives(i, 0)
                  << " effortEnergy=" << result.front_objectives(i, 1) << "\n";
    }

    const bool ok = result.front_params.rows() >= 1 && result.front_objectives.allFinite();
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
