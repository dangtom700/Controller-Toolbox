/**
 * @file ex106_constrained_tuning.cpp
 * @brief Phase 3 Roadmap Phase 2 (MO3): tuning a PID subject to a closed-loop pole constraint.
 *
 * AutoTuner/TunerSuite only support box bounds on the parameters themselves today.
 * tuneConstrained adds a general nonlinear constraint - here, that the closed-loop system's
 * dominant pole magnitude stays below a stability-margin threshold - via an exterior-penalty
 * wrapper around the existing AutoTuner CMA-ES search.
 */

#include "ControllerToolbox.h"
#include <iostream>

namespace
{
const double Ts = 0.05;
const ctrl::TransferFunction kPlantTf({0.0, 1.0}, {1.0, -0.9}, Ts);

double trackingCost(const Eigen::VectorXd &gains)
{
    ctrl::PIDParams pp; pp.Kp = gains(0); pp.Ki = gains(1); pp.Kd = 0.0;
    ctrl::DiscretePID pid(pp, Ts);
    const auto sys = ctrl::tf2ss(kPlantTf);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(sys.stateSize());

    double cost = 0.0, y = 0.0;
    for (int k = 0; k < 100; ++k)
    {
        const double e = 1.0 - y;
        const double u = pid.compute(e);
        Eigen::VectorXd uv(1); uv << u;
        y = ctrl::ssStep(sys, x, uv)(0);
        cost += e * e;
    }
    return cost;
}

// Closed-loop dominant pole magnitude for this 1st-order plant + PI controller, via the
// standard PID+plant augmented closed-loop state matrix (plant state + PID integrator state).
double closedLoopPoleMagnitude(const Eigen::VectorXd &gains)
{
    const double a = kPlantTf.den[1]; // plant pole (open loop): x[k+1] = -a*x[k] + b*u[k]
    const double b = kPlantTf.num[1];
    const double Kp = gains(0), Ki = gains(1);

    // PI: u[k] = Kp*e[k] + I[k], I[k+1] = I[k] + Ki*Ts*e[k+1], e = r - y, y = plant output = x.
    // Augmented state z = [x, I]. With r held at a constant reference, treat r=0 for the pole
    // analysis (poles are reference-independent for a linear closed loop).
    Eigen::Matrix2d Acl;
    Acl(0, 0) = -a - b * Kp;
    Acl(0, 1) = b;
    Acl(1, 0) = -Ki * Ts * (-a - b * Kp);
    Acl(1, 1) = 1.0 - Ki * Ts * b;

    return Acl.eigenvalues().cwiseAbs().maxCoeff();
}
} // namespace

int main()
{
    const double poleLimit = 0.85;

    ctrl::ConstrainedTuneParams cp;
    cp.constraints = [poleLimit](const Eigen::VectorXd &gains) {
        return Eigen::VectorXd::Constant(1, closedLoopPoleMagnitude(gains) - poleLimit);
    };
    cp.outer_iters = 6;

    ctrl::AutoTunerParams atp;
    atp.n = 2;
    atp.lower = Eigen::Vector2d(0.0, 0.0);
    atp.upper = Eigen::Vector2d(10.0, 10.0);
    ctrl::AutoTuner tuner(atp);

    auto optimizerRun = [&](const ctrl::AutoTuner::CostFn &c, const Eigen::VectorXd &x0) {
        return tuner.tune(c, x0);
    };

    const auto result = ctrl::tuneConstrained(optimizerRun, trackingCost, cp,
                                               Eigen::Vector2d(1.0, 0.5));

    const double finalPole = closedLoopPoleMagnitude(result.params);
    std::cout << "Tuned gains: Kp=" << result.params(0) << " Ki=" << result.params(1) << "\n";
    std::cout << "Closed-loop dominant pole magnitude: " << finalPole
              << " (limit " << poleLimit << ")\n";
    std::cout << "Tracking cost: " << result.cost << "\n";

    const bool ok = std::isfinite(result.cost) && finalPole <= poleLimit + 0.05;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
