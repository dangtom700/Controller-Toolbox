/**
 * ex89_resonant_controller.cpp
 * Additional Controller Types: ResonantController multi-harmonic disturbance rejection.
 *
 * Demonstrates the composition pattern from
 * docs/superpowers/specs/2026-06-24-resonant-notch-pll-controllers-design.md: a
 * ControllerStack(Additive) holds a base PID plus one ResonantController per harmonic. A
 * periodic process disturbance (5th + 7th harmonic of a 0.5 Hz fundamental) enters at the
 * plant input; the PID alone leaves a steady-state ripple, the stack with the resonant
 * correctors added largely cancels it - the stack, not a one-off manual sum, does the work.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

namespace
{

double runScenario(const ctrl::StateSpace &sys, double Ts, bool withResonant)
{
    ctrl::PIDParams pp;
    pp.Kp = 2.0; pp.Ki = 5.0; pp.Kd = 0.0;
    pp.uMin = -1e6; pp.uMax = 1e6;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::ControllerStack stack(ctrl::StackMode::Additive, Ts);
    stack.addController(pid, "PID-base", 1.0);

    if (withResonant)
    {
        ctrl::ResonantParams rp5;
        rp5.targetFreqHz = 2.5; // 5th harmonic of the 0.5Hz fundamental
        rp5.dampingRadPerSec = 5.0;
        rp5.Kr = 200.0;
        auto rc5 = std::make_shared<ctrl::ResonantController>(rp5, Ts);

        ctrl::ResonantParams rp7;
        rp7.targetFreqHz = 3.5; // 7th harmonic
        rp7.dampingRadPerSec = 5.0;
        rp7.Kr = 200.0;
        auto rc7 = std::make_shared<ctrl::ResonantController>(rp7, Ts);

        stack.addController(rc5, "5th-harmonic-RC", 1.0);
        stack.addController(rc7, "7th-harmonic-RC", 1.0);
    }

    const double r = 1.0;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(1);
    double y = 0.0;
    const int N = 60000;                  // 60s = 30 cycles of the 0.5Hz fundamental
    const int samplesPerFundCycle = 2000;  // 1/0.5Hz / Ts
    double maxAbsErrLastCycle = 0.0;

    for (int k = 0; k < N; ++k)
    {
        const double e = r - y;
        const double u = stack.compute(e);
        const double d = 0.3 * std::sin(2.0 * M_PI * 2.5 * k * Ts)
                        + 0.2 * std::sin(2.0 * M_PI * 3.5 * k * Ts);
        Eigen::VectorXd uv(1); uv(0) = u + d; // process disturbance, enters with the control input
        y = ctrl::ssStep(sys, x, uv)(0);
        if (k >= N - samplesPerFundCycle)
            maxAbsErrLastCycle = std::max(maxAbsErrLastCycle, std::fabs(e));
    }
    return maxAbsErrLastCycle;
}

} // namespace

int main()
{
    const double Ts = 1e-3;
    ctrl::StateSpace sys_c(
        Eigen::MatrixXd::Constant(1, 1, -1.0),
        Eigen::MatrixXd::Constant(1, 1,  1.0),
        Eigen::MatrixXd::Constant(1, 1,  1.0),
        Eigen::MatrixXd::Zero(1, 1), 0.0);
    const ctrl::StateSpace sys = ctrl::c2d(sys_c, Ts, ctrl::C2dMethod::ZOH);

    const double ripplePidOnly = runScenario(sys, Ts, false);
    const double rippleWithRC  = runScenario(sys, Ts, true);

    std::cout << "PID-only steady-state ripple:     " << ripplePidOnly << "\n";
    std::cout << "PID+resonant steady-state ripple: " << rippleWithRC << "\n";
    std::cout << "ratio (with/without):              " << (rippleWithRC / ripplePidOnly) << "\n";

    const bool ok = std::isfinite(rippleWithRC) && std::isfinite(ripplePidOnly)
                  && rippleWithRC < 0.3 * ripplePidOnly;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
