/**
 * @file ex117_event_triggered_wrapper.cpp
 * @brief Aperiodic-sampling (event-triggered) control via EventTriggeredWrapper.
 *
 * Wraps a PID controller so it only recomputes when the tracking error drifts more than a
 * deadband threshold since the last triggered computation, holding the output (zero-order
 * hold) otherwise. Demonstrates the bandwidth saving (trigger_count() << total steps) for a
 * slowly-varying reference, and that it still tracks within a deadband-sized error envelope.
 *
 * @see docs/control_strategies_deep_dive.md (Event-Triggered Control section).
 * @see docs/superpowers/specs/2026-06-26-small-extensions-batch-design.md, item 5.
 */

#include "ControllerToolbox.h"
#include <cmath>
#include <iostream>

int main()
{
    const double Ts = 0.1;

    // Plant: G(s) = 1/(s+1) -> ZOH discretisation.
    const ctrl::StateSpace plant_c(
        Eigen::MatrixXd::Constant(1, 1, -1.0),
        Eigen::MatrixXd::Constant(1, 1,  1.0),
        Eigen::MatrixXd::Constant(1, 1,  1.0),
        Eigen::MatrixXd::Zero(1, 1), 0.0);
    const ctrl::StateSpace plant = ctrl::c2d(plant_c, Ts, ctrl::C2dMethod::ZOH);

    ctrl::PIDParams pp;
    pp.Kp = 2.0; pp.Ki = 0.8; pp.Kd = 0.0; pp.N = 10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::EventTriggeredParams etp;
    etp.sigma = 0.05; // recompute only when |error| moves more than 0.05 since last trigger
    ctrl::EventTriggeredWrapper etw(pid, etp);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;
    const double ref = 1.0;
    const int N = 300;

    for (int k = 0; k < N; ++k)
    {
        const double e = ref - y;
        const double u = etw.compute(e);
        Eigen::VectorXd uv(1);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);
    }

    const int total_calls = N;
    std::cout << "Total compute() calls: " << total_calls << "\n";
    std::cout << "Triggered: " << etw.triggerCount() << "  Held: " << etw.holdCount() << "\n";
    std::cout << "Final y = " << y << " (reference = " << ref << ")\n";

    const bool fewer_triggers_than_calls = etw.triggerCount() < total_calls;
    const bool tracks_well = std::isfinite(y) && std::fabs(y - ref) < 4.0 * etp.sigma;
    const bool counts_consistent =
        (etw.triggerCount() + etw.holdCount()) == total_calls;

    const bool ok = fewer_triggers_than_calls && tracks_well && counts_consistent;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
