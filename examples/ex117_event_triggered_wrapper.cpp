/**
 * @file ex117_event_triggered_wrapper.cpp
 * @brief Aperiodic-sampling (event-triggered) control via EventTriggeredWrapper.
 *
 * Wraps a PID controller so it only recomputes when the tracking error drifts more than a
 * deadband threshold since the last triggered computation, holding the output (zero-order
 * hold) otherwise. Demonstrates the bandwidth saving (trigger_count() << total steps) for a
 * slowly-varying (sinusoidal) reference, and that it still tracks within a bounded error
 * envelope.
 *
 * A *step* reference does not exercise this honestly: once the closed loop settles to a fixed
 * point, the error stops drifting, so it can permanently fall under the deadband and never
 * trigger again - even though the settled error itself may sit well outside the deadband. A
 * continuously-varying reference keeps the error drifting indefinitely, so the wrapper keeps
 * re-triggering for the life of the run, which is what the deadband bound below actually
 * verifies (checked over the back half of the run, past the startup transient).
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
    pp.Kp = 4.0; pp.Ki = 2.0; pp.Kd = 0.0; pp.N = 10.0;
    auto pid = std::make_shared<ctrl::DiscretePID>(pp, Ts);

    ctrl::EventTriggeredParams etp;
    etp.sigma = 0.05; // recompute only when |error| moves more than 0.05 since last trigger
    ctrl::EventTriggeredWrapper etw(pid, etp);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(plant.stateSize());
    double y = 0.0;
    const double ref_base = 1.0, ref_amp = 0.3, ref_period_s = 20.0;
    const int N = 300;

    double max_err_tail = 0.0; // worst |ref - y| over the back half (transient excluded)
    for (int k = 0; k < N; ++k)
    {
        const double t = k * Ts;
        const double ref = ref_base + ref_amp * std::sin(2.0 * M_PI * t / ref_period_s);
        const double e = ref - y;
        const double u = etw.compute(e);
        Eigen::VectorXd uv(1);
        uv << u;
        y = ctrl::ssStep(plant, x, uv)(0);
        if (k >= N / 2)
            max_err_tail = std::max(max_err_tail, std::fabs(ref - y));
    }

    const int total_calls = N;
    std::cout << "Total compute() calls: " << total_calls << "\n";
    std::cout << "Triggered: " << etw.triggerCount() << "  Held: " << etw.holdCount() << "\n";
    std::cout << "Max |error| (back half) = " << max_err_tail << "\n";

    const bool fewer_triggers_than_calls = etw.triggerCount() < total_calls;
    const bool tracks_well = std::isfinite(max_err_tail) && max_err_tail < 4.0 * etp.sigma;
    const bool counts_consistent =
        (etw.triggerCount() + etw.holdCount()) == total_calls;

    const bool ok = fewer_triggers_than_calls && tracks_well && counts_consistent;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
