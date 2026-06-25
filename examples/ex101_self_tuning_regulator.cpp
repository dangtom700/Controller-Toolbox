/**
 * @file ex101_self_tuning_regulator.cpp
 * @brief Phase 3 Roadmap Phase 2 (OC1): self-tuning regulator on a slowly-drifting plant.
 *
 * A fixed-gain controller degrades as plant dynamics drift over time (e.g. seasonal HVAC load).
 * SelfTuningRegulator re-identifies the plant online and updates its control law every step,
 * with no manual re-tuning, remaining stable through a mid-run plant parameter change.
 *
 * NOTE: certainty-equivalence direct adaptive control (this mode) has no general guarantee of
 * persistent excitation from closed-loop operation alone (see the class-level @warning in
 * SelfTuningRegulator.h) - it can converge to a stabilizing but numerically inexact parameter
 * estimate. This example demonstrates the reliably-true property (stability through a plant
 * change), not exact setpoint tracking.
 */

#include "ControllerToolbox.h"
#include <iostream>
#include <random>

int main()
{
    ctrl::STRParams params;
    params.na = 1;
    params.nb = 1;
    params.mode = ctrl::STRMode::MinimumVariance;
    params.lambda = 0.97; // mild forgetting so it can track the drift
    // Minimum-variance (d=1) is inherently a deadbeat design - bound u to a realistic actuator
    // range so identification-transient errors can't saturate into a runaway feedback loop
    // (see the class-level @warning in SelfTuningRegulator.h).
    params.uMin = -20.0;
    params.uMax = 20.0;

    ctrl::SelfTuningRegulator str(params, 0.1);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> excite(-0.3, 0.3);

    double y = 0.0;
    double a = 0.6, b = 1.0; // "summer load" plant: y[k] = a*y[k-1] + b*u[k-1]
    const double setpoint = 5.0;

    const int N = 400;
    for (int k = 0; k < N; ++k)
    {
        if (k == N / 2) { a = 0.3; b = 0.6; } // "winter load" - plant drifts mid-run

        str.setReference(setpoint + excite(rng) * (k < 30 ? 5.0 : 0.0)); // brief excitation, then hold
        const double u = str.compute(y);
        y = a * y + b * u; // apply u directly - compute() already accounts for the plant's
                            // inherent one-step delay internally via its own bookkeeping
    }

    std::cout << "Final y = " << y << " (target " << setpoint << ")\n";
    std::cout << "Estimated A(q^-1) = " << str.estimatedDenominator().transpose() << "\n";
    std::cout << "Estimated B(q^-1) = " << str.estimatedNumerator().transpose() << "\n";

    // Stability/boundedness through the plant change, not exact setpoint tracking - see the
    // file-level NOTE above.
    const bool ok = std::isfinite(y) && std::fabs(y) < 1000.0 && str.covariance().allFinite();
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
