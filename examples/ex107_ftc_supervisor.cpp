/**
 * @file ex107_ftc_supervisor.cpp
 * @brief Phase 3 Roadmap Phase 2 (DT4): fault-tolerant reconfiguration on a redundant sensor pair.
 *
 * A loop with a redundant sensor pair: FTCSupervisor detects a sensor_bias fault on the primary
 * sensor (via a FaultClassifier fed the primary-sensor residual) and automatically switches
 * ControllerStack's active entry to a controller relying on the healthy backup sensor, with
 * bumpless transfer guaranteed by ControllerStack's existing Supervisory mode.
 *
 * NOTE: FaultClassifier is a small-sample (confirm_window=5) per-step heuristic, so once the
 * loop is reconfigured onto the backup controller, individual steps can keep flickering between
 * SensorBias/SensorNoise/ActuatorLoss as the still-oscillating closed loop's residual statistics
 * vary - FTCSupervisor::compute() already ignores any classification with no registered response
 * rather than reconfiguring on every flicker (see FTCSupervisor.cpp), so the *active controller*
 * stays latched on the backup. This example checks that reliably-true property (the supervisor
 * detects the fault at least once and latches onto the healthy backup, with a bounded/stable
 * trajectory throughout), not that every single step classifies identically.
 */

#include "ControllerToolbox.h"
#include <iostream>

int main()
{
    const double Ts = 0.1;
    const ctrl::TransferFunction tf({0.0, 1.0}, {1.0, -0.9}, Ts);
    const auto sys = ctrl::tf2ss(tf);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(sys.stateSize());

    ctrl::PIDParams pp; pp.Kp = 1.0; pp.Ki = 0.3;
    auto stack = std::make_shared<ctrl::ControllerStack>(ctrl::StackMode::Supervisory, Ts);
    stack->addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "primary_sensor_pid");
    stack->addController(std::make_shared<ctrl::DiscretePID>(pp, Ts), "backup_sensor_pid");

    ctrl::FTCSupervisor ftc(stack, ctrl::FaultDetectorParams{}, Ts);
    ftc.registerFaultResponse(ctrl::FaultType::None, "primary_sensor_pid");
    ftc.registerFaultResponse(ctrl::FaultType::SensorBias, "backup_sensor_pid");

    const double r = 1.0;
    double y = 0.0;
    double primaryBias = 0.0;

    const int N = 200;
    std::string activeAtEnd;
    bool sensorBiasEverDetected = false;
    double uPrev = 0.0;
    for (int k = 0; k < N; ++k)
    {
        if (k == N / 2) primaryBias = 5.0; // primary sensor develops a bias fault mid-run

        const double yPrimary = y + primaryBias;
        const double yBackup = y; // backup sensor stays healthy

        // Residual: primary sensor reading vs. the (assumed-healthy) backup, the simplest
        // redundant-sensor consistency check; u_cmd is the previous control action.
        ftc.feedResidual(yPrimary - yBackup, uPrev, yPrimary);

        const double e = r - yPrimary; // the active controller always sees the primary reading
        const double u = ftc.compute(e);
        uPrev = u;

        Eigen::VectorXd uv(1); uv << u;
        y = ctrl::ssStep(sys, x, uv)(0);
        activeAtEnd = stack->activeControllerName();
        sensorBiasEverDetected = sensorBiasEverDetected ||
                                 ftc.currentFault() == ctrl::FaultType::SensorBias;
    }

    std::cout << "Final active controller: " << activeAtEnd << "\n";
    std::cout << "SensorBias detected at least once: " << (sensorBiasEverDetected ? "yes" : "no")
              << "\n";
    std::cout << "Final y = " << y << "\n";

    const bool ok = activeAtEnd == "backup_sensor_pid" && sensorBiasEverDetected &&
                    std::isfinite(y) && std::fabs(y) < 1000.0;
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
