#include "simulation_runner.h"
#include <nlohmann/json.hpp>
#include "controllers.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <filesystem>

using json = nlohmann::json;

namespace boiler {

ScenarioConfig ScenarioConfig::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open scenario: " + path);
    json cfg = json::parse(f);

    ScenarioConfig s{};
    s.id             = cfg.value("id", "unknown");
    s.description    = cfg.value("description", "");
    s.operating_point = cfg.value("operating_point", "B");
    s.mode           = cfg.value("mode", "regulation");
    s.duration_s     = cfg.value("duration_s", 3600.0);
    s.Ts             = cfg.value("Ts", 1.0);

    auto dx0_j = cfg.value("dx0", std::vector<double>{0.0, 0.0, 0.0});
    s.dx0 = Eigen::Vector3d(dx0_j[0], dx0_j[1], dx0_j[2]);

    auto sp_j  = cfg.value("setpoint_dy", std::vector<double>{0.0, 0.0, 0.0});
    s.setpoint_dy = Eigen::Vector3d(sp_j[0], sp_j[1], sp_j[2]);

    if (cfg.contains("transition_time_s")) {
        s.has_transition    = true;
        s.transition_time_s = cfg["transition_time_s"].get<double>();
        s.transition_to_op  = cfg.value("transition_to_op", "B");
    }

    if (s.mode == "periodic_tracking") {
        s.has_periodic      = true;
        auto amp_j = cfg.value("periodic_amp", std::vector<double>{0.0, 0.0, 0.0});
        s.periodic_amp      = Eigen::Vector3d(amp_j[0], amp_j[1], amp_j[2]);
        s.periodic_freq_hz  = cfg.value("periodic_freq_hz", 0.005);
    }

    return s;
}

void runSimulation(const ScenarioConfig& scenario,
                   ControllerBase& controller,
                   const std::string& log_dir)
{
    const double Ts       = scenario.Ts;
    const int    N_steps  = static_cast<int>(std::lround(scenario.duration_s / Ts));

    const OperatingPoint& op_start = getOperatingPoint(scenario.operating_point);

    // Second operating point for s07 transition
    const OperatingPoint* op_end_ptr = nullptr;
    if (scenario.has_transition)
        op_end_ptr = &getOperatingPoint(scenario.transition_to_op);

    // Initialise plant at starting operating point + perturbation
    BoilerTurbine bt;
    bt.initAt(op_start);
    bt.setState(op_start.x1 + scenario.dx0(0),
                op_start.x2 + scenario.dx0(1),
                op_start.x3 + scenario.dx0(2));

    controller.reset();

    // CSV
    std::string log_path = log_dir + "/run_" + scenario.id + "_" + controller.name() + ".csv";
    TelemetryLogger logger(log_path);

    std::cout << "  [" << std::setw(30) << std::left << scenario.id
              << " | " << std::setw(24) << controller.name() << "] ";
    std::cout.flush();

    const Eigen::Vector3d u0_start(op_start.u1, op_start.u2, op_start.u3);
    const Eigen::Vector3d y0_start(op_start.y1, op_start.y2, op_start.y3);

    for (int k = 0; k < N_steps; ++k) {
        double t = k * Ts;

        // Handle operating-point transition (s07)
        const OperatingPoint* cur_op = &op_start;
        Eigen::Vector3d ref_dy       = scenario.setpoint_dy;

        if (scenario.has_transition && op_end_ptr && t >= scenario.transition_time_s) {
            cur_op = op_end_ptr;
            // Track the new operating point: setpoint = new op outputs - old op outputs
            ref_dy = Eigen::Vector3d(
                op_end_ptr->y1 - op_start.y1,
                op_end_ptr->y2 - op_start.y2,
                op_end_ptr->y3 - op_start.y3);
        }

        // Periodic tracking (s08): add sinusoidal component to the base setpoint.
        // The sinusoid is consistent across all controllers for the same scenario run
        // (deterministic, no PRNG).  RepetitiveController learns this waveform; PID/MPC
        // cannot cancel it entirely and will show persistent residual periodic error.
        if (scenario.has_periodic) {
            double phase = 2.0 * M_PI * scenario.periodic_freq_hz * t;
            ref_dy += scenario.periodic_amp * std::sin(phase);
        }

        const Eigen::Vector3d u0(cur_op->u1, cur_op->u2, cur_op->u3);
        const Eigen::Vector3d y0(cur_op->y1, cur_op->y2, cur_op->y3);

        // Measure outputs and compute deviation
        Eigen::Vector3d y  = bt.measureOutputs();
        Eigen::Vector3d dy = y - y0;

        // Controller computes valve increment
        Eigen::Vector3d du = controller.compute(ref_dy, dy);

        // Apply: absolute valve = clamp(u0 + du, 0, 1)
        Eigen::Vector3d u_abs;
        for (int i = 0; i < 3; ++i)
            u_abs(i) = std::clamp(u0(i) + du(i), 0.0, 1.0);

        bt.setControls(u_abs(0), u_abs(1), u_abs(2));
        bt.constrainValveRate();
        bt.update();

        // Absolute setpoint for logger
        Eigen::Vector3d ref_abs = y0 + ref_dy;

        TickData td;
        td.t     = t;
        td.y     = y;
        td.u     = bt.controls();
        td.du    = Eigen::Vector3d(bt.du1, bt.du2, bt.du3);
        td.ref_y = ref_abs;
        logger.log(td);
    }

    logger.flush();
}

} // namespace boiler
