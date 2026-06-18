#include "simulation_runner.h"
#include "telemetry_logger.h"
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace humid {

void runSimulation(const ScenarioConfig& sc,
                   ControllerBase& ctrl,
                   const PlantParams& params,
                   const std::string& log_dir)
{
    const int N = static_cast<int>(std::lround(sc.duration_s / sc.Ts));

    std::filesystem::create_directories(log_dir);
    std::string csv_path = log_dir + "/run_" + sc.id + "_" + ctrl.name() + ".csv";

    TelemetryLogger logger(csv_path);

    HumidificationPlant plant(params);
    plant.reset(sc.phi_init);
    ctrl.reset();

    std::cout << "  [" << std::setw(28) << std::left << sc.id
              << " | " << std::setw(10) << ctrl.name() << "] ";
    std::cout.flush();

    // Prime: one step at initial condition so controller has a valid first reading
    Disturbance d0 = sc.disturbance_fn(0.0);
    double ref0    = sc.setpoint_fn(0.0);
    ControlInput u = {kFanMid, kTa_nom};
    PlantOutput out = plant.step(u, d0);

    // Provide outdoor phi to FFPID if applicable
    auto* ffpid = dynamic_cast<FFPIDHumidCtrl*>(&ctrl);

    for (int k = 0; k < N; ++k) {
        double t   = k * sc.Ts;
        Disturbance d = sc.disturbance_fn(t);
        double ref    = sc.setpoint_fn(t);

        if (ffpid) ffpid->setOutdoorPhi(d.phi_out);

        u   = ctrl.compute(out.phi_measured, ref);
        out = plant.step(u, d);

        TickData td;
        td.t       = t;
        td.dist    = d;
        td.control = u;
        td.output  = out;
        td.ref_phi = ref;
        logger.log(td);
    }

    logger.flush();
}

} // namespace humid
