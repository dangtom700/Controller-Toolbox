#include "simulation_runner.h"
#include "telemetry_logger.h"
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace solar {

void runSimulation(const ScenarioConfig& sc,
                   ControllerBase& ctrl,
                   const PlantParams& params,
                   const std::string& log_dir)
{
    const int N = static_cast<int>(std::lround(sc.duration_s / sc.Ts));

    std::filesystem::create_directories(log_dir);
    std::string csv_path = log_dir + "/run_" + sc.id + "_" + ctrl.name() + ".csv";

    TelemetryLogger logger(csv_path);
    SolarCoolingPlant plant(params);
    ctrl.reset();

    std::cout << "  [" << std::setw(32) << std::left << sc.id
              << " | " << std::setw(8) << ctrl.name() << "] ";
    std::cout.flush();

    // Warm-start: one step at nominal to prime the controller
    WeatherInput w0  = sc.weather_fn(0.0);
    ControlInput u   = {0.10, 0.85};
    PlantOutput  out = plant.step(w0, u);

    for (int k = 0; k < N; ++k) {
        double t        = k * sc.Ts;
        WeatherInput w  = sc.weather_fn(t);
        double ref_Tw1  = sc.setpoint_fn(t);

        u   = ctrl.compute(ref_Tw1, out.Tw1_C, w.G);
        out = plant.step(w, u);
        ctrl.setLastEER(out.EER_grid);   // notify ESCSolarCtrl (no-op for others)

        TickData td;
        td.t       = t;
        td.weather = w;
        td.control = u;
        td.output  = out;
        td.ref_Tw1 = ref_Tw1;
        logger.log(td);
    }

    logger.flush();
}

} // namespace solar
