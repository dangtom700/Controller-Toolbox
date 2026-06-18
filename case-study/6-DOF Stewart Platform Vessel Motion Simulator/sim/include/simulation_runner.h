#pragma once
#include "stewart_plant.h"
#include "cfd_input_model.h"
#include <string>

namespace stewart {

class ControllerBase;

struct RunResult {
    std::string name;
    std::string scenario_id;
    double rmse_pos_mm  = 0.0; // 3-axis combined position RMSE [mm]
    double rmse_att_deg = 0.0; // 3-axis combined attitude RMSE [deg]
    double rod_rms_mm   = 0.0; // per-rod RMS displacement error, averaged over all 6 rods [mm]
    double iae          = 0.0;
    double wall_ms       = 0.0;
    std::string csv;
};

RunResult runSimulation(const PlantParams&    plant,
                        const SeaStateConfig& cfg,
                        ControllerBase&        controller,
                        const std::string&     log_dir);

} // namespace stewart
