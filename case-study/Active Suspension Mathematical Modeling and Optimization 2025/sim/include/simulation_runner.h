#pragma once
#include "susp_plant.h"
#include "road_profile.h"
#include "controllers.h"
#include <string>

// Runs one (scenario, controller) pair and writes CSV telemetry to logs/.
//
// CSV columns:
//   t, z_s, dz_s, z_u, dz_u, z_r, F_act, body_acc, susp_defl, tyre_defl, error, iae_cumulative
//
// Console summary:
//   [Sk | Name]  IAE_zs=<>  RMS_acc=<>  MaxDefl=<>  RMS_force=<>  wall=<> ms

namespace susp {

void runSimulation(const PlantParams&    plant,
                   const ScenarioConfig& scenario,
                   ControllerBase&       controller,
                   const std::string&    log_dir);

} // namespace susp
