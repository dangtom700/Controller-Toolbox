#pragma once
#include "solar_cooker_plant.h"
#include <string>

namespace cooker {

class ControllerBase;

void runSimulation(const PlantParams&    plant,
                   const ScenarioConfig& scenario,
                   ControllerBase&       controller,
                   const std::string&    log_dir);

} // namespace cooker
