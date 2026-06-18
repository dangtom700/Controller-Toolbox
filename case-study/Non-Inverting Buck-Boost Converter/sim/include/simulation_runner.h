#pragma once
#include "buck_boost_plant.h"
#include "input_profile.h"
#include <string>

namespace conv {

// Forward declaration
class ControllerBase;

void runSimulation(const PlantParams&    plant,
                   const ScenarioConfig& scenario,
                   ControllerBase&       controller,
                   const std::string&    log_dir);

} // namespace conv
