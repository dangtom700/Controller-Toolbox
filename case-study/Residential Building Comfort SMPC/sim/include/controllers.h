#pragma once
// controllers.h - controller roster for Residential Building Comfort SMPC (TEMPLATE)
#include "IController.h"          // from lib/ (umbrella: ControllerToolbox.h)
#include <memory>
#include <vector>
#include <string>

namespace residentialbuildingcomfortsmpc {

struct NamedController {
    std::string name;
    std::shared_ptr<ctrl::IController> ctrl;
};

// Build all controllers to be benchmarked (12 entries).
std::vector<NamedController> makeControllers(double Ts);

}  // namespace residentialbuildingcomfortsmpc
