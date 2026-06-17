#pragma once
// controllers.h - controller roster for Differential Drive Robot Tracking (TEMPLATE)
#include "IController.h"          // from lib/ (umbrella: ControllerToolbox.h)
#include <memory>
#include <vector>
#include <string>

namespace differentialdriverobottracking {

struct NamedController {
    std::string name;
    std::shared_ptr<ctrl::IController> ctrl;
};

// Build all controllers to be benchmarked (12 entries).
std::vector<NamedController> makeControllers(double Ts);

}  // namespace differentialdriverobottracking
