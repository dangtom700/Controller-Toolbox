#pragma once
// controllers.h - controller roster for Underwater Glider Trajectory Tracking (TEMPLATE)
#include "IController.h"          // from lib/ (umbrella: ControllerToolbox.h)
#include <memory>
#include <vector>
#include <string>

namespace underwaterglidertrajectorytracking {

struct NamedController {
    std::string name;
    std::shared_ptr<ctrl::IController> ctrl;
};

// Build all controllers to be benchmarked (12 entries).
std::vector<NamedController> makeControllers(double Ts);

}  // namespace underwaterglidertrajectorytracking
