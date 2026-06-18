#include "input_profile.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace conv {

using json = nlohmann::json;

double ScenarioConfig::vRefAt(double t) const
{
    if (steps.empty()) return 0.0;
    double v = steps.front().v_ref;
    for (auto& s : steps) {
        if (t >= s.t_start) v = s.v_ref;
        else break;
    }
    return v;
}

ScenarioConfig ScenarioConfig::fromJson(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open scenario JSON: " + path);
    json j; f >> j;

    ScenarioConfig cfg;
    cfg.id          = j.value("id", "unknown");
    cfg.description = j.value("description", "");
    cfg.v_in        = j.value("v_in", 10.0);
    cfg.duration_s  = j.value("duration_s", 0.060);

    if (j.contains("steps") && j["steps"].is_array()) {
        for (auto& s : j["steps"]) {
            VrefStep step;
            step.t_start = s.value("t_start", 0.0);
            step.v_ref   = s.value("v_ref", 8.0);
            cfg.steps.push_back(step);
        }
    }
    return cfg;
}

} // namespace conv
