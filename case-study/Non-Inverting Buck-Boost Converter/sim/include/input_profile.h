#pragma once
#include <string>
#include <vector>

namespace conv {

struct VrefStep {
    double t_start = 0.0;
    double v_ref   = 8.0;
};

struct ScenarioConfig {
    std::string           id;
    std::string           description;
    double                v_in       = 10.0;
    double                duration_s = 0.060;
    std::vector<VrefStep> steps;

    // Returns the reference voltage active at time t.
    double vRefAt(double t) const;

    static ScenarioConfig fromJson(const std::string& path);
};

} // namespace conv
